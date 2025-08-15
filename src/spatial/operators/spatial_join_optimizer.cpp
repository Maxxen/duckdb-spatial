#include "spatial_join_optimizer.hpp"
#include "spatial_join_logical.hpp"
#include "spatial/util/distance_extract.hpp"

#include "duckdb/main/database.hpp"
#include "duckdb/planner/expression/bound_function_expression.hpp"
#include "duckdb/planner/operator/logical_any_join.hpp"
#include "duckdb/catalog/catalog_entry/scalar_function_catalog_entry.hpp"
#include "duckdb/planner/operator/logical_filter.hpp"
#include "spatial/spatial_types.hpp"

namespace duckdb {

// All of these imply bounding box intersection
static const case_insensitive_set_t spatial_predicate_map = {
    "ST_Equals",   "ST_Intersects", "ST_Touches",   "ST_Crosses",          "ST_Within",         "ST_Contains",
    "ST_Overlaps", "ST_Covers",     "ST_CoveredBy", "ST_ContainsProperly", "ST_WithinProperly", "ST_DWithin",
};

static const case_insensitive_map_t<string> spatial_predicate_inverse_map = {
    {"ST_Equals", "ST_Equals"},
    {"ST_Intersects", "ST_Intersects"},           // Symmetric
    {"ST_Touches", "ST_Touches"},                 // Symmetric
    {"ST_Crosses", "ST_Crosses"},                 // Symmetric
    {"ST_Within", "ST_Contains"},                 // Inverse
    {"ST_Contains", "ST_Within"},                 // Inverse
    {"ST_Overlaps", "ST_Overlaps"},               // Symmetric
    {"ST_Covers", "ST_CoveredBy"},                // Inverse
    {"ST_CoveredBy", "ST_Covers"},                // Inverse
    {"ST_WithinProperly", "ST_ContainsProperly"}, // Inverse
    {"ST_ContainsProperly", "ST_WithinProperly"}, // Inverse
    {"ST_DWithin", "ST_DWithin"},                 // Symmetric (when distance is constant)

};

unique_ptr<Expression> TryGetInversePredicate(ClientContext &context, unique_ptr<Expression> expr) {
	auto &func = expr->Cast<BoundFunctionExpression>();
	const auto it = spatial_predicate_inverse_map.find(func.function.name);

	if (it == spatial_predicate_inverse_map.end()) {
		// We cant do anything
		return nullptr;
	}

	// Swap the arguments
	std::swap(func.children[0], func.children[1]);

	if (it->first == it->second) {
		// We've already swapped the child, so just return the expression
		return expr;
	}

	// Get the function from the catalog
	auto &catalog = Catalog::GetSystemCatalog(context);
	auto &entry = catalog.GetEntry<ScalarFunctionCatalogEntry>(context, DEFAULT_SCHEMA, it->second);
	auto inverse_func =
	    entry.functions.GetFunctionByArguments(context, {func.children[0]->return_type, func.children[1]->return_type});

	return make_uniq_base<Expression, BoundFunctionExpression>(func.return_type, inverse_func, std::move(func.children),
	                                                           nullptr, func.is_operator);
}

static void InsertSpatialJoin(OptimizerExtensionInput &input, unique_ptr<LogicalOperator> &plan) {
	auto &op = *plan;

	// We only care about ANY_JOIN operators
	if (op.type != LogicalOperatorType::LOGICAL_ANY_JOIN) {
		return;
	}

	auto &any_join = op.Cast<LogicalAnyJoin>();

	// We also only support simple join types
	auto join_supported = false;
	switch (any_join.join_type) {
	case JoinType::INNER:
	case JoinType::LEFT:
	case JoinType::RIGHT:
	case JoinType::OUTER:
		join_supported = true;
		break;
	default:
		break;
	}
	if (!join_supported) {
		return;
	}

	// Inspect the join condition
	vector<unique_ptr<Expression>> expressions;
	expressions.push_back(any_join.condition->Copy()); // TODO: Maybe move instead of copy

	// Split by AND
	LogicalFilter::SplitPredicates(expressions);

	// Get the table indexes that are reachable from the left and right children
	auto &left_child = any_join.children[0];
	auto &right_child = any_join.children[1];
	unordered_set<idx_t> left_bindings;
	unordered_set<idx_t> right_bindings;
	LogicalJoin::GetTableReferences(*left_child, left_bindings);
	LogicalJoin::GetTableReferences(*right_child, right_bindings);

	// TODO: Only support a single predicate for now
	if (expressions.size() != 1) {
		return;
	}

	// TODO: This whole logic is a work in progress.
	// it does a bunch of extra work trying to separate the predicates, which doesnt matter because we only support
	// a single join condition for now anyway

	// The spatial join condition
	unique_ptr<Expression> spatial_pred_expr = nullptr;

	// Extra predicates that are not spatial predicates
	vector<unique_ptr<Expression>> extra_predicates;

	// Now, check each expression to see if it contains a spatial predicate
	for (auto &expr : expressions) {
		auto total_side = JoinSide::GetJoinSide(*expr, left_bindings, right_bindings);

		if (total_side != JoinSide::BOTH) {
			// Throw?. No, push down the filter
			extra_predicates.push_back(std::move(expr));
			continue;
		}

		// Check if the expression is a spatial predicate
		if (expr->type != ExpressionType::BOUND_FUNCTION) {
			extra_predicates.push_back(std::move(expr));
			continue;
		}

		auto &func = expr->Cast<BoundFunctionExpression>();

		// The function must be a binary predicate
		if (func.children.size() != 2) {
			extra_predicates.push_back(std::move(expr));
			continue;
		}

		// The function must operate on two GEOMETRY types
		if (func.children[0]->return_type.id() != LogicalTypeId::GEOMETRY ||
		    func.children[1]->return_type.id() != LogicalTypeId::GEOMETRY) {
			extra_predicates.push_back(std::move(expr));
			continue;
		}

		// The function must be a recognized spatial predicate
		if (spatial_predicate_map.count(func.function.name) == 0) {
			extra_predicates.push_back(std::move(expr));
			continue;
		}

		auto left_side = JoinSide::GetJoinSide(*func.children[0], left_bindings, right_bindings);
		auto right_side = JoinSide::GetJoinSide(*func.children[1], left_bindings, right_bindings);

		// Can the condition can be cleanly split into two sides?
		if (left_side == JoinSide::BOTH || right_side == JoinSide::BOTH) {
			extra_predicates.push_back(std::move(expr));
			continue;
		}

		if (left_side == JoinSide::RIGHT) {
			expr = TryGetInversePredicate(input.context, std::move(expr));
			if (expr == nullptr) {
				// We cant flip this, abort!
				return;
			}
		}

		spatial_pred_expr = std::move(expr);
	}

	// Nope! No spatial predicate found
	if (!spatial_pred_expr) {
		return;
	}

	// TODO: Push a filter for the extra conditions?

	// Cool, now we have spatial join conditions. Proceed to create a new LogicalSpatialJoin operator
	auto spatial_join = make_uniq<LogicalSpatialJoin>(any_join.join_type);

	// Steal the properties from the any join
	spatial_join->spatial_predicate = std::move(spatial_pred_expr);
	spatial_join->extra_conditions = std::move(extra_predicates);
	spatial_join->children = std::move(any_join.children);
	spatial_join->expressions = std::move(any_join.expressions);
	spatial_join->types = std::move(any_join.types);
	spatial_join->left_projection_map = std::move(any_join.left_projection_map);
	spatial_join->right_projection_map = std::move(any_join.right_projection_map);
	spatial_join->join_stats = std::move(any_join.join_stats);
	spatial_join->mark_index = any_join.mark_index;
	spatial_join->has_estimated_cardinality = any_join.has_estimated_cardinality;
	spatial_join->estimated_cardinality = any_join.estimated_cardinality;

	// If this is ST_DWithin, try to extract the constant distance value
	const auto &pred_func = spatial_join->spatial_predicate->Cast<BoundFunctionExpression>();
	if (pred_func.function.name == "ST_DWithin") {
		// Try to get the constant distance value from the bind data;
		spatial_join->has_const_distance =
		    ST_DWithinHelper::TryGetConstDistance(pred_func.bind_info, spatial_join->const_distance);
	}

	// Replace the operator
	plan = std::move(spatial_join);
}

static void TryInsertSpatialJoin(OptimizerExtensionInput &input, unique_ptr<LogicalOperator> &plan) {

	InsertSpatialJoin(input, plan);

	// Recursively call this function on all children
	for (auto &child : plan->children) {
		TryInsertSpatialJoin(input, child);
	}
}

void SpatialJoinOptimizer::Register(ExtensionLoader &loader) {

	OptimizerExtension optimizer;
	optimizer.optimize_function = TryInsertSpatialJoin;

	auto &db = loader.GetDatabaseInstance();
	db.config.optimizer_extensions.push_back(optimizer);
}

} // namespace duckdb
