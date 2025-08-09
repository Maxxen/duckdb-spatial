#include "duckdb/planner/expression/bound_function_expression.hpp"
#include "spatial/geometry/bbox.hpp"
#include "spatial/geometry/geometry_serialization.hpp"
#include "spatial/geometry/sgl.hpp"
#include "spatial/geometry/geometry_type.hpp"
#include "spatial/modules/main/spatial_functions.hpp"
#include "spatial/spatial_types.hpp"
#include "spatial/util/function_builder.hpp"

#include <sys/stat.h>

namespace duckdb {

namespace {

struct ExtentAggState {
	bool is_set;
	double xmin;
	double xmax;
	double ymin;
	double ymax;
};

//------------------------------------------------------------------------
// ENVELOPE AGG
//------------------------------------------------------------------------
struct ExtentAggFunction {
	template <class STATE>
	static void Initialize(STATE &state) {
		state.is_set = false;
	}

	template <class STATE, class OP>
	static void Combine(const STATE &source, STATE &target, AggregateInputData &) {
		if (!source.is_set) {
			return;
		}
		if (!target.is_set) {
			target = source;
			return;
		}
		target.xmin = std::min(target.xmin, source.xmin);
		target.xmax = std::max(target.xmax, source.xmax);
		target.ymin = std::min(target.ymin, source.ymin);
		target.ymax = std::max(target.ymax, source.ymax);
	}

	template <class INPUT_TYPE, class STATE, class OP>
	static void Operation(STATE &state, const INPUT_TYPE &input, AggregateUnaryInput &aggregate) {

		// TODO: Vectorize this so we dont clear the arena after each row
		sgl::geometry geom;
		Serde::Deserialize(geom, aggregate.input.allocator, input.GetDataUnsafe(), input.GetSize());

		auto bbox = sgl::extent_xy::smallest();
		if (sgl::ops::get_total_extent_xy(geom, bbox)) {

			if (!state.is_set) {
				state.is_set = true;
				state.xmin = bbox.min.x;
				state.xmax = bbox.max.x;
				state.ymin = bbox.min.y;
				state.ymax = bbox.max.y;
			} else {
				state.xmin = std::min(state.xmin, bbox.min.x);
				state.xmax = std::max(state.xmax, bbox.max.x);
				state.ymin = std::min(state.ymin, bbox.min.y);
				state.ymax = std::max(state.ymax, bbox.max.y);
			}
		}

		aggregate.input.allocator.Reset();
	}

	template <class INPUT_TYPE, class STATE, class OP>
	static void ConstantOperation(STATE &state, const INPUT_TYPE &input, AggregateUnaryInput &agg, idx_t) {
		Operation<INPUT_TYPE, STATE, OP>(state, input, agg);
	}

	template <class T, class STATE>
	static void Finalize(STATE &state, T &target, AggregateFinalizeData &finalize_data) {
		if (!state.is_set) {
			finalize_data.ReturnNull();
		} else {
			// We can create the bounding box polygon directly on the stack
			double buf[10];
			buf[0] = state.xmin;
			buf[1] = state.ymin;

			buf[2] = state.xmin;
			buf[3] = state.ymax;

			buf[4] = state.xmax;
			buf[5] = state.ymax;

			buf[6] = state.xmax;
			buf[7] = state.ymin;

			buf[8] = state.xmin;
			buf[9] = state.ymin;

			sgl::geometry ring(sgl::geometry_type::LINESTRING, false, false);
			ring.set_vertex_array(buf, 5);

			sgl::geometry bbox(sgl::geometry_type::POLYGON, false, false);
			bbox.append_part(&ring);

			const auto size = Serde::GetRequiredSize(bbox);
			auto blob = StringVector::EmptyString(finalize_data.result, size);
			Serde::Serialize(bbox, blob.GetDataWriteable(), size);
			blob.Finalize();

			target = blob;
		}
	}

	static bool IgnoreNull() {
		return true;
	}
};

//------------------------------------------------------------------------------
// Documentation
//------------------------------------------------------------------------------
// static constexpr DocTag DOC_TAGS[] = {{"ext", "spatial"}, {"category", "construction"}};
static constexpr const char *DOC_DESCRIPTION = R"(
    Computes the minimal-bounding-box polygon containing the set of input geometries
)";
static constexpr const char *DOC_EXAMPLE = R"(
	SELECT ST_Extent_Agg(geom) FROM UNNEST([ST_Point(1,1), ST_Point(5,5)]) AS _(geom);
	-- POLYGON ((1 1, 1 5, 5 5, 5 1, 1 1))
)";

static constexpr const char *DOC_ALIAS_DESCRIPTION = R"(
	Alias for [ST_Extent_Agg](#st_extent_agg).

	Computes the minimal-bounding-box polygon containing the set of input geometries.
)";

} // namespace

//======================================================================================================================
// ST_DBSCAN_List
//======================================================================================================================
namespace {

class SmallRTree {
public:
	void Init(uint32_t item_count_p, uint32_t node_space_p) {

		// Set the item count and node space
		item_count = item_count_p;
		node_space = node_space_p;

		uint32_t count = item_count;
		uint32_t layer = item_count;

		layer_bounds.push_back(layer);

		if (item_count == 0) {
			return;
		}

		// Figure out how many layers we need
		do {
			count = (count + node_space - 1) / node_space;
			layer += count;
			layer_bounds.push_back(layer);
		} while (count > 1);

		box_array.resize(layer);
		idx_array.resize(layer);
	}

	uint32_t Push(const sgl::extent_xy &box) {
		D_ASSERT(current_position < item_count);

		idx_array[current_position] = current_position;
		box_array[current_position] = box;

		tree_box.union_with(box);

		return current_position++;
	}

	void Build() {
		D_ASSERT(item_count == current_position);

		if (item_count <= node_space) {
			box_array[current_position++] = tree_box;
			return;
		}

		constexpr auto max_hilbert = std::numeric_limits<uint16_t>::max();
		const auto hw = max_hilbert / (tree_box.max.x - tree_box.min.x);
		const auto hh = max_hilbert / (tree_box.max.y - tree_box.min.y);

		vector<uint32_t> curve(item_count);
		for (idx_t i = 0; i < item_count; i++) {
			const auto &node_box = box_array[i];

			const auto hx = static_cast<uint32_t>(hw * ((node_box.min.x + node_box.max.x) / 2 - tree_box.min.x));
			const auto hy = static_cast<uint32_t>(hh * ((node_box.min.y + node_box.max.y) / 2 - tree_box.min.y));

			curve[i] = sgl::math::hilbert_encode(16, hx, hy);
		}

		// Now, sort the indices based on their curve value
		Sort(curve);

		size_t layer_idx = 0;
		size_t entry_idx = 0;

		while (layer_idx < layer_bounds.size() - 1) {
			const auto entry_end = layer_bounds[layer_idx];

			while (entry_idx < entry_end) {
				auto node_idx = entry_idx;
				auto node_box = box_array[entry_idx];

				size_t child_idx = 0;
				while (child_idx < node_space && entry_idx < entry_end) {

					node_box.union_with(box_array[entry_idx]);

					child_idx++;
					entry_idx++;
				}

				// Add a new parent node
				idx_array[current_position] = node_idx;
				box_array[current_position] = node_box;
				current_position++;
			}

			// Go to the next layer
			layer_idx++;
		}
	}

	class ScanState {
		friend class SmallRTree;
	private:
		queue<size_t> search_queue;
		sgl::extent_xy query;
		size_t entry_beg = 0;
		size_t entry_pos = 0;
		bool exhausted = true;
	};

	void InitScan(ScanState &state, const sgl::extent_xy &query) const {
		while (!state.search_queue.empty()) {
			state.search_queue.pop();
		}
		state.query = query;
		state.entry_beg = box_array.size() - 1;
		state.entry_pos = state.entry_beg;
		state.exhausted = false;
	}

	template<class CALLBACK>
	void Lookup(ScanState &state, CALLBACK &&callback) const {
		while (true) {
			const auto entry_end = std::min(state.entry_beg + node_space, UpperBound(state.entry_beg));

			while (state.entry_pos < entry_end) {
				if (!state.query.intersects(box_array[state.entry_pos])) {
					state.entry_pos++;
					continue;
				}

				auto yield = false;

				if (state.entry_beg >= item_count) {
					// Internal node
					state.search_queue.push(idx_array[state.entry_pos]);
				} else {
					// Leaf node
					yield = callback(idx_array[state.entry_pos]);
				}

				state.entry_pos++;

				if (yield) {
					// Yield!, return true to signal that there might be more rows!
					return;
				}
			}

			if (state.search_queue.empty()) {
				// There is no more nodes to search, return false!
				state.exhausted = true;
				return;
			}

			state.entry_beg = state.search_queue.front();
			state.entry_pos = state.entry_beg;
			state.search_queue.pop();
		}
	}

private:
	size_t UpperBound(size_t node_idx) const {
		const auto it = std::upper_bound(layer_bounds.begin(), layer_bounds.end(), node_idx);
		if (it == layer_bounds.end()) {
			return layer_bounds.back();
		}
		return *it;
	}

	void Sort(vector<uint32_t> &curve) {
		Sort(curve, 0, curve.size() - 1);
	}

	void Sort(vector<uint32_t> &curve, size_t l_idx, size_t r_idx) {
		if (l_idx < r_idx) {
			const auto pivot = curve[(l_idx + r_idx) >> 1];
			auto pivot_l = l_idx - 1;
			auto pivot_r = r_idx + 1;

			while (true) {
				do {
					++pivot_l;
				} while (curve[pivot_l] < pivot);
				do {
					--pivot_r;
				} while (curve[pivot_r] > pivot);

				if (pivot_l >= pivot_r) {
					break;
				}

				// Reorder the curve, boxes and indices
				// TODO: Pass callback here and make static
				std::swap(curve[pivot_l], curve[pivot_r]);
				std::swap(box_array[pivot_l], box_array[pivot_r]);
				std::swap(idx_array[pivot_l], idx_array[pivot_r]);
			}

			Sort(curve, l_idx, pivot_r);
			Sort(curve, pivot_r + 1, r_idx);
		}
	}

private:
	sgl::extent_xy tree_box = sgl::extent_xy::smallest();
	vector<uint32_t> layer_bounds;

	uint32_t item_count = 0;
	uint32_t node_space = 0;

	uint32_t current_position = 0;
	vector<uint32_t> idx_array;
	vector<sgl::extent_xy> box_array;
};

struct ST_DBSCAN_Agg {

	struct BindData final : FunctionData {

		uint32_t min_points;
		double epsilon;

		BindData(uint32_t min_points, double epsilon)
		    : min_points(min_points), epsilon(epsilon) {
		}

		unique_ptr<FunctionData> Copy() const override {
			return make_uniq<BindData>(min_points, epsilon);
		}

		bool Equals(const FunctionData &other_p) const override {
			auto &other = other_p.Cast<BindData>();
			return min_points == other.min_points && epsilon == other.epsilon;
		}
	};

	static unique_ptr<FunctionData> Bind(ClientContext &context, AggregateFunction &function, vector<unique_ptr<Expression>> &arguments) {

		return make_uniq<BindData>(5, 2000); // Default values for min_points and epsilon

	}

	//
	// static void Update(Vector inputs[], AggregateInputData &aggr_input_data, idx_t input_count, Vector &state_vector,
	// 					idx_t count){
	// 	throw NotImplementedException("SpatialFunctionsAggregate::Update");
	// }
	//
	// static void Combine(Vector &states_vector, Vector &combined, AggregateInputData &aggr_input_data, idx_t count) {
	// 	throw NotImplementedException("SpatialFunctionsAggregate::Combine");
	// }
	//

	struct GlobalIndex {
		explicit GlobalIndex(const uint32_t count) : geoms(count), clusters(count, 0) {}
		vector<unique_ptr<sgl::prepared_geometry>> geoms;
		SmallRTree rtree;

		vector<int32_t> clusters;
	};

	struct State {
		unique_ptr<GlobalIndex> index = nullptr;
	};

	static void Initialize(const AggregateFunction &, data_ptr_t state_mem) {
		new (state_mem) State();
	}

	static idx_t StateSize(const AggregateFunction &) {
		return sizeof(State);
	}

	// This is really all I need.
	static void WindowInit(AggregateInputData &aggr, const WindowPartitionInput &partition,
								data_ptr_t g_state) {
		// Scan the partition, construct the index, perform the clustering
		D_ASSERT(partition.inputs);
		auto &cdc = *partition.inputs;

		auto &state = *reinterpret_cast<State *>(g_state);
		state.index = make_uniq<GlobalIndex>(cdc.Count());

		// Initialize the R-Tree index
		state.index->rtree.Init(cdc.Count(), 8); // 16 is the node space, can be tuned

		// Scan the CDC
		ColumnDataScanState scan_state;
		DataChunk scan_chunk;
		cdc.InitializeScan(scan_state, partition.column_ids);
		cdc.InitializeScanChunk(scan_chunk);

		idx_t out_idx = 0;

		while (cdc.Scan(scan_state, scan_chunk)) {
			auto &geom_vec = scan_chunk.data[0];
			UnifiedVectorFormat geom_data;
			geom_vec.ToUnifiedFormat(scan_chunk.size(), geom_data);
			auto geom_ptr = UnifiedVectorFormat::GetData<string_t>(geom_data);

			for (idx_t rel_idx = 0; rel_idx < scan_chunk.size(); rel_idx++) {
				const auto row_idx = geom_data.sel->get_index(rel_idx);
				if (!geom_data.validity.RowIsValid(row_idx)) {
					throw NotImplementedException("Null geometries are not supported in ST_DBSCAN_Agg");
				}
				auto &geom = geom_ptr[geom_data.sel->get_index(rel_idx)];

				// Deserialize the geometry
				auto ptr = make_uniq<sgl::prepared_geometry>(sgl::geometry_type::INVALID, false, false);
				Serde::DeserializePrepared(*ptr, aggr.allocator, geom.GetDataUnsafe(), geom.GetSize());

				// Compute the bounding box
				sgl::extent_xy bbox = sgl::extent_xy::smallest();;
				if (!sgl::ops::get_total_extent_xy(*ptr, bbox)) {
					// Skip empty geometries // TODO: Should we throw an error here?
					continue;
				}

				// Push the geometry into the array and the R-Tree
				state.index->geoms[out_idx] = std::move(ptr);
				state.index->rtree.Push(bbox);

				out_idx++;
			}
		}
		D_ASSERT(out_idx == cdc.Count());

		// Build the R-Tree now that we have all geometries
		state.index->rtree.Build();

		const auto f_epsilon = MathUtil::DoubleToFloatUp(aggr.bind_data->Cast<BindData>().epsilon);
		const auto min_points = aggr.bind_data->Cast<BindData>().min_points;

		// Now, compute the clustering
		// To keep it simple, just compare how many geometries are within epsilon distance
		SmallRTree::ScanState rtree_scan_state;

		int32_t clusted_counter = 1;

		for (idx_t i = 0; i < out_idx; i++) {
			auto &geom = *state.index->geoms[i];
			auto &cluster = state.index->clusters[i];

			if (cluster != 0) {
				// Already assigned a cluster
				continue;
			}

			// Now query the R-Tree for all geometries that are within epsilon distance
			sgl::extent_xy bbox = sgl::extent_xy::smallest();
			sgl::ops::get_total_extent_xy(geom, bbox);

			// Expand the bounding box by epsilon
			bbox.min.x -= f_epsilon;
			bbox.min.y -= f_epsilon;
			bbox.max.x += f_epsilon;
			bbox.max.y += f_epsilon;

			// Now, lookup the geometries in the R-Tree
			state.index->rtree.InitScan(rtree_scan_state, bbox);

			vector<uint32_t> neighbors;
			state.index->rtree.Lookup(rtree_scan_state, [&](uint32_t idx) {
				// We have a geometry that is within epsilon distance
				if (idx == i) {
					// Skip self
					return false; // Continue scanning
				}
				double result;
				if (sgl::ops::get_euclidean_distance(geom, *state.index->geoms[idx], result)) {
					if (result <= aggr.bind_data->Cast<BindData>().epsilon) {
						neighbors.push_back(idx);
					}
				}
				return false; // Continue scanning
			});

			// If we have enough neighbors, we can assign a cluster
			if (neighbors.size() < min_points) {
				cluster = -1; // Mark as noise
				continue;
			}

			// Assign a new cluster
			cluster = clusted_counter++;

			while (!neighbors.empty()) {
				auto n = neighbors.back();
				neighbors.pop_back();

				if (state.index->clusters[n] == -1) {
					// Previously assigned to noise, add it to the current cluster
					state.index->clusters[n] = cluster;
					continue; // TODO: Do we continue here?
					// Yes. if it was labeled as noise before, there is no reason to expand it, as it has
					// less than min_points neighbors
				}

				if (state.index->clusters[n] != 0) {
					// Already assigned to a cluster, skip
					continue;
				}

				// Assign the neighbor to the current cluster
				state.index->clusters[n] = cluster;

				// Expand
				// Get the bounding box of the neighbor geometry
				auto &neighbor_geom = *state.index->geoms[n];
				sgl::extent_xy neighbor_bbox = sgl::extent_xy::smallest();;
				sgl::ops::get_total_extent_xy(neighbor_geom, neighbor_bbox);

				// Expand the bounding box by epsilon
				neighbor_bbox.min.x -= f_epsilon;
				neighbor_bbox.min.y -= f_epsilon;
				neighbor_bbox.max.x += f_epsilon;
				neighbor_bbox.max.y += f_epsilon;

				state.index->rtree.InitScan(rtree_scan_state, neighbor_bbox);
				vector<uint32_t> new_neighbors;
				state.index->rtree.Lookup(rtree_scan_state, [&](uint32_t idx) {
					if (idx == n) {
						// Skip self
						return false; // Continue scanning
					}
					double result;
					if (sgl::ops::get_euclidean_distance(neighbor_geom, *state.index->geoms[idx], result)) {
						if (result <= aggr.bind_data->Cast<BindData>().epsilon) {
							new_neighbors.push_back(idx);
						}
					}
					return false; // Continue scanning
				});

				if (new_neighbors.size() >= min_points) {
					// This is a core point, add its neighbors to the list
					neighbors.insert(neighbors.end(), new_neighbors.begin(), new_neighbors.end());
				}
			}
		}
	}

	static void WindowUpdate(AggregateInputData &aggr, const WindowPartitionInput &partition,
								   const_data_ptr_t g_state, data_ptr_t l_state, const SubFrames &subframes,
								   Vector &result, idx_t rid) {
		// Scan the resulting clustering and output rows
		const auto &gstate = *reinterpret_cast<const State *>(g_state);

		const auto rdata = FlatVector::GetData<int32_t>(result);
		// auto &rmask = FlatVector::Validity(result);

		rdata[rid] = gstate.index->clusters[rid];
	}

	static void Finalize(Vector &states_vector, AggregateInputData &aggr_input_data, Vector &result, idx_t count,
				  idx_t offset) {

	}

	static void Register(ExtensionLoader &loader) {
		AggregateFunction agg({GeoTypes::GEOMETRY()}, LogicalType::INTEGER, StateSize, Initialize, nullptr, nullptr, Finalize, nullptr);
		agg.window_init = WindowInit;
		agg.window = WindowUpdate;
		agg.bind = Bind;

		FunctionBuilder::RegisterAggregate(loader, "ST_DBSCAN_Agg", [&](AggregateFunctionBuilder &func) {
			func.SetFunction(agg);
			func.SetDescription("Performs a DBSCAN clustering on the input geometries.");
			func.SetExample("SELECT ST_DBSCAN_Agg(geom, 5, 0.1) FROM UNNEST([ST_Point(1,1), ST_Point(5,5)]) AS _(geom);");

			func.SetTag("ext", "spatial");
			func.SetTag("category", "construction");
		});
	}
};

} // namespace
//------------------------------------------------------------------------
// Register
//------------------------------------------------------------------------
void RegisterSpatialAggregateFunctions(ExtensionLoader &loader) {

	ST_DBSCAN_Agg::Register(loader);

	// TODO: Dont use geometry_t here
	const auto agg = AggregateFunction::UnaryAggregate<ExtentAggState, string_t, string_t, ExtentAggFunction>(
	    GeoTypes::GEOMETRY(), GeoTypes::GEOMETRY());

	FunctionBuilder::RegisterAggregate(loader, "ST_Extent_Agg", [&](AggregateFunctionBuilder &func) {
		func.SetFunction(agg);
		func.SetDescription(DOC_DESCRIPTION);
		func.SetExample(DOC_EXAMPLE);

		func.SetTag("ext", "spatial");
		func.SetTag("category", "construction");
	});

	FunctionBuilder::RegisterAggregate(loader, "ST_Envelope_Agg", [&](AggregateFunctionBuilder &func) {
		func.SetFunction(agg);
		func.SetDescription(DOC_ALIAS_DESCRIPTION);
		func.SetExample(DOC_EXAMPLE);

		func.SetTag("ext", "spatial");
		func.SetTag("category", "construction");
	});
}

} // namespace duckdb
