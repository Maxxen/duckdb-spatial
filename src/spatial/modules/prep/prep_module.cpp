#include "prep_module.hpp"
#include "sglib.hpp"
#include "../../util/binary_reader.hpp"
#include "../../util/binary_writer.hpp"
#include "../../util/math.hpp"
#include "duckdb/common/types/data_chunk.hpp"
#include "duckdb/common/vector_operations/unary_executor.hpp"
#include "duckdb/execution/expression_executor_state.hpp"
#include "duckdb/main/extension/extension_loader.hpp"

namespace duckdb {

namespace {
// WKB geometry type constants (2D only for now)
static constexpr uint32_t WKB_POINT = 1;
static constexpr uint32_t WKB_LINESTRING = 2;
static constexpr uint32_t WKB_POLYGON = 3;
static constexpr uint32_t WKB_MULTIPOINT = 4;
static constexpr uint32_t WKB_MULTILINESTRING = 5;
static constexpr uint32_t WKB_MULTIPOLYGON = 6;
static constexpr uint32_t WKB_GEOMETRYCOLLECTION = 7;

// Size of a GeometryPart structure
static constexpr uint32_t PART_SIZE = sizeof(sgl2::GeometryPart);
// Size of a single XY vertex
static constexpr uint32_t VERTEX_SIZE = sizeof(sgl2::VertexXY);


static uint32_t GetIndexSize(uint32_t vert_count) {
	// Layout is:
	// items_count
	// level_count;
	//    level1 count
	//    level1 offset
	//    level2 count
	//    levelN count
	//    levelN offset
	// levels...
	//    entries...

	uint32_t layer_bound[sgl2::GeometryIndex::MAX_DEPTH] = {};
	uint32_t layer_count = 0;

	const uint32_t count = (vert_count + sgl2::GeometryIndex::NODE_SIZE - 1) / sgl2::GeometryIndex::NODE_SIZE;
	while (true) {
		layer_bound[layer_count] = static_cast<uint32_t>(
			std::ceil(static_cast<double>(count) / std::pow(static_cast<double>(sgl2::GeometryIndex::NODE_SIZE), layer_count)));

		if (layer_bound[layer_count++] <= 1) {
			break; // We have reached the last layer
		}
	}
	std::reverse(layer_bound, layer_bound + layer_count);

	uint32_t total_size = sizeof(uint32_t) * 2 + layer_count * sizeof(uint32_t) * 2;

	for (uint32_t layer_idx = 0; layer_idx < layer_count; layer_idx++) {
		total_size += layer_bound[layer_idx] * sizeof(sgl2::ExtentXY);
	}

	return total_size;
}

// Get the required size to store the prepared geometry
static uint32_t GetRequiredSize(BinaryReader &reader) {
	// Skip byte order (we assume little endian)
	reader.Skip(1);

	// Read geometry type
	const uint32_t wkb_type = reader.Read<uint32_t>();

	// Mask off any Z/M/SRID flags to get base type
	const uint32_t base_type = wkb_type & 0xFF;

	switch (base_type) {
	case WKB_POINT: {
		reader.Skip(2 * sizeof(double));
		return PART_SIZE + VERTEX_SIZE;
	}
	case WKB_LINESTRING: {
		const uint32_t num_points = reader.Read<uint32_t>();
		reader.Skip(num_points * 2 * sizeof(double));
		return PART_SIZE + num_points * VERTEX_SIZE;
	}
	case WKB_POLYGON: {
		const uint32_t num_rings = reader.Read<uint32_t>();
		uint32_t total_size = PART_SIZE + num_rings * PART_SIZE;
		for (uint32_t i = 0; i < num_rings; i++) {
			const uint32_t num_points = reader.Read<uint32_t>();
			reader.Skip(num_points * 2 * sizeof(double));
			total_size += num_points * VERTEX_SIZE;

			if (num_points >= sgl2::GeometryIndex::NODE_SIZE) {
				total_size += GetIndexSize(num_points);
			}
		}
		return total_size;
	}
	case WKB_MULTIPOINT:
	case WKB_MULTILINESTRING:
	case WKB_MULTIPOLYGON:
	case WKB_GEOMETRYCOLLECTION: {
		const uint32_t num_geoms = reader.Read<uint32_t>();
		uint32_t total_size = PART_SIZE + num_geoms * PART_SIZE;
		for (uint32_t i = 0; i < num_geoms; i++) {
			total_size += GetRequiredSize(reader) - PART_SIZE;
		}
		return total_size;
	}
	default:
		throw InternalException("Unknown WKB geometry type: %u", wkb_type);
	}
}

static void Write(BinaryReader &reader, BinaryWriter &head_writer, BinaryWriter &body_writer) {

	reader.Skip(1);
	const uint32_t wkb_type = reader.Read<uint32_t>();
	const uint32_t base_type = wkb_type & 0xFF;

	switch (base_type) {
	case WKB_POINT: {

		head_writer.Write<uint8_t>(static_cast<uint8_t>(base_type));
		head_writer.Write<uint8_t>(0);  // set flag
		head_writer.Write<uint16_t>(1); // set padd to 1 to indicate serialized
		head_writer.Write<uint32_t>(1); // size

		const auto data_offset = body_writer.GetPosition() - head_writer.GetPosition() - sizeof(uint64_t);
		head_writer.Write<uint64_t>(data_offset);

		body_writer.Write<double>(reader.Read<double>());
		body_writer.Write<double>(reader.Read<double>());

	} break;
	case WKB_LINESTRING: {
		const uint32_t num_points = reader.Read<uint32_t>();

		head_writer.Write<uint8_t>(static_cast<uint8_t>(base_type));
		head_writer.Write<uint8_t>(0);           // set flag
		head_writer.Write<uint16_t>(1);          // set padd to 1 to indicate serialized
		head_writer.Write<uint32_t>(num_points); // size

		const auto data_offset = body_writer.GetPosition() - head_writer.GetPosition() - sizeof(uint64_t);
		head_writer.Write<uint64_t>(data_offset);

		for (uint32_t i = 0; i < num_points; i++) {
			body_writer.Write<double>(reader.Read<double>());
			body_writer.Write<double>(reader.Read<double>());
		}

	} break;
	case WKB_POLYGON: {
		const uint32_t num_rings = reader.Read<uint32_t>();

		head_writer.Write<uint8_t>(static_cast<uint8_t>(base_type));
		head_writer.Write<uint8_t>(0);          // set flag
		head_writer.Write<uint16_t>(1);         // set padd to 1 to indicate serialized
		head_writer.Write<uint32_t>(num_rings); // size

		const auto part_offset = body_writer.GetPosition() - head_writer.GetPosition() - sizeof(uint64_t);
		head_writer.Write<uint64_t>(part_offset);

		auto ring_writer = body_writer;

		body_writer.Skip(num_rings * PART_SIZE);

		for (uint32_t ring_idx = 0; ring_idx < num_rings; ring_idx++) {
			const uint32_t num_points = reader.Read<uint32_t>();

			const auto build_index = num_points >= sgl2::GeometryIndex::NODE_SIZE;

			ring_writer.Write<uint8_t>(static_cast<uint8_t>(sgl2::GeometryType::LINESTRING));
			ring_writer.Write<uint8_t>(build_index ? 1 : 0);           // set flag (has index)
			ring_writer.Write<uint16_t>(1);          // set padd to 1 to indicate serialized
			ring_writer.Write<uint32_t>(num_points); // size

			const auto ring_offset = body_writer.GetPosition() - ring_writer.GetPosition() - sizeof(uint64_t);
			ring_writer.Write<uint64_t>(ring_offset);

			const auto vert_ptr = body_writer.GetStart() + body_writer.GetPosition();

			for (uint32_t vert_idx = 0; vert_idx < num_points; vert_idx++) {
				body_writer.Write<double>(reader.Read<double>());
				body_writer.Write<double>(reader.Read<double>());
			}

			if (build_index) {
				// Layout is:
				// items_count
				// level_count;
				//    level1 count
				//    level1 offset
				//    level2 count
 				//    levelN count
				//    levelN offset
				// levels...
				//    entries...

				uint32_t layer_bound[sgl2::GeometryIndex::MAX_DEPTH] = {};
				uint32_t layer_offset[sgl2::GeometryIndex::MAX_DEPTH] = {};
				uint32_t layer_count = 0;

				const uint32_t count = (num_points + sgl2::GeometryIndex::NODE_SIZE - 1) / sgl2::GeometryIndex::NODE_SIZE;
				while (true) {
					layer_bound[layer_count] = static_cast<uint32_t>(
						std::ceil(static_cast<double>(count) / std::pow(static_cast<double>(sgl2::GeometryIndex::NODE_SIZE), layer_count)));

					if (layer_bound[layer_count++] <= 1) {
						break; // We have reached the last layer
					}
				}
				std::reverse(layer_bound, layer_bound + layer_count);

				body_writer.Write<uint32_t>(num_points);
				body_writer.Write<uint32_t>(layer_count);

				// Compute offsets and fill in level headers
				uint32_t entries_offset = 0; //layer_count * sizeof(uint32_t) * 2;

				for (uint32_t layer_idx = 0; layer_idx < layer_count; layer_idx++) {
					layer_offset[layer_idx] = entries_offset;
					uint32_t entries_count = layer_bound[layer_idx];

					body_writer.Write<uint32_t>(entries_count);
					body_writer.Write<uint32_t>(entries_offset);

					entries_offset += entries_count;
				}

				// Save pointer to entries start
				const auto entry_ptr = body_writer.GetPtr();

				// Jump over all the entries with the writer
				for (uint32_t layer_idx = 0; layer_idx < layer_count; layer_idx++) {
					body_writer.Skip(layer_bound[layer_idx] * sizeof(sgl2::ExtentXY));
				}

				// Now. Fill in the last layer
				for (uint32_t entry_idx = 0; entry_idx < layer_bound[layer_count-1]; entry_idx++) {

					auto target_ptr = entry_ptr + (layer_offset[layer_count - 1] + entry_idx) * sizeof(sgl2::ExtentXY);

					sgl2::ExtentXY extent;
					extent.xmin = std::numeric_limits<double>::max();
					extent.ymin = std::numeric_limits<double>::max();
					extent.xmax = std::numeric_limits<double>::lowest();
					extent.ymax = std::numeric_limits<double>::lowest();

					const auto beg = entry_idx * sgl2::GeometryIndex::NODE_SIZE;

					// We add +1 to the node size here, to get not just the start point of the segment, but also the end point,
					// which may be in the next node. This ensures there is no gaps between node bounding boxes.
					const auto end = std::min(beg + sgl2::GeometryIndex::NODE_SIZE + 1, num_points);

					for (uint32_t vert_idx = beg; vert_idx < end; vert_idx++) {
						sgl2::VertexXY vertex;
						memcpy(&vertex, vert_ptr + vert_idx * VERTEX_SIZE, VERTEX_SIZE);
						extent.xmin = std::min(extent.xmin, vertex.x);
						extent.ymin = std::min(extent.ymin, vertex.y);
						extent.xmax = std::max(extent.xmax, vertex.x);
						extent.ymax = std::max(extent.ymax, vertex.y);
					}

					// Write the extent
					memcpy(target_ptr, &extent, sizeof(sgl2::ExtentXY));
				}

				// Fill in the upper layers, bottom up
				for (int64_t layer_idx = static_cast<int64_t>(layer_count) - 2; layer_idx >= 0; layer_idx--) {

					auto target_layer_ptr = entry_ptr + layer_offset[layer_idx] * sizeof(sgl2::ExtentXY);
					auto target_layer_len = layer_bound[layer_idx];

					auto source_layer_ptr = entry_ptr + layer_offset[layer_idx + 1] * sizeof(sgl2::ExtentXY);
					auto source_layer_len = layer_bound[layer_idx + 1];

					for (uint32_t target_entry_idx = 0; target_entry_idx < target_layer_len; target_entry_idx++) {

						sgl2::ExtentXY target_entry;
						target_entry.xmin = std::numeric_limits<double>::max();
						target_entry.ymin = std::numeric_limits<double>::max();
						target_entry.xmax = std::numeric_limits<double>::lowest();
						target_entry.ymax = std::numeric_limits<double>::lowest();

						const auto beg = target_entry_idx * sgl2::GeometryIndex::NODE_SIZE;
						const auto end = std::min(beg + sgl2::GeometryIndex::NODE_SIZE, source_layer_len);

						for (uint32_t source_entry_idx = beg; source_entry_idx < end; source_entry_idx++) {

							// Read the source entry
							sgl2::ExtentXY source_entry;
							memcpy(&source_entry, source_layer_ptr + source_entry_idx * sizeof(sgl2::ExtentXY), sizeof(sgl2::ExtentXY));

							target_entry.xmin = std::min(target_entry.xmin, source_entry.xmin);
							target_entry.ymin = std::min(target_entry.ymin, source_entry.ymin);
							target_entry.xmax = std::max(target_entry.xmax, source_entry.xmax);
							target_entry.ymax = std::max(target_entry.ymax, source_entry.ymax);

						}

						// Write the target entry
						memcpy(target_layer_ptr + target_entry_idx * sizeof(sgl2::ExtentXY), &target_entry, sizeof(sgl2::ExtentXY));
					}
				}
			}
		}
	} break;
	case WKB_MULTIPOINT:
	case WKB_MULTILINESTRING:
	case WKB_MULTIPOLYGON:
	case WKB_GEOMETRYCOLLECTION: {
		const uint32_t num_geoms = reader.Read<uint32_t>();

		head_writer.Write<uint8_t>(static_cast<uint8_t>(base_type));
		head_writer.Write<uint8_t>(0);          // set flag
		head_writer.Write<uint16_t>(1);         // set padd to 1 to indicate serialized
		head_writer.Write<uint32_t>(num_geoms); // size

		const auto part_offset = body_writer.GetPosition() - head_writer.GetPosition() - sizeof(uint64_t);
		head_writer.Write<uint64_t>(part_offset);

		auto part_writer = body_writer;

		body_writer.Skip(num_geoms * PART_SIZE);

		for (uint32_t i = 0; i < num_geoms; i++) {
			Write(reader, part_writer, body_writer);
		}
	} break;
	default:
		break;
	}
}

struct ST_Prepare {

	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {

		UnaryExecutor::Execute<string_t, string_t>(args.data[0], result, args.size(), [&](string_t &wkb) -> string_t {
			BinaryReader reader(wkb.GetData(), wkb.GetSize());

			const auto required_size = GetRequiredSize(reader);

			auto buffer = StringVector::EmptyString(result, required_size);

			BinaryWriter head_writer(buffer.GetDataWriteable(), required_size);
			BinaryWriter body_writer(buffer.GetDataWriteable(), required_size);
			body_writer.Skip(sizeof(sgl2::GeometryPart));

			reader.Reset();

			Write(reader, head_writer, body_writer);

			buffer.Finalize();
			return buffer;
		});
	}

	static void Register(ExtensionLoader &loader) {
		loader.RegisterFunction(
		    ScalarFunction("ST_Prepare", {LogicalType::GEOMETRY()}, LogicalType::BLOB, ST_Prepare::Execute));
	}
};

sgl2::GeometryPart FromString(const string_t &str) {
	sgl2::GeometryPart geom_part;
	memcpy(&geom_part, str.GetData(), sizeof(sgl2::GeometryPart));

	geom_part.padd = 0;
	geom_part.data = const_cast<char *>(str.GetData()) + sizeof(sgl2::GeometryPart) + reinterpret_cast<uint64_t>(geom_part.data);
	return geom_part;
}

struct ST_Contains {

	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {
		BinaryExecutor::Execute<string_t, string_t, bool>(args.data[0], args.data[1], result, args.size(),
		                                 [&](const string_t &geom1_str, const string_t &geom2_str) -> bool {
             const auto geom1 = FromString(geom1_str);
             const auto geom2 = FromString(geom2_str);
             return sgl2::GeometryRelation::Contains(geom1, geom2);
        });
	}

	static void Register(ExtensionLoader &loader) {
		loader.RegisterFunction(
		    ScalarFunction("ST_Contains", {LogicalType::BLOB, LogicalType::BLOB}, LogicalType::BOOLEAN, ST_Contains::Execute));
	}
};

struct ST_Within {

	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {
		BinaryExecutor::Execute<string_t, string_t, bool>(args.data[0], args.data[1], result, args.size(),
		                                 [&](const string_t &geom1_str, const string_t &geom2_str) -> bool {
			 const auto geom1 = FromString(geom1_str);
			 const auto geom2 = FromString(geom2_str);

			 return sgl2::GeometryRelation::Within(geom1, geom2);
		});
	}

	static void Register(ExtensionLoader &loader) {
		loader.RegisterFunction(
		    ScalarFunction("ST_Within", {LogicalType::BLOB, LogicalType::BLOB}, LogicalType::BOOLEAN, ST_Within::Execute));
	}
};

struct ST_IsEmpty {

	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {
		UnaryExecutor::Execute<string_t, bool>(args.data[0], result, args.size(), [&](const string_t &geom_str) -> bool {
			const auto geom = FromString(geom_str);
			return geom.GetCount() == 0;
		});
	}

	static void Register(ExtensionLoader &loader) {
		loader.RegisterFunction(
		    ScalarFunction("ST_IsEmpty", {LogicalType::BLOB}, LogicalType::BOOLEAN, ST_IsEmpty::Execute));
	}
};

struct ST_Extent_Approx {

	static uint32_t GetExtent(const sgl2::GeometryPart &part, sgl2::ExtentXY &extent) {
		uint32_t count = 0;
		switch (part.GetType()) {
		case sgl2::GeometryType::POINT:
		case sgl2::GeometryType::LINESTRING: {
			const uint32_t num_points = part.GetCount();
			for (uint32_t i = 0; i < num_points; i++) {
				const auto vertex = part.GetVertexXY(i);
				extent.xmin = std::min(extent.xmin, vertex.x);
				extent.ymin = std::min(extent.ymin, vertex.y);
				extent.xmax = std::max(extent.xmax, vertex.x);
				extent.ymax = std::max(extent.ymax, vertex.y);
				count++;
			}
		} break;
		case sgl2::GeometryType::POLYGON:
		case sgl2::GeometryType::MULTI_POINT:
		case sgl2::GeometryType::MULTI_LINESTRING:
		case sgl2::GeometryType::MULTI_POLYGON:
		case sgl2::GeometryType::GEOMETRY_COLLECTION: {
			const uint32_t num_parts = part.GetCount();
			for (uint32_t i = 0; i < num_parts; i++) {
				const auto sub_part = part.GetPart(i);
				count += GetExtent(sub_part, extent);
			}
		} break;
		default:
			break;
		}
		return count;
	}

	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {
		const auto count = args.size();
		auto &input = args.data[0];

		const auto &struct_vec = StructVector::GetEntries(result);
		const auto min_x_data = FlatVector::GetData<float>(*struct_vec[0]);
		const auto min_y_data = FlatVector::GetData<float>(*struct_vec[1]);
		const auto max_x_data = FlatVector::GetData<float>(*struct_vec[2]);
		const auto max_y_data = FlatVector::GetData<float>(*struct_vec[3]);

		UnifiedVectorFormat input_vdata;
		input.ToUnifiedFormat(count, input_vdata);
		const auto input_data = UnifiedVectorFormat::GetData<string_t>(input_vdata);

		for (idx_t i = 0; i < count; i++) {
			const auto row_idx = input_vdata.sel->get_index(i);
			if (input_vdata.validity.RowIsValid(row_idx)) {
				auto &blob = input_data[row_idx];

				// Try to get the cached bounding box from the blob
				sgl2::ExtentXY extent;
				extent.xmin = std::numeric_limits<float>::max();
				extent.ymin = std::numeric_limits<float>::max();
				extent.xmax = std::numeric_limits<float>::lowest();
				extent.ymax = std::numeric_limits<float>::lowest();

				sgl2::GeometryPart geom_part = FromString(blob);
				if (GetExtent(geom_part, extent) != 0) {
					min_x_data[i] = MathUtil::DoubleToFloatDown(extent.xmin);
					min_y_data[i] = MathUtil::DoubleToFloatDown(extent.ymin);
					max_x_data[i] = MathUtil::DoubleToFloatUp(extent.xmax);
					max_y_data[i] = MathUtil::DoubleToFloatUp(extent.ymax);
				} else {
					// No bounding box, return null
					FlatVector::SetNull(result, i, true);
				}
			} else {
				// Null input, return null
				FlatVector::SetNull(result, i, true);
			}
		}

		if (input.GetVectorType() == VectorType::CONSTANT_VECTOR) {
			result.SetVectorType(VectorType::CONSTANT_VECTOR);
		}
	}

	static void Register(ExtensionLoader &loader) {
		loader.RegisterFunction(
		    ScalarFunction("ST_Extent_Approx", {LogicalType::BLOB}, LogicalType::BLOB, ST_Extent_Approx::Execute));
	}
};

} // namespace

void RegisterPrepModule(ExtensionLoader &loader) {

	ST_Prepare::Register(loader);
	ST_Contains::Register(loader);
	ST_Within::Register(loader);

	ST_IsEmpty::Register(loader);
	ST_Extent_Approx::Register(loader);
}

} // namespace duckdb