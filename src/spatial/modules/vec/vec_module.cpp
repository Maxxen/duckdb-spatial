#include "spatial/modules/vec/vec_module.hpp"

#include "duckdb/main/extension/extension_loader.hpp"
#include "sgl/sgl.hpp"
#include "spatial/spatial_types.hpp"
#include "spatial/util/binary_reader.hpp"
#include "spatial/util/binary_writer.hpp"

#include <unistd.h>

namespace duckdb {

namespace {

//======================================================================================================================
// Scanner
//======================================================================================================================

struct GeomScanner {
	explicit GeomScanner(Vector &geom_vec, const idx_t count) {
		geom_vec.Flatten(count);

		auto &geom_fields = StructVector::GetEntries(geom_vec);
		auto &parts_vector = *geom_fields[0];
		auto &verts_vector = *geom_fields[1];
		auto &child_vector = *geom_fields[2];

		auto &parts_elem_vec = ListVector::GetEntry(parts_vector);
		auto &verts_elem_vec = ListVector::GetEntry(verts_vector);
		auto &child_elem_vec = ListVector::GetEntry(child_vector);

		auto &parts_fields = StructVector::GetEntries(parts_elem_vec);
		auto &verts_fields = StructVector::GetEntries(verts_elem_vec);

		geom_types = FlatVector::GetData<uint8_t>(*parts_fields[0]);
		geom_offset = FlatVector::GetData<uint32_t>(*parts_fields[1]);
		geom_length = FlatVector::GetData<uint32_t>(*parts_fields[2]);

		geom_xdata = FlatVector::GetData<double>(*verts_fields[0]);
		geom_ydata = FlatVector::GetData<double>(*verts_fields[1]);

		geom_child = FlatVector::GetData<uint32_t>(child_elem_vec);

		parts_lists = ListVector::GetData(parts_vector);
	}

	double *geom_xdata;
	double *geom_ydata;
	uint8_t *geom_types;
	uint32_t *geom_child;
	uint32_t *geom_length;
	uint32_t *geom_offset;

	list_entry_t* parts_lists;

	/*
	struct Part {
		uint32_t id;

		Part GetChild(GeomScanner &all, const uint32_t child) const {
			return Part{all.geom_child[all.geom_offset[id] + child]};
		}
		uint32_t GetCount(GeomScanner &all) const {
			return all.geom_length[id];
		}
		uint8_t GetType(GeomScanner &all) const {
			return all.geom_types[id];
		}
		double& GetX(GeomScanner &all, const uint32_t idx) const {
			return all.geom_xdata[all.geom_offset[id] + idx];
		}
		double& GetY(GeomScanner &all, const uint32_t idx) const {
			return all.geom_ydata[all.geom_offset[id] + idx];
		}
	};*/

	struct Part {
		uint32_t part;
		uint32_t type;
		uint32_t offset;
		uint32_t length;

		uint32_t GetType() const {
			return type;
		}
		uint32_t GetCount() const {
			return length;
		}
	};

	Part GetChild(const Part &part, const uint32_t idx) const {
		const auto child_id = geom_child[geom_offset[part.part] + idx];
		const auto child_offset = geom_offset[child_id];
		const auto child_length = geom_length[child_id];
		const auto child_type = geom_types[child_id];
		return Part{child_id, child_type, child_offset, child_length};
	}

	const double & GetX(const Part &part, const uint32_t idx) const {
		return geom_xdata[part.offset + idx];
	}

	const double & GetY(const Part &part, const uint32_t idx) const {
		return geom_ydata[part.offset + idx];
	}

	Part GetFirstPart(idx_t row_idx) const {
		auto &parts_list = parts_lists[row_idx];
		D_ASSERT(parts_list.length > 0);

		const auto part = parts_list.offset;
		const auto type = geom_types[part];
		const auto offset = geom_offset[part];
		const auto length = geom_length[part];
		return Part{static_cast<uint32_t>(part), type, offset, length};
	}
};



//======================================================================================================================
// Types
//======================================================================================================================

struct Types {
	static LogicalType GEOM_VEC() {
		auto vertex_list = LogicalType::LIST(LogicalType::STRUCT({
			{"x", LogicalType::DOUBLE},
			{"y", LogicalType::DOUBLE},
			{"z", LogicalType::DOUBLE},
			{"m", LogicalType::DOUBLE}
		})); // TODO: Add support for 3D and 4D vertices
		auto child_list = LogicalType::LIST(LogicalType::UINTEGER);
		auto part_list = LogicalType::LIST(LogicalType::STRUCT({
			{"type", LogicalType::UTINYINT},
			{"offset", LogicalType::UINTEGER},
			{"length", LogicalType::UINTEGER}
		}));
		auto type = LogicalType::STRUCT({
			{"parts", std::move(part_list)},
			{"vertices", std::move(vertex_list)},
			{"children", std::move(child_list)},
		});
		return type;
	}
};


//======================================================================================================================
// FromWKB
//======================================================================================================================
struct FromWKB {

	static void GetStats(BinaryReader &reader, uint32_t &num_verts, uint32_t &num_child, uint32_t &num_parts) {

		const auto le = reader.Read<uint8_t>() == 1;
		const auto type_id = reader.Read<uint32_t>(le);
		const auto type = static_cast<sgl::geometry_type>((type_id & 0xffff) % 1000);
		//const auto flags = (type_id & 0xffff) / 1000;
		//const auto has_z = (flags == 1) || (flags == 3) || ((type_id & 0x80000000) != 0);
		//const auto has_m = (flags == 2) || (flags == 3) || ((type_id & 0x40000000) != 0);
		const auto has_srid = (type_id & 0x20000000) != 0;
		if (has_srid) {
			reader.Skip(sizeof(uint32_t)); // Skip SRID
		}

		switch (type) {
		case sgl::geometry_type::POINT: {
			num_parts += 1; // One part for the point
			num_verts += 1;
			reader.Skip(2 * sizeof(double)); // Skip the point coordinates
		} break;
		case sgl::geometry_type::LINESTRING: {
			const auto vertex_count = reader.Read<uint32_t>(le);
			num_parts += 1; // One part for the linestring
			num_verts += vertex_count;
			reader.Skip(vertex_count * 2 * sizeof(double)); // Skip the vertices
		} break;
		case sgl::geometry_type::POLYGON: {
			const auto ring_count = reader.Read<uint32_t>(le);
			num_parts += 1; // One part for the polygon
			for (uint32_t r_idx = 0; r_idx < ring_count; r_idx++) {
				const auto vertex_count = reader.Read<uint32_t>(le);
				num_verts += vertex_count;
				num_child += 1; // Each ring is a child
				num_parts += 1; // Each ring is a part
				reader.Skip(vertex_count * 2 * sizeof(double)); // Skip the vertices
			}
		} break;
		case sgl::geometry_type::MULTI_POINT:
		case sgl::geometry_type::MULTI_LINESTRING:
		case sgl::geometry_type::MULTI_POLYGON:
		case sgl::geometry_type::GEOMETRY_COLLECTION: {
			const auto part_count = reader.Read<uint32_t>(le);
			num_parts += 1; // One part for the collection
			for (uint32_t p_idx = 0; p_idx < part_count; p_idx++) {
				num_child += 1; // Each part is a child
				GetStats(reader, num_verts, num_child, num_parts);
			}
		} break;
		default:
			throw SerializationException("Unknown WKB Geometry type: %d", static_cast<int>(type));
		}
	}

	static void ScanWKB(BinaryReader &reader, idx_t &written_parts, idx_t &written_verts, idx_t &written_child,
		uint8_t* geom_type, uint32_t *geom_offset, uint32_t *geom_length, uint32_t* geom_child, double *geom_xdata, double *geom_ydata) {

		const auto le = reader.Read<uint8_t>() == 1;
		const auto type_id = reader.Read<uint32_t>(le);
		const auto type = static_cast<sgl::geometry_type>((type_id & 0xffff) % 1000);
		//const auto flags = (type_id & 0xffff) / 1000;
		//const auto has_z = (flags == 1) || (flags == 3) || ((type_id & 0x80000000) != 0);
		//const auto has_m = (flags == 2) || (flags == 3) || ((type_id & 0x40000000) != 0);
		const auto has_srid = (type_id & 0x20000000) != 0;
		if (has_srid) {
			reader.Skip(sizeof(uint32_t)); // Skip SRID
		}

		// Now we can start filling the result vector
		switch (type) {
		case sgl::geometry_type::POINT: {
			const auto x = reader.Read<double>(le);
			const auto y = reader.Read<double>(le);

			geom_type[written_parts] = 1; // POINT
			geom_offset[written_parts] = written_verts;
			geom_length[written_parts] = 1; // One vertex
			geom_xdata[written_verts] = x;
			geom_ydata[written_verts] = y;

			written_verts++;
			written_parts++;
		} break;
		case sgl::geometry_type::LINESTRING: {
			const auto vertex_count = reader.Read<uint32_t>(le);

			geom_type[written_parts] = 2;
			geom_offset[written_parts] = written_verts;
			geom_length[written_parts] = vertex_count;

			for (uint32_t v_idx = 0; v_idx < vertex_count; v_idx++) {
				const auto x = reader.Read<double>(le);
				const auto y = reader.Read<double>(le);

				geom_xdata[written_verts] = x;
				geom_ydata[written_verts] = y;

				written_verts++;
			}
			written_parts++;
		} break;
		case sgl::geometry_type::POLYGON: {
			const auto ring_count = reader.Read<uint32_t>(le);
			geom_type[written_parts] = 3; // POLYGON
			geom_offset[written_parts] = written_child;
			geom_length[written_parts] = ring_count;
			written_parts++;

			auto current_child = written_child;
			written_child += ring_count;

			for (uint32_t r_idx = 0; r_idx < ring_count; r_idx++) {
				const auto vertex_count = reader.Read<uint32_t>(le);
				geom_child[current_child] = written_parts; // This ring is a child of the polygon
				geom_offset[written_parts] = written_verts;
				geom_length[written_parts] = vertex_count;

				current_child++;
				written_parts++;

				for (uint32_t v_idx = 0; v_idx < vertex_count; v_idx++) {
					const auto x = reader.Read<double>(le);
					const auto y = reader.Read<double>(le);

					geom_xdata[written_verts] = x;
					geom_ydata[written_verts] = y;

					written_verts++;
				}
			}
		} break;
		case sgl::geometry_type::MULTI_POINT: {
			throw SerializationException("MULTIPOINT is not supported yet in Vec Geometry");
		} break;
		case sgl::geometry_type::MULTI_LINESTRING: {
			throw SerializationException("MULTILINESTRING is not supported yet in Vec Geometry");
		} break;
		case sgl::geometry_type::MULTI_POLYGON: {
			const auto part_count = reader.Read<uint32_t>(le);
			geom_type[written_parts] = 6; // MULTIPOLYGON
			geom_offset[written_parts] = written_child;
			geom_length[written_parts] = part_count;
			written_parts++;

			auto current_child = written_child;
			written_child += part_count;

			for (uint32_t p_idx = 0; p_idx < part_count; p_idx++) {
				geom_child[current_child] = written_parts;

				ScanWKB(reader, written_parts, written_verts, written_child,
						geom_type, geom_offset, geom_length, geom_child, geom_xdata, geom_ydata);

				current_child++;
			}
		} break;
		case sgl::geometry_type::GEOMETRY_COLLECTION: {
			throw SerializationException("GEOMETRYCOLLECTION is not supported yet in Vec Geometry");
		} break;
		default:
			throw SerializationException("Unknown WKB Geometry type: %d", static_cast<int>(type));
			break;
		}
	}

	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {

		// First pass, compute all the sizes required. That is:
		// 1. How many vertices we have in total
		// 2. How many children we have in total
		// 3. How many parts we have in total

		uint32_t total_verts_count = 0;
		uint32_t total_child_count = 0;
		uint32_t total_parts_count = 0;

		UnifiedVectorFormat input_format;
		args.data[0].ToUnifiedFormat(args.size(), input_format);
		const auto &input_data = UnifiedVectorFormat::GetData<string_t>(input_format);

		for (idx_t out_idx = 0; out_idx < args.size(); out_idx++) {
			const auto row_idx = input_format.sel->get_index(out_idx);
			if (!input_format.validity.RowIsValid(row_idx)) {
				continue;
			}
			const auto &blob = input_data[row_idx];

			// Compute the sizes
			BinaryReader reader(blob.GetData(), blob.GetSize());
			GetStats(reader, total_verts_count, total_child_count, total_parts_count);
		}

		// Now, setup the result vector sizes
		auto &output_fields = StructVector::GetEntries(result);
		auto &parts_vector = *output_fields[0];
		auto &verts_vector = *output_fields[1];
		auto &child_vector = *output_fields[2];

		// Allocate space for the lists
		ListVector::Reserve(parts_vector, total_parts_count);
		ListVector::Reserve(verts_vector, total_verts_count);
		ListVector::Reserve(child_vector, total_child_count);
		ListVector::SetListSize(parts_vector, total_parts_count);
		ListVector::SetListSize(verts_vector, total_verts_count);
		ListVector::SetListSize(child_vector, total_child_count);

		auto parts_lists = ListVector::GetData(parts_vector);
		auto verts_lists = ListVector::GetData(verts_vector);
		auto child_lists = ListVector::GetData(child_vector);

		auto parts_elem_vec = ListVector::GetEntry(parts_vector);
		auto verts_elem_vec = ListVector::GetEntry(verts_vector);
		auto child_elem_vec = ListVector::GetEntry(child_vector);

		auto &parts_fields = StructVector::GetEntries(parts_elem_vec);
		auto &verts_fields = StructVector::GetEntries(verts_elem_vec);

		auto geom_xdata = FlatVector::GetData<double>(*verts_fields[0]);
		auto geom_ydata = FlatVector::GetData<double>(*verts_fields[1]);

		if (verts_fields.size() == 4) {
			auto geom_zdata = FlatVector::GetData<double>(*verts_fields[2]);
			auto geom_mdata = FlatVector::GetData<double>(*verts_fields[3]);

			// Zero M and Z to begin with
			memset(geom_zdata, 0, sizeof(double) * total_verts_count);
			memset(geom_mdata, 0, sizeof(double) * total_verts_count);
		}

		auto geom_child = FlatVector::GetData<uint32_t>(child_elem_vec);
		auto geom_type = FlatVector::GetData<uint8_t>(*parts_fields[0]);
		auto geom_offset = FlatVector::GetData<uint32_t>(*parts_fields[1]);
		auto geom_length = FlatVector::GetData<uint32_t>(*parts_fields[2]);

		idx_t written_parts = 0;
		idx_t written_verts = 0;
		idx_t written_child = 0;

		for (idx_t out_idx = 0; out_idx < args.size(); out_idx++) {
			const auto row_idx = input_format.sel->get_index(out_idx);
			if (!input_format.validity.RowIsValid(row_idx)) {
				continue;
			}

			// The range of parts in this geometry
			auto &parts_list = parts_lists[out_idx];
			auto &verts_list = verts_lists[out_idx];
			auto &child_list = child_lists[out_idx];

			parts_list.offset = written_parts;
			verts_list.offset = written_verts;
			child_list.offset = written_child;

			const auto &blob = input_data[row_idx];

			// Now parse the blob...
			BinaryReader reader(blob.GetData(), blob.GetSize());

			// Loop here:
			ScanWKB(reader, written_parts, written_verts, written_child,
					geom_type, geom_offset, geom_length, geom_child, geom_xdata, geom_ydata);

			parts_list.length = written_parts - parts_list.offset;
			verts_list.length = written_verts - verts_list.offset;
			child_list.length = written_child - child_list.offset;
		}

		D_ASSERT(written_parts == total_parts_count);
		D_ASSERT(written_verts == total_verts_count);
		D_ASSERT(written_child == total_child_count);
	}

	static void Register(ExtensionLoader &loader) {
		// Register the FromWKB function
		ScalarFunction func("vec_from_wkb", {LogicalType::BLOB}, Types::GEOM_VEC(), Execute);
		loader.RegisterFunction(std::move(func));
	}
};

//======================================================================================================================
// ST_AsText
//======================================================================================================================

//======================================================================================================================
// ST_Extent
//======================================================================================================================

struct ST_Extent {

	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {

		const auto &output_fields = StructVector::GetEntries(result);
		const auto min_x = FlatVector::GetData<double>(*output_fields[0]);
		const auto min_y = FlatVector::GetData<double>(*output_fields[1]);
		const auto max_x = FlatVector::GetData<double>(*output_fields[2]);
		const auto max_y = FlatVector::GetData<double>(*output_fields[3]);

		args.Flatten();

		auto &geom_fields = StructVector::GetEntries(args.data[0]);
		auto &verts_list = *geom_fields[1];
		auto &verts_fields = StructVector::GetEntries(ListVector::GetEntry(verts_list));

		const auto verts_lists = ListVector::GetData(verts_list);
		const auto verts_xdata = FlatVector::GetData<double>(*verts_fields[0]);
		const auto verts_ydata = FlatVector::GetData<double>(*verts_fields[1]);

		const auto &validity = FlatVector::Validity(args.data[0]);

		for (idx_t out_idx = 0; out_idx < args.size(); out_idx++) {

			if (!validity.RowIsValid(out_idx)) {
				FlatVector::SetNull(result, out_idx, true);
				continue;
			}

			// Initialize the extent to extreme values
			double min_x_val = std::numeric_limits<double>::max();
			double min_y_val = std::numeric_limits<double>::max();
			double max_x_val = std::numeric_limits<double>::lowest();
			double max_y_val = std::numeric_limits<double>::lowest();

			const auto &verts = verts_lists[out_idx];

			for (idx_t vert_idx = verts.offset; vert_idx < verts.offset + verts.length; vert_idx++) {
				min_x_val = std::min(min_x_val, verts_xdata[vert_idx]);
				min_y_val = std::min(min_y_val, verts_ydata[vert_idx]);
				max_x_val = std::max(max_x_val, verts_xdata[vert_idx]);
				max_y_val = std::max(max_y_val, verts_ydata[vert_idx]);
			}

			min_x[out_idx] = min_x_val;
			min_y[out_idx] = min_y_val;
			max_x[out_idx] = max_x_val;
			max_y[out_idx] = max_y_val;
		}

		if (args.AllConstant() || args.size() == 1) {
			result.SetVectorType(VectorType::CONSTANT_VECTOR);
		}
	}

	static void Register(ExtensionLoader &loader) {
		// Register the extent function
		ScalarFunction func("st_extent", {Types::GEOM_VEC()}, GeoTypes::BOX_2D(), Execute);
		loader.RegisterFunction(std::move(func));
	}
};

//======================================================================================================================
// ST_IntersectsExtent
//======================================================================================================================

struct ST_IntersectsExtent {
	//------------------------------------------------------------------------------------------------------------------
	// GEOMETRY
	//------------------------------------------------------------------------------------------------------------------
	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {

		//args.Flatten();

		UnifiedVectorFormat lhs_format;
		args.data[0].ToUnifiedFormat(args.size(), lhs_format);

		UnifiedVectorFormat rhs_format;
		args.data[1].ToUnifiedFormat(args.size(), rhs_format);

		const auto &lhs_geom_fields = StructVector::GetEntries(args.data[0]);
		const auto &rhs_geom_fields = StructVector::GetEntries(args.data[1]);
		auto &lhs_verts_list = *lhs_geom_fields[1];
		auto &rhs_verts_list = *rhs_geom_fields[1];

		const auto &lhs_verts_fields = StructVector::GetEntries(ListVector::GetEntry(lhs_verts_list));
		const auto &rhs_verts_fields = StructVector::GetEntries(ListVector::GetEntry(rhs_verts_list));

		const auto lhs_verts_lists = ListVector::GetData(lhs_verts_list);
		const auto rhs_verts_lists = ListVector::GetData(rhs_verts_list);

		const auto lhs_verts_xdata = FlatVector::GetData<double>(*lhs_verts_fields[0]);
		const auto lhs_verts_ydata = FlatVector::GetData<double>(*lhs_verts_fields[1]);
		const auto rhs_verts_xdata = FlatVector::GetData<double>(*rhs_verts_fields[0]);
		const auto rhs_verts_ydata = FlatVector::GetData<double>(*rhs_verts_fields[1]);

		const auto result_data = FlatVector::GetData<bool>(result);

		for (idx_t out_idx = 0; out_idx < args.size(); out_idx++) {
			const auto rhs_idx = rhs_format.sel->get_index(out_idx);
			const auto lhs_idx = lhs_format.sel->get_index(out_idx);

			if (!lhs_format.validity.RowIsValid(lhs_idx) || !rhs_format.validity.RowIsValid(rhs_idx)) {
				FlatVector::SetNull(result, out_idx, true);
				continue;
			}

			// Initialize the extent to extreme values
			double lhs_min_x_val = std::numeric_limits<double>::max();
			double lhs_min_y_val = std::numeric_limits<double>::max();
			double lhs_max_x_val = std::numeric_limits<double>::lowest();
			double lhs_max_y_val = std::numeric_limits<double>::lowest();
			const auto &lhs_verts = lhs_verts_lists[lhs_idx];

			for (idx_t vert_idx = lhs_verts.offset; vert_idx < lhs_verts.offset + lhs_verts.length; vert_idx++) {
				lhs_min_x_val = std::min(lhs_min_x_val, lhs_verts_xdata[vert_idx]);
				lhs_min_y_val = std::min(lhs_min_y_val, lhs_verts_ydata[vert_idx]);
				lhs_max_x_val = std::max(lhs_max_x_val, lhs_verts_xdata[vert_idx]);
				lhs_max_y_val = std::max(lhs_max_y_val, lhs_verts_ydata[vert_idx]);
			}

			double rhs_min_x_val = std::numeric_limits<double>::max();
			double rhs_min_y_val = std::numeric_limits<double>::max();
			double rhs_max_x_val = std::numeric_limits<double>::lowest();
			double rhs_max_y_val = std::numeric_limits<double>::lowest();
			const auto &rhs_verts = rhs_verts_lists[rhs_idx];

			for (idx_t vert_idx = 0; vert_idx < rhs_verts.offset; vert_idx++) {
				rhs_min_x_val = std::min(rhs_min_x_val, rhs_verts_xdata[vert_idx]);
				rhs_min_y_val = std::min(rhs_min_y_val, rhs_verts_ydata[vert_idx]);
				rhs_max_x_val = std::max(rhs_max_x_val, rhs_verts_xdata[vert_idx]);
				rhs_max_y_val = std::max(rhs_max_y_val, rhs_verts_ydata[vert_idx]);
			}

			// Now we can check if the extents intersect
			const auto intersects = !(lhs_min_x_val > rhs_max_x_val || lhs_max_x_val < rhs_min_x_val ||
			                    lhs_min_y_val > rhs_max_y_val || lhs_max_y_val < rhs_min_y_val);


			result_data[out_idx] = intersects;
		}

		if (args.AllConstant() || args.size() == 1) {
			result.SetVectorType(VectorType::CONSTANT_VECTOR);
		}
	}

	static void Register(ExtensionLoader &loader) {
		ScalarFunction func("st_intersects_extent", {Types::GEOM_VEC(), Types::GEOM_VEC()}, LogicalType::BOOLEAN, Execute);
		loader.RegisterFunction(std::move(func));
	}

};



class BitWriter {
	vector<uint8_t> buffer;
	size_t bit_position = 0;
public:

	size_t GetBitPosition() const {
		return bit_position;
	}

	void Zero() {
		Add(false);
	}
	void One() {
		Add(true);
	}

	template<class T>
	void Bits(const T &value, size_t bits) {
		D_ASSERT(bits <= sizeof(T) * 8);
		for (size_t i = 0; i < bits; i++) {
			Add((value & (1ULL << (bits - i - 1))) != 0);
		}
	}

	void Varint(uint32_t value) {
		// Write a unsigned LEB128 varint
		idx_t offset = 0;
		do {
			uint8_t byte = value & 0x7F;
			value >>= 7;
			if (value != 0) {
				byte |= 0x80; // Set the continuation bit
			}
			Bits(byte, 8); // Write the byte
			offset++;
		} while (value != 0 && offset < 5); // Max 5 bytes for a varint
	}

	void Add(bool bit) {
		if (bit_position / 8 >= buffer.size()) {
			buffer.push_back(0);
		}
		if (bit) {
			buffer[bit_position / 8] |= (1 << (bit_position % 8));
		}
		bit_position++;
	}
	const vector<uint8_t> &GetBuffer() {
		return buffer;
	}
	void Reset() {
		buffer.clear();
		bit_position = 0;
	}
};

class BitReader {
private:
	const char* ptr;
	const char* end;
	size_t bit_position = 0;
public:
	BitReader(const char *buffer, const size_t size) : ptr(buffer), end(buffer + size) {}

	// Not copyable
	BitReader(const BitReader &) = delete;
	BitReader &operator=(const BitReader &) = delete;

	// Not movable
	BitReader(BitReader &&) = delete;
	BitReader &operator=(BitReader &&) = delete;

	bool Bit() {
		if (bit_position / 8 >= (end - ptr)) {
			throw std::runtime_error("BitReader: Attempt to read beyond the end of the buffer");
		}
		const auto byte = ptr[bit_position / 8];
		const bool bit = (byte & (1 << (bit_position % 8))) != 0;
		bit_position++;
		return bit;
	}

	template<class T>
	T Bits(size_t bits) {
		D_ASSERT(bits <= sizeof(T) * 8);
		T value = 0;
		for (size_t i = 0; i < bits; i++) {
			if (Bit()) {
				value |= (1ULL << (bits - i - 1));
			}
		}
		return value;
	}

	uint32_t Varint() {
		uint32_t value = 0;
		idx_t shift = 0;
		uint8_t byte;
		do {
			byte = Bits<uint8_t>(8);
			value |= static_cast<uint32_t>(byte & 0x7F) << shift;
			shift += 7;
		} while (byte & 0x80);
		return value;

	}
};

size_t leading_zeroes(uint64_t value) {
	if (value == 0) {
		return 64;
	}
	size_t count = 0;
	while ((value & (1ULL << (63 - count))) == 0) {
		count++;
	}
	return count;
}

size_t trailing_zeroes(uint64_t value) {
	if (value == 0) {
		return 64;
	}
	size_t count = 0;
	while ((value & (1ULL << count)) == 0) {
		count++;
	}
	return count;
}

void CompressStream(BitWriter &writer, const char* array, uint32_t count, uint32_t stride) {



	uint64_t last = 0;
	uint64_t last_lead_zeroes = 65;
	uint64_t last_back_zeroes = 65;
	uint64_t last_significant_bits = 0;
	uint64_t regret = 0;

	static constexpr auto MAX_REGRET = 30;

	memcpy(&last, array, sizeof(uint64_t));

	// Write first
	writer.Bits(last, 64);

	for (uint32_t i = 1; i < count; i++) {
		uint64_t next = 0;
		memcpy(&next, array + i * sizeof(uint64_t) * stride, sizeof(uint64_t));

		const auto xor_val = next ^ last;
		const auto back_zeroes = trailing_zeroes(xor_val);
		const auto lead_zeroes = leading_zeroes(xor_val);

		if (back_zeroes == 64) {
			writer.Zero();
			continue;
		}

		const auto significant_bits = 64 - back_zeroes - lead_zeroes;
		if (lead_zeroes >= last_lead_zeroes && back_zeroes >= last_back_zeroes && (regret < MAX_REGRET || significant_bits == last_significant_bits)) {
			writer.One();
			writer.Zero();
			const auto xor_new = xor_val >> last_back_zeroes;
			writer.Bits(xor_new, last_significant_bits);

			regret += last_significant_bits - significant_bits;

		} else {
			last_back_zeroes = back_zeroes;
			last_lead_zeroes = lead_zeroes;
			last_significant_bits = significant_bits;
			regret = 0;

			writer.One();
			writer.One();
			writer.Bits(lead_zeroes, 5);
			writer.Bits(significant_bits - 1, 6);
			const auto xor_new = xor_val >> last_back_zeroes;
			writer.Bits(xor_new, significant_bits);
		}

		last = next;
	}
}

void DecompressStream(BitReader &reader, uint32_t count, vector<double> &buffer) {

	buffer.reserve(count * sizeof(double));

	const auto push_double = [&](const uint64_t &bits) {
		// Push a double value to the buffer
		double value = 0;
		memcpy(&value, &bits, sizeof(double));
		buffer.push_back(value);
	};

	const auto first = reader.Bits<uint64_t>(64);
	push_double(first);

	uint64_t last = first;
	uint64_t last_lead_zeroes = 0;
	uint64_t last_back_zeroes = 0;
	uint64_t last_significant_bits = 0;

	for (uint32_t i = 1; i < count; i++) {
		if (!reader.Bit()) {
			push_double(last);
			continue;
		}

		if (reader.Bit()) {
			last_lead_zeroes = reader.Bits<uint64_t>(5);
			last_significant_bits = reader.Bits<uint64_t>(6) + 1;
			last_back_zeroes = 64 - last_lead_zeroes - last_significant_bits;
		}

		const auto xor_val = reader.Bits<uint64_t>(last_significant_bits);
		last ^= (xor_val << last_back_zeroes);
		push_double(last);
	}
}

void DecompressStream(BitReader &reader, uint32_t count, BinaryWriter &writer) {

	const auto push_double = [&](const uint64_t &bits) {
		// Push a double value to the buffer
		double value = 0;
		memcpy(&value, &bits, sizeof(double));
		writer.Write<double>(value);
	};

	const auto first = reader.Bits<uint64_t>(64);
	push_double(first);

	uint64_t last = first;
	uint64_t last_lead_zeroes = 0;
	uint64_t last_back_zeroes = 0;
	uint64_t last_significant_bits = 0;

	for (uint32_t i = 1; i < count; i++) {
		if (!reader.Bit()) {
			push_double(last);
			continue;
		}

		if (reader.Bit()) {
			last_lead_zeroes = reader.Bits<uint64_t>(5);
			last_significant_bits = reader.Bits<uint64_t>(6) + 1;
			last_back_zeroes = 64 - last_lead_zeroes - last_significant_bits;
		}

		const auto xor_val = reader.Bits<uint64_t>(last_significant_bits);
		last ^= (xor_val << last_back_zeroes);
		push_double(last);
	}
}

class GeometryDecompressor {
public:
	string_t Decompress(const string_t &blob, Vector &result) {
		BitReader reader(blob.GetData(), blob.GetSize());

		first_vertex = true; // Reset the first vertex flag

		const auto total_size = reader.Varint();
		auto geom = StringVector::EmptyString(result, total_size);
		BinaryWriter writer(geom.GetDataWriteable(), geom.GetSize());
		DecompressRecursive(reader, writer);
		geom.Finalize();
		return geom;
	}
	sgl::geometry_type first_type = sgl::geometry_type::INVALID;
private:

	uint64_t last_bits[4] = {};	// last bits read from the stream
	uint64_t last_sign[4] = {};	// number of significant bits in the last bits
	uint64_t last_lead[4] = {};	// number of leading zeroes in the last bits
	uint64_t last_back[4] = {};	// number of trailing zeroes in the last bits
	bool first_vertex = true; // Flag to indicate if we are processing the first vertex

	template<int N>
	void DecompressVertices(BitReader &reader, BinaryWriter &writer, uint32_t count) {
		D_ASSERT(count > 0);
		static_assert(N <= 4, "N must be less than or equal to 4");
		uint32_t vert_idx = 0;

		if (first_vertex) {
			for (uint32_t axis_idx = 0; axis_idx < N; axis_idx++) {
				const auto first = reader.Bits<uint64_t>(64);

				// Initialize the state
				last_bits[axis_idx] = first;
				last_sign[axis_idx] = 0;
				last_lead[axis_idx] = 0;  // 65 leading zeroes means no bits set
				last_back[axis_idx] = 65; // 65 trailing zeroes means no bits set

				// Write the first value
				double val;
				memcpy(&val, &first, sizeof(double));
				writer.Write<double>(val);
			}

			first_vertex = false; // We have processed the first vertex
			vert_idx = 1;
		}

		for (; vert_idx < count; vert_idx++) {
			for (uint32_t axis_idx = 0; axis_idx < N; axis_idx++) {
				if (!reader.Bit()) {
					double val;
					memcpy(&val, &last_bits[axis_idx], sizeof(double));
					writer.Write<double>(val);
				} else {

					if (reader.Bit()) {
						last_lead[axis_idx] = static_cast<uint32_t>(reader.Bits<uint64_t>(5));
						last_sign[axis_idx] = static_cast<uint32_t>(reader.Bits<uint64_t>(6)) + 1;
						last_back[axis_idx] = 64 - last_lead[axis_idx] - last_sign[axis_idx];
					}

					const auto xor_val = reader.Bits<uint64_t>(last_sign[axis_idx]);
					last_bits[axis_idx] ^= (xor_val << last_back[axis_idx]);
					double val;
					uint64_t bits = last_bits[axis_idx];
					memcpy(&val, &bits, sizeof(double));
					writer.Write<double>(val);
				}
			}
		}
	}

	void DecompressRecursive(BitReader &reader, BinaryWriter &writer) {
		const auto type = static_cast<sgl::geometry_type>(reader.Bits<uint8_t>(3));
		if (first_type == sgl::geometry_type::INVALID) {
			first_type = type; // We need to remember the first type
			writer.Write<uint8_t>(static_cast<uint8_t>(type) - 1);
			writer.Write<uint8_t>(0); // No flags for now
			writer.Write<uint16_t>(0); // SRID, not used
			writer.Write<uint32_t>(0); // Padding, not used
		}

		writer.Write<uint32_t>(static_cast<uint32_t>(type) - 1); // Write the type ID
		const auto empty = reader.Bit() == 0;
		if (empty) {
			writer.Write<uint32_t>(0); // Write empty count
			return;
		}
		const auto count = reader.Varint();
		writer.Write<uint32_t>(count); // Write count;

		switch (type) {
		case sgl::geometry_type::POINT:
		case sgl::geometry_type::LINESTRING:
			DecompressVertices<2>(reader, writer, count);
			break;
		case sgl::geometry_type::POLYGON: {
			auto ring_writer = writer;
			writer.Skip((count * 4) + (count % 2 == 1 ? 4 : 0)); // Reserve space for rings
			for (uint32_t i = 0; i < count; i++) {
				const auto ring_type = static_cast<sgl::geometry_type>(reader.Bits<uint8_t>(3) - 1);
				D_ASSERT(ring_type == sgl::geometry_type::LINESTRING);

				const auto ring_empty = reader.Bit() == 0;
				if (ring_empty) {
					ring_writer.Write<uint32_t>(0); // Write empty ring size
					continue;
				}
				const auto ring_size = reader.Varint();
				ring_writer.Write<uint32_t>(ring_size); // Write the ring size
				DecompressVertices<2>(reader, writer, ring_size);
			}
		} break;
		case sgl::geometry_type::MULTI_POINT:
		case sgl::geometry_type::MULTI_LINESTRING:
		case sgl::geometry_type::MULTI_POLYGON:
		case sgl::geometry_type::GEOMETRY_COLLECTION: {
			for (uint32_t i = 0; i < count; i++) {
				DecompressRecursive(reader, writer);
			}
		} break;
		default:
			throw SerializationException("Unknown WKB Geometry type: %d", static_cast<int>(type));
		}
	}
};


class GeometryCompressor {
public:
	string_t Compress(const string_t &geom, Vector &result) {

		BinaryReader reader(geom.GetData(), geom.GetSize());

		reader.Skip(sizeof(uint8_t));
		const auto flag = reader.Read<uint8_t>();
		reader.Skip(sizeof(uint16_t)); // Skip SRID
		reader.Skip(sizeof(uint32_t)); // Skip padding

		// Parse flags
		const auto has_z = (flag & 0x01) != 0;
		const auto has_m = (flag & 0x02) != 0;
		const auto has_bbox = (flag & 0x04) != 0;

		const auto format_v1 = (flag & 0x040) != 0;
		const auto format_v0 = (flag & 0x080) != 0;

		if (format_v0 || format_v1) {
			// This is a legacy format, we need to convert it to the new format
			throw SerializationException("Legacy WKB format is not supported in Vec Geometry");
		}

		if (has_bbox) {
			reader.Skip(sizeof(float) * 2 * (2 + has_z + has_m)); // Skip bounding box
		}

		if (has_z || has_m) {
			// Skip Z and M values
			throw SerializationException("Z and M values are not supported in Vec Geometry");
		}

		// Reset states
		stream.Reset();
		first_vertex = true;

		// Write the total uncompressed length as uint32_t varint
		stream.Varint(geom.GetSize());
		CompressRecursive(reader);

		const auto &buffer = stream.GetBuffer();

		auto blob = StringVector::EmptyString(result, buffer.size());
		memcpy(blob.GetDataWriteable(), buffer.data(), buffer.size());
		blob.Finalize();
		return blob;
	}
private:
	uint64_t last_bits[4] = {};	// last bits read from the stream
	uint64_t last_sign[4] = {};	// number of significant bits in the last bits
	uint64_t last_lead[4] = {};	// number of leading zeroes in the last bits
	uint64_t last_back[4] = {};	// number of trailing zeroes in the last bits
	uint64_t regret[4] = {};		// how many bits we regret not writing in the last step
	bool first_vertex = true; // Flag to indicate if we are processing the first vertex

	template<int N>
	void CompressVertices(BinaryReader &reader, uint32_t count) {
		D_ASSERT(count > 0);
		const auto vertex_array = reader.Reserve(static_cast<size_t>(count) * 2 * sizeof(double));

		static_assert(N <= 4, "N must be less than or equal to 4");
		uint32_t vert_idx = 0;

		if (first_vertex) {
			for (uint32_t axis_idx = 0; axis_idx < N; axis_idx++) {
				// Initialize the state
				last_bits[axis_idx] = 0;
				last_sign[axis_idx] = 0;
				last_lead[axis_idx] = 65; // 65 leading zeroes means no bits set
				last_back[axis_idx] = 65; // 65 trailing zeroes means no bits set
				regret[axis_idx] = 0;

				//Read the first value for this axis
				uint64_t val;
				memcpy(&val, vertex_array + axis_idx * sizeof(double), sizeof(double));

				last_bits[axis_idx] = val;

				// Write the first value
				stream.Bits(last_bits[axis_idx], 64);
			}

			first_vertex = false; // We have processed the first vertex
			vert_idx = 1;
		}

		for (; vert_idx < count; vert_idx++) {
			for (uint32_t axis_idx = 0; axis_idx < N; axis_idx++) {

				uint64_t next_bits;
				memcpy(&next_bits, vertex_array + (vert_idx * N + axis_idx) * sizeof(double), sizeof(double));

				const auto xor_val = next_bits ^ last_bits[axis_idx];

				uint64_t next_back = trailing_zeroes(xor_val);
				uint64_t next_lead = std::min<uint64_t>(leading_zeroes(xor_val), 31);

				if (next_back == 64) {
					stream.Zero();
					continue; // No significant bits, write zero
				}

				uint64_t next_sign = 64 - next_back - next_lead;

				if (next_lead >= last_lead[axis_idx] &&
					next_back >= last_back[axis_idx] &&
					(regret[axis_idx] < 30 || next_sign == last_sign[axis_idx])) {
					stream.One();
					stream.Zero();
					const auto xor_new = xor_val >> last_back[axis_idx];
					stream.Bits(xor_new, last_sign[axis_idx]);
					regret[axis_idx] += last_sign[axis_idx] - next_sign;
				} else {
					last_back[axis_idx] = next_back;
					last_lead[axis_idx] = next_lead;
					last_sign[axis_idx] = next_sign;
					regret[axis_idx] = 0;

					stream.One();
					stream.One();
					stream.Bits(next_lead, 5);
					stream.Bits(next_sign - 1, 6);
					const auto xor_new = xor_val >> next_back;
					stream.Bits(xor_new, next_sign);
				}
				last_bits[axis_idx] = next_bits;
			}
		}
	}

	void CompressRecursive(BinaryReader &reader) {
		const auto type = static_cast<sgl::geometry_type>(reader.Read<uint32_t>() + 1);
		const auto count = reader.Read<uint32_t>();

		// 3 bit type + 1 bit empty
		stream.Bits(static_cast<uint8_t>(type), 3);
		if (count == 0) {
			stream.Zero();
			return; // Nothing to compress
		}
		stream.One(); // Not empty

		// Write the count of items
		stream.Varint(count);

		switch (type) {
			case sgl::geometry_type::POINT:
			case sgl::geometry_type::LINESTRING: {
				CompressVertices<2>(reader, count);
			} break;
			case sgl::geometry_type::POLYGON: {
				auto ring_reader = reader;
				reader.Skip((count * 4) + (count % 2 == 1 ? 4 : 0));
				for (uint32_t i = 0; i < count; i++) {
					const auto ring_count = ring_reader.Read<uint32_t>();

					stream.Bits(static_cast<uint8_t>(sgl::geometry_type::LINESTRING) + 1, 3); // LINESTRING type
					if (ring_count == 0) {
						stream.Zero();
						continue; // Nothing to compress
					}
					stream.One(); // Not empty

					// TODO: write varint
					stream.Varint(ring_count);

					// Compress the vertices of the ring
					CompressVertices<2>(reader, ring_count);
				}
			} break;
			case sgl::geometry_type::MULTI_POINT:
			case sgl::geometry_type::MULTI_LINESTRING:
			case sgl::geometry_type::MULTI_POLYGON:
			case sgl::geometry_type::GEOMETRY_COLLECTION: {
				for (uint32_t i = 0; i < count; i++) {
					CompressRecursive(reader);
				}
			} break;
			default:
				throw SerializationException("Unknown Geometry type: %d", static_cast<int>(type));
		}
	}

	BitWriter stream;
};

struct Compressor {
	BinaryWriter &writer;
	BitWriter &x_writer;
	BitWriter &y_writer;
};

static void CompressGeometryInternal(BinaryReader &reader, Compressor &encoder) {
	const auto type = static_cast<sgl::geometry_type>(reader.Read<uint32_t>() + 1);
	const auto count = reader.Read<uint32_t>();

	encoder.writer.Write(static_cast<uint8_t>(type));
	encoder.writer.Write(static_cast<uint32_t>(count));

	if (count == 0) {
		return; // Nothing to compress
	}

	switch (type) {
		case sgl::geometry_type::POINT:
		case sgl::geometry_type::LINESTRING: {
			encoder.writer.Write<uint32_t>(encoder.x_writer.GetBitPosition());
			encoder.writer.Write<uint32_t>(encoder.y_writer.GetBitPosition());

			const auto vertex_array = reader.Reserve(count * 2 * sizeof(double));

			CompressStream(encoder.x_writer, vertex_array, count, 2);
			CompressStream(encoder.y_writer, vertex_array + sizeof(double), count, 2);

		} break;
		case sgl::geometry_type::POLYGON: {
			auto ring_reader = reader;
			reader.Skip((count * 4) + (count % 2 == 1 ? 4 : 0));
			for (uint32_t i = 0; i < count; i++) {
				const auto ring_count = ring_reader.Read<uint32_t>();

				encoder.writer.Write<uint8_t>(static_cast<uint8_t>(sgl::geometry_type::LINESTRING));
				encoder.writer.Write<uint32_t>(ring_count);
				encoder.writer.Write<uint32_t>(encoder.x_writer.GetBitPosition());
				encoder.writer.Write<uint32_t>(encoder.y_writer.GetBitPosition());

				const auto vertex_array = reader.Reserve(ring_count * 2 * sizeof(double));

				CompressStream(encoder.x_writer, vertex_array, ring_count, 2);
				CompressStream(encoder.y_writer, vertex_array + sizeof(double), ring_count, 2);

			}
		} break;
		case sgl::geometry_type::MULTI_POINT:
		case sgl::geometry_type::MULTI_LINESTRING:
		case sgl::geometry_type::MULTI_POLYGON:
		case sgl::geometry_type::GEOMETRY_COLLECTION: {
			for (uint32_t i = 0; i < count; i++) {
				CompressGeometryInternal(reader, encoder);
			}
		} break;
		default:
			throw SerializationException("Unknown Geometry type: %d", static_cast<int>(type));
	}
}

static string_t CompressGeometry(const string_t &geom, Vector &result) {
	BinaryReader reader(geom.GetData(), geom.GetSize());

	const auto type = static_cast<sgl::geometry_type>(reader.Read<uint8_t>() + 1);
	const auto flag = reader.Read<uint8_t>();
	reader.Skip(sizeof(uint16_t)); // Skip SRID
	reader.Skip(sizeof(uint32_t)); // Skip padding

	// Parse flags
	const auto has_z = (flag & 0x01) != 0;
	const auto has_m = (flag & 0x02) != 0;
	const auto has_bbox = (flag & 0x04) != 0;

	const auto format_v1 = (flag & 0x040) != 0;
	const auto format_v0 = (flag & 0x080) != 0;

	if (format_v0 || format_v1) {
		// This is a legacy format, we need to convert it to the new format
		throw SerializationException("Legacy WKB format is not supported in Vec Geometry");
	}

	if (has_bbox) {
		reader.Skip(sizeof(float) * 2 * (2 + has_z + has_m)); // Skip bounding box
	}

	if (has_z || has_m) {
		// Skip Z and M values
		throw SerializationException("Z and M values are not supported in Vec Geometry");
	}

	vector<char> main_buffer;
	main_buffer.resize(geom.GetSize());
	BinaryWriter writer(main_buffer.data(), main_buffer.size());

	BitWriter x_writer;
	BitWriter y_writer;

	Compressor compressor{writer, x_writer, y_writer};

	CompressGeometryInternal(reader, compressor);

	auto &x_buffer = x_writer.GetBuffer();
	auto &y_buffer = y_writer.GetBuffer();

	const auto main_buffer_size = writer.GetWrittenSize();
	auto required_size = 2 * sizeof(uint32_t) + x_buffer.size() + y_buffer.size() + main_buffer_size;

	auto blob = StringVector::EmptyString(result, required_size);

	BinaryWriter blob_writer(blob.GetDataWriteable(), blob.GetSize());
	// Write the offsets for the x and y buffers
	blob_writer.Write<uint32_t>(main_buffer_size);
	blob_writer.Write<uint32_t>(main_buffer_size + x_buffer.size());

	// Now write the main buffer
	blob_writer.Copy(main_buffer.data(), main_buffer_size);
	// Write the x buffer
	blob_writer.Copy(const_char_ptr_cast(x_buffer.data()), x_buffer.size());
	// Write the y buffer
	blob_writer.Copy(const_char_ptr_cast(y_buffer.data()), y_buffer.size());

	// Finalize
	blob.Finalize();

	return blob;
}

struct ST_Compress {
	static void ExecuteList(DataChunk &args, ExpressionState &state, Vector &result) {

		const auto elem_vec = ListVector::GetEntry(args.data[0]);
		const auto elem_data = FlatVector::GetData<double>(elem_vec);

		BitWriter writer;
		UnaryExecutor::Execute<list_entry_t, string_t>(args.data[0], result, args.size(),
			[&](const list_entry_t &entry) {
				writer.Reset();

				writer.Bits(entry.length, 32); // Write the count of elements
				CompressStream(writer, reinterpret_cast<const char*>(elem_data + entry.offset), entry.length, 1);
				const auto &buffer = writer.GetBuffer();
				return StringVector::AddStringOrBlob(result, const_char_ptr_cast(buffer.data()), buffer.size());
			});
	}

	static void ExecuteGeometry(DataChunk &args, ExpressionState &state, Vector &result) {
		UnaryExecutor::Execute<string_t, string_t>(args.data[0], result, args.size(),
			[&](const string_t &geom) {
				GeometryCompressor compressor;
				return compressor.Compress(geom, result);
			});
	}

	static void Register(ExtensionLoader &loader) {
		ScalarFunction func_v("st_compress_list", {LogicalType::LIST(LogicalType::DOUBLE)}, LogicalType::BLOB, ExecuteList);
		loader.RegisterFunction(std::move(func_v));

		ScalarFunction func_g("st_compress", {GeoTypes::GEOMETRY()}, LogicalType::BLOB, ExecuteGeometry);
		loader.RegisterFunction(std::move(func_g));
	}
};

struct ST_Decompress {
	static void ExecuteList(DataChunk &args, ExpressionState &state, Vector &result) {

		vector<double> buffer;
		idx_t total_size = 0;
		auto &elem_vec = ListVector::GetEntry(result);

		UnaryExecutor::Execute<string_t, list_entry_t>(args.data[0], result, args.size(),
			[&](const string_t &blob) {
				buffer.clear();

				BitReader reader(blob.GetData(), blob.GetSize());
				const auto count = reader.Bits<uint32_t>(32); // Read the count of elements
				if (count != 0) {
					DecompressStream(reader, count, buffer);
				}

				list_entry_t entry = { total_size, buffer.size()};

				total_size += buffer.size();

				ListVector::Reserve(result, total_size);
				ListVector::SetListSize(result, total_size);

				const auto elem_data = FlatVector::GetData<double>(elem_vec);
				for (size_t i = 0; i < buffer.size(); i++) {
					elem_data[entry.offset + i] = buffer[i];
				}
				return entry;
			});
	}

	static void ExecuteGeometry(DataChunk &args, ExpressionState &state, Vector &result) {
		UnaryExecutor::Execute<string_t, string_t>(args.data[0], result, args.size(),
			[&](const string_t &blob) {
				GeometryDecompressor decompressor;
				return decompressor.Decompress(blob, result);
			});
	}

	static void Register(ExtensionLoader &loader) {
		ScalarFunction func("st_decompress_list", {LogicalType::BLOB}, LogicalType::LIST(LogicalType::DOUBLE), ExecuteList);
		loader.RegisterFunction(std::move(func));

		ScalarFunction func_g("st_decompress", {LogicalType::BLOB}, GeoTypes::GEOMETRY(), ExecuteGeometry);
		loader.RegisterFunction(std::move(func_g));
	}
};

struct ST_Size {
	static void Execute(DataChunk &args, ExpressionState &state, Vector &result) {
		UnaryExecutor::Execute<string_t, int64_t>(args.data[0], result, args.size(),
			[&](const string_t &blob) {
				return blob.GetSize();
			});
	}

	static void Register(ExtensionLoader &loader) {
		ScalarFunctionSet set("ST_Size");
		set.AddFunction(ScalarFunction({LogicalType::BLOB}, LogicalType::BIGINT, Execute));
		set.AddFunction(ScalarFunction({GeoTypes::GEOMETRY()}, LogicalType::BIGINT, Execute));
		loader.RegisterFunction(std::move(set));
	}
};

} // namespace

//######################################################################################################################
// Register
//######################################################################################################################

void RegisterVecModule(ExtensionLoader &loader) {
	// Register the Vec module
	loader.RegisterType("GEOM_VEC", Types::GEOM_VEC());
	FromWKB::Register(loader);

	ST_Extent::Register(loader);
	ST_IntersectsExtent::Register(loader);

	ST_Compress::Register(loader);
	ST_Decompress::Register(loader);
	ST_Size::Register(loader);
}

}