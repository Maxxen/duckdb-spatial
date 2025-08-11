#include "spatial/modules/geos/geos_serde.hpp"

#include "duckdb/common/typedefs.hpp"
#include "geos_c.h"

#include <duckdb/common/assert.hpp>
#include <spatial/util/binary_writer.hpp>
#include <spatial/util/math.hpp>
#include "spatial/geometry/geometry_processor.hpp"

namespace duckdb {

template <class T>
static T StorageTypeFromGEOS(int type) {
	switch (type) {
	case GEOS_POINT:
		return static_cast<T>(0);
	case GEOS_LINESTRING:
		return static_cast<T>(1);
	case GEOS_POLYGON:
		return static_cast<T>(2);
	case GEOS_MULTIPOINT:
		return static_cast<T>(3);
	case GEOS_MULTILINESTRING:
		return static_cast<T>(4);
	case GEOS_MULTIPOLYGON:
		return static_cast<T>(5);
	case GEOS_GEOMETRYCOLLECTION:
		return static_cast<T>(6);
	default:
		throw InvalidInputException("Unsupported GEOS geometry type %d", type);
	}
}

//----------------------------------------------------------------------------------------------------------------------
// Get Required Size
//----------------------------------------------------------------------------------------------------------------------

static size_t GetCoordSeqLength(const GEOSContextHandle_t ctx, const GEOSCoordSequence *seq) {
	uint32_t len = 0;
	GEOSCoordSeq_getSize_r(ctx, seq, &len);
	return len;
}

static size_t GetRequiredSizeInternal(const GEOSContextHandle_t ctx, const GEOSGeometry *geom) {
	const auto type = GEOSGeomTypeId_r(ctx, geom);
	const bool has_z = GEOSHasZ_r(ctx, geom);
	const bool has_m = GEOSHasM_r(ctx, geom);

	const auto vsize = sizeof(double) * (2 + has_z + has_m);

	switch (type) {
	case GEOS_POINT: {
		return 4 + 4 + (GEOSisEmpty_r(ctx, geom) ? 0 : vsize);
	}
	case GEOS_LINESTRING: {
		const auto line_seq = GEOSGeom_getCoordSeq_r(ctx, geom);
		uint32_t line_len = 0;
		GEOSCoordSeq_getSize_r(ctx, line_seq, &line_len);
		return 4 + 4 + line_len * vsize;
	}
	case GEOS_POLYGON: {
		// 4 bytes for type,
		// 4 bytes for num rings
		//   4 bytes for num points in shell,
		//   vertex_size bytes per point in shell,
		// 4 bytes for num holes,
		//   4 bytes for num points in hole,
		// 	 vertex_size bytes per point in hole
		// 4 bytes padding if (shell + holes) % 2 == 1
		size_t size = 4 + 4;

		const auto exterior_ptr = GEOSGetExteriorRing_r(ctx, geom);
		const auto exterior_seq = GEOSGeom_getCoordSeq_r(ctx, exterior_ptr);
		uint32_t exterior_len = 0;
		GEOSCoordSeq_getSize_r(ctx, exterior_seq, &exterior_len);
		size += 4 + exterior_len * vsize;

		const auto num_rings = GEOSGetNumInteriorRings_r(ctx, geom);
		for (auto i = 0; i < num_rings; i++) {
			const auto interior_ptr = GEOSGetInteriorRingN_r(ctx, geom, i);
			const auto interior_seq = GEOSGeom_getCoordSeq_r(ctx, interior_ptr);
			uint32_t interior_len = 0;
			GEOSCoordSeq_getSize_r(ctx, interior_seq, &interior_len);
			size += 4 + interior_len * vsize;
		}

		// We need to count the shell as well
		if ((num_rings + 1) % 2 != 0) {
			size += 4;
		}
		return size;
	}
	case GEOS_MULTIPOINT:
	case GEOS_MULTILINESTRING:
	case GEOS_MULTIPOLYGON:
	case GEOS_GEOMETRYCOLLECTION: {
		size_t size = 4 + 4;
		const auto num_items = GEOSGetNumGeometries_r(ctx, geom);
		for (auto i = 0; i < num_items; i++) {
			const auto item = GEOSGetGeometryN_r(ctx, geom, i);
			const auto item_size = GetRequiredSizeInternal(ctx, item);
			if (item_size == 0) {
				// Unsupported geometry type
				return 0;
			}
			size += item_size;
		}
		return size;
	}
	default:
		// Unsupported geometry type
		return 0;
	}
}

size_t GeosSerde::GetRequiredSize(GEOSContextHandle_t ctx, const GEOSGeom_t *geom) {
	const auto is_point = (GEOSGeomTypeId_r(ctx, geom) == GEOS_POINT);
	const auto is_empty = GEOSisEmpty_r(ctx, geom);

	const auto has_bbox = !is_point && !is_empty;
	const auto has_z = GEOSHasZ_r(ctx, geom);
	const auto has_m = GEOSHasM_r(ctx, geom);

	const auto dims = 2 + (has_z ? 1 : 0) + (has_m ? 1 : 0);

	const auto head_size = 4 + 4; // type + props + padding
	const auto geom_size = GetRequiredSizeInternal(ctx, geom);
	const auto bbox_size = has_bbox ? dims * sizeof(float) * 2 : 0;

	const auto full_size = head_size + geom_size + bbox_size;

	// Check that the size is a multiple of 8
	D_ASSERT(full_size % 8 == 0);

	return full_size;
}

//----------------------------------------------------------------------------------------------------------------------
// Serialization
//----------------------------------------------------------------------------------------------------------------------

static void SerializeCoordSeq(const GEOSContextHandle_t ctx, const GEOSCoordSequence *seq, bool has_z, bool has_m,
                              size_t len, BinaryWriter &cursor) {
	const auto buffer = cursor.Reserve(len * sizeof(double) * (2 + has_z + has_m));
	GEOSCoordSeq_copyToBuffer_r(ctx, seq, reinterpret_cast<double *>(buffer), has_z, has_m);
}

static void SerializeInternal(const GEOSContextHandle_t ctx, const GEOSGeometry *geom, BinaryWriter &cursor) {
	const auto type = GEOSGeomTypeId_r(ctx, geom);
	const bool has_z = GEOSHasZ_r(ctx, geom);
	const bool has_m = GEOSHasM_r(ctx, geom);

	cursor.Write(StorageTypeFromGEOS<uint32_t>(type));

	switch (type) {
	case GEOS_POINT:
	case GEOS_LINESTRING: {
		if (GEOSisEmpty_r(ctx, geom)) {
			cursor.Write<uint32_t>(0);
			return;
		}
		const auto seq = GEOSGeom_getCoordSeq_r(ctx, geom);
		const auto len = GetCoordSeqLength(ctx, seq);
		cursor.Write<uint32_t>(len);
		SerializeCoordSeq(ctx, seq, has_z, has_m, len, cursor);
		return;
	}
	case GEOS_POLYGON: {
		if (GEOSisEmpty_r(ctx, geom)) {
			cursor.Write<uint32_t>(0);
			return;
		}

		const auto num_rings = GEOSGetNumInteriorRings_r(ctx, geom);

		cursor.Write<uint32_t>(num_rings + 1);

		const auto exterior_ptr = GEOSGetExteriorRing_r(ctx, geom);
		const auto exterior_seq = GEOSGeom_getCoordSeq_r(ctx, exterior_ptr);
		const auto exterior_len = GetCoordSeqLength(ctx, exterior_seq);

		// Save the cursor position to write the ring lengths later
		BinaryWriter len_cursor = cursor;

		// Jump over the ring lengths
		cursor.Skip(sizeof(uint32_t) * (num_rings + 1));

		// Add padding if odd number of rings
		if ((num_rings + 1) % 2 != 0) {
			cursor.Write<uint32_t>(0);
		}

		// Now write both the length and the coordinates in one pass

		// Starting with the exterior ring
		len_cursor.Write<uint32_t>(exterior_len);
		SerializeCoordSeq(ctx, exterior_seq, has_z, has_m, exterior_len, cursor);

		// And for each interior ring
		for (auto i = 0; i < num_rings; i++) {
			const auto interior_ptr = GEOSGetInteriorRingN_r(ctx, geom, i);
			const auto interior_seq = GEOSGeom_getCoordSeq_r(ctx, interior_ptr);
			const auto interior_len = GetCoordSeqLength(ctx, interior_seq);
			len_cursor.Write<uint32_t>(interior_len);
			SerializeCoordSeq(ctx, interior_seq, has_z, has_m, interior_len, cursor);
		}
		return;
	}
	case GEOS_MULTIPOINT:
	case GEOS_MULTILINESTRING:
	case GEOS_MULTIPOLYGON:
	case GEOS_GEOMETRYCOLLECTION: {
		const auto num_items = GEOSGetNumGeometries_r(ctx, geom);
		cursor.Write<uint32_t>(num_items);
		for (auto i = 0; i < num_items; i++) {
			const auto item = GEOSGetGeometryN_r(ctx, geom, i);
			SerializeInternal(ctx, item, cursor);
		}
		return;
	}
	default:
		// Unsupported geometry type
		D_ASSERT(false);
		break;
	}
}

namespace {

struct Point {
	double x;
	double y;
	double z;
	double m;
};

struct Extent {
	Point min;
	Point max;
};

} // namespace

inline void GetCoordSeqExtent(const GEOSContextHandle_t ctx, const GEOSCoordSeq_t *geom, bool has_z, bool has_m,
                              Extent &extent) {

	double x;
	double y;
	double z;
	double m;

	const auto len = GetCoordSeqLength(ctx, geom);

	for (size_t i = 0; i < len; i++) {
		GEOSCoordSeq_getXY_r(ctx, geom, i, &x, &y);
		extent.min.x = std::min(extent.min.x, x);
		extent.min.y = std::min(extent.min.y, y);
		extent.max.x = std::max(extent.max.x, x);
		extent.max.y = std::max(extent.max.y, y);
	}

	if (has_z && has_m) {
		for (size_t i = 0; i < len; i++) {
			GEOSCoordSeq_getZ_r(ctx, geom, i, &z);
			GEOSCoordSeq_getOrdinate_r(ctx, geom, i, 3, &m);
			extent.min.z = std::min(extent.min.z, z);
			extent.min.m = std::min(extent.min.m, m);
			extent.max.z = std::max(extent.max.z, z);
			extent.max.m = std::max(extent.max.m, m);
		}
	} else if (has_z) {
		for (size_t i = 0; i < len; i++) {
			GEOSCoordSeq_getZ_r(ctx, geom, i, &z);
			extent.min.z = std::min(extent.min.z, z);
			extent.max.z = std::max(extent.max.z, z);
		}
	} else if (has_m) {
		for (size_t i = 0; i < len; i++) {
			GEOSCoordSeq_getOrdinate_r(ctx, geom, i, 2, &m);
			extent.min.m = std::min(extent.min.m, m);
			extent.max.m = std::max(extent.max.m, m);
		}
	}
}

inline void GetGeometryExtent(const GEOSContextHandle_t ctx, const GEOSGeometry *geom, bool has_z, bool has_m,
                              Extent &extent) {
	switch (GEOSGeomTypeId_r(ctx, geom)) {
	case GEOS_POINT:
	case GEOS_LINESTRING: {
		if (GEOSisEmpty_r(ctx, geom)) {
			return;
		}
		const auto seq = GEOSGeom_getCoordSeq_r(ctx, geom);
		GetCoordSeqExtent(ctx, seq, has_z, has_m, extent);
		break;
	}
	case GEOS_POLYGON: {
		// We only need to check the exterior ring
		if (GEOSisEmpty_r(ctx, geom)) {
			return;
		}
		const auto exterior_ptr = GEOSGetExteriorRing_r(ctx, geom);
		const auto exterior_seq = GEOSGeom_getCoordSeq_r(ctx, exterior_ptr);
		GetCoordSeqExtent(ctx, exterior_seq, has_z, has_m, extent);
		break;
	}
	case GEOS_MULTIPOINT:
	case GEOS_MULTILINESTRING:
	case GEOS_MULTIPOLYGON:
	case GEOS_GEOMETRYCOLLECTION: {
		const auto num_items = GEOSGetNumGeometries_r(ctx, geom);
		for (auto i = 0; i < num_items; i++) {
			const auto item = GEOSGetGeometryN_r(ctx, geom, i);
			GetGeometryExtent(ctx, item, has_z, has_m, extent);
		}
		break;
	}
	default:
		// Unsupported geometry type
		break;
	}
}

inline void SerializeExtent(const GEOSContextHandle_t ctx, const GEOSGeometry *geom, bool has_z, bool has_m,
                            BinaryWriter &cursor) {

	Extent extent = {};
	extent.min.x = std::numeric_limits<double>::max();
	extent.min.y = std::numeric_limits<double>::max();
	extent.min.z = std::numeric_limits<double>::max();
	extent.min.m = std::numeric_limits<double>::max();
	extent.max.x = std::numeric_limits<double>::lowest();
	extent.max.y = std::numeric_limits<double>::lowest();
	extent.max.z = std::numeric_limits<double>::lowest();
	extent.max.m = std::numeric_limits<double>::lowest();

	GetGeometryExtent(ctx, geom, has_z, has_m, extent);

	cursor.Write<float>(MathUtil::DoubleToFloatDown(extent.min.x));
	cursor.Write<float>(MathUtil::DoubleToFloatDown(extent.min.y));
	cursor.Write<float>(MathUtil::DoubleToFloatUp(extent.max.x));
	cursor.Write<float>(MathUtil::DoubleToFloatUp(extent.max.y));

	if (has_z) {
		cursor.Write<float>(MathUtil::DoubleToFloatDown(extent.min.z));
		cursor.Write<float>(MathUtil::DoubleToFloatUp(extent.max.z));
	}

	if (has_m) {
		cursor.Write<float>(MathUtil::DoubleToFloatDown(extent.min.m));
		cursor.Write<float>(MathUtil::DoubleToFloatUp(extent.max.m));
	}
}

void GeosSerde::Serialize(GEOSContextHandle_t ctx, const GEOSGeom_t *geom, char *buffer, size_t buffer_size) {
	BinaryWriter cursor(buffer, buffer_size);

	const auto type = GEOSGeomTypeId_r(ctx, geom);
	if (type < GEOS_POINT || type > GEOS_GEOMETRYCOLLECTION) {
		// Unsupported geometry type
		throw InvalidInputException("Unsupported GEOS geometry type %d", type);
	}

	const auto has_bbox = (type != GEOS_POINT && (GEOSisEmpty_r(ctx, geom) == 0));
	const auto has_z = GEOSHasZ_r(ctx, geom);
	const auto has_m = GEOSHasM_r(ctx, geom);

	// Set flags
	uint8_t flags = 0;
	flags |= has_z ? 0x01 : 0;
	flags |= has_m ? 0x02 : 0;
	flags |= has_bbox ? 0x04 : 0;

	cursor.Write<uint8_t>(StorageTypeFromGEOS<uint8_t>(type));
	cursor.Write<uint8_t>(flags);
	cursor.Write<uint16_t>(0); // unused
	cursor.Write<uint32_t>(0); // padding

	if (has_bbox) {
		SerializeExtent(ctx, geom, has_z, has_m, cursor);
	}

	// Serialize the geometry
	SerializeInternal(ctx, geom, cursor);
}

//------------------------------------------------------------------------------
// Deserialize
//------------------------------------------------------------------------------
// TODO: Remove the GeometryProcessor from here, come up with something better.

namespace {

template <class T>
bool IsPointerAligned(const void *ptr) {
	auto uintptr = reinterpret_cast<uintptr_t>(ptr);
	return (uintptr % alignof(T)) == 0;
}

class GEOSDeserializer final : GeometryProcessor<GEOSGeometry *> {
private:
	GEOSContextHandle_t ctx;
	vector<double> aligned_buffer;

private:
	GEOSCoordSeq_t *HandleVertexData(const VertexData &vertices) {
		auto n_dims = 2 + (HasZ() ? 1 : 0) + (HasM() ? 1 : 0);
		auto vertex_size = sizeof(double) * n_dims;

		// We know that the data is interleaved :^)
		auto data = vertices.data[0];
		auto count = vertices.count;

		if (HasZ()) {
			// GEOS does a memcpy in this case, so we can pass the buffer directly even if it's not aligned
			return GEOSCoordSeq_copyFromBuffer_r(ctx, reinterpret_cast<const double *>(data), count, HasZ(), HasM());
		} else {
			auto data_ptr = data;
			auto vertex_data = reinterpret_cast<const double *>(data_ptr);
			if (!IsPointerAligned<double>(data_ptr)) {
				// If the pointer is not aligned we need to copy the data to an aligned buffer before passing it to GEOS
				aligned_buffer.clear();
				aligned_buffer.resize(count * n_dims);
				memcpy(aligned_buffer.data(), data_ptr, count * vertex_size);
				vertex_data = aligned_buffer.data();
			}

			return GEOSCoordSeq_copyFromBuffer_r(ctx, vertex_data, count, HasZ(), HasM());
		}
	}

	GEOSGeometry *ProcessPoint(const VertexData &data) override {
		if (data.IsEmpty()) {
			return GEOSGeom_createEmptyPoint_r(ctx);
		} else {
			auto seq = HandleVertexData(data);
			return GEOSGeom_createPoint_r(ctx, seq);
		}
	}

	GEOSGeometry *ProcessLineString(const VertexData &data) override {
		if (data.IsEmpty()) {
			return GEOSGeom_createEmptyLineString_r(ctx);
		} else {
			auto seq = HandleVertexData(data);
			return GEOSGeom_createLineString_r(ctx, seq);
		}
	}

	GEOSGeometry *ProcessPolygon(PolygonState &state) override {
		auto num_rings = state.RingCount();
		if (num_rings == 0) {
			return GEOSGeom_createEmptyPolygon_r(ctx);
		} else {
			// TODO: Make a vector here instead of using new
			auto geoms = new GEOSGeometry *[num_rings];
			for (uint32_t i = 0; i < num_rings; i++) {
				auto vertices = state.Next();
				auto seq = HandleVertexData(vertices);
				geoms[i] = GEOSGeom_createLinearRing_r(ctx, seq);
			}
			auto result = GEOSGeom_createPolygon_r(ctx, geoms[0], geoms + 1, num_rings - 1);
			delete[] geoms;
			return result;
		}
	}

	GEOSGeometry *ProcessCollection(CollectionState &state) override {
		GEOSGeomTypes collection_type = GEOS_GEOMETRYCOLLECTION;
		switch (CurrentType()) {
		case SpatialGeometryType::MULTIPOINT:
			collection_type = GEOS_MULTIPOINT;
			break;
		case SpatialGeometryType::MULTILINESTRING:
			collection_type = GEOS_MULTILINESTRING;
			break;
		case SpatialGeometryType::MULTIPOLYGON:
			collection_type = GEOS_MULTIPOLYGON;
			break;
		default:
			break;
		}
		auto item_count = state.ItemCount();
		if (item_count == 0) {
			return GEOSGeom_createEmptyCollection_r(ctx, collection_type);
		} else {
			auto geoms = new GEOSGeometry *[item_count];
			for (uint32_t i = 0; i < item_count; i++) {
				geoms[i] = state.Next();
			}
			auto result = GEOSGeom_createCollection_r(ctx, collection_type, geoms, item_count);
			delete[] geoms;
			return result;
		}
	}

public:
	explicit GEOSDeserializer(GEOSContextHandle_t ctx) : ctx(ctx) {
	}
	virtual ~GEOSDeserializer() {
	}

	GEOSGeom_t *Execute(const geometry_t &geom) {
		return Process(geom);
	}
};

} // namespace

GEOSGeom_t *GeosSerde::Deserialize(GEOSContextHandle_t ctx, const char *buffer, size_t buffer_size) {
	geometry_t blob(string_t(buffer, buffer_size));
	GEOSDeserializer deserializer(ctx);
	return deserializer.Execute(blob);
}

} // namespace duckdb
