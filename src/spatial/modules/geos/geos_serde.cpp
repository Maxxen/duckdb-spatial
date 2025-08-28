#include "spatial/modules/geos/geos_serde.hpp"
#include "spatial/util/binary_reader.hpp"
#include "spatial/util/binary_writer.hpp"
#include "spatial/util/math.hpp"

#include "duckdb/common/types/geometry.hpp"

#include "geos_c.h"

namespace duckdb {

//----------------------------------------------------------------------------------------------------------------------
// Get Required Size
//----------------------------------------------------------------------------------------------------------------------

static size_t GetCoordSeqLength(const GEOSContextHandle_t ctx, const GEOSCoordSequence *seq) {
	uint32_t len = 0;
	GEOSCoordSeq_getSize_r(ctx, seq, &len);
	return len;
}

static void GetRequiredSizeRecursive(GEOSContextHandle_t ctx, const GEOSGeom_t *geom, uint32_t &total_size) {
	const auto type = GEOSGeomTypeId_r(ctx, geom);
	const auto has_z = GEOSHasZ_r(ctx, geom);
	const auto has_m = GEOSHasM_r(ctx, geom);

	const auto vertex_width = sizeof(double) * (2 + has_z + has_m);

	switch (type) {
	case GEOS_POINT: {
		total_size += 1 + 4 + vertex_width;
	} break;
	case GEOS_LINESTRING: {
		const auto seq = GEOSGeom_getCoordSeq_r(ctx, geom);
		const auto len = GetCoordSeqLength(ctx, seq);
		total_size += 1 + 4 + 4 + len * vertex_width;
	} break;
	case GEOS_POLYGON: {
		total_size += 1 + 4 + 4;
		if (GEOSisEmpty_r(ctx, geom)) {
			return;
		}
		const auto shell = GEOSGetExteriorRing_r(ctx, geom);
		const auto shell_seq = GEOSGeom_getCoordSeq_r(ctx, shell);
		const auto shell_len = GetCoordSeqLength(ctx, shell_seq);

		total_size += 4 + shell_len * vertex_width;

		const auto num_rings = GEOSGetNumInteriorRings_r(ctx, geom);
		for (auto i = 0; i < num_rings; i++) {
			const auto ring = GEOSGetInteriorRingN_r(ctx, geom, i);
			const auto ring_seq = GEOSGeom_getCoordSeq_r(ctx, ring);
			const auto ring_len = GetCoordSeqLength(ctx, ring_seq);
			total_size += 4 + ring_len * vertex_width;
		}
	} break;
	case GEOS_MULTIPOINT:
	case GEOS_MULTILINESTRING:
	case GEOS_MULTIPOLYGON:
	case GEOS_GEOMETRYCOLLECTION: {
		total_size += 1 + 4 + 4;
		const auto num_items = GEOSGetNumGeometries_r(ctx, geom);
		for (auto i = 0; i < num_items; i++) {
			const auto item = GEOSGetGeometryN_r(ctx, geom, i);
			GetRequiredSizeRecursive(ctx, item, total_size);
		}
	} break;
	default:
		break;
	}
}

size_t GeosSerde::GetRequiredSize(GEOSContextHandle_t ctx, const GEOSGeom_t *geom) {
	uint32_t total_size = 0;
	GetRequiredSizeRecursive(ctx, geom, total_size);
	return total_size;
}

static void SerializeRecursive(GEOSContextHandle_t ctx, const GEOSGeom_t *geom, BinaryWriter &writer) {
	const auto type = GEOSGeomTypeId_r(ctx, geom);
	const auto has_z = GEOSHasZ_r(ctx, geom);
	const auto has_m = GEOSHasM_r(ctx, geom);

	writer.Write<uint8_t>(1);

	const auto vertex_width = sizeof(double) * (2 + has_z + has_m);
	const auto flags = static_cast<uint32_t>((has_z ? 1000 : 0) + (has_m ? 2000 : 0));

	switch (type) {
	case GEOS_POINT: {
		const auto type_id = static_cast<uint32_t>(GeometryType::POINT) + flags;
		writer.Write<uint32_t>(type_id);
		if (GEOSisEmpty_r(ctx, geom)) {
			constexpr auto nan = std::numeric_limits<double>::quiet_NaN();
			constexpr double empty_vertex[4] = {nan, nan, nan, nan};
			writer.Copy(reinterpret_cast<const char *>(empty_vertex), vertex_width);
			return;
		}
		const auto seq = GEOSGeom_getCoordSeq_r(ctx, geom);
		const auto len = GetCoordSeqLength(ctx, seq);
		const auto buffer = writer.Reserve(len * vertex_width);
		GEOSCoordSeq_copyToBuffer_r(ctx, seq, reinterpret_cast<double *>(buffer), has_z, has_m);
	} break;
	case GEOS_LINESTRING: {
		const auto type_id = static_cast<uint32_t>(GeometryType::LINESTRING) + flags;
		writer.Write<uint32_t>(type_id);
		if (GEOSisEmpty_r(ctx, geom)) {
			writer.Write<uint32_t>(0);
			return;
		}
		const auto seq = GEOSGeom_getCoordSeq_r(ctx, geom);
		const auto len = GetCoordSeqLength(ctx, seq);
		writer.Write<uint32_t>(len);
		const auto buffer = writer.Reserve(len * vertex_width);
		GEOSCoordSeq_copyToBuffer_r(ctx, seq, reinterpret_cast<double *>(buffer), has_z, has_m);
	} break;
	case GEOS_POLYGON: {
		const auto type_id = static_cast<uint32_t>(GeometryType::POLYGON) + flags;
		writer.Write<uint32_t>(type_id);
		if (GEOSisEmpty_r(ctx, geom)) {
			writer.Write<uint32_t>(0);
			return;
		}
		const auto num_rings = GEOSGetNumInteriorRings_r(ctx, geom);
		writer.Write<uint32_t>(num_rings + 1);

		const auto shell = GEOSGetExteriorRing_r(ctx, geom);
		const auto shell_seq = GEOSGeom_getCoordSeq_r(ctx, shell);
		const auto shell_len = GetCoordSeqLength(ctx, shell_seq);

		writer.Write<uint32_t>(shell_len);
		const auto shell_buffer = writer.Reserve(shell_len * vertex_width);
		GEOSCoordSeq_copyToBuffer_r(ctx, shell_seq, reinterpret_cast<double *>(shell_buffer), has_z, has_m);

		for (auto i = 0; i < num_rings; i++) {
			const auto ring = GEOSGetInteriorRingN_r(ctx, geom, i);
			const auto ring_seq = GEOSGeom_getCoordSeq_r(ctx, ring);
			const auto ring_len = GetCoordSeqLength(ctx, ring_seq);
			writer.Write<uint32_t>(ring_len);
			const auto ring_buffer = writer.Reserve(ring_len * vertex_width);
			GEOSCoordSeq_copyToBuffer_r(ctx, ring_seq, reinterpret_cast<double *>(ring_buffer), has_z, has_m);
		}
	} break;
	case GEOS_MULTIPOINT: {
		const auto type_id = static_cast<uint32_t>(GeometryType::MULTIPOINT) + flags;
		writer.Write<uint32_t>(type_id);
		const auto num_items = GEOSGetNumGeometries_r(ctx, geom);
		writer.Write<uint32_t>(num_items);
		for (auto i = 0; i < num_items; i++) {
			const auto item = GEOSGetGeometryN_r(ctx, geom, i);
			SerializeRecursive(ctx, item, writer);
		}
	} break;
	case GEOS_MULTILINESTRING: {
		const auto type_id = static_cast<uint32_t>(GeometryType::MULTILINESTRING) + flags;
		writer.Write<uint32_t>(type_id);
		const auto num_items = GEOSGetNumGeometries_r(ctx, geom);
		writer.Write<uint32_t>(num_items);
		for (auto i = 0; i < num_items; i++) {
			const auto item = GEOSGetGeometryN_r(ctx, geom, i);
			SerializeRecursive(ctx, item, writer);
		}
	} break;
	case GEOS_MULTIPOLYGON: {
		const auto type_id = static_cast<uint32_t>(GeometryType::MULTIPOLYGON) + flags;
		writer.Write<uint32_t>(type_id);
		const auto num_items = GEOSGetNumGeometries_r(ctx, geom);
		writer.Write<uint32_t>(num_items);
		for (auto i = 0; i < num_items; i++) {
			const auto item = GEOSGetGeometryN_r(ctx, geom, i);
			SerializeRecursive(ctx, item, writer);
		}
	} break;
	case GEOS_GEOMETRYCOLLECTION: {
		const auto type_id = static_cast<uint32_t>(GeometryType::GEOMETRYCOLLECTION) + flags;
		writer.Write<uint32_t>(type_id);
		const auto num_items = GEOSGetNumGeometries_r(ctx, geom);
		writer.Write<uint32_t>(num_items);
		for (auto i = 0; i < num_items; i++) {
			const auto item = GEOSGetGeometryN_r(ctx, geom, i);
			SerializeRecursive(ctx, item, writer);
		}
	} break;
	default:
		throw InvalidInputException("Unsupported GEOS geometry type %d", type);
	}
}

void GeosSerde::Serialize(GEOSContextHandle_t ctx, const GEOSGeom_t *geom, char *buffer, size_t buffer_size) {
	BinaryWriter writer(buffer, buffer_size);
	// Serialize the geometry
	SerializeRecursive(ctx, geom, writer);
}

//----------------------------------------------------------------------------------------------------------------------
// Deserialize
//----------------------------------------------------------------------------------------------------------------------
template <class T>
static bool IsPointerAligned(const void *ptr) {
	auto uintptr = reinterpret_cast<uintptr_t>(ptr);
	return (uintptr % alignof(T)) == 0;
}

static GEOSGeometry *DeserializeRecursive(GEOSContextHandle_t ctx, BinaryReader &reader,
                                          vector<double> &aligned_buffer) {
	const auto le = reader.Read<uint8_t>();
	if (le != 1) {
		throw InvalidInputException("Unsupported byte order %d in WKB", le);
	}

	const auto meta = reader.Read<uint32_t>();
	const auto type = static_cast<GeometryType>(meta % 1000);
	const auto has_z = ((meta / 1000) & 0x01) != 0;
	const auto has_m = ((meta / 1000) & 0x02) != 0;

	const auto vertex_width = sizeof(double) * (2 + has_z + has_m);

	switch (type) {
	case GeometryType::POINT: {
		if (has_z && has_m) {
			constexpr auto vert_width = sizeof(double) * 4;
			const auto vert_array = reader.Reserve(vert_width);
			double vert[4] = {};
			memcpy(vert, vert_array, sizeof(double) * 4);
			for (const auto &v : vert) {
				if (!std::isnan(v)) {
					const auto seq = GEOSCoordSeq_copyFromBuffer_r(ctx, vert, 1, has_z, has_m);
					return GEOSGeom_createPoint_r(ctx, seq);
				}
			}
			return GEOSGeom_createEmptyPoint_r(ctx);
		} else if (has_z || has_m) {
			constexpr auto vert_width = sizeof(double) * 3;
			const auto vert_array = reader.Reserve(vert_width);
			double vert[3] = {};
			memcpy(vert, vert_array, sizeof(double) * 3);
			for (const auto &v : vert) {
				if (!std::isnan(v)) {
					const auto seq = GEOSCoordSeq_copyFromBuffer_r(ctx, vert, 1, has_z, has_m);
					return GEOSGeom_createPoint_r(ctx, seq);
				}
			}
			return GEOSGeom_createEmptyPoint_r(ctx);
		} else {
			constexpr auto vert_width = sizeof(double) * 2;
			const auto vert_array = reader.Reserve(vert_width);
			double vert[2] = {};
			memcpy(vert, vert_array, sizeof(double) * 2);
			for (const auto &v : vert) {
				if (!std::isnan(v)) {
					const auto seq = GEOSCoordSeq_copyFromBuffer_r(ctx, vert, 1, has_z, has_m);
					return GEOSGeom_createPoint_r(ctx, seq);
				}
			}
			return GEOSGeom_createEmptyPoint_r(ctx);
		}
	}
	case GeometryType::LINESTRING: {
		const auto vert_count = reader.Read<uint32_t>();
		auto vert_array = reinterpret_cast<const double *>(reader.Reserve(vert_count * vertex_width));
		if (vert_count == 0) {
			return GEOSGeom_createEmptyLineString_r(ctx);
		}

		if (!IsPointerAligned<double>(vert_array)) {
			aligned_buffer.resize(vert_count * (2 + has_z + has_m));
			memcpy(aligned_buffer.data(), vert_array, vert_count * vertex_width);
			vert_array = aligned_buffer.data();
		}

		const auto seq = GEOSCoordSeq_copyFromBuffer_r(ctx, vert_array, vert_count, has_z, has_m);
		return GEOSGeom_createLineString_r(ctx, seq);
	}
	case GeometryType::POLYGON: {
		const auto ring_count = reader.Read<uint32_t>();
		if (ring_count == 0) {
			return GEOSGeom_createEmptyPolygon_r(ctx);
		}
		const auto rings = new GEOSGeometry *[ring_count];
		for (uint32_t i = 0; i < ring_count; i++) {
			const auto vert_count = reader.Read<uint32_t>();
			auto vert_array = reinterpret_cast<const double *>(reader.Reserve(vert_count * vertex_width));
			if (vert_count == 0) {
				rings[i] = GEOSGeom_createEmptyLineString_r(ctx);
				continue;
			}

			if (!IsPointerAligned<double>(vert_array)) {
				aligned_buffer.resize(vert_count * (2 + has_z + has_m));
				memcpy(aligned_buffer.data(), vert_array, vert_count * vertex_width);
				vert_array = aligned_buffer.data();
			}

			const auto seq = GEOSCoordSeq_copyFromBuffer_r(ctx, vert_array, vert_count, has_z, has_m);
			rings[i] = GEOSGeom_createLinearRing_r(ctx, seq);
		}
		const auto result = GEOSGeom_createPolygon_r(ctx, rings[0], rings + 1, ring_count - 1);
		delete[] rings;
		return result;
	}
	case GeometryType::MULTIPOINT: {
		const auto part_count = reader.Read<uint32_t>();
		if (part_count == 0) {
			return GEOSGeom_createEmptyCollection_r(ctx, GEOS_MULTIPOINT);
		}
		const auto geoms = new GEOSGeometry *[part_count];
		for (uint32_t i = 0; i < part_count; i++) {
			geoms[i] = DeserializeRecursive(ctx, reader, aligned_buffer);
		}
		const auto result = GEOSGeom_createCollection_r(ctx, GEOS_MULTIPOINT, geoms, part_count);
		delete[] geoms;
		return result;
	}
	case GeometryType::MULTILINESTRING: {
		const auto part_count = reader.Read<uint32_t>();
		if (part_count == 0) {
			return GEOSGeom_createEmptyCollection_r(ctx, GEOS_MULTILINESTRING);
		}
		const auto geoms = new GEOSGeometry *[part_count];
		for (uint32_t i = 0; i < part_count; i++) {
			geoms[i] = DeserializeRecursive(ctx, reader, aligned_buffer);
		}
		const auto result = GEOSGeom_createCollection_r(ctx, GEOS_MULTILINESTRING, geoms, part_count);
		delete[] geoms;
		return result;
	}
	case GeometryType::MULTIPOLYGON: {
		const auto part_count = reader.Read<uint32_t>();
		if (part_count == 0) {
			return GEOSGeom_createEmptyCollection_r(ctx, GEOS_MULTIPOLYGON);
		}
		const auto geoms = new GEOSGeometry *[part_count];
		for (uint32_t i = 0; i < part_count; i++) {
			geoms[i] = DeserializeRecursive(ctx, reader, aligned_buffer);
		}
		const auto result = GEOSGeom_createCollection_r(ctx, GEOS_MULTIPOLYGON, geoms, part_count);
		delete[] geoms;
		return result;
	}
	case GeometryType::GEOMETRYCOLLECTION: {
		const auto part_count = reader.Read<uint32_t>();
		if (part_count == 0) {
			return GEOSGeom_createEmptyCollection_r(ctx, GEOS_GEOMETRYCOLLECTION);
		}
		const auto geoms = new GEOSGeometry *[part_count];
		for (uint32_t i = 0; i < part_count; i++) {
			geoms[i] = DeserializeRecursive(ctx, reader, aligned_buffer);
		}
		const auto result = GEOSGeom_createCollection_r(ctx, GEOS_GEOMETRYCOLLECTION, geoms, part_count);
		delete[] geoms;
		return result;
	}
	default:
		throw InvalidInputException("GEOS: Unsupported geometry type %d", static_cast<int>(type));
	}
}

GEOSGeom_t *GeosSerde::Deserialize(GEOSContextHandle_t ctx, const char *buffer, size_t buffer_size) {
	BinaryReader reader(buffer, buffer_size);
	vector<double> aligned_buffer;
	return DeserializeRecursive(ctx, reader, aligned_buffer);
}

} // namespace duckdb
