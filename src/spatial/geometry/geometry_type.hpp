#pragma once

#include "duckdb/common/string_util.hpp"
#include "duckdb/common/types/string_type.hpp"
#include "spatial/geometry/bbox.hpp"
#include "spatial/geometry/geometry_properties.hpp"
#include "spatial/util/cursor.hpp"

#include "duckdb/common/type_util.hpp"
#include "spatial/util/binary_reader.hpp"
#include "spatial/util/math.hpp"

namespace duckdb {

enum class SpatialGeometryType : uint8_t {
	POINT = 0,
	LINESTRING,
	POLYGON,
	MULTIPOINT,
	MULTILINESTRING,
	MULTIPOLYGON,
	GEOMETRYCOLLECTION
};

struct GeometryTypes {
	static bool IsSinglePart(SpatialGeometryType type) {
		return type == SpatialGeometryType::POINT || type == SpatialGeometryType::LINESTRING;
	}

	static bool IsMultiPart(SpatialGeometryType type) {
		return type == SpatialGeometryType::POLYGON || type == SpatialGeometryType::MULTIPOINT ||
		       type == SpatialGeometryType::MULTILINESTRING || type == SpatialGeometryType::MULTIPOLYGON ||
		       type == SpatialGeometryType::GEOMETRYCOLLECTION;
	}

	static bool IsCollection(SpatialGeometryType type) {
		return type == SpatialGeometryType::MULTIPOINT || type == SpatialGeometryType::MULTILINESTRING ||
		       type == SpatialGeometryType::MULTIPOLYGON || type == SpatialGeometryType::GEOMETRYCOLLECTION;
	}

	static string ToString(SpatialGeometryType type) {
		switch (type) {
		case SpatialGeometryType::POINT:
			return "POINT";
		case SpatialGeometryType::LINESTRING:
			return "LINESTRING";
		case SpatialGeometryType::POLYGON:
			return "POLYGON";
		case SpatialGeometryType::MULTIPOINT:
			return "MULTIPOINT";
		case SpatialGeometryType::MULTILINESTRING:
			return "MULTILINESTRING";
		case SpatialGeometryType::MULTIPOLYGON:
			return "MULTIPOLYGON";
		case SpatialGeometryType::GEOMETRYCOLLECTION:
			return "GEOMETRYCOLLECTION";
		default:
			return StringUtil::Format("UNKNOWN(%d)", static_cast<int>(type));
		}
	}
};

enum class SerializedGeometryType : uint32_t {
	POINT = 0,
	LINESTRING,
	POLYGON,
	MULTIPOINT,
	MULTILINESTRING,
	MULTIPOLYGON,
	GEOMETRYCOLLECTION
};

// A serialized geometry
class geometry_t {
private:
	string_t data;

public:
	geometry_t() = default;
	// NOLINTNEXTLINE
	explicit geometry_t(string_t data) : data(data) {
	}

	// NOLINTNEXTLINE
	operator string_t() const {
		return data;
	}

	SpatialGeometryType GetType() const {
		// return the type
		const auto type = Load<SpatialGeometryType>(const_data_ptr_cast(data.GetPrefix()));
		const auto props = Load<GeometryProperties>(const_data_ptr_cast(data.GetPrefix() + 1));
		props.CheckVersion();
		return type;
	}

	GeometryProperties GetProperties() const {
		const auto props = Load<GeometryProperties>(const_data_ptr_cast(data.GetPrefix() + 1));
		// Check the version
		props.CheckVersion();
		return props;
	}

	bool TryGetCachedBounds(Box2D<float> &bbox) const {
		auto extent = GeometryExtent::Empty();
		if (Geometry::GetExtent(data, extent) != 0) {
			bbox.min.x = MathUtil::DoubleToFloatDown(extent.min_x);
			bbox.min.y = MathUtil::DoubleToFloatDown(extent.min_y);
			bbox.max.x = MathUtil::DoubleToFloatUp(extent.max_x);
			bbox.max.y = MathUtil::DoubleToFloatUp(extent.max_y);
			return true;
		}
		return false;
	}

	/*
	bool TryGetCachedBounds(Box2D<float> &bbox) const {
		Cursor cursor(data);

		// Read the header
		auto header_type = cursor.Read<SpatialGeometryType>();
		auto properties = cursor.Read<GeometryProperties>();
		auto hash = cursor.Read<uint16_t>();
		(void)hash;

		// Check the version
		properties.CheckVersion();

		if (properties.HasBBox()) {
			cursor.Skip(4); // skip padding

			// Now set the bounding box
			bbox.min.x = cursor.Read<float>();
			bbox.min.y = cursor.Read<float>();
			bbox.max.x = cursor.Read<float>();
			bbox.max.y = cursor.Read<float>();
			return true;
		}

		if (header_type == SpatialGeometryType::POINT) {
			cursor.Skip(4); // skip padding

			// Read the point
			auto type = cursor.Read<SerializedGeometryType>();
			D_ASSERT(type == SerializedGeometryType::POINT);
			(void)type;

			auto count = cursor.Read<uint32_t>();
			if (count == 0) {
				// If the point is empty, there is no bounding box
				return false;
			}

			const auto x = cursor.Read<double>();
			const auto y = cursor.Read<double>();
			bbox.min.x = MathUtil::DoubleToFloatDown(x);
			bbox.min.y = MathUtil::DoubleToFloatDown(y);
			bbox.max.x = MathUtil::DoubleToFloatUp(x);
			bbox.max.y = MathUtil::DoubleToFloatUp(y);
			return true;
		}
		return false;
	}
	*/
};

template <>
inline PhysicalType GetTypeId<geometry_t>() {
	return PhysicalType::VARCHAR;
}

static_assert(sizeof(geometry_t) == sizeof(string_t), "geometry_t should be the same size as string_t");

} // namespace duckdb
