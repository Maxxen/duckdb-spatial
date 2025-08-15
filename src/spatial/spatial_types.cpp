#include "spatial/spatial_types.hpp"
#include "duckdb/main/extension/extension_loader.hpp"

namespace duckdb {

LogicalType GeoTypes::POINT_2D() {
	auto type = LogicalType::STRUCT({{"x", LogicalType::DOUBLE}, {"y", LogicalType::DOUBLE}});
	type.SetAlias("POINT_2D");
	return type;
}

LogicalType GeoTypes::POINT_3D() {
	auto type =
	    LogicalType::STRUCT({{"x", LogicalType::DOUBLE}, {"y", LogicalType::DOUBLE}, {"z", LogicalType::DOUBLE}});
	type.SetAlias("POINT_3D");
	return type;
}

LogicalType GeoTypes::POINT_4D() {
	auto type = LogicalType::STRUCT({{"x", LogicalType::DOUBLE},
	                                 {"y", LogicalType::DOUBLE},
	                                 {"z", LogicalType::DOUBLE},
	                                 {"m", LogicalType::DOUBLE}});
	type.SetAlias("POINT_4D");
	return type;
}

LogicalType GeoTypes::BOX_2D() {
	auto type = LogicalType::STRUCT({{"min_x", LogicalType::DOUBLE},
	                                 {"min_y", LogicalType::DOUBLE},
	                                 {"max_x", LogicalType::DOUBLE},
	                                 {"max_y", LogicalType::DOUBLE}});
	type.SetAlias("BOX_2D");
	return type;
}

LogicalType GeoTypes::BOX_2DF() {
	auto type = LogicalType::STRUCT({{"min_x", LogicalType::FLOAT},
	                                 {"min_y", LogicalType::FLOAT},
	                                 {"max_x", LogicalType::FLOAT},
	                                 {"max_y", LogicalType::FLOAT}});
	type.SetAlias("BOX_2DF");
	return type;
}

LogicalType GeoTypes::LINESTRING_2D() {
	auto type = LogicalType::LIST(LogicalType::STRUCT({{"x", LogicalType::DOUBLE}, {"y", LogicalType::DOUBLE}}));
	type.SetAlias("LINESTRING_2D");
	return type;
}

LogicalType GeoTypes::LINESTRING_3D() {
	auto type = LogicalType::LIST(
	    LogicalType::STRUCT({{"x", LogicalType::DOUBLE}, {"y", LogicalType::DOUBLE}, {"z", LogicalType::DOUBLE}}));
	type.SetAlias("LINESTRING_3D");
	return type;
}

LogicalType GeoTypes::POLYGON_2D() {
	auto type = LogicalType::LIST(
	    LogicalType::LIST(LogicalType::STRUCT({{"x", LogicalType::DOUBLE}, {"y", LogicalType::DOUBLE}})));
	type.SetAlias("POLYGON_2D");
	return type;
}

LogicalType GeoTypes::POLYGON_3D() {
	auto type = LogicalType::LIST(LogicalType::LIST(
	    LogicalType::STRUCT({{"x", LogicalType::DOUBLE}, {"y", LogicalType::DOUBLE}, {"z", LogicalType::DOUBLE}})));
	type.SetAlias("POLYGON_3D");
	return type;
}

/*
LogicalType GeoTypes::GEOMETRY() {
	auto blob_type = LogicalType(LogicalTypeId::BLOB);
	blob_type.SetAlias("GEOMETRY");
	return blob_type;
}
*/

LogicalType GeoTypes::WKB_BLOB() {
	auto blob_type = LogicalType(LogicalTypeId::BLOB);
	blob_type.SetAlias("WKB_BLOB");
	return blob_type;
}

LogicalType GeoTypes::CreateEnumType(const string &name, const vector<string> &members) {
	auto varchar_vector = Vector(LogicalType::VARCHAR, members.size());
	auto varchar_data = FlatVector::GetData<string_t>(varchar_vector);
	for (idx_t i = 0; i < members.size(); i++) {
		auto str = string_t(members[i]);
		varchar_data[i] = str.IsInlined() ? str : StringVector::AddString(varchar_vector, str);
	}
	auto enum_type = LogicalType::ENUM(name, varchar_vector, members.size());
	enum_type.SetAlias(name);
	return enum_type;
}

void GeoTypes::Register(ExtensionLoader &loader) {

	// POINT_2D
	loader.RegisterType("POINT_2D", GeoTypes::POINT_2D());

	// POINT_3D
	loader.RegisterType("POINT_3D", GeoTypes::POINT_3D());

	// POINT_4D
	loader.RegisterType("POINT_4D", GeoTypes::POINT_4D());

	// LineString2D
	loader.RegisterType("LINESTRING_2D", GeoTypes::LINESTRING_2D());

	// LineString3D
	loader.RegisterType("LINESTRING_3D", GeoTypes::LINESTRING_3D());

	// Polygon2D
	loader.RegisterType("POLYGON_2D", GeoTypes::POLYGON_2D());

	// Polygon3D
	loader.RegisterType("POLYGON_3D", GeoTypes::POLYGON_3D());

	// Box2D
	loader.RegisterType("BOX_2D", GeoTypes::BOX_2D());

	// Box2DF
	loader.RegisterType("BOX_2DF", GeoTypes::BOX_2DF());

	// GEOMETRY
	//loader.RegisterType("GEOMETRY", GeoTypes::GEOMETRY());

	// WKB_BLOB
	loader.RegisterType("WKB_BLOB", GeoTypes::WKB_BLOB());
}

} // namespace duckdb
