#pragma once

#include "duckdb/common/string.hpp"
#include "duckdb/common/vector.hpp"

namespace duckdb {

class ExtensionLoader;
struct LogicalType;

struct GeoTypes {
	static LogicalType POINT_2D();
	static LogicalType POINT_3D();
	static LogicalType POINT_4D();
	static LogicalType LINESTRING_2D();
	static LogicalType LINESTRING_3D();
	static LogicalType POLYGON_2D();
	static LogicalType POLYGON_3D();
	static LogicalType BOX_2D();
	static LogicalType BOX_2DF();
	static LogicalType WKB_BLOB();

	// This used to be the old geometry type defined by the spatial extension, before it was added to core duckdb
	static LogicalType EXTENSION_GEOMETRY();

	static void Register(ExtensionLoader &loader);

	static LogicalType CreateEnumType(const string &name, const vector<string> &members);
};

} // namespace duckdb
