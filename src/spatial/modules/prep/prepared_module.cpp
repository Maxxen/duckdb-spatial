#include "spatial/geometry/sgl.hpp"
#include "spatial/util/binary_reader.hpp"
#include "spatial/util/binary_writer.hpp"

#include "duckdb/main/extension/extension_loader.hpp"

namespace duckdb {

void RegisterPreparedModule(ExtensionLoader &loader) {



	// // Plan: Deserialize into prepared geometry, then serialize the index data into a separate blob,
	// // and return a struct. Then every ring has it own prepared geometry index in the separate blob.
	// // We basically serialize item_count + entry_count, and the for each level: entry_count + entry_array.
	//
	// ScalarFunction prep("prepare_geometry",
	// 	{LogicalType::GEOMETRY()}, PreparedType(), PrepareGeometry);
	//
	// //loader.RegisterFunction(prep);

}

} // namespace duckdb
