#pragma once

namespace duckdb {

class ExtensionLoader;

void RegisterPreparedModule(ExtensionLoader &loader);

} // namespace duckdb
