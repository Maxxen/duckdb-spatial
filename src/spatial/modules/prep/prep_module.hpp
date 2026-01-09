#pragma once

namespace duckdb {

class ExtensionLoader;

void RegisterPrepModule(ExtensionLoader &db);

} // namespace duckdb
