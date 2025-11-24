#pragma once

#include "bbox.hpp"

#include <cstddef>
#include <cstdint>

namespace sgl {
class geometry;
class prepared_geometry;
} // namespace sgl

namespace duckdb {

class ArenaAllocator;

// todo:
struct Serde {
	static size_t GetRequiredSize(const sgl::geometry &geom);
	static void Serialize(const sgl::geometry &geom, char *buffer, size_t buffer_size);
	static size_t GetRequiredSizePrepared(const sgl::geometry &geom);
	static void SerializePrepared(const sgl::geometry &geom, char *buffer, size_t buffer_size);
	static void Deserialize(sgl::geometry &result, ArenaAllocator &arena, const char *buffer, size_t buffer_size);
	static void DeserializePrepared(sgl::prepared_geometry &result, ArenaAllocator &arena, const char *buffer,
	                                size_t buffer_size);

	static uint32_t TryGetBounds(const string_t &blob, Box2D<float> &bbox);
};

} // namespace duckdb
