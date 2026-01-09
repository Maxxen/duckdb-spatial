#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace sgl2 {

//----------------------------------------------------------------------------------------------------------------------
// Definitions
//----------------------------------------------------------------------------------------------------------------------

// TODO: Enable assertions
//#define SGL2_ASSERT(x) do { if (!(x)) throw std::runtime_error("Assertion failed: " #x); } while(0)
#define SGL2_ASSERT(x)

enum class GeometryType : uint8_t {
	INVALID = 0,
	POINT = 1,
	LINESTRING = 2,
	POLYGON = 3,
	MULTI_POINT = 4,
	MULTI_LINESTRING = 5,
	MULTI_POLYGON = 6,
	GEOMETRY_COLLECTION = 7,
};

struct VertexXY {
	double x;
	double y;
};

struct ExtentXY {
	double xmin;
	double ymin;
	double xmax;
	double ymax;
};

enum class ContainsResult : uint8_t {
	INVALID = 0,
	INTERIOR,
	EXTERIOR,
	BOUNDARY,
};

enum class RayCastResult : uint8_t {
	NONE = 0,
	CROSS = 1,
	BOUNDARY = 2,
};

struct GeometryIndex {
	static constexpr uint32_t NODE_SIZE = 32;
	static constexpr uint32_t MAX_DEPTH = 8; // 16^8 >= max(uint32_t)

	// Layout is:
	// items_count
	// level_count;
	//    level1 count
	//    level1 offset
	//    level2 count
	//    levelN count
	//    levelN offset
	// levels...
	//    box1
	//    ...
	//    boxN

	uint32_t items_count;
	uint32_t level_count;

	char* level_data;
	char* entry_data;
	char* vertex_array;

	struct Layer {
	public:
		Layer(char* data_ptr, uint32_t count) : data(data_ptr), count(count) {}

		uint32_t GetEntryCount() const { return count; }

		ExtentXY GetEntry(const uint32_t index) const {
			ExtentXY result;
			memcpy(&result, data + index * sizeof(ExtentXY), sizeof(ExtentXY));
			return result;
		}
	private:
		uint32_t count;
		char* data;
	};

	Layer GetLayer(const uint32_t index) const {
		SGL2_ASSERT(index < level_count);

		const auto layer_ptr = level_data + index * sizeof(uint32_t) * 2;

		uint32_t layer_entry_count = 0;
		memcpy(&layer_entry_count, layer_ptr, sizeof(uint32_t));

		uint32_t layer_entry_offset = 0;
		memcpy(&layer_entry_offset, layer_ptr + sizeof(uint32_t), sizeof(uint32_t));

		return Layer(entry_data + layer_entry_offset * sizeof(ExtentXY), layer_entry_count);
	}

	uint32_t GetLayerCount() const {
		return level_count;
	}

	uint32_t GetVertexCount() const {
		return items_count;
	}

	// Returns true if the vertex is contained in the geometry index
	ContainsResult Contains(const VertexXY& vertex) const;
};

struct GeometryPart {
public:

	GeometryPart() : type(GeometryType::INVALID), flag(0), padd(0), size(0), data(nullptr) {}
	GeometryPart(GeometryType type, void* data, void* count);

	GeometryType	GetType() const { return type; }
	void*			GetData() const { return data; }
	uint32_t		GetCount() const { return size; }

	VertexXY		GetVertexXY(const uint32_t index) const {
		VertexXY result;
		memcpy(&result, static_cast<char*>(data) + index * sizeof(VertexXY), sizeof(VertexXY));
		return result;
	}


	GeometryPart	GetPart(const uint32_t index) const {
		GeometryPart result;

		const auto target_ptr = static_cast<char*>(data) + index * sizeof(GeometryPart);
		memcpy(&result, target_ptr, sizeof(GeometryPart));

		if (result.padd == 1) {
			// Is serialized, need to fix data pointer
			result.data = target_ptr + sizeof(GeometryPart) + reinterpret_cast<uint64_t>(result.data);
			result.padd = 0;
		}

		return result;
	}

	// TODO: Dont put index in flag
	bool HasIndex() const { return flag & 0x1; }

	GeometryIndex GetIndex() const {
		const auto index_ptr = static_cast<char*>(data) + size * sizeof(VertexXY);

		GeometryIndex result;
		memcpy(&result.items_count, index_ptr, sizeof(uint32_t));
		memcpy(&result.level_count, index_ptr + sizeof(uint32_t), sizeof(uint32_t));

		result.level_data = index_ptr + sizeof(uint32_t) * 2;
		result.entry_data = result.level_data + (result.level_count * sizeof(uint32_t) * 2);
		result.vertex_array = static_cast<char*>(data);

		return result;
	}

	GeometryType	type;
	uint8_t			flag;
	uint16_t		padd;
	uint32_t		size;
	void*			data;
};

static_assert(sizeof(GeometryPart) == 16, "GeometryPart must be 16 bytes");

struct MathRelation {

	// Returns the orientation of the triplet (p, q, r)
	// 0 if collinear, >0 if clockwise, <0 if counter-clockwise
	static int32_t Orient2D(const VertexXY& p, const VertexXY& q, const VertexXY& r);

	// Perform a ray-cast test between the segment pq and point r
	static RayCastResult RayCast2D(const VertexXY& p, const VertexXY& q, const VertexXY& r);

	// Branchless version of the ray-cast test, only returns if interior, so
	// need to handle boundary/exterios case with more precise test
	static int RayCast2DFast(const VertexXY &p, const VertexXY &q, const VertexXY &r);

	// Returns whether the point is in the ring
	static ContainsResult PointInRing(const VertexXY& point, const GeometryPart& ring);
};

//----------------------------------------------------------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------------------------------------------------------

inline int32_t MathRelation::Orient2D(const VertexXY &p, const VertexXY &q, const VertexXY &r) {
	const auto det_l = (p.x - r.x) * (q.y - r.y);
	const auto det_r = (p.y - r.y) * (q.x - r.x);
	const auto det = det_l - det_r;
	return (det > 0) - (det < 0);
}

inline RayCastResult MathRelation::RayCast2D(const VertexXY &prev, const VertexXY &next, const VertexXY &vert) {

	if (prev.x < vert.x && next.x < vert.x) {
		// The segment is to the left of the point
		return RayCastResult::NONE;
	}

	if (next.x == vert.x && next.y == vert.y) {
		// The point is on the segment, they share a vertex
		return RayCastResult::BOUNDARY;
	}

	if (prev.y == vert.y && next.y == vert.y) {
		// The segment is horizontal, check if the point is within the min/max x
		double minx = prev.x;
		double maxx = next.x;

		if (minx > maxx) {
			minx = next.x;
			maxx = prev.x;
		}

		if (vert.x >= minx && vert.x <= maxx) {
			// if its inside, then its on the boundary
			return RayCastResult::BOUNDARY;
		}

		// otherwise it has no impact on the result
		return RayCastResult::NONE;
	}

	if ((prev.y > vert.y && next.y <= vert.y) || (next.y > vert.y && prev.y <= vert.y)) {
		int sign = Orient2D(prev, next, vert);
		if (sign == 0) {
			return RayCastResult::BOUNDARY;
		}

		if (next.y < prev.y) {
			sign = -sign;
		}

		if (sign > 0) {
			return RayCastResult::CROSS;
		}
	}

	return RayCastResult::NONE;
}

inline ContainsResult MathRelation::PointInRing(const VertexXY &point, const GeometryPart &ring) {

	SGL2_ASSERT(ring.GetType() == GeometryType::LINESTRING);

	const auto vertex_count = ring.GetCount();

	if (vertex_count < 3) {
		// Not a valid ring
		return ContainsResult::INVALID;
	}

	if (ring.HasIndex()) {
		const auto index = ring.GetIndex();
		return index.Contains(point);
	}

	uint32_t crossings = 0;

	auto prev = ring.GetVertexXY(0);

	for (uint32_t i = 1; i < vertex_count; i++) {
		auto next = ring.GetVertexXY(i);

		const auto result = MathRelation::RayCast2D(prev, next, point);
		if (result == RayCastResult::BOUNDARY) {
			return ContainsResult::BOUNDARY;
		}
		crossings += static_cast<uint32_t>(result);

		prev = next;
	}

	// Even number of crossings means exterior, odd means interior
	return crossings % 2 == 0 ? ContainsResult::EXTERIOR : ContainsResult::INTERIOR;
}

struct GeometryRelation {
public:
	static bool Contains(const GeometryPart &g1, const GeometryPart &g2);
	static bool Within(const GeometryPart &g1, const GeometryPart &g2);

private:
	static bool PolygonContainsPoint(const GeometryPart &polygon, const GeometryPart &point);
	static bool PolygonContainsLineString(const GeometryPart &polygon, const GeometryPart &linestring);

	// Helper: Check if a single vertex is contained in a polygon (interior or boundary)
	static ContainsResult PolygonContainsVertex(const GeometryPart &polygon, const VertexXY &vertex);
};

inline bool GeometryRelation::Within(const GeometryPart &g1, const GeometryPart &g2) {
	// g1 is within g2 if g2 contains g1
	return Contains(g2, g1);
}

inline bool GeometryRelation::Contains(const GeometryPart &g1, const GeometryPart &g2) {

	switch (g1.GetType()) {
		case GeometryType::POINT: {
			// TODO: Implement
			return false;
		} break;
		case GeometryType::LINESTRING: {
			// TODO: Implement
			return false;
		} break;
		case GeometryType::POLYGON: {

			switch (g2.GetType()) {
				case GeometryType::POINT: {
					return PolygonContainsPoint(g1, g2);
				}
				case GeometryType::LINESTRING: {
					return PolygonContainsLineString(g1, g2);
				}
				default: {
					SGL2_ASSERT(false);
					return false;
				}
			}
		} break;
		case GeometryType::MULTI_POINT:
		case GeometryType::MULTI_LINESTRING:
		case GeometryType::MULTI_POLYGON:
		case GeometryType::GEOMETRY_COLLECTION: {
			// Loop over all parts and check containment
			const uint32_t part_count = g1.GetCount();
			for (uint32_t i = 0; i < part_count; i++) {
				const GeometryPart part = g1.GetPart(i);
				if (Contains(part, g2)) {
					return true;
				}
			}
			return false;
		}
		case GeometryType::INVALID:
		default: {
			SGL2_ASSERT(false);
			return false;
		}
	}
}


inline bool GeometryRelation::PolygonContainsPoint(const GeometryPart &polygon, const GeometryPart &point) {

	SGL2_ASSERT(polygon.GetType() == GeometryType::POLYGON);
	SGL2_ASSERT(point.GetType() == GeometryType::POINT);

	const VertexXY vertex = point.GetVertexXY(0);
	return PolygonContainsVertex(polygon, vertex) == ContainsResult::INTERIOR;
}

inline ContainsResult GeometryRelation::PolygonContainsVertex(const GeometryPart &polygon, const VertexXY &vertex) {

	SGL2_ASSERT(polygon.GetType() == GeometryType::POLYGON);

	// A polygon must have at least one ring (the exterior/shell)
	if (polygon.GetCount() == 0) {
		return ContainsResult::EXTERIOR;
	}

	// Check the exterior ring (shell)
	const GeometryPart shell = polygon.GetPart(0);
	const ContainsResult shell_result = MathRelation::PointInRing(vertex, shell);

	if (shell_result == ContainsResult::EXTERIOR) {
		return ContainsResult::EXTERIOR;
	}

	if (shell_result == ContainsResult::BOUNDARY) {
		return ContainsResult::BOUNDARY;
	}

	// Point is in the interior of the shell, now check the holes
	const uint32_t ring_count = polygon.GetCount();

	for (uint32_t i = 1; i < ring_count; i++) {
		const GeometryPart hole = polygon.GetPart(i);
		const ContainsResult hole_result = MathRelation::PointInRing(vertex, hole);

		if (hole_result == ContainsResult::INTERIOR) {
			// Point is inside a hole
			return ContainsResult::EXTERIOR;
		}

		if (hole_result == ContainsResult::BOUNDARY) {
			// Point is on the boundary of a hole
			return ContainsResult::BOUNDARY;
		}
	}

	return ContainsResult::INTERIOR;
}

inline bool GeometryRelation::PolygonContainsLineString(const GeometryPart &polygon, const GeometryPart &linestring) {

	SGL2_ASSERT(polygon.GetType() == GeometryType::POLYGON);
	SGL2_ASSERT(linestring.GetType() == GeometryType::LINESTRING);

	const uint32_t vertex_count = linestring.GetCount();

	// Empty linestring is not contained
	if (vertex_count == 0) {
		return false;
	}

	// Track if we have at least one interior point
	bool has_interior_point = false;

	// Check all vertices of the linestring
	for (uint32_t i = 0; i < vertex_count; i++) {
		const VertexXY vertex = linestring.GetVertexXY(i);
		const ContainsResult result = PolygonContainsVertex(polygon, vertex);

		if (result == ContainsResult::EXTERIOR) {
			// Any vertex outside means not contained
			return false;
		}

		if (result == ContainsResult::INTERIOR) {
			has_interior_point = true;
		}
	}

	// For proper containment, we need at least one interior point
	// If all points are on the boundary, the linestring is not properly contained
	if (!has_interior_point) {
		return false;
	}

	// Now check that no segment of the linestring passes through the exterior
	// This can happen if a segment crosses a hole or exits and re-enters the polygon
	// We check the midpoint of each segment - if both endpoints are inside/boundary
	// and the midpoint is exterior, the segment crosses the exterior
	for (uint32_t i = 0; i < vertex_count - 1; i++) {
		const VertexXY v1 = linestring.GetVertexXY(i);
		const VertexXY v2 = linestring.GetVertexXY(i + 1);

		// Compute midpoint
		const VertexXY midpoint = {(v1.x + v2.x) / 2.0, (v1.y + v2.y) / 2.0};
		const ContainsResult mid_result = PolygonContainsVertex(polygon, midpoint);

		if (mid_result == ContainsResult::EXTERIOR) {
			// Segment passes through exterior (e.g., through a hole)
			return false;
		}
	}

	return true;
}


inline ContainsResult GeometryIndex::Contains(const VertexXY &vert) const {

	uint32_t stack[MAX_DEPTH] = {0};
	uint32_t depth = 0;

	uint32_t crossings = 0;

	// Early-out if we are not contained in the bounding box of the root
	const auto root_box = GetLayer(0).GetEntry(0);
	if (vert.x < root_box.xmin || vert.x > root_box.xmax || vert.y < root_box.ymin || vert.y > root_box.ymax) {
		return ContainsResult::EXTERIOR;
	}

	// Traverse the tree
	while (true) {
		const auto level = GetLayer(depth);
		const auto entry = stack[depth];

		const auto box = level.GetEntry(entry);

		// Check if the vertex is in the box y-slice
		SGL2_ASSERT(box.ymin <= box.ymax);
		if (box.ymin <= vert.y && box.ymax >= vert.y) {
			if (depth != GetLayerCount() - 1) {
				// We are not at a leaf, so go downwards
				depth++;
				stack[depth] = entry * NODE_SIZE;
				continue;
			}

			// Now, we are at a leaf, so we need to check the segments
			const auto beg_idx = entry * NODE_SIZE;
			// We +1 to the node size here to get the end index, because the last segment spans across
			// the end of the node. And our indexes are always sequential.
			const auto end_idx = std::min(beg_idx + NODE_SIZE + 1, GetVertexCount());

			// Loop over the segments
			constexpr auto vertex_width = sizeof(VertexXY);

			VertexXY prev;
			memcpy(&prev, vertex_array + beg_idx * vertex_width, sizeof(VertexXY));

			for (uint32_t i = beg_idx + 1; i < end_idx; i++) {
				VertexXY next;
				memcpy(&next, vertex_array + i * vertex_width, sizeof(VertexXY));

				switch (MathRelation::RayCast2D(prev, next, vert)) {
				case RayCastResult::NONE:
					// No intersection
					break;
				case RayCastResult::CROSS:
					// The ray crosses the segment, so we count it
					crossings++;
					break;
				case RayCastResult::BOUNDARY:
					// The point is on the boundary, so we return BOUNDARY
					return ContainsResult::BOUNDARY;
				}
				prev = next;
			}
		}

		while (true) {

			if (depth == 0) {
				// Even number of crossings means the point is outside the polygon
				return crossings % 2 == 0 ? ContainsResult::EXTERIOR : ContainsResult::INTERIOR;
			}

			// The end of this node is either the end of the current node, or the end of the level
			const auto node_end = ((stack[depth - 1] + 1) * NODE_SIZE) - 1;
			const auto levl_end = GetLayer(depth).GetEntryCount() - 1;

			const auto end = std::min(node_end, levl_end);

			if (stack[depth] != end) {
				// Go sideways!
				stack[depth]++;
				break;
			}

			// Go upwards!
			depth--;
		}
	}
}


} // namespace sgl