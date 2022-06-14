#ifndef CONCAVE_POLY_H
#define CONCAVE_POLY_H
#include <utility>
#include <vector>
#include <cmath>
#include <map>
#include <iostream>
#include <limits>
#include <algorithm>
namespace cxd
{
struct Vec2
{
	float x;
	float y;
	Vec2() = default;
	Vec2(const float xIn, const float yIn) :
		x(xIn),
		y(yIn) {}

	static float Length(Vec2 const & v) {
		return sqrtf(v.x * v.x + v.y * v.y);
	}

	static Vec2 Norm(Vec2 const & v) {
		if(Length(v) < 1e-30) {
			return {0.0f, 0.0f};
		} else {
			return (v / Length(v));
		}
	}

	static float Dot(Vec2 const & v1, Vec2 const & v2) {
		return (v1.x * v2.x + v1.y * v2.y);
	}

	static float Square(Vec2 const & v) {
		return Dot(v, v);
	}

	static float Cross(Vec2 const & v1, Vec2 const & v2) {
		return (v1.x * v2.y - v1.y * v2.x);
	}

	Vec2 operator - (Vec2 const & v1) const {
		return {x - v1.x, y - v1.y};
	}

	Vec2 operator + (Vec2 const & v1) const {
		return {x + v1.x, y + v1.y};
	}

	Vec2 operator * (float const & f) const {
		return {f * x, f * y};
	}

	Vec2 operator / (float const & f) const {
		return {x / f, y / f};
	}

	static float GetSignedArea(Vec2 const & v1, Vec2 const & v2) {
		return (v2.x - v1.x) * (v2.y + v1.y);
	}
};

struct Vertex
{
	Vec2 position;

	Vertex() = default;
	Vertex(Vec2 const & _position) :
		position{_position} {}

	static float GetHandedness(Vertex const & v1, Vertex const & v2, Vertex const & v3) {
		Vec2 edge1 = v2.position-v1.position;
		Vec2 edge2 = v3.position-v2.position;
		return Vec2::Cross(edge1, edge2);
	}
};

struct SliceVertex : public Vertex {
	SliceVertex() = default;
	explicit SliceVertex(Vec2 const & _position) : Vertex{_position} {}

	int index{};
	float distance_slice{};
};

struct LineSegment {
	Vec2 start_pos{};
	Vec2 final_pos{};

	LineSegment() = default;
	LineSegment(Vec2 const & start_pos_in, Vec2 const & final_pos_in) :
			start_pos{start_pos_in},
			final_pos{final_pos_in} {}

	Vec2 Direction() const {
		return final_pos - start_pos;
	}

	Vec2 NormalisedDirection() {
		return Vec2::Norm(final_pos - start_pos);
	}

	LineSegment operator + (LineSegment const & ls) const {
		Vec2 newStartPos = (start_pos + ls.start_pos) / 2.0f;
		Vec2 newFinalPos = (final_pos + ls.final_pos) / 2.0f;

		return {newStartPos, newFinalPos};
	}

	static std::pair<bool, Vec2> Intersects(LineSegment s1, LineSegment s2) {
		const float tolerance = 1e-5;
		Vec2 p1 = s1.start_pos;
		Vec2 p2 = s2.start_pos;
		Vec2 d1 = s1.Direction();
		Vec2 d2 = s2.Direction();

		if (((std::abs(p1.x - p2.x) < 1e-10) && (std::abs(p1.y - p2.y) < 1e-10)) ||
		    ((std::abs(s1.final_pos.x - p2.x) < 1e-10) && (std::abs(s1.final_pos.y - p2.y) < 1e-10))) {
			return {true, p2};
		}

		if(std::abs(Vec2::Cross(d1, d2)) < 1e-30) {
			return {false, {0.0f, 0.0f}};
		}

		float t1 = Vec2::Cross(p2 - p1, d2) / Vec2::Cross(d1, d2);

		if((t1 < (0.0f - tolerance)) || (t1 > (1.0f + tolerance))) {
			return {false, {0.0f, 0.0f}};
		}

		Vec2 pIntersect = p1 + d1 * t1;

		float t2 = Vec2::Dot(pIntersect - p2, s2.final_pos - p2);

		if(t2 < (0.0f - tolerance) || (t2 / Vec2::Square(s2.final_pos - p2) >= 1.0f - tolerance)) {
			return {false, {0.0f, 0.0f}};
		}

		return {true, pIntersect};
	}
};

class ConcavePolygon {
	using IntArray = std::vector<int>;
	using VertexArray = std::vector<Vertex>;
	using PolygonArray = std::vector<ConcavePolygon>;
	using VertexIntMap = std::map<int, Vertex>;
public:
	ConcavePolygon() = default;

	explicit ConcavePolygon(VertexArray  _vertices) :
			vertices{std::move(_vertices)} {
		if(vertices.size() > 2U && (!CheckIfRightHanded())) {
			FlipPolygon();
		}
	}

	inline bool CheckIfRightHanded() {
		return CheckIfRightHanded(vertices);
	}


	inline void ConvexDecomp() {
		if(vertices.size() > 3U) {
			ConvexDecomp(vertices);
		}
	}

	inline VertexArray const & GetVertices() const {
		return vertices;
	}

	inline ConcavePolygon const & GetSubPolygon(size_t subPolyIndex) const {
		if((!sub_polygons.empty()) && (subPolyIndex < sub_polygons.size())) {
			return sub_polygons[subPolyIndex];
		}
		return (*this);
	}

	inline size_t GetNumberSubPolys() const {
		return sub_polygons.size();
	}

	inline void ReturnLowestLevelPolys(std::vector<ConcavePolygon>& returnArr) {
		if(!sub_polygons.empty()) {
			sub_polygons[0].ReturnLowestLevelPolys(returnArr);
			sub_polygons[1].ReturnLowestLevelPolys(returnArr);
		} else {
			returnArr.emplace_back(*this);
		}
	}

	inline void Reset() {
		if(!sub_polygons.empty()) {
			sub_polygons[0].Reset();
			sub_polygons[1].Reset();
			sub_polygons.clear();
		}
	}

	inline Vec2 GetPoint(size_t index) const {
		if(index < vertices.size()) {
			return vertices[index].position;
		}
		return {0.0f, 0.0f};
	}

	inline size_t GetPointCount() const {
		return vertices.size();
	}

	void ConvexDecomp(VertexArray const & polygon_vertices);

private:
	VertexArray vertices;
	PolygonArray sub_polygons;

	static int Mod(const int x, const int m) {
		int r = x % m;
		return ((r < 0) ? (r + m) : r);
	}

	static void FlipPolygon(VertexArray & verts) {
		int iMax = static_cast<int>(verts.size()) / 2;

		if(verts.size() % 2 != 0) {
			iMax += 1;
		}
		for(int i = 1; i < iMax; ++i) {
			std::swap(verts[i], verts[verts.size() - i]);
		}
	}

	static bool CheckVisibility(Vec2 const & original_position, Vertex const & vert,
		VertexArray const & polygon_vertices) {
		LineSegment ls(original_position, vert.position);
		VertexIntMap intersecting_verts = VerticesAlongLineSegment(ls, polygon_vertices);
		if(intersecting_verts.size() > 3U) {
			return false;
		}
		return true;
	}

	inline void FlipPolygon() {
		FlipPolygon(vertices);
	}


	static bool CheckIfRightHanded(VertexArray & vertex_array);

	static bool IsVertexInCone(LineSegment const & ls1, LineSegment const & ls2,
		Vec2 const & origin, Vertex const & vert);

	static IntArray FindVerticesInCone(LineSegment const & ls1, LineSegment const & ls2,
		Vec2 const & origin, VertexArray const & input_verts);

	static int GetBestVertexToConnect(IntArray const & indices, VertexArray const & polygon_vertices,
		Vec2 const & origin);

	static int FindFirstReflexVertex(VertexArray const & vertices);

	static VertexIntMap CullByDistance(VertexIntMap const & input, Vec2 const & origin, int const & max_verts_keep);

	static VertexIntMap VerticesAlongLineSegment(LineSegment const & segment, VertexArray const & vertices);

	void SlicePolygon(LineSegment segment);
};
}

#endif // CONCAVE_POLY_H