#ifndef CONCAVE_POLY_H
#define CONCAVE_POLY_H
#include <utility>
#include <vector>
#include <cmath>
#include <map>
#include <iostream>
#include <limits>
#include <algorithm>
namespace cxd {
struct Vec2 {
    float x;
    float y;

    Vec2() = default;

    Vec2(const float xIn, const float yIn) : x(xIn), y(yIn) {}

    static float Length(Vec2 const &v) { return sqrtf(v.x * v.x + v.y * v.y); }

    static Vec2 Norm(Vec2 const &v) {
        if (Length(v) < 1e-30) {
            return {0.0f, 0.0f};
        } else {
            return (v / Length(v));
        }
    }

    static float Dot(Vec2 const &v1, Vec2 const &v2) {
        return (v1.x * v2.x + v1.y * v2.y);
    }

    static float Square(Vec2 const &v) { return Dot(v, v); }

    static float Cross(Vec2 const &v1, Vec2 const &v2) {
        return (v1.x * v2.y - v1.y * v2.x);
    }

    Vec2 operator-(Vec2 const &v1) const { return {x - v1.x, y - v1.y}; }

    Vec2 operator+(Vec2 const &v1) const { return {x + v1.x, y + v1.y}; }

    Vec2 operator*(float const &f) const { return {f * x, f * y}; }

    Vec2 operator/(float const &f) const { return {x / f, y / f}; }

    static float GetSignedArea(Vec2 const &v1, Vec2 const &v2) {
        return (v2.x - v1.x) * (v2.y + v1.y);
    }
};

struct Vertex {
    Vec2 position;

    Vertex() = default;

    Vertex(Vec2 const &_position) : position{_position} {}

    static float GetHandedness(Vertex const &v1, Vertex const &v2,
                               Vertex const &v3) {
        Vec2 edge1 = v2.position - v1.position;
        Vec2 edge2 = v3.position - v2.position;
        return Vec2::Cross(edge1, edge2);
    }
};

struct SliceVertex : public Vertex {
    SliceVertex() = default;

    explicit SliceVertex(Vec2 const &_position) : Vertex{_position} {}

    int index{};
    float distanceToSlice{};
};

struct LineSegment {
    Vec2 startPos{};
    Vec2 finalPos{};

    LineSegment() = default;

    LineSegment(Vec2 const &_startPos, Vec2 const &_finalPos)
            : startPos{_startPos}, finalPos{_finalPos} {}

    Vec2 Direction() const { return finalPos - startPos; }

    Vec2 NormalisedDirection() { return Vec2::Norm(finalPos - startPos); }

    LineSegment operator+(LineSegment const &ls) const {
        Vec2 newStartPos = (startPos + ls.startPos) / 2.0f;
        Vec2 newFinalPos = (finalPos + ls.finalPos) / 2.0f;

        return {newStartPos, newFinalPos};
    }

    static std::pair<bool, Vec2> Intersects(LineSegment s1, LineSegment s2) {
        const float TOLERANCE = 1e-5;
        Vec2 p1 = s1.startPos;
        Vec2 p2 = s2.startPos;
        Vec2 d1 = s1.Direction();
        Vec2 d2 = s2.Direction();

        if (((std::abs(p1.x - p2.x) < 1e-10) && (std::abs(p1.y - p2.y) < 1e-10)) ||
            ((std::abs(s1.finalPos.x - p2.x) < 1e-10) &&
             (std::abs(s1.finalPos.y - p2.y) < 1e-10))) {
            return {true, p2};
        }

        if (std::abs(Vec2::Cross(d1, d2)) < 1e-30) {
            return {false, {0.0f, 0.0f}};
        }

        float t1 = Vec2::Cross(p2 - p1, d2) / Vec2::Cross(d1, d2);

        if ((t1 < (0.0f - TOLERANCE)) || (t1 > (1.0f + TOLERANCE))) {
            return {false, {0.0f, 0.0f}};
        }

        Vec2 pIntersect = p1 + d1 * t1;

        float t2 = Vec2::Dot(pIntersect - p2, s2.finalPos - p2);

        if (t2 < (0.0f - TOLERANCE) ||
            (t2 / Vec2::Square(s2.finalPos - p2) >= 1.0f - TOLERANCE)) {
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

    explicit ConcavePolygon(VertexArray _vertices)
            : vertices{std::move(_vertices)} {
        if (vertices.size() > 2U && (!CheckIfRightHanded())) {
            FlipPolygon();
        }
    }

    inline bool CheckIfRightHanded() { return CheckIfRightHanded(vertices); }

    inline void ConvexDecomp() {
        ConvexDecomp(vertices);
    }

    inline VertexArray const &GetVertices() const {
        return vertices;
    }

    inline VertexArray &GetLeftVertices() {
        return left_vertices;
    }

    inline VertexArray &GetRightVertices() {
        return right_vertices;
    }

    inline void Reset() {
        if (!left_vertices.empty()) {
            left_vertices.clear();
        }
        if (!right_vertices.empty()) {
            right_vertices.clear();
        }
    }

    inline Vec2 GetPoint(size_t index) const {
        if (index < vertices.size()) {
            return vertices[index].position;
        }
        return {0.0f, 0.0f};
    }

    inline size_t GetPointCount() const { return vertices.size(); }

    void ConvexDecomp(VertexArray const &polygonVertices);

private:
    VertexArray vertices;
    VertexArray left_vertices;
    VertexArray right_vertices;

    static int Mod(const int x, const int m) {
        int r = x % m;
        return ((r < 0) ? (r + m) : r);
    }

    static void FlipPolygon(VertexArray &_verts) {
        int iMax = static_cast<int>(_verts.size()) / 2;

        if (_verts.size() % 2 != 0) {
            iMax += 1;
        }
        for (int i = 1; i < iMax; ++i) {
            std::swap(_verts[i], _verts[_verts.size() - i]);
        }
    }

    static bool CheckVisibility(Vec2 const &originalPosition, Vertex const &vert,
                                VertexArray const &polygonVertices) {
        LineSegment ls(originalPosition, vert.position);
        VertexIntMap intersectingVerts =
                VerticesAlongLineSegment(ls, polygonVertices);
        if (intersectingVerts.size() > 3U) {
            return false;
        }
        return true;
    }

    inline void FlipPolygon() { FlipPolygon(vertices); }

    static bool CheckIfRightHanded(VertexArray &vertexArray);

    static bool IsVertexInCone(LineSegment const &ls1, LineSegment const &ls2,
                               Vec2 const &origin, Vertex const &vert);

    static IntArray FindVerticesInCone(LineSegment const &ls1,
                                       LineSegment const &ls2, Vec2 const &origin,
                                       VertexArray const &inputVerts);

    static int GetBestVertexToConnect(IntArray const &indices,
                                      VertexArray const &polygonVertices,
                                      Vec2 const &origin);

    static int FindFirstReflexVertex(VertexArray const &vertices);

    static VertexIntMap CullByDistance(VertexIntMap const &input,
                                       Vec2 const &origin,
                                       int const &maxVertsToKeep);

    static VertexIntMap VerticesAlongLineSegment(LineSegment const &segment,
                                                 VertexArray const &_vertices);

    void SlicePolygon(int vertex1, int vertex2);

    void SlicePolygon(LineSegment segment);
};
}
#endif // CONCAVE_POLY_H