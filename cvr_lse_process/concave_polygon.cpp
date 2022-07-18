//
// Created by ghm on 2021/12/29.
//

#include "concave_polygon.h"

namespace cxd {
bool ConcavePolygon::CheckIfRightHanded(VertexArray& vertexArray) {
    if (vertexArray.size() < 3U) {
        return false;
    }

    float signedArea = 0.0f;

    for (int i = 0; i < static_cast<int>(vertexArray.size()); ++i) {
        signedArea += Vec2::GetSignedArea(vertexArray[i].position,
            vertexArray[Mod(i + 1, static_cast<int>(vertexArray.size()))].position);
    }

    return (signedArea < 0.0f);
}

bool ConcavePolygon::IsVertexInCone(LineSegment const& ls1, LineSegment const& ls2, Vec2 const& origin,
    Vertex const& vert) {
    auto relativePos = vert.position - origin;
    float ls1Product = Vec2::Cross(relativePos, ls1.Direction());
    float ls2Product = Vec2::Cross(relativePos, ls2.Direction());
    return ((ls1Product < 0.0f) && (ls2Product > 0.0f));
}

ConcavePolygon::IntArray ConcavePolygon::FindVerticesInCone(LineSegment const& ls1, LineSegment const& ls2,
    Vec2 const& origin, VertexArray const& inputVerts) {
    IntArray result;
    for (size_t i = 0U; i < inputVerts.size(); ++i) {
        if (IsVertexInCone(ls1, ls2, origin, inputVerts[i])) {
            result.emplace_back(static_cast<int>(i));
        }
    }
    return result;
}

int ConcavePolygon::GetBestVertexToConnect(IntArray const& indices, VertexArray const& polygonVertices,
    Vec2 const& origin) {
    if (indices.size() == 1U) {
        if (CheckVisibility(origin, polygonVertices[indices[0]], polygonVertices)) {
            return indices[0];
        }
    } else if (indices.size() > 1U) {
        for (int index : indices) {
            int vertSize = static_cast<int>(polygonVertices.size());
            Vertex prevVert = polygonVertices[Mod(index - 1, vertSize)].position;
            Vertex currVert = polygonVertices[index].position;
            Vertex nextVert = polygonVertices[Mod(index + 1, vertSize)].position;

            LineSegment ls1(prevVert.position, currVert.position);
            LineSegment ls2(nextVert.position, currVert.position);

            if ((Vertex::GetHandedness(prevVert, currVert, nextVert) < 0.0f) &&
                IsVertexInCone(ls1, ls2, polygonVertices[index].position, Vertex(origin)) &&
                CheckVisibility(origin, polygonVertices[index], polygonVertices)) {
                return index;
            }
        }

        for (int index : indices) {
            int vertSize = static_cast<int>(polygonVertices.size());
            Vertex prevVert = polygonVertices[Mod(index - 1, vertSize)].position;
            Vertex currVert = polygonVertices[index].position;
            Vertex nextVert = polygonVertices[Mod(index + 1, vertSize)].position;

            LineSegment ls1(prevVert.position, currVert.position);
            LineSegment ls2(nextVert.position, currVert.position);

            if ((Vertex::GetHandedness(prevVert, currVert, nextVert) < 0.0f) &&
                CheckVisibility(origin, polygonVertices[index], polygonVertices)) {
                return index;
            }
        }

        float minDistance = std::numeric_limits<float>::max();
        int closest = indices[0];
        for (int index : indices) {
            float currDistance = Vec2::Square(polygonVertices[index].position - origin);
            if (currDistance < minDistance) {
                minDistance = currDistance;
                closest = index;
            }
        }
        return closest;
    }
    return -1;
}

void ConcavePolygon::ConvexDecomp(VertexArray const& polygonVertices) {
    int reflexIndex = FindFirstReflexVertex(polygonVertices);
    if (reflexIndex == -1) return;

    Vec2 prevVertPos = polygonVertices[Mod(reflexIndex - 1, static_cast<int>(polygonVertices.size()))].position;
    Vec2 currVertPos = polygonVertices[reflexIndex].position;
    Vec2 nextVertPos = polygonVertices[Mod(reflexIndex + 1, static_cast<int>(polygonVertices.size()))].position;

    LineSegment ls1(prevVertPos, currVertPos);
    LineSegment ls2(nextVertPos, currVertPos);

    IntArray vertsInCone = FindVerticesInCone(ls1, ls2, currVertPos, polygonVertices);

    int bestVert = -1;

    if (!vertsInCone.empty()) {
        bestVert = GetBestVertexToConnect(vertsInCone, polygonVertices, currVertPos);
        if (bestVert != -1) {
            LineSegment newLine(currVertPos, polygonVertices[bestVert].position);
            SlicePolygon(newLine);
        }
    }
    if (vertsInCone.empty() || (bestVert == -1)) {
        LineSegment newLine(currVertPos, (ls1.Direction() + ls2.Direction()) * 1e+10);
        SlicePolygon(newLine);
    }
}

int ConcavePolygon::FindFirstReflexVertex(VertexArray const& vertices) {
    for (size_t i = 0U; i < vertices.size(); ++i) {
        float handedness = Vertex::GetHandedness(vertices[Mod(static_cast<int>(i) - 1,
            static_cast<int>(vertices.size()))], vertices[i], vertices[Mod(static_cast<int>(i) + 1,
            static_cast<int>(vertices.size()))]);
        if (handedness < 0.0f) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

ConcavePolygon::VertexIntMap ConcavePolygon::CullByDistance(VertexIntMap const& input, Vec2 const& origin,
    int const& maxVertsToKeep) {
    const auto num = static_cast<int>(input.size());
    if (maxVertsToKeep >= num) {
        return input;
    }
    std::vector<SliceVertex> sliceVertices(input.size());
    size_t vert_index = 0U;
    for (auto it : input) {
        SliceVertex vert(it.second.position);
        vert.index = it.first;
        vert.distanceToSlice = Vec2::Square(it.second.position - origin);
        sliceVertices[vert_index] = vert;
        ++vert_index;
    }

    for (int i = 1; i < num; ++i) {
        for (int j = i; j > 0; --j) {
            if (sliceVertices[j].distanceToSlice <
                sliceVertices[j - 1].distanceToSlice) {
                std::swap(sliceVertices[j], sliceVertices[j - 1]);
            }
        }
    }

    for (int i = 1; i < maxVertsToKeep; ++i) {
        for (int j = i; j > 0; --j) {
            if (sliceVertices[j].index < sliceVertices[j - 1].index) {
                std::swap(sliceVertices[j], sliceVertices[j - 1]);
            }
        }
    }
    VertexIntMap result;
    for (int i = 0; i < maxVertsToKeep; ++i) {
        result.insert({sliceVertices[i].index, Vertex(sliceVertices[i].position)});
    }
    return result;
}

ConcavePolygon::VertexIntMap ConcavePolygon::VerticesAlongLineSegment(
        LineSegment const& segment, VertexArray const& _vertices) {
    VertexIntMap result;
    LineSegment tempSegment;

    for (size_t i = 0U; i < _vertices.size(); ++i) {
        tempSegment.startPos = _vertices[i].position;
        tempSegment.finalPos = _vertices[Mod(static_cast<int>(i) + 1, static_cast<int>(_vertices.size()))].position;

        std::pair<bool, Vec2> intersectionResult =
                LineSegment::Intersects(segment, tempSegment);

        if (intersectionResult.first) {
            result.insert({i, Vertex(intersectionResult.second)});
        }
    }

    return result;
}

void ConcavePolygon::SlicePolygon(int vertex1, int vertex2) {
    if ((vertex1 == vertex2) || (vertex2 == vertex1 + 1) ||
        (vertex2 == vertex1 - 1)) {
        return;
    }

    if (vertex1 > vertex2) {
        std::swap(vertex1, vertex2);
    }

    for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
        if ((i == vertex1) || (i == vertex2)) {
            left_vertices.emplace_back(vertices[i]);
            right_vertices.emplace_back(vertices[i]);
        } else if (i > vertex1 && i < vertex2) {
            left_vertices.emplace_back(vertices[i]);
        } else {
            right_vertices.emplace_back(vertices[i]);
        }
    }
}

void ConcavePolygon::SlicePolygon(LineSegment segment) {
    constexpr int num_max_verts = 2;
    VertexIntMap slicedVertices = VerticesAlongLineSegment(segment, vertices);
    slicedVertices = CullByDistance(slicedVertices, segment.startPos, num_max_verts);

    if (slicedVertices.size() < 2U) {
        return;
    }

    const float TOLERANCE = 1e-10;
    for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
        auto relativePosition = vertices[i].position - segment.startPos;
        float perpDistance = std::abs(Vec2::Cross(relativePosition, segment.Direction()));
        auto it = slicedVertices.begin();
        if ((perpDistance > TOLERANCE) ||
            ((perpDistance <= TOLERANCE) &&
             (slicedVertices.find(i) == slicedVertices.end()))) {
            if ((i > it->first) && (i <= (++it)->first)) {
                left_vertices.emplace_back(vertices[i]);
            } else {
                right_vertices.emplace_back(vertices[i]);
            }
        }

        if (slicedVertices.find(i) != slicedVertices.end()) {
            left_vertices.emplace_back(slicedVertices[i]);
            right_vertices.emplace_back(slicedVertices[i]);
        }
    }
}
}