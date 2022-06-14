//
// Created by ghm on 2021/12/29.
//

#include "concave_polygon.h"

namespace cxd {
bool ConcavePolygon::CheckIfRightHanded(VertexArray& vertex_array) {
	if(vertex_array.size() < 3U) {
		return false;
	}

	float signed_area = 0.0f;

	for(int i = 0; i< static_cast<int>(vertex_array.size()); ++i) {
		signed_area += Vec2::GetSignedArea(vertex_array[i].position,
			vertex_array[Mod(i + 1, static_cast<int>(vertex_array.size()))].position);
	}

	return (signed_area < 0.0f);
}

bool ConcavePolygon::IsVertexInCone(LineSegment const & ls1, LineSegment const & ls2,
                           Vec2 const & origin, Vertex const & vert) {
	auto relative_pos = vert.position - origin;
	float ls1_product = Vec2::Cross(relative_pos, ls1.Direction());
	float ls2_product = Vec2::Cross(relative_pos, ls2.Direction());

	return ((ls1_product < 0.0f) && (ls2_product > 0.0f));
}

ConcavePolygon::IntArray ConcavePolygon::FindVerticesInCone(LineSegment const & ls1, LineSegment const & ls2,
	Vec2 const & origin, VertexArray const & input_verts) {
	IntArray result;
	for(size_t i = 0U; i < input_verts.size(); ++i) {
		if(IsVertexInCone(ls1, ls2, origin, input_verts[i])) {
			result.emplace_back(static_cast<int>(i));
		}
	}
	return result;
}


int ConcavePolygon::GetBestVertexToConnect(IntArray const & indices, VertexArray const & polygon_vertices,
	Vec2 const & origin) {
	if(indices.size() == 1U) {
		if(CheckVisibility(origin, polygon_vertices[indices[0]], polygon_vertices))
			return indices[0];
	} else if(indices.size() > 1U) {
		for(int index : indices) {
			int vert_size = static_cast<int>(polygon_vertices.size());
			Vertex prev_vert = polygon_vertices[Mod(index - 1, vert_size)].position;
			Vertex curr_vert = polygon_vertices[index].position;
			Vertex next_vert = polygon_vertices[Mod(index + 1, vert_size)].position;

			LineSegment ls1(prev_vert.position, curr_vert.position);
			LineSegment ls2(next_vert.position, curr_vert.position);

			if((Vertex::GetHandedness(prev_vert, curr_vert, next_vert) < 0.0f) &&
			   IsVertexInCone(ls1, ls2, polygon_vertices[index].position, Vertex(origin)) &&
			   CheckVisibility(origin, polygon_vertices[index], polygon_vertices)) {
				return index;
			}
		}

		for(int index : indices) {
			int vertSize = static_cast<int>(polygon_vertices.size());
			Vertex prev_vert = polygon_vertices[Mod(index - 1, vertSize)].position;
			Vertex curr_vert = polygon_vertices[index].position;
			Vertex next_vert = polygon_vertices[Mod(index + 1, vertSize)].position;

			LineSegment ls1(prev_vert.position, curr_vert.position);
			LineSegment ls2(next_vert.position, curr_vert.position);

			if((Vertex::GetHandedness(prev_vert, curr_vert, next_vert) < 0.0f) &&
			   CheckVisibility(origin, polygon_vertices[index], polygon_vertices)) {
				return index;
			}
		}


		float min_distance = std::numeric_limits<float>::max();
		int closest = indices[0];
		for(int index : indices) {
			float curr_distance = Vec2::Square(polygon_vertices[index].position - origin);
			if(curr_distance < min_distance) {
				min_distance = curr_distance;
				closest = index;
			}
		}
		return closest;
	}
	return -1;
}

void ConcavePolygon::ConvexDecomp(VertexArray const & polygon_vertices) {
	if(!sub_polygons.empty()) {
		return;
	}

	int reflex_index = FindFirstReflexVertex(polygon_vertices);
	if(reflex_index == -1)
		return;

	Vec2 prev_vert_pos = polygon_vertices[Mod(reflex_index - 1, static_cast<int>(polygon_vertices.size()))].position;
	Vec2 curr_vert_pos = polygon_vertices[reflex_index].position;
	Vec2 next_vert_pos = polygon_vertices[Mod(reflex_index + 1, static_cast<int>(polygon_vertices.size()))].position;

	LineSegment ls1(prev_vert_pos, curr_vert_pos);
	LineSegment ls2(next_vert_pos, curr_vert_pos);

	IntArray verts_in_cone = FindVerticesInCone(ls1, ls2, curr_vert_pos, polygon_vertices);

	int best_vert = -1;

	if(!verts_in_cone.empty()) {
		best_vert = GetBestVertexToConnect(verts_in_cone, polygon_vertices, curr_vert_pos);
		if(best_vert != -1) {
			LineSegment new_line(curr_vert_pos, polygon_vertices[best_vert].position);
			SlicePolygon(new_line);
		}
	}
	if(verts_in_cone.empty() || (best_vert == -1)) {
		LineSegment new_line(curr_vert_pos, (ls1.Direction() + ls2.Direction()) * 1e+10);
		SlicePolygon(new_line);
	}

	for(auto& sub_polygon : sub_polygons) {
		sub_polygon.ConvexDecomp();
	}
}

int ConcavePolygon::FindFirstReflexVertex(VertexArray const & vertices) {
	for(size_t i = 0U; i < vertices.size(); ++i) {
		float handedness = Vertex::GetHandedness(vertices[Mod(static_cast<int>(i) - 1,
			static_cast<int>(vertices.size()))], vertices[i], vertices[Mod(static_cast<int>(i) + 1,
			static_cast<int>(vertices.size()))]);
		if(handedness < 0.0f) {
			return static_cast<int>(i);
		}
	}
	return -1;
}

ConcavePolygon::VertexIntMap ConcavePolygon::CullByDistance(VertexIntMap const & input, Vec2 const & origin,
	int const & max_verts_keep) {
	if(max_verts_keep >= static_cast<int>(input.size())) {
		return input;
	}

	std::vector<SliceVertex> slice_vertices;

	for(const auto& it : input) {
		SliceVertex vert(it.second.position);
		vert.index = it.first;
		vert.distance_slice = Vec2::Square(it.second.position - origin);

		slice_vertices.emplace_back(vert);
	}

	std::sort(slice_vertices.begin(), slice_vertices.end(), [](const SliceVertex& vetA, const SliceVertex& vetB) {
		return vetA.distance_slice < vetB.distance_slice;
	});
	slice_vertices.erase(slice_vertices.begin() + max_verts_keep, slice_vertices.end());

	std::sort(slice_vertices.begin(), slice_vertices.end(), [](const SliceVertex& vetA, const SliceVertex& vetB) {
		return vetA.index < vetB.index;
	});

	VertexIntMap result;
	for(const auto& sliceVertice : slice_vertices) {
		result.insert({sliceVertice.index, Vertex(sliceVertice.position)});
	}

	return result;
}

ConcavePolygon::VertexIntMap ConcavePolygon::VerticesAlongLineSegment(LineSegment const & segment,
	VertexArray const & vertices) {
	VertexIntMap result;
	LineSegment temp_segment;

	for(size_t i = 0U; i < vertices.size(); ++i) {
		temp_segment.start_pos = vertices[i].position;
		temp_segment.final_pos = vertices[Mod(static_cast<int>(i) + 1,
		                                      static_cast<int>(vertices.size()))].position;

		std::pair<bool, Vec2> intersection_result = LineSegment::Intersects(segment, temp_segment);

		if(intersection_result.first) {
			result.insert({i, Vertex(intersection_result.second)});
		}
	}

	return result;
}

void ConcavePolygon::SlicePolygon(LineSegment segment) {
	if(!sub_polygons.empty()) {
		sub_polygons[0].SlicePolygon(segment);
		sub_polygons[1].SlicePolygon(segment);
		return;
	}

	VertexIntMap sliced_vertices = VerticesAlongLineSegment(segment, vertices);
	sliced_vertices = CullByDistance(sliced_vertices, segment.start_pos, 2);

	if(sliced_vertices.size() < 2U) {
		return;
	}
	VertexArray left_verts;
	VertexArray right_verts;

	const float tolerance = 1e-5;
	for(int i = 0; i < static_cast<int>(vertices.size()); ++i) {
		auto relative_position = vertices[i].position - segment.start_pos;
		float perp_distance = std::abs(Vec2::Cross(relative_position, segment.Direction()));
		auto it = sliced_vertices.begin();
		if((perp_distance > tolerance) || ((perp_distance <= tolerance) &&
			(sliced_vertices.find(i) == sliced_vertices.end()))) {
			if((i > it->first) && (i <= (++it)->first)) {
				left_verts.emplace_back(vertices[i]);
			} else {
				right_verts.emplace_back(vertices[i]);
			}

		}

		if(sliced_vertices.find(i) != sliced_vertices.end()) {
			right_verts.emplace_back(sliced_vertices[i]);
			left_verts.emplace_back(sliced_vertices[i]);
		}
	}

	sub_polygons.emplace_back(ConcavePolygon(left_verts));
	sub_polygons.emplace_back(ConcavePolygon(right_verts));
}
}