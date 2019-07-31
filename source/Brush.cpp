#include "polymesh3/Brush.h"

#include <sm_const.h>
#include <SM_Calc.h>

namespace
{

const sm::vec3 BASE_AXES[] =
{
    sm::vec3( 0.0f,  0.0f,  1.0f), sm::vec3( 1.0f,  0.0f,  0.0f), sm::vec3( 0.0f, -1.0f,  0.0f),
    sm::vec3( 0.0f,  0.0f, -1.0f), sm::vec3( 1.0f,  0.0f,  0.0f), sm::vec3( 0.0f, -1.0f,  0.0f),
    sm::vec3( 1.0f,  0.0f,  0.0f), sm::vec3( 0.0f,  1.0f,  0.0f), sm::vec3( 0.0f,  0.0f, -1.0f),
    sm::vec3(-1.0f,  0.0f,  0.0f), sm::vec3( 0.0f,  1.0f,  0.0f), sm::vec3( 0.0f,  0.0f, -1.0f),
    sm::vec3( 0.0f,  1.0f,  0.0f), sm::vec3( 1.0f,  0.0f,  0.0f), sm::vec3( 0.0f,  0.0f, -1.0f),
    sm::vec3( 0.0f, -1.0f,  0.0f), sm::vec3( 1.0f,  0.0f,  0.0f), sm::vec3( 0.0f,  0.0f, -1.0f),
};

}

namespace pm3
{

//////////////////////////////////////////////////////////////////////////
// class BrushFace
//////////////////////////////////////////////////////////////////////////

void BrushFace::SortVertices(const Brush& brush)
{
	sm::vec3 center;
	for (auto& v : vertices) {
		center += brush.vertices[v];
	}
	center /= static_cast<float>(vertices.size());

	for (size_t i = 0; i < vertices.size() - 2; ++i)
	{
		float smallest_angle = -1;
		int   smallest_idx = -1;

        auto& pos = brush.vertices[vertices[i]];

		sm::vec3 a = (pos - center).Normalized();
		sm::Plane p(pos, center, center + plane.normal);
		for (size_t j = i + 1; j < vertices.size(); ++j)
		{
            auto& pos = brush.vertices[vertices[j]];
			float dis = p.normal.Dot(pos) + p.dist;
			// black
			if (dis < -SM_LARGE_EPSILON) {
				;
			} else {
				sm::vec3 b = (pos - center).Normalized();
				float angle = a.Dot(b);
				if (angle > smallest_angle) {
					smallest_angle = angle;
					smallest_idx = j;
				}
			}
		}

		if (smallest_idx == -1 )
		{
			assert(0);
			//std::cout << "Error: Degenerate polygon!" << std::endl
			//abort ( );
		}

		if (smallest_idx != i + 1) {
			std::swap(vertices[smallest_idx], vertices[i + 1]);
		}
	}

	// fix back faces
    sm::vec3 normal = CalcNormal(brush);
	if (normal.Dot(plane.normal) < -SM_LARGE_EPSILON) {
		std::reverse(std::begin(vertices), std::end(vertices));
	}
}

void BrushFace::InitTexCoordSys(const Brush& brush)
{
    sm::vec3 normal = CalcNormal(brush);

	size_t best_idx = 0;
	float  best_dot = 0;
	for (size_t i = 0; i < 6; ++i)
	{
		float dot = normal.Dot(BASE_AXES[i * 3]);
		if (dot > best_dot) {
			best_dot = dot;
			best_idx = i;
		}
	}

	tc_sys.index = best_idx;
	tc_sys.x_axis = BASE_AXES[best_idx * 3 + 1];
	tc_sys.y_axis = BASE_AXES[best_idx * 3 + 2];
}

sm::vec2 BrushFace::CalcTexCoords(const sm::vec3& pos, float tex_w, float tex_h) const
{
	sm::vec2 ret;

	float sx = scale.x, sy = scale.y;
	if (fabs(sx) < std::numeric_limits<float>::epsilon()) {
		sx = 1;
	}
	if (fabs(sy) < std::numeric_limits<float>::epsilon()) {
		sy = 1;
	}

	ret.x = (pos.Dot(tc_sys.x_axis / sx) + offset.x) / tex_w;
	ret.y = (pos.Dot(tc_sys.y_axis / sy) + offset.y) / tex_h;

	return ret;
}

bool BrushFace::HasSamePos(const Brush& brush, const sm::vec3& pos) const
{
    for (auto& vert : vertices) {
        if (sm::dis_square_pos3_to_pos3(brush.vertices[vert], pos) < SM_LARGE_EPSILON) {
            return true;
        }
    }
    return false;
}

sm::vec3 BrushFace::CalcNormal(const Brush& brush) const
{
    assert(vertices.size() > 2);
    auto& p0 = brush.vertices[vertices[0]];
    auto& p1 = brush.vertices[vertices[1]];
    auto& p2 = brush.vertices[vertices[2]];
    return (p1 - p0).Cross(p2 - p0).Normalized();
}

//////////////////////////////////////////////////////////////////////////
// class Brush
//////////////////////////////////////////////////////////////////////////

Brush::Brush(const Brush& brush)
    : vertices(brush.vertices)
    , faces(brush.faces)
{
    BuildGeometry();
}

Brush::Brush(const std::vector<BrushFacePtr>& faces)
	: faces(faces)
{
	//static sm::cube WORLD_BOUND(1024, 1024, 1024);
	//RebuildGeometry(WORLD_BOUND);
}

void Brush::BuildVertices()
{
	// create vertices
	int n = faces.size();
	for (int i = 0; i < n - 2; ++i) {
		for (int j = i + 1; j < n - 1; ++j) {
			for (int k = j + 1; k < n; ++k) {
				sm::vec3 v;
				if (!sm::intersect_planes(faces[i]->plane, faces[j]->plane, faces[k]->plane, &v)) {
					continue;
				}

				// Test if the point is outside the brush
				bool legal = true;
				for (auto& f : faces) {
					// plane front, outside
					if (f->plane.normal.Dot(v) + f->plane.dist > SM_LARGE_EPSILON) {
						legal = false;
						break;
					}
				}

				if (legal)
				{
					bool find0 = faces[i]->HasSamePos(*this, v);
					bool find1 = faces[j]->HasSamePos(*this, v);
					bool find2 = faces[k]->HasSamePos(*this, v);
					if (!find0 || !find1 || !find2)
                    {
                        const int idx = vertices.size();
						vertices.push_back(v);
                        if (!find0) { faces[i]->vertices.push_back(idx); }
                        if (!find1) { faces[j]->vertices.push_back(idx); }
                        if (!find2) { faces[k]->vertices.push_back(idx); }
					}
				}
			}
		}
	}

	// sort vertices
	for (auto& f : faces)
	{
		assert(f->vertices.size() >= 3);
		f->SortVertices(*this);
		f->InitTexCoordSys(*this);
	}
}

void Brush::BuildGeometry()
{
	std::vector<std::vector<sm::vec3>> faces_pos;
	faces_pos.reserve(faces.size());
	for (auto& face : faces) {
		std::vector<sm::vec3> vs;
        vs.reserve(face->vertices.size());
		for (auto& vert : face->vertices) {
            vs.push_back(vertices[vert]);
		}
		faces_pos.push_back(vs);
	}

	geometry = std::make_shared<he::Polyhedron>(faces_pos);
}

void Brush::RebuildGeometry(const sm::cube& world_bound)
{
	geometry = std::make_shared<he::Polyhedron>(world_bound);
}

}