#include "polymesh3/Geometry.h"

#include <SM_Calc.h>
#include <halfedge/Polyhedron.h>

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
// struct TextureMapping
//////////////////////////////////////////////////////////////////////////

sm::vec2 TextureMapping::CalcTexCoords(const sm::vec3& pos, float tex_w, float tex_h) const
{
	sm::vec2 ret;

	float sx = scale.x, sy = scale.y;
	if (fabs(sx) < std::numeric_limits<float>::epsilon()) {
		sx = 1;
	}
	if (fabs(sy) < std::numeric_limits<float>::epsilon()) {
		sy = 1;
	}

	ret.x = (pos.Dot(sys.x_axis / sx) + offset.x) / tex_w;
	ret.y = (pos.Dot(sys.y_axis / sy) + offset.y) / tex_h;

	return ret;
}

//////////////////////////////////////////////////////////////////////////
// class Polytope
//////////////////////////////////////////////////////////////////////////

Polytope::Polytope(const std::vector<FacePtr>& faces)
{
    CopyFaces(faces);

    Build();
}

Polytope::Polytope(const Polytope& poly)
{
    operator = (poly);
}

Polytope& Polytope::operator = (const Polytope& poly)
{
    m_points = poly.m_points;
    CopyFaces(poly.m_faces);

    BuildHalfedge();

    return *this;
}

void Polytope::Build()
{
    BuildVertices();
    BuildHalfedge();
}

void Polytope::SetFaces(const std::vector<FacePtr>& faces)
{
    m_faces.clear();
    CopyFaces(faces);

    Build();
}

void Polytope::CopyFaces(const std::vector<FacePtr>& faces)
{
    m_faces.reserve(faces.size());
    for (auto& f : faces) {
        m_faces.push_back(std::make_shared<Face>(*f));
    }
}

void Polytope::BuildVertices()
{
	// create vertices
	int n = m_faces.size();
	for (int i = 0; i < n - 2; ++i) {
		for (int j = i + 1; j < n - 1; ++j) {
			for (int k = j + 1; k < n; ++k) {
				sm::vec3 v;
				if (!sm::intersect_planes(m_faces[i]->plane, m_faces[j]->plane, m_faces[k]->plane, &v)) {
					continue;
				}

				// Test if the point is outside the brush
				bool legal = true;
				for (auto& f : m_faces) {
					// plane front, outside
					if (f->plane.normal.Dot(v) + f->plane.dist > SM_LARGE_EPSILON) {
						legal = false;
						break;
					}
				}

				if (legal)
				{
                    bool find0 = IsPosExistInFace(v, *m_faces[i]);
                    bool find1 = IsPosExistInFace(v, *m_faces[j]);
                    bool find2 = IsPosExistInFace(v, *m_faces[k]);
					if (!find0 || !find1 || !find2)
                    {
                        const int idx = m_points.size();
                        m_points.push_back(v);
                        if (!find0) { m_faces[i]->points.push_back(idx); }
                        if (!find1) { m_faces[j]->points.push_back(idx); }
                        if (!find2) { m_faces[k]->points.push_back(idx); }
					}
				}
			}
		}
	}

	// sort vertices
	for (auto& f : m_faces)
	{
		assert(f->points.size() >= 3);
        SortFacePoints(*f);
        InitFaceTexCoordSys(*f);
	}
}

void Polytope::BuildHalfedge()
{
	std::vector<std::vector<sm::vec3>> faces_pos;
	faces_pos.reserve(m_faces.size());
	for (auto& face : m_faces) {
		std::vector<sm::vec3> vs;
        vs.reserve(face->points.size());
		for (auto& p : face->points) {
            vs.push_back(m_points[p]);
		}
		faces_pos.push_back(vs);
	}

	m_halfedge = std::make_shared<he::Polyhedron>(faces_pos);
}

bool Polytope::IsPosExistInFace(const sm::vec3& pos, const Face& face) const
{
    for (auto& p : face.points) {
        if (sm::dis_square_pos3_to_pos3(m_points[p], pos) < SM_LARGE_EPSILON) {
            return true;
        }
    }
    return false;
}

void Polytope::SortFacePoints(Face& face)
{
	sm::vec3 center;
	for (auto& v : face.points) {
		center += m_points[v];
	}
	center /= static_cast<float>(face.points.size());

	for (size_t i = 0; i < face.points.size() - 2; ++i)
	{
		float smallest_angle = -1;
		int   smallest_idx = -1;

        auto& pos = m_points[face.points[i]];

		sm::vec3 a = (pos - center).Normalized();
		sm::Plane p(pos, center, center + face.plane.normal);
		for (size_t j = i + 1; j < face.points.size(); ++j)
		{
            auto& pos = m_points[face.points[j]];
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
			std::swap(face.points[smallest_idx], face.points[i + 1]);
		}
	}

	// fix back faces
    sm::vec3 normal = CalcFaceNormal(face);
	if (normal.Dot(face.plane.normal) < -SM_LARGE_EPSILON) {
		std::reverse(std::begin(face.points), std::end(face.points));
	}
}

void Polytope::InitFaceTexCoordSys(Face& face)
{
    sm::vec3 normal = CalcFaceNormal(face);

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

	face.tex_map.sys.index = best_idx;
    face.tex_map.sys.x_axis = BASE_AXES[best_idx * 3 + 1];
    face.tex_map.sys.y_axis = BASE_AXES[best_idx * 3 + 2];
}

sm::vec3 Polytope::CalcFaceNormal(const Face& face) const
{
    assert(face.points.size() > 2);
    auto& p0 = m_points[face.points[0]];
    auto& p1 = m_points[face.points[1]];
    auto& p2 = m_points[face.points[2]];
    return (p1 - p0).Cross(p2 - p0).Normalized();
}

}