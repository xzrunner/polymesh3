#include "polymesh3/Polytope.h"

#include <SM_Calc.h>
#include <halfedge/Polyhedron.h>
#include <halfedge/Utility.h>

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

Polytope::Polytope(const Polytope& poly)
{
    CopyPoints(poly.m_points);
    CopyFaces(poly.m_faces);

    SetTopoDirty();
}

Polytope::Polytope(const std::vector<FacePtr>& faces)
{
    CopyFaces(faces);

    BuildVertices();

    SetTopoDirty();
}

Polytope::Polytope(const std::vector<PointPtr>& points,
                   const std::vector<FacePtr>& faces)
{
    CopyPoints(points);
    CopyFaces(faces);

    SetTopoDirty();
}

Polytope::Polytope(const he::PolyhedronPtr& topo)
    : m_topo_poly(topo)
{
    BuildFromTopo();
}

Polytope& Polytope::operator = (const Polytope& poly)
{
    m_points.clear();
    CopyPoints(poly.m_points);

    m_faces.clear();
    CopyFaces(poly.m_faces);

    SetTopoDirty();

    return *this;
}

void Polytope::BuildFromFaces()
{
    BuildVertices();

    SetTopoDirty();
}

void Polytope::BuildFromTopo()
{
    m_points.clear();
    m_faces.clear();

    std::map<he::vert3*, size_t> vert2idx;
    BuildPointsFromTopo(vert2idx);

    auto& faces = m_topo_poly->GetFaces();
    m_faces.reserve(faces.size());
    for (auto& face : faces) {
        m_faces.push_back(BuildFaceFromTopo(face, vert2idx));
    }
}

void Polytope::SetFaces(const std::vector<FacePtr>& faces)
{
    m_faces.clear();
    CopyFaces(faces);

    BuildFromFaces();
}

he::PolyhedronPtr Polytope::GetTopoPoly() 
{
    if (m_topo_dirty) {
        BuildTopoPoly();
    }
    return m_topo_poly; 
}

void Polytope::Combine(const Polytope& poly)
{
    size_t offset = m_points.size();

    m_points.reserve(m_points.size() + poly.m_points.size());
    for (auto& p : poly.m_points) {
        auto point = std::make_shared<Point>(*p);
        m_points.push_back(point);
    }

    m_faces.reserve(m_faces.size() + poly.m_faces.size());
    for (auto& f : poly.m_faces)
    {
        auto face = std::make_shared<Face>(*f);
        for (auto& p : face->border) {
            p += offset;
        }
        for (auto& hole : face->holes) {
            for (auto& p : hole) {
                p += offset;
            }
        }
        m_faces.push_back(face);
    }

    SetTopoDirty();
}

bool Polytope::CalcFaceNormal(const Face& face, sm::vec3& normal) const
{
    if (face.border.size() < 3) {
        return false;
    }

    sm::vec3 v;
    for (size_t i = 0, n = face.border.size(); i < n; ++i)
    {
        auto& p0 = m_points[face.border[i]]->pos;
        auto& p1 = m_points[face.border[(i + 1) % n]]->pos;
        v += p0.Cross(p1);
    }
    normal = v.Normalized();

    return true;
}

void Polytope::SortVertices()
{
    for (auto& f : m_faces)
    {
        assert(f->border.size() >= 3);
        SortFacePoints(*f);
        InitFaceTexCoordSys(*f);
    }
}

void Polytope::BuildPointsFromTopo(std::map<he::vert3*, size_t>& vert2idx)
{
    auto& vertices = m_topo_poly->GetVerts();
    if (vertices.Size() == 0) {
        return;
    }
    m_points.reserve(vertices.Size());

    auto curr_vert = vertices.Head();
    auto first_vert = curr_vert;
    do {
        vert2idx.insert({ curr_vert, m_points.size() });

        auto point = std::make_shared<Point>(curr_vert->position, curr_vert->ids);
        m_points.push_back(point);

        curr_vert = curr_vert->linked_next;
    } while (curr_vert != first_vert);
}

std::vector<size_t> Polytope::BuildLoopFromTopo(const he::loop3& loop, const std::map<he::vert3*, size_t>& vert2idx)
{
    std::vector<size_t> ret;

    auto first_e = loop.edge;
    auto curr_e = first_e;
    do {
        auto itr = vert2idx.find(curr_e->vert);
        assert(itr != vert2idx.end());
        ret.push_back(itr->second);

        curr_e = curr_e->next;
    } while (curr_e != first_e);

    return ret;
}

Polytope::FacePtr Polytope::BuildFaceFromTopo(const he::Polyhedron::Face& face, const std::map<he::vert3*, size_t>& vert2idx)
{
    auto ret = std::make_shared<Face>();

    he::Utility::LoopToPlane(*face.border, ret->plane);

    ret->border = BuildLoopFromTopo(*face.border, vert2idx);
    ret->holes.reserve(face.holes.size());
    for (auto& hole : face.holes) {
        ret->holes.push_back(BuildLoopFromTopo(*hole, vert2idx));
    }

    // todo tex_map

    ret->topo_id = face.border->ids;

    return ret;
}

void Polytope::CopyPoints(const std::vector<PointPtr>& points)
{
    m_points.reserve(points.size());
    for (auto& p : points) {
        m_points.push_back(std::make_shared<Point>(*p));
    }
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
					if (f->plane.GetDistance(v) > SM_LARGE_EPSILON) {
						legal = false;
						break;
					}
				}

				if (legal)
				{
                    bool find0 = IsPosExist(v, *m_faces[i]);
                    bool find1 = IsPosExist(v, *m_faces[j]);
                    bool find2 = IsPosExist(v, *m_faces[k]);
					if (!find0 || !find1 || !find2)
                    {
                        const int idx = m_points.size();
                        m_points.push_back(std::make_shared<Point>(v));
                        if (!find0) { m_faces[i]->border.push_back(idx); }
                        if (!find1) { m_faces[j]->border.push_back(idx); }
                        if (!find2) { m_faces[k]->border.push_back(idx); }
					}
				}
			}
		}
	}

    SortVertices();
}

void Polytope::BuildTopoPoly()
{
    std::vector<he::Polyhedron::in_vert> verts;
    verts.reserve(m_points.size());
    for (auto& point : m_points) {
        verts.push_back({ point->topo_id, point->pos });
    }

    std::vector<he::Polyhedron::in_face> faces;
    faces.reserve(m_faces.size());
    std::vector<size_t> loop_n;
    for (auto& face : m_faces)
    {
        he::Polyhedron::in_loop border;
        std::vector<he::Polyhedron::in_loop> holes;

        border = face->border;

        holes.reserve(face->holes.size());
        for (auto& loop : face->holes) {
            holes.push_back(loop);
        }

        faces.push_back({ face->topo_id, border, holes });
        loop_n.push_back(face->holes.size() + 1);
    }

	m_topo_poly = std::make_shared<he::Polyhedron>(verts, faces);

    assert(m_points.size() == m_topo_poly->GetVerts().Size());
    auto first_vert = m_topo_poly->GetVerts().Head();
    auto curr_vert = first_vert;
    size_t idx_vert = 0;
    do {
        assert(m_points[idx_vert]->pos == curr_vert->position);
        m_points[idx_vert]->topo_id = curr_vert->ids;

        ++idx_vert;
        curr_vert = curr_vert->linked_next;
    } while (curr_vert != first_vert);

    assert(loop_n.size() == m_faces.size());
    auto first_face = m_topo_poly->GetLoops().Head();
    auto curr_face = first_face;
    size_t idx_face = 0;
    do {
        m_faces[idx_face]->topo_id = curr_face->ids;

        for (size_t i = 0; i < loop_n[idx_face] - 1; ++i) {
            curr_face = curr_face->linked_next;
            assert(curr_face != first_face);
        }

        ++idx_face;

        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);

    m_topo_dirty = false;
}

bool Polytope::IsPosExist(const sm::vec3& pos, const Face& face) const
{
    for (auto& p : face.border) {
        if (sm::dis_square_pos3_to_pos3(m_points[p]->pos, pos) < SM_LARGE_EPSILON) {
            return true;
        }
    }
    for (auto& hole : face.holes) {
        for (auto& p : hole) {
            if (sm::dis_square_pos3_to_pos3(m_points[p]->pos, pos) < SM_LARGE_EPSILON) {
                return true;
            }
        }
    }
    return false;
}

void Polytope::SortFacePoints(Face& face)
{
	sm::vec3 center;
	for (auto& v : face.border) {
		center += m_points[v]->pos;
	}
	center /= static_cast<float>(face.border.size());

	for (size_t i = 0; i < face.border.size() - 2; ++i)
	{
		float smallest_angle = -1;
		int   smallest_idx = -1;

        auto& pos = m_points[face.border[i]]->pos;

		sm::vec3 a = (pos - center).Normalized();
		sm::Plane p(pos, center, center + face.plane.normal);
		for (size_t j = i + 1; j < face.border.size(); ++j)
		{
            auto& pos = m_points[face.border[j]]->pos;
            float dis = p.GetDistance(pos);
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
			std::swap(face.border[smallest_idx], face.border[i + 1]);
		}
	}

	// fix back faces
    sm::vec3 normal = CalcFaceNormal(face);
	if (normal.Dot(face.plane.normal) < -SM_LARGE_EPSILON) {
		std::reverse(std::begin(face.border), std::end(face.border));
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
    assert(face.border.size() > 2);

    sm::vec3 invalid;
    invalid.MakeInvalid();
    if (face.border.size() < 3) {
        return invalid;
    }

    for (size_t i = 0, n = face.border.size(); i < n; ++i)
    {
        auto& p0 = m_points[face.border[i]]->pos;
        auto& p1 = m_points[face.border[(i + 1) % n]]->pos;
        auto& p2 = m_points[face.border[(i + 2) % n]]->pos;

        auto cross = (p1 - p0).Cross(p2 - p0);
        if (cross.LengthSquared() > 0) {
            return cross.Normalized();
        }
    }

    assert(0);
    return invalid;
}

}