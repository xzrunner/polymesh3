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

Polytope::Polytope(const Polytope& poly)
{
    CopyPoints(poly.m_points);
    CopyFaces(poly.m_faces);

    BuildHalfedge();
}

Polytope::Polytope(const std::vector<FacePtr>& faces)
{
    CopyFaces(faces);

    BuildVertices();
    BuildHalfedge();
}

Polytope::Polytope(const std::vector<PointPtr>& points,
                   const std::vector<FacePtr>& faces)
{
    CopyPoints(points);
    CopyFaces(faces);

    BuildHalfedge();
}

Polytope::Polytope(const he::PolyhedronPtr& halfedge)
    : m_geo(halfedge)
{
    BuildFromGeo();
}

Polytope& Polytope::operator = (const Polytope& poly)
{
    m_points.clear();
    CopyPoints(poly.m_points);

    m_faces.clear();
    CopyFaces(poly.m_faces);

    BuildHalfedge();

    return *this;
}

void Polytope::Build()
{
    BuildVertices();
    BuildHalfedge();
}

void Polytope::BuildFromGeo()
{
    m_points.clear();
    m_faces.clear();

    auto& vertices = m_geo->GetVertices();
    if (vertices.Size() == 0) {
        return;
    }
    m_points.reserve(vertices.Size());
    std::map<he::Vertex*, size_t> vert2idx;
    auto curr_vert = vertices.Head();
    auto first_vert = curr_vert;
    do {
        vert2idx.insert({ curr_vert, m_points.size() });

        auto point = std::make_shared<Point>();
        point->pos = curr_vert->position;
        point->topo_id = curr_vert->ids;
        m_points.push_back(point);

        curr_vert = curr_vert->linked_next;
    } while (curr_vert != first_vert);

    auto& faces = m_geo->GetFaces();
    m_faces.reserve(faces.Size());
    auto curr_face = faces.Head();
    auto first_face = curr_face;
    do {
        auto face = std::make_shared<Face>();
        he::face_to_plane(*curr_face, face->plane);
        face->topo_id = curr_face->ids;
        m_faces.push_back(face);

        auto curr_edge = curr_face->edge;
        auto first_edge = curr_edge;
        do {
            auto itr = vert2idx.find(curr_edge->vert);
            assert(itr != vert2idx.end());
            face->points.push_back(itr->second);

            curr_edge = curr_edge->prev;
        } while (curr_edge != first_edge);

        // todo tex_map

        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);
}

void Polytope::SetFaces(const std::vector<FacePtr>& faces)
{
    m_faces.clear();
    CopyFaces(faces);

    Build();
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
        for (auto& p : face->points) {
            p += offset;
        }
        m_faces.push_back(face);
    }

    BuildHalfedge();
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
                        auto point = std::make_shared<Point>();
                        point->pos = v;
                        m_points.push_back(point);
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
    std::vector<std::pair<he::TopoID, sm::vec3>> vertices;
    vertices.reserve(m_points.size());
    for (auto& point : m_points) {
        vertices.push_back({ point->topo_id, point->pos });
    }

    std::vector<std::pair<he::TopoID, std::vector<size_t>>> faces;
    faces.reserve(m_faces.size());
    for (auto& face : m_faces)
    {
        auto points = face->points;
        std::reverse(points.begin(), points.end());
        faces.push_back({ face->topo_id, points });
    }

	m_geo = std::make_shared<he::Polyhedron>(vertices, faces);

    assert(m_points.size() == m_geo->GetVertices().Size());
    auto first_vert = m_geo->GetVertices().Head();
    auto curr_vert = first_vert;
    size_t idx_vert = 0;
    do {
        assert(m_points[idx_vert]->pos == curr_vert->position);
        m_points[idx_vert]->topo_id = curr_vert->ids;

        ++idx_vert;
        curr_vert = curr_vert->linked_next;
    } while (curr_vert != first_vert);

    assert(m_faces.size() == m_geo->GetFaces().Size());
    auto first_face = m_geo->GetFaces().Head();
    auto curr_face = first_face;
    size_t idx_face = 0;
    do {
        m_faces[idx_face]->topo_id = curr_face->ids;

        ++idx_face;
        curr_face = curr_face->linked_next;
    } while (curr_face != first_face);
}

bool Polytope::IsPosExistInFace(const sm::vec3& pos, const Face& face) const
{
    for (auto& p : face.points) {
        if (sm::dis_square_pos3_to_pos3(m_points[p]->pos, pos) < SM_LARGE_EPSILON) {
            return true;
        }
    }
    return false;
}

void Polytope::SortFacePoints(Face& face)
{
	sm::vec3 center;
	for (auto& v : face.points) {
		center += m_points[v]->pos;
	}
	center /= static_cast<float>(face.points.size());

	for (size_t i = 0; i < face.points.size() - 2; ++i)
	{
		float smallest_angle = -1;
		int   smallest_idx = -1;

        auto& pos = m_points[face.points[i]]->pos;

		sm::vec3 a = (pos - center).Normalized();
		sm::Plane p(pos, center, center + face.plane.normal);
		for (size_t j = i + 1; j < face.points.size(); ++j)
		{
            auto& pos = m_points[face.points[j]]->pos;
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
    auto& p0 = m_points[face.points[0]]->pos;
    auto& p1 = m_points[face.points[1]]->pos;
    auto& p2 = m_points[face.points[2]]->pos;
    return (p1 - p0).Cross(p2 - p0).Normalized();
}

}