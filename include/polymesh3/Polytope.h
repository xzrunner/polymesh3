#pragma once

#include "polymesh3/TexCoord.h"

#include <SM_Plane.h>
#include <halfedge/TopoID.h>
#include <halfedge/typedef.h>

namespace pm3
{

class Polytope
{
public:
    struct Point
    {
        Point(const sm::vec3& pos, const he::TopoID id = he::TopoID())
            : pos(pos), topo_id(id)
        {
        }

        sm::vec3 pos;

        he::TopoID topo_id;

    }; // Point

    using PointPtr = std::shared_ptr<Point>;

    struct Face
    {
        sm::Plane plane;
        std::vector<size_t> points;

        TextureMapping tex_map;

        he::TopoID topo_id;

    }; // Face

    using FacePtr = std::shared_ptr<Face>;

    typedef std::pair<size_t, size_t>   EdgeIndex;
    typedef std::shared_ptr<EdgeIndex>  EdgePtr;

public:
    Polytope() {}
    Polytope(const Polytope& poly);
    Polytope(const std::vector<FacePtr>& faces);
    Polytope(const std::vector<PointPtr>& points, const std::vector<FacePtr>& faces);
    Polytope(const he::PolyhedronPtr& halfedge);
    Polytope& operator = (const Polytope& poly);

    void BuildFromFaces();
    void BuildFromPoly();

    auto& Points() { return m_points; }
    auto& Points() const { return m_points; }
    auto& Faces() const { return m_faces; }

    void SetFaces(const std::vector<FacePtr>& faces);

    auto GetHePoly() const { return m_he_poly; }

    void Combine(const Polytope& poly);

private:
    void CopyPoints(const std::vector<PointPtr>& points);
    void CopyFaces(const std::vector<FacePtr>& faces);

    void BuildVertices();
    void BuildHalfedge();

    bool IsPosExistInFace(const sm::vec3& pos, const Face& face) const;

    void SortFacePoints(Face& face);
    void InitFaceTexCoordSys(Face& face);

    sm::vec3 CalcFaceNormal(const Face& face) const;

private:
    std::vector<PointPtr> m_points;
    std::vector<FacePtr>  m_faces;

    he::PolyhedronPtr m_he_poly = nullptr;

}; // Polytope

using PolytopePtr = std::shared_ptr<Polytope>;

}