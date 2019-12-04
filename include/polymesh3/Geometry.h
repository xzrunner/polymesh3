#pragma once

#include "polymesh3/typedef.h"

#include <SM_Vector.h>
#include <SM_Plane.h>
#include <halfedge/TopoID.h>
#include <halfedge/typedef.h>

#include <vector>
#include <map>
#include <memory>

namespace pm3
{

struct Point
{
    Point(const sm::vec3& pos, const he::TopoID id = he::TopoID())
        : pos(pos), topo_id(id)
    {
    }

    sm::vec3 pos;

    he::TopoID topo_id;

}; // Point

struct TexCoordSystem
{
    size_t   index = 0;
    sm::vec3 x_axis;
    sm::vec3 y_axis;

}; // TexCoordSystem

struct TextureMapping
{
    std::string tex_name;

    TexCoordSystem sys;
	sm::vec2 offset;
	float    angle = 0;
	sm::vec2 scale;

    sm::vec2 CalcTexCoords(const sm::vec3& pos, float tex_w, float tex_h) const;

}; // TextureMapping

struct Face
{
    sm::Plane plane;
    std::vector<size_t> points;

    TextureMapping tex_map;

    he::TopoID topo_id;

}; // Face

class Polytope
{
public:
    Polytope() {}
    Polytope(const Polytope& poly);
    Polytope(const std::vector<FacePtr>& faces);
    Polytope(const std::vector<PointPtr>& points, const std::vector<FacePtr>& faces);
    Polytope(const he::PolyhedronPtr& halfedge);
    Polytope& operator = (const Polytope& poly);

    void Build();
    void BuildFromGeo();

    auto& Points() { return m_points; }
    auto& Points() const { return m_points; }
    auto& Faces() const { return m_faces; }

    void SetFaces(const std::vector<FacePtr>& faces);

    auto& GetGeometry() const { return m_geo; }

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

    he::PolyhedronPtr m_geo = nullptr;

}; // Polytope

}