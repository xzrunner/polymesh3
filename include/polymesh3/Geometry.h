#pragma once

#include "polymesh3/typedef.h"

#include <SM_Vector.h>
#include <SM_Plane.h>
#include <halfedge/typedef.h>

#include <vector>
#include <map>
#include <memory>

namespace pm3
{

struct Point
{
    sm::vec3 pos;

}; // Point

struct Vertex
{
    int face_num  = -1;
    int point_num = -1;
    //sm::vec3 normal;

}; // Vertex

struct Edge
{
    int begin = -1;
    int end   = -1;

}; // Edge

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

}; // Face

enum class GroupType
{
    Points,
    Vertices,
    Edges,
    Face,
};

struct Group
{
    std::string name;

    GroupType type = GroupType::Face;

    std::vector<size_t> items;

    void Clear() { items.clear(); }

}; // Group

class Polytope
{
public:
    Polytope() {}
    Polytope(const std::vector<FacePtr>& faces);
    Polytope(const Polytope& poly);
    Polytope& operator = (const Polytope& poly);

    void Build();

    auto& Points() { return m_points; }
    auto& Points() const { return m_points; }
    auto& Faces() const { return m_faces; }

    void SetFaces(const std::vector<FacePtr>& faces);

    auto& GetHalfedge() const { return m_halfedge; }

    void AddGroup(const std::shared_ptr<Group>& group);
    std::shared_ptr<Group> QueryGroup(const std::string& name) const;
    //void RenameGroup(const std::string& src, const std::string& dst);

    void Combine(const Polytope& poly);

private:
    void CopyFaces(const std::vector<FacePtr>& faces);
    void CopyGroups(const Polytope& poly);

    void BuildVertices();
    void BuildHalfedge();

    bool IsPosExistInFace(const sm::vec3& pos, const Face& face) const;

    void SortFacePoints(Face& face);
    void InitFaceTexCoordSys(Face& face);

    sm::vec3 CalcFaceNormal(const Face& face) const;

private:
    std::vector<sm::vec3> m_points;
    std::vector<FacePtr>  m_faces;

    he::PolyhedronPtr m_halfedge = nullptr;

    std::map<std::string, std::shared_ptr<Group>> m_groups;

}; // Polytope

}