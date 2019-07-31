#pragma once

#include "polymesh3/typedef.h"

#include <SM_Vector.h>
#include <SM_Plane.h>
#include <SM_Cube.h>
#include <halfedge/Polyhedron.h>

#include <memory>
#include <string>
#include <vector>

namespace pm3
{

struct TexCoordSystem
{
	size_t   index;
	sm::vec3 x_axis;
	sm::vec3 y_axis;

}; // TexCoordSystem

struct Brush;

struct BrushFace
{
	std::string tex_name;

	sm::Plane plane;
    std::vector<uint32_t> vertices;

	// texcoords
	TexCoordSystem tc_sys;
	sm::vec2 offset;
	float    angle = 0;
	sm::vec2 scale;

	void SortVertices(const Brush& brush);

	void InitTexCoordSys(const Brush& brush);

	sm::vec2 CalcTexCoords(const sm::vec3& pos, float tex_w, float tex_h) const;

    bool HasSamePos(const Brush& brush, const sm::vec3& pos) const;

private:
    sm::vec3 CalcNormal(const Brush& brush) const;

}; // BrushFace

struct Brush
{
    Brush() {}
    Brush(const Brush& brush);
	Brush(const std::vector<BrushFacePtr>& faces);

	void BuildVertices();

	void BuildGeometry();
	void RebuildGeometry(const sm::cube& world_bound);

    std::vector<sm::vec3>     vertices;
	std::vector<BrushFacePtr> faces;

	he::PolyhedronPtr geometry = nullptr;

}; // Brush

}
