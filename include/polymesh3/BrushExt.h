#pragma once

#include "polymesh3/typedef.h"

#include <string>
#include <vector>

namespace pm3
{

struct Brush;

struct BrushEdge
{
	BrushEdge() : begin(nullptr), end(nullptr) {}
	BrushEdge(const BrushVertexPtr& begin, const BrushVertexPtr& end)
		: begin(begin), end(end) {}

	bool operator == (const BrushEdge& e) const {
		return begin == e.begin && end == e.end;
	}
	operator bool() const {
		return begin && end;
	}

	BrushVertexPtr begin;
	BrushVertexPtr end;

}; // BrushEdge

struct BrushGroup
{
    std::string name;

    std::shared_ptr<Brush> parent = nullptr;

    std::vector<BrushVertexPtr> vertices;
    std::vector<BrushEdgePtr>   edges;
    std::vector<BrushFacePtr>   faces;

}; // BrushGroup

}