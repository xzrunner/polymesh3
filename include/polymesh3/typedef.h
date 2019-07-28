#pragma once

#include <memory>

namespace pm3
{

struct BrushVertex;
using BrushVertexPtr = std::shared_ptr<BrushVertex>;

struct BrushEdge;
using BrushEdgePtr = std::shared_ptr<BrushEdge>;

struct BrushFace;
using BrushFacePtr = std::shared_ptr<BrushFace>;

}