#pragma once

#include <memory>

namespace pm3
{

typedef size_t                    BrushVertex;
typedef std::pair<size_t, size_t> BrushEdge;

typedef std::shared_ptr<BrushVertex> BrushVertexPtr;
typedef std::shared_ptr<BrushEdge>   BrushEdgePtr;

struct BrushFace;
using BrushFacePtr = std::shared_ptr<BrushFace>;

}