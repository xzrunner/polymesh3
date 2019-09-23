#pragma once

#include <memory>

namespace pm3
{

struct Point;
using PointPtr = std::shared_ptr<Point>;

typedef std::pair<size_t, size_t>   EdgeIndex;
typedef std::shared_ptr<EdgeIndex>  EdgePtr;

struct Face;
using FacePtr = std::shared_ptr<Face>;

class Polytope;
using PolytopePtr = std::shared_ptr<Polytope>;

}