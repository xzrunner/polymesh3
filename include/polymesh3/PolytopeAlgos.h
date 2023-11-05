#pragma once

#include "Polytope.h"

namespace pm3
{

class PolytopeAlgos
{
public:
	static std::vector<PolytopePtr> 
		Intersect(const std::vector<PolytopePtr>& a, 
			const std::vector<PolytopePtr>& b);

	static std::vector<PolytopePtr>
		Subtract(const std::vector<PolytopePtr>& a,
			const std::vector<PolytopePtr>& b);

	static std::vector<PolytopePtr>
		Union(const std::vector<PolytopePtr>& a,
			const std::vector<PolytopePtr>& b);

	static void Extrude(PolytopePtr& poly, float dist);

	static PolytopePtr Offset(const PolytopePtr& poly,
		int keep, float dist);

private:
	static std::vector<PolytopePtr>
		SubtractImpl(const std::vector<PolytopePtr>& a,
			const std::vector<PolytopePtr>& b);

}; // PolytopeAlgos

}