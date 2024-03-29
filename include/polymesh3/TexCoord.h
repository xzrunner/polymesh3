#pragma once

#include <SM_Vector.h>

#include <string>

namespace pm3
{

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
	float    uw_factor = 0;

    sm::vec2 CalcTexCoords(const sm::vec3& pos, float tex_w, float tex_h) const
    {
	    sm::vec2 ret;

	    float sx = scale.x, sy = scale.y;
	    if (fabs(sx) < std::numeric_limits<float>::epsilon()) {
		    sx = 1;
	    }
	    if (fabs(sy) < std::numeric_limits<float>::epsilon()) {
		    sy = 1;
	    }

		auto p = pos;
		if (uw_factor != 0)
		{
			auto z_axis = sys.x_axis.Cross(sys.y_axis);
			p += sys.x_axis * z_axis.Dot(p) * uw_factor;
		}

	    ret.x = (p.Dot(sys.x_axis / sx) + offset.x) / tex_w;
	    ret.y = (p.Dot(sys.y_axis / sy) + offset.y) / tex_h;

	    return ret;
    }

}; // TextureMapping

}