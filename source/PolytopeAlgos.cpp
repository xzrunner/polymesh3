#include "polymesh3/PolytopeAlgos.h"
#include "polymesh3/TopoPolyAdapter.h"

#include <halfedge/Polygon.h>
#include <halfedge/Polyhedron.h>

#include <iterator>

namespace pm3
{

std::vector<PolytopePtr>
PolytopeAlgos::Intersect(const std::vector<PolytopePtr>& a, const std::vector<PolytopePtr>& b,
                         std::vector<std::pair<PolytopePtr, PolytopePtr>>& hist)
{
    std::vector<PolytopePtr> ret;
    for (auto& pa : a)
    {
        auto topo_a = pa->GetTopoPoly();
        if (!topo_a) {
            continue;
        }
        for (auto& pb : b)
        {
            auto topo_b = pb->GetTopoPoly();
            if (!topo_b) {
                continue;
            }

            auto poly = topo_a->Intersect(*topo_b);
            if (poly && poly->GetLoops().Size() > 0) 
            {
                auto dst = std::make_shared<Polytope>(poly);
                hist.push_back({ pa, dst });
                ret.push_back(dst);
            }
        }
    }
    return ret;
}

std::vector<PolytopePtr>
PolytopeAlgos::Subtract(const std::vector<PolytopePtr>& a, const std::vector<PolytopePtr>& b,
                        std::vector<std::pair<PolytopePtr, PolytopePtr>>& hist)
{
    std::vector<PolytopePtr> polytopes = a;
    for (auto& cb : b)
    {
        auto intersect = PolytopeAlgos::Intersect(polytopes, { cb }, hist);
        if (!intersect.empty()) {
            polytopes = PolytopeAlgos::SubtractImpl(polytopes, intersect, hist);
        }
    }
    return polytopes;
}

std::vector<PolytopePtr>
PolytopeAlgos::Union(const std::vector<PolytopePtr>& a, const std::vector<PolytopePtr>& b,
                     std::vector<std::pair<PolytopePtr, PolytopePtr>>& hist)
{
    std::vector<PolytopePtr> polytopes;

    auto intersect = PolytopeAlgos::Intersect(a, b, hist);
    if (intersect.empty())
    {
        std::copy(a.begin(), a.end(), std::back_inserter(polytopes));
        std::copy(b.begin(), b.end(), std::back_inserter(polytopes));
    }
    else
    {
        auto a_left = PolytopeAlgos::Subtract(a, intersect, hist);
        auto b_left = PolytopeAlgos::Subtract(b, intersect, hist);
        polytopes = intersect;
        std::copy(a_left.begin(), a_left.end(), std::back_inserter(polytopes));
        std::copy(b_left.begin(), b_left.end(), std::back_inserter(polytopes));
    }

    return polytopes;
}

void PolytopeAlgos::Extrude(PolytopePtr& poly, float dist)
{
    auto topo_poly = poly->GetTopoPoly();
    if (!topo_poly) {
        return;
    }

    bool create_face[he::Polyhedron::ExtrudeMaxCount];
    create_face[he::Polyhedron::ExtrudeFront] = true;
    create_face[he::Polyhedron::ExtrudeBack] = true;
    create_face[he::Polyhedron::ExtrudeSide] = true;

    std::vector<he::TopoID> face_ids;
    auto& faces = topo_poly->GetLoops();
    face_ids.reserve(faces.Size());
    auto first_f = faces.Head();
    auto curr_f = first_f;
    do {
        face_ids.push_back(curr_f->ids);
        curr_f = curr_f->linked_next;
    } while (curr_f != first_f);

    if (!topo_poly->Extrude(dist, face_ids, create_face)) {
        return;
    }

    poly->BuildFromTopo();
}

PolytopePtr PolytopeAlgos::Offset(const PolytopePtr& poly, int keep, float dist)
{
    std::vector<Polytope::PointPtr> dst_pts;
    std::vector<Polytope::FacePtr>  dst_faces;

    auto& src_points = poly->Points();
    auto& src_faces = poly->Faces();
    for (auto& src_f : src_faces)
    {
        std::vector<sm::vec3> src_poly;
        src_poly.reserve(src_f->border.size());
        for (auto& p : src_f->border) {
            src_poly.push_back(src_points[p]->pos);
        }

        TopoPolyAdapter topo_poly(src_poly);
        topo_poly.GetPoly()->Offset(dist, static_cast<he::Polygon::KeepType>(keep));
        topo_poly.TransToPolymesh(dst_pts, dst_faces);
    }

    return std::make_shared<Polytope>(dst_pts, dst_faces);
}

std::vector<PolytopePtr>
PolytopeAlgos::SubtractImpl(const std::vector<PolytopePtr>& a, const std::vector<PolytopePtr>& b,
                            std::vector<std::pair<PolytopePtr, PolytopePtr>>& hist)
{
    std::vector<PolytopePtr> ret = a;
    for (auto& pa : a)
    {
        auto topo_a = pa->GetTopoPoly();
        if (!topo_a) {
            continue;
        }
        for (auto& pb : b)
        {
            auto topo_b = pb->GetTopoPoly();
            if (!topo_b) {
                continue;
            }

            auto intersect = topo_a->Intersect(*topo_b);
            if (intersect && intersect->GetLoops().Size() > 0)
            {
                auto a_left = a;
                for (auto itr = a_left.begin(); itr != a_left.end(); ++itr) {
                    if (*itr == pa) {
                        a_left.erase(itr);
                        break;
                    }
                }
                auto a_sub = topo_a->Subtract(*topo_b);
                for (auto& p : a_sub) 
                {
                    auto dst = std::make_shared<Polytope>(p);
                    hist.push_back({ pa, dst });
                    a_left.push_back(dst);
                }

                return Subtract(a_left, b, hist);
            }
        }
    }
    return ret;
}

}