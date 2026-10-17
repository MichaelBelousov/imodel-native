/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
// Rebuilds Parasolid bodies from their transmit data with SmBrepConstructor: each face gets an
// SMLib surface covering it, each edge an SMLib curve running from its start vertex to its end
// vertex, and SMLib's healer then adds most seams and the trimming curves.
//
// Parasolid conventions relied on: a fin's vertex is at its forward end; an edge runs along its
// curve unless the curve's sense is '-'; a fin runs along its edge unless its sense is '-'; loops
// keep the face on their left seen against the face normal, which is the surface's natural normal
// reversed by each of the surface's and the face's '-' senses.

#include "SmKernelImpl.h"
#include "ParasolidXT.h"

#include <SmAttribute.h>
#include <SmAxis2Placement.h>
#include <SmBrepData.h>
#include <SmBSplineSurface.h>
#include <SmCone.h>
#include <SmCylinder.h>
#include <SmEdge.h>
#include <SmExtent1d.h>
#include <SmExtent2d.h>
#include <SmFace.h>
#include <SmOffsetSurface.h>
#include <SmPlane.h>
#include <SmSphere.h>
#include <SmTorus.h>
#include <SmVertex.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <set>

namespace SmKernel {

namespace {

namespace XT = ParasolidXT;
using XT::Node;
using Vec = SmVector3d;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegrees = 180.0 / kPi;
constexpr ULONG kFaceTagAttribute = SM_AI_USER_1 + 2718;

Vec V(Node const& node, char const* field) {
    double const* v = node.Values(field);
    return 3 <= node.Count(field) ? Vec(v[0], v[1], v[2]) : Vec(0, 0, 0);
}
Vec Unit(Vec const& v) {
    double const length = v.Length();
    return length > 0.0 ? v / length : v;
}
double Sense(Node const& node) { return '-' == node.Char("sense") ? -1.0 : 1.0; }
double Wrap2Pi(double a) {
    a = std::fmod(a, 2.0 * kPi);
    return a < 0.0 ? a + 2.0 * kPi : a;
}
double WrapPi(double a) {
    a = Wrap2Pi(a);
    return a > kPi ? a - 2.0 * kPi : a;
}

/*---------------------------------------------------------------------------------**//**
* NURBS from transmit data, in homogeneous form while they are evaluated or clamped.
+---------------+---------------+---------------+---------------+---------------+------*/
struct Nurbs {
    int degree = 1;
    int dim = 3;           //!< Euclidean coordinates per point
    bool rational = false;
    std::vector<double> knots;
    std::vector<double> points; //!< dim (+1 weight if rational) values per point, premultiplied by the weight

    int Stride() const { return dim + (rational ? 1 : 0); }
    size_t Count() const { return points.size() / Stride(); }
    double Start() const { return knots[degree]; }
    double End() const { return knots[Count()]; }
};

std::vector<double> ExpandKnots(Node const* mult, Node const* knots, int count) {
    std::vector<double> out;
    if (nullptr == mult || nullptr == knots)
        return out;
    for (int i = 0; i < count && i < static_cast<int>(mult->Count("mult")) && i < static_cast<int>(knots->Count("knots")); ++i)
        for (int j = 0; j < mult->Int("mult", i); ++j)
            out.push_back(knots->Real("knots", i));
    return out;
}

bool ReadCurve(Node const* bcurve, Nurbs& out) {
    Node const* nurbs = bcurve ? bcurve->Ptr("nurbs") : nullptr;
    if (nullptr == nurbs)
        return false;
    out.degree = nurbs->Int("degree");
    out.rational = 0 != nurbs->Int("rational");
    out.dim = nurbs->Int("vertex_dim") - (out.rational ? 1 : 0);
    out.knots = ExpandKnots(nurbs->Ptr("knot_mult"), nurbs->Ptr("knots"), nurbs->Int("n_knots"));
    Node const* vertices = nurbs->Ptr("bspline_vertices");
    int const n = nurbs->Int("n_vertices");
    if (nullptr == vertices || out.degree < 1 || out.degree > 32 || out.dim < 2 || out.dim > 3 || n < 2 || vertices->Count("vertices") < static_cast<size_t>(n * out.Stride()) ||
        out.knots.size() != static_cast<size_t>(n + out.degree + 1))
        return false;
    out.points.assign(vertices->Values("vertices"), vertices->Values("vertices") + n * out.Stride());
    return true;
}

//! The index of the knot span containing t, and the nonzero basis functions there.
int Basis(int p, std::vector<double> const& U, size_t n, double t, double* N) {
    int k = p;
    while (k + 1 < static_cast<int>(n) && t >= U[k + 1])
        ++k;
    double left[33], right[33];
    N[0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = t - U[k + 1 - j];
        right[j] = U[k + j] - t;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            double const denominator = right[r + 1] + left[j - r];
            double const temp = denominator != 0.0 ? N[r] / denominator : 0.0;
            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
    return k;
}

void Evaluate(Nurbs const& c, double t, double* out) {
    double N[33];
    int const k = Basis(c.degree, c.knots, c.Count(), std::clamp(t, c.Start(), c.End()), N);
    int const stride = c.Stride();
    double h[4] = {0, 0, 0, 0};
    for (int j = 0; j <= c.degree; ++j)
        for (int i = 0; i < stride; ++i)
            h[i] += N[j] * c.points[(k - c.degree + j) * stride + i];
    for (int i = 0; i < c.dim; ++i)
        out[i] = c.rational && h[c.dim] != 0.0 ? h[i] / h[c.dim] : h[i];
}

//! Insert a knot once into a sequence of control blocks, each a fixed number of doubles.
void InsertKnot(int p, std::vector<double>& U, std::vector<std::vector<double>>& P, double u) {
    int k = 0;
    for (int i = 0; i + 1 < static_cast<int>(U.size()); ++i)
        if (U[i] <= u)
            k = i;
    k = std::min(k, static_cast<int>(P.size()) - 1);
    std::vector<std::vector<double>> Q;
    for (int i = 0; i <= static_cast<int>(P.size()); ++i) {
        if (i <= k - p) {
            Q.push_back(P[i]);
        } else if (i >= k + 1) {
            Q.push_back(P[i - 1]);
        } else {
            double const a = (u - U[i]) / (U[i + p] - U[i]);
            std::vector<double> q(P[i].size());
            for (size_t j = 0; j < q.size(); ++j)
                q[j] = a * P[i][j] + (1.0 - a) * P[i - 1][j];
            Q.push_back(std::move(q));
        }
    }
    U.insert(U.begin() + k + 1, u);
    P = std::move(Q);
}

//! Make the knot vector clamped at the ends of its domain, as periodic data isn't.
bool Clamp(int p, std::vector<double>& U, std::vector<std::vector<double>>& P) {
    size_t const n = P.size();
    if (U.size() != n + p + 1)
        return false;
    double const a = U[p], b = U[n];
    double const tolerance = 1.0e-12 * std::max(1.0, b - a);
    auto multiplicity = [&](double u) { return std::count_if(U.begin(), U.end(), [&](double t) { return std::fabs(t - u) <= tolerance; }); };
    if (multiplicity(a) == p + 1 && multiplicity(b) == p + 1 && U.front() == a && U.back() == b)
        return true;
    while (multiplicity(a) < p)
        InsertKnot(p, U, P, a);
    while (multiplicity(b) < p)
        InsertKnot(p, U, P, b);
    size_t first = 0, last = 0;
    for (size_t i = 0; i < U.size(); ++i) {
        if (std::fabs(U[i] - a) <= tolerance && 0 == first)
            first = i;
        if (std::fabs(U[i] - b) <= tolerance)
            last = i;
    }
    if (first < 1)
        return true; // already clamped at the start
    std::vector<double> knots(1, a);
    knots.insert(knots.end(), U.begin() + first, U.begin() + last + 1);
    knots.push_back(b);
    size_t const count = knots.size() - p - 1;
    if (first - 1 + count > P.size())
        return false;
    std::vector<std::vector<double>> points(P.begin() + first - 1, P.begin() + first - 1 + count);
    U = std::move(knots);
    P = std::move(points);
    return true;
}

void UniqueKnots(std::vector<double> const& U, SmTArray<double>& knots, SmTArray<ULONG>& multiplicities) {
    double const span = U.back() - U.front();
    for (double u : U) {
        if (knots.GetSize() > 0 && std::fabs(u - knots[knots.GetSize() - 1]) <= 1.0e-12 * span) {
            multiplicities[multiplicities.GetSize() - 1]++;
        } else {
            knots.Add(u);
            multiplicities.Add(1);
        }
    }
}

//! A clamped 3d curve in SmKernel's form from homogeneous NURBS.
bool ToCurve(Nurbs c, Curve& out) {
    if (3 != c.dim)
        return false;
    int const stride = c.Stride();
    std::vector<std::vector<double>> blocks;
    for (size_t i = 0; i < c.Count(); ++i)
        blocks.emplace_back(c.points.begin() + i * stride, c.points.begin() + (i + 1) * stride);
    if (!Clamp(c.degree, c.knots, blocks))
        return false;
    out = Curve();
    out.degree = c.degree;
    out.knots = c.knots;
    for (auto const& b : blocks) {
        double const w = c.rational ? b[3] : 1.0;
        if (w <= 0.0)
            return false;
        out.poles.insert(out.poles.end(), {b[0] / w, b[1] / w, b[2] / w});
        if (c.rational)
            out.weights.push_back(w);
    }
    return true;
}

void Reverse(Curve& c) {
    size_t const n = c.poles.size() / 3;
    for (size_t i = 0; i < n / 2; ++i)
        for (int j = 0; j < 3; ++j)
            std::swap(c.poles[3 * i + j], c.poles[3 * (n - 1 - i) + j]);
    std::reverse(c.weights.begin(), c.weights.end());
    double const sum = c.knots.front() + c.knots.back();
    std::reverse(c.knots.begin(), c.knots.end());
    for (double& k : c.knots)
        k = sum - k;
}

Curve Line(Vec const& a, Vec const& b) {
    Curve c;
    c.degree = 1;
    c.poles = {a.x, a.y, a.z, b.x, b.y, b.z};
    double const length = Vec(b - a).Length();
    c.knots = {0, 0, length, length};
    return c;
}

//! The arc of c + a cos(t) X + b sin(t) Y for t from t0 up to t1, exactly as rational quadratics.
Curve ConicArc(Vec const& c, Vec const& X, Vec const& Y, double a, double b, double t0, double t1) {
    int const segments = std::max(1, static_cast<int>(std::ceil((t1 - t0) / (0.5 * kPi) - 1.0e-9)));
    double const dt = (t1 - t0) / segments, w = std::cos(0.5 * dt);
    auto at = [&](double t, double scale) { return c + (a * std::cos(t) * scale) * X + (b * std::sin(t) * scale) * Y; };
    Curve curve;
    curve.degree = 2;
    curve.knots = {t0, t0, t0};
    for (int i = 0; i < segments; ++i) {
        double const s = t0 + i * dt;
        Vec const p0 = at(s, 1.0), p1 = at(s + 0.5 * dt, 1.0 / w);
        if (0 == i) {
            curve.poles.insert(curve.poles.end(), {p0.x, p0.y, p0.z});
            curve.weights.push_back(1.0);
        }
        Vec const p2 = at(s + dt, 1.0);
        curve.poles.insert(curve.poles.end(), {p1.x, p1.y, p1.z, p2.x, p2.y, p2.z});
        curve.weights.insert(curve.weights.end(), {w, 1.0});
        double const knot = i + 1 == segments ? t1 : s + dt;
        curve.knots.insert(curve.knots.end(), i + 1 == segments ? 3 : 2, knot);
    }
    return curve;
}

//! The poles of the clamped cubic (or lower degree, for few points) through points at evenly
//! spaced parameters, with knots averaged from the parameters. Points may be of any dimension.
bool InterpolateEvenly(std::vector<std::vector<double>> const& points, int& degree, std::vector<double>& knots, std::vector<std::vector<double>>& poles) {
    int const n = static_cast<int>(points.size());
    if (n < 2)
        return false;
    degree = std::min(3, n - 1);
    knots.assign(degree + 1, 0.0);
    for (int j = 1; j < n - degree; ++j) {
        double sum = 0.0;
        for (int i = j; i < j + degree; ++i)
            sum += static_cast<double>(i) / (n - 1);
        knots.push_back(sum / degree);
    }
    knots.insert(knots.end(), degree + 1, 1.0);
    size_t const width = points[0].size();
    std::vector<std::vector<double>> A(n, std::vector<double>(n, 0.0));
    poles = points;
    for (int k = 0; k < n; ++k) {
        double N[33];
        int const span = Basis(degree, knots, n, static_cast<double>(k) / (n - 1), N);
        for (int j = 0; j <= degree; ++j)
            A[k][span - degree + j] = N[j];
    }
    for (int c = 0; c < n; ++c) {
        int pivot = c;
        for (int r = c + 1; r < n; ++r)
            if (std::fabs(A[r][c]) > std::fabs(A[pivot][c]))
                pivot = r;
        if (std::fabs(A[pivot][c]) < 1.0e-14)
            return false;
        std::swap(A[c], A[pivot]);
        std::swap(poles[c], poles[pivot]);
        for (int r = 0; r < n; ++r) {
            if (r == c || 0.0 == A[r][c])
                continue;
            double const f = A[r][c] / A[c][c];
            for (int k = c; k < n; ++k)
                A[r][k] -= f * A[c][k];
            for (size_t k = 0; k < width; ++k)
                poles[r][k] -= f * poles[c][k];
        }
    }
    for (int r = 0; r < n; ++r)
        for (size_t k = 0; k < width; ++k)
            poles[r][k] /= A[r][r];
    return true;
}

/*---------------------------------------------------------------------------------**//**
* Evaluation of Parasolid surfaces in their own parameters, for curves given on them.
+---------------+---------------+---------------+---------------+---------------+------*/
struct SurfaceNurbs {
    int pu = 1, pv = 1, nu = 0, nv = 0, stride = 3;
    bool rational = false;
    std::vector<double> U, V, points; //!< homogeneous, v varying fastest
};

bool ReadSurface(Node const* bsurface, SurfaceNurbs& s) {
    Node const* nurbs = bsurface ? bsurface->Ptr("nurbs") : nullptr;
    if (nullptr == nurbs)
        return false;
    s.pu = nurbs->Int("u_degree");
    s.pv = nurbs->Int("v_degree");
    s.nu = nurbs->Int("n_u_vertices");
    s.nv = nurbs->Int("n_v_vertices");
    s.rational = 0 != nurbs->Int("rational");
    s.stride = nurbs->Int("vertex_dim");
    s.U = ExpandKnots(nurbs->Ptr("u_knot_mult"), nurbs->Ptr("u_knots"), nurbs->Int("n_u_knots"));
    s.V = ExpandKnots(nurbs->Ptr("v_knot_mult"), nurbs->Ptr("v_knots"), nurbs->Int("n_v_knots"));
    Node const* vertices = nurbs->Ptr("bspline_vertices");
    size_t const count = static_cast<size_t>(s.nu) * s.nv * s.stride;
    if (nullptr == vertices || s.pu < 1 || s.pv < 1 || s.pu > 32 || s.pv > 32 || s.stride != (s.rational ? 4 : 3) || vertices->Count("vertices") < count ||
        s.U.size() != static_cast<size_t>(s.nu + s.pu + 1) || s.V.size() != static_cast<size_t>(s.nv + s.pv + 1))
        return false;
    s.points.assign(vertices->Values("vertices"), vertices->Values("vertices") + count);
    return true;
}

Vec EvaluateSurface(SurfaceNurbs const& s, double u, double v) {
    double Nu[33], Nv[33];
    int const ku = Basis(s.pu, s.U, s.nu, std::clamp(u, s.U[s.pu], s.U[s.nu]), Nu);
    int const kv = Basis(s.pv, s.V, s.nv, std::clamp(v, s.V[s.pv], s.V[s.nv]), Nv);
    double h[4] = {0, 0, 0, 0};
    for (int a = 0; a <= s.pu; ++a)
        for (int b = 0; b <= s.pv; ++b) {
            double const* p = &s.points[(static_cast<size_t>(ku - s.pu + a) * s.nv + (kv - s.pv + b)) * s.stride];
            for (int i = 0; i < s.stride; ++i)
                h[i] += Nu[a] * Nv[b] * p[i];
        }
    if (s.rational && h[3] != 0.0)
        return Vec(h[0] / h[3], h[1] / h[3], h[2] / h[3]);
    return Vec(h[0], h[1], h[2]);
}

struct Evaluator {
    std::map<Node const*, SurfaceNurbs> m_nurbs;

    SurfaceNurbs const* Nurbs(Node const& bsurface) {
        auto found = m_nurbs.find(&bsurface);
        if (found == m_nurbs.end()) {
            SurfaceNurbs s;
            if (!ReadSurface(&bsurface, s))
                s.nu = 0;
            found = m_nurbs.emplace(&bsurface, std::move(s)).first;
        }
        return found->second.nu > 0 ? &found->second : nullptr;
    }

    bool Point(Node const& surface, double u, double v, Vec& p, int depth = 0) {
        switch (surface.Type()) {
            case XT::PLANE: {
                Vec const X = V(surface, "x_axis"), Y = V(surface, "normal") * X;
                p = V(surface, "pvec") + u * X + v * Y;
                return true;
            }
            case XT::CYLINDER: {
                Vec const A = V(surface, "axis"), X = V(surface, "x_axis"), Y = A * X;
                p = V(surface, "pvec") + surface.Real("radius") * (std::cos(u) * X + std::sin(u) * Y) + v * A;
                return true;
            }
            case XT::B_SURFACE: {
                SurfaceNurbs const* s = Nurbs(surface);
                if (nullptr == s)
                    return false;
                p = EvaluateSurface(*s, u, v);
                return true;
            }
            case XT::OFFSET_SURF: {
                Node const* base = surface.Ptr("surface");
                Vec n;
                if (nullptr == base || depth > 4 || !Point(*base, u, v, p, depth + 1) || !NaturalNormal(*base, u, v, n, depth + 1))
                    return false;
                p = p + (surface.Real("offset") * Sense(*base)) * n;
                return true;
            }
            default:
                return false;
        }
    }

    bool NaturalNormal(Node const& surface, double u, double v, Vec& n, int depth = 0) {
        double const h = 1.0e-6;
        Vec p, pu, pv;
        if (!Point(surface, u, v, p, depth) || !Point(surface, u + h, v, pu, depth) || !Point(surface, u, v + h, pv, depth))
            return false;
        n = Unit(Vec(pu - p) * Vec(pv - p));
        if (n.Length() > 0.5)
            return true;
        if (!Point(surface, u - h, v, pu, depth) || !Point(surface, u, v - h, pv, depth))
            return false;
        n = Unit(Vec(p - pu) * Vec(p - pv));
        return n.Length() > 0.5;
    }
};

//! Points along a curve given in the parameters of a surface, from t0 to t1.
bool LiftSpCurve(Node const& sp, double t0, double t1, Evaluator& evaluator, std::vector<Vec>& points) {
    Nurbs uv;
    Node const* surface = sp.Ptr("surface");
    if (nullptr == surface || !ReadCurve(sp.Ptr("b_curve"), uv) || 2 != uv.dim)
        return false;
    auto at = [&](double t, Vec& p) {
        double q[3];
        Evaluate(uv, t, q);
        return evaluator.Point(*surface, q[0], q[1], p);
    };
    // Start evenly, then split spans whose middle strays from their chord, as the surface may
    // bend much more than the curve's own parameter space shows.
    int const initial = std::clamp(static_cast<int>(8 * uv.Count()), 16, 400);
    std::vector<std::pair<double, Vec>> samples;
    for (int i = 0; i <= initial; ++i) {
        double const t = t0 + (t1 - t0) * i / initial;
        Vec p;
        if (!at(t, p))
            return false;
        samples.emplace_back(t, p);
    }
    for (size_t i = 0; i + 1 < samples.size() && samples.size() < 4000;) {
        double const t = 0.5 * (samples[i].first + samples[i + 1].first);
        Vec p;
        if (!at(t, p))
            return false;
        Vec const a = samples[i].second, b = samples[i + 1].second;
        double const chord = Vec(b - a).Length();
        if (Vec(p - 0.5 * (a + b)).Length() > 1.0e-3 * chord && chord > 0.0) {
            samples.insert(samples.begin() + i + 1, std::make_pair(t, p));
            continue;
        }
        ++i;
    }
    points.clear();
    for (auto const& sample : samples)
        points.push_back(sample.second);
    return true;
}

SmBSplineCurve* Interpolate(std::vector<Vec> const& points) {
    SmTArray<SmPoint3d> through;
    for (Vec const& p : points) {
        if (0 == through.GetSize() || Vec(p - through[through.GetSize() - 1]).Length() > 1.0e-12)
            through.Add(p);
    }
    if (through.GetSize() < 2)
        return nullptr;
    SmBSplineCurve* curve = nullptr;
    if (through.GetSize() == 2) {
        if (SM_SUCCESS != SmBSplineCurve::CreateLineSegment(Context(), 3, through[0], through[1], curve))
            return nullptr;
        return curve;
    }
    SmTArray<SmVector3d> none;
    ULONG const degree = through.GetSize() > 3 ? 3 : 2;
    if (SM_SUCCESS != SmBSplineCurve::CreateInterpolatingCurve(Context(), SM_CP_CHORDLENGTH, 3, degree, through, none, nullptr, FALSE, curve))
        return nullptr;
    return curve;
}

//! Follow an owner's chain of attributes for an integer field of the named definition.
bool FindIntAttribute(Node const& owner, char const* name, int& value) {
    int guard = 0;
    for (Node const* a = owner.Ptr("attributes_features"); nullptr != a && guard < 1000; a = a->Ptr("next"), ++guard) {
        if (!a->Is(XT::ATTRIBUTE))
            continue;
        Node const* definition = a->Ptr("definition");
        Node const* identifier = definition ? definition->Ptr("identifier") : nullptr;
        if (nullptr == identifier)
            continue;
        std::string id;
        for (size_t i = 0; i < identifier->Count("string"); ++i)
            id += static_cast<char>(identifier->Int("string", i));
        if (id != name)
            continue;
        Node const* values = a->Ptr("fields");
        if (values && values->Is(XT::INT_VALUES) && values->Count("values") > 0) {
            value = values->Int("values");
            return true;
        }
    }
    return false;
}

/*---------------------------------------------------------------------------------**//**
* Scale the lengths in a model, leaving directions, angles and curve parameters.
+---------------+---------------+---------------+---------------+---------------+------*/
void ScaleValues(Node const& node, char const* field, double k, size_t stride = 1, size_t count = 1) {
    double* values = const_cast<double*>(node.Values(field));
    size_t const n = node.Count(field);
    for (size_t i = 0; values && i < n; ++i)
        if (i % stride < count)
            values[i] *= k;
}

void ScaleModel(XT::Model& model, double k) {
    std::set<Node const*> scaledVertices;
    auto scaleNurbs = [&](Node const* nurbs, size_t count) {
        Node const* vertices = nurbs ? nurbs->Ptr("bspline_vertices") : nullptr;
        if (nullptr == vertices || 0 == count || !scaledVertices.insert(vertices).second)
            return;
        ScaleValues(*vertices, "vertices", k, nurbs->Int("vertex_dim"), count);
    };
    for (auto& [index, node] : model.m_nodes) {
        switch (node.Type()) {
            case XT::POINT: case XT::LINE: case XT::PLANE: ScaleValues(node, "pvec", k); break;
            case XT::CIRCLE: ScaleValues(node, "centre", k); ScaleValues(node, "radius", k); break;
            case XT::ELLIPSE: ScaleValues(node, "centre", k); ScaleValues(node, "major_radius", k); ScaleValues(node, "minor_radius", k); break;
            case XT::CYLINDER: case XT::CONE: ScaleValues(node, "pvec", k); ScaleValues(node, "radius", k); break;
            case XT::SPHERE: ScaleValues(node, "centre", k); ScaleValues(node, "radius", k); break;
            case XT::TORUS: ScaleValues(node, "centre", k); ScaleValues(node, "major_radius", k); ScaleValues(node, "minor_radius", k); break;
            case XT::TRIMMED_CURVE: ScaleValues(node, "point_1", k); ScaleValues(node, "point_2", k); break;
            case XT::OFFSET_SURF: ScaleValues(node, "offset", k); break;
            case XT::SWEPT_SURF: ScaleValues(node, "sweep", k); break;
            case XT::BLENDED_EDGE: ScaleValues(node, "range", k); break;
            case XT::CHART: ScaleValues(node, "hvec", k); break;
            case XT::NURBS_SURF: scaleNurbs(&node, 3); break;
            case XT::NURBS_CURVE:
                if (3 == node.Int("vertex_dim") - (node.Int("rational") ? 1 : 0))
                    scaleNurbs(&node, 3);
                break;
            default: break;
        }
    }
    // Curves in surface parameters: lengths on planes, and along the axis of cylinders.
    for (auto& [index, node] : model.m_nodes) {
        Node const* surface = node.Is(XT::SP_CURVE) ? node.Ptr("surface") : nullptr;
        Node const* bcurve = node.Is(XT::SP_CURVE) ? node.Ptr("b_curve") : nullptr;
        Node const* nurbs = bcurve ? bcurve->Ptr("nurbs") : nullptr;
        if (nullptr == surface || nullptr == nurbs)
            continue;
        Node const* vertices = nurbs->Ptr("bspline_vertices");
        if (nullptr == vertices || !scaledVertices.insert(vertices).second)
            continue;
        size_t const stride = nurbs->Int("vertex_dim");
        if (surface->Is(XT::PLANE))
            ScaleValues(*vertices, "vertices", k, stride, 2);
        else if (surface->Is(XT::CYLINDER)) {
            double* values = const_cast<double*>(vertices->Values("vertices"));
            for (size_t i = 1; values && i < vertices->Count("vertices"); i += stride)
                values[i] *= k;
        }
    }
}

//! A power of two that brings the body to a few tens of units across, where SMLib's tolerances
//! and healing work well. Bodies far smaller can take SMLib seconds to heal.
double NormalizingScale(XT::Model const& model) {
    double low[3] = {SM_BIG_DOUBLE, SM_BIG_DOUBLE, SM_BIG_DOUBLE}, high[3] = {-SM_BIG_DOUBLE, -SM_BIG_DOUBLE, -SM_BIG_DOUBLE};
    auto extend = [&](Vec const& p, double r) {
        for (int i = 0; i < 3; ++i) {
            low[i] = std::min(low[i], p[i] - r);
            high[i] = std::max(high[i], p[i] + r);
        }
    };
    for (auto const& [index, node] : model.m_nodes) {
        switch (node.Type()) {
            case XT::POINT: extend(V(node, "pvec"), 0.0); break;
            case XT::CIRCLE: extend(V(node, "centre"), node.Real("radius")); break;
            case XT::ELLIPSE: extend(V(node, "centre"), node.Real("major_radius")); break;
            case XT::SPHERE: extend(V(node, "centre"), node.Real("radius")); break;
            case XT::TORUS: extend(V(node, "centre"), node.Real("major_radius") + node.Real("minor_radius")); break;
            case XT::NURBS_SURF: case XT::NURBS_CURVE: {
                bool const rational = 0 != node.Int("rational");
                int const stride = node.Int("vertex_dim");
                Node const* vertices = node.Ptr("bspline_vertices");
                if (nullptr == vertices || stride - (rational ? 1 : 0) != 3)
                    break;
                double const* v = vertices->Values("vertices");
                for (size_t i = 0; v && i + stride <= vertices->Count("vertices"); i += stride) {
                    double const w = rational ? v[i + 3] : 1.0;
                    if (w > 0.0)
                        extend(Vec(v[i] / w, v[i + 1] / w, v[i + 2] / w), 0.0);
                }
                break;
            }
            default: break;
        }
    }
    if (low[0] > high[0])
        return 1.0;
    double const diagonal = std::sqrt((high[0] - low[0]) * (high[0] - low[0]) + (high[1] - low[1]) * (high[1] - low[1]) + (high[2] - low[2]) * (high[2] - low[2]));
    if (!(diagonal > 0.0) || !std::isfinite(diagonal))
        return 1.0;
    return std::ldexp(1.0, std::clamp(static_cast<int>(std::lround(std::log2(32.0 / diagonal))), -30, 30));
}

/*---------------------------------------------------------------------------------**//**
* The conversion of one body.
+---------------+---------------+---------------+---------------+---------------+------*/
struct EdgeData {
    Node const* m_start = nullptr; //!< Parasolid vertices in the edge's direction, null for a ring
    Node const* m_end = nullptr;
    Vec m_startPoint, m_endPoint;
    bool m_closed = false;
    SmBSplineCurve* m_curve = nullptr;
    bool m_limitsFromPoints = false;
    std::vector<Vec> m_samples; //!< along the edge, start to end
    SmEdge* m_edge = nullptr;
    SmVertex* m_ringVertex = nullptr;

    ~EdgeData() { delete m_curve; }
};

struct Converter {
    XT::Model const& m_model;
    std::string* m_log;
    Evaluator m_evaluator;
    std::map<Node const*, std::unique_ptr<EdgeData>> m_edges;
    std::map<Node const*, SmVertex*> m_vertices;
    std::map<Node const*, std::unique_ptr<SmSurface>> m_projectors;
    double m_size = 1.0;

    Converter(XT::Model const& model, std::string* log) : m_model(model), m_log(log) { }

    void Log(std::string const& message) {
        if (m_log)
            *m_log += message + "\n";
    }

    static Node const* FinStart(Node const& fin) {
        Node const* back = fin.Ptr("backward");
        if (back && back != &fin && back->Ptr("vertex"))
            return back->Ptr("vertex");
        Node const* other = fin.Ptr("other");
        return other ? other->Ptr("vertex") : nullptr;
    }

    static Vec PointOf(Node const* vertex) {
        Node const* point = vertex ? vertex->Ptr("point") : nullptr;
        return point ? V(*point, "pvec") : Vec(0, 0, 0);
    }

    /*-----------------------------------------------------------------------------------
    * Edge curves
    +----------------------------------------------------------------------------------*/
    //! An exact conic arc of the edge, given the conic's frame with Y set for the edge direction.
    bool ConicEdge(EdgeData& e, Vec const& c, Vec const& X, Vec const& Y, double a, double b, Curve& out) {
        auto angle = [&](Vec const& p) { Vec const w = p - c; return std::atan2(w.Dot(Y) / b, w.Dot(X) / a); };
        double t0 = 0.0, t1 = 2.0 * kPi;
        if (!e.m_start) {
            e.m_startPoint = e.m_endPoint = c + a * X;
        } else {
            t0 = angle(e.m_startPoint);
            t1 = angle(e.m_endPoint);
            if (e.m_closed)
                t1 = t0 + 2.0 * kPi;
            while (t1 <= t0 + 1.0e-12)
                t1 += 2.0 * kPi;
        }
        out = ConicArc(c, X, Y, a, b, t0, t1);
        return true;
    }

    //! The curve of a line or conic between the edge's ends, going the way its sense says.
    bool BasicEdge(EdgeData& e, Node const& curve, double sense, Curve& out) {
        switch (curve.Type()) {
            case XT::LINE:
                if (!e.m_start)
                    return false;
                out = Line(e.m_startPoint, e.m_endPoint);
                return true;
            case XT::CIRCLE: case XT::ELLIPSE: {
                bool const circle = curve.Is(XT::CIRCLE);
                double const a = circle ? curve.Real("radius") : curve.Real("major_radius"), b = circle ? a : curve.Real("minor_radius");
                if (a <= 0.0 || b <= 0.0)
                    return false;
                Vec const X = Unit(V(curve, "x_axis")), Y = Unit(V(curve, "normal") * X) * sense;
                return ConicEdge(e, V(curve, "centre"), X, Y, a, b, out);
            }
            default:
                return false;
        }
    }

    //! A trimmed curve runs from point_1 to point_2 along its owner, the edge or one of its fins;
    //! `ownerAlong` says whether the owner runs along the edge.
    bool TrimmedEdge(EdgeData& e, Node const& trimmed, bool ownerAlong, SmBSplineCurve*& result) {
        Node const* basis = trimmed.Ptr("basis_curve");
        if (nullptr == basis)
            return false;
        Vec const p1 = V(trimmed, "point_1"), p2 = V(trimmed, "point_2");
        double const t1 = trimmed.Real("parm_1"), t2 = trimmed.Real("parm_2");
        bool forward = true;
        if (e.m_start && Vec(p1 - p2).Length() > 1.0e-9 * m_size)
            forward = Vec(e.m_startPoint - p1).Length() <= Vec(e.m_startPoint - p2).Length();
        else
            forward = ownerAlong;
        if (!e.m_start) {
            e.m_startPoint = forward ? p1 : p2;
            e.m_endPoint = e.m_startPoint;
        }
        Curve curve;
        switch (basis->Type()) {
            case XT::LINE:
                curve = Line(e.m_startPoint, e.m_endPoint);
                break;
            case XT::CIRCLE: case XT::ELLIPSE: {
                bool const circle = basis->Is(XT::CIRCLE);
                double const a = circle ? basis->Real("radius") : basis->Real("major_radius"), b = circle ? a : basis->Real("minor_radius");
                Vec const X = Unit(V(*basis, "x_axis")), Y = Unit(V(*basis, "normal") * X);
                double end = t2;
                while (end <= t1 + 1.0e-12)
                    end += 2.0 * kPi;
                curve = ConicArc(V(*basis, "centre"), X, Y, a, b, t1, end);
                if (!forward)
                    Reverse(curve);
                break;
            }
            case XT::SP_CURVE: {
                std::vector<Vec> points;
                if (!LiftSpCurve(*basis, forward ? t1 : t2, forward ? t2 : t1, m_evaluator, points))
                    return false;
                points.front() = e.m_startPoint;
                points.back() = e.m_endPoint;
                result = Interpolate(points);
                return nullptr != result;
            }
            case XT::B_CURVE: {
                Nurbs nurbs;
                if (!ReadCurve(basis, nurbs) || !ToCurve(nurbs, curve))
                    return false;
                if (!forward)
                    Reverse(curve);
                e.m_limitsFromPoints = !e.m_closed;
                break;
            }
            default:
                return false;
        }
        result = ToSmCurve(curve);
        return nullptr != result;
    }

    //! Move a point onto a surface, near where it is.
    bool Project(Node const& surface, Vec& p) {
        switch (surface.Type()) {
            case XT::PLANE: {
                Vec const n = Unit(V(surface, "normal"));
                p = p - Vec(p - V(surface, "pvec")).Dot(n) * n;
                return true;
            }
            case XT::CYLINDER: {
                Vec const o = V(surface, "pvec"), A = Unit(V(surface, "axis")), w = p - o;
                Vec const radial = Unit(w - w.Dot(A) * A);
                if (radial.Length() < 0.5)
                    return false;
                p = o + w.Dot(A) * A + surface.Real("radius") * radial;
                return true;
            }
            case XT::SPHERE: {
                Vec const c = V(surface, "centre"), radial = Unit(p - c);
                if (radial.Length() < 0.5)
                    return false;
                p = c + surface.Real("radius") * radial;
                return true;
            }
            case XT::TORUS: {
                Vec const c = V(surface, "centre"), A = Unit(V(surface, "axis")), w = p - c;
                Vec const radial = Unit(w - w.Dot(A) * A);
                Vec const tube = c + surface.Real("major_radius") * radial;
                Vec const out = Unit(p - tube);
                if (radial.Length() < 0.5 || out.Length() < 0.5)
                    return false;
                p = tube + surface.Real("minor_radius") * out;
                return true;
            }
            case XT::B_SURFACE: case XT::OFFSET_SURF: {
                auto& projector = m_projectors[&surface];
                if (!projector && surface.Is(XT::B_SURFACE))
                    projector.reset(BSplineSurface(surface));
                if (!projector && surface.Is(XT::OFFSET_SURF)) {
                    Node const* base = surface.Ptr("surface");
                    SmSurface* baseSurface = base && base->Is(XT::B_SURFACE) ? BSplineSurface(*base) : nullptr;
                    if (baseSurface)
                        projector.reset(new (Context()) SmOffsetSurface(surface.Real("offset") * Sense(*base), *baseSurface, TRUE));
                }
                if (!projector)
                    return false;
                SmBoolean success = FALSE, multivalued = FALSE;
                SmPoint2d uv;
                double gap = 0.0;
                SmPoint3d q;
                if (SM_SUCCESS != projector->DropPoint(p, projector->GetNaturalUVDomain(), nullptr, success, uv, gap, multivalued) || !success ||
                    SM_SUCCESS != projector->EvaluatePoint(uv, q))
                    return false;
                p = q;
                return true;
            }
            default:
                return false;
        }
    }

    //! A curve along an intersection between the edge's ends, through its chart's points refined
    //! onto both surfaces.
    bool IntersectionEdge(EdgeData& e, Node const& intersection, SmBSplineCurve*& result) {
        Node const* chart = intersection.Ptr("chart");
        Node const* a = intersection.Ptr("surface", 0);
        Node const* b = intersection.Ptr("surface", 1);
        if (nullptr == chart || nullptr == a || nullptr == b)
            return false;
        size_t const count = chart->Count("hvec") / 3;
        double const* h = chart->Values("hvec");
        std::vector<Vec> chartPoints;
        for (size_t i = 0; i < count; ++i)
            chartPoints.push_back(Vec(h[3 * i], h[3 * i + 1], h[3 * i + 2]));
        if (chartPoints.size() < 2)
            return false;
        bool const ring = !e.m_start;
        if (ring && Vec(chartPoints.back() - chartPoints.front()).Length() > 1.0e-9 * m_size)
            chartPoints.push_back(chartPoints.front());
        if (Sense(intersection) < 0.0)
            std::reverse(chartPoints.begin(), chartPoints.end());

        size_t i0 = 0, i1 = chartPoints.size() - 1;
        if (!ring) {
            auto nearest = [&](Vec const& p) {
                size_t best = 0;
                for (size_t i = 1; i < chartPoints.size(); ++i)
                    if (Vec(chartPoints[i] - p).Length() < Vec(chartPoints[best] - p).Length())
                        best = i;
                return best;
            };
            i0 = nearest(e.m_startPoint);
            i1 = nearest(e.m_endPoint);
            if (i0 > i1)
                std::swap(i0, i1);
        }
        int const steps = 8;
        std::vector<Vec> path;
        for (size_t i = i0; i <= i1; ++i) {
            for (int j = 0; j < (i < i1 ? steps : 1); ++j) {
                Vec p = chartPoints[i] + (static_cast<double>(j) / steps) * Vec((i < i1 ? chartPoints[i + 1] : chartPoints[i]) - chartPoints[i]);
                for (int k = 0; k < 8; ++k) {
                    Vec const before = p;
                    if (!Project(*a, p) || !Project(*b, p))
                        return false;
                    if (Vec(p - before).Length() < 1.0e-12 * m_size)
                        break;
                }
                path.push_back(p);
            }
        }
        if (ring) {
            path.back() = path.front();
            e.m_startPoint = e.m_endPoint = path.front();
        } else {
            if (Vec(path.front() - e.m_startPoint).Length() > Vec(path.back() - e.m_startPoint).Length())
                std::reverse(path.begin(), path.end());
            path.front() = e.m_startPoint;
            path.back() = e.m_endPoint;
        }
        result = Interpolate(path);
        return nullptr != result;
    }

    //! A tolerant edge has no curve of its own; use one of its fins' curves.
    bool FinCurveEdge(EdgeData& e, Node const& edge, SmBSplineCurve*& result) {
        Node const* fin = edge.Ptr("halfedge");
        for (int i = 0; i < 2 && fin; ++i, fin = fin->Ptr("other")) {
            Node const* curve = fin->Ptr("curve");
            if (curve && curve->Is(XT::TRIMMED_CURVE) && TrimmedEdge(e, *curve, '-' != fin->Char("sense"), result))
                return true;
        }
        return false;
    }

    bool MakeEdge(Node const& edge, EdgeData& e) {
        Node const* fin = edge.Ptr("halfedge");
        if (nullptr == fin)
            return false;
        Node const* finEnd = fin->Ptr("vertex");
        Node const* finStart = FinStart(*fin);
        bool const along = '-' != fin->Char("sense");
        e.m_start = along ? finStart : finEnd;
        e.m_end = along ? finEnd : finStart;
        if (!e.m_start || !e.m_end)
            e.m_start = e.m_end = nullptr;
        e.m_closed = nullptr == e.m_start || e.m_start == e.m_end;
        e.m_startPoint = PointOf(e.m_start);
        e.m_endPoint = PointOf(e.m_end);

        Node const* curve = edge.Ptr("curve");
        SmBSplineCurve* result = nullptr;
        bool ok = false;
        Curve exact;
        if (nullptr == curve) {
            ok = FinCurveEdge(e, edge, result);
        } else {
            switch (curve->Type()) {
                case XT::LINE: case XT::CIRCLE: case XT::ELLIPSE:
                    ok = BasicEdge(e, *curve, Sense(*curve), exact) && nullptr != (result = ToSmCurve(exact));
                    break;
                case XT::TRIMMED_CURVE:
                    ok = TrimmedEdge(e, *curve, true, result);
                    break;
                case XT::B_CURVE: {
                    Nurbs nurbs;
                    ok = ReadCurve(curve, nurbs) && ToCurve(nurbs, exact);
                    if (ok && Sense(*curve) < 0.0)
                        Reverse(exact);
                    if (ok && !e.m_start) {
                        double first[3];
                        Evaluate(nurbs, nurbs.Start(), first);
                        e.m_startPoint = e.m_endPoint = Vec(first[0], first[1], first[2]);
                    }
                    e.m_limitsFromPoints = ok && !e.m_closed;
                    ok = ok && nullptr != (result = ToSmCurve(exact));
                    break;
                }
                case XT::INTERSECTION:
                    ok = IntersectionEdge(e, *curve, result);
                    break;
                case XT::SP_CURVE: {
                    Nurbs uv;
                    std::vector<Vec> points;
                    ok = ReadCurve(curve->Ptr("b_curve"), uv) && LiftSpCurve(*curve, uv.Start(), uv.End(), m_evaluator, points);
                    if (ok && Sense(*curve) < 0.0)
                        std::reverse(points.begin(), points.end());
                    if (ok && e.m_start) {
                        points.front() = e.m_startPoint;
                        points.back() = e.m_endPoint;
                    }
                    ok = ok && nullptr != (result = Interpolate(points));
                    break;
                }
                default:
                    break;
            }
        }
        if (!ok || nullptr == result) {
            delete result;
            Log(std::string("edge ") + std::to_string(edge.Int("node_id")) + ": unsupported curve " + (curve ? curve->Name() : "(none)"));
            return false;
        }
        e.m_curve = result;

        SmExtent1d interval = result->GetNaturalInterval();
        if (e.m_limitsFromPoints) {
            double t0, t1;
            if (!ParameterOf(*result, e.m_startPoint, t0) || !ParameterOf(*result, e.m_endPoint, t1) || t1 <= t0) {
                Log("edge " + std::to_string(edge.Int("node_id")) + ": vertices not on its curve");
                return false;
            }
            interval = SmExtent1d(t0, t1);
        }
        int const samples = 32;
        for (int i = 0; i <= samples; ++i) {
            SmPoint3d p;
            result->EvaluatePoint(interval.GetMin() + interval.GetLength() * i / samples, p);
            e.m_samples.push_back(p);
        }
        return true;
    }

    static bool ParameterOf(SmCurve const& curve, Vec const& p, double& t) {
        SmBoolean success = FALSE;
        double distance = 0.0;
        if (SM_SUCCESS != curve.DropPoint(curve.GetNaturalInterval(), p, nullptr, SM_BIG_DOUBLE, nullptr, success, t, distance) || !success)
            return false;
        return true;
    }

    EdgeData* Edge(Node const& edge) {
        auto found = m_edges.find(&edge);
        if (found != m_edges.end())
            return found->second.get();
        auto data = std::make_unique<EdgeData>();
        if (!MakeEdge(edge, *data))
            data.reset();
        return (m_edges[&edge] = std::move(data)).get();
    }

    /*-----------------------------------------------------------------------------------
    * Face surfaces
    +----------------------------------------------------------------------------------*/
    struct Boundary {
        std::vector<std::vector<Vec>> m_loops; //!< points around each loop in its direction
        std::vector<Node const*> m_loopNodes;
        std::vector<Vec> m_hints;              //!< points just inside the face
    };

    static std::vector<Node const*> Fins(Node const& loop) {
        std::vector<Node const*> fins;
        Node const* first = loop.Ptr("halfedge");
        for (Node const* fin = first; fin && fins.size() < 100000; fin = fin->Ptr("forward")) {
            fins.push_back(fin);
            if (fin->Ptr("forward") == first)
                break;
        }
        return fins;
    }

    static std::vector<Node const*> Loops(Node const& face) {
        std::vector<Node const*> loops;
        for (Node const* loop = face.Ptr("loop"); loop && loops.size() < 100000; loop = loop->Ptr("next"))
            loops.push_back(loop);
        return loops;
    }

    //! The Parasolid face normal direction at a point near the face, for analytic surfaces.
    static bool AnalyticNormal(Node const& surface, Vec const& p, Vec& n) {
        switch (surface.Type()) {
            case XT::PLANE: n = Unit(V(surface, "normal")); return true;
            case XT::CYLINDER: case XT::CONE: {
                Vec const A = Unit(V(surface, "axis")), w = p - V(surface, "pvec");
                n = Unit(w - w.Dot(A) * A);
                return n.Length() > 0.5;
            }
            case XT::SPHERE: n = Unit(p - V(surface, "centre")); return n.Length() > 0.5;
            case XT::TORUS: {
                Vec const A = Unit(V(surface, "axis")), w = p - V(surface, "centre");
                Vec const radial = Unit(w - w.Dot(A) * A);
                n = Unit(w - surface.Real("major_radius") * radial);
                return n.Length() > 0.5;
            }
            default: return false;
        }
    }

    bool CollectBoundary(Node const& face, Node const& surface, double faceSense, Boundary& boundary) {
        for (Node const* loop : Loops(face)) {
            std::vector<Vec> points;
            for (Node const* fin : Fins(*loop)) {
                Node const* edge = fin->Ptr("edge");
                EdgeData* e = edge ? Edge(*edge) : nullptr;
                if (nullptr == e)
                    continue;
                std::vector<Vec> samples = e->m_samples;
                if ('-' == fin->Char("sense"))
                    std::reverse(samples.begin(), samples.end());
                points.insert(points.end(), samples.begin(), samples.end());
                size_t const mid = samples.size() / 2;
                if (samples.size() < 3)
                    continue;
                Vec const tangent = Unit(samples[mid + 1] - samples[mid - 1]);
                Vec normal;
                if (!AnalyticNormal(surface, samples[mid], normal))
                    continue;
                Vec const left = Unit((faceSense * normal) * tangent);
                double const step = std::max(1.0e-9, 1.0e-3 * Vec(samples.back() - samples.front()).Length() + 1.0e-4 * Vec(samples[mid] - samples.front()).Length());
                boundary.m_hints.push_back(samples[mid] + step * left);
            }
            if (!points.empty()) {
                boundary.m_loops.push_back(std::move(points));
                boundary.m_loopNodes.push_back(loop);
            }
        }
        return true;
    }

    struct Range {
        double m_start = 0.0; //!< radians
        double m_span = 2.0 * kPi;
    };

    //! The angles a face covers about an axis, from angles on its boundary and inside it. Loops
    //! that wind about the axis make the range a full turn.
    static Range AngleRange(std::vector<double> angles, std::vector<double> const& hints, bool winds, double seam) {
        Range range;
        range.m_start = seam;
        if (winds || angles.size() < 2)
            return range;
        for (double& a : angles)
            a = Wrap2Pi(a);
        std::sort(angles.begin(), angles.end());
        double bestGap = 0.0, bestEnd = 0.0;
        for (size_t i = 0; i < angles.size(); ++i) {
            double const from = angles[i], to = i + 1 < angles.size() ? angles[i + 1] : angles[0] + 2.0 * kPi;
            double const gap = to - from;
            if (gap <= bestGap)
                continue;
            bool const covered = std::any_of(hints.begin(), hints.end(), [&](double h) {
                double const d = Wrap2Pi(h - from);
                return d > 0.0 && d < gap;
            });
            if (!covered) {
                bestGap = gap;
                bestEnd = to;
            }
        }
        if (bestGap < 1.0e-6)
            return range;
        double const margin = std::min(1.0e-3, 0.25 * bestGap);
        range.m_start = bestEnd - margin;
        range.m_span = std::min(2.0 * kPi, 2.0 * kPi - bestGap + 2.0 * margin);
        return range;
    }

    //! The total turn of a loop about an axis, in turns.
    static int Winding(std::vector<Vec> const& loop, Vec const& origin, Vec const& X, Vec const& Y) {
        double total = 0.0;
        for (size_t i = 0; i < loop.size(); ++i) {
            Vec const a = loop[i] - origin, b = loop[(i + 1) % loop.size()] - origin;
            total += WrapPi(std::atan2(b.Dot(Y), b.Dot(X)) - std::atan2(a.Dot(Y), a.Dot(X)));
        }
        return static_cast<int>(std::lround(total / (2.0 * kPi)));
    }

    //! For surfaces of revolution: the angle range about the axis, and whether the face reaches
    //! the axis above (+1) or below (-1) its boundary, given the face normal points away from the
    //! axis if faceSense is positive.
    struct Revolution {
        Range m_range;
        bool m_toTop = false, m_toBottom = false;
    };

    Revolution AboutAxis(Boundary const& b, Vec const& origin, Vec const& A, Vec const& X, double faceSense) {
        Vec const Y = A * X;
        auto angle = [&](Vec const& p) { Vec const w = p - origin; return std::atan2(w.Dot(Y), w.Dot(X)); };
        std::vector<double> angles, hints;
        bool winds = false;
        double topHeight = -SM_BIG_DOUBLE, bottomHeight = SM_BIG_DOUBLE;
        int topTurn = 0, bottomTurn = 0;
        for (auto const& loop : b.m_loops) {
            for (Vec const& p : loop)
                angles.push_back(angle(p));
            int const turn = Winding(loop, origin, X, Y);
            if (0 == turn)
                continue;
            winds = true;
            double height = 0.0;
            for (Vec const& p : loop)
                height += Vec(p - origin).Dot(A);
            height /= loop.size();
            if (height > topHeight)
                topHeight = height, topTurn = turn;
            if (height < bottomHeight)
                bottomHeight = height, bottomTurn = turn;
        }
        for (Vec const& h : b.m_hints)
            hints.push_back(angle(h));
        Revolution r;
        r.m_range = AngleRange(angles, hints, winds, angles.empty() ? 0.0 : angles.front());
        // A loop going counterclockwise about the axis seen from above has the face above it
        // when the face normal points away from the axis.
        r.m_toTop = winds && topTurn * faceSense > 0.0;
        r.m_toBottom = winds && bottomTurn * faceSense < 0.0;
        return r;
    }

    static void Extent(Boundary const& b, Vec const& origin, Vec const& axis, double& low, double& high) {
        low = SM_BIG_DOUBLE;
        high = -SM_BIG_DOUBLE;
        for (auto const& loop : b.m_loops)
            for (Vec const& p : loop) {
                double const d = Vec(p - origin).Dot(axis);
                low = std::min(low, d);
                high = std::max(high, d);
            }
        for (Vec const& p : b.m_hints) {
            double const d = Vec(p - origin).Dot(axis);
            low = std::min(low, d);
            high = std::max(high, d);
        }
    }

    static SmAxis2Placement Rotated(Vec const& origin, Vec const& A, Vec const& X0, double angle) {
        Vec const Y0 = A * X0;
        Vec const X = Unit(std::cos(angle) * X0 + std::sin(angle) * Y0);
        return SmAxis2Placement(origin, X, A * X);
    }

    SmSurface* PlaneSurface(Node const& s, Boundary const& b) {
        Vec const o = V(s, "pvec"), N = Unit(V(s, "normal")), X = Unit(V(s, "x_axis")), Y = N * X;
        double u0, u1, v0, v1;
        Extent(b, o, X, u0, u1);
        Extent(b, o, Y, v0, v1);
        if (u0 > u1)
            return nullptr;
        double const margin = 1.0e-3 * std::max(u1 - u0, v1 - v0) + 1.0e-9 * m_size;
        SmPlane* plane = nullptr;
        if (SM_SUCCESS != SmPlane::CreateCanonical(Context(), SmAxis2Placement(o, X, Y), plane) || nullptr == plane)
            return nullptr;
        if (SM_SUCCESS != plane->AdjustSTEPUVDomain(SmExtent2d(u0 - margin, v0 - margin, u1 + margin, v1 + margin))) {
            delete plane;
            return nullptr;
        }
        return plane;
    }

    SmSurface* CylinderSurface(Node const& s, Boundary const& b, double faceSense) {
        Vec const o = V(s, "pvec"), A = Unit(V(s, "axis")), X = Unit(V(s, "x_axis"));
        Revolution const r = AboutAxis(b, o, A, X, faceSense);
        double v0, v1;
        Extent(b, o, A, v0, v1);
        if (v0 >= v1 || r.m_toTop || r.m_toBottom)
            return nullptr;
        double const margin = 1.0e-3 * (v1 - v0);
        SmCylinder* cylinder = nullptr;
        if (SM_SUCCESS != SmCylinder::CreateCanonical(Context(), Rotated(o, A, X, r.m_range.m_start), s.Real("radius"), cylinder) || nullptr == cylinder)
            return nullptr;
        if (SM_SUCCESS != cylinder->AdjustSTEPUVDomain(SmExtent2d(0.0, v0 - margin, r.m_range.m_span * kDegrees, v1 + margin))) {
            delete cylinder;
            return nullptr;
        }
        return cylinder;
    }

    SmSurface* ConeSurface(Node const& s, Boundary const& b, double faceSense) {
        Vec const o = V(s, "pvec"), A = Unit(V(s, "axis")), X = Unit(V(s, "x_axis"));
        double const radius = s.Real("radius"), tangent = s.Real("sin_half_angle") / s.Real("cos_half_angle");
        // The radius changes along the axis one way or the other; see which way fits the boundary.
        double fit = 0.0;
        for (auto const& loop : b.m_loops)
            for (Vec const& p : loop) {
                Vec const w = p - o;
                double const h = w.Dot(A);
                fit += (Vec(w - h * A).Length() - radius) * h;
            }
        double const slope = fit >= 0.0 ? tangent : -tangent;
        Revolution const r = AboutAxis(b, o, A, X, faceSense);
        double v0, v1;
        Extent(b, o, A, v0, v1);
        double const apex = slope != 0.0 ? -radius / slope : 0.0;
        if ((r.m_toTop && apex > v1) || (r.m_toBottom && apex < v0)) {
            v0 = std::min(v0, apex);
            v1 = std::max(v1, apex);
        }
        if (v0 >= v1)
            return nullptr;
        double const margin = 1.0e-3 * (v1 - v0);
        auto radiusAt = [&](double v) { return std::max(0.0, radius + slope * v); };
        if (std::fabs(v0 - apex) > margin)
            v0 -= margin;
        if (std::fabs(v1 - apex) > margin)
            v1 += margin;
        SmCone* cone = nullptr;
        SmAxis2Placement const placement = Rotated(o + v0 * A, A, X, r.m_range.m_start);
        if (SM_SUCCESS != SmCone::CreateCanonical(Context(), placement, radiusAt(v0), radiusAt(v1), v1 - v0, cone) || nullptr == cone)
            return nullptr;
        if (SM_SUCCESS != cone->AdjustSTEPUVDomain(SmExtent2d(0.0, 0.0, r.m_range.m_span * kDegrees, v1 - v0))) {
            delete cone;
            return nullptr;
        }
        return cone;
    }

    SmSurface* SphereSurface(Node const& s, Boundary const& b, double faceSense) {
        Vec const c = V(s, "centre");
        Vec A = Unit(V(s, "axis")), X = Unit(V(s, "x_axis"));
        double const radius = s.Real("radius");
        // Any axis will do for a sphere. When the face fits in a cap, put the cap's middle on the
        // equator so the face keeps clear of the poles and the seam.
        Vec middle(0, 0, 0);
        for (auto const& loop : b.m_loops)
            for (Vec const& p : loop)
                middle = middle + Unit(p - c);
        middle = Unit(middle);
        if (middle.Length() > 0.5) {
            bool inCap = true;
            for (auto const& loop : b.m_loops)
                for (Vec const& p : loop)
                    inCap = inCap && Unit(p - c).Dot(middle) > 0.2;
            Vec const side = Unit(middle * (std::fabs(middle.z) < 0.9 ? Vec(0, 0, 1) : Vec(1, 0, 0)));
            Revolution const cap = AboutAxis(b, c, middle, side, faceSense);
            if (inCap && cap.m_toTop && !cap.m_toBottom) {
                A = side;
                X = middle;
            }
        }
        Revolution const r = AboutAxis(b, c, A, X, faceSense);
        double h0, h1;
        Extent(b, c, A, h0, h1);
        double lat0 = b.m_loops.empty() ? -90.0 : std::asin(std::clamp(h0 / radius, -1.0, 1.0)) * kDegrees;
        double lat1 = b.m_loops.empty() ? 90.0 : std::asin(std::clamp(h1 / radius, -1.0, 1.0)) * kDegrees;
        if (r.m_toTop)
            lat1 = 90.0;
        if (r.m_toBottom)
            lat0 = -90.0;
        lat0 = std::max(-90.0, lat0 - 0.01);
        lat1 = std::min(90.0, lat1 + 0.01);
        SmSphere* sphere = nullptr;
        if (SM_SUCCESS != SmSphere::CreateCanonical(Context(), Rotated(c, A, X, r.m_range.m_start), radius, sphere) || nullptr == sphere)
            return nullptr;
        if (SM_SUCCESS != sphere->AdjustSTEPUVDomain(SmExtent2d(0.0, lat0, r.m_range.m_span * kDegrees, lat1))) {
            delete sphere;
            return nullptr;
        }
        return sphere;
    }

    SmSurface* TorusSurface(Node const& s, Boundary const& b, double faceSense) {
        Vec const c = V(s, "centre"), A = Unit(V(s, "axis")), X = Unit(V(s, "x_axis"));
        double const major = s.Real("major_radius"), minor = s.Real("minor_radius");
        Revolution const r = AboutAxis(b, c, A, X, faceSense);
        SmAxis2Placement const placement = Rotated(c, A, X, r.m_range.m_start);
        Vec const X1 = placement.GetXAxisRef(), Y1 = placement.GetYAxisRef();
        // The angle about the tube, zero on the outside, increasing towards the axis direction.
        auto tube = [&](Vec const& p) {
            Vec const w = p - c;
            double const h = w.Dot(A);
            return std::atan2(h, Vec(w - h * A).Length() - major);
        };
        std::vector<double> angles, hints;
        bool winds = false;
        for (auto const& loop : b.m_loops) {
            std::vector<Vec> local;
            for (Vec const& p : loop) {
                angles.push_back(tube(p));
                Vec const w = p - c;
                double const h = w.Dot(A);
                local.push_back(Vec(Vec(w - h * A).Length() - major, h, 0.0));
            }
            winds = winds || 0 != Winding(local, Vec(0, 0, 0), Vec(1, 0, 0), Vec(0, 1, 0));
        }
        for (Vec const& h : b.m_hints)
            hints.push_back(tube(h));
        Range v = AngleRange(angles, hints, winds, angles.empty() ? 0.0 : angles.front());
        double v0 = WrapPi(v.m_start) * kDegrees;
        if (v0 + v.m_span * kDegrees > 360.0)
            v0 -= 360.0;
        SmExtent2d const domain(0.0, v0, r.m_range.m_span * kDegrees, v0 + v.m_span * kDegrees);
        SmTorus* torus = nullptr;
        if (major <= 0.0 || minor <= 0.0 || SM_SUCCESS != SmTorus::CreateCanonical(Context(), SmAxis2Placement(c, X1, Y1), major, minor, torus, FALSE, FALSE, nullptr, &domain) || nullptr == torus)
            return nullptr;
        return torus;
    }

    SmBSplineSurface* BSplineSurface(Node const& s) {
        SurfaceNurbs const* nurbs = m_evaluator.Nurbs(s);
        if (nullptr == nurbs)
            return nullptr;
        SurfaceNurbs n = *nurbs;
        // Clamp in u with blocks of whole u rows, then in v with blocks of v columns.
        std::vector<std::vector<double>> rows(n.nu);
        for (int i = 0; i < n.nu; ++i)
            rows[i].assign(n.points.begin() + static_cast<size_t>(i) * n.nv * n.stride, n.points.begin() + static_cast<size_t>(i + 1) * n.nv * n.stride);
        if (!Clamp(n.pu, n.U, rows))
            return nullptr;
        n.nu = static_cast<int>(rows.size());
        std::vector<std::vector<double>> columns(n.nv, std::vector<double>(static_cast<size_t>(n.nu) * n.stride));
        for (int i = 0; i < n.nu; ++i)
            for (int j = 0; j < n.nv; ++j)
                for (int k = 0; k < n.stride; ++k)
                    columns[j][static_cast<size_t>(i) * n.stride + k] = rows[i][static_cast<size_t>(j) * n.stride + k];
        if (!Clamp(n.pv, n.V, columns))
            return nullptr;
        n.nv = static_cast<int>(columns.size());

        SmTArray<SmPoint3d> poles;
        SmTArray<double> weights;
        for (int i = 0; i < n.nu; ++i)
            for (int j = 0; j < n.nv; ++j) {
                double const* p = &columns[j][static_cast<size_t>(i) * n.stride];
                double const w = n.rational ? p[3] : 1.0;
                if (w <= 0.0)
                    return nullptr;
                poles.Add(SmPoint3d(p[0] / w, p[1] / w, p[2] / w));
                weights.Add(w);
            }
        SmTArray<double> uKnots, vKnots;
        SmTArray<ULONG> uMult, vMult;
        UniqueKnots(n.U, uKnots, uMult);
        UniqueKnots(n.V, vKnots, vMult);
        SmBSplineSurface* surface = nullptr;
        if (SM_SUCCESS != SmBSplineSurface::CreateCanonical(Context(), n.pu, n.pv, poles, SM_SF_UNSPECIFIED, uMult, vMult, uKnots, vKnots, SM_KT_UNSPECIFIED, n.rational ? &weights : nullptr, nullptr, surface))
            return nullptr;
        return surface;
    }

    //! A linear sweep of a conic or B-curve section, over the extent of the face along the sweep.
    SmBSplineSurface* SweptSurface(Node const& s, Boundary const& b) {
        Node const* section = s.Ptr("section");
        Vec const sweep = V(s, "sweep");
        if (nullptr == section || sweep.Length() <= 0.0)
            return nullptr;
        Curve curve;
        Vec normal = Unit(sweep), base(0, 0, 0);
        if (section->Is(XT::CIRCLE) || section->Is(XT::ELLIPSE)) {
            bool const circle = section->Is(XT::CIRCLE);
            double const a = circle ? section->Real("radius") : section->Real("major_radius"), r = circle ? a : section->Real("minor_radius");
            Vec const X = Unit(V(*section, "x_axis")), N = Unit(V(*section, "normal"));
            curve = ConicArc(V(*section, "centre"), X, N * X, a, r, 0.0, 2.0 * kPi);
            normal = N;
            base = V(*section, "centre");
        } else if (section->Is(XT::B_CURVE)) {
            Nurbs nurbs;
            if (!ReadCurve(section, nurbs) || !ToCurve(nurbs, curve))
                return nullptr;
            base = Vec(curve.poles[0], curve.poles[1], curve.poles[2]);
        } else {
            return nullptr;
        }
        double const along = sweep.Dot(normal);
        if (std::fabs(along) <= 1.0e-12 * sweep.Length())
            return nullptr;
        double v0 = SM_BIG_DOUBLE, v1 = -SM_BIG_DOUBLE;
        for (auto const& loop : b.m_loops)
            for (Vec const& p : loop) {
                double const v = Vec(p - base).Dot(normal) / along;
                v0 = std::min(v0, v);
                v1 = std::max(v1, v);
            }
        if (v0 >= v1)
            return nullptr;
        double const margin = 1.0e-3 * (v1 - v0);
        v0 -= margin;
        v1 += margin;
        SmTArray<SmPoint3d> poles;
        SmTArray<double> weights;
        size_t const count = curve.poles.size() / 3;
        for (size_t i = 0; i < count; ++i) {
            Vec const p(curve.poles[3 * i], curve.poles[3 * i + 1], curve.poles[3 * i + 2]);
            poles.Add(p + v0 * sweep);
            poles.Add(p + v1 * sweep);
            double const w = curve.weights.empty() ? 1.0 : curve.weights[i];
            weights.Add(w);
            weights.Add(w);
        }
        SmTArray<double> uKnots, vKnots;
        SmTArray<ULONG> uMult, vMult;
        UniqueKnots(curve.knots, uKnots, uMult);
        UniqueKnots({v0, v0, v1, v1}, vKnots, vMult);
        SmBSplineSurface* surface = nullptr;
        if (SM_SUCCESS != SmBSplineSurface::CreateCanonical(Context(), curve.degree, 1, poles, SM_SF_UNSPECIFIED, uMult, vMult, uKnots, vKnots, SM_KT_UNSPECIFIED, curve.weights.empty() ? nullptr : &weights, nullptr, surface))
            return nullptr;
        return surface;
    }

    //! Parasolid often extends b-surfaces well past their faces; trim one to the face's parameter
    //! box so SMLib doesn't tessellate and cache the rest.
    static SmBSplineSurface* FitToFace(SmBSplineSurface* surface, Boundary const& b) {
        if (nullptr == surface)
            return nullptr;
        SmExtent2d const domain = surface->GetNaturalUVDomain();
        double u0 = SM_BIG_DOUBLE, u1 = -SM_BIG_DOUBLE, v0 = SM_BIG_DOUBLE, v1 = -SM_BIG_DOUBLE;
        for (auto const& loop : b.m_loops)
            for (size_t i = 0; i < loop.size(); i += 4) {
                SmBoolean success = FALSE, multivalued = FALSE;
                SmPoint2d uv;
                double gap = 0.0;
                if (SM_SUCCESS != surface->DropPoint(loop[i], domain, nullptr, success, uv, gap, multivalued) || !success)
                    return surface;
                u0 = std::min(u0, uv.x);
                u1 = std::max(u1, uv.x);
                v0 = std::min(v0, uv.y);
                v1 = std::max(v1, uv.y);
            }
        if (u0 > u1)
            return surface;
        // Keep a margin, and the whole range of a direction the face nearly covers or may wrap in.
        auto fit = [](double& low, double& high, double min, double max) {
            double const length = max - min, margin = 0.05 * length;
            if (high - low > 0.8 * length) {
                low = min;
                high = max;
                return false;
            }
            low = std::max(min, low - margin);
            high = std::min(max, high + margin);
            return true;
        };
        bool const fitU = fit(u0, u1, domain.GetUMin(), domain.GetUMax()), fitV = fit(v0, v1, domain.GetVMin(), domain.GetVMax());
        if ((fitU || fitV) && SM_SUCCESS != surface->AdjustSTEPUVDomain(SmExtent2d(u0, v0, u1, v1))) {
            delete surface;
            return nullptr;
        }
        return surface;
    }

    //! A rolling-ball blend around a conic spine: circles across, of the ball's radius, swept
    //! along the part of the spine the face covers. Exact across, interpolated along.
    SmBSplineSurface* BlendSurface(Node const& s, Boundary const& b) {
        Node const* spine = s.Ptr("spine");
        if (nullptr == spine || !(spine->Is(XT::CIRCLE) || spine->Is(XT::ELLIPSE)) || b.m_loops.empty())
            return nullptr;
        double radius = std::fabs(s.Real("range", 0));
        if (!(radius > 0.0))
            radius = std::fabs(s.Real("range", 1));
        bool const circle = spine->Is(XT::CIRCLE);
        double const a = circle ? spine->Real("radius") : spine->Real("major_radius"), c = circle ? a : spine->Real("minor_radius");
        if (!(radius > 0.0) || !(a > 0.0) || !(c > 0.0))
            return nullptr;
        Vec const centre = V(*spine, "centre"), N = Unit(V(*spine, "normal")), X = Unit(V(*spine, "x_axis")), Y = N * X;
        auto parameter = [&](Vec const& p) { Vec const w = p - centre; return std::atan2(w.Dot(Y) / c, w.Dot(X) / a); };
        auto at = [&](double t, Vec& point, Vec& across) {
            point = centre + (a * std::cos(t)) * X + (c * std::sin(t)) * Y;
            across = Unit(Unit((-a * std::sin(t)) * X + (c * std::cos(t)) * Y) * N);
        };
        std::vector<double> along, around;
        for (auto const& loop : b.m_loops)
            for (Vec const& p : loop) {
                double const t = parameter(p);
                Vec point, across;
                at(t, point, across);
                Vec const w = p - point;
                along.push_back(t);
                around.push_back(std::atan2(w.Dot(N), w.Dot(across)));
            }
        Range const tRange = AngleRange(along, {}, false, along.front());
        Range const phiRange = AngleRange(around, {}, false, around.front());
        Curve const arc = ConicArc(Vec(0, 0, 0), Vec(1, 0, 0), Vec(0, 1, 0), radius, radius, phiRange.m_start, phiRange.m_start + phiRange.m_span);
        size_t const arcPoles = arc.poles.size() / 3;
        int const samples = std::max(8, static_cast<int>(std::ceil(tRange.m_span / (kPi / 32.0))));
        std::vector<std::vector<double>> rows;
        for (int k = 0; k <= samples; ++k) {
            Vec point, across;
            at(tRange.m_start + tRange.m_span * k / samples, point, across);
            std::vector<double> row;
            for (size_t j = 0; j < arcPoles; ++j) {
                Vec const q = point + arc.poles[3 * j] * across + arc.poles[3 * j + 1] * N;
                row.insert(row.end(), {q.x, q.y, q.z});
            }
            rows.push_back(std::move(row));
        }
        int degree;
        std::vector<double> knots;
        std::vector<std::vector<double>> poles;
        if (!InterpolateEvenly(rows, degree, knots, poles))
            return nullptr;
        SmTArray<SmPoint3d> net;
        SmTArray<double> weights;
        for (auto const& row : poles)
            for (size_t j = 0; j < arcPoles; ++j) {
                net.Add(SmPoint3d(row[3 * j], row[3 * j + 1], row[3 * j + 2]));
                weights.Add(arc.weights[j]);
            }
        SmTArray<double> uKnots, vKnots;
        SmTArray<ULONG> uMult, vMult;
        UniqueKnots(knots, uKnots, uMult);
        UniqueKnots(arc.knots, vKnots, vMult);
        SmBSplineSurface* surface = nullptr;
        if (SM_SUCCESS != SmBSplineSurface::CreateCanonical(Context(), degree, 2, net, SM_SF_UNSPECIFIED, uMult, vMult, uKnots, vKnots, SM_KT_UNSPECIFIED, &weights, nullptr, surface))
            return nullptr;
        return surface;
    }

    SmSurface* Surface(Node const& s, Boundary const& b, double faceSense) {
        switch (s.Type()) {
            case XT::PLANE: return PlaneSurface(s, b);
            case XT::CYLINDER: return CylinderSurface(s, b, faceSense);
            case XT::CONE: return ConeSurface(s, b, faceSense);
            case XT::SPHERE: return SphereSurface(s, b, faceSense);
            case XT::TORUS: return TorusSurface(s, b, faceSense);
            case XT::B_SURFACE: return FitToFace(BSplineSurface(s), b);
            case XT::SWEPT_SURF: return SweptSurface(s, b);
            case XT::BLENDED_EDGE: return BlendSurface(s, b);
            case XT::OFFSET_SURF: {
                Node const* base = s.Ptr("surface");
                SmSurface* baseSurface = nullptr;
                if (base && base->Is(XT::B_SURFACE))
                    baseSurface = FitToFace(BSplineSurface(*base), b);
                else if (base && base->Is(XT::SWEPT_SURF))
                    baseSurface = SweptSurface(*base, b);
                if (nullptr == baseSurface)
                    return nullptr;
                return new (Context()) SmOffsetSurface(s.Real("offset") * Sense(*base), *baseSurface, TRUE);
            }
            default: return nullptr;
        }
    }

    //! Whether the SMLib surface's normal agrees with the Parasolid face's normal.
    static bool NormalAt(SmSurface& surface, Vec const& p, Vec& normal) {
        SmBoolean success = FALSE, multivalued = FALSE;
        SmPoint2d uv;
        double gap = 0.0;
        return SM_SUCCESS == surface.DropPoint(p, surface.GetNaturalUVDomain(), nullptr, success, uv, gap, multivalued) && success &&
            SM_SUCCESS == surface.EvaluateNormal(uv, TRUE, TRUE, normal);
    }

    //! For a blend, whose normal is continuous with its neighbours': compare with a neighbour's
    //! normal along a shared edge.
    bool SameAsNeighbour(SmSurface& surface, Node const& face) {
        for (Node const* loop : Loops(face))
            for (Node const* fin : Fins(*loop)) {
                Node const* other = fin->Ptr("other");
                Node const* otherLoop = other ? other->Ptr("loop") : nullptr;
                Node const* neighbour = otherLoop ? otherLoop->Ptr("face") : nullptr;
                Node const* neighbourSurface = neighbour ? neighbour->Ptr("surface") : nullptr;
                EdgeData* e = fin->Ptr("edge") ? Edge(*fin->Ptr("edge")) : nullptr;
                if (nullptr == neighbourSurface || nullptr == e || e->m_samples.empty())
                    continue;
                Vec const p = e->m_samples[e->m_samples.size() / 2];
                Vec expected, normal;
                if (!AnalyticNormal(*neighbourSurface, p, expected) || !NormalAt(surface, p, normal))
                    continue;
                return normal.Dot(expected) * Sense(*neighbour) * Sense(*neighbourSurface) >= 0.0;
            }
        return true;
    }

    bool SameNormal(SmSurface& surface, Node const& face, Node const& psSurface, Boundary const& b, double faceSense) {
        switch (psSurface.Type()) {
            case XT::B_SURFACE: case XT::OFFSET_SURF: case XT::SWEPT_SURF:
                return faceSense > 0.0; // same parameterization
            case XT::BLENDED_EDGE:
                return SameAsNeighbour(surface, face);
            default: break;
        }
        double best = 0.0;
        std::vector<Vec> points = b.m_hints;
        for (auto const& loop : b.m_loops)
            for (size_t i = 0; i < loop.size() && i < 64; i += 7)
                points.push_back(loop[i]);
        for (Vec const& p : points) {
            Vec expected;
            if (!AnalyticNormal(psSurface, p, expected))
                continue;
            SmBoolean success = FALSE, multivalued = FALSE;
            SmPoint2d uv;
            double gap = 0.0;
            SmVector3d normal;
            if (SM_SUCCESS != surface.DropPoint(p, surface.GetNaturalUVDomain(), nullptr, success, uv, gap, multivalued) || !success ||
                SM_SUCCESS != surface.EvaluateNormal(uv, TRUE, TRUE, normal))
                continue;
            double const d = normal.Dot(expected) * faceSense;
            if (std::fabs(d) > std::fabs(best))
                best = d;
            if (std::fabs(best) > 0.9)
                break;
        }
        return best >= 0.0;
    }

    void CheckBoundary(Node const& face, SmSurface& surface, Boundary const& b) {
        double worst = 0.0;
        for (auto const& loop : b.m_loops)
            for (Vec const& p : loop) {
                SmBoolean success = FALSE, multivalued = FALSE;
                SmPoint2d uv;
                double gap = 0.0;
                if (SM_SUCCESS != surface.DropPoint(p, surface.GetNaturalUVDomain(), nullptr, success, uv, gap, multivalued) || !success)
                    gap = SM_BIG_DOUBLE;
                worst = std::max(worst, gap);
            }
        if (worst > 1.0e-7 * m_size)
            Log("face " + std::to_string(face.Int("node_id")) + " " + face.Ptr("surface")->Name() + ": boundary off surface by " + std::to_string(worst));
    }

    /*-----------------------------------------------------------------------------------
    * Topology
    +----------------------------------------------------------------------------------*/
    SmVertex* Vertex(SmBrepConstructor& builder, Node const* vertex) {
        auto found = m_vertices.find(vertex);
        if (found != m_vertices.end())
            return found->second;
        return m_vertices[vertex] = builder.StartVertexOfLoop(PointOf(vertex));
    }

    bool AddFin(SmBrepConstructor& builder, Node const& fin, bool reversed) {
        Node const* edge = fin.Ptr("edge");
        EdgeData* e = edge ? Edge(*edge) : nullptr;
        if (nullptr == e)
            return false;
        bool const along = ('-' != fin.Char("sense")) != reversed;
        SmOrientType const orientation = along ? SM_OT_SAME : SM_OT_OPPOSITE;
        if (e->m_edge) {
            if (nullptr == builder.StartEdge(orientation, e->m_edge, nullptr, nullptr))
                return false;
            return SM_SUCCESS == builder.EndEdge();
        }
        SmVertex* start;
        SmVertex* end;
        if (e->m_start) {
            start = Vertex(builder, e->m_start);
            end = Vertex(builder, e->m_end);
        } else {
            if (nullptr == e->m_ringVertex)
                e->m_ringVertex = builder.StartVertexOfLoop(e->m_startPoint);
            start = end = e->m_ringVertex;
        }
        // In the edge's direction, whatever the fin's.
        e->m_edge = builder.StartEdge(orientation, nullptr, start, end);
        if (nullptr == e->m_edge || SM_SUCCESS != builder.SetEdgeCurve(e->m_curve, FALSE))
            return false;
        SmBSplineCurve* curve = e->m_curve;
        e->m_curve = nullptr; // the edge owns it
        SmStatus status;
        if (e->m_limitsFromPoints)
            status = builder.SetLimits(SmVertexNode(e->m_startPoint), SmVertexNode(e->m_endPoint));
        else
            status = builder.SetLimits(SmVertexNode(curve->GetNaturalInterval().GetMin()), SmVertexNode(curve->GetNaturalInterval().GetMax()));
        return SM_SUCCESS == status && SM_SUCCESS == builder.EndEdge();
    }

    SmVertex* EndVertex(SmBrepConstructor& builder, EdgeData& e) {
        if (e.m_start)
            return Vertex(builder, e.m_start);
        if (nullptr == e.m_ringVertex)
            e.m_ringVertex = builder.StartVertexOfLoop(e.m_startPoint);
        return e.m_ringVertex;
    }

    //! A closed b-surface face bounded by a ring at each end, both starting on the surface's seam.
    //! SMLib's healer doesn't add the seam edge such a face needs, so make it here.
    struct Seam {
        std::unique_ptr<SmBSplineCurve> m_curve;
        Node const* m_low = nullptr; //!< the ring fin at the seam curve's start
        Node const* m_high = nullptr;
        Vec m_lowPoint, m_highPoint;
    };

    bool FindSeam(SmSurface& surface, std::vector<Node const*> const& loops, Seam& seam) {
        if (2 != loops.size())
            return false;
        Node const* fins[2];
        EdgeData* edges[2];
        SmPoint2d uvs[2];
        SmExtent2d const domain = surface.GetNaturalUVDomain();
        for (int i = 0; i < 2; ++i) {
            auto loopFins = Fins(*loops[i]);
            Node const* edge = 1 == loopFins.size() ? loopFins[0]->Ptr("edge") : nullptr;
            edges[i] = edge ? Edge(*edge) : nullptr;
            if (nullptr == edges[i] || !edges[i]->m_closed)
                return false;
            fins[i] = loopFins[0];
            SmBoolean success = FALSE, multivalued = FALSE;
            double gap = 0.0;
            if (SM_SUCCESS != surface.DropPoint(edges[i]->m_startPoint, domain, nullptr, success, uvs[i], gap, multivalued) || !success)
                return false;
        }
        for (int direction = 0; direction < 2; ++direction) {
            SmSurfParamType const along = 0 == direction ? SM_SP_U : SM_SP_V;
            double const min = 0 == direction ? domain.GetUMin() : domain.GetVMin(), max = 0 == direction ? domain.GetUMax() : domain.GetVMax();
            double const tolerance = 1.0e-6 * (max - min);
            auto onSeam = [&](SmPoint2d const& uv) { double const t = 0 == direction ? uv.x : uv.y; return std::fabs(t - min) < tolerance || std::fabs(t - max) < tolerance; };
            if (!surface.IsClosed(domain, along) || !onSeam(uvs[0]) || !onSeam(uvs[1]))
                continue;
            SmBSplineCurve* curve = nullptr;
            if (SM_SUCCESS != surface.CreateIsoParametricCurve(Context(), along, min, 0.0, curve, &domain) || nullptr == curve)
                return false;
            seam.m_curve.reset(curve);
            double const t0 = 0 == direction ? uvs[0].y : uvs[0].x, t1 = 0 == direction ? uvs[1].y : uvs[1].x;
            int const low = t0 <= t1 ? 0 : 1;
            seam.m_low = fins[low];
            seam.m_high = fins[1 - low];
            seam.m_lowPoint = edges[low]->m_startPoint;
            seam.m_highPoint = edges[1 - low]->m_startPoint;
            return true;
        }
        return false;
    }

    //! The face's single loop: ring, seam, the other ring, and the seam back.
    bool AddSeamLoop(SmBrepConstructor& builder, Seam& seam, Node const* first, bool same) {
        struct Use { Node const* fin; bool along; };
        Node const* second = first == seam.m_low ? seam.m_high : seam.m_low;
        bool const up = first == seam.m_low;
        std::vector<Use> uses = {{first, true}, {nullptr, up}, {second, true}, {nullptr, !up}};
        if (!same) {
            std::reverse(uses.begin(), uses.end());
            for (Use& use : uses)
                use.along = !use.along;
        }
        if (nullptr == builder.StartLoop(SM_OT_SAME))
            return false;
        SmEdge* seamEdge = nullptr;
        for (Use const& use : uses) {
            if (use.fin) {
                if (!AddFin(builder, *use.fin, !same))
                    return false;
                continue;
            }
            SmOrientType const orientation = use.along ? SM_OT_SAME : SM_OT_OPPOSITE;
            if (seamEdge) {
                if (nullptr == builder.StartEdge(orientation, seamEdge, nullptr, nullptr) || SM_SUCCESS != builder.EndEdge())
                    return false;
                continue;
            }
            SmVertex* low = EndVertex(builder, *Edge(*seam.m_low->Ptr("edge")));
            SmVertex* high = EndVertex(builder, *Edge(*seam.m_high->Ptr("edge")));
            seamEdge = builder.StartEdge(orientation, nullptr, low, high);
            if (nullptr == seamEdge || SM_SUCCESS != builder.SetEdgeCurve(seam.m_curve.get(), FALSE))
                return false;
            seam.m_curve.release();
            if (SM_SUCCESS != builder.SetLimits(SmVertexNode(seam.m_lowPoint), SmVertexNode(seam.m_highPoint)) || SM_SUCCESS != builder.EndEdge())
                return false;
        }
        return SM_SUCCESS == builder.EndLoop();
    }

    bool AddFace(SmBrepConstructor& builder, Node const& face, std::vector<std::pair<SmFace*, uint32_t>>& tags, bool useTags) {
        Node const* psSurface = face.Ptr("surface");
        if (nullptr == psSurface)
            return false;
        double const faceSense = Sense(face) * Sense(*psSurface);
        Boundary boundary;
        CollectBoundary(face, *psSurface, faceSense, boundary);
        SmSurface* surface = Surface(*psSurface, boundary, faceSense);
        if (nullptr == surface) {
            Log("face " + std::to_string(face.Int("node_id")) + ": unsupported surface " + psSurface->Name());
            return false;
        }
        std::unique_ptr<SmSurface> surfaceOwner(surface);
        bool const same = SameNormal(*surface, face, *psSurface, boundary, faceSense);

        // Check every fin can be built before starting the face, so a failure leaves no partial face.
        auto loops = Loops(face);
        // SMLib wants the outer loop first. On a plane it's the one going counterclockwise.
        if (psSurface->Is(XT::PLANE) && loops.size() > 1) {
            Vec const normal = Unit(V(*psSurface, "normal")) * faceSense;
            double best = -SM_BIG_DOUBLE;
            Node const* outer = nullptr;
            for (size_t i = 0; i < boundary.m_loops.size(); ++i) {
                Vec area(0, 0, 0);
                auto const& points = boundary.m_loops[i];
                for (size_t j = 0; j < points.size(); ++j)
                    area = area + Vec(points[j] - points[0]) * Vec(points[(j + 1) % points.size()] - points[0]);
                if (area.Dot(normal) > best) {
                    best = area.Dot(normal);
                    outer = boundary.m_loopNodes[i];
                }
            }
            auto found = std::find(loops.begin(), loops.end(), outer);
            if (found != loops.end())
                std::rotate(loops.begin(), found, found + 1);
        }
        for (Node const* loop : loops)
            for (Node const* fin : Fins(*loop)) {
                Node const* edge = fin->Ptr("edge");
                if (nullptr == edge || nullptr == Edge(*edge)) {
                    Log("face " + std::to_string(face.Int("node_id")) + ": missing edge");
                    return false;
                }
            }

        SmFace* smFace = builder.StartFace(same ? SM_OT_SAME : SM_OT_OPPOSITE);
        if (nullptr == smFace || SM_SUCCESS != builder.SetFaceSurface(surface, surface->GetNaturalUVDomain()))
            return false;
        surfaceOwner.release();
        if (m_log)
            CheckBoundary(face, *surface, boundary);
        if (useTags) {
            int index = 0;
            FindIntAttribute(face, "BSI_FaceMatIdx", index);
            tags.emplace_back(smFace, static_cast<uint32_t>(std::max(0, index)));
        }
        Seam seam;
        if ((psSurface->Is(XT::B_SURFACE) || psSurface->Is(XT::OFFSET_SURF)) && FindSeam(*surface, loops, seam))
            return AddSeamLoop(builder, seam, Fins(*loops[0])[0], same) && SM_SUCCESS == builder.EndFace();
        for (Node const* loop : loops) {
            auto fins = Fins(*loop);
            if (!same)
                std::reverse(fins.begin(), fins.end());
            if (nullptr == builder.StartLoop(SM_OT_SAME))
                return false;
            for (Node const* fin : fins)
                if (!AddFin(builder, *fin, !same))
                    return false;
            if (SM_SUCCESS != builder.EndLoop())
                return false;
        }
        return SM_SUCCESS == builder.EndFace();
    }

    static std::vector<Node const*> ShellFaces(Node const& shell) {
        std::vector<Node const*> faces;
        std::set<Node const*> seen;
        for (Node const* face = shell.Ptr("face"); face && seen.insert(face).second; face = face->Ptr("next"))
            faces.push_back(face);
        for (Node const* face = shell.Ptr("front_face"); face && seen.insert(face).second; face = face->Ptr("next_front"))
            faces.push_back(face);
        return faces;
    }

    BodyPtr Convert(std::vector<uint32_t>& faceTags) {
        Node const& body = *m_model.m_body;
        m_size = std::max(1.0e-6, body.Real("res_size"));
        int const bodyType = body.Int("body_type");
        if (1 != bodyType && 3 != bodyType) {
            Log("unsupported body type " + std::to_string(bodyType));
            return nullptr;
        }
        bool useTags = false;
        for (auto const& [index, node] : m_model.m_nodes) {
            if (!node.Is(XT::ATT_DEF_ID))
                continue;
            std::string id;
            for (size_t i = 0; i < node.Count("string"); ++i)
                id += static_cast<char>(node.Int("string", i));
            useTags = useTags || id == "BSI_FaceMatIdx";
        }

        // Shells by region: those bounding solid regions, and those in void regions for sheets.
        std::vector<std::pair<bool, Node const*>> shells;
        int guard = 0;
        for (Node const* region = body.Ptr("region"); region && guard < 10000; region = region->Ptr("next"), ++guard) {
            bool const solid = 'S' == region->Char("type");
            if (1 == bodyType && !solid)
                continue;
            for (Node const* shell = region->Ptr("shell"); shell && guard < 10000; shell = shell->Ptr("next"), ++guard)
                shells.emplace_back(solid, shell);
        }

        // SMLib's default tolerance suits bodies tens of units across; scale it down for small ones.
        double low[3] = {SM_BIG_DOUBLE, SM_BIG_DOUBLE, SM_BIG_DOUBLE}, high[3] = {-SM_BIG_DOUBLE, -SM_BIG_DOUBLE, -SM_BIG_DOUBLE};
        for (auto const& [index, node] : m_model.m_nodes) {
            EdgeData const* e = node.Is(XT::EDGE) ? Edge(node) : nullptr;
            for (Vec const& p : e ? e->m_samples : std::vector<Vec>())
                for (int i = 0; i < 3; ++i) {
                    low[i] = std::min(low[i], p[i]);
                    high[i] = std::max(high[i], p[i]);
                }
        }
        double const diagonal = low[0] <= high[0] ? std::sqrt((high[0] - low[0]) * (high[0] - low[0]) + (high[1] - low[1]) * (high[1] - low[1]) + (high[2] - low[2]) * (high[2] - low[2])) : 0.0;

        SmBrepConstructor builder;
        SmBrep* brep = builder.StartBrep(Context());
        if (nullptr == brep)
            return nullptr;
        BodyPtr result = Wrap(brep);
        if (diagonal > 0.0)
            brep->SetTolerance(std::clamp(1.0e-6 * diagonal, 1.0e-9, SM_ZONE_TOL_3D), FALSE);
        std::vector<std::pair<SmFace*, uint32_t>> tags;
        size_t added = 0, skipped = 0;
        bool openRegion = false;
        for (auto const& [solid, shell] : shells) {
            auto faces = ShellFaces(*shell);
            if (faces.empty())
                continue;
            bool const inRegion = solid && !openRegion;
            if (inRegion && nullptr == builder.StartRegion(TRUE, nullptr))
                return nullptr;
            SmShell* outer = nullptr;
            SmShell* inner = nullptr;
            if (SM_SUCCESS != builder.StartShell(outer, inner))
                return nullptr;
            for (Node const* face : faces) {
                if (AddFace(builder, *face, tags, useTags))
                    ++added;
                else
                    ++skipped;
            }
            if (SM_SUCCESS != builder.EndShell(FALSE) || (inRegion && SM_SUCCESS != builder.EndRegion()))
                return nullptr;
        }
        if (SM_SUCCESS != builder.EndBrep() || 0 == added)
            return nullptr;
        if (skipped > 0)
            Log(std::to_string(skipped) + " of " + std::to_string(added + skipped) + " faces skipped");

        for (auto const& [face, tag] : tags)
            face->AddAttribute(new (Context()) SmLongAttribute(kFaceTagAttribute, static_cast<long>(tag), SM_AB_COPY));

        brep->m_bEditingEnabled = TRUE;
        if (SM_SUCCESS != brep->HealBrep(SM_HO_ALL, nullptr, nullptr, nullptr, TRUE))
            Log("healing reported failure");

        faceTags.clear();
        SmTArray<SmFace*> faces;
        brep->GetFaces(faces);
        for (ULONG i = 0; i < faces.GetSize(); ++i) {
            SmTArray<SmAttribute*> attributes;
            faces[i]->GetAttributes(attributes);
            uint32_t tag = 0;
            for (ULONG j = 0; j < attributes.GetSize(); ++j) {
                if (attributes[j]->GetAttributeID() != kFaceTagAttribute)
                    continue;
                tag = static_cast<uint32_t>(static_cast<SmLongAttribute*>(attributes[j])->GetValue());
                faces[i]->RemoveAttribute(attributes[j]);
            }
            if (useTags)
                faceTags.push_back(tag);
        }
        if (0 == faces.GetSize())
            return nullptr;
        return result;
    }
};

} // namespace

BodyPtr RestoreParasolid(uint8_t const* data, size_t size, std::vector<uint32_t>& faceTags, double& scale, std::string* log) {
    faceTags.clear();
    scale = 1.0;
    XT::Model model;
    if (!XT::Read(data, size, model)) {
        if (log)
            *log += "not readable transmit data\n";
        return nullptr;
    }
    try {
        scale = NormalizingScale(model);
        if (scale != 1.0)
            ScaleModel(model, scale);
        Converter converter(model, log);
        return converter.Convert(faceTags);
    } catch (...) {
        if (log)
            *log += "exception\n";
        return nullptr;
    }
}

} // namespace SmKernel
