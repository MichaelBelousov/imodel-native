/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
// Checks the SmKernel operations behind createBRepGeometry against known volumes.
#include "../SmKernel.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>

using namespace SmKernel;

namespace {

int s_failures = 0;
constexpr double kPi = 3.14159265358979323846;

Curve Line(double x0, double y0, double z0, double x1, double y1, double z1) {
    Curve c;
    c.degree = 1;
    c.poles = {x0, y0, z0, x1, y1, z1};
    c.knots = {0, 0, 1, 1};
    return c;
}

Curve Polyline(std::vector<double> const& xyz) {
    Curve c;
    c.degree = 1;
    c.poles = xyz;
    size_t const n = xyz.size() / 3;
    c.knots.push_back(0);
    for (size_t i = 0; i < n; ++i)
        c.knots.push_back(static_cast<double>(i));
    c.knots.push_back(static_cast<double>(n - 1));
    return c;
}

//! A full circle in a plane parallel to xy, counterclockwise seen from +z unless reversed.
Curve Circle(double cx, double cy, double cz, double r, bool reversed = false) {
    Curve c;
    c.degree = 2;
    double const s = std::sqrt(0.5);
    double const xs[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1}, ys[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    for (int i = 0; i < 9; ++i) {
        double const y = reversed ? -ys[i] : ys[i];
        c.poles.insert(c.poles.end(), {cx + r * xs[i], cy + r * y, cz});
        c.weights.push_back(i % 2 ? s : 1.0);
    }
    c.knots = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};
    return c;
}

Curves Rectangle(double x0, double y0, double x1, double y1, double z) {
    return {Line(x0, y0, z, x1, y0, z), Line(x1, y0, z, x1, y1, z), Line(x1, y1, z, x0, y1, z), Line(x0, y1, z, x0, y0, z)};
}

BodyPtr Box(double x0, double y0, double z0, double x1, double y1, double z1) {
    double const origin[3] = {x0, y0, z0}, size[3] = {x1 - x0, y1 - y0, z1 - z0};
    return CreateBox(origin, size);
}

char const* KindName(Kind kind) { return Kind::Solid == kind ? "solid" : Kind::Sheet == kind ? "sheet" : "wire"; }

double Volume(Body const& body) {
    MassProperties props;
    return GetMassProperties(body, 1.0e-6, props) ? props.volume : NAN;
}

void Report(char const* name, BodyPtr const& body, Kind expectedKind, double expectedVolume = NAN) {
    if (!body) {
        std::printf("FAIL %-28s no body\n", name);
        ++s_failures;
        return;
    }
    Kind const kind = GetKind(*body);
    double low[3] = {}, high[3] = {};
    GetRange(*body, low, high);
    double const volume = Kind::Solid == kind ? Volume(*body) : NAN;
    bool ok = kind == expectedKind;
    if (!std::isnan(expectedVolume))
        ok = ok && std::fabs(volume - expectedVolume) <= 1.0e-3 * std::max(1.0, std::fabs(expectedVolume));
    std::printf("%s %-28s %s faces=%zu volume=%.5f (expected %.5f) range=[%.3f %.3f %.3f]..[%.3f %.3f %.3f]\n", ok ? "ok  " : "FAIL", name, KindName(kind),
        GetFaceCount(*body), volume, expectedVolume, low[0], low[1], low[2], high[0], high[1], high[2]);
    if (!ok)
        ++s_failures;
    std::vector<uint8_t> saved;
    std::vector<uint32_t> tags;
    double scale;
    if (!Save(*body, tags, saved) || !Restore(saved.data(), saved.size(), tags, scale)) {
        std::printf("FAIL %-28s save/restore\n", name);
        ++s_failures;
    }
}

void Run(char const* name, std::function<void()> const& test) {
    std::lock_guard<std::recursive_mutex> lock(Mutex());
    try {
        test();
    } catch (...) {
        std::printf("FAIL %-28s threw\n", name);
        ++s_failures;
    }
}

} // namespace

int main() {
    Run("planar sheet", [] {
        auto sheet = CreatePlanarSheet({Rectangle(0, 0, 5, 5, 0)});
        Report("planar sheet", sheet, Kind::Sheet);
        double origin[3], normal[3];
        if (sheet && GetPlane(*sheet, origin, normal))
            std::printf("     planar sheet normal (%.3f %.3f %.3f)\n", normal[0], normal[1], normal[2]);
        auto disc = CreatePlanarSheet({{Circle(0, 0, 0, 1)}});
        Report("disc sheet", disc, Kind::Sheet);
        if (disc && GetPlane(*disc, origin, normal))
            std::printf("     disc normal (%.3f %.3f %.3f)\n", normal[0], normal[1], normal[2]);
        auto reversed = CreatePlanarSheet({{Circle(0, 0, 0, 1, true)}});
        if (reversed && GetPlane(*reversed, origin, normal))
            std::printf("     reversed disc normal (%.3f %.3f %.3f)\n", normal[0], normal[1], normal[2]);
    });
    Run("extrude", [] {
        double const up[3] = {0, 0, 1};
        Report("extrude circle", Extrude({{Circle(0, 0, 0, 2)}}, up, 3), Kind::Solid, kPi * 4 * 3);
        Report("extrude rectangle", Extrude({Rectangle(0, 0, 2, 3, 0)}, up, 4), Kind::Solid, 24);
        double const down[3] = {0, 0, -1};
        Report("extrude circle down", Extrude({{Circle(0, 0, 0, 2)}}, down, 3), Kind::Solid, kPi * 4 * 3);
    });
    Run("cut", [] {
        auto box = Box(0, 0, 0, 5, 5, 5);
        double const up[3] = {0, 0, 1};
        BodyPtr tool = Extrude({{Circle(2.5, 2.5, 2.5, 2)}}, up, 10);
        Report("cut tool", tool, Kind::Solid);
        bool ok = Boolean(box, std::move(tool), BooleanOp::Subtract);
        std::printf("     subtract %s\n", ok ? "ok" : "failed");
        Report("cut half", box, Kind::Solid, 125 - kPi * 4 * 2.5);
    });
    Run("half space", [] {
        auto box = Box(-5, -5, -5, 5, 5, 5);
        auto halfSpace = Box(-20, -20, 0, 20, 20, 20);
        Boolean(box, std::move(halfSpace), BooleanOp::Intersect);
        Report("half space", box, Kind::Solid, 500);
    });
    Run("sew", [] {
        std::vector<BodyPtr> sheets;
        sheets.push_back(CreatePlanarSheet({{Line(0, 0, 0, 0, 2, 0), Line(0, 2, 0, 1, 2, 0), Line(1, 2, 0, 1, 0, 0), Line(1, 0, 0, 0, 0, 0)}}));
        sheets.push_back(CreatePlanarSheet({{Line(0, 0, 3, 1, 0, 3), Line(1, 0, 3, 1, 2, 3), Line(1, 2, 3, 0, 2, 3), Line(0, 2, 3, 0, 0, 3)}}));
        sheets.push_back(CreatePlanarSheet({{Line(0, 0, 0, 1, 0, 0), Line(1, 0, 0, 1, 0, 3), Line(1, 0, 3, 0, 0, 3), Line(0, 0, 3, 0, 0, 0)}}));
        sheets.push_back(CreatePlanarSheet({{Line(1, 2, 0, 0, 2, 0), Line(0, 2, 0, 0, 2, 3), Line(0, 2, 3, 1, 2, 3), Line(1, 2, 3, 1, 2, 0)}}));
        sheets.push_back(CreatePlanarSheet({{Line(0, 2, 0, 0, 0, 0), Line(0, 0, 0, 0, 0, 3), Line(0, 0, 3, 0, 2, 3), Line(0, 2, 3, 0, 2, 0)}}));
        sheets.push_back(CreatePlanarSheet({{Line(1, 0, 0, 1, 2, 0), Line(1, 2, 0, 1, 2, 3), Line(1, 2, 3, 1, 0, 3), Line(1, 0, 3, 1, 0, 0)}}));
        Report("sew", Sew(std::move(sheets)), Kind::Solid, 6);
    });
    Run("hollow", [] {
        auto box = Box(0, 0, 0, 5, 5, 2);
        bool ok = Hollow(box, 0.25);
        std::printf("     hollow +0.25 %s\n", ok ? "ok" : "failed");
        Report("hollow out", box, Kind::Solid, 5.5 * 5.5 * 2.5 - 50);
        auto box2 = Box(0, 0, 0, 5, 5, 2);
        ok = Hollow(box2, -0.25);
        std::printf("     hollow -0.25 %s\n", ok ? "ok" : "failed");
        Report("hollow in", box2, Kind::Solid, 50 - 4.5 * 4.5 * 1.5);
    });
    Run("offset", [] {
        auto box = Box(0, 0, 0, 5, 5, 2);
        bool ok = Offset(box, 0.25);
        std::printf("     offset solid %s\n", ok ? "ok" : "failed");
        Report("offset solid", box, Kind::Solid, 5.5 * 5.5 * 2.5);
        auto sheet = CreatePlanarSheet({Rectangle(0, 0, 5, 5, 0)});
        ok = Offset(sheet, 0.25);
        std::printf("     offset sheet %s\n", ok ? "ok" : "failed");
        Report("offset sheet", sheet, Kind::Sheet);
    });
    Run("thicken", [] {
        auto sheet = CreatePlanarSheet({Rectangle(0, 0, 5, 5, 0)});
        bool ok = Thicken(sheet, 0.25, 0);
        std::printf("     thicken front %s\n", ok ? "ok" : "failed");
        Report("thicken front", sheet, Kind::Solid, 6.25);
        auto sheet2 = CreatePlanarSheet({Rectangle(0, 0, 5, 5, 0)});
        ok = Thicken(sheet2, 0.1, 0.1);
        std::printf("     thicken both %s\n", ok ? "ok" : "failed");
        Report("thicken both", sheet2, Kind::Solid, 5);
    });
    Run("round", [] {
        auto box = Box(0, 0, 0, 5, 5, 2);
        bool ok = Round(box, 0.5);
        std::printf("     round %s\n", ok ? "ok" : "failed");
        Report("round", box, Kind::Solid);
    });
    Run("sweep", [] {
        Report("sweep", Sweep({Circle(0, 0, 0, 1)}, {Polyline({0, 0, 0, 0, 0, 3, 5, 0, 3, 8, 10, 10})}), Kind::Solid);
        Report("sweep elbow", Sweep({Circle(0, 0, 0, 1)}, {Polyline({0, 0, 0, 0, 0, 3, 5, 0, 3})}), Kind::Solid, kPi * 8);
        Report("sweep straight", Sweep({Circle(0, 0, 0, 1)}, {Polyline({0, 0, 0, 0, 0, 3})}), Kind::Solid, kPi * 3);
    });
    Run("loft", [] {
        Report("loft", Loft({{Circle(0.5, 0, 5, 1.25)}, {Circle(0, 0, 3, 1)}, {Circle(0, 0, 2, 2)}, {Circle(0, 0, 0, 1.5)}, {Circle(-0.5, 0, -2, 3)}}), Kind::Solid);
        Report("loft cylinder", Loft({{Circle(0, 0, 0, 1)}, {Circle(0, 0, 2, 1)}}), Kind::Solid, kPi * 2);
    });
    Run("separate", [] {
        double const c0[3] = {2, 2, 0}, c1[3] = {2, 2, 5}, c2[3] = {2, 2, -5};
        auto sphere = CreateSphere(c0, 5);
        auto a = CreateSphere(c1, 3), b = CreateSphere(c2, 3);
        Boolean(a, std::move(b), BooleanOp::Unite);
        Boolean(sphere, std::move(a), BooleanOp::Intersect);
        auto parts = Separate(std::move(sphere));
        std::printf("%s separate into %zu bodies\n", parts.size() == 2 ? "ok  " : "FAIL", parts.size());
        if (parts.size() != 2)
            ++s_failures;
        for (auto& part : parts)
            Report("separated part", part, Kind::Solid);
    });
    Run("hollow complex", [] {
        double const c0[3] = {0, 0, 0}, c1[3] = {0, 0, 5}, c2[3] = {0, 0, -5};
        auto body = CreateSphere(c0, 5);
        Boolean(body, CreateSphere(c1, 3), BooleanOp::Subtract);
        Boolean(body, CreateSphere(c2, 3), BooleanOp::Subtract);
        Boolean(body, Box(-4, -4, -5, 4, 4, 5), BooleanOp::Intersect);
        bool ok = Hollow(body, -0.5);
        std::printf("     hollow complex %s\n", ok ? "ok" : "failed");
        Report("hollow complex", body, Kind::Solid);
        auto copy = Copy(*body);
        double const up[3] = {0, 0, 1};
        auto through = Extrude({{Circle(0, 0, -6, 1)}}, up, 12);
        ok = Boolean(body, std::move(through), BooleanOp::Subtract);
        std::printf("     cut hollow complex %s\n", ok ? "ok" : "failed");
        Report("cut hollow complex", body, Kind::Solid);
        auto offAxis = Extrude({{Circle(0.1, 0.05, -6, 1)}}, up, 12);
        ok = Boolean(copy, std::move(offAxis), BooleanOp::Subtract);
        std::printf("     cut hollow complex off axis %s\n", ok ? "ok" : "failed");

        auto again = CreateSphere(c0, 5);
        Boolean(again, CreateSphere(c1, 3), BooleanOp::Subtract);
        Boolean(again, CreateSphere(c2, 3), BooleanOp::Subtract);
        Boolean(again, Box(-4, -4, -5, 4, 4, 5), BooleanOp::Intersect);
        std::vector<uint8_t> saved;
        std::vector<uint32_t> tags;
        double scale;
        Save(*again, tags, saved);
        again = Restore(saved.data(), saved.size(), tags, scale);
        ok = Hollow(again, -0.5);
        std::printf("     restored hollow %s\n", ok ? "ok" : "failed");
        Save(*again, tags, saved);
        again = Restore(saved.data(), saved.size(), tags, scale);
        ok = Boolean(again, Extrude({{Circle(0, 0, -6, 1)}}, up, 12), BooleanOp::Subtract);
        std::printf("     restored cut %s\n", ok ? "ok" : "failed");
    });
    // A failed SMLib shell used to leave state that broke later operations at random.
    Run("repeat cut", [] {
        double const c0[3] = {0, 0, 0}, c1[3] = {0, 0, 5}, c2[3] = {0, 0, -5}, up[3] = {0, 0, 1};
        int failed = 0;
        int const runs = getenv("SMKERNEL_REPEAT") ? atoi(getenv("SMKERNEL_REPEAT")) : 8;
        for (int i = 0; i < runs; ++i) {
            auto body = CreateSphere(c0, 5);
            Boolean(body, CreateSphere(c1, 3), BooleanOp::Subtract);
            Boolean(body, CreateSphere(c2, 3), BooleanOp::Subtract);
            Boolean(body, Box(-4, -4, -5, 4, 4, 5), BooleanOp::Intersect);
            if (!Hollow(body, -0.5) || !Boolean(body, Extrude({{Circle(0, 0, -6, 1)}}, up, 12), BooleanOp::Subtract))
                ++failed;
        }
        std::printf("%s repeat cut: %d of %d failed\n", failed ? "FAIL" : "ok  ", failed, runs);
        s_failures += failed ? 1 : 0;
    });
    std::printf("%d failures\n", s_failures);
    return s_failures ? 1 : 0;
}
