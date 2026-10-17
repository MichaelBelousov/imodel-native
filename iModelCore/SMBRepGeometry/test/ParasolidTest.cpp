/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
// Rebuilds the Parasolid transmit data in each file given and prints one tab-separated line per
// file: name, status, kind, face count, volume, area, range, triangle count, milliseconds, the
// volume of a fine tessellation, log. Lengths are in the data's units.
// PARASOLID_TEST_CHORD sets the fine tessellation's chord tolerance relative to the body's size
// (default 1e-4), PARASOLID_TEST_ACCURACY the relative accuracy of the volume (default 1e-4), and
// PARASOLID_TEST_VERBOSE adds timings, SMLib's validity checks and a line per face.
#include "../SmKernelImpl.h"

#include <SmAssertArray.h>
#include <SmEdgeuse.h>
#include <SmFace.h>
#include <SmLoop.h>
#include <SmSurface.h>

#include <chrono>
#include <typeinfo>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <cstdlib>
#include <string>

using namespace SmKernel;

int main(int argc, char** argv) {
    int failures = 0;
    for (int i = 1; i < argc; ++i) {
        std::ifstream in(argv[i], std::ios::binary);
        std::vector<uint8_t> data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        std::string log;
        std::vector<uint32_t> tags;
        auto start = std::chrono::steady_clock::now();
        double scale = 1.0;
        BodyPtr body = RestoreParasolid(data.data(), data.size(), tags, scale, &log);
        auto lap = [&](char const* what) {
            if (std::getenv("PARASOLID_TEST_VERBOSE"))
                std::printf("%s %.1f ms\n", what, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
        };
        lap("restore");
        std::string kind = "-";
        size_t faces = 0, triangles = 0;
        double meshVolume = 0.0;
        double low[3] = {0, 0, 0}, high[3] = {0, 0, 0};
        MassProperties props;
        props.volume = props.area = 0.0;
        if (body) {
            Kind const k = GetKind(*body);
            kind = Kind::Solid == k ? "solid" : Kind::Sheet == k ? "sheet" : "wire";
            faces = GetFaceCount(*body);
            lap("faces");
            GetRange(*body, low, high);
            lap("range");
            if (Kind::Solid == k)
                GetMassProperties(*body, std::getenv("PARASOLID_TEST_ACCURACY") ? std::atof(std::getenv("PARASOLID_TEST_ACCURACY")) : 1.0e-4, props);
            lap("mass");
            TessellationOptions options;
            options.angleToleranceDegrees = 20.0;
            Mesh mesh;
            if (Tessellate(*body, options, mesh))
                for (uint32_t size : mesh.facetSizes)
                    triangles += size - 2;
            lap("tessellate");
            if (std::getenv("PARASOLID_TEST_VERBOSE")) {
                SmAssertArray reports;
                std::printf("valid %d\n", body->m_brep->AssertValid(&reports, SM_LEVEL_2) ? 1 : 0);
                for (ULONG r = 0; r < reports.GetSize() && r < 20; ++r)
                    if (!reports[r]->m_bOK)
                        std::printf("  assert %s: %s\n", reports[r]->m_pName ? reports[r]->m_pName : "", reports[r]->m_pMessage ? reports[r]->m_pMessage : "");
                SmTArray<SmFace*> smFaces;
                body->m_brep->GetFaces(smFaces);
                for (ULONG f = 0; f < smFaces.GetSize(); ++f) {
                    SmSurface* surface = smFaces[f]->GetSurface();
                    SmExtent2d const domain = smFaces[f]->GetUVDomain();
                    SmTArray<SmLoop*> loops;
                    smFaces[f]->GetLoops(loops);
                    size_t uses = 0, missing = 0;
                    for (ULONG l = 0; l < loops.GetSize(); ++l) {
                        SmTArray<SmEdgeuse*> edgeuses;
                        loops[l]->GetEdgeuses(edgeuses);
                        uses += edgeuses.GetSize();
                        for (ULONG e = 0; e < edgeuses.GetSize(); ++e)
                            missing += nullptr == edgeuses[e]->GetUVTrimCurvePointer();
                    }
                    size_t facets = 0;
                    for (uint32_t face : mesh.facetFaces)
                        facets += face == f;
                    std::printf("  face %lu %s planar=%d domain=[%g %g]x[%g %g] loops=%lu uses=%zu notrim=%zu facets=%zu\n", (unsigned long) f, surface ? typeid(*surface).name() : "-",
                                surface && surface->IsPlanar() ? 1 : 0, domain.GetUMin(), domain.GetUMax(), domain.GetVMin(), domain.GetVMax(), (unsigned long) loops.GetSize(), uses, missing, facets);
                }
            }
            double const diagonal = std::sqrt((high[0] - low[0]) * (high[0] - low[0]) + (high[1] - low[1]) * (high[1] - low[1]) + (high[2] - low[2]) * (high[2] - low[2]));
            options.chordTolerance = (std::getenv("PARASOLID_TEST_CHORD") ? std::atof(std::getenv("PARASOLID_TEST_CHORD")) : 1.0e-4) * diagonal;
            options.angleToleranceDegrees = 5.0;
            if (Tessellate(*body, options, mesh)) {
                double const* c = low;
                for (size_t f = 0, start = 0; f < mesh.facetSizes.size(); start += mesh.facetSizes[f++]) {
                    double const* a = &mesh.points[3 * mesh.indices[start]];
                    for (uint32_t k = 1; k + 1 < mesh.facetSizes[f]; ++k) {
                        double const* p = &mesh.points[3 * mesh.indices[start + k]];
                        double const* q = &mesh.points[3 * mesh.indices[start + k + 1]];
                        double const u[3] = {a[0] - c[0], a[1] - c[1], a[2] - c[2]}, v[3] = {p[0] - c[0], p[1] - c[1], p[2] - c[2]}, w[3] = {q[0] - c[0], q[1] - c[1], q[2] - c[2]};
                        meshVolume += (u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) + u[2] * (v[0] * w[1] - v[1] * w[0])) / 6.0;
                    }
                }
            }
            lap("fine tessellate");
        } else {
            ++failures;
        }
        double const ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        for (char& c : log)
            if ('\n' == c || '\t' == c)
                c = ';';
        double const k = scale, k3 = k * k * k;
        props.volume /= k3;
        props.area /= k * k;
        meshVolume /= k3;
        for (int j = 0; j < 3; ++j)
            low[j] /= k, high[j] /= k;
        std::printf("%s\t%s\t%s\t%zu\t%.9g\t%.9g\t%.9g\t%.9g\t%.9g\t%.9g\t%.9g\t%.9g\t%zu\t%.1f\t%.9g\t%s\n", argv[i], body ? "ok" : "fail", kind.c_str(), faces,
                    props.volume, props.area, low[0], low[1], low[2], high[0], high[1], high[2], triangles, ms, meshVolume, log.c_str());
        std::fflush(stdout);
    }
    return failures > 0 ? 1 : 0;
}
