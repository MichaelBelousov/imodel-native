/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
#pragma once

// A narrow interface to SMLib. SMLib's class layouts depend on its own build macros, which clash
// with Bentley's, so only SmKernel.cpp includes SMLib headers and this header uses neither.
//
// SMLib is not thread-safe: hold Mutex() around every call into this namespace.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace SmKernel {

std::recursive_mutex& Mutex();

struct Body;
void Delete(Body*);
struct BodyDeleter { void operator()(Body* body) const { Delete(body); } };
using BodyPtr = std::unique_ptr<Body, BodyDeleter>;

enum class Kind { Solid, Sheet, Wire };

//! Read a body written by Save, along with its face tags. Returns null for anything else,
//! including Parasolid data.
BodyPtr Restore(uint8_t const* data, size_t size, std::vector<uint32_t>& faceTags);
//! Serialize a body with one caller-defined tag per face (e.g. a symbology index), since SMLib
//! faces carry no persistent attributes here. Tags may be empty.
bool Save(Body const& body, std::vector<uint32_t> const& faceTags, std::vector<uint8_t>& out);
BodyPtr Copy(Body const& body);

Kind GetKind(Body const& body);
size_t GetFaceCount(Body const& body);
//! The tight axis-aligned box of the body. Expensive; cache it.
bool GetRange(Body const& body, double low[3], double high[3]);
bool HasCurvedFaceOrEdge(Body const& body);
//! Apply a row-major 3x4 affine transform to the body's geometry.
bool Transform(Body& body, double const matrix[3][4]);

struct TessellationOptions {
    double chordTolerance = 0.0;    //!< 0 = unconstrained
    double angleToleranceDegrees = 0.0; //!< 0 = unconstrained
    double maxEdgeLength = 0.0;     //!< 0 = unconstrained
};

//! An indexed polygon mesh. Normals and params are per facet corner.
struct Mesh {
    std::vector<double> points;     //!< xyz triples
    std::vector<double> normals;    //!< xyz triples, one per entry of `indices`
    std::vector<double> params;     //!< uv pairs on the face's surface, one per entry of `indices`
    std::vector<uint32_t> facetSizes;
    std::vector<uint32_t> indices;  //!< 0-based point indices, facet after facet
    std::vector<uint32_t> facetFaces; //!< for each facet, the index of the body face it came from
};
bool Tessellate(Body const& body, TessellationOptions const& options, Mesh& mesh);

//! Volume properties with unit density. Moments are about axes through the centroid, products are
//! raw integrals about the centroid.
struct MassProperties {
    double volume = 0.0;
    double area = 0.0;
    double centroid[3] = {};
    double moments[3] = {};  //!< Ixx, Iyy, Izz
    double products[3] = {}; //!< Ixy, Ixz, Iyz
};
bool GetMassProperties(Body const& body, double relativeAccuracy, MassProperties& props);

// Primitives, in local coordinates.
BodyPtr CreateBox(double const origin[3], double const size[3]);
BodyPtr CreateSphere(double const center[3], double radius);
//! A cylinder or truncated cone along the segment from base to top.
BodyPtr CreateCone(double const base[3], double const top[3], double baseRadius, double topRadius);
BodyPtr CreateTorus(double const center[3], double const axis[3], double majorRadius, double minorRadius);

enum class BooleanOp { Unite, Subtract, Intersect };
//! Combine tool into target. On success target holds the result.
bool Boolean(BodyPtr& target, BodyPtr tool, BooleanOp op);

} // namespace SmKernel
