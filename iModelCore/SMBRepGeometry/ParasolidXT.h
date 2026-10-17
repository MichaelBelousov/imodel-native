/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
#pragma once

// A reader for Parasolid transmit (XT) data in the bare binary format, as stored in the geometry
// streams of iModels written with Parasolid. It decodes the node graph using the field layouts of
// the schema the data names, and knows nothing about what the nodes mean.

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ParasolidXT {

struct NodeType;

struct Node {
    NodeType const* m_type = nullptr;
    int32_t m_index = 0;
    std::vector<double> m_values;     //!< every field's values, in field order
    std::vector<uint32_t> m_offsets;  //!< start of each field in m_values, plus the end
    std::unordered_map<int32_t, Node>* m_nodes = nullptr;

    int Type() const;
    char const* Name() const;
    //! The number of scalars in a field, 0 if the node has no such field.
    size_t Count(char const* field) const;
    double Real(char const* field, size_t i = 0) const;
    int Int(char const* field, size_t i = 0) const { return static_cast<int>(Real(field, i)); }
    char Char(char const* field) const { return static_cast<char>(Int(field)); }
    //! The node a pointer field refers to, null for a null pointer or a missing field.
    Node const* Ptr(char const* field, size_t i = 0) const;
    //! The scalars of a field.
    double const* Values(char const* field) const;
    bool Is(int type) const { return Type() == type; }
};

struct Model {
    Model() = default;
    Model(Model const&) = delete;
    Model& operator=(Model const&) = delete;

    std::string m_version;
    std::string m_schema;
    std::unordered_map<int32_t, Node> m_nodes;
    Node const* m_body = nullptr;
};

enum : int {
    BODY = 12, SHELL = 13, FACE = 14, LOOP = 15, EDGE = 16, HALFEDGE = 17, VERTEX = 18, REGION = 19,
    POINT = 29, LINE = 30, CIRCLE = 31, ELLIPSE = 32, INTERSECTION = 38, CHART = 40,
    PLANE = 50, CYLINDER = 51, CONE = 52, SPHERE = 53, TORUS = 54, BLENDED_EDGE = 56, OFFSET_SURF = 60,
    SWEPT_SURF = 67, SPUN_SURF = 68, ATTRIB_DEF = 80, ATTRIBUTE = 81, INT_VALUES = 82, ATT_DEF_ID = 79,
    B_SURFACE = 124, NURBS_SURF = 126, TRIMMED_CURVE = 133, B_CURVE = 134, NURBS_CURVE = 136, SP_CURVE = 137,
};

//! True if the data looks like Parasolid transmit data this reader understands.
bool IsTransmitData(uint8_t const* data, size_t size);

//! Decode transmit data. Fails for text or neutral binary data and for unknown schemas.
bool Read(uint8_t const* data, size_t size, Model& model);

} // namespace ParasolidXT
