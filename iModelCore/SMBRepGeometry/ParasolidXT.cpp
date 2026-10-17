/*---------------------------------------------------------------------------------------------
* Copyright (c) Bentley Systems, Incorporated. All rights reserved.
* See LICENSE.md in the repository root for full copyright notice.
*--------------------------------------------------------------------------------------------*/
#include "ParasolidXT.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <sstream>

namespace ParasolidXT {

/*---------------------------------------------------------------------------------**//**
* The transmitted fields of each node type in schema SCH_12006, the base schema Parasolid writes
* when asked for version 12 data. Each entry is "<type> <name> <variable> <field>:<code><count>..."
* where the code is d int, p pointer, n short, c/u/l byte, f double, v vector, b box, i interval,
* h hvec and t tag; a count of 1 in a variable-length node means the field has the node's length.
+---------------+---------------+---------------+---------------+---------------+------*/
static char const s_schema12006[] = R"(
1 NULLP 0
2 WORKSPACE 1 ws:c1
3 PARTITION 0 current_pmark:p highest_id:d
4 PMARK 0 preceding:p first_following:p next_sibling:p prev_sibling:p n_new_nodes:d n_del_nodes:d
    n_copy_mod_nodes:d delta_key:d delta_is_forward:l id:d
10 ASSEMBLY 0 highest_node_id:d attributes_features:p attribute_chains:p list:p surface:p curve:p point:p
    key:p res_size:f res_linear:f ref_instance:p next:p previous:p state:u owner:p type:u sub_instance:p
11 INSTANCE 0 node_id:d attributes_features:p type:u part:p transform:p assembly:p next_in_part:p
    prev_in_part:p next_of_part:p prev_of_part:p
12 BODY 0 highest_node_id:d attributes_features:p attribute_chains:p surface:p curve:p point:p key:p
    res_size:f res_linear:f ref_instance:p next:p previous:p state:u owner:p body_type:u nom_geom_state:u
    shell:p boundary_surface:p boundary_curve:p boundary_point:p region:p edge:p vertex:p
13 SHELL 0 node_id:d attributes_features:p body:p next:p face:p edge:p vertex:p region:p front_face:p
14 FACE 0 node_id:d attributes_features:p tolerance:f next:p previous:p loop:p shell:p surface:p sense:c
    next_on_surface:p previous_on_surface:p next_front:p previous_front:p front_shell:p
15 LOOP 0 node_id:d attributes_features:p halfedge:p face:p next:p
16 EDGE 0 node_id:d attributes_features:p tolerance:f halfedge:p previous:p next:p curve:p next_on_curve:p
    previous_on_curve:p owner:p
17 HALFEDGE 0 attributes_features:p loop:p forward:p backward:p vertex:p other:p edge:p curve:p next_at_vx:p
    sense:c
18 VERTEX 0 node_id:d attributes_features:p halfedge:p previous:p next:p point:p tolerance:f owner:p
19 REGION 0 node_id:d attributes_features:p body:p next:p previous:p shell:p type:c
29 POINT 0 node_id:d attributes_features:p owner:p next:p previous:p pvec:v
30 LINE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c pvec:v
    direction:v
31 CIRCLE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c centre:v
    normal:v x_axis:v radius:f
32 ELLIPSE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c centre:v
    normal:v x_axis:v major_radius:f minor_radius:f
33 PARABOLA 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c origin:v
    normal:v x_axis:v focal_length:f
34 HYPERBOLA 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c origin:v
    normal:v x_axis:v transverse_radius:f conjugate_radius:f
35 PARACURVE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c seg:d
    cpc:p
36 OBSOLETE_CPC 1 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    vertex_dimension:n segment_count:n order:n segment:p1
37 PATCH_BOUND 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    boundary:c lh_patch:p rh_patch:p
38 INTERSECTION 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    surface:p2 chart:p start:p end:p
39 SILHOUETTE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    analytic_root:c from_infinity:l surface:p start:p end:p eye:v
40 CHART 1 base_parameter:f base_scale:f chart_count:d chordal_error:f angular_error:f parameter_error:f2
    hvec:h1
41 LIMIT 1 type:c hvec:h1
43 BSPLINE_CURVE 0 knot_vector:p vertex_dimension:n vertex_count:d order:n bspline_vertices:p
44 KNOT_VECTOR 1 periodic:l knot_count:d knots:f1
45 BSPLINE_VERTICES 1 vertices:f1
46 OFFSET_CURVE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    surface:p curve:p offset:f
48 CPC 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c bezier:p
    bspline:p
49 OBSOLETE_SP_CURVE 1 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    const_param:l segment_count:n order:n surface:p bezier_vertices:f1
50 PLANE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c pvec:v
    normal:v x_axis:v
51 CYLINDER 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c pvec:v
    axis:v radius:f x_axis:v
52 CONE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c pvec:v axis:v
    radius:f sin_half_angle:f cos_half_angle:f x_axis:v
53 SPHERE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c centre:v
    radius:f axis:v x_axis:v
54 TORUS 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c centre:v
    axis:v major_radius:f minor_radius:f x_axis:v
55 PIPE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c spine:p
    radius:f
56 BLENDED_EDGE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    blend_type:c surface:p2 spine:p range:f2 thumb_weight:f2 boundary:p2 start:p end:p
57 BLENDED_VERTEX 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    blend_type:c surface:p3 sub_surface:p3 boundary:p3 range:f3 thumb_weight:f3 centre:v
58 BLEND_OVERLAP 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    surface:p2 sub_surface:p4 range:f4 thumb_weight:f4 blend_type:c2 overlap_type:c swap_u_v:l
59 BLEND_BOUND 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    boundary:n blend:p
60 OFFSET_SURF 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c check:c
    true_offset:l surface:p offset:f scale:f
61 PARASURF 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c col:d
    row:d cps:p
62 OBSOLETE_CPS 1 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    vertex_dimension:n col_count:n row_count:n u_order:n v_order:n patch:p1
63 SILH_SURF 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    from_infinity:l surface:p eye:v
66 BSPLINE_SURF 0 row_knots:p col_knots:p vertex_dimension:n col_count:d row_count:d u_order:n v_order:n
    bspline_vertices:p
67 SWEPT_SURF 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    section:p sweep:v scale:f
68 SPUN_SURF 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c profile:p
    base:v axis:v start:v end:v start_param:f end_param:f x_axis:v scale:f
69 CPS 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c bezier:p
    bspline:p
70 LIST 0 node_id:d owner:p next:p previous:p list_type:d list_length:d block_length:d size_of_entry:d
    list_block:p finger_block:p finger_index:d notransmit:l
74 POINTER_LIS_BLOCK 1 n_entries:d next_block:p entries:p1
79 ATT_DEF_ID 1 string:c1
80 ATTRIB_DEF 1 next:p identifier:p type_id:d actions:u8 legal_owners:l14 fields:u1
81 ATTRIBUTE 1 node_id:d definition:p owner:p next:p previous:p next_of_type:p previous_of_type:p fields:p1
82 INT_VALUES 1 values:d1
83 REAL_VALUES 1 values:f1
84 CHAR_VALUES 1 values:c1
85 POINT_VALUES 1 values:v1
86 VECTOR_VALUES 1 values:v1
87 AXIS_VALUES 1 values:v1
88 TAG_VALUES 1 values:t1
89 DIRECTION_VALUES 1 values:v1
90 FEATURE 0 node_id:d attributes_features:p owner:p next:p previous:p type:u first_member:p
91 MEMBER_OF_FEATURE 0 dummy_node_id:d owning_feature:p owner:p next:p previous:p next_member:p
    previous_member:p
96 SHORT_VALUES 1 values:n1
97 BOX_VALUES 1 values:b1
100 TRANSFORM 0 node_id:d owner:p next:p previous:p rotation_matrix:f9 translation_vector:v scale:f flag:d
    perspective_vector:v
101 WORLD 0 assembly:p attribute:p body:p transform:p surface:p curve:p point:p alive:l attrib_def:p
    highest_id:d current_id:d
102 KEY 1 string:c1
103 BEZIER_CURVE 1 vertex_dimension:n segment_count:d order:n check:c segment:p1
104 BEZIER_SURF 1 vertex_dimension:n col_count:d row_count:d u_order:n v_order:n check:c patch:p1
110 SET_ELEMENT_TAG 0 next:p forward:p backward:p class:d set:p node:p
111 FACE_SET 0 tag:p next:p class:d he_set:p surfaces:p
112 HALFEDGE_SET 0 tag:p next:p previous:p class:d fa_set:p co_he_set:p
120 PE_SURF 1 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c type:c
    data:p tf:p internal_geom:p1
121 INT_PE_DATA 0 geom_type:d real_array:p int_array:p
122 EXT_PE_DATA 1 key:p real_array:p int_array:p
124 B_SURFACE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c nurbs:p
    data:p
125 SURFACE_DATA 0 original_uint:i original_vint:i extended_uint:i extended_vint:i self_int:u
    original_u_start:c original_u_end:c original_v_start:c original_v_end:c extended_u_start:c
    extended_u_end:c extended_v_start:c extended_v_end:c analytic_form_type:c swept_form_type:c
    spun_form_type:c blend_form_type:c analytic_form:p swept_form:p spun_form:p blend_form:p
126 NURBS_SURF 0 u_periodic:l v_periodic:l u_degree:n v_degree:n n_u_vertices:d n_v_vertices:d u_knot_type:u
    v_knot_type:u n_u_knots:d n_v_knots:d rational:l u_closed:l v_closed:l surface_form:u vertex_dim:n
    bspline_vertices:p u_knot_mult:p v_knot_mult:p u_knots:p v_knots:p
127 KNOT_MULT 1 mult:n1
128 KNOT_SET 1 knots:f1
130 PE_CURVE 1 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c type:c
    data:p tf:p internal_geom:p1
132 PCURVE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c bsp_parms:l
    surface:p bspline:p
133 TRIMMED_CURVE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c
    basis_curve:p point_1:v point_2:v parm_1:f parm_2:f
134 B_CURVE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c nurbs:p
    data:p
135 CURVE_DATA 0 self_int:u analytic_form:p
136 NURBS_CURVE 0 degree:n n_vertices:d vertex_dim:n n_knots:d knot_type:u periodic:l closed:l rational:l
    curve_form:u bspline_vertices:p knot_mult:p knots:p
137 SP_CURVE 0 node_id:d attributes_features:p owner:p next:p previous:p geometric_owner:p sense:c surface:p
    b_curve:p original:p tolerance_to_original:f
141 GEOMETRIC_OWNER 0 owner:p next:p previous:p shared_geometry:p
150 PLANE_FORM 0 pvec:v normal:v
151 CYLINDER_FORM 0 pvec:v axis:v radius:f sense:c
152 CONE_FORM 0 pvec:v axis:v radius:f sin_half_angle:f cos_half_angle:f sense:c
153 SPHERE_FORM 0 centre:v radius:f sense:c
154 TORUS_FORM 0 centre:v axis:v major_radius:f minor_radius:f sense:c
155 SWEPT_FORM 0 sweep:v subtype:c
156 SWEPT_UV_FORM 0 sweep_u:v sweep_v:v subtype_u:c subtype_v:c
157 SPUN_FORM 0 base:v axis:v subtype:c
158 VAR_RADIUS_PIPE_FORM 0 max_radius:f min_radius:f max_radius_bdry:c min_radius_bdry:c
163 HELIX_SU_FORM 0 axis_pt:v axis_dir:v hand:c turns:i pitch:f gap:f tol:f
184 HELIX_CU_FORM 0 axis_pt:v axis_dir:v point:v hand:c turns:i pitch:f tol:f
)";

struct Field {
    std::string m_name;
    char m_code = 0;
    int m_count = 0;
};

struct NodeType {
    int m_type = 0;
    std::string m_name;
    bool m_variable = false;
    std::vector<Field> m_fields;

    int FieldIndex(char const* name) const {
        for (size_t i = 0; i < m_fields.size(); ++i)
            if (m_fields[i].m_name == name)
                return static_cast<int>(i);
        return -1;
    }
};

namespace {

using Schema = std::map<int, NodeType>;

Schema const& Schema12006() {
    static Schema const s_types = [] {
        Schema types;
        std::istringstream in(s_schema12006);
        std::string line;
        NodeType* current = nullptr;
        while (std::getline(in, line)) {
            if (line.empty())
                continue;
            std::istringstream words(line);
            if (line[0] != ' ') {
                NodeType type;
                int variable = 0;
                words >> type.m_type >> type.m_name >> variable;
                type.m_variable = 0 != variable;
                current = &(types[type.m_type] = std::move(type));
            }
            std::string word;
            while (current && words >> word) {
                size_t const colon = word.find(':');
                if (colon == std::string::npos || colon + 1 >= word.size())
                    continue;
                Field field;
                field.m_name = word.substr(0, colon);
                field.m_code = word[colon + 1];
                field.m_count = colon + 2 < word.size() ? std::atoi(word.c_str() + colon + 2) : 0;
                current->m_fields.push_back(field);
            }
        }
        return types;
    }();
    return s_types;
}

//! The number of doubles a field value occupies, by type code; 0 for integer codes.
int Doubles(char code) {
    switch (code) {
        case 'f': return 1;
        case 'i': return 2;
        case 'v': case 'h': return 3;
        case 'b': return 6;
        default: return 0;
    }
}

struct Cursor {
    uint8_t const* m_data;
    uint8_t const* m_end;

    bool Has(size_t n) const { return static_cast<size_t>(m_end - m_data) >= n; }
    template <typename T> bool Get(T& value) {
        if (!Has(sizeof(T)))
            return false;
        std::memcpy(&value, m_data, sizeof(T));
        m_data += sizeof(T);
        return true;
    }
    bool GetString(std::string& value) {
        int32_t length;
        if (!Get(length) || length < 0 || !Has(length))
            return false;
        value.assign(reinterpret_cast<char const*>(m_data), length);
        m_data += length;
        return true;
    }
};

bool ReadValue(Cursor& cursor, char code, std::vector<double>& out) {
    if (int const doubles = Doubles(code)) {
        for (int i = 0; i < doubles; ++i) {
            double value;
            if (!cursor.Get(value))
                return false;
            out.push_back(value);
        }
        return true;
    }
    switch (code) {
        case 'd': case 't': {
            int32_t value;
            if (!cursor.Get(value))
                return false;
            out.push_back(value);
            return true;
        }
        case 'p': case 'n': {
            int16_t value;
            if (!cursor.Get(value) || ('p' == code && value < 0))
                return false; // larger indices use an encoding this reader doesn't know
            out.push_back(value);
            return true;
        }
        case 'c': case 'u': case 'l': {
            uint8_t value;
            if (!cursor.Get(value))
                return false;
            out.push_back(value);
            return true;
        }
        default:
            return false;
    }
}

} // namespace

int Node::Type() const { return m_type ? m_type->m_type : 0; }
char const* Node::Name() const { return m_type ? m_type->m_name.c_str() : ""; }

size_t Node::Count(char const* field) const {
    int const i = m_type ? m_type->FieldIndex(field) : -1;
    return i < 0 ? 0 : m_offsets[i + 1] - m_offsets[i];
}

double const* Node::Values(char const* field) const {
    int const i = m_type ? m_type->FieldIndex(field) : -1;
    return i < 0 || m_offsets[i] == m_offsets[i + 1] ? nullptr : m_values.data() + m_offsets[i];
}

double Node::Real(char const* field, size_t i) const {
    double const* values = Values(field);
    return values && i < Count(field) ? values[i] : 0.0;
}

Node const* Node::Ptr(char const* field, size_t i) const {
    if (nullptr == m_nodes || i >= Count(field))
        return nullptr;
    auto found = m_nodes->find(static_cast<int32_t>(Values(field)[i]));
    return found == m_nodes->end() ? nullptr : &found->second;
}

bool IsTransmitData(uint8_t const* data, size_t size) {
    static char const kPrefix[] = ": TRANSMIT FILE";
    return nullptr != data && size > 5 + sizeof(kPrefix) && 'B' == data[0] && 0 == std::memcmp(data + 5, kPrefix, sizeof(kPrefix) - 1);
}

bool Read(uint8_t const* data, size_t size, Model& model) {
    if (!IsTransmitData(data, size))
        return false;
    Cursor cursor{data + 1, data + size};
    int32_t userFieldSize;
    if (!cursor.GetString(model.m_version) || !cursor.GetString(model.m_schema) || !cursor.Get(userFieldSize) || 0 != userFieldSize)
        return false;
    // Only data in the version 12 schema, which Bentley's Parasolid admin writes, is understood.
    if (model.m_schema.size() < 6 || 0 != model.m_schema.compare(model.m_schema.size() - 6, 6, "_12006"))
        return false;
    Schema const& schema = Schema12006();

    model.m_nodes.clear();
    while (true) {
        int16_t typeCode;
        if (!cursor.Get(typeCode))
            return false;
        if (1 == typeCode)
            break;
        auto type = schema.find(typeCode);
        if (type == schema.end())
            return false;
        int32_t length = 1;
        if (type->second.m_variable && (!cursor.Get(length) || length < 0))
            return false;
        int16_t index;
        if (!cursor.Get(index) || index < 0)
            return false;

        Node node;
        node.m_type = &type->second;
        node.m_index = index;
        node.m_nodes = &model.m_nodes;
        for (Field const& field : type->second.m_fields) {
            node.m_offsets.push_back(static_cast<uint32_t>(node.m_values.size()));
            int const count = 1 == field.m_count && type->second.m_variable ? length : std::max(1, field.m_count);
            for (int i = 0; i < count; ++i) {
                if (!ReadValue(cursor, field.m_code, node.m_values))
                    return false;
            }
        }
        node.m_offsets.push_back(static_cast<uint32_t>(node.m_values.size()));
        if (BODY == typeCode && nullptr == model.m_body)
            model.m_body = &(model.m_nodes[index] = std::move(node));
        else
            model.m_nodes[index] = std::move(node);
    }
    return nullptr != model.m_body;
}

} // namespace ParasolidXT
