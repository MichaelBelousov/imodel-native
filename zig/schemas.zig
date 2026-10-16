//! Generated from bis-schemas SchemaInventory.json: the latest released version of each schema the
//! addon loads, and where it goes under Assets/ECSchemas.

pub const Schema = struct { src: []const u8, dest: []const u8 };

pub const schemas = [_]Schema{
    .{ .src = "System/Released/SchemaUpgradeCustomAttributes.01.00.00.ecschema.xml", .dest = "Standard/SchemaUpgradeCustomAttributes.01.00.00.ecschema.xml" },
    .{ .src = "System/Released/CoreCustomAttributes.01.00.04.ecschema.xml", .dest = "Standard/CoreCustomAttributes.01.00.04.ecschema.xml" },
    .{ .src = "System/Released/BisCustomAttributes.01.00.00.ecschema.xml", .dest = "Standard/BisCustomAttributes.01.00.00.ecschema.xml" },
    .{ .src = "System/Released/SchemaLocalizationCustomAttributes.01.00.00.ecschema.xml", .dest = "Standard/SchemaLocalizationCustomAttributes.01.00.00.ecschema.xml" },
    .{ .src = "System/Released/Units.01.00.12.ecschema.xml", .dest = "Standard/Units.01.00.12.ecschema.xml" },
    .{ .src = "System/Released/Formats.01.00.00.ecschema.xml", .dest = "Standard/Formats.01.00.00.ecschema.xml" },
    .{ .src = "Domains/0-Core/Released/BisCore.01.00.26.ecschema.xml", .dest = "Dgn/BisCore.ecschema.xml" },
    .{ .src = "Domains/0-Core/Released/Generic.01.00.05.ecschema.xml", .dest = "Dgn/Generic.ecschema.xml" },
    .{ .src = "Domains/0-Core/Released/Analytical.01.00.01.ecschema.xml", .dest = "Domain/Analytical.ecschema.xml" },
    .{ .src = "Domains/0-Core/Released/Functional.01.00.04.ecschema.xml", .dest = "Domain/Functional.ecschema.xml" },
    .{ .src = "Domains/1-Common/LinearReferencing/Released/LinearReferencing.02.00.03.ecschema.xml", .dest = "Domain/LinearReferencing.ecschema.xml" },
    .{ .src = "Domains/0-Core/Released/PhysicalMaterial.01.00.02.ecschema.xml", .dest = "Domain/PhysicalMaterial.ecschema.xml" },
    .{ .src = "Domains/4-Application/PresentationRules/Released/PresentationRules.01.00.01.ecschema.xml", .dest = "Domain/PresentationRules.ecschema.xml" },
};
