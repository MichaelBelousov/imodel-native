//! Zig build for imodel-native on Linux. Replaces BentleyBuild/bmake for the libraries and the
//! Node addon. Source lists come from the bmake files (see zig/sources.zig).

const std = @import("std");
const src = @import("zig/sources.zig");
const bis_schemas = @import("zig/schemas.zig").schemas;

const LazyPath = std.Build.LazyPath;
const Compile = std.Build.Step.Compile;

const Ctx = struct {
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    /// Headers staged the way Bentley's build context delivers them (`<icu4c/...>`, `<zlib/...>`).
    staged: LazyPath,
    zlib: *std.Build.Dependency,
    libpng: *std.Build.Dependency,
    pugixml: *std.Build.Dependency,

    const Lib = struct {
        sources: []const []const u8 = &.{},
        /// Sources from a dependency package instead of this repo.
        dep_sources: ?struct { dep: *std.Build.Dependency, files: []const []const u8 } = null,
        defines: []const []const u8 = &.{},
        includes: []const []const u8 = &.{},
        /// Include iModelCore's public headers and the staged vendor headers.
        core: bool = true,
        c_flags: []const []const u8 = &.{},
        cpp_flags: []const []const u8 = &.{},
        link: []const *Compile = &.{},
    };

    fn lib(ctx: *const Ctx, name: []const u8, spec: Lib) *Compile {
        const b = ctx.b;
        const mod = b.createModule(.{
            .target = ctx.target,
            .optimize = ctx.optimize,
            .link_libc = true,
            .link_libcpp = true,
            .pic = true,
            .sanitize_c = .off,
        });
        for (spec.includes) |inc| mod.addIncludePath(b.path(inc));
        if (spec.core) ctx.addCoreIncludes(mod);
        for (spec.defines) |d| addDefine(mod, d);
        if (ctx.optimize != .Debug) mod.addCMacro("NDEBUG", "1");

        var c_flags: std.ArrayList([]const u8) = .empty;
        c_flags.appendSlice(b.allocator, &.{ "-std=gnu11", "-w" }) catch @panic("OOM");
        c_flags.appendSlice(b.allocator, spec.c_flags) catch @panic("OOM");
        var cpp_flags: std.ArrayList([]const u8) = .empty;
        cpp_flags.appendSlice(b.allocator, &.{ "-std=c++20", "-w", "-fno-sanitize=undefined" }) catch @panic("OOM");
        cpp_flags.appendSlice(b.allocator, spec.cpp_flags) catch @panic("OOM");

        addSources(mod, b.path("."), spec.sources, c_flags.items, cpp_flags.items);
        if (spec.dep_sources) |ds| addSources(mod, ds.dep.path("."), ds.files, c_flags.items, cpp_flags.items);
        for (spec.link) |l| mod.linkLibrary(l);

        const l = b.addLibrary(.{ .name = name, .linkage = .static, .root_module = mod });
        const install = b.addInstallArtifact(l, .{});
        b.getInstallStep().dependOn(&install.step);
        b.step(name, b.fmt("Build {s}", .{name})).dependOn(&install.step);
        return l;
    }

    fn addCoreIncludes(ctx: *const Ctx, mod: *std.Build.Module) void {
        const b = ctx.b;
        mod.addIncludePath(ctx.staged);
        mod.addIncludePath(ctx.staged.path(b, "icu4c"));
        for (core_includes) |inc| mod.addIncludePath(b.path(inc));
    }
};

/// Public headers of every iModelCore module, plus header-only vendor libraries.
const core_includes = [_][]const u8{
    "iModelCore/Bentley/PublicAPI",
    "iModelCore/Bentley/BeRapidJson/PublicAPI",
    "iModelCore/Bentley/BeJpeg/PublicAPI",
    "iModelCore/Bentley/v8serial/PublicAPI",
    "iModelCore/ecobjects/PublicApi",
    "iModelCore/GeomLibs/PublicAPI",
    "iModelCore/BeSQLite/PublicAPI",
    "iModelCore/ECDb/PublicAPI",
    "iModelCore/Units/PublicAPI",
    "iModelCore/iModelPlatform/PublicAPI",
    "iModelCore/ECPresentation/PublicAPI",
    "iModelCore/GeoCoord/PublicAPI",
    "iModelCore/Visualization/PublicAPI",
    "node-addon-api",
    "iModelCore/libsrc/rapidjson/vendor/include",
    "iModelCore/libsrc/compress/VendorAPI",
    "iModelCore/libsrc/google_re2",
    "iModelCore/libsrc/facebook/VendorApi",
    "iModelCore/libsrc/flatbuffers/source/include",
    "iModelCore/libsrc/jsoncpp/include",
};

fn addDefine(mod: *std.Build.Module, d: []const u8) void {
    if (std.mem.indexOfScalar(u8, d, '=')) |eq| {
        mod.addCMacro(d[0..eq], d[eq + 1 ..]);
    } else {
        mod.addCMacro(d, "1");
    }
}

fn addSources(mod: *std.Build.Module, root: LazyPath, files: []const []const u8, c_flags: []const []const u8, cpp_flags: []const []const u8) void {
    for (files) |f| {
        const is_c = std.mem.endsWith(u8, f, ".c");
        mod.addCSourceFile(.{ .file = root.path(mod.owner, f), .flags = if (is_c) c_flags else cpp_flags });
    }
}

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const zlib = b.dependency("zlib", .{});
    const libpng = b.dependency("libpng", .{});
    const pugixml = b.dependency("pugixml", .{});

    const ctx: Ctx = .{
        .b = b,
        .target = target,
        .optimize = optimize,
        .staged = stageHeaders(b, zlib, libpng, pugixml),
        .zlib = zlib,
        .libpng = libpng,
        .pugixml = pugixml,
    };
    const third = thirdParty(&ctx);
    const core = coreLibs(&ctx, third);
    addon(&ctx, third, core);
}

/// Headers that Bentley's prewire steps link into `$(BuildContext)VendorAPI` under a different
/// prefix than their source location.
fn stageHeaders(b: *std.Build, zlib: *std.Build.Dependency, libpng: *std.Build.Dependency, pugixml: *std.Build.Dependency) LazyPath {
    const wf = b.addWriteFiles();
    const h_only: std.Build.Step.WriteFile.Directory.Options = .{ .include_extensions = &.{".h"} };
    const icu = "iModelCore/libsrc/icu4c/vendor/source/";
    for ([_][]const u8{ "common", "i18n", "io" }) |dir| {
        _ = wf.addCopyDirectory(b.path(b.fmt("{s}{s}/unicode", .{ icu, dir })), "icu4c/unicode", h_only);
    }
    _ = wf.addCopyDirectory(b.path(icu ++ "layoutex/layout"), "icu4c/layout", h_only);

    _ = wf.addCopyFile(zlib.path("zlib.h"), "zlib/zlib.h");
    _ = wf.addCopyFile(zlib.path("zconf.h"), "zlib/zconf.h");

    _ = wf.addCopyFile(libpng.path("png.h"), "png/png.h");
    _ = wf.addCopyFile(libpng.path("pngconf.h"), "png/pngconf.h");
    _ = wf.addCopyFile(libpng.path("scripts/pnglibconf.h.prebuilt"), "png/pnglibconf.h");

    _ = wf.addCopyDirectory(b.path("iModelCore/libsrc/csmap/Include"), "csmap", .{});

    _ = wf.addCopyFile(pugixml.path("src/pugixml.hpp"), "pugixml/src/pugixml.hpp");
    _ = wf.addCopyFile(pugixml.path("src/pugiconfig.hpp"), "pugixml/src/pugiconfig.hpp");
    _ = wf.addCopyFile(b.path("iModelCore/libsrc/pugixml/src/BePugiXml.h"), "pugixml/src/BePugiXml.h");
    _ = wf.addCopyFile(b.path("iModelCore/libsrc/pugixml/src/BePugiXmlHelper.h"), "pugixml/src/BePugiXmlHelper.h");

    const jpeg = "iModelCore/libsrc/libjpegturbo/vendor/src/";
    for ([_][]const u8{ "turbojpeg.h", "jpeglib.h", "jconfig.h", "jmorecfg.h", "jerror.h" }) |h| {
        _ = wf.addCopyFile(b.path(b.fmt("{s}{s}", .{ jpeg, h })), b.fmt("libjpeg-turbo/{s}", .{h}));
    }
    return wf.getDirectory();
}

const ThirdParty = struct {
    zlib: *Compile,
    png: *Compile,
    lzma: *Compile,
    snappy: *Compile,
    pugixml: *Compile,
    icu: *Compile,
    folly: *Compile,
    csmap: *Compile,
    freetype: *Compile,
    re2: *Compile,
    jpeg_turbo: *Compile,
};

fn thirdParty(ctx: *const Ctx) ThirdParty {
    const zlib = ctx.lib("BeZlib", .{
        .core = false,
        .dep_sources = .{ .dep = ctx.zlib, .files = &.{
            "adler32.c", "compress.c", "crc32.c",   "deflate.c", "gzclose.c",  "gzlib.c", "gzread.c",
            "gzwrite.c", "infback.c",  "inffast.c", "inflate.c", "inftrees.c", "trees.c", "uncompr.c",
            "zutil.c",
        } },
        .defines = &.{ "HAVE_UNISTD_H", "_LARGEFILE64_SOURCE" },
    });
    const png = ctx.lib("BePng", .{
        .core = false,
        .dep_sources = .{ .dep = ctx.libpng, .files = &.{
            "png.c",      "pngerror.c", "pngget.c", "pngmem.c",   "pngpread.c", "pngread.c",  "pngrio.c",
            "pngrtran.c", "pngrutil.c", "pngset.c", "pngtrans.c", "pngwio.c",   "pngwrite.c", "pngwtran.c",
            "pngwutil.c",
        } },
        .defines = &.{ "PNG_ARM_NEON_OPT=0", "PNG_INTEL_SSE_OPT=0", "PNG_MIPS_MSA_OPT=0", "PNG_MIPS_MMI_OPT=0", "PNG_POWERPC_VSX_OPT=0", "PNG_LOONGARCH_LSX_OPT=0", "PNG_RISCV_RVV_OPT=0" },
        .link = &.{zlib},
    });
    png.root_module.addIncludePath(ctx.staged.path(ctx.b, "png"));
    png.root_module.addIncludePath(ctx.staged.path(ctx.b, "zlib"));

    const lzma = ctx.lib("lzma", .{
        .core = false,
        .sources = &src.lzma,
        .defines = &.{"_7ZIP_ST"},
        .includes = &.{ "iModelCore/libsrc/compress/VendorAPI/liblzma", "iModelCore/libsrc/compress/lzma" },
    });
    const snappy = ctx.lib("snappy", .{
        .core = false,
        .sources = &src.snappy,
        .defines = &.{"HAVE_CONFIG_H"},
        .includes = &.{ "iModelCore/libsrc/compress/VendorAPI/snappy", "iModelCore/libsrc/compress/snappy" },
    });
    const pugi = ctx.lib("iTwinPugixml", .{
        .sources = &src.iTwinPugixml,
        .dep_sources = .{ .dep = ctx.pugixml, .files = &.{"src/pugixml.cpp"} },
        .defines = &.{"__PUGIXML_BUILD__"},
    });
    const icu = ctx.lib("iTwinIcu4c", .{
        .core = false,
        .sources = &src.iTwinIcu4c,
        .defines = &.{
            "U_ATTRIBUTE_DEPRECATED=",   "U_USING_ICU_NAMESPACE=0", "U_NO_DEFAULT_INCLUDE_UTF_HEADERS=1", "U_PLATFORM=U_PF_LINUX",
            "U_STATIC_IMPLEMENTATION",   "U_COMMON_IMPLEMENTATION", "U_I18N_IMPLEMENTATION",              "U_LAYOUT_IMPLEMENTATION",
            "U_LAYOUTEX_IMPLEMENTATION", "U_IO_IMPLEMENTATION",     "U_TOOLUTIL_IMPLEMENTATION",
        },
        .includes = &.{
            "iModelCore/libsrc/icu4c/vendor/source",          "iModelCore/libsrc/icu4c/vendor/source/common",
            "iModelCore/libsrc/icu4c/vendor/source/i18n",     "iModelCore/libsrc/icu4c/vendor/source/io",
            "iModelCore/libsrc/icu4c/vendor/source/layoutex", "iModelCore/libsrc/icu4c/vendor/source/tools/toolutil",
        },
    });
    const folly = ctx.lib("iTwinFolly", .{
        .sources = &src.iTwinFolly,
        .defines = &.{"__BE_FOLLY_DLL__"},
    });
    const csmap = ctx.lib("iTwinCsmapStatic", .{
        .core = false,
        .sources = &src.iTwinCsmapStatic,
        .defines = &.{ "TOTAL_SPECIAL", "GEOCOORD_ENHANCEMENT", "BENTLEY_TOOLSUBS_MEMALLOC", "__CPP__" },
        .includes = &.{"iModelCore/libsrc/csmap/Include"},
        .c_flags = &.{"-Wno-error=implicit-function-declaration"},
    });
    const freetype = ctx.lib("iTwinFreetype2", .{
        .core = false,
        .sources = &src.iTwinFreetype2,
        .defines = &.{ "FT2_BUILD_LIBRARY", "FT_CONFIG_OPTION_SYSTEM_ZLIB" },
        .includes = &.{"iModelCore/libsrc/freetype2/vendor/include"},
        .link = &.{zlib},
    });
    freetype.root_module.addIncludePath(ctx.staged.path(ctx.b, "zlib"));
    const re2 = ctx.lib("iTwin_google_re2", .{
        .core = false,
        .sources = &src.iTwin_google_re2,
        .defines = &.{"NOMINMAX"},
        .includes = &.{"iModelCore/libsrc/google_re2"},
    });
    const jpeg_turbo = ctx.lib("BeLibJpegTurbo", .{
        .core = false,
        .sources = &src.BeLibJpegTurbo,
        .defines = &.{"NO_SIMD=1"},
        .includes = &.{ "iModelCore/libsrc/libjpegturbo/vendor/src", "iModelCore/libsrc/libjpegturbo/vendor/platforms/linux" },
        .c_flags = &.{"-fno-stack-protector"},
    });
    return .{
        .zlib = zlib,
        .png = png,
        .lzma = lzma,
        .snappy = snappy,
        .pugixml = pugi,
        .icu = icu,
        .folly = folly,
        .csmap = csmap,
        .freetype = freetype,
        .re2 = re2,
        .jpeg_turbo = jpeg_turbo,
    };
}

const Core = struct {
    bentley: *Compile,
    jpeg: *Compile,
    sqlite: *Compile,
    units: *Compile,
    ecobjects: *Compile,
    geocoord: *Compile,
    geom: *Compile,
    geom_serialization: *Compile,
    ecdb: *Compile,
    presentation: *Compile,
    platform: *Compile,
    visualization: *Compile,
    brep: *Compile,
};

/// Defines every iModelCore module is built with. The private toolchain supplied these.
const core_defines = [_][]const u8{ "BETHREAD_USE_PTHREAD", "BENTLEY_HAVE_VARIANT", "USE_GTEST", "BETEST_NO_INCLUDE_GTEST" };

fn coreDefines(b: *std.Build, extra: []const []const u8) []const []const u8 {
    return std.mem.concat(b.allocator, []const u8, &.{ &core_defines, extra }) catch @panic("OOM");
}

fn coreLibs(ctx: *const Ctx, third: ThirdParty) Core {
    const b = ctx.b;
    const bentley = ctx.lib("iTwinBentley", .{
        .sources = &src.iTwinBentley,
        .defines = coreDefines(b, &.{ "__BENTLEYDLL_BUILD__", "BC__RWSTD_LIB_SRC" }),
        .includes = &.{ "iModelCore/Bentley/Bentley", "iModelCore/Bentley/Bentley/nonport" },
        .link = &.{third.icu},
    });
    const jpeg = ctx.lib("iTwinJpeg", .{
        .sources = &src.iTwinJpeg,
        .defines = coreDefines(b, &.{}),
        .link = &.{third.jpeg_turbo},
    });
    const sqlite = ctx.lib("iTwinSQLite", .{
        .sources = &src.iTwinSQLite,
        .defines = coreDefines(b, &.{ "__BE_SQLITE_HOST_DLL__", "SQLITE_BCV_CURL_HANDLE_CONFIG=besqlite_bcv_curl_handle_config" }),
        .includes = &.{"iModelCore/BeSQLite"},
        .link = &.{ third.zlib, third.lzma, third.snappy, third.re2, third.icu },
    });
    sqlite.root_module.addIncludePath(prgHeader(b));
    const units = ctx.lib("iTwinUnits", .{
        .sources = &src.iTwinUnits,
        .defines = coreDefines(b, &.{"__UNITS_BUILD__"}),
        .includes = &.{ "iModelCore/Units/PublicAPI/Units", "iModelCore/Units/src" },
    });
    const ecobjects = ctx.lib("iTwinECObjects", .{
        .sources = &src.iTwinECObjects,
        .defines = coreDefines(b, &.{"__ECOBJECTS_BUILD__"}),
        .includes = &.{"iModelCore/ecobjects/src"},
        .link = &.{third.pugixml},
    });
    const geocoord = ctx.lib("iTwinGeoCoord", .{
        .sources = &src.iTwinGeoCoord,
        .defines = coreDefines(b, &.{ "STANDALONE_BSI", "__BASEGEOCOORD_BUILD__", "TOTAL_SPECIAL", "GEOCOORD_ENHANCEMENT" }),
        .link = &.{ third.csmap, third.pugixml },
    });
    const geom = ctx.lib("iTwinGeom", .{
        .sources = &src.iTwinGeom,
        .defines = coreDefines(b, &.{"jmdlgeom_internal"}),
        .includes = &.{ "iModelCore/GeomLibs/geom/build/BentleyGeom", "iModelCore/GeomLibs/geom/src/regions" },
    });
    const geom_serialization = ctx.lib("iTwinGeomSerialization", .{
        .sources = &src.iTwinGeomSerialization,
        .defines = coreDefines(b, &.{ "__GEOMLIBS_SERIALIZATION_BUILD__", "BGFB_INTERNAL", "BuildCGJsonValueBuilder" }),
        .includes = &.{"iModelCore/GeomLibs/serialization/src"},
    });
    const ecdb = ctx.lib("iTwinSQLiteEC", .{
        .sources = &src.iTwinSQLiteEC,
        .defines = coreDefines(b, &.{"__ECDB_BUILD__"}),
        .includes = &.{"iModelCore/ECDb/ECDb"},
    });
    const presentation = ctx.lib("iTwinECPresentation", .{
        .sources = &src.iTwinECPresentation,
        .defines = coreDefines(b, &.{"__ECPRESENTATION_DLL_BUILD__"}),
        .includes = &.{"iModelCore/ECPresentation/Source"},
    });
    const platform = ctx.lib("iTwinPlatform", .{
        .sources = &src.iTwinPlatform,
        .defines = coreDefines(b, &.{ "__DGNPLATFORM_BUILD__", "NOMINMAX" }),
        .includes = &.{ "iModelCore/iModelPlatform", "iModelCore/iModelPlatform/PrivateApi", "iModelCore/libsrc/freetype2/vendor/include" },
    });
    const meshoptimizer = b.dependency("meshoptimizer", .{});
    const visualization = ctx.lib("iTwinVisualization", .{
        .sources = &.{
            "iModelCore/Visualization/Visualization.cpp",
            "iModelCore/Visualization/ElementMesher.cpp",
            "iModelCore/Visualization/ImdlWriter.cpp",
        },
        .dep_sources = .{ .dep = meshoptimizer, .files = &.{
            "src/allocator.cpp",
            "src/simplifier.cpp",
            "src/vfetchoptimizer.cpp",
            "src/indexgenerator.cpp",
        } },
        .defines = coreDefines(b, &.{}),
    });
    visualization.root_module.addIncludePath(meshoptimizer.path("src"));
    const brep = smlibBRep(ctx);
    return .{
        .bentley = bentley,
        .jpeg = jpeg,
        .sqlite = sqlite,
        .units = units,
        .ecobjects = ecobjects,
        .geocoord = geocoord,
        .geom = geom,
        .geom_serialization = geom_serialization,
        .ecdb = ecdb,
        .presentation = presentation,
        .platform = platform,
        .visualization = visualization,
        .brep = brep,
    };
}

/// `<prg.h>`: version macros the private build generated from the package version.
fn prgHeader(b: *std.Build) LazyPath {
    const version = std.mem.trim(u8, @embedFile("iModelJsNodeAddon/package_version.txt"), " \t\r\n");
    var it = std.mem.splitScalar(u8, version, '.');
    const major = it.next() orelse "0";
    const minor = it.next() orelse "0";
    const sub = it.next() orelse "0";
    const wf = b.addWriteFiles();
    _ = wf.add("prg.h", b.fmt(
        \\#pragma once
        \\#define REL_V "{s}"
        \\#define MAJ_V "{s}"
        \\#define MIN_V "{s}"
        \\#define SUBMIN_V "0"
        \\
    , .{ major, minor, sub }));
    return wf.getDirectory();
}

/// The Node addon, `imodeljs.node`, with its `Assets` directory beside it.
fn addon(ctx: *const Ctx, third: ThirdParty, core: Core) void {
    const b = ctx.b;
    const strip = b.option(bool, "strip", "Strip debug info from imodeljs.node (default: true unless Debug)") orelse (ctx.optimize != .Debug);
    const mod = b.createModule(.{
        .target = ctx.target,
        .optimize = ctx.optimize,
        .link_libc = true,
        .link_libcpp = true,
        .pic = true,
        .sanitize_c = .off,
        .strip = strip,
    });
    ctx.addCoreIncludes(mod);
    mod.addIncludePath(b.path("iModelJsNodeAddon"));
    mod.addIncludePath(packageVersionHeader(b));
    for (coreDefines(b, &.{ "BUILDING_NODE_EXTENSION", "BENTLEYCONFIG_SMLIB" })) |d| addDefine(mod, d);
    mod.addIncludePath(b.path("iModelCore/SMBRepGeometry/PublicAPI"));
    if (ctx.optimize != .Debug) mod.addCMacro("NDEBUG", "1");
    const flags: []const []const u8 = &.{ "-std=c++20", "-w", "-fno-sanitize=undefined" };
    addSources(mod, b.path("."), &src.imodeljs_node, &.{}, flags);
    addSources(mod, b.path("."), &.{"iModelJsNodeAddon/CrashReportingNone.cpp"}, &.{}, flags);

    // Static archives, each before the ones it depends on.
    for ([_]*Compile{
        core.brep,   core.visualization, core.platform,  core.presentation, core.ecdb,               core.geocoord,
        third.csmap, core.sqlite,        core.ecobjects, core.units,        core.geom_serialization, core.geom,
        core.jpeg,   third.jpeg_turbo,   third.png,      third.freetype,    third.folly,             third.pugixml,
        third.re2,   third.lzma,         third.snappy,   third.zlib,        core.bentley,            third.icu,
    }) |l| mod.linkLibrary(l);
    // CloudSQLite's networking and crypto come from the system.
    mod.linkSystemLibrary("curl", .{});
    mod.linkSystemLibrary("ssl", .{});
    mod.linkSystemLibrary("crypto", .{});

    const lib = b.addLibrary(.{ .name = "imodeljs", .linkage = .dynamic, .root_module = mod });
    // napi_* resolve against the node process that loads the addon.
    lib.linker_allow_shlib_undefined = true;
    // Export nothing: the module registers itself from a static constructor, and keeping the
    // bundled libraries' symbols local avoids clashes with node's own OpenSSL and zlib.
    lib.setVersionScript(b.addWriteFiles().add("imodeljs.map", "{ local: *; };\n"));

    const dir = "imodeljs-linux-x64";
    const install = b.addInstallFileWithDir(lib.getEmittedBin(), .{ .custom = dir }, "imodeljs.node");
    b.getInstallStep().dependOn(&install.step);
    const assets = installAssets(b, dir);
    b.getInstallStep().dependOn(assets);
    const step = b.step("addon", "Build imodeljs.node and its Assets");
    step.dependOn(&install.step);
    step.dependOn(assets);

    const smoke = b.addSystemCommand(&.{ "node", "zig/smoke-test.js" });
    smoke.addArg(b.getInstallPath(.{ .custom = dir }, "imodeljs.node"));
    smoke.step.dependOn(&install.step);
    smoke.step.dependOn(assets);
    smoke.has_side_effects = true;
    b.step("smoke", "Create an ECDb and an iModel with the built addon").dependOn(&smoke.step);
}

/// `<imodeljs-nodeaddonapi.package.version.h>`, generated from package_version.txt.
fn packageVersionHeader(b: *std.Build) LazyPath {
    const version = std.mem.trim(u8, @embedFile("iModelJsNodeAddon/package_version.txt"), " \t\r\n");
    const template = @embedFile("iModelJsNodeAddon/imodeljs-nodeaddonapi.package.version.h.template");
    const header = std.mem.replaceOwned(u8, b.allocator, template, "${PACKAGE_VERSION}", version) catch @panic("OOM");
    const wf = b.addWriteFiles();
    _ = wf.add("imodeljs-nodeaddonapi.package.version.h", header);
    return wf.getDirectory();
}

/// The runtime assets the addon loads from `<addon dir>/Assets`.
fn installAssets(b: *std.Build, dir: []const u8) *std.Build.Step {
    const step = b.step("assets", "Install the addon's Assets directory");
    const assets = b.fmt("{s}/Assets", .{dir});
    const files = [_]struct { []const u8, []const u8 }{
        .{ "iModelCore/libsrc/icu4c/icudt78l.dat", "icudt78l.dat" },
        .{ "iModelCore/iModelPlatform/Workspaces/FallbackFonts.itwin-workspace", "FallbackFonts.itwin-workspace" },
        .{ "iModelCore/iModelPlatform/Workspaces/RscFonts.itwin-workspace", "RscFonts.itwin-workspace" },
        .{ "iModelCore/iModelPlatform/ECSchemas/ChangedElements.01.00.00.ecschema.xml", "ECSchemas/Dgn/ChangedElements.01.00.00.ecschema.xml" },
    };
    for (files) |f| {
        step.dependOn(&b.addInstallFileWithDir(b.path(f[0]), .{ .custom = assets }, f[1]).step);
    }
    for ([_][]const u8{
        "ECDbFileInfo.02.00.01", "ECDbMap.02.00.04", "ECDbMeta.04.00.03", "ECDbSchemaPolicies.01.00.01", "ECDbChange.01.00.01",
    }) |name| {
        const file = b.fmt("{s}.ecschema.xml", .{name});
        step.dependOn(&b.addInstallFileWithDir(b.path(b.fmt("iModelCore/ECDb/ECDb/ECSchemas/{s}", .{file})), .{ .custom = assets }, b.fmt("ECSchemas/ECDb/{s}", .{file})).step);
    }
    // EC2 schemas that schema conversion and the published schema locaters expect beside the
    // EC3 standard schemas.
    for ([_]struct { []const u8, []const u8 }{
        .{ "schemas/ECStandards/EC3ConversionSchemas/MetaSchema_V3Conversion.02.04", "V3Conversion" },
        .{ "schemas/ECStandards/schemas/General/Bentley_Common_Classes.01.01", "Standard" },
        .{ "schemas/ECStandards/schemas/Bentley_ECSchemaMap.01.00", "Standard" },
        .{ "schemas/ECStandards/schemas/Bentley_Standard_Classes.01.01", "Standard" },
        .{ "schemas/ECStandards/schemas/Bentley_Standard_CustomAttributes.01.14", "Standard" },
        .{ "schemas/ECStandards/schemas/ECDbMap.01.00", "Standard" },
        .{ "schemas/ECStandards/schemas/ECv3ConversionAttributes.01.01", "Standard" },
        .{ "schemas/ECStandards/schemas/EditorCustomAttributes.01.03", "Standard" },
        .{ "schemas/ECStandards/schemas/LibraryUnits/Dimension_Schema.01.00", "Standard" },
        .{ "schemas/ECStandards/schemas/LibraryUnits/KindOfQuantity_Schema.01.01", "Standard" },
        .{ "schemas/ECStandards/schemas/LibraryUnits/SIUnitSystemDefaults.01.00", "Standard" },
        .{ "schemas/ECStandards/schemas/LibraryUnits/USCustomaryUnitSystemDefaults.01.00", "Standard" },
        .{ "schemas/ECStandards/schemas/LibraryUnits/Unit_Attributes.01.00", "Standard" },
        .{ "schemas/ECStandards/schemas/LibraryUnits/Units_Schema.01.00", "Standard" },
        .{ "schemas/ECStandards/schemas/LibraryUnits/iip_mdb_customAttributes.01.00", "Standard" },
        .{ "schemas/ECStandards/schemas/LibraryUnits/rdl_customAttributes.01.00", "Standard" },
    }) |schema| {
        const name = std.fs.path.basename(schema[0]);
        step.dependOn(&b.addInstallFileWithDir(b.path(b.fmt("{s}.ecschema.xml", .{schema[0]})), .{ .custom = assets }, b.fmt("ECSchemas/{s}/{s}.ecschema.xml", .{ schema[1], name })).step);
    }
    const bis = b.dependency("bis_schemas", .{});
    for (bis_schemas) |schema| {
        step.dependOn(&b.addInstallFileWithDir(bis.path(schema.src), .{ .custom = assets }, b.fmt("ECSchemas/{s}", .{schema.dest})).step);
    }
    return step;
}

/// The b-rep geometry admin backed by SMLib. Two libraries, because SMLib's headers need the macros
/// SMLib was built with and those clash with Bentley's: iTwinSmKernel sees only SMLib, and
/// iTwinSMBRepGeometry sees only Bentley headers plus SmKernel.h.
fn smlibBRep(ctx: *const Ctx) *Compile {
    const b = ctx.b;
    const usd_brep = b.dependency("usd_brep", .{
        .target = ctx.target,
        .optimize = ctx.optimize,
        .linkage = std.builtin.LinkMode.static,
        .pic = true,
        .@"no-debug-gfx" = true,
    });
    const kernel = ctx.lib("iTwinSmKernel", .{
        .core = false,
        .sources = &.{ "iModelCore/SMBRepGeometry/SmKernel.cpp", "iModelCore/SMBRepGeometry/SmParasolid.cpp", "iModelCore/SMBRepGeometry/ParasolidXT.cpp" },
        // Must match how usd-brep compiled SMLib: its class layouts depend on these.
        .defines = if (ctx.optimize == .Debug)
            &.{ "NOMINMAX", "SM_NO_DEBUG_CODE", "SM_NO_GFX_OUTPUT_CODE", "_DEBUG" }
        else
            &.{ "NOMINMAX", "SM_NO_DEBUG_CODE", "SM_NO_GFX_OUTPUT_CODE" },
        .link = &.{ usd_brep.artifact("SM_API"), usd_brep.artifact("SMLib") },
    });
    kernel.root_module.addIncludePath(usd_brep.path("source/SMLib/inc"));
    kernel.root_module.addIncludePath(usd_brep.path("source/SM_API/inc"));

    const test_mod = b.createModule(.{ .target = ctx.target, .optimize = ctx.optimize, .link_libc = true, .link_libcpp = true });
    test_mod.addCSourceFile(.{ .file = b.path("iModelCore/SMBRepGeometry/test/SmKernelTest.cpp"), .flags = &.{"-std=c++20"} });
    test_mod.linkLibrary(kernel);
    const run_test = b.addRunArtifact(b.addExecutable(.{ .name = "smkernel-test", .root_module = test_mod }));
    b.step("smkernel-test", "Check the SMLib operations behind createBRepGeometry").dependOn(&run_test.step);

    const xt_mod = b.createModule(.{ .target = ctx.target, .optimize = ctx.optimize, .link_libc = true, .link_libcpp = true });
    xt_mod.addCSourceFile(.{ .file = b.path("iModelCore/SMBRepGeometry/test/ParasolidTest.cpp"), .flags = &.{"-std=c++20"} });
    xt_mod.linkLibrary(kernel);
    xt_mod.addIncludePath(usd_brep.path("source/SMLib/inc"));
    xt_mod.addIncludePath(usd_brep.path("source/SM_API/inc"));
    for (kernel.root_module.c_macros.items) |m| xt_mod.c_macros.append(b.allocator, m) catch @panic("OOM");
    const xt_test = b.addExecutable(.{ .name = "parasolid-test", .root_module = xt_mod });
    b.step("parasolid-test", "Build bin/parasolid-test, which rebuilds Parasolid transmit files with SMLib").dependOn(&b.addInstallArtifact(xt_test, .{}).step);

    return ctx.lib("iTwinSMBRepGeometry", .{
        .sources = &.{"iModelCore/SMBRepGeometry/SMBRepGeometry.cpp"},
        .defines = coreDefines(b, &.{}),
        .includes = &.{"iModelCore/SMBRepGeometry/PublicAPI"},
        .link = &.{kernel},
    });
}
