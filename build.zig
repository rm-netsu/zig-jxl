const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // brotli-zig: full native Zig Brotli (RFC 7932) implementation, used for
    // the `jbrd` payload (real Brotli compression/decompression, replacing
    // the previous stored-meta-block-only stub).
    const brotli_dep = b.dependency("brotli", .{
        .target = target,
        .optimize = optimize,
    });
    const brotli_mod = brotli_dep.module("brotli");

    const lib_mod = b.createModule(.{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });
    lib_mod.addImport("brotli", brotli_mod);

    const lib = b.addLibrary(.{
        .name = "jxl",
        .root_module = lib_mod,
    });
    b.installArtifact(lib);

    const exe_mod = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    exe_mod.addImport("jxl", lib_mod);
    exe_mod.addImport("brotli", brotli_mod);

    const exe = b.addExecutable(.{
        .name = "jxltool",
        .root_module = exe_mod,
    });
    b.installArtifact(exe);

    const dbg_mod = b.createModule(.{
        .root_source_file = b.path("src/debug_dump.zig"),
        .target = target,
        .optimize = optimize,
    });
    dbg_mod.addImport("jxl", lib_mod);
    const dbg_exe = b.addExecutable(.{
        .name = "jxldbg",
        .root_module = dbg_mod,
    });
    b.installArtifact(dbg_exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| run_cmd.addArgs(args);
    const run_step = b.step("run", "Run jxltool");
    run_step.dependOn(&run_cmd.step);

    const lib_tests = b.addTest(.{ .root_module = lib_mod });
    const run_lib_tests = b.addRunArtifact(lib_tests);
    const test_step = b.step("test", "Run library tests");
    test_step.dependOn(&run_lib_tests.step);
}
