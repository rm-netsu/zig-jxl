//! jxltool: minimal CLI front-end for the JXL library.
//!
//! Usage:
//!   jxltool encode <input.pnm> <output.jxl> [--group-size-shift N]
//!   jxltool decode <input.jxl> <output.pnm>

const std = @import("std");
const jxl = @import("jxl");

pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const io = init.io;

    var args_iter = try std.process.Args.Iterator.initAllocator(
        init.minimal.args,
        allocator,
    );
    defer args_iter.deinit();

    var args: std.ArrayList([]const u8) = .empty;
    defer args.deinit(allocator);
    while (args_iter.next()) |arg| {
        try args.append(allocator, arg);
    }

    var stderr_buf: [4096]u8 = undefined;
    var stderr_writer = std.Io.File.stderr().writer(io, &stderr_buf);
    const stderr = &stderr_writer.interface;

    if (args.items.len < 2) {
        try stderr.print("usage: {s} encode <in.pnm> <out.jxl> [--group-size-shift N]\n", .{args.items[0]});
        try stderr.print("                  [--responsive 0|1] [--progressive|-p] [--colorspace -1..41]\n", .{});
        try stderr.print("                  [--palette-colors N] [--channel-colors-global-pct F]\n", .{});
        try stderr.print("                  [--channel-colors-group-pct F] [--icc <file.icc>]\n", .{});
        try stderr.print("                  [--patches -1|0|1] (cjxl: -1 default, 0 off, 1 on)\n", .{});
        try stderr.print("                  [-d/--distance D]   (lossy VarDCT, D > 0; patches at -d)\n", .{});
        try stderr.print("       {s} decode <in.jxl> <out.pnm> [--dump-icc <file.icc>]\n", .{args.items[0]});
        try stderr.print("       {s} jpeg-roundtrip <in.jpg> <out.jpg>\n", .{args.items[0]});
        try stderr.print("       {s} jpeg-transcode <in.jpg> <out.jxl>\n", .{args.items[0]});
        try stderr.print("       {s} jpeg-untranscode <in.jxl> <out.jpg>\n", .{args.items[0]});
        try stderr.flush();
        std.process.exit(2);
    }

    const cmd = args.items[1];
    if (std.mem.eql(u8, cmd, "encode")) {
        try encodeCommand(allocator, io, args.items[2..], stderr);
    } else if (std.mem.eql(u8, cmd, "decode")) {
        try decodeCommand(allocator, io, args.items[2..], stderr);
    } else if (std.mem.eql(u8, cmd, "decode-anim")) {
        try decodeAnimCommand(allocator, io, args.items[2..], stderr);
    } else if (std.mem.eql(u8, cmd, "encode-anim")) {
        try encodeAnimCommand(allocator, io, args.items[2..], stderr);
    } else if (std.mem.eql(u8, cmd, "jpeg-roundtrip")) {
        try jpegRoundtripCommand(allocator, io, args.items[2..], stderr);
    } else if (std.mem.eql(u8, cmd, "jpeg-transcode")) {
        try jpegTranscodeCommand(allocator, io, args.items[2..], stderr);
    } else if (std.mem.eql(u8, cmd, "jpeg-untranscode")) {
        try jpegUntranscodeCommand(allocator, io, args.items[2..], stderr);
    } else {
        try stderr.print("unknown command: {s}\n", .{cmd});
        try stderr.flush();
        std.process.exit(2);
    }
    try stderr.flush();
}

/// Parses a JPEG, reconstructs it from the parsed representation and
/// verifies byte-exactness. Usage: jxltool jpeg-roundtrip <in.jpg> <out.jpg>
fn jpegRoundtripCommand(
    allocator: std.mem.Allocator,
    io: std.Io,
    args: []const []const u8,
    stderr: *std.Io.Writer,
) !void {
    if (args.len != 2) return error.MissingArguments;
    const input = try std.Io.Dir.cwd().readFileAlloc(
        io,
        args[0],
        allocator,
        .limited(1 << 31),
    );
    defer allocator.free(input);

    var jpg = jxl.jpeg.jpeg_data.JPEGData{};
    defer jpg.deinit(allocator);
    try jxl.jpeg.reader.readJpeg(
        input,
        .read_all,
        &jpg,
        allocator,
    );

    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(allocator);
    try jxl.jpeg.writer.writeJpeg(&jpg, &out, allocator);

    const identical = std.mem.eql(u8, input, out.items);
    if (args[1].len > 0 and !std.mem.eql(u8, args[1], "-")) {
        const file = try std.Io.Dir.cwd().createFile(
            io,
            args[1],
            .{},
        );
        defer file.close(io);
        try file.writeStreamingAll(io, out.items);
    }
    try stderr.print(
        "{s}: {d}x{d} {d} comps {d} scans -> {d} bytes: {s}\n",
        .{
            args[0],
            jpg.width,
            jpg.height,
            jpg.components.items.len,
            jpg.scan_info.items.len,
            out.items.len,
            if (identical) "IDENTICAL" else "MISMATCH",
        },
    );
    if (!identical) return error.JpegRoundtripMismatch;
}

/// Lossless JPEG -> JXL container transcode.
/// Usage: jxltool jpeg-transcode <in.jpg> <out.jxl> [--brotli-effort N]
///        [--no-cfl] [--no-coeff-reorder] [--group-order 0|1]
fn jpegTranscodeCommand(
    allocator: std.mem.Allocator,
    io: std.Io,
    args: []const []const u8,
    stderr: *std.Io.Writer,
) !void {
    var options = jxl.jpeg.transcode.EncodeOptions{};
    var positional: [2][]const u8 = undefined;
    var num_positional: usize = 0;
    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const arg = args[i];
        if (std.mem.eql(u8, arg, "--brotli-effort")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            options.brotli_quality = try std.fmt.parseInt(
                u32,
                args[i],
                10,
            );
            if (options.brotli_quality > 11) return error.InvalidArgument;
        } else if (std.mem.eql(u8, arg, "--no-cfl")) {
            options.cfl = false;
        } else if (std.mem.eql(u8, arg, "--no-coeff-reorder")) {
            options.coeff_orders = false;
        } else if (std.mem.eql(u8, arg, "--group-order") or
            std.mem.eql(u8, arg, "--centerfirst"))
        {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            const v = try std.fmt.parseInt(i32, args[i], 10);
            if (v != 0 and v != 1) return error.InvalidGroupOrder;
            options.group_order = v == 1;
        } else if (std.mem.startsWith(u8, arg, "--")) {
            return error.UnknownOption;
        } else {
            if (num_positional >= 2) return error.TooManyArguments;
            positional[num_positional] = arg;
            num_positional += 1;
        }
    }
    if (num_positional != 2) return error.MissingArguments;
    const input = try std.Io.Dir.cwd().readFileAlloc(
        io,
        positional[0],
        allocator,
        .limited(1 << 31),
    );
    defer allocator.free(input);
    const out_bytes = try jxl.jpeg.transcode.encodeTranscodeOptions(
        allocator,
        input,
        options,
    );
    defer allocator.free(out_bytes);
    const file = try std.Io.Dir.cwd().createFile(
        io,
        positional[1],
        .{},
    );
    defer file.close(io);
    try file.writeStreamingAll(io, out_bytes);
    try stderr.print(
        "{s}: {d} bytes -> {s}: {d} bytes ({d:.2}x){s}{s}{s}\n",
        .{
            positional[0],
            input.len,
            positional[1],
            out_bytes.len,
            if (out_bytes.len > 0)
                @as(f64, @floatFromInt(input.len)) / @as(f64, @floatFromInt(out_bytes.len))
            else
                0,
            if (options.cfl) " cfl" else "",
            if (options.coeff_orders) " coeff-reorder" else "",
            if (options.group_order) " group-order" else "",
        },
    );
}

/// Reconstructs the original JPEG from a transcode container.
/// Usage: jxltool jpeg-untranscode <in.jxl> <out.jpg>
fn jpegUntranscodeCommand(
    allocator: std.mem.Allocator,
    io: std.Io,
    args: []const []const u8,
    stderr: *std.Io.Writer,
) !void {
    if (args.len != 2) return error.MissingArguments;
    const input = try std.Io.Dir.cwd().readFileAlloc(
        io,
        args[0],
        allocator,
        .limited(1 << 31),
    );
    defer allocator.free(input);
    const out_bytes = try jxl.jpeg.transcode.decodeTranscode(
        allocator,
        input,
    );
    defer allocator.free(out_bytes);
    const file = try std.Io.Dir.cwd().createFile(
        io,
        args[1],
        .{},
    );
    defer file.close(io);
    try file.writeStreamingAll(io, out_bytes);
    try stderr.print(
        "{s}: {d} bytes -> {s}: {d} bytes\n",
        .{ args[0], input.len, args[1], out_bytes.len },
    );
}

fn encodeCommand(
    allocator: std.mem.Allocator,
    io: std.Io,
    args: []const []const u8,
    stderr: *std.Io.Writer,
) !void {
    var group_size_shift: u32 = 1;
    var responsive = false;
    var progressive = false;
    var colorspace: i32 = -1;
    var palette_colors: i32 = 1 << 10;
    var channel_colors_global_percent: f32 = 95.0;
    var channel_colors_group_percent: f32 = 80.0;
    var icc_path: ?[]const u8 = null;
    var resampling: u32 = 1;
    var ec_resampling: u32 = 0;
    var float16 = false;
    var effort: u32 = 7;
    var group_order = false;
    var patches: jxl.patches.enc_patches.PatchOverride = .default;
    var distance: f32 = 0;
    var positional: [2][]const u8 = undefined;
    var num_positional: usize = 0;
    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const arg = args[i];
        if (std.mem.eql(u8, arg, "--group-size-shift")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            group_size_shift = try std.fmt.parseInt(
                u32,
                args[i],
                10,
            );
            if (group_size_shift > 3) return error.InvalidGroupSizeShift;
        } else if (std.mem.eql(u8, arg, "--effort") or
            std.mem.eql(u8, arg, "-e"))
        {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            effort = try std.fmt.parseInt(u32, args[i], 10);
            if (effort < 1 or effort > 10) return error.InvalidEffort;
        } else if (std.mem.eql(u8, arg, "--group-order") or
            std.mem.eql(u8, arg, "--centerfirst"))
        {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            const v = try std.fmt.parseInt(i32, args[i], 10);
            if (v != 0 and v != 1) return error.InvalidGroupOrder;
            group_order = v == 1;
        } else if (std.mem.eql(u8, arg, "--responsive") or
            std.mem.eql(u8, arg, "-R"))
        {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            const v = try std.fmt.parseInt(i32, args[i], 10);
            if (v != 0 and v != 1) return error.InvalidResponsive;
            responsive = v == 1;
        } else if (std.mem.eql(u8, arg, "--progressive") or
            std.mem.eql(u8, arg, "-p"))
        {
            progressive = true;
        } else if (std.mem.eql(u8, arg, "--resampling")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            resampling = try std.fmt.parseInt(u32, args[i], 10);
        } else if (std.mem.eql(u8, arg, "--ec-resampling")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            ec_resampling = try std.fmt.parseInt(u32, args[i], 10);
        } else if (std.mem.eql(u8, arg, "--float16")) {
            float16 = true;
        } else if (std.mem.eql(u8, arg, "--colorspace") or
            std.mem.eql(u8, arg, "-C"))
        {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            colorspace = try std.fmt.parseInt(i32, args[i], 10);
            if (colorspace < -1 or colorspace > 41)
                return error.InvalidColorspace;
        } else if (std.mem.eql(u8, arg, "--palette-colors")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            palette_colors = try std.fmt.parseInt(
                i32,
                args[i],
                10,
            );
        } else if (std.mem.eql(u8, arg, "--channel-colors-global-pct")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            channel_colors_global_percent = try std.fmt.parseFloat(
                f32,
                args[i],
            );
        } else if (std.mem.eql(u8, arg, "--channel-colors-group-pct")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            channel_colors_group_percent = try std.fmt.parseFloat(
                f32,
                args[i],
            );
        } else if (std.mem.eql(u8, arg, "--patches") or
            std.mem.startsWith(u8, arg, "--patches="))
        {
            var v: i32 = undefined;
            if (std.mem.startsWith(u8, arg, "--patches=")) {
                v = std.fmt.parseInt(
                    i32,
                    arg["--patches=".len..],
                    10,
                ) catch return error.InvalidPatches;
            } else {
                i += 1;
                if (i >= args.len) return error.MissingArgument;
                v = std.fmt.parseInt(
                    i32,
                    args[i],
                    10,
                ) catch return error.InvalidPatches;
            }
            if (v < -1 or v > 1) return error.InvalidPatches;
            patches = if (v == 0) .off else if (v == 1) .on else .default;
        } else if (std.mem.eql(u8, arg, "--distance") or
            std.mem.eql(u8, arg, "-d"))
        {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            distance = std.fmt.parseFloat(
                f32,
                args[i],
            ) catch return error.InvalidDistance;
        } else if (std.mem.eql(u8, arg, "--icc")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            icc_path = args[i];
        } else if (std.mem.startsWith(u8, arg, "--")) {
            return error.UnknownOption;
        } else {
            if (num_positional >= 2) return error.TooManyArguments;
            positional[num_positional] = arg;
            num_positional += 1;
        }
    }
    if (num_positional != 2) return error.MissingArguments;

    const input = try std.Io.Dir.cwd().readFileAlloc(
        io,
        positional[0],
        allocator,
        .limited(1 << 31),
    );
    defer allocator.free(input);

    var icc_bytes: []const u8 = &.{};
    if (icc_path) |path| {
        icc_bytes = try std.Io.Dir.cwd().readFileAlloc(
            io,
            path,
            allocator,
            .limited(1 << 31),
        );
    }
    defer if (icc_bytes.len > 0) allocator.free(icc_bytes);

    var writer = jxl.base.BitWriter.init(allocator);
    defer writer.deinit();

    const encode_opts = jxl.codec.encoder.EncodeOptions{
        .group_size_shift = group_size_shift,
        .responsive = responsive,
        .progressive = progressive,
        .colorspace = colorspace,
        .palette_colors = palette_colors,
        .channel_colors_global_percent = channel_colors_global_percent,
        .channel_colors_group_percent = channel_colors_group_percent,
        .icc = icc_bytes,
        .resampling = resampling,
        .ec_resampling = ec_resampling,
        .float16 = float16,
        .effort = effort,
        .patches = patches,
        .group_order = group_order,
        .distance = distance,
    };

    if (jxl.pnm.isPfm(input)) {
        // Float input (PFM): fp32 by default, --float16 for binary16.
        var fb = try jxl.pnm.readFloat(
            allocator,
            input,
        );
        defer fb.deinit();
        try jxl.codec.encoder.encodeFloat(
            allocator,
            &fb,
            encode_opts,
            &writer,
        );
        writer.zeroPadToByte();
        const out_bytes = try writer.takeBytes();
        defer allocator.free(out_bytes);
        const file = try std.Io.Dir.cwd().createFile(
            io,
            positional[1],
            .{},
        );
        defer file.close(io);
        try file.writeStreamingAll(io, out_bytes);
        const ratio = if (out_bytes.len > 0)
            @as(f64, @floatFromInt(input.len)) / @as(f64, @floatFromInt(out_bytes.len))
        else
            0;
        try stderr.print(
            "{s}: {d}x{d} float{d}{s} -> {s}: {d} bytes ({d:.2}x)\n",
            .{
                positional[0],
                fb.xsize,
                fb.ysize,
                if (float16) @as(u8, 16) else @as(u8, 32),
                if (resampling != 1) " (resampled)" else "",
                positional[1],
                out_bytes.len,
                ratio,
            },
        );
        return;
    }

    var pb = try jxl.pnm.read(allocator, input);
    defer pb.deinit();

    if (distance > 0) {
        // Lossy VarDCT (cjxl `-d D`): the XYB patch subset.
        try jxl.codec.encoder.encodeLossy(
            allocator,
            &pb,
            encode_opts,
            &writer,
        );
    } else {
        try jxl.codec.encoder.encode(
            allocator,
            &pb,
            encode_opts,
            &writer,
        );
    }
    writer.zeroPadToByte();
    const out_bytes = try writer.takeBytes();
    defer allocator.free(out_bytes);

    const file = try std.Io.Dir.cwd().createFile(
        io,
        positional[1],
        .{},
    );
    defer file.close(io);
    try file.writeStreamingAll(io, out_bytes);

    const ratio = if (out_bytes.len > 0)
        @as(f64, @floatFromInt(input.len)) / @as(f64, @floatFromInt(out_bytes.len))
    else
        0;
    try stderr.print(
        "{s}: {d}x{d} {d}-bit{s}{s}{s} -> {s}: {d} bytes ({d:.2}x)\n",
        .{
            positional[0],
            pb.xsize,
            pb.ysize,
            pb.bitdepth,
            if (pb.has_alpha) "+alpha" else "",
            if (icc_bytes.len > 0) " +icc" else "",
            if (distance > 0) " lossy" else "",
            positional[1],
            out_bytes.len,
            ratio,
        },
    );
}

fn decodeCommand(
    allocator: std.mem.Allocator,
    io: std.Io,
    args: []const []const u8,
    stderr: *std.Io.Writer,
) !void {
    var icc_path: ?[]const u8 = null;
    var linear = false;
    var positional: [2][]const u8 = undefined;
    var num_positional: usize = 0;
    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const arg = args[i];
        if (std.mem.eql(u8, arg, "--dump-icc")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            icc_path = args[i];
        } else if (std.mem.eql(u8, arg, "--linear")) {
            linear = true;
        } else if (std.mem.startsWith(u8, arg, "--")) {
            return error.UnknownOption;
        } else {
            if (num_positional >= 2) return error.TooManyArguments;
            positional[num_positional] = arg;
            num_positional += 1;
        }
    }
    if (num_positional != 2) return error.MissingArguments;

    const input = try std.Io.Dir.cwd().readFileAlloc(
        io,
        positional[0],
        allocator,
        .limited(1 << 31),
    );
    defer allocator.free(input);

    var icc_bytes: ?[]u8 = null;
    defer if (icc_bytes) |b| allocator.free(b);
    var img = try jxl.codec.decoder.decode(
        allocator,
        input,
        .{
            .icc_out = if (icc_path != null) &icc_bytes else null,
            .prefer_linear_float = linear,
        },
    );
    defer img.deinit();

    if (icc_path) |path| {
        if (icc_bytes) |b| {
            const icc_file = try std.Io.Dir.cwd().createFile(
                io,
                path,
                .{},
            );
            defer icc_file.close(io);
            try icc_file.writeStreamingAll(io, b);
            try stderr.print(
                "icc: {d} bytes -> {s}\n",
                .{ b.len, path },
            );
        } else {
            try stderr.print("icc: none embedded\n", .{});
        }
    }

    switch (img) {
        .int => |*pb| {
            const out_bytes = try jxl.pnm.write(allocator, pb);
            defer allocator.free(out_bytes);
            const file = try std.Io.Dir.cwd().createFile(
                io,
                positional[1],
                .{},
            );
            defer file.close(io);
            try file.writeStreamingAll(io, out_bytes);
            try stderr.print(
                "{s}: {d}x{d} {d}-bit{s} -> {s}: {d} bytes\n",
                .{
                    positional[0],
                    pb.xsize,
                    pb.ysize,
                    pb.bitdepth,
                    if (pb.has_alpha) "+alpha" else "",
                    positional[1],
                    out_bytes.len,
                },
            );
        },
        .float => |*fb| {
            const out_bytes = try jxl.pnm.writeFloat(
                allocator,
                fb,
            );
            defer allocator.free(out_bytes);
            const file = try std.Io.Dir.cwd().createFile(
                io,
                positional[1],
                .{},
            );
            defer file.close(io);
            try file.writeStreamingAll(io, out_bytes);
            try stderr.print(
                "{s}: {d}x{d} float{d} (exp {d}) -> {s}: {d} bytes\n",
                .{
                    positional[0],
                    fb.xsize,
                    fb.ysize,
                    fb.bits_per_sample,
                    fb.exponent_bits_per_sample,
                    positional[1],
                    out_bytes.len,
                },
            );
        },
    }
}

/// Decodes an animation into per-frame PAM files `<out_prefix>_NNNN.pam`
/// plus a `<out_prefix>.frames.txt` sidecar with one duration_ms per line.
/// Usage: jxltool decode-anim <in.jxl> <out_prefix> [--dump-icc <file.icc>]
fn decodeAnimCommand(
    allocator: std.mem.Allocator,
    io: std.Io,
    args: []const []const u8,
    stderr: *std.Io.Writer,
) !void {
    var icc_path: ?[]const u8 = null;
    var positional: [2][]const u8 = undefined;
    var num_positional: usize = 0;
    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const arg = args[i];
        if (std.mem.eql(u8, arg, "--dump-icc")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            icc_path = args[i];
        } else if (std.mem.startsWith(u8, arg, "--")) {
            return error.UnknownOption;
        } else {
            if (num_positional >= 2) return error.TooManyArguments;
            positional[num_positional] = arg;
            num_positional += 1;
        }
    }
    if (num_positional != 2) return error.MissingArguments;

    const input = try std.Io.Dir.cwd().readFileAlloc(
        io,
        positional[0],
        allocator,
        .limited(1 << 31),
    );
    defer allocator.free(input);

    var icc_bytes: ?[]u8 = null;
    defer if (icc_bytes) |b| allocator.free(b);
    var anim = try jxl.codec.animation.decodeAnimation(
        allocator,
        input,
        .{
            .icc_out = if (icc_path != null) &icc_bytes else null,
        },
    );
    defer anim.deinit();

    if (icc_path) |path| {
        if (icc_bytes) |b| {
            const icc_file = try std.Io.Dir.cwd().createFile(
                io,
                path,
                .{},
            );
            defer icc_file.close(io);
            try icc_file.writeStreamingAll(io, b);
            try stderr.print(
                "icc: {d} bytes -> {s}\n",
                .{ b.len, path },
            );
        } else {
            try stderr.print("icc: none embedded\n", .{});
        }
    }

    var sidecar: std.ArrayList(u8) = .empty;
    defer sidecar.deinit(allocator);
    for (anim.frames.items, 0..) |*f, idx| {
        var name_buf: [512]u8 = undefined;
        const name = try std.fmt.bufPrint(
            &name_buf,
            "{s}_{d:0>4}.pam",
            .{ positional[1], idx },
        );
        const out_bytes = try jxl.pnm.write(
            allocator,
            &f.pixels,
        );
        defer allocator.free(out_bytes);
        const file = try std.Io.Dir.cwd().createFile(
            io,
            name,
            .{},
        );
        defer file.close(io);
        try file.writeStreamingAll(io, out_bytes);
        const dur = anim.frameDurationMs(idx);
        const dur_line = try std.fmt.allocPrint(
            allocator,
            "{d:.3}\n",
            .{dur},
        );
        defer allocator.free(dur_line);
        try sidecar.appendSlice(allocator, dur_line);
        try stderr.print(
            "frame {d}: {d}x{d} {d}-bit{s} duration {d:.3} ms -> {s}\n",
            .{
                idx,
                f.pixels.xsize,
                f.pixels.ysize,
                f.pixels.bitdepth,
                if (f.pixels.has_alpha) "+alpha" else "",
                dur,
                name,
            },
        );
    }
    var sidecar_name_buf: [512]u8 = undefined;
    const sidecar_name = try std.fmt.bufPrint(
        &sidecar_name_buf,
        "{s}.frames.txt",
        .{positional[1]},
    );
    const sf = try std.Io.Dir.cwd().createFile(
        io,
        sidecar_name,
        .{},
    );
    defer sf.close(io);
    try sf.writeStreamingAll(io, sidecar.items);
    try stderr.print(
        "{s}: {d} frames, tps {d}/{d}, loops {d}\n",
        .{
            positional[0],
            anim.frames.items.len,
            anim.tps_numerator,
            anim.tps_denominator,
            anim.num_loops,
        },
    );
}

/// Encodes a sequence of PNM/PAM frames into an animation.
/// Usage: jxltool encode-anim <out.jxl> <f1.pnm> [f2.pnm ...] [--duration-ms N]
///        [--loops N] [--group-size-shift N] [--responsive 0|1] [--progressive] [encode opts]
fn encodeAnimCommand(
    allocator: std.mem.Allocator,
    io: std.Io,
    args: []const []const u8,
    stderr: *std.Io.Writer,
) !void {
    var duration_ms: u32 = 100;
    var loops: u32 = 0;
    var group_size_shift: u32 = 1;
    var responsive = false;
    var progressive = false;
    var colorspace: i32 = -1;
    var palette_colors: i32 = 1 << 10;
    var channel_colors_global_percent: f32 = 95.0;
    var channel_colors_group_percent: f32 = 80.0;
    var effort: u32 = 7;
    var group_order = false;
    var patches: jxl.patches.enc_patches.PatchOverride = .default;
    var inputs: std.ArrayList([]const u8) = .empty;
    defer inputs.deinit(allocator);

    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const arg = args[i];
        if (std.mem.eql(u8, arg, "--duration-ms")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            duration_ms = try std.fmt.parseInt(u32, args[i], 10);
        } else if (std.mem.eql(u8, arg, "--loops")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            loops = try std.fmt.parseInt(u32, args[i], 10);
        } else if (std.mem.eql(u8, arg, "--group-size-shift")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            group_size_shift = try std.fmt.parseInt(
                u32,
                args[i],
                10,
            );
        } else if (std.mem.eql(u8, arg, "--effort") or
            std.mem.eql(u8, arg, "-e"))
        {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            effort = try std.fmt.parseInt(u32, args[i], 10);
            if (effort < 1 or effort > 10) return error.InvalidEffort;
        } else if (std.mem.eql(u8, arg, "--group-order") or
            std.mem.eql(u8, arg, "--centerfirst"))
        {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            const v = try std.fmt.parseInt(i32, args[i], 10);
            if (v != 0 and v != 1) return error.InvalidGroupOrder;
            group_order = v == 1;
        } else if (std.mem.eql(u8, arg, "--responsive")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            responsive = try std.fmt.parseInt(
                u32,
                args[i],
                10,
            ) != 0;
        } else if (std.mem.eql(u8, arg, "--progressive") or
            std.mem.eql(u8, arg, "-p"))
        {
            progressive = true;
        } else if (std.mem.eql(u8, arg, "--colorspace")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            colorspace = try std.fmt.parseInt(i32, args[i], 10);
        } else if (std.mem.eql(u8, arg, "--palette-colors")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            palette_colors = try std.fmt.parseInt(
                i32,
                args[i],
                10,
            );
        } else if (std.mem.eql(
            u8,
            arg,
            "--channel-colors-global-pct",
        )) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            channel_colors_global_percent = try std.fmt.parseFloat(
                f32,
                args[i],
            );
        } else if (std.mem.eql(u8, arg, "--channel-colors-group-pct")) {
            i += 1;
            if (i >= args.len) return error.MissingArgument;
            channel_colors_group_percent = try std.fmt.parseFloat(
                f32,
                args[i],
            );
        } else if (std.mem.eql(u8, arg, "--patches") or
            std.mem.startsWith(u8, arg, "--patches="))
        {
            var v: i32 = undefined;
            if (std.mem.startsWith(u8, arg, "--patches=")) {
                v = std.fmt.parseInt(
                    i32,
                    arg["--patches=".len..],
                    10,
                ) catch return error.InvalidPatches;
            } else {
                i += 1;
                if (i >= args.len) return error.MissingArgument;
                v = std.fmt.parseInt(
                    i32,
                    args[i],
                    10,
                ) catch return error.InvalidPatches;
            }
            if (v < -1 or v > 1) return error.InvalidPatches;
            patches = if (v == 0) .off else if (v == 1) .on else .default;
        } else if (std.mem.startsWith(u8, arg, "--")) {
            return error.UnknownOption;
        } else {
            try inputs.append(allocator, arg);
        }
    }
    if (inputs.items.len < 2) return error.MissingArguments; // out.jxl + >=1 frame

    var frames: std.ArrayList(*jxl.codec.pixel.PixelBuffer) = .empty;
    defer {
        for (frames.items) |f| {
            f.deinit();
            allocator.destroy(f);
        }
        frames.deinit(allocator);
    }
    for (inputs.items[1..]) |path| {
        const data = try std.Io.Dir.cwd().readFileAlloc(
            io,
            path,
            allocator,
            .limited(1 << 31),
        );
        defer allocator.free(data);
        const pb = try allocator.create(
            jxl.codec.pixel.PixelBuffer,
        );
        errdefer allocator.destroy(pb);
        pb.* = try jxl.pnm.read(allocator, data);
        try frames.append(allocator, pb);
    }

    const durations = try allocator.alloc(u32, frames.items.len);
    defer allocator.free(durations);
    @memset(durations, duration_ms);

    var w = jxl.base.BitWriter.init(allocator);
    defer w.deinit();
    try jxl.codec.encoder.encodeAnimation(
        allocator,
        frames.items,
        .{
            .durations = durations,
            .num_loops = loops,
        },
        .{
            .group_size_shift = group_size_shift,
            .responsive = responsive,
            .progressive = progressive,
            .colorspace = colorspace,
            .palette_colors = palette_colors,
            .channel_colors_global_percent = channel_colors_global_percent,
            .channel_colors_group_percent = channel_colors_group_percent,
            .effort = effort,
            .patches = patches,
            .group_order = group_order,
        },
        &w,
    );
    w.zeroPadToByte();
    const bytes = try w.takeBytes();
    defer allocator.free(bytes);

    const file = try std.Io.Dir.cwd().createFile(
        io,
        inputs.items[0],
        .{},
    );
    defer file.close(io);
    try file.writeStreamingAll(io, bytes);
    try stderr.print(
        "{d} frames ({d}x{d} {d}-bit{s}) -> {s}: {d} bytes\n",
        .{
            frames.items.len,
            frames.items[0].xsize,
            frames.items[0].ysize,
            frames.items[0].bitdepth,
            if (frames.items[0].has_alpha) "+alpha" else "",
            inputs.items[0],
            bytes.len,
        },
    );
}
