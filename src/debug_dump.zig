// Debug harness: dump modular stream structure of a .jxl file, or (with
// `--frames`) walk every frame header of an animation.
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
    _ = args_iter.next(); // argv0
    var argv: std.ArrayList([]const u8) = .empty;
    defer argv.deinit(allocator);
    while (args_iter.next()) |arg|
        try argv.append(allocator, arg);

    var stderr_buf: [4096]u8 = undefined;
    var stderr_writer = std.Io.File.stderr().writer(
        io,
        &stderr_buf,
    );
    const stderr = &stderr_writer.interface;

    if (argv.items.len >= 1 and std.mem.eql(
        u8,
        argv.items[0],
        "--hdrdiff",
    )) {
        if (argv.items.len < 2) return error.MissingArgument;
        return hdrDiffMain(
            allocator,
            io,
            argv.items[1],
            stderr,
        );
    }
    if (argv.items.len >= 1 and std.mem.eql(
        u8,
        argv.items[0],
        "--toc",
    )) {
        if (argv.items.len < 2) return error.MissingArgument;
        return tocMain(
            allocator,
            io,
            argv.items[1],
            stderr,
        );
    }
    const frames_mode = argv.items.len >= 1 and std.mem.eql(
        u8,
        argv.items[0],
        "--frames",
    );
    const file = if (frames_mode)
        (if (argv.items.len >= 2)
            argv.items[1]
        else
            return error.MissingArgument)
    else
        (if (argv.items.len >= 1)
            argv.items[0]
        else
            return error.MissingArgument);

    const input = try std.Io.Dir.cwd().readFileAlloc(
        io,
        file,
        allocator,
        .limited(1 << 31),
    );
    defer allocator.free(input);

    var codestream: []const u8 = input;
    if (jxl.header.container.isContainer(input)) {
        const cs = try jxl.header.container.extractCodestream(
            allocator,
            input,
        );
        defer allocator.free(cs);
        codestream = try allocator.dupe(u8, cs);
    }
    var br = jxl.base.BitReader.init(codestream);
    _ = br.readFixedBits(16);
    var sh: jxl.header.size.SizeHeader = .{};
    try sh.read(&br);
    var metadata = jxl.header.metadata.ImageMetadata.init(
        allocator,
    );
    defer metadata.deinit();
    try metadata.read(&br);
    var ctd = jxl.header.metadata.CustomTransformData{};
    try ctd.read(&br, metadata.xyb_encoded);
    if (metadata.color_encoding.want_icc) {
        const icc_bytes = try jxl.header.icc.readICC(
            allocator,
            &br,
        );
        defer allocator.free(icc_bytes);
    }
    try br.jumpToByteBoundary();

    if (frames_mode) return dumpFrames(
        allocator,
        &br,
        &metadata,
        sh,
        codestream,
        stderr,
    );
    return dumpStream(
        allocator,
        &br,
        &metadata,
        sh,
        codestream,
        stderr,
    );
}

fn hdrDiffMain(
    allocator: std.mem.Allocator,
    io: std.Io,
    path: []const u8,
    stderr: *std.Io.Writer,
) !void {
    const input = try std.Io.Dir.cwd().readFileAlloc(
        io,
        path,
        allocator,
        .limited(1 << 31),
    );
    defer allocator.free(input);
    var cs: []const u8 = input;
    if (jxl.header.container.isContainer(input)) {
        const c2 = try jxl.header.container.extractCodestream(
            allocator,
            input,
        );
        defer allocator.free(c2);
        cs = try allocator.dupe(u8, c2);
    }
    var br = jxl.base.BitReader.init(cs);
    _ = br.readFixedBits(16);
    var sh: jxl.header.size.SizeHeader = .{};
    try sh.read(&br);
    var md = jxl.header.metadata.ImageMetadata.init(allocator);
    defer md.deinit();
    try md.read(&br);
    var ctd = jxl.header.metadata.CustomTransformData{};
    try ctd.read(&br, md.xyb_encoded);
    if (md.color_encoding.want_icc) {
        const ib = try jxl.header.icc.readICC(
            allocator,
            &br,
        );
        defer allocator.free(ib);
    }
    try br.jumpToByteBoundary();
    return hdrDiff(
        allocator,
        &br,
        &md,
        cs,
        stderr,
    );
}

/// Dumps the first frame's TOC (with permutation) as text: one line
/// `perm <i> = <v>` per entry plus the physical sizes.
fn tocMain(
    allocator: std.mem.Allocator,
    io: std.Io,
    path: []const u8,
    stderr: *std.Io.Writer,
) !void {
    const input = try std.Io.Dir.cwd().readFileAlloc(
        io,
        path,
        allocator,
        .limited(1 << 31),
    );
    defer allocator.free(input);
    var cs: []const u8 = input;
    if (jxl.header.container.isContainer(input)) {
        const c2 = try jxl.header.container.extractCodestream(
            allocator,
            input,
        );
        defer allocator.free(c2);
        cs = try allocator.dupe(u8, c2);
    }
    var br = jxl.base.BitReader.init(cs);
    _ = br.readFixedBits(16);
    var sh: jxl.header.size.SizeHeader = .{};
    try sh.read(&br);
    var md = jxl.header.metadata.ImageMetadata.init(allocator);
    defer md.deinit();
    try md.read(&br);
    var ctd = jxl.header.metadata.CustomTransformData{};
    try ctd.read(&br, md.xyb_encoded);
    if (md.color_encoding.want_icc) {
        const ib = try jxl.header.icc.readICC(
            allocator,
            &br,
        );
        defer allocator.free(ib);
    }
    try br.jumpToByteBoundary();

    var frame = jxl.header.frame.FrameHeader.init(allocator);
    defer frame.deinit();
    try frame.read(&md, &br);
    const fd = jxl.modular.stream.FrameDim.init(
        sh.xsizeOf(),
        sh.ysizeOf(),
        frame.group_size_shift,
        1,
    );
    const num_toc = jxl.header.toc.numTocEntries(
        fd.num_groups,
        fd.num_dc_groups,
        frame.passes.num_passes,
    );
    var toc = try jxl.header.toc.readTocWithPermutation(
        allocator,
        num_toc,
        &br,
    );
    defer toc.deinit(allocator);
    try stderr.print(
        "frame: groups={d} dc_groups={d} passes={d} toc entries={d} total={d}\n",
        .{
            fd.num_groups,
            fd.num_dc_groups,
            frame.passes.num_passes,
            num_toc,
            toc.totalSize(),
        },
    );
    if (toc.permutation) |p| {
        try stderr.print("permutation:", .{});
        for (p) |v| try stderr.print(" {d}", .{v});
        try stderr.print("\n", .{});
    } else {
        try stderr.print("permutation: (none)\n", .{});
    }
    for (toc.sizes, 0..) |s, i|
        try stderr.print("size[{d}] = {d}\n", .{ i, s });
    try stderr.flush();
}

/// Walks frame headers + TOCs, printing per-frame info.
fn dumpFrames(
    allocator: std.mem.Allocator,
    br: *jxl.base.BitReader,
    metadata: *jxl.header.metadata.ImageMetadata,
    sh: jxl.header.size.SizeHeader,
    codestream: []const u8,
    stderr: *std.Io.Writer,
) !void {
    var frame_index: usize = 0;
    var done = false;
    while (!done) {
        const header_bitpos = br.totalBitsConsumed();
        var frame = jxl.header.frame.FrameHeader.init(allocator);
        defer frame.deinit();
        try frame.read(metadata, br);
        const fd = jxl.modular.stream.FrameDim.init(
            sh.xsizeOf(),
            sh.ysizeOf(),
            frame.group_size_shift,
            1,
        );
        const num_toc = jxl.header.toc.numTocEntries(
            fd.num_groups,
            fd.num_dc_groups,
            frame.passes.num_passes,
        );
        const toc_sizes = try jxl.header.toc.readToc(
            allocator,
            num_toc,
            br,
        );
        defer allocator.free(toc_sizes);
        var total: usize = 0;
        for (toc_sizes) |s| total += s;
        const base = br.totalBitsConsumed() / 8;
        if (base + total > codestream.len) {
            try stderr.print(
                "frame[{d}]: TRUNCATED (needs {d} bytes, {d} left)\n",
                .{ frame_index, total, codestream.len -| base },
            );
            try stderr.flush();
            return;
        }
        br.skipBits(total * 8);

        try stderr.print(
            "frame[{d}] hdrbit={d} secbyte={d}: type={s} enc={s} ct={s} gshift={d} size={d}x{d}+{d}+{d} dur={d} tc=0x{x} last={} ref={d} sbct={} mode={s} src={d} crop={} ups={d} passes={d}\n",
            .{
                frame_index,                        header_bitpos,
                base,                               @tagName(frame.frame_type),
                @tagName(frame.encoding),           @tagName(frame.color_transform),
                frame.group_size_shift,             frame.frame_size_x,
                frame.frame_size_y,                 frame.frame_origin_x,
                frame.frame_origin_y,               frame.animation_frame_duration,
                frame.animation_frame_timecode,     frame.is_last,
                frame.save_as_reference,            frame.save_before_color_transform,
                @tagName(frame.blending_info.mode), frame.blending_info.source,
                frame.custom_size_or_origin,        frame.upsampling,
                frame.passes.num_passes,
            },
        );
        for (frame.extra_channel_blending_info.items, 0..) |*ec, i| {
            try stderr.print(
                "  ec[{d}]: mode={s} src={d} alpha={d} clamp={}\n",
                .{
                    i,
                    @tagName(ec.mode),
                    ec.source,
                    ec.alpha_channel,
                    ec.clamp,
                },
            );
        }
        try stderr.flush();
        frame_index += 1;
        done = frame.is_last;
    }
}

/// Original behavior: dump the first frame's modular stream structure.
fn dumpStream(
    allocator: std.mem.Allocator,
    br: *jxl.base.BitReader,
    metadata: *jxl.header.metadata.ImageMetadata,
    sh: jxl.header.size.SizeHeader,
    codestream: []const u8,
    stderr: *std.Io.Writer,
) !void {
    var frame = jxl.header.frame.FrameHeader.init(allocator);
    defer frame.deinit();
    try frame.read(metadata, br);

    try stderr.print(
        "frame: encoding={} color_transform={} group_shift={} groups={}\n",
        .{
            frame.encoding, frame.color_transform, frame.group_size_shift, 1,
        },
    );

    // Walk the modular global stream manually like the decoder does.
    const fd = jxl.modular.stream.FrameDim.init(
        sh.xsizeOf(),
        sh.ysizeOf(),
        frame.group_size_shift,
        1,
    );
    const num_toc = jxl.header.toc.numTocEntries(
        fd.num_groups,
        fd.num_dc_groups,
        frame.passes.num_passes,
    );
    const toc_sizes = try jxl.header.toc.readToc(
        allocator,
        num_toc,
        br,
    );
    defer allocator.free(toc_sizes);
    try stderr.print(
        "num_groups={d} num_dc_groups={d} toc entries={d}\n",
        .{ fd.num_groups, fd.num_dc_groups, num_toc },
    );
    for (toc_sizes, 0..) |s, i|
        try stderr.print("  toc[{d}] = {d} bytes\n", .{ i, s });

    var offsets = try allocator.alloc(usize, num_toc + 1);
    defer allocator.free(offsets);
    offsets[0] = 0;
    for (toc_sizes, 0..) |s, i| offsets[i + 1] = offsets[i] + s;
    const base = br.totalBitsConsumed() / 8;

    // Section 0.
    var sec = jxl.base.BitReader.init(
        codestream[base .. base + offsets[1]],
    );
    const dc_all_default = sec.readBits(1);
    try stderr.print("dequant all_default={d}\n", .{dc_all_default});
    const has_tree = sec.readBits(1);
    try stderr.print("has_tree={d}\n", .{has_tree});

    // Build the full image and run decodeStream with transform tracing.
    var full_image = jxl.modular.image.Image.create(allocator);
    defer full_image.deinit();
    const n_color: usize = if (metadata.color_encoding.isGray()) 1 else 3;
    for (0..n_color + metadata.extra_channel_info.items.len) |_| {
        _ = try full_image.addChannel(sh.xsizeOf(), sh.ysizeOf());
    }

    // Decode tree + histograms if present.
    var global = jxl.modular.stream.EntropyCode.init(allocator);
    defer global.deinit();
    if (has_tree != 0) {
        var tree_code = jxl.modular.stream.EntropyCode.init(
            allocator,
        );
        defer tree_code.deinit();
        try jxl.entropy.dec_ans.decodeHistograms(
            allocator,
            &sec,
            jxl.modular.ma.kNumTreeContexts,
            &tree_code.code,
            &tree_code.context_map,
            false,
        );
        var tree_reader = try jxl.modular.stream.AnsMaReader.create(
            allocator,
            &tree_code.code,
            tree_code.context_map.items,
            0,
            &sec,
        );
        defer tree_reader.ans.deinit();
        try jxl.modular.ma.decodeTree(
            &sec,
            &tree_reader,
            &global.tree,
            allocator,
            1 << 20,
        );
        try stderr.print("tree size={d}\n", .{global.tree.items.len});
        try jxl.entropy.dec_ans.decodeHistograms(
            allocator,
            &sec,
            global.numContexts(),
            &global.code,
            &global.context_map,
            false,
        );
    }

    // Now read the GroupHeader manually to dump transforms.
    var header = jxl.modular.stream.GroupHeader.init(allocator);
    defer header.deinit();
    try header.read(&sec);
    try stderr.print(
        "global group header: use_global_tree={} n_transforms={d}\n",
        .{ header.use_global_tree, header.transforms.items.len },
    );
    for (header.transforms.items, 0..) |*t, i| {
        try stderr.print(
            "  transform[{d}]: id={s} begin_c={d}",
            .{ i, @tagName(t.id), t.begin_c },
        );
        switch (t.id) {
            .rct => try stderr.print(" rct_type={d}", .{t.rct_type}),
            .palette => try stderr.print(
                " num_c={d} nb_colors={d} nb_deltas={d} predictor={d}",
                .{ t.num_c, t.nb_colors, t.nb_deltas, t.predictor },
            ),
            .squeeze => {
                try stderr.print(
                    " squeezes={d}:",
                    .{t.squeezes.items.len},
                );
                for (t.squeezes.items) |sq| {
                    try stderr.print(
                        " [h={any} in_place={any} begin_c={d} num_c={d}]",
                        .{
                            sq.horizontal,
                            sq.in_place,
                            sq.begin_c,
                            sq.num_c,
                        },
                    );
                }
            },
            .invalid => {},
        }
        try stderr.print("\n", .{});
    }

    // Apply transforms meta and dump the channel list.
    for (header.transforms.items) |*t| {
        try jxl.modular.transforms.metaApply(&full_image, t);
    }
    try stderr.print(
        "after metaApply: nb_meta={d} channels:\n",
        .{full_image.nb_meta_channels},
    );
    for (full_image.channels.items, 0..) |*ch, i| {
        try stderr.print(
            "  ch[{d}]: {d}x{d} hshift={d} vshift={d}\n",
            .{ i, ch.w, ch.h, ch.hshift, ch.vshift },
        );
    }
    try stderr.flush();
}

/// Re-encodes the first frame header with the port's writer and diffs the
/// bits against the file (LSB-first stream).
fn hdrDiff(
    allocator: std.mem.Allocator,
    br: *jxl.base.BitReader,
    metadata: *jxl.header.metadata.ImageMetadata,
    codestream: []const u8,
    stderr: *std.Io.Writer,
) !void {
    const start_bit = br.totalBitsConsumed();
    var frame = jxl.header.frame.FrameHeader.init(allocator);
    defer frame.deinit();
    try frame.read(metadata, br);
    const end_bit = br.totalBitsConsumed();

    var w = jxl.base.BitWriter.init(allocator);
    defer w.deinit();
    try frame.write(metadata, &w);
    w.zeroPadToByte();
    const written = w.takeBytes() catch w.getSpan();
    const nbits = w.bitsWritten();

    var port_bits = try allocator.alloc(u8, @max(nbits, 1));
    defer allocator.free(port_bits);
    for (0..nbits) |i| {
        port_bits[i] = '0' + @as(u8, (written[i / 8] >> @intCast(i % 8)) & 1);
    }
    var file_bits = try allocator.alloc(u8, @max(nbits, 1));
    defer allocator.free(file_bits);
    for (0..nbits) |i| {
        const bitpos = start_bit + i;
        file_bits[i] = '0' + @as(
            u8,
            (codestream[bitpos / 8] >> @intCast(bitpos % 8)) & 1,
        );
    }
    try stderr.print(
        "read {d} bits ({d}..{d}); wrote {d} bits\n",
        .{ end_bit - start_bit, start_bit, end_bit, nbits },
    );
    try stderr.print("port: {s}\n", .{port_bits});
    try stderr.print("file: {s}\n", .{file_bits});
    for (0..@min(nbits, port_bits.len)) |i| {
        if (port_bits[i] != file_bits[i]) {
            try stderr.print(
                "FIRST DIFF at bit {d} (abs {d}): port={c} file={c}\n",
                .{ i, start_bit + i, port_bits[i], file_bits[i] },
            );
            break;
        }
    }
    try stderr.flush();
}
