//! Histogram clustering. Port of libjxl `enc_cluster.h/cc` (scalar
//! equivalents of the Highway kernels) and the `Histogram` entropy helpers
//! from `enc_ans_params.h`.

const std = @import("std");
const ans_common = @import("ans_common.zig");
const fast_math = @import("../base/fast_math.zig");
const enc_ans = @import("enc_ans.zig");

const Histogram = enc_ans.Histogram;
const HistogramParams = enc_ans.HistogramParams;
const ANSHistBin = ans_common.ANSHistBin;
const fastLog2f = fast_math.fastLog2f;

pub const k_infinity: f32 = std.math.inf(f32);

/// libjxl: kClustersLimit (enc_context_map.h).
pub const k_clusters_limit: usize = 128;

/// libjxl `Histogram::Condition` (HistogramCondition): truncate trailing
/// all-zero 8-lane chunks and recompute total_count.
pub fn histogramCondition(a: *Histogram) void {
    var total: i64 = 0;
    var nz_pos: isize = -@as(isize, @intCast(Histogram.k_rounding));
    var i: usize = 0;
    while (i < a.counts.items.len) : (i += Histogram.k_rounding) {
        const chunk = a.counts.items[i .. i + Histogram.k_rounding];
        var nz = false;
        for (chunk) |c| {
            total += c;
            if (c != 0) nz = true;
        }
        if (nz) nz_pos = @intCast(i);
    }
    a.counts.shrinkRetainingCapacity(@intCast(nz_pos + Histogram.k_rounding));
    a.total_count = @intCast(total);
}

/// Per-bin entropy contribution (highway `Entropy`): 0 if count == total,
/// else -(count * FastLog2f(inv_total * count)).
fn entropyTerm(count: ANSHistBin, inv_total: f32, total: f32) f32 {
    if (@as(f32, @floatFromInt(count)) == total) return 0.0;
    const cf: f32 = @floatFromInt(count);
    return 0.0 - cf * fastLog2f(inv_total * cf);
}

/// libjxl `HistogramEntropy`: computes and caches Shannon entropy.
pub fn histogramEntropy(a: *Histogram) void {
    a.entropy = 0.0;
    if (a.total_count == 0) return;

    const inv_tot: f32 = 1.0 / @as(f32, @floatFromInt(a.total_count));
    const total: f32 = @floatFromInt(a.total_count);
    var entropy_sum: f32 = 0;
    for (a.counts.items) |c| {
        entropy_sum += entropyTerm(c, inv_tot, total);
    }
    a.entropy += entropy_sum;
}

/// libjxl `HistogramDistance`: cost of encoding both histograms with their
/// union distribution, minus individual entropies.
pub fn histogramDistance(a: *const Histogram, b: *const Histogram) f32 {
    if (a.total_count == 0 or b.total_count == 0) return 0;

    const inv_tot: f32 = 1.0 / @as(f32, @floatFromInt(a.total_count + b.total_count));
    const total: f32 = @floatFromInt(a.total_count + b.total_count);
    var distance: f32 = 0;
    const max_len = @max(a.counts.items.len, b.counts.items.len);
    for (0..max_len) |i| {
        const ac: ANSHistBin = if (i < a.counts.items.len) a.counts.items[i] else 0;
        const bc: ANSHistBin = if (i < b.counts.items.len) b.counts.items[i] else 0;
        distance += entropyTerm(ac + bc, inv_tot, total);
    }
    return distance - a.entropy - b.entropy;
}

/// libjxl `HistogramKLDivergence`.
pub fn histogramKLDivergence(actual: *const Histogram, coding: *const Histogram) f32 {
    if (actual.total_count == 0) return 0;
    if (coding.total_count == 0) return k_infinity;

    const coding_inv: f32 = 1.0 / @as(f32, @floatFromInt(coding.total_count));
    var cost: f32 = 0;

    for (actual.counts.items, 0..) |c, i| {
        if (c == 0) continue;
        const coding_count: ANSHistBin = if (i < coding.counts.items.len) coding.counts.items[i] else 0;
        if (coding_count == 0) {
            return k_infinity;
        }
        const coding_prob: f32 = @as(f32, @floatFromInt(coding_count)) * coding_inv;
        cost -= @as(f32, @floatFromInt(c)) * fastLog2f(coding_prob);
    }
    return cost - actual.entropy;
}

/// libjxl `FastClusterHistograms`: first step of k-means clustering with a
/// fancy distance metric.
fn fastClusterHistograms(
    in: []Histogram,
    max_histograms: usize,
    out: *std.ArrayList(Histogram),
    histogram_symbols: *std.ArrayList(u32),
    allocator: std.mem.Allocator,
) !void {
    const prev_histograms = out.items.len;
    try out.ensureTotalCapacity(allocator, max_histograms);
    try histogram_symbols.resize(allocator, in.len);
    @memset(histogram_symbols.items, @intCast(max_histograms));

    const dists = try allocator.alloc(f32, in.len);
    defer allocator.free(dists);
    @memset(dists, std.math.floatMax(f32));
    var largest_idx: usize = 0;
    for (in, 0..) |*h, i| {
        if (h.total_count == 0) {
            histogram_symbols.items[i] = 0;
            dists[i] = 0.0;
            continue;
        }
        histogramEntropy(&in[i]);
        if (in[i].total_count > in[largest_idx].total_count) {
            largest_idx = i;
        }
    }

    if (prev_histograms > 0) {
        for (out.items) |*h| {
            histogramEntropy(h);
        }
        for (0..in.len) |i| {
            if (dists[i] == 0.0) continue;
            for (0..prev_histograms) |j| {
                dists[i] = @min(histogramKLDivergence(&in[i], &out.items[j]), dists[i]);
            }
        }
        var max_dist: usize = 0;
        for (1..dists.len) |i| {
            if (dists[i] > dists[max_dist]) max_dist = i;
        }
        if (dists[max_dist] > 0.0) {
            largest_idx = max_dist;
        }
    }

    const k_min_distance_for_distinct: f32 = 48.0;
    while (out.items.len < max_histograms) {
        histogram_symbols.items[largest_idx] = @intCast(out.items.len);
        try out.append(allocator, in[largest_idx].clone());
        dists[largest_idx] = 0.0;
        largest_idx = 0;
        for (0..in.len) |i| {
            if (dists[i] == 0.0) continue;
            dists[i] = @min(histogramDistance(&in[i], &out.items[out.items.len - 1]), dists[i]);
            if (dists[i] > dists[largest_idx]) largest_idx = i;
        }
        if (dists[largest_idx] < k_min_distance_for_distinct) break;
    }

    for (0..in.len) |i| {
        if (histogram_symbols.items[i] != max_histograms) continue;
        var best: usize = 0;
        var best_dist: f32 = std.math.floatMax(f32);
        for (0..out.items.len) |j| {
            const dist = if (j < prev_histograms)
                histogramKLDivergence(&in[i], &out.items[j])
            else
                histogramDistance(&in[i], &out.items[j]);
            if (dist < best_dist) {
                best = j;
                best_dist = dist;
            }
        }
        if (best_dist == std.math.floatMax(f32)) return error.InvalidData;
        if (best >= prev_histograms) {
            try out.items[best].addHistogram(allocator, &in[i]);
            histogramEntropy(&out.items[best]);
        }
        histogram_symbols.items[i] = @intCast(best);
    }
}

/// Reorder histograms in `out` so that the new symbols in `symbols` come in
/// increasing order (libjxl `HistogramReindex`).
fn histogramReindex(out: *std.ArrayList(Histogram), prev_histograms: usize, symbols: *std.ArrayList(u32), allocator: std.mem.Allocator) !void {
    // Deep copy (C++: std::vector<Histogram> tmp(*out) copies the buffers).
    const tmp = try allocator.alloc(Histogram, out.items.len);
    defer {
        for (tmp) |*h| h.deinit();
        allocator.free(tmp);
    }
    for (out.items, 0..) |*h, i| tmp[i] = h.clone();

    var new_index = std.AutoHashMap(u32, u32).init(allocator);
    defer new_index.deinit();
    for (0..prev_histograms) |i| {
        try new_index.put(@intCast(i), @intCast(i));
    }
    var next_index: u32 = @intCast(prev_histograms);
    for (symbols.items) |symbol| {
        if (!new_index.contains(symbol)) {
            try new_index.put(symbol, next_index);
            out.items[next_index].deinit(); // free overwritten contents
            out.items[next_index] = tmp[symbol].clone();
            next_index += 1;
        }
    }
    // Free the dropped tail before shrinking.
    for (out.items[next_index..]) |*h| h.deinit();
    out.shrinkRetainingCapacity(next_index);
    for (symbols.items) |*symbol| {
        symbol.* = new_index.get(symbol.*).?;
    }
}

/// A pair of histograms with the merge cost; used by the kBest refinement.
const HistogramPair = struct {
    cost: f32,
    first: u32,
    second: u32,
    version: u32,
};

/// Min-heap ordered by (cost, first, second, version) ascending — mirrors
/// libjxl's priority_queue with inverted tuple comparison.
const PairHeap = struct {
    items: std.ArrayList(HistogramPair) = .empty,
    allocator: std.mem.Allocator,

    fn less(a: HistogramPair, b: HistogramPair) bool {
        if (a.cost != b.cost) return a.cost < b.cost;
        if (a.first != b.first) return a.first < b.first;
        if (a.second != b.second) return a.second < b.second;
        return a.version < b.version;
    }

    fn push(self: *PairHeap, pair: HistogramPair) !void {
        try self.items.append(self.allocator, pair);
        var i = self.items.items.len - 1;
        while (i > 0) {
            const parent = (i - 1) / 2;
            if (less(self.items.items[i], self.items.items[parent])) {
                std.mem.swap(HistogramPair, &self.items.items[i], &self.items.items[parent]);
                i = parent;
            } else break;
        }
    }

    fn pop(self: *PairHeap) ?HistogramPair {
        if (self.items.items.len == 0) return null;
        const top = self.items.items[0];
        const last = self.items.pop().?;
        if (self.items.items.len > 0) {
            self.items.items[0] = last;
            var i: usize = 0;
            while (true) {
                const l = 2 * i + 1;
                const r = 2 * i + 2;
                var smallest = i;
                if (l < self.items.items.len and less(self.items.items[l], self.items.items[smallest])) smallest = l;
                if (r < self.items.items.len and less(self.items.items[r], self.items.items[smallest])) smallest = r;
                if (smallest == i) break;
                std.mem.swap(HistogramPair, &self.items.items[i], &self.items.items[smallest]);
                i = smallest;
            }
        }
        return top;
    }

    fn isEmpty(self: *const PairHeap) bool {
        return self.items.items.len == 0;
    }
};

/// Port of `jxl::ClusterHistograms`.
pub fn clusterHistograms(
    allocator: std.mem.Allocator,
    params: *const HistogramParams,
    in: []Histogram,
    max_histograms_in: usize,
    out: *std.ArrayList(Histogram),
    histogram_symbols: *std.ArrayList(u32),
) !void {
    const prev_histograms = out.items.len;
    var max_histograms = @min(max_histograms_in, params.max_histograms);
    max_histograms = @min(max_histograms, in.len);
    if (params.clustering == .kFastest) {
        max_histograms = @min(max_histograms, 4);
    }

    try fastClusterHistograms(in, prev_histograms + max_histograms, out, histogram_symbols, allocator);

    if (prev_histograms == 0 and params.clustering == .kBest) {
        for (out.items) |*histo| {
            histo.entropy = try histo.ansPopulationCost(allocator);
        }
        var next_version: u32 = 2;
        const version = try allocator.alloc(u32, out.items.len);
        defer allocator.free(version);
        @memset(version, 1);
        const renumbering = try allocator.alloc(u32, out.items.len);
        defer allocator.free(renumbering);
        for (renumbering, 0..) |*r, i| r.* = @intCast(i);

        var heap = PairHeap{ .allocator = allocator };
        defer heap.items.deinit(allocator);

        // Create list of all pairs by increasing merging cost.
        for (0..out.items.len) |i| {
            for (i + 1..out.items.len) |j| {
                var histo = Histogram.init(allocator);
                defer histo.deinit();
                try histo.addHistogram(allocator, &out.items[i]);
                try histo.addHistogram(allocator, &out.items[j]);
                var cost = try histo.ansPopulationCost(allocator);
                cost -= out.items[i].entropy + out.items[j].entropy;
                // Avoid enqueueing pairs that are not advantageous to merge.
                if (cost >= 0) continue;
                try heap.push(.{
                    .cost = cost,
                    .first = @intCast(i),
                    .second = @intCast(j),
                    .version = @max(version[i], version[j]),
                });
            }
        }

        // Merge the best pair to merge, add new pairs that get formed.
        while (!heap.isEmpty()) {
            const top = heap.pop().?;
            const first: usize = top.first;
            const second: usize = top.second;
            const ver = top.version;
            if (ver != @max(version[first], version[second]) or
                version[first] == 0 or version[second] == 0)
            {
                continue;
            }
            try out.items[first].addHistogram(allocator, &out.items[second]);
            out.items[first].entropy = try out.items[first].ansPopulationCost(allocator);
            for (renumbering) |*item| {
                if (item.* == second) {
                    item.* = @intCast(first);
                }
            }
            version[second] = 0;
            version[first] = next_version;
            next_version += 1;
            for (0..out.items.len) |j| {
                if (j == first) continue;
                if (version[j] == 0) continue;
                var histo = Histogram.init(allocator);
                defer histo.deinit();
                try histo.addHistogram(allocator, &out.items[first]);
                try histo.addHistogram(allocator, &out.items[j]);
                var merge_cost = try histo.ansPopulationCost(allocator);
                merge_cost -= out.items[first].entropy + out.items[j].entropy;
                if (merge_cost >= 0) continue;
                try heap.push(.{
                    .cost = merge_cost,
                    .first = @intCast(@min(first, j)),
                    .second = @intCast(@max(first, j)),
                    .version = @max(version[first], version[j]),
                });
            }
        }
        const reverse_renumbering = try allocator.alloc(u32, out.items.len);
        defer allocator.free(reverse_renumbering);
        @memset(reverse_renumbering, std.math.maxInt(u32));
        var num_alive: usize = 0;
        for (0..out.items.len) |i| {
            if (version[i] == 0) continue;
            if (num_alive != i) {
                out.items[num_alive].deinit(); // free overwritten contents
                out.items[num_alive] = out.items[i];
                // The moved-from slot still aliases the moved buffer (the
                // shallow struct copy shares `counts`); empty it so neither
                // a later destination deinit nor the tail free below can
                // double-free the histogram that now lives at `num_alive`.
                out.items[i] = Histogram.init(allocator);
            }
            num_alive += 1;
            reverse_renumbering[i] = @intCast(num_alive - 1);
        }
        // Free the dropped tail before shrinking.
        for (out.items[num_alive..]) |*h| h.deinit();
        out.shrinkRetainingCapacity(num_alive);
        for (histogram_symbols.items) |*item| {
            item.* = reverse_renumbering[renumbering[item.*]];
        }
    }

    // Convert the context map to a canonical form.
    try histogramReindex(out, prev_histograms, histogram_symbols, allocator);
}

// ---------------------------------------------------------------------------

test "histogram entropy basics" {
    const alloc = std.testing.allocator;
    var h = Histogram.init(alloc);
    defer h.deinit();
    try h.add(alloc, 0);
    try h.add(alloc, 0);
    try h.add(alloc, 1);
    const e = h.shannonEntropy();
    // libjxl's ShannonEntropy is NOT normalized: it returns
    // -sum(count * log2(count/total)) = total * H = 3 * 0.9182958.
    try std.testing.expectApproxEqAbs(@as(f32, 2.754888), e, 1e-3);
    // Condition should not truncate anything here.
    histogramCondition(&h);
    try std.testing.expectEqual(@as(usize, 8), h.counts.items.len);
    try std.testing.expectEqual(@as(usize, 3), h.total_count);
}

test "histogram condition truncates trailing zeros" {
    const alloc = std.testing.allocator;
    var h = Histogram.init(alloc);
    defer h.deinit();
    try h.ensureCapacity(alloc, 32);
    h.counts.items[3] = 5;
    h.total_count = 5;
    histogramCondition(&h);
    // Only the first chunk (indices 0..7) contains nonzero.
    try std.testing.expectEqual(@as(usize, 8), h.counts.items.len);
    try std.testing.expectEqual(@as(usize, 5), h.total_count);
}

test "cluster identical histograms" {
    const alloc = std.testing.allocator;
    var params = HistogramParams{};
    params.clustering = .kBest;
    var in: std.ArrayList(Histogram) = .empty;
    defer {
        for (in.items) |*h| h.deinit();
        in.deinit(alloc);
    }
    for (0..4) |_| {
        var h = Histogram.init(alloc);
        try h.add(alloc, 1);
        try h.add(alloc, 1);
        try h.add(alloc, 2);
        try in.append(alloc, h);
    }
    var out: std.ArrayList(Histogram) = .empty;
    defer {
        for (out.items) |*h| h.deinit();
        out.deinit(alloc);
    }
    var symbols: std.ArrayList(u32) = .empty;
    defer symbols.deinit(alloc);

    try clusterHistograms(alloc, &params, in.items, k_clusters_limit, &out, &symbols);
    // All four identical histograms should merge into one cluster.
    try std.testing.expectEqual(@as(usize, 1), out.items.len);
    for (symbols.items) |s| try std.testing.expectEqual(@as(u32, 0), s);
}
