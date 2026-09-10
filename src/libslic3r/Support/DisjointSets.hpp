#pragma once

#include <numeric>
#include <vector>

namespace Slic3r {

// Disjoint sets over dense indices, with path halving on find. `join(a, b)` hangs b's root under a's,
// so the root a caller reads afterwards is a's.
struct DisjointSets
{
    std::vector<size_t> parent;

    explicit DisjointSets(size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), size_t(0)); }
    size_t find(size_t i) { while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; }
    bool   join(size_t a, size_t b) { a = find(a); b = find(b); if (a == b) return false; parent[b] = a; return true; }
};

} // namespace Slic3r
