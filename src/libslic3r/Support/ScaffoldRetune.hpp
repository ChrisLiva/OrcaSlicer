#pragma once
#include <array>
#include <cstdint>
#include <map>
#include <vector>
#include "../ScaffoldPoints.hpp"
// The density slider's calls, apart from ScaffoldSupport.hpp: the GUI cannot include the tree generator's headers.
namespace Slic3r {
class PrintObject;
namespace ScaffoldSupport {
struct Candidates;
// The grades `draw` gave tips, by object layer and scaled x and y (`grade_key`).
using TipGrades = std::map<std::array<int64_t, 3>, double>;
// The density the candidates' slice ran at, where the slider starts, and the grades that slice gave its tips, where
// the grades `retune_points` fills start.
double    density_of(const Candidates &candidates);
TipGrades grades_of(const Candidates &candidates);
// The points an auto slice would place at `density`, graded as `draw` grades them, in ModelObject::raw_mesh()'s frame:
// the need planner again at the density, the wall skip and the alias merge. Reads the object's layers, so its slice
// must be done. `grades` takes the grades this call samples: start it from `grades_of` and pass it to every later call
// on the same candidates, so a tip graded once is not sampled again.
ScaffoldPoints retune_points(const PrintObject &object, const Candidates &candidates, double density, TipGrades &grades);
}
}
