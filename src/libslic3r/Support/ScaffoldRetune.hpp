#pragma once
#include <vector>
#include "../ScaffoldPoints.hpp"
// The density slider's calls, apart from ScaffoldSupport.hpp: the GUI cannot include the tree generator's headers.
namespace Slic3r {
class PrintObject;
namespace ScaffoldSupport {
struct Candidates;
// The density the candidates' slice ran at, where the slider starts, and the grades that slice gave its tips, where
// the grades `retune_points` fills start.
double              density_of(const Candidates &candidates);
std::vector<double> grades_of(const Candidates &candidates);
// The points an auto slice would place at `density`, graded as `draw` grades them, in ModelObject::raw_mesh()'s frame.
// The selection's decimation and add-back run again at the density's distance, but not its placement against the risk
// field: a contact the slice kept stands where the slice put it, and one it dropped at its source. Reads the object's
// layers, so its slice must be done. `grades` holds a grade per problem seed, 0 for one not yet graded, and takes the
// grades this call samples: start it from `grades_of` and pass it to every later call on the same candidates.
ScaffoldPoints retune_points(const PrintObject &object, const Candidates &candidates, double density, std::vector<double> &grades);
}
}
