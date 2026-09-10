#ifndef SLIC3R_TEST_SUPPORT_VALIDATION_HPP
#define SLIC3R_TEST_SUPPORT_VALIDATION_HPP

// Corpus setup and measurement the two hidden validation harnesses share: the miniature-contact
// harness in test_miniature_contacts.cpp and the auto-tilt harness in test_auto_tilt.cpp. Only
// measurement lives here; every TEST_CASE stays in the file that owns its subsystem
// (tests/AGENTS.md, "Where a test goes").

#include "libslic3r/AutoTilt.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Support/SupportAnalysis.hpp"

#include <cstddef>
#include <functional>
#include <iosfwd>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r {

class Print;
class PrintObject;

namespace Test { namespace SupportValidation {

// One contact cluster per ExPolygon of union_ex(sl->tree_roof_gap_areas()) on every support layer
// whose print_z exceeds first_print_layer_height (plate contact is not object contact). Areas in
// mm2, sorted ascending; centroids as (scaled Point, print_z) in layer order so two runs compare
// exactly.
struct ContactClusters
{
    std::vector<double>                   areas_mm2;   // sorted ascending
    std::vector<std::pair<Point, double>> centroids;   // (contour centroid, print_z), layer order
    double                                total_mm2 = 0.;

    size_t count() const { return areas_mm2.size(); }
};

ContactClusters contact_clusters(const PrintObject &po);

// Same count, every centroid Point equal and print_z within 1e-9, every sorted area within 1e-9 mm2.
bool same_clusters(const ContactClusters &a, const ContactClusters &b);

// The two support measures the legacy generator does not reproduce from run to run (AGENTS.md
// "Testing"): the volume of support material a print extrudes, and how many separate footprint
// regions those extrusions cover. Both read SupportLayer::support_fills, the paths the generator
// emitted, and the regions are the union of the ground each path actually covers on its own layer.
struct SupportMetrics
{
    double volume_mm3        = 0.;
    double footprint_regions = 0.;
};

SupportMetrics support_metrics(const PrintObject &po);

// One leg's repeated measurements of a metric that varies: the spread it covered and its median.
struct Envelope
{
    double min = 0., max = 0., median = 0.;

    double width() const { return max - min; }
};

// Precondition: `v` is nonempty. Takes its argument by value and sorts it in place.
Envelope envelope_of(std::vector<double> v);

// Whether the search made a false move, over what a generated evaluation measured. `quality` holds
// one objective tuple per pose in column order, index 0 the root, and the tuples are compared in the
// order the verified ranking compares them: the four removal-damage fields, then the material. A
// false move is the search handing back a pose the print measures as worse than the root's; what the
// best pose in the grid scores is a matter for regret, not for this. Either end unmeasured leaves
// nothing to compare, so no false move is reported.
bool is_false_move(const std::vector<AutoTilt::Objectives> &quality, const std::vector<char> &measured, size_t chosen);

// How far the pose the search settled on sits from the best pose in the grid, on the first
// continuous objective of the production ranking the two do not tie on: the worst removal group,
// then the sum of the groups, then the material. `(selected - best) / max(|best|, 1e-9)`, so a best
// of zero is divided by the floor rather than by nothing. Two poses that tie on all three regret
// nothing; an unmeasured end regrets nothing measurable and answers NaN.
double regret_of(const AutoTilt::Objectives &selected, const AutoTilt::Objectives &best);

// Whether the selected pose carries a worse discrete safety classification than the best pose: more
// contacts nothing could answer for, or more groups nothing can reach. A worse classification is a
// failure whatever the scalar regret says, and an unmeasured end classifies nothing.
bool discrete_worse(const AutoTilt::Objectives &selected, const AutoTilt::Objectives &best);

// Whether one repeated measurement clears another: lower is better throughout, so the candidate's
// worst reading has to sit at least `gain` below the root's best reading. Overlapping spreads are
// inconclusive and answer false, and so does a gain nobody configured (zero or negative), because a
// generator that is not run-to-run reproducible cannot be read for an improvement of nothing.
bool envelope_clears(const Envelope &root, const Envelope &candidate, double gain);

// What one generated pass came to, in the vocabulary the manifest declares outcomes in.
enum class Outcome { Complete, UnresolvedCoverage, OrganicEstimate, Unknown, Invalid };

const char *outcome_name(Outcome outcome);

// The names a report's own enumerators carry into a row, so a failing outcome states its reason
// code rather than a number a reader has to look up.
const char *reason_name(SupportAnalysis::Reason reason);

// The numbers one row carries out of a SupportAnalysis::Report. Each availability flag answers for
// its own group: a domain nothing measured is not a result of zero, and the validator reads a number
// only where its flag says it was taken.
struct Metrics
{
    double support_volume_mm3      = 0.;
    double raft_volume_mm3         = 0.;
    size_t missing_critical_anchors = 0;
    size_t invalid_paths           = 0;   // printed support components reaching no permitted termination
    size_t unrooted_groups         = 0;
    double min_bed_margin          = 0.;
    double max_slenderness         = 0.;
    size_t unknown_contacts        = 0;
    size_t inaccessible_groups     = 0;
    double max_group_risk          = 0.;
    double total_group_risk        = 0.;
    bool   coverage_available      = false;
    bool   stability_available     = false;
    bool   damage_available        = false;
};

Metrics metrics_of(const SupportAnalysis::Report &report);

// The same numbers over one evaluated pose, summed across every affected instance so no instance
// hides behind another. A domain some instance did not measure is unavailable for the pose.
Metrics metrics_of(const AutoTilt::PoseEvaluation &evaluation);

// The outcome one measured report stands for, before the manifest says what was expected of it.
Outcome outcome_of(const SupportAnalysis::Report &report);
Outcome outcome_of(const AutoTilt::PoseEvaluation &evaluation);

// The worse of two outcomes, so one row over several objects states the worst any of them came to
// rather than whichever was read last.
Outcome worst_outcome(Outcome a, Outcome b);

// What the exhaustive auto-tilt sweep found over one case: which pose the production search settled
// on, what the grid's best pose measured, and how the two compare on the production comparator.
struct SelectionSummary
{
    bool           present         = false;
    AutoTilt::Pose root_pose;
    AutoTilt::Pose selected_pose;
    AutoTilt::Pose best_pose;
    size_t         grid_entries    = 0;   // poses the unchanged grid returned
    size_t         evaluated_poses = 0;   // of those, the ones the plate could hold and the evaluator measured
    size_t         invalid_poses   = 0;   // the ones exact plate containment refused
    bool           false_move      = false;
    bool           discrete_worse  = false;  // a worse safety or damage classification than the best pose
    // The scalar regret, and whether both ends of it were measured. An unmeasured comparison is a
    // null under an explicit marker, never a zero that reads like a pose with nothing to regret.
    bool           regret_available = false;
    double         regret          = 0.;
};

// One measurement of one case, one style, one feature mode, one pose, one repeat: the row both
// harnesses write and scripts/validate_miniature_supports.py reads back.
struct CaseResult
{
    std::string              row_type      = "pose";   // "pose" or "selection"
    std::string              harness;
    std::string              case_id;
    std::string              style;
    std::string              feature_mode;
    AutoTilt::Pose           pose;
    size_t                   repeat        = 0;
    Outcome                  status        = Outcome::Unknown;
    bool                     measured      = false;
    bool                     printable     = true;
    bool                     plate_contained = true;
    bool                     estimate_only = false;
    bool                     verified      = true;
    std::vector<std::string> reason_codes;
    Metrics                  metrics;
    double                   elapsed_s     = 0.;
    // Negative means the platform gave no reading: the row then carries an explicit unavailable
    // marker and a null, never a zero that reads like a measurement.
    long long                peak_memory_bytes = -1;
    std::string              source_sha256;
    std::string              config_digest;
    std::string              build_revision;
    SelectionSummary         selection_summary;
};

// Fills one row's status, metrics and reason codes from the analyses of every object a sliced print
// holds, the way a pose's instances are summed. A manifest case selects objects, plural, so reading
// the row off whichever object came first would drop the other objects' open requirements, missing
// anchors, invalid paths and risk. The row's outcome is the worst outcome any object reached, and a
// print holding an object with no analysis at all is Unknown rather than a row read off the objects
// that did answer.
void read_print_analyses(const Print &print, CaseResult &row);

// The same for one pose evaluation: status, metrics, whether the plate held the pose, and every
// reason code the evaluation and its instances carried. The caller adds the case's own identity and
// the timings it measured around the call.
void read_evaluation_analyses(const AutoTilt::PoseEvaluation &evaluation, CaseResult &row);

// One JSON object per line, terminated by a newline. Serialised through the JSON library the tests
// already carry, so no value is escaped by hand.
void write_result(const CaseResult &result, std::ostream &out);

// The process's peak resident set in bytes, or a negative value where the platform offers none.
long long peak_memory_bytes();

// The commit this binary was built from, or "unknown" where the build did not stamp one.
std::string build_revision();

// One manifest case, as scripts/validate_miniature_supports.py validated it before this binary ran.
struct ManifestCase
{
    std::string              id;
    std::string              model;          // relative to the manifest's model root
    std::string              sha256;
    std::string              config_digest;
    std::vector<int>         instance_ids;   // 0-based indices into the file's flattened instance list
    std::vector<std::string> selectors;      // "name:<object name>" or "index:<object index>"
    double                   scale = 1.;
    std::string              category;
    std::vector<std::string> styles;
    std::vector<std::string> feature_modes;
    double                   nozzle_diameter_mm = 0.;
    double                   extrusion_width_mm = 0.;
    size_t                   repeats = 0;
    std::string              expected_outcome;
    std::vector<std::pair<std::string, std::string>> config_overrides;
};

struct Manifest
{
    int                       version = 0;
    std::string               model_root;   // absolute, resolved against the manifest's own directory
    double                    improvement_gain_mm3 = 0.;
    std::vector<ManifestCase> cases;
};

// Reads the manifest at `path`. Throws std::runtime_error naming the file when it does not exist,
// does not parse, or is not version 1.
Manifest load_manifest(const std::string &path);

// Model::get_backup_path() builds from temporary_dir(), which is "" in a test process, so each 3mf
// load logs two "Failed to create backup path /orcaslicer_model/...: Read-only file system" errors
// that read like a failure but are caught and non-fatal. Points it at the OS temp dir, the way the
// app's CLI startup does with set_temporary_dir.
void use_os_temporary_dir();

// One corpus object as both harnesses slice it: exactly one object carrying exactly one instance,
// centred on the bed and dropped onto it, under the config its file carried overlaid on the base.
struct CorpusObject
{
    std::string        stem;
    Slic3r::Model      model;
    DynamicPrintConfig config;
};

// Loads every .stl and .3mf directly under `dir` in name order and calls `visit` once per printable
// object, numbering them from `first_index`. A file that will not load, and an object with no
// printable instance, are reported on std::cout and skipped. An empty or missing directory visits
// nothing. Returns the index after the last object visited.
size_t for_each_corpus_object(const std::string &dir, const DynamicPrintConfig &base, size_t first_index,
                              const std::function<void(size_t index, const CorpusObject &object)> &visit);

// The model a manifest case names, loaded and reduced to the objects and instances it selects,
// centred on the bed and dropped onto it, under `base` overlaid with the file's own config and then
// the case's documented overrides. Throws std::runtime_error when the file will not load or the
// selection names nothing.
CorpusObject case_object(const Manifest &manifest, const ManifestCase &entry, const DynamicPrintConfig &base,
                         const std::string &style, const std::string &feature_mode);

// Slices one manifest case under one style, one feature mode and one repeat, asks the print for the
// legacy support analysis and reads the row off it: status, metrics, reason codes, the runtime the
// generation took and the peak the process reached. A style the evaluator does not answer for -
// Organic - is recorded as an estimate and never as a generated result. Every field but `harness` is
// filled in; nothing here throws, because a case that will not slice is a row that says so.
CaseResult measure_case(const Manifest &manifest, const ManifestCase &entry, const DynamicPrintConfig &base,
                        const std::string &style, const std::string &feature_mode, size_t repeat);

}}} // namespace Slic3r::Test::SupportValidation

#endif // SLIC3R_TEST_SUPPORT_VALIDATION_HPP
