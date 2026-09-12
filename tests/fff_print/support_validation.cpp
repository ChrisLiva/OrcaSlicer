#include "support_validation.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Utils.hpp"

#if __has_include("git_commit_hash.h")
#include "git_commit_hash.h"
#else
#define GIT_COMMIT_HASH "0000000" // no generated header: a build without the GUI
#endif

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <fstream>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>

#if defined(_WIN32)
#else
#include <sys/resource.h>
#endif

#include "test_helpers.hpp"

namespace Slic3r { namespace Test { namespace SupportValidation {

ContactClusters contact_clusters(const PrintObject &po)
{
    ContactClusters clusters;
    const double    first_z = po.slicing_parameters().first_print_layer_height;
    for (const SupportLayer *sl : po.support_layers()) {
        if (sl->print_z <= first_z + EPSILON)
            continue; // plate contact, not object contact
        for (const ExPolygon &p : union_ex(sl->tree_roof_gap_areas())) {
            clusters.areas_mm2.push_back(p.area() * SCALING_FACTOR * SCALING_FACTOR);
            clusters.centroids.emplace_back(p.contour.centroid(), sl->print_z);
        }
    }
    std::sort(clusters.areas_mm2.begin(), clusters.areas_mm2.end());
    // Serial, in sorted order: two runs sum the same floats in the same order.
    for (double a : clusters.areas_mm2)
        clusters.total_mm2 += a;
    return clusters;
}

bool same_clusters(const ContactClusters &a, const ContactClusters &b)
{
    if (a.count() != b.count())
        return false;
    for (size_t i = 0; i < a.areas_mm2.size(); ++ i)
        if (std::abs(a.areas_mm2[i] - b.areas_mm2[i]) > 1e-9)
            return false;
    for (size_t i = 0; i < a.centroids.size(); ++ i) {
        if (a.centroids[i].first != b.centroids[i].first)
            return false;
        if (std::abs(a.centroids[i].second - b.centroids[i].second) > 1e-9)
            return false;
    }
    return true;
}

SupportMetrics support_metrics(const PrintObject &po)
{
    SupportMetrics m;
    for (const SupportLayer *sl : po.support_layers()) {
        m.volume_mm3 += sl->support_fills.total_volume();
        m.footprint_regions += double(union_ex(sl->support_fills.polygons_covered_by_width(0.f)).size());
    }
    return m;
}

Envelope envelope_of(std::vector<double> v)
{
    std::sort(v.begin(), v.end());
    Envelope e;
    e.min          = v.front();
    e.max          = v.back();
    const size_t n = v.size();
    e.median       = n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
    return e;
}

bool is_false_move(const std::vector<AutoTilt::Objectives> &quality, const std::vector<char> &measured, size_t chosen)
{
    if (quality.empty() || chosen >= quality.size() || ! measured[0] || ! measured[chosen])
        return false;
    return AutoTilt::compare_objectives(quality[0], quality[chosen]) > 0;
}

double regret_of(const AutoTilt::Objectives &selected, const AutoTilt::Objectives &best)
{
    if (! selected.damage.available || ! best.damage.available)
        return std::numeric_limits<double>::quiet_NaN();
    const double mine[3]  = { selected.damage.max_group_risk, selected.damage.total_group_risk, selected.volume_mm3 };
    const double theirs[3] = { best.damage.max_group_risk, best.damage.total_group_risk, best.volume_mm3 };
    for (size_t i = 0; i < 3; ++ i)
        if (mine[i] != theirs[i])
            return (mine[i] - theirs[i]) / std::max(std::abs(theirs[i]), 1e-9);
    return 0.;
}

bool discrete_worse(const AutoTilt::Objectives &selected, const AutoTilt::Objectives &best)
{
    if (! selected.damage.available || ! best.damage.available)
        return false;
    if (selected.damage.unknown_contacts > best.damage.unknown_contacts)
        return true;
    return selected.damage.inaccessible_groups > best.damage.inaccessible_groups;
}

bool envelope_clears(const Envelope &root, const Envelope &candidate, double gain)
{
    if (! (gain > 0.))
        return false; // no gain was configured, so nothing here decides an improvement
    return root.min - candidate.max >= gain;
}

const char *outcome_name(Outcome outcome)
{
    switch (outcome) {
    case Outcome::Complete:           return "complete";
    case Outcome::UnresolvedCoverage: return "unresolved_coverage";
    case Outcome::OrganicEstimate:    return "organic_estimate";
    case Outcome::Invalid:            return "invalid";
    case Outcome::Unknown:            break;
    }
    return "unknown";
}

Metrics metrics_of(const SupportAnalysis::Report &report)
{
    Metrics m;
    m.support_volume_mm3 = report.support_volume_mm3;
    m.raft_volume_mm3    = report.raft_volume_mm3;
    // Both id lists are sorted, so the anchors that are both missing and critical are their meet.
    std::vector<uint64_t> missing_critical;
    std::set_intersection(report.missing_anchor_ids.begin(), report.missing_anchor_ids.end(),
                          report.critical_anchor_ids.begin(), report.critical_anchor_ids.end(),
                          std::back_inserter(missing_critical));
    m.missing_critical_anchors = missing_critical.size();
    m.coverage_available       = report.coverage_available;

    m.invalid_paths       = report.stability.unsupported_paths;
    m.unrooted_groups     = report.stability.unrooted_groups;
    m.min_bed_margin      = report.stability.min_bed_margin;
    m.max_slenderness     = report.stability.max_slenderness;
    m.stability_available = report.stability.available;

    m.unknown_contacts    = report.damage.unknown_contacts;
    m.inaccessible_groups = report.damage.inaccessible_groups;
    m.max_group_risk      = report.damage.max_group_risk;
    m.total_group_risk    = report.damage.total_group_risk;
    m.damage_available    = report.damage.available;
    return m;
}

Outcome outcome_of(const SupportAnalysis::Report &report)
{
    if (report.status == SupportAnalysis::Report::Status::Unknown)
        return Outcome::Unknown;
    if (report.status == SupportAnalysis::Report::Status::UnresolvedCoverage)
        return Outcome::UnresolvedCoverage;
    return Outcome::Complete;
}

// The set's outcome is the worst outcome anything in it reached: a clean object never covers for an
// object whose requirement stayed open or that nothing could measure.
Outcome worst_outcome(Outcome a, Outcome b)
{
    const auto rank = [](Outcome outcome) {
        switch (outcome) {
        case Outcome::Unknown:            return 5;
        case Outcome::Invalid:            return 4;
        case Outcome::UnresolvedCoverage: return 3;
        case Outcome::OrganicEstimate:    return 2;
        case Outcome::Complete:           break;
        }
        return 0;
    };
    return rank(a) >= rank(b) ? a : b;
}

namespace {

// The numbers one row carries out of a set of analyses measured together - the instances of one
// posed evaluation, or the objects one manifest case selected. Counts and volumes add up, the worst
// single group and the longest unbraced run are taken at their maxima and the smallest margin at its
// minimum, and a domain is available for the set only where every analysis in it measured that
// domain: an analysis that measured nothing is not a result of zero. Two entries sharing one
// analysis (a print object reading its layers through a shared owner) are two printed copies and
// count twice, the way two instances of one pose do.
Metrics accumulate_metrics(const std::vector<const SupportAnalysis::Report *> &reports)
{
    Metrics m;
    m.coverage_available  = ! reports.empty();
    m.stability_available = m.coverage_available;
    m.damage_available    = m.coverage_available;
    double min_bed_margin = std::numeric_limits<double>::infinity();
    for (const SupportAnalysis::Report *entry : reports) {
        const Metrics one = metrics_of(*entry);
        m.support_volume_mm3       += one.support_volume_mm3;
        m.raft_volume_mm3          += one.raft_volume_mm3;
        m.missing_critical_anchors += one.missing_critical_anchors;
        m.invalid_paths            += one.invalid_paths;
        m.unrooted_groups          += one.unrooted_groups;
        m.unknown_contacts         += one.unknown_contacts;
        m.inaccessible_groups      += one.inaccessible_groups;
        m.max_group_risk            = std::max(m.max_group_risk, one.max_group_risk);
        m.total_group_risk         += one.total_group_risk;
        m.max_slenderness           = std::max(m.max_slenderness, one.max_slenderness);
        min_bed_margin              = std::min(min_bed_margin, one.min_bed_margin);
        m.coverage_available        = m.coverage_available && one.coverage_available;
        m.stability_available       = m.stability_available && one.stability_available;
        m.damage_available          = m.damage_available && one.damage_available;
    }
    m.min_bed_margin = std::isfinite(min_bed_margin) ? min_bed_margin : 0.;
    return m;
}

void add_reason(std::vector<std::string> &codes, std::string code)
{
    if (std::find(codes.begin(), codes.end(), code) == codes.end())
        codes.push_back(std::move(code));
}

} // namespace

Metrics metrics_of(const AutoTilt::PoseEvaluation &evaluation)
{
    std::vector<const SupportAnalysis::Report *> reports;
    reports.reserve(evaluation.instances.size());
    for (const SupportAnalysis::Report &report : evaluation.instances)
        reports.push_back(&report);
    Metrics m = accumulate_metrics(reports);
    // The evaluation totals the material of the whole posed plate itself, so its own reading stands
    // rather than the sum of what each instance reported.
    m.support_volume_mm3 = evaluation.support_volume_mm3;
    m.raft_volume_mm3    = evaluation.raft_volume_mm3;
    return m;
}

void read_print_analyses(const Print &print, CaseResult &row)
{
    // The report each print object owns outlives this call, so the raw pointer stands in for it.
    std::vector<const SupportAnalysis::Report *> reports;
    for (const PrintObject *object : print.objects()) {
        const SupportAnalysis::Report *analysis = object->support_analysis().get();
        if (analysis == nullptr) {
            // One object nothing analysed leaves the case unmeasured: reading the row off the
            // objects that did answer would hand the gate a number for a print it does not describe.
            row.status = Outcome::Unknown;
            row.reason_codes.clear();
            row.reason_codes.emplace_back("NoSupportAnalysis");
            return;
        }
        reports.push_back(analysis);
    }
    if (reports.empty()) {
        row.status = Outcome::Unknown;
        row.reason_codes.emplace_back("NoSupportAnalysis");
        return;
    }

    row.status  = Outcome::Complete;
    row.metrics = accumulate_metrics(reports);
    for (const SupportAnalysis::Report *entry : reports) {
        row.status = worst_outcome(row.status, outcome_of(*entry));
        for (SupportAnalysis::Reason reason : entry->reasons)
            add_reason(row.reason_codes, reason_name(reason));
    }
}

void read_evaluation_analyses(const AutoTilt::PoseEvaluation &evaluation, CaseResult &row)
{
    row.status          = outcome_of(evaluation);
    row.measured        = true;
    row.plate_contained = evaluation.status != AutoTilt::PoseEvaluation::Status::Invalid;
    row.printable       = row.plate_contained;
    row.metrics         = metrics_of(evaluation);
    row.reason_codes    = evaluation.reason_codes;
}

Outcome outcome_of(const AutoTilt::PoseEvaluation &evaluation)
{
    switch (evaluation.status) {
    case AutoTilt::PoseEvaluation::Status::Invalid:
        return Outcome::Invalid;
    case AutoTilt::PoseEvaluation::Status::UnresolvedCoverage:
        return Outcome::UnresolvedCoverage;
    case AutoTilt::PoseEvaluation::Status::Complete:
        break;
    case AutoTilt::PoseEvaluation::Status::Canceled:
    case AutoTilt::PoseEvaluation::Status::Unknown:
        return Outcome::Unknown;
    }
    return Outcome::Complete;
}

const char *reason_name(SupportAnalysis::Reason reason)
{
    switch (reason) {
    case SupportAnalysis::Reason::NoProblem:              return "NoProblem";
    case SupportAnalysis::Reason::EmittedMaterialMissing: return "EmittedMaterialMissing";
    case SupportAnalysis::Reason::MissingAnchor:          return "MissingAnchor";
    case SupportAnalysis::Reason::StabilityUnavailable:   return "StabilityUnavailable";
    case SupportAnalysis::Reason::DamageUnavailable:      return "DamageUnavailable";
    }
    return "UnknownReason";
}

void write_result(const CaseResult &result, std::ostream &out)
{
    nlohmann::json row;
    row["row_type"]        = result.row_type;
    row["harness"]         = result.harness;
    row["case_id"]         = result.case_id;
    row["style"]           = result.style;
    row["feature_mode"]    = result.feature_mode;
    row["pose"]            = { { "tilt_deg", result.pose.tilt_deg }, { "lean_deg", result.pose.lean_deg } };
    row["repeat"]          = result.repeat;
    row["status"]          = outcome_name(result.status);
    row["measured"]        = result.measured;
    row["printable"]       = result.printable;
    row["plate_contained"] = result.plate_contained;
    row["estimate_only"]   = result.estimate_only;
    row["verified"]        = result.verified;
    row["reason_codes"]    = result.reason_codes;
    row["elapsed_s"]       = result.elapsed_s;
    // A peak nothing could read is a null under an explicit marker, never a zero that reads like a
    // measurement.
    row["peak_memory_available"] = result.peak_memory_bytes >= 0;
    if (result.peak_memory_bytes >= 0)
        row["peak_memory_bytes"] = result.peak_memory_bytes;
    else
        row["peak_memory_bytes"] = nullptr;
    row["source_sha256"]  = result.source_sha256;
    row["config_digest"]  = result.config_digest;
    row["build_revision"] = result.build_revision;

    nlohmann::json metrics;
    metrics["support_volume_mm3"]       = result.metrics.support_volume_mm3;
    metrics["raft_volume_mm3"]          = result.metrics.raft_volume_mm3;
    metrics["missing_critical_anchors"] = result.metrics.missing_critical_anchors;
    metrics["invalid_paths"]            = result.metrics.invalid_paths;
    metrics["unrooted_groups"]          = result.metrics.unrooted_groups;
    metrics["min_bed_margin"]           = result.metrics.min_bed_margin;
    metrics["max_slenderness"]          = result.metrics.max_slenderness;
    metrics["unknown_contacts"]         = result.metrics.unknown_contacts;
    metrics["inaccessible_groups"]      = result.metrics.inaccessible_groups;
    metrics["max_group_risk"]           = result.metrics.max_group_risk;
    metrics["total_group_risk"]         = result.metrics.total_group_risk;
    metrics["coverage_available"]       = result.metrics.coverage_available;
    metrics["stability_available"]      = result.metrics.stability_available;
    metrics["damage_available"]         = result.metrics.damage_available;
    row["metrics"] = std::move(metrics);

    if (result.selection_summary.present) {
        const SelectionSummary &s = result.selection_summary;
        nlohmann::json          summary;
        summary["root_pose"]       = { { "tilt_deg", s.root_pose.tilt_deg }, { "lean_deg", s.root_pose.lean_deg } };
        summary["selected_pose"]   = { { "tilt_deg", s.selected_pose.tilt_deg }, { "lean_deg", s.selected_pose.lean_deg } };
        summary["best_pose"]       = { { "tilt_deg", s.best_pose.tilt_deg }, { "lean_deg", s.best_pose.lean_deg } };
        summary["grid_entries"]    = s.grid_entries;
        summary["evaluated_poses"] = s.evaluated_poses;
        summary["invalid_poses"]   = s.invalid_poses;
        summary["false_move"]      = s.false_move;
        summary["discrete_worse"]   = s.discrete_worse;
        summary["regret_available"] = s.regret_available;
        if (s.regret_available)
            summary["regret"] = s.regret;
        else
            summary["regret"] = nullptr;
        row["selection_summary"] = std::move(summary);
    }

    out << row.dump() << "\n";
}

long long peak_memory_bytes()
{
#if defined(_WIN32)
    // No getrusage here, and psapi is not linked into the test binaries: the row says so rather than
    // reporting a zero that reads like a measurement.
    return -1;
#else
    struct rusage usage;
    if (getrusage(RUSAGE_SELF, &usage) != 0)
        return -1;
#if defined(__APPLE__)
    return static_cast<long long>(usage.ru_maxrss);          // bytes
#else
    return static_cast<long long>(usage.ru_maxrss) * 1024LL; // kilobytes
#endif
#endif
}

std::string build_revision()
{
    return std::string(GIT_COMMIT_HASH);
}

namespace {

std::vector<std::string> json_strings(const nlohmann::json &node, const char *key)
{
    std::vector<std::string> out;
    if (node.contains(key) && node.at(key).is_array())
        for (const nlohmann::json &item : node.at(key))
            if (item.is_string())
                out.push_back(item.get<std::string>());
    return out;
}

} // namespace

void use_os_temporary_dir()
{
    Slic3r::set_temporary_dir(std::filesystem::temp_directory_path().string());
}

Manifest load_manifest(const std::string &path)
{
    std::ifstream in(path);
    if (! in)
        throw std::runtime_error("manifest " + path + " does not exist");
    nlohmann::json document;
    try {
        in >> document;
    } catch (const std::exception &e) {
        throw std::runtime_error("manifest " + path + " does not parse: " + e.what());
    }
    Manifest manifest;
    manifest.version = document.value("version", 0);
    if (manifest.version != 1)
        throw std::runtime_error("manifest " + path + " is not version 1");

    const std::filesystem::path dir = std::filesystem::path(path).parent_path();
    manifest.model_root = (dir / document.value("model_root", std::string("."))).string();
    if (document.contains("improvement_gain") && document.at("improvement_gain").is_object())
        manifest.improvement_gain_mm3 = document.at("improvement_gain").value("support_volume_mm3", 0.);

    if (! document.contains("cases") || ! document.at("cases").is_array() || document.at("cases").empty())
        throw std::runtime_error("manifest " + path + " lists no cases");
    for (const nlohmann::json &node : document.at("cases")) {
        ManifestCase entry;
        entry.id                 = node.value("id", std::string());
        entry.model              = node.value("model", std::string());
        entry.sha256             = node.value("sha256", std::string());
        entry.config_digest      = node.value("config_digest", std::string());
        entry.scale              = node.value("scale", 1.);
        entry.category           = node.value("category", std::string());
        entry.styles             = json_strings(node, "styles");
        entry.feature_modes      = json_strings(node, "feature_modes");
        entry.nozzle_diameter_mm = node.value("nozzle_diameter_mm", 0.);
        entry.extrusion_width_mm = node.value("extrusion_width_mm", 0.);
        entry.repeats            = node.value("repeats", size_t(0));
        entry.expected_outcome   = node.value("expected_outcome", std::string());
        if (node.contains("objects") && node.at("objects").is_object()) {
            const nlohmann::json &objects = node.at("objects");
            if (objects.contains("instance_ids") && objects.at("instance_ids").is_array())
                for (const nlohmann::json &item : objects.at("instance_ids"))
                    entry.instance_ids.push_back(item.get<int>());
            entry.selectors = json_strings(objects, "selectors");
        }
        if (node.contains("config_overrides") && node.at("config_overrides").is_object())
            for (const auto &item : node.at("config_overrides").items())
                entry.config_overrides.emplace_back(item.key(), item.value().is_string() ?
                    item.value().get<std::string>() : item.value().dump());
        if (entry.id.empty() || entry.model.empty())
            throw std::runtime_error("manifest " + path + " carries a case with no id or model");
        if (entry.instance_ids.empty() && entry.selectors.empty())
            throw std::runtime_error("manifest " + path + " case " + entry.id + " selects no object");
        manifest.cases.push_back(std::move(entry));
    }
    return manifest;
}

CorpusObject case_object(const Manifest &manifest, const ManifestCase &entry, const DynamicPrintConfig &base,
                         const std::string &style, const std::string &feature_mode)
{
    const std::filesystem::path path = std::filesystem::path(manifest.model_root) / entry.model;
    DynamicPrintConfig          loaded;
    Model                       source;
    try {
        source = Model::read_from_file(path.string(), &loaded, nullptr,
            LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::AddDefaultInstances);
    } catch (const std::exception &e) {
        throw std::runtime_error("case " + entry.id + ": " + path.string() + " will not load: " + e.what());
    }

    CorpusObject out;
    out.stem = entry.id;
    // The file's own settings stay authoritative; what the case declares - the style and feature mode
    // it is being measured under, the nozzle it prints with, and the overrides it documents - is
    // applied on top of them, in that order.
    out.config = corpus_config(base, loaded);
    out.config.set_deserialize_strict({ { "support_style", style },
                                        { "support_miniature_contacts", feature_mode == "on" ? "1" : "0" } });
    if (entry.nozzle_diameter_mm > 0.) {
        std::ostringstream nozzle, width;
        nozzle << entry.nozzle_diameter_mm;
        width << entry.extrusion_width_mm;
        out.config.set_deserialize_strict({ { "nozzle_diameter", nozzle.str() } });
        if (entry.extrusion_width_mm > 0.)
            out.config.set_deserialize_strict({ { "line_width", width.str() },
                                                { "support_line_width", width.str() } });
    }
    for (const std::pair<std::string, std::string> &item : entry.config_overrides)
        out.config.set_deserialize_strict({ { item.first, item.second } });

    // The selection: object selectors name whole objects, instance ids index the file's flattened
    // instance list. Either way an empty selection is an error, never a whole-file fallback.
    std::vector<std::pair<size_t, size_t>> selected; // (object index, instance index)
    size_t                                 flat = 0;
    for (size_t oi = 0; oi < source.objects.size(); ++ oi) {
        const ModelObject *object = source.objects[oi];
        bool               whole  = false;
        for (const std::string &selector : entry.selectors) {
            if (selector.rfind("name:", 0) == 0 && selector.substr(5) == object->name)
                whole = true;
            else if (selector.rfind("index:", 0) == 0 && selector.substr(6) == std::to_string(oi))
                whole = true;
        }
        for (size_t ii = 0; ii < object->instances.size(); ++ ii, ++ flat)
            if (whole || std::find(entry.instance_ids.begin(), entry.instance_ids.end(), int(flat)) != entry.instance_ids.end())
                selected.emplace_back(oi, ii);
    }
    if (selected.empty())
        throw std::runtime_error("case " + entry.id + ": the object selection names nothing in " + path.string());

    for (size_t oi = 0; oi < source.objects.size(); ++ oi) {
        std::vector<size_t> instances;
        for (const std::pair<size_t, size_t> &pick : selected)
            if (pick.first == oi)
                instances.push_back(pick.second);
        if (instances.empty())
            continue;
        ModelObject *object = out.model.add_object(*source.objects[oi]);
        for (size_t ii = object->instances.size(); ii-- > 0; )
            if (std::find(instances.begin(), instances.end(), ii) == instances.end())
                object->delete_instance(ii);
        if (entry.scale > 0. && entry.scale != 1.)
            for (ModelInstance *instance : object->instances)
                instance->set_scaling_factor(instance->get_scaling_factor() * entry.scale);
    }
    out.model.center_instances_around_point(unscale(BoundingBox(get_bed_shape(out.config)).center()));
    for (ModelObject *object : out.model.objects)
        object->ensure_on_bed();
    return out;
}

CaseResult measure_case(const Manifest &manifest, const ManifestCase &entry, const DynamicPrintConfig &base,
                        const std::string &style, const std::string &feature_mode, size_t repeat)
{
    CaseResult row;
    row.case_id        = entry.id;
    row.style          = style;
    row.feature_mode   = feature_mode;
    row.repeat         = repeat;
    row.source_sha256  = entry.sha256;
    row.config_digest  = entry.config_digest;
    row.build_revision = build_revision();

    const auto start = std::chrono::high_resolution_clock::now();
    try {
        const CorpusObject object = case_object(manifest, entry, base, style, feature_mode);
        Slic3r::Print      print;
        print.set_status_silent();
        print.apply(object.model, object.config);
        print.request_legacy_support_analysis();
        print.process();
        row.measured = true;

        if (style == "organic") {
            // The generated evaluator refuses Organic, so what an Organic case carries is the
            // estimate and nothing the print measured: an estimate is not a verified result.
            row.status        = Outcome::OrganicEstimate;
            row.estimate_only = true;
            row.verified      = false;
            row.reason_codes.emplace_back("OrganicEstimate");
        } else {
            // A case selects objects, plural, so the row is read off every object the print holds.
            read_print_analyses(print, row);
        }
    } catch (const std::exception &e) {
        row.status    = Outcome::Unknown;
        row.verified  = false;
        row.printable = false;
        row.reason_codes.emplace_back(std::string("SliceFailed: ") + e.what());
    }
    row.elapsed_s         = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    row.peak_memory_bytes = peak_memory_bytes();
    return row;
}

size_t for_each_corpus_object(const std::string &dir, const DynamicPrintConfig &base, size_t first_index,
                              const std::function<void(size_t index, const CorpusObject &object)> &visit)
{
    size_t index = first_index;
    for (const std::filesystem::path &path : corpus_files(dir)) {
        const std::string      stem = path.stem().string();
        DynamicPrintConfig     loaded;
        Model                  model;
        try {
            // The 3mf importer creates no object without LoadModel and reads no config without
            // LoadConfig (_BBS_3MF_Importer reads both off the strategy), so the default strategy
            // hands back an empty model; load with the LoadStrategy flags the CLI's model loading in
            // OrcaSlicer.cpp uses.
            model = Model::read_from_file(path.string(), &loaded, nullptr,
                LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::AddDefaultInstances);
        } catch (const std::exception &e) {
            std::cout << "model " << index << " " << stem << " skipped: " << e.what() << std::endl;
            ++ index;
            continue;
        }
        if (model.objects.empty()) {
            std::cout << "model " << index << " " << stem << " skipped: no printable instance" << std::endl;
            ++ index;
            continue;
        }
        const DynamicPrintConfig config = corpus_config(base, loaded);
        // One harness model per object: the print measured has to hold exactly the one object, and
        // its single instance is centred on the bed and dropped onto it before slicing.
        for (const ModelObject *src : model.objects) {
            const std::string obj_stem = corpus_stem(path, model.objects.size(), src->name);
            if (src->instances.empty()) {
                std::cout << "model " << index << " " << obj_stem << " skipped: no printable instance" << std::endl;
                ++ index;
                continue;
            }
            CorpusObject one;
            one.stem         = obj_stem;
            one.config       = config;
            ModelObject *obj = one.model.add_object(*src);
            while (obj->instances.size() > 1)
                obj->delete_last_instance();
            one.model.center_instances_around_point(unscale(BoundingBox(get_bed_shape(config)).center()));
            obj->ensure_on_bed();

            visit(index, one);
            ++ index;
        }
    }
    return index;
}

}}} // namespace Slic3r::Test::SupportValidation
