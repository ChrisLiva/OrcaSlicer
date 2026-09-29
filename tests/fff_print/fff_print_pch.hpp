// Precompiled header of fff_print_tests. Every test source parses Model.hpp and Print.hpp through test_helpers.hpp,
// and those two headers cost about 5 s per source without it. A header listed here rebuilds this header on each edit,
// so keep out anything a test edits more often than the hub headers. CMake opens the generated header with
// "#pragma clang system_header", which silences warnings in everything listed, so test headers stay out.
#include "pchheader.hpp"   // libslic3r's own precompiled set, found through the target's src/libslic3r include path

#include <catch2/catch_all.hpp>

#include "libslic3r/Geometry.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"
