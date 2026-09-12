get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
add_executable(${_TEST_NAME}_tests 
	${_TEST_NAME}_tests.cpp
	test_helpers.cpp
	test_helpers.hpp
	support_validation.cpp
	support_validation.hpp
	test_auto_tilt.cpp
	test_cooling.cpp
	test_extrusion_entity.cpp
	test_extrusion_processor.cpp
	test_fill.cpp
	test_flow.cpp
	test_gcode_timing.cpp
	test_gcodewriter.cpp
	test_miniature_contacts.cpp
	test_model.cpp
	test_multifilament.cpp
	test_perimeters.cpp
	test_print.cpp
	test_printobject.cpp
	test_mixed_filament.cpp
	test_skirt_brim.cpp
	test_slicing_pipeline_hook.cpp
	test_support_material.cpp
	test_tree_support.cpp
	test_trianglemesh.cpp
	test_wipe_tower.cpp
	)
# build_revision() reads GIT_COMMIT_HASH from the header src/slic3r's git_commit_hash_header target writes.
# Only this source sees that directory, so a new commit recompiles one object. A build without the GUI has
# no such target, and its harness rows carry "0000000", which the validator rejects.
if (TARGET git_commit_hash_header)
    set_source_files_properties(support_validation.cpp PROPERTIES INCLUDE_DIRECTORIES "${CMAKE_BINARY_DIR}/src/slic3r")
    add_dependencies(${_TEST_NAME}_tests git_commit_hash_header)
endif ()
target_link_libraries(${_TEST_NAME}_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(${_TEST_NAME}_tests PRIVATE ${CMAKE_SOURCE_DIR}/src/libslic3r)
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

orcaslicer_copy_test_dlls()

orcaslicer_discover_tests(${_TEST_NAME}_tests)
