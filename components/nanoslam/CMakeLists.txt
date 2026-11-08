set(NANOSLAM_SOURCES
    icp/icp.c
    icp/icp_line.c
    ls-slam/circular-queue.c
    ls-slam/graph-based-slam.c
    ls-slam/heapsort.c
    ls-slam/rcm-sparse-matrix.c
    ls-slam/sparse-matrix.c
    ls-slam/utils_math.c
    scan.c
    slam.c
    slam_graph.c
)
set(NANOSLAM_INCLUDES "." "icp" "ls-slam")

if(COMMAND idf_component_register)
    idf_component_register(SRCS ${NANOSLAM_SOURCES}
                           INCLUDE_DIRS ${NANOSLAM_INCLUDES}
                           REQUIRES esp_timer freertos heap log)
else()
    # Host build for unit tests:
    #   cmake -S components/nanoslam -B /tmp/nanoslam-build
    #   cmake --build /tmp/nanoslam-build && ctest --test-dir /tmp/nanoslam-build
    cmake_minimum_required(VERSION 3.16)
    project(nanoslam C)

    add_library(nanoslam STATIC ${NANOSLAM_SOURCES})
    target_include_directories(nanoslam PUBLIC ${NANOSLAM_INCLUDES})
    target_compile_features(nanoslam PUBLIC c_std_11)
    target_compile_options(nanoslam PRIVATE -Wall -Wextra)
    target_link_libraries(nanoslam PUBLIC m)

    enable_testing()
    add_executable(nanoslam_tests tests/test_nanoslam.c)
    target_link_libraries(nanoslam_tests PRIVATE nanoslam)
    target_compile_options(nanoslam_tests PRIVATE -Wall -Wextra -UNDEBUG)
    target_compile_definitions(nanoslam_tests PRIVATE
        NANOSLAM_TEST_DATA="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
    add_test(NAME nanoslam_tests COMMAND nanoslam_tests)
endif()
