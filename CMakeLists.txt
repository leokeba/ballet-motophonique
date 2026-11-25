# Ballet Motophonique's code: an ESP-IDF component when built by idf.py (the
# firmware in firmware/ adds this directory), host library + tests otherwise.
set(MOTOPHONIQUE_SRCS
    src/motophonique/motor.cpp
    src/motophonique/tree.cpp
    src/motophonique/views.cpp)

if(ESP_PLATFORM)
    # TesserAPI, TesserUI and TesserMotion, under whatever component name
    # they were added with (a directory in EXTRA_COMPONENT_DIRS, or the
    # component manager's "<namespace>__<name>"), as TesserKIT finds its own.
    idf_build_get_property(MOTOPHONIQUE_BUILD_COMPONENTS BUILD_COMPONENTS)
    set(MOTOPHONIQUE_DEPS "")
    foreach(want tesserapi tesserui tessermotion)
        foreach(c ${MOTOPHONIQUE_BUILD_COMPONENTS})
            string(TOLOWER "${c}" c_lower)
            if(c_lower MATCHES "(^|__)${want}$")
                list(APPEND MOTOPHONIQUE_DEPS ${c})
            endif()
        endforeach()
    endforeach()
    list(LENGTH MOTOPHONIQUE_DEPS found)
    if(NOT found EQUAL 3 AND NOT CMAKE_BUILD_EARLY_EXPANSION)
        message(FATAL_ERROR "ballet-motophonique needs TesserAPI, TesserUI and TesserMotion as components")
    endif()
    idf_component_register(SRCS ${MOTOPHONIQUE_SRCS} INCLUDE_DIRS "src")
    if(found EQUAL 3)
        foreach(dep ${MOTOPHONIQUE_DEPS})
            idf_component_get_property(lib ${dep} COMPONENT_LIB)
            target_link_libraries(${COMPONENT_LIB} PUBLIC ${lib})
        endforeach()
    endif()
    return()
endif()

cmake_minimum_required(VERSION 3.16)
project(ballet_motophonique CXX)

option(MOTOPHONIQUE_SANITIZE "Build with ASan/UBSan" ON)
set(TESSERMOTION_DIR "" CACHE PATH "TesserMotion source tree (default: ../TesserMotion)")
set(TESSERUI_DIR "" CACHE PATH "TesserUI source tree (default: ../TesserUI)")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# TesserMotion, which brings TesserAPI (../TesserAPI next to it, or
# TESSERAPI_DIR), and its testing target: the fakes, a TMC2209 among them.
if(NOT TESSERMOTION_DIR)
    get_filename_component(TESSERMOTION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../TesserMotion" ABSOLUTE)
endif()
if(NOT EXISTS "${TESSERMOTION_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "TesserMotion not found at ${TESSERMOTION_DIR}: clone it there, or set TESSERMOTION_DIR")
endif()
set(TESSERMOTION_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${TESSERMOTION_DIR}" "${CMAKE_BINARY_DIR}/tessermotion")

# TesserUI for the views (tesserui_core), on the TesserAPI already added.
if(NOT TESSERUI_DIR)
    get_filename_component(TESSERUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../TesserUI" ABSOLUTE)
endif()
if(NOT EXISTS "${TESSERUI_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "TesserUI not found at ${TESSERUI_DIR}: clone it there, or set TESSERUI_DIR")
endif()
set(TESSERUI_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${TESSERUI_DIR}" "${CMAKE_BINARY_DIR}/tesserui")

# Each checkout against deps.lock: a warning when it isn't the pinned commit
# (TesserKIT and NowTP are the firmware's, beside this repository).
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/deps.cmake")
if(NOT TESSERAPI_DIR)
    get_filename_component(TESSERAPI_DIR "${TESSERMOTION_DIR}/../TesserAPI" ABSOLUTE)
endif()
motophonique_check_sibling(TesserAPI "${TESSERAPI_DIR}")
motophonique_check_sibling(TesserUI "${TESSERUI_DIR}")
motophonique_check_sibling(TesserMotion "${TESSERMOTION_DIR}")
foreach(dep TesserKIT NowTP)
    get_filename_component(dir "${CMAKE_CURRENT_SOURCE_DIR}/../${dep}" ABSOLUTE)
    motophonique_check_sibling(${dep} "${dir}")
endforeach()

set(MOTOPHONIQUE_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror -fno-exceptions -fno-rtti)

add_library(motophonique_core ${MOTOPHONIQUE_SRCS})
target_include_directories(motophonique_core PUBLIC src)
target_link_libraries(motophonique_core PUBLIC tessermotion_core tesserui_core)
set_target_properties(motophonique_core PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
target_compile_options(motophonique_core PRIVATE ${MOTOPHONIQUE_WARNINGS})

enable_testing()
add_executable(motophonique_tests test/test_main.cpp)
target_link_libraries(motophonique_tests PRIVATE motophonique_core tessermotion_testing)
set_target_properties(motophonique_tests PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_options(motophonique_tests PRIVATE -Wall -Wextra -Werror -fno-exceptions -fno-rtti)
add_test(NAME motophonique_tests COMMAND motophonique_tests)

# A node serving the same tree over stdin/stdout, for UI checks without a
# board (DESIGN section 10): TesserUI's web/scripts/device.mjs --exe
# build/motophonique_demo.
add_executable(motophonique_demo test/demo_node.cpp)
target_link_libraries(motophonique_demo PRIVATE motophonique_core tessermotion_testing)
set_target_properties(motophonique_demo PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_options(motophonique_demo PRIVATE -Wall -Wextra -Werror -fno-exceptions -fno-rtti)

if(MOTOPHONIQUE_SANITIZE)
    # Every library in the binary, so it is instrumented alike.
    foreach(t tesser_core tesserui_core tessermotion_core tessermotion_fake_hw tessermotion_testing
              motophonique_core motophonique_tests motophonique_demo)
        target_compile_options(${t} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -g)
        target_link_options(${t} PRIVATE -fsanitize=address,undefined)
    endforeach()
endif()
