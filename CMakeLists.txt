cmake_minimum_required(VERSION 3.16)

# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  HIGH-SPEED WATER BOTTLING CONVEYOR CONTROL SYSTEM                          ║
# ║  Claude, tutored under the hand of Qiyas CC.                                ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

project(bottling-firmware
    VERSION     1.0.0
    LANGUAGES   CXX
    DESCRIPTION "5000 BPM water bottling conveyor — C++17 embedded control"
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ─────────────────────────────────────────────────────────────────────────────
# COMPILER FLAGS — safety-critical embedded profile
# ─────────────────────────────────────────────────────────────────────────────
add_compile_options(
    # Hard real-time: no exceptions, no RTTI
    -fno-exceptions
    -fno-rtti

    # Warnings as errors — zero tolerance
    -Wall
    -Wextra
    -Werror
    -Wpedantic
    -Wshadow
    -Wdouble-promotion       # float → double is a performance trap in embedded
    -Wformat=2
    -Wconversion
    -Wsign-conversion
    -Wnull-dereference
    -Wunused

    # Safety-critical extras
    -fstack-protector-strong
    -ffunction-sections       # dead code elimination with --gc-sections
    -fdata-sections
)

# ─────────────────────────────────────────────────────────────────────────────
# BUILD TARGETS
# ─────────────────────────────────────────────────────────────────────────────

# Header-only library (the control system)
add_library(bottling-core INTERFACE)
target_include_directories(bottling-core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# Simulation executable
add_executable(bottling-sim main.cpp)
target_link_libraries(bottling-sim PRIVATE bottling-core)
target_compile_definitions(bottling-sim PRIVATE BOTTLING_SIMULATION)

# ─────────────────────────────────────────────────────────────────────────────
# STATIC ANALYSIS (optional, requires clang-tidy)
# ─────────────────────────────────────────────────────────────────────────────
option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" OFF)
if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXE NAMES clang-tidy)
    if(CLANG_TIDY_EXE)
        set(CMAKE_CXX_CLANG_TIDY
            ${CLANG_TIDY_EXE}
            -checks=-*,readability-*,modernize-*,performance-*,bugprone-*,cert-*
            -warnings-as-errors=*
        )
    endif()
endif()

# ─────────────────────────────────────────────────────────────────────────────
# INSTALL
# ─────────────────────────────────────────────────────────────────────────────
install(DIRECTORY config/ hal/ middleware/ application/
        DESTINATION include/bottling
        FILES_MATCHING PATTERN "*.h")
