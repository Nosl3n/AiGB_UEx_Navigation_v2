set(ACTIVE_INFERENCE_COMMON_DIR "${CMAKE_CURRENT_LIST_DIR}/../../common")

# Shared ai_common::* libraries (see common/CMakeLists.txt), placed FIRST so they keep their own flags.
add_subdirectory(${ACTIVE_INFERENCE_COMMON_DIR} ${CMAKE_CURRENT_BINARY_DIR}/ai_common EXCLUDE_FROM_ALL)
LIST(APPEND LIBS
  ai_common::presence     # agent presence monitor + coordinator
  ai_common::core         # header-only helpers (concept_presence, world_frame, vegetation/rows.h, ...)
)

# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
)

# libdsr_core defines symbols that libdsr_{api,gui} only reference (UND); keep all libs (see bottle_concept).
add_link_options(-Wl,--no-as-needed)

# no-char8_t to avoid a problem with ICE Connection lib.
# CRITICAL — match the prebuilt libdsr's Eigen ABI: DEFAULT alignment. NEITHER -march=native NOR
# -DEIGEN_MAX_ALIGN_BYTES=0 (active_inference/CLAUDE.md, Build).
add_definitions(-O3 -fmax-errors=1 -fno-char8_t)
