cmake_minimum_required(VERSION 3.22)

# Box3D for TouchDesigner — standalone plugin project.
#
# Box3D itself is pulled from source and its src/ directory is added DIRECTLY,
# bypassing box3d's root CMakeLists on purpose: the root forces the static MSVC
# runtime (/MT) and a TD plugin DLL must use the dynamic runtime (/MD).
#
# Box3D source resolution order:
#   1. -DBOX3D_SOURCE_DIR=<path to a box3d checkout> (or a sibling ../box3d)
#   2. FetchContent from GitHub, pinned to a known-good commit.

# VERSION matters: box3d's src/CMakeLists.txt reads PROJECT_VERSION from the
# enclosing project. Keep it in sync with the box3d version being built.
project(box3d_touchdesigner VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

# Deterministic math, same flag the box3d root build uses
if (MINGW OR APPLE OR UNIX)
	add_compile_options(-ffp-contract=off)
endif()

# ---- locate box3d sources ----
set(BOX3D_SOURCE_DIR "" CACHE PATH "Path to a local box3d checkout (leave empty to auto-detect / fetch)")
set(BOX3D_GIT_TAG "0f6fdad541d30de435e13bb9b48f0c32c0b033c3" CACHE STRING "box3d commit to fetch when no local checkout is used")

if(NOT BOX3D_SOURCE_DIR AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../box3d/src/CMakeLists.txt")
	set(BOX3D_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../box3d")
endif()

if(BOX3D_SOURCE_DIR)
	message(STATUS "Box3D: using local sources at ${BOX3D_SOURCE_DIR}")
else()
	include(FetchContent)
	FetchContent_Declare(
		box3d_src
		GIT_REPOSITORY https://github.com/erincatto/box3d.git
		GIT_TAG ${BOX3D_GIT_TAG}
	)
	# Populate (download) only — we add src/ ourselves to skip the root CMakeLists
	FetchContent_GetProperties(box3d_src)
	if(NOT box3d_src_POPULATED)
		FetchContent_Populate(box3d_src)
	endif()
	set(BOX3D_SOURCE_DIR "${box3d_src_SOURCE_DIR}")
	message(STATUS "Box3D: fetched to ${BOX3D_SOURCE_DIR}")
endif()

add_subdirectory("${BOX3D_SOURCE_DIR}/src" box3d)

# ---- outputs ----
set(PLUGIN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/plugin)
foreach(config "" _DEBUG _RELEASE _MINSIZEREL _RELWITHDEBINFO)
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY${config} ${PLUGIN_OUTPUT_DIRECTORY})
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY${config} ${PLUGIN_OUTPUT_DIRECTORY})
endforeach()

# Shared core DLL: world registry + solver cores, with box3d statically
# linked inside. TD allows one custom operator per plugin DLL, so the
# operators are separate DLLs sharing state through this one. It must ship
# next to the plugin DLLs (the install script copies all DLLs).
add_library(Box3DCore SHARED
	common/Box3DTDCore.h
	common/Box3DTDCore.cpp
)
target_compile_definitions(Box3DCore PRIVATE BOX3DTD_BUILD)
target_link_libraries(Box3DCore PRIVATE box3d::box3d)
target_include_directories(Box3DCore PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/common
	# box3d headers stay visible to plugins for inline math only; box3d
	# link-level symbols are not exported from this DLL
	${BOX3D_SOURCE_DIR}/include
)

function(add_td_plugin OPNAME)
	file(GLOB PLUGIN_SRC
		"${CMAKE_CURRENT_SOURCE_DIR}/${OPNAME}/*.h"
		"${CMAKE_CURRENT_SOURCE_DIR}/${OPNAME}/*.cpp"
	)

	add_library(${OPNAME} MODULE ${PLUGIN_SRC})

	set_target_properties(${OPNAME} PROPERTIES
		PREFIX ""
		LINKER_LANGUAGE CXX
	)
	if (APPLE)
		set_target_properties(${OPNAME} PROPERTIES BUNDLE TRUE BUNDLE_EXTENSION "plugin")
	endif()

	target_include_directories(${OPNAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/sdk)
	target_link_libraries(${OPNAME} PRIVATE Box3DCore)
endfunction()

add_td_plugin(Box3DSolverCHOP)
add_td_plugin(Box3DBodiesCHOP)
add_td_plugin(Box3DBodySOP)
