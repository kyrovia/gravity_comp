cmake_minimum_required(VERSION 3.16)
project(gravity_comp VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(GRAVITY_COMP_ENABLE_MUJOCO "Build MuJoCo simulator, scene, and related tools" OFF)
option(GRAVITY_COMP_BUILD_EXAMPLES "Build example programs" ON)
option(GRAVITY_COMP_BUILD_TESTS "Build unit tests" ON)

# Pinocchio / Eigen / Catch2: activated conda env, ROS, or GRAVITY_COMP_PINOCCHIO_ROOT.
set(GRAVITY_COMP_PINOCCHIO_ROOT "" CACHE PATH
    "Optional extra prefix for Pinocchio (e.g. ROS). Default: \$CONDA_PREFIX")
if(GRAVITY_COMP_PINOCCHIO_ROOT STREQUAL "${CMAKE_SOURCE_DIR}/.deps")
  message(STATUS "Ignoring stale GRAVITY_COMP_PINOCCHIO_ROOT=.deps; activate a conda env instead")
  unset(GRAVITY_COMP_PINOCCHIO_ROOT CACHE)
  set(GRAVITY_COMP_PINOCCHIO_ROOT "" CACHE PATH
      "Optional extra prefix for Pinocchio (e.g. ROS). Default: \$CONDA_PREFIX")
endif()

set(_gravity_comp_prefix "${GRAVITY_COMP_PINOCCHIO_ROOT}")
if(NOT _gravity_comp_prefix AND DEFINED ENV{CONDA_PREFIX})
  set(_gravity_comp_prefix "$ENV{CONDA_PREFIX}")
endif()
if(_gravity_comp_prefix)
  list(PREPEND CMAKE_PREFIX_PATH "${_gravity_comp_prefix}")
  if(EXISTS "${_gravity_comp_prefix}/lib")
    set(CMAKE_BUILD_RPATH "${_gravity_comp_prefix}/lib")
    set(CMAKE_INSTALL_RPATH "${_gravity_comp_prefix}/lib")
  endif()
  message(STATUS "Dependency prefix: ${_gravity_comp_prefix}")
endif()

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(gravity_comp
  src/pin_model.cpp
  src/gravity_compensator.cpp
)
add_library(gravity_comp::gravity_comp ALIAS gravity_comp)
target_include_directories(gravity_comp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(gravity_comp PUBLIC Eigen3::Eigen pinocchio::pinocchio)
target_compile_options(gravity_comp PRIVATE -Wall -Wextra -Wpedantic)

# MuJoCo scene keeps Menagerie MJCF armature unless GRAVITY_COMP_JOINT_ARMATURE is set.
set(GRAVITY_COMP_JOINT_ARMATURE "" CACHE STRING
    "Optional uniform MuJoCo armature override; empty keeps Menagerie MJCF values")
set(GRAVITY_COMP_MENAGERIE_UR5E "${CMAKE_SOURCE_DIR}/third_party/mujoco_menagerie/universal_robots_ur5e")
set(GRAVITY_COMP_MJCF_PATH "${GRAVITY_COMP_MENAGERIE_UR5E}/ur5e.xml")
set(GRAVITY_COMP_MODEL_PATH "${GRAVITY_COMP_MJCF_PATH}")
set(GRAVITY_COMP_MUJOCO_SCENE_PATH
    "${CMAKE_BINARY_DIR}/models/ur5e_menagerie_torque.mjb")

function(gravity_comp_use_test_headers target)
  target_include_directories(${target} PRIVATE
    "${CMAKE_SOURCE_DIR}/tests/fixtures"
  )
  target_compile_definitions(${target} PRIVATE
    GRAVITY_COMP_TEST_MODEL_PATH="${GRAVITY_COMP_MODEL_PATH}"
  )
endfunction()

if(GRAVITY_COMP_ENABLE_MUJOCO)
  set(GRAVITY_COMP_MUJOCO_ROOT "" CACHE PATH
      "Directory containing libmujoco and include/mujoco")

  set(_gravity_comp_mj_roots)
  if(GRAVITY_COMP_MUJOCO_ROOT)
    list(APPEND _gravity_comp_mj_roots "${GRAVITY_COMP_MUJOCO_ROOT}")
  endif()
  if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND _gravity_comp_mj_roots "$ENV{CONDA_PREFIX}")
    file(GLOB _gravity_comp_conda_mj
      "$ENV{CONDA_PREFIX}/lib/python*/site-packages/mujoco")
    list(APPEND _gravity_comp_mj_roots ${_gravity_comp_conda_mj})
  endif()

  set(MUJOCO_LIBRARY "")
  set(_gravity_comp_mj_root "")
  foreach(_root IN LISTS _gravity_comp_mj_roots)
    if(NOT IS_DIRECTORY "${_root}")
      continue()
    endif()
    set(_lib "")
    if(EXISTS "${_root}/lib/libmujoco.so")
      set(_lib "${_root}/lib/libmujoco.so")
    else()
      file(GLOB _libs "${_root}/libmujoco.so*")
      if(_libs)
        list(GET _libs 0 _lib)
      endif()
    endif()
    if(_lib)
      set(MUJOCO_LIBRARY "${_lib}")
      set(_gravity_comp_mj_root "${_root}")
      break()
    endif()
  endforeach()

  if(NOT MUJOCO_LIBRARY)
    find_library(MUJOCO_LIBRARY NAMES mujoco)
  endif()
  if(NOT MUJOCO_LIBRARY)
    message(FATAL_ERROR
      "MuJoCo not found. Pass -DGRAVITY_COMP_MUJOCO_ROOT=/path/to/mujoco "
      "(libmujoco + include/mujoco) or -DGRAVITY_COMP_ENABLE_MUJOCO=OFF.")
  endif()
  if(NOT _gravity_comp_mj_root)
    get_filename_component(_gravity_comp_mj_root "${MUJOCO_LIBRARY}" DIRECTORY)
  endif()

  set(GRAVITY_COMP_MUJOCO_INCLUDE "")
  foreach(_inc IN ITEMS
      "${_gravity_comp_mj_root}/include"
      "${_gravity_comp_mj_root}/../include")
    if(EXISTS "${_inc}/mujoco/mujoco.h")
      get_filename_component(GRAVITY_COMP_MUJOCO_INCLUDE "${_inc}" ABSOLUTE)
      break()
    endif()
  endforeach()
  if(NOT GRAVITY_COMP_MUJOCO_INCLUDE)
    find_path(GRAVITY_COMP_MUJOCO_INCLUDE mujoco/mujoco.h)
  endif()
  if(NOT GRAVITY_COMP_MUJOCO_INCLUDE)
    message(FATAL_ERROR "MuJoCo headers not found next to ${MUJOCO_LIBRARY}")
  endif()

  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  set(_gravity_comp_scene_python "${Python3_EXECUTABLE}")
  if(_gravity_comp_mj_root MATCHES "site-packages/mujoco/?$")
    get_filename_component(_gravity_comp_mj_site "${_gravity_comp_mj_root}" DIRECTORY)
    get_filename_component(_gravity_comp_mj_pyver "${_gravity_comp_mj_site}" DIRECTORY)
    get_filename_component(_gravity_comp_mj_lib "${_gravity_comp_mj_pyver}" DIRECTORY)
    get_filename_component(_gravity_comp_mj_env "${_gravity_comp_mj_lib}" DIRECTORY)
    foreach(_py IN ITEMS "${_gravity_comp_mj_env}/bin/python3" "${_gravity_comp_mj_env}/bin/python")
      if(EXISTS "${_py}")
        set(_gravity_comp_scene_python "${_py}")
        break()
      endif()
    endforeach()
  endif()

  if(NOT EXISTS "${GRAVITY_COMP_MJCF_PATH}")
    message(FATAL_ERROR
      "MuJoCo Menagerie UR5e not found at ${GRAVITY_COMP_MJCF_PATH}. "
      "Run: ./tools/setup_menagerie.sh")
  endif()

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/models")
  if(GRAVITY_COMP_JOINT_ARMATURE STREQUAL "")
    set(_gravity_comp_armature_args)
  else()
    set(_gravity_comp_armature_args --armature "${GRAVITY_COMP_JOINT_ARMATURE}")
  endif()

  add_custom_command(
    OUTPUT "${GRAVITY_COMP_MUJOCO_SCENE_PATH}"
    COMMAND "${_gravity_comp_scene_python}" "${CMAKE_SOURCE_DIR}/tools/build_menagerie_torque_scene.py"
            --mjcf "${GRAVITY_COMP_MJCF_PATH}"
            --output "${GRAVITY_COMP_MUJOCO_SCENE_PATH}"
            ${_gravity_comp_armature_args}
    DEPENDS "${GRAVITY_COMP_MJCF_PATH}" "${CMAKE_SOURCE_DIR}/tools/build_menagerie_torque_scene.py"
    COMMENT "Building Menagerie UR5e torque scene (ur5e_menagerie_torque.mjb)"
    VERBATIM
  )
  add_custom_target(gravity_comp_mujoco_scene DEPENDS "${GRAVITY_COMP_MUJOCO_SCENE_PATH}")

  add_library(gravity_comp_mujoco
    src/mujoco_arm_simulator.cpp
    src/gravity_validation.cpp
  )
  add_library(gravity_comp::mujoco ALIAS gravity_comp_mujoco)
  target_link_libraries(gravity_comp_mujoco PUBLIC gravity_comp)
  target_link_libraries(gravity_comp_mujoco PRIVATE "${MUJOCO_LIBRARY}")
  target_include_directories(gravity_comp_mujoco
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
      "${GRAVITY_COMP_MUJOCO_INCLUDE}"
  )
  target_compile_options(gravity_comp_mujoco PRIVATE -Wall -Wextra -Wpedantic)
  get_filename_component(MUJOCO_LIB_DIR "${MUJOCO_LIBRARY}" DIRECTORY)
  set_property(TARGET gravity_comp_mujoco APPEND PROPERTY BUILD_RPATH "${MUJOCO_LIB_DIR}")
  if(_gravity_comp_prefix AND EXISTS "${_gravity_comp_prefix}/lib")
    set_property(TARGET gravity_comp_mujoco APPEND PROPERTY BUILD_RPATH "${_gravity_comp_prefix}/lib")
  endif()
endif()

if(GRAVITY_COMP_BUILD_EXAMPLES)
  if(GRAVITY_COMP_ENABLE_MUJOCO)
    add_custom_target(ur5e_mujoco_viewer
      COMMAND "${_gravity_comp_scene_python}" "${CMAKE_SOURCE_DIR}/examples/ur5e_mujoco_viewer.py"
              --mode compare --real-time
      DEPENDS gravity_comp_mujoco_scene
      USES_TERMINAL
      COMMENT "Launch UR5e viewer: all three modes in sequence"
    )
    add_custom_target(ur5e_mujoco_viewer_no_gc
      COMMAND "${_gravity_comp_scene_python}" "${CMAKE_SOURCE_DIR}/examples/ur5e_mujoco_viewer.py"
              --mode no-gc --real-time
      DEPENDS gravity_comp_mujoco_scene
      USES_TERMINAL
      COMMENT "Launch UR5e viewer: mode 1 no gravity compensation"
    )
    add_custom_target(ur5e_mujoco_viewer_hold
      COMMAND "${_gravity_comp_scene_python}" "${CMAKE_SOURCE_DIR}/examples/ur5e_mujoco_viewer.py"
              --mode hold --real-time
      DEPENDS gravity_comp_mujoco_scene
      USES_TERMINAL
      COMMENT "Launch UR5e viewer: mode 2 gravity hold"
    )
    add_custom_target(ur5e_mujoco_viewer_push
      COMMAND "${_gravity_comp_scene_python}" "${CMAKE_SOURCE_DIR}/examples/ur5e_mujoco_viewer.py"
              --mode push --law gravity --real-time
      DEPENDS gravity_comp_mujoco_scene
      USES_TERMINAL
      COMMENT "Launch UR5e viewer: push with gravity only"
    )
    add_custom_target(ur5e_mujoco_viewer_pd
      COMMAND "${_gravity_comp_scene_python}" "${CMAKE_SOURCE_DIR}/examples/ur5e_mujoco_viewer.py"
              --mode push --law pd --real-time
      DEPENDS gravity_comp_mujoco_scene
      USES_TERMINAL
      COMMENT "Launch UR5e viewer: push with PD+gravity (recovers home)"
    )
    add_custom_target(ur5e_mujoco_viewer_servo
      COMMAND "${_gravity_comp_scene_python}" "${CMAKE_SOURCE_DIR}/examples/ur5e_mujoco_viewer.py"
              --mode push --law position-servo --real-time
      DEPENDS gravity_comp_mujoco_scene
      USES_TERMINAL
      COMMENT "Launch UR5e viewer: push with position-servo (recovers home)"
    )
  endif()
endif()

if(GRAVITY_COMP_BUILD_TESTS)
  find_package(Catch2 3 REQUIRED)
  set(GRAVITY_COMP_TEST_SOURCES
    tests/test_gravity_comp.cpp
  )
  if(GRAVITY_COMP_ENABLE_MUJOCO)
    list(APPEND GRAVITY_COMP_TEST_SOURCES tests/test_mujoco.cpp)
  endif()
  add_executable(gravity_comp_tests ${GRAVITY_COMP_TEST_SOURCES})
  target_link_libraries(gravity_comp_tests PRIVATE gravity_comp Catch2::Catch2 Catch2::Catch2WithMain)
  gravity_comp_use_test_headers(gravity_comp_tests)
  target_compile_options(gravity_comp_tests PRIVATE -Wall -Wextra -Wpedantic)
  if(GRAVITY_COMP_ENABLE_MUJOCO)
    add_dependencies(gravity_comp_tests gravity_comp_mujoco_scene)
    target_link_libraries(gravity_comp_tests PRIVATE gravity_comp_mujoco)
    target_compile_definitions(gravity_comp_tests PRIVATE
      GRAVITY_COMP_TEST_MUJOCO_SCENE_PATH="${GRAVITY_COMP_MUJOCO_SCENE_PATH}"
    )
    set_property(TARGET gravity_comp_tests APPEND PROPERTY BUILD_RPATH "${MUJOCO_LIB_DIR}")
  endif()

  enable_testing()
  add_test(NAME gravity_comp_tests COMMAND gravity_comp_tests)
  set(_gravity_comp_test_rpath "")
  if(_gravity_comp_prefix)
    set(_gravity_comp_test_rpath "${_gravity_comp_prefix}/lib")
  endif()
  if(GRAVITY_COMP_ENABLE_MUJOCO)
    set(_gravity_comp_test_rpath "${MUJOCO_LIB_DIR}:${_gravity_comp_test_rpath}")
  endif()
  set_tests_properties(gravity_comp_tests PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${_gravity_comp_test_rpath}")
endif()
