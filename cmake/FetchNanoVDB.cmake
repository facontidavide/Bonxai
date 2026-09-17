# Fetches the NanoVDB headers so that benchmark_nanovdb can be built.
#
# NanoVDB ships inside the OpenVDB repository and is header only, so we download
# a release tarball and extract only the nanovdb/ subtree. OpenVDB's own
# CMakeLists is deliberately never added: configuring it would pull in TBB,
# Blosc and the rest of the OpenVDB build, none of which we need.
#
# By default the latest published release tag is resolved from the GitHub API.
#   -DNANOVDB_TAG=v13.1.0   pin a specific release
#   -DNANOVDB_ROOT=/path    use headers already on disk (the directory that
#                           contains nanovdb/NanoVDB.h), and skip the download
#
# On success this defines NanoVDB_FOUND and an INTERFACE target `nanovdb`.

set(NanoVDB_FOUND FALSE)

# Known-good fallback, used when the GitHub API cannot be reached.
set(NANOVDB_FALLBACK_TAG "v13.1.0")

if(CMAKE_VERSION VERSION_LESS 3.19 AND NOT NANOVDB_ROOT)
  message(WARNING
    "NanoVDB: downloading needs CMake 3.19 or newer (found ${CMAKE_VERSION}). "
    "Pass -DNANOVDB_ROOT=<dir> to use headers already on disk.")
  return()
endif()

if(NANOVDB_ROOT)
  set(_nanovdb_include_dir "${NANOVDB_ROOT}")
else()
  if(NOT NANOVDB_TAG)
    message(STATUS "NanoVDB: resolving the latest OpenVDB release tag")
    set(_nanovdb_json_file "${CMAKE_CURRENT_BINARY_DIR}/openvdb_latest_release.json")
    file(DOWNLOAD
      "https://api.github.com/repos/AcademySoftwareFoundation/openvdb/releases/latest"
      "${_nanovdb_json_file}"
      STATUS _nanovdb_status
      TIMEOUT 20
      INACTIVITY_TIMEOUT 10)

    list(GET _nanovdb_status 0 _nanovdb_status_code)
    if(_nanovdb_status_code EQUAL 0)
      file(READ "${_nanovdb_json_file}" _nanovdb_json)
      string(JSON NANOVDB_TAG ERROR_VARIABLE _nanovdb_json_error
             GET "${_nanovdb_json}" "tag_name")
      if(_nanovdb_json_error)
        set(NANOVDB_TAG "")
      endif()
    endif()

    if(NOT NANOVDB_TAG)
      set(NANOVDB_TAG "${NANOVDB_FALLBACK_TAG}")
      message(STATUS "NanoVDB: could not resolve the latest tag, falling back to ${NANOVDB_TAG}")
    endif()

    # cache it, so re-configuring does not hit the API again and so the build
    # keeps using the version it was configured with
    set(NANOVDB_TAG "${NANOVDB_TAG}" CACHE STRING "OpenVDB release providing the NanoVDB headers")
  endif()

  set(_nanovdb_dir "${CMAKE_CURRENT_BINARY_DIR}/nanovdb-${NANOVDB_TAG}")
  set(_nanovdb_tarball "${_nanovdb_dir}/openvdb.tar.gz")

  if(NOT EXISTS "${_nanovdb_dir}/extracted.stamp")
    message(STATUS "NanoVDB: downloading OpenVDB ${NANOVDB_TAG}")
    file(MAKE_DIRECTORY "${_nanovdb_dir}")
    file(DOWNLOAD
      "https://github.com/AcademySoftwareFoundation/openvdb/archive/refs/tags/${NANOVDB_TAG}.tar.gz"
      "${_nanovdb_tarball}"
      STATUS _nanovdb_status
      SHOW_PROGRESS)

    list(GET _nanovdb_status 0 _nanovdb_status_code)
    if(NOT _nanovdb_status_code EQUAL 0)
      list(GET _nanovdb_status 1 _nanovdb_status_msg)
      message(WARNING "NanoVDB: download failed (${_nanovdb_status_msg})")
      return()
    endif()

    file(ARCHIVE_EXTRACT
      INPUT "${_nanovdb_tarball}"
      DESTINATION "${_nanovdb_dir}"
      PATTERNS "*/nanovdb/nanovdb/*")
    file(REMOVE "${_nanovdb_tarball}")
    file(TOUCH "${_nanovdb_dir}/extracted.stamp")
  else()
    message(STATUS "NanoVDB: reusing OpenVDB ${NANOVDB_TAG} in ${_nanovdb_dir}")
  endif()

  # headers are included as <nanovdb/...>, so the include root is the directory
  # that *contains* the nanovdb/ folder
  file(GLOB_RECURSE _nanovdb_header "${_nanovdb_dir}/*/nanovdb/NanoVDB.h")
  if(_nanovdb_header)
    list(GET _nanovdb_header 0 _nanovdb_header)
    get_filename_component(_nanovdb_include_dir "${_nanovdb_header}" DIRECTORY)
    get_filename_component(_nanovdb_include_dir "${_nanovdb_include_dir}" DIRECTORY)
  endif()
endif()

if(_nanovdb_include_dir AND EXISTS "${_nanovdb_include_dir}/nanovdb/NanoVDB.h")
  add_library(nanovdb INTERFACE)
  target_include_directories(nanovdb SYSTEM INTERFACE "${_nanovdb_include_dir}")
  target_compile_features(nanovdb INTERFACE cxx_std_17)
  set(NanoVDB_FOUND TRUE)
  message(STATUS "NanoVDB: headers at ${_nanovdb_include_dir}")
else()
  message(WARNING "NanoVDB: NanoVDB.h not found, skipping benchmark_nanovdb")
endif()
