# cmake/external/eigen.cmake
#
# Successor to install_eigen() in Sirokujira/pcl-superbuild@83cc231 —
# external-project-macros.cmake. See docs/LINEAGE.md.
#
# Eigen is a header-only library, so we install it once for the host. Every
# cross-compiled target sees the same headers via -I${install_prefix}/eigen.

include_guard(GLOBAL)
include(ExternalProject)

function(install_eigen)
    set(_dest "${install_prefix}/eigen")
    if(EXISTS "${_dest}/include/eigen3/Eigen/Core")
        message(STATUS "[eigen] already installed at ${_dest}, skipping fetch.")
        add_custom_target(eigen)
        return()
    endif()

    if(NOT DEFINED EIGEN_URL)
        set(EIGEN_URL
            https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz)
    endif()
    if(NOT DEFINED EIGEN_URL_HASH)
        # NOTE: placeholder; replace with the real SHA256 once verified.
        set(EIGEN_URL_HASH
            SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72)
    endif()

    ExternalProject_Add(eigen
        PREFIX            ${base}
        URL               ${EIGEN_URL}
        URL_HASH          ${EIGEN_URL_HASH}
        CMAKE_ARGS
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5
            -DCMAKE_INSTALL_PREFIX=${_dest}
            -DBUILD_TESTING=OFF
            -DEIGEN_BUILD_DOC=OFF
        BUILD_COMMAND     ""
        LOG_DOWNLOAD      ON
        LOG_CONFIGURE     ON
    )
endfunction()
