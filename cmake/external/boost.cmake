# cmake/external/boost.cmake
#
# Successor to fetch_boost()/crosscompile_boost() in Sirokujira/pcl-superbuild@83cc231 —
# external-project-macros.cmake. See docs/LINEAGE.md.
#
# Cross-compile a slim Boost (PCL-required components only) for each target
# slice.
#
# Why two different b2 toolsets:
#   - iOS / iPhoneSimulator → `using darwin : <id> : <cmd> : ... ;`
#     Boost's `darwin` module is the canonical Apple cross-compile path. It
#     handles -isysroot, -arch and the `iphone` target-os flag. Importantly
#     it does NOT validate the version subfeature against a fixed list, so
#     custom slice ids work.
#
#   - Android → `using gcc : <id> : <cmd> ;`
#     `clang` toolset would route through `clang-darwin` on a macOS host and
#     reject our subfeature, mirroring the iOS problem. `gcc` is the most
#     permissive toolset; we point it at NDK's clang++ and pass `-target
#     <triple>` via the command itself.
#
# Why Android specifies `<archiver>` in the user-config.jam:
#   On macOS hosts, b2 otherwise falls back to Apple's `ar`, which cannot
#   create usable Android ELF archives. Use the NDK archiver explicitly.

include_guard(GLOBAL)
include(ExternalProject)

set(BOOST_VERSION 1.84.0)
set(BOOST_VERSION_UNDERSCORED 1_84_0)

if(NOT DEFINED BOOST_URL)
    set(BOOST_URL
        https://archives.boost.io/release/${BOOST_VERSION}/source/boost_${BOOST_VERSION_UNDERSCORED}.tar.bz2)
endif()
if(NOT DEFINED BOOST_URL_HASH)
    # NOTE: placeholder. CMake prints the actual SHA256 on first download
    # mismatch; replace this constant with that value (or pass a different
    # one via -DBOOST_URL_HASH=SHA256=... at the configure command line).
    set(BOOST_URL_HASH
        SHA256=cc4b893acf645c9d4b698e9a0f08ca8846aa5d6c68275c14c3e7949c24109454)
endif()

set(BOOST_LIBRARIES
    filesystem system thread program_options iostreams date_time regex)

# ----------------------------------------------------------------------
# Helper: replace hyphens with underscores. b2 uses '-' as the
# subfeature separator, so 'ios-arm64' confuses it. Underscores are safe.
# ----------------------------------------------------------------------
function(_boost_jam_id IN OUT_VAR)
    string(REPLACE "-" "_" _id "${IN}")
    set(${OUT_VAR} "${_id}" PARENT_SCOPE)
endfunction()

# ----------------------------------------------------------------------
# Resolve NDK path (ANDROID_NDK_HOME, falling back to ANDROID_NDK).
# ----------------------------------------------------------------------
function(_boost_resolve_ndk OUT_VAR)
    set(_ndk "$ENV{ANDROID_NDK_HOME}")
    if(_ndk STREQUAL "")
        set(_ndk "$ENV{ANDROID_NDK}")
    endif()
    if(_ndk STREQUAL "")
        message(FATAL_ERROR "[boost] ANDROID_NDK_HOME (or ANDROID_NDK) must be set.")
    endif()
    set(${OUT_VAR} "${_ndk}" PARENT_SCOPE)
endfunction()

# ----------------------------------------------------------------------
# Resolve the NDK's prebuilt-toolchain subdirectory for the current host.
# NDK r25+ ships universal `darwin-x86_64` binaries that work on Apple
# Silicon too; on Linux we want `linux-x86_64` instead.
# ----------------------------------------------------------------------
function(_boost_ndk_host_tag OUT_VAR)
    if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
        set(${OUT_VAR} "darwin-x86_64" PARENT_SCOPE)
    elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
        set(${OUT_VAR} "linux-x86_64" PARENT_SCOPE)
    elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
        set(${OUT_VAR} "windows-x86_64" PARENT_SCOPE)
    else()
        message(FATAL_ERROR "[boost] unknown host: ${CMAKE_HOST_SYSTEM_NAME}")
    endif()
endfunction()

# ----------------------------------------------------------------------
# Render a user-config.jam for one slice and return:
#   OUT_PATH    – absolute path to the generated jam file
#   OUT_TOOLSET – the toolset string to pass to b2 (`darwin-<id>` or
#                  `gcc-<id>`)
#   OUT_AR      – absolute path to the archiver to use (passed via env)
#   OUT_RANLIB  – absolute path to ranlib to use (passed via env)
#   OUT_PROPS   – extra b2 build properties (e.g. architecture/target-os)
# ----------------------------------------------------------------------
function(_boost_write_user_config TAG OUT_PATH OUT_TOOLSET OUT_AR OUT_RANLIB OUT_PROPS)
    set(_jam_path "${CMAKE_BINARY_DIR}/boost-user-config-${TAG}.jam")
    _boost_jam_id("${TAG}" _jam_id)

    if(TAG MATCHES "^android-(.+)$")
        set(_abi ${CMAKE_MATCH_1})
        pcl_mobile_android_platform(_android_platform _android_api_level)
        _boost_resolve_ndk(_ndk)
        _boost_ndk_host_tag(_host_tag)

        set(_clangxx "${_ndk}/toolchains/llvm/prebuilt/${_host_tag}/bin/clang++")
        set(_ar      "${_ndk}/toolchains/llvm/prebuilt/${_host_tag}/bin/llvm-ar")
        set(_ranlib  "${_ndk}/toolchains/llvm/prebuilt/${_host_tag}/bin/llvm-ranlib")

        if(_abi STREQUAL "arm64-v8a")
            set(_target  "aarch64-linux-android${_android_api_level}")
            set(_b2_arch "arm")
            set(_b2_addr "64")
        elseif(_abi STREQUAL "armeabi-v7a")
            set(_target  "armv7a-linux-androideabi${_android_api_level}")
            set(_b2_arch "arm")
            set(_b2_addr "32")
        elseif(_abi STREQUAL "x86_64")
            set(_target  "x86_64-linux-android${_android_api_level}")
            set(_b2_arch "x86")
            set(_b2_addr "64")
        else()
            message(FATAL_ERROR "[boost] unsupported Android ABI=${_abi}")
        endif()

        # On macOS hosts the bare `clang` toolset routes through
        # clang-darwin which validates subfeatures against a fixed list.
        # `gcc` is more permissive and works fine with NDK's clang++.
        set(_jam_content
"# Generated by cmake/external/boost.cmake -- do not edit.
using gcc : ${_jam_id} :
${_clangxx} -target ${_target} -fPIC -std=c++17 -stdlib=libc++ :
<cxxflags>\"-target ${_target} -fPIC -std=c++17 -stdlib=libc++\"
<linkflags>\"-target ${_target} -stdlib=libc++\"
<archiver>\"${_ar}\"
;
")
        set(_toolset_for_b2 "gcc-${_jam_id}")
        set(_props
            "target-os=android"
            "binary-format=elf"
            "abi=aapcs"
            "architecture=${_b2_arch}"
            "address-model=${_b2_addr}"
            "threading=multi"
        )

    elseif(TAG MATCHES "^(ios|iossim)-(.+)$")
        set(_kind ${CMAKE_MATCH_1})
        set(_arch ${CMAKE_MATCH_2})

        if(_kind STREQUAL "ios")
            set(_sdk      "iphoneos")
            set(_min_flag "-miphoneos-version-min=13.0")
        else()
            set(_sdk      "iphonesimulator")
            set(_min_flag "-mios-simulator-version-min=13.0")
        endif()

        execute_process(COMMAND xcrun --sdk ${_sdk} --show-sdk-path
                        OUTPUT_VARIABLE _sysroot
                        OUTPUT_STRIP_TRAILING_WHITESPACE)
        execute_process(COMMAND xcrun --sdk ${_sdk} --find clang++
                        OUTPUT_VARIABLE _clangxx
                        OUTPUT_STRIP_TRAILING_WHITESPACE)
        execute_process(COMMAND xcrun --sdk ${_sdk} --find ar
                        OUTPUT_VARIABLE _ar
                        OUTPUT_STRIP_TRAILING_WHITESPACE)
        execute_process(COMMAND xcrun --sdk ${_sdk} --find ranlib
                        OUTPUT_VARIABLE _ranlib
                        OUTPUT_STRIP_TRAILING_WHITESPACE)

        if(_arch STREQUAL "arm64")
            set(_b2_arch "arm")
            set(_b2_addr "64")
            set(_b2_abi  "aapcs")
        elseif(_arch STREQUAL "x86_64")
            set(_b2_arch "x86")
            set(_b2_addr "64")
            set(_b2_abi  "sysv")
        else()
            message(FATAL_ERROR "[boost] unsupported iOS arch=${_arch}")
        endif()

        # `darwin` toolset is the canonical Apple cross-compile path for
        # b2. Architecture flags MUST be in the command (4th `:` position),
        # not in <cxxflags>; <striper> and <root> go in the options block.
        #
        # `-fno-autolink` keeps the resulting `.o` files free of
        # LC_LINKER_OPTION directives (-framework UIUtilities etc.) that
        # the iPhoneOS 26+ SDK Foundation module map otherwise injects.
        # See cmake/toolchains/ios.toolchain.cmake for the equivalent
        # setting on CMake-driven dependencies.
        set(_jam_content
"# Generated by cmake/external/boost.cmake -- do not edit.
using darwin : ${_jam_id} :
${_clangxx} -arch ${_arch} -isysroot ${_sysroot} ${_min_flag} -std=c++17 -stdlib=libc++ -fPIC -fno-autolink :
<striper>
<root>\"${_sysroot}\"
:
<architecture>${_b2_arch}
<target-os>iphone
;
")
        set(_toolset_for_b2 "darwin-${_jam_id}")
        set(_props
            "target-os=iphone"
            "binary-format=mach-o"
            "abi=${_b2_abi}"
            "architecture=${_b2_arch}"
            "address-model=${_b2_addr}"
            "threading=multi"
        )

    else()
        message(FATAL_ERROR "[boost] unknown TAG=${TAG}")
    endif()

    file(WRITE "${_jam_path}" "${_jam_content}")

    set(${OUT_PATH}    "${_jam_path}"        PARENT_SCOPE)
    set(${OUT_TOOLSET} "${_toolset_for_b2}"  PARENT_SCOPE)
    set(${OUT_AR}      "${_ar}"              PARENT_SCOPE)
    set(${OUT_RANLIB}  "${_ranlib}"          PARENT_SCOPE)
    set(${OUT_PROPS}   "${_props}"           PARENT_SCOPE)
endfunction()

function(crosscompile_boost TAG)
    set(_dest        "${install_prefix}/boost-${TAG}")
    set(_target_name boost-${TAG})

    list(JOIN BOOST_LIBRARIES "," _with_libs)

    _boost_write_user_config(${TAG}
        _user_jam _toolset _ar _ranlib _b2_props)

    ExternalProject_Add(${_target_name}
        PREFIX            ${base}/${_target_name}
        URL               ${BOOST_URL}
        URL_HASH          ${BOOST_URL_HASH}
        # bootstrap.sh must run on the *host* to produce a host b2 binary.
        # Clear any iOS env vars our parent CMake might have leaked in.
        CONFIGURE_COMMAND ${CMAKE_COMMAND} -E env
            --unset=CC
            --unset=CXX
            --unset=CFLAGS
            --unset=CXXFLAGS
            --unset=LDFLAGS
            <SOURCE_DIR>/bootstrap.sh
                --prefix=${_dest}
                --with-libraries=${_with_libs}
        BUILD_IN_SOURCE   1
        # AR / RANLIB go through the env: b2 picks them up for the archive
        # step. The cross compile flags are inside user-config.jam.
        BUILD_COMMAND     ${CMAKE_COMMAND} -E env
            "AR=${_ar}"
            "RANLIB=${_ranlib}"
            <SOURCE_DIR>/b2
                -j4
                --user-config=${_user_jam}
                --prefix=${_dest}
                toolset=${_toolset}
                link=static
                variant=release
                runtime-link=static
                ${_b2_props}
                install
        INSTALL_COMMAND   ""
        LOG_DOWNLOAD      ON
        LOG_CONFIGURE     ON
        LOG_BUILD         ON
    )
endfunction()
