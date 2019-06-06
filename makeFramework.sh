#!/bin/bash

#------------------------------------------------------------------------------
# base
install=./CMakeExternals/Install
PACKAGENAME=pcl_mobile
CONFIGURATION=Release
IOSWRAPPER_DIR=../iOSWrapper
DEVICE_TARGET_DIR=build.ios
SIMULATOR_TARGET_DIR=build.sim64
# custom
PCL_VERSION=1.9
if [ ! -d ${install} ]; then
  echo "Install directory not found.  This script should be run from the top level build directory that contains ${install}."
  exit 1
fi

#------------------------------------------------------------------------------
make_framework_device ()
{
  # Xcode device Wrapper

  ###
  # Variable setting
  ###
  # Xcode device/simulation Wrapper framework path
  current_ios_device_framework=${IOSWRAPPER_DIR}/${DEVICE_TARGET_DIR}/${CONFIGURATION}-iphoneos/${PACKAGENAME}.framework
  current_debug_ios_device_framework=${IOSWRAPPER_DIR}/${DEVICE_TARGET_DIR}/Debug-iphoneos/${PACKAGENAME}.framework
  current_release_ios_device_framework=${IOSWRAPPER_DIR}/${DEVICE_TARGET_DIR}/Release-iphoneos/${PACKAGENAME}.framework

  device_libs=''
  module_array=("pcl" "boost" "flann" "qhull")
  # device_array=("arm64" "arm64e" "armv7" "armv7s")
  device_array=("arm64" "armv7")
  for fd in "${module_array[@]}"; do
    for ed in "${device_array[@]}"; do
      temp_libs=`find ${install}/${fd}-ios-device-${ed} -name *.a`
      device_libs+=${temp_libs}
      device_libs+=' '
    done
  done

  # common
  pcl_header_dir=${install}/pcl-ios-device-arm64/include/pcl-${PCL_VERSION}
  boost_header_dir=${install}/boost-ios-device-arm64/include
  eigen_header_dir=${install}/eigen
  flann_header_dir=${install}/flann-ios-device-arm64/include
  qhull_header_dir=${install}/qhull-ios-device-arm64/include

  # Step 2. Copy the framework structure (from iphoneos build) to the device folder
  pcl_framework=${install}/frameworks-device/${PACKAGENAME}.framework
  mkdir -p ${pcl_framework}
  rm -rf ${pcl_framework}/*

  cp -R ${current_ios_device_framework} ${pcl_framework}/..

  # mkdir ${pcl_framework}/Headers
  # common
  cp -R ${pcl_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${boost_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${eigen_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${flann_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${qhull_header_dir}/* ${pcl_framework}/Headers/ 2>&1

  # mkdir ${pcl_framework}/Modules
  cp ${IOSWRAPPER_DIR}/module.modulemap ${pcl_framework}/Modules

  # combine libraries
  # device
  libtool -static -o ${pcl_framework}/${PACKAGENAME}_device ${device_libs} ${current_ios_device_framework}/${PACKAGENAME}
  # libtool -static -o ${pcl_framework}/${PACKAGENAME}_device ${device_libs} ${current_debug_ios_device_framework}/${PACKAGENAME} ${current_release_ios_device_framework}/${PACKAGENAME}

  # combine Xcode generator frameworks and external build libraries
  lipo -create -output ${pcl_framework}/${PACKAGENAME} ${pcl_framework}/${PACKAGENAME}_device

  # remove tmp library files
  rm -rf ${pcl_framework}/${PACKAGENAME}_*
}

make_framework_simulator ()
{
  # Xcode simulation Wrapper

  ###
  # Variable setting
  ###
  current_ios_sim_framework=${IOSWRAPPER_DIR}/${SIMULATOR_TARGET_DIR}/${CONFIGURATION}-iphonesimulator/${PACKAGENAME}.framework
  current_debug_ios_sim_framework=${IOSWRAPPER_DIR}/${SIMULATOR_TARGET_DIR}/Debug-iphonesimulator/${PACKAGENAME}.framework
  current_release_ios_sim_framework=${IOSWRAPPER_DIR}/${SIMULATOR_TARGET_DIR}/Release-iphonesimulator/${PACKAGENAME}.framework

  module_array=("pcl" "boost" "flann" "qhull")
  simulator_libs=''
  # x86_64 only(64bit)
  simulation_array=("")
  # simulation_array=("-x86_64" "-i386")
  for fs in "${module_array[@]}"; do
    for es in "${simulation_array[@]}"; do
      temp_libs=`find ${install}/${fs}-ios-simulator${es} -name *.a`
      simulator_libs+=${temp_libs}
      simulator_libs+=' '
    done
  done

  pcl_header_dir=${install}/pcl-ios-simulator/include/pcl-${PCL_VERSION}
  boost_header_dir=${install}/boost-ios-simulator/include
  eigen_header_dir=${install}/eigen
  flann_header_dir=${install}/flann-ios-simulator/include
  qhull_header_dir=${install}/qhull-ios-simulator/include

  # Step 3. Copy the framework structure (from iphoneos-simulator build) to the device folder
  pcl_framework=${install}/frameworks-simulator/${PACKAGENAME}.framework
  mkdir -p ${pcl_framework}
  rm -rf ${pcl_framework}/*

  cp -R "${current_ios_sim_framework}" "${pcl_framework}/.."

  # Public Header
  # mkdir ${pcl_framework}/Headers
  cp -R ${pcl_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${boost_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${eigen_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${flann_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${qhull_header_dir}/* ${pcl_framework}/Headers/ 2>&1

  # mkdir ${pcl_framework}/Modules
  # cp module.modulemap ${pcl_framework}/Modules/
  cp ${IOSWRAPPER_DIR}/module.modulemap ${pcl_framework}/Modules

  # debug libs
  # ranlib $boost_sim_libs
  # ranlib $flann_sim_libs
  # ranlib $qhull_sim_libs
  # ranlib $pcl_sim_libs

  # combine libraries
  # simulator
  libtool -static -o ${pcl_framework}/${PACKAGENAME}_sim ${simulator_libs} ${current_ios_sim_framework}/${PACKAGENAME}
  # libtool -static -o ${pcl_framework}/${PACKAGENAME}_sim ${simulator_libs} ${current_debug_ios_sim_framework}/${PACKAGENAME} ${current_release_ios_sim_framework}/${PACKAGENAME}

  # combine Xcode generator frameworks and external build libraries
  lipo -create -output ${pcl_framework}/${PACKAGENAME} ${pcl_framework}/${PACKAGENAME}_sim

  # remove tmp library files
  rm -rf ${pcl_framework}/${PACKAGENAME}_*
}

make_framework_universal ()
{
  # Xcode device/simulation Wrapper framework path

  ###
  # Variable setting
  ###
  # Xcode device/simulation Wrapper framework path
  current_ios_device_framework=${IOSWRAPPER_DIR}/${DEVICE_TARGET_DIR}/${CONFIGURATION}-iphoneos/${PACKAGENAME}.framework
  current_debug_ios_device_framework=${IOSWRAPPER_DIR}/${DEVICE_TARGET_DIR}/Debug-iphoneos/${PACKAGENAME}.framework
  current_release_ios_device_framework=${IOSWRAPPER_DIR}/${DEVICE_TARGET_DIR}/Release-iphoneos/${PACKAGENAME}.framework
  current_ios_sim_framework=${IOSWRAPPER_DIR}/${SIMULATOR_TARGET_DIR}/${CONFIGURATION}-iphonesimulator/${PACKAGENAME}.framework
  current_debug_ios_sim_framework=${IOSWRAPPER_DIR}/${SIMULATOR_TARGET_DIR}/Debug-iphonesimulator/${PACKAGENAME}.framework
  current_release_ios_sim_framework=${IOSWRAPPER_DIR}/${SIMULATOR_TARGET_DIR}/Release-iphonesimulator/${PACKAGENAME}.framework

  module_array=("pcl" "boost" "flann" "qhull")
  device_array=("arm64" "arm64e" "armv7" "armv7s")

  device_libs=''
  for fd in ${module_array[@]}; do
    for ed in ${device_array[@]}; do
      temp_libs=`find ${install}/${fd}-ios-device-${ed} -name *.a`
      device_libs+=${temp_libs}
      device_libs+=' '
    done
  done

  simulator_libs=''
  # x86_64 only(64bit)
  simulation_array=("")
  # simulation_array=("-x86_64" "-i386")
  for fs in "${module_array[@]}"; do
    for es in "${simulation_array[@]}"; do
      temp_libs=`find ${install}/${fs}-ios-simulator${es} -name *.a`
      simulator_libs+=${temp_libs}
      simulator_libs+=' '
    done
  done

  # version ${PCL_VERSION}
  # select arm64(use device popular?)
  pcl_header_dir=${install}/pcl-ios-device-arm64/include/pcl-${PCL_VERSION}
  boost_header_dir=${install}/boost-ios-device-arm64/include
  eigen_header_dir=${install}/eigen
  flann_header_dir=${install}/flann-ios-device-arm64/include
  qhull_header_dir=${install}/qhull-ios-device-arm64/include

  ###
  # Create Framework Directory
  ###
  pcl_framework=${install}/frameworks-universal/${PACKAGENAME}.framework
  mkdir -p ${pcl_framework}
  rm -rf ${pcl_framework}/*

  ###
  # Create Framework
  ###
  cp -R "${current_ios_device_framework}" "${pcl_framework}/.."
  cp -R "${current_ios_sim_framework}" "${pcl_framework}/.."

  mkdir ${pcl_framework}/Headers
  cp -R ${pcl_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${boost_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${eigen_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${flann_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${qhull_header_dir}/* ${pcl_framework}/Headers/ 2>&1

  # mkdir ${pcl_framework}/Modules
  # cp module.modulemap ${pcl_framework}/Modules/
  cp ${IOSWRAPPER_DIR}/module.modulemap ${pcl_framework}/Modules

  # debug libs
  # ranlib $boost_sim_libs
  # ranlib $flann_sim_libs
  # ranlib $qhull_sim_libs
  # ranlib $pcl_sim_libs

  # combine libraries
  # device
  libtool -static -o ${pcl_framework}/${PACKAGENAME}_device ${device_libs} ${current_ios_device_framework}/${PACKAGENAME}
  # libtool -static -o ${pcl_framework}/${PACKAGENAME}_device ${device_libs} ${current_debug_ios_device_framework}/${PACKAGENAME} ${current_release_ios_device_framework}/${PACKAGENAME}
  # simulator
  libtool -static -o ${pcl_framework}/${PACKAGENAME}_sim ${simulator_libs} ${current_ios_sim_framework}/${PACKAGENAME}
  # libtool -static -o ${pcl_framework}/${PACKAGENAME}_sim ${simulator_libs} ${current_debug_ios_sim_framework}/${PACKAGENAME} ${current_release_ios_sim_framework}/${PACKAGENAME}

  # combine Xcode generator frameworks and external build libraries
  lipo -create -output ${pcl_framework}/${PACKAGENAME} ${pcl_framework}/${PACKAGENAME}_device ${pcl_framework}/${PACKAGENAME}_sim

  # remove tmp library files
  rm -rf ${pcl_framework}/${PACKAGENAME}_*
}

make_framework_universal2 ()
{
  # framework tree style

  ###
  # Variable setting
  ###
  # Xcode device/simulation Wrapper framework path
  current_ios_device_framework=${IOSWRAPPER_DIR}/${DEVICE_TARGET_DIR}/${CONFIGURATION}-iphoneos/${PACKAGENAME}.framework
  current_debug_ios_device_framework=${IOSWRAPPER_DIR}/${DEVICE_TARGET_DIR}/Debug-iphoneos/${PACKAGENAME}.framework
  current_release_ios_device_framework=${IOSWRAPPER_DIR}/${DEVICE_TARGET_DIR}/Release-iphoneos/${PACKAGENAME}.framework
  current_ios_sim_framework=${IOSWRAPPER_DIR/}${SIMULATOR_TARGET_DIR}/${CONFIGURATION}-iphonesimulator/${PACKAGENAME}.framework
  current_debug_ios_sim_framework=${IOSWRAPPER_DIR}/${SIMULATOR_TARGET_DIR}/Debug-iphonesimulator/${PACKAGENAME}.framework
  current_release_ios_sim_framework=${IOSWRAPPER_DIR}/${SIMULATOR_TARGET_DIR}/Release-iphonesimulator/${PACKAGENAME}.framework

  FRAMEWORK_VERSION="A"

  device_array=("arm64" "arm64e" "armv7" "armv7s")
  module_array=("pcl" "boost" "flann" "qhull")

  device_libs=''
  for ed in ${device_array[@]}; do
    for fd in ${module_array[@]}; do
      temp_libs=`find ${install}/${fd}-ios-device-${ed} -name *.a`
      device_libs+=${temp_libs}
      device_libs=' '
    done
  done

  simulator_libs=''
  # x86_64 only(64bit)
  simulation_array=("")
  # simulation_array=("-x86_64" "-i386")
  for fs in "${module_array[@]}"; do
    for es in "${simulation_array[@]}"; do
      temp_libs=`find ${install}/${fs}-ios-simulator${es} -name *.a`
      simulator_libs+=${temp_libs}
      simulator_libs+=' '
    done
  done

  # common(use arm64 headers)
  pcl_header_dir=${install}/pcl-ios-device-arm64/include/pcl-${PCL_VERSION}
  boost_header_dir=${install}/boost-ios-device-arm64/include
  eigen_header_dir=${install}/eigen
  flann_header_dir=${install}/flann-ios-device-arm64/include
  qhull_header_dir=${install}/qhull-ios-device-arm64/include

  # Create Framework Directory
  pcl_framework=${install}/frameworks-universal/${PACKAGENAME}.framework
  mkdir -p ${pcl_framework}
  rm -rf ${pcl_framework}/*

  mkdir -p ${pcl_framework}/Versions
  mkdir -p ${pcl_framework}/Versions/${FRAMEWORK_VERSION}
  mkdir -p ${pcl_framework}/Versions/${FRAMEWORK_VERSION}/Resources
  mkdir -p ${pcl_framework}/Versions/${FRAMEWORK_VERSION}/Headers
  ln -s ${FRAMEWORK_VERSION} ${pcl_framework}/Versions/Current
  ln -s Versions/Current/Headers ${pcl_framework}/Headers
  ln -s Versions/Current/Resources ${pcl_framework}/Resources
  ln -s Versions/Current/${FRAMEWORK_NAME} ${pcl_framework}/${PACKAGENAME}

  # Create Framework
  # cp -R "${current_ios_device_framework}" "${pcl_framework}/.."
  # cp -R "${current_ios_sim_framework}" "${pcl_framework}/.."
  cp -R "${current_ios_device_framework}/*.hpp" "${pcl_framework}/Headers/"
  cp -R "${current_ios_device_framework}/*.h" "${pcl_framework}/Headers/"
  cp -R "${current_ios_device_framework}/*.plist" "${pcl_framework}/Resources/"
  cp -R "${current_ios_sim_framework}/*.hpp" "${pcl_framework}/Headers/"
  cp -R "${current_ios_sim_framework}/*.h" "${pcl_framework}/Headers/"
  cp -R "${current_ios_sim_framework}/*.plist" "${pcl_framework}/Resources/"

  # Public Header
  mkdir ${pcl_framework}/Headers
  cp -R ${pcl_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${boost_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${eigen_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${flann_header_dir}/* ${pcl_framework}/Headers/ 2>&1
  cp -R ${qhull_header_dir}/* ${pcl_framework}/Headers/ 2>&1

  # mkdir ${pcl_framework}/Modules
  # cp module.modulemap ${pcl_framework}/Modules/
  cp ${IOSWRAPPER_DIR}/module.modulemap ${pcl_framework}/Modules

  # debug libs
  # ranlib $boost_sim_libs
  # ranlib $flann_sim_libs
  # ranlib $qhull_sim_libs
  # ranlib $pcl_sim_libs

  # combine libraries
  # device
  # libtool -static -o ${pcl_framework}/${PACKAGENAME}_device ${device_libs} ${current_ios_device_framework}/${PACKAGENAME}
  libtool -static -o ${pcl_framework}/${PACKAGENAME}_device ${device_libs} ${current_debug_ios_device_framework}/${PACKAGENAME} ${current_release_ios_device_framework}/${PACKAGENAME}
  # simulator
  # libtool -static -o ${pcl_framework}/${PACKAGENAME}_sim ${simulator_libs} ${current_ios_sim_framework}/${PACKAGENAME}
  libtool -static -o ${pcl_framework}/${PACKAGENAME}_sim ${simulator_libs} ${current_debug_ios_sim_framework}/${PACKAGENAME} ${current_release_ios_sim_framework}/${PACKAGENAME}

  # combine Xcode generator frameworks and external build libraries
  lipo -create -output ${pcl_framework}/Versions/Current/${PACKAGENAME} ${pcl_framework}/${PACKAGENAME}_device ${pcl_framework}/${PACKAGENAME}_sim

  # remove tmp library files
  rm -rf ${pcl_framework}/${PACKAGENAME}_*
}

#------------------------------------------------------------------------------
if [ "$1" == "universal" ]; then
  make_framework_universal
elif [ "$1" == "device" ]; then
  make_framework_device
elif [ "$1" == "simulator" ]; then
  make_framework_simulator
else
  echo "Usage: $0 universal/device/simulator"
  exit 1
fi
