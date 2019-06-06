if [ -e 'build' ]; then
    echo "Folder exists."
else
    mkdir build
fi

cd build
cmake -DBUILD_ANDROID:BOOL="OFF" -DBUILD_IOS_DEVICE:BOOL="OFF" -DBUILD_IOS_DEVICE_ARM64:BOOL="ON" -DBUILD_IOS_DEVICE_ARM64E:BOOL="ON" -DBUILD_IOS_DEVICE_ARMV7:BOOL="ON" -DBUILD_IOS_DEVICE_ARMV7S:BOOL="ON" -DBUILD_IOS_SIMULATOR:BOOL="ON" -DBUILD_IOS_SIMULATOR_X86_64:BOOL="OFF" -DBUILD_IOS_SIMULATOR_I386:BOOL="OFF" ../
make -j8
# cmake -DBUILD_ANDROID:BOOL="OFF" -DBUILD_IOS_DEVICE:BOOL="OFF" -DBUILD_IOS_DEVICE_ARM64:BOOL="ON" -DBUILD_IOS_DEVICE_ARMV7:BOOL="ON" -DBUILD_IOS_DEVICE_ARMV7S:BOOL="ON" -DBUILD_IOS_SIMULATOR:BOOL="ON" -DBUILD_IOS_SIMULATOR_X86_64:BOOL="OFF" -DBUILD_IOS_SIMULATOR_I386:BOOL="OFF" ../
# cmake --build . --config Release
# cmake --build . --config Debug

cd ../iOSWrapper
# generate device/simulator project file.
# bash build-ios.sh
# bash build-sim64.sh
# Build Device and Simulator versions
PROJECT_NAME=pcl_mobile
CONFIGURATION=Release
# build error(conflict CMake?)
# xcodebuild -target "${PROJECT_NAME}" ONLY_ACTIVE_ARCH=NO -configuration ${CONFIGURATION} -sdk iphoneos BUILD_DIR="${BUILD_DIR}" BUILD_ROOT="${BUILD_ROOT}" clean build
# xcodebuild -target "${PROJECT_NAME}" -configuration ${CONFIGURATION} -sdk iphonesimulator ONLY_ACTIVE_ARCH=NO BUILD_DIR="${BUILD_DIR}" BUILD_ROOT="${BUILD_ROOT}" clean build
cd build.universal
xcodebuild -target "${PROJECT_NAME}" ONLY_ACTIVE_ARCH=NO -configuration ${CONFIGURATION} -sdk iphoneos clean build
xcodebuild -target "${PROJECT_NAME}" -configuration Release -sdk iphonesimulator ONLY_ACTIVE_ARCH=NO clean build
# test
# xcodebuild -target "${PROJECT_NAME}" ONLY_ACTIVE_ARCH=NO -configuration Release -sdk iphoneos clean build
# xcodebuild -target "${PROJECT_NAME}" -configuration Release -sdk iphonesimulator ONLY_ACTIVE_ARCH=NO clean build
# xcodebuild -target "${PROJECT_NAME}" ONLY_ACTIVE_ARCH=NO -configuration Debug -sdk iphoneos clean build
# xcodebuild -target "${PROJECT_NAME}" -configuration Debug -sdk iphonesimulator ONLY_ACTIVE_ARCH=NO clean build
# Breaking down this command: 
# eval is used here, otherwise environment variables are not present. 
# `set -o pipefail && ` is so that the return code isn't gobbled by xcpretty (per https://github.com/supermarin/xcpretty#usage)
# then it's standard xcodebuild | xcpretty.
# eval "set -o pipefail && xcodebuild -project ${PROJECT_NAME}.xcodeproj -scheme $SCHEME -sdk $SDK -destination $DESTINATION -enableCodeCoverage YES GCC_INSTRUMENT_PROGRAM_FLOW_ARCS=YES GCC_GENERATE_TEST_COVERAGE_FILES=YES OTHERCFLAGS='-Werror' test | xcpretty" 
# bash <(curl -s https://codecov.io/bash)
cd ..

cd ../build
bash ../makeFramework.sh universal

# cd ../iOSWrapper
# bash build-test.sh
