cmake -DBUILD_ANDROID:BOOL="OFF" -DBUILD_IOS_DEVICE:BOOL="OFF" -DBUILD_IOS_DEVICE_ARM64:BOOL="ON" -DBUILD_IOS_DEVICE_ARM64E:BOOL="ON" -DBUILD_IOS_DEVICE_ARMV7:BOOL="ON" -DBUILD_IOS_DEVICE_ARMV7S:BOOL="ON" -DBUILD_IOS_SIMULATOR:BOOL="OFF" -DBUILD_IOS_SIMULATOR_X86_64:BOOL="OFF" -DBUILD_IOS_SIMULATOR_I386:BOOL="OFF"
cd build
make -j8
# cmake --build . --config Release
# cmake --build . --config Debug

cd ../iOSWrapper
# generate device project file.
# bash build-ios.sh
# Build Device versions
PROJECT_NAME=pcl_mobile
CONFIGURATION=Release
# build
cmake --build build.ios/ --config ${CONFIGURATION}
# build error(conflict CMake?)
# xcodebuild -target "${PROJECT_NAME}" ONLY_ACTIVE_ARCH=NO -configuration ${CONFIGURATION} -sdk iphoneos BUILD_DIR="${BUILD_DIR}" BUILD_ROOT="${BUILD_ROOT}" clean build
# cd build.ios
# xcodebuild -target "${PROJECT_NAME}" ONLY_ACTIVE_ARCH=NO -configuration ${CONFIGURATION} -sdk iphoneos clean build
# cd ..

cd ../build
bash ../makeFramework.sh device

