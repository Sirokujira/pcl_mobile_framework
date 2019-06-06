if [ -e 'build' ]; then
    echo "Folder exists."
else
    mkdir build
fi

cd build
cmake -DBUILD_ANDROID:BOOL="OFF" -DBUILD_IOS_DEVICE:BOOL="OFF" -DBUILD_IOS_DEVICE_ARM64:BOOL="OFF" -DBUILD_IOS_DEVICE_ARM64E:BOOL="OFF" -DBUILD_IOS_DEVICE_ARMV7:BOOL="OFF" -DBUILD_IOS_DEVICE_ARMV7S:BOOL="OFF" -DBUILD_IOS_SIMULATOR:BOOL="ON" -DBUILD_IOS_SIMULATOR_X86_64:BOOL="OFF" -DBUILD_IOS_SIMULATOR_I386:BOOL="OFF" ../
make -j8
# cmake --build . --config Release
# cmake --build . --config Debug

cd ../iOSWrapper
# generate simulator project file.
# bash build-sim64.sh
# Build Simulator versions
PROJECT_NAME=pcl_mobile
CONFIGURATION=Release
# build
cmake --build build.sim64/ --config ${CONFIGURATION}
# build error(conflict CMake?)
# xcodebuild -target "${PROJECT_NAME}" -configuration ${CONFIGURATION} -sdk iphonesimulator ONLY_ACTIVE_ARCH=NO BUILD_DIR="${BUILD_DIR}" BUILD_ROOT="${BUILD_ROOT}" clean build
# cd build.sim64
# xcodebuild -target "${PROJECT_NAME}" -configuration ${CONFIGURATION} -sdk iphonesimulator ONLY_ACTIVE_ARCH=NO clean build
# cd ..

cd ../build
bash ../makeFramework.sh simulator

