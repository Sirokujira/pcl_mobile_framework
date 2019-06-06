if [ -e 'build' ]; then
    # exists build Folder
    echo "Folder exists."
else
    # not exists build Folder
    mkdir build
fi

# change current install path and set enviroment parameters.
# export ANDROID_HOME=/opt/android-sdk-linux
# export ANDROID_NDK=/opt/android-ndk-r16b
# Target settings
# export ANDROID_ABIs=arm64-v8a
# export TOOLCHAIN_NAME=arm-linux-android-clang3.6
# export ANDROID_ABIs=x86
# export TOOLCHAIN_NAME=x86-linux-android-clang3.6
# export ANDROID_ABIs=x86_64
# export TOOLCHAIN_NAME=x86_64-linux-android-clang3.6
# export ANDROID_TARGET_API=26
# set compiler(gcc or clang[default])
# export TARGET_COMPILER=clang

cd build
# cmake -G"Ninja" -DCMAKE_TOOLCHAIN_FILE:FILEPATH=${ANDROID_NDK}/build/cmake/android.toolchain.cmake -DANDROID_TOOLCHAIN_NAME=${TOOLCHAIN_NAME} -DANDROID_NATIVE_API_LEVEL=${ANDROID_TARGET_API} -DANDROID_ABI=${ANDROID_ABIs} -DANDROID_TOOLCHAIN=${TARGET_COMPILER} -DBUILD_ANDROID:BOOL="ON" -DBUILD_IOS_DEVICE:BOOL="OFF" -DBUILD_IOS_SIMULATOR:BOOL="OFF" ../
cmake -DCMAKE_TOOLCHAIN_FILE:FILEPATH=${ANDROID_NDK}/build/cmake/android.toolchain.cmake -DANDROID_TOOLCHAIN_NAME=${TOOLCHAIN_NAME} -DANDROID_NATIVE_API_LEVEL=${ANDROID_TARGET_API} -DANDROID_ABI=${ANDROID_ABIs} -DANDROID_TOOLCHAIN=${TARGET_COMPILER} -DBUILD_ANDROID:BOOL="ON" -DBUILD_IOS_DEVICE:BOOL="OFF" -DBUILD_IOS_SIMULATOR:BOOL="OFF" ../
cmake --build . --config Release

