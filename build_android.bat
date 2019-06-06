REM if [ -e 'build' ]; then
REM     # ë∂ç›Ç∑ÇÈèÍçá
REM     echo "Folder exists."
REM else
REM     # ë∂ç›ÇµÇ»Ç¢èÍçá
REM     mkdir build
REM fi

set ANDROID_NDK="C:/projects/android-ndk-r16b"
set CMAKE_COMMAND="C:\\Program Files (x86)\\cmake\\bin"
REM set ANDROID_ABIs="arm64-v8a"
REM set TOOLCHAIN_NAME="arm-linux-android-clang3.6"
REM set ANDROID_ABIs="x86"
REM set TOOLCHAIN_NAME="x86-linux-android-clang3.6"
set ANDROID_ABIs="x86_64"
set TOOLCHAIN_NAME="x86_64-linux-android-clang3.6"
set ANDROID_TARGET_API="26"

set TARGET_COMPILER="clang"

set PATH=%PATH:C:\Program Files\Git\usr\bin;=%
set PATH=%ANDROID_NDK%/toolchains/%TOOLCHAIN_NAME%/prebuilt/windows-x86_64/bin;%PATH%

cd build
cmake -G "Ninja" -DCMAKE_TOOLCHAIN_FILE:FILEPATH=%ANDROID_NDK%/build/cmake/android.toolchain.cmake -DANDROID_TOOLCHAIN_NAME=%TOOLCHAIN_NAME% -DANDROID_NATIVE_API_LEVEL=%ANDROID_TARGET_API% -DANDROID_ABI=%ANDROID_ABIs% -DANDROID_TOOLCHAIN=%TARGET_COMPILER% -DBUILD_ANDROID:BOOL="ON" -DBUILD_IOS_DEVICE:BOOL="OFF" -DBUILD_IOS_SIMULATOR:BOOL="OFF" ../
cmake --build . --config Release

