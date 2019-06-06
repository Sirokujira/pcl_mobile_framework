# download ObjectiveSharpie package url
# https://download.xamarin.com/objective-sharpie/ObjectiveSharpie.pkg
# sharpie update
# sharpie -v
# sharpie xcode -sdks

# read header define <string> error.
# sharpie bind --output ~/pcl_mobile_framework/XamarinOutput --namespace pcl_mobile_xamarin --sdk iphoneos ~/pcl_mobile_framework/build/CMakeExternals/Install/framework-universal/pcl.framework/Headers/*.h
# sharpie bind --output ~/pcl_mobile_framework/XamarinOutput --namespace pcl_mobile_xamarin --sdk iphonesimulator ~/pcl_mobile_framework/build/CMakeExternals/Install/frameworks-universal/pcl.framework/Headers/*.h
sharpie bind --output ~/pcl_mobile_framework/XamarinOutput --namespace pcl_mobile_xamarin --sdk iphoneos ~/pcl_mobile_framework/build/CMakeExternals/Install/frameworks-universal/pcl.framework/Headers/*.h

# https://docs.microsoft.com/ja-jp/xamarin/cross-platform/macios/binding/objective-sharpie/examples/cocoapod
# sharpie pod init ios pcl_mobile
# sharpie pod bind

# https://forums.xamarin.com/discussion/81418/running-objectsharpie-agains-a-framework-and-getting-errors-about-nsstring
# cd ${PCL_FRAMEWORK_FOLDER}
# target object-c header
# sharpie bind -sdk iphoneos12.1 pcl.framework/Headers/PointCloudLibraryWrapper.hpp -scope pcl.framework/Headers -c -F pcl.framework
