Pod::Spec.new do |s|
  s.name             = 'pcl_mobile_flutter'
  s.version          = '0.1.0'
  s.summary          = 'Flutter bridge for pclMobile.'
  s.description      = 'Calls the existing PCLMobile iOS framework from Flutter.'
  s.homepage         = 'https://github.com/Sirokujira/pcl_mobile_framework'
  s.license          = { :file => '../../../LICENSE' }
  s.author           = { 'Sirokujira' => 'Sirokujira' }
  s.source           = { :path => '.' }
  s.source_files     = 'Classes/**/*'
  s.dependency 'Flutter'
  s.platform = :ios, '13.0'
  s.swift_version = '5.0'
  s.vendored_frameworks = '../../../../../../../../build/ios/xcframework/PCLMobile.xcframework'
  s.pod_target_xcconfig = {
    'DEFINES_MODULE' => 'YES',
    'FRAMEWORK_SEARCH_PATHS[sdk=iphoneos*]' => '$(inherited) "${PODS_ROOT}/../../../../../build/ios/xcframework/PCLMobile.xcframework/ios-arm64"',
    'FRAMEWORK_SEARCH_PATHS[sdk=iphonesimulator*]' => '$(inherited) "${PODS_ROOT}/../../../../../build/ios/xcframework/PCLMobile.xcframework/ios-arm64_x86_64-simulator"',
    'OTHER_LDFLAGS' => '$(inherited) -framework PCLMobile',
  }
  s.user_target_xcconfig = {
    'FRAMEWORK_SEARCH_PATHS[sdk=iphoneos*]' => '$(inherited) "${PODS_ROOT}/../../../../../build/ios/xcframework/PCLMobile.xcframework/ios-arm64"',
    'FRAMEWORK_SEARCH_PATHS[sdk=iphonesimulator*]' => '$(inherited) "${PODS_ROOT}/../../../../../build/ios/xcframework/PCLMobile.xcframework/ios-arm64_x86_64-simulator"',
    'OTHER_LDFLAGS' => '$(inherited) -framework PCLMobile',
  }
end
