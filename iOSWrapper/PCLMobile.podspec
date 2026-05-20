Pod::Spec.new do |s|
  s.name             = 'PCLMobile'
  s.version          = '0.1.2' # x-release-please-version
  s.summary          = 'Point Cloud Library wrapper for iOS.'
  s.description      = <<-DESC
    PCLMobile bundles a curated subset of PCL (Point Cloud Library) together
    with Boost, Eigen, FLANN and Qhull as a single iOS XCFramework.
    The public surface is a small Objective-C facade — Swift consumers never
    see the underlying C++ types.
  DESC
  s.homepage         = 'https://github.com/Sirokujira/pcl_mobile_framework'

  # License: pinning the type alone is enough for an SPDX identifier
  # CocoaPods recognises. We don't include LICENSE inside the .xcframework
  # zip, so referencing :file would force consumers (and `pod trunk push`'s
  # validator) to download the source repo instead of just the binary.
  s.license          = { :type => 'Apache-2.0' }
  s.author           = { 'Sirokujira' => 'https://github.com/Sirokujira' }
  s.source           = {
    :http => "https://github.com/Sirokujira/pcl_mobile_framework/releases/download/v#{s.version}/PCLMobile.xcframework.zip"
  }

  s.platform              = :ios, '13.0'
  s.ios.deployment_target = '13.0'

  s.vendored_frameworks   = 'PCLMobile.xcframework'
  s.libraries             = 'c++'
  s.frameworks            = 'Foundation'

  s.pod_target_xcconfig = {
    'DEFINES_MODULE'              => 'YES',
    'CLANG_CXX_LIBRARY'           => 'libc++',
    'CLANG_CXX_LANGUAGE_STANDARD' => 'c++17',
    'ENABLE_BITCODE'              => 'NO',
    # Xcode 26+ / iPhoneOS 18+ SDK auto-links the private framework
    # `UIUtilities`, which CocoaPods Trunk's lint environment can't find.
    # Two-pronged fix: stop emitting LC_LINKER_OPTION for new compilations,
    # and weak-link UIUtilities so older static archives that already carry
    # the auto-link directive don't fail at link time.
    'CLANG_MODULES_AUTOLINK'      => 'NO',
    'OTHER_LDFLAGS'               => '$(inherited) -weak_framework UIUtilities',
  }
end
