# FlutterWrapper

Flutter plugin sample code for calling `pclMobile` on Android and iOS.

See:

```text
pcl_mobile_sample/
```

The sample uses a Flutter `MethodChannel` and delegates to the existing native
wrappers:

- Android: `com.sirokujira.pclmobile.pclmobileJNILib`
- iOS: `PCLMobile.framework`

It includes plugin metadata, Android/iOS native bridges, and a small Flutter
example app.
