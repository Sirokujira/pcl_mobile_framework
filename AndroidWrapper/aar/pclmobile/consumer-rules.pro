# Rules consumed by apps that depend on this AAR.
# Keep all native-method-bearing classes intact: ProGuard mustn't rename them
# or the JNI symbol table breaks.
-keepclasseswithmembers class com.sirokujira.pclmobile.** {
    native <methods>;
}
