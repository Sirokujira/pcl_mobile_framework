plugins {
    id("com.android.library")
    id("org.jetbrains.kotlin.android")
    id("maven-publish")
    id("signing")
}

group = "io.github.sirokujira"
version = providers.environmentVariable("PCLMOBILE_VERSION")
    .orElse(providers.gradleProperty("PCLMOBILE_VERSION"))
    .orElse("0.1.0") // x-release-please-version
    .get()

val androidAbis = providers.environmentVariable("ANDROID_ABIS")
    .map { value -> value.split(Regex("\\s+")).filter { it.isNotBlank() } }
    .orElse(listOf("arm64-v8a", "armeabi-v7a", "x86_64"))
    .get()
val supportedAndroidAbis = setOf("arm64-v8a", "armeabi-v7a", "x86_64")
require(androidAbis.all { it in supportedAndroidAbis }) {
    "Unsupported ANDROID_ABIS=${androidAbis.joinToString(" ")}. " +
        "Allowed: ${supportedAndroidAbis.joinToString(" ")}"
}

val androidNdkVersion = providers.environmentVariable("ANDROID_NDK_VERSION")
    .orElse(providers.environmentVariable("ANDROID_NDK_HOME").map { file(it).name })
    .orElse("29.0.14206865")
    .get()

val androidCmakeVersion = providers.environmentVariable("ANDROID_CMAKE_VERSION")
    .orElse(providers.gradleProperty("ANDROID_CMAKE_VERSION"))
    .orElse("3.31.6")
    .get()
val androidPlatform = providers.environmentVariable("ANDROID_PLATFORM")
    .orElse(providers.gradleProperty("ANDROID_PLATFORM"))
    .orElse("android-24")
    .get()

val mavenRepositoryUrl = providers.environmentVariable("MAVEN_REPOSITORY_URL")
val mavenUsername = providers.environmentVariable("MAVEN_USERNAME")
    .orElse(providers.environmentVariable("GITHUB_ACTOR"))
val mavenPassword = providers.environmentVariable("MAVEN_PASSWORD")
    .orElse(providers.environmentVariable("GITHUB_TOKEN"))

// GPG signing — required by Maven Central / Sonatype OSSRH.
// SIGNING_KEY:        Armored ASCII GPG private key (multi-line, kept in a secret).
// SIGNING_PASSWORD:   Passphrase guarding the private key.
// SIGNING_KEY_ID:     Optional 8-char short key ID (use last 8 chars of fingerprint).
// When all three are unset, signing is skipped and publishing still works for
// non-Central targets (GitHub Packages, local maven, internal Nexus, etc.).
val signingKey = providers.environmentVariable("SIGNING_KEY")
val signingPassword = providers.environmentVariable("SIGNING_PASSWORD")
val signingKeyId = providers.environmentVariable("SIGNING_KEY_ID")

android {
    namespace = "com.sirokujira.pclmobile"
    compileSdk = 34
    ndkVersion = androidNdkVersion

    defaultConfig {
        minSdk = 24
        // targetSdk is no longer used by AGP 8.x for libraries.

        ndk {
            abiFilters += androidAbis
        }

        externalNativeBuild {
            cmake {
                cppFlags += listOf("-frtti", "-fexceptions", "-std=c++17")
                arguments += listOf(
                    "-DANDROID_STL=c++_shared",
                    "-DANDROID_PLATFORM=$androidPlatform",
                )
            }
        }

        consumerProguardFiles("consumer-rules.pro")
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro",
            )
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    externalNativeBuild {
        cmake {
            path = file("CMakeLists.txt")
            version = androidCmakeVersion
        }
    }

    publishing {
        singleVariant("release") {
            withSourcesJar()
        }
    }
}

dependencies {
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test:runner:1.6.1")
    androidTestImplementation("androidx.test.ext:junit:1.2.1")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.6.1")
}

afterEvaluate {
    publishing {
        repositories {
            maven {
                name = "localRelease"
                url = uri(layout.buildDirectory.dir("repo").get().asFile)
            }

            if (mavenRepositoryUrl.isPresent) {
                maven {
                    name = "remote"
                    url = uri(mavenRepositoryUrl.get())
                    credentials {
                        username = mavenUsername.orNull
                        password = mavenPassword.orNull
                    }
                }
            }
        }

        publications {
            create<MavenPublication>("release") {
                from(components["release"])
                groupId = project.group.toString()
                artifactId = "pclmobile"
                version = project.version.toString()

                pom {
                    name.set("PCL Mobile (Android)")
                    description.set("Point Cloud Library wrapper for Android (AAR)")
                    url.set("https://github.com/Sirokujira/pcl_mobile_framework")
                    licenses {
                        license {
                            name.set("Apache License 2.0")
                            url.set("https://www.apache.org/licenses/LICENSE-2.0")
                        }
                    }
                    scm {
                        url.set("https://github.com/Sirokujira/pcl_mobile_framework")
                        connection.set("scm:git:https://github.com/Sirokujira/pcl_mobile_framework.git")
                        developerConnection.set("scm:git:ssh://git@github.com/Sirokujira/pcl_mobile_framework.git")
                    }
                    developers {
                        developer {
                            id.set("Sirokujira")
                            name.set("Sirokujira")
                        }
                    }
                }
            }
        }
    }

    signing {
        // Only sign when GPG credentials are wired in — otherwise non-Central
        // publishing targets (GitHub Packages, internal Nexus, local repo)
        // still work without signing.
        if (signingKey.isPresent && signingPassword.isPresent) {
            if (signingKeyId.isPresent) {
                useInMemoryPgpKeys(
                    signingKeyId.get(),
                    signingKey.get(),
                    signingPassword.get(),
                )
            } else {
                useInMemoryPgpKeys(signingKey.get(), signingPassword.get())
            }
            sign(publishing.publications)
        }
    }
}
