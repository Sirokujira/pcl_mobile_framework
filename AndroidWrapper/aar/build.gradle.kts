// Top-level build file. Apply plugins on the subprojects that actually need them.
plugins {
    id("com.android.application") version "8.5.2" apply false
    id("com.android.library") version "8.5.2" apply false
    id("org.jetbrains.kotlin.android") version "1.9.24" apply false
    // Nexus Publish Plugin: stages the signed AAR + POM to Sonatype, then
    // closes and releases the staging repository in one task. Works against
    // both the legacy OSSRH endpoint (s01.oss.sonatype.org) and the new
    // Central Portal (central.sonatype.com) via the same Gradle API.
    id("io.github.gradle-nexus.publish-plugin") version "2.0.0"
}

// ----------------------------------------------------------------------------
// Sonatype Maven Central staging repository
// ----------------------------------------------------------------------------
//
// Drives the `publishToSonatype`, `closeSonatypeStagingRepository` and
// `releaseSonatypeStagingRepository` tasks. Sourced from the same env vars
// as the existing maven-publish flow so secrets stay in one place.
//
// Endpoint defaults to the s01 legacy URL because most existing PCL Mobile
// Sonatype accounts predate the Central Portal migration. Override with
// SONATYPE_URL=https://central.sonatype.com/api/v1/publisher/upload (and
// the matching snapshot URL) when the namespace is on the new Portal.

nexusPublishing {
    repositories {
        sonatype {
            nexusUrl.set(
                uri(
                    providers.environmentVariable("SONATYPE_URL")
                        .orElse("https://s01.oss.sonatype.org/service/local/")
                        .get()
                )
            )
            snapshotRepositoryUrl.set(
                uri(
                    providers.environmentVariable("SONATYPE_SNAPSHOT_URL")
                        .orElse("https://s01.oss.sonatype.org/content/repositories/snapshots/")
                        .get()
                )
            )
            username.set(providers.environmentVariable("OSSRH_USERNAME"))
            password.set(providers.environmentVariable("OSSRH_PASSWORD"))
        }
    }
}

tasks.register<Delete>("clean") {
    delete(rootProject.layout.buildDirectory)
}
