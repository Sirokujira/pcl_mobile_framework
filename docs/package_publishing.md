# Package publishing setup

## TL;DR — release flow

PCLMobile releases are driven by Conventional Commits + release-please:

1. Land changes on `master` with Conventional Commits
   (`feat:`, `fix:`, `feat!:`, etc.).
2. [`release-please.yml`](../.github/workflows/release-please.yml)
   maintains a rolling **Release PR** with the next version + CHANGELOG.
3. Merge the Release PR. release-please tags `vX.Y.Z` automatically.
4. The tag push triggers
   [`ios-release.yml`](../.github/workflows/ios-release.yml):
   builds the XCFramework, patches `Package.swift` with the actual
   build-time SHA256, force-moves the tag onto the patched commit, then
   uploads the zip to the GitHub Release.
5. The matching GitHub Release `published` event triggers
   [`android-release.yml`](../.github/workflows/android-release.yml)
   to publish the AAR to GitHub Packages.
6. **Maven Central release is manual** — run
   [`android-maven-central.yml`](../.github/workflows/android-maven-central.yml)
   via `workflow_dispatch` once the tag exists. See §"Android — Sonatype
   Maven Central" below for the secret/key setup.
7. **CocoaPods Trunk push is manual** — run `ios-release.yml` via
   `workflow_dispatch` with `publish_cocoapods=true`. See §"iOS —
   CocoaPods Trunk".

The chicken-and-egg between the SPM checksum and the build output is
absorbed by step 4: the tag commit's `Package.swift` is updated in-place
to match the freshly built XCFramework, so SPM consumers resolving
`from: "X.Y.Z"` always see a checksum that matches the published zip.

PCLMobile is distributed through several channels:

| Platform | Channel | Manifest | Workflow |
|---|---|---|---|
| iOS | Swift Package Manager | [`Package.swift`](../Package.swift) | [`ios-release.yml`](../.github/workflows/ios-release.yml) |
| iOS | CocoaPods | [`iOSWrapper/PCLMobile.podspec`](../iOSWrapper/PCLMobile.podspec) | `ios-release.yml` (optional Trunk push) |
| Android | Sonatype Maven Central | [`pclmobile/build.gradle.kts`](../AndroidWrapper/aar/pclmobile/build.gradle.kts) | [`android-maven-central.yml`](../.github/workflows/android-maven-central.yml) |
| Android | GitHub Packages (snapshots) | same | [`android-release.yml`](../.github/workflows/android-release.yml) |

This doc covers the user-side setup (account creation, key generation,
secret wiring) needed for each channel beyond what CI already automates.

---

## iOS — CocoaPods Trunk

GitHub Release distribution works without any extra setup — the
`ios-release.yml` workflow attaches `PCLMobile.xcframework.zip` to the
tagged release, and `Package.swift` / `PCLMobile.json` point at that
URL.

Optional: also push the podspec to the CocoaPods Trunk (the public
CocoaPods registry).

### One-time Trunk setup

```bash
# 1. Register the email that will own the pod
pod trunk register you@example.com "Your Name" --description="Laptop"

# 2. Check that the verification email arrived and click the link.
# 3. Confirm registration
pod trunk me
```

The session token lands in `~/.netrc`. You'll need its value for the CI
secret.

### Wire into GitHub Actions

1. `cat ~/.netrc | grep -A1 trunk.cocoapods.org` → copy the `password` value.
2. **GitHub repo → Settings → Secrets and variables → Actions → New secret**:
   ```
   COCOAPODS_TRUNK_TOKEN = <password from ~/.netrc>
   ```
3. Trigger `ios-release.yml` via **workflow_dispatch** with
   `publish_cocoapods: true` to push the podspec. Tag-push triggers
   currently skip Trunk to avoid accidental re-pushes (Trunk rejects
   republished versions).

### Local validation

Validate the podspec without uploading:
```bash
./scripts/lint_podspec.sh
```

---

## Android — Sonatype Maven Central

This is the biggest setup. Plan for **3–5 business days** to clear
Sonatype's account approval and namespace verification before your
first release.

### 1. Sonatype Central account

As of March 2024, Sonatype moved new signups to the **Central Portal**
(`https://central.sonatype.com`). Existing OSSRH accounts (Jira-based
`s01.oss.sonatype.org`) keep working but are being phased out.

1. Sign up at <https://central.sonatype.com> with the email you want
   listed as the publishing user.
2. Generate a **User Token** under **View Account** → it'll show a
   `username` and `password` pair scoped to a specific token name.
   Save both — they go into GitHub secrets later.

### 2. Namespace verification

Maven Central enforces `groupId` ownership. The project ships with
`group = "io.github.sirokujira"` in
[`pclmobile/build.gradle.kts`](../AndroidWrapper/aar/pclmobile/build.gradle.kts).

For the `io.github.<username>` pattern, verification is automated:

1. Open **Central Portal → Namespaces → Add namespace**.
2. Enter `io.github.sirokujira` (case-insensitive).
3. Central generates a **verification key** (random string).
4. Create a public GitHub repo named exactly that verification key, push
   it, then click **Verify** in the Central UI. The repo can be deleted
   after verification succeeds.

For a custom namespace (e.g. `com.example`) you must serve a TXT DNS
record under the matching domain instead — see Sonatype's docs.

### 3. GPG signing key

Maven Central requires that **every artifact ships with a detached
`.asc` GPG signature**. The build already wires this in
[`pclmobile/build.gradle.kts`](../AndroidWrapper/aar/pclmobile/build.gradle.kts)
when `SIGNING_KEY` + `SIGNING_PASSWORD` are set.

```bash
# 1. Generate a key (RSA 4096, no expiry recommended for CI keys)
gpg --full-generate-key

# 2. Pick out its fingerprint
gpg --list-secret-keys --keyid-format=long
# sec   rsa4096/ABCDEF1234567890 2026-05-13 [SC]
#                            ^^^^^^^^^^^^^^^^^^^^ this is the long key ID

# 3. Publish the public key — Maven Central looks it up here
gpg --keyserver hkps://keys.openpgp.org --send-keys ABCDEF1234567890
gpg --keyserver hkps://keyserver.ubuntu.com --send-keys ABCDEF1234567890

# 4. Export the *private* key in ASCII-armoured form for the secret
gpg --armor --export-secret-keys ABCDEF1234567890 > signing-key.asc
```

Don't commit `signing-key.asc` anywhere — it lands in GitHub Secrets only.

### 4. Wire secrets into GitHub Actions

**GitHub repo → Settings → Secrets and variables → Actions → New secret**:

| Secret | Value |
|---|---|
| `OSSRH_USERNAME` | Sonatype user-token name |
| `OSSRH_PASSWORD` | Sonatype user-token password |
| `SIGNING_KEY` | Full content of `signing-key.asc` (paste the entire file, including `-----BEGIN PGP PRIVATE KEY BLOCK-----` headers) |
| `SIGNING_PASSWORD` | GPG key passphrase |
| `SIGNING_KEY_ID` | Last 8 chars of the GPG long key ID (optional but recommended when multiple keys exist) |

### 5. First publish

The workflow uses the [gradle-nexus-publish-plugin] to drive staging,
closing and releasing through one Gradle invocation — no Sonatype UI
clicks needed once the secrets are wired in.

[gradle-nexus-publish-plugin]: https://github.com/gradle-nexus/publish-plugin

1. **Dry-run first** — builds + signs + publishes to a local repo and
   uploads the bundle as a workflow artifact for inspection. Nothing
   touches Sonatype:
   ```
   gh workflow run android-maven-central.yml \
     -f version=0.1.0 \
     -f dry_run=true
   ```
   Inspect the `pclmobile-0.1.0-maven-bundle` artifact: expect `.aar`,
   `.pom`, `-sources.jar`, plus matching `.asc` + `.md5` + `.sha1` next
   to each.

2. **Staging-only upload** — uploads to Sonatype but stops before
   promoting to Maven Central. Useful for the very first publish so
   you can poke around the staging repo in the Sonatype UI:
   ```
   gh workflow run android-maven-central.yml \
     -f version=0.1.0 \
     -f dry_run=false \
     -f auto_release=false
   ```
   Then log in to the matching Sonatype UI and click **Close** →
   **Release** (legacy OSSRH) or **Publish** (Central Portal).

3. **Full automated release** — uploads, closes the staging repo,
   waits for Nexus validation, and promotes to Maven Central in one
   shot. Use this once the dry-run path is validated:
   ```
   gh workflow run android-maven-central.yml \
     -f version=0.1.0 \
     -f dry_run=false \
     -f auto_release=true
   ```

4. Released artifacts appear on Maven Central within ~30 minutes and
   become indexed (searchable in <https://search.maven.org>) within
   ~4 hours.

### Consuming from Maven Central

Once released, downstream Android apps just declare:
```kotlin
dependencies {
    implementation("io.github.sirokujira:pclmobile:0.1.0")
}
```
…no extra repository config needed (Maven Central is in every project's
default repository list).

### Switching to the new Central Portal endpoint

If your account was created after March 2024, you cannot use the legacy
OSSRH URL — you must publish through the Central Portal upload API
instead. The simplest path is to plug in the
[Vanniktech Maven Publish plugin](https://github.com/vanniktech/gradle-maven-publish-plugin),
which adds a `publishToMavenCentral` task that bundles + uploads in one
step. Override `maven_repository_url` to the Central Portal's bundle
upload endpoint to use it via the existing workflow.

---

## Android — GitHub Packages

Already handled by [`android-release.yml`](../.github/workflows/android-release.yml).
No additional secrets needed beyond `GITHUB_TOKEN` (auto-provided).

Triggers automatically on `release: published` events. Consumers add:

```kotlin
repositories {
    maven {
        url = uri("https://maven.pkg.github.com/Sirokujira/pcl_mobile_framework")
        credentials {
            username = providers.gradleProperty("github.user").get()
            password = providers.gradleProperty("github.token").get()
        }
    }
}
```

GitHub Packages requires authentication even for public packages, which
is why Maven Central is the preferred long-term distribution channel.
