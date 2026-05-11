# Bitrise CI/CD setup

This repo ships a [`bitrise.yml`](../bitrise.yml) at the repository root that
builds both the iOS XCFramework + sample app and the Android AAR + sample
APK, then deploys to **TestFlight** (iOS) and **DeployGate** (Android).

The PCL/Boost/Eigen/FLANN/Qhull cross-compile takes 30–60 min cold. The
cache pull/save pair on `build/ios`, `build/android` and
`AndroidWrapper/aar/pclmobile/libs` brings warm builds down to a few
minutes.

---

## 1. Initial Bitrise setup

1. Sign in at <https://bitrise.io> and click **Add new app**.
2. Connect this Git repo (GitHub/GitLab/Bitbucket) and let Bitrise scan.
3. When asked **"How do you want to configure the build?"** choose
   **"Use existing bitrise.yml in the repo"**. Path: `bitrise.yml`.
4. Pick the **macOS / Xcode 15.4+** stack and an **M1/M2** machine type.
   The cross-compile is CPU-bound — Apple Silicon roughly halves wall time
   versus Intel. The yaml file already pins this.
5. Add a webhook so pushes/PRs/tags trigger builds (Bitrise's onboarding
   wizard creates one automatically when connecting via OAuth).

The `trigger_map` at the top of `bitrise.yml` already wires:

| Trigger | Workflow |
|---|---|
| Push to `master` | `ci` (build only) |
| Tag `vX.Y.Z` | `release` (build + deploy iOS + Android) |
| Pull request | `ci` |

You can also kick `ios_only` or `android_only` manually from the Bitrise UI
when you need to deploy just one platform.

---

## 2. iOS — TestFlight credentials

TestFlight upload uses the **App Store Connect API**, not username/password
(2FA-incompatible).

### 2.1 Create an API key

1. Go to <https://appstoreconnect.apple.com/access/api>.
2. Click **+** under **Active Keys**, give it the **App Manager** role.
3. Download the `AuthKey_XXXXXX.p8` file — *this can only be downloaded once.*
4. Note the **Key ID** (10-char string) and **Issuer ID** (UUID at the top
   of the page).

### 2.2 Wire into Bitrise

1. **Bitrise → Workspace settings → Apple service connection**, paste:
   - the `.p8` file
   - the Key ID
   - the Issuer ID
2. Open the app's **Code Signing & Files** tab and confirm the
   `BITRISE_APP_STORE_CONNECT_API_KEY_URL` env appears.
3. Add to **Secrets**:
   ```
   APP_STORE_CONNECT_API_KEY_ID        = <Key ID>
   APP_STORE_CONNECT_API_KEY_ISSUER_ID = <Issuer ID>
   ```

### 2.3 Distribution certificate + provisioning profile

The `xcode-archive` step uses `automatic_code_signing: api-key`, so Bitrise
generates and renews these for you. The app's bundle ID
(`io.github.sirokujira.PCLMobileARSample`) must already exist in
[App Store Connect](https://appstoreconnect.apple.com/) with TestFlight
enabled.

> Want to deploy a different sample app? Override `IOS_SCHEME`,
> `IOS_PROJECT_PATH` and `IOS_APP_DIR` in the workflow's env or in
> Bitrise's **Env Vars** tab. The yaml defaults to `PCLMobileARSample`.

---

## 3. Android — DeployGate credentials

DeployGate uses a personal API token + your username/organisation.

1. Sign in at <https://deploygate.com> and open **Account Settings → API**.
2. Copy the **API Key** (32-hex-char token).
3. **Bitrise → app → Secrets**, add:
   ```
   DEPLOYGATE_API_TOKEN       = <token>
   DEPLOYGATE_USER            = <your username or org slug>
   DEPLOYGATE_DISTRIBUTION_KEY = <optional, distribution-page hash>
   DEPLOYGATE_RELEASE_NOTE    = <optional, defaults to build # + branch>
   ```
   Mark `DEPLOYGATE_API_TOKEN` as **Make it protected** so it never appears
   in build logs.

### 3.1 APK signing

The default workflow ships an **unsigned debug-keystore APK** because
that's all CI needs to validate the build. To sign for DeployGate
distribution:

1. Generate a release keystore locally:
   ```bash
   keytool -genkey -v -keystore release.jks -alias pclmobile \
           -keyalg RSA -keysize 2048 -validity 10000
   ```
2. **Bitrise → app → Code Signing & Files → ANDROID KEYSTORE FILE**
   upload `release.jks` and fill in the alias + passwords.
3. In `bitrise.yml`, change the `sign-apk` step's
   `run_if: '{{enveq "BITRISEIO_ANDROID_KEYSTORE_URL" ""}}'` to
   `run_if: 'true'` (or just delete the `run_if` line so it always runs).

DeployGate happily accepts unsigned APKs for internal builds, but the
Play Console will not — sign anything that's headed for production.

---

## 4. Caching strategy

The `_setup` workflow restores three paths if the cache key matches:

| Path | Why |
|---|---|
| `build/ios` | Cross-compiled PCL/Boost/Eigen/FLANN/Qhull static libs + the per-slice `PCLMobile.framework` outputs |
| `build/android` | The same static libs for each Android ABI |
| `AndroidWrapper/aar/pclmobile/libs` | Staged libraries the Android Gradle module consumes |

The cache key hashes `CMakeLists.txt`, `scripts/build_ios.sh` and
`scripts/build_android.sh`. Bumping a dep version, a script, or a slice
list invalidates the cache automatically.

To save the cache after a green build, add `_post_cache` to `after_run`
in the `ci`/`release` workflows. (It's defined but not wired by default
so first-time setup doesn't blow up the cache budget on a half-built
artefact.)

---

## 5. Local validation

Before pushing, validate the yaml with the Bitrise CLI:

```bash
brew install bitrise          # one-time
bitrise validate              # parses bitrise.yml
bitrise run ci                # runs the ci workflow locally on macOS
```

`bitrise run` honours every step the same way the cloud agent does, but
omits anything that needs Bitrise-issued signing assets (e.g. the
TestFlight upload). Use it to catch yaml typos and broken script paths
without burning CI minutes.

---

## 6. Cost / minute budget

A cold build on a Bitrise `g2-m1.8core` macOS machine runs roughly:

| Phase | Time |
|---|---|
| iOS superbuild (3 slices) | 25–35 min |
| iOS XCFramework | 3–5 min |
| iOS sample archive | 1–2 min |
| Android native libs (1 ABI) | 8–12 min |
| Android sample APK | 1–2 min |
| **Cold total** | **~45–55 min** |
| **Warm total (cache hit)** | **~5–8 min** |

Tag releases sparingly until you're sure the green-path is stable, and
keep `IOS_SLICES` set to `OS64 SIMULATORARM64` (skip the Intel sim slice)
unless you need x86_64 simulator support — that single slice adds ~10 min.
