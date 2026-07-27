# pcl_mobile_framework 改修計画書

最終更新: 2026-04-26
対象: `pcl_mobile_framework`(PCL を iOS / Android 向けにクロスコンパイルし、ライブラリとして配布するプロジェクト)
ゴール: 現代的なツールチェーン・依存に揃え、ビルドスクリプトを整理統合し、ラッパー(JNI / Objective-C++)を整え、配布パッケージ(AAR / XCFramework / SwiftPM / Maven)に対応した形にする。

---

## 0. 現状サマリ

| 項目 | 現状 | 望ましい現状(2026 時点) |
|---|---|---|
| PCL | 1.9 | 1.14.x(stable) |
| Boost | 1.60.0(custom patch) | 1.84+(stable, PCL 1.14 が要求) |
| Eigen | 3.3.4 | 3.4.0 |
| FLANN | 1.9.1 | 1.9.2 |
| Qhull | 2015.2 | 2020.2(または `qhull_r` モジュール内蔵化) |
| CMake | 3.4.1〜3.12.4 が混在 | 3.24+(`XCFRAMEWORK` 等のため) |
| Android NDK | r16b(GCC 4.9 + Clang 3.6) | r26d / r27(Clang 17+, GCC 廃止) |
| Android Gradle | AGP 3.4.1 / Gradle 5.1.1 / Kotlin 1.3.11 | AGP 8.5 / Gradle 8.7 / Kotlin 1.9.x |
| Android SDK | compile/target 28, min 23 | compile/target 34, min 21〜24 |
| Xcode / iOS | Xcode 9.3, iOS 6.0/8.0 deploy target, armv7/armv7s/i386/arm64e サポート | Xcode 15+, iOS 13+ deploy target, arm64 + simulator(arm64/x86_64) |
| バイナリ形式 | `lipo` で fat framework | XCFramework |
| 配布 | 言及のみ(Pod/Carthage/SPM) | Maven(AAR), Swift Package(binaryTarget), CocoaPods(podspec) を実体化 |
| CI | AppVeyor(VS2015), Travis(終了済), CircleCI(api-26) | GitHub Actions(macos-14, ubuntu-22.04)に統一 |

---

## 1. 致命的な問題(これが直らないと現状ビルドが通らない)

### 1.1 `external-project-macros.cmake` が repo に存在しない
`CMakeLists.txt:43` で `include(external-project-macros.cmake)` しているが、`fetch_pcl()` / `crosscompile_boost()` 等を定義するこのファイル自体が repo に含まれていない。2019 年に前身 `Sirokujira/pcl-superbuild` からファイルをコピーして本 repo を起こした際(`98c6c50`)、前身の `83cc231` にあった 72 ファイルのうち 44 ファイルが取り込まれておらず、本ファイルもその 1 つ(前身側の `83cc231` には存在する)。他の欠落分には `iOSWrapper/` の実装一式 38 ファイル(→ 1.2)と CI 設定 3 種(`.travis.yml` / `.circleci/config.yml` / `.azure-pipelines.yml`)が含まれる。詳細は [docs/LINEAGE.md](./docs/LINEAGE.md) を参照。

- 影響: `cmake` を打った瞬間に `Could not find include file "external-project-macros.cmake"` で停止する。**現状、本リポジトリは誰の環境でもビルドできない。**
- 対応: 前身 `Sirokujira/pcl-superbuild@83cc231` の `external-project-macros.cmake`(およびその源流である `hirotakaster/pcl-superbuild`, `patmarion/pcl-superbuild`)を元に、依存ごとのファイルへ分割して再実装する。マクロ名(`install_eigen` / `crosscompile_boost` / `crosscompile_flann` / `crosscompile_qhull` / `crosscompile_pcl`)は前身から引き継ぐ。系譜の詳細は [docs/LINEAGE.md](./docs/LINEAGE.md) を参照。

### 1.2 `iOSWrapper/` が実質空
- 中身は `README.md`(別プロジェクトからのコピペ)と `module.modulemap` のみ。
- `build_ios_device_framework.sh` / `build_ios_simulator_framework.sh` が参照する `iOSWrapper/build.ios/`、`iOSWrapper/build.sim64/` を生成する Xcode プロジェクトや CMakeLists が存在しない。
- 対応: iOSWrapper 配下に Xcode プロジェクト(または CMake で Xcode generator 用)、Public Header(`PCLMobile.h`)、Objective-C++ ファサード(`PCLMobile.mm`)を新設する。

### 1.3 ハードコードされた絶対パス
`AndroidWrapper/aar/pclmobile/CMakeLists.txt:13`
```
set (BASE_ROOT "/absolute/path/to/pcl_mobile_framework/AndroidWrapper/aar/pclmobile/libs")
```
他環境では動かない。`${CMAKE_CURRENT_SOURCE_DIR}/libs` に直す。

### 1.4 `build_android.sh` のシェル構文崩壊
26-27 行目で `cmake` のあと改行していて、`-DC_COMPILER_TOOL=...` が CMake 引数ではなくシェルコマンドとして実行されようとしている。
```
cmake -G"Unix Makefiles" -H. -Bbuild ... \      # ← '\' が無いので
-DC_COMPILER_TOOL=$ANDROID_NDK/...               # ← 別コマンド扱い
```

### 1.5 `AndroidWrapper/InstallPointCloudLibrary.sh` がそもそも Bash として無効
PowerShell 風 `$ZIP_FILE = '...'`、`ZIP_FILE = ='...'`、`$` 付きの代入など多数の構文エラー。さらに参照している AppVeyor の `buildjobs/<id>/artifacts` URL は AppVeyor 側のジョブ消滅で失効済みの公算が高い。

### 1.6 JNI シンボル不整合(リンカ/実行時 NG)

| Java 側 | JNI 宣言 | C++ 実装 | 問題 |
|---|---|---|---|
| `tracking1(filename)` | `Java_..._track1` | `Java_..._tracking1` | 宣言名と Java 名が不一致 → `UnsatisfiedLinkError` |
| `feature1(filename)` | `Java_..._feature1(env, this)`(引数なし宣言) | `(env, obj, jstring filename)`(実装は引数あり) | シグネチャ不一致 |
| `rangeimages1(filename)` | `Java_..._rangeImages1` | `Java_..._rangeimages1` | 大文字小文字の typo |

(`pclmobileJNILib.java` ⇔ `native-lib.cpp` の対比結果)

### 1.7 `makeFramework*.sh` の未定義変数 typo
`makeFramework.sh` / `makeFramework2.sh` は arm64/armv7/armv7s ごとに `boost_device_arm64_libs` と命名している箇所と、libtool 行で `boost_arm64_device_libs` と引いている箇所が混在。シェルでは未定義変数は空展開されるので、**実際にはほとんどのライブラリがリンクされていなかった**可能性が高い。

---

## 2. 古さに起因する問題

### 2.1 Android Gradle / NDK
- `compileSdkVersion 28`, `targetSdkVersion 28`, `buildToolsVersion 28.0.3`, `support-v7:28.0.0`(AndroidX 未移行)
- `AndroidJUnitRunner` が `android.support.test.runner.*` 系(AndroidX 前)
- `jcenter()` リポジトリ(2022 年 read-only、2025 年廃止予定)
- `abiFilters 'x86', 'armeabi-v7a'`(arm64-v8a が抜けている。**現代 Android で必須**)
- `kotlin-android` plugin を読み込んでいるのに Kotlin ファイルが存在しない
- `gradle-5.1.1` ⇔ AGP 3.4.1 は AGP 8.x 系・Java 17 とは互換しない。

### 2.2 iOS toolchain ファイルの過剰
`toolchains/` 配下に 18 ファイルあり、その大半は `iOS_Device.cmake` を ARM64/ARM64e/ARMv7/ARMv7s ごとに丸ごとコピペしたもの。差分は `IOS_ARCH` と `BUILD_ARM64*` 程度。さらに別系統の以下が並列:
- `Toolchain-iPhoneOS_Xcode.cmake`(`common-ios-toolchain.cmake` を include)
- `iOS.toolchain.cmake`(cristeab/ios-cmake 系、より新しい)
- `iOS.cmake`, `iOS_xcode.cmake`(独自)
- `iOS_Simulation.toolchain.cmake`(cristeab fork)

`include(CMakeForceCompiler)` は CMake 3.0+ で deprecated、3.27 で削除予定。
`CMAKE_OSX_DEPLOYMENT_TARGET "9.3"`(iOS 9.3 → 現代では iOS 13 以上が標準)。
`armv7`/`armv7s`/`i386` のターゲットは iOS 11(2017)以降の実機では走らない。

### 2.3 ビルドスクリプト過剰
- `build_ios_device_framework.sh` / `build_ios_simulator_framework.sh` / `build_ios_universal_binary.sh` / `build_ios_universal_framework.sh` の 4 つは中身が大半重複。
- `makeFramework.sh`(354 行) と `makeFramework2.sh`(409 行) はほぼ同じ内容で別バージョン。`make_framework_universal2`(symlink ベースの旧 macOS framework 形式)は iOS では使えないのにコメント付きで残存。
- `xamarinObjevtiveSharpie.sh` は使われない Xamarin 連携の名残。
- `build_android.bat` は r16b ハードコード。

### 2.4 ライブラリのパッケージング/配布が未実装
- README は CocoaPods / Carthage / SPM へのインストール例を載せているが、実体となる `*.podspec`, `Package.swift`, Carthage 用の prebuilt zip 生成は無い。
- AAR の Maven 公開設定(`maven-publish` plugin)もない。
- `pclmobile/build.gradle` は `apply plugin: 'com.android.library'` までしかなく、`publishing { ... }` ブロックが無い。

### 2.5 ドキュメントが現状と合っていない
- `iOSWrapper/README.md` は KD-Tree ライブラリのテンプレからのコピーで、PCL とは無関係な API 例(`tree.nearest(to:)` など)を載せている。
- ルート `README.md` は NDK r15c/r16b、Xcode 8.3/9.3、Ubuntu 14.04/16.04 を前提としている。
- Travis / AppVeyor / CircleCI のバッジは現存しない/到達不能なリポジトリを指している(`Sirokujira/template` 等)。

---

## 3. 改修の方針(全体像)

```
.
├─ cmake/                         # ★ 新設: 共有 .cmake モジュール置き場
│   ├─ external/                  #   - external-project-macros.cmake を分割再実装
│   │   ├─ pcl.cmake
│   │   ├─ boost.cmake
│   │   ├─ eigen.cmake
│   │   ├─ flann.cmake
│   │   └─ qhull.cmake
│   └─ toolchains/                #   - 旧 toolchains/ をここに集約
│       ├─ ios.toolchain.cmake    #     ★ leetal/ios-cmake をベースに 1 本に統合
│       └─ android.toolchain.cmake#     ★ NDK 同梱版を薄くラップ
├─ scripts/                       # ★ 新設: 旧 build_*.sh の後継
│   ├─ build_android.sh           #   - 全 ABI を 1 スクリプトで(配列で iterate)
│   ├─ build_ios.sh               #   - device + simulator を 1 本で
│   └─ make_xcframework.sh        #   - 旧 makeFramework*.sh の代替(XCFramework 生成)
├─ android/                       # ★ AndroidWrapper/aar/ をリネーム & 整理
│   ├─ build.gradle.kts           #   - Kotlin DSL に移行
│   ├─ settings.gradle.kts
│   ├─ pclmobile/                 #   - ライブラリ本体(AAR)
│   └─ sample-app/                #   - サンプルアプリ(配布物には含めない)
├─ ios/                           # ★ iOSWrapper/ をリネーム & 実装
│   ├─ PCLMobile.xcodeproj/
│   ├─ Sources/
│   │   ├─ include/PCLMobile/
│   │   │   ├─ PCLMobile.h        #   - 公開 Objective-C ヘッダ(C++ を露出させない)
│   │   │   └─ PCLMPointCloud.h
│   │   └─ PCLMobile.mm           #   - Objective-C++ ファサード
│   ├─ module.modulemap
│   └─ PCLMobile.podspec
├─ Package.swift                  # ★ SwiftPM(binaryTarget で XCFramework を配布)
├─ .github/workflows/             # ★ CI を GitHub Actions に統一
│   ├─ android.yml
│   └─ ios.yml
├─ CMakeLists.txt                 # 大幅整理(下記)
├─ cmake/SetupSuperbuild.cmake    # 旧 setup-superbuild.cmake を移設・改名
├─ cmake/ProjectVariables.cmake   # 旧 setup-project-variables.cmake を移設・改名
└─ README.md                      # 全面書き直し
```

---

## 4. 作業を 4 フェーズに分割

進行は依存順に並んでおり、各フェーズが完了するごとに「ビルドが通る」状態を保てる粒度に切ってある。

### フェーズ A: 整理と土台(破壊変更なし、現状の挙動を変えない最小整備)
A-1. レポジトリのインポート前検証
- `git ls-files | sort` で**実際に存在するファイルだけ**を一覧化し、`CMakeLists.txt` 内の `include` / `add_subdirectory` を grep して、欠落参照を洗い出す。
- 洗い出した結果(現状: `external-project-macros.cmake`)をすべてリスト化。

A-2. ハードコードパスを撲滅
- `AndroidWrapper/aar/pclmobile/CMakeLists.txt:13` のハードコードされた絶対パスを `${CMAKE_CURRENT_SOURCE_DIR}/libs` に変更。
- `MainActivity.java` の `/storage/emulated/0/lamppost.pcd` を `assets/` または `intent` 経由に変更。

A-3. シェルスクリプトの単純バグ修正
- `build_android.sh:25-27` を 1 行 or 行末 `\` に。
- `InstallPointCloudLibrary.sh` を一旦 `.deprecated` に退避(ダウンロード URL が死んでいるため)。
- `xamarinObjevtiveSharpie.sh` を `.deprecated` に退避。

A-4. JNI シンボル名の修復(API は変えない)
- `pclmobileJNILib.java` の `tracking1` 宣言と `native-lib.cpp` の `_track1` 実装を一致させる(実装側を `_tracking1` に揃える)。
- `feature1` の引数を Java/JNI/C++ で `(jstring filename)` に揃える。
- `rangeImages1`/`rangeimages1` の大文字小文字を Java 側基準で揃える。

A-5. `makeFramework*.sh` の未定義変数 typo を修正
- `${boost_arm64_device_libs}` 等の **逆順** 命名箇所を、定義側 `boost_device_arm64_libs` に合わせる。

> **ここまででビルド可能な状態にはまだ戻らない**(1.1, 1.2 のため)。が、後続フェーズの土台が揃う。

### フェーズ B: ビルドシステムの再構築(ここで初めて再びビルド可能になる)

B-1. `external-project-macros.cmake` の再実装
`cmake/external/` 配下に **依存ごとに分離した `.cmake`** を新設。例:
```cmake
# cmake/external/eigen.cmake
function(install_eigen)
  ExternalProject_Add(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH SHA256=...
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${install_prefix}/eigen
    BUILD_COMMAND ""
  )
endfunction()

function(crosscompile_boost tag)
  ExternalProject_Add(boost-${tag}
    DEPENDS ...
    URL https://archives.boost.io/release/1.84.0/source/boost_1_84_0.tar.bz2
    CONFIGURE_COMMAND <SOURCE_DIR>/bootstrap.sh
    BUILD_COMMAND <SOURCE_DIR>/b2 ${BOOST_B2_ARGS_${tag}}
    INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory <SOURCE_DIR>/boost ${install_prefix}/boost-${tag}/include/boost
  )
endfunction()
```
ここに `crosscompile_pcl`, `crosscompile_flann`, `crosscompile_qhull` も同様に書き起こす。

B-2. ルート `CMakeLists.txt` のスリム化
```cmake
cmake_minimum_required(VERSION 3.24 FATAL_ERROR)
project(pcl_mobile_superbuild NONE)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(SetupSuperbuild)
include(ProjectVariables)
include(external/eigen)
include(external/boost)
include(external/flann)
include(external/qhull)
include(external/pcl)

install_eigen()
foreach(tag IN LISTS PCL_MOBILE_TARGETS)
  crosscompile_boost(${tag})
  crosscompile_flann(${tag})
  crosscompile_qhull(${tag})
  crosscompile_pcl(${tag})
endforeach()
```
- `BUILD_IOS_DEVICE_ARMV7`, `_ARMV7S`, `_ARM64E`, `BUILD_IOS_SIMULATOR_I386` のオプションは削除。
- 残すターゲット: `android-arm64`, `android-armv7`, `android-x86_64`, `ios-arm64`, `iossim-arm64`, `iossim-x86_64`。
- iOS deployment target は `13.0` を既定に(PCL 1.14 が `c++17` を要求するため)。

B-3. iOS toolchain の集約
- `toolchains/` から `Toolchain-iPhoneOS_Xcode.cmake`, `Toolchain-iPhoneSimulator_Xcode*.cmake`, `iOS.cmake`, `iOS_xcode.cmake`, `iOS_Device_*.cmake`(全 4)を削除。
- `iOS_Simulator_*.cmake` も削除。
- `cmake/toolchains/ios.toolchain.cmake` 1 本に統合(参考: `leetal/ios-cmake` v4)。`PLATFORM` 変数で `OS64` / `SIMULATOR64` / `SIMULATORARM64` を切り替える。
- 旧 `iOS.toolchain.cmake`(cristeab) は履歴用に残しつつ deprecated 表示にし、最終的に削除。
- `pcl-try-run-results.cmake` / `vtk-try-run-results.cmake` は cross-compile 時の `try_run()` 結果キャッシュ。残すが内容を Apple Silicon Mac で再生成する。

B-4. ビルドスクリプトの統合
- `build_android.sh` / `build_android.bat` を `scripts/build_android.sh` 1 本に。
  ```bash
  for ABI in arm64-v8a armeabi-v7a x86_64; do
    cmake -B build/$ABI -G Ninja \
      -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake \
      -DANDROID_ABI=$ABI -DANDROID_PLATFORM=android-24 \
      -DBUILD_ANDROID=ON
    cmake --build build/$ABI --parallel
  done
  ```
- `build_ios_*_framework.sh` 4 本 + `build_ios_universal_binary.sh` を `scripts/build_ios.sh` 1 本に。中で `iphoneos`/`iphonesimulator` の SDK ごとに 2 ビルド回す。
- `makeFramework.sh` / `makeFramework2.sh` を `scripts/make_xcframework.sh` 1 本に。`xcodebuild -create-xcframework -library ...` を呼ぶ。`lipo` で fat binary を作るのは廃止する(Apple Silicon Mac 上で sim arm64 と device arm64 が衝突するため)。

B-5. AndroidWrapper(AAR)モダナイゼーション
- `AndroidWrapper/aar/` → `android/`
- `build.gradle` → `build.gradle.kts`(Kotlin DSL)
- AGP 8.5、Gradle 8.7、Kotlin 1.9、JDK 17。
- `compileSdk = 34`、`minSdk = 24`(=Android 7.0、PCL 1.14 の C++17 STL に必要な libc++ shared を確保)、`targetSdk = 34`。
- `support-v7` 削除、AndroidX に置換(`androidx.appcompat:appcompat:1.7.x` 等)。
- `jcenter()` 削除、`mavenCentral()` + `google()` のみ。
- `abiFilters = ['arm64-v8a', 'armeabi-v7a', 'x86_64']`(`x86` 削除、`arm64-v8a` 追加)。
- ライブラリの `import { x86, armeabi-v7a }` は `arm64-v8a, armeabi-v7a, x86_64` に。
- `pclmobile/CMakeLists.txt` の **14 個の add_library(lib_pcl1..14 STATIC IMPORTED)** ループ展開を、`foreach(comp ...)` でまとめる。
- ライブラリ名のネーミング統一: `com.sirokujira.pclmobile` → `io.github.<owner>.pclmobile`(group)。
- `kotlin-android` を本当に使うのか/使わないのかを決定(JNI クラスのみなら不要)。
- 公開向けに `pclmobile/build.gradle.kts` に `maven-publish` プラグインと `publishing { ... }` ブロックを追加。

B-6. iOSWrapper の実装
- `iOSWrapper/` → `ios/PCLMobile/`
- `Sources/include/PCLMobile/PCLMobile.h`(Objective-C 公開ヘッダ。**C++ ヘッダは露出させない**)
  ```objc
  #import <Foundation/Foundation.h>

  NS_ASSUME_NONNULL_BEGIN
  @interface PCLMPointCloud : NSObject
  + (instancetype)cloudFromPCDFile:(NSString *)path error:(NSError **)error;
  - (NSUInteger)pointCount;
  - (PCLMPointCloud *)voxelGridDownsampleWithLeaf:(double)leaf;
  @end
  NS_ASSUME_NONNULL_END
  ```
- `Sources/PCLMobile.mm`(C++ を内包する Objective-C++ 実装)
- `module.modulemap` を `framework module PCLMobile { umbrella header "PCLMobile.h"; export *; module * { export * } }` に修正。
- iOS deployment target を 13.0 に、Bitcode は `ENABLE_BITCODE = NO`(Xcode 14+ 既定)。
- 結果として **静的 XCFramework** `PCLMobile.xcframework` を `scripts/make_xcframework.sh` で生成する。

### フェーズ C: 配布(パッケージ化)

C-1. CocoaPods
- `ios/PCLMobile.podspec`
  ```
  Pod::Spec.new do |s|
    s.name         = "PCLMobile"
    s.version      = "0.1.0"
    s.platforms    = { :ios => "13.0" }
    s.ios.vendored_frameworks = "build/PCLMobile.xcframework"
    s.libraries    = "c++"
    ...
  end
  ```

C-2. Swift Package Manager
- リポジトリ直下に `Package.swift` を新設し、`binaryTarget(name: "PCLMobile", path: "build/PCLMobile.xcframework")` で参照。
- リリース時は GitHub Release の zip を `binaryTarget(url:checksum:)` で公開する形に切り替えていく。

C-3. Carthage
- `binary` JSON(`PCLMobile.json`)で XCFramework を指す。

C-4. Maven 公開(AAR)
- `android/pclmobile/build.gradle.kts` に `maven-publish` を入れる。
- 配布先候補: GitHub Packages or Maven Central(`io.github.<owner>:pclmobile:<ver>`)。
- `gradle :pclmobile:publish` で AAR + sources + javadoc をアップロード。

C-5. Pre-built バイナリ(任意)
- GitHub Release に
  - `pcl-mobile-android-<ABI>-<ver>.zip`(各 ABI ごと)
  - `PCLMobile.xcframework.zip`(SHA256 を `Package.swift` に書く)
を置く。

### フェーズ D: CI / ドキュメント / 仕上げ

D-1. CI を GitHub Actions に集約
- `.github/workflows/android.yml`
  - `runs-on: ubuntu-22.04`
  - `actions/setup-java@v4`(JDK 17)
  - `android-actions/setup-android@v3`(NDK r26d、SDK 34)
  - matrix: `[arm64-v8a, armeabi-v7a, x86_64]`
- `.github/workflows/ios.yml`
  - `runs-on: macos-14`(Apple Silicon)
  - matrix: `[ios-arm64, iossim-arm64, iossim-x86_64]`
  - 最後に `make_xcframework.sh` で 1 つにまとめる
- `.appveyor.yml` / `AndroidWrapper/.circleci/config.yml` / `AndroidWrapper/bitrise_android.yml` を `.deprecated/` に移動(後で削除)。
- README からも Travis/AppVeyor/CircleCI のバッジを削除し、新しい GitHub Actions のバッジに差し替える。

D-2. README 全面書き直し
- セクション順: `What is this?` / `Supported platforms` / `Quick start (CocoaPods/SPM/Maven)` / `Building from source` / `Architecture` / `License`。
- Travis/AppVeyor/Azure(死んでる)バッジを削除。GHA バッジに差し替え。
- iOSWrapper/README.md は完全に書き直す(別プロジェクトのコピペ部分を削除)。

D-3. ライセンス整理
- ルート `LICENSE` ファイルが現状無いので追加(Apache-2.0 推奨。PCL は BSD で互換)。
- `strip-frameworks.sh` の Realm Inc 由来 Apache-2.0 ヘッダは保存(派生時の表示義務)。

D-4. テスト
- Android 側: `pclmobileJNILib.load()` のスモークテスト(`assets/lamppost.pcd`)。
- iOS 側: XCTest で `[PCLMPointCloud cloudFromPCDFile:...]` のスモークテスト。
- CI で各 ABI/SDK のビルドが通ることを確認。

---

## 5. 既存ファイルの処遇一覧

| ファイル / ディレクトリ | 処遇 | 理由 |
|---|---|---|
| `CMakeLists.txt` | 書き直し | フェーズ B-2 |
| `setup-superbuild.cmake` | `cmake/SetupSuperbuild.cmake` に移動 | 命名統一 |
| `setup-project-variables.cmake` | `cmake/ProjectVariables.cmake` に移動 + 大幅整理 | 古いオプション削除 |
| `external-project-macros.cmake` | **新規作成**(分割して `cmake/external/`) | 1.1 |
| `build_android.sh` / `.bat` | `scripts/build_android.sh` に統合 | B-4 |
| `build_ios_device_framework.sh` | `scripts/build_ios.sh` に統合・削除 | B-4 |
| `build_ios_simulator_framework.sh` | 同上 | B-4 |
| `build_ios_universal_binary.sh` | 削除 | fat binary 廃止 |
| `build_ios_universal_framework.sh` | 削除 | XCFramework 化 |
| `makeFramework.sh` | 削除 | `scripts/make_xcframework.sh` に置換 |
| `makeFramework2.sh` | 削除 | 同上 |
| `strip-frameworks.sh` | 残置 | App Store 用、現代でも有効 |
| `xamarinObjevtiveSharpie.sh` | `.deprecated/` 移動 | Xamarin 自体が EOL |
| `toolchains/iOS_Device_*.cmake`(4 本) | 削除 | `cmake/toolchains/ios.toolchain.cmake` に統合 |
| `toolchains/iOS_Simulator_*.cmake`(3 本) | 削除 | 同上 |
| `toolchains/Toolchain-iPhone*.cmake`(3 本) | 削除 | 同上 |
| `toolchains/iOS.cmake` / `iOS_xcode.cmake` | 削除 | 同上 |
| `toolchains/iOS.toolchain.cmake` | 削除 | 同上 |
| `toolchains/common-ios-toolchain.cmake` | 削除 | 同上 |
| `toolchains/iOS_Simulation.toolchain.cmake` | 削除 | 同上 |
| `toolchains/pcl-try-run-results.cmake` | `cmake/toolchains/` に移動 | cross-compile 時に必要 |
| `toolchains/vtk-try-run-results.cmake` | 削除 | VTC は使わない方針(`fetch_vtk()` がコメントアウト) |
| `appveyor.yml` | `.deprecated/` 移動 | GHA に移行 |
| `appveyor/`(PS-Zip 等) | 削除 | 同上 |
| `AndroidWrapper/aar/` | `android/` にリネーム | フェーズ B-5 |
| `AndroidWrapper/CMakeLists.txt` | 削除 | aar/pclmobile 側に統一 |
| `AndroidWrapper/InstallPointCloudLibrary.sh` / `.bat` | `.deprecated/` 移動 | 構文崩壊 + URL 失効 |
| `AndroidWrapper/.circleci/config.yml` | `.deprecated/` 移動 | GHA に移行 |
| `AndroidWrapper/bitrise_android.yml` | `.deprecated/` 移動 | 同上 |
| `iOSWrapper/README.md` | 全面書き直し | KD-Tree のコピペを削除 |
| `iOSWrapper/module.modulemap` | 書き直し | umbrella header 化 |

---

## 6. 互換性 / マイグレーション注意点

- **iOS deployment target を 13.0 へ引き上げる**ため、iOS 9.0–12.x をサポートしていた既存ユーザは取り残される。アナウンスを README に明記。
- **`armv7` / `armv7s` / `i386` / `arm64e` / `x86`(Android) を切る**ため、互換ビルド設定をリリースノートに残す(必要なら最後の対応バージョンを `legacy/` ブランチで保存)。
- **lipo fat framework から XCFramework へ**変えるため、CocoaPods で `s.vendored_frameworks` の参照先が `pcl_mobile.framework` → `PCLMobile.xcframework` に変わる。Major version を 0.x → 1.0 に上げて非互換変更を明示する。
- **JNI のシンボル名を直す**ため、フェーズ A-4 のついでに Java パッケージ名も `com.sirokujira.pclmobile` から再考(古い Owner 名が残っているため)。
- AGP 3.4.1 → 8.5 へジャンプアップするので、Android Studio Hedgehog 以降が必須。

---

## 7. リスクと前提

- PCL 1.14 は内部で C++17 を要求する。iOS で C++ 静的ライブラリを `libc++` にリンクする際、`libc++_shared.so`(Android)と `libc++.dylib`(iOS、システム)のリンクモードを誤ると未定義シンボルが発生する。各 `crosscompile_*()` で **必ず** `-stdlib=libc++` を一致させる。
- Boost の `b2` クロスビルドは NDK r26 以降の Clang に追従していない時期があるため、Boost 1.85+(2024-04 リリース) を選ぶか、`boost-build` のパッチを最新 SHA に当て直す必要がある。
- Apple は静的ライブラリの XCFramework での配布を許容するが、`-fembed-bitcode` は Xcode 14 以降廃止。CocoaPods の `s.pod_target_xcconfig` で `BITCODE_GENERATION_MODE=marker` 等の旧設定が残っていないかも確認。
- フェーズ B-1(`external-project-macros.cmake` 再実装)が**最大の不確実性**。参考実装が古く、PCL 1.14 ではモジュール構造(VTK 切り離しや `qhull_r` への移行)が違うため、各 `ExternalProject_Add` の `CMAKE_ARGS` を逐一検証する必要がある。**1〜2 週間規模の実装**を見込む。

---

## 8. 工数見積もり(目安)

| フェーズ | 作業内容 | 規模 |
|---|---|---|
| A | 修復(現状のまま小修正) | 0.5 〜 1 日 |
| B-1 | `external-project-macros.cmake` 再実装 | 5 〜 10 日 |
| B-2 | ルート CMakeLists / Variables 整理 | 1 日 |
| B-3 | iOS toolchain 統合 | 1 〜 2 日 |
| B-4 | ビルドスクリプト統合 | 1 〜 2 日 |
| B-5 | Android Gradle / AGP 8 / AndroidX 移行 | 3 〜 5 日 |
| B-6 | iOSWrapper 実装(ファサード API + XCFramework) | 5 〜 10 日 |
| C | 配布(podspec / Package.swift / maven-publish) | 2 〜 3 日 |
| D | CI(GHA)・README・テスト | 2 〜 3 日 |
| **合計** |  | **概ね 4 〜 6 週間** |

---

## 9. 次にやること(承認をいただいた後の最初の 3 ステップ)

1. **フェーズ A-1 と A-2** をまとめてコミット。これは破壊的変更がないので独立にレビューできる。
2. **フェーズ A-3 〜 A-5**(JNI シンボル / makeFramework typo / シェル構文)を別コミット。テストの土台ができる。
3. **フェーズ B-1**: `external-project-macros.cmake` の再実装に着手。最初は **eigen と boost のみ** でビルドが通る形にし、続いて flann → qhull → pcl の順で増やす。

---

この計画書で進めて良いか、優先したいフェーズの並べ替え、サポートを切るアーキテクチャ(armv7 など)の継続要否、配布チャネルの取捨選択(CocoaPods / SPM / Maven のどれを必須にするか)、をご確認ください。
