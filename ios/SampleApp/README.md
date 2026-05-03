# PCLMobile iOS サンプルアプリ

このディレクトリには 2 種類のサンプルがあります:

| ディレクトリ | 用途 | 動作環境 |
|---|---|---|
| [`PCLMobileBasicSample/`](./PCLMobileBasicSample/) | 合成点群を作って **voxel grid → pass-through → PCD save/load** を全部見せる、最短のデモ | iOS シミュレータで OK(カメラもセンサも不要) |
| [`PCLMobileARSample/`](./PCLMobileARSample/) | ARKit `rawFeaturePoints` / LiDAR mesh を取って PCLMobile に流す実機向けサンプル | iPhone 実機(LiDAR があると密な点群) |

新しく PCLMobile を試したい人は `PCLMobileBasicSample/` から始めるのが速いです。

---

## PCLMobileARSample — iOS 実機テスト

`PCLMobile.xcframework` を ARKit と組み合わせて動かすサンプルです。

* `PCLMobileARSample/` … SwiftUI + ARKit のアプリ本体
* `PCLMobileARSampleTests/` … XCTest 一式
   * `PCLMobileTests.swift` — シミュレータでも実機でも動く API テスト
   * `ARKitIntegrationTests.swift` — 実機専用、`rawFeaturePoints` を取って PCLMobile にかける

---

## 動作要件

| | バージョン |
|---|---|
| Xcode            | 15.0 以上 |
| iOS デプロイ先   | 13.0 以上 |
| ターゲット端末   | iPhone XR 以降推奨。LiDAR (iPhone 12 Pro 以降 / iPad Pro M1 以降) があると劇的に密な点群が取れる |
| `PCLMobile.xcframework` | リポジトリルートの `scripts/build_ios.sh` で生成済みであること |

---

## セットアップ手順

このディレクトリに `.xcodeproj` は同梱していません(差分ノイズになる&Xcode バージョンに依存するため)。下記いずれかでプロジェクトを作ってください。

### 方法 0: XcodeGen で一発生成(最短コース)

```bash
brew install xcodegen      # 一度だけ
cd ios/SampleApp
xcodegen generate          # PCLMobileARSample.xcodeproj が生成される
open PCLMobileARSample.xcodeproj
```

`project.yml` がリポジトリルートの `build/ios/xcframework/PCLMobile.xcframework` を相対参照しているので、事前に `./scripts/build_ios.sh` を完走させて XCFramework を作っておく必要があります。

### 方法 A: 新規 Xcode プロジェクトを起こす(GUI 派の人向け)

1. Xcode → **File → New → Project → iOS → App** で空のプロジェクトを作る:
   * Product Name: `PCLMobileARSample`
   * Interface: **SwiftUI**
   * Language: **Swift**
   * Include Tests: ✅
   * 保存先: `ios/SampleApp/` の中(既存の `PCLMobileARSample/` フォルダを上書きする形)

2. Xcode が生成した `App.swift` / `ContentView.swift` を**削除**し、本リポの `PCLMobileARSample/*.swift` をプロジェクトの **App ターゲット**に追加。

3. テストターゲットに `PCLMobileARSampleTests/*.swift` を追加。

4. **App ターゲット**の `Info.plist` を本リポの `PCLMobileARSample/Info.plist` の内容で置き換え(あるいは Xcode の Info タブで以下を追加):
   * `NSCameraUsageDescription` (string)
   * `UIRequiredDeviceCapabilities` に `arkit` を追加

5. **PCLMobile.xcframework のリンク**:
   * App ターゲット → **General → Frameworks, Libraries, and Embedded Content** → **+** →
     **Add Other → Add Files…** → リポジトリルート `build/ios/xcframework/PCLMobile.xcframework` を選択
   * Embed: `Embed & Sign` を選ぶ
   * Build Settings → **Other Linker Flags** に `-lc++` を追加

6. **テストターゲットも同じ XCFramework をリンク**:
   * Tests ターゲット → General → Frameworks, Libraries, and Embedded Content → **+** → 上記の XCFramework を追加
   * Embed: `Do Not Embed`(テストの場合は明示的に埋め込まない)

7. **Build Settings**:
   * Always Embed Swift Standard Libraries: `Yes` (Apps with binary frameworks need it)
   * Enable Modules (C and Objective-C): `Yes`
   * Build Library for Distribution: `No`(アプリ側は OFF で OK)

8. デバイスを USB 接続 → スキームで実機を選択 → `Cmd+R`。

### 方法 B: SwiftPM 経由(将来用)

リポルートの `Package.swift` は `.binaryTarget(name: "PCLMobile", path: "build/ios/xcframework/PCLMobile.xcframework")` を提供しています。Xcode で **File → Add Package Dependencies → Add Local…** からこのリポを指定し、App ターゲットに `PCLMobile` プロダクトをリンクすれば SwiftPM 経由でも動きます。

---

## 使い方

1. アプリを実機で起動 → カメラ権限を許可。
2. テクスチャのある場所(壁紙、家具、絨毯など)にゆっくりカメラを向ける → 画面上に黄色い feature points が出る。
3. **Capture & process** をタップ → 画面下に「raw / voxel / passthrough」3 段の点数が表示される。
4. **Save PCD** をタップ → アプリの Documents ディレクトリに `pcl-mobile-<timestamp>.pcd` が書き出される(Files.app からも参照可)。

---

## テストを走らせる

### シミュレータで API テストだけ走らせる
```
Xcode → Scheme: PCLMobileARSampleTests
Destination: iPhone 15 (Simulator)
Cmd+U
```

`PCLMobileTests` のみ走り、`ARKitIntegrationTests` は `XCTSkipIf` で全部スキップされます。

### 実機で完全テストを走らせる
```
Destination: <実機>
Cmd+U
```

`ARKitIntegrationTests.testCaptureFeaturePointsAndDownsample` がカメラ起動 → 10 秒以内に 1 回でも feature point が取れれば成功します。テスト中は何かテクスチャのある場所を映してください。

---

## パラメータの調整

`ARPointCloudCoordinator.swift` の冒頭:

```swift
let voxelLeaf: Float = 0.02   // 2cm voxel grid
let zMin: Float = -1.5
let zMax: Float = 1.5
```

LiDAR デバイスでは `voxelLeaf` を 0.05〜0.10 にして 5〜10 cm 解像度にすると現実的です。`zMin`/`zMax` は ARKit のワールド座標(原点 = セッション開始時の端末位置、+y は上、-z は端末の向き先)で「カメラ前方 1.5m 以内」を取り出す指定です。

---

## トラブルシュート

| 症状 | 原因 | 対処 |
|---|---|---|
| `dyld: Library not loaded: @rpath/PCLMobile.framework/PCLMobile` | XCFramework を Embed していない | アプリターゲットの Frameworks で **Embed & Sign** に |
| `Undefined symbols: pcl::PCLBase…` | `-lc++` 不足 | Other Linker Flags に追加 |
| 点群が常に 0 | 暗所 / 平面が無くて ARKit が tracking できていない | テクスチャのある場所で再試行、`debugOptions = .showFeaturePoints` で確認 |
| 実機でアプリが起動しない | Provisioning profile 未設定 | Xcode の Signing & Capabilities タブで Apple ID を選択 |
