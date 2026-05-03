# PCLMobileBasicSample — シミュレータでも動く基本サンプル

ARKit やカメラに依存しない、`PCLMobile` の API 一通りを動作確認する SwiftUI ベースのサンプルです。

* 並列で存在する [`PCLMobileARSample`](../PCLMobileARSample/) は ARKit + 実機が必須。
* 本サンプルは `iPhone` シミュレータでそのまま走り、合成点群を生成 → voxel grid → passthrough → PCD save / load を端末画面で観察できます。

```
PCLMobileBasicSample/
├── PCLMobileBasicSampleApp.swift   # @main エントリ
├── ContentView.swift                # SwiftUI: パラメータ + 結果表示
├── PointCloudLab.swift              # PCLMobile を呼ぶ ObservableObject
├── Info.plist                       # ファイル共有・サポート向き
├── project.yml                      # XcodeGen 用
└── Podfile                          # CocoaPods から取りたい場合用
```

## ビルド方法 (3 通り)

### 1. XcodeGen + ローカル XCFramework(おすすめ・自己完結)

```bash
brew install xcodegen        # まだなら
cd ios/SampleApp/PCLMobileBasicSample

# 事前にリポルートで ./scripts/build_ios.sh が完了していて
#   build/ios/xcframework/PCLMobile.xcframework が存在することが前提。

xcodegen generate
open PCLMobileBasicSample.xcodeproj
# Cmd+R でシミュレータ起動
```

`project.yml` がリポルートの `build/ios/xcframework/PCLMobile.xcframework` を相対参照しています。リビルドした場合は `xcodegen generate` をもう一度走らせる必要はありませんが、Xcode のキャッシュは Cmd+Shift+K で消してください。

### 2. CocoaPods (Trunk からダウンロード)

```bash
cd ios/SampleApp/PCLMobileBasicSample
pod install --project-directory=.
open PCLMobileBasicSample.xcworkspace
```

公開済みの `PCLMobile` pod を取得します。`Podfile` の `:path => '../../../iOSWrapper'` 行をコメント解除すれば**ローカル**の podspec を使います(Trunk 公開前のテストに便利)。

### 3. 手動 (どうしてもツールに頼りたくない場合)

1. Xcode で **iOS App** プロジェクトを新規作成
2. 本ディレクトリの `*.swift` と `Info.plist` をプロジェクトに追加
3. `build/ios/xcframework/PCLMobile.xcframework` を **Embed & Sign** でリンク
4. **Build Settings** → Other Linker Flags: `-lc++`、CLANG_MODULES_AUTOLINK = NO

## 使い方

起動すると 1 画面のフォームが出ます:

| セクション | 何をする |
|---|---|
| **Source** | 合成する点群のサイズ。1k 〜 500k で調整 |
| **Voxel grid** | leaf size を 1cm 〜 20cm スライダで設定 |
| **Pass-through filter** | x / y / z のうちどの軸でクロップするか・min / max |
| **Run pipeline** | タップ → 合成 → voxel → passthrough → PCD save → reload まで一気に走る |
| **Results** | 各段階の点数と、保存された PCD の場所 |

合成点群は「z ≈ 0.5 のノイジな平面 + 単位立方体内のランダム点」を半々で混ぜています。`z[0.25..0.75]` のパススルーで平面側だけ残せるよう設計してあるので、軸とレンジを切り替えると効果が観察できます。

保存された PCD は `Files.app → On My iPhone → PCLMobileBasicSample` から取り出せます。

## テスト

XCTest が `PCLMobileBasicSampleTests/` にあります。XcodeGen 経由でプロジェクトを作っていれば、Xcode の **Cmd+U** で:

* `testPipelineProducesShrinkingCounts` — voxel → passthrough → save → reload の往復で、
   サイズが減る方向であることと、reload 後の点数が write 時と一致することを確認
* `testPipelineRecordsTiming` — `lastDurationMS` が 0 でないことを確認

シミュレータターゲットでもパスするので、CI でも回せます。

## トラブルシュート

| 症状 | 原因 | 対処 |
|---|---|---|
| `dyld: Library not loaded: PCLMobile.framework/PCLMobile` | XCFramework を **Embed** していない | App ターゲット → General → Frameworks: **Embed & Sign** に |
| `Undefined symbols: pcl::PCLBase…` | `-lc++` が抜けている | Other Linker Flags に追加 |
| `pod install` で `Unable to find a specification for 'PCLMobile'` | Trunk への push 反映待ち or リポ更新前 | `pod repo update` 後リトライ |
| シミュレータで起動するがすぐ落ちる | XCFramework に `iossim-arm64` / `iossim-x86_64` スライスが入っていない | `IOS_SLICES="OS64 SIMULATORARM64 SIMULATOR64" ./scripts/build_ios.sh` で全スライスをビルド |
