# ビード構築 検討フローチャート

作成: 2026-08-24(Fable 5セッションからの引継ぎ)
**更新: 2026-08-24 — 【解決済み】曲げ跨ぎビードの全7手順がスパイクで完走。§6 参照。**
前提文書: `docs/catia_bead_fillet_investigation_log.md`(調査の全履歴・CATIA仕様)、`docs/synthetic_two_joint_generation_roadmap.md` §6.25(プロジェクト文脈)

> **注意**: 以下の §1〜§5 は解決前の検討記録として残してある。
> 現在の到達点と確定レシピは **§6「決着」** を読むこと。

## 0. プロジェクト文脈(最終目標)

CATIAで作成した板金中立面データをAIに学習させ、**締結点を入力したら板金中立面形状が生成されるAI**を作る。
現在は締結点2個に限定し、多様な学習データをバッチ生成するスクリプト(`synthetic_generator/`)を開発中。
本検討はその中の「曲げをまたぐビード補強」形状の自動構築(roadmap §6.25 Step 2)。

## 1. 現在地(2026-08-24時点の確定事実)

### 1.1 【最重要確定】エッジ(BRep)参照のフィレットはスクリプトから自動化不能

`investigation_log.md` §3.5 に完全な検証記録。要点:
- `Selection.Search`参照・DisplayName→BRepName再構築参照・マクロ原文のBRepName文字列、いずれも
  `CreateReferenceFromBRepName`と`AddObjectToFillet`は**受理される**が、`part.Update()`の解決で必ず失敗。
- 最小のExtractですら同参照で失敗 → フィレット演算ではなく**参照解決自体が失敗**。
- 手動成功したCATPart上でマクロを一字一句再生してもCOMから失敗。`SystemService.Evaluate`(CATIAプロセス内VBScript)でも失敗。
- **結論: GUIダイアログのみ成功。スクリプト経路は全滅。エッジフィレット依存の設計は放棄。**

### 1.2 【解決】BiTangentシェイプフィレット路線で壁バンドまで完成

`AddNewFilletBiTangent(iElement1, iElement2, iRadius, iOrientation1, iOrientation2, iSupportsTrimMode, iRibbonRelimitationMode)`
は**サーフェスフィーチャー参照のみ**(CreateReferenceFromObject、BRep不要)で動く。実証済み(`tools/probe_output/bitangent_band_success.CATPart`):

1. 壁を1本の閉ループスイープではなく、**4枚の個別オーバーサイズ壁**として作る:
   - 壁A/C: `root_left`/`root_right`(AddNewCurvePar、全長のまま)のMode=4スイープ
   - 壁B/D: `guide_near/far`(マージン20mm付き)のMode=4スイープ
   - **開曲線のMode=4スイープは正常動作**(新知見)
2. BiTangent3回で4隅R付き連続壁バンドに統合:
   - f1 = BiT(A,B) — 成功方向 o1=-1,o2=-1
   - f2 = BiT(C,D) — 成功方向 o1=1,o2=1
   - f3 = BiT(f1,f2) — 成功方向 o1=-1,o2=1、**2隅同時(2交差リボン)も成功**
   - 方向は総当たり+プローブ点(「残るべき側の点」との距離<0.1)で機械的に選定(orientationは形状依存なので毎回総当たりが必須)
   - プローブ点は**ベンドフィレット領域外**(平面部、接線点run≈R·tan(θ/2)より外)に置くこと(円筒面上だと平面計算座標が浮いて誤判定)
3. 面積検証: バンド2693.84mm² ≈ 理論値(ループ長287.31×slant9.14=2626)+R補正 → 幾何学的に正しい

### 1.3 【未解決】頂稜線R: topR = BiT(バンド, 頂面オフセット) が全方向失敗

- 頂面 = `AddNewOffset(base, depth, orient=1)`(probe-and-flipで方向確定)は成功
- 壁を上下延長(SetLength(1, slant*1.4) / SetLength(2, 3.0) — **SetLength(2)による下方向延長は動く**)して頂面を貫通させた
- しかし BiT(バンド, 頂面) はコーナーR=2でもR=6でも全4方向失敗(汎用Update失敗)
- 仮説(未検証): 交線が閉ループ+8パッチ(4壁+4コーナーリボン)跨ぎのリボン生成が失敗する

### 1.4 【ユーザー観察・2026-08-24】

ユーザーがCATPartを目視確認した結果:
- **スイープのドラフト方向がおかしい**(壁の立ち上がり向きが意図とずれている) — 要修正
- **フィレット(BiTangentのコーナーR)はうまくいっている**
- **「後はトリムさえできれば」** — 残る本質課題はトリム統合という認識

## 2. 検討フローチャート(次にやることの分岐)

```mermaid
flowchart TD
    START[現在地: 4隅R付き壁バンド完成済み] --> C1[C: ドラフト方向の修正\nユーザー指摘の事実確認が最優先]
    C1 --> C2{壁の立ち上がりが\n意図した向きか?}
    C2 -- おかしい --> C3[SetAngleの符号/角度の取り方を検証\n(90-θ)や負角、Mode=4の角度基準を\nスパイクで確認し正しい向きに直す]
    C3 --> A1
    C2 -- 正しい --> A1[A: ユーザー7手順の順序に戻す\n⑤トリム統合を稜線Rより先に]
    A1 --> A2["trim1 = AddNewHybridTrim(base, band)\n実績パターン: o1,o2総当たり+プローブ\n(§2.6で(1,1)成功実績あり)"]
    A2 --> A3["頂面を壁バンドでSplitして内側だけに\n(フィーチャー同士、BRep不要)\nまたはtrim2 = Trim(trim1, top)"]
    A3 --> A4{R無し稜線の\nビード形状が完成?}
    A4 -- Yes --> A5[まずこれを形状として確定\nバッチ生成に載せられる状態にする\n稜線Rの要否はユーザー判断を仰ぐ]
    A4 -- No --> B1
    A5 --> B1[B: 稜線Rの決着(並行調査)]
    B1 --> B2["B-1: footR = BiT(バンド, 基準面)を先に試す\n(topRと同じ閉ループ+パッチ跨ぎ構造\n→失敗すれば原因が構造側と確定)"]
    B2 --> B3["B-2: topRのRibbonRelimitationMode(0/2)\nSupportsTrimMode(0/2)総当たり\n(1,1しか試していない)"]
    B3 --> B4["B-3: 頂面を先にSplitで小さくしてから\nBiT(バンド, 頂面内側)\n(外側の広大な面がトリム解決を\n壊している可能性)"]
    B4 --> B5{どれか成功?}
    B5 -- Yes --> DONE[⑥⑦完了→bead.py/gsd_build.pyへ統合]
    B5 -- No --> ALT["代替: 稜線シャープのまま採用\n(実ビードの稜線Rは小さく、\n学習データとしての影響をユーザーと相談)\nまたは断面カーブ自体にRを含めた\nスイープ(プロファイル側にR)を検討"]
```

## 3. 実装時の確定パラメータ・レシピ(コピペ用)

- フィレット列挙値(§investigation_log 4.2): `EdgePropagation`: Minimal=0/Tangency=1、`FilletBoundaryRelimitation`: 0..4(Connect=2)、`FilletTrimSupport`: Trim=0/NoTrim=1
- BiTangent呼び出し実績: `hsf.AddNewFilletBiTangent(ref1, ref2, R, o1, o2, 1, 1)` + `body.AppendHybridShape` + `part.Update()`
- 方向選定: (o1,o2)∈{±1}²総当たり、成功後 `GetMinimumDistance(probe)<0.1` で採用判定、不採用は必ず`cleanup_failed`(Selection.Add→Delete→Update)で削除(残すと後続Updateを巻き添えにする)
- コーナーR=6mm・稜線R=2mm(コーナーR>稜線Rの関係を維持する設計とした — 曲率競合回避、製造実務とも整合)
- トリム統合実績: `AddNewHybridTrim(surface_ref, 1, sweep_ref, 1)`(ただし方向は形状依存、総当たり+プローブで)
- **gencache絶対禁止**(investigation_log §2.4)。列挙値調査は`_oleobj_.GetTypeInfo()`直接読み(同§4.5)

## 4. 検証済みスパイクスクリプト(再現用)

session scratchpad(`C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\`)にあり、
主要なものは `tools/` にコピー済み or 要コピー:

| スクリプト | 内容 | 結果 |
|---|---|---|
| `test_brepname_rebuild.py` | DisplayName→BRepName再構築フィレット | 参照取得OK/Update失敗(§3.5) |
| `test_fillet_variants.py` / `test_fillet_reopen.py` | InWorkObject/Append無し/UpdateObject/再Open | 全滅 |
| `test_extract_and_trim_state.py` | Extractで参照解決を切り分け | Extractも失敗=参照解決の問題 |
| `test_replay_macro.py` / `test_inprocess_vbs.py` | マクロ再生(外部COM/in-process) | 両方失敗 |
| `test_bitangent_walls.py` | 壁2枚+BiTangent最小実証 | 成功 |
| `test_bitangent_band.py` | 4隅R付きバンド(3回BiTangent) | **成功**(`tools/probe_output/bitangent_band_success.CATPart`) |
| `test_bitangent_full_bead.py` | バンド+頂面+topR/footR | topRで停止(バンド・頂面まではOK) |
| `test_topr_diagnosis.py` | B-1/B-2/B-3の3フェーズ診断 | **作成途中・未実行**(フローチャートB相当) |

## 5. 生成済みCATPart(目視確認用)

`tools/probe_output/`:
- `bitangent_band_success.CATPart` — 4隅R付き壁バンド(ユーザーが目視確認済み: ドラフト方向に難、フィレットOK)
- `bitangent_walls_success.CATPart` — 壁2枚+コーナーR最小例
- `repro_case_seam_fillet.CATPart` — エッジフィレット問題の再現ケース(手動なら成功する)

---

## 6. 【決着】2026-08-24: 曲げ跨ぎビードの全7手順が完走

スパイク `tools/probe_bead_bitangent_full.py` で、ユーザー指定の7手順が**エッジ(BRep)参照を一切使わずに**
端から端まで通ることを実証した。成果物: `tools/probe_output/bead_strict2.CATPart`

### 6.1 根本原因は2つの符号バグだった

topR が失敗し続けた原因も、ユーザーが目視で指摘した「ドラフト方向がおかしい」も、
どちらも**私(アシスタント)側が向きを解析的に決め打ちしていたこと**が原因で、CATIA側の制約ではなかった。

**バグ1: スイープのドラフト角は壁ごとに違う**

`AddNewSweepLine` の `SetAngle(1, θ)` に固定値 `+50` を渡していたが、これは壁を基準面の**下方向**に掃引していた
(診断: 頂部の期待点までの距離が内側候補・外側候補とも `wall_slant` ちょうど = 壁が上に来ておらず最近点が根元曲線のまま)。
正しい角度は掃引するガイド曲線自身の向きに依存し、壁ごとに異なる:

| 壁 | 正しい角度 |
|---|---|
| A(left, root_left) | −50 |
| B(near, guide_near) | −50 |
| C(right, root_right) | −130 |
| D(far, guide_far) | −130 |

→ **固定値を渡してはいけない。候補 `[−θ, −(180−θ), +(180−θ), +θ]` を総当たりし、
頂部エッジが理論上の内側位置に来るものを距離判定で選ぶ。**

**バグ2: 谷折り面ではパネル間で「立ち上がり法線」の符号が反転する**

基準面は2枚のパネルが谷折り(V字)になっており、連続な面法線はパネル1で `+n1` のとき
パネル2では **`−n2`**(くさび内側)になる。ここを `+n2` と決め打ちしていたため、
壁Dだけが裏返しに立ち、topR の keep 判定が `keep_max = 7.00`(= depth ちょうど)で全滅していた。

→ **先に頂面オフセットを作り、そのオフセット面に対して `±n2` の両候補を距離測定して
実際の向き `n2_eff` を確定してから**、壁Dの期待頂部位置とプローブを組み立てる。

### 6.2 BiTangentの向き選定には「消えるべき点」も必要

「残るべき点1〜2個の距離 < tol」だけで採否を決めると、**角に残ったスリバーに当たって誤合格**する
(実際、基準面の大半が消えた形状を一度採用してしまった)。確実な判定には3点セットが要る:

1. `keep_probes`: 基準面**全体に広く散らす**(四隅+辺中央+面中央、実装では16点)。1点では不足。
2. `remove_probes`: 消えるべき点(ビード直下の基準面など)が実際に消えていること(距離 > 1mm)。
3. 面積のサニティチェック(任意だが有効)。

### 6.3 確定した構築レシピ

```
① 基準面: rect_fill ×2 → join → edge_fillet(主曲げR)        [既存]
② 頂面:   AddNewOffset(base, depth, orient) — orientはprobe-and-flip
   → 同時に n2_eff(パネル2の立ち上がり向き)をオフセット面への距離測定で確定
③ 輪郭:   centerline = Intersection(plane, base)
          root_left/right = AddNewCurvePar(centerline, base, half_footprint, False/True, True)
          guide_near/far  = 幅方向の直線(MARGIN=20mm オーバーサイズ)
④ 壁:     4本を個別に AddNewSweepLine(Mode=4, FirstGuideSurf=base)
          SetAngle(1, ang) ← 角度は総当たり+頂部距離判定で選定(§6.1)
          SetLength(1, slant*1.6) / SetLength(2, 3.0) で上下に延長し貫通させる
⑤ バンド: f1=BiT(A,B), f2=BiT(C,D), f3=BiT(f1,f2)  ※f3は2隅同時にリボン生成
⑥ 頂稜線R: hat = BiT(band, top, RIDGE_R)
⑦ 足元R:   bead = BiT(hat, base, RIDGE_R)
   ⑤⑥⑦すべて (o1,o2)∈{±1}² を総当たりし keep/remove プローブで採択
```

実測パラメータ例(検証時): base=15828.84mm² → 最終ビード=16070.55mm²、
CORNER_R=6mm、RIDGE_R=2mm、depth=7mm、wall_angle=50°、top_width=30mm、run_out=15mm

### 6.4 幾何検証の結果(`tools/probe_verify_bead.py`)

- ランアウト端より外側の基準面(中心線上 run=70/75/80): **両パネルとも距離0.000で残存** ✓
- ビード直下の基準面(run=50): 除去され最近点は7.000mm上(=頂面) ✓
- 基準面の遠端 w=−60/0/+60: **全て残存** ✓
- ビード頂面の平坦部(run=50): 距離0.000で一致 ✓
- ※ run=30(ベンドR領域、接線点は run=21)と run=60(ランアウト端R域)での「NG」は
  平面前提の期待値計算が曲面に合わないだけで、形状の欠陥ではない

### 6.5 副次的に判明したCATIA仕様

- **視点のスクリプト設定**: `StartCommand("Isometric View")` は日本語版CATIAでは効かない。
  `viewer.Viewpoint3D.PutSightDirection(tuple)` / `.PutUpDirection(tuple)` を使う
  (メソッド名は `PutSight` ではなく **`PutSightDirection`**)。設定後 `viewer.Update()` → `Reframe()`。
- **真上(−z)からの投影は2枚のパネルが完全に重なる**ため、V字基準面の検証には不適。
  各パネルの法線方向からの正面図(`sight = −n`, `up = u`)を使うこと。

### 6.6 残タスク

1. **本パイプラインを `bead.py` / `gsd_build.py` へ統合**(現状はスパイクのみ)。
   旧セル分解方式を置き換える。既存の `MAX_MITER_AMPLIFICATION` / `run_out_diagonal_mm` ガードは
   マイターオフセット式を使わなくなるため、大半が不要になる見込み。
2. 一般ケース(任意法線・任意位置、単曲げ/多曲げ)での成立確認。現在の検証は対称な2パネル1曲げのみ。
3. `tests/test_bead.py` の更新、roadmap §6.25 の更新。
4. 未コミットの既存修正(ミッター発散・ランアウト自己交差)の扱いをユーザーと確認。
