# PartMaker 刷新: OCCT バックエンド実装報告(2026-09-04)

対象: `docs/HANDOVER_partmaker_renewal.md` の段取り S0〜S2
差出: PartMaker 側(Claude セッション)
前提: Gemini が 2026-09-03 に着手し、掃引がねじれて失敗(`docs/REVIEW_bead_sweep_twist.md`、`ref/*.png`)

---

## 0. 結論

**CATIA/DELMIA を使わずに STEP を直接生成するバックエンドが動いている。**

| 指標 | CATIA/DELMIA(現行) | OCCT(新) |
|---|---|---|
| 速度 | 11.6 → 27.5 秒/部品(セッション劣化あり) | **0.11 秒/部品**(300部品バッチ実測、params/features 書き出し込み) |
| 実行環境 | Windows + DELMIA ライセンス + GUI + クリップボード | Python のみ。Linux/Kaggle 可、`multiprocessing` 可 |
| チャンク毎の再起動 | 必要(メモリ 0.8→1.8GB) | 不要 |
| エッジのプリミティブ適合(ゲートA) | 0.25t 超過 0.0% | **最悪 0.000000mm**(300部品全エッジ) |
| 0.05mm 未満のゴミエッジ | 607本 | **0本** |
| シェルの妥当性 | — | `BRepCheck_Analyzer` 300/300 有効 |
| 面数/部品 | ビード 125 / フランジ 31 | ビード 57 / フランジ 15 / 素 3〜5 |

Gemini 版の `gsd_build.py` 上書きは `git stash`(`gemini-occt-gsd_build-attempt`)へ退避し、
**CATIA 版 `gsd_build.py` は復元した**(引継ぎ書 §3.3「CATIA は捨てない — 移行ゲートの対照」)。
OCCT は新しいファイル `synthetic_generator/src/synthetic_generator/occt_build.py` に置いた。

---

## 1. Gemini 版が失敗した理由と、採らなかった理由

レビュー(`REVIEW_bead_sweep_twist.md`)の指摘どおり `_orient_profile_to_frame` の
`gp_Trsf.SetTransformation` が逆向きで、断面が接線を含む平面に寝ていた。ただし
**レビューが推奨した「分解構築 + 単断面 `MakePipeShell`」も採らなかった。**理由:

`BRepOffsetAPI_MakePipeShell` は掃引フレームをスパインから内部規則(Frenet/CorrectedFrenet/
binormal)で決めるので、**どの規則を選んでもフレームの決定権が OCCT 側にある**。レビュー自身が
「修正後は 3 モードとも同じ結果 = トライヘドロン法則は原因でも解でもない」と書いており、
面の種類も B-spline に落ちていた(修正後 6 断面で「全 45 面 B-spline」)。

代わりに**掃引を自前で行う**。この部品族には、それを可能にする性質がある。

---

## 2. 構築原理: 折れ目軸が共通なら、部品は「断面 × 中心線」で厳密に書ける

### 2.1 実測した性質

自由折れ目チェーン(`classify.free_fold_seed`)は **w 平行構成**である。
`solve_free_fold` の傾き摂動を 0 にすると、2 本の折れ目軸は厳密に平行になる:

| 傾き摂動 | 計画の成立率 | 2軸のなす角 中央値 / p90 / 最大 |
|---|---|---|
| ±3度(従来) | 90.0% | 1.00° / 3.02° / **7.28°** |
| **0(採用)** | **97.2%** | 0.00° / 0.00° / **0.07°** |

(400 試行、`templates.general_two_point.sample` → `plan_general_two_point`)

摂動を 0 にすると**成立率が 7 ポイント上がる**という副産物もある。失うのは ±3度 の
シアー多様性だけで、締結点の横ズレ由来の傾き(seed 自身の a1、中央値 約5度)は残る。
`FOLD_TILT_PERTURBATION_RANGE_DEG = (0.0, 0.0)` に変更した(理由をコメントに明記)。

### 2.2 構築

共通軸を **w** とすると、中立面は「w に直交する平面内の 2D 中心線(直線-円弧-直線-円弧-直線)」に
沿って**断面プロファイル**を掃引したものになる。掃引は 2 種類しかない:

* 直線区間 → `BRepPrimAPI_MakePrism`(平行移動)
* 曲げ区間 → `BRepPrimAPI_MakeRevol`(軸 = (曲げ中心, **w**) まわりの回転)

**w は大域固定方向なので、掃引フレームが捻れることが原理的に起こり得ない。**
断面は「現在フレーム」を平行移動/回転して持ち回るので、区間の継ぎ目は浮動小数点精度で一致し、
`BRepBuilderAPI_Sewing(0.01)` が必ず閉じる。

面の種類も構成的に決まる:

| 断面要素 | 直線区間 | 曲げ区間 |
|---|---|---|
| 直線(平地・ビード壁・頂部・フランジ壁) | 平面 | 円錐 / 円筒 / 平面(円環) |
| 円弧(稜線R・足R・フランジ根本R) | 円筒(横ズレがあると斜め押し出し) | トーラス |

### 2.3 曲げ中心の式(検算済み)

パネル A(法線 n_A、面内で折れ目に直交する前向き h_A)とパネル B が、w まわりに符号付き角 φ で
つながるとき、接線長 T = R·|tan(φ/2)| として

```
接点_A = F - T·h_A ,  接点_B = F + T·h_B ,  曲げ中心 = 接点_A - sign(φ)·R·n_A
```

すなわち曲げ中心は φ>0 で −n 側、φ<0 で +n 側。この結果、曲げ上で基準面から法線方向に
高さ h 離れた点の半径は **R + sign(φ)·h** になる。ビード/フランジの凹側交差判定はこの式で行う。

### 2.4 横ズレ(シアー)の扱い

締結点2は締結点1から w 方向に dw だけずれる(`MAX_LATERAL_OFFSET_RATIO = 0.10`)。
曲げ区間は w 座標を保存する(w 軸まわりの回転だから)ので、**直線区間の変位に w 成分を上乗せ**して
帯全体をシアーさせる。配分は直線区間の長さ比。

> 落とし穴(実測): 補正後の方向ベクトルを**正規化してはいけない**。正規化すると u 方向成分が
> `length·rate²/2` だけ縮み、終端が締結点2から 0.08mm ずれる。変位ベクトルをそのまま使う。

この構成では、帯の側端は曲げ区間で w 一定(=円弧)になる。材料としての正しい側端は螺旋になるが、
CATIA のエッジフィレットも同じく円弧でトリムしている。差は Rφ·tanγ/2 ≈ 0.65mm 以下。

---

## 3. ビード / フランジ / 余肉カット

### 3.1 ビード(9要素断面 + 両端ランアウト)

断面は `平地 / 足R / 壁 / 頂稜線R / 頂部 / 頂稜線R / 壁 / 足R / 平地` の 9 要素で G1 に閉じる。
中心線に沿って s = 2·bearing半径 から始まり、深さの2倍(空きが足りなければ短縮、下限3mm)の
**ランアウト**で平地断面へ戻す。ランアウトは要素ペアごとの `brepfill.Face`(直線織り面)。

> `BRepOffsetAPI_ThruSections` は断面エッジを B-spline に作り直す(実測: 部品あたり 20 本の
> 非解析エッジ)。`brepfill.Face` は入力エッジをそのまま境界にするので、円弧が円弧のまま残る。

立ち上げ向き(±法線)は**部品ごとに選ぶ**: 曲げ上でビード頂部の半径は R + sign(φ)·lift·深さ に
なるので、最小半径が最大になる向きを採る。それでも中立面R最小(5mm)を割る組み合わせは
Infeasible として弾く(サンプラーが引き直す)。CATIA 版にはこの検査が無かったので、
**既存 2807 部品には曲げ上のビード頂部が R5 未満のものが混ざっている可能性がある**。

### 3.2 フランジ(3要素断面、ランアウト不要)

断面の端に `根本R(90度) + 壁` を足すだけ。根本Rは**名目幅の外側**に置くので、
CATIA 版で必要だった `side_extension`(外挿の代替)が要らない。全長で断面が変わらないので
ランアウトも不要 = **B-spline 0 枚**。曲げ上では壁が「軸に直交する平面内の円環」になる(平面)。

### 3.3 余肉カット

両端の隅を R = 締結点の必要最小半径 で落とす(ビード4隅 / フランジは反フランジ側2隅)。
接線円弧と隅点で囲んだ面を法線方向 ±5mm に押し出し、`BRepAlgoAPI_Cut`。

> ツールの厚みは薄くする(±5mm)。厚くすると U 字部品で反対側のパネルまで削る。
> また `hw - R` がほぼ0になる組み合わせ(half_width は bearing半径の1.0〜1.3倍)で
> 0.04mm のゴミエッジが出るので、R を `half_width - 0.5` で頭打ちにしている。

### 3.4 面ラベル

面名は構築時に確定している(`panel_1_bead_wall_l`、`bend_0_flange_root`、`runout_0_bead_top` …)。
STEP には XCAF で書き、`features/*.json` の `faces[]` に
`{name, role, feature, geo, area_mm2, radius_mm, centroid}` として出す(引継ぎ書 §4.2)。

> 落とし穴(実測): `Interface_Static.SetIVal("write.stepcaf.subshapes.name", 1)` は
> **`STEPCAFControl_Writer` を1つ構築するまで存在しない**。先に呼ぶと `SetIVal` が False を返し、
> 面名が1つも書かれない(`ADVANCED_FACE('')` になる)。読み側の
> `read.stepcaf.subshapes.name` も同じ。

---

## 4. 実測(2026-09-04)

### 4.1 品質(`synthetic_parts/occt01/chunk_01`、300部品)

```
parts 300  invalid 0  worst primitive deviation 0.000000mm  edges<0.05mm 0
faces  min 3 med 57 max 59
edges  min 14 med 136 max 140
problems: none
```

* **ゲートA(エッジ = 単一の直線/円弧)**: 全部品の全エッジで最大ずれ 0.000000mm。
  ランアウトの織り面の稜線は OCCT 上 B-spline 型だが**幾何的に厳密な直線**なので、
  抽出器のプリミティブ当てはめには影響しない(型ではなく当てはめ残差で判定した)。
* **ゲートB(面ループ)**: 自由エッジは外形のみ(ビード16本 / フランジ18本)。
  内部に縫合漏れなし。`face_ids` は推定ではなく**構築時の名前**。
* **ゲートC(合理性の床)**: ゴミエッジ 0、締結点は面上(<0.1mm)、
  ビードのねじれ検出(頂部 ≤0.1mm / 鏡像 ≥0.8×深さ)を全ビード部品で通過。
* **ゲートF(速度)**: 0.11 秒/部品(要求 ≤2秒)。

### 4.2 面の種類の内訳(200部品)

```
plane 2105 / bspline 1918 / extrusion 1680 / torus 1125 / cylinder 973 / cone 538 / revolution 15
```

* `bspline` はランアウトの織り面だけ(ビード部品あたり14枚)。フランジ部品には無い。
* `extrusion`(= `surface_of_linear_extrusion`)は、横ズレのある部品の直線区間で
  円弧要素を斜めに押し出した面。**エッジは厳密な円のまま**。真の円筒にするには
  横ズレを 0 にする(=入力の多様性を捨てる)しかないので、こちらを採った。

### 4.3 歩留まり

200部品で 57.1%(350試行)。棄却の内訳は「補強の解決不能」86件が最大で、これは既存サンプラーの
挙動。OCCT 固有の棄却は「ビード深さ > 曲げR − 5mm」「ランアウトが平坦区間に収まらない」
「折れ角が小さすぎてスリバー面になる(円弧長 < 1mm)」。**棄却は全て OCCT に触る前の純Python判定**
なので、0.11秒/部品は成功部品あたりの実効値。

### 4.4 配置クラスの分布(300部品、クォータ無し)

```
oblique 146 / parallel_same_offset 113 / orthogonal 35 / parallel_opposite 4 / coplanar_flat 2
```

偏り 73倍。`generate_general_batch(class_quota=...)` はそのまま使えるので、本番では
prod02 と同様にクォータを渡すこと(未適用)。

---

## 5. 現行契約との差分(意図的な変更)

**既存フィールドの削除・意味変更はしていない。**追加のみ(引継ぎ書 §2.4 の約束を維持)。

| 項目 | 変更 | 理由 |
|---|---|---|
| `fold1_tilt_perturbation_rad` | 常に 0 | 折れ目軸を共通にするため(§2.1)。値は spec に残る |
| ビードの平面視形状 | 閉じた角丸ループ → **両端ランアウト付きの帯** | 解析曲面だけで構築するため。実物の BIW ビードとしてはむしろ一般的 |
| `bead.corner_radius_mm` | サンプルされるが**幾何に反映されない** | 平面視の四隅が無くなったため。`reinforcement_direction_deg` と同じ扱い |
| `catpart_path` | 空文字 | CATPart は作らない(OCCT では意味がない)。`joints.json` は `stp_file` しか使わない |
| `features` schema | `partmaker_features/1` → `/2` | `faces[]` と `backend` を**追加**しただけ |
| `manifest.json` / `_COMPLETE` | 新規 | 引継ぎ書 §4.2/§4.6 |
| `joints.json` | 25部品ごとに逐次保存 | 途中で落ちても失われないように(2026-08-26 に315部品で発生した事故の再発防止) |
| 締結点の穴 | 開けない | ユーザー指定(2026-08-30 / 2026-09-04) |

---

## 6. 追加/変更したファイル

| ファイル | 内容 |
|---|---|
| `synthetic_generator/src/synthetic_generator/occt_build.py`(新) | OCCT バックエンド本体。`OcctPartBuilder.build_general_two_point` は CATIA 版と同じ引数 |
| `tools/run_occt_batch.py`(新) | 1チャンク生成 + params/features/manifest/_COMPLETE |
| `tools/occt_smoke.py`(新) | スモーク + 幾何合格判定(ゲートA〜C、ねじれ検出) |
| `tools/render_views.py`(新) | ヘッドレス4方向レンダリング(GUI不要、CI/エージェント用) |
| `templates/general_two_point.py` | `FOLD_TILT_PERTURBATION_RANGE_DEG = (0.0, 0.0)` |
| `batch_generate.py` | `joints.json` の逐次保存 |
| `gsd_build.py` | **CATIA 版を復元**(Gemini の上書きは `git stash` へ退避) |

---

## 7. 残っていること

| 段 | 内容 | 状況 |
|---|---|---|
| S1/S2 ゲート A・B・C・F | 上記 §4.1 | **通過** |
| S1 ゲート D(CATIA との幾何一致 ≤0.3mm) | 未実施。ビードの平面視形状を意図的に変えたので、ビード部品では原理的に一致しない。基準面のみでの比較なら意味がある | 要判断 |
| S3 ゲート E(既存2300部品を再生成して ML 再学習) | 未実施(ML 側の作業) | ML 側へ |
| クラスクォータ | `class_quota` を本番バッチに渡す | 未適用 |
| 新族(S4: 締結点3〜6、穴、複数特徴) | 未着手。断面掃引の枠組みは締結点N個の折れ目チェーンにそのまま延長できる(共通軸の条件は要確認) | 次段 |
| 展開図 / 構築プログラム / キャプション(§4.2) | 未着手。断面 + 中心線 + 区間列がそのまま「構築プログラム」なので、`occt_build` の経路構築から機械的に出せる | 次段 |
| 既存 2807 部品(CATIA製)の扱い | そのまま残っている。再生成するかは判断待ち | 要判断 |

---

## 8. 使い方

```bash
# 1チャンク生成(既存の出力契約 + manifest + features)
python tools/run_occt_batch.py occt01 1 300 20260904

# 品質チェック(ゲートA〜C + ねじれ検出)
python tools/occt_smoke.py 200 424242

# 目視(ヘッドレス)
python tools/render_views.py out.png synthetic_parts/occt01/chunk_01/mid/<id>_mid.stp
python tools/render_views.py grid.png --grid <stp> <stp> ...

# 対話ビューワ(ユーザー用)
python tools/step_viewer.py
```
