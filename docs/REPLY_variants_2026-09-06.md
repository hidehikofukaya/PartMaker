# 回答: 設計等価バリアントの拡張と、締結点摂動ペア(2026-09-06)

宛先: AutoMetalSheet(ML 側) / 差出: PartMaker
対象: `docs/REQUEST_variants_2026-09-06.md`

## 0. 結論

- **依頼 A は実装し、全 8 チャンク(occt11/12/13/15/16/17/18/19、5,400 部品)に適用した。**
  ツールは `tools/emit_variants.py`(暫定版を置き換え)、列挙の本体は
  `synthetic_generator/src/synthetic_generator/variants.py`。
- **依頼 B は 2 点族(occt11/12)と 007 型(occt13)で各 100 部品 × 3 摂動を作った。**
  `tools/emit_perturbations.py`。011 型 / 014 型(occt15/16)は掃引のアンカーが実点ではなく、
  実点だけ動かしても面が変わらない(対にならない)ので**未対応**(§3.3)。
- 件数・歩留まりは §4。

## 1. 何が「入力から決まらない」か(族ごとの自由度の棚卸し)

締結点(位置・法線・座面半径)と板厚・曲げRを固定して、生成器のコードを読んで洗い出した。
**締結点が動いてしまう選択は変えられない**(それは別の入力に対する部品になる)。

| 族 | 自由 = knob | 固定(締結点が動く) |
|---|---|---|
| 2曲げ(occt11 の 74%) | `slack`(折り位置 2 本)、`width`(帯の半幅)、`bead`(断面)、`side`/`height`/`root_r`(フランジ) | 折れ角(点と slack で決まる) |
| 1曲げ(rib/plain、3点族) | `width`、`bead`、`side`/`height`/`root_r`、`rib`(幅と膨らみ) | **折り位置**(2点と法線の交線で一意。slack は幾何を動かさない) |
| 平板 occt17 | `margin`(余白)、`corner_r`(隅R) | 凸包(締結点で決まる。「最小 vs 余裕」は margin で表現) |
| 分岐 occt18 | `corner_r`(ハブ角のくぼみ)、`arm_len`(腕の長さ。締結点+座面+先端逃げを含む最小値から +0/+12/+24) | ハブの形・腕の幅・折れ角(腕上の締結点が動く)。ガセットは無効 |
| チャンネル+座面 occt19 | `length`(ウェブ長、奥側へ)、`seat_depth`(座面の奥行き) | 幅・深さ・壁角・斜辺・**座面の張り出し側**(座面上の締結点が動く) |

依頼の候補表との差:
- 「折り位置」は 2 曲げ部品だけ。1 曲げ部品(occt12/13/15/16 の全部)は折り線が一意。
- 「ビードの位置」は無い(ビードは全長を貫くのが規則)。幅は `bead` で振る。
- 「リブの位置」は 1 曲げ部品では曲げが 1 本なので無い。幅・膨らみは `rib` で振る。
- 分岐の「ハブの形・腕の幅」、チャンネルの「幅・深さ・座面の側」は締結点が動くので不可。

値の取り方: 連続量は範囲の分位点(15/50/85%)から元の値と 1mm 以上違うものだけ、
離散(側)は反転、断面(ビード/リブ)は族のサンプラーで引き直して事前判定に通ったもの。
帯の半幅の下限は「座面半径」「絞り幅+2」「追加の締結点の幅方向オフセット+座面半径」の最大
(必要平面のルールを守る)。1 部品あたり最大 8、knob を交互に並べて少数でも全自由度が入るようにした。

## 2. 出力の契約(依頼 A)

```
<chunk>/variants/<part_id>__<knob>=<value>_mid.stp   形状(ゲートは本体と同一: 直線/円弧のみ、
                                                     外形ループ 1、二面角 ≤150°、締結点が面上)
<chunk>/variants/params/<name>.json                 spec + bead/flange/rib + knob/value/changed
<chunk>/variants/features/<name>.json               faces[](features v2 と同じ面ラベル)
<chunk>/variants/<name>.infeasible                  不成立の理由
<chunk>/variants/manifest.json                      {"parts": {part_id: [{name, knob, value,
                                                     changed:{field:[before,after]}, status, file|reason}]}}
```

- `annotated_points` / joints は元部品と同一(params の spec に同じ値が入っている)。
- 既存の `mid/ params/ features/ annotations/` は触っていない。
- 暫定ツールが置いた `__side±1_mid.stp`(occt11/13/15/16、各 600)は残してある。新しい
  `__side=±1` と同じ設計なので、どちらかを消してよい(こちらは消していない)。
- 再実行は済んだ名前を飛ばす。部品ごとの乱数は part_id から決まる(再現可能)。
- フランジのキラリティ再試行(本体の `build_general_part` が根本フィレット失敗時に側×方向を
  反転する救済)は**使っていない**。knob を勝手に変えないため。

## 3. 出力の契約(依頼 B)

```
<chunk>/perturb/<part_id>__p<k>_<dx>_<dy>_mid.stp
<chunk>/perturb/<part_id>__p<k>_<dx>_<dy>.json
    {source_part_id, kind, point_index, moved_from, moved_to, vector_xyz,
     distance_over_bearing, joints[], faces: {changed, unchanged, added, removed}, face_labels[]}
<chunk>/perturb/manifest.json
```

### 3.1 摂動の取り方

- 点は point1/point2 から等確率で 1 つ。法線は保ち、**面内**(法線に直交する乱数方向)に
  座面半径の 0.5〜2.0 倍だけ動かす。
- 他の設計選択(slack、帯幅、ビード/フランジ/リブのパラメータ)は元のまま。
- 成立しなければ引き直し(1 摂動あたり最大 6 回)。不成立は manifest に理由つきで残す。

### 3.2 面の差分

features v2 の `faces[].name` で対応づけ、面積が 1% 以上または重心が 0.5mm 以上動いた面を
`changed`、名前が増えた/消えたものを `added`/`removed`。掃引族では締結点を動かすと中心線が
変わるので、多くの面が `changed` になる(端パネルの座面まわりが最も大きく動く)。

### 3.3 未対応: 011 型 / 014 型

occt15/16 は掃引のアンカー(point1/point2)が実点ではなく、対の中点などの**仮想点**で、
実点は `annotated_points` として面上判定だけに使っている。実点を 1 つ動かしても
面は変わらず(X' = X)、動かした点が帯から外れれば不成立になるだけで、教師にならない。
対応するには「動かした実点からアンカーを再導出する」(族の対配置ロジックの逆問題)が要る。
必要なら次の依頼で。「1 点追加」も同じ理由で未着手。

## 4. 結果

### 4.1 依頼 A(全部品、1 部品あたり最大 8)

| チャンク | 族 | 部品 | バリアント | 不成立 | 1部品あたり(中央値 / 最小 / 最大) | knob の内訳 |
|---|---|---|---|---|---|---|
| occt11 | bead / flange(2曲げ 74%) | 1,200 | **7,775** | 234 | 7 / 1 / 8 | slack 2203, width 1933, bead 1596, height 966, side 600, root_r 477 |
| occt12 | rib / plain(1曲げ) | 800 | 3,365 | 0 | 5 / 1 / 6 | rib 1740, width 1625 |
| occt13 | three_point(007) | 600 | 3,785 | 0 | 6 / 2 / 8 | width 1448, height 1137, side 600, root_r 600 |
| occt15 | three_point_tri(011) | 600 | 2,754 | 0 | 5 / 2 / 7 | height 1135, side 600, root_r 600, width 419 |
| occt16 | three_point_span(014) | 600 | 2,993 | 3 | 5 / 1 / 8 | height 1209, width 896, bead 550, root_r 338 |
| occt17 | flat_plate | 400 | 1,899 | 3 | 5 / 3 / 6 | margin 1045, corner_r 854 |
| occt18 | branch | 600 | 3,028 | 0 | 5 / 5 / 6 | arm_len 1800, corner_r 1228 |
| occt19 | channel_seat | 600 | 3,097 | 0 | 5 / 3 / 6 | length 1619, seat_depth 1478 |
| occt20 | tab_bracket(1285-18、2026-09-06 追加) | 600 | 2,439 | 0 | 4 / 2 / 5 | arm_len 1685, fillet_r 754 |
| 合計 | | 6,000 | **31,135** | 240 | | |

- バリアントが 0 の部品は無い。4 個未満は 444 部品(plain と 011 型に多い — 自由度が
  帯幅とフランジしか無く、帯幅は 3 点目の必要平面で下限が上がる)。
- 不成立 240 のうち 219 は occt11 の `slack`(折り位置を変えると折れ角が 3°台になり
  「ほぼ平坦な無駄折れ」のゲートに当たる)。残りはビードの足が帯に収まらない等。全て
  `.infeasible` に理由がある。
- 011 型の `width` が少ない(419)のは、3 点目の幅方向オフセット + 座面半径が帯幅の上限に
  近く、振れる幅が 1mm 未満の部品が多いため。
- ホールドアウト(`tools/_val_ids.json`)は各チャンクで先に処理してある。

### 4.2 依頼 B(2 点族 + 007 型、各 100 部品 × 3)

| チャンク | ペア | 引き直しで不成立 | 変化した面の数(中央値) |
|---|---|---|---|
| occt11 | 298 | 254 | 39 |
| occt12 | 300 | 48 | 12 |
| occt13 | 300 | 37 | 7 |

- occt11 の不成立が多いのは 2 曲げ部品で、点を動かすと slack 固定のままでは折りが
  成立しない(`free_fold_seed` 不成立、傾き 5° 超)配置が多いため。引き直しで 298/300 は
  埋まった(2 ペア分は 18 回の引き直しでも成立せず)。
- occt11 は掃引の中心線が全体で動くので変化面がほぼ全面(39/約 41)になる。1 曲げの
  occt12/13 は締結点側のパネルと曲げだけが動く(7〜12 面)。

## 5. 使い方

```bash
python tools/emit_variants.py synthetic_parts/occt11/chunk_01 --count 8      # ホールドアウト優先
python tools/emit_variants.py synthetic_parts/occt19/chunk_01 --ids SYN_general_two_point_0001
python tools/emit_perturbations.py synthetic_parts/occt11/chunk_01 --parts 100 --per-part 3
```

テスト: `synthetic_generator/tests/test_variants.py`(候補が締結点・座面半径・板厚・曲げRを
変えないこと、摂動が面内で 1 点だけ動かすこと、面差分の判定)。
