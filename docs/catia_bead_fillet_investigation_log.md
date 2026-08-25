# CATIA V5 × Python COM — ビード補強(曲げまたぎ)実装調査ログ

最終更新: 2026-08-23

## 0. この文書の位置付け

`bead.py`/`gsd_build.py` の「曲げをまたぐビード補強」実装(§6.25、`docs/synthetic_two_joint_generation_roadmap.md`)を進める過程で、
CATIA V5 Automation API の低レベルな挙動調査に長時間を要した。個々の試行錯誤はスクラッチパッド上の
使い捨てスクリプト(`tools/probe_*.py`, `tools/spike_*.py`、および一時セッションのscratchpad配下)に散らばっており、
再現性・再参照性のために、**トラブルの背景・現状不明点・解決済み事項・判明したCATIA内部仕様**をここに集約する。

roadmap本体(`synthetic_two_joint_generation_roadmap.md` §6.25)は「プロジェクトの意思決定履歴」を追う文書であるのに対し、
本ドキュメントは「CATIA API そのものの癖・仕様・落とし穴」を集約した技術リファレンスとして使う。

---

## 1. やりたかったこと(背景)

### 1.1 最終目標

板金ブラケットの**曲げ部分をカバーするビード補強**を、以下の製造業設計レベルの手順で自動生成する(ユーザー指定の7手順):

1. ビードの底面を、基準面からのオフセットで作成
2. ビードの壁を平面で作成
3. 全ての壁をトリムで1つの連続した壁に仕上げる
4. 壁のコーナーのRをフィレットする
5. 底面と壁、基準面をトリムして1つの形状にする
6. 底面と壁の稜線(閉曲線になるはず)をフィレットする
7. 壁と基準面の稜線(閉曲線になるはず)をフィレットする

平面パネルのみで構成される単曲げ・単純ケースでは②が素直な平面スイープで済むが、**ビードが曲げ(ベンド)をまたぐ場合**、
壁は平坦ではなく曲面(ベンドのフィレット領域)の上を這うため、②③のナイーブな「平面パネル→トリム結合」だけでは対応できない。
これが本調査の主題。

### 1.2 これまでの段階分け(roadmap §6.25 より)

- Step 1: 平面パネル上のビード(完了)
- Step 2: 曲げをまたぐ、端の処理なし(**本ドキュメントの主戦場**)
- Step 3: ビード端の逃げ+締結点クリアランス(未着手)

---

## 2. 解決できたこと

### 2.1 ミッターオフセットの発散バグ(修正済み・commit待ち)

- **症状**: `bead.py` の `_miter_offset` 式 `d*(na+nb)/(1+na.nb)` が、隣接パネル法線na/nbが反平行(180°)に近づくにつれ発散し、最大53倍に増幅されるケースがあった。
- **修正**: `MAX_MITER_AMPLIFICATION = 5.0` 定数を導入し、`bead_fits()` でベクトル長ベースのガードを追加。
- **検証**: 実機で89/89試行、raw failureなしを確認。

### 2.2 ランアウト端壁の自己交差バグ(修正済み・commit待ち)

- **症状**: ランアウト端壁の対角線 `sqrt(2*wall_run_mm**2 + depth_mm**2)` が `bend_radius_mm` に対して短すぎる場合、フィレットが自己交差し、スクリーンショット上で視認できる欠陥(スライバー)が生成物に混入した。ユーザーがスクリーンショットで発見。
- **修正**: `BeadParams.run_out_diagonal_mm` プロパティを追加し、`bead_fits()` に `run_out_diagonal_mm >= 2*bend_radius_mm`(ランアウトインセットが有効な場合)のチェックを追加。
- **ユーザー承認**: 「一旦それで行きましょう」

### 2.3 CATIAビルド時間の非決定性(根本原因未解明、運用で回避)

- 同一ジオメトリのビルドが48秒〜90分以上とセッション/実行ごとに大きくばらつく現象を確認。根本原因は特定できず。
- 実務上の回避策: `bead_probability` を1.0→0.5に下げ、必要ビルド試行回数を減らすことで対応。ユーザーもこれを既知の未解決課題として受容。

### 2.4 `win32com.client.gencache` によるキャッシュ破壊(自己解決)

- 列挙定数(`catTrimFilletSupport` 等)の整数値を調べようとして `gencache.EnsureDispatch`/`EnsureModule` を使用したところ、
  共有の `C:\Users\hide2\AppData\Local\Temp\gen_py\` キャッシュが破損し、`doc.Part` へのアクセスが**プロジェクト全体で**失敗するようになった(本番の `SyntheticPartBuilder` を含む)。
- **修正**: `C:\Users\hide2\AppData\Local\Temp\gen_py\3.13` フォルダを削除して復旧。
- **恒久ルール**: 以後、CATIA自動化では `gencache` 系API(`EnsureDispatch`/`EnsureModule`)を**一切使用しない**。列挙値等の調査は §5.5 の安全な方法(型ライブラリ直接読み取り)を使う。

### 2.5 `AddNewSweepExplicit` が曲げをまたぐと壁がねじれる問題 → `AddNewSweepLine` Mode=4への切り替えで解決

- `AddNewSweepExplicit(iProfile, iGuide)` は単一パネル上の平坦な閉曲線には正しく機能するが、曲げをまたぐガイド曲線に対しては**壁がねじれる/正しくない形状**になる(面積不一致・目視で確認)。
- **原因**: プロファイルを局所サーフェス法線に対して正しく向き付けていないため。
- **解決**: `hsf.AddNewSweepLine(loop_ref)` + `.Mode = 4` + `.FirstGuideSurf = surface_ref` + `.SetAngle(1, angle_deg)` + `.SetLength(1, slant_mm)` に切り替え。曲げをまたいでも理論値(面積・長さ)と厳密に一致する壁を生成できることを確認。
  - この技法は並行セッション("claude rc")の `tools/spike_bead_curve_loop.py` から継承。

### 2.6 基準面+壁のトリム統合(Join ではなく Trim)

- ユーザーの実機検証により判明: 基準面は先にエッジフィレット(主曲げR)を当てて良い。その後、壁(鋭角のまま)を基準面に統合する際は **Join ではなく Trim** を使う必要がある。
- `AddNewHybridTrim(surface_ref, 1, sweep_ref, 1)`(`o1=1, o2=1`)で安定して成功することを確認。

### 2.7 丸めコーナー入り閉曲線の構築(トリム挿入方式)

ユーザー提案: 「先に鋭角の閉曲線を作ってから、頂点ごとにコーナーをトリムで挿入する」方式。

- 4隅それぞれで `AddNewCorner(iElement1, iElement2, iSupport, iRadius, iOrientation1, iOrientation2, iTrim)` を **全4方向(o1,o2 ∈ {1,-1})** 試し、候補を1つに絞らず**全て保持**する。
- 各コーナー候補を、既存の閉曲線に `AddNewHybridTrim` で挿入する際も **候補×トリム方向(4通り)=最大16通り** を全探索する(ユーザーの明示的な指摘: 「作るコーナーの数は4つですむはずで、トリムも…16通りの内の1回は成功するはず」)。
- **バグとその修正(本セッションで発見・修正)**: 最初の実装は「keep_probe(ループ上の既知点)が生き残る**最初の**成功」を採用していたが、これは**遠回りの誤った断片**を拾ってしまうことがあった。
  実例: 4隅を順にトリム挿入していく過程で、全長が 287.31→178.65→176.65→192.80→**350.45**mm と、最後の1手だけ+157mmも跳躍する異常が発生。
  修正: 「keep_probe合格の中で、**直前の全長に最も近い結果**」を採用するよう変更(コーナー丸めは本来わずかに短縮するはずで、大きく伸びるのは誤り)。
  修正後は 287.31→286.45→285.59→284.73→283.88mm と、丸め半径2mmに対して妥当な単調減少になることを確認。
  → **この閉曲線構築ロジック自体は解決済み**(`tools/`配下には未反映、scratchpadの `test_trim_corners_into_loop.py` で検証)。

---

## 3. 現時点で不明なこと(未解決)

### 3.1 【最重要・未解決】ベンド跨ぎ・トリム統合済みシェルへの縦シームエッジフィレットが失敗する

**症状**: 基準面(ベンドフィレット済み)+鋭角壁(SweepLine Mode=4)を Trim で統合したシェルに対し、
壁の4隅にある縦方向の「シームエッジ」(隣接壁パネルの継ぎ目)にエッジフィレットをかけようとすると、
`AddNewSurfaceEdgeFilletWithConstantRadius` が汎用的な `E_FAIL`(`メソッド Update が失敗しました` 等)で失敗する。

**再現条件**: 平面のみ(ベンドなし)の単一パネル + 鋭角ベース + 直接 `edge_fillet` は**成功する**(`test_sharp_base.py` で確認)。
ベンドフィレット済み基準面 + Trim統合済み壁になると**失敗する**。

**試した(が解決に至らなかった)アプローチ**:

1. `AddNewSurfaceEdgeFilletWithConstantRadius` のコンストラクタ引数(edge直接渡し vs `None`+`AddObjectToFillet`)の違い
2. `FilletBoundaryRelimitation` / `FilletTrimSupport` / `EdgePropagation` プロパティの総当たり(ただし後述§3.1.2の理由で**汚染された結果**)
3. 新規ドキュメント(汚染なし)での再現 → やはり失敗、ドキュメント破損が原因ではないと判明
4. ユーザー提供の成功マクロの再現(§4.1参照) → BRepName直接構築は汎用再現不可と判明
5. コーナー丸め(§2.7)によりシームエッジ自体を「フィレット不要な形状」にする回避策 → **これも未解決**(§3.2)

**現在の作業仮説(未検証)**: シームエッジの自動検出ロジックが誤ったエッジを拾っている可能性が高い(§3.1.2参照)。
正しいエッジであれば §4.2 で判明した正しい列挙値(`FilletTrimSupport=0`, `FilletBoundaryRelimitation=2`, `EdgePropagation=1` など)で成功する可能性がまだ残っている。

#### 3.1.1 「頂点近傍探索」の落とし穴が再発

これまでのセッションで確立していたはずの教訓(`find_edge_near`は**エッジの中点**で探索すべきで、**頂点**での探索は複数エッジが交わり曖昧、というルール)を、
本セッションのシームエッジ探索コードで**再び踏んでしまった**:

```python
corner_probes = [c_ln, c_lf, c_rn, c_rf]   # ← これは頂点(交点)
seam_edges = []
for i in range(1, sel.Count2 + 1):
    ...
    dmins = [meas_i.GetMinimumDistance(cp) for cp in corner_probes]
    if min(dmins) < 0.05:
        seam_edges.append(item.Reference)
```

この探索で「距離0.000」としてヒットしたエッジの長さは 22.156mm・41.747mm であり、
理論値であるはずの `wall_slant_mm`(= `depth_mm / sin(wall_angle_deg)` ≒ 9.138mm、本テストのパラメータでの値)と**全く一致しない**。
→ 頂点に複数本集まるエッジ(縦シーム以外に、壁上端・下端の周回エッジなど)を誤ってヒットさせている可能性が高い。

**次にやるべきこと(未着手)**: 各エッジの**両端点3D座標**を明示的に取得し(`AddNewExtract`→`AddNewPointOnCurveFromDistance` 等)、
理論的に計算した「縦シームの始点・終点」の座標と直接照合する。中点距離ベースの曖昧な探索から、幾何学的に厳密な照合に置き換える必要がある。

#### 3.1.2 プロパティ総当たりの結果が汚染されていた可能性

`test_macro_props_bend.py`(初回)では、頂点近傍探索で見つかった6本の「候補エッジ」全てに対し `AddObjectToFillet` が
**プロパティ値に関わらず即座に失敗**した。これは §3.1.1 の通り、そもそも間違ったエッジを掴んでいたためである可能性が高く、
「正しい列挙値の組み合わせでも失敗する」ことの証明にはなっていない。**プロパティ値の当否は未確定のまま**。

#### 3.1.3 【新仮説・2026-08-23追加】CATIA Generic Naming(安定名前付け)の対称性ambiguityが根本原因の可能性

公式文献調査(§4.6〜4.8)により、**より説得力のある根本原因の仮説**が浮上した:

- IBM公式サポート文書に、Trim操作由来のエッジが「不正に生成され選択不能になる」既知の不具合(V5R17以前、一部V5-6R2013でようやく修正)が記録されている(§4.7)。
- CATIAの Generic Naming は、**同じトポロジー的近傍を持つ複数の要素を区別できない**という既知の制約があり、公式の回避策は「**対称的な形状をBreakコマンドで別パーツに分割する**」こと(§4.8)。
- 本プロジェクトの壁は **1本の対称な閉曲線**(4隅が回転対称に近い構造)を`AddNewSweepLine`でスイープしたものであり、4つの縦シームエッジは互いに酷似したトポロジー的近傍を持つ。これはGeneric Naming ambiguityが起きる典型条件と一致する。

**含意**: `Selection.Search`で得られるReferenceが「どのエッジを指しているか」自体が、CATIA内部で曖昧になっている可能性がある。
これが正しければ、プロパティ値の調整や探索ロジックの改善だけでは解決せず、**壁の構築方法自体を変える**必要がある —
すなわちユーザーが元々指定した7手順の通り、壁を**1本の対称なスイープではなく、4枚の個別パネル(または非対称な構築過程を経る形)として作り、トリムで結合する**方式に戻すことで対称性を崩し、Generic Naming の曖昧さを回避できる可能性が高い。

**検証方法**: §3.4(新設)参照 — 現在失敗する形状をCATPartとして保存し、手動でのフィレット試行を依頼する。

### 3.4 【解決・確定】手動検証の結果と、結論

再現用ファイル `repro_case_seam_fillet.CATPart` をユーザーに手動で試して頂いた結果、**手動では成功した**(マクロで確認、§4.1後半参照)。
記録されたマクロを解析した結果、以下が判明した:

1. フィレットの対象(サポート)は **`スイープ.1`(壁の生スイープフィーチャー自身)** であり、`ref_wall`(Trim統合後のシェル)ではなかった。
   ダイアログの「サポート: スイープ.1」「フィレットをかけるオブジェクト: スイープ.1¥エッジ.2」が示す通り。
2. 半径は 5→4→3→2mm と段階的に下げられている(最終的に2mmで成功)。
3. プロパティは `EdgePropagation=Tangency(1)`, `FilletBoundaryRelimitation=Connect(2)`, `FilletTrimSupport=Trim(0)`(§4.2の確定値と一致)。

**これを受けて即座に検証した(`test_fillet_before_trim.py`)**: 「Trim統合前に、生スイープ自身の縦シームをフィレットする」という順序に変更し、
上記と全く同じプロパティ値を使い、`Selection.Search("Topology.CGMEdge,sel")` で得た生スイープの**全24エッジ**(部分的な候補選定ではなく全数)に対して
`AddObjectToFillet`→プロパティ設定→`part.Update()` を試したが、**24/24 全て失敗した**(`AddObjectToFillet` 自体の即時失敗、または `Update()` 失敗が混在)。

**結論(確定)**: ユーザーが手動クリックで得たエッジは同じ `スイープ.1` フィーチャーの edge であり、私が `Selection.Search` で
機械的に列挙した24本のエッジの**いずれか**と topological に同一のはずである。にもかかわらず**全数字がプロパティ値によらず失敗した**ということは、
問題は「どのエッジを選ぶか」でも「プロパティ値」でもなく、**`Selection.Search` + `.Reference` という参照取得方法そのものが、
この壁(3枚のサーフェスパッチ=平面2枚+ベンドフィレット1枚をまたぐ、対称な閉曲線スイープ)に対しては本質的に機能しない**ということを強く示す。

これは §3.1.3 / §4.8 の **Generic Naming ambiguity 仮説を実験的に裏付ける決定的な証拠**である
(単純な単一パッチの平面ケース(`test_sharp_base.py`)では同じ `Selection.Search` 方式が確実に成功しているため、
方式自体が汎用的に壊れているのではなく、**このベンド跨ぎ・対称構造のケースに特有の問題**である)。

**残された2つの道**(詳細は §5.1 に統合):

- (a) `CreateReferenceFromBRepName` を動的に構築する — 実装難度が高く、フィーチャー内部名に依存するため脆い(§3.3で既に非実用的と判断済み)。
- (b) **壁の構築方法を、対称な1本のスイープから、非対称な複数フィーチャーの組み合わせに変更する**(公式ワークアラウンド、§4.8) — こちらを本命とする。

### 3.5 【2026-08-24確定】BRepName動的再構築の完全検証 — 参照は取得できるがCOMでは解決不能

前節(a)を「DisplayNameテクニック」で実際に検証した(`test_brepname_rebuild.py`)。結果、次の一連の事実が**実験で確定**した:

1. **`Selection.Search`で得たReferenceの`DisplayName`はBRepName文字列を返す**。形式は
   `Selection_REdge:(Edge:(...);Cf14:());<サポートフィーチャー内部名>;Z0;G<世代番号>)`。
   縦シームエッジ4本のDisplayNameの`Edge:(...)`部分は、**手動成功マクロのBRepNameと完全一致**した
   (=Generic Naming ambiguity仮説(§3.1.3)は**誤りだった**。CATIAは4本のシームを完全に区別できている)。
2. 変換規則: 先頭の`Selection_`を除去し、末尾の`;<フィーチャー名>;Z0;G<番号>)`を
   `;WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR29)`に**置換**すると、
   マクロと同一形式のBRepNameが得られ、`CreateReferenceFromBRepName`が成功し、`AddObjectToFillet`にも**受理される**。
3. しかし`part.Update()`は失敗する。以下の全バリエーションでも同じ(`test_fillet_variants.py` / `test_fillet_reopen.py`):
   `InWorkObject`をスイープに戻す / `AppendHybridShape`を呼ばない(マクロと同一) / 半径5→2mmの段階下げ /
   `part.UpdateObject(fx)`(個別更新) / SaveAs→Close→再Openしたドキュメント上で実行。
4. **切り分けの決定打**(`test_extract_and_trim_state.py`): 同じ再構築参照を最も単純な`AddNewExtract`に渡しても**Updateで失敗**する。
   → フィレット演算の問題ではなく、**外部COMプロセスから作ったBRepName参照はUpdate時のトポロジー解決自体が失敗する**。
5. 手動成功したまさにそのCATPart上で、マクロのBRepName文字列を一字一句そのまま使いCOMから再生(`test_replay_macro.py`)→ **失敗**。
6. `SystemService.Evaluate`でVBScriptを**CATIAプロセス内部で実行**(`test_inprocess_vbs.py`)→ **同じUpdate失敗**。
   → **記録されたマクロは、スクリプトとして再生しても成功しない**。GUIダイアログのOKは、
   マクロに記録されるAPI列とは異なる内部経路(ダイアログが保持する生のセル選択)でフィレットを構築している。

**総括**: 「ベンド跨ぎスイープのエッジにスクリプトからフィレットをかける」ことは、参照の与え方・プロパティ・実行環境の
いかなる組み合わせでも不可能(GUI手動操作のみ可能)と結論する。エッジ(BRep)参照ベースのフィレットに依存しない設計へ転換する。

**転換先(検証中)**: `AddNewFilletBiTangent`(GSDシェイプフィレット)は**2枚のサーフェスフィーチャー参照**(CreateReferenceFromObject、
BRep不要)を入力にとる双接フィレット。壁を4枚の個別オーバーサイズスイープとして作り、縦シームRを「隣接壁2枚の双接フィレット」として構築すれば、
エッジ参照を全廃したパイプラインが設計できる(ユーザーの7手順=壁を個別に作りトリムで繋ぐ、とも整合)。→ `test_bitangent_walls.py`で検証中。

### 3.6 【2026-08-24】BiTangent路線の検証結果 — 壁バンド成功、頂稜線Rで停止

シグネチャ(introspectionで確定): `AddNewFilletBiTangent(iElement1, iElement2, iRadius, iOrientation1, iOrientation2, iSupportsTrimMode, iRibbonRelimitationMode)`。
兄弟に `AddNewFilletTriTangent` もあり。

**成功したこと**(`test_bitangent_walls.py` / `test_bitangent_band.py`):

- **開曲線のMode=4スイープは正常動作する**(これまで閉ループでしか使っていなかった。新知見)。
- **`SetLength(2, x)`で壁を第2方向(下側)へも延長できる**。
- 壁4枚(root_left/root_right全長、guide_near/far マージン20mm付き)を個別スイープし、
  BiTangent3回(f1=A×B、f2=C×D、f3=f1×f2)で**4隅R付きの連続壁バンド**が完成。
  f3では**2つの交線に同時にリボンが生成**された(1フィーチャーで2隅を閉じられる)。
- 方向(o1,o2)は形状依存なので**毎回4通り総当たり+プローブ点判定**(残るべき側の点との距離<0.1)で選定する。
  プローブ点は**ベンドフィレット円筒領域の外**(接線点run≈R·tan(θ/2)より遠く)に置くこと。
- 面積が理論値と整合(バンド2693.84mm² vs 概算2626mm²+R補正)。ユーザー目視確認でもコーナーRは良好。
- 成果物: `tools/probe_output/bitangent_band_success.CATPart`

**未解決**(`test_bitangent_full_bead.py`):

- 頂面(`AddNewOffset(base, depth, orient=1)`、probe-and-flip)までは成功するが、
  **topR = BiT(壁バンド, 頂面) が全方向失敗**(コーナーR=2でも6でも)。
  仮説: 交線が閉ループかつ8パッチ(4壁+4コーナーリボン)をまたぐリボン生成の失敗。RibbonRelimitationMode等は未探索(1,1のみ試行)。
- **ユーザー目視観察**: 壁スイープのドラフト方向が意図とずれている(要修正)。フィレットは良好。「後はトリムさえできれば」。

**次の検討手順は `docs/bead_construction_flowchart.md` のフローチャートに集約した(Opus引継ぎ用)。**

### 3.7 【2026-08-24 解決】topR失敗とドラフト方向不正の根本原因 — どちらも「向きの決め打ち」

§3.6の未解決事項は解決した。詳細は `docs/bead_construction_flowchart.md` §6。要点のみ:

- **原因はCATIA側の制約ではなく、こちら側が向きを解析的に決め打ちしていたこと**だった。
- **ドラフト角**: `SetAngle(1, +50)` 固定は壁を基準面の**下**へ掃引していた。正しい角度は
  ガイド曲線自身の向きに依存し壁ごとに異なる(実測: A=−50, B=−50, C=−130, D=−130)。
  候補 `[−θ, −(180−θ), +(180−θ), +θ]` を総当たりし、頂部エッジの理論位置への距離で選定すること。
- **谷折り面の法線反転**: パネル1が `+n1` のときパネル2の立ち上がり向きは **`−n2`**。
  決め打ちすると壁が1枚だけ裏返り、topRのkeep判定が `keep_max = depth ちょうど` で全滅する。
  頂面オフセットに対して `±n2` を距離測定して実測で確定すること。
- **プローブ設計**: keep判定は基準面**全体に散らした多点**(16点)で行う。1〜2点だと
  角に残ったスリバーに当たって誤合格する。加えて「消えるべき点」の距離>閾値も要求する。
- 結果、ユーザー指定の7手順が**エッジ参照ゼロで完走**(`tools/probe_bead_bitangent_full.py`、
  成果物 `tools/probe_output/bead_strict2.CATPart`)。幾何検証も合格(`tools/probe_verify_bead.py`)。

**副次的なCATIA仕様**: 視点のスクリプト設定は `viewer.Viewpoint3D.PutSightDirection(tuple)` /
`.PutUpDirection(tuple)`(`PutSight` ではない)。`StartCommand("Isometric View")` は日本語版では効かない。
また真上(−z)からの投影はV字基準面の2パネルが完全に重なるため検証に使えない — 各パネルの法線方向からの正面図を使う。

### 3.2 丸めコーナー入り閉曲線のスイープが失敗する(未解決)

§2.7 で構築に成功した「丸めコーナー入り閉曲線」(長さ283.88mm、幾何学的に妥当)を、
鋭角ループと**全く同じ** `AddNewSweepLine` Mode=4 パターンでスイープすると、汎用 `E_FAIL` で失敗する
(鋭角ループは同じ手順で確実に成功する)。

**視覚確認**: スイープ前の丸めループ単体をスクリーンショットで確認したが、目視で明らかな自己交差やアーティファクトは見つからなかった
(ただしビューが縮小されすぎており、コーナー近傍の微細な欠陥は判別できていない)。

**仮説(未検証)**: 円弧(コーナー)セグメントと直線セグメントが混在する閉曲線は、`AddNewSweepLine` Mode=4 のガイド曲線として
何らかの制約(パラメータ化の連続性、または `FirstGuideSurf` との整合性)を満たしていない可能性がある。

### 3.3 マクロの手動クリック選択を汎用的に再現できるか

**ユーザーの質問**: 「マクロでは手動で要素をクリックしましたが、それを再現できないのでしょうか？」

**調査結果(§4.1)**: マクロは `CreateReferenceFromBRepName(...)` という、**特定のフィーチャーインスタンス番号**
(`GSMSweep.1`, `GSMLine.11`, `GSMFill.2`, `GSMCurvePar.1`, `GSMIntersect.1`, `GSMPlane.1` 等)を直接埋め込んだ文字列でエッジを参照していた。
これは「手動クリックで得られる、CATIA内部の永続的トポロジー名(persistent naming)」そのものであり、
**パラメトリックに繰り返し生成されるジオメトリでは、実行のたびにフィーチャー番号が変わるため、文字列をそのまま再利用することはできない**。

→ 結論として、「クリックの再現」自体は不可能ではないが実用的でない。代わりに以下のいずれかが必要:
  - (a) `Selection.Search` + 幾何学的照合(現状のアプローチ、§3.1.1の精度向上が必要)
  - (b) 生成のたびに `CreateReferenceFromBRepName` の文字列を**動的に構築**する(構築中の各フィーチャーの `.Name` を都度取得して埋め込む) — 未検証、実現可能性も未確認

---

## 4. 重要なCATIA仕様・API知見

### 4.1 ユーザー提供の成功マクロ(原文)

CATIAの「エッジをクリック→エッジフィレット→OK」を録画したマクロ(VBScript):

```vbscript
Language="VBSCRIPT"
Sub CATMain()
Set partDocument1 = CATIA.ActiveDocument
Set part1 = partDocument1.Part
Set shapeFactory1 = part1.ShapeFactory
Set constRadEdgeFillet1 = shapeFactory1.AddNewSurfaceEdgeFilletWithConstantRadius(Nothing, catTangencyFilletEdgePropagation, 5.000000)
constRadEdgeFillet1.FilletBoundaryRelimitation = catConnectFilletBoundaryRelimitation
constRadEdgeFillet1.EdgePropagation = catTangencyFilletEdgePropagation
constRadEdgeFillet1.FilletBoundaryRelimitation = catConnectFilletBoundaryRelimitation
constRadEdgeFillet1.FilletTrimSupport = catTrimFilletSupport
Set hybridBodies1 = part1.HybridBodies
Set hybridBody1 = hybridBodies1.Item("形状セット.1")
Set hybridShapes1 = hybridBody1.HybridShapes
Set hybridShapeSweepLine1 = hybridShapes1.Item("スイープ.1")
Set reference1 = part1.CreateReferenceFromBRepName("REdge:(Edge:(Face:(Brp:(GSMSweep.1;(Brp:(GSMLine.11);Brp:(GSMFill.2)));None:();Cf14:());Face:(Brp:(GSMSweep.1;(Brp:(GSMCurvePar.1;(Brp:(GSMIntersect.1;(Brp:(GSMPlane.1);Brp:(GSMFill.2)));Brp:(GSMFill.2)));Brp:(GSMFill.2)));None:();Cf14:());None:(Limits1:();Limits2:());Cf14:());WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR29)", hybridShapeSweepLine1)
constRadEdgeFillet1.AddObjectToFillet reference1
part1.Update
End Sub
```

読み取れるポイント:

- 壁は `スイープ.1`(SweepLine)フィーチャーで作られている → 本プロジェクトの `AddNewSweepLine` Mode=4 方針と一致。
- `EdgePropagation` は `catTangencyFilletEdgePropagation`(**接線伝播**)を明示的に設定している。ただしこれはCATIAダイアログの**デフォルト表示値**をユーザーがそのままOKした可能性があり、必須とは限らない(§4.4参照: 接線伝播は隣接面が接線連続でないエッジに対して即時拒否されることを確認済み)。
- `FilletBoundaryRelimitation` は `catConnectFilletBoundaryRelimitation` を2回(同じ値)設定している。
- `FilletTrimSupport` は `catTrimFilletSupport` を設定している。
- エッジ参照は `CreateReferenceFromBRepName` によるフィーチャー履歴埋め込み文字列(§3.3参照、汎用再現不可)。

### 4.2 フィレット関連の列挙定数の実際の整数値(確認済み・重要)

`win32com.client.gencache` を使わない、**安全な型ライブラリ直接読み取り**で確認(手法は§4.5参照)。
従来「0/1/2」等と推測していた値の一部が誤りだったことが判明:

| プロパティ | 定数名 | 整数値 |
|---|---|---|
| `EdgePropagation` | `catMinimalFilletEdgePropagation` | **0** |
| | `catTangencyFilletEdgePropagation` | **1** |
| `FilletBoundaryRelimitation` | `catAutomaticFilletBoundaryRelimitation` | **0** |
| | `catUVFilletBoundaryRelimitation` | **1** |
| | `catConnectFilletBoundaryRelimitation` | **2** |
| | `catMinimumFilletBoundaryRelimitation` | **3** |
| | `catMaximumFilletBoundaryRelimitation` | **4** |
| `FilletTrimSupport` | `catTrimFilletSupport` | **0** |
| | `catNoTrimFilletSupport` | **1** |

現在の `gsd_build.py` の `edge_fillet()` はこれら3プロパティを**一切設定していない**(コンストラクタの `propagation_mode` 引数のみ、デフォルト0)。
マクロの成功に必要な組み合わせかどうかは§3.1.2の理由で未確定だが、少なくとも**正しい値そのもの**は本ドキュメントで確定した。

### 4.3 `AddObjectToFillet` は不正なエッジに対して即座に失敗する

`AddNewSurfaceEdgeFilletWithConstantRadius(None, propagation, radius)` で feature を構築したあと、
`.AddObjectToFillet(edge_ref)` を呼んだ際、そのエッジが不正(例: フィレット不可能な形状・トポロジー)である場合、
**`part.Update()` を待たず、`AddObjectToFillet` の呼び出し自体が即座に例外を送出する**ことを確認した。

これは「後で`Update()`が失敗する」パターンとは異なる、新しい診断シグナルとして有用。
逆に言えば、`AddObjectToFillet` が成功したのに `Update()` で失敗する場合は、エッジ自体は妥当だが**フィレット半径や周辺形状との幾何学的な非両立性**が原因である可能性が高い、という切り分けに使える。

### 4.4 `EdgePropagation=Tangency` は接線非連続なエッジで即時拒否される(仮説)

`AddObjectToFillet` 呼び出し時に `propagation=1`(Tangency)を指定していると、CATIAは追加しようとしているエッジが
隣接する面と接線連続かどうかを**その場で検証**し、非連続であれば即座に拒否する、という挙動を観察した(§3.1.2の実験より)。
ビードの縦シームエッジは意図的に鋭角(丸めたい対象そのもの)であるため、`propagation=1` は原理的に不適切であり、
**`propagation=0`(Minimal)を使うべき**と考えられる。ただし §3.1.2 の通り、この実験自体が誤ったエッジを使っていた疑いがあり、再検証が必要。

### 4.5 CATIA列挙定数の値を安全に調べる方法(gencacheを使わない)

`win32com.client.gencache.EnsureModule`/`EnsureDispatch` は共有の `gen_py` キャッシュを汚染し、本番の `SyntheticPartBuilder` を壊すリスクがある(§2.4)。
代わりに、**対象feature自身の型情報から直接、生の `pythoncom` APIで列挙型を辿る**方法が安全:

```python
import pythoncom

# 1. 対象オブジェクト(例: フィレットfeature)からITypeInfoを取得
ti = feature_obj._oleobj_.GetTypeInfo()
attr = ti.GetTypeAttr()
nfuncs = attr[6]

# 2. 目的のプロパティ(例: "EdgePropagation")のFUNCDESCを探す
for fi in range(nfuncs):
    fd = ti.GetFuncDesc(fi)
    memid = fd[0]
    name = ti.GetNames(memid)[0]   # 注意: GetNames はメンバID一つだけを渡す(第2引数なし)
    if name != "EdgePropagation":
        continue
    typedesc = fd[8][0]            # 戻り値の型記述
    vt = typedesc[0]
    if vt == pythoncom.VT_USERDEFINED:
        href = typedesc[1]
        enum_ti = ti.GetRefTypeInfo(href)   # 列挙型自体のITypeInfoを取得

        # 3. 列挙型のVARDESCを全部舐めて名前と値を得る
        eattr = enum_ti.GetTypeAttr()
        for vi in range(eattr[7]):
            vd = enum_ti.GetVarDesc(vi)
            ename = enum_ti.GetNames(vd[0])[0]
            evalue = vd[2]          # 例: (memid, "", value, elemdescVar, wVarFlags, varkind)
            print(ename, evalue)
```

ポイント:

- `GetContainingTypeLib()` で得られる「そのインターフェースが属する型ライブラリ」を`TKIND_ENUM`について総当たりする方法は**失敗する**
  (ShapeFactoryインターフェース自身の型ライブラリには、フィレット関連の列挙型が含まれていなかった。169個の型情報を全走査したが該当なし)。
  → 列挙型は、プロパティの**戻り値の型記述(TYPEDESC)が指す先(`GetRefTypeInfo`)**を辿って初めて見つかる。これは別の(インポートされた)型ライブラリに属することがある。
- `GetVarDesc(vi)` の戻りタプルの `[2]` が実際の列挙定数値(pywin32の `PyVARDESC` オブジェクトの `.value` フィールド)。
- `ti.GetNames(memid)` は **memid 一つだけを引数に取る**(`GetNames(memid, 1)` のように第2引数を渡すと `TypeError` になる、pywin32のこのビルドでの挙動)。

再現用スクリプト: `dump_enums2.py` (session scratchpad, 本文書末尾のsource一覧参照)。

### 4.6 【公式文献】`AddObjectToFillet` は `TriDimFeatEdge` 型のみサポート

pycatia(V5R28自動生成ドキュメントのミラー)より、`ConstRadEdgeFillet.AddObjectToFillet` の公式説明:

> The sub-element to be filleted…Boundary object supported: **TriDimFeatEdge**.

つまり `AddObjectToFillet` に渡せるのは `TriDimFeatEdge` 型の境界オブジェクトのみ、という制約が明記されている。
`Selection.Search("Topology.CGMEdge,sel")` で得られる `.Reference` が常にこの型として扱われるとは限らない可能性があり、
特にTrimフィーチャーの結果に対する検索で得られる参照がこの制約に抵触していないか、要検証(§3.1.1の次のアクションと合わせて調査する)。

また `EdgePropagation` の公式説明:

> Returns or sets the edge fillet propagation mode used when computing the **edges to be filleted**.

これは「追加で指定した1本のエッジから、接線連続な隣接エッジへ自動的にフィレットを**拡張**するかどうか」を制御するものであり、
`catMinimalFilletEdgePropagation` は「最小限のエッジ数のみフィレットする」("fillets minimum number of edges")という意味。
→ これ自体は「1本の孤立した鋭角エッジをフィレットできるか」には直接関係しないはずで、§4.4の「即時拒否」仮説は
§3.1.1で判明した「そもそも間違ったエッジを掴んでいた」ことが原因だった可能性が高くなった(要再検証)。

出典: [pycatia part_interfaces docs](https://pycatia.readthedocs.io/en/0.3.6/api_part_interfaces.html)

### 4.7 【公式文献・IBM APAR】Trim由来のエッジがフィレット選択不能になる既知の不具合

IBM公式サポート(APAR = 既知不具合の追跡票)に、本調査と直接関連する複数の記録が見つかった:

- **エッジフィレットとTrimの既知の不具合**: 「Edge fillet may not be created and a feature definition error appears
  stating 'Edges or face not found, Re-select the edges or faces on current geometry'. Some edges and faces
  created by different trim operations were **badly created, making edge selection impossible**」。
  V5R17, V5R20, V5R21, V5-6R2013 で順次修正された、とされる(裏を返せば古いバージョンほど発生しやすい)。
- **HE09979「SOME OF EDGES ARE NOT SHARP」エラー**: 原因は Generic Naming (GN) がエッジを正しく命名できないこと。
  「The edge has the **same base naming as many other edges**, and the adjacent faces (used to discriminate
  the edges) also have the **same base naming**. In these cases, GN was not able to function correctly.」
  V5-6R2013 GAで修正。
- **HD69551「INTERSECTION EDGES ACTIVATION OPTION DOES NOT WORK」**: Trim/交差由来のエッジ選択に関するUI上の既知の癖(V5R17→V5R18でモード仕様が変更された)。

これらは全て「**Trim操作の結果として生成されたエッジは、通常のフィーチャーのエッジよりも選択・命名が不安定になりやすい**」
という一貫したパターンを示している。本プロジェクトの `ref_wall = AddNewHybridTrim(surface_ref, 1, sweep_ref, 1)` は
まさにこのパターンに該当する。

出典: IBM Support APAR検索結果(HD69551, HE09979, および関連するTrim由来エッジフィレット不具合の要約)。個別ページは認証/403で直接閲覧不可のため、検索エンジンのサマリーを引用。

### 4.8 【公式文献・最重要】CATIA Generic Naming の対称性ambiguity — 公式の回避策

Generic Naming(安定名前付け)の解説文書、および関連するAPAR/フォーラムから:

> Generic naming ambiguity happens because subelements **share the same topological neighbors**.
> This is particularly relevant to **symmetric geometry** scenarios where corresponding elements
> may have identical topological characteristics.

**公式に推奨されている2つの回避策**:

1. 曖昧になっている頂点/エッジの位置に「On curve」型の点を作成し、その**点を経由して**間接的に参照する。
2. **Breakコマンドで形状を別パーツ(別フィーチャー)に分割し、対称性を崩す**ことでGeneric Naming の曖昧さを解消する。

→ 本プロジェクトの壁は、4隅が回転対称に近い**1本の対称なスイープ**(`AddNewSweepLine` on a symmetric closed loop)であり、
これはGeneric Naming ambiguity が起きる典型条件そのものである。回避策2は、ユーザーが元々指定した7手順(壁を個別パネルとして作りトリムで結合する)と本質的に一致する —
**対称な1本のスイープではなく、非対称な複数フィーチャーの組み合わせとして壁を構築することが、根本的な回避策になる可能性が高い**。

詳細な検証手順は §3.1.3 / §3.4 参照。

出典: [Generic Naming Overview](https://www.maruf.ca/files/caadoc/CAAMmrTechArticles/CAAMmrGenericNaming.htm)、関連フォーラム・APARのまとめ。

### 4.9 これまでに確立していた既存の重要ルール(再掲・本セッションで再確認)

以下は前セッションまでに確立し、今回のセッションでも(一部再発を経て)再確認された事項:

- **`find_edge_near`の落とし穴**: 共有頂点での探索は複数エッジがヒットし曖昧。**エッジの中点**で探索する必要がある(`Selection.Search("Topology.CGMEdge,sel")` + 各アイテムの中点距離判定)。
  → 本セッションで一度これを怠り(頂点=交点での探索に戻ってしまい)、誤ったエッジを拾って混乱した(§3.1.1)。
- **有効な `Selection.Search` フィルタ文字列**: `"Topology.CGMEdge,sel"` と `"Topology.CGMFace,sel"` のみ確認済み。
  `"Edge,sel"`, `"CATGSMEdge,sel"`, `"Topology.CATIEdge,sel"`, `"Face,sel"`, `"Topology.Face,sel"`, `"Body,sel"`, `"Cell,sel"` は全て検索構文エラーで失敗。
- **`AddNewHybridTrim(iElement1, iOrientation1, iElement2, iOrientation2)`**: 正しい `(o1, o2)` の組み合わせは**グローバルに一定ではない**。呼び出しごとに4通り全てを試し、
  期待する参照点が距離≈0で残るかを確認して決定する必要がある。
- **`AddNewCorner(...)`**: 入力カーブは真のコーナーを**越えてオーバーサイズ**である必要がある(でないと構築に失敗する)。
  4通りの向き(o1,o2)の中で最短長のものが「単独オブジェクトとして使う」場合は正解のことが多いが、**既存の閉曲線にトリムで挿入する場合は最短だけでなく全4通りを試す必要がある**(§2.7)。
- **`AddNewHybridSplit(curve_ref, cut_point_ref, orientation)` + `.AddCuttingElem(cut_point2_ref, orientation2)`**: 2点でのカットは**1フィーチャーにまとめる**必要がある。
  `AddNewHybridSplit` を2回連続で呼ぶと、以降の呼び出しが不安定になる。
- **失敗フィーチャーのクリーンアップ**: `part.Update()` が失敗すると、その失敗フィーチャーがツリーに残存し、以降の全ての `part.Update()` を巻き添えで失敗させる。
  `Selection.Add(feature); Selection.Delete(); part.Update()` で明示的に削除する(`cleanup_failed` ヘルパーとして各スクリプトで再利用)。
- **ドキュメント衛生**: テストスクリプトは必ず `doc.Close()` を呼ぶこと。本セッション中、これを怠ったテストの積み重ねでCATIA上に44個のドキュメントが開いたままになり、ユーザーの指摘で一括クローズした。

---

## 5. ビード形状完成までの残タスクと優先順位(2026-08-23整理)

完成に必要な工程を洗い出すと、以下の依存関係になる:

```
① 基準面(ベンドフィレット済み)         [解決済み §2]
② 鋭角壁(SweepLine Mode=4)            [解決済み §2.5]
③ ①②のトリム統合                      [解決済み §2.6]
④ 壁の縦シームコーナーのR処理            [未解決 — 経路A/Bどちらか一方が通ればよい]
   経路A: ③の後に縦シームをエッジフィレット      [§3.1 未解決、Generic Naming仮説あり §3.1.3]
   経路B: ②の前に丸めコーナー入りループをスイープ  [§3.2 未解決、スイープ自体が失敗]
⑤ ④の結果に対し、底面/壁の稜線フィレット(手順⑥)  [未着手、単一パネルでは実績あり(test_full_bead_v4/v5)]
⑥ ④の結果に対し、壁/基準面の稜線フィレット(手順⑦) [未着手、同上]
⑦ bead.py/gsd_build.pyへの統合、既存ガードとの整合、テスト更新 [未着手]
```

**現状のボトルネックは④のみ**。④さえ解決すれば、⑤⑥は単一パネルケースで実績のある技法(`test_full_bead_v4.py`/`v5.py`)をベンド跨ぎケースに適用するだけで、大きな障害はない見込み(未検証ではあるが、④より難度は低いと予想)。

### 5.1 経路A: 【結論・確定】Generic Naming ambiguityが原因、`Selection.Search`方式は使えない

§3.4の手動検証(成功)+ 全24エッジ総当たり(24/24失敗)により確定した。**経路A(現状の対称な1本スイープ+`Selection.Search`によるエッジ参照取得)は、
プロパティ値やエッジ選択をどう工夫しても解決しない**。次に進むべきは §5.3(壁の非対称構築)。

### 5.2 経路Bの調査(優先度: 中、経路Aとは独立に価値がある保険)

丸めコーナー入り閉曲線を先に作ってからスイープする方式(§2.7で閉曲線構築自体は解決済み、スイープが未解決)。
経路Aが解決不能と確定した今、こちらも並行して詰める価値が上がった(フィレットという操作自体を避けられるため、Generic Naming問題と無関係に成立しうる)。

1. §3.2: 丸めループのスイープ失敗原因調査。コーナー近傍のクローズアップ視覚確認(現在のスクリーンショットは縮小されすぎている)。
2. 代替仮説: `AddNewSweepLine` Mode=4以外のスイープモード(Mode=0/1/2、`tools/probe_bead_sweep_*.py`で過去に試した形跡あり)で丸めループが通るか確認。

### 5.3 【本命】壁の非対称構築(次の実装ステップ)

ユーザー元々の7手順に立ち返り、壁を **1本の対称なスイープではなく、4枚の個別パネル(または非対称な構築順序)** として作り、
トリムで結合する方式に変更する。設計方針(検討中、実装前にユーザーと合意する):

- 壁の4区間(left_mid上/guide_near上/right_mid上/guide_far上)を、**それぞれ独立した `AddNewSweepLine` フィーチャー4本**として作る(現状は1本の閉曲線を1回のスイープで作っている)。
- 4本を順にTrimで結合していく(§2.6で確立した`AddNewHybridTrim`の全方向探索パターンを流用)。
- 各セグメントが独立フィーチャーになるため、縦シームエッジは「隣接する2つの異なるフィーチャーの境界」として生まれ、対称性由来のGeneric Naming衝突が起きにくくなる(公式ワークアラウンド§4.8と同じ発想)。
- 4本のスイープを個別に作る場合、`test_sharp_base.py`で実績のある「単純な単一パネルスイープ→直接edge_fillet」パターンに近づくため、経路Aで確立済みの知見(プロパティ値等)がそのまま活きる可能性が高い。

**未確認点**: 4本の個別スイープを先にTrimで結合してから縦シームフィレットするのか(現状の失敗パターンに戻ってしまう恐れ)、
それとも各セグメントのスイープ単体の状態でまず縦シームをフィレットしてからTrim結合するのか(§3.4の知見=生スイープ状態でのフィレットが必要、と整合)。
後者を優先して試すべき。

### 5.4 統合作業(④解決後)

④が解決し次第、`bead.py`/`gsd_build.py` へのトリム/スイープベース構築方式の統合、既存の`MAX_MITER_AMPLIFICATION`/`run_out_diagonal_mm`ガードとの整合性確認、`tests/test_bead.py`の更新、roadmap本体の更新。

## 6. 参照スクリプト(session scratchpad, 再現用)

パスは全て `C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\` 配下(セッション終了で消える一時ディレクトリのため、恒久保存が必要なものは `tools/` への移動を検討):

- `test_sharp_base.py` — 鋭角ベースでのフィレット成功を確認した最初のテスト
- `test_trim_sequence.py` / `test_clean_sequence.py` — ユーザー指定の正しい手順(フィレット→トリム統合→縦フィレット)の再現
- `test_rounded_loop.py` — コーナー丸めループの初期(失敗した)再構築試行
- `test_trim_corners_into_loop.py` — コーナー丸めループ構築の最終版(§2.7で解決)、丸めループのスイープ失敗(§3.2で未解決)を含む
- `dump_enums.py` / `dump_enums2.py` — CATIA列挙定数の安全な調査スクリプト(§4.5)
- `test_macro_props_bend.py` / `test_macro_props_bend2.py` — マクロのプロパティ組み合わせ・エッジ検出方式の検証(§3.1)

**恒久保存済み(`tools/`配下、セッション終了後もアクセス可能)**:

- `tools/probe_export_repro_case.py` — §3.4の手動検証用CATPart(`tools/probe_output/repro_case_seam_fillet.CATPart`)を生成するスクリプト
- `tools/probe_fillet_before_trim.py` — §3.4/§5.1で確定した「生スイープの全24エッジ総当たりで24/24失敗」を再現する決定的な検証スクリプト
- `tools/probe_dump_fillet_enums.py` — §4.2の列挙定数の実際の値をgencache不使用で安全に取得するスクリプト

---

## 7. 本番統合(2026-08-24)と、そこで露呈した基準面の設計問題

BiTangent方式のスパイク(§6/`docs/bead_construction_flowchart.md`)を `bead.py`/`gsd_build.py` の
本番経路へ統合した。その過程で**ビードそのものではなく基準面の作り方**に構造的な問題が
見つかったので、経緯と数値を残す。

### 7.1 統合内容

- 旧セル分解方式(`_apply_bead`、パネルを5ストリップに割ってシャープjoin→エッジフィレット)を
  **削除**し、「完成した基準面の上にビードを載せる後処理」方式に置き換えた。
  純粋幾何は `bead.plan_bead_on_surface`、CATIA呼び出しは `gsd_build._add_bead_to_surface`。
- マイターオフセット式を使わなくなったため、`MAX_MITER_AMPLIFICATION` による発散ガード
  (§2.1)は原理的に不要になった。
- ユーザー確定のパラメータ変更(2026-08-24):
  - 全フィレットの最小R: **R4 → R5**(`MIN_NEUTRAL_PLANE_RADIUS_MM`)
  - 基準面(メイン曲げ)のRは慣例で **R10以上**(`MIN_BASE_BEND_RADIUS_MM`、新設)
  - 板幅 **25〜50mm**(締結点1つの最小必要平面が直径25mmのため。bearing radius 12.5〜25mm)
  - ビード壁のドラフト角 **45〜70度**(垂直90度は成形上まず不可)
  - ビードの稜線R・コーナーRを別パラメータ化し、いずれもR5以上でランダム性を持たせた

### 7.2 実機で判明した不具合と修正(いずれも自分側のバグ)

1. **プローブ点が曲げフィレットの円筒面上に乗っていた** — 平面前提で計算した座標が実際の面から
   1.3〜2.3mmずれ、頂面オフセットの向き判定が全滅していた(25件中21件)。
   各パネルの(近端,遠端)でフィレットが食う長さ(接線長)を呼び出し側から渡し、
   その外側にしかプローブを置かないよう修正 → **オフセット失敗 21件 → 0件**。
   ランプは2つの曲げフィレットに挟まれて平坦部が残らないことがあるため、
   平坦区間が取れないパネルはプローブ自体を置かない。
2. **壁の掃引でCATIAの例外が生のまま漏れていた** — 角度探索(等倍)は通るのに、
   頂面/基準面へ貫通させる延長(1.6倍+3mm)で`Update`が落ちるケース。
   本プロジェクトの方針(CATIA側の失敗はInfeasibleに変換)に反していたので修正し、
   併せて延長量を大きい順に試す適応方式にした。

### 7.3 【未解決・要設計判断】基準面の「カニ歩き」問題

統合後、ビードが載る前提の spec に絞っても成功率が上がらない。根本原因はビードではなく
**基準面の作り方**にある。

`two_point_frame` は「両パネルが直線の折れ目を持てること」から幅方向 `w = n1 × n2` を
一意に決め、各パネルを**自分の締結点を中心に**配置する。したがって2枚の端パネルの
幅方向のズレは `(p2 - p1)` の `w` 成分そのものになる。締結点2の位置は3Dランダム方向・
60〜200mmで独立にサンプリングされるため、この成分は板幅より大きくなるのが普通:

| 指標(3000サンプル実測) | 中央値 | 範囲 |
|---|---|---|
| 締結点の幅方向ズレ | **58.7mm** | 0〜197.8 |
| 板幅 | 42.6mm | 25.2〜50.0 |
| ランプの幅方向への傾き | 37.3° | 0〜89.4° |
| 2枚の端パネルの帯が重なる割合 | **36.4%** | — |

結果、板は横に「カニ歩き」して繋がる形になる。単一の幅座標を走る直線ビードは板から外れるため、
ビードが幾何学的に成立するのは約16%に留まる。**これはビードだけの問題ではなく、
幅42mmの板が37°(最大89°)横に寝たランプで繋がる形状が学習データとして妥当かという
メイン形状側の問題**でもある。

### 7.4 ユーザー提案の是正案(2026-08-24、次の実装対象)

「各締結点を通り法線に垂直な平面(座面)を2枚作り、その**交線 L で折れる V 字面**を作る。
その面上で締結点から締結点への**最短経路(測地線)**を引き、それを中心に一定幅で切り取って
フィレットすれば基準面になる。その最短経路はそのままビードの中心線にもなる。」

幾何学的な裏付け:

- 2枚の座面の交線 L は自動的に `n1 × n2` に平行 = 従来の `w` と同じ折れ目方向になる(整合)。
- V字面は可展面なので、L まわりに展開すると**平面になり、測地線は直線**になる。
  p1 と p2 は展開後 L の反対側に来るので、直線は必ず L を横切る(常に解が存在する)。
- 展開した直線が L と成す角は L の両側で等しいため、**一定幅の帯は折れ目で幅が連続する**。
  → 従来のような横ズレ(カニ歩き)が**原理的に発生しない**。
- 折れ角は2平面の成す角として一意に決まる(=従来のように fold1/fold2 の走行距離を
  独立にサンプリングする自由度は無くなる)。形状の多様性は板幅・各R・ビード寸法・
  締結点/法線のサンプリング自体から得ることになる。

**併せて必要**(ユーザー指摘): 交線 L が両点から遠いと帯が極端に長くなるため、
成立しやすいように締結点2の方向・法線相対角の分布に偏りを入れる操作が要る。


---

## 8. 中間折れ+測地線バンド方式(2026-08-24、ユーザー指示①②の検討結果)

§7.4 の「座面2枚のV字面+測地線」案に対し、ユーザーから以下の確定指示があった。

> ①平行系にかかわらず、交線が締結点から一定距離以上遠く、長手方向が長すぎる場合には**中間折れを追加**する仕様にすべき。
> その際、締結点の最小必要平面のクリアランスと、基準面フィレットのサイズなどを加味して中間折れ位置を決定すべき。
> 中間折れ位置とベクトルは若干のランダム性を持たせて多様性を持たせる。**なので基準面フィレットの半径は先にランダム生成する必要がある**。
> ②上記ロジックを用いれば、形状多様性を確保することは可能。
> また、上記ロジックを実現するために、締結点同士はある程度距離を離す必要がある。

本節はこの指示を幾何学的に定式化し、数値で裏を取った結果を残す。

### 8.1 定式化 — 全ては「展開面上の2D問題」に落ちる

`two_point_frame` の不変条件(全ての折れ目が `w = n1 x n2` に平行)を保つ限り、基準面は
**「w直交断面上のプロファイル折れ線」を w 方向に押し出した可展面**になる。したがって次の
展開写像が定義できる:

```
phi(s, t) = profile3d(s) + t * w
    s = 断面プロファイル(フィレット込み)に沿った弧長
    t = w 方向の座標
```

`tools/probe_unfold_map.py` で3000ケースを検証し、以下を数値的に確認した:

| 検証項目 | 最大誤差 |
|---|---|
| 閉合 `phi(0, t1) == p1` | 0.0 mm |
| 閉合 `phi(s2, t2) == p2` | 1.1e-13 mm |
| パネル1の面法線 == n1 | 3.3e-16 |
| パネル2の面法線 == n2 | 8.9e-16 |
| **第一基本形式の単位行列からのずれ**(=等長性) | 2.8e-08 |
| 平坦区間での帯幅誤差(公称40mm) | 2.8e-14 mm |

**第一基本形式が単位行列** = `phi` は厳密な等長写像。よって展開面 (s,t) に描いた図形は
基準面上で「板を曲げただけ」の合同像になる。ここから直ちに:

- **カニ歩きは原理的に消える。** 帯を「展開面上の直線 `(0,t1) -> (s2,t2)` を中心とする
  一定幅の帯」として引けば、展開幅は厳密に一定で、両端はちょうど締結点に着地する。
  §7.3 の `bead_half_width_mm`(中央値が負になっていた)という概念自体が不要になる。
- 折れ目をまたぐ2点の**直線弦**が縮むのは板金として正常な現象であり欠陥ではない
  (最初の検証でこれを幅のばらつきと誤判定した。測るべきは弦ではなく計量テンソル)。
- **測地線 = 展開面上の直線 = ビード中心線**。ユーザーの当初主張どおり、基準面の中心線が
  そのままビード中心線になる。

### 8.2 単曲げ / 中間折れ の統一

断面プロファイルは `q1 -> (A) -> (B) -> q2`(`q` は w直交断面への射影)。

- **単曲げ(§7.4のV字面)**: A=B= 座面2枚の交線 L。**自由度ゼロ**。n1とn2が平行に近いと
  L が遠のき、プロファイル長 S が発散する(実測 max 6491mm、p90 404mm)。これが §7.4 の
  棄却サンプリング受理率25.4%の主因だった。
- **中間折れ1枚**: A, B を独立に選べる(自由度2)。プロファイル長を**自由に選べる**ので
  常に短く作れる。**これは既存の flat1-ランプ-flat2 構成そのもの**であり、`gsd_build.py` の
  構築経路は変更不要。新しいのは「帯の引き方」だけ。
- **法線が平行なケース(parallel_same/opposite)は L が存在しない** = 中間折れが必然。
  ユーザー①「平行系にかかわらず」はここで自動的に満たされる。実測でも成立後の
  クラス分布に parallel_same_offset 約20%、parallel_opposite 約2%が含まれる。

選択規則: **展開全長 D = hypot(s2, dw) が閾値以下なら単曲げ、超えたら中間折れを1枚入れる。**
(既存の `SINGLE_FOLD_MIN_EXCESS_RAD` による「無駄な曲げ量」ヒューリスティックは、
この長さ基準に置き換えられる。長さ基準の方が §7.3 の失敗モードを直接見ている。)

### 8.3 中間折れ位置の決め方 — Rを先にサンプリングする必要性(ユーザー①の根拠)

中間折れ位置は `a = bearing_radius + T + slack`(T = 曲げフィレットの接線長)で決まる。
ところが `T = R * tan(theta/2)` の `theta`(曲げ角)は A, B の位置が決まって初めて定まるため、
**a と T は相互依存する**。これを不動点反復で解く(実測: 8反復以内、通常3〜4反復で1e-4収束)。

この反復を回すには `R` が既知でなければならない。**したがってユーザー①の指摘どおり、
基準面フィレット半径 R を先にランダム生成する必要がある**(現行 `general_two_point.sample()` は
逆順 — fold_run を先に振ってから R の実行可能上限を逆算している。ここが反転する)。

多様性は `slack1, slack2` の実行可能領域から取る。実測での領域の広さ(12x12グリッド上):
中央値 16〜24%、p90 約73% — ユーザー②「上記ロジックを用いれば形状多様性を確保可能」を裏付ける。
狭いケース(p10 = 2.1%)もあるので、領域を求めてからサンプリングする既定方針(実行可能範囲を
先に計算 → その中で振る)をそのまま適用する。

### 8.4 数値結果(`tools/probe_intermediate_fold_v2.py`、各n=2000)

**【重要】初版の数値は誤ったモデルで測っていた(2026-08-24 修正)。**
断面プロファイルを 3D のまま `B - A`(`A = p1 + a*u1`, `B = p2 - b*u2`)で作っていたが、
`p1` と `p2` は w 座標が `dw` だけ違うため `B - A` に w 成分が乗り、**ランプ面が w に平行に
ならない**。つまり「w平行版」を測ったつもりで別物を測っていた。正しくは**先に w直交断面へ
射影してから**プロファイルを作る(`q1 = c(p1)`, `q2 = c(p2)`, `A = q1 + a*u1`, `B = q2 - b*u2`)。
発見の経緯: §8.8 のホモトピー初期値変換で `rot(n1,w,phi)` は誤差1.6e-16で一致するのに
`rot(u1s,w,phi)` だけ 0.57 ずれる、という非対称な症状から `d ⊥ w` の不成立に行き着いた。
なお `probe_unfold_map.py`(§8.1)と `probe_perpendicular_strip_theorem.py`(§8.7)は
正しく射影しており**無傷**。以下は全て修正後の数値。

総合成立率(単曲げで足りる分 + 中間折れで成立する分)、曲げ角上限135度:

| 断面内距離の下限 | 単曲げで足りた | 中間折れで成立 | **総合** | 主な不成立理由 |
|---|---|---|---|---|
| なし | 22.8% | 47.2% | **70.0%** | 曲げ角超過 581 / 干渉 17 |
| >=40mm | 22.9% | 49.0% | **72.0%** | 曲げ角超過 556 |
| >=60mm | 26.8% | 56.5% | **83.3%** | 曲げ角超過 333 |
| >=80mm | 27.4% | 65.1% | **92.5%** | 曲げ角超過 149 |

§7.4 の棄却サンプリング(2枚V字面のみ)が **25.4%** だったのに対し、中間折れを入れると
**下限なしで70.0%**。

曲げ角上限の感度(修正後モデル、断面内距離の下限なし、n=1500):

| 曲げ角上限 | 総合 | 単曲げ | 中間折れ |
|---|---|---|---|
| 110度 | 60.3% | 23.3% | 37.0% |
| 120度 | 66.1% | 22.3% | 43.8% |
| **135度** | **70.7%** | 22.8% | 47.9% |
| 150度 | 70.0% | 22.1% | 47.9% |

修正前モデルと同じく **135度が頭打ち点**(150度にしても改善しない)。ユーザー確定値135度は
修正後モデルでも妥当。

成立した形状の性質(曲げ角上限135度、断面内距離の下限なし):

| 指標 | p10 | 中央値 | p90 | max |
|---|---|---|---|---|
| 部品全長 D(展開) | 121.8 | 200.7 | 266.0 | 300.0 mm |
| **帯の傾き gamma**(折れ目に対する帯の斜度) | 2.8 | **16.0** | 39.4 | 70.6 deg |
| 曲げ角 | 25.5 | 84.6 | 126.0 | 157.5 deg |
| 中間折れの実行可能領域の広さ(12x12グリッド比) | 1.4 | 19.4 | 81.2 | 100 % |

クラス分布(下限なし): oblique 65.3% / parallel_same_offset 20.0% / orthogonal 12.3% /
parallel_opposite 2.3% / coplanar_flat 0.1%。**平行系が約22%含まれる** = 中間折れ方式は
`parallel_same_offset` 専用経路を原理的に包含する。

**既知の穴**: 曲げ角 max が上限135度を超えて157.5度になっているのは、**単曲げ(交線L)経路に
そもそも曲げ角チェックが無い**ため(`single_fold()` は bearing 侵食しか見ていない)。
実装時に単曲げ側にも同じ上限を入れる。

### 8.5 ユーザー指摘「締結点同士はある程度距離を離す必要」の定量化

**効くのは3D距離ではなく、w直交断面内での距離**(`|p2-p1|` から w成分を除いた長さ)。
w方向のズレ `dw` をいくら大きくしても折れ目を配置する余地は増えないため。

ただし実測では、**下限を課さなくても既に72.7%(120度)/80.7%(135度)成立する**。
下限80mmまで上げれば91.8%まで上がるが、その代償として生の締結点分布の約40%を捨てる
(断面内距離の実測分布: p10=48.4, 中央値=98.3, p90=165.3mm)。既存の skip-and-retry
(1部品あたり最大50回)は26%程度の失敗を十分吸収できるので、**分布の広さを優先して
下限は課さない(または40mm程度の弱い下限に留める)** のが妥当と考える。最終判断はユーザー。

### 8.6 ビードへの影響 — 中心線が折れ目に対して斜めになる

帯が斜行するため(gamma 中央値17.5度、p90 31.9度、max 67.5度)、**ビード中心線は折れ目を
斜めに横切る**。現行 `bead.plan_bead_on_surface` は中心線を「単一の幅座標の平面 x 基準面」
として切り出しており(`center_offset_mm`/`plane_point`)、これは gamma=0 のときしか正しくない。
中心線を `phi` から3Dポリライン/スプラインとして明示的に構築する必要がある。

なお §2.5 で解決した `AddNewSweepLine` Mode=4 による壁の掃引は、曲げをまたぐ掃引そのものは
既に扱えているので、斜行が追加で壊すかは実機確認が要る(§3.1 の未解決な頂稜線フィレットとは
独立の論点)。

### 8.7 【重要・確定】gamma=0(帯を折れ目に垂直にする)は原理的に不可能

§8.6 の初稿で「折れ目方向を自由にすれば自由度6・拘束5となり gamma=0 にできる可能性が高い」と
**未検証の見込み**として書いたが、これは**誤り**である。以下の定理が成立する。

> **定理**: 中心線が全ての折れ目に垂直な可展帯は、折れ目の枚数・方向によらず、
> 必ず「全折れ目が同一方向 g に平行」かつ `g ∥ n_start x n_end` かつ
> `dot(p_end - p_start, g) = 0` を満たす。

証明: 折れ目軸 `g1 = n1 x u1` まわりの回転 `R1` は `g1` を固定する。次のパネルでの折れ目軸は
`g2 = n_m x u_m = R1 n1 x R1 u1 = R1(n1 x u1) = R1 g1 = g1`。帰納的に全ての折れ目軸が一致する。
全ての法線・全ての中心線方向は `g1` に垂直なので、中心線は `g1` に垂直な平面内に収まる。∎

`tools/probe_perpendicular_strip_theorem.py` で折れ目1/2/3/5枚・11,991ケースを順方向に生成して確認
(`|g_k x g_1|` 最大 5.1e-16、`|w x g_1|` 最大 7.6e-14、`|dot(p2-p1, w)|` 最大 8.1e-12 mm)。

**帰結**: 締結点ペアの幅方向ズレ `dw = dot(p2-p1, n1 x n2)` は入力で決まる量であり
(実測 中央値 57.5mm)、これが 0 でない限り gamma=0 の基準面は**存在しない**。
折れ目方向を自由にしても、折れ数を増やしても不可能。**帯が折れ目を斜めに横切ること
(gamma 中央値17.5deg、p90 31.9deg)は、この問題設定に内在する性質**として受け入れるしかない。

したがって「折れ目方向を自由にする」ことの利点は gamma の削減ではなく、
**形状多様性が2パラメータ族から3パラメータ族に増えること**だけになる。その代償は
「全ての折れ目が w に平行」という `two_point_frame` 以来の不変条件が崩れ、`gsd_build.py`/
`bead.py` の広範囲に波及すること。多様性は既に slack1/slack2 の実行可能領域
(§8.3、グリッドの中央値16〜24%)+ R + bearing半径 + ビード寸法から取れているため、
**w平行版で進めるのが妥当**と考える。

### 8.8 折れ目方向を自由にする版(ユーザー確定、2026-08-24)

§8.7 で gamma=0 が不可能と確定した後も、ユーザーは**折れ目方向を自由にする**方針を選択した
(利点は gamma の削減ではなく、形状多様性が2パラメータ族→3パラメータ族に増えること)。

#### パラメータ化

尖り角モデルで扱う(フィレットは接線位置を動かさないので閉合式は同じ)。

```
p1 で法線 n1、中心線方向 u1 = cos(psi)*e_a + sin(psi)*e_b   (e_a,e_b は P1 の正規直交基底)
L1 進んで折れ目1。折れ目軸 g1 = cos(a1)*v1 + sin(a1)*u1     (v1 = n1 x u1、a1=0 が中心線に垂直)
g1 まわりに phi1 回転 -> 中間パネル。L2 進んで折れ目2、同様に g2/a2/phi2。L3 進んで終点 E。
拘束: E == p2 (3本) + n3 == ±n2 (2本) = 5本
未知数: psi, L1, a1, phi1, L2, a2, phi2, L3 = 8個  ->  3パラメータ族
```

#### 変数の取り方(ユーザー①に対応)

素朴に `(psi, a1, a2)` をランダムに振って `L` を解くと**成立5.0%・帯全長 中央値486mm・
p90 1007mm**で実用外だった(`tools/probe_free_fold_direction.py`)。`psi`(帯が p1 を出る方向)を
勝手に決めると閉合のしわ寄せが全部 `L` に行くため。

正しい取り方は、ユーザー①「中間折れ位置は締結点の最小必要平面のクリアランスと基準面
フィレットのサイズを加味して決定」に素直に従うこと:

- **固定する**: `L1`, `L3`(= bearing半径 + 接線長 + ランダムslack)、`a1`(折れ目1の傾き = ランダム性の源)
- **解く**: `psi`, `phi1`, `L2`, `a2`, `phi2` (5未知数 = 5拘束、ニュートン法+数値ヤコビアン)

#### 初期値: w平行解からのホモトピー継続

**w平行解(§8.2の閉形式)は自由折れ目族の厳密な一員**である。実測で変換残差 **9.2e-14**
(`tools/probe_free_fold_homotopy.py`)。したがって w平行解を初期値に置き、`a1` を目標値まで
3段階で動かしながら追いかける(ホモトピー継続)ことで安定に解ける。

変換時の注意(実装時に踏んだ2つの罠):
1. 回転角は**回転軸に垂直な成分**で測る。`u1s = cos(gamma)*u1 + sin(gamma)*w` は w 成分を
   持つので `signed_angle_about(u1s, u_m, w)` は誤り。w に垂直な `u1 -> d` で測った値を使う。
2. 折れ目軸 `g` と `-g` は同一直線なので、`phi` の符号と対で2通りの表現がある。法線の
   写り方(`rot(n_in, g, phi) == n_out`)が一致する側を選び、`|alpha|` が小さい方を採る。

#### 数値結果(各n=120)

| a1 の摂動幅 | 収束率 | 実際に動かせた傾き量 | 帯全長 | 曲げ角 | 折れ目交差角 |
|---|---|---|---|---|---|
| ±20度 | **75.8%** | med 8.8 / p90 17.3 deg | med 205.9 / p90 274.2 mm | med 63.3 / p90 120.3 deg | med 12.7 / p90 35.5 / max 58.3 deg |
| ±35度 | **67.5%** | med 15.0 / p90 30.8 deg | med 205.8 / p90 274.6 mm | med 64.7 / p90 117.6 deg | med 14.8 / p90 40.2 / max 53.7 deg |

初回測定では折れ目交差角が max 1520度/3171度になった。原因は(a)ソルバが `a2` を 2pi の
整数倍だけ流すこと、(b)折れ目軸の向き `g`/`-g` の同一性を扱っていないこと。
`(-90, 90]` への畳み込みと、**交差角の上限60度**(90度で折れ目が帯の軸と平行になりパネルが
縮退する。半幅 hw に対し折れ目が s 方向に食う長さは `hw*tan|alpha|`)を入れて解決。

#### 下流への波及範囲(実コード確認済み)

- **`gsd_build.py` の組み立て層は変更不要**。`_assemble_general_two_point` は
  `panel_corner_sets`(4点平面パッチのリスト)と `fillet_groups`((折れ目中点, R)の組)しか見ておらず、
  `rect_fill` は「4点 -> 4線 -> Fill」の汎用実装。折れ目が非平行になるとパネルは矩形から
  **平面台形**に変わるが、データ構造も CATIA 呼び出しも同じまま通る。
- **改修が要る箇所**:
  - `classify.py`: `two_point_frame` / `end_panel_corners` / `single_fold_layout`(w平行前提)
  - `templates/general_two_point.py`: サンプリング順の反転(R先出し) + 閉合ソルバ
  - `bead.py` `plan_bead_on_surface`: 中心線を「単一幅座標の平面 x 基準面」から
    展開写像由来の3Dポリラインへ(§8.6)

### 8.9 実装完了と実機検証(2026-08-24) — 基準面は成功、ビード中心線は未解決

SS8.8までの設計に基づき、`classify.py`(自由折れ目チェーン幾何一式)・
`templates/general_two_point.py`(サンプラーのR先出し化)・`gsd_build.py`
(`build_general_two_point`の再構築)・`bead.py`(`plan_bead_on_surface`の
簡素化)を実装した。テストは新規追加分含め83件全て成功。

#### 成功: 基準面(ビード無し)の実機構築

傾いた折れ目(台形パネル)でも、既存の`rect_fill`→`join`→`edge_fillet_group`が
そのまま通ることを実機で確認した(`tools/probe_free_fold_chain_real_build.py`)。
複数の締結点ペア・複数の曲げ角度(37度〜80度)で再現。この経路は本番投入可能と判断する。

#### 解決: ビード中心線は「解析標本点 -> スプライン -> 投影」で作る

当初SS8.6で設計した「パネルごとにv法線平面と基準面の交線を取り、3本をJoinする」方式は
実機で失敗した。**根本原因を特定した(2026-08-24)**:

- 各交線は単独では健全な単一曲線である(`GetMeasurable().Length`が測定でき、
  多枝ではない。`AddNewNear`で枝を絞る必要も無かった)。
- しかし**平面は無限に広がるため、1枚の平面と基準面の交線が自分のパネルだけでなく
  部品全長を貫く**。実測で3本とも 167.72 / 167.98 / 168.03 mm = 部品の全長そのもの。
  つまりほぼ重なった3本ができ、その重複のためJoinが失敗していた。
- 4点(p1/fold1/fold2/p2)への最小二乗平面で代用する案も評価したが、平面からのずれが
  ビードの横方向余裕内に収まるのは800サンプル中**55%**に留まり不十分
  (ずれ: 中央値1.15mm / p90 5.24mm / max 30.9mm、余裕: 中央値1.85mm / p90 6.82mm)。

**確定方式**: 中心線が乗るべき位置(各パネルのv=0のu軸)は解析的に厳密に分かっているので、
曲げフィレット領域を避けた平坦区間から標本点を採り(`plan.centreline_points`、
1パネル5点)、CATIA側で`AddNewSpline`で結んでから`AddNewProject(Normal=True)`で
基準面へ落とす。フィレット領域はスプラインの補間+投影に任せる。交線もJoinも使わない。
実機検証(`tools/probe_curvepar_isolate.py`)で、後続の`AddNewCurvePar`が
オフセット2/5/10mm x 向き2通り x Euclidean2通りの**24通り全てで成功**することを確認した。

注意: `HybridShapeProject`の投影方向指定は`SetNormalMode()`ではなく`Normal`プロパティ。

**中心線が折れ目を斜めに横切ることの影響**: 接線長T=R*tan(phi/2)は折れ目に垂直に測る量
だが、中心線は折れ目を傾きaで横切るので**走行方向には T/cos(a) 消費する**。プローブ点と
中心線標本点をこの外側にしか置かないよう、`fold_tangents`にはT/cos(a)を渡す。

#### 実機で発覚した既存バグ2件(SS7以前から潜在、2026-08-24修正)

1. **端の幅ガイドが板幅を超えていた**: 幅ガイドは「コーナーRで削られる前提で
   オーバーサイズにする」設計で `half_footprint + 20mm` としていたが、板幅の制約が
   無かった。実測で half_footprint 12.5 + 20 = 32.5mm に対し板の半幅は25mmしかなく、
   ガイド線が基準面から外れていた。`AddNewSweepLine` Mode=4 は曲線が案内サーフェス上に
   あることを要求するため、始端の壁が**全ドラフト角で掃引失敗**していた。
   `min(half_footprint + margin, half_width * 0.98)` にクランプして解決。
2. **BiTangentのkeep判定に、その段階で存在しない壁のプローブまで渡していた**:
   手順⑤は left+start / right+end / band の3段階でBiTangentを重ねるが、最初の
   left+startの中間結果に対して**右壁の根元プローブまで**「残っているべき」と要求して
   いた。右壁はまだ存在しないので必ず不合格になる。`wall_root_probes`を左右に分け、
   段階に応じた部分集合だけを渡すよう修正。SS7.3で「ビードが成立するのは約16%」と
   記録した低成功率の一因と考えられる。

#### 現在の到達点(2026-08-24時点)

ビード構築7手順のうち、①頂面オフセット ②中心線 ③根元曲線+幅ガイド ④壁4枚の掃引
⑤-1 corner(left,start) ⑤-2 corner(right,end) までが実機で通るようになった。
**残りは ⑤-3 corner(band)**(2つのL字リボンを統合して4隅R付きの連続バンドにする段)で
`no BiTangent orientation satisfied the geometric probes` となる。次の調査対象。

`_add_bead_to_surface`の中心線構築とビード全体はtry/exceptでInfeasible(ValueError)に
変換され、skip-and-retryに委ねられる(roadmap SS6.20と同じ方針)。`bead_probability`は
既定0.0の任意機能なので、ビード無しのメイン形状生成はこの未解決点の影響を受けない。

#### 教訓: CATIAセッションの残留状態と、失敗フィーチャーの後始末

- 同一のCATIA COMセッションで何度もprobeスクリプトを実行する際は、検証の直前に必ず
  全ドキュメントを明示的に`Close()`すること。そうしないと残留状態による偽陽性を
  「実機で確認した」と誤認する(実際に一度誤認した)。
- **失敗したフィーチャーは必ず`_delete_feature`で消すこと**。失敗したまま放置すると
  ツリーに残り、以降の**全ての`part.Update()`を巻き添えにして失敗させる**。
  中心線検証で「投影済み曲線でもCurveParが失敗する」と誤認した原因はこれだった
  (直前に試した未投影スプラインでの失敗フィーチャーが残っていた)。総当たり探索を
  するコードでは、失敗パスでの後始末が必須。

### 8.10 【解決】ビードの壁バンドは「丸めコーナー入り閉曲線の単発スイープ」で作る(2026-08-25)

SS8.9で残っていた ⑤-3 corner(band)(2つのL字リボンをBiTangentで統合して4隅R付きの
連続バンドにする段)は、**向き4通り・trimMode・relimitationMode・コーナーR(9.1〜3.0mm)の
いずれを振っても成立しなかった**(必ず片側のリボンが丸ごとトリムで消える)。

**採用した方式**: ユーザー元仕様の手順③「全ての壁をトリムで1つの連続した壁に仕上げる」に
素直に従い、**コーナーRを最初から織り込んだ閉じた外形曲線**を作って**1回だけスイープ**する。
コーナーRが曲線に含まれているので、隅をフィレットで作る必要が無い = BiTangentでループを
閉じる必要が無い。手順④⑤が丸ごと1ステップに畳まれる。

構築は中心線と同じ「解析標本点 → `AddNewSpline`(閉) → `AddNewProject(Normal=True)`」:

- 外形は (run, width) のローカル座標で丸め長方形として描き、各パネルのフレームで3Dへ写す。
- 長辺は複数パネルをまたぐので、曲げフィレット領域には標本点を置かず平坦区間だけから採る
  (フィレット上はスプライン補間+投影に任せる)。折れ目は傾いているので、幅座標wでの
  境界runはシアーぶん `w*tan(tilt)` ずれることを織り込む。
- スイープは `AddNewSweepLine` Mode=4。ドラフト角は従来通りprobe-and-selectだが、
  閉曲線を一括で掃引するので壁4枚は同じ向きに倒れる → 長辺の左右2点で判定する。

実測(`tools/probe_bead_closed_outline.py`): 閉スプライン375.95mm・投影377.19mm、
スイープ成功、面積7801mm²(理論値7584mm²、比1.029)。

#### 実装で踏んだ2つの落とし穴

1. **連続重複点でAddNewSplineが退化する**: 円弧の終点とキャップ/長辺の始点は同じ位置に
   なるので、素直に連結すると重複点が残る。閉曲線なので末尾と先頭も一致する。
   `Update`が失敗するので、0.01mm許容で間引く。
2. **ビードの逃げ位置が曲げフィレットに乗ってはいけない**: 端パネルが短いと、逃げを取った
   位置が曲げのR上に来る。実測で「終端パネル長21.8mm、フィレット接線8.5mm、逃げ位置が
   わずか0.12mmしか外に出ていない」ケースがあり、頂面オフセットの向き判定が
   0.19〜0.35mmのずれで落ちていた。**オフセット面自体は全パネル・全位置で誤差0.00mmと
   完全に正確**(`tools/probe_offset_deviation.py`で確認)だったので、原因はプローブ点が
   円筒面上に乗っていたこと。形状としても不正なので`plan_bead_on_surface`で
   明示的にInfeasibleとして弾くようにした。

#### 実機検証(`tools/probe_bead_single_sweep_build.py`)

本番経路`build_general_two_point(bead=...)`で**①頂面オフセット〜⑦足元Rまでの全工程が完走**。
生成物を目視確認し、Z字に2回曲がった板の中心線に沿ってビードが走り、両端は締結パッドの
手前で丸く逃げ、稜線も滑らかな、製造設計レベルのブラケットになっていることを確認した
(`tools/probe_output/view_b140_iso.png`)。

#### 歩留まりの現状(332試行、成功2件)

| 失敗理由 | 件数 | 帰属 |
|---|---|---|
| `free_fold_seed found no feasible w-parallel construction` | 192 | **基準面**(ビード無関係) |
| `bead wall band: no draft angle` | 32 | ビード |
| `could not build a usable offset surface` | 15 | ビード |
| `no BiTangent ... top ridge` / `foot ridge` | 25 | ビード |
| `could not sweep the extended wall` | 3 | ビード |

最大の要因(58%)は基準面のチェーン構築側で、ビードとは独立。次の改善対象。

### 8.11 歩留まり改善: 中間折れ位置のslackを距離から逆算する(2026-08-25)

`free_fold_seed`の失敗を分類したところ、**62%が「折れ角が上限135度超」**で、しかも
**ちょうど180度が34.8%**と突出していた。180度はランプが逆走している状態、つまり
中間折れ目2本を置くだけの距離が締結点間に無いということ。

これはリモートセッションでのユーザー指摘「**上記ロジックを実現するために、締結点同士は
ある程度距離を離す必要があります**」(2026-08-24)そのもので、未実装だった。

実測(seed成立/失敗で分離): 中間折れに必要な量 `2*bearing + 2*T + slack1 + slack2` に対し、
w直交断面上の締結点間距離の**余裕は成立時+20.5mm / 失敗時-51.6mm(いずれも中央値)**。
`FOLD_SLACK_RANGE_MM=(0,60)`を両側独立に引くと最大120mmを消費する一方、断面距離の
中央値は75mmしかなかった。

**対策**(このプロジェクト共通の feasible-by-construction 方針):

1. slackを独立サンプリングせず、**実際の距離から使える予算を逆算**してその中で配分する
   (`slack_budget = 断面距離 - 最小ランプ長 - 2*bearing - 2*R`)。1本あたりの上限は従来通り。
2. 締結点間距離の下限を 60mm → **90mm** に引き上げ。

| 条件 | seed成立率 |
|---|---|
| 現状(距離60〜200・slack独立0〜60) | 36.8% |
| slackを距離から逆算 | **74.9%** |
| 上記 + 距離下限90mm | **80.8%** |
| 距離下限90mmのみ | 46.4% |

本番サンプラーへ適用して実測 **81.6%**(改修前36.8%)。

### 8.12 併せて修正した3件(2026-08-25)

1. **傾いた折れ目で台形パネルが潰れる** — 幅端では走行長が
   `half_width*|tan(far_tilt)-tan(near_tilt)|` だけ削られる。潰れるとFill/Joinが失敗し、
   **生のcom_errorがバッチ全体を止めていた**(120試行中1件)。CATIA側の脆さではなく
   純粋な幾何なので、`MIN_SHEARED_PANEL_SPAN_MM`による事前チェックで弾くようにした
   (「joinの失敗＝本物のバグの兆候」という既存方針を保つため)。
2. **ビード構築中のCATIA例外が変換されていなかった** — `_add_bead_to_surface`の呼び出しを
   try/exceptで包み、Infeasible(ValueError)へ変換してskip-and-retryに委ねる。
3. **投影された外形曲線の検算を追加** — 標本点の折れ線長と比べて0.9〜1.5倍の範囲に
   入らなければ、投影が別の場所へ落ちている(閉曲線が壊れている)とみなして弾く。
   実測で「折れ線に対し投影が36〜56mmしかない」ケースが実在した。

### 8.13 現在の残課題(2026-08-25時点)

120試行での失敗内訳:

| 失敗理由 | 件数 | 帰属 |
|---|---|---|
| `bead wall band: no draft angle` | 24 | **ビード(最大の未解決)** |
| `free_fold_seed found no feasible construction` | 29 | 基準面 |
| `could not build a usable offset surface` | 14 | ビード |
| 投影の検算で棄却 | 3 | ビード |
| BiTangent(top ridge / foot ridge) | 4 | ビード |

**`no draft angle`の性質**: 4つの角度候補すべてで壁は掃引できるが、期待した頂部位置から
**一様に30〜60mm離れている**(例: `[-56deg: 59.90mm, -124deg: 53.95mm, +124deg: 58.56mm,
+56deg: 59.90mm]`)。角度によらず同じくらい離れているので、**角度選択の問題ではなく
外形曲線かプローブ点のどちらかが想定と違う場所にある**とみるのが自然。
部品全長と同オーダーのずれなので、次は失敗ケースのCATPartを保存して目視で切り分ける。


---

## 9. 実行環境の変更と、基準面の形状品質の是正(2026-08-25)

### 9.1 CATIAライセンス喪失 → DELMIAへ切り替え

セッション途中でCATIAが起動できなくなった(`サーバーの実行に失敗しました`)。調べたところ
**`CATIA.Application` というProgIDがレジストリに存在しない**(登録されているのは
`DELMIA.Application` のみ)。ユーザーの指摘通りDELMIAは起動できたので、そちらへ切り替えた。

V5では製品が違っても**同一の自動化オブジェクトモデル**を公開する。実機で必要な機能が
全て動くことを確認済み: `AddNewOffset` / `AddNewSpline` / `AddNewSweepLine` /
`AddNewFilletBiTangent` / `AddNewProject` / `AddNewCurvePar` / `AddNewJoin` /
`AddNewIntersection` / `SPAWorkbench`(計測)。

`SyntheticPartBuilder.__init__` を「**稼働中のセッションに接続する**」方式へ変更した
(`GetActiveObject` を先に試し、起動を伴う `Dispatch` は後回し。ProgIDは
`CATIA.Application` → `DELMIA.Application` の順に試す)。ライセンス取得の頻度も下がる。

**運用上の注意**: 検証スクリプトが1試行ごとにドキュメントを開閉するのを短時間に何百回も
繰り返すと負荷が高い。生成と描画を別スクリプトに分けると開閉回数が倍になるので、
可能なら1セッション内で完結させること。

### 9.2 【重要】基準面の形状品質 — 折れ目の傾きに上限が必要だった

失敗形状だけでなく**成功形状**を目視したところ、「CATIAが構築できた」ことと
「板金部品として成立している」ことが別物だと判明した。ビード無しの成功率53%のうち
相当数が、**ねじれたS字リボン**や**先端が尖ったスリバー**といった非現実的な形状だった
(`tools/probe_output/base_only/png/base_003_mid.png` 等)。

原因は自由折れ目チェーンの傾き自由度。実測で**傾きは中央値29.4度・p90で59.2度**あり、
tan(59度)=1.66なので半幅25mmのパネルは走行方向に±41mmずれた平行四辺形になる。

**ユーザー承認(2026-08-25)を得て、折れ目の傾きに上限30度(`MAX_FOLD_TILT_DEG`)を設けた。**
単に棄却するだけでなく、`solve_free_fold` に渡す目標傾き `target_a1` を上限内へ
クランプしてから解く(a2は閉合条件で決まるので保証できず、解けた後に改めて検査する)。

実測のトレードオフ(純粋幾何、n=4000):

| 傾き上限 | 通過率 | 最悪シアー比 中央値 | 比>=0.8(ほぼ長方形)の割合 |
|---|---|---|---|
| 15度 | 14.6% | 0.94 | 100% |
| 20度 | 21.4% | 0.93 | 100% |
| **30度(採用)** | **34.4%** | **0.90** | **89%** |
| 40度 | 45.8% | 0.86 | 71% |
| 無制限 | 56.0% | 0.83 | 58% |

**効果**(実機、60試行):

- 基準面のみの成功率: 53% → 37%(ただし**形状は劇的に改善**。均一な幅の3パネルが
  穏やかな2曲げで繋がった、まともな板金ストリップになった)
- **ビード固有の失敗が大幅減**: `no draft angle` 11→6件、`offset surface` 6→3件。
  ビードの失敗の多くが「破綻した基準面の上にビードを置こうとしていた」ことの
  裏返しだったと確認できた。

### 9.3 併せて入れた3件

1. **パネルのシアー比率チェック**(`MIN_SHEARED_PANEL_SPAN_RATIO = 0.5`) — 絶対値
   (最小走行長2mm)だけでは「幅端で3mm・反対端で40mm」のような極端なスリバーが通る。
   実測でシアー比率の最小値は**-49.4**(公称走行長の50倍のシアー)だった。
2. **外形の標本点の空白を細分**(`BEAD_OUTLINE_MAX_GAP_MM = 15.0`) — 曲げフィレット領域に
   点を置かない設計のため長い空白ができ(実測**最大172.6mm**)、閉スプラインが支え無しに
   補間して**大きく渦を巻いていた**(失敗形状の目視で発見。長さ検算は蛇行しても通るので
   見逃していた)。細分後は最大間隔15.0mm、投影失敗も半減。
3. **締結点間距離からslack予算を逆算**(SS8.11) — seed成立率 36.8% → 81.6%。

### 9.4 現在の到達点と残課題

- **基準面のみ**: 成功37%、形状は板金部品として妥当。**実運用可能**。
- **ビード付き**: 60試行中1件。基準面が健全なケースに限れば約5%。
  残る失敗は `no draft angle` 6 / `offset surface` 3 / `projected outline` 2 /
  `BiTangent(top/foot ridge)` 2。いずれもビード固有で、次の調査対象。

### 9.5 基準面のみの成功率を80%→90%へ(2026-08-25、層別分析にもとづく)

**失敗を段階で分解し、入力変数で層別**したところ、残りの失敗は「不運」ではなく
2つの物理的パターンに集約されると判明した。

| 判別変数 | 通過率の変化 | 失敗モード |
|---|---|---|
| **小さい方の折れ角** <10度 → 20〜40度 | 66% → 89% | 傾き上限超(a2側) |
| **大きい方の折れ角** <70度 → 110度以上 | 92% → 71% | solve未収束 |
| 法線相対角 70〜110度の帯 | 他84〜91%に対し78〜80% | 両方 |

- **パターン①(傾き上限超)**: 法線がほぼ直交(中央値81.9度)で全体の回転量が大きいのに、
  片方の折れがほとんど仕事をしていない(折れ角min中央値10.6度)。もう片方が全部を負担し、
  閉合条件がa2に大きな傾きを要求して上限を超える。
- **パターン②(solve未収束)**: 両方の折れが急(max 111.7度 / min 58.5度)で、
  ホモトピー継続が追従できない。

**対策(ユーザー承認: 配分制御と法線側を「ほどほどに」両方、多様性に配慮)**

1. **折れ配分の探索**(`_choose_fold_slacks`): slackは中間折れ目の位置を決め、それが
   ランプ方向→折れ角の配分を決める。実測でslackを0〜60mmで振ると小さい方の折れ角は
   中央値40度・大きい方は43.5度動かせ、**97%のケースで「min>=25度 かつ max<=95度」を
   満たす組合せが存在する**。候補を8通り試して満たすものを採る。
   → 82.4% → 86.2%(seed不成立は1.5%→0.1%とほぼ解消)
2. **谷の帯の重み下げ**(`NORMAL_ANGLE_DIP_*`): 法線相対角70〜110度だけ通過率が落ちるが、
   単調ではなく中央が谷なので「上限を絞る」のは非効率(直交クラスを丸ごと失う)。
   除外せず**引き直し1回**で重みだけ下げた。→ +1ポイント程度
3. **傾き摂動の探索**(`_choose_tilt_perturbation`): 残る2モードはどちらも
   **解いてみないと分からない**量(a1は上限へクランプできるがa2は閉合条件で決まる)。
   サンプラー側で`solve_free_fold`を6回まで試し、収束しかつ傾きが上限内の摂動を採る。
   CATIAの1ビルドが数秒なのに対しここは純Pythonで軽いので、先に潰す方が得。
   → 86.2% → **94.5%**(傾き上限超 6.5%→1.3%、solve未収束 5.9%→1.0%)

**実機検証: 80試行中72件成功 = 90%**(改修前37% → 80% → **90%**)。

**多様性は維持**: 配置クラスは5種すべて出現(oblique 63.3% / parallel_same_offset 24.8% /
orthogonal 9.1% / parallel_opposite 2.7% / coplanar_flat 0.1%)、法線相対角はp10-p90で
10〜130度、締結点間距離134〜216mm、基準面R 10.7〜19.2mm、板幅31〜50mm。
形状も均一幅・穏やかな2曲げで、ねじれやスリバーは出ていない。

**副作用**: サンプラーが`free_fold_seed`/`solve_free_fold`を内部で複数回呼ぶため重くなった
(テスト実行が0.5秒→3.6秒)。CATIAビルド1回の数秒に対しては十分割に合う。

---

## 10. ビード外形を「基準面の境界オフセット」で作る(2026-08-25、ユーザー提案)

### 10.1 きっかけ — 解析点+投影方式の限界を層別で確定させた

SS8.10で確定した「コーナーRを織り込んだ解析標本点 → 閉スプライン → `AddNewProject`で
基準面へ投影」方式は、実機の歩留まりが 6/60 (10%) で頭打ちになっていた。
`_bead_wall`を診断版に差し替えて41件を層別した結果、**失敗が2種類に分離**した。

| 症状 | 件数 | 判定方法 |
|---|---|---|
| ① 投影した閉曲線そのものが不正 | 14/41 (34%) | 長さ1mm・全ドラフト角で掃引が落ちる |
| ② 内向き掃引だけが退化 | 9/27 (33%) | 1mmなら全角度OK、フル長で+θのみ落ちる |
| 成功 | 18/41 (44%) | |

②についてはビード側のパラメータ(depth / 壁角度 / 頂部幅 / コーナーR / フットプリント半幅)を
すべて層別したが、**成功群と失敗群を分ける変数は見つからなかった**。途中で2つの仮説を立てて
実測で否定している。記録として残す:

- **仮説A: コーナーRが壁の倒れ込み量(`depth/tanθ`)より小さいと頂部側が自己交差する**
  → 制約を入れて実機再測定。4/60 → 6/60 の微増のみ。主要因ではない。
- **仮説B: フットプリント半幅 hf > 13.5mm で内向き掃引が退化する**
  → 層別上は hf>13.5 が 5/5 全滅、hf<=13.5 が 13勝2敗と綺麗に分かれた。
  しかし上限を入れて再測定すると**失敗ケースは失敗のまま**。相関であって機構ではなかった。

①については原因の見当がついた。長辺の標本は曲げフィレットの外(平坦区間)からしか採れないので、
**フィレットを横切る区間が基準面から離れた3Dの弦**になる。面から浮いたまま走る閉スプラインを
投影すると分断・失敗する。フィレット上の点を解析的に置いて塞ぐ実装まで書いたが、
そこで**折れ目が傾いているとパネルAの幅wの点とパネルBの幅wの点は同じ母線に乗らない**ことが判明
(R11.08・37.3度で理論弦7.09に対し実測9.26)。母線軸まわりの回転で作り直しても、
今度は回転角にuベクトル間の角度を使っていたのが誤り(傾いた折れ目では二面角と一致しない)。
**この方向は本質的に、実際に構築される面の座標系をPython側で再現し続ける戦いになる。**

### 10.2 【確定】ユーザー提案: 曲線を最初から面の上に作る

> 基準面の外形を接線連続で抽出し、並行曲線でスイープの起点を作成し、スイープしてからトリム、
> もしくは起点となる曲線をトリムして閉曲線にしてからスイープ、というやり方ではいけないのですか？

`tools/probe_bead_boundary_offset.py`で3点を実機検証した結果、**12件中10件(CATIA段階に
到達した全件)で全ステップ・全ドラフト角・全長さが成功**した。

| | 解析点+投影(旧) | 境界オフセット(新) |
|---|---|---|
| 曲線が使えるか | 27/41 (66%) | **10/10 (100%)** |
| 内向きスイープ | 18/27 (67%) | **10/10 (100%)** |

事前に懸念した3点はいずれも空振りだった:

1. `AddNewBoundaryOfSurface(surface_ref)` はJoin済み基準面にそのまま効く。
   **エッジ参照フィレットが自動化できない件(SS4)の地雷は踏まない** — サーフェス参照を取り、
   カーブ*フィーチャー*を返すため。
2. `AddNewCurvePar(境界, 支持面, オフセット, 反転, Geodesic)` は内側オフセットを一発で返す
   (境界467mm → 352mm)。SS8.9で24通り全成功していた実績どおり。
3. コーナーが尖っていてもMode=4スイープは通る。

### 10.3 端の逃げ — 基準面を先にSplitしてから境界を取る

一様な平行曲線だと横方向のオフセットと端の逃げが同じ値に縛られる。ユーザー案の後者
(先にトリムして閉曲線にする)を、**曲線ではなく基準面に対して**適用する:

```
d = half_width - bead.half_footprint_mm          # 一様オフセット量
基準面を run = start_run - d と end_run + d で2枚の平面Split
  → その断片の境界(AddNewBoundaryOfSurface)
  → 内側へd平行オフセット(AddNewCurvePar)
  → 長辺は ±half_footprint、端は start_run / end_run にちょうど出る
```

Splitの残す側と平行曲線の向きは形状依存なので決め打ちせず、**得られた閉曲線が
長辺の中点と両端キャップの中点を通ることを実測して選ぶ**(コーナーの丸め方はCATIA任せなので
四隅は使わない)。実測では`to_top`(外形→期待される頂部エッジの距離)が全件で斜辺長`slant`と
2桁一致し、**失敗ケースでも外形曲線は正確な位置に出ている**ことを確認した。

### 10.4 トリムが持ち込んだ失敗と、その切り分け

トリム無しの境界オフセットが12件×4角度すべて成功するのに対し、トリムを挟むと5/12が落ちた。
**トリムが唯一の原因**であることをA/Bで確定させたうえで、3つの仮説を順に潰した:

| 仮説 | 実装した対策 | 結果 |
|---|---|---|
| 無限平面が他パネルも切り、Splitが離れた2枚になる | `cuts_other_panels`でPython側に事前判定 | 5件中1件を弾いた |
| 切断位置 `start_run - d` が曲げフィレット上に落ちる | 逃げ位置と同じ平坦区間判定を切断位置にも適用 | 変化なし(元から平坦区間内) |
| keepプローブだけでは切り落とし側の残存を見逃す | 切断ごとにremoveプローブを1点追加 | 全件通過 = Splitは健全 |

**removeプローブは切断ごとに対応する側の1点だけを見ること。** 両端まとめて見ると、
1回目のSplitの時点で反対側がまだ残っているため必ず不合格になる(一度踏んだ)。

残る4件の機構は未解明。ビード側のパラメータでは層別できず、基準面側の形状変数と見ている。
ただし**失敗は例外ではなく明示的なInfeasible判定として落ちる**ので、バッチは止まらない。

### 10.5 削除したもの

外形をCATIA側で導出するようになったので、以下は不要になり削除した:

- `plan_bead_on_surface`の外形点生成一式(長辺標本・コーナー円弧・フィレット横断弧・
  隙間細分・重複除去)と、定数`BEAD_OUTLINE_ARC_POINTS` / `_EDGE_POINTS_PER_PANEL` /
  `_DEDUPE_TOLERANCE_MM` / `_MAX_GAP_MM`、ヘルパ`_rotate_about`
- `gsd_build`側の閉スプライン構築・`AddNewProject`・折れ線長による検算
- `BeadParams.corner_radius_mm`とそのサンプリング。**平面視のコーナーRはCATIAの
  オフセット処理が決めるので、Python側で持つ意味が無くなった**
- 否定された仮説の定数`BEAD_CORNER_RADIUS_WALL_RUN_RATIO` / `BEAD_MAX_HALF_FOOTPRINT_MM` /
  `BEAD_CORNER_RADIUS_FOOTPRINT_RATIO`

### 10.6 到達点(60試行)

| 段階 | 改修前 | 改修後 |
|---|---|---|
| 外形曲線が作れない | 14/41 (34%) | 構造的に解消 |
| 壁の掃引で落ちる | 23 | 16 |
| **BiTangent(⑥⑦の稜線R)で落ちる** | 11 | **18** |
| その他(フットプリント不成立・逃げ位置・トリム判定など) | 13 | 14 |
| 成功 | 6/60 | 6/60 |

総数は変わらないが、**ボトルネックが外形曲線から下流のBiTangentフィレットへ移った**。
外形が安定して作れるようになったぶん、より多くのケースが⑥⑦まで到達している(11→18)。
次に手を入れるべきはBiTangent段。

---

## 11. 壁と稜線Rの失敗要因を層別で切り分ける(2026-08-25、ユーザー指示)

SS10で外形曲線の問題を構造的に解消したあとも歩留まりは6/60で頭打ちだった。
ボトルネックが下流へ移っただけだったので、**壁の掃引**と**稜線R(BiTangent)**の
2つを同一バッチで層別した(`tools/probe_bead_stratify.py`。本番経路には触らず、
`plan_bead_on_surface` / `_bead_wall` / `_bead_bitangent` を包んで観測だけ取る)。

### 11.1 壁の掃引 — 「長さ」ではなく「ビードがどれだけフィレットの上に乗るか」

最初の層別で、失敗16件のうち14件は**掃引自体は成功しており、頂部エッジが期待位置から
3.3〜10.0mmずれている**ことが分かった。そこでユーザーから「スイープの長さを適切に
とれば問題ないのでは」という問いを受け、1.0/1.5/2/3/5倍で振って実測した
(`tools/probe_bead_wall_length.py`)。

**答えはNo。距離は全ての長さで1ミクロンも動かない。** 判定は
`GetMinimumDistance(バンド, 期待頂部点)`なので、バンドを伸ばせば距離は単調に縮むしかない。
動かないということは長さ不足ではない。

さらに**ずれ量がほぼ常に斜辺長`slant`そのもの**だった(7.44 vs 7.4、9.36 vs 9.4)。
これは最小距離が「根元曲線↔目標点」の距離に張り付いている指紋であり、実体は
**内向き(正しい向き)の掃引が構築できず、逆向きに倒れた壁だけが構築されている**。
最初の「掃引は成功しているが位置がずれる」という読みは誤りだった。

決定変数は**ビードが載る平坦区間を持つパネルの数**(`n_bead_panels`)。

| ビード区間があるパネル数 | 壁OK | 壁NG |
|---|---|---|
| 3パネル(全部) | 28 | **0** |
| 2パネル | 8 | 7 |
| 1パネル | 2 | 9 |

曲げが急でRが大きいとフィレットがパネルを食い尽くす。外形の閉曲線はそれでも端から端まで
走るので、**ビードの大半がフィレットの上に乗り**、平面前提で計算した頂部エッジの解析位置が
実物と合わなくなる。「頂部プローブが端パネルへ移るせい」という仮説は外れた
(端パネル3勝2敗 vs 中央パネル21勝14敗で比率が同じ)。

### 11.2 稜線R — 折れ目の傾きが第一、消費率が第二

最初の層別では**傾きだけが分離した**(成功 4.5度 [p10 1.1, p90 9.3] / 失敗 14.4度
[p10 8.5, p90 23.3]、10度以上は0/15)。壁段階は傾きに全く反応しない(12.0 vs 11.6)ので、
両者は独立した制御軸である。ユーザー判断で`MAX_FOLD_TILT_DEG`を30→**5度**に変更。

→ CATIA段階の全成功率 15% → **36%**、稜線R通過 25% → 47%。

傾きを絞ると今度はどの変数でも層別できなくなった。ここでユーザーから

> 成功しているシードの基準面でも、ビードのパラメータ次第で失敗する可能性がありますし、
> その逆もありうると思います

という指摘を受け、**基準面 × ビードの直交行列**に切り替えた
(`tools/probe_bead_fillet_matrix.py`)。ランダムな(基準面, ビード)の組では両者の寄与が
混ざっていて分離できない、という指摘は正しかった。

粗い格子(3深さ×3稜線R×6基準面)では**まだら**になり、行(基準面)でも列(ビード)でも
説明できなかった。深さを固定して稜線Rだけを細かく振ると規則が出た:

```
消費率 2*R*tan(θ/2)/slant:  0.61  0.73  0.85  0.91  0.97  1.04  1.10  1.22  1.40  1.58
base#63                       O     O     O     O     O     O     O     O     O     t
base#44                       O     O     O     O     O     O     O     O     O     t
base#55                       O     O     O     f     f     f     O     O     O     t
base#40                       O     O     O     O     f     O     f     f     f     t
base#2                        O     O     O     f     f     f     f     f     t     t
base#20                       O     O     f     f     f     t     t     t     t     t
失敗/6                        0     0     1     3     4     3     3     3     3     6
```

**端は完全に決定的、中間は基準面依存かつ非単調**:

- 消費率 **≤0.73 → 12/12成功**(6つの異なる基準面すべて)
- 消費率 1.58 → **0/6**
- 中間は基準面ごとに違い、単調ですらない(#55は0.91-1.04で失敗し1.10-1.40で成功)

非単調ということは、そこでは幾何が既に退化していてCATIA内部処理の当たり外れになっている。
**設計として使ってはいけない領域**である。なお途中で「各基準面が単調な閾値を持つ」と
読んだが、追試(#55/#40)で崩れた。3基準面での過適合だった。

### 11.3 発見: 実行可能性チェックが本番経路から呼ばれていなかった

`bead_fits`(内部で`ridges_fit_on_wall` = 消費率≤1 を検査)は**テストからしか呼ばれて
おらず、生成経路から一度も呼ばれていなかった**。旧セル分解方式(SS8.10で廃止)の遺物で、
`run_out_diagonal_mm`や頂部ストリップのマイター潰れなど、現在の構築方式には存在しない
概念を検査している。そのためサンプラーが出す**消費率>1の退化ビード(13.4%)がそのまま
構築されていた**。`bead_fits` / `run_out_diagonal_mm` / `MAX_MITER_AMPLIFICATION` を削除。

### 11.4 R5最小との構造的な衝突

中立面R最小5mmは、消費率を抑えるために**深さの下限**を強制する:

| 壁角度 | 消費率≤1に必要な深さ | 消費率≤0.73に必要な深さ |
|---|---|---|
| 45度 | 2.9mm | 4.0mm |
| 55度 | 4.3mm | 5.8mm |
| 65度 | 5.8mm | 7.9mm |
| 70度 | 6.6mm | **9.0mm** |

深さ範囲(4,10)mm・壁角度(45,70)度に対し、壁角度70度では深さ9.0〜10.0mmしか使えない。
そこで`sample_bead`の依存順を **壁角度 → 深さ(下限を逆算) → 稜線R → 頂部幅** に変更した。
深さを先に振ると壁角度が縛られてしまうため、角度が先。これで消費率は構造上0.73を超えない
(実測: 中央0.68・最大0.73・超過0.00%)。副作用として**壁が立つほどビードが深くなる**という
相関が入るが、これはR5最小が要求する物理的な帰結である。

### 11.5 到達点

240試行での実測(いずれもCATIA段階に到達したものの比率):

| 対策 | CATIA到達 | 壁OK | 全成功 |
|---|---|---|---|
| SS10終了時点 | 40 | 60% | 15% |
| + 傾き上限5度 | 56 | 77% | 36% |
| + 消費率上限0.73 | 54 | 70% | **50%** |
| + 全パネルにビード区間 | 37 | **100%** | **89%** |

最終実測(300試行、`tools/probe_bead_stratify.py`): CATIA到達37件、壁の掃引 **37/37**、
全成功 **33/37 = 89%**。早期棄却は263件(傾き上限187、全パネル必須47、フットプリント不成立8、
solve未収束6ほか)で、いずれも純Python。

早期棄却は240試行中180件前後だが、いずれも純Pythonで判定できるので実質無コスト
(105件/秒)。10秒かかるCATIAビルドを無駄にしないことが重要。

**残る課題**: 失敗は4件のみ(top ridgeのUpdate失敗3、foot ridge 1)。パネル長・平坦区間は
むしろ成功群より大きく(69.2 vs 64.7 / 58.5 vs 54.4)、11.1の変数はもう効いていない。
n=4では層別できないので、次に触るならまた直交実験を組む必要がある。

**未承認のまま入れた項目**: 「全パネルにビード区間」はユーザーの明示承認を得ていない
(提案直後に傾きの調査へ話が移ったため)。実測で成功33件中3件を落とす代わりに、
無駄なCATIAビルドを23回省く。外す場合は`plan_bead_on_surface`の該当チェックを削除する。

**旧方式の残骸**: `bead_layout`とその一群(`_panel_axes` / `_miter_offset` /
`_top_level_shifts` / `_cross_section` / `_boundary_points` / `_run_out_cells`)は
生成経路から呼ばれておらず完全に死んでいるが、今回の承認範囲外なので残してある。

---

## 12. 設計レビューにもとづく歩留まり・品質・データセット改善(2026-08-25、ユーザー承認①②③④)

SS11の到達点(CATIA段階89%)を受け、生成スクリプト全体の設計レビューを行い、
4項目を実装した。原則(feasible-by-construction / probe-and-select / fail-fast)は
正しく機能しており、改善点の大半は**原則が一部の箇所で守られていないこと**だった。

### 12.1 ① サンプラーと傾き上限の整合

傾き上限5度(tan5deg=0.087)に対し、サンプラーは横ズレ比を±0.40で振っていた。
実測(300試行): 比0.15-0.45の帯(試行の65%)は84%が傾き棄却・OK率4%、比<0.09なら
OK率36%。`MAX_LATERAL_OFFSET_RATIO` 0.40 -> **0.10**、`FOLD_TILT_PERTURBATION_RANGE_DEG`
±6 -> **±3度**(上限5度に対して±6は候補の大半がクランプされ無駄)。

出力分布は変わらない(上限が既に排除している領域)。**入力カバレッジの注記**: 横ズレ比
>0.1の締結点ペアは現行の3パネル構成では作れない。カバレッジを広げる場合は傾き上限と
セットで再検討(ビード無し部品のみ30度に戻す分岐が可能)。

### 12.2 ② 事前解決(resolve_bead_slacks) — 棄却は締結点の性質にだけ適用する

**切り分けの根拠**(2026-08-25実測): 棄却理由ごとにslack再探索での回収率が全く違う。

| 棄却理由 | 回収率 | 解釈 |
|---|---|---|
| 平坦区間不足(no_flat) | 27/42 = **64%** | 折れ目の置き方の性質 -> 選び直しで解決 |
| その他のビード計画棄却 | 9/18 = 50% | 同上(ビード再抽選も併用) |
| 傾き上限 | 15/191 = 8% | **締結点の性質** -> 棄却が正しい |

実装:
- `general_geometry.py` 新設。`gsd_build.build_general_two_point`のCATIA非依存部分
  (seed→solve→傾き→接線長→干渉→シアー→パネル構成決定)を`plan_general_two_point`
  として抽出し、builderはそれを呼ぶだけにした(単一の真実 — 診断プローブに重複していた
  同じチェック群の乖離リスクを解消)。`check_bead_feasible`は実機構築時と同一引数で
  `plan_bead_on_surface`を呼ぶ。
- `templates.general_two_point.resolve_bead_slacks`: 元のslackで通るか試し、駄目なら
  slack20候補、それでも駄目ならslack+ビード再抽選20候補。純Python(1判定数ms)。
- `batch_generate.generate_general_batch`: ビード指定時はbuilderを呼ぶ前に解決。
  解決不能(=締結点の性質で落ちる)ならspecを捨てて次へ。

**効果(300試行)**: CATIA適格率 13% -> **68%**(205/300、うち123件はリゾルバが回収)。
純Python処理は300試行で8.7秒。

### 12.3 ③ ビードの(壁角度, 深さ)を実行可能領域上で同時一様に

逐次サンプリング(角度→下限つき深さ)は、下限が角度の関数であるために**物理必然以上の
相関 r=+0.81** をデータに入れていた(「急な壁 ⇒ 必ず深い」という板金設計に存在しない
規則を学習させる恐れ)。矩形からのリジェクションサンプリング(実行可能率~60%、平均1.7回)
に変更し、**r=+0.46**(実行可能領域の形が強制する分のみ)へ。深さの実現域も5.2mm~ から
4.1mm~ に回復(浅い壁との組で浅いビードが出るようになった)。

### 12.4 ④ 部品ごとの生成パラメータ保存

joints.jsonは締結点と板厚しか持たず、slack・ビード寸法・傾き実績値がどこにも残らなかった。
成功部品ごとに `params/{part_id}.json`(spec全体・bead・fold_tilts_deg・geometry_label・
attempts_used)を保存。specはリゾルバ差し替え後の値なので、これだけで形状を再構築できる。

### 12.5 実機スモーク(本番経路エンドツーエンド)

`generate_general_batch`(count=10, bead_probability=1.0)を実機で実行:
**10部品 / 2.3分(10部品あたり2.3分、SS11時点の3.4分から32%短縮)**、全部品ビード付き、
params 10件保存、形状は目視で正常(2曲げを跨ぐビード・逃げ・ドラフト・稜線R)。
スループット向上はリゾルバによってCATIAに渡る前に失敗が解決されるため
(builderに渡る時点でCATIA非依存の失敗理由は残っていない)。

### 12.6 見送り・保留

- **⑤ 配置クラスの層化**(orthogonal 9%→3%低下への対処): 学習方針の判断待ち。
- **⑥ probe-and-selectの向きキャッシュ**(~20-30%短縮見込み): 任意、未実装。
- **⑦ 稜線R失敗(傾き5-10度帯)の機構解明**: 解明できれば上限10度でカバレッジ倍増。
  CATIA側調査が必要、効果不確実。

---

## 13. 平面視の四隅R復活 — 「縮めた尖りループの外向きオフセット」(2026-08-25、ユーザー指摘)

### 13.1 問題

SS10で外形を境界オフセット方式に切り替えた際、`BeadParams.corner_radius_mm`を削除した。
凸角の**内側**オフセットは角を尖ったまま残すので、掃引した壁バンドの四隅(壁同士の
縦エッジ)が尖る — プレス成形では成立しない形状(ユーザーがレンダリングで発見)。
ユーザー指定の工程順: 壁スイープ後**まず**このコーナーをフィレット、根本・頂稜線は最後。

掃引後のエッジフィレットはBRep参照が必要で自動化不可(SS4)なので、**外形曲線側に
コーナーRを織り込む**方針(コーナー処理が根本・頂稜線より必ず先行し、指定の工程順と
同じ結果になる)。

### 13.2 失敗した構成: 8要素Join

長辺2本(中心線のCurvePar±hf)+キャップ直線2本+解析円弧4本をJoinする案を先に試した。
段階的に3つの実装問題を潰した記録:

1. **スプラインの端接線**: 投影した中心線の端接線が補間条件で暴れ、CurvePar端点が
   解析コーナー接点から最大1.1mmずれる。端の0.8mm内側に補助点を置くと0.004〜0.009mmまで
   改善(joinの既定許容差0.001mmにはまだ足りない)。
2. **端点の厳密一致**: `Measurable.GetPointsOnCurve`は失敗する(COM地雷)。
   `AddNewPointOnCurveFromPercent(0%/100%)`で端点を点フィーチャー化し、円弧スプラインの
   始点として**参照を直接**使うと解決。
3. それでも掃引成立は**3/8**止まり。8要素の入れ子Joinというフィーチャー素性が
   Mode=4に嫌われている疑いが濃く、これ以上の深追いをやめた。

### 13.3 【確定】縮めた尖りループの外向きオフセット

**凸角の外向き測地オフセットは、角を自動的に「半径=オフセット量の円弧」にする。**

```
①基準面をトリム(位置は従来と同一: start_run - d / end_run + d)
②境界を (d + cR) 内側へオフセット
   -> 長辺±(hf-cR)・キャップstart_run+cR の**尖った**内側ループ
③それを cR **外向き**へオフセット
   -> 長辺±hf・キャップstart_run・四隅R=cR の閉曲線
```

d = half_width - hf とおくと ②のトリム位置 start_run + cR - (d + cR) = start_run - d で
**従来のトリム位置と厳密に一致**する。差分は「オフセットを2段にする」だけ。
Join・スプライン・端点合わせが全て不要で、使うのは実証済みプリミティブのみ。

実機12件: **11件で全ドラフト角のスイープ成立**(Join方式3/8、単段オフセットは四隅が
尖ったまま)。各段の向きは形状依存なのでプローブで選ぶ(内側ループは中央パネルの
±(hf-cR)点、外形は従来の長辺・キャップ中点)。

### 13.4 実装と検証

- `BeadParams.corner_radius_mm`復活。上限は内側ループのキャップ幅が残る条件
  cR <= hf - 1.5(と0.8*hf)、下限は中立面R最小。
- `plan_bead_on_surface`: キャップ幅チェックと内側ループプローブ(`inner_loop_probe`)を追加。
  トリム位置・トリム検査は無変更。
- `gsd_build`: 単段CurvePar選択を`_parallel_curve`ヘルパの2段呼び出しへ。

実機検証: 本番経路スモーク10部品/3.7分(全部品ビード付き、四隅Rを目視確認)。
リゾルバ経由の歩留まり測定(seed 31337): CATIA 40件中28成功(70%)。内訳は
トリム向き選定6・稜線R BiTangent 5・壁1で、いずれも既存機構の失敗
(四隅R由来の新規モードは無し)。SS12の89%とはシードとリゾルバ有無が違うため
直接比較不可 — リゾルバが救う「成立境界に近い」形状が母集団に加わったぶん
トリム向き選定の失敗率が上がった可能性がある。次の層別対象。

### 13.5 【訂正・真の解】CurveParType=1 — 外向きオフセットだけでは四隅は丸まらない(2026-08-25)

SS13.3の「凸角の外向き測地オフセットが四隅を自動的に円弧にする」は**実機では偽**だった。
ユーザーが完成品で四隅の尖りを発見。段階別ビルド(`tools/probe_corner_stages.py`)の実測:
2段オフセット後もバンドは尖り角位置を距離0.00mmで通過(=尖ったまま)。

原因: CATIAのParallel Curveには**コーナータイプ(Sharp/Round)**があり、既定はSharp
(接線延長で角を尖らせて繋ぐ)。自動化オブジェクト`HybridShapeCurvePar`では
**`CurveParType`プロパティ**(0=Sharp / 1=Round)。型情報ダンプで発見した
(GetTypeInfoの列挙。gencacheは使わない)。

外向きオフセット(2段目)に`CurveParType=1`を設定すると:
尖り角位置→バンド 0.00 → **3.95mm**(理論3.62)、円弧中点→バンド 2.79 → **0.00mm**。
四隅が真に半径cRの円弧になった。目視でも確認(根本・頂稜線フィレットがコーナーを
滑らかに回る)。

併せて**頂部の四隅R**を保証: 壁は内側へwall_run倒れるので頂部のコーナー半径は
cR - wall_run に縮む。根本だけ下限5にすると頂部が実質尖る(cR=5, wall_run≈5)。
サンプリング下限を wall_run + R最小 に変更し、planにも権威チェックを追加。

段階別CATPart(ユーザー確認用、tools/probe_output/corner_stages/):
stage_sharp(R適用直前) / stage_rounded(R適用後のバンド) / stage3_full(全工程)。
歩留まり測定はDELMIAセッションが途中終了したため未完(15部品成功まで確認)。

---

## 14. 側端フランジ — 折れ目を跨ぐフランジ生成(2026-08-25、ユーザー仕様)

### 14.1 使い分け(ユーザー指定)

**最大折れ角20度以下ならフランジ、それ以外(急でフランジ不成立)はビード。**
補強の種類が基準面の幾何の決定的な関数になるので、学習器は「締結点 -> 補強の種類」も
学べる。バッチは`reinforcement_probability`で補強の有無だけを確率制御し、種類は
`resolve_reinforcement`(templates/general_two_point.py)が自動選択する。

### 14.2 ユーザー指定の工程①〜⑤と実装の写像

| 指定 | 実装 |
|---|---|
| ①可否判断 | 純Python: 折れ角<=20度、平坦区間、凹側交差の高さ制約 |
| ②外側の弧のエッジ | 側端ポリラインの長い側=凸側。差<0.5mmなら乱数(多様性) |
| ③コーナーR+1〜2mmの面拡張(外挿) | **外挿コマンドは使わない**(未実証・BRep境界参照の疑い)。面生成を自前で握っているので、`plan_general_two_point(side_extension_mm=...)`でフランジ側だけ最初から広く作る(ユーザー承認) |
| ④外周エッジから90度スイープ | 中心線(標本点->スプライン->投影)のCurveParで根本曲線 -> Mode=4・±90度スイープ(向きはプローブ選定) |
| ⑤結合+エッジフィレット | **BiTangentフィレット**(基準面x壁、トリム込み — エッジフィレットはBRep参照で自動化不可、SS4)(ユーザー承認) |

### 14.3 制約の導出

- **凹側交差の高さ上限**: フランジが凹側の折れ目を跨ぐと、高さhの平行面は半径
  R_bend - h に縮み h >= R_bend で自己交差する。方向は凹側交差の無い側を優先し、
  やむを得ない場合(S字)は h <= R_bend - 2mm。下限10mmを割るなら不成立->ビードへ。
- 高さ10〜20mm、根本R 5〜8mm(1〜3t慣例とR5最小則の衝突はユーザー承認済みでR5優先)。
- 中心線の端接線補助点は**隣接点間の内分点**として挿入(端の外側に置くとパネル先頭点と
  逆行して自己交差スプラインになる — スパイクで実測)。

### 14.4 検証

- スパイク(spike_flange_over_bends.py): 側端曲線・±90度スイープ・根本R、6/6成立
  (折れ角17〜47度、単曲げ含む)。
- 本番経路(builder+resolve_reinforcement): フランジ4/6成功、1部品6秒。
  失敗2件は根本BiTangentのUpdate失敗(n=2、未層別)。

### 14.5 既知の論点: フランジ対象が希少(3.6%)

`_choose_fold_slacks`は折れ角の目標を「最小25度以上」に置いている(SS9.5、ほぼ平坦な
無駄折れを避ける品質判断)ため、最大折れ角<=20度の基準面は補強対象の**3.6%**
(実測: 走査265件中フランジ6・ビード162)しか出ない。フランジ部品の学習データを
増やすには、締結点サンプラーかslack目標に「フランジ帯(<=20度)を狙う割合」を
入れる必要がある — データセット構成の判断としてユーザーに提起する。
