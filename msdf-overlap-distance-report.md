# 重叠轮廓下的最近边距离与有符号距离分歧 — 源码分析报告

> 仓库：msdfgen 1.13.1（`ec84cd5`），分析范围 `core/` 与 `main.cpp`
> 方式：只读源码审阅 + 用仓库源码在 `/tmp` 外部编译的探针程序做数值复现（未修改仓库、未启动服务）

---

## 0. 结论速览

1. **“最近边距离”和“最终有符号距离”不是同一件事。** 边段只回答“从点到这条边段的最近距离及其由**轮廓方向**决定的符号”；`ShapeDistanceFinder` + `ContourCombiner` 才把它综合成一个形状级的有符号距离。二者在重叠/嵌套时分歧。
2. **两条代码路径的根本差别**：
   - **legacy 生成路径**（`generate*_legacy`）对所有边做**扁平的最小 `|距离|`** 扫描，没有轮廓分组、没有 winding、没有“填充/非填充边界”概念。隐藏的内部边可以获胜并翻转符号。
   - **支持重叠的路径**（`ShapeDistanceFinder<OverlappingContourCombiner<...>>`）按轮廓分别求有符号距离，用**每个轮廓的 winding** 把边界分成“内边界（正 winding，d≥0）/外边界（负 winding，d≤0）”，只在真正构成填充分界的轮廓里挑选，从而修正同向重叠的符号。
3. **符号有两个独立来源**：(a) 边段 `signedDistance` 的符号由**边界方向/winding** 决定；(b) `FillRule` 只在**扫描线符号修正阶段**（`distanceSignCorrection`）或预处理阶段介入。二者可互相矛盾。
4. **在 `(6.1, 5.0)` 处存在干净反例**：同向重叠正方形 A=[0,6]²、B=[4,10]² 并集内部，legacy/`Simple` 返回 **−0.1**（把 A 的隐藏右边 `x=6` 当成普通最近边），而最终期望值为 **+1.0**（到并集真实外边界的距离）。

---

## 1. 源码地图（本报告引用的关键位置）

| 主题 | 位置 |
|---|---|
| 形状距离综合入口 | `core/ShapeDistanceFinder.hpp:17-33`（`distance`）、`:37-57`（`oneShotDistance`） |
| 每轮廓累加边 | `core/ShapeDistanceFinder.hpp:18-31` |
| 简单合并器 | `core/contour-combiners.cpp:47-51`、`core/contour-combiners.h:10-25` |
| 重叠合并器（核心算法） | `core/contour-combiners.cpp:57-63`（构造时缓存 winding）、`:78-133`（`distance()`） |
| 距离/中值解析 | `core/contour-combiners.cpp:9-33`（`initDistance`/`resolveDistance`） |
| 距离选择器 | `core/edge-selectors.cpp:19-38`（`TrueDistanceSelector`）、`:166-258`（`MultiDistanceSelector`） |
| 边段有符号距离（符号来源） | `core/edge-segments.cpp:173-186`（线性）、`:187-227`（二次）、`:228-270`（三次），关键 `nonZeroSign(crossProduct(...))` |
| 有符号距离比较（按 `fabs`） | `core/SignedDistance.hpp:22-37` |
| winding 计算 | `core/Contour.cpp:57-80` |
| 轮廓定向 | `core/Shape.cpp:144-198`（`orientContours`） |
| 采样填充规则 | `core/Scanline.cpp:13-24`（`interpretFillRule`）、`:66-75`（`preprocess`）、`:121-123`（`filled`） |
| 扫描线符号修正 | `core/rasterization.cpp:19-32`（单通道）、`:34-88`（多通道） |
| 现代生成器分派 | `core/msdfgen.cpp:79-135`（按 `config.overlapSupport` 选择 Combiner） |
| legacy 生成器 | `core/msdfgen.cpp:166-213`（SDF/PSDF）、`:221-270`（MSDF）、`:281-330`（MTSDF） |
| 误差修正中的距离预言器 | `core/msdf-error-correction.cpp:41-44` |
| CLI 预处理/重叠开关 | `main.cpp:577-587`、`:750-780`、`:1151-1170`、`:1269-1343` |
| 公开注释（“legacy 不能处理重叠轮廓”） | `msdfgen.h:70-72` |
| 配置语义注释 | `core/generator-config.h:51-52` |

---

## 2. 基础：一层距离的符号从哪里来

### 2.1 边段级：符号 = 轮廓方向（winding）

`LinearSegment::signedDistance` 返回：

```cpp
return SignedDistance(nonZeroSign(crossProduct(aq, ab))*endpointDistance, ...);
```

（`core/edge-segments.cpp:184`；二次/三次曲线同理，见 `:199/205/214`、`:235/241/265`）

即：符号 = `sign(cross(origin-point0, 边方向))`。对封闭逆时针（`Contour::winding()==+1`，见 `core/Contour.cpp:57-80` 的鞋带公式）的方形，**内部为正**；把该轮廓反向，则内部变为负。实测（探针，见 §9）：

```
单个 + 方形 [0,10]²：内部 (5,5)=+5，外部 (12,5)=-2
单个 - 方形 [0,10]²：内部 (5,5)=-5，外部 (12,5)=+2
```

因此**原始 SDF 的符号由边界方向决定，与 `FillRule` 无关**。`FillRule` 是另一层机制（§6）。

### 2.2 合并级：`ShapeDistanceFinder` 只是“聚合器”

`ShapeDistanceFinder::distance`（`core/ShapeDistanceFinder.hpp:17-33`）按轮廓遍历，把每个轮廓的边交给该轮廓的 selector：

```cpp
typename ContourCombiner::EdgeSelectorType &edgeSelector =
    contourCombiner.edgeSelector(int(contour - shape.contours.begin()));
...
edgeSelector.addEdge(*edgeCache++, prevEdge, curEdge, nextEdge);
```

最后 `return contourCombiner.distance();`。**关键事实：每个轮廓有独立的 selector（见下），合并器才是决定“如何跨轮廓组合”的地方。**

- `SimpleContourCombiner::edgeSelector(int)` 忽略索引，所有轮廓共用**同一个** selector（`core/contour-combiners.cpp:41-44`）。
- `OverlappingContourCombiner::edgeSelector(int i)` 返回 `edgeSelectors[i]`，**每轮廓一个**（`core/contour-combiners.cpp:72-75`），构造时缓存每个轮廓的 `winding()`（`:57-63`）。

`TrueDistanceSelector::addEdge/merge` 都用 `SignedDistance::operator<` 取最小，而该比较**先比较 `fabs(distance)`、其次用 `dot` 破平**（`core/SignedDistance.hpp:22-37`）。这正是“扁平最近边”语义，也是 legacy 行为的内核。

---

## 3. `SimpleContourCombiner`：跨轮廓的“普通最近边”

```cpp
template <class EdgeSelector>
EdgeSelector &SimpleContourCombiner<EdgeSelector>::edgeSelector(int) { return shapeEdgeSelector; }
template <class EdgeSelector>
... SimpleContourCombiner<EdgeSelector>::distance() const { return shapeEdgeSelector.distance(); }
```

（`core/contour-combiners.cpp:41-51`）

所有轮廓的边进入同一个 selector，结果 = 全体边中 `|signedDistance|` 最小者。**它不看 winding，不区分填充/非填充边界。** 因此：

- 对**嵌套孔**（外正内负）：孔的外侧与真实边界一致，通常正确（§7）。
- 对**同向重叠**：某轮廓的“隐藏内部边”常常 `|距离|` 更小，于是胜出，给出**错误符号**（§5、§8）。

---

## 4. `OverlappingContourCombiner`：按 winding 分“内/外边界”

核心算法 `core/contour-combiners.cpp:78-133`。逐段解释：

### 4.1 分类（`:86-93`）

```cpp
DistanceType edgeDistance = edgeSelectors[i].distance();
shapeEdgeSelector.merge(edgeSelectors[i]);          // 全体轮廓：普通最近边
if (windings[i] > 0 && resolveDistance(edgeDistance) >= 0)
    innerEdgeSelector.merge(edgeSelectors[i]);      // “内边界”候选
if (windings[i] < 0 && resolveDistance(edgeDistance) <= 0)
    outerEdgeSelector.merge(edgeSelectors[i]);      // “外边界”候选
```

- **内边界候选**：正 winding 且该轮廓对当前点给出 **d≥0**（点在轮廓内）。
- **外边界候选**：负 winding 且 **d≤0**（点在“洞”内）。
- `resolveDistance`：单通道恒等；`MultiDistance` 取三通道中值（`core/contour-combiners.cpp:26-33`）。

> 注意：这里 `d≥0`/`d≤0` 的符号直接来自 §2.1 的**边界方向**。所以 **winding/方向决定了某轮廓被分到哪一类**。

### 4.2 主分支（`:95-122`）

令 `shapeDistance`=普通最近边，`innerScalar`/`outerScalar`=内/外候选的解析标量：

- **内分支**（`innerScalar>=0 && |inner|<=|outer|`，`:104-112`）：
  `distance = innerDistance`，`winding=+1`；随后在所有正 winding 轮廓中，**挑出同号且“最外”（值最大）** 的轮廓距离：
  ```cpp
  if (fabs(resolveDistance(contourDistance)) < fabs(outerScalarDistance)
      && resolveDistance(contourDistance) > resolveDistance(distance))
      distance = contourDistance;
  ```
- **外分支**（`outerScalar<=0 && |outer|<|inner|`，`:113-121`）：对称地取最负者。
- **否则**（`:122-123`）：`return shapeDistance;`（退回普通最近边）。

### 4.3 异号修补与回退（`:126-132`）

```cpp
for (int i = 0; i < contourCount; ++i)
    if (windings[i] != winding) {
        DistanceType contourDistance = edgeSelectors[i].distance();
        if (resolveDistance(contourDistance)*resolveDistance(distance) >= 0
            && fabs(resolveDistance(contourDistance)) < fabs(resolveDistance(distance)))
            distance = contourDistance;   // 相反 winding 但同号、更近者胜出（洞切入岛）
    }
if (resolveDistance(distance) == resolveDistance(shapeDistance))
    distance = shapeDistance;
```

**这套逻辑的本质**：先用 winding+符号把“哪些轮廓在当前方向上是真实填充分界”筛出来，再在这些候选里挑“最外/最深的同号边界”。它解决同向重叠的符号问题，但**不是精确并集 SDF**——见 §5 的 `+1.5 vs +1.5811`。

---

## 5. 三类重叠几何的行为

以下所有数值由**用仓库源码在 `/tmp` 编译的探针**实测（§9），并与暴力采样得到的“精确并集 SDF”对照。

### 5.1 嵌套孔（annulus：外 `[0,10]²` 正，孔 `[3,7]²` 负）

| 点 | `Simple` | `Overlap` | 暴力精确 | 判定 |
|---|---|---|---|---|
| 环 (2,5) | +1 | +1 | +1 | 全对 |
| 孔心 (5,5) | −2 | −2 | −2 | 全对 |
| 外部 (12,5) | −2 | −2 | −2 | 全对 |

机制：环内 `(2,5)`，外轮廓 d=+2（正 winding，内候选）、孔 d=+1（但孔是负 winding 且 d>0，不进入外候选），主分支选 `+1`（离孔更近）。孔心 `(5,5)`，外 d=+5（内候选）、孔 d=−2（负 winding 且 d≤0，外候选），走外分支得 `−2`。**嵌套孔是 `Simple` 也能处理的“正常”情形**，因为负 winding 孔的边界方向本就与真实填充分界一致。

### 5.2 同向重叠（A=`[0,6]²` 正，B=`[4,10]²` 正）

| 点 | `Simple`/legacy | `Overlap` | 暴力精确 | 备注 |
|---|---|---|---|---|
| 重叠中心 (5.5,5.5) | **+0.5** | **+1.5** | +1.5811 | `Simple` 取到 A 的隐藏上/右边；`Overlap` 取 B 的隐藏下/左边 |
| 仅 A (2,2) | +2 | +2 | +2 | 一致 |
| 仅 B (7,7) | **−1.414** | **+3** | +3 | `Simple` 把 B 的边当最近边 → 错号 |
| A 隐藏右缘右侧 (6.1,5) | **−0.1** | **+1.0** | +1.0 | **反例**（§8） |
| 并集外空隙 (7.5,1) | −1.5 | −1.5 | −1.5 | 一致（确实在外部） |

机制（重叠中心）：A、B 都是正 winding 且都 d≥0 → 均入内候选；主分支先取 `innerScalar=min=+0.5`，再在正 winding 中取“最外” → `+1.5`（B）。可见 **`Overlap` 修正了符号并避开了 A 的隐藏边，但结果是分轴近似的并集边界，并非精确欧氏并集距离（1.5≠1.5811）**。这属于该算法的已知近似，不影响“符号正确”的核心结论。

### 5.3 独立岛（两个 `+` 方形 `[0,4]²`、`[8,12]²`）

| 点 | `Simple` | `Overlap` | 精确 |
|---|---|---|---|
| 岛间 (6,2) | −2 | −2 | −2 |
| 左岛内 (2,2) | +2 | +2 | +2 |

机制：两轮廓都对 `(6,2)` 给出负距离且 winding 为正 → 既非内候选也非外候选，两个标量都保持 `-DBL_MAX`，走 `else` 分支 `return shapeDistance`（`:122-123`）。**独立岛时 `Overlap` 退化为普通最近边，不会误伤。**

---

## 6. 边界方向与填充规则如何改变结果

### 6.1 边界方向（winding）改变“原始符号”与“分类”

- 把外轮廓反向为 −1、孔设 +1：环内 `(2,5)` 原始 `Overlap = −1`、`Simple = −1`（因为内部按负 winding 记负）。
- 把“同向嵌套”的内层反向（`Contour::reverse()`，`core/Contour.cpp:83+`）：`(2,5)` 从 `Simple −1 / Overlap +2` 变为 `Simple +1 / Overlap +1`，即内层由“岛”变为“孔”，符号与候选结构随之改变。

方向与 winding 的一致化由 `Shape::orientContours`（`core/Shape.cpp:144-198`）在预处理中完成；`resolveShapeGeometry` 也会调用它（`ext/resolve-shape-geometry.cpp:117-128`）。

### 6.2 填充规则只在“符号修正阶段”生效

`distanceSignCorrection`（`core/rasterization.cpp:19-32`）逐像素用 `Scanline::filled(..., fillRule)` 重算填充，并把**整段距离**按 `2*zero − sd` 翻转（并非只改符号位）：

```cpp
bool fill = scanline.filled(projection.unprojectX(x+.5), fillRule);
if ((sd > sdfZeroValue) != fill) sd = doubleSdfZeroValue - sd;
```

`interpretFillRule`（`core/Scanline.cpp:13-24`）：
- `FILL_NONZERO`：累计 winding ≠ 0 为填充；
- `FILL_ODD`：累计 winding 为奇数；
- `FILL_POSITIVE` / `FILL_NEGATIVE`：累计 >0 / <0。

`Scanline::preprocess`（`:66-75`）把各交点方向做前缀和，得到每个区间的累计 winding。

实测同向重叠中心 `(5.5,5.5)`（原始 `Overlap=+1.5`）：

| 修正 | 结果 | 原因 |
|---|---|---|
| 无修正 | +1.5 | 原始符号（两轮廓都 +） |
| `nonzero` + 修正 | +1.5 | 累计 winding=2≠0 → 填充 |
| `odd` + 修正 | **−1.5** | 累计 winding=2 → 偶 → 非填充，翻转 |

即**同一几何、同一边界方向，仅因 `FillRule` 不同，最终符号可以相反**。而 `Simple` 在同样点上给出 `+0.5 / −0.5`，因为其原始幅值本身就是错的（取到了隐藏边）。

---

## 7. 责任边界：支持重叠的路径 vs legacy 生成路径

| 维度 | legacy（`generate*_legacy`） | 现代 `Simple`（`overlapSupport=false`） | 现代 `Overlap`（`overlapSupport=true`） |
|---|---|---|---|
| 跨轮廓组合 | 扁平 `|距离|` 最小（`core/msdfgen.cpp:179-181`） | 扁平 `|距离|` 最小（共享 selector） | 按 winding 分内/外候选（`contour-combiners.cpp:86-133`） |
| 是否感知 winding | 否 | 否 | 是（构造时缓存，`:57-63`） |
| 隐藏内部边 | 会作为普通最近边获胜 | 同左 | 通过分类排除大部分 |
| 同向重叠符号 | 错 | 错 | 对（近似幅值） |
| 填充规则 | 不参与（需外部 `distanceSignCorrection`） | 同左 | 同左 |

**关键分工**：

1. **预处理负责“消除重叠”**：默认 Skia 构建走 `FULL_PREPROCESS`（`main.cpp:577-583`），调用 `resolveShapeGeometry` 把自交/重叠解析为合规几何（`ext/resolve-shape-geometry.cpp:117-128`），于是 `overlapSupport` 与扫描线都**默认关闭**（`main.cpp:586-587`：`generatorConfig.overlapSupport = geometryPreproc == NO_PREPROCESS;`）。CHANGELOG 1.8 明确：预处理使重叠模式与扫描线“obsolete”并被默认禁用（`CHANGELOG.md:88-91`）。
2. **无预处理时才由 `Overlap`+扫描线兜底**：非 Skia 构建默认 `NO_PREPROCESS` → `overlapSupport=true`、`scanlinePass=true`（同两行）。
3. **符号最终由扫描线阶段按 `FillRule` 定夺**：`main.cpp:1327-1343` 在生成后调用 `distanceSignCorrection`（单/多通道），这才是“填充规则”真正落地的地方。生成器本身只给出“按边界方向的原始有符号距离”。
4. **legacy 是“原算法保底”**：由 `-legacy` 切到 `generate*_legacy`（`main.cpp:1287/1307`），其 MSDF 还强制 `MSDFGeneratorConfig(false, ...)`（即误差修正也走非重叠，`core/msdfgen.cpp:272-273/331-332`）。`msdfgen.h:70-72` 明确其“不能处理重叠轮廓”。
5. **误差修正复用同一 Combiner 作为距离预言器**：`msdf-error-correction.cpp:41-44` 按 `config.overlapSupport` 在 `OverlappingContourCombiner`/`SimpleContourCombiner` 间切换，保证“预测的精确距离”与生成路径一致。

---

## 8. 反例：把重叠误当成普通最近边

**几何**：A=`[0,6]²`（正）、B=`[4,10]²`（正），二者同向重叠。考察点 **P=(6.1, 5.0)**，它位于**并集内部**（y=5∈[4,10]、x=6.1∈[4,10] → 属于 B）。

**真实语义**：`nonzero` 填充下 P 在形状内；到并集边界的最近距离是 B 的下边 `y=4` → **+1.0**。

**legacy/`Simple` 的实际行为**：扁平 `|距离|` 最小者是 A 的隐藏右边 `x=6`，距离 0.1，且因 P 在 A 外而带负号 → 返回 **−0.1**。

```
(x, y)       填充分类   legacy/Simple   Overlap   精确并集 SDF
(6.1, 5.0)   内(union)     −0.1000      +1.0000     +1.0000
(7.0, 7.0)   内(union)     −1.4142      +3.0000     +3.0000
(5.5, 5.5)   内(union)     +0.5000      +1.5000     +1.5811
```

- `(6.1,5.0)`：`Simple` 把**不该存在的内部边**当成普通最近边，既错符号又错幅值（0.1 而非 1.0）。
- `(7.0,7.0)`：`Simple` 给出 `−1.4142`（对 B 的两条隐藏边取直角距离），而真实应为 `+3.0`（到 B 的右边 `x=10` 或上边 `y=10`）——**符号完全颠倒**。
- `Overlap` 在前两点给出正确的符号与幅值；在重叠中心给出 `+1.5`（符号正确，幅值为分轴近似，略小于精确 `1.5811`）。

这正是任务所述“把重叠误当成普通最近边”的典型错误：**仅最小化 `|signedDistance|` 的扁平策略，无法区分“真实填充分界”和“被另一轮廓覆盖掉的隐藏边”。**

附带地，`MSDF` 三通道路径同样中招。用真实 `edgeColoringSimple` 上色后测（探针）：

```
(6.1,5.0) : Simple  rgb=(+1.000,−0.100,−0.100) med=−0.100   Overlap rgb=(+1.000,+2.100,+1.000) med=+1.000
(7.0,7.0) : Simple  rgb=(−1.000,−1.000,−1.000) med=−1.000   Overlap rgb=(+3.000,+3.000,+3.000) med=+3.000
```

即 `Simple` 的**中值**在重叠内部为负（渲染为“外部”），`Overlap` 修正为正。

---

## 9. 有限几何场景的预期距离/符号（汇总表）

统一约定：形状 y 轴向上，逆时针为正 winding，`FillRule=nonzero`，距离为“到该几何真实填充分界的欧氏距离”，正=内部。

| # | 场景 | 点 | 期望 SDF（精确） | `Simple`/legacy 原始 | `Overlap` 原始 | `Overlap`+扫描线(nonzero) |
|---|---|---|---|---|---|---|
| 1 | annulus 外`[0,10]²`+ / 孔`[3,7]²`− | (2,5) | +1 | +1 | +1 | +1 |
| 1 | 同上 | (5,5) 孔心 | −2 | −2 | −2 | −2 |
| 2 | 同向嵌套 外`[0,10]²`+ / 内`[3,7]²`+ | (2,5) | +2 | **−1** | +2 | +2 |
| 2 | 同上 | (5,5) | +5 | +2 | +5 | +5 |
| 3 | 同向重叠 A`[0,6]²`+ / B`[4,10]²`+ | (6.1,5) | +1.0 | **−0.1** | +1.0 | +1.0 |
| 3 | 同上 | (7,7) | +3.0 | **−1.414** | +3.0 | +3.0 |
| 3 | 同上 | (5.5,5.5) | +1.5811 | **+0.5** | +1.5 | +1.5（`odd` 规则下变 −1.5） |
| 4 | 独立岛 `[0,4]²`+ / `[8,12]²`+ | (6,2) | −2 | −2 | −2 | −2 |
| 5 | 退化：外+ / 两个同号嵌套孔 | (6,6) | +2 | **−2** | **−4** | +4（符号对、幅值偏大） |
| 6 | 外反向(−) / 孔(+) 环 | (2,5) | +1 | −1 | −1 | +1 |

全部数值由探针实测，并与暴力边界采样（在真实填充/非填充交界面附近二分）交叉验证；“期望精确”列对重叠中心为 `1.5811`，`Overlap` 给出 `1.5`，差异已在 §5.2 说明。

---

## 10. 测试/证据说明（如实交代）

- **仓库内没有单元测试套件**：`CMakeLists.txt` 与 `cmake/` 中不存在 `enable_testing`/`add_test`/`CTest`，`core/`、`ext/`、`main.cpp` 中也没有 `*_test`/`gtest`/`catch` 之类用例。故“测试证据”只能来自以下三类：
  1. **源码内注释与契约**：`msdfgen.h:70-72`（legacy “cannot deal with overlapping contours”）；`core/generator-config.h:51-52`（`overlapSupport` 语义）；`core/contour-combiners.h:27`（`OverlappingContourCombiner` “Selects the nearest contour that actually forms a border between filled and unfilled area”）。
  2. **内建可运行预言器**：`core/sdf-error-estimation.cpp` 用 `Scanline::overlap` 对比栅格化与 SDF 渲染的填充差（`:135-151`），`main.cpp:1007` 的 `-estimateerror` 与 `main.cpp:983` 的 `-testrender` 是仓库自带的端到端口径，可作为裁判，但不是回归测试。
  3. **本次复现探针**：在 `/tmp/msdf-probe` 下用仓库 `core/*.cpp` 直接编译（含 `Contour/Shape/edge-segments/edge-selectors/contour-combiners/equation-solver/EdgeHolder/convergent-curve-ordering/Scanline/edge-coloring`），分别调用 `ShapeDistanceFinder` 的各 Combiner、`generateSDF`/`generateSDF_legacy`、`distanceSignCorrection`，并另写暴力采样器核对精确并集距离。探针**只读**仓库源码，全部产物在 `/tmp`，未改仓库、未起服务。
- **`CHANGELOG` 作为历史证据**：1.4 引入重叠合并逻辑与 `_legacy` 保留（`CHANGELOG.md:137`）；1.8 说明预处理令重叠模式/扫描线默认关闭（`CHANGELOG.md:88-91`）。

---

## 11. 实践含义

- **若输入已由 Skia 预处理消除重叠**（默认构建）：用 `overlapSupport=false` 即可，`Overlap` 的额外开销无收益；最终符号由预处理+（可选）扫描线保证。
- **若输入可能含同向重叠且无预处理**（非 Skia 或 `-nopreprocess`）：必须用 `Overlap` + 扫描线符号修正，否则会出现 §8 的符号翻转；但需知 `Overlap` 的幅值是分轴近似，深重叠区（如 `(5.5,5.5)`）可能略小于精确并集距离。
- **嵌套孔彼此同号**（异常输入）：`Overlap` 可能给出比 `Simple` 更大的幅值（§9 场景 5），符号仍依赖扫描线阶段按 winding 纠正。
- **MSDF 的误差修正**会复用与生成相同的 Combiner（`core/msdf-error-correction.cpp:41-44`），所以切换 `overlapSupport` 时两条路径保持一致，不会互相“打架”。

---

### 附：探针复现要点（可在任意只读副本外重建）

```bash
# 1) 生成一个最小 msdfgen-config.h 到 /tmp 的私有 include 目录
# 2) 编译仓库 core/*.cpp + 自定义 main，链接 ShapeDistanceFinder 等
g++ -std=c++11 -DMSDFGEN_USE_CPP11 -I/tmp/msdf-probe/inc -I<repo> probe.cpp \
    <repo>/core/{Contour,Shape,edge-segments,edge-selectors,contour-combiners,\
equation-solver,EdgeHolder,convergent-curve-ordering,Scanline,edge-coloring}.cpp -o probe
```

其中方形轮廓用四条 `EdgeSegment::create` 线性边构造，正/负 winding 由顶点顺序 `(x0,y0)→(x0,y1)→(x1,y1)→(x1,y0)` / `(x0,y0)→(x1,y0)→(x1,y1)→(x0,y1)` 控制；这与 `Contour::winding()`（鞋带）实测一致（见 §4.1 的 winding 打印）。
