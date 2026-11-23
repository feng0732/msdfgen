# msdfgen 边缘着色的通道分派：实现分析与可重复性报告

> 分析对象：仓库快照 `task/107`（`ec84cd5 Version 1.13.1`）。
> 仓库源码未作任何修改。所有实验均在 `/tmp` 下用仓库自带的 `build/libmsdfgen-core.a`（或由仓库 `core/*.cpp` 原样重新编译的目标文件）驱动，未改动仓库文件。
> 仓库**没有自动化测试**：`CMakeLists.txt` 中无 `enable_testing`/`add_test`，无 gtest/catch2/doctest，也无 CI 测试文件。因此下文所说的“已有测试证据”只指仓库**内建的验证设施**（`estimateSDFError`、`-testrender`/`-testrendermulti`、`-estimateerror`）以及以仓库代码为 oracle 的行为实验；没有任何既有断言测试可以引用。

---

## 0. 结论摘要

1. **RGB 分派是“通道掩码（bitmask）”语义，不是“每边一种颜色”。** `EdgeColor` 是 3 位掩码（`core/EdgeColor.h:7-17`）。生成器把每条边的掩码写进 `edge->color`；`MultiDistanceSelector` 再按位决定该边进入 R/G/B 中哪几个独立选择器（`core/edge-selectors.cpp:174-222`）；中值重构对三个通道距离取中值（`core/contour-combiners.cpp:30-32`、`core/msdfgen.cpp:32-34`）。
2. **“尖角被锁住”的精确条件是：角两侧两条边的掩码交集恰为 1 个通道。** 三条策略产出的颜色都落在 `{YELLOW, CYAN, MAGENTA}` 这个两两交集恰为 1 位的三色集里（`switchColor` 的性质）；错误修正的角保护也恰好以“公共色为单比特”为判据（`core/MSDFErrorCorrection.cpp:127-129`）。因此本报告把“同掩码角”（交集 2 位或完全相同）定义为**错误分派**。
3. **角度阈值只有一条判定式**：`isCorner(aDir,bDir,sinθ) = dot≤0 || |cross|>sinθ`（`core/edge-coloring.cpp:23-25`），对单位向量等价于“转角 > θ”（θ∈(0,π)）。默认 θ=3.0 rad=171.887°，即**只有转角 > 8.113° 才算角**（`main.cpp:32`）。`-angle` 的 CLI 帮助称其为“夹角/最小角”，`edge-coloring.h:12` 注释称其为“max angle”，二者互补且 CLI 文案方向相反——属可解释性缺陷。
4. **随机种子是全局、跨轮廓共享的抽取器**（`core/edge-coloring.cpp:38-58`）。同一条轮廓的颜色取决于它在 `shape.contours` 中的位置及其前面轮廓消耗的 bit/trit；distance 策略的 `colorSecondDegreeGraph` 还有 **seed bit 复用**（同一 2 进制位既判 case 3，又在 case 5 取反使用）。
5. **可重复性**：对“同一 `Shape` 对象、同一输入边序列、同一种子”，重复运行幂等；但**首次运行与后续运行可能不同**——首次运行可能用 `splitInThirds` 把边从 2 条拆成 6 条，改变后续角点/分派输入。cross-contour 顺序、顶点旋转也会改变结果（distance 尤甚）。
6. **发现一处会真实产生的错误分派**：在零距离图不可 3 着色、或 `colorSecondDegreeGraph` 因 `possibleColors==0` 静默落空时，distance 策略会产出**尖角两侧掩码相同**的着色。此时 `protectCorners` 检测不到该角（它靠掩码交集是否为单比特来判角），错误修正会把该处三通道等化（`core/MSDFErrorCorrection.cpp:460-476`），尖角被抹圆。
7. 给出**两个有限反例**（§7）：CE1（五边形边界 P 点上接一个尖刺；尖刺两侧同为 `YELLOW`，而零距离图本身**可 3 着色**，是启发式贪心失败）与 CE2（K 个同向三角形共顶点；零距离图整体不可 3 着色，但“每个尖角两侧不同”这一可满足目标也未达成）。两者都能稳定复现，并与人工可行着色对照。

---

## 1. 数据模型：掩码、通道、中值重构

### 1.1 掩码定义（代码事实）

`core/EdgeColor.h:7-17`：

```cpp
enum EdgeColor {
    BLACK = 0, RED = 1, GREEN = 2, YELLOW = 3,
    BLUE = 4, MAGENTA = 5, CYAN = 6, WHITE = 7
};
```

颜色名只是 3 位掩码的别名；`RED|GREEN=YELLOW` 之类语义由位运算给出。三条策略通过 `switchColor` 只在 `{CYAN(110), MAGENTA(101), YELLOW(011)}` 之间轮换（distance 最后也映射到这一组，`core/edge-coloring.cpp:522`），因此普通边总是 2 位掩码；唯一的 3 位边是泪滴分支里被刻意固定为 `WHITE(111)` 的那一段。**这三色两两交集恰为 1 位**（C∩M=BLUE、C∩Y=GREEN、M∩Y=RED），这是后面所有“尖角可锁”性质的来源。

### 1.2 通道分派（代码事实）

`MultiDistanceSelector::addEdge`（`core/edge-selectors.cpp:174-222`）：

```cpp
if ((edge->color&RED   && r.isEdgeRelevant(...)) ||
    (edge->color&GREEN && g.isEdgeRelevant(...)) ||
    (edge->color&BLUE  && b.isEdgeRelevant(...))) {
    SignedDistance distance = edge->signedDistance(p, param);
    if (edge->color&RED)   r.addEdgeTrueDistance(edge, distance, param);
    if (edge->color&GREEN) g.addEdgeTrueDistance(edge, distance, param);
    if (edge->color&BLUE)  b.addEdgeTrueDistance(edge, distance, param);
    ...
}
```

每个通道维护自己的 `PerpendicularDistanceSelectorBase`，互不影响。因此“一条边属于哪些通道”完全由掩码决定，**不存在主色/副色，也不存在先分派再覆盖**。

写出端：`DistancePixelConversion<MultiDistance>` 把 `distance.r/g/b` 依次写进像素 0/1/2（`core/msdfgen.cpp:30-34`），即 **像素 R←r、G←g、B←b**。重构端 `median(r,g,b)`（`core/render-sdf.cpp:77,88`；README 的 GLSL 示例同）。

### 1.3 中值重构与错误修正的耦合（代码事实 + 推导）

- `protectCorners` 用 `prevEdge->color & edge->color` 是否为**单比特**判断该接点是不是角（`core/MSDFErrorCorrection.cpp:127-129`）：
  ```cpp
  int commonColor = prevEdge->color&(*edge)->color;
  if (!(commonColor&(commonColor-1))) { /* 标记角周围 4 个 texel 为 PROTECTED */ }
  ```
  若两侧掩码相同（2 位），`commonColor` 有 2 位，条件为假，**不保护**。
- `apply` 对判定为错误的像素做三通道取中值（`core/MSDFErrorCorrection.cpp:460-476`），等价于把该局部单色化，尖角被永久抹圆。

因此**合理解释不变式**：尖锐角两侧边应满足 `popcount(mask_a & mask_b) == 1`；simple/inktrap 由三色 switch 集合天然满足，distance 在图着色成功时也满足，失败时破坏。

---

## 2. 共用的角点识别与角度阈值

### 2.1 判定式（代码事实）

`core/edge-coloring.cpp:23-25`：

```cpp
static bool isCorner(const Vector2 &aDir, const Vector2 &bDir, double crossThreshold) {
    return dotProduct(aDir, bDir) <= 0 || fabs(crossProduct(aDir, bDir)) > crossThreshold;
}
```

三条策略在各自“Identify corners”循环里都调用 `isCorner(prevDir.normalize(), curDir.normalize(), sin(angleThreshold))`（`core/edge-coloring.cpp:69,153,400`）。

- 单位向量下 `dot=cosφ`、`|cross|=|sinφ|`（φ 为方向改变角，0..π）。
- `dot≤0` ⟺ φ≥90°：**“回头”永远算角，与阈值无关**。
- 否则 `|sinφ|>sinθ` ⟺ φ>θ（θ∈(0,π)）。故**角 = 转角严格大于 θ**。
- `-angle` 解析器 `parseAngle` 支持以 `d` 结尾的度数（`main.cpp:99-110`），内部统一换算成弧度；CLI 限制 θ<π。

### 2.2 实验证据（复现）

以 `u=(1,0)`、`v=(cos t, sin t)` 调用同一判定式（脚本复刻该语义，等价于库内行为）：

| 转角 t | θ=0.1 | θ=0.5 | θ=1.0 | θ=3.0（默认） |
|---|---|---|---|---|
| 5° | 否 | 否 | 否 | 否 |
| 8° | **是** | 否 | 否 | 否 |
| 9° | 是 | 否 | 否 | **是** |
| 30° | 是 | **是** | 否 | 是 |
| 60° | 是 | 是 | **是** | 是 |
| 89° | 是 | 是 | 是 | 是 |

默认 θ=3.0 的等价“最小转角”为 `180° − 3.0·180/π = 8.113°`，上表 8°/9° 的分界与之吻合。

> **可解释性缺陷（代码事实）**：`main.cpp:426-427` 把 `-angle` 描述为“minimum angle between adjacent edges”，实际却是“方向改变角（转角）阈值”，两者互补。按 CLI 文案设置会得到相反效果；`edge-coloring.h:12` 注释 “maximum angle ... for example 3 (~172 degrees)” 才与代码一致。

### 2.3 近共线短边与退化边（代码事实 + 经验）

- 近共线短边只要转角 ≤ θ 就被并入同一 spline（simple/inktrap 的区间同色；distance 的同一 spline），不改变分派结构。
- **零长度边是真正的危险输入**：`Vector2::normalize(false)` 对零向量返回 `(0,1)`（`core/Vector2.hpp:42-46`），所以零长度边在角点识别里贡献一个**人为 (0,1) 方向**，可能凭空产生/吞掉角。
- 读入端会把“三点共线”的二次曲线降级为直线（`core/edge-segments.cpp:13-17`），形状描述里的近共线短二次段可能直接坍缩。

---

## 3. 三条策略的分派机制

### 3.1 共用的随机抽取器（代码事实）

`core/edge-coloring.cpp:38-58`：

```cpp
static int seedExtract2(unsigned long long &seed) { int v = int(seed)&1; seed >>= 1; return v; }
static int seedExtract3(unsigned long long &seed) { int v = int(seed%3); seed /= 3; return v; }
static EdgeColor initColor(unsigned long long &seed) {
    static const EdgeColor colors[3] = { CYAN, MAGENTA, YELLOW };
    return colors[seedExtract3(seed)];
}
static void switchColor(EdgeColor &color, unsigned long long &seed) {
    int shifted = color<<(1+seedExtract2(seed));
    color = EdgeColor((shifted|shifted>>3)&WHITE);
}
```

穷举确认 `switchColor` 只在三色间轮换：`YELLOW→CYAN/MAGENTA`、`MAGENTA→YELLOW/CYAN`、`CYAN→MAGENTA/YELLOW`（取决于 1 个 seed bit）。种子是单条 `unsigned long long`，**在 `shape.contours` 循环中被持续消耗**，所以同一条轮廓的颜色取决于它前面有多少轮廓、消耗了多少 bit/trit。

### 3.2 `edgeColoringSimple`（简单着色）

流程（`core/edge-coloring.cpp:68-142`）：

1. 收集角点下标 `corners`。
2. **无角**：`switchColor` 一次，整轮廓同色。
3. **单角**（“泪滴”，`corners.size()==1`）：
   - 三色 `colors[0]`、`colors[1]=WHITE`、`colors[2]`；`colors[0]` 与 `colors[2]` 各由一次 `switchColor` 得到（穷举 1e6 种子确认二者永不相同），`colors[1]` 固定 `WHITE`。
   - 边数 ≥3：以角为首下标，第 i 条边取 `colors[1+symmetricalTrichotomy(i,m)]`，`symmetricalTrichotomy(p,n)=(3p+1)/n−1`（`edge-coloring.cpp:19-21`）。对 n 求和恒为 0（已验 n=3..11），把 m 条边按“前/中/后”三等分（含余数分配）。于是角的两个 flank 分别是 `edge[corner-1]`→`colors[2]`、`edge[corner]`→`colors[0]`，二者交集恰 1 位。
   - 边数 1 或 2：`splitInThirds` 把每条边拆成 3 段（2 条边共 6 段），再按**成对**赋色：`parts[0..1]=colors[0]`、`parts[2..3]=colors[1]`、`parts[4..5]=colors[2]`（`edge-coloring.cpp:107-121`）。2 边情形（corner=0）得到 `colors[0] colors[0] colors[1] colors[1] colors[2] colors[2]`，即实测的 `M M W W Y Y`。
4. **多角**：从 `corners[0]` 起沿轮廓遍历，每跨一个角 `switchColor(color, seed, banned)`；banned 在最后一个角回填 `initialColor`（`edge-coloring.cpp:125-140`）。带 banned 的分支：若 `color&banned` 恰为单通道，则取 `(color&banned)^WHITE`（保证 ≠ banned）；否则走普通 switchColor。穷举确认 banned 分支永不会落在 banned 上。

### 3.3 `edgeColoringInkTrap`（墨迹陷阱着色）

流程（`core/edge-coloring.cpp:151-258`）：

1. 角点识别同 simple，但额外记录**每条角点前的 spline 长度** `prevEdgeLengthEstimate`（沿轮廓累加 `estimateEdgeLength`，4 段折线近似，`core/edge-coloring.h:6`、`core/edge-coloring.cpp:27-36`）。
2. 无角 / 单角分支与 simple 完全一致（含 m<3 的 `splitInThirds`）。
3. 多角且 `cornerCount>3`：**墨迹陷阱判定**——若角 i 前段长 > 角 i+1 前段长 **且** 角 i+1 前段长 < 角 i+2 前段长，则角 i 标为 `minor`（`edge-coloring.cpp:217-228`）。这就是头文件“短于两侧邻边的边”规则。
4. 先把所有 major 角用 `switchColor(..., banned)` 赋色（banned 由最后一个 major 角回填，避免首尾相撞，`edge-coloring.cpp:229-238`）；再给 minor 角赋 `(当前色 & 下一个角颜色) ^ WHITE`（`edge-coloring.cpp:239-245`）。
5. 按遍历顺序把角的颜色铺到边段（`edge-coloring.cpp:246-255`）。

**代码事实 / 推断**：
- minor 的补色保证 minor 角两侧掩码交集为单比特（实测 quill 星形上 minor 角均被正确区分），因此 ink trap 的颜色代数本身是自洽的。
- 但 minor 判定依赖**折线近似的长度**与**严格不等号**；理论上在近似相等长度上会抖动。本报告在中/长边比 0.28 vs 1.0 的用例及 1e-4 级扰动下**未复现**抖动（见 §6 证据缺口，属理论风险而非代码事实）。

### 3.4 `edgeColoringByDistance`（按距离着色，实验性）

源码头注释 `core/edge-coloring.cpp:260` 明确写 `EXPERIMENTAL IMPLEMENTATION - WORK IN PROGRESS`。

流程（`core/edge-coloring.cpp:395-529`）：

1. 角点识别同 simple；把**两个角之间的每段 spline** 作为一个图顶点。单角轮廓在 `i==m/2` 处**再插一个 spline 起点**（`edge-coloring.cpp:425-426`），把泪滴拆成两段参与图着色；单角且 m<3 时拆分后按 `parts[0,1]`/`parts[2,3]`/`parts[4,5]` 分组。
2. 对每对 spline 计算 `edgeToEdgeDistance`（双向各采样 `EDGE_DISTANCE_PRECISION=16` 点，`edge-coloring.cpp:264-280`）。
3. 把**距离恰为 0** 的 spline 对作为硬约束图边；其余按距离升序逐条 `tryAddEdge`。
4. 先对零距离图做 `colorSecondDegreeGraph`（贪心，`edge-coloring.cpp:292-325`），再逐步加正距离边，用 `tryAddEdge` 做有限步局部重着色（`MAX_RECOLOR_STEPS=16`）。
5. 把 spline 的 0/1/2 索引映射回 `{YELLOW,CYAN,MAGENTA}`（`edge-coloring.cpp:522`）。

**关键代码事实**：
- **零距离边是硬约束，但 `colorSecondDegreeGraph` 不保证 3 可着色。** `possibleColors==0` 时 `switch` 没有匹配分支，`color` 保持初值 0，**静默落空**（实验日志：`vertex 5 has possibleColors=0 ... assigned color 0`），该顶点即与某些零距离邻居同色。
- **seed bit 复用**：case 3 用 `seedExtract2(seed)`，case 5 用 `!seedExtract2(seed)<<1`，case 6 用 `seedExtract2(seed)+1`；同一轮循环最多消耗 1 bit，不同顶点持续抽取。总体可重现，但并非逐顶点独立随机。
- **`tryAddEdge` 失败会回滚**该边（`edge-coloring.cpp:382-386`），正距离边可能被静默放弃；实验中 12 边的 pinch 有 7/66 条正距离边被拒（MAX_RECOLOR_STEPS 用尽）。
- **硬约束阈值是精确 0**：只有 `edgeToEdgeDistance` 返回 0 才进硬约束图；16 点采样意味着“非常接近但非 0”的两条边不会进图，可能被分到同色。

### 3.5 三策略差异（实验证据）

同一薄楔形（3 条边、2 个尖角）下三策略给出不同色序（如 `M Y C` vs `Y C M`），但都满足“相邻边掩码交集为单比特”。差异只在“选色序列”，角点识别是共享的。

---

## 4. 单尖角、多尖角、近平行短边与重复运行

### 4.1 单尖角（泪滴）

用真实三次曲线的 2 边泪滴（`A(0,0)→B(2,0)` 直线 + `B→ctrl1(2.6,0)→ctrl2(0.5,0.6)→A` 三次，只有 A 为尖角）：

- **三策略都把 2 条边拆成 6 条**，得到形如 `M M W W Y Y`（simple/inktrap）或 `Y Y W W C C`（distance seed0）的分段。中间两段 `WHITE`（3 通道），两侧各 2 通道。
- 尖角 A 侧（末段）与角前一段（首段）掩码交集恰 1 位（如 `Y` 与 `M`），尖角被锁住。
- **种子行为**：simple 随种子在三循环排列间切换（seed 0..5 → `M,Y,C,Y,C,M` 的首色）；distance 因先经零距离图再映射，循环顺序不同。
- **repeated run**：对**同一 Shape 对象**连续调用，第 1 次后边数变 6，第 2/3 次结果与第 1 次一致（幂等）；但若对**新建对象**每次都跑，则“每次都是第一次”，故“重新构造再跑”可重复，“在已着色对象上再跑”第一次与后续可能不同（2→6 边）。这是实际的可解释性陷阱：增量改色会污染后续着色。

### 4.2 多尖角：阈值悬崖

默认 θ=3.0（8.113°）下规则 N 边形：

| 多边形 | 每角转角 | 角数 | simple seed0 结果 |
|---|---|---|---|
| 44 边形 | 8.1818° | 44 | 44 段颜色游程，`M Y C M Y C ...` 循环 |
| 45 边形 | 8.0000° | **0** | 全部同色（`M M M ...`） |
| 90 边形 | 4.0000° | 0 | 全部同色 |

44→45 边形只差 0.18°，输出却从“三色交替”翻转为“单色”。distance 在 45 边形上同样单色（无角 → 整轮廓一条 spline → `splineCount=1`）；inktrap 在 44 边形上未标记任何 minor（所有 spline 等长，严格不等号失败），但仍给出与 simple 不同的循环序列（seed 消耗模式不同）。

### 4.3 近乎共线短边

在正方形底边中段加一个宽 0.02、高按参数 `j` 缩放的对称小凸起（构造参数 `j` 不是真实转角；实测两处小转角约为 `0.995j` 与 `1.036j`，另加 4 个 90° 直角）。默认阈值 θ=3.0（8.113°）下：

| 构造参数 j | 两处小转角（实测） | 被识别角数 | simple seed0 颜色 |
|---|---|---|---|
| 5° | 4.98° / 5.19° | 4 | `M M M Y C Y` |
| 8° | 7.92° / 8.25° | 5（仅一处过阈值） | `M M Y C M Y` |
| 9° | 8.89° / 9.26° | 6 | `M Y C M Y C` |

8° 一档尤其能说明问题：两个本应“对称”的小转角只差 0.33°，却被阈值卡在两侧，导致只识别出一个角，颜色序列与 5°、9° 都不同。只差 1° 就会多切一个角，使三等分/换色节奏整体错位。**零长度边**更甚：在 45 边形（每角 8.000°<8.113°，原本 0 角、整轮廓同色）插入一条零长度边后，角数变 2，颜色游程从 1 变 3；在 4 顶点正方形上插入零长度边，使 `M Y C Y` 变成 `M Y Y C Y`。

### 4.4 同一轮廓重复运行

- **新建 Shape + 同 seed** 时三策略在同机同库下**逐位一致**。
- **seed 全局**：同一条三角形轮廓作为第 2 个轮廓放在 10 边形之后时，simple seed0 从 `M Y C` 变成 `C M Y`（distance → `Y C M`）。**轮廓顺序/前置轮廓数量改变 → 同一条轮廓颜色改变。**
- **顶点旋转**：4 叶 pinch 的顶点序列循环平移，simple 不变（对称），**inktrap 与 distance 随旋转改变**（minor 判定依赖遍历起点；distance 的贪心零距离图依赖顶点顺序）。distance 的“全局最优”其实是**顺序敏感**的近似。

---

## 5. 中值重构视角：为什么“同掩码角”是错误分派

- 每个通道取“该通道参与的边的最近垂直距离”再取中值。角两侧若掩码交集恰 1 位（如 `Y` 与 `M` 共享 R），则 R 通道看到两条边、G/B 各看到一条，中值在角两侧保持尖锐。
- 若两侧**掩码相同且同属三色集**（如两侧同为 `YELLOW`，交集 2 位 = R,G）：R、G 两通道都同时看到两条边，B 通道看不到任何边（`-DBL_MAX` 经 `DistanceMapping` 变成 `-inf`），中值为 `median(dist,dist,-inf)=dist`，**退化为这两条边联合的单通道距离**（即单通道 SDF），角不再被尖锐化。若两侧同为 `WHITE`（3 位），三通道都看到两条边，同样退化。
- `protectCorners` 只对“公共色为单比特”的接点保护（`core/MSDFErrorCorrection.cpp:127-129`），**同掩码角完全不被保护**；`apply` 随后会把该处三通道等化，尖角被抹圆。
- 因此 §7 的反例虽未必产生“中值符号翻转”，但**破坏了尖角可解释性**：同一几何只因掩码失误而变圆角，且 `protectCorners` 的 stencil 对该角为 0。这属于可稳定复现的错误分派。

---

## 6. 证据缺口（明确区分）

1. **没有既有自动化测试**：仓库无 `add_test`/gtest/CI 测试文件，无法引用“通过/失败的测试用例”；本报告用内建估计器 + 对照实验。
2. **`estimateSDFError` 对单像素尖角不敏感**：在反例上 produced 与 feasible 的差异极小（CE1 在 W=64/128/256 上约 0.0002% 量级；CE2 K=3/4 同量级），**不能**作为同掩码角的直接证据。直接证据是 stencil 审计与逐通道场对照（§7）。
3. **中值场未见符号差**：用 `OverlappingContourCombiner<MultiDistanceSelector>` 与 `TrueDistanceSelector` 逐点对照，produced 与 feasible 的**中值**在采样区域未出现符号差；但**逐通道**差异显著（CE1 seed5：`max|ΔR|=1.20, max|ΔG|=1.24, max|ΔB|=1.50`；CE2 K=3：`0.50/0.42/0.54`，K=4：`0.51/0.37/1.26`）。这是**证据缺口**：没有找到“同掩码角导致中值像素翻符号”的有限反例，只有通道场层面的破坏与尖角抹圆。
4. **跨平台逐位一致性未测**：`double` 累积与迭代顺序可能因编译器/CPU 不同；本报告只在 Linux + g++ 13.3 上验证。
5. **inktrap “严格不等号抖动”**：理论上存在，本报告 1e-4 级扰动未复现，判定为**未能证实的理论风险**。
6. **CLI 渲染未实际运行**：仓库当前构建为 core-only（无 standalone），`-testrender` 等未直接运行；本报告用等价的库内 API 与自建渲染循环验证。附带发现（代码事实）：单色（无角）轮廓的未用通道会写出 `-inf`，双线性插值两个 `-inf` texel 会得到 `NaN`（`median(-0.2f, NaN, -0.2f)` 仍为 `-0.2`，但 `median(NaN, -0.2f, -0.2f)` 为 `NaN`），属独立风险，不在主结论内。

---

## 7. 两个暴露错误通道分派的有限反例

两个反例都满足：可手算坐标、`Shape::validate()==true`、可由仓库库稳定复现，并配有“人工可行着色”作对照。

### 7.1 反例一：背景轮廓上的尖刺（CE1）

**几何**（两个轮廓，均 `winding==+1`）：

```
轮廓 0（五边形，P=(0,0) 是其左边上的共线“平滑”顶点）：
  L(0,-1) → P(0,0) → U(0,1) → V(1.2,1) → W(1.2,-1) → L(0,-1)
轮廓 1（尖刺三角形，顶点 P 处是真正的尖角，转角 157.4°）：
  P(0,0) → X(-0.5,2.5) → Y(0.5,2.5) → P(0,0)
```

`validate()==true`，`contours[0].winding()==contours[1].winding()==+1`。P 在轮廓 0 上是转角 0° 的平滑点，在轮廓 1 上是 157.4° 的尖角。

- `edgeColoringByDistance(shape, 3.0, seed=5)` 的结果（按轮廓打印）：
  - 轮廓 0：`M M C M Y`
  - 轮廓 1：`Y C Y` → **尖刺尖角两侧（e2 与 e0）同为 `YELLOW`**（掩码 3，交集 2 位）。
  - `protectCorners` 在 P 处的 stencil 为 `[-][-][-][-]`：**该尖角确实未被保护**（P 在轮廓 0 上是平滑点，不产生保护；轮廓 1 的同掩码角也不满足单比特判据）。
- **这是“图可 3 着色但启发式失败”的实例**：导出该 shape 的零距离图得 7 个 spline、9 条零距离边，**该图可 3 着色**（例如合法着色 `(0,1,0,1,1,0,2)`）。被违反的零距离边恰是 `(4,6)`——即轮廓 1 的两条尖角边 `P→X` 与 `Y→P`。内部日志显示 `colorSecondDegreeGraph: vertex 6 has possibleColors=0 ... assigned color 0`，**贪心落空**导致这两条互为零距离（共享 P）的 spline 同色。因此问题不在“图无解”，而在启发式静默失败。
- **对照**（同一 CE1 的其它分派，全部在 P 处被保护、尖角两侧交集 1 位）：
  - `simple seed0`：轮廓 1 = `C M Y`，尖刺两侧 `Y|C`，stencil `[P][P][P][P]`。
  - `inktrap seed0`：轮廓 1 = `M Y C`，尖刺两侧 `C|M`，stencil `[P][P][P][P]`。
  - `distance seed0`：轮廓 1 = `M C Y`，尖刺两侧 `Y|M`（不同）。
  - 人工可行着色 `pal[(edgeIndex+contourIndex) mod 3]`：两侧不同且被保护。

### 7.2 反例二：K 个同向三角形共顶点（CE2）

**几何**（K=3，全部 `winding==+1`，共享 `P=(0,0)`）：

```
for k in 0..K-1:
  a = 2πk/K
  三角形: P → (cos(a-π/8)·0.5, sin(a-π/8)·0.5)
              → (cos(a+π/8)·0.5, sin(a+π/8)·0.5) → P
```

- `validate()==true`，每个三角形 `winding()==+1`；P 处每个三角形都有 135° 的真实尖角。
- 零距离图（K=3）：9 个 spline、21 条零距离边，**不是 3 可着色的**（穷举 3^9 无解）。K=2 的零距离图（含跨三角形的零距离对）同样**不可 3 着色**——所以“是否可 3 着色”不是区分这个反例的关键，关键是**尖角两侧**是否同色。
- `edgeColoringByDistance(shape, 3.0, seed=0)`：`[Y C M] [C M Y] [Y C Y]`。**第 3 个三角形的尖角两侧（e2 与 e0）同为 `YELLOW`**。
- K=4 时 `[Y C M] [C M Y] [Y M Y] [Y C Y]`，三角形 2、3 出现同掩码尖角；K=2 时两个尖角两侧都不同（`[Y C M][C M Y]`）。
- **对照（针对“尖角不变式”，不是针对全零图）**：手工按叶循环置换 `[Y C M][C M Y][M Y C]`，三个尖角两侧分别为 `M|Y`、`Y|C`、`C|M`，全部交集 1 位。这个着色并不能满足**所有**零距离对（因为 K≥2 的零距离图本身不可 3 着色），但它能让每个尖角的两个 flank 不同——这正是尖角可锁的实际条件。因此反例暴露的是：**启发式连“每个尖角 flank 不同”这个更弱、且可满足的目标都没达成**。
- 机制：全零图不可 3 着色 + `colorSecondDegreeGraph` 的 `possibleColors==0` 落空，二者叠加；日志显示 `vertex 5/6/8/9/11 has possibleColors=0`。

### 7.3 两个反例的检查清单（可稳定复现）

| 检查 | CE1（seed5） | CE2 K=3（seed0） | 可行着色对照 |
|---|---|---|---|
| 目标尖角两侧掩码是否相等 | **相等（Y&#124;Y，交集 2 位）** | **相等（Y&#124;Y，交集 2 位）** | 均交集 1 位 |
| `protectCorners` 是否在该角产生保护 | **否**（stencil 全 `-`） | **否**（该三角形单独取出时全 `-`；整体被邻角覆盖） | 是 |
| 零距离图可 3 着色 | **是**（但仍失败） | 否 | — |
| 尖角两侧不同的可行着色存在 | **是** | **是**（如循环置换） | — |
| 逐通道场差 `max|ΔR/ΔG/ΔB|`（produced vs 可行） | 1.20 / 1.24 / 1.50 | 0.50 / 0.42 / 0.54 | 0（自比） |
| 中值场符号差 | 采样网格 0 次 | 采样网格 0 次 | 0 次 |

> 说明：CE2 的 P 点 `protectCorners` stencil 仍为 `[P][P][P][P]`，但这是同一 P 上**其它三角形**的角（遍历相邻、掩码不同）贡献的覆盖。把 CE2 中同掩码的那个三角形（K=3 的轮廓 2，`Y C Y`）**单独**取出做 `protectCorners`，P 处 stencil 为 `[-][-][-][-]`；把该三角形改为可行着色 `M Y C` 后，P 处恢复为 `[P][P][P][P]`。所以该同掩码角确实不被保护，只是它的作用被邻角掩盖；要单独判断必须按“掩码相等性”逐角筛选，不能只看 P 处汇总 stencil。CE1 是更干净的反例：P 在背景轮廓上是平滑点，尖刺角 stencil 直接为全 0。

---

## 8. 可解释性与可重复性评分（基于证据）

| 维度 | simple | inktrap | distance |
|---|---|---|---|
| 同输入同种子逐位可重复 | 是 | 是 | 是（同机同库） |
| 新建对象 vs 已着色对象再跑 | **否**（2→6 边拆分） | 否 | 否 |
| 轮廓顺序改变是否改变结果 | **是**（全局 seed） | 是 | 是 |
| 顶点旋转是否改变结果 | 对称样例否 | **是** | **是** |
| 尖角两侧掩码交集恰 1 位 | 非退化输入上是 | minor/major 判定成立时是 | **否**（零距离图不可 3 着色或落空时） |
| 近共线 / 零长度边鲁棒 | 否 | 否 | 否 |
| 文档可解释性 | 中（阈值文案相反） | 中（minor 规则未文档化） | 低（实验性、bit 复用、静默落空） |

---

## 9. 源码索引（引用位置）

- `core/EdgeColor.h:7-17` — 掩码枚举。
- `core/edge-coloring.h:6,12-27` — 阈值注释、`MSDFGEN_EDGE_LENGTH_PRECISION`、三策略声明。
- `core/edge-coloring.cpp:19-25` — `symmetricalTrichotomy`、`isCorner`。
- `core/edge-coloring.cpp:27-36` — `estimateEdgeLength`（4 段折线）。
- `core/edge-coloring.cpp:38-58` — seed 抽取、`switchColor`。
- `core/edge-coloring.cpp:68-142` — `edgeColoringSimple`（泪滴 m<3 拆分 `105-122`）。
- `core/edge-coloring.cpp:151-258` — `edgeColoringInkTrap`（minor 判定 `217-228`，minor 补色 `242`）。
- `core/edge-coloring.cpp:260-529` — `edgeColoringByDistance`（零距离硬约束、`colorSecondDegreeGraph` 落空、`tryAddEdge` 回滚）。
- `core/edge-selectors.cpp:174-222` — 三通道按位分派。
- `core/contour-combiners.cpp:30-32`、`core/msdfgen.cpp:30-34` — 中值 / 通道写出。
- `core/render-sdf.cpp:77,88` — 渲染端中值。
- `core/MSDFErrorCorrection.cpp:121-140` — `protectCorners` 的单比特判据。
- `core/MSDFErrorCorrection.cpp:460-476` — `apply` 三通道等化。
- `core/Vector2.hpp:42-46` — 零向量归一化返回 `(0,1)`。
- `core/edge-segments.cpp:13-17` — 共线二次曲线降级为直线。
- `main.cpp:32`（默认阈值）、`99-110`（`parseAngle`）、`426-427`（CLI 文案）、`619,892-896`（`-angle`）、`947-954`（策略选择）、`1023-1026`（`-seed`）。

---

## 10. 附：复现脚本说明（非仓库内容，未提交）

所有实验脚本位于 `/tmp/msdf_analysis/`，均以 `-I <repo> -DMSDFGEN_PUBLIC= -DMSDFGEN_USE_CPP11` 编译，链接仓库的 `build/libmsdfgen-core.a`。代表脚本：

- `evidence.cpp` / `final_evidence.cpp` — 阈值表、N 边形悬崖、跨轮廓耦合、重复运行。
- `ce_correct.cpp` / `ce1_focus.cpp` — CE1 的几何、掩码审计、通道场对照与 stencil 审计。
- `ce2.cpp` / `bilobe*.cpp` — CE2 与额外的自接触双叶观察。
- `evenm.cpp` / `splitorder3.cpp` — 泪滴 m=2 的 6 段拆分次序（含 corner 下标 1 的重排）。
- `realinf.cpp` / `nanprobe2.cpp` — 未用通道 `-inf` 与双线性插值产生 `NaN` 的实证（补充 §6.6）。
- `colorcheck.py` — 从 `COLOR_GRAPH_DUMP` 导出的零距离图做 3 可着色性穷举（`python3 -I` 运行）。

---

### 一页话总结

- 分派规则是“边掩码 → 三通道独立选择器 → 中值”，不是“一条边一种色”；可解释性关键在**角两侧掩码交集是否为单通道**。
- 角度阈值是“转角 > θ”，默认 θ=3.0 rad ≈ 8.113°；CLI 文案把“转角”写成“夹角”，易致相反理解。
- 种子是全局跨轮廓 LFSR；轮廓顺序、顶点旋转、首次/再次运行都会改变同一条轮廓的颜色。
- simple/inktrap 在非退化输入上能锁住尖角；distance 是实验性的，其零距离图**不可 3 着色、或贪心落空惰性赋值时会静默产生同掩码尖角**，而 `protectCorners` 检测不到——属可稳定复现的**通道分派错误**。
- 两个有限反例：CE1（尖刺，seed5，图可 3 着色但启发式失败，stencil 全 0）与 CE2（K=3 共顶点，seed0，图不可 3 着色）；均给出尖角两侧同为 `YELLOW`，并以人工可行着色对照；复现脚本与断言见 §7。
