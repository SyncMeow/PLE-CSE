# PLE / CSE v2 計畫（草稿，2026-11-13）

v2 有兩個目標：

1. **降低資源**：縮小 PLE 與 CSE 的 LUT / FF / BRAM。
2. **對齊前處理介面**：PLE/CSE 直接使用組員 `vcc-based-accelerator` 的 `stage_a` + `build_engine` 建出的環境。

狀態：草稿。第 6 節的決定已由使用者確認（2026-11-13）；尚未和 CSE 作者、前處理作者討論。標「估計」的數字都還沒有合成驗證。

## 0. 前提與範圍

**板子**：最後使用的板子不一定是 100T，可能拿到更大的（型號未定）。所以 v2 不為 100T 寫死架構，而是**同一份 RTL、用參數切換兩組設定**，兩組都要通過同一套回歸測試。

| 設定 | N_CU | 用途 |
|---|---|---|
| 100T 設定 | 2 | 放進 xc7a100t |
| 大板子設定 | 4 或 8 | 板子確定後提升效能 |

**範圍（誰可以改什麼）**

| 區塊 | 負責 | 我們在 v2 做什麼 |
|---|---|---|
| PLE（`accel/rtl/ple`）、golden（`sw_rrtc/fx`）、testbench | 我們 | 直接修改 |
| PLE ↔ CSE 介面、CSE ↔ 環境記憶體介面（`ple_cse_pkg.sv`、埠列表） | 我們定義 | 修改介面定義，寫成規格 |
| CSE 內部（`accel/rtl/cse`） | 另一位組員 | **不改**。最後打包規格 + pkg 交給他 |
| 環境記憶體、建表引擎（前處理） | 前處理作者 | **不改**。我們只知道它的讀取介面，並以它的格式為準 |

---

## 1. 基準（2026-11-13 實測）

兩邊都是 Vivado 2024.1、OOC、**只做到合成**。

### 1.1 我們：`ple_cse_top`，N_CU=2，13.333 ns

合計 40,447 LUT（其中 LUTRAM 9,664）/ 38,467 FF / 132 RAMB36 / 142 DSP，合成 WNS +1.091 ns。
報告：`build/accel/ooc_res/ple_cse_top_ncu2_synth_util_hier.rpt`。

| 模組 | LUT | 其中 LUTRAM | FF | RAMB36 | DSP |
|---|---|---|---|---|---|
| `cse_env_mem` | 353 | 0 | 230 | 124 | 0 |
| `ple_tree` | 10,168 | 7,856 | 604 | 6 | 0 |
| `ple_evu`（FKU + SCU + QD + RKG） | 16,179 | 1,152 | 19,235 | 2 | 50 |
| `ple_core` 本身 | 4,324 | 28 | 10,497 | 0 | 53 |
| `ple_div7` | 1,864 | 0 | 942 | 0 | 0 |
| `ple_smp` | 1,825 | 0 | 1,190 | 0 | 1 |
| `ple_nnu` | 1,124 | 0 | 470 | 0 | 28 |
| `ple_isqrt` | 360 | 0 | 168 | 0 | 0 |
| `cse_cu` × 2 | 3,174 | 608 | 3,204 | 0 | 10 |
| CSE 其他（FIFO、dispatcher、receiver） | 736 | 20 | 1,148 | 0 | 0 |

觀察：

- **BRAM 幾乎全部是環境記憶體**（124 / 132）：每份 31 個，乘上 2 個 CU 和 2 份 buffer。
- **`ple_tree` 的 7,856 個 LUTRAM 是 BRAM 不夠造成的**：PLE 單獨合成時 tree 是 19 個 RAMB36，LUT 約 2.3k。
- `ple_core` 的 10,497 個 FF 大多是路徑緩衝（`path_mem` 64 × 112 bit）和 backtrace 陣列。
- 每個 CU 裡兩個深度 2 的 FIFO 佔 1,006 LUT（`u_cell_fifo` 622、`u_voxel_fifo` 384），比 CU 本身的邏輯（約 580）還多。
- EVU 內部：依 `records/20261108-ple-rtl.md`，FKU 約 5.7k LUT / 26 DSP、SCU 約 3.0k LUT / 24 DSP，其餘約 7.5k 是 QD、pose slot 和 RKG（由相減得到，不是直接量的）。

### 1.2 組員：`vcc_core`，10 ns，`HASH_AW=14 CELL_AW=12 MAXN_AW=9`

合計 18,077 LUT / 10,488 FF / 110 RAMB36 + 3 RAMB18 / 53 DSP，合成 WNS +0.732 ns。

| 模組 | LUT | RAMB36 | DSP | v2 是否使用 |
|---|---|---|---|---|
| `build_engine` + `stage_a` | 2,449 | 96（+2 RAMB18） | 4 | 是 |
| `rrt_engine` | 10,168 | 13（+1 RAMB18） | 29 | 否（參考實作） |
| `fk_engine` | 4,969 | 0 | 12 | 否 |
| `query_engine` | 487 | 0 | 8 | 否 |

`build_engine` 的 96 個 RAMB36：hash 28、點 24、計數/offset 16、cell id 目錄 14、AABB 11、start/count 3。
其中 hash 和計數表（44.5 tile）是建表暫存，其餘 52 個是環境本身（單份、每塊一個讀取埠）。

### 1.3 100T 的參考預算

| 項目 | LUT | BRAM tile | 來源 |
|---|---|---|---|
| 元件上限 | 63,400 | 135 | xc7a100t |
| SoC | 31,335 | 約 19 | LUT 見 `CLAUDE.md`；BRAM 由「合計 112%」反推，需在 SoC repo 確認 |
| 前處理（含環境記憶體，現況） | 2,449 | 97 | 1.2 |
| 留給 PLE + CSE | 29,616 | 19 | 相減 |

- **BRAM 是硬限制**：前處理現況（97 tile）加 SoC 之後只剩 19 tile，而 PLE 的 tree 就需要 19 個 RAMB36，其他 PLE 記憶體再加約 6 tile。差額要靠前處理那邊縮小，或靠大板子。
- LUT 的工作目標先訂 **PLE + CSE ≤ 27,000**。整體使用率是否可接受這個問題先不處理（第 6 節 D1）。

---

## 2. 降低資源

「結果」欄：**不變**表示規劃結果和 golden 逐 bit 相同，只影響 cycle 數或面積。

### 2.1 我們可以直接做的（PLE）

| # | 改動 | 節省（估計） | 代價 | 結果 |
|---|---|---|---|---|
| P1 | `path_mem` 和 backtrace 陣列改成 RAM（同步讀） | 約 −7.4k FF、−1～2k LUT | 主機讀路徑多 1 cycle | 不變 |
| P2 | 確保 `ple_tree` 在 BRAM（合成屬性 + OOC 檢查） | 約 −7.9k LUT（相對於現況） | 需要 19 個 RAMB36 | 不變 |
| P3 | QD 的 pose slot 8 → 4（既有參數 `NSLOT`） | 約 −1～2k LUT | 需用 `perf_sweep.py` 量 cycle 增加量 | 不變 |
| P4 | SCU lane 8 → 4（產生器參數） | 約 −1k LUT、−12 DSP | 自碰撞檢查變慢 | 不變 |
| P5 | N_CU = 1（既有參數） | −1.6k LUT、−5 DSP | cycle 約 ×1.7（`records/20261110`） | 不變 |
| P6 | **備案**：`ple_div7` 7 個除法器共用；`ple_smp` 7 維逐維計算 | 約 −2.3k LUT | 要重寫模組；每次 steer / 取樣多數十 cycle | 不變 |

- **P1、P2 是去掉浪費**，兩組設定都做，不影響之後的效能。
- **P3–P5 是既有參數**，只在 100T 設定調小；大板子設定維持原值或調大。不改架構。
- **P6 要重寫模組**，大板子用不到。只有確定留在 100T、且差額仍不夠時才做。
- P2 之後估計約 32k LUT，P1 再減 1～2k；到 27k 的差額由 P3–P5 補，等實際數字出來再選。

### 2.2 交給 CSE 作者的建議（列入第 5 節的發包文件，不是我們改）

| # | 建議 | 節省（估計） | 需要他確認的事 |
|---|---|---|---|
| C1 | CU 內的 FIFO 不再攜帶整份查詢 context | 每個 CU 約 −0.7k LUT | CU 是否一次只處理一筆查詢 |
| C2 | `KILL_GEN_W` 32 → 8 | FF 與比較器 | generation 回繞是否安全 |
| C3 | 單一 buffer 之後移除 `rd_buf` 選擇 | 少量 | 無 |

### 2.3 對環境記憶體的需求（前處理作者的範圍，我們只提需求）

- **每個 CU 一組讀取埠**。`build_engine` 現在每塊記憶體只有一個讀取埠；N_CU=2 需要第二組。
- 建議的做法是每兩個 CU 用 true dual-port 共用一份（份數 = ⌈N_CU / 2⌉），每加兩個 CU 約多 52 個 RAMB36（以他們現在的環境大小計）。這由前處理作者決定。
- 單一 buffer（D2 已同意）。
- 效能參考（`records/20261110`，cycle 數相對於理想 CSE）：N_CU=2 是 2.53 倍、4 是 1.63 倍、8 是 1.26 倍。

---

## 3. 對齊前處理介面

原則（D3、D4）：**環境的格式與記憶體以前處理為準**，我們改 PLE 的常數、golden、以及 CSE 的介面定義。

### 3.1 座標：直接採用前處理的座標

前處理的座標是「世界 Q14 + 32768」，也就是原點在 −2 m，每軸 u16。

- 我們把 **w_min 改成 −32768 LSB（−2 m）**。PLE 的球心 = 世界 Q14 + 32768，和環境的點在同一個座標系。
- 查詢格 = `q >> 11`，每軸 **32 格**（原本 20 格）；細格 = `q >> 9`。和前處理完全相同。
- 前處理不需要為我們改任何東西。

> 之前討論過的 −20480（保留 20 格）不採用：目錄以前處理的 Morton 索引為準（D4），而 Morton 索引只有在座標完全相同時才能直接使用。

### 3.2 環境讀取介面（取代格式 v0 與 I3）

以 `build_engine` 現有的讀取埠為準（`hw/rtl/build_engine.v` 的 `ext_*`），每個 CU 一組，位址送出後下一個 cycle 資料有效（和現在的 `ENV_RD_LAT = 1` 相同）。

| 內容 | 位址 | 資料 | 格式 v0 的對應 |
|---|---|---|---|
| 目錄 | 15 bit，粗格 Morton key（x、y、z 各 5 bit 交錯，x 在最低位） | 16 bit cell id，`0xFFFF` 代表空格 | DIR：13 bit 線性位址、`{occ, slot}` |
| cell | 12 bit cell id | `{count[15:0], start[15:0]}` 與 AABB `{hi z,y,x, lo z,y,x}`，各 16 bit **絕對座標** | CELL：128 bit，AABB 是 11 bit 偏移 |
| 點 | 最多 15 bit | `{z, y, x}` 各 u16 | PTS：相同 |
| 狀態（暫存器） | — | `env_valid`、`status`、`kept`、`ncells`、全域 AABB `glo` / `ghi` | HDR |

- 點依細格 Morton key 排序，同一個查詢格的點連續存放。這和格式 v0 的假設相同。
- 容量（前處理的上板設定）：最多 12,288 點、4,096 個 cell。
- 單一 buffer：`q_env_buf` 留在 Q 介面上但固定為 0。

### 3.3 PLE 與共用定義要改的地方

| 檔案 | 改動 |
|---|---|
| `accel/rtl/common/ple_cse_pkg.sv` | `W_MIN_LSB` = −32768；`ENV_G` = 32；`DIR_AW` = 15；目錄與 cell 的型別依 3.2；移除 `AOFF_W`、`env_region_e`、HDR 的 word 配置 |
| `sw_rrtc/tools/gen_fx_fku_rtl.py` | `W_MIN`，然後重新產生 `ple_fku.sv`（球心 + 32768） |
| `sw_rrtc/fx/fx_pose.c` | `fx_w_min` |
| `sw_rrtc/host/fx_envcheck.c` | 寫死的 `w_min = -19497` |
| `accel/tb/mutants.py` | `fku_m6` 比對的字串 `18'sd19497` |
| `accel/rtl/ple_cse_top.sv` | 拿掉 `cse_env_mem` 與 `env_wr_*`，改成對外的環境讀取埠 |
| `docs/INTERFACES.md`、設計文件第 3、5.1 節 | I2、I3、常數表 |

PLE 內部不用改的部分：

- `COORD_W` = 18（有號）仍然足夠：球心範圍約 32768 ± 20000。`RQ_W`、Q/R/K 介面不變。
- **r_q 表**：`fx_pose.c` 已經為點座標留了每軸 1 LSB 的誤差（`e_p = √3`），前處理的「向零截斷」誤差也在 1 LSB 內。
- `ple_top` 的 W_MIN 暫存器（0x030–0x038）直接讀 package 常數。

### 3.4 golden model 要跟著前處理的行為

`sw_rrtc/fx/fx_env.c` 的 `fx_env_build` 要改成和 `stage_a` + `build_engine` 一致：

| 項目 | 目前 | 改成 |
|---|---|---|
| 量化 | `floor(p · 2¹⁴) − w_min` | 向零截斷，再加 32768 |
| 範圍剔除 | 每軸方塊 `[w_min, −w_min)` | 徑向：`x² + y² + z² ≥ 19497²` 的點丟棄 |
| 非有限值 | 未處理 | 回報 INVALID |
| 容量 | 65,535 點 / 4,096 格 | 12,288 點（建表規則 `kept·4 ≥ 3·2¹⁴`）/ 4,096 格 |
| 目錄與 cell | 20³ 線性、偏移 AABB | 32³ Morton、絕對 AABB |
| trace 檔 | `_hdr/_dir/_cell/_ptx/_pty/_ptz.hex` | 依 3.2 重新定義（測試用的環境模型讀這些檔） |

- 選點規則本來就相同：S=4、同樣的距離公式、平手留先到的點。
- 改完後**所有題目的環境都會變**，規劃路徑也會變，golden trace 要全部重新產生。
- 交叉驗證：用組員 repo 的 `hw/vectors`（`core.cpp` 產生）和新的 `fx_env_build` 逐項比對（點、目錄、cell、全域 AABB）。座標相同之後可以直接逐 bit 比。

### 3.5 測試用的環境模型

環境記憶體不由我們維護，所以模擬需要一個**只放在 `tb/` 的行為模型**：介面和 3.2 完全相同，內容由 golden 的 trace 載入。它取代現在 `cse_env_mem` 在測試裡的角色，不進合成。

### 3.6 主機介面（D5）

- **兩個獨立的 AXI-Lite**：`ple_top` 的位址表不動；建表引擎用組員的 `vcc_axil_top`，只留建表相關的暫存器。
- **中斷各一條**：建表完成、規劃完成分開。
- **點雲輸入**：先用 CPU 逐 word 寫暫存器打通流程；`build_engine` 的串流輸入埠保留給之後的 DMA。
- **驅動程式分開**：`accel/sw/ple_drv.c` 不動，建表另一個檔案。
- `MODE.env_buf` 固定為 0。「PLE 規劃中不能建表」仍由 CPU 軟體遵守，兩個硬體之間沒有握手。

---

## 4. 里程碑

每一步結束都要有證據：`bash scripts/sim_regress.sh` 的摘要，或 OOC 報告的數字。**CSE 的改動排在最後**（D4）。

| # | 內容 | 完成條件 |
|---|---|---|
| M0 | 基準量測 | 已完成（第 1 節） |
| M1 | golden 對齊前處理（3.1、3.4） | 新的 `fx_env_build` 和 `core.cpp` 向量逐項相同；`fx_envcheck` 通過 |
| M2 | PLE 對齊（3.3）+ 測試用環境模型（3.5） | PLE 的單元、core、top、drv 測試搭配 `cse_bfm` 全過；mutant 測試仍全部被抓到 |
| M3 | PLE 去掉浪費（P1、P2） | 回歸 bit-exact；PLE 單獨 OOC 佈線後 75 MHz 收斂；記錄 LUT / FF / cycle 變化 |
| M4 | 100T 設定的參數選擇（P3–P5，必要時 P6） | `perf_sweep.py` 的 cycle 對面積表 |
| M5 | **CSE 發包**（第 5 節） | 文件 + pkg + 測試向量交給 CSE 作者 |
| M6 | CSE 作者交回後整合 | `test-cse`、`test-ple-cse` 在 N_CU=2 和 4 都通過；`ple_cse_top` OOC |
| M7 | 接上真正的建表引擎與環境記憶體 | 模擬：點雲 → 建表 → 規劃，結果和 golden 相同 |
| M8 | SoC 整合 | 在 SoC repo 進行 |

**M2 到 M6 之間，現有的 CSE RTL 無法和新的 PLE 一起跑**（它還是格式 v0、20 格）。這段期間：

- PLE 的驗證靠 `cse_bfm`（行為模型，直接呼叫 golden）。
- `test-cse`、`test-ple-cse`、`test-env-mem` 會暫時從回歸清單移出，並在 `scripts/sim_regress.sh` 註明原因；M6 時加回來。
- 這是 D4「CSE 留到最後」的直接後果，需要使用者知道（第 7 節）。

---

## 4.1 進度（2026-11-13）

**第一步已完成：座標與 golden 對齊**（branch `feature/v2-align-preprocessing`，紀錄 `records/20261113-v2-align-preprocessing.md`）。

- w_min = −32768、每軸 32 格、golden 的建表規則和前處理相同。和 `core.cpp` 的 7 組向量逐項相同。
- **發現：現有的 CSE RTL 不用改就能跑**。它的格數和目錄位址寬度都引用 package 常數，改成 32 格後回歸 10/10 通過。所以「M2–M6 期間沒有 PLE + 真正 CSE 的回歸」這個風險**目前不存在**。
- **發現：Morton 目錄只是位址線的重新排列**。32 格時線性索引就是 `{z, y, x}` 各 5 bit 串接，Morton 是同樣 15 條線交錯排列，不需要運算。CSE 之後要改的只剩：目錄位址的排列、空格的編碼（`0xFFFF`）、cell 的格式（絕對座標 AABB、start/count）。
- **資源變差（需要決定，見第 7 節）**：目錄 8,000 → 32,768 項後，我們暫用的 `cse_env_mem`（2 CU × 2 buffer）從 124 變成 148 個 RAMB36 + 7,936 LUTRAM；`ple_cse_top` 合計 150 RAMB36 / 53,615 LUT，在 100T 上放不下。

**第二步已完成：環境記憶體改成單一 buffer**（`cse_env_mem` 的參數 `N_BUF`，預設 1）。

- OOC 合成（N_CU=2）：**28,181 LUT / 101 RAMB36 + 8 RAMB18 / 140 DSP**。環境 80 個 RAMB36，tree 回到 BRAM（19 個）。100T 單獨放得下，可以繼續跑佈線。
- **發現：前處理的環境格式可以用純接線轉成格式 v0**，所以 CSE 不需要為了功能而修改。第 5 節的發包內容因此縮小成「變更說明 + 建議的資源縮減」，草稿在 `docs/CSE_V2_HANDOFF.md`。3.2 的介面改由我們這邊的一層轉接實作（下一步）。

## 5. CSE 發包內容（M5）

交給 CSE 作者的一包東西，讓他不用讀 PLE 的 code 就能改：

1. **規格文件**（`docs/CSE_V2_HANDOFF.md`，M5 時撰寫）：
   - 環境讀取介面（3.2）：埠、時序、每個欄位的位元配置、和格式 v0 的逐項對照。
   - CSE 內部必須改的地方：目錄位址改成 Morton、格數 20 → 32（範圍夾限）、cell 的讀法、AABB 改成絕對座標、狀態來源、單一 buffer。
   - 不變的部分：Q/R/K 介面、C1–C6 義務、HIT 的定義。
   - 建議的資源縮減（2.2 的 C1–C3）與目前的資源數字（1.1）。
   - 驗收條件：哪些測試、哪些 N_CU。
2. **`ple_cse_pkg.sv` v2**：新的常數與型別。
3. **測試用環境模型**（3.5）與新格式的 **golden trace**（環境 + 查詢 + 期望結果）。
4. **`ple_cse_if_checker.sv`**（不變）。

---

## 6. 決定紀錄（2026-11-13，使用者）

| # | 問題 | 決定 |
|---|---|---|
| D1 | 整體 LUT 使用率是否可接受；要不要縮 SoC 或換元件 | 先忽略 |
| D2 | 100T 設定用單一 buffer | 同意 |
| D3 | 環境記憶體由誰維護 | 不由我們維護；我們只知道它的介面 |
| D4 | 目錄的索引方式 | 以前處理的 Morton 為準，改我們的 CSE 介面。CSE 內部由另一位組員負責，改動留到最後，打包規格 + pkg 發包。我們能改的只有 CSE 介面 |
| D5 | 建表引擎的暫存器與 `ple_top` 的位址表 | 兩個獨立的 AXI-Lite、中斷各一條、點雲先寫暫存器、驅動分開（3.6） |
| D6 | 大板子的型號與時脈 | 還不清楚 |

---

## 7. 風險與待確認

- `fx-xcheck` 需要組員 repo 的向量，目前不在回歸清單內。選項：把 7 組向量（約 2.5 MB）複製進本 repo 並記錄來源，或固定要求 clone 在旁邊。
- golden 新增的 `trunc` / `isfinite` 在 RISC-V 的 SoC 版本（`sw_rrtc/soc`）沒有編譯過，要在 SoC repo 確認。
- `docs/CSE_GUIDE.md`、`docs/CSE_SPEC.md` 仍寫舊的原點與 20 格；這兩份是 CSE 作者的文件，沒有改。
- **BRAM**：前處理現況加 SoC 後，100T 上留給 PLE 的 BRAM 不夠放 tree（1.3）。在前處理縮小或換板子之前，100T 設定的 tree 仍可能落在 LUTRAM。
- **每個 CU 一組讀取埠**是對前處理作者的新需求（2.3），他現在的記憶體只有一個讀取埠。
- M1 會改掉所有既有的 golden 數字（迭代數、路徑、查詢數）。`records/` 裡舊實驗的數字之後只能當相對比較。
- 大板子設定在拿到板子前只能做模擬和合成。N_CU 變大後 `q_ready` 到 K 的組合路徑（目前的最差路徑）可能先成為時序瓶頸。
- N_CU=8 之後和理想 CSE 只差 1.26 倍，再往上要改 PLE 本身，不在 v2 範圍內。
