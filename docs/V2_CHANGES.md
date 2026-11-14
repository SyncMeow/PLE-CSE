# v1 到 v2 的變更整理

- v1 基準：commit `7733701`
- 目前：commit `a642315`（2026-11-14），branch `feature/v2-align-preprocessing`，沒有 push
- 計畫：`docs/V2_PLAN.md`。量測細節：`docs/PLE_ARCH_STUDY.md`、`records/20261113-v2-align-preprocessing.md`

這份文件只整理「改了什麼、結果是多少」。每一項的證據在對應的 commit 和上面兩份紀錄裡。

---

## 1. 總覽

N_CU = 2，75 MHz（13.333 ns），OOC。

| | v1（`7733701`） | 目前（`a642315`） |
|---|---|---|
| LUT | 40,187 | 28,905（佈線後） |
| RAMB36 | 132 | 61 |
| RAMB18 | — | 10 |
| DSP | 142 | 140 |
| 佈線 WNS | +0.433 ns | +0.737 ns |
| 環境記憶體 | 124 個 RAMB36 | 40 個 RAMB36 |
| SCU 每次檢查 | 63 個 cycle | 27 個 cycle |

- v1 的數字取自 `CLAUDE.md` 的紀錄（當時的 OOC 結果），和目前的數字不是同一次報告，只能看趨勢。
- v1 加上 SoC（31,335 LUT）會超過 100T（113% LUT、112% BRAM）。目前 PLE + CSE 的 RAMB36 約是 100T（135 個）的 45%。
- **cycle 數不能跨 v1 和 v2 比較**：座標改了之後環境的點不同。下面的 cycle 都是 v2 座標下的數字。

每一步的合成結果（`build/accel/ooc_v2a` 到 `ooc_v2d`，合成後的 Slice LUT 與 RAMB36）：

| 步驟 | commit | LUT | RAMB36 |
|---|---|---|---|
| v1 | `7733701` | 40,447 | 132 |
| 座標對齊 | `392727a` | 53,615 | 150 |
| 單一 buffer | `61b72dc` | 28,181 | 101 |
| 兩個 CU 共用記憶體 | `71b37e9` | 28,154 | 61 |
| SCU 列過濾 + ERROR 修正 | `a642315` | 29,189 | 61 |

座標對齊那一步資源變多（目錄變大、tree 被擠出 BRAM），下一步改成單一 buffer 後才降下來。

---

## 2. 架構變更

### 2.1 座標與環境格式對齊前處理（`392727a`）

目的：和組員的前處理（`kd229y/vcc-based-accelerator`）用同一套座標和建表規則。

- 座標原點改成 −2 m：`w_min` −19497 → −32768。座標 = 世界 Q14 + 32768。
- 查詢格每軸 20 → 32 格，目錄 8,000 → 32,768 項（`DIR_AW` 13 → 15）。
- golden 的建表（`sw_rrtc/fx/fx_env.c`）改成和前處理的 `core.cpp` 一致：向零截斷、徑向剔除、非有限值與容量的錯誤碼。
- RTL 只改 `ple_cse_pkg.sv` 的三個常數，和重新產生的 `ple_fku.sv`。**CSE 的 RTL 沒有改。**
- 發現：32 格時線性索引就是 `{z, y, x}` 串接，前處理的格式可以用純接線轉成我們的格式 v0。所以 CSE 不需要為了功能而修改。

### 2.2 環境記憶體改成單一 buffer（`61b72dc`）

- `cse_env_mem` 新增參數 `N_BUF`（1 或 2，預設 1；v1 固定 2）。`ple_cse_top` 有同名參數。
- `N_BUF = 1` 時，buffer 1 的 header 讀到 invalid、資料讀到 0、寫入被丟棄。
- 效果：RAMB36 150 → 101；tree 回到 BRAM 後 LUT 少約 12k。

### 2.3 兩個 CU 共用一份環境記憶體（`71b37e9`）

- 每張表存 ⌈讀取埠數 / 2⌉ 份 true dual-port 記憶體。port A = 寫入或偶數讀取埠，port B = 奇數讀取埠。
- 埠、延遲、超出深度讀 0 都不變。
- **新的寫入規則**（`docs/INTERFACES.md` I3）：HDR valid 清 0 → 等 `cse_idle` → 寫表 → valid 設 1。valid = 1 時寫表，模擬會報錯。
- 效果：環境記憶體 80 → 40 個 RAMB36。
- 階層路徑變成 `g_buf[b].g_cp[c]`；CSE 作者的 `tb_cse_array_worst.sv` 已配合修改，需要他確認。

### 2.4 SCU 的列過濾（`a642315`）

- 自碰撞檢查多一層：外包球相交的兩個 link，先拿 link A 的每顆球對 link B 的外包球測一次，通過的才和 link B 的每顆球比。
- 原因：link 5 的外包球和 link 7、手掌的外包球在 100% 的姿態都相交，每次都要比 276 對球，但幾乎從來沒有真的撞到。
- 門檻用三角不等式推導（`sw_rrtc/fx/fx_pose.c` 的 `fx_pose_init`），依賴「同一個 link 的球是剛體」。
- golden 新增 `fx_self_ok_filt`（含過濾）。在 FK 算出的姿態上，它和原本的 `fx_self_ok` 結果相同。

| | 修改前 | 修改後 |
|---|---|---|
| SCU 每次檢查 | 63 個 cycle | 27 個 cycle |
| 理想 CSE 的總 cycle | 4,607,307 | 4,215,215（−8.5%） |
| 真正的 CSE，N_CU = 8 | 5,958,739 | 5,723,585（−3.9%） |
| 真正的 CSE，N_CU = 2 | 11,844,300 | 11,666,479（−1.5%） |
| LUT（佈線後） | 27,887 | 28,905 |

- 乘法器沒有增加。
- 限制：SCU 的產生器現在只支援 8 條以上的 lane；暫存器 0x0D4（自碰撞測試次數）現在包含過濾的測試。

---

## 3. Bug 修正

### ERROR 之後的 run 誤報 ENV_ERR（`ed06a1f`）

- 現象：一個 run 以 ENV_ERR 結束後，下一個正常的 run 也回報 ENV_ERR。
- 根因：`ple_qd` 對任何 tag 的 ERROR 都設 `env_err`，而新的邊只等另一個 tag 的回應歸零。上一個 run 殘留的 ERROR 被下一個 run 收下。
- 修正：只接受目前 tag 的 ERROR（`ple_qd.sv:201`）。
- 這是 v1 就有的缺口。它和時序有關，只在姿態後端變快且 N_CU = 8 時才撞得到。

---

## 4. 測試與工具

| 項目 | 內容 |
|---|---|
| `make -C sw_rrtc fx-xcheck VCC_VECTORS=<路徑>` | golden 的環境建表對照前處理的向量。需要組員的 repo，沒有放進回歸 |
| `fx_scucheck`（在 `fx-check` 內） | 列過濾不改變結果；剛體假設成立 |
| `tb_env_mem` | 所有讀取埠同時讀、寫入期間的讀取、valid 時寫表會報錯 |
| `tb_qd` | 每個 cycle 把 `env_err` 和模型比對，含舊 tag 的 ERROR |
| `tb_scu` | 對照含過濾的 golden；690 對球各自獨立的邊界測試 |
| `tb_evu` | 新增一段很慢的 CSE（QD 塞住超過一次 FKU 的時間） |
| `tb_core +scu_trace=<檔>` | 每次 SCU 檢查的 cycle 數與相交的組 |
| mutant | 74 → 79 個，全部被抓到 |
| `accel/scripts/` | `fku_stats.py`、`fku_modulo.py`、`scu_trace_stats.py` |

---

## 5. 文件

- `docs/` 從 `.gitignore` 移除，改成有追蹤（`d477c89`）。
- `docs/V2_PLAN.md`：v2 的計畫與決定。
- `docs/PLE_ARCH_STUDY.md`：架構研究紀錄，之後的研究都更新在這裡。
- `docs/CSE_V2_HANDOFF.md`：給 CSE 作者的發包草稿。
- `docs/INTERFACES.md`：環境記憶體的內部結構與寫入規則。

---

## 6. 研究結果（還沒改 RTL 的部分）

詳見 `docs/PLE_ARCH_STUDY.md`。

- 換碰撞檢查演算法，PLE 幾乎不用改（推論）。和 RRT-Connect 綁定的只有 `ple_core` 的狀態機。
- FKU 單一姿態的延遲下限是 28 個 cycle；吞吐量大致和乘法器數量成正比。
- 列過濾之後瓶頸回到 FKU。FKU 改 16 lane 加上列過濾，理想 CSE 下合計約 −26.8%（列過濾那一半當時是用時序模擬量的），代價是多 36 個 DSP。
- N_CU = 2 時瓶頸在 CSE：N_CU 從 2 加到 8 約快 2 倍。

---

## 7. 還沒做的

- 前處理格式的轉接層沒有實作；共用記憶體的方案還沒和前處理的作者討論。
- CSE 的最終發包；環境怎麼進到 `cse_env_mem`（CPU 經 AXI 或建構引擎）。
- SoC 整合：AXI-Lite、IRQ、位址表。
- 沒有上板。SoC 的 bare-metal 版本沒有編譯過。
- 加快 FKU（研究已做，還沒改 RTL）。
- V2 計畫的其他縮減項目（P1 `path_mem` 改 RAM、P3–P5）。
- ERROR 誤報沒有系統層的固定回歸測試，只有單元測試和 mutant。
- `.claude/` 和 `scripts/sim_regress.sh` 還沒有 commit。
