# CSE v2 發包說明（草稿，2026-11-14 第二次更新）

給 CSE 作者。說明 v2 對 CSE 的影響：**哪些已經改了（你不用動）、哪些是建議你改的、怎麼驗收**。
背景與整體計畫見 `docs/V2_PLAN.md`。

## 結論

- **CSE 的 RTL 目前不需要為了功能而修改。** 座標、格數、buffer 數的改變都透過 `ple_cse_pkg.sv` 的常數和 `cse_env_mem` 的參數完成，`accel/rtl/cse/cse_cu.sv`、`cse_top.sv` 等檔案沒有被動過，回歸全部通過。
- 之後接上前處理的環境記憶體時，格式差異會由我們這邊的一層轉接（只有接線，沒有運算）吸收，CSE 仍然看到現在的 I3 埠。
- 下面第 3 節是**建議**的資源縮減，由你決定要不要做。

## 1. 已經生效的變更（你不用改，但要知道）

branch `feature/v2-align-preprocessing`（commit `4073cdf` 之後）。從 v1 到現在的完整變更見 `docs/V2_CHANGES.md`。

| 項目 | 以前 | 現在 | 對 CSE 的影響 |
|---|---|---|---|
| 座標原點 `W_MIN_LSB` | −19497 | −32768（−2 m） | 無。CSE 只在 workspace 座標下運算 |
| 每軸查詢格數 `ENV_G` | 20 | 32 | `GRID_LAST`、`clamp_grid` 自動跟著變；格座標本來就是 5 bit |
| 目錄位址寬度 `DIR_AW` | 13 | 15 | `dir_addr = (gz·32 + gy)·32 + gx`，最大 32767 |
| 目錄項數 `ENV_NCELL` | 8,000 | 32,768 | 只影響記憶體 |
| 環境 buffer 數 | 固定 2 | `cse_env_mem` 參數 `N_BUF`，預設 1 | 埠不變。`N_BUF = 1` 時 `hdr_valid[1] = 0`，所以 `q_env_buf = 1` 的查詢會回 ERROR（`cse_cu.sv:158` 現有的邏輯） |
| 環境記憶體的份數 | 每個 CU 各一份 | 每兩個 CU 共用一份（true dual-port），份數 = ⌈N_CU / 2⌉ | 埠與延遲不變，CU 仍然各有一組讀取埠。多了一條寫入規則，見 1.1 |
| golden 的環境 | 方塊剔除、floor | 徑向剔除、向零截斷（和前處理相同） | trace 的內容全部不同，格式相同 |

球心的範圍：約 32768 ± 22000，半徑最大 8192。`cx ± r` 仍在 19 bit 有號數內（獨立審查逐項檢查過 `cse_cu.sv` 的寬度）。

### 1.1 兩個 CU 共用一份環境記憶體（2026-11-14）

`cse_env_mem` 內部每張表（DIR、CELL、PTS）改成 true dual-port：port A 接第 2c 個 CU（或寫入），port B 接第 2c+1 個 CU。

- **CSE 核心（`cse_top`、`cse_cu`）沒有改**，I3 的埠、`ENV_RD_LAT` = 1、超出深度讀到 0 都相同。
- 效果（OOC 合成，N_CU = 2）：環境記憶體 80 → 40 個 RAMB36，`ple_cse_top` 合計 101 → 61 個。
- **新的寫入規則**：寫表的那個 cycle 會佔用該表的 port A，偶數編號的 CU 在那個 cycle 讀到的資料未定義。載入的順序必須是：HDR valid 清成 0 → 等 `cse_idle` → 寫表 → valid 設回 1。原因是 `cse_cu.sv:158` 只在查詢進入 CU 時檢查一次 valid，已在途的查詢還會讀表。valid = 1 時寫表，模擬會報 `table written while valid`。
- **對你的測試的影響**：直接用階層路徑寫記憶體的 testbench 要改路徑。`dirm`、`cellm`、`ptm` 從 `g_buf[b]` 移到 `g_buf[b].g_cp[c]`（每份一個），`hdr` 仍在 `g_buf[b]`。`tb/tb_cse_array_worst.sv` 我們已經改成寫入兩份（巨集 `ENV_SET`，只適用 N_CU = 4），請你確認。
- 如果你要換掉 `cse_env_mem` 的內部實作，請保留「每兩組讀取埠共用一份」，否則 BRAM 會再變成兩倍。

### 1.2 PLE 這邊的兩個變更（CSE 不用改）

- **PLE 只接受目前 tag 的 ERROR**（commit `ed06a1f`）。以前任何 tag 的 ERROR 都會結束目前的邊；現在已結束的邊晚到的 ERROR 會被丟掉。CSE 的行為不用變：header invalid 或 err 時查詢照常回 ERROR，`cse_err` 照常記錄。
- **SCU 加了列過濾**（commit `a642315`）。和 CSE 無關，但姿態後端變快了，CSE 收到查詢的節奏會比較密（查詢的內容與總數的規則不變）。

目前的 OOC 佈線結果（N_CU = 2，75 MHz）：28,905 LUT / 61 RAMB36 + 10 RAMB18 / 140 DSP，WNS +0.737 ns。

`docs/CSE_GUIDE.md`（126–127 行）和 `docs/CSE_SPEC.md`（167 行）還寫著舊的原點與 20 格。這兩份是你的文件，我們沒有改。

## 2. 之後接上前處理時的轉接（我們做，供你參考）

前處理的建表引擎（`kd229y/vcc-based-accelerator` 的 `build_engine`）提供的讀取介面和格式 v0 不同，但差異都可以用接線轉換，所以 CSE 的 I3 埠維持不變：

| I3 訊號 | 來自前處理的 | 轉換 |
|---|---|---|
| `dir_addr[14:0]` = `{gz, gy, gx}` | 目錄位址 = 粗格 Morton key | 15 條線重新排列：Morton 的 bit `3b`、`3b+1`、`3b+2` 分別是 `gx[b]`、`gy[b]`、`gz[b]` |
| `dir_data` = `{occ, 3'b0, slot[11:0]}` | cell id（16 bit，`0xFFFF` = 空） | `occ = (id != 16'hFFFF)`，`slot = id[11:0]` |
| `cell_data.base`、`.count` | `{count[15:0], start[15:0]}` | `base = start`，`count = count[9:0]`（每格最多 64 點） |
| `cell_data.lo_*`、`.hi_*`（11 bit 偏移） | AABB 的 6 個 16 bit 絕對座標 | 取低 11 bit（AABB 一定落在自己的格子內，高 5 bit 就是格座標） |
| `pt_data` = `{z, y, x}` | 相同 | 直接接 |
| `hdr_valid`、`hdr_err`、`hdr_n_pts`、`hdr_glo`、`hdr_ghi` | `env_valid`、`status`、`kept`、`glo`、`ghi` | 直接接；`err = (status != 0)` |

讀取延遲兩邊都是 1 個 cycle。這一層和對應的測試模型還沒有實作。

## 3. 建議的資源縮減（由你決定）

量測：OOC 合成，N_CU = 2，13.333 ns（2026-11-13）。

| 模組 | LUT | 其中 LUTRAM | FF | DSP |
|---|---|---|---|---|
| `cse_cu`（每個） | 約 1,590 | 304 | 1,602 | 5 |
| 　其中 `u_cell_fifo` | 622 | 110 | | |
| 　其中 `u_voxel_fifo` | 384 | 194 | | |
| 　其中 CU 本身的邏輯 | 約 580 | 0 | 1,594 | 5 |
| `u_query_fifo` | 332 | 0 | 908 | |
| `u_dispatcher` | 242 | 0 | 116 | |

| # | 建議 | 預期（估計） | 需要你確認 |
|---|---|---|---|
| C1 | CU 內兩個深度 2 的 FIFO 不要攜帶整份 `cse_ctx_t`（每筆約 270 bit）。context 在一筆查詢內不變，可以放在 CU 的暫存器，FIFO 只傳會變的欄位（格座標、目錄位址、base、count） | 每個 CU 約 −0.7k LUT | CU 是否一次只處理一筆查詢；若有兩筆交疊，context 要存幾份 |
| C2 | `KILL_GEN_W` 32 → 8 左右 | context 與 query FIFO 的 FF、比較器變小 | generation 回繞時不會誤判（在途查詢數遠小於 2⁸ 即可） |
| C3 | `N_BUF = 1` 之後，`mem_rd_buf`、`ctx.env_buf`、`hdr_*[1]` 的選擇邏輯可以拿掉 | 少量 | 之後是否還要支援雙緩衝（大板子設定可能會用） |
| C4 | `q_ready` 在組合邏輯上依賴當 cycle 的 K（`records/20261110`：N_CU = 2 的最差路徑，24 級） | 改成暫存器輸出可改善 timing | 是否影響 P2（K 握手當下已在 Q 上等待的查詢照常送完）。註：這條路徑在 v1 和共用記憶體後都是最差路徑（+0.433、+0.415 ns）；最新一次佈線的最差路徑換成 PLE 的 SCU 內部（+0.737 ns），這條路徑的餘量沒有重新查 |

C1–C3 都不應該改變任何查詢的結果，只影響面積。

## 4. 不變的部分

- Q / R / K 介面的埠與語意、C1–C6 義務、`MAX_LAT` = 65,536。
- HIT 的定義：存在點 p，`Σ(c − p)² ≤ q_r²`。
- I3 的讀取埠與 `ENV_RD_LAT` = 1。
- `ple_cse_if_checker.sv`。

## 5. 驗收條件（如果你做了第 3 節的修改）

1. `bash scripts/sim_regress.sh` 最後一行 `10/10 passed`（`test-env-mem` 現在也檢查所有讀取埠同時讀、寫入期間的讀取、以及 valid 時寫表會被報出）。它包含 `test-cse`（trace 重播 N_CU = 1、2、3、4、8，kill、control、worst、boundary、單元測試、比較器 mutant）和 `test-ple-cse NCU=2`。
2. 另外跑 `make -C accel test-ple-cse NCU=1` 和 `NCU=4`。
3. `make lint` 沒有 warning。
4. `ple_cse_top` OOC 合成（`accel/scripts/ooc_impl_plecse.tcl`，13.333 ns，N_CU = 2），附上階層報告裡 `u_cse` 的 LUT / FF / DSP 前後對照。
5. 只影響面積的修改：`TB_CSE_TRACE` 的 `queries`、`responses` 和修改前相同。

## 6. 還沒定的事

- 前處理的記憶體目前每塊只有一個讀取埠。1.1 的共用做法（建表完成後把兩個 port 分給兩個 CU）已經在 `cse_env_mem` 上驗證，但還沒和前處理作者談怎麼套到他的記憶體上。
- 第 2 節的轉接與測試模型尚未實作，實作後這份文件會更新。
