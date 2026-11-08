# 對接介面清單（PLE / CSE 整合用）

完整規格在 `records/20261107-ple-cse-design.md`（以下簡稱「設計文件」）。本檔只列出**每個區塊之間要對齊的東西、目前狀態、以哪個檔案為準、怎麼驗證**。介面有任何修改，先改設計文件與 `rtl/common/ple_cse_pkg.sv`，再更新本檔的狀態欄。

狀態：**凍結**（雙方簽核，修改需雙方同意）、**草稿**（內容已定，待簽核）、**暫定**（可能再改）、**待定**（尚未決定）。

## 總覽

| # | 介面 | 兩端 | 狀態 | 以此為準 |
|---|---|---|---|---|
| I1 | 查詢 / 回應 / kill | PLE ↔ CSE | 草稿 v0.1 | 設計文件第 4 節、`rtl/common/ple_cse_pkg.sv` |
| I2 | 環境格式 v0 | 建構引擎 → 環境 SRAM → CSE | 暫定（PLE / CSE 先定） | 設計文件 5.1 節、`ple_cse_pkg.sv`（`dir_t`、`cell_t`）、`sw_rrtc/fx/fx_env.c` |
| I3 | 環境記憶體埠 | 環境記憶體 ↔ CSE 核心（建構引擎寫入） | 暫定（已決定，待簽核） | `rtl/cse/cse_env_mem.sv`、`ple_cse_pkg.sv`（`ENV_RD_LAT` 等） |
| I4 | 主 CPU 暫存器與中斷 | 主 CPU ↔ PLE | 暫定 | 設計文件第 6 節、`rtl/ple/ple_top.sv` |
| I5 | 環境緩衝交接 | 主 CPU、建構引擎、PLE | 暫定（v1 由 CPU 交接） | 設計文件第 11 節 C2 |
| I6 | 時脈與重置 | 全部 | 草稿 | 設計文件 4.1、第 13 節 |
| I7 | 測試向量與 trace | golden model ↔ 兩邊的 testbench | 暫定 | `records/20261108-fx-golden.md` |

## I1：PLE ↔ CSE

- **埠**：`rtl/cse/cse_top.sv` 與 `rtl/ple/ple_top.sv` 的埠列表就是合約，兩邊不可自行增減或改名。位寬全部引用 `ple_cse_pkg`（`TAG_W` = 4、`USER_W` = 16、`COORD_W` = 18、`RQ_W` = 14、`R_MAX` = 8192、`status_e`）。
- **語意**：HIT ⇔ 存在點 p，`Σ(c − p)² ≤ q_r²`（整數、精確）。每筆被接受的查詢恰好一筆回應，回應可亂序，`r_tag`、`r_user` 原樣送回。KILLED 只代表「沒算」。
- **雙方義務**：C1–C6（CSE）、P1–P6（PLE），見設計文件 4.5。P2 已釐清：K 握手當下已在 Q 上等待的查詢照常送完。
- **v1 約定**：CSE 忽略 kill（`k_ready` 接 1、照常回真正結果）；PLE 的 `r_ready` 接 1；`MAX_LAT` = 65,536 cycles（C5：有在途查詢且 `r_ready` 為 1 時，相鄰兩筆回應的間隔上限）。
- **驗證**：`tb/ple_cse_if_checker.sv` 是被動監看模組，PLE 測試、CSE 測試、整合測試都掛同一個。它檢查握手穩定性（H1–H3）、C1/C2、C4（逐筆檢查 KILLED 的生命週期）、C5、C6、P2、P5，以及測試結束時是否全部回應。`make -C Platform/accel test-checker` 是它的自測：乾淨流量 0 個錯誤，8 種植入的違規全部被抓到。
- **待辦**：雙方簽核後把狀態改成「凍結 v1.0」。

## I2：環境格式 v0

- **網格**：座標 u16、LSB = 2⁻¹⁴ m、原點 w_min = −19497 LSB（−1.19 m）；細格 `q >> 9`（3.125 cm）；查詢格 `q >> 11`（12.5 cm），每軸 G = 20。
- **內容**：HDR（valid、err、點數、格數、全域 AABB）；DIR 8000 項 × 16 bit（`dir_t`：`{occ, rsvd[2:0], slot[11:0]}`）；CELL 每個非空格 128 bit（`cell_t`：base、count[9:0]、lo/hi 各 3 × 11 bit 相對於格子原點）；PTS 為 x、y、z 三個 u16 陣列。
- **建構引擎必須保證**（CSE 正確性依賴）：B1–B5，見設計文件第 5 節。最重要的是不可默默丟點，以及 AABB 必須包含格內所有點。
- **驗證**：`tools/check_cse_trace.py` 從 hex 檔重建環境並檢查格子表；`fx_envcheck` 比對 `cse_ref` 與定義。
- **待辦**：建構引擎負責人加入後重新討論格式（是否改用 HackMD 方案 B 兩層區塊）；改格式時只影響 CSE 的 ENVR 子模組與 golden model 的環境產生器。

## I3：環境記憶體埠（2026-11-08 決定）

- **模組**：`rtl/cse/cse_env_mem.sv`（雙緩衝）。它是 CSE 核心與記憶體之間的邊界，建構引擎之後也從這裡接入。內部是參考實作（一般陣列）；CSE 負責人可以改成分 bank 或複製的 BRAM，**但埠與延遲不可變**。
- **讀取埠**：參數 `NRD` 組（每個 CU 一組），每組有三個獨立的同步讀取埠，每個 cycle 都可以同時讀：
  - DIR：`dir_addr[12:0]` → `dir_data[15:0]`
  - CELL：`cell_addr[11:0]` → `cell_data[127:0]`
  - PTS：`pt_addr[15:0]` → `pt_data[47:0]`，排列為 `{z, y, x}`
  - 每組有自己的 `rd_buf`（選第 0 或第 1 份緩衝）。
- **延遲**：`ENV_RD_LAT` = 1。位址在第 t 個上升緣取樣，資料在第 t+1 個上升緣之後有效。超出深度的位址讀到 0。
- **header**：兩份緩衝的 valid、err、點數、格數、全域 AABB 直接以暫存器輸出，讀取不需要等 cycle。
- **深度**：CELL 1024 項、PTS 16384 點。全部 699 題的最大值是 651 格、11,670 點。格式欄位仍保留 12 bit / 16 bit。
- **寫入埠**（建構引擎用，v1 模擬接 0）：`wr_en`、`wr_buf`、`wr_region`（HDR / DIR / CELL / PTS）、`wr_addr`、`wr_data[127:0]`。HDR 的 word 配置：0 `{err, valid}`、1 點數、2 格數、3–5 glo x/y/z、6–8 ghi x/y/z。
- **模擬載入**：`+env0=<trace 前綴>`、`+env1=<前綴>`，讀 `<前綴>_{hdr,dir,cell,ptx,pty,ptz}.hex`（golden model `rrtc_fx -T` 產生）。
- **驗證**：`make -C Platform/accel test-env-mem` 把兩份 trace 載入兩份緩衝，經由不同埠把每個位址讀回並和原檔比對，全部相同；對照組（第 1 份載入錯的 trace）報出 DIFF。

## I4：主 CPU ↔ PLE（暫定）

- **介面**：AXI-Lite slave（12 bit 位址、32 bit 資料）加一條 `irq`，埠在 `rtl/ple/ple_top.sv`。
- **暫存器**：見設計文件第 6 節（CTRL、STATUS、IRQ、MODE、演算法參數、W_MIN、START / GOAL、結果、效能計數器、路徑緩衝 0x400–0x7FF）。
- **待辦**：PLE 實作 HIF 時把位址表定案，寫回設計文件第 6 節。

## I5：環境緩衝交接（v1：CPU 交接）

- CPU 讀建構引擎的完成狀態，把緩衝編號寫進 PLE 的 `MODE.env_buf`，再啟動 PLE；PLE 在整題中固定使用同一份（P4），並經由 `q_env_buf` 傳給 CSE。
- PLE 完成並回到 idle（`cse_idle = 1`）之後，CPU 才允許建構引擎覆寫那一份緩衝。
- **之後可選**：建構引擎與 PLE 之間的硬體握手（`env_ready`、緩衝使用中）。

## I6：時脈與重置

- 單一時脈 `clk`（目標約 100 MHz）。`rst_n` 低電位有效、同步重置；PLE 與 CSE 必須同時重置，重置會丟棄所有在途查詢且不產生回應。

## I7：測試向量與 trace

- **CSE**：`rrtc_fx -p <題目> -n <seed 數> -T <目錄>` 產生環境影像與查詢清單（每行 `tag user kill_on_hit env_buf cx cy cz rq expected`）。格式見 `records/20261108-fx-golden.md` 的「CSE trace」段。
- **PLE**：各模組（NNU、SMP、RKG、FKU、SCU）的單元測試向量由 golden model 另外輸出（待做）；PLE 與 CSE 的互動先用 CSE BFM 測（待做，`tb/tb_if_checker.sv` 裡的 CSE 端模型是它的雛形）。
- **整合**：PLE + CSE RTL 跑 699 題，每題的迭代數、樹大小、路徑雜湊與 golden model 逐題比對（`tools/cmp_fx.py` 的作法）。

## 跨區塊必須一致的常數

| 常數 | 值 | 用到的區塊 | 定義處 |
|---|---|---|---|
| workspace LSB | 2⁻¹⁴ m | PLE（FKU 輸出、r_q 表）、CSE、建構引擎 | `ple_cse_pkg`（`COORD_W`、`PT_W` 註解）、`fx_pose.h` |
| w_min | −19497 LSB（floor(−1.19 · 2¹⁴)） | PLE（球心轉換）、建構引擎（點量化） | `ple_cse_pkg::W_MIN_LSB`、`fx_pose.c` |
| 查詢格 / 細格位移 | 11 / 9 | CSE、建構引擎 | `ple_cse_pkg::ENV_QSHIFT` / `ENV_FSHIFT` |
| r_q 表、自碰撞門檻 | 由 golden model 產生（δ_FK = 0.28 mm、r_point = 2.5 mm） | PLE（CTB） | `fx_pose.c` 的 `fx_pose_init` |
| 點座標解碼 `pt_decode` | 恆等（存完整 16 bit） | CSE、建構引擎 | 設計文件 5.1 |

## 整合前檢查清單

- [ ] I1 雙方簽核，設計文件第 4 節標為 v1.0
- [ ] CSE 確認最壞服務時間低於 `MAX_LAT`（65,536）
- [ ] 兩邊的頂層埠與 `rtl/*/…_top.sv` 骨架一致（`make -C Platform/accel lint`）
- [ ] 兩邊的 testbench 都掛 `ple_cse_if_checker`，且 0 個錯誤
- [ ] CSE 通過 golden model 的 trace（0 筆不同）
- [ ] PLE 搭配 CSE BFM 通過整題比對
- [ ] 整合後 699 題與 golden model 逐題相同
