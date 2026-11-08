# CSE 實作須知

這份文件整理 CSE 實作者需要知道、而且 PLE 端已經決定的事。完整的背景與理由在 `records/20261107-ple-cse-design.md`（以下簡稱「設計文件」），對接狀態在 `INTERFACES.md`。這裡的內容和那兩份衝突時，以設計文件第 4 節與 `rtl/common/ple_cse_pkg.sv` 為準，並請告訴 PLE 端修正本檔。

## 1. CSE 要做什麼

- **只回答一個問題**：給一顆球（球心、半徑），環境點雲中是否有任何一個點落在球內。
- **CSE 不需要知道的事**：機器人、關節角、FK、路徑、樹、邊的概念。外包球與細球的階層、自碰撞都由 PLE 處理。
- **查詢量**：每個姿態平均 16.4 顆球，總共約 1.17 億筆查詢（699 題 × 10 個 seed）。

## 2. 檔案與指令

| 檔案 | 用處 |
|---|---|
| `rtl/common/ple_cse_pkg.sv` | 兩邊共用的參數、編碼、struct。**位寬一律引用它，不要在模組內寫死** |
| `rtl/cse/cse_top.sv` | CSE 頂層埠骨架。埠列表是合約，不可增減或改名；內部由你實作 |
| `rtl/cse/cse_env_mem.sv` | 環境記憶體（雙緩衝）參考實作，可直接載入 trace。可以換掉內部，埠與延遲不可變 |
| `tb/ple_cse_if_checker.sv` | 介面檢查器。你的 testbench 必須掛上它，且結束時 0 個錯誤 |
| `tb/tb_if_checker.sv` | 檢查器的自測；裡面的 PLE 端驅動器可以拿來當你 testbench 的起點 |
| `INTERFACES.md` | 對接清單與整合前檢查清單 |

指令（在 repo 根目錄執行）：

| 指令 | 用處 |
|---|---|
| `make -C Platform/accel lint` | lint（每個 RTL 檔都要 0 個警告） |
| `make -C Platform/accel test-checker` | 檢查器自測 |
| `make -C Platform/accel test-env-mem` | 環境記憶體自測 |
| `make -C Platform/sw_rrtc fx` | 建置 golden model |
| `Platform/build/sw_rrtc_host/rrtc_fx -p 0:9 -n 1 -T <目錄>` | 產生 10 題的環境影像與查詢 trace |
| `python3 Platform/sw_rrtc/tools/check_cse_trace.py <目錄> <tag> <queries.txt>` | 獨立檢查 trace，也可當格式說明 |

規則：SystemVerilog（Verilator 5.032 能編譯的子集）、單一時脈 `clk`、`rst_n` 低電位有效且同步重置、檔名等於模組名、CSE 的模組用 `cse_` 前綴、註解一行（設計文件第 13 節）。

## 3. 介面（PLE ↔ CSE）

埠與位寬見 `cse_top.sv`。三個通道都用 valid / ready 握手，規則與 AXI-Stream 相同：
1. `valid && ready` 在上升緣成立即完成一次傳輸。
2. `valid` 不可組合依賴同一通道的 `ready`。
3. `valid` 拉高後，傳輸完成前不可拉低，payload 不可改變。

**Q（查詢，PLE → CSE）**：`q_tag[3:0]`、`q_user[15:0]`、`q_kill_on_hit`、`q_env_buf`、`q_cx/q_cy/q_cz`（有號 18 bit）、`q_r`（無號 14 bit）。
- `q_user` 你**不需要解讀**，回應時原樣送回即可。
- `q_env_buf` 選擇查第 0 份還是第 1 份環境。
- 球心可能在 workspace 外，甚至是負值。

**R（回應，CSE → PLE）**：`r_tag`、`r_user`（等於該查詢的值）、`r_status`：

| `r_status` | 意義 |
|---|---|
| 0 MISS | 確定沒有碰到 |
| 1 HIT | 確定碰到 |
| 2 KILLED | 沒算（只有被 kill 過的查詢可以回） |
| 3 ERROR | 環境不可用（例如 header 的 valid = 0 或 err = 1），同時把 `cse_err` 設為 1 |

**K（kill，PLE → CSE）**：`k_tag`、`k_all`。K 在握手的那個 cycle，只作用在「已被接受、回應還沒送出」且 tag 相符的查詢（`k_all` = 1 時作用在全部）；這些查詢可以回 KILLED，也可以回真正的結果。與 K 同一個 cycle 才被接受的查詢、以及之後才被接受的查詢，都不受這次 kill 影響，必須照常回應真正的結果。

**狀態**：`cse_idle`（沒有在途查詢）、`cse_err`（黏著，重置清除）。

**你必須保證的事**（設計文件 4.5，檢查器會自動檢查）：

| 編號 | 內容 |
|---|---|
| C1 | 每筆被接受的查詢恰好回應一次；不能憑空產生回應 |
| C2 | `r_tag`、`r_user` 原樣送回 |
| C3 | MISS / HIT 必須和 golden model 完全一致（見第 4 節） |
| C4 | 沒有被 kill 過的查詢不能回 KILLED |
| C5 | 有在途查詢且 `r_ready` 維持為 1 時，相鄰兩筆回應的間隔 ≤ `MAX_LAT` = 65,536 cycles |
| C6 | `cse_idle` = 1 時，不能有在途查詢 |

**v1 的約定**
- **kill**：可以完全忽略。`k_ready` 接 1，被 kill 的查詢照常算完並回真正的結果。規格允許這樣做，kill 留到效能優化時再加。
- **回應順序**：不限，哪一筆先算完就先回。
- **`r_ready`**：PLE 會一直接 1，但你的設計仍須正確處理 `r_ready` = 0（例如回應先放進 FIFO）。

**你可以自由決定的事**：CU 數量、FIFO 深度、回應順序、內部 pipeline、空格怎麼跳過、局部剔除用哪種方法、環境記憶體內部怎麼分 bank。只要符合 C1–C6 即可。

## 4. 判定的精確定義（C3）

HIT ⇔ 存在環境中的點 p，使得

```
(q_cx − p.x)² + (q_cy − p.y)² + (q_cz − p.z)² ≤ q_r²
```

- **全部用整數**，比較是 `≤`（距離剛好等於半徑算 HIT）。
- **點座標**：存的 16 bit 值就是座標本身（`pt_decode` 為恆等）。
- **r_point 和誤差都已經算在 `q_r` 裡**，CSE 不要再加。
- **位寬**：球心 18 bit 有號減掉點 16 bit 無號，差值要 19 bit 有號；平方 36 bit；三項和 38 bit；`q_r²` 28 bit。不可溢位。

下面是建議的計算步驟，與 golden model 的 `fx_cse_ref` 相同。只要最後的 MISS / HIT 和定義一致，步驟可以調整：

| 步驟 | 做法 |
|---|---|
| 全域剔除 | 任一軸 `c + r < glo` 或 `c − r > ghi` → MISS（glo / ghi 來自 header） |
| 範圍 | 每軸 `lo = clamp((c − r) >>> 11, 0, 19)`、`hi = clamp((c + r) >>> 11, 0, 19)`；若 `c + r < 0` 或 `(c − r) >>> 11 > 19`，該軸沒有格子 → MISS。`>>>` 是算術右移（往負無限大取整）。**格數依半徑而定，不可寫死 3 格**：最大的球（`q_r` = 8192）每軸要走 9 格 |
| 列舉 | 三層迴圈走過範圍內每一格，讀 DIR；`occ` = 0 表示空格，跳過 |
| 局部剔除 | 非空格讀 CELL，得到 AABB：格子原點 `(g << 11)` 加上 lo / hi 偏移。可以用精確的 clamp 距離，或三軸區間比較 |
| 點距離 | 讀 `base` 起的 `count` 個點，逐點比較；一碰到就回 HIT，提早結束 |

golden model 的 C 參考實作在 `Platform/sw_rrtc/fx/fx_env.c` 的 `fx_cse_ref()`，結構和上表一一對應，可以直接對照。

## 5. 環境記憶體（`cse_env_mem`）

- **讀取埠**：每個 CU 一組，含 DIR、CELL、PTS 三個獨立的同步讀取埠，每個埠帶自己的 `rd_buf`。
- **延遲**：1 cycle。位址在第 t 個上升緣送進去，第 t+1 個上升緣之後資料有效。
- **header**：兩份緩衝的 valid、err、點數、格數、全域 AABB 直接用暫存器輸出，不需要等 cycle。
- **深度**：CELL 1024、PTS 16384。全部題目最大 651 格、11,670 點。
- **模擬載入**：`+env0=<trace 前綴>`（第 1 份用 `+env1=`），例如 `+env0=Platform/build/sw_rrtc_host/cse_trace/s0_p1`。
- **建構引擎寫入埠**：v1 接 0（`wr_en` = 0）。
- 詳細埠列表見 `INTERFACES.md` I3 與 `cse_env_mem.sv`。

## 6. 環境格式 v0

| 區塊 | 內容 |
|---|---|
| 網格 | 座標 u16，LSB = 2⁻¹⁴ m（約 61 µm），原點 w_min。查詢格 12.5 cm（`q >> 11`），每軸 20 格 |
| DIR | 8000 項，索引 `(z·20 + y)·20 + x`；16 bit：[15] occ、[11:0] slot |
| CELL | 每個非空格 128 bit：[15:0] base、[25:16] count、[36:26] lo_x、[47:37] lo_y、[58:48] lo_z、[69:59] hi_x、[80:70] hi_y、[91:81] hi_z、其餘 0。lo / hi 是相對於格子原點的偏移，AABB 緊貼格內的點 |
| PTS | 同一格的點連續存放，`{z, y, x}` 各 16 bit |
| HDR | 9 個 word：0 `{err, valid}`、1 點數、2 格數、3–5 glo x/y/z、6–8 ghi x/y/z |

`ple_cse_pkg` 裡有對應的 `dir_t`、`cell_t`，可以直接用 struct 拆欄位。

## 7. 設計規模參考（golden model，699 題 × 10 個 seed，1.17 億筆查詢）

| 量 | 值 |
|---|---|
| 被全域剔除的查詢 | 38.2% |
| 每筆查詢走過的格子 | 平均 6.63；沒被剔除的平均 10.7；最壞 729（`q_r` = 8192） |
| 每筆查詢的非空格 | 1.15 |
| 每筆查詢通過局部 AABB 的格子 | 0.115 |
| 每筆查詢測試的點 | 2.32 |
| HIT 比例 | 4.6% |
| 每格最多點數 | 64（Panda） |

**結論**：瓶頸在走訪空格（約 83% 的格子是空的），不在距離運算。空格跳得越快越好；距離 lanes（P）不必很多。

## 8. 驗證

1. **trace**：`rrtc_fx -T` 產生每題的環境影像與查詢清單（每行 `tag user kill_on_hit env_buf cx cy cz rq expected`）。把環境載入 `cse_env_mem`，依序送出查詢，逐筆比對 `expected`。必須 **0 筆不同**。
2. **隨機球**：trace 裡大多是 MISS（約 95%），所以要再加隨機球：半徑到 8192、球心在 workspace 外、距離平方剛好等於 `q_r²`。golden model 的 `host/fx_envcheck.c` 有這些產生方式；預期值可以用 `check_cse_trace.py` 的暴力法算。
3. **介面**：testbench 掛 `ple_cse_if_checker`，並隨機拉低 `r_ready`、隨機送 K（v1 忽略也必須通過）。結束時要拉高 `eot`，確認所有查詢都有回應。
4. **對照組**：至少要有一個故意改錯的版本被抓到，例如把 `≤` 改成 `<`、範圍少走最後一格、AABB 少一個 LSB。golden model 用這三種都抓到過。

## 9. 交付給 PLE 端前

- [ ] `make -C Platform/accel lint` 0 個警告
- [ ] trace（至少 s0_p1–p3，建議 10 題以上）0 筆不同
- [ ] 隨機球測試 0 筆不同，對照組被抓到
- [ ] 掛 `ple_cse_if_checker` 0 個錯誤
- [ ] 最壞情況（`q_r` = 8192、範圍內全是非空格）的服務時間低於 65,536 cycles
- [ ] 告訴 PLE 端你選的 CU 數 N、距離 lanes P、空格跳過方式，以及實際的最壞延遲
