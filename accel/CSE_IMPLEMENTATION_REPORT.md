# CSE 硬體實作報告

> 本報告依據 2026-11-10 的 SystemVerilog 實作整理，描述目前程式碼已完成的行為，而非僅重述設計規格。

## 1. 摘要

CSE（Collision Search Engine）接收一顆以固定點座標表示的查詢球，搜尋環境點雲中是否存在任一點滿足：

```text
(cx - px)^2 + (cy - py)^2 + (cz - pz)^2 <= r^2
```

若存在則回覆 `HIT`，否則回覆 `MISS`。環境無效時回覆 `ERROR`；查詢在完成前被取消時，可以回覆 `KILLED`。所有幾何運算均採整數，因此不會受到浮點捨入模式影響，且距離剛好等於半徑時必須判定為 `HIT`。

目前 CSE 已實作為可參數化的多 CU 架構。預設配置包含 4 個完整 CU、深度 8 的輸入 FIFO 與深度 8 的回應 FIFO。查詢由 round-robin dispatcher 分配給可接收的 CU，完成結果再由 round-robin receiver 收集。不同 CU 的回應可以亂序，但每一筆頂層接受的查詢都只會得到一次回應。

環境記憶體不屬於 `cse_top`。`cse_top` 是 CSE 核心，只提供每個 CU 的環境讀取埠；目前由外層 `cse_system` wrapper 將它連接到 `cse_env_mem` 參考記憶體。

## 2. 設計目標與介面合約

CSE 對 PLE 提供三個 valid/ready 通道：

| 通道 | 方向 | 用途 |
|---|---|---|
| Q | PLE → CSE | 傳入球心、半徑、tag、user 與環境 buffer |
| R | CSE → PLE | 回傳原 tag/user 與碰撞狀態 |
| K | PLE → CSE | 取消指定 tag 或取消所有尚未完成的查詢 |

一次傳輸在時脈上升緣的 `valid && ready` 成立時完成。當 `valid=1` 且 `ready=0` 時，傳送端必須保持 valid 與 payload 不變。

主要欄位如下：

| 欄位 | 位寬 | 說明 |
|---|---:|---|
| `q_tag` | 4 | 查詢識別，可同時使用 16 個 tag |
| `q_user` | 16 | CSE 不解析，回應時原樣帶回 |
| `q_cx/q_cy/q_cz` | 18 signed | 球心，LSB 為 2^-14 m |
| `q_r` | 14 unsigned | 半徑，最大值 8192，即 0.5 m |
| `q_env_buf` | 1 | 選擇雙緩衝環境的其中一份 |
| `q_kill_on_hit` | 1 | 隨查詢保存；目前碰撞判定本身不使用此欄位，取消仍由 K 通道控制 |
| `r_status` | 2 | `MISS/HIT/KILLED/ERROR` |

狀態編碼為：

| 編碼 | 名稱 | 意義 |
|---:|---|---|
| 0 | `ST_MISS` | 已完整判定且未碰撞 |
| 1 | `ST_HIT` | 至少一點位於球內或球面上 |
| 2 | `ST_KILLED` | 查詢遭取消，未作為 MISS 使用 |
| 3 | `ST_ERROR` | 環境 header 無效或帶有錯誤 |

頂層另外提供：

- `cse_idle`：頂層已接受但尚未完成 R 握手的查詢數為零。
- `cse_err`：黏著式錯誤旗標；任一 CU 產生環境錯誤後保持為 1，直到 reset。

實作遵守下列核心正確性條件：

1. 每筆被接受的查詢恰好回應一次。
2. 回應的 tag/user 與查詢相同。
3. `HIT/MISS` 必須與整數 golden model 一致。
4. 未受 kill 影響的查詢不可回覆 `KILLED`。
5. 在 `r_ready` 持續為 1 且仍有工作時，回應間隔不得超過 65,536 cycles。
6. `cse_idle=1` 時不得仍有未完成查詢。

## 3. 整體架構

```text
                         cse_top

 Q ──> kill-aware ──> round-robin ──> CU 0 ──┐
       Input FIFO       dispatcher  ├─> CU 1 ──┤
                                      ...      ├─> round-robin ─> Response ─> R
 K ──> kill generation ────────────> CU N-1 ──┤   receiver       FIFO
                │                             │
                └─> KILLED bypass/holder ─────┘

          N_CU 組 DIR/CELL/PTS 讀取埠與共用 header
                              │
                 cse_top 外部的 cse_env_mem
```

目前模組階層為：

```text
cse_system                         模擬／目前整合 wrapper
├── u_cse : cse_top                CSE core
│   ├── u_query_fifo
│   ├── u_dispatcher
│   ├── u_cancel_holder
│   ├── g_cu[0:N_CU-1].u_cu
│   ├── u_receiver
│   └── u_response_fifo
└── u_env_mem : cse_env_mem        CSE core 外部的環境記憶體
```

各 RTL 檔案的責任如下：

| 檔案 | 功能 |
|---|---|
| `cse_top.sv` | 輸入排隊、取消世代、查詢分派、CU array、回應收集與頂層狀態 |
| `cse_cu.sv` | 單一完整碰撞搜尋運算單元 |
| `cse_cu_pkg.sv` | CU 內部 context、token 與距離 pipeline 型別 |
| `cse_query_fifo.sv` | 可參數化且能逐 entry 標示 kill 的輸入 FIFO |
| `cse_dispatcher.sv` | 鎖定式 round-robin 查詢分派及前端取消旁路 |
| `cse_receiver.sv` | 鎖定式 round-robin 回應仲裁 |
| `cse_fifo.sv` | 一般用途同步 valid/ready FIFO |
| `cse_env_mem.sv` | 雙緩衝環境記憶體參考實作 |
| `cse_system.sv` | 在 `cse_top` 外連接 `cse_env_mem` 的 wrapper |

## 4. 頂層查詢資料流

### 4.1 查詢接受與 Input FIFO

`cse_top` 在 Q 握手時將平坦介面組成 `query_t`，再附加該 tag 的 `birth_gen`，形成 `cse_query_meta_t`。輸入 FIFO 保存完整 payload，並另外為每個實體 entry 保存 `occupied` 與 `killed` bit。

FIFO 支援：

- 任意深度，包含 depth=1 與非 2 次方深度。
- 同 cycle push/pop。
- FIFO 已滿但同 cycle pop 時仍可接受新資料。
- K(tag) 更新所有符合 tag 的有效 entry。
- K(all) 更新所有有效 entry。
- kill 與隊首移交同 cycle 發生時，使用 `head_kill_now`，不會漏掉取消。

FIFO 不會直接丟棄被取消的 entry。隊首仍交給 dispatcher，由 dispatcher 建立唯一的 `KILLED` 回應，因此每筆查詢的回應義務不會消失。

### 4.2 Dispatcher

Dispatcher 內含一筆 hold register。它從 `rr_ptr` 開始循環掃描 `cu_ready`，選擇第一個能接收查詢的 CU。

選定目標後會鎖住 `target_idx`：

- 若目標 CU 暫時取消 ready，查詢仍留給原 CU，不會改派。
- `cu_valid` 與 payload 在握手完成前保持穩定。
- round-robin 指標只在 CU 真正接受查詢後前進。
- 同 cycle 最多只有一個 CU 完成查詢握手。

如果查詢在尚未對 CU 展示 valid 前已遭取消，dispatcher 不再分派，而是送往取消回應路徑。若已經鎖定某一 CU，則保持既有 valid/payload，由 CU 接收後自行依 generation 判定取消。這使查詢只會走「CU 計算」或「前端 KILLED 旁路」其中一條完成路徑。

取消旁路後接一個深度 1 的 `cse_fifo`，讓 `ST_KILLED` 回應能在 receiver 忙碌時保持穩定。

### 4.3 CU Array

`cse_top` 依 `N_CU` 產生多個相同的 `cse_cu`。每個 CU 擁有自己的：

- 查詢入口與回應保持暫存器。
- voxel FIFO 與 cell FIFO。
- DIR、CELL、PTS 讀取位址及資料通道。
- 搜尋狀態與在途點計數。

所有 CU 共用兩份環境 header 與頂層 kill generation。查詢不要求依 tag 固定分配到特定 CU。

### 4.4 Receiver 與 Response FIFO

Receiver 的輸入來源數為 `N_CU+1`：N 個 CU 回應，加上一個前端取消回應來源。它從 round-robin 指標開始尋找 valid source，選中後鎖住來源，直到下游接受該回應。

因此：

- 下游 backpressure 不會使來源或 payload 改變。
- 多個 CU 同時完成時，一次只收一筆，其他 CU 在自己的 S4 保持結果。
- 取消旁路不是永久高優先權，不會使 CU 飢餓。
- 指標只在實際完成 transfer 後更新。

Receiver 後方的共用 Response FIFO 吸收短期 backpressure，並直接驅動頂層 R 通道。已進入回應 FIFO 的結果不再受後續 kill 修改。

## 5. Kill 機制

### 5.1 每 tag generation

頂層為 16 個 tag 各維護一個 32-bit `kill_gen[tag]`。每次 K(tag) 握手時增加對應 tag 的 generation；K(all) 則增加全部 tag 的 generation。

查詢在 Q 握手時保存：

```text
birth_gen = 當下該 tag 的 kill generation
```

之後判定方式為：

```text
cancelled = (birth_gen != kill_gen[tag])
```

這個 generation 會隨查詢通過 Input FIFO、dispatcher 及 CU 全部階段，因此可以取消位於任一內部位置的查詢，而不需要在每一級建立不同的取消 ID。

### 5.2 Q 與 K 同 cycle

`kill_gen_eff` 是包含當 cycle K 握手效果的組合值：

- 既有查詢用 `kill_gen_eff` 比對，因此不會漏掉當 cycle kill。
- 與 K 同 cycle 才完成 Q 握手的新查詢，保存更新後的 generation，因此不受這次 K 影響。

這符合 K 只作用於「K 握手前已接受、但尚未完成 R 握手」查詢的時間語意。

### 5.3 不同位置的取消處理

| 查詢位置 | 取消後行為 |
|---|---|
| Input FIFO | entry 設為 killed，抵達隊首後走取消旁路 |
| Dispatcher 尚未鎖定 CU | 產生一次 `ST_KILLED` |
| Dispatcher 已鎖定 CU | 保持 valid/payload，由 CU 接手取消 |
| CU S0/S1/S2 | 停止或跳過後續搜尋，仍傳遞 END token |
| CU S3 | 停止發出新 point read，排空已在途的 pipeline |
| CU 已形成 S4 回應 | 回應已定案，不再被後續 kill 改寫 |
| Response FIFO | 回應已定案，不再被後續 kill 改寫 |

若 CU 已經確認 HIT，即使同時觀察到取消，S4 的優先順序仍是 `ERROR`、`HIT`、`KILLED`、`MISS`。因此真正完成的 HIT 不會被改寫成 KILLED。

`KILL_ENABLE=0` 是效能與行為對照模式。此時 K 通道仍可握手，但所有查詢都會回覆真正的 `HIT/MISS/ERROR`，不會產生 `KILLED`。

## 6. 單一 CU 微架構

單一 CU 採 S0 至 S4 的串流設計，S1/S2 與 S2/S3 之間各有 FIFO 解耦。每顆球在 token stream 中以一個 END token 結束，使下游即使跳過空格、AABB reject、HIT 或 kill，仍能準確知道查詢何時結束。

```text
Q
│
▼
S0 球預處理
│
▼
S1 voxel 列舉 ──> Voxel FIFO
                       │
                       ▼
                 S2 DIR/CELL 查詢與 AABB 剔除 ──> Cell FIFO
                                                           │
                                                           ▼
                                                     S3 點距離 pipeline
                                                           │
                                                           ▼
                                                     S4 回應保持 ──> R
```

### 6.1 S0：查詢預處理

S0 接收查詢後計算：

- `r2 = q_r * q_r`，保存為 28 bit。
- 球的整數 AABB：`center ± radius`。
- 每軸 voxel 範圍：`(center ± radius) >>> 11`。
- 將 voxel index clamp 到 0 至 19。
- 依環境 header 的 `glo/ghi` 做全域 AABB reject。
- 檢查環境 `valid/err` 與 `n_pts`。

球心是 signed 18 bit、半徑是 unsigned 14 bit；中間的 AABB 與格子計算擴展為 signed 19 bit，以正確處理負座標及 workspace 外查詢。

下列情況設定 `skip_search`：

- header 無效或帶有錯誤。
- 環境點數為零。
- 球 AABB 與環境全域 AABB 不相交。
- 任一軸完全落在 20×20×20 網格之外。

即使 skip，查詢仍會經 END token 到達 S4，產生一次正確回應。

### 6.2 S1：Voxel 列舉

S1 以 x 為最快變化軸，再依序遞增 y、z，列舉 `[lo, hi]` 的所有 voxel。DIR 位址計算為：

```text
dir_addr = (gz * 20 + gy) * 20 + gx
```

最大半徑 `q_r=8192` 時，每軸最多跨越 9 格，總計最多 729 個 voxel。列舉完最後一格後額外送出一個 `CSE_VOXEL_END`；skip 或已取消的查詢則直接送 END。

Voxel FIFO 預設深度為 2，用來隔離列舉器與同步記憶體查詢 FSM 的停頓。

### 6.3 S2：DIR/CELL 讀取與局部剔除

S2 使用四狀態 FSM：

```text
S2_IDLE -> S2_DIR_WAIT -> S2_CELL_WAIT -> S2_OUT
```

處理流程為：

1. 對一般 voxel 發出 DIR read。
2. 下一 cycle 檢查 DIR 的 `occ`。
3. 空格直接跳過；非空格依 `slot` 發出 CELL read。
4. 取得 CELL 後，以 voxel 原點加上 CELL 的 lo/hi offset，重建該格點集合的 AABB。
5. 以三軸區間相交測試做保守剔除。
6. 通過者輸出 `{base, count}` 的 `CSE_CELL` token。

此處採 AABB 對 AABB 測試，而非完整 sphere-to-box 距離，因此可能保留一些最後不會 HIT 的 cell，但不會錯誤移除真正可能碰撞的 cell。這是以少量額外 point read 換取較簡單控制與算術的保守設計。

Voxel END 會轉成 `CSE_CELL_END`，空 DIR、`count=0`、局部 reject 或取消則不產生一般 CELL token。

### 6.4 S3：點讀取與距離 pipeline

S3 依 CELL 的 `base/count` 連續讀取 PTS，最多每 cycle 發出一個 point read。PTS word 為 48 bit：

```text
{z[15:0], y[15:0], x[15:0]}
```

距離資料路徑分成數級：

1. 等待同步 PTS memory 回傳。
2. D0：計算 19-bit signed `dx/dy/dz`。
3. D1：分別平方為 38-bit unsigned 值。
4. D2：三軸平方和形成 40-bit `d2`。
5. 比較 `d2 <= r2`。

雖然規格所需最大平方和可用較少位元表示，實作採 38-bit 單軸平方與 40-bit總和，保留充足的位寬餘裕，避免 signed/unsigned 轉換或加總溢位。

`MAX_PTS_OUTSTANDING` 預設為 8。`s3_pending` 記錄已發出、尚未完成比較的 point 數量；達到上限時停止發出新讀取。遇到 HIT 或 kill 後也停止發出新點，但仍排空已在途資料，避免下一筆查詢收到前一筆的回傳資料。

一旦任何點滿足 `d2 <= r2`，`s3_hit_seen` 會保持為 1，並停止目前及後續 cell 的點讀取。收到 CELL END、沒有 active cell、所有在途點已排空且 S4 有空間時，該球才算完成。

### 6.5 S4：回應保持

S4 是一筆 elastic response register。結果產生後，`r_valid` 與 payload 會保持不變，直到 receiver 拉高 ready 並完成握手。

狀態決策優先順序為：

```text
環境錯誤 -> ERROR
已確認碰撞 -> HIT
遭取消     -> KILLED
其他       -> MISS
```

CU 自身也維護接受與回應計數，用來產生 `cse_idle`。頂層實際使用自己的全域 outstanding 計數作為對外 `cse_idle`，確保包含 Input FIFO、dispatcher、CU、receiver 與 Response FIFO 中的全部工作。

## 7. 環境記憶體與資料格式

### 7.1 核心邊界

`cse_env_mem` 明確位於 `cse_top` 外部。`cse_top` 對每個 CU提供：

- 一個 `rd_buf`，同時選擇該 CU 的 DIR/CELL/PTS buffer。
- 一組 DIR address/data。
- 一組 CELL address/data。
- 一組 PTS address/data。
- 兩份環境共用的 header 輸入。

目前 `cse_system` 只是方便模擬與現階段整合的 wrapper：它實例化 `cse_top` 與 `cse_env_mem(NRD=N_CU)`，並將建構引擎 write port tie low。未來若由 SoC memory subsystem 或建構引擎提供環境，只需在 `cse_top` 外替換這一層連接，不必改動 CSE core。

### 7.2 記憶體內容

環境採雙緩衝，每份包含：

| 區域 | 規格 |
|---|---|
| HDR | 9 個 32-bit word，保存 valid、err、點數、cell 數與全域 AABB |
| DIR | 8000 × 16 bit，對應 20×20×20 voxel |
| CELL | 預設深度 1024，每項 128 bit |
| PTS | 預設深度 16384，每點 48 bit |

DIR word 的 bit 15 是 `occ`，bit 11:0 是 CELL slot。CELL word 保存 point `base/count` 與相對 voxel 原點的局部 AABB。相同 cell 的點在 PTS 中連續排列。

DIR、CELL、PTS 都是同步讀取，`ENV_RD_LAT=1`：在 edge t 提供地址，資料於 edge t+1 後有效。Header 則直接由暫存器輸出，不需要讀取等待。

參考 `cse_env_mem` 以每個 CU 一組邏輯讀取埠描述普通陣列。這能正確模擬介面，但不代表 FPGA 能免費提供大量實體 BRAM port；實際實作時仍需依目標元件評估 banking、複製或仲裁。

## 8. Backpressure、並行性與順序

整條路徑均以 valid/ready 或顯式狀態保存工作，因此 backpressure 可以向前傳遞而不丟資料：

- Input FIFO 滿時降低頂層 `q_ready`。
- 找不到 ready CU 時，dispatcher 保留查詢。
- CU 內 FIFO 滿時停止上游 token 產生。
- S4 忙碌時延後完成下一筆球。
- Response FIFO 滿時 receiver 不再接受來源。
- `r_ready=0` 時 Response FIFO 隊首保持不變。

每個 CU 的 token 流維持查詢先後順序；不同 CU 可以並行搜尋，所以頂層回應允許跨 CU 亂序。PLE 必須使用 tag/user 對應回原查詢，不可假設 R 順序等於 Q 順序。

## 9. 主要參數

| 參數 | 預設值 | 影響 |
|---|---:|---|
| `N_CU` | 4 | CU 數量、環境讀取埠數與 receiver source 數 |
| `INPUT_FIFO_DEPTH` | 8 | 頂層查詢吸收能力 |
| `RESPONSE_FIFO_DEPTH` | 8 | 頂層回應 backpressure 吸收能力 |
| `KILL_ENABLE` | 1 | 是否啟用取消判斷 |
| `CU_VOXEL_FIFO_DEPTH` | 2 | S1 到 S2 的解耦深度 |
| `CU_CELL_FIFO_DEPTH` | 2 | S2 到 S3 的解耦深度 |
| `CU_MAX_PTS_OUTSTANDING` | 8 | 每個 CU 最多在途 point read/compare 數 |

`N_CU`、各 FIFO depth 與 point outstanding 都有 elaboration-time 合法性檢查。`cse_fifo`、query FIFO、dispatcher 與 receiver 的指標寬度也特別處理 depth/source count 等於 1 的情況。

## 10. Reset、錯誤與可合成邊界

所有 CSE 控制狀態使用單一 `clk` 與低有效同步 reset `rst_n`。Reset 會清除：

- FIFO、dispatcher/receiver 鎖定狀態。
- CU pipeline valid 與 active state。
- kill generation。
- outstanding 計數。
- `cse_err`。

程式中的 `` `ifndef SYNTHESIS `` 區塊不屬於合成後的功能電路，內容包括：

- `cse_top` 與 `cse_cu` 的效能統計計數器及結束時報表。
- generation 回繞、outstanding underflow 等模擬期檢查。
- `cse_env_mem` 的 `$readmemh` trace 載入與模擬初始化。

實際 Q/K/R 控制、取消、碰撞運算、FIFO 與錯誤處理都位於這些條件區塊之外，可正常參與合成。合成環境必須由外部建構引擎或記憶體初始化流程填入環境內容，不能依賴模擬用 plusarg 與 `$readmemh`。

## 11. 驗證方法與結果

目前驗證分成模組測試、完整 trace、控制情境、數值邊界、worst case 與介面 checker。

### 11.1 單元與控制測試

| 測試 | 覆蓋內容 | 目前結果 |
|---|---|---|
| `tb_cse_query_fifo` | depth=1、depth=3、push/pop、full reuse、逐 entry kill | 0 error |
| `tb_cse_arbiters` | dispatcher/receiver round-robin、鎖定、backpressure、取消 | 0 error |
| `tb_cse_kill` | K(tag)、K(all)、Q/K 同 cycle、held response、黏著錯誤 | 0 error |
| `tb_cse_control` | 飽和流量、Response FIFO 填滿、backpressure、中途 reset | 0 error |
| `tb_env_mem` | 雙 buffer trace 載入、所有區域回讀、不同 memory depth | 全部相符 |

### 11.2 Golden trace

完整 array 使用 `s0_p1`、`s0_p2`、`s0_p3` trace 回放，共驗證 8,634 筆查詢：

| Trace | 查詢數 | mismatch | checker error |
|---|---:|---:|---:|
| s0_p1 | 1,874 | 0 | 0 |
| s0_p2 | 2,601 | 0 | 0 |
| s0_p3 | 4,159 | 0 | 0 |

同一份 s0_p1 亦以 `N_CU=1/2/3/4/8` 與多種 Input/Response FIFO 深度重跑，結果皆為 0 mismatch。測試期間會隨機施加 response backpressure。

### 11.3 數值邊界與 mutation test

`tb_cse_boundary` 建立 `d²=25`、`r²=25` 的精確邊界案例，確認結果為 HIT。測試流程也自動產生將 `<=` 錯改為 `<` 的 mutant；該錯誤版本會被測試抓到，證明邊界案例具有辨識力。

### 11.4 Worst-case latency

| 測試 | 工作量 | 實測 latency | 限制 |
|---|---|---:|---:|
| 單 CU dense workload | 729 voxels、46,656 point evaluations | 47,396 cycles | < 65,536 |
| 4-CU array legal maximum image | 729 voxels、256 cells、16,384 points | 17,272 cycles | < 65,536 |

兩者皆通過 `MAX_LAT=65,536` 的限制。第二個案例較快是因為它受合法 `PT_DEPTH` 限制，只有 256 個非空 cell 與 16,384 個點，工作量小於第一個刻意讓 729 格各含 64 點的 CU 壓力案例；這不是將單一查詢拆給 4 個 CU 的加速結果。單筆球始終只由一個 CU 完成，不會跨 CU 拆分 voxel，多 CU 的效益是同時處理多筆球。

### 11.5 介面 checker 與 lint

`ple_cse_if_checker` 檢查一次且僅一次回應、tag/user、KILLED 合法性、response 穩定性、最大延遲及 idle 語意。checker 自測會注入八類錯誤，合法流量為 0 error，各故障注入均能被偵測。

目前以下命令均通過：

```bash
make -C Platform/accel lint
make -C Platform/accel test-cse
make -C Platform/accel test-checker
make -C Platform/accel test-env-mem
```

Verilator lint 對 `cse_top`、`cse_system`、`cse_cu`、query FIFO、dispatcher、receiver 與 env memory 均為 0 warning/error。

## 12. 效能特性

目前資料顯示平均查詢的主要成本在 DIR 空格走訪，而非點距離運算：環境約 83% voxel 為空，平均每筆查詢只實際測試少量點。因此本版架構選擇：

- 多 CU 提升球級平行度。
- 每 CU 每 cycle 最多發出一個 point read。
- 以 voxel/cell FIFO 重疊列舉、記憶體查詢與距離運算。
- DIR 空格直接略過。
- 全域 AABB 與 CELL AABB 在點讀取前先行剔除。
- HIT 後提早停止新 point read。

目前 S2 的 DIR/CELL FSM 一次處理一個 voxel，這是清楚且容易驗證的第一版控制方式，也可能是後續提升單 CU throughput 的優先最佳化目標。

## 13. 目前限制與後續工作

1. `cse_env_mem` 是可驗證介面的參考模型，多 CU 多讀埠尚未對特定 FPGA BRAM 結構完成 banking/replication 與資源估算。
2. `cse_system` 將 write port tie low；實際 SoC 整合仍需在 `cse_top` 外接上環境建構或載入模組。
3. S2 每次只容許一筆 DIR/CELL 查詢，尚未利用多筆 memory request outstanding。
4. 每個 CU 每 cycle 最多處理一個點；若目標環境的 cell 點數遠高於目前資料，可再評估多 lane 距離運算。
5. 目前 `cse_err` 只由環境 header 錯誤觸發並保持；尚未建立可由軟體讀取的詳細錯誤原因暫存器。
6. generation 採 32 bit。模擬會在仍有 outstanding 工作時檢查回繞；正常系統中幾乎不可能達到，但長時間形式驗證仍應把回繞假設列入環境限制。
7. 完整 RTL 已通過功能模擬與 lint，但尚未在本報告範圍內提供 synthesis timing、LUT/FF/BRAM 使用量或實板量測。

## 14. 結論

目前 CSE 已形成完整的多 CU、可 backpressure、可取消且能與外部環境記憶體解耦的 RTL。其正確性核心包括：整數且不溢位的距離比較、END token 保證每題完整收尾、generation-based kill 覆蓋所有在途位置、鎖定式 round-robin 保證 valid/payload 穩定，以及頂層 outstanding 計數保證正確 idle 語意。

功能驗證已涵蓋 golden trace、不同 CU/FIFO 參數、kill、backpressure、reset、邊界比較、故障注入與 worst-case latency。下一階段工作應集中在目標 FPGA 上的環境 memory 實體化、綜合後資源／時序量測，以及依量測結果決定是否提高 DIR/CELL 查詢並行度。
