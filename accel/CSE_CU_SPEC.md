# 單一 CSE CU 串流 Pipeline 實作規格

日期：2026-11-09。狀態：交接提案，尚未完成 RTL 或驗證。

範圍更新：完整 CSE 的交付以 `CSE_SPEC.md` 為準，已包含 dispatcher、CU array 與 receiver。本文件保留為 CU 內部詳細規格；以下「本次只實作單 CU」及「不實作 array」的舊範圍限制由完整規格取代。完整 CSE 的輸入 FIFO 直接支援 kill：取消隊首走 receiver 的 KILLED 旁路，不進 CU。已展示 CU valid 的工作不能撤回，仍由 CU 接手取消。取消 generation 統一在 cse_top 接受球時捕捉，原值一路傳入 CU；CU 不能在本地接受時重新捕捉而遺失先前 kill。

## 1. 任務與已確認的選擇

實作單一 CU 的球對環境點雲查詢。使用分級 valid/ready 串流，不設中央多球查詢槽或多球排程器。輸入一次接受一顆球；前級把工作交給下級後即可接收後續工作。不同階段可同時處理不同球，但不保證每 cycle 接受新球。

第一版必須支援 PLE 發出的 K(tag) 與 K(all)，不做 auto-kill。這是使用者對既有「v1 可忽略 kill」約定的明確更新。

沿用現有環境格式 v0、dense DIR 與每 CU 一組讀取埠。第一版每 cycle 最多讀一個點，距離 lanes 固定為 1，距離運算本身切 pipeline。

不實作多 CU cluster、跨 CU 工作池、FK、自碰撞、階層球展開、點雲濾波、Morton 排序與建表。

### 1.1 未來多 CU array 的整合方向

單一 CU 是未來可重複實例化的運算單元。最終架構將由多個完整 CU 組成 array，前後各有電路負責工作分派與回應收集：

```text
PLE Q -> 輸入緩衝 / 工作分派器
                   |
          +--------+--------+
          v        v        v
        CU 0     CU 1     CU N-1
          |        |        |
          +--------+--------+
                   v
             回應收集 / 仲裁器 -> PLE R

PLE K -> kill 廣播控制 -> 所有 CU
環境 memory -> 每個 CU 一組 DIR/CELL/PTS 讀取埠
```

- 前端以各 CU 的 q_ready 分派，每筆被接受的球只能交給一個 CU；球一旦分派就由該 CU 完整處理。
- 後端以 valid/ready 收集回應並仲裁輸出，對各 CU 提供背壓。不同 CU 的回應可以亂序，tag/user 不變。
- 同一 tag 的不同球可以分布在不同 CU，因此 kill 必須廣播，不能只送到某一顆 CU。未來 array 控制須定義一致的 kill 生效邊界，維持「同 cycle 新接受球不受影響」的對外語意。
- array 層彙整 idle（所有 CU 與前後緩衝皆排空）及 error；CU 不需知道其他 CU 的進度。
- 這與把球、voxel、點三種工作拆開成跨 CU 共用工作池不同；本案仍是複製完整的 S0–S4 CU。

本次只實作單 CU，不實作 array、前端分派器或後端仲裁器，但必須保留上述模組邊界，避免未來整合時重寫核心。

### 1.2 本次模組邊界

- 必須提供獨立 `cse_cu.sv`，封裝單一 CU 的完整 pipeline、取消控制與完成管理。
- `cse_top.sv` 作為既有平坦介面的 wrapper，本版連接一個 cse_cu 與一組環境讀取埠。
- `cse_env_mem` 在 CU 外實例化；CU 透過讀取介面取得環境，不在內部私有化整份環境陣列。
- 未來 array 可重複實例化 cse_cu，並把每個 CU 接到 cse_env_mem 的不同讀取埠組。banking、複製與實體頻寬另行評估，不能假設增加 CU 免費得到記憶體頻寬。
- CU 邊界本身必須接受回應背壓、kill 與雙緩衝選擇，不依賴只有一個 CU 才成立的頂層特例。

## 2. 規格來源與優先順序

開始寫 RTL 前，閱讀下列本機檔案，路徑均相對 repo 根目錄：

- `records/20261107-ple-cse-design.md` 第 4、5、13 節。
- `Platform/accel/CSE_GUIDE.md`。
- `Platform/accel/INTERFACES.md`，尤其 I1、I2、I3、I5。
- `Platform/accel/rtl/common/ple_cse_pkg.sv`。
- `Platform/accel/rtl/cse/cse_top.sv`、`cse_env_mem.sv`。
- `Platform/sw_rrtc/fx/fx_env.c` 的 `fx_cse_ref()`。
- `Platform/accel/tb/ple_cse_if_checker.sv`。

PLE 對外合約以設計文件第 4 節及共用 package 為準。本文件規定 CU 內部架構，不能自行改變 Q/R/K 埠、資料格式或 memory 邊界。既有文件中的「v1 忽略 kill」須同步註記為本版已選擇支援外部 kill；既有的 C1–C6 語意不變。

組員 HackMD 的 Morton 建表報告只作背景，不取代目前實際對接的 DIR/CELL/PTS v0。發現新的來源衝突時，記錄差異，不自行改介面。

## 3. 功能與數值

對 q_env_buf 指定的環境回答：

```text
HIT iff 存在一點 p，使
(cx-px)^2 + (cy-py)^2 + (cz-pz)^2 <= q_r^2
```

- 全部精確整數運算，相等算 HIT，不得溢位或截掉有效位元。
- q_r 已包含點半徑和誤差，不再增加。
- 球心 signed 18 bit；點座標 unsigned 16 bit；共同 LSB 為 2^-14 m。
- 座標皆已相對 w_min 表示；CU 不再扣除 w_min。
- q_r unsigned 14 bit，PLE 保證不超過 R_MAX=8192。
- pt_decode 為恆等，點的 16-bit 值就是座標。
- 使用 package 常數與型別，不在各模組重複定義外部位寬。

本版保守採用的運算位寬：

| 項目 | 位寬 |
|---|---|
| c ± r | signed 19 bit |
| c - p | signed 19 bit |
| 每軸平方 | unsigned 38 bit |
| 三軸平方和 | unsigned 40 bit |
| r² | unsigned 28 bit |

運算前明確做符號/零擴展，平方後轉成非負 unsigned；比較時將 r² 零擴展。不可依賴隱含 expression sizing。這比指南的平方與加總建議位寬更寬，語意相容。

## 4. 對外合約

沿用 cse_top.sv 的平坦 Q/R/K 介面，埠名、方向、位寬不變。

- Q：q_valid/q_ready、q_tag、q_user、q_kill_on_hit、q_env_buf、q_cx/cy/cz、q_r。
- R：r_valid/r_ready、r_tag、r_user、r_status。
- K：k_valid/k_ready、k_all、k_tag。
- 狀態：cse_idle、cse_err。

tag 表示邊，可有多顆球共用；user 為不透明欄位，不解碼、不假設其全域唯一。回應原樣帶回 tag/user。

狀態使用 package 的 ST_MISS、ST_HIT、ST_KILLED、ST_ERROR。q_kill_on_hit 保留，但本版不觸發 auto-kill。

每筆 Q 握手恰好有一次 R 握手，重置中止者除外。回應可依輸入順序，本版不追求亂序。r_ready=0 必須正確處理。

## 5. 環境格式與讀取邊界

### 5.1 格式

- 每軸 ENV_G=20，查詢格邊長 2048 LSB，即 12.5 cm。
- DIR 有 8000 項，位址為 `(gz*20 + gy)*20 + gx`，不是 Morton key。
- DIR 16 bit：occ[15]、reserved[14:12]、slot[11:0]。
- CELL 128 bit：base[15:0]、count[25:16]；後續 lo_x/lo_y/lo_z/hi_x/hi_y/hi_z 各 11 bit，引用 cell_t 解碼。
- CELL AABB 為相對格子原點偏移；共同座標為 `(g << ENV_QSHIFT) + offset`。
- PTS 同格連續，單次回傳一個 48-bit `{z,y,x}` 點。
- CELL_DEPTH=1024，PT_DEPTH=16384；Panda 每格最多 64 點。

格內點即使依 Morton 排序，CSE 仍由 dense DIR 查詢，不需要為查表產生 Morton key。

### 5.2 既有 cse_env_mem

每 CU 使用一組 DIR/CELL/PTS 獨立同步讀取埠，三者可以同時讀取。三個埠共用該組 rd_buf，不是每埠獨立選 buffer。

ENV_RD_LAT=1，位址與資料對齊依現有 RTL 與 test-env-mem 確認；不得改變介面延遲。header 兩份緩衝直接輸出，不需讀取等待。

原始 memory 埠沒有 read-valid/read-ready，會持續依位址讀取。內部 adapter 必須自行延遲請求有效標記與 context，忽略沒有邏輯請求的讀值。

若下游可能停等，發讀取前先預留回傳空間。不能要求 SRAM 停住已發出的回傳。

保留建構引擎寫入埠；本版模擬 wr_en 接 0。在 cse_top 內實例化 NRD=1 的 memory，並連接獨立 cse_cu 子模組的讀取介面；不可新增或修改既有對外頂層埠。

同一題 q_env_buf 固定；只在題目間且 cse_idle=1 時切換。這使 S2 和 S3 共用 rd_buf 不會產生跨題選擇衝突。

## 6. Pipeline 與內部工作單位

```text
Q -> S0 球前處理 -> S1 voxel 列舉
                       |
                VOXEL / END 串流
                       v
                 S2 查表與剔除
                       |
                 CELL / END 串流
                       v
                 S3 點讀取及距離
                       |
                    球結果
                       v
                 S4 回應保持 -> R
```

各邊界保持順序：A 的工作、END A、B 的工作、END B。不把不同球的 voxel 混合排程。

END 是每顆球恰好一次的結束標記，不等於 MISS。即使全域剔除、空環境、ERROR 或 kill，仍須產生 END，讓統一完成端回應。

context 包括 tag/user/env_buf、球心、r²、球外接立方體，以及取消識別資訊。為簡化，先隨 token 攜帶，不設共享球心查詢表。

Stage 邊界資料：

| 邊界 | 內容 |
|---|---|
| S0 -> S1 | context、格子上下限、skip_search、error |
| S1 -> S2 | VOXEL(context、gx/gy/gz、DIR 位址)，或 END(context、error) |
| S2 -> S3 | CELL(context、base、count)，或 END(context、error) |
| S3 -> S4 | tag、user、status |

### 6.1 SystemVerilog 與 struct 編碼要求

實作必須使用可綜合 SystemVerilog，不以純 Verilog 或軟體模型取代 RTL。經常一起傳遞、保存或更新的訊號，必須定義成 `typedef struct packed`，並用於內部模組埠、FIFO payload 與 pipeline 暫存器。

- 重用 ple_cse_pkg 的 query_t、resp_t、kill_t、dir_t、cell_t，不建立另一套不相容的同義型別。
- 新增 CU 私有型別集中在 `rtl/cse/cse_cu_pkg.sv`；先編譯 ple_cse_pkg，再編譯 cse_cu_pkg 與模組。不要為了內部整理而改動共用對外型別。
- 建議定義球 context、BALL payload、VOXEL/END payload、CELL/END payload、距離各級 payload、memory 讀取請求及回傳 payload。
- 在各 stage 共用 context_t，內含 tag/user/env_buf、球心、r²、查詢邊界及取消世代；各級再以巢狀 struct 增加自己的資料。
- token 種類與 FSM 狀態使用 enum。VOXEL/CELL 與 END 的識別放在 token struct 中，避免另外維護可能與 payload 錯位的散落旗標。
- valid/ready 維持為獨立方向的握手訊號，例如 input_valid、input_ready、input_payload。不要把相反方向的 ready 與輸入 payload 包在同一個 input struct；若包裝請求/回應，兩個方向分開定義。
- 每軸座標的 signedness 要在欄位型別中明確表達；不得因改成 packed struct 而讓負座標算術變成 unsigned。
- 不用無型別的大 bit vector 加手寫位元切片，取代已有語意的 struct；memory 固定位元格式則用既有 dir_t/cell_t 或明確邊界轉換。
- 優先以整份 payload 暫存、傳遞與重置，降低漏接欄位風險；不同運算級只攜帶後續仍需要的資訊，不盲目複製全部欄位。

既有 cse_top、cse_env_mem 的平坦埠維持不變，於 wrapper/adapter 邊界轉換成內部 struct。struct 整理不改變任何握手、取消或資料格式語意。

## 7. S0：球前處理

一次保存一顆球，完成：

1. Q 握手，保存 context。
2. header valid=0 或 err=1：記錄 ERROR。
3. 有效 header 且 n_pts=0：空環境，直接 skip_search。
4. 計算 r² 與 b_min=c-r、b_max=c+r。
5. 任一軸 b_max<glo 或 b_min>ghi：全域剔除。
6. 各軸先判斷 b_max<0 或 floor(b_min/2048)>19，成立即無格子。
7. 其餘計算 lo=clamp(b_min>>>11,0,19)、hi=clamp(b_max>>>11,0,19)。

算術右移須以 signed 值實現 floor。不得先轉 unsigned。

輸出一筆 BALL；無須搜尋者設 skip_search。輸出握手後才釋放本級儲存。可以使用多 cycle FSM，先不追求 S0 每 cycle 接球。

被 kill 時可以跳過剩餘前處理，但不能刪掉該球；繼續傳遞完成所需 context。

## 8. S1：候選 voxel 列舉

一次保存一顆球的範圍與目前 gx/gy/gz。

- x 最快、接著 y、最後 z。
- 每格產生一筆 VOXEL，只有握手才推進座標。
- 不固定查 3 格；R_MAX 時每軸最多 9 格，共 729 格。
- 所有 VOXEL 發完後輸出 END；END 握手後才能接下一顆球。
- skip_search/error 的球直接產生 END。
- 被 kill 後停止產生新 VOXEL，完成已展示 token 的握手，再產生 END。

已 valid、未握手的 token 不可撤回或修改；接收端自行辨識其取消狀態。

## 9. S2：DIR/CELL 查表與局部剔除

第一版採單一 FSM，一次處理一個 VOXEL：

1. 發 DIR 位址，等待對齊的讀值。
2. occ=0：結束此 VOXEL。
3. occ=1：以 slot 發 CELL 讀取。
4. 解析 base/count/local AABB。
5. 轉成共同座標 AABB，執行三軸區間剔除。
6. 未被剔除才輸出 CELL；count=0 可直接跳過。

局部剔除通過只代表可能碰撞，不能回 HIT。

END 必須等前面的查表與 CELL 輸出完成後才傳出，不可超車。

被 kill 的工作不再發新讀取；已發出的讀取接收後丟棄。可略過同球後續 VOXEL，但仍傳 END。已展示的 CELL 仍遵守握手穩定性。

S2 的 FSM 雖不是每 cycle 接一格，仍可與 S1、S3 重疊運作。先不加入多筆交錯查表。

## 10. S3：點讀取與距離 pipeline

S3 一次處理一顆球的 CELL 串流，保存 context、hit_seen、error_seen、當前 CELL 點進度與 pending_points。

每個 CELL 逐點讀取 base 至 base+count-1。每 cycle 最多發一個有效 PTS 讀取。同一 CELL 可連續發請求；全部請求發完後可接下一個 CELL，不必等先前所有距離結果。

### 10.1 距離級

| 級 | 工作 |
|---|---|
| D0 | 接收有效點，計算 dx/dy/dz |
| D1 | 三軸平方，乘法器後暫存 |
| D2 | 平方加總 |
| D3 | 比較 d²<=r²，更新 hit_seen |

各級有效標記與資料一起停等或前進。固定本版切級，不先做任意切級參數。

### 10.2 在途記帳

- 一筆有效點讀取被邏輯發出：pending_points+1。
- 該點完成 D3，或被安全取消並退出資料路徑：pending_points-1。
- 同 cycle 增減合併更新。
- memory 回傳、距離暫存器與緩衝中的有效點都屬在途。

取消資料可提早退出計算，但必須恰好記帳一次。為降低第一版風險，也可讓已發出的點排空整條距離 pipeline，忽略不需要的結果。

### 10.3 HIT 與 kill

hit_seen=1 或發現該球已取消時，停止發新的點讀取，略過同球剩餘 CELL，繼續消化到 END。已發出的讀取必須接收，不能把稍後回來的舊資料當成新球的點。

HIT 只停止這顆球的工作，不自動取消同 tag 其他球；由 PLE 決定是否送 K。

本版不另加 S3->S1 的 HIT 反向取消路徑。故 HIT 後前段可能繼續走格子，直到 PLE kill 或正常 END。這是本版性能取捨，不影響判定。

### 10.4 完成

收到 END、沒有待發點工作、pending_points=0 後形成一筆結果。按以下優先順序：

1. 已確認環境 ERROR -> ERROR。
2. 已確認 HIT -> HIT。
3. 該球被取消 -> KILLED。
4. 完整搜尋且無命中 -> MISS。

等待排空期間出現的有效 HIT 必須納入判定。結果交給 S4 後，才清除本球狀態並處理下一球。

## 11. S4：回應保持

保存一筆 tag/user/status。r_valid=1 且 r_ready=0 時 payload 固定。握手後釋放；可同 cycle 送舊結果、接新結果。

S4 滿時對 S3 背壓。已形成的真正結果允許在 kill 後照常回傳；已展示的回應不得因 kill 改寫。

## 12. Kill 的精確語意與建議實作

### 12.1 必須保證

K 只影響握手前已接受、尚未回應的符合 tag 查詢。與 K 同 cycle 才 Q 握手的新球，以及之後接受的新球，均不受舊 K 影響。

不能直接清掉整個 stage：一個 stage/FIFO 可能含其他 tag；被取消球的 tag/user 與一次回應義務也必須保留。

kill 必須涵蓋所有內部 FIFO、等待中的輸出與讀取，不只是正在計算的暫存器。球在兩級交接同 cycle 發生 K，也不能漏掉取消。

每球的取消資訊沿原資料路徑到統一完成端，不新增各級獨立 KILLED 回應出口。

### 12.2 建議：每 tag 的 kill generation

為避免「停等 token 不可改 payload，但仍需記住 kill」的問題，建議用小型每 tag 取消世代表，而不是永久 killed[tag] 位元：

```text
kill_gen[16]：每個 tag 的 generation 計數器
球 context：保存 cse_top Q 接受當下的 birth_gen，跨 FIFO 與 dispatcher 保留

K(tag) 握手：該 tag 的 kill_gen + 1
K(all) 握手：全部 tag 的 kill_gen + 1

球被取消 iff birth_gen != kill_gen[球.tag]
```

此處 Q 指 cse_top 的外部 Q。外部 Q 與命中它的 K 同 cycle 握手時，新球保存「增加後」的 generation，因此不受本次 K 影響。CU 本地新接受但早已由 cse_top 接受的球仍屬既有球，不能重新取 birth_gen。獨立 CU 測試 wrapper 可在測試輸入握手時建立同等識別。

需要在 kill 發生當 cycle 立即停止新工作時，使用 effective_gen（包含當次 K 的 next 值）判定既有 context。內部 token 不必改寫，payload 可以保持穩定；移到任何級、停在任何 FIFO 都能以 birth_gen 辨識取消。

整個 CSE 共用一份 16 個 tag 的取消記錄，CU 讀取全域世代；它不是多球查詢槽，也不負責排程。沒有取消的舊球即使同 tag 仍可共存；新球捕捉最新 generation 即可。

generation 必須防止回繞別名。可採 32 bit，模擬加入檢查：generation 即將回繞且仍有在途球時報錯；只有確認整個 CSE 已排空、且同 cycle 沒有接受新球時，才可安全重設。若不採本方案，替代設計必須通過本節所有時序測試，尤其 FIFO 與停等 END 的取消。

S4 已鎖定的結果不依 generation 改寫。generation 變動本身不產生回應，不得把同 tag 的多顆球合成一筆回應。

### 12.3 Kill enable

KILL_ENABLE=1 為本版預設。設為 0 只用於效能對照：k_ready 仍為 1，所有球回真正結果，不產生 KILLED。不影響外部埠。

## 13. 握手、idle 與 reset

- 所有 valid 不組合依賴同通道 ready。
- valid 展示後到握手前，payload 不變；kill 不是例外。
- 只有握手才推進對應的格子/區段輸出進度。
- 不設不受控的組合迴圈；長 ready 鏈必要時用暫存緩衝切開。
- 可用 Q 接受數減 R 送出數的 outstanding_count，判斷對外查詢是否排空；不是 query slot table。
- cse_idle=1 當且僅當無在途球且無尚待執行的取消。k_ready 固定 1、取消立即生效時，不需額外 kill queue。
- cse_err 為黏著位元，確認 ERROR 時設定，重置清除。
- 單一 clk，rst_n 同步低有效；PLE/CSE 同時重置。CU 的有效標記與 adapter 追蹤共同清空，memory 內容不必清除。
- 重置丟棄在途球，不補回應；重置期間及重置後第一個 cycle，所有本端輸出 valid 為 0。

## 14. 實驗參數與計數器

| 參數 | 初值 | 實驗範圍/用途 |
|---|---:|---|
| VOXEL_FIFO_DEPTH | 2 | 1/2/4/8，S1->S2 緩衝 |
| CELL_FIFO_DEPTH | 2 | 1/2/4/8，S2->S3 緩衝 |
| MAX_PTS_OUTSTANDING | 8 | 1/2/4/8/16，含 memory 及距離 pipeline 的在途點 |
| KILL_ENABLE | 1 | 0/1，量測取消效益 |

FIFO 深度先限制 >=1，不實作零深度旁路。預留的回傳/運算空間必須能支援 MAX_PTS_OUTSTANDING，不能只放寬計數上限。

POINT_LANES 本版為 1；不把 8-lane 點供應列入交付。ENV_RD_LAT=1、外部位寬與網格格式引用 package，不獨立掃描。額外延遲與停等可在測試用 adapter 注入，不能修改正式 memory 合約。

建議測量：總 cycles、Q/R 數、各級 busy/stall cycles、DIR/CELL/PTS 有效讀取數、KILLED 數、最大回應間隔。可先用模擬統計輸出，不為此擴充頂層埠。

## 15. 驗證與交付要求

### 15.1 功能

- 空環境、無效 header/err、全域與局部剔除。
- 單點 HIT/MISS、d²恰好等於r²。
- 負球心、workspace 外球心、格子邊界、R_MAX。
- count=1/64、多個 CELL、最後一格最後一點命中。
- 沒有 CELL、只有 END；多球連續輸入、同 tag 不同 user。
- 各級背壓與 r_ready 隨機拉低。
- trace 至少 s0_p1–p3，建議 10 題以上。

### 15.2 Kill

- 分別在 S0/S1/S2/S3、兩個 FIFO、停等 END 時取消。
- 同 tag 多顆球在不同階段，其他 tag 不受影響。
- K(all)、重複 K、Q/K 同 cycle、內部轉移/K 同 cycle。
- 舊 K 之後接受相同 tag 的新球，仍正常判定。
- PTS 讀取發出後取消、pipeline 有多點在途時取消。
- HIT 後 K、回應停等時 K、已取消球只有一次回應。
- KILL_ENABLE=0/1 都要通過介面合約測試。

對未取消的 HIT/MISS，逐筆比 golden；對 KILLED 檢查生命週期是否真的受 K 影響，不要求等於 trace 的 HIT/MISS。未取消 trace 重播必須 0 差異。

### 15.3 既有工具

testbench 必須掛 ple_cse_if_checker；結束拉高 eot，確認全部查詢回應。

```text
make -C Platform/accel lint
make -C Platform/accel test-checker
make -C Platform/accel test-env-mem
make -C Platform/sw_rrtc fx
Platform/build/sw_rrtc_host/rrtc_fx -p 0:9 -n 1 -T <輸出目錄>
```

新增 CSE 自測 target，讓 trace、隨機、kill、背壓測試可重跑。至少一個錯誤植入版本（例如 <= 改 <）必須被抓到。RTL lint 0 警告、無 latch。

### 15.4 延遲合約

有在途查詢且 r_ready 持續為 1 時，相鄰回應間隔不得超過 MAX_LAT=65,536 cycles。首筆回應等待亦由現有 checker 驗證；不能只測平均吞吐量。

建構合法的最大半徑、密集格子、不命中情境，實測單點讀取與各級停等下的最壞時間。若未達標，先調整查表空泡、在途點上限或已固定級數的停等控制；不能自行放寬 MAX_LAT。尚未量測前不得聲稱通過。

## 16. Codex 實作順序與交付

1. 核對來源、現有 Makefile、memory 時序與工作區變更，保留既有使用者修改。
2. 定義 SystemVerilog packed struct/enum、獨立 cse_cu 邊界與 cse_top wrapper，再實作 token/context、緩衝及 memory 對齊邏輯。
3. 串起 S0–S4 的無 kill 路徑，完成 golden trace 與數值邊界驗證。
4. 加入外部 kill、generation 或等價控制，通過完整取消時序測試。
5. 跑 lint、既有 checker、自測、背壓、錯誤植入與最壞延遲。
6. 同步文件中的本版 kill 選擇，提供實際模組圖、參數、測試指令與結果。

中間先驗證無 kill 路徑只是開發順序；最後交付必須支援 kill。未經使用者改變範圍，不換成中央查詢槽架構或多 CU 工作池。

RTL 放 rtl/cse/cse_*.sv，一檔一模組；測試放 tb；產物放 Platform/build/accel，不進 git。使用 SystemVerilog、Verilator 5.032、--assert 與共用 package。未被要求時不提交 commit、不發布，也不修改 PLE 演算法或建構端格式。

交付報告應列明：完成與未完成項目、採用參數、kill 實作、struct/package 與模組邊界、未來 array 的接入方式、trace/隨機/checker/錯誤植入結果、最大回應間隔。硬體資源與時脈只有在完成相應綜合後才能列為實測。
