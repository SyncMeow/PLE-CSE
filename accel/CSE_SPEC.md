# CSE 完整實作規格：Dispatcher、CU Array、Receiver

日期：2026-11-10。狀態：交接規格，功能及效能尚待實作驗證。

## 1. 目標與範圍

使用可綜合 SystemVerilog，實作由輸入 FIFO、dispatcher、多個完整 CU、receiver 與輸出 FIFO 組成的 CSE。

正常球分派到一個 CU，由該 CU 完整完成查詢；在輸入 FIFO 或尚未展示 CU valid 的 dispatcher 暫存中被取消的球，直接產生 KILLED 回應，不進 CU。CU 內是可停等的串流 pipeline，不設中央多球查詢槽，也不把球、voxel、點工作拆成跨 CU 共用工作池。

支援外部 K(tag)/K(all)，不做 auto-kill。未來可比較更多分派策略，本版只要求可接收者優先的 round-robin。

本文件將原單 CU 交付範圍擴充為完整 array。`CSE_CU_SPEC.md` 仍定義 CU 內部細節；其「本次只實作單 CU」限制由本文件取代。CU 外部環境記憶體、SystemVerilog packed struct 與數值/取消正確性要求維持。

## 2. 必讀來源與介面約束

路徑相對 repo 根目錄：

- `records/20261107-ple-cse-design.md` 第 4、5、13 節。
- `Platform/accel/rtl/common/ple_cse_pkg.sv`：對外位寬、編碼及環境格式。
- `Platform/accel/CSE_GUIDE.md`、`INTERFACES.md`。
- `Platform/accel/CSE_CU_SPEC.md`：CU 各級功能與驗證細節。
- `Platform/accel/rtl/cse/cse_env_mem.sv`：實際環境讀取介面。
- `Platform/sw_rrtc/fx/fx_env.c`：golden `fx_cse_ref()`。

PLE Q/R/K 的埠名、方向、位寬與握手規則不變；cse_top 可新增編譯期參數，不新增 PLE 訊號埠。支援 kill 是使用者對原文件「v1 可忽略 kill」的更新，不改變 C1–C6 合約。不要自行改動 PLE、環境格式或 memory 埠/延遲。

## 3. 整體架構

```text
                            cse_top
PLE Q -> Kill-aware Input FIFO -> Dispatcher --正常球--> CU array
                                       |                   |
                                   取消球                   | CU 回應
                                       v                   v
                              KILLED 回應保持器 -------> Receiver
                                                           |
                                                           v
                                                    Response FIFO -> PLE R

PLE K -> Input FIFO / Dispatcher / 所有 CU 的取消控制
所有在途工作與回應 -> 頂層 idle / error / outstanding 彙整
cse_env_mem(NRD=N_CU) -> 每個 CU 一組 DIR/CELL/PTS 埠
```

每 cycle 最多從 PLE 接受一顆球、向一個 CU 分派一顆球、收集一筆 CU 或取消旁路回應、向 PLE 送出一筆回應。四者可同時發生。

每個 CU 可在不同 pipeline 階段保存不同球，不必 idle 才能接新球。各 CU 可按本地順序回應；整體允許亂序。

## 4. 必須參數化的配置

### 4.1 頂層主要參數

| 參數 | 建議初值 | 要求/實驗範圍 |
|---|---:|---|
| N_CU | 4 | >=1；測 1/2/3/4/8，須支援非 2 次方 |
| INPUT_FIFO_DEPTH | 8 | >=1；測 1/2/4/8/16 |
| RESPONSE_FIFO_DEPTH | 8 | >=1；測 1/2/4/8/16 |
| KILL_ENABLE | 1 | 0/1，取消效益對照 |

兩個 FIFO 深度須獨立配置，不強迫相等；不要求深度 0 旁路。這些是 elaboration-time 參數，本版不增加執行時設定暫存器。

### 4.2 建議保留的 CU 參數

| 參數 | 建議初值 | 用途 |
|---|---:|---|
| CU_VOXEL_FIFO_DEPTH | 2 | 各 CU 的 S1->S2 緩衝 |
| CU_CELL_FIFO_DEPTH | 2 | 各 CU 的 S2->S3 緩衝 |
| CU_MAX_PTS_OUTSTANDING | 8 | 各 CU memory 至距離完成的最大在途點數 |

本版所有 CU 使用相同參數，由頂層向下傳遞。不要求異質 CU 配置。

FIFO 指標/計數、CU 編號、round-robin 指標寬度由參數推導；處理 N_CU=1、DEPTH=1 的零寬陷阱。非 2 次方的回繞必須顯式判斷上限，不能只靠截位。

### 4.3 本版不需要的實驗參數

- 距離 POINT_LANES 固定 1：memory 每組每 cycle 只回一個完整點。
- 距離切級先固定，不提供任意 pipeline 級數切換。
- dispatcher/receiver 策略先固定 round-robin，不增加策略選擇器。
- ENV_RD_LAT=1、TAG_W/USER_W、網格及資料位寬引用共用 package，不獨立改值。
- memory stall、r_ready stall、kill 機率與亂數 seed 放 testbench，不改正式 memory 合約。

## 5. SystemVerilog 與資料型別

- 使用 `typedef struct packed` 封裝成組訊號，內部模組埠、FIFO payload、pipeline 暫存器使用具名型別。
- 直接重用 `query_t`、`resp_t`、`kill_t`、`dir_t`、`cell_t`。
- CU context 與 stage token 型別集中於 `cse_cu_pkg.sv` 或明確的 CSE 私有 package，避免重複定義。
- FSM、token 種類使用 enum。
- valid/ready 與 payload 分開；不同方向資料不可包成同一個 input struct。
- cse_top 與 cse_env_mem 的既有平坦埠在 wrapper 邊界轉換，不為 struct 改動對外合約。
- signed 座標欄位明確標註，不因結構封裝而改變算術 signedness。
- 使用 generate 實例化 N_CU，不手寫固定數量的 CU。

建議模組：cse_top、cse_query_fifo（支援 kill）、cse_dispatcher、cse_cu、cse_receiver、cse_fifo（一般 FIFO）、cse_env_mem。可將小型控制整合，但需維持清楚邊界；已有實作先檢查重用，不無故重寫。

## 6. 支援 Kill 的 Input FIFO

使用專用 query FIFO，直接接收 kill_valid/kill_ready（或已接受的 kill_fire）、kill_tag、kill_all，以及 KILL_ENABLE。它不能只是完全不感知 kill 的一般 FIFO。

### 6.1 保存內容與握手

- 每個 entry 保存完整 query_t、birth_gen，以及獨立的 valid/occupied、killed metadata。
- `q_fire=q_valid && q_ready`：頂層接受一顆球，正式開始一次回應義務。
- FIFO 有空間時可接收，滿時背壓；支援同 cycle enqueue/dequeue。
- 第一版不需 fall-through；新入隊資料不在同 cycle 直接出隊。
- query/birth_gen 為穩定 payload；killed 是 FIFO 自有可更新 metadata，不把會隨 K 改變的 killed 位元混入已展示、等待握手的 payload。
- dispatcher 取得 entry 所有權時，鎖存 query/birth_gen 與「該 entry 的 killed，或同 cycle 命中既有 entry 的 K」。之後才從 FIFO 移除。

### 6.2 Kill 行為

每次有效 K，對握手前已 occupied 的所有 entry 平行比較 tag；符合 K(tag) 或 K(all) 者設 killed=1，其他 entry 不變。

- 不只比較隊首，隊中與隊尾也必須能被標記。
- 重複 K 保持 killed，不重複產生回應。
- 同 cycle 新入隊 entry 的 killed 初始化為 0，不受本次 K 影響。
- 同 cycle 舊 entry 出隊與 K 相遇，dispatcher 必須拿到已取消判定，不能漏掉。
- 同一實體儲存格同 cycle pop/push 重用時，新 entry 的初始化優先，不能繼承舊 killed。
- 不搬移 FIFO 中間資料、不調整其他 entry 順序；取消標記本身不立即釋放空間。

### 6.3 隊首路由與移除

- 隊首正常：移交 dispatcher，之後分派到一個 CU。
- 隊首被取消：移交前端取消完成路徑，形成一筆原 tag/user 的 ST_KILLED；不等待任何 CU q_ready，也不進 CU。
- 取消回應保持器無空間時保留隊首或 dispatcher 所有權，不丟資料。
- entry 只有在 dispatcher 或取消回應保持器確實接管一次回應義務後才能移除。
- 預設取消保持器容量 1，使用 valid/ready；本版不另設其深度參數。

每顆球始終只有一個所有者：FIFO、dispatcher、CU 或已形成的回應路徑。不能一面旁路回 KILLED、一面又把同球送進 CU。

## 7. Dispatcher

### 7.1 分派策略

保存 rr_ptr，從該位置開始循環掃描 CU q_ready，選第一個可接收的 CU。

- 應檢查 q_ready，不是 cse_idle。
- 沒有 CU 可接時，不移除球，等待。
- 選擇結果暫存後再展示目標 CU 的 valid，避免該通道 valid 組合依賴 ready。
- q_ready 的觀察不代表預約成功：下 cycle 目標若不 ready，保持原目標等待，不能換派。
- 最終 CU Q 握手後，rr_ptr 更新為選中 CU 的下一個位置。
- 未握手時不推進 rr_ptr。

第一版接受暫存選擇造成的空泡，不保證每 cycle 都能分派。之後以量測判斷是否最佳化。

### 7.2 Dispatcher 中途收到 kill

- 尚未展示任何 CU valid 的暫存球被 K 命中時，轉交取消回應保持器，不再等待或選擇 CU。
- 已對某 CU 展示 valid、但尚未握手時，不撤回、不修改 payload、不改派。保留原 birth_gen，由 CU 入站比對全域世代並走取消完成路徑。
- 同 cycle 本來準備首次展示 CU valid 又收到 K，若尚未鎖定展示，優先走取消旁路；若已展示則由 CU 接手。無論如何只允許一條完成路徑。
- 取消回應 payload 一旦 valid，保持 tag/user/ST_KILLED 直到 receiver 接受。

### 7.3 必須保證

- 同 cycle 至多一個 CU 的查詢握手。
- 只對選中 CU 展示有效查詢。
- 查詢已 valid 未握手時，payload 與目標固定。
- 每顆頂層接受的球恰好走一條路：交給一個 CU，或由前端取消旁路產生一次 KILLED；不能同時兩者、複製或靜默刪除。
- 不利用 user 的內容決定 CU，不要求同 tag 綁定同 CU。

## 8. CU Array 與環境 memory

實例化 N_CU 個完整 cse_cu，全部共用 clk/rst_n。

環境 memory 在 CU 外部，實例化 cse_env_mem(NRD=N_CU)。每 CU 接一組 DIR/CELL/PTS 獨立讀取埠與一個共用 rd_buf。

- DIR 地址 `(gz*20+gy)*20+gx`；不是 Morton key。
- DIR 8000×16 bit；CELL 128 bit；PTS 單點 48 bit `{z,y,x}`。
- CELL_DEPTH=1024、PT_DEPTH=16384。
- header 為兩份緩衝直接輸出。
- 同步讀延遲 ENV_RD_LAT=1，對齊現有 memory RTL 與自測。
- memory 原始埠沒有 valid/ready。CU/adapter 自行追蹤有效讀取並預留回傳空間。
- 各 CU 可同時讀取。現有參考模型多埠不代表最終 FPGA 免費擁有這些埠；banking/複製及資源估算需另外量測。
- PLE 保證同一題固定 env_buf，只在全域 idle 後交接環境；不可只看到某個 CU idle 就切換或覆寫緩衝。

### 8.1 CU 內部摘要

依 CSE_CU_SPEC.md：

1. S0：球資料、r²、header、全域 AABB、格子範圍。
2. S1：x/y/z 列舉格子，產生 VOXEL，最後 END。
3. S2：DIR/CELL 查詢、局部 AABB 剔除，產生 CELL/END。
4. S3：逐點讀取，經相減、平方、加總、比較 pipeline；收到 END 且所有點排空後形成一次回應。
5. S4：保持回應直到 receiver 接受。

每個 CU 自行處理取消及在途點記帳；receiver 不負責判斷碰撞。

## 9. Receiver 與 Response FIFO

各 CU 的 S4 已提供一筆回應保持，不先增加每 CU 私有回應 FIFO。

Receiver 採 round-robin，在 N_CU 個 CU 回應及 1 個前端 KILLED 回應保持器之間選擇，共 N_CU+1 個來源，將完整 resp_t 送到共用 Response FIFO。來源數與索引位寬由 N_CU 推導，不另設可不一致的參數。

- 同 cycle 至多一筆來源回應被接受、至多一次 FIFO 寫入；CU 與取消旁路同時有效時照常仲裁。
- 只有選中的來源能完成 R 握手，其他 CU/取消保持器保持結果。
- FIFO 無空間時停止收取，不丟棄任何回應。
- 已向 FIFO 展示有效 payload 而尚未接受時，鎖定來源與 payload。
- round-robin 指標只在實際接受一筆回應後更新。
- 接受某來源後，從其下一個來源開始下一輪；CU 與取消旁路都不得被無限飢餓，不採永久高優先權的取消旁路。
- receiver 的輸出 valid 不組合依賴其 ready；CU 的 r_valid 不依賴 receiver ready。

Response FIFO 直接對 PLE 提供 r_valid/r_ready 與隊首 payload。r_ready=0 時輸出不變；支援同 cycle enqueue/dequeue。FIFO 裡的 HIT/MISS/ERROR/KILLED 都視為已形成結果，不受後續 kill 改寫。

兩個 CU 同時完成是正常情況：一筆先收，另一筆保留在該 CU 的 S4，後續再收。

## 10. Kill：array 的時間範圍與第一版取捨

### 10.1 對外時間邊界

以 PLE 與 cse_top 的 Q/K/R 握手定義生命週期：K 握手作用於此前已接受、尚未 R 握手的符合 tag 球。與 K 同 cycle 才 Q 握手的新球不受影響。

合約允許受 kill 影響的球回 KILLED 或真正結果，因此不是所有受影響球都必須停算。

### 10.2 所有尚未完成工作均可取消

採用全域每 tag 的 kill generation，從 cse_top 的 Q 握手建立球的取消識別，不再忽略 Input FIFO 或 dispatcher 中的 kill。

```text
全域 kill_gen[tag]：每次對應 K 握手增加
Input FIFO payload：{ query_t query; birth_gen; }
已取消 iff birth_gen != kill_gen[query.tag]
```

- Q 接受時捕捉 birth_gen，此值一路傳過 FIFO、dispatcher 與 CU，不在 CU 入站時重新取值。
- Q 與命中同 tag 的 K 同 cycle 握手，新球保存增加後的 generation，避免被同 cycle K 取消。
- 比對當 cycle 既有工作時，使用包含當次 K 的 effective_gen，使內部轉移同 cycle 的取消也不遺漏。
- 每個 CU 使用同一份頂層世代資訊，不另建互不相容的本地 kill 世代。
- K(all) 同時增加所有 tag 的世代；後續新球保存最新值，正常執行。
- generation 位寬及回繞防護依 CU 詳細規格；只有整個 CSE 排空且沒有同 cycle 接收新球時，才可安全重新建立世代基準。不能因單一 CU idle 就清除。

| 球所在位置 | K 到達後的策略 |
|---|---|
| Input FIFO | 直接接收 K，標記所有符合的既有 entry；到隊首後走 KILLED 旁路，不進 CU |
| Dispatcher 尚未展示 CU valid | 標記取消並轉交 KILLED 回應保持器 |
| Dispatcher 已展示 CU valid | 保持握手，攜帶 birth_gen 進 CU，由全域世代辨識取消 |
| CU 入站/S0 | 若世代不符，跳過環境查詢，沿 END 路徑產生 KILLED |
| CU 其餘階段/內部 FIFO | 停止新工作，保留 END 與在途記帳，讀取排空後回應 |
| 取消回應保持器 / Receiver / Response FIFO | 已完成的結果保留，不修改等待握手的 payload |

輸入 FIFO 的 killed metadata 明確由 K 更新；全域世代是補足 dispatcher/CU 交接及穩定 payload 的機制，不取代 FIFO 的 kill 功能。取消 entry 仍按隊首先後移除，但不再送進 CU。取消不保證當 cycle 釋放空間。

### 10.3 廣播與同 cycle 分派

- k_ready 固定 1，KILL_ENABLE=1 時，頂層同步更新全域世代，並將同一有效 K 廣播給 Input FIFO、dispatcher 及所有 CU。
- 已接受的舊球在 K 同 cycle 從 FIFO 或 dispatcher 進入 CU，仍應被取消，不能當成本地新球而漏掉 K。
- 唯一必須排除本次 K 的新球，是與 K 同 cycle 在 cse_top Q 握手的球。
- 內部 valid/payload 不因 K 改寫；取消由 birth_gen 與世代側帶資訊比對，不破壞握手穩定性。
- KILL_ENABLE=0 時世代不因 K 更新，FIFO killed 不因 K 設定、前端不產生取消回應，全部球回真正結果，作為效能對照。
- 發現 HIT/ERROR 或已鎖定完整結果時，可保留真正結果，不要求覆寫成 KILLED。

### 10.4 取消後仍要回應

每筆全域接受的球始終保持一次回應義務。CU 被取消時保留 END、tag/user 與排空記帳；不能把整個 stage 清空且不回應。已展示的內部/外部 payload 不因 K 改變。

## 11. 在途計數、idle、error、reset

### 11.1 全域在途計數

```text
outstanding_next = outstanding + top_q_fire - top_r_fire
```

不在 dispatcher 送出、CU 完成或 Response FIFO 入隊時扣除。每次增減合併更新，檢查不下溢/溢位。

計數位寬依輸入 FIFO、dispatcher 暫存、所有 CU 的最大在途容量、receiver 暫存、前端取消回應保持器、回應 FIFO 的容量上界推導，或提供保守固定寬度並斷言上界。這是數量計數，不是中央查詢槽。

cse_idle 當且僅當沒有全域在途球且無待生效 K。K 即時廣播、沒有 K queue 時，outstanding=0 即可作為主要判定，另以斷言交叉檢查所有 FIFO/暫存/CU 排空。不可直接只 AND 各 CU idle。

### 11.2 Error

cse_err 為頂層黏著位元，任一 CU 確認 ERROR 即設定，僅 reset 清除。不能只看當 cycle receiver 是否選到 ERROR 來源。實際檢查並確認無效環境的球逐筆回 ERROR；前端已取消且尚未檢查環境的球可直接回 KILLED，不因此設定 cse_err。

### 11.3 Reset

單一 clk、同步低有效 rst_n；PLE/CSE 同時 reset。清空 FIFO（含每個 entry 的 killed）、前端取消回應保持器、分派/仲裁鎖定、rr_ptr、CU 狀態與有效標記、在途計數及 cse_err。重置丟棄舊查詢，不補回應。memory 內容不需清空，舊讀取回傳靠有效標記失效。

重置期間及重置後第一個 cycle，本端輸出 valid 為 0。

## 12. 正確性與時限

- 精確整數 HIT iff 存在點滿足三軸平方和 <= q_r²。
- q_r 已含誤差，不再增加；pt_decode 恆等。
- 本版採 signed 19-bit 差、unsigned 38-bit 平方、40-bit 加總、28-bit r²，明確擴展。
- 球心已相對 w_min，不再扣除原點；格子上下限使用 signed 算術右移及 workspace 交集判斷。
- 最大球可能 9×9×9 格，不能寫死每軸 3 格。
- 每顆球恰好一筆回應，tag/user 不變；KILLED 必須對應有效 K。
- ready 持續允許收回應且有在途球時，全域回應間隔 <= MAX_LAT=65,536 cycles，並通過既有 checker 的首筆等待檢查。
- 時限以整個 CSE 對外介面量測，不能只提供單 CU 的時間。dispatcher 空泡、receiver 仲裁與 FIFO 都計入。
- 不保證任意巨大 FIFO/N_CU 組合均滿足時限。每次交付/實驗配置需驗證，不能自行放寬 MAX_LAT。

## 13. 驗證計畫

### 單元

- FIFO：depth=1、非 2 次方（如 3）、空/滿、同時讀寫、背壓保持、reset；kill 命中隊首/中間/隊尾、pop/push/K 同 cycle、實體 entry 重用、同 cycle 新球不取消。
- Dispatcher：N_CU=1/3/4、單一/多個可接收者、全部忙、選中後 ready 降低、鎖定 payload、公平性與恰好分派一次，包括隊首取消不進 CU 的旁路與已展示 CU valid 後取消的接手路徑。
- Receiver：全部 CU 與取消保持器同 cycle 有結果、FIFO 滿、PLE 長時間背壓、鎖定來源、N_CU+1 來源索引及公平性。
- CU：沿用 CSE_CU_SPEC.md 的數值、memory、kill、END/排空與背壓測試。

### 整體

- 配置至少包含 (N_CU, INPUT_DEPTH, RESPONSE_DEPTH)=(1,1,1)、(3,3,3)、(4,8,8)。其他 CU 參數另做小量代表性掃描。
- 正常 trace 至少 s0_p1–p3，建議 10 題以上，HIT/MISS 0 差異。
- 隨機球、邊界相等、R_MAX、負球心、無效環境。
- 查詢/回應亂序，以 scoreboard 配對；不要假設跨 CU 回應順序，也不自行賦予 user 唯一性要求。測試可自訂唯一 user 以方便比對。
- K(tag)/K(all) 發生於 Input FIFO、dispatcher、CU、receiver、Response FIFO；包括 top Q/K 同 cycle、CU Q/K 同 cycle、舊 K 後新同 tag 球。
- 背壓直到兩個 FIFO 滿與 CU 停等，解除後全部排空且無遺失/重複。
- CU 全忙時，已取消隊首仍能進取消保持器，receiver 可用時最終回應，不依賴 CU q_ready。
- 確認取消球不曾同時被 CU 接受和旁路回應；旁路計數與全域在途義務一致。
- KILL_ENABLE=0/1；未被取消的 HIT/MISS 比 golden，KILLED 用全域生命週期檢查。
- 全域 reset 中斷在途資料後重新查詢，不收到舊回應。
- 合法密集環境、最大半徑、無命中長路徑，量測全域最壞回應間隔。

掛既有 ple_cse_if_checker，eot 時確認全部回應；lint 0 警告、無 latch。至少一個錯誤植入版本被抓到。增加明確 Makefile target，讓單元及整體測試可重跑。

## 14. 效能統計與交付

至少輸出模擬統計：總 cycles、頂層接受/回應數、兩個 FIFO 峰值佔用量與滿載 cycles、dispatcher 等 CU cycles、各 CU 接受/完成數、receiver 等待/滿載 cycles、DIR/CELL/PTS 有效讀取數、KILLED 數（區分前端旁路與 CU）、最大全域回應間隔。

先不為統計擴充 PLE 埠，可由內部計數器或 testbench 取得。N_CU 與 FIFO 實驗需同時比較吞吐量、等待與最壞回應間隔；沒有綜合資料時不宣稱資源/頻率實測。

Codex 實作時：

1. 先檢查現有工作區；保留他人新增/修改的 CU、FIFO、Makefile，確認可重用部分。
2. 定義/重用 struct，完成支援 kill 的參數化 Input FIFO、dispatcher、取消回應保持器、N_CU+1 來源 receiver、Response FIFO 與 array 連接。
3. 先以 CU stub 驗證分派/收集不遺失、背壓與公平性，再接真實 CU/memory；stub 也須保存 birth_gen 以測取消。
4. 完成全域 kill 時間範圍、在途計數、idle/error/reset。
5. 跑代表性配置、trace、隨機/kill/背壓、checker、錯誤植入與最壞時限。
6. 更新對接文件的本版選擇，交付架構、參數、測試指令/結果與尚未達成項目。

產物放 Platform/build/accel，不進 git。RTL 放 rtl/cse/cse_*.sv，測試放 tb，使用 Verilator 5.032 與 --assert。未被要求時不提交 commit 或發布；本交接要求實作完整 CSE，不以單 CU 或 stub 測試通過作為最終完成。
