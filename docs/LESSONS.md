# 規則的來源事件（CUSTOM_SoC_Platform，2026-08 ～ 2026-11）

每條規則都來自一次真實的漏網或卡關。新 repo 遇到同類事件時，在這裡加一行，並更新 SKILL.md 或 PROJECT.md。

| # | 事件 | 為什麼漏掉或卡住 | 規則 |
|---|---|---|---|
| E1 | 整天以為「RTL 改壞開機」，接著又以為「PnR 不穩定」 | 真正原因是 build 方法（scratch 目錄 `create_project -force`，0/6 開機；open_project 重用 2/2 開機、cycle 數差 36/7.12 億） | 板子異常先比對 build 流程；兩個假設被推翻就停下討論 |
| E2 | 一整天在查「USB/IP 壞了」 | agent 非互動 shell 讀 UART 永遠 0 bytes，和斷線一模一樣 | 結果是 0 時，先證明檢查工具能抓到東西 |
| E3 | load-use × DIV hazard 被會跑第二輪的 tb 蓋掉 | 第二輪 cache hit 把錯的結果覆蓋成對的 | 比對第一次寫入／每次 commit，不只看最終狀態 |
| E4 | 同一 cache line 不同 word 的 hit 回傳錯 word | 單元 tb 的 hit 剛好都是 offset 0 | 每個欄位掃所有值；單元過了還要跑整合 |
| E5 | tb 期望值打錯、harness 每拍都前進 | tb 本身有 bug | tb 用參考實作比對，或植入 bug 確認會 FAIL |
| E6 | 上游 rv32um 對有 bug 的除法器全部 PASS | 36 組向量太少 | 用 mutation 衡量測試強度；依演算法結構 sweep（自製套件抓到 40/4518） |
| E7 | `fence.i` 不寫回 dirty D-cache | 沒有測試跑自我修改的程式 | 採用標準 compliance 套件 |
| E8 | RAS hazard bug 早就存在 | 沒有中斷時很少觸發 | 隨機中斷注入 + 與無中斷 run 比對 trace |
| E9 | 清 msip 後 `mret` 再次 trap | tb 的 bus 0 延遲，實際約 4 拍 | bus 模型預設真實延遲，並掃延遲 |
| E10 | 加 CLINT 後板子完全沒輸出（UART DECERR） | tb 用軟體位址建 bus，不知道 BD address map | 模擬抓不到 address map 錯誤 → 靜態檢查 + 人類檢查點 |
| E11 | `.sbss` 沒清 0，板上 ASSERT | tb 記憶體預設 0 蓋掉問題 | 初始狀態比硬體惡劣（非零 pattern、隨機初值） |
| E12 | CPU clock 常數寫錯，晶片內無法察覺 | mtime 和 mcycle 數同一個 clock | 需要外部參考的性質用 host wallclock 量 |
| E13 | 別人的模組被切 pipeline 後 revert | 不是自己負責的模組 | 改別人的模組或介面前先問 |
| E14 | 80 MHz 版本差 23 ps 又用了錯的 build 方法 | 兩個風險疊在一起 | 交接時風險列最前面 |
| E15 | 兩次 build WNS 差 0.3–1.1 ns | PnR 變異 | 一組 WNS 差不能歸因，比對 path family |
| E16 | Bash 安全檢查連續 10 次沒回應 | 工具故障 | 重試 2 次就停下說明 |
| E17 | 工作 03:25 跑完，等待迴圈跑到 03:46 | `while pgrep -f` 比對到自己，又沒上限 | 用 PID 等、設上限、先證明等待會結束 |
| E18 | `( make ... & )` 背景工作完成沒通知 | 脫離 harness | 直接用 `run_in_background` |
| E19 | Vivado 擋了約 7 分鐘 | `source` 了會 `exec vivado` 的包裝腳本 | 不 source 包裝腳本；EDA 只用 batch |
| E20 | 停 watcher 時連自己的 shell 一起殺掉 | `ps \| awk '/pat/' \| xargs kill` 比對到自己 | 用 TaskStop 或確認過的 PID |
| E21 | （PLE-CSE，2026-11-13）兩個回歸同時跑，log 互相覆蓋，只好全部停掉重跑 | 已用 `run_in_background` 啟動後，同一輪又手動啟動了第二份；腳本沒有防止重複執行 | 背景工作啟動後不再手動啟動同一個工作；共用輸出的腳本應該加鎖 |

## 其他從經驗得到的觀察
- 泛泛的角色設定（「你是資深硬體工程師」）幫助很小；有效的是具體的不變量、模擬盲點、PASS 條件，以及「沒看過作者推理的獨立 reviewer」。
- 參考外部設計：標準介面讀規格書；效能點子只借概念不搬 code；repo 特有的耦合（buffer、握手）不要硬套別人的設計。
- 不要加讓失敗變通過的開關（關掉檢查、`|| true`）。
- 外部程式碼進 repo 前做安全檢查並記錄來源（VENDORED.md）。
