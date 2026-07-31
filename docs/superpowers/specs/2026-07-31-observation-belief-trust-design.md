# 觀測式信任層設計（Observation-sourced belief layer）

2026-07-31。取代現行 Gate 2 的單發殘差判決。

## 1. 問題

Gate 2 比對「同儕廣播的宣稱」與「同儕自己發的 odom」——**兩條都是被測者自陳**。
同時偽造兩條的攻擊者殘差恆 0，偵測永遠不觸發（Pasqualetti et al. 2013：內容型偵測器
看不見與所有自陳一致的攻擊）。已實測：`LIE=0.30` 全程合作、殘差穩定 0.424 < 門檻，
零 block、零 evict，真實車身間隙被吃到 0.038 m。

判決層另有兩個已實測的失效：LOGONLY 對照臂的 split-brain（兩隻倖存者判決差 9.5 s、
keep-out 圈差 4.5 m）、凍結攻擊者的過期 roster 仍在投票造成 coordinator 死鎖。

## 2. 目標與非目標

**目標**：讓物理安全不再依賴「宣稱誠實」。偵測地板必須低於傷害地板（D_MIN 1.3 −
接觸線 0.867 = **0.433 m**）。

**非目標**：真感知（本 spec 用零保真替身，見 §4）；reputation 加權（N=3 無串謀情境可驗，
留待 N≥5）；改動 ADMM 迭代、HOCBF QP、Laplacian 編隊耦合、`admm_core` 數學、下層控制。

**設計紀律**（使用者要求，優先於一切便利）：泛用規則，不得有情境特化的 fallback。
每個離散行為都必須是同一條規則的讀數，不得是獨立分支。參數必須有物理意義與量測/推導程序。

## 3. 架構

信任層＝slot 級 admission control，掛在現行 `gate2()` 的位置。四步：

```
① 證據  我對同儕 j 的觀測 - j 廣播的宣稱 = 殘差 r_ij   （看不到 j → 無證據，不更新）
② 共享  r_ij 進 AgentState 隨 cycle-head 廣播 → 全隊同一份證據流
③ 信念  每狗對每同儕跑同一條似然更新 → b_ij ∈ (0,1)，決定性 → 全隊同拍收斂
④ 讀數  同一個 b 出三個行為：協作權重 / keep-out 裕度 / 成員資格
```

信念取端點 {1,0} 時精確退化為現行系統（分階段部署用）。

### 3.1 證據（步驟 ①）

- 觀測來源：`peer_truth_`（已存在的 map）。模擬期由 Gazebo plugin 的 `/robotN/hardware/odom`
  餵入——**它由模型世界位姿發布，不經 robot j 的估測器**，故 j 偽造不了。
- 可見性：純函式 `admm::visible(p_i, p_j, obstacles, range, fov)`，線段-圓遮擋測試，
  帶單元測試。不可見 → **不產生證據**（不是產生反向證據）。
- 噪聲：觀測加 N(0, σ²)，per-robot 固定種子（同 OSQP determinism pin 的理由）。

### 3.2 訊息欄位（步驟 ②）

`AgentState.msg` 追加兩欄（加欄位，非改協定；前例 `members[]`）：

```
int32[]   ev_peer     # 這一拍我有觀測到的同儕 id
float64[] ev_resid    # 對應的殘差 r_ij（公尺）；長度必須等於 ev_peer
```

空陣列＝我這拍誰都沒看到，語意上與舊版發送者相同（不影響 loopback 測試與舊 peer）。

### 3.3 信念更新（步驟 ③）

對每個同儕 j 維護對數勝算 L_j（log-odds），初值 0（無先驗）。每 slot：

```
L_j ← λ · L_j + Σ_i w_i · ℓ(r_ij)          # 無證據的 i 不進總和
b_j = 1 / (1 + e^{−L_j})
```

- `ℓ(r)` = 序貫似然比（SPRT 一族）：誠實假設 r ~ N(0, σ²) vs 說謊假設 r ≥ d_min_lie，
  取對數比並上下截斷（截斷防單筆離群值主宰，等價於 Zikratov 的 ±1 量化的連續版）。
- `w_i` = **對證據來源 i 的信念 b_i**（上一拍值）。自陳與轉述一視同仁 →
  抹黑防禦是這條規則的結果，不是補丁。
- `λ` = 遺忘因子（指數衰減）→ 過期證據自然失效，stale-vote 死鎖消失，無需過期規則。

### 3.4 行為映射（步驟 ④）

| 行為 | 映射 |
|---|---|
| 協作權重 | 宣稱進 ADMM 的可信度隨 b 單調；b=1 等同現況 |
| keep-out | HOCBF 半徑/軟化係數是 b 的連續函數；b→0 即現行 corpse（`corpse_keepout` 的 mobile 項是此函數的端點） |
| 成員資格 | b 跌破 b_evict → 走**現行** `members[]` 多數決驅逐路徑；回升過 b_rejoin → rejoin（雙門檻遲滯，防抖動） |

沒有 OK/SUSPECT/EVICTED 狀態機——離散狀態只是 b 的讀數。
新成員/rejoin 無特製觀察期：零證據 → L=0 → b=0.5，隨證據爬升（論文 F(l) 的正規版）。

**分層**：CBF 物理安全層維持獨立於信任層（分層防禦，非 fallback）。

## 4. 保真度限定（必須寫進論文的限制段）

觀測通道是**零保真替身**：ground truth + 噪聲 + range/FOV/遮擋。量的是「殘差路徑與
判決端到端行為」，**不是**感測可行性。真感知（光達/`vision60_perception`）換掉 `peer_truth_`
的來源即可，上層一行不動。

## 5. 參數與推導程序

| 參數 | 來源 |
|---|---|
| σ（觀測噪聲） | 量測：乾淨飛行校準 run 取殘差分布（repo 已有此程序，現行 log 無條件印殘差） |
| λ（遺忘率） | 推導：由「攻陷後行為改變的時間尺度」定，同現行 5-slot danger window 推法 |
| (α, β)（誤殺率/漏檢率） | **需求**，非旋鈕：給定目標錯誤率 → SPRT 標準式導出 b_evict、b_rejoin |
| d_min_lie（待偵測謊言下限） | 綁 0.433 安全緩衝：偵測地板必須低於傷害地板 |

零情境旋鈕：不得為追擊/引誘/潛伏各設一組。

## 6. 實驗矩陣（3 狗，plum arena）

三臂天梯，同一攻擊者、同腳本：

| 臂 | 設定 |
|---|---|
| A0 無防禦 | 偵測關（現有 `LIE_LOGONLY`） |
| A1 自陳 gate | 現行 Gate 2（`obs_gate2=0`） |
| A2 觀測信任層 | 本 spec（`obs_gate2=1`＋信念層） |

攻擊者：**雙通道偽造**（宣稱與 odom 同時造假，即 `ODOM_FAKE`＝`LIE`）——A1 結構上盲、A2 應可偵測。
各臂 n ≥ 3。

**量測**（全部已有工具）：`phys_gap_logger.py` 真實車身間隙（**唯一的接觸判準**）、
`dist.csv` 逐列掃（非 walk_until 稀疏輪詢）、time-to-evict、殘差/信念軌跡、任務完成、
`achieved_rounds`、WBC deactivation。

**驗收**：

1. A2 偵測到 A1 偵測不到的雙通道偽造攻擊者；報出 time-to-evict（**會比現行 0.405 s 慢**，
   累積證據的代價要誠實報，不得隱藏）。
2. NO_KILL soak 零誤殺（噪聲＋不可見情境下）。
3. **遮擋語意**：攻擊者躲柱後 → 無證據、信念凍住不誤殺；現身 → 證據恢復、被偵測。
   （這是自陳架構連表達都表達不了的新宣稱。）
4. 重跑 stale-vote 死鎖場景：遺忘因子應自然消解，無需特例。
5. 回歸：44 oracle + 5 ctest 全過；`b≡1` 時與現行系統行為一致。

## 7. 已知盲區（誠實條款，寫進論文）

對稱 1v1：j 說謊、i 指控 j、j 反咬 i、第三方 k 恰好誰都看不到 → 證據對稱，
似然到不了門檻 → **不開除**（i、j 互相 keep-out，安全不賠、活性降級）。
理論上不可判定（BFT 需 3f+1），非實作缺陷。N 增大時目擊者增多，盲區自動縮小，零改碼。

## 8. 影響面

改：`admm_agent_node.cpp`（gate2 → 信念層）、`AgentState.msg`（+2 欄）、
`fleet_config.hpp/cpp`（`visible()`、信念更新純函式、keep-out 隨 b 縮放）、
`d_run.sh`（接 OBS_GATE2/ODOM_FAKE 與三臂矩陣）、對應單元測試。

不改：ADMM 迭代與 barrier、HOCBF QP、Laplacian 耦合、`admm_core` 數學、
coordinator 指派、下層 OCS2/WBC。G1 bit-identical parity 必須維持。

## 9. 論文定位（附帶結論，非本 spec 交付物）

HOCBF、Laplacian formation、ADMM-DMPC 三者**各自引用來源**，系統段誠實寫成
「整合＋真分散部署」，不對單一元件宣稱 novelty。主貢獻是：
(a) 失效幾何三發現（偵測門檻 > 安全緩衝、攻擊者速度優勢、共識活性死鎖）；
(b) 本信任層——常數由系統自身物理導出，使物理安全不依賴宣稱誠實。
related work 寫作前需再掃一次 ADMM+HOCBF+formation 是否有同款組合的前作。
