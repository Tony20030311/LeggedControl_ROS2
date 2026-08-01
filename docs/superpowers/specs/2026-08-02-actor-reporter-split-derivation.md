# 把「行為者信任」和「證人資格」拆開 —— 一頁推導

決定用：**是 / 不是**。若是，我照這頁實作；若不是，說哪一步不同意。

對應問題 1️⃣（造謠者在任何 N 都告不倒）。前置的 4️⃣ 已修（`c308787`）。

---

## 1. 現在為什麼告不倒

`trust_total` 把第一手信念和轉述信念相加，兩邊各有上下限：

```
l_self 下限 = l_evict − clamp_step = −9.2 − 2.0 = −11.2      (trust.hpp:273)
轉述總和上限 = +l_max                =            +4.6      (trust.hpp:298)
最負可能的 total                     =            −6.6
定罪門檻 l_evict                     =            −9.2
                                     −6.6 > −9.2  → 永遠告不倒
```

造謠者**自己的位置是誠實的**，所以每個人對它的觀測都佐證它自己的宣稱 → 每一筆轉述都是**正的** →
第一手懷疑撞到下限之後，正的佐證再加回去。

實測 N=5 三個第三方各自 **−6.600**，與 `floor + l_max` 到小數點後三位吻合。

**根因：一個純量同時承載兩件不同的事。**

| 事實 | 現在寫進哪 |
|---|---|
| 「robot2 在它宣稱的位置」（它自己的行為） | `l_self_[2]` |
| 「robot2 謊報 robot1 的位置」（它的證詞品質） | **同一個** `l_self_[2]`（`admm_agent_node.cpp:1360-1363`） |

而 `l_self_[2]` 又被拿去當轉述加權（`:1272`、`:1298`）。**三個角色，一個變數。**

---

## 2. Zikratov 的架構本來就是分開的

`Trust and Reputation Mechanisms for Multi-agent Robotic Systems`（LNCS 8638, pp. 106–120）：

- **Definition 1 — Trust**：對**被評價對象**的信任，決定要不要跟它互動、要不要封鎖它。
- **Definition 2 — Reputation**：對**投票者本身**的評價；低 reputation 的投票者「influence on
  trust computation will be smaller」。
- 結論："an agent needs **not only to execute functions for serving a target** but also **to give a
  correct feedback on the actions of other robots**." —— 明確是兩個要求。

⚠️ **它的 reputation 加權我們已經有了**（`trust_step_relay` 的 `w = trust_prob(l_src)`），
但**餵給它的是同一個混合純量**，所以救不了我們這一種：幫造謠者背書的是**誠實的高信譽狗**，
不是同夥，加權再準也擋不住。

---

## 3. 改什麼

拆成兩個 map，各自只吃一種證據：

| 新變數 | 吃什麼證據 | 用在哪 |
|---|---|---|
| `l_act_[j]` 行為者信任 | 第一手位置殘差（`trust_llr(‖observed − claimed‖)`） | **定罪 / 封鎖**：`trust_fences_peer(trust_total(l_act_[j], l_relay_[j]))` |
| `l_rep_[j]` 證人資格 | smear check（`smear_llr`） | **轉述加權**：`trust_step_relay(..., l_src = l_rep_[i])` |

具體動四處：

1. **`admm_agent_node.cpp:1360-1363`** — 拆成兩次呼叫。第一手殘差進 `l_act_`（`extra_llr = 0`），
   `smear_contrib` 進 `l_rep_`（不帶位置項）。`trust_step_observed` 簽章不用改，兩次都能用。
2. **`:1272`、`:1298`** — `l_self_[i]` → `l_rep_[i]`。
3. **`:1383`** — `total` 改由 `l_act_` 算。
4. **遙測** — `tel` 欄從 `peer:resid:L_self:total:abstain` 變成
   `peer:resid:L_act:L_rep:total:abstain`（六欄）。**`trust_summary.py` / `g5_ab.py` 要同步改，
   否則舊解析器會靜默讀錯欄** —— 就是 `min_pair` 那個坑，不要再踩一次。

---

## 4. 改完的預測數字（可據此判定成敗）

**造謠者 robot2**（自己誠實、謊報 robot1）：

| | 現在 | 改完 |
|---|---|---|
| `l_act_[2]` | — | **+4.6**（位置誠實，維持上限） |
| `l_rep_[2]` | — | **−11.2**（約 8 個 slot 觸底） |
| 定罪 total | −6.600 | **≈ +9.2 → 不定罪** |
| robot2 轉述的權重 | 正常 | `cap = clamp(−11.2, 0, 4.6) = 0`，`w = trust_prob(−11.2) = 1.37e−5` → **貢獻恰好 0** |

**誠實的狗**：`l_rep_` 停在 +4.6 → `cap = 4.6`、`w = 0.990` → **與現在完全相同**。

**a0/a1/a2 位置謊報臂**：攻擊者謊報**自己**的位置 → 打到 `l_act_` → **定罪路徑不變，
5 個 slot 的結果不受影響**。這是主要成果，必須不動。

---

## 5. ⚠️ 這會改掉一個驗收條件的定義 —— 最需要你點頭的一點

驗收條件 5(b) 現在寫的是「**造謠者被抓到**」。改完之後**它永遠不會被抓到，而且那是刻意的**。

理由：
- **它不是安全威脅。** CBF 用位置算安全距離，而它的位置是誠實的 → 它不會撞到人。
- **實測傷害已經是零。** 所有 smear 實驗（N=3 三趟、N=5 一趟）**被造謠的那隻從來沒被踢過**。
- **踢它反而有代價**：少一隻可用機器人，而且屍體禁區會多一個障礙物，在梅花樁那種窄場地是實害。

➡️ 建議把 5(b) 改寫成：**「造謠者的證詞不再計入」**，並用 `l_rep_` 觸底 + 轉述權重歸零來驗收。

**如果你認為造謠應該要能導致驅逐，這頁就不成立，要改走別的方案（降 `l_max` 或降下限），
代價我列在結果文件 §11.3。**

---

## 6. 誰來抓錯

| 守門 | 會不會動 |
|---|---|
| 47 個 Python oracle | **不會** —— 查證過 `python/` 底下沒有任何 trust/l_evict/clamp_step 的鏡像 |
| G1 分散/集中一致性 | **不會** —— loopback 不產生 evidence，`l_rep_` 恆空 |
| `test_trust.cpp`（122 條斷言） | **會，這是主守門**。要補：拆分後兩個變數各自只吃自己的證據、造謠者轉述權重歸零、誠實者不變 |
| `test_false_signal.cpp` | 會，smear 相關案例要更新 |
| Gazebo | smear 臂 N=3 與 N=5 各重跑一趟，比對上表的預測值 |

**做法照 4️⃣ 那次**：先寫一個並排比對新舊行為的小程式，證明「只有該變的變了」，再改正式碼。

---

## 7. 風險

- **中**：動到核心信任加總，但有 122 條斷言 + G1 + 可重跑的 Gazebo 擋著，改壞會當場現形。
- 最容易出錯的是**遙測欄位**（靜默讀錯欄），所以解析器與產生端要同一個 commit 改。
- 可完全還原：一個 commit，`git revert` 即可。

**估時**：實作 + 測試約 1.5 小時；兩趟 Gazebo 驗證約 25 分鐘。
