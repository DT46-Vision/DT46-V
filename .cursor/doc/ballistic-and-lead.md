# 阻力与提前量

分支语境：`Tracker_to_C++`（与 `stable` 思路相同）。

## 提前量是什么

公式就是：

```text
t = (rough_dist / bullet_speed) + system_delay
```

- `t` 是**时间**（秒），不是角度
- `rough_dist ≈ 目标中心距离 − 装甲半径`
- 含义：现在到「子弹大概飞到目标附近」要多久（按匀速估飞行时间 + 发弹延迟）

`t` 用来外推目标未来位姿（位置 / 高度 / 车体 yaw），再选板、算瞄准角。  
目标在动时影响预瞄；静靶速度≈0 时几乎不影响角度。

## 空气阻力用在哪

只用在弹道仰角（`solve_ballistic` / LUT）：平方阻力 + 重力积分，求该抬多少俯仰，好打到板子的**高度**。

**阻力没有进入上面的 `t`。**

## 两者的关系（现状）

| | 有没有阻力 | 管什么 |
|--|------------|--------|
| 提前量 `t` | 无（`d/v`） | 未来瞄哪里 |
| 弹道 | 有（`k_v2`） | 枪口抬多少才能打到高度 |

`d/v` 偏理想：真实有阻力时弹丸变慢，飞行时间往往更长，动目标预瞄容易偏短。  
若用带阻力的飞行时间再算 `t`，通常有利于更好预瞄；那是改进方向，不是现状。

## 代码位置

- C++：`src/rm_tracker/src/tracker.cpp` → `predict_future_state`、`solve_ballistic` / LUT
- stable Python：`src/rm_tracker/rm_tracker/modules/tracker.py` 同名函数（`git show origin/stable:...`）
