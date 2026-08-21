# /tracker/enemy_nav_img 导航数据可视化话题 实施计划

## 1. 背景

分支 `nav_interface` 中 rm_tracker 已切换为 Python 实现
（`src/rm_tracker/rm_tracker/node.py`，C++ 版 CMakeLists 已删除）。
视觉方向导航方下发的数据为 `EnemyCenter`（yaw / x / y / z / enemy_yaw / armor_id / tracked），
经 `/tracker/enemy_datas` 发布（node.py:529-550）。

现有 `/tracker/tracking_state_img` 已绘制敌人 4 块虚拟装甲板与车体中心
（tracker.py:875 `draw_tracking_state_with_snapshot`），但未叠加导航下发数据，
且由 `display` 参数统一控制。

## 2. 目标

新增一个图像话题 `/tracker/enemy_nav_img`：

- 画面 = 敌方 4 块虚拟装甲板 + 车体中心（复用现有绘制逻辑）
- 左上角绘制下发给导航的 `EnemyCenter` 全部字段
- 自带独立 display 开关 `nav_visual_display`
- 文字大小由 `text_size` 参数控制（参数位置移到 `display` 下方）

## 3. 改动清单

### 3.1 src/rm_tracker/rm_tracker/node.py

1. `__init__` 参数声明区（display 附近）新增：
   - `declare_parameter('nav_visual_display', False)`
   - 取值存 `self.nav_visual_display`
2. 新增 publisher（与现有两个图像发布器并列）：
   - `self.pub_enemy_nav_img = self.create_publisher(Image, '/tracker/enemy_nav_img', qos_profile_sensor_data)`
3. `_on_params` 新增分支：
   - `elif name == 'nav_visual_display': self.nav_visual_display = value`
4. `res_img_cb` 调整：
   - 门控改为 `if not (self.display or self.nav_visual_display): return`
   - `nav_visual_display` 为真时：由 snapshot 重建 EnemyCenter 数值
     （tracked / yaw / x / y / z / enemy_yaw / armor_id，与发布给导航的字段同源），
     调用 tracker 新绘制函数并发布 `/tracker/enemy_nav_img`
   - LOST 时同样发布（面板显示全 0、tracked=False）
5. 新增代码按 AGENTS.md 风格加类型标注、注释只解释"为什么"

### 3.2 src/rm_tracker/rm_tracker/modules/tracker.py

1. 从 `draw_tracking_state_with_snapshot` 抽出第二层整车绘制逻辑为
   `_draw_virtual_robot(draw, snapshot, tf, imu_rpy)`（4 板 + 中心 + 对角连线 + 速度箭头）
2. 新增 `draw_enemy_nav(snapshot, tf, img, imu_rpy)`：
   - 非 LOST 时：调用 `_draw_virtual_robot` 绘制整车结构
   - 无论 LOST 与否：左上角文字面板，按 `snapshot['text_size']` 缩放：
     ```
     [NAV] tracked: True/False
     yaw      : xx.xx deg
     x        : x.xxx m
     y        : x.xxx m
     z        : x.xxx m
     enemy_yaw: x.xx rad
     armor_id : n
     ```

### 3.3 4 份 tracker_params yaml（generic / infantry / hero / sentry）

每个文件 `display` 下方插入：

```yaml
text_size: <原值>         # 从文件底部移上来
nav_visual_display: false # 是否发布 /tracker/enemy_nav_img 导航可视化图像
```

- 各变体 text_size 原值：generic 2.0 / infantry 1.5 / hero 2.0 / sentry 2.0
- 删除文件底部原 text_size 条目

### 3.4 不改动

- `Kielas_Vision.perspective`（本次不修改，之后在机器人 rqt 手动添加 Image View 选话题）
- `/tracker/gimbal_control`、`/tracker/enemy_datas` 发布逻辑

## 4. 验证（机器人上执行）

```bash
colcon build --symlink-install
source install/setup.bash
# 启动后开启开关
ros2 param set /rm_tracker nav_visual_display true
ros2 topic hz /tracker/enemy_nav_img
ros2 run rqt_image_view rqt_image_view /tracker/enemy_nav_img
```

- 跟踪目标时：4 板 + 中心 + 左上角数据连续合理
- LOST 时：面板全 0、tracked=False，图像仍发布
- 关闭开关后 `ros2 topic hz` 显示无消息

## 5. 风险与备注

- QoS 沿用 `qos_profile_sensor_data`（BEST_EFFORT），rqt_image_view 已验证兼容
- 渲染限频由 `display_fps_limit` 统一控制（30fps）
- 左上角数据从 render_snapshot 重建，与 `enemy_datas` 话题数值同源、帧同步
