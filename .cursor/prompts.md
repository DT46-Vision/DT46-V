# 复制即用。一句说清改哪个文件、不要做什么。

## 改一处逻辑

```
只改 src/rm_tracker/src/tracker.cpp 里 <函数名>。
先读这个函数，不要搜整个仓库，不要读 hikSDK 和 README。
改完用 colcon build --packages-select rm_tracker 编译。
不要顺手重构旁边的代码。
```

## 查一个现象

```
现象：<一句话>。
相关包：rm_detector。
先给出最可能的原因和要看的文件，我确认后再改。
不要全量编译，不要读相机 SDK。
```

## 调参

```
只改 src/rm_tracker/config/tracker_params_sentry.yaml 的 <参数名>。
对照 tracker.cpp 里这个参数怎么用，改这一处。
不要改 infantry/hero 的 yaml，除非我点名。
```

## 编译失败

```
这是 rm_detector 的编译错误：
<贴最后 30 行>
只修导致这次错误的代码。编过的包不要重编。
```
