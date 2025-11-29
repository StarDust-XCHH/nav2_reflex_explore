# REFLEX Explorer — Reactive Frontier Exploration with Local Exemptions

## Explorer & Wander

本Fork对原有功能进行了添补和修改，可以实现的额外功能有：

1. 允许随时使用服务**暂停和恢复**小车的探索。

2. 允许小车再完成探索后，继续进行**无限制漫游，也可进行暂停和回复**

---
##  原始项目

本 Fork 基于仓库：[**kishore-saravanan/nav2_reflex_explore**](https://github.com/kishore-saravanan/nav2_reflex_explore)  
如需参数说明和部署方式等，请前往原仓库。

---

## command


### **0. 开始漫游 (wadering)**

**请于探索完成，生成地图后再执行该功能**

```bash
ros2 run nav2_reflex_explore coverage_wanderer_node
```

### **1. 暂停探索 (exploration)**
```bash
ros2 service call /reflex_explorer/pause_exploration std_srvs/srv/Trigger
```
- **功能**：暂停 `reflex_explorer` 节点的探索任务。
- **使用场景**：当需要临时停止机器人自主探索时使用。



### **2. 恢复探索 (exploration)**
```bash
ros2 service call /reflex_explorer/resume_exploration std_srvs/srv/Trigger
```
- **功能**：恢复之前被暂停的探索任务。
- **使用场景**：在完成其他操作后，重新启动探索过程。



### **3. 暂停漫游 (wandering)**
```bash
ros2 service call /coverage_wanderer/pause_wandering std_srvs/srv/Trigger
```
- **功能**：暂停 `coverage_wanderer` 节点的漫游任务。
- **使用场景**：当需要让机器人停止漫游以进行其他操作（如充电或避障）时使用。



### **4. 恢复漫游 (wandering)**
```bash
ros2 service call /coverage_wanderer/resume_wandering std_srvs/srv/Trigger
```
- **功能**：恢复被暂停的漫游任务。
- **使用场景**：在完成其他任务后，重新开始漫游。





