# PROJECT ARES — 低成本火星车原型 (Operation MROS)

一辆 150 美元以内的六轮火星车模型：`ESP32 NodeMCU + ESP32-CAM` 双脑架构、`Rocker-Bogie` 摇臂-转向架悬挂、3D 打印四层主舱、可承受 2 米跌落，能自动驾驶、拍 360° 全景与伪 3D 照片、绘制区域地图，并支持电脑端实时监控与人工接管。

![Rocker-Bogie 示意](hardware/drawings/Rocker-Bogie%20摇臂减震示意图.png)

## 当前状态

- ✅ 设计定稿（V3 基线：见 `AGENTS.md` 与 `docs/Ares_V3_决策记录.md`）
- ✅ 硬件已采购（`hardware/drawings/部件.jpg`）、结构件已建模（`hardware/cad/`）
- ✅ 固件、网页控制台、电脑端图像处理脚本已编写
- ⏳ 待完成：3D 打印、补采购、装配、烧录、标定与验收测试 → 见 `docs/下一步你要做什么.md`

## 目录导航

| 目录 | 内容 |
| --- | --- |
| `docs/` | 设计文档、决策记录、装配与测试指南、行动清单 |
| `hardware/` | 引脚表（★`Ares_V3_GPIO_Map.md`）、BOM、接线资料、手绘图纸、STL |
| `firmware/commander/` | 主控 ESP32 固件（Arduino IDE 工程） |
| `firmware/cam/` | ESP32-CAM 固件（Arduino IDE 工程） |
| `dashboard/` | 网页控制台（浏览器直接打开 `index.html`） |
| `vision/` | 电脑端 Python：收图服务、全景拼接、红蓝 3D、地图、遥测记录、模拟器 |

## 三条上手路径

**① 我想了解设计** → 依次读 `AGENTS.md` → `docs/楼层布局.md` → `hardware/drawings/总体设计.jpg` → `docs/Ares_Control_Architecture.md`。

**② 我要造车** → 按 `docs/下一步你要做什么.md` 打印/采购/装配，装配细节见 `docs/装配与测试指南.md`，接线唯一以 `hardware/Ares_V3_GPIO_Map.md` 为准。

**③ 我要跑软件（可以完全不接硬件先试）**

```bash
# 1. 安装电脑端依赖
cd vision && pip install -r requirements.txt

# 2. 启动"假火星车"模拟器
python mock_rover.py

# 3. 浏览器打开 dashboard/index.html，
#    车辆地址填 127.0.0.1 后点"连接"，即可看到遥测并用 WASD 遥控
```

有硬件后：按 `firmware/commander/README.md` 与 `firmware/cam/README.md` 烧录两块板，dashboard 里把地址换成火星车真实 IP。

## 安全提醒

18650 电池必须经 2S BMS，充电只用 2S 专用充电器；HC-SR04 的 Echo 引脚需 1k/2k 分压后再接 ESP32；调试电机先把车架空。完整底线见 `AGENTS.md` 的"安全与工程底线"。

## License

Apache-2.0（见 `LICENSE`）。
