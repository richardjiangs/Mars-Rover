# vision/ — 电脑端工具集

负责收图、遥测记录与三大任务的后处理（全景 / 伪 3D / 地图）。全部是命令行脚本，Python ≥ 3.9。

```bash
pip install -r requirements.txt
```

| 脚本 | 作用 | 何时用 |
| --- | --- | --- |
| `mock_rover.py` | 假火星车（模拟 Commander 的 WebSocket 接口） | 无硬件时联调 dashboard、生成演示数据 |
| `receive_server.py` | 收图 HTTP 服务，存照片到 `incoming/` | 火星车拍摄时**必须一直开着** |
| `telemetry_logger.py` | 记录遥测为 JSONL 到 `logs/` | 想画地图的行驶全程开着 |
| `stitch_panorama.py` | 全景拼接（OpenCV，失败自动保底横排） | 全景任务拍完后 |
| `make_anaglyph.py` | 左右眼照片 → 红蓝 3D 图 | 伪 3D 任务拍完后 |
| `plot_map.py` | 遥测航迹推算 → 轨迹+障碍点地图 PNG | 录完遥测日志后 |

## 典型流程

### A. 无硬件试跑（今天就能玩）

```bash
python mock_rover.py                 # 终端 1：假火星车
python telemetry_logger.py --host 127.0.0.1   # 终端 2：录遥测（可选）
# 浏览器打开 dashboard/index.html，地址填 127.0.0.1，切 AUTO 让它自己跑一会儿
python plot_map.py logs/telemetry_*.jsonl -o map.png --speed 30   # 画出模拟地图
```

### B. 实车任务日

```bash
python receive_server.py             # 终端 1：收图（车拍的照片都会进 incoming/）
python telemetry_logger.py --host <车IP>      # 终端 2：录遥测
# dashboard 里点"拍 360° 全景" / "拍伪 3D"，或让 AUTO 巡航建图
python stitch_panorama.py incoming/ -o panorama.jpg
python make_anaglyph.py --auto incoming/
python plot_map.py logs/telemetry_*.jsonl -o map.png --speed <T2实测速度>
```

## 说明

- `--speed` 是满油门实测速度（cm/s），来自 `docs/装配与测试指南.md` 的 T2 标定，
  和固件 `config.h` 的 `CM_PER_SEC_CRUISE` 保持一致，地图比例才准。
- 地图是演示级航迹推算（无编码器），误差随里程累积，适合几米范围的房间演示。
- 全景拼接要求相邻照片有约 30% 重叠：舵机步进 30° 配相机横向视场约 60°，正好满足。
