#!/usr/bin/env python3
"""ARES 遥测记录器：连上火星车 WebSocket，把每条遥测存成 JSONL 日志。

用法:
    python telemetry_logger.py --host 192.168.137.50          # 车的 IP
    python telemetry_logger.py --host 127.0.0.1               # 连 mock_rover 试跑

生成 logs/telemetry_YYYYmmdd_HHMMSS.jsonl，之后交给 plot_map.py 画地图。
依赖: pip install websockets
"""
import argparse
import asyncio
import json
import time
from datetime import datetime
from pathlib import Path

import websockets


async def run(host: str, port: int, out: Path):
    url = f"ws://{host}:{port}/"
    out.parent.mkdir(parents=True, exist_ok=True)
    n = 0
    with out.open("a", encoding="utf-8") as f:
        while True:
            try:
                async with websockets.connect(url) as ws:
                    print(f"已连接 {url} -> {out}")
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except json.JSONDecodeError:
                            continue
                        msg["pc_ts"] = time.time()  # 用电脑时间做统一时间轴
                        f.write(json.dumps(msg, ensure_ascii=False) + "\n")
                        f.flush()
                        n += 1
                        if msg.get("type") == "telemetry" and n % 25 == 0:
                            print(f"  {n:6d} 条 | mode={msg.get('mode')} "
                                  f"yaw={msg.get('yaw')} front={msg.get('front_cm')}cm")
            except (OSError, websockets.WebSocketException) as e:
                print(f"连接断开（{e}），3 秒后重试…")
                await asyncio.sleep(3)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", required=True, help="Commander 的 IP")
    ap.add_argument("--port", type=int, default=81)
    ap.add_argument("--out", default=None, help="输出文件（默认 logs/telemetry_时间戳.jsonl）")
    args = ap.parse_args()
    out = Path(args.out) if args.out else \
        Path(__file__).parent / "logs" / f"telemetry_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
    try:
        asyncio.run(run(args.host, args.port, out))
    except KeyboardInterrupt:
        print(f"\n已停止，日志在 {out}")


if __name__ == "__main__":
    main()
