#!/usr/bin/env python3
"""假火星车（模拟器）：不接任何硬件，在电脑上模拟 Commander 的 WebSocket 接口。

用途:
    1. 无硬件联调 dashboard/index.html（遥测、驾驶、模式、任务、E-STOP 全可玩）
    2. 给 telemetry_logger.py / plot_map.py 生成演示数据

用法:
    python mock_rover.py [--port 81]
    然后浏览器打开 dashboard/index.html，车辆地址填 127.0.0.1

模拟内容：4m x 3m 房间，超声波用射线求交模拟；AUTO 模式按 AUTO-L2 逻辑
巡航避障；协议与 firmware/commander/comms.h 完全一致。
依赖: pip install websockets
"""
import argparse
import asyncio
import json
import math
import random

import websockets

ROOM_W, ROOM_H = 400.0, 300.0     # cm
DT = 0.1                          # 仿真步长
CRUISE_CM_S = 30.0                # 满油门速度
TURN_DEG_S = 120.0                # 满转向角速度


class Rover:
    def __init__(self):
        self.x, self.y, self.yaw = ROOM_W / 2, ROOM_H / 3, 0.0
        self.mode = "IDLE"
        self.thr = self.turn = 0.0          # 当前实际
        self.cmd_thr = self.cmd_turn = 0.0  # 遥控目标
        self.estop = False
        self.last_heartbeat = 0.0
        self.task = None
        self.task_index = self.task_total = 0
        self.task_timer = 0.0
        self.battery = 7.9
        self.wake = 3
        self.auto_phase = "CRUISE"
        self.phase_until = 0.0
        self.turn_dir = 1

    # 从 (x,y) 沿角度 ang(弧度) 射线到房间墙壁的距离
    def ray(self, ang):
        dx, dy = -math.sin(ang), math.cos(ang)
        best = 1e9
        if dx > 1e-9:  best = min(best, (ROOM_W - self.x) / dx)
        if dx < -1e-9: best = min(best, (0 - self.x) / dx)
        if dy > 1e-9:  best = min(best, (ROOM_H - self.y) / dy)
        if dy < -1e-9: best = min(best, (0 - self.y) / dy)
        return max(2.0, min(best, 300.0))

    def sonar(self):
        a = math.radians(self.yaw)
        return (self.ray(a),
                self.ray(a + math.radians(35)),
                self.ray(a - math.radians(35)))

    def step(self, now):
        f, l, r = self.sonar()
        if self.estop or self.mode in ("IDLE", "SLEEP", "E-STOP"):
            self.thr = self.turn = 0.0
        elif self.mode == "TELEOP-PC":
            if now - self.last_heartbeat > 1.5:
                self.mode = "IDLE"
                self.thr = self.turn = 0.0
            else:
                self.thr, self.turn = self.cmd_thr, self.cmd_turn
        elif self.mode == "AUTO":
            if self.auto_phase == "CRUISE":
                self.thr, self.turn = 0.55, random.uniform(-0.02, 0.02)
                if f < 20:
                    self.auto_phase, self.phase_until = "BACK", now + 0.4
                    self.turn_dir = 1 if r >= l else -1
            elif self.auto_phase == "BACK":
                self.thr, self.turn = -0.45, 0.0
                if now > self.phase_until:
                    self.auto_phase, self.phase_until = "TURN", now + 0.6
            else:  # TURN
                self.thr, self.turn = 0.0, 0.6 * self.turn_dir
                if now > self.phase_until:
                    self.auto_phase = "CRUISE"
        elif self.mode == "CAPTURE":
            self.thr = self.turn = 0.0

        # 运动积分（turn>0 右转 = yaw 减小，与固件差速方向一致）
        self.yaw -= self.turn * TURN_DEG_S * DT
        while self.yaw >= 180: self.yaw -= 360
        while self.yaw < -180: self.yaw += 360
        v = self.thr * CRUISE_CM_S
        a = math.radians(self.yaw)
        self.x = min(max(self.x - v * DT * math.sin(a), 5), ROOM_W - 5)
        self.y = min(max(self.y + v * DT * math.cos(a), 5), ROOM_H - 5)
        self.battery = max(6.8, self.battery - abs(self.thr) * 0.0001)

    def telemetry(self):
        f, l, r = self.sonar()
        t = {
            "type": "telemetry", "mode": self.mode,
            "battery_v": round(self.battery, 2),
            "yaw": round(self.yaw, 1), "thr": round(self.thr, 2),
            "front_cm": int(f), "left_cm": int(l), "right_cm": int(r),
            "thermal": "NORMAL", "inside_temp_c": round(24 + random.uniform(-0.3, 0.3), 1),
            "cam": "READY", "wake_count": self.wake, "final_mission": False,
            "light": 2800,
        }
        if self.task:
            t["task_index"], t["task_total"] = self.task_index, self.task_total
        return t


rover = Rover()
clients = set()


async def broadcast(obj):
    if clients:
        msg = json.dumps(obj)
        await asyncio.gather(*(c.send(msg) for c in list(clients)), return_exceptions=True)


async def set_mode(mode, reason):
    rover.mode = mode
    await broadcast({"type": "mode", "mode": mode, "reason": reason})


async def handle_cmd(m):
    now = asyncio.get_event_loop().time()
    t = m.get("type")
    if t == "heartbeat":
        rover.last_heartbeat = now
    elif t == "set_mode" and not rover.estop:
        await set_mode(m.get("mode", "IDLE"), "pc_request")
    elif t == "drive":
        rover.last_heartbeat = now
        thr, turn = float(m.get("throttle", 0)), float(m.get("turn", 0))
        if rover.mode == "AUTO" and (abs(thr) > 0.05 or abs(turn) > 0.05):
            await set_mode("TELEOP-PC", "operator_override")
        rover.cmd_thr, rover.cmd_turn = thr, turn
    elif t == "estop":
        rover.estop = bool(m.get("active", True))
        if rover.estop:
            rover.mode = "E-STOP"
            rover.task = None          # 与固件一致：E-STOP 同时终止拍摄任务
            await broadcast({"type": "alert", "level": "ERROR", "code": "ESTOP_ACTIVE"})
        else:
            await set_mode("IDLE", "estop_cleared")
            await broadcast({"type": "alert", "level": "INFO", "code": "ESTOP_CLEARED"})
    elif t == "task" and not rover.estop:
        name = m.get("name", "")
        if name == "stop":
            rover.task = None
            await set_mode("IDLE", "task_stopped")
        elif name in ("capture_panorama", "capture_stereo"):
            rover.task = name
            rover.task_index = 0
            rover.task_total = 7 if name == "capture_panorama" else 2
            rover.task_timer = now + 1.0
            await set_mode("CAPTURE", "task_start")
            await broadcast({"type": "task_state", "name": name, "state": "RUNNING",
                             "index": 0, "total": rover.task_total})
    elif t == "sleep_now":
        await set_mode("SLEEP", "pc_request")
        await broadcast({"type": "alert", "level": "INFO", "code": "ENTER_SLEEP"})
    elif t == "pan":
        pass  # 云台仅记录，无可视化


async def task_engine():
    while True:
        await asyncio.sleep(0.2)
        now = asyncio.get_event_loop().time()
        if rover.task and now >= rover.task_timer:
            rover.task_index += 1
            done = rover.task_index >= rover.task_total
            await broadcast({"type": "task_state", "name": rover.task,
                             "state": "DONE" if done else "RUNNING",
                             "index": rover.task_index, "total": rover.task_total})
            if done:
                rover.task = None
                await set_mode("IDLE", "capture_done")
            else:
                rover.task_timer = now + 1.0


async def sim_loop():
    while True:
        rover.step(asyncio.get_event_loop().time())
        await asyncio.sleep(DT)


async def telemetry_loop():
    while True:
        await broadcast(rover.telemetry())
        await asyncio.sleep(0.2)


async def on_client(ws):
    clients.add(ws)
    print(f"客户端接入（共 {len(clients)}）")
    try:
        async for raw in ws:
            try:
                await handle_cmd(json.loads(raw))
            except (json.JSONDecodeError, ValueError, TypeError):
                pass
    finally:
        clients.discard(ws)
        print(f"客户端断开（剩 {len(clients)}）")


async def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=81)
    args = ap.parse_args()
    async with websockets.serve(on_client, "0.0.0.0", args.port):
        print(f"假火星车已启动 ws://127.0.0.1:{args.port} —— "
              "打开 dashboard/index.html，车辆地址填 127.0.0.1")
        await asyncio.gather(sim_loop(), telemetry_loop(), task_engine())


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n模拟器已停止")
