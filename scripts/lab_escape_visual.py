#!/usr/bin/env python3
"""
視覺化 lab escape 狀態機（機器人 YZ 公分平面）。

執行（專案根目錄）:
  poetry run python scripts/lab_escape_visual.py
  poetry run python scripts/lab_escape_visual.py --profile smooth
  poetry run python scripts/lab_escape_visual.py --backend mpl

若系統 Python 沒有 Tcl/Tk（import tkinter 失敗），請用 --backend mpl 並安裝:
  poetry add --group dev matplotlib

操作:
  - 在工作區內移動/拖曳滑鼠 = 手的位置 (y_cm, z_cm)
  - 移出工作區外再放開 = 無手（測試 IDLE / 盲區）
  - R: 冰淇淋回到中心並重置狀態機（matplotlib 視窗需聚焦後按鍵）
  - C: 切換 enable_close_retreat 後重置狀態機
  - Q: 結束（matplotlib）

座標: 橫軸 = 機器人 +Y（右），縱軸 = 機器人 +Z（上）。
"""

from __future__ import annotations

import argparse
import sys

_parser = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter,
)
_parser.add_argument(
    "--profile",
    choices=("turkish_spec", "smooth", "performer", "safe"),
    default="turkish_spec",
    help="對應 state_machine 的演算法 profile",
)
_parser.add_argument(
    "--backend",
    choices=("auto", "tk", "mpl"),
    default="auto",
    help="auto: 優先 tkinter，失敗則用 matplotlib",
)
_args = _parser.parse_args()

import core.app.lab_escape.state_machine as _state_machine

_state_machine.ALGORITHM_PROFILE = _args.profile

from core.app.lab_escape.common import Vec2, WorldConfig
from core.app.lab_escape.state_machine import ArmStateMachine

try:
    import tkinter as tk
    from tkinter import font as tkfont

    _TKINTER_OK = True
except ModuleNotFoundError:
    tk = None  # type: ignore[assignment,misc]
    tkfont = None  # type: ignore[assignment,misc]
    _TKINTER_OK = False


def _hex(rgb: tuple[int, int, int]) -> str:
    return f"#{rgb[0]:02x}{rgb[1]:02x}{rgb[2]:02x}"


class LabEscapeVisualTk:
    def __init__(self, root: tk.Tk, world: WorldConfig) -> None:
        self.root = root
        self.world = world
        self.sm = ArmStateMachine(world)

        self.ice = Vec2(0.0, 0.0)
        self.hand: Vec2 | None = None
        self.mouse_in_workspace = False
        self.dt = 1.0 / 60.0

        root.title(
            f"Lab escape 視覺 — profile={_state_machine.ALGORITHM_PROFILE} (YZ cm)"
        )

        self.canvas_w = 720
        self.canvas_h = 720
        self.margin = 40
        self.cx = self.canvas_w / 2
        self.cy = self.canvas_h / 2
        span = max(
            world.y_max_cm - world.y_min_cm,
            world.z_max_cm - world.z_min_cm,
        )
        self.px_per_cm = (min(self.canvas_w, self.canvas_h) - 2 * self.margin) / span

        self.canvas = tk.Canvas(
            root,
            width=self.canvas_w,
            height=self.canvas_h + 100,
            bg="#2a2a2a",
            highlightthickness=0,
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.info_var = tk.StringVar(value="")
        assert tkfont is not None
        mono = tkfont.nametofont("TkFixedFont")
        lbl = tk.Label(
            root,
            textvariable=self.info_var,
            justify=tk.LEFT,
            font=mono,
            fg="#e0e0e0",
            bg="#1e1e1e",
            anchor="w",
            padx=8,
            pady=6,
        )
        lbl.pack(fill=tk.X)

        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<ButtonPress-1>", self._on_press)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        root.bind("<Key-r>", self._reset_ice)
        root.bind("<Key-R>", self._reset_ice)
        root.bind("<Key-c>", self._toggle_close)
        root.bind("<Key-C>", self._toggle_close)
        root.bind("<Key-q>", lambda e: root.destroy())
        root.bind("<Key-Q>", lambda e: root.destroy())

        self._legend_drawn = False

        self._tick()

    def _world_to_canvas(self, p: Vec2) -> tuple[float, float]:
        x = self.cx + p.x * self.px_per_cm
        y = self.cy - p.y * self.px_per_cm
        return x, y

    def _canvas_to_world(self, cx: float, cy: float) -> Vec2:
        y = (cx - self.cx) / self.px_per_cm
        z = (self.cy - cy) / self.px_per_cm
        return Vec2(y, z)

    def _in_workspace(self, p: Vec2) -> bool:
        return (
            self.world.y_min_cm <= p.x <= self.world.y_max_cm
            and self.world.z_min_cm <= p.y <= self.world.z_max_cm
        )

    def _on_motion(self, event: tk.Event) -> None:
        p = self._canvas_to_world(float(event.x), float(event.y))
        self.mouse_in_workspace = self._in_workspace(p)
        if self.mouse_in_workspace:
            self.hand = p

    def _on_press(self, event: tk.Event) -> None:
        p = self._canvas_to_world(float(event.x), float(event.y))
        if self._in_workspace(p):
            self.hand = p
            self.mouse_in_workspace = True

    def _on_release(self, event: tk.Event) -> None:
        p = self._canvas_to_world(float(event.x), float(event.y))
        if not self._in_workspace(p):
            self.hand = None
            self.mouse_in_workspace = False

    def _reset_ice(self, _event: tk.Event | None = None) -> None:
        self.ice = Vec2(0.0, 0.0)
        self.sm = ArmStateMachine(self.world)

    def _toggle_close(self, _event: tk.Event | None = None) -> None:
        self.world.enable_close_retreat = not self.world.enable_close_retreat
        self.sm = ArmStateMachine(self.world)

    def _draw_static(self) -> None:
        if self._legend_drawn:
            return
        self._legend_drawn = True

        p0 = Vec2(self.world.y_min_cm, self.world.z_min_cm)
        p1 = Vec2(self.world.y_max_cm, self.world.z_max_cm)
        x0, y0 = self._world_to_canvas(Vec2(p0.x, p1.y))
        x1, y1 = self._world_to_canvas(Vec2(p1.x, p0.y))
        self.canvas.create_rectangle(
            x0, y0, x1, y1,
            outline="#888",
            width=2,
            fill="#f8f8f8",
        )

        bz = self.world.blind_zone
        if bz is not None:
            a = Vec2(bz.x, bz.y)
            c = Vec2(bz.x + bz.w, bz.y + bz.h)
            ax, ay = self._world_to_canvas(Vec2(a.x, c.y))
            cx_, cy_ = self._world_to_canvas(Vec2(c.x, a.y))
            self.canvas.create_rectangle(
                ax, ay, cx_, cy_,
                outline="#a060c0",
                width=1,
                dash=(4, 4),
            )

        ox, oy = self._world_to_canvas(Vec2(0, 0))
        self.canvas.create_line(ox, 0, ox, self.canvas_h, fill="#ccc", width=1)
        self.canvas.create_line(0, oy, self.canvas_w, oy, fill="#ccc", width=1)
        self.canvas.create_text(ox + 8, oy + 8, text="(0,0)", anchor="nw", fill="#555")

        self.canvas.create_text(
            self.canvas_w - 8,
            12,
            text="+Y →",
            anchor="ne",
            fill="#666",
            font=("Helvetica", 10),
        )
        self.canvas.create_text(
            12,
            12,
            text="+Z ↑",
            anchor="nw",
            fill="#666",
            font=("Helvetica", 10),
        )

    def _tick(self) -> None:
        self._draw_static()

        hand_for_sm = self.hand if self.mouse_in_workspace else None
        result = self.sm.update(self.ice, hand_for_sm, self.dt)
        self.ice = result.pos

        self.canvas.delete("dynamic")

        r_ice = self.world.ice_radius_cm * self.px_per_cm
        ix, iy = self._world_to_canvas(self.ice)
        if result.visible:
            self.canvas.create_oval(
                ix - r_ice,
                iy - r_ice,
                ix + r_ice,
                iy + r_ice,
                outline=_hex(result.border_color),
                width=3,
                fill=_hex(result.color),
                tags="dynamic",
            )
        else:
            self.canvas.create_oval(
                ix - r_ice,
                iy - r_ice,
                ix + r_ice,
                iy + r_ice,
                outline=_hex(result.border_color),
                width=3,
                fill="#b0b0b0",
                stipple="gray50",
                tags="dynamic",
            )

        if self.hand is not None and self.mouse_in_workspace:
            r_hand = self.world.hand_radius_cm * self.px_per_cm
            hx, hy = self._world_to_canvas(self.hand)
            self.canvas.create_oval(
                hx - r_hand,
                hy - r_hand,
                hx + r_hand,
                hy + r_hand,
                outline="#e04040",
                width=2,
                fill="",
                tags="dynamic",
            )

        if result.planned_landing is not None:
            lx, ly = self._world_to_canvas(result.planned_landing)
            r = 8
            self.canvas.create_oval(
                lx - r,
                ly - r,
                lx + r,
                ly + r,
                outline="#2080e0",
                width=2,
                dash=(2, 2),
                tags="dynamic",
            )

        reason = self.sm.transition_reason.replace("\n", " ")
        if len(reason) > 100:
            reason = reason[:97] + "..."
        self.info_var.set(
            f"state={self.sm.state.name}  |  hand_speed={self.sm.hand_speed:.2f} cm/s  "
            f"|  close_retreat={self.world.enable_close_retreat}\n"
            f"reason: {reason}\n"
            f"algo: {result.status}\n"
            f"ice=({self.ice.x:.2f}, {self.ice.y:.2f}) cm"
            + (
                f"  hand=({self.hand.x:.2f}, {self.hand.y:.2f})"
                if self.hand and self.mouse_in_workspace
                else "  hand=(none)"
            )
        )

        self.root.after(int(self.dt * 1000), self._tick)


class LabEscapeVisualMpl:
    """matplotlib 後端（不依賴 Tcl/Tk）。"""

    def __init__(self, world: WorldConfig) -> None:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.patches import Circle, Rectangle

        self._plt = plt
        self._FuncAnimation = FuncAnimation
        self._Circle = Circle
        self._Rectangle = Rectangle

        self.world = world
        self.sm = ArmStateMachine(world)
        self.ice = Vec2(0.0, 0.0)
        self.hand: Vec2 | None = None
        self.mouse_in_workspace = False
        self.dt = 1.0 / 60.0

        self.fig, self.ax = plt.subplots(figsize=(7.5, 7.5))
        mgr = getattr(self.fig.canvas, "manager", None)
        if mgr is not None and hasattr(mgr, "set_window_title"):
            mgr.set_window_title(
                f"Lab escape — profile={_state_machine.ALGORITHM_PROFILE} (YZ cm)"
            )

        pad = 1.5
        self.ax.set_xlim(world.y_min_cm - pad, world.y_max_cm + pad)
        self.ax.set_ylim(world.z_min_cm - pad, world.z_max_cm + pad)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("y_cm (robot Y)")
        self.ax.set_ylabel("z_cm (robot Z)")
        self.ax.set_facecolor("#f0f0f0")
        self.ax.grid(True, alpha=0.35)

        ws = Rectangle(
            (world.y_min_cm, world.z_min_cm),
            world.y_max_cm - world.y_min_cm,
            world.z_max_cm - world.z_min_cm,
            linewidth=1.5,
            edgecolor="#555",
            facecolor="white",
            zorder=1,
        )
        self.ax.add_patch(ws)

        bz = world.blind_zone
        if bz is not None:
            blind = Rectangle(
                (bz.x, bz.y),
                bz.w,
                bz.h,
                linewidth=1.2,
                edgecolor="#a060c0",
                facecolor="none",
                linestyle="--",
                zorder=2,
            )
            self.ax.add_patch(blind)

        self.ax.axhline(0, color="#bbb", linewidth=0.8, zorder=0)
        self.ax.axvline(0, color="#bbb", linewidth=0.8, zorder=0)

        self._ice_patch = Circle(
            (self.ice.x, self.ice.y),
            world.ice_radius_cm,
            zorder=5,
        )
        self.ax.add_patch(self._ice_patch)

        self._hand_patch = Circle(
            (0.0, 0.0),
            world.hand_radius_cm,
            zorder=4,
            fill=False,
            linewidth=2,
            edgecolor="#e04040",
            visible=False,
        )
        self.ax.add_patch(self._hand_patch)

        self._planned_patch = Circle(
            (0.0, 0.0),
            0.35,
            zorder=3,
            fill=False,
            linewidth=1.5,
            edgecolor="#2080e0",
            linestyle="--",
            visible=False,
        )
        self.ax.add_patch(self._planned_patch)

        self._text = self.ax.text(
            0.02,
            0.98,
            "",
            transform=self.ax.transAxes,
            fontsize=9,
            verticalalignment="top",
            fontfamily="monospace",
            bbox=dict(boxstyle="round", facecolor="#1e1e1e", alpha=0.88),
            color="#e8e8e8",
        )

        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

        self._anim = FuncAnimation(
            self.fig,
            self._frame,
            interval=int(self.dt * 1000),
            blit=False,
            cache_frame_data=False,
        )

    def _in_workspace(self, p: Vec2) -> bool:
        return (
            self.world.y_min_cm <= p.x <= self.world.y_max_cm
            and self.world.z_min_cm <= p.y <= self.world.z_max_cm
        )

    def _on_motion(self, event) -> None:
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            self.mouse_in_workspace = False
            return
        p = Vec2(float(event.xdata), float(event.ydata))
        self.mouse_in_workspace = self._in_workspace(p)
        if self.mouse_in_workspace:
            self.hand = p

    def _on_release(self, event) -> None:
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            self.hand = None
            self.mouse_in_workspace = False
            return
        p = Vec2(float(event.xdata), float(event.ydata))
        if not self._in_workspace(p):
            self.hand = None
            self.mouse_in_workspace = False

    def _on_key(self, event) -> None:
        if event.key in ("r", "R"):
            self.ice = Vec2(0.0, 0.0)
            self.sm = ArmStateMachine(self.world)
        elif event.key in ("c", "C"):
            self.world.enable_close_retreat = not self.world.enable_close_retreat
            self.sm = ArmStateMachine(self.world)
        elif event.key in ("q", "Q"):
            self._plt.close(self.fig)

    def _frame(self, _i: int) -> None:
        hand_for_sm = self.hand if self.mouse_in_workspace else None
        result = self.sm.update(self.ice, hand_for_sm, self.dt)
        self.ice = result.pos

        self._ice_patch.center = (self.ice.x, self.ice.y)
        if result.visible:
            self._ice_patch.set_facecolor(
                tuple(c / 255.0 for c in result.color)
            )
            self._ice_patch.set_edgecolor(
                tuple(c / 255.0 for c in result.border_color)
            )
            self._ice_patch.set_alpha(1.0)
        else:
            self._ice_patch.set_facecolor((0.75, 0.75, 0.75))
            self._ice_patch.set_edgecolor(
                tuple(c / 255.0 for c in result.border_color)
            )
            self._ice_patch.set_alpha(0.55)

        if self.hand is not None and self.mouse_in_workspace:
            self._hand_patch.center = (self.hand.x, self.hand.y)
            self._hand_patch.set_visible(True)
        else:
            self._hand_patch.set_visible(False)

        if result.planned_landing is not None:
            self._planned_patch.center = (
                result.planned_landing.x,
                result.planned_landing.y,
            )
            self._planned_patch.set_visible(True)
        else:
            self._planned_patch.set_visible(False)

        reason = self.sm.transition_reason.replace("\n", " ")
        if len(reason) > 110:
            reason = reason[:107] + "..."
        self._text.set_text(
            f"state={self.sm.state.name}  hand_spd={self.sm.hand_speed:.2f} cm/s  "
            f"close_r={self.world.enable_close_retreat}\n"
            f"reason: {reason}\n"
            f"algo: {result.status}\n"
            f"ice=({self.ice.x:.2f},{self.ice.y:.2f})"
            + (
                f"  hand=({self.hand.x:.2f},{self.hand.y:.2f})"
                if self.hand and self.mouse_in_workspace
                else "  hand=none"
            )
        )

    def show(self) -> None:
        self._plt.show()


def main() -> int:
    world = WorldConfig()
    backend = _args.backend

    if backend == "tk":
        if not _TKINTER_OK:
            print(
                "錯誤: 此 Python 未編入 Tk（無 _tkinter）。請改用:\n"
                "  poetry run python scripts/lab_escape_visual.py --backend mpl\n"
                "並安裝: poetry add --group dev matplotlib",
                file=sys.stderr,
            )
            return 1
        assert tk is not None
        root = tk.Tk()
        LabEscapeVisualTk(root, world)
        root.mainloop()
        return 0

    if backend == "mpl":
        try:
            LabEscapeVisualMpl(world).show()
        except ImportError as e:
            print(
                "錯誤: 需要 matplotlib。\n"
                "  poetry add --group dev matplotlib\n"
                f"詳情: {e}",
                file=sys.stderr,
            )
            return 1
        return 0

    # auto
    if _TKINTER_OK:
        assert tk is not None
        root = tk.Tk()
        LabEscapeVisualTk(root, world)
        root.mainloop()
        return 0

    try:
        LabEscapeVisualMpl(world).show()
    except ImportError:
        print(
            "錯誤: 無 tkinter 且未安裝 matplotlib。\n"
            "  請安裝其一: 系統 Python 加 Tcl/Tk，或執行\n"
            "  poetry add --group dev matplotlib\n"
            "  poetry run python scripts/lab_escape_visual.py --backend mpl",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
