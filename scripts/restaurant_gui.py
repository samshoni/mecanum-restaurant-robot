#!/usr/bin/env python3
"""
Restaurant delivery-robot operator GUI.

Tkinter front-end + rclpy back-end in the same process:
  - rclpy executor runs on a daemon thread
  - Tk mainloop runs on the main thread
  - ROS callbacks push work onto the Tk event loop via `root.after(0, ...)`

Features: order queue, per-table dish notes, sequential delivery with
configurable dwell, live robot pose on a mini-map, cancel + emergency stop.
"""
import math
import os
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import tkinter as tk
from tkinter import ttk, messagebox

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose


# ──────────────────────────── data model ────────────────────────────

@dataclass
class TableDef:
    id: int
    zone: str
    center_x: float
    center_y: float
    pose_x: float
    pose_y: float
    pose_yaw: float


@dataclass
class Order:
    table_id: int
    dishes: str
    status: str = "pending"   # pending | in_progress | done | failed


@dataclass
class RestaurantConfig:
    home_name: str
    home_x: float
    home_y: float
    home_yaw: float
    tables: dict = field(default_factory=dict)   # id → TableDef


def load_config(path: str) -> RestaurantConfig:
    with open(path) as f:
        raw = yaml.safe_load(f)
    home = raw["home"]["pose"]
    cfg = RestaurantConfig(
        home_name=raw["home"]["name"],
        home_x=home["x"], home_y=home["y"], home_yaw=home["yaw"],
    )
    for t in raw["tables"]:
        tdef = TableDef(
            id=t["id"], zone=t["zone"],
            center_x=t["table_center"]["x"], center_y=t["table_center"]["y"],
            pose_x=t["pose"]["x"], pose_y=t["pose"]["y"],
            pose_yaw=t["pose"]["yaw"],
        )
        cfg.tables[tdef.id] = tdef
    return cfg


# ──────────────────────────── ROS node ────────────────────────────

class RobotBridge(Node):
    """Thin ROS wrapper: action client, pose subscriber, cmd_vel publisher."""

    def __init__(self):
        super().__init__("restaurant_gui")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # AMCL publishes /amcl_pose transient-local in Humble+; use compatible QoS.
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, pose_qos,
        )
        self.latest_pose: Optional[tuple] = None   # (x, y, yaw)
        self._pose_cb = None   # GUI installs a callback

    def set_pose_listener(self, cb):
        self._pose_cb = cb

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.latest_pose = (p.x, p.y, yaw)
        if self._pose_cb:
            self._pose_cb(p.x, p.y, yaw)

    # ───── navigation helpers ─────

    def make_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def publish_zero_cmd(self):
        self.cmd_vel_pub.publish(Twist())


# ──────────────────────────── GUI ────────────────────────────

# World → canvas transform. World x[-10..14] y[-8..12]. We draw with a margin.
WORLD_MIN_X, WORLD_MAX_X = -11.0, 15.0
WORLD_MIN_Y, WORLD_MAX_Y = -9.0, 13.0
CANVAS_W, CANVAS_H = 520, 440


def world_to_canvas(x, y):
    sx = (x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X)
    sy = (WORLD_MAX_Y - y) / (WORLD_MAX_Y - WORLD_MIN_Y)
    return sx * CANVAS_W, sy * CANVAS_H


class RestaurantGUI:
    STATE_IDLE = "IDLE"
    STATE_NAV = "NAVIGATING"
    STATE_WAIT = "WAITING"
    STATE_RETURN = "RETURNING"
    STATE_STOPPED = "EMERGENCY STOP"

    STATE_COLORS = {
        STATE_IDLE: "#2ecc71",
        STATE_NAV: "#3498db",
        STATE_WAIT: "#f39c12",
        STATE_RETURN: "#9b59b6",
        STATE_STOPPED: "#e74c3c",
    }

    def __init__(self, root: tk.Tk, bridge: RobotBridge, cfg: RestaurantConfig):
        self.root = root
        self.bridge = bridge
        self.cfg = cfg
        self.state = self.STATE_IDLE
        self.queue: list[Order] = []
        self.current_order: Optional[Order] = None
        self.dwell_seconds = tk.DoubleVar(value=2.0)

        # Action-client goal bookkeeping. These come back from ROS threads;
        # protect with a lock.
        self._lock = threading.Lock()
        self._goal_handle = None
        self._nav_worker: Optional[threading.Thread] = None
        self._cancel_requested = threading.Event()
        self._goal_done_event = threading.Event()
        self._goal_succeeded = False

        self._build_ui()
        self.bridge.set_pose_listener(self._on_pose_update)
        self._refresh_queue_view()
        self._update_status_banner()

    # ───── UI layout ─────

    def _build_ui(self):
        self.root.title("Restaurant Delivery Robot — Operator Console")
        self.root.geometry("1200x780")
        self.root.configure(bg="#1e1e2e")

        # Top status banner
        self.status_var = tk.StringVar(value="IDLE — Ready")
        self.status_banner = tk.Label(
            self.root, textvariable=self.status_var,
            font=("Helvetica", 20, "bold"),
            fg="white", bg=self.STATE_COLORS[self.STATE_IDLE],
            pady=12,
        )
        self.status_banner.pack(fill=tk.X)

        # Main split: left (map) | right (orders + controls)
        body = tk.Frame(self.root, bg="#1e1e2e")
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

        # ── LEFT: map canvas ─────────────────────────────
        left = tk.Frame(body, bg="#1e1e2e")
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        tk.Label(left, text="Restaurant Map", font=("Helvetica", 13, "bold"),
                 fg="white", bg="#1e1e2e").pack(anchor="w")
        self.canvas = tk.Canvas(left, width=CANVAS_W, height=CANVAS_H,
                                bg="#2a2a3e", highlightthickness=0)
        self.canvas.pack()
        self._draw_static_map()
        self.robot_id = None
        self._pose_for_redraw: Optional[tuple] = None

        legend = tk.Label(
            left,
            text=("● table idle   ● queued   ● active target   ▲ robot\n"
                  "Click a table dot to pre-fill the order form."),
            font=("Helvetica", 9), fg="#b0b0c0", bg="#1e1e2e",
            justify=tk.LEFT,
        )
        legend.pack(anchor="w", pady=(4, 0))

        # ── RIGHT: order entry + queue + controls ────────
        right = tk.Frame(body, bg="#1e1e2e")
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Order entry
        entry = tk.LabelFrame(right, text="  New Order  ",
                              font=("Helvetica", 11, "bold"),
                              fg="white", bg="#1e1e2e", bd=2, relief="groove")
        entry.pack(fill=tk.X, pady=(0, 8))

        row = tk.Frame(entry, bg="#1e1e2e")
        row.pack(fill=tk.X, padx=8, pady=6)
        tk.Label(row, text="Table:", fg="white", bg="#1e1e2e").pack(side=tk.LEFT)
        self.table_var = tk.StringVar()
        table_choices = [
            f"T{t.id}  ({t.zone})" for t in sorted(
                self.cfg.tables.values(), key=lambda t: t.id)
        ]
        self.table_dropdown = ttk.Combobox(
            row, textvariable=self.table_var, values=table_choices,
            state="readonly", width=22,
        )
        self.table_dropdown.current(0)
        self.table_dropdown.pack(side=tk.LEFT, padx=(6, 14))

        tk.Label(row, text="Dishes:", fg="white", bg="#1e1e2e").pack(side=tk.LEFT)
        self.dish_entry = tk.Entry(row, width=28)
        self.dish_entry.pack(side=tk.LEFT, padx=(6, 14), fill=tk.X, expand=True)
        self.dish_entry.bind("<Return>", lambda _e: self._add_order())

        tk.Button(row, text="Add to Queue", command=self._add_order,
                  bg="#3498db", fg="white", activebackground="#2980b9",
                  padx=12).pack(side=tk.LEFT)

        # Queue view
        qframe = tk.LabelFrame(right, text="  Delivery Queue  ",
                               font=("Helvetica", 11, "bold"),
                               fg="white", bg="#1e1e2e", bd=2, relief="groove")
        qframe.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

        cols = ("idx", "table", "zone", "dishes", "status")
        self.tree = ttk.Treeview(qframe, columns=cols, show="headings",
                                 height=10, selectmode="browse")
        for c, w, anchor in [
            ("idx", 40, "center"), ("table", 60, "center"),
            ("zone", 130, "w"), ("dishes", 340, "w"),
            ("status", 100, "center"),
        ]:
            self.tree.heading(c, text=c.upper())
            self.tree.column(c, width=w, anchor=anchor)
        self.tree.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        btnrow = tk.Frame(qframe, bg="#1e1e2e")
        btnrow.pack(fill=tk.X, padx=6, pady=(0, 6))
        tk.Button(btnrow, text="Remove Selected",
                  command=self._remove_selected_order,
                  bg="#7f8c8d", fg="white").pack(side=tk.LEFT)
        tk.Button(btnrow, text="Clear Queue",
                  command=self._clear_queue,
                  bg="#7f8c8d", fg="white").pack(side=tk.LEFT, padx=6)
        tk.Label(btnrow, text="Dwell (s):", fg="white", bg="#1e1e2e").pack(
            side=tk.LEFT, padx=(20, 4))
        tk.Spinbox(btnrow, from_=0.5, to=20.0, increment=0.5,
                   textvariable=self.dwell_seconds, width=6).pack(side=tk.LEFT)

        # Control buttons
        ctl = tk.Frame(right, bg="#1e1e2e")
        ctl.pack(fill=tk.X, pady=(0, 8))
        tk.Button(ctl, text="▶  START DELIVERY",
                  command=self._start_delivery,
                  bg="#27ae60", fg="white", font=("Helvetica", 11, "bold"),
                  padx=14, pady=6).pack(side=tk.LEFT)
        tk.Button(ctl, text="⏭  Cancel Current",
                  command=self._cancel_current,
                  bg="#e67e22", fg="white",
                  padx=10, pady=6).pack(side=tk.LEFT, padx=6)
        tk.Button(ctl, text="🏠  Return to Kitchen",
                  command=self._return_home,
                  bg="#8e44ad", fg="white",
                  padx=10, pady=6).pack(side=tk.LEFT, padx=6)
        tk.Button(ctl, text="■  EMERGENCY STOP",
                  command=self._emergency_stop,
                  bg="#c0392b", fg="white", font=("Helvetica", 11, "bold"),
                  padx=14, pady=6).pack(side=tk.RIGHT)

        # Activity log
        logf = tk.LabelFrame(right, text="  Activity Log  ",
                             font=("Helvetica", 11, "bold"),
                             fg="white", bg="#1e1e2e", bd=2, relief="groove")
        logf.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(logf, height=8, bg="#0f0f1a", fg="#a0f0a0",
                                font=("Monaco", 9), state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        self._log("GUI ready. Load orders, then press START DELIVERY.")

    # ───── map drawing ─────

    def _draw_static_map(self):
        c = self.canvas
        # zone rectangles (in world coords, then convert)
        zones = [
            ("Kitchen",        -10, 4, -2, 12, "#2d3a2d"),
            ("Private Dining",  -2, 4,  6, 12, "#3a2d3a"),
            ("Lounge",           6, 4, 14, 12, "#3a3a2d"),
            ("Main Dining",    -10, -8, 14, 4, "#2d2d3a"),
        ]
        for name, x0, y0, x1, y1, color in zones:
            cx0, cy0 = world_to_canvas(x0, y1)   # top-left of canvas
            cx1, cy1 = world_to_canvas(x1, y0)
            c.create_rectangle(cx0, cy0, cx1, cy1, fill=color, outline="#444",
                               width=1)
            c.create_text((cx0 + cx1) / 2, cy0 + 12, text=name,
                          fill="#888", font=("Helvetica", 8, "italic"))
        # doorways (gaps on y=4 wall)
        for (xl, xr), label in [((-6.75, -5.25), "K"),
                                 ((1.25, 2.75), "P"),
                                 ((9.25, 10.75), "L")]:
            dx0, dy = world_to_canvas(xl, 4)
            dx1, _ = world_to_canvas(xr, 4)
            c.create_line(dx0, dy, dx1, dy, fill="#1e1e2e", width=4)

        # tables (as small filled rectangles / circles + id label)
        self.table_items = {}
        for t in self.cfg.tables.values():
            cx, cy = world_to_canvas(t.center_x, t.center_y)
            r = 10
            dot = c.create_oval(cx - r, cy - r, cx + r, cy + r,
                                fill="#34495e", outline="#ecf0f1", width=2)
            label = c.create_text(cx, cy, text=str(t.id),
                                  fill="white", font=("Helvetica", 9, "bold"))
            # make table dots clickable to preselect in dropdown
            for item in (dot, label):
                c.tag_bind(item, "<Button-1>",
                           lambda _e, tid=t.id: self._preselect_table(tid))
            self.table_items[t.id] = dot

        # home marker
        hx, hy = world_to_canvas(self.cfg.home_x, self.cfg.home_y)
        c.create_rectangle(hx - 6, hy - 6, hx + 6, hy + 6,
                           fill="#16a085", outline="white")
        c.create_text(hx, hy - 14, text="HOME", fill="#16a085",
                      font=("Helvetica", 8, "bold"))

    def _preselect_table(self, tid: int):
        for i, v in enumerate(self.table_dropdown["values"]):
            if v.startswith(f"T{tid} "):
                self.table_dropdown.current(i)
                self.dish_entry.focus_set()
                break

    def _update_table_colors(self):
        """Recolor table dots to reflect queue state."""
        queued = {o.table_id for o in self.queue if o.status == "pending"}
        active = self.current_order.table_id if self.current_order else None
        for tid, item in self.table_items.items():
            if tid == active:
                fill = "#27ae60"  # green = active
            elif tid in queued:
                fill = "#f1c40f"  # yellow = queued
            else:
                fill = "#34495e"
            self.canvas.itemconfigure(item, fill=fill)

    def _on_pose_update(self, x, y, yaw):
        # called from ROS thread → defer to Tk thread
        self._pose_for_redraw = (x, y, yaw)
        self.root.after(0, self._draw_robot)

    def _draw_robot(self):
        if self._pose_for_redraw is None:
            return
        x, y, yaw = self._pose_for_redraw
        cx, cy = world_to_canvas(x, y)
        size = 9
        # triangle pointing in yaw direction
        pts = []
        for ang_off, dist in [(0, size), (2.4, size * 0.7), (-2.4, size * 0.7)]:
            a = yaw + ang_off
            px = cx + math.cos(-a) * dist  # canvas y is flipped
            py = cy + math.sin(-a) * dist
            pts.extend([px, py])
        if self.robot_id is None:
            self.robot_id = self.canvas.create_polygon(
                pts, fill="#3498db", outline="white", width=1.5)
        else:
            self.canvas.coords(self.robot_id, *pts)
            self.canvas.tag_raise(self.robot_id)

    # ───── logging / status ─────

    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{ts}] {msg}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _set_state(self, new_state: str, detail: str = ""):
        self.state = new_state
        msg = new_state if not detail else f"{new_state} — {detail}"
        self.status_var.set(msg)
        self.status_banner.configure(bg=self.STATE_COLORS.get(new_state, "#444"))
        self._update_table_colors()

    def _update_status_banner(self):
        if self.state == self.STATE_IDLE:
            pending = sum(1 for o in self.queue if o.status == "pending")
            self.status_var.set(
                f"IDLE — {pending} order(s) queued" if pending
                else "IDLE — Ready")
            self._update_table_colors()

    # ───── queue management ─────

    def _parse_table_id(self) -> Optional[int]:
        sel = self.table_var.get()
        if not sel or not sel.startswith("T"):
            return None
        try:
            return int(sel.split()[0][1:])
        except ValueError:
            return None

    def _add_order(self):
        tid = self._parse_table_id()
        if tid is None or tid not in self.cfg.tables:
            messagebox.showerror("Invalid table", "Pick a valid table.")
            return
        dishes = self.dish_entry.get().strip() or "(no notes)"
        self.queue.append(Order(table_id=tid, dishes=dishes))
        self.dish_entry.delete(0, tk.END)
        self._log(f"Added order: T{tid} — {dishes}")
        self._refresh_queue_view()
        if self.state == self.STATE_IDLE:
            self._update_status_banner()

    def _remove_selected_order(self):
        sel = self.tree.selection()
        if not sel:
            return
        idx = int(sel[0])
        if 0 <= idx < len(self.queue):
            o = self.queue[idx]
            if o.status == "in_progress":
                messagebox.showwarning(
                    "In progress",
                    "This order is running. Use Cancel Current instead.")
                return
            del self.queue[idx]
            self._refresh_queue_view()
            self._log(f"Removed queued order for T{o.table_id}")
            if self.state == self.STATE_IDLE:
                self._update_status_banner()

    def _clear_queue(self):
        before = len(self.queue)
        self.queue = [o for o in self.queue if o.status == "in_progress"]
        if before:
            self._log(f"Cleared {before - len(self.queue)} pending order(s)")
        self._refresh_queue_view()
        if self.state == self.STATE_IDLE:
            self._update_status_banner()

    def _refresh_queue_view(self):
        self.tree.delete(*self.tree.get_children())
        for i, o in enumerate(self.queue):
            t = self.cfg.tables[o.table_id]
            self.tree.insert("", tk.END, iid=str(i),
                             values=(i + 1, f"T{o.table_id}", t.zone,
                                     o.dishes, o.status))
        self._update_table_colors()

    # ───── delivery loop (runs on its own thread) ─────

    def _start_delivery(self):
        if self._nav_worker and self._nav_worker.is_alive():
            self._log("Delivery already running.")
            return
        if not any(o.status == "pending" for o in self.queue):
            messagebox.showinfo("Empty queue",
                                "Add at least one order first.")
            return
        self._cancel_requested.clear()
        self._nav_worker = threading.Thread(
            target=self._run_delivery_loop, daemon=True)
        self._nav_worker.start()

    def _run_delivery_loop(self):
        self._log("=== Delivery run started ===")
        for order in self.queue:
            if order.status != "pending":
                continue
            if self._cancel_requested.is_set():
                self._log("Delivery cancelled — abandoning remaining orders.")
                break

            self.current_order = order
            order.status = "in_progress"
            self.root.after(0, self._refresh_queue_view)

            t = self.cfg.tables[order.table_id]
            self.root.after(0, self._set_state, self.STATE_NAV,
                            f"→ Table {t.id} ({t.zone})")
            self._log(f"Navigating to T{t.id} [{t.zone}] — {order.dishes}")

            ok = self._drive_to(t.pose_x, t.pose_y, t.pose_yaw)
            if self._cancel_requested.is_set():
                order.status = "failed"
                self.current_order = None
                self.root.after(0, self._refresh_queue_view)
                break
            if not ok:
                order.status = "failed"
                self._log(f"✗ Nav to T{t.id} failed — skipping to next.")
                self.current_order = None
                self.root.after(0, self._refresh_queue_view)
                continue

            # dwell
            dwell = float(self.dwell_seconds.get())
            self.root.after(0, self._set_state, self.STATE_WAIT,
                            f"Table {t.id} ({dwell:.1f}s)")
            self._log(f"✓ Arrived at T{t.id}. Dwelling {dwell:.1f}s.")
            self._sleep_cancellable(dwell)

            order.status = "done"
            self.current_order = None
            self.root.after(0, self._refresh_queue_view)

        # return home
        if not self._cancel_requested.is_set():
            self.root.after(0, self._set_state, self.STATE_RETURN,
                            f"→ {self.cfg.home_name}")
            self._log(f"Returning to {self.cfg.home_name}.")
            if self._drive_to(self.cfg.home_x, self.cfg.home_y,
                              self.cfg.home_yaw):
                self._log("✓ Back home. Delivery run complete.")
            else:
                self._log("✗ Return-to-home failed.")

        self.current_order = None
        self.root.after(0, self._set_state, self.STATE_IDLE)
        self.root.after(0, self._update_status_banner)
        self._log("=== Delivery run ended ===")

    def _sleep_cancellable(self, seconds: float):
        # break sleep into ticks so cancel/emergency responds fast
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self._cancel_requested.is_set():
                return
            time.sleep(0.1)

    def _drive_to(self, x: float, y: float, yaw: float) -> bool:
        """Send a NavigateToPose goal and block until result/cancel. Returns success."""
        if not self.bridge.nav_client.wait_for_server(timeout_sec=5.0):
            self._log("Nav2 action server unavailable.")
            return False

        self._goal_done_event.clear()
        self._goal_succeeded = False

        goal = self.bridge.make_goal(x, y, yaw)
        send_future = self.bridge.nav_client.send_goal_async(goal)

        # wait for goal to be accepted
        start = time.monotonic()
        while not send_future.done():
            if time.monotonic() - start > 10.0:
                self._log("Goal send timed out.")
                return False
            if self._cancel_requested.is_set():
                return False
            time.sleep(0.05)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._log("Goal rejected by Nav2.")
            return False

        with self._lock:
            self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result_done)

        # wait for done or cancel
        while not self._goal_done_event.is_set():
            if self._cancel_requested.is_set():
                goal_handle.cancel_goal_async()
                # let result callback resolve
            time.sleep(0.05)

        with self._lock:
            self._goal_handle = None
        return self._goal_succeeded

    def _on_result_done(self, fut):
        try:
            status = fut.result().status
            # 4 = STATUS_SUCCEEDED in action_msgs/msg/GoalStatus
            self._goal_succeeded = (status == 4)
        except Exception as e:
            self._goal_succeeded = False
            self._log(f"Goal result error: {e}")
        self._goal_done_event.set()

    def _cancel_current(self):
        if self.state == self.STATE_IDLE:
            return
        self._log("Cancel current requested.")
        with self._lock:
            gh = self._goal_handle
        if gh is not None:
            gh.cancel_goal_async()

    def _return_home(self):
        if self._nav_worker and self._nav_worker.is_alive():
            messagebox.showinfo("Busy",
                                "Delivery in progress — press Cancel first.")
            return
        self._cancel_requested.clear()
        self._nav_worker = threading.Thread(
            target=self._return_home_thread, daemon=True)
        self._nav_worker.start()

    def _return_home_thread(self):
        self.root.after(0, self._set_state, self.STATE_RETURN,
                        f"→ {self.cfg.home_name}")
        self._log(f"Manual return to {self.cfg.home_name}.")
        ok = self._drive_to(self.cfg.home_x, self.cfg.home_y,
                            self.cfg.home_yaw)
        self._log("✓ Home." if ok else "✗ Return failed.")
        self.root.after(0, self._set_state, self.STATE_IDLE)
        self.root.after(0, self._update_status_banner)

    def _emergency_stop(self):
        self._log("!!! EMERGENCY STOP !!!")
        self._cancel_requested.set()
        with self._lock:
            gh = self._goal_handle
        if gh is not None:
            gh.cancel_goal_async()
        # burst zero-velocity so the base stops even if controller is stuck
        for _ in range(5):
            self.bridge.publish_zero_cmd()
            time.sleep(0.05)
        self._set_state(self.STATE_STOPPED, "zero cmd_vel published")


# ──────────────────────────── main ────────────────────────────

def find_tables_yaml() -> str:
    # try install share first (normal launch path), then source tree
    env = os.environ.get("MECANUM_BOT_TABLES")
    if env and os.path.isfile(env):
        return env
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory("mecanum_bot"), "config", "tables.yaml"))
    except Exception:
        pass
    candidates.append(os.path.expanduser(
        "~/mecanum_ws/src/mecanum_bot/config/tables.yaml"))
    for p in candidates:
        if os.path.isfile(p):
            return p
    raise FileNotFoundError("tables.yaml not found in share or source tree")


def main():
    rclpy.init()
    bridge = RobotBridge()

    executor_thread = threading.Thread(
        target=rclpy.spin, args=(bridge,), daemon=True)
    executor_thread.start()

    cfg = load_config(find_tables_yaml())

    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except tk.TclError:
        pass
    app = RestaurantGUI(root, bridge, cfg)

    def on_close():
        try:
            bridge.publish_zero_cmd()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
