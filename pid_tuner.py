#!/usr/bin/env python3
"""
PID Tuner Dashboard for STM32 Line Follower Robot
==================================================
Real-time plotting of error, correction, setpoint, and motor speeds.
Send PID values and parameters over serial/Bluetooth.

Usage:  python pid_tuner.py
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import serial
import serial.tools.list_ports
import threading
import time
from collections import deque
import csv
from datetime import datetime

# ── Config ───────────────────────────────────────────────────────────────────
MAX_POINTS = 400
PLOT_INTERVAL_MS = 60
BAUD_OPTIONS = [9600, 19200, 38400, 57600, 115200]
PACKET_LOG_MAX = 200

JUNCTION_NAMES = {
    0: "NONE", 1: "LEFT", 2: "RIGHT", 3: "T-JUNC",
    4: "CROSS", 5: "DEAD END", 6: "FINISH",
}

# ── Dark palette ─────────────────────────────────────────────────────────────
C = {
    'bg': '#0f0f1a', 'card': '#1a1a2e', 'card2': '#16213e',
    'accent': '#0f3460', 'hl': '#e94560', 'text': '#e0e0e0',
    'dim': '#888899', 'green': '#00d4aa', 'blue': '#4da6ff',
    'orange': '#ff9f43', 'purple': '#a855f7', 'red': '#ff4757',
    'cyan': '#00f0ff', 'grid': '#2a2a3e', 'plotbg': '#0d0d1a',
    'setpoint': '#ffffff',
}


class PIDTuner:
    def __init__(self, root):
        self.root = root
        self.root.title("⚡ PID Tuner — Line Follower")
        self.root.geometry("1440x900")
        self.root.minsize(1100, 700)
        self.root.configure(bg=C['bg'])

        # Serial
        self.ser = None
        self.thread = None
        self.running = False

        # Data buffers
        self.t = deque(maxlen=MAX_POINTS)
        self.err = deque(maxlen=MAX_POINTS)
        self.corr = deque(maxlen=MAX_POINTS)
        self.sp = deque(maxlen=MAX_POINTS)
        self.lspd = deque(maxlen=MAX_POINTS)
        self.rspd = deque(maxlen=MAX_POINTS)
        self.batt = deque(maxlen=MAX_POINTS)

        # Dynamic sensor count — auto-detected from first IR packet
        self.n_sensors = 0
        self.ir_snapshot = []

        self.lock = threading.Lock()
        self.t0 = None
        self.new_data = False
        self.paused = False
        self.ir_flip = False
        self.junction = 0

        # Recording
        self.recording = False
        self.rec_data = []

        # Packet rate tracking
        self._pkt_count = 0
        self._pkt_time = time.time()
        self._pkt_rate = 0

        # Debug tab data
        self._packet_log = deque(maxlen=PACKET_LOG_MAX)
        self._latest = {
            'error': 0, 'correction': 0, 'left': 0, 'right': 0,
            'battery': 0, 'junction': 0, 'ir_raw': '',
        }

        self._build_ui()
        self._build_plots()
        self._build_debug_tab()
        self._tick()

    # ═════════════════════════════════════════════════════════════════════════
    #  UI
    # ═════════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        # ── Dark theme for Notebook tabs ─────────────────────────────────
        style = ttk.Style()
        style.theme_use('default')
        style.configure('Dark.TNotebook', background=C['bg'], borderwidth=0)
        style.configure('Dark.TNotebook.Tab',
                        background=C['card'], foreground=C['text'],
                        padding=[14, 6], font=('Segoe UI', 10, 'bold'))
        style.map('Dark.TNotebook.Tab',
                  background=[('selected', C['accent'])],
                  foreground=[('selected', C['hl'])])

        # ── Top bar ──────────────────────────────────────────────────────
        top = tk.Frame(self.root, bg=C['card'], pady=7, padx=10)
        top.pack(fill='x')

        tk.Label(top, text="⚡ PID Tuner", font=("Segoe UI", 15, "bold"),
                 fg=C['hl'], bg=C['card']).pack(side='left', padx=(0, 18))

        # Port
        tk.Label(top, text="Port:", fg=C['text'], bg=C['card'],
                 font=("Segoe UI", 10)).pack(side='left')
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var,
                                     width=11, state='readonly')
        self.port_cb.pack(side='left', padx=(2, 4))
        tk.Button(top, text="↻", command=self._refresh_ports, bg=C['accent'],
                  fg=C['text'], relief='flat', font=("Segoe UI", 10),
                  cursor='hand2').pack(side='left', padx=(0, 10))

        # Baud
        tk.Label(top, text="Baud:", fg=C['text'], bg=C['card'],
                 font=("Segoe UI", 10)).pack(side='left')
        self.baud_var = tk.StringVar(value="9600")
        ttk.Combobox(top, textvariable=self.baud_var, width=7,
                     values=[str(b) for b in BAUD_OPTIONS],
                     state='readonly').pack(side='left', padx=(2, 10))

        # Connect / Disconnect
        self.conn_btn = tk.Button(top, text="Connect", command=self._connect,
                                   bg=C['green'], fg='#000', relief='flat',
                                   font=("Segoe UI", 10, "bold"), padx=14,
                                   cursor='hand2')
        self.conn_btn.pack(side='left', padx=(0, 4))
        self.disc_btn = tk.Button(top, text="Disconnect", command=self._disconnect,
                                   bg=C['red'], fg='#fff', relief='flat',
                                   font=("Segoe UI", 10, "bold"), padx=14,
                                   cursor='hand2', state='disabled')
        self.disc_btn.pack(side='left', padx=(0, 14))

        self.conn_lbl = tk.Label(top, text="● Disconnected", fg=C['red'],
                                  bg=C['card'], font=("Segoe UI", 10))
        self.conn_lbl.pack(side='left')

        self.rate_lbl = tk.Label(top, text="", fg=C['dim'], bg=C['card'],
                                  font=("Segoe UI", 9))
        self.rate_lbl.pack(side='right')

        self.sensor_lbl = tk.Label(top, text="Sensors: —", fg=C['dim'],
                                    bg=C['card'], font=("Segoe UI", 9))
        self.sensor_lbl.pack(side='right', padx=(0, 14))

        self._refresh_ports()

        # ── Body ─────────────────────────────────────────────────────────
        body = tk.Frame(self.root, bg=C['bg'])
        body.pack(fill='both', expand=True, padx=5, pady=4)

        # Left panel
        left = tk.Frame(body, bg=C['bg'], width=260)
        left.pack(side='left', fill='y', padx=(0, 5))
        left.pack_propagate(False)

        self._card_pid(left)
        self._card_params(left)
        self._card_ctrl(left)
        self._card_status(left)
        self._card_data(left)

        # Right panel — Notebook with tabs
        self.notebook = ttk.Notebook(body, style='Dark.TNotebook')
        self.notebook.pack(side='left', fill='both', expand=True)

        # Tab 1: PID Plots
        self.plot_frame = tk.Frame(self.notebook, bg=C['plotbg'])
        self.notebook.add(self.plot_frame, text='  📊 PID Plots  ')

        # Tab 2: Debug Monitor
        self.debug_frame = tk.Frame(self.notebook, bg=C['bg'])
        self.notebook.add(self.debug_frame, text='  🔍 Debug Monitor  ')

    def _card(self, parent, title):
        f = tk.LabelFrame(parent, text=f"  {title}  ", fg=C['hl'],
                           bg=C['card'], font=("Segoe UI", 11, "bold"),
                           bd=1, relief='groove', padx=10, pady=7)
        f.pack(fill='x', pady=(0, 5))
        return f

    def _row(self, parent, label, default=""):
        r = tk.Frame(parent, bg=C['card'])
        r.pack(fill='x', pady=2)
        tk.Label(r, text=label, fg=C['text'], bg=C['card'],
                 font=("Segoe UI", 10), width=9, anchor='w').pack(side='left')
        v = tk.StringVar(value=default)
        tk.Entry(r, textvariable=v, width=10, bg=C['card2'], fg=C['text'],
                 insertbackground=C['text'], relief='flat',
                 font=("Consolas", 11), bd=2).pack(side='left', fill='x', expand=True)
        return v

    def _card_pid(self, p):
        c = self._card(p, "PID Parameters")
        self.kp = self._row(c, "Kp:", "1.0")
        self.ki = self._row(c, "Ki:", "0.5")
        self.kd = self._row(c, "Kd:", "1.0")
        tk.Button(c, text="Send PID", command=self._send_pid,
                  bg=C['blue'], fg='#fff', relief='flat',
                  font=("Segoe UI", 10, "bold"), cursor='hand2'
                  ).pack(fill='x', pady=(6, 0))

    def _card_params(self, p):
        c = self._card(p, "Motion Parameters")
        self.base_spd = self._row(c, "Base Spd:", "500")
        self.pid_lim = self._row(c, "PID Limit:", "300")
        tk.Button(c, text="Send Params", command=self._send_params,
                  bg=C['orange'], fg='#000', relief='flat',
                  font=("Segoe UI", 10, "bold"), cursor='hand2'
                  ).pack(fill='x', pady=(6, 0))

    def _card_ctrl(self, p):
        c = self._card(p, "Robot Control")
        r = tk.Frame(c, bg=C['card'])
        r.pack(fill='x')
        tk.Button(r, text="▶ START", command=self._send_start,
                  bg=C['green'], fg='#000', relief='flat',
                  font=("Segoe UI", 11, "bold"), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(0, 3))
        tk.Button(r, text="■ STOP", command=self._send_stop,
                  bg=C['red'], fg='#fff', relief='flat',
                  font=("Segoe UI", 11, "bold"), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True, padx=(3, 0))
        tk.Button(c, text="📖 PID Tuning Guide", command=self._show_guide,
                  bg=C['accent'], fg=C['text'], relief='flat',
                  font=("Segoe UI", 9), cursor='hand2'
                  ).pack(fill='x', pady=(6, 0))

    def _card_status(self, p):
        c = self._card(p, "Live Values")
        self.sv = {}
        items = [("Battery", "batt", C['green']),
                 ("Error", "err", C['cyan']),
                 ("Correction", "corr", C['orange']),
                 ("Left Spd", "left", C['blue']),
                 ("Right Spd", "right", C['purple'])]
        for label, key, color in items:
            r = tk.Frame(c, bg=C['card'])
            r.pack(fill='x', pady=1)
            tk.Label(r, text=f"{label}:", fg=C['dim'], bg=C['card'],
                     font=("Segoe UI", 9), width=10, anchor='w').pack(side='left')
            v = tk.StringVar(value="—")
            tk.Label(r, textvariable=v, fg=color, bg=C['card'],
                     font=("Consolas", 11, "bold")).pack(side='left')
            self.sv[key] = v

    def _card_data(self, p):
        c = self._card(p, "Data Tools")
        r1 = tk.Frame(c, bg=C['card'])
        r1.pack(fill='x', pady=(0, 4))
        self.rec_btn = tk.Button(r1, text="⏺ Record", command=self._toggle_rec,
                                  bg=C['purple'], fg='#fff', relief='flat',
                                  font=("Segoe UI", 10, "bold"), cursor='hand2')
        self.rec_btn.pack(side='left', fill='x', expand=True, padx=(0, 3))
        tk.Button(r1, text="💾 Export CSV", command=self._export,
                  bg=C['accent'], fg=C['text'], relief='flat',
                  font=("Segoe UI", 10), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True)

        r2 = tk.Frame(c, bg=C['card'])
        r2.pack(fill='x')
        self.pause_btn = tk.Button(r2, text="⏸ Pause Plot", command=self._toggle_pause,
                                    bg=C['accent'], fg=C['text'], relief='flat',
                                    font=("Segoe UI", 10), cursor='hand2')
        self.pause_btn.pack(side='left', fill='x', expand=True, padx=(0, 3))
        tk.Button(r2, text="🗑 Clear", command=self._clear_data,
                  bg=C['accent'], fg=C['text'], relief='flat',
                  font=("Segoe UI", 10), cursor='hand2'
                  ).pack(side='left', fill='x', expand=True)

        r3 = tk.Frame(c, bg=C['card'])
        r3.pack(fill='x', pady=(4, 0))
        self.flip_btn = tk.Button(r3, text="⇄ Flip IR Order", command=self._toggle_ir_flip,
                                   bg=C['accent'], fg=C['text'], relief='flat',
                                   font=("Segoe UI", 10), cursor='hand2')
        self.flip_btn.pack(fill='x')

    # ═════════════════════════════════════════════════════════════════════════
    #  PLOTS
    # ═════════════════════════════════════════════════════════════════════════
    def _build_plots(self):
        self.fig = Figure(figsize=(10, 7), facecolor=C['plotbg'])

        # GridSpec: top row full-width (error+correction), bottom row split
        gs = self.fig.add_gridspec(
            2, 2,
            height_ratios=[1.6, 1],
            hspace=0.45, wspace=0.38,
            left=0.08, right=0.97, top=0.95, bottom=0.08
        )

        # ── Plot 1 (top, full width): Error & Correction — dual Y-axis ─────
        self.ax1   = self.fig.add_subplot(gs[0, :])
        self.ax1.set_facecolor(C['plotbg'])
        self.ax1_r = self.ax1.twinx()          # right axis = correction

        self.line_err,  = self.ax1.plot(
            [], [], color=C['cyan'], lw=2.0, label='Line Error', clip_on=True)
        self.line_sp,   = self.ax1.plot(
            [], [], color=C['setpoint'], lw=1.0, ls='--', alpha=0.55,
            label='Setpoint (0)', clip_on=True)
        self.line_corr, = self.ax1_r.plot(
            [], [], color=C['orange'], lw=1.8, alpha=0.9, label='Correction', clip_on=True)

        self.ax1.set_ylabel('Error',      color=C['cyan'],   fontsize=10)
        self.ax1_r.set_ylabel('Correction', color=C['orange'], fontsize=10)
        self.ax1.set_title(
            'Line Error  &  PID Correction  (dual axis)',
            color=C['hl'], fontsize=11, fontweight='bold', pad=6)
        self.ax1.set_xlabel('Time (s)', color=C['dim'], fontsize=9)

        # merged legend for dual-axis plot
        lns  = [self.line_err, self.line_sp, self.line_corr]
        labs = [l.get_label() for l in lns]
        self.ax1.legend(lns, labs, loc='upper right', fontsize=8,
                        facecolor=C['card'], edgecolor=C['grid'],
                        labelcolor=C['text'])
        self._style_ax(self.ax1)
        self._style_ax(self.ax1_r, right=True)

        # ── Plot 2 (bottom-left): Motor speeds ───────────────────────────────
        self.ax2 = self.fig.add_subplot(gs[1, 0])
        self.line_l, = self.ax2.plot([], [], color=C['blue'],   lw=1.5, label='Left', clip_on=True)
        self.line_r, = self.ax2.plot([], [], color=C['purple'], lw=1.5, label='Right', clip_on=True)
        self.ax2.set_ylabel('Speed (PWM)', color=C['text'], fontsize=9)
        self.ax2.set_xlabel('Time (s)',    color=C['dim'],  fontsize=9)
        self.ax2.set_title('Motor Speeds', color=C['hl'],
                            fontsize=11, fontweight='bold', pad=6)
        self.ax2.legend(loc='upper right', fontsize=8, facecolor=C['card'],
                        edgecolor=C['grid'], labelcolor=C['text'])
        self._style_ax(self.ax2)

        # ── Plot 3 (bottom-right): IR sensor live bar chart ──────────────────
        # Bars are created dynamically when sensor count is auto-detected
        self.ax3 = self.fig.add_subplot(gs[1, 1])
        self.ax3.set_facecolor(C['plotbg'])
        self.ax3.set_ylim(0, 4095)
        self.ax3.set_title('IR Sensors (waiting...)', color=C['hl'],
                            fontsize=11, fontweight='bold', pad=6)
        self._style_ax(self.ax3)
        self.ir_bars = []

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

    def _rebuild_ir_bars(self, n):
        """Rebuild IR bar chart when sensor count changes."""
        self.ax3.cla()
        self.ax3.set_facecolor(C['plotbg'])
        self.ir_bars = self.ax3.bar(
            range(n), [0] * n,
            color=C['cyan'], alpha=0.80,
            edgecolor=C['grid'], linewidth=0.6
        )
        self.ax3.set_ylim(0, 4095)
        self.ax3.set_xlim(-0.6, n - 0.4)
        self.ax3.set_xticks(range(n))
        self.ax3.set_xticklabels([f'S{i+1}' for i in range(n)], fontsize=7)
        self.ax3.set_xlabel(
            f'S1=ADC-CH0 → S{n}=ADC-CH{n-1}  (⇄ Flip if reversed)',
            color=C['dim'], fontsize=7)
        self.ax3.set_title(f'IR Sensors ×{n} (live)', color=C['hl'],
                            fontsize=11, fontweight='bold', pad=6)
        self._style_ax(self.ax3)

    def _style_ax(self, ax, right=False):
        ax.set_facecolor(C['plotbg'])
        ax.tick_params(colors=C['dim'], labelsize=8)
        if right:
            # Twin axis — style only the right spine / ticks in orange
            ax.tick_params(axis='y', colors=C['orange'], labelsize=8)
            ax.spines['right'].set_color(C['orange'])
            ax.spines['left'].set_visible(False)
            ax.spines['top'].set_visible(False)
            ax.spines['bottom'].set_visible(False)
        else:
            ax.spines['bottom'].set_color(C['grid'])
            ax.spines['left'].set_color(C['grid'])
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.grid(True, color=C['grid'], alpha=0.4, ls='--', lw=0.5)

    # ═════════════════════════════════════════════════════════════════════════
    #  DEBUG MONITOR TAB
    # ═════════════════════════════════════════════════════════════════════════
    def _build_debug_tab(self):
        # Main scrollable area
        outer = tk.Frame(self.debug_frame, bg=C['bg'])
        outer.pack(fill='both', expand=True, padx=10, pady=8)

        # ── Section 1: DMA Buffer / Raw ADC ─────────────────────────────
        sec1 = tk.LabelFrame(outer, text="  DMA Buffer / Raw ADC Values  ",
                              fg=C['hl'], bg=C['card'],
                              font=("Segoe UI", 11, "bold"),
                              bd=1, relief='groove', padx=12, pady=8)
        sec1.pack(fill='x', pady=(0, 8))

        self.dma_canvas = tk.Canvas(sec1, bg=C['card'], height=120,
                                     highlightthickness=0)
        self.dma_canvas.pack(fill='x')
        self._dma_items = []  # (bar_id, text_id, label_id) per sensor

        # ── Section 2: Telemetry Values ─────────────────────────────────
        sec2 = tk.LabelFrame(outer, text="  Telemetry Values  ",
                              fg=C['hl'], bg=C['card'],
                              font=("Segoe UI", 11, "bold"),
                              bd=1, relief='groove', padx=12, pady=8)
        sec2.pack(fill='x', pady=(0, 8))

        tele_grid = tk.Frame(sec2, bg=C['card'])
        tele_grid.pack(fill='x')

        self.dbg_vars = {}
        tele_items = [
            ("Battery",    "battery",    C['green'],  "V"),
            ("Error",      "error",      C['cyan'],   ""),
            ("Correction", "correction", C['orange'], ""),
            ("Left Speed", "left",       C['blue'],   "PWM"),
            ("Right Speed","right",      C['purple'], "PWM"),
            ("Junction",   "junction",   C['hl'],     ""),
        ]
        for idx, (label, key, color, unit) in enumerate(tele_items):
            row = idx // 3
            col = idx % 3
            f = tk.Frame(tele_grid, bg=C['card2'], padx=10, pady=6)
            f.grid(row=row, column=col, padx=4, pady=3, sticky='nsew')
            tele_grid.columnconfigure(col, weight=1)

            tk.Label(f, text=label, fg=C['dim'], bg=C['card2'],
                     font=("Segoe UI", 9)).pack(anchor='w')
            v = tk.StringVar(value="—")
            tk.Label(f, textvariable=v, fg=color, bg=C['card2'],
                     font=("Consolas", 16, "bold")).pack(anchor='w')
            if unit:
                tk.Label(f, text=unit, fg=C['dim'], bg=C['card2'],
                         font=("Segoe UI", 8)).pack(anchor='w')
            self.dbg_vars[key] = v

        # ── Section 3: Sensor Detection Bar ─────────────────────────────
        sec3 = tk.LabelFrame(outer, text="  Sensor Detection (low ADC = line detected)  ",
                              fg=C['hl'], bg=C['card'],
                              font=("Segoe UI", 11, "bold"),
                              bd=1, relief='groove', padx=12, pady=8)
        sec3.pack(fill='x', pady=(0, 8))

        self.detect_canvas = tk.Canvas(sec3, bg=C['card'], height=50,
                                        highlightthickness=0)
        self.detect_canvas.pack(fill='x')
        self._detect_items = []

        # ── Section 4: Raw Command ──────────────────────────────────────
        sec4 = tk.LabelFrame(outer, text="  Send Raw Command  ",
                              fg=C['hl'], bg=C['card'],
                              font=("Segoe UI", 11, "bold"),
                              bd=1, relief='groove', padx=12, pady=8)
        sec4.pack(fill='x', pady=(0, 8))

        cmd_row = tk.Frame(sec4, bg=C['card'])
        cmd_row.pack(fill='x')
        self.raw_cmd = tk.StringVar()
        tk.Entry(cmd_row, textvariable=self.raw_cmd, bg=C['card2'],
                 fg=C['text'], insertbackground=C['text'], relief='flat',
                 font=("Consolas", 11), bd=2).pack(side='left', fill='x',
                                                     expand=True, padx=(0, 6))
        tk.Button(cmd_row, text="Send", command=self._send_raw,
                  bg=C['blue'], fg='#fff', relief='flat',
                  font=("Segoe UI", 10, "bold"), padx=16,
                  cursor='hand2').pack(side='left')

        # ── Section 5: Packet Log ───────────────────────────────────────
        sec5 = tk.LabelFrame(outer, text="  Packet Log  ",
                              fg=C['hl'], bg=C['card'],
                              font=("Segoe UI", 11, "bold"),
                              bd=1, relief='groove', padx=12, pady=8)
        sec5.pack(fill='both', expand=True)

        log_frame = tk.Frame(sec5, bg=C['card'])
        log_frame.pack(fill='both', expand=True)

        sb = tk.Scrollbar(log_frame)
        sb.pack(side='right', fill='y')

        self.pkt_log_text = tk.Text(log_frame, bg=C['plotbg'], fg=C['green'],
                                     font=("Consolas", 9), wrap='none',
                                     relief='flat', padx=8, pady=6, height=8,
                                     yscrollcommand=sb.set, state='disabled')
        self.pkt_log_text.pack(fill='both', expand=True)
        sb.config(command=self.pkt_log_text.yview)

        # Clear log button
        tk.Button(sec5, text="🗑 Clear Log", command=self._clear_log,
                  bg=C['accent'], fg=C['text'], relief='flat',
                  font=("Segoe UI", 9), cursor='hand2').pack(anchor='e',
                                                               pady=(4, 0))

    def _send_raw(self):
        cmd = self.raw_cmd.get().strip()
        if cmd:
            self._send(cmd)
            self.raw_cmd.set("")

    def _clear_log(self):
        with self.lock:
            self._packet_log.clear()
        self.pkt_log_text.config(state='normal')
        self.pkt_log_text.delete('1.0', 'end')
        self.pkt_log_text.config(state='disabled')

    def _update_debug(self):
        """Update debug monitor tab values — called from _tick on main thread."""
        with self.lock:
            latest = dict(self._latest)
            ir = list(reversed(self.ir_snapshot)) if self.ir_flip else list(self.ir_snapshot)
            n = self.n_sensors
            log_lines = list(self._packet_log)

        # Telemetry values
        self.dbg_vars['battery'].set(f"{latest['battery']:.2f}")
        self.dbg_vars['error'].set(f"{latest['error']:.1f}")
        self.dbg_vars['correction'].set(f"{latest['correction']:.1f}")
        self.dbg_vars['left'].set(str(latest['left']))
        self.dbg_vars['right'].set(str(latest['right']))
        jc = latest['junction']
        self.dbg_vars['junction'].set(
            f"{jc} — {JUNCTION_NAMES.get(jc, 'UNKNOWN')}")

        # DMA buffer bars (canvas)
        if n > 0 and ir:
            self._draw_dma_bars(ir, n)
            self._draw_detect_bar(ir, n)

        # Packet log (only update every few ticks to avoid lag)
        if log_lines:
            self.pkt_log_text.config(state='normal')
            self.pkt_log_text.delete('1.0', 'end')
            # Show last 50 lines
            for ln in log_lines[-50:]:
                self.pkt_log_text.insert('end', ln + '\n')
            self.pkt_log_text.see('end')
            self.pkt_log_text.config(state='disabled')

    def _draw_dma_bars(self, ir_vals, n):
        """Draw vertical bars on DMA canvas showing raw ADC values."""
        c = self.dma_canvas
        c.delete('all')
        w = c.winfo_width() or 600
        h = c.winfo_height() or 120
        if n == 0:
            return

        bar_w = max(20, (w - 20) // n - 6)
        gap = (w - bar_w * n) / (n + 1)

        for i, val in enumerate(ir_vals[:n]):
            x = gap + i * (bar_w + gap)
            bar_h = (val / 4095.0) * (h - 35)
            y_top = h - 15 - bar_h

            # Bar
            ratio = val / 4095.0
            color = C['hl'] if ratio < 0.45 else C['cyan']
            c.create_rectangle(x, y_top, x + bar_w, h - 15,
                               fill=color, outline=C['grid'], width=1)
            # Value text
            c.create_text(x + bar_w / 2, y_top - 8,
                          text=str(val), fill=C['text'],
                          font=("Consolas", 9, "bold"), anchor='s')
            # Channel label
            c.create_text(x + bar_w / 2, h - 5,
                          text=f"CH{i}", fill=C['dim'],
                          font=("Segoe UI", 8), anchor='s')

    def _draw_detect_bar(self, ir_vals, n):
        """Draw horizontal detection indicator showing which sensors see the line."""
        c = self.detect_canvas
        c.delete('all')
        w = c.winfo_width() or 600
        h = c.winfo_height() or 50
        if n == 0:
            return

        cell_w = max(30, (w - 20) // n)
        gap = (w - cell_w * n) / (n + 1)

        for i, val in enumerate(ir_vals[:n]):
            x = gap + i * (cell_w + gap)
            ratio = val / 4095.0
            detected = ratio < 0.45

            # Detection cell
            fill = C['hl'] if detected else C['card2']
            c.create_rectangle(x, 5, x + cell_w, h - 5,
                               fill=fill, outline=C['grid'], width=1)
            # Label
            txt_color = '#fff' if detected else C['dim']
            label = f"S{i+1}\n●" if detected else f"S{i+1}\n○"
            c.create_text(x + cell_w / 2, h / 2,
                          text=label, fill=txt_color,
                          font=("Segoe UI", 9, "bold"), anchor='center')

    # ═════════════════════════════════════════════════════════════════════════
    #  SERIAL
    # ═════════════════════════════════════════════════════════════════════════
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb['values'] = ports
        if ports:
            self.port_var.set(ports[0])

    def _connect(self):
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        if not port:
            messagebox.showwarning("No port", "Select a COM port first.")
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.running = True
            self.thread = threading.Thread(target=self._reader, daemon=True)
            self.thread.start()
            self.conn_lbl.config(text=f"● Connected ({port})", fg=C['green'])
            self.conn_btn.config(state='disabled')
            self.disc_btn.config(state='normal')
            self.t0 = time.time()
        except serial.SerialException as e:
            messagebox.showerror("Connection failed", str(e))

    def _disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.conn_lbl.config(text="● Disconnected", fg=C['red'])
        self.conn_btn.config(state='normal')
        self.disc_btn.config(state='disabled')

    def _reader(self):
        buf = ""
        while self.running and self.ser and self.ser.is_open:
            try:
                n = self.ser.in_waiting or 1
                raw = self.ser.read(n)
                if not raw:
                    time.sleep(0.005)
                    continue
                buf += raw.decode('utf-8', errors='replace')
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip('\r\n ')
                    if line:
                        self._parse(line)
            except Exception as e:
                print(f"[reader] error: {e}")
                break

    def _parse(self, line):
        """Parse: IR:v0,v1,...;PL:left;PR:right;BV:bat;PE:error;PO:correction;JC:junction"""
        vals = {}
        try:
            for part in line.split(';'):
                part = part.strip()
                if ':' not in part:
                    continue
                key, value = part.split(':', 1)
                vals[key.strip()] = value.strip()
        except Exception:
            return

        # Must have at least one known telemetry key
        if not any(k in vals for k in ('PE', 'BV', 'PL', 'PR', 'PO')):
            print(f"[parse] unrecognised packet: {line[:80]}")
            return

        now = time.time() - (self.t0 or time.time())
        try:
            error      = float(vals.get('PE', 0))
            correction = float(vals.get('PO', 0))
            left       = int(float(vals.get('PL', 0)))
            right      = int(float(vals.get('PR', 0)))
            battery    = float(vals.get('BV', 0))
            junction   = int(float(vals.get('JC', 0)))
        except (ValueError, TypeError) as e:
            print(f"[parse] conversion error: {e} in '{line[:80]}'")
            return

        # Parse IR values — auto-detect sensor count from data
        ir_str = vals.get('IR', '')
        ir_vals = []
        if ir_str:
            try:
                ir_vals = [int(x) for x in ir_str.split(',') if x.strip()]
            except Exception:
                ir_vals = []

        # Auto-detect / update sensor count
        new_n = len(ir_vals) if ir_vals else self.n_sensors
        need_rebuild = (new_n != self.n_sensors and new_n > 0)

        with self.lock:
            self.t.append(now)
            self.err.append(error)
            self.corr.append(correction)
            self.sp.append(0)
            self.lspd.append(left)
            self.rspd.append(right)
            self.batt.append(battery)
            self.junction = junction
            if ir_vals:
                self.ir_snapshot = ir_vals
                self.n_sensors = len(ir_vals)
            self.new_data = True

            # Debug tab data
            self._latest = {
                'error': error, 'correction': correction,
                'left': left, 'right': right,
                'battery': battery, 'junction': junction,
                'ir_raw': ir_str,
            }
            self._packet_log.append(
                f"[{now:7.2f}s] {line[:120]}"
            )

            # Recording
            if self.recording:
                self.rec_data.append({
                    'time': now, 'error': error, 'correction': correction,
                    'left': left, 'right': right, 'battery': battery,
                    'ir': ir_str, 'junction': junction,
                })

        # Rebuild IR bars on main thread if sensor count changed
        if need_rebuild:
            self.root.after(0, lambda n=new_n: self._rebuild_ir_bars(n))
            self.root.after(0, lambda n=new_n: self.sensor_lbl.config(
                text=f"Sensors: {n}", fg=C['green']))

        # ── Update live display safely from the main thread ──────────────────
        def _update_ui():
            self.sv['err'].set(f"{error:.1f}")
            self.sv['corr'].set(f"{correction:.1f}")
            self.sv['left'].set(str(left))
            self.sv['right'].set(str(right))
            self.sv['batt'].set(f"{battery:.1f} V")
        self.root.after(0, _update_ui)

        # Packet rate
        self._pkt_count += 1
        elapsed = time.time() - self._pkt_time
        if elapsed >= 1.0:
            self._pkt_rate = self._pkt_count / elapsed
            self._pkt_count = 0
            self._pkt_time = time.time()

    # ═════════════════════════════════════════════════════════════════════════
    #  SEND COMMANDS
    # ═════════════════════════════════════════════════════════════════════════
    def _send(self, msg):
        if self.ser and self.ser.is_open:
            self.ser.write((msg + '\r\n').encode())
        else:
            messagebox.showwarning("Not connected", "Connect to a port first.")

    def _send_pid(self):
        try:
            p, i, d = float(self.kp.get()), float(self.ki.get()), float(self.kd.get())
            self._send(f"PID:{p},{i},{d}")
        except ValueError:
            messagebox.showerror("Invalid", "Enter valid numbers for Kp, Ki, Kd.")

    def _send_params(self):
        try:
            bs = int(self.base_spd.get())
            pl = int(self.pid_lim.get())
            self._send(f"PARAM:BS{bs}PL{pl}")
        except ValueError:
            messagebox.showerror("Invalid", "Enter valid integers.")

    def _send_start(self):
        self._send("START")

    def _send_stop(self):
        self._send("STOP")

    def _show_guide(self):
        """Open a non-blocking PID tuning guide window."""
        win = tk.Toplevel(self.root)
        win.title("📖 PID Tuning Guide — Line Follower")
        win.geometry("680x720")
        win.configure(bg=C['bg'])
        win.resizable(True, True)

        # ── header ──────────────────────────────────────────────────────────
        tk.Label(win, text="PID Tuning Guide",
                 font=("Segoe UI", 16, "bold"), fg=C['hl'], bg=C['bg']
                 ).pack(pady=(16, 2))
        tk.Label(win,
                 text="Use the dual-axis chart: cyan = Error, orange = Correction",
                 font=("Segoe UI", 10), fg=C['dim'], bg=C['bg']
                 ).pack(pady=(0, 12))

        # ── scrollable text ──────────────────────────────────────────────────
        frame = tk.Frame(win, bg=C['bg'])
        frame.pack(fill='both', expand=True, padx=16, pady=(0, 16))

        sb = tk.Scrollbar(frame)
        sb.pack(side='right', fill='y')

        txt = tk.Text(frame, bg=C['card'], fg=C['text'],
                      font=("Consolas", 10), wrap='word',
                      relief='flat', padx=14, pady=10,
                      yscrollcommand=sb.set, state='normal')
        txt.pack(fill='both', expand=True)
        sb.config(command=txt.yview)

        # ── colour tags ─────────────────────────────────────────────────────
        txt.tag_config('h1',   foreground=C['hl'],     font=("Segoe UI", 12, "bold"))
        txt.tag_config('h2',   foreground=C['cyan'],   font=("Segoe UI", 10, "bold"))
        txt.tag_config('good', foreground=C['green'],  font=("Consolas", 10))
        txt.tag_config('bad',  foreground=C['red'],    font=("Consolas", 10))
        txt.tag_config('tip',  foreground=C['orange'], font=("Segoe UI", 10, "italic"))
        txt.tag_config('code', foreground=C['blue'],   font=("Consolas", 10))
        txt.tag_config('dim',  foreground=C['dim'],    font=("Segoe UI",  9))

        def h1(s):  txt.insert('end', s + '\n', 'h1')
        def h2(s):  txt.insert('end', s + '\n', 'h2')
        def ok(s):  txt.insert('end', s + '\n', 'good')
        def bad(s): txt.insert('end', s + '\n', 'bad')
        def tip(s): txt.insert('end', s + '\n', 'tip')
        def cod(s): txt.insert('end', s + '\n', 'code')
        def dim(s): txt.insert('end', s + '\n', 'dim')
        def nl():   txt.insert('end', '\n')

        # ── content ─────────────────────────────────────────────────────────
        h1("STEP 0 — Reset all gains before starting")
        cod("  Kp = 0.5   Ki = 0.0   Kd = 0.0")
        tip("  Always start from a known baseline. Click Send PID.")
        nl()

        h1("STEP 1 — Tune Kp first (Proportional)")
        txt.insert('end', "Goal: robot follows the line but may oscillate.\n")
        nl()
        h2("  What you see on the chart:")
        bad("  Kp too LOW  → Error barely decreases, robot drifts off line")
        bad("               Correction line stays small even with big Error")
        bad("  Kp too HIGH → Error oscillates (zigzag), Correction overshoots")
        ok("  Kp correct  → Error decays toward 0, small overshoot is OK")
        nl()
        h2("  How to adjust:")
        txt.insert('end', "  Raise Kp in steps of 0.5 until you see oscillation,\n")
        txt.insert('end', "  then back off ~20%. That is your starting Kp.\n")
        nl()

        h1("STEP 2 — Tune Kd (Derivative) to kill oscillation")
        txt.insert('end', "Goal: damp out the Kp oscillations without jerking.\n")
        nl()
        h2("  What you see on the chart:")
        bad("  Kd too LOW  → Error still oscillates (zigzag correction)")
        bad("  Kd too HIGH → Correction spikes sharply on every sensor change")
        bad("               (looks like narrow spikes on orange line)")
        ok("  Kd correct  → Error converges smoothly, Correction is clean")
        nl()
        h2("  How to adjust:")
        txt.insert('end', "  Raise Kd from 0 in steps of 0.2 until oscillation\n")
        txt.insert('end', "  is gone. Stop before correction spikes appear.\n")
        nl()

        h1("STEP 3 — Tune Ki (Integral) last, and carefully")
        txt.insert('end', "Goal: eliminate steady-state offset (robot slightly off centre).\n")
        nl()
        h2("  What you see on the chart:")
        bad("  Ki too LOW  → Error never quite reaches 0 (flat offset line)")
        bad("  Ki too HIGH → Error slowly grows until correction saturates")
        bad("               ('integral windup' — correction rail at ±limit)")
        ok("  Ki correct  → Error reaches 0 cleanly, correction stays bounded")
        nl()
        h2("  How to adjust:")
        txt.insert('end', "  For a line follower Ki is often 0.0 or very small (0.01-0.1).\n")
        txt.insert('end', "  Only add Ki if robot consistently misses centre.\n")
        nl()

        h1("STEP 4 — Tune Base Speed & PID Limit")
        h2("  Base Speed")
        txt.insert('end', "  Higher speed → tighter curves need higher Kp/Kd.\n")
        txt.insert('end', "  Start at 400-500, increase after PID is stable.\n")
        nl()
        h2("  PID Limit (correction cap)")
        txt.insert('end', "  Caps how much correction can be applied.\n")
        txt.insert('end', "  If motors hit 0 or max every turn → limit too high.\n")
        txt.insert('end', "  Rule of thumb: limit ≈ Base Speed × 0.6\n")
        nl()

        h1("QUICK DIAGNOSTIC — chart patterns")
        patterns = [
            ("Error zigzags fast, Correction tracks it",    "→ Kp too high",     'bad'),
            ("Error barely moves toward 0",                  "→ Kp too low",      'bad'),
            ("Correction has sharp narrow spikes",           "→ Kd too high",     'bad'),
            ("Error oscillates, Correction is smooth",       "→ Kd too low",      'bad'),
            ("Correction rail at ±limit permanently",        "→ windup, Ki↓ or Limit↑", 'bad'),
            ("Error decays smoothly to ~0",                  "→ well tuned ✓",    'good'),
            ("Left/Right speeds differ by Correction",       "→ PID working ✓",   'good'),
        ]
        for symptom, diagnosis, tag in patterns:
            txt.insert('end', f"  {symptom}\n", 'dim')
            txt.insert('end', f"      {diagnosis}\n", tag)
            nl()

        h1("SUGGESTED STARTING POINT")
        cod("  Kp=1.0   Ki=0.0   Kd=0.5   Base=450   Limit=270")
        tip("  Adjust from here using the steps above.")

        txt.config(state='disabled')
        tk.Button(win, text="Close", command=win.destroy,
                  bg=C['hl'], fg='#fff', relief='flat',
                  font=("Segoe UI", 10, "bold"), padx=20, cursor='hand2'
                  ).pack(pady=(0, 14))

    # ═════════════════════════════════════════════════════════════════════════
    #  DATA TOOLS
    # ═════════════════════════════════════════════════════════════════════════
    def _toggle_rec(self):
        self.recording = not self.recording
        if self.recording:
            self.rec_data = []
            self.rec_btn.config(text="⏹ Stop Rec", bg=C['red'])
        else:
            self.rec_btn.config(text="⏺ Record", bg=C['purple'])

    def _export(self):
        data = self.rec_data if self.rec_data else None
        if not data:
            # Export current buffer
            with self.lock:
                if not self.t:
                    messagebox.showinfo("No data", "No data to export.")
                    return
                data = []
                for i in range(len(self.t)):
                    data.append({
                        'time': self.t[i], 'error': self.err[i],
                        'correction': self.corr[i],
                        'left': self.lspd[i], 'right': self.rspd[i],
                        'battery': self.batt[i], 'ir': '',
                    })

        path = filedialog.asksaveasfilename(
            defaultextension='.csv',
            filetypes=[("CSV files", "*.csv")],
            initialfile=f"pid_data_{datetime.now():%Y%m%d_%H%M%S}.csv")
        if not path:
            return
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['time', 'error', 'correction',
                                               'left', 'right', 'battery', 'ir'])
            w.writeheader()
            w.writerows(data)
        messagebox.showinfo("Exported", f"Saved {len(data)} rows to:\n{path}")

    def _toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(
            text="▶ Resume" if self.paused else "⏸ Pause Plot",
            bg=C['green'] if self.paused else C['accent'])

    def _toggle_ir_flip(self):
        self.ir_flip = not self.ir_flip
        self.flip_btn.config(
            text="⇄ Flip IR  [ON]"  if self.ir_flip else "⇄ Flip IR Order",
            bg=C['orange']          if self.ir_flip else C['accent'],
            fg='#000'               if self.ir_flip else C['text'])

    def _clear_data(self):
        with self.lock:
            for d in (self.t, self.err, self.corr, self.sp,
                      self.lspd, self.rspd, self.batt):
                d.clear()
            self.new_data = True
        self.t0 = time.time()

    # ═════════════════════════════════════════════════════════════════════════
    #  PERIODIC UPDATE
    # ═════════════════════════════════════════════════════════════════════════
    def _tick(self):
        with self.lock:
            has_data = self.new_data
            if has_data and not self.paused:
                t  = list(self.t)
                er = list(self.err)
                co = list(self.corr)
                sp = list(self.sp)
                ls = list(self.lspd)
                rs = list(self.rspd)
                # Apply flip here so it's in one place
                ir = list(reversed(self.ir_snapshot)) if self.ir_flip else list(self.ir_snapshot)
                self.new_data = False

        if has_data and not self.paused:
            # ── Error + Correction (dual axis) ───────────────────────────────
            self.line_err.set_data(t, er)
            self.line_sp.set_data(t, sp)
            self.line_corr.set_data(t, co)

            # Compute Y-limits directly from data (relim+autoscale is broken on twinx)
            if er:
                e_lo, e_hi = min(min(er), 0), max(max(er), 0)
                pad = max((e_hi - e_lo) * 0.1, 4)  # 10% padding, min ±4
                self.ax1.set_ylim(e_lo - pad, e_hi + pad)
            if co:
                c_lo, c_hi = min(min(co), 0), max(max(co), 0)
                pad = max((c_hi - c_lo) * 0.1, 50)  # 10% padding, min ±50
                self.ax1_r.set_ylim(c_lo - pad, c_hi + pad)
            # Shared X-axis for both time-series charts
            if t:
                self.ax1.set_xlim(t[0], t[-1])
                self.ax2.set_xlim(t[0], t[-1])

            # ── Motor speeds ─────────────────────────────────────────────────
            self.line_l.set_data(t, ls)
            self.line_r.set_data(t, rs)
            if ls and rs:
                all_spd = ls + rs
                s_lo, s_hi = min(all_spd), max(all_spd)
                pad = max((s_hi - s_lo) * 0.1, 20)
                self.ax2.set_ylim(s_lo - pad, s_hi + pad)

            # ── IR sensor bars (dynamic count) ───────────────────────────
            if self.ir_bars and ir:
                for i, (bar, val) in enumerate(zip(self.ir_bars, ir)):
                    bar.set_height(val)
                    ratio = val / 4095.0
                    bar.set_color(C['hl'] if ratio < 0.45 else C['cyan'])

            self.canvas.draw_idle()

        # Update debug tab
        self._update_debug()

        # Update packet-rate label
        self.rate_lbl.config(text=f"{self._pkt_rate:.0f} pkt/s")

        self.root.after(PLOT_INTERVAL_MS, self._tick)

    def on_close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()


if __name__ == '__main__':
    print("Starting PID Tuner Dashboard...")
    root = tk.Tk()
    app = PIDTuner(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    # Force window to front on Windows
    root.lift()
    root.attributes('-topmost', True)
    root.after(100, lambda: root.attributes('-topmost', False))
    root.focus_force()
    print("Dashboard window opened. If you don't see it, check your taskbar.")
    root.mainloop()
