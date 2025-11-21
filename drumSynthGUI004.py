import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import random
import importlib

try:
    ttkb = importlib.import_module("ttkbootstrap")
    Style = ttkb.Style
except ModuleNotFoundError:
    Style = None

BAUDRATE = 115200


def find_teensy_port():
    """Try to auto-detect a Teensy serial port."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        dev = (p.device or "").lower()
        if "teensy" in desc or "usbmodem" in dev:
            return p.device
    return None


class DrumGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Teensy Prok-Style Kick Controller")
        self.style = None

        if Style is not None:
            self.style = Style(theme="darkly")
            self.style.configure("TFrame", padding=4)
            self.style.configure("TLabelFrame", padding=8)
            self.style.configure("TLabel", font=("Inter", 11))
            self.style.configure("TButton", font=("Inter", 11, "bold"))
            self.style.configure("Horizontal.TScale",
                                 troughcolor=self.style.colors.dark,
                                 background=self.style.colors.bg,
                                 sliderlength=18,
                                 thickness=6)
        else:
            self.root.configure(bg="#202020")

        self.ser = None

        # --------------- CONNECTION FRAME ---------------
        conn_frame = ttk.LabelFrame(root, text="Connection")
        conn_frame.pack(fill="x", padx=10, pady=10)

        self.port_var = tk.StringVar()
        auto = find_teensy_port()
        self.port_var.set(auto or "/dev/cu.usbmodemXXXX")

        ttk.Label(conn_frame, text="Port:").pack(side="left", padx=5)
        ttk.Entry(conn_frame, textvariable=self.port_var, width=28).pack(side="left", padx=5)
        ttk.Button(conn_frame, text="Connect", command=self.connect).pack(side="left", padx=5)
        ttk.Button(conn_frame, text="Disconnect", command=self.disconnect).pack(side="left", padx=5)

        self.status_var = tk.StringVar(value="Not connected")
        ttk.Label(conn_frame, textvariable=self.status_var).pack(side="left", padx=10)

        #-------------SPACE BAR TRIGGER---------
        self.root.bind_all("<space>", self.on_space)

        # --------------- MAIN PARAM FRAME ---------------
        main_frame = ttk.Frame(root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        for col in range(3):
            main_frame.columnconfigure(col, weight=1)

        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        left_frame.columnconfigure(1, weight=1)

        center_frame = ttk.Frame(main_frame)
        center_frame.grid(row=0, column=1, sticky="nsew", padx=5)
        center_frame.columnconfigure(1, weight=1)

        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=2, sticky="nsew", padx=(5, 0))
        right_frame.columnconfigure(1, weight=1)

        # ---------- SINE A PITCH (OSC 0) ----------
        sineA_frame = ttk.LabelFrame(left_frame, text="Sine A (Osc 0) Pitch Env")
        sineA_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=5)
        sineA_frame.columnconfigure(1, weight=1)

        self.o0b = tk.DoubleVar(value=0.10)
        self.o0a = tk.DoubleVar(value=0.10)
        self.o0d = tk.DoubleVar(value=0.10)
        self.o0m = tk.DoubleVar(value=0.10)
        self.o0x = tk.DoubleVar(value=0.10)

        ttk.Label(sineA_frame, text="Base").grid(row=0, column=0, sticky="w")
        tk.Scale(sineA_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o0b,
                 command=self.on_o0b).grid(row=0, column=1, sticky="ew")

        ttk.Label(sineA_frame, text="Attack").grid(row=1, column=0, sticky="w")
        tk.Scale(sineA_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o0a,
                 command=self.on_o0a).grid(row=1, column=1, sticky="ew")

        ttk.Label(sineA_frame, text="Decay").grid(row=2, column=0, sticky="w")
        tk.Scale(sineA_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o0d,
                 command=self.on_o0d).grid(row=2, column=1, sticky="ew")

        ttk.Label(sineA_frame, text="Amount").grid(row=3, column=0, sticky="w")
        tk.Scale(sineA_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o0m,
                 command=self.on_o0m).grid(row=3, column=1, sticky="ew")

        ttk.Label(sineA_frame, text="Extend").grid(row=4, column=0, sticky="w")
        tk.Scale(sineA_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o0x,
                 command=self.on_o0x).grid(row=4, column=1, sticky="ew")

        # ---------- SINE B PITCH (OSC 1) ----------
        sineB_frame = ttk.LabelFrame(left_frame, text="Sine B (Osc 1) Pitch Env")
        sineB_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=5)
        sineB_frame.columnconfigure(1, weight=1)

        self.o1b = tk.DoubleVar(value=0.12)
        self.o1a = tk.DoubleVar(value=0.14)
        self.o1d = tk.DoubleVar(value=0.16)
        self.o1m = tk.DoubleVar(value=0.15)
        self.o1x = tk.DoubleVar(value=0.20)

        ttk.Label(sineB_frame, text="Base").grid(row=0, column=0, sticky="w")
        tk.Scale(sineB_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o1b,
                 command=self.on_o1b).grid(row=0, column=1, sticky="ew")

        ttk.Label(sineB_frame, text="Attack").grid(row=1, column=0, sticky="w")
        tk.Scale(sineB_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o1a,
                 command=self.on_o1a).grid(row=1, column=1, sticky="ew")

        ttk.Label(sineB_frame, text="Decay").grid(row=2, column=0, sticky="w")
        tk.Scale(sineB_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o1d,
                 command=self.on_o1d).grid(row=2, column=1, sticky="ew")

        ttk.Label(sineB_frame, text="Amount").grid(row=3, column=0, sticky="w")
        tk.Scale(sineB_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o1m,
                 command=self.on_o1m).grid(row=3, column=1, sticky="ew")

        ttk.Label(sineB_frame, text="Extend").grid(row=4, column=0, sticky="w")
        tk.Scale(sineB_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o1x,
                 command=self.on_o1x).grid(row=4, column=1, sticky="ew")

        # ---------- OSC MIX FRAME ----------
        mix_frame = ttk.LabelFrame(left_frame, text="Oscillator Mix")
        mix_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=5)
        mix_frame.columnconfigure(1, weight=1)

        self.k1 = tk.DoubleVar(value=1.0)
        self.k2 = tk.DoubleVar(value=0.2)
        self.t1 = tk.DoubleVar(value=0.0)
        self.t2 = tk.DoubleVar(value=0.0)
        self.kr = tk.DoubleVar(value=0.5)

        ttk.Label(mix_frame, text="Sine A Level").grid(row=0, column=0, sticky="w")
        tk.Scale(mix_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.k1,
                 command=self.on_k1).grid(row=0, column=1, sticky="ew")

        ttk.Label(mix_frame, text="Sine B Level").grid(row=1, column=0, sticky="w")
        tk.Scale(mix_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.k2,
                 command=self.on_k2).grid(row=1, column=1, sticky="ew")

        ttk.Label(mix_frame, text="Tri A Level").grid(row=2, column=0, sticky="w")
        tk.Scale(mix_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.t1,
                 command=self.on_t1).grid(row=2, column=1, sticky="ew")

        ttk.Label(mix_frame, text="909 Core Level").grid(row=3, column=0, sticky="w")
        tk.Scale(mix_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.t2,
                 command=self.on_t2).grid(row=3, column=1, sticky="ew")

        ttk.Label(mix_frame, text="Sine B Ratio").grid(row=4, column=0, sticky="w")
        tk.Scale(mix_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.kr,
                 command=self.on_kr).grid(row=4, column=1, sticky="ew")

        # ---------- AMP A FRAME ----------
        amp_frame = ttk.LabelFrame(left_frame, text="AMP A (Kick Amp Env)")
        amp_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=5)
        amp_frame.columnconfigure(1, weight=1)

        self.aa = tk.DoubleVar(value=0.0)
        self.ad = tk.DoubleVar(value=0.4)
        self.ae = tk.DoubleVar(value=0.0)
        self.ax = tk.DoubleVar(value=0.5)

        ttk.Label(amp_frame, text="Attack").grid(row=0, column=0, sticky="w")
        tk.Scale(amp_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.aa,
                 command=self.on_aa).grid(row=0, column=1, sticky="ew")

        ttk.Label(amp_frame, text="Decay").grid(row=1, column=0, sticky="w")
        tk.Scale(amp_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.ad,
                 command=self.on_ad).grid(row=1, column=1, sticky="ew")

        ttk.Label(amp_frame, text="Extend Level").grid(row=2, column=0, sticky="w")
        tk.Scale(amp_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.ae,
                 command=self.on_ae).grid(row=2, column=1, sticky="ew")

        ttk.Label(amp_frame, text="Extend Factor").grid(row=3, column=0, sticky="w")
        tk.Scale(amp_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.ax,
                 command=self.on_ax).grid(row=3, column=1, sticky="ew")

        # ---------- TRANSIENT (TICK/CLAP) ----------
        transient_frame = ttk.LabelFrame(left_frame, text="Transient")
        transient_frame.grid(row=4, column=0, columnspan=2, sticky="ew", pady=5)
        transient_frame.columnconfigure(1, weight=1)

        self.tl = tk.DoubleVar(value=0.0)   # transient level
        self.tf = tk.DoubleVar(value=0.7)   # transient cutoff
        self.td = tk.DoubleVar(value=0.1)   # transient decay

        ttk.Label(transient_frame, text="Level").grid(row=0, column=0, sticky="w")
        tk.Scale(transient_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.tl,
                 command=self.on_tl).grid(row=0, column=1, sticky="ew")

        ttk.Label(transient_frame, text="Cutoff").grid(row=1, column=0, sticky="w")
        tk.Scale(transient_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.tf,
                 command=self.on_tf).grid(row=1, column=1, sticky="ew")

        ttk.Label(transient_frame, text="Decay").grid(row=2, column=0, sticky="w")
        tk.Scale(transient_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.td,
                 command=self.on_td).grid(row=2, column=1, sticky="ew")

        # ---------- TRI A PITCH (OSC 2) ----------
        triA_frame = ttk.LabelFrame(center_frame, text="Tri A (Osc 2) Pitch Env")
        triA_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=5)
        triA_frame.columnconfigure(1, weight=1)

        self.o2b = tk.DoubleVar(value=0.10)
        self.o2a = tk.DoubleVar(value=0.05)
        self.o2d = tk.DoubleVar(value=0.10)
        self.o2m = tk.DoubleVar(value=0.10)
        self.o2x = tk.DoubleVar(value=0.10)

        ttk.Label(triA_frame, text="Base").grid(row=0, column=0, sticky="w")
        tk.Scale(triA_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o2b,
                 command=self.on_o2b).grid(row=0, column=1, sticky="ew")

        ttk.Label(triA_frame, text="Attack").grid(row=1, column=0, sticky="w")
        tk.Scale(triA_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o2a,
                 command=self.on_o2a).grid(row=1, column=1, sticky="ew")

        ttk.Label(triA_frame, text="Decay").grid(row=2, column=0, sticky="w")
        tk.Scale(triA_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o2d,
                 command=self.on_o2d).grid(row=2, column=1, sticky="ew")

        ttk.Label(triA_frame, text="Amount").grid(row=3, column=0, sticky="w")
        tk.Scale(triA_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o2m,
                 command=self.on_o2m).grid(row=3, column=1, sticky="ew")

        ttk.Label(triA_frame, text="Extend").grid(row=4, column=0, sticky="w")
        tk.Scale(triA_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o2x,
                 command=self.on_o2x).grid(row=4, column=1, sticky="ew")

        # ---------- 909 CORE PITCH (OSC 3) ----------
        triB_frame = ttk.LabelFrame(center_frame, text="909 Core (Osc 3) Pitch Env")
        triB_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=5)
        triB_frame.columnconfigure(1, weight=1)

        self.o3b = tk.DoubleVar(value=0.60)
        self.o3a = tk.DoubleVar(value=0.00)
        self.o3d = tk.DoubleVar(value=0.30)
        self.o3m = tk.DoubleVar(value=0.50)
        self.o3x = tk.DoubleVar(value=0.30)

        ttk.Label(triB_frame, text="Base").grid(row=0, column=0, sticky="w")
        tk.Scale(triB_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o3b,
                 command=self.on_o3b).grid(row=0, column=1, sticky="ew")

        ttk.Label(triB_frame, text="Attack").grid(row=1, column=0, sticky="w")
        tk.Scale(triB_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o3a,
                 command=self.on_o3a).grid(row=1, column=1, sticky="ew")

        ttk.Label(triB_frame, text="Decay").grid(row=2, column=0, sticky="w")
        tk.Scale(triB_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o3d,
                 command=self.on_o3d).grid(row=2, column=1, sticky="ew")

        ttk.Label(triB_frame, text="Amount").grid(row=3, column=0, sticky="w")
        tk.Scale(triB_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o3m,
                 command=self.on_o3m).grid(row=3, column=1, sticky="ew")

        ttk.Label(triB_frame, text="Extend").grid(row=4, column=0, sticky="w")
        tk.Scale(triB_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.o3x,
                 command=self.on_o3x).grid(row=4, column=1, sticky="ew")

        # ---------- NOISE ENGINE ----------
        noise_frame = ttk.LabelFrame(center_frame, text="Noise Engine")
        noise_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=5)
        noise_frame.columnconfigure(1, weight=1)

        self.nl = tk.DoubleVar(value=0.1)
        self.nf = tk.DoubleVar(value=0.3)
        self.nq = tk.DoubleVar(value=0.3)
        self.na = tk.DoubleVar(value=0.0)
        self.nd = tk.DoubleVar(value=0.08)

        ttk.Label(noise_frame, text="Noise Level").grid(row=0, column=0, sticky="w")
        tk.Scale(noise_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.nl,
                 command=self.on_nl).grid(row=0, column=1, sticky="ew")

        ttk.Label(noise_frame, text="Cutoff").grid(row=1, column=0, sticky="w")
        tk.Scale(noise_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.nf,
                 command=self.on_nf).grid(row=1, column=1, sticky="ew")

        ttk.Label(noise_frame, text="Resonance").grid(row=2, column=0, sticky="w")
        tk.Scale(noise_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.nq,
                 command=self.on_nq).grid(row=2, column=1, sticky="ew")

        ttk.Label(noise_frame, text="Attack").grid(row=3, column=0, sticky="w")
        tk.Scale(noise_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.na,
                 command=self.on_na).grid(row=3, column=1, sticky="ew")

        ttk.Label(noise_frame, text="Decay").grid(row=4, column=0, sticky="w")
        tk.Scale(noise_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.nd,
                 command=self.on_nd).grid(row=4, column=1, sticky="ew")

        # ---------- DISTORTION ----------
        dist_frame = ttk.LabelFrame(right_frame, text="Distortion")
        dist_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=5)
        dist_frame.columnconfigure(1, weight=1)

        self.dr = tk.DoubleVar(value=0.0)
        ttk.Label(dist_frame, text="Drive Amount").grid(row=0, column=0, sticky="w")
        tk.Scale(dist_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.dr,
                 command=self.on_dr).grid(row=0, column=1, sticky="ew")

        # ---------- OSC Tone Shaper ---
        self.oc = tk.DoubleVar(value=0.5)   # cutoff
        self.oq = tk.DoubleVar(value=0.3)   # resonance
        self.od = tk.DoubleVar(value=0.2)   # distortion
        self.fa = tk.DoubleVar(value=0.0)   # filter env attack
        self.fd = tk.DoubleVar(value=0.4)   # filter env decay
        self.fm = tk.DoubleVar(value=0.5)   # filter env amount
        self.mv = tk.DoubleVar(value=0.12)  # master volume

        oscTone_frame = ttk.LabelFrame(right_frame, text="OSC Tone (Post-Osc Mix)")
        oscTone_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=5)
        oscTone_frame.columnconfigure(1, weight=1)

        ttk.Label(oscTone_frame, text="OSC Cutoff").grid(row=0, column=0, sticky="w")
        tk.Scale(oscTone_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.oc,
                 command=self.on_oc).grid(row=0, column=1, sticky="ew")

        ttk.Label(oscTone_frame, text="OSC Q").grid(row=1, column=0, sticky="w")
        tk.Scale(oscTone_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.oq,
                 command=self.on_oq).grid(row=1, column=1, sticky="ew")

        ttk.Label(oscTone_frame, text="OSC Distort").grid(row=2, column=0, sticky="w")
        tk.Scale(oscTone_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.od,
                 command=self.on_od).grid(row=2, column=1, sticky="ew")

        filtEnv_frame = ttk.LabelFrame(right_frame, text="Filter Envelope")
        filtEnv_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=5)
        filtEnv_frame.columnconfigure(1, weight=1)

        ttk.Label(filtEnv_frame, text="Attack").grid(row=0, column=0, sticky="w")
        tk.Scale(filtEnv_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.fa,
                 command=self.on_fa).grid(row=0, column=1, sticky="ew")

        ttk.Label(filtEnv_frame, text="Decay").grid(row=1, column=0, sticky="w")
        tk.Scale(filtEnv_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.fd,
                 command=self.on_fd).grid(row=1, column=1, sticky="ew")

        ttk.Label(filtEnv_frame, text="Amount").grid(row=2, column=0, sticky="w")
        tk.Scale(filtEnv_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.fm,
                 command=self.on_fm).grid(row=2, column=1, sticky="ew")

        # ---------- RANDOMIZATION ----------
        rand_frame = ttk.LabelFrame(right_frame, text="Randomization")
        rand_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=5)
        rand_frame.columnconfigure(1, weight=1)

        self.randAmt = tk.DoubleVar(value=0.3)
        ttk.Label(rand_frame, text="Random Amount").grid(row=0, column=0, sticky="w")
        tk.Scale(rand_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.randAmt
                 ).grid(row=0, column=1, sticky="ew")

        ttk.Button(rand_frame, text="Randomize Now",
                   command=self.randomize_params).grid(row=1, column=0, columnspan=2, pady=5)

        volume_frame = ttk.LabelFrame(right_frame, text="Output")
        volume_frame.grid(row=4, column=0, columnspan=2, sticky="ew", pady=5)
        volume_frame.columnconfigure(1, weight=1)

        ttk.Label(volume_frame, text="Main Volume").grid(row=0, column=0, sticky="w")
        tk.Scale(volume_frame, from_=0, to=1, resolution=0.01,
                 orient="horizontal", variable=self.mv,
                 command=self.on_mv).grid(row=0, column=1, sticky="ew")

        # ---------- SEND ALL BUTTON ----------
        btn_frame = ttk.Frame(root)
        btn_frame.pack(fill="x", padx=10, pady=(0, 10))
        ttk.Button(btn_frame, text="Send All Params", command=self.send_all_params).pack(side="left")

    # --------------- SERIAL ---------------

    def connect(self):
        port = self.port_var.get().strip()
        try:
            self.ser = serial.Serial(port, BAUDRATE, timeout=1)
            self.status_var.set(f"Connected to {port}")
            self.send_all_params()
        except Exception as e:
            self.status_var.set(f"Error: {e}")
            self.ser = None

    def disconnect(self):
        """Disconnect from serial port to allow flashing Teensy"""
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
                self.status_var.set("Disconnected - ready to flash")
            except Exception as e:
                self.status_var.set(f"Disconnect error: {e}")
        else:
            self.status_var.set("Not connected")
        self.ser = None

    def send_cmd(self, cmd, val):
        if self.ser is None or not self.ser.is_open:
            return
        msg = f"{cmd} {val:.3f}\n"
        try:
            self.ser.write(msg.encode("ascii"))
        except Exception as e:
            self.status_var.set(f"Write error: {e}")

    def send_all_params(self):
        # Osc mix
        self.send_cmd("K1", self.k1.get())
        self.send_cmd("K2", self.k2.get())
        self.send_cmd("T1", self.t1.get())
        self.send_cmd("T2", self.t2.get())
        self.send_cmd("KR", self.kr.get())

        # Amp A
        self.send_cmd("AA", self.aa.get())
        self.send_cmd("AD", self.ad.get())
        self.send_cmd("AE", self.ae.get())
        self.send_cmd("AX", self.ax.get())

        # Osc 0 (Sine A)
        self.send_cmd("0B", self.o0b.get())
        self.send_cmd("0A", self.o0a.get())
        self.send_cmd("0D", self.o0d.get())
        self.send_cmd("0M", self.o0m.get())
        self.send_cmd("0X", self.o0x.get())

        # Osc 1 (Sine B)
        self.send_cmd("1B", self.o1b.get())
        self.send_cmd("1A", self.o1a.get())
        self.send_cmd("1D", self.o1d.get())
        self.send_cmd("1M", self.o1m.get())
        self.send_cmd("1X", self.o1x.get())

        # Osc 2 (Tri A)
        self.send_cmd("2B", self.o2b.get())
        self.send_cmd("2A", self.o2a.get())
        self.send_cmd("2D", self.o2d.get())
        self.send_cmd("2M", self.o2m.get())
        self.send_cmd("2X", self.o2x.get())

        # Osc 3 (Tri B)
        self.send_cmd("3B", self.o3b.get())
        self.send_cmd("3A", self.o3a.get())
        self.send_cmd("3D", self.o3d.get())
        self.send_cmd("3M", self.o3m.get())
        self.send_cmd("3X", self.o3x.get())

        # Noise
        self.send_cmd("NL", self.nl.get())
        self.send_cmd("NF", self.nf.get())
        self.send_cmd("NQ", self.nq.get())
        self.send_cmd("NA", self.na.get())
        self.send_cmd("ND", self.nd.get())

        # Transient
        self.send_cmd("TL", self.tl.get())
        self.send_cmd("TF", self.tf.get())
        self.send_cmd("TD", self.td.get())

        # Distortion
        self.send_cmd("DR", self.dr.get())

        # OSC Tone Shaper
        self.send_cmd("OC", self.oc.get())
        self.send_cmd("OQ", self.oq.get())
        self.send_cmd("OD", self.od.get())
        self.send_cmd("FA", self.fa.get())
        self.send_cmd("FD", self.fd.get())
        self.send_cmd("FM", self.fm.get())
        self.send_cmd("MV", self.mv.get())

    # --------------- RANDOMIZATION (controlled) ---------------

    def randomize_params(self):
        depth = self.randAmt.get()
        if depth <= 0.0:
            return

        # Helper to blend old->random based on depth
        def rblend(v):
            r = random.random()
            return depth * r + (1.0 - depth) * v

        # List of (var, command) pairs
        params = [
            # Osc mix
            (self.k1, "K1"), (self.k2, "K2"),
            (self.t1, "T1"), (self.t2, "T2"),
            (self.kr, "KR"),

            # Amp A
            (self.aa, "AA"), (self.ad, "AD"),
            (self.ae, "AE"), (self.ax, "AX"),

            # Osc 0
            (self.o0b, "0B"), (self.o0a, "0A"),
            (self.o0d, "0D"), (self.o0m, "0M"),
            (self.o0x, "0X"),

            # Osc 1
            (self.o1b, "1B"), (self.o1a, "1A"),
            (self.o1d, "1D"), (self.o1m, "1M"),
            (self.o1x, "1X"),

            # Osc 2
            (self.o2b, "2B"), (self.o2a, "2A"),
            (self.o2d, "2D"), (self.o2m, "2M"),
            (self.o2x, "2X"),

            # Osc 3
            (self.o3b, "3B"), (self.o3a, "3A"),
            (self.o3d, "3D"), (self.o3m, "3M"),
            (self.o3x, "3X"),

            # Noise
            (self.nl, "NL"), (self.nf, "NF"),
            (self.nq, "NQ"), (self.na, "NA"),
            (self.nd, "ND"),

            # Transient
            (self.tl, "TL"), (self.tf, "TF"), (self.td, "TD"),

            # Drive
            (self.dr, "DR"),

            # OSC Tone Shaper
            (self.oc, "OC"), (self.oq, "OQ"), (self.od, "OD"),

            # Filter Envelope
            (self.fa, "FA"), (self.fd, "FD"), (self.fm, "FM"),

            # Master volume (optional for randomization?)
        ]

        for var, cmd in params:
            old = var.get()
            new = rblend(old)
            # Keep within [0,1]
            if new < 0.0:
                new = 0.0
            if new > 1.0:
                new = 1.0
            var.set(new)
            self.send_cmd(cmd, new)

    # --------------- CALLBACKS ---------------

    # Osc mix
    def on_k1(self, _): self.send_cmd("K1", self.k1.get())
    def on_k2(self, _): self.send_cmd("K2", self.k2.get())
    def on_t1(self, _): self.send_cmd("T1", self.t1.get())
    def on_t2(self, _): self.send_cmd("T2", self.t2.get())
    def on_kr(self, _): self.send_cmd("KR", self.kr.get())

    # Amp A
    def on_aa(self, _): self.send_cmd("AA", self.aa.get())
    def on_ad(self, _): self.send_cmd("AD", self.ad.get())
    def on_ae(self, _): self.send_cmd("AE", self.ae.get())
    def on_ax(self, _): self.send_cmd("AX", self.ax.get())

    # Osc 0
    def on_o0b(self, _): self.send_cmd("0B", self.o0b.get())
    def on_o0a(self, _): self.send_cmd("0A", self.o0a.get())
    def on_o0d(self, _): self.send_cmd("0D", self.o0d.get())
    def on_o0m(self, _): self.send_cmd("0M", self.o0m.get())
    def on_o0x(self, _): self.send_cmd("0X", self.o0x.get())

    # Osc 1
    def on_o1b(self, _): self.send_cmd("1B", self.o1b.get())
    def on_o1a(self, _): self.send_cmd("1A", self.o1a.get())
    def on_o1d(self, _): self.send_cmd("1D", self.o1d.get())
    def on_o1m(self, _): self.send_cmd("1M", self.o1m.get())
    def on_o1x(self, _): self.send_cmd("1X", self.o1x.get())

    # Osc 2
    def on_o2b(self, _): self.send_cmd("2B", self.o2b.get())
    def on_o2a(self, _): self.send_cmd("2A", self.o2a.get())
    def on_o2d(self, _): self.send_cmd("2D", self.o2d.get())
    def on_o2m(self, _): self.send_cmd("2M", self.o2m.get())
    def on_o2x(self, _): self.send_cmd("2X", self.o2x.get())

    # Osc 3
    def on_o3b(self, _): self.send_cmd("3B", self.o3b.get())
    def on_o3a(self, _): self.send_cmd("3A", self.o3a.get())
    def on_o3d(self, _): self.send_cmd("3D", self.o3d.get())
    def on_o3m(self, _): self.send_cmd("3M", self.o3m.get())
    def on_o3x(self, _): self.send_cmd("3X", self.o3x.get())

    # Noise
    def on_nl(self, _): self.send_cmd("NL", self.nl.get())
    def on_nf(self, _): self.send_cmd("NF", self.nf.get())
    def on_nq(self, _): self.send_cmd("NQ", self.nq.get())
    def on_na(self, _): self.send_cmd("NA", self.na.get())
    def on_nd(self, _): self.send_cmd("ND", self.nd.get())

    # Transient
    def on_tl(self, _): self.send_cmd("TL", self.tl.get())
    def on_tf(self, _): self.send_cmd("TF", self.tf.get())
    def on_td(self, _): self.send_cmd("TD", self.td.get())

    # Distortion
    def on_dr(self, _): self.send_cmd("DR", self.dr.get())

    # OSC Tone Shaper callbacks
    def on_oc(self, _): self.send_cmd("OC", self.oc.get())
    def on_oq(self, _): self.send_cmd("OQ", self.oq.get())
    def on_od(self, _): self.send_cmd("OD", self.od.get())

    # Filter envelope
    def on_fa(self, _): self.send_cmd("FA", self.fa.get())
    def on_fd(self, _): self.send_cmd("FD", self.fd.get())
    def on_fm(self, _): self.send_cmd("FM", self.fm.get())

    # Master volume
    def on_mv(self, _): self.send_cmd("MV", self.mv.get())

    # fire the kick via serial (space bar)
    def on_space(self, event): self.send_cmd("TK", 1.0)


if __name__ == "__main__":
    root = tk.Tk()
    app = DrumGUI(root)
    root.mainloop()
