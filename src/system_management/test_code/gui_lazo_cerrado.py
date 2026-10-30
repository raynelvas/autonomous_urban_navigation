#!/usr/bin/env python3
"""
GUI + Nodo ROS2 para enviar referencias de velocidad angular a ambas ruedas,
visualizar en tiempo real la velocidad medida y grabar datos para identificación.

- Publishes geometry_msgs/Twist on /cmd_vel:
    .linear.x -> right wheel reference (rad/s)
    .angular.z -> left  wheel reference (rad/s)
- Subscribes nav_msgs/Odometry on /odom/unfiltered:
    .twist.twist.linear.x  -> right wheel measured (rad/s)
    .twist.twist.angular.z -> left  wheel measured (rad/s)

Controles:
- Sinusoidal: genera y envía una onda senoidal con parámetros Amplitude (rad/s) y Frequency (Hz)
- Step:     : envía una referencia constante (valor configurable)
- Stop motors: publica un 0 único y detiene generación
- Start/Stop recording: crea CSV con columnas:
    t_cmd_local, t_cmd_ros(n/a), cmd_right, cmd_left, t_odom_local, t_odom_ros, w_right, w_left
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

import threading, time, csv, os
from collections import deque
import numpy as np
from datetime import datetime
import signal
import sys

# -----------------------
# CONFIGURABLE PARAMETERS
# -----------------------
PUBLISH_PERIOD = 0.1            # [s] Period between publishes / GUI update (default 100 ms)
PLOT_WINDOW = 10.0              # [s] seconds shown in the plot
LOG_DIR = os.path.expanduser("~/robot_logs")  # default folder to save CSVs
MAX_POINTS = int(PLOT_WINDOW / PUBLISH_PERIOD)  # samples to keep in plot
# -----------------------

class RosGuiNode(Node):
    def __init__(self):
        super().__init__('gui_identificacion_node')
        self.cli_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom/unfiltered', self.odom_cb, 10)
        # state variables for odometry
        self.w_right = 0.0
        self.w_left = 0.0
        self.t_odom_ros = None
        self.t_odom_local = None

    def odom_cb(self, msg: Odometry):
        # Read measured speeds (as used in your Arduino code)
        try:
            self.w_right = float(msg.twist.twist.linear.x)
            self.w_left = float(msg.twist.twist.angular.z)
        except Exception:
            # fallback if fields missing
            self.w_right = 0.0
            self.w_left = 0.0

        # timestamp from ROS msg header (seconds)
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        # if header stamp is zero (unsynced), treat as None
        if sec == 0 and nsec == 0:
            self.t_odom_ros = None
        else:
            self.t_odom_ros = sec + nsec * 1e-9

        # also record local time at reception
        self.t_odom_local = time.time()

    def publish_cmd(self, right_val, left_val):
        msg = Twist()
        msg.linear.x = float(right_val)
        msg.angular.z = float(left_val)
        # record local time of publishing
        t_cmd_local = time.time()
        # ROS doesn't require us to set header timestamps for cmd messages
        self.cli_pub.publish(msg)
        return t_cmd_local  # return local timestamp for record
    
    def destroy_node(self):
        super().destroy_node()

class GUI:
    def __init__(self, master, ros_node: RosGuiNode):
        self.master = master
        self.node = ros_node
        self.master.title("Identificación - Control de Ruedas")
        self.running = True
        self._after_ids = []  # Track all after callbacks

        # generator states
        self.mode = tk.StringVar(value="idle")  # idle | sine | step
        self.sine_params = {'A': 1.0, 'f': 0.5}  # amplitude (rad/s), frequency (Hz)
        self.step_value = 1.0  # rad/s

        # recording state
        self.recording = False
        self.csv_file = None
        self.csv_writer = None

        # command values currently being published
        self.cmd_right = 0.0
        self.cmd_left = 0.0
        self.last_publish_time = 0.0

        # plot buffers
        self.t_buf = deque(maxlen=MAX_POINTS)
        self.cmd_r_buf = deque(maxlen=MAX_POINTS)
        self.cmd_l_buf = deque(maxlen=MAX_POINTS)
        self.meas_r_buf = deque(maxlen=MAX_POINTS)
        self.meas_l_buf = deque(maxlen=MAX_POINTS)

        # UI layout
        self._build_controls()
        self._build_plot()

        # start periodic update
        self._schedule_next()

    def _build_controls(self):
        frm = ttk.Frame(self.master)
        frm.pack(side=tk.TOP, fill=tk.X, padx=6, pady=6)

        # Sine controls
        sframe = ttk.LabelFrame(frm, text="Senoidal (afecta ambas ruedas)")
        sframe.pack(side=tk.LEFT, padx=6)
        ttk.Label(sframe, text="Amplitud [rad/s]").grid(row=0, column=0)
        self.entry_amp = ttk.Entry(sframe, width=8)
        self.entry_amp.grid(row=0, column=1)
        self.entry_amp.insert(0, str(self.sine_params['A']))

        ttk.Label(sframe, text="Freq [Hz]").grid(row=1, column=0)
        self.entry_freq = ttk.Entry(sframe, width=8)
        self.entry_freq.grid(row=1, column=1)
        self.entry_freq.insert(0, str(self.sine_params['f']))

        btn_sine = ttk.Button(sframe, text="Start Sinusoidal", command=self.start_sine)
        btn_sine.grid(row=2, column=0, columnspan=2, pady=4)

        # Step controls
        stframe = ttk.LabelFrame(frm, text="Escalón (afecta ambas ruedas)")
        stframe.pack(side=tk.LEFT, padx=6)
        ttk.Label(stframe, text="Valor [rad/s]").grid(row=0, column=0)
        self.entry_step = ttk.Entry(stframe, width=8)
        self.entry_step.grid(row=0, column=1)
        self.entry_step.insert(0, str(self.step_value))
        btn_step = ttk.Button(stframe, text="Start Step", command=self.start_step)
        btn_step.grid(row=1, column=0, columnspan=2, pady=4)

        # Stop motors
        btn_stop = ttk.Button(frm, text="Parar motores", command=self.stop_motors)
        btn_stop.pack(side=tk.LEFT, padx=10)

        # Recording controls
        rframe = ttk.LabelFrame(frm, text="Grabación")
        rframe.pack(side=tk.LEFT, padx=6)
        ttk.Label(rframe, text="Folder:").grid(row=0, column=0)
        self.entry_folder = ttk.Entry(rframe, width=28)
        self.entry_folder.grid(row=0, column=1)
        self.entry_folder.insert(0, LOG_DIR)
        btn_browse = ttk.Button(rframe, text="...", width=3, command=self.browse_folder)
        btn_browse.grid(row=0, column=2)
        self.btn_start_rec = ttk.Button(rframe, text="Start Recording", command=self.start_record)
        self.btn_start_rec.grid(row=1, column=0, columnspan=2, pady=4)
        self.btn_stop_rec = ttk.Button(rframe, text="Stop Recording", command=self.stop_record, state=tk.DISABLED)
        self.btn_stop_rec.grid(row=1, column=2, pady=4)

        # status
        self.status_label = ttk.Label(self.master, text="Estado: idle")
        self.status_label.pack(side=tk.TOP, pady=4)

    def _build_plot(self):
        self.fig, (self.ax_r, self.ax_l) = plt.subplots(2,1, figsize=(7,5))
        plt.tight_layout()

        # initial lines
        (self.line_cmd_r,) = self.ax_r.plot([], [], label='cmd_r')
        (self.line_meas_r,) = self.ax_r.plot([], [], label='meas_r')
        self.ax_r.set_title('Rueda Derecha')
        self.ax_r.set_ylabel('rad/s')
        self.ax_r.legend()
        self.ax_r.grid(True)

        (self.line_cmd_l,) = self.ax_l.plot([], [], label='cmd_l')
        (self.line_meas_l,) = self.ax_l.plot([], [], label='meas_l')
        self.ax_l.set_title('Rueda Izquierda')
        self.ax_l.set_ylabel('rad/s')
        self.ax_l.set_xlabel('t [s]')
        self.ax_l.legend()
        self.ax_l.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    def browse_folder(self):
        path = filedialog.askdirectory(initialdir=self.entry_folder.get())
        if path:
            self.entry_folder.delete(0, tk.END)
            self.entry_folder.insert(0, path)

    # ----------------------------
    # Control actions (buttons)
    # ----------------------------
    def start_sine(self):
        try:
            A = float(self.entry_amp.get())
            f = float(self.entry_freq.get())
        except ValueError:
            messagebox.showerror("Error", "Parámetros de sinusoide inválidos")
            return
        self.sine_params['A'] = A
        self.sine_params['f'] = f
        self.mode.set("sine")
        self.sine_start_time = time.time()
        self.status_label.config(text=f"Estado: sinusoidal A={A} f={f}")
        # ensure continuous publish will pick it up in the periodic callback

    def start_step(self):
        try:
            v = float(self.entry_step.get())
        except ValueError:
            messagebox.showerror("Error", "Valor de escalón inválido")
            return
        self.step_value = v
        self.mode.set("step")
        self.status_label.config(text=f"Estado: step v={v}")

    def stop_motors(self):
        # stop generation and send a single zero command
        self.mode.set("idle")
        self.cmd_right = 0.0
        self.cmd_left = 0.0
        tpub = self.node.publish_cmd(0.0, 0.0)
        # optionally record this stop event
        if self.recording and self.csv_writer:
            self.csv_writer.writerow([tpub, None, 0.0, 0.0,
                                      None, None, 0.0, 0.0])
        self.status_label.config(text="Estado: idle (motores detenidos)")

    def start_record(self):
        if self.recording:
            return
        folder = self.entry_folder.get()
        os.makedirs(folder, exist_ok=True)
        filename = datetime.now().strftime("ident_%Y%m%d_%H%M%S.csv")
        filepath = os.path.join(folder, filename)
        self.csv_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # header: t_cmd_local, t_cmd_ros(n/a), cmd_right, cmd_left, t_odom_local, t_odom_ros, w_right, w_left
        self.csv_writer.writerow(['t_cmd_local', 't_cmd_ros', 'cmd_right', 'cmd_left',
                                  't_odom_local', 't_odom_ros', 'w_right', 'w_left'])
        self.recording = True
        self.btn_start_rec.config(state=tk.DISABLED)
        self.btn_stop_rec.config(state=tk.NORMAL)
        self.status_label.config(text=f"Recording -> {filepath}")

    def stop_record(self):
        if not self.recording:
            return
        self.recording = False
        try:
            self.csv_file.close()
        except Exception:
            pass
        self.csv_file = None
        self.csv_writer = None
        self.btn_start_rec.config(state=tk.NORMAL)
        self.btn_stop_rec.config(state=tk.DISABLED)
        self.status_label.config(text="Recording stopped")

    # ----------------------------
    # Periodic update (publishing, plotting)
    # ----------------------------
    def _schedule_next(self):
        # called periodically in the main thread via Tk after()
        after_id = self.master.after(int(PUBLISH_PERIOD*1000), self._schedule_next)
        self._after_ids.append(after_id)
        self._periodic_update()

    def _periodic_update(self):
        # compute desired command depending on mode
        t_now = time.time()
        dt = t_now - getattr(self, 'last_publish_time', t_now)
        self.last_publish_time = t_now

        mode = self.mode.get()
        if mode == "sine":
            t_rel = t_now - self.sine_start_time
            A = self.sine_params['A']
            f = self.sine_params['f']
            val = A * np.sin(2.0 * np.pi * f * t_rel)
            # both wheels same reference
            self.cmd_right = val
            self.cmd_left = val
        elif mode == "step":
            val = self.step_value
            self.cmd_right = val
            self.cmd_left = val
        else:
            # idle: keep previous commands but we do not continuously publish unless nonzero?
            # We'll continue publishing to keep controller updated, but values are whatever cmd_right is
            pass

        # publish command regularly (even in idle we publish last command, but we send 0 when user pressed Stop)
        tcmd_local = self.node.publish_cmd(self.cmd_right, self.cmd_left)

        # record (if recording)
        if self.recording and self.csv_writer:
            # t_cmd_ros is not applicable for cmd messages; we write None
            t_cmd_ros = None
            t_odom_local = self.node.t_odom_local
            t_odom_ros = self.node.t_odom_ros
            w_r = self.node.w_right
            w_l = self.node.w_left
            self.csv_writer.writerow([f"{tcmd_local:.6f}", t_cmd_ros,
                                      f"{self.cmd_right:.6f}", f"{self.cmd_left:.6f}",
                                      f"{t_odom_local:.6f}" if t_odom_local else None,
                                      f"{t_odom_ros:.6f}" if t_odom_ros else None,
                                      f"{w_r:.6f}", f"{w_l:.6f}"])

        # update buffers for plotting
        tref = t_now - getattr(self, 't0', t_now)
        if not hasattr(self, 't0'):
            self.t0 = t_now
            tref = 0.0

        self.t_buf.append(tref)
        self.cmd_r_buf.append(self.cmd_right)
        self.cmd_l_buf.append(self.cmd_left)
        self.meas_r_buf.append(self.node.w_right)
        self.meas_l_buf.append(self.node.w_left)

        # redraw plot
        self._update_plot()

    def _update_plot(self):
        xs = np.array(self.t_buf)
        if xs.size == 0:
            return
        self.line_cmd_r.set_data(xs, np.array(self.cmd_r_buf))
        self.line_meas_r.set_data(xs, np.array(self.meas_r_buf))
        self.line_cmd_l.set_data(xs, np.array(self.cmd_l_buf))
        self.line_meas_l.set_data(xs, np.array(self.meas_l_buf))

        # adjust axes - FIXED: prevent singular transformations
        xmin = xs.max() - PLOT_WINDOW if xs.max() > PLOT_WINDOW else 0
        xmax = xs.max()
        
        # Prevent setting identical left == right
        if xmin == xmax:
            xmin = max(0, xmin - 0.1)
            xmax = xmax + 0.1
            
        self.ax_r.set_xlim(xmin, xmax)
        self.ax_l.set_xlim(xmin, xmax)

        # autoscale y a algo razonable
        all_y = np.concatenate([np.array(self.cmd_r_buf), np.array(self.meas_r_buf),
                                np.array(self.cmd_l_buf), np.array(self.meas_l_buf)])
        if all_y.size:
            ymin = float(np.min(all_y))
            ymax = float(np.max(all_y))
            if abs(ymax - ymin) < 1e-3:
                ymin -= 0.5; ymax += 0.5
            self.ax_r.set_ylim(ymin - 0.1*abs(ymin+1e-6), ymax + 0.1*abs(ymax+1e-6))
            self.ax_l.set_ylim(ymin - 0.1*abs(ymin+1e-6), ymax + 0.1*abs(ymax+1e-6))

        self.canvas.draw()

    def cleanup(self):
        """Limpia todos los recursos antes de cerrar"""
        self.running = False
        
        # Cancela todos los callbacks programados de Tkinter
        for after_id in self._after_ids:
            try:
                self.master.after_cancel(after_id)
            except Exception:
                pass
        self._after_ids.clear()
        
        # Para motores
        self.stop_motors()
        
        # Detiene grabación si está activa
        if self.recording:
            self.stop_record()

    def on_close(self):
        """Maneja el cierre de la aplicación de forma ordenada"""
        self.cleanup()
        # Programa la destrucción de la ventana después de limpiar
        self.master.after(100, self.master.quit)

def ros_spin_thread(node: RosGuiNode, shutdown_event):
    """Hilo para ejecutar ROS2 spin con manejo de cierre"""
    try:
        while rclpy.ok() and not shutdown_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as e:
        print("ROS spin thread exception:", e)

def signal_handler(sig, frame, app, ros_node, shutdown_event):
    """Maneja señales como Ctrl+C"""
    print("\nRecibida señal de interrupción, cerrando...")
    if app:
        app.cleanup()
    shutdown_event.set()
    try:
        ros_node.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()
    sys.exit(0)

def main():
    # init rclpy
    rclpy.init()
    ros_node = RosGuiNode()
    shutdown_event = threading.Event()

    # Build GUI in main thread
    root = tk.Tk()
    app = GUI(root, ros_node)

    # Configurar manejador de señales
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, app, ros_node, shutdown_event))

    # start spin thread
    t = threading.Thread(target=ros_spin_thread, args=(ros_node, shutdown_event), daemon=True)
    t.start()

    # handle close
    def on_closing():
        if messagebox.askokcancel("Quit", "¿Cerrar y detener?"):
            app.on_close()
            shutdown_event.set()
            # shutdown ROS after GUI quits
            try:
                time.sleep(0.2)
                ros_node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass

    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("Interrupción por teclado en main loop")
    finally:
        # Limpieza final garantizada
        app.cleanup()
        shutdown_event.set()
        try:
            ros_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()