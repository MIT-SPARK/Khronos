#!/usr/bin/env python3
import tkinter as tk
import tkinter.ttk as ttk
import rospy
import rosservice
from khronos_msgs.srv import (
    SpatioTemporalVisualizerSetup,
    SpatioTemporalVisualizerSetTimeMode,
    SpatioTemporalVisualizerSetState,
    SpatioTemporalVisualizerSetStateRequest,
)
from std_srvs.srv import SetBool
from khronos_msgs.msg import SpatioTemporalVisualizerState
import numpy as np
import time

# Can disable this for dummy GUI development
USE_ROS = True


class SpatioTemporalVisualizerGUIConfig:
    def __init__(self) -> None:
        # Where to listen to and request data from
        self.visualizer_ns = "/spatio_temporal_visualizer"
        self.query_time_unit = "s"  # s, ns, idx
        self.robot_time_unit = "s"  # s, ns, idx

    @staticmethod
    def from_rosparam(sub_namespace="gui"):
        ns = "/" + sub_namespace + "/" if sub_namespace else ""
        config = SpatioTemporalVisualizerGUIConfig()

        if not USE_ROS:
            return config

        config.visualizer_ns = rospy.get_param(
            f"~{ns}visualizer_ns", "/spatio_temporal_visualizer"
        )
        config.query_time_unit = rospy.get_param(f"~{ns}query_time_unit", "s")
        config.robot_time_unit = rospy.get_param(f"~{ns}robot_time_unit", "s")
        return config

    def verify(self):
        if self.query_time_unit not in ["s", "ns"]:
            print(f"Invalid query time unit: {self.query_time_unit}, using 's' instead")
            self.query_time_unit = "s"
        if self.robot_time_unit not in ["s", "ns", "idx"]:
            print(f"Invalid robot time unit: {self.robot_time_unit}, using 's' instead")
            self.robot_time_unit = "s"


class SpatioTemporalVisualizerGUI(tk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.grid()

        # Data.
        self.config = SpatioTemporalVisualizerGUIConfig.from_rosparam()
        self.config.verify()
        self.map_stamps = []
        self.playing = False

        # The underlying times as stamps
        self.rt_index = 0
        self.robot_time = 0
        self.query_time = 0
        self.time_mode = 0

        # Setup.
        self.create_window()
        self.connect()
        self.initialize_variables()
        self.create_widgets()
        self.update_query_time()
        self.update_robot_time()

    def create_window(self):
        self.master.title("Spatio Temporal Visualizer")
        self.width = 1100
        self.height = 200
        self.master.geometry(f"{self.width}x{self.height}")

    def connect(self):
        if not USE_ROS:
            self.map_stamps = [int(i * 1e9) for i in [2, 3, 4, 5]]
            return
        # Wait till the spatio temporal visualizer is up by waiting for the service to be available.
        # Show a temporary window while waiting.
        srv_name = rospy.resolve_name(f"{self.config.visualizer_ns}/is_setup")
        srv = rospy.ServiceProxy(srv_name, SpatioTemporalVisualizerSetup)
        srv_name = f"{self.config.visualizer_ns}/is_setup"
        msg = f"Waiting for spatio-temporal visualizer to setup on '{srv_name}'"
        tmp_label = tk.Label(self, text=msg, justify="left", anchor=tk.W)
        tmp_label.grid(row=0, column=0)
        tmp_label_dots = tk.Label(self, text="...", justify="left", anchor=tk.W)
        tmp_label_dots.grid(row=0, column=1)

        # Wait for the service to be available.
        dots = 0
        while not rospy.is_shutdown():
            available_srvs = rosservice.get_service_list()
            # NOTE(lschmid): srv in srvs didn't work for some reason
            found = False
            for s in available_srvs:
                if srv_name == s:
                    found = True
                    break
            if found:
                break
            dots = (dots + 1) % 4
            tmp_label_dots["text"] = "." * dots
            tmp_label_dots.update()
            rospy.sleep(0.25)

        # Remove the temporary label and store the setup.
        tmp_label.destroy()
        tmp_label_dots.destroy()
        setup = srv()
        self.map_stamps = [int(i) for i in setup.map_stamps]
        self.robot_time = setup.initial_robot_time
        self.query_time = setup.initial_query_time
        self.time_mode = setup.initial_time_mode

        # Connect to the visualizer services and topics.
        self.play_srv = rospy.ServiceProxy(f"{self.config.visualizer_ns}/play", SetBool)
        self.set_time_mode_srv = rospy.ServiceProxy(
            f"{self.config.visualizer_ns}/set_time_mode",
            SpatioTemporalVisualizerSetTimeMode,
        )
        self.set_state_srv = rospy.ServiceProxy(
            f"{self.config.visualizer_ns}/set_state", SpatioTemporalVisualizerSetState
        )
        self.set_play_forward_srv = rospy.ServiceProxy(
            f"{self.config.visualizer_ns}/set_play_forward", SetBool
        )
        self.vis_state_sub = rospy.Subscriber(
            f"{self.config.visualizer_ns}/state",
            SpatioTemporalVisualizerState,
            self.vis_state_cb,
        )
        print("[SpatioTemporalVisualizerGUI] Connected to the visualizer.")

    def initialize_variables(self):
        # Robot Time Unit
        self.rt_unit_var = tk.StringVar(self)
        self.rt_unit_var.set(self.config.robot_time_unit)

        # Query Time Unit
        self.qt_unit_var = tk.StringVar(self)
        self.qt_unit_var.set(self.config.query_time_unit)

        # Robot Time Options
        self.rt_options = self.compute_time_options(self.rt_unit_var.get())
        self.rt_entry_var = tk.StringVar(self)

        # Query Time Options
        self.qt_options = self.compute_time_options(self.qt_unit_var.get())
        self.qt_entry_var = tk.StringVar(self)

        # Time Mode
        self.time_mode_var = tk.StringVar(self)
        self.time_mode_var.set(
            "Time Mode: " + ["Robot", "Query", "Online"][self.time_mode]
        )

        # Play Forward
        self.play_forward = True

        # Frame Rate
        self.fr_var = tk.StringVar(self)
        self.fr_var.set("--- FPS")
        self.fr_data = []
        self.fr_last_update = None

    def create_widgets(self):
        # ========= Time Display =========
        # Query Time Display
        self.frame_1 = tk.Frame(self)
        self.frame_1.pack(side=tk.TOP)
        self.qt_label = tk.Label(self.frame_1, text="Query time:")
        self.qt_label.grid(row=0, column=0)
        self.qt_bar_var = tk.DoubleVar(self)
        self.qt_bar = ttk.Progressbar(
            self.frame_1,
            variable=self.qt_bar_var,
            maximum=1.0,
            orient=tk.HORIZONTAL,
            length=self.width - 320,
        )
        self.qt_bar.grid(row=0, column=1)
        self.qt_entry = ttk.Combobox(
            self.frame_1,
            textvariable=self.qt_entry_var,
            values=self.qt_options,
            width=18,
        )
        self.qt_entry.bind("<<ComboboxSelected>>", self.set_qt_cb)
        self.qt_entry.bind("<Return>", self.set_qt_cb)
        self.qt_entry.grid(row=0, column=2)
        self.qt_unit = tk.OptionMenu(
            self.frame_1,
            self.qt_unit_var,
            "s",
            "ns",
            "idx",
            command=self.set_qt_unit_cb,
        )
        self.qt_unit.configure(width=3)
        self.qt_unit.grid(row=0, column=3)

        # Robot Time Display
        self.rt_label = tk.Label(self.frame_1, text="Robot time:")
        self.rt_label.grid(row=1, column=0)
        self.rt_bar_var = tk.DoubleVar(self)
        self.rt_bar = ttk.Progressbar(
            self.frame_1,
            variable=self.rt_bar_var,
            maximum=1.0,
            orient=tk.HORIZONTAL,
            length=self.width - 320,
        )
        self.rt_bar.grid(row=1, column=1)
        self.rt_entry = ttk.Combobox(
            self.frame_1,
            textvariable=self.rt_entry_var,
            values=self.rt_options,
            width=18,
        )
        self.rt_entry.bind("<<ComboboxSelected>>", self.set_rt_cb)
        self.rt_entry.bind("<Return>", self.set_rt_cb)
        self.rt_entry.grid(row=1, column=2)
        self.rt_unit = tk.OptionMenu(
            self.frame_1,
            self.rt_unit_var,
            "s",
            "ns",
            "idx",
            command=self.set_rt_unit_cb,
        )
        self.rt_unit.configure(width=3)
        self.rt_unit.grid(row=1, column=3)

        # Scale Bar
        self.scale_bar = tk.Scale(
            self.frame_1,
            from_=0,
            to=len(self.map_stamps) - 1,
            tickinterval=1,
            orient=tk.HORIZONTAL,
            command=self.set_scale_cb,
            length=self.width - 320,
            showvalue=False,
        )
        self.scale_bar.grid(row=2, column=1)

        # ========= Play and Mode Controls =========
        self.frame_2 = tk.Frame(self)
        self.frame_2.pack(side=tk.TOP)

        # Play Button
        self.play_btn = tk.Button(
            self.frame_2, text="Play", command=self.play_cb, bg="green", width=20
        )
        self.play_btn.grid(row=0, column=0)

        # Time Mode Menu
        self.time_mode_menu = tk.OptionMenu(
            self.frame_2,
            self.time_mode_var,
            "Robot",
            "Query",
            "Online",
            command=self.set_time_mode_cb,
        )
        self.time_mode_menu.configure(width=20)
        self.time_mode_menu.configure(bg="yellow")
        self.time_mode_menu.grid(row=0, column=1)

        # Play Forward Button
        self.play_forward_btn = tk.Button(
            self.frame_2,
            text="Play: Forward",
            command=self.play_forward_cb,
            width=10,
            bg="light blue",
        )
        self.play_forward_btn.grid(row=0, column=2)

        # Frame rate
        self.fr_entry = tk.Entry(
            self.frame_2, width=10, state="disabled", textvariable=self.fr_var
        )
        self.fr_entry.grid(row=0, column=3)

    def vis_state_cb(self, msg: SpatioTemporalVisualizerState):
        if msg.query_time > 0:
            self.query_time = int(msg.query_time)
            self.update_query_time()
        if msg.robot_time > 0:
            self.robot_time = int(msg.robot_time)
            self.update_robot_time()

        if not self.playing:
            return

        # Update frame rate in a sliding window.
        curr_time = time.time()
        if self.fr_last_update is not None:
            self.fr_data.append(curr_time - self.fr_last_update)
            if len(self.fr_data) > 30:
                self.fr_data.pop(0)
            self.fr_var.set(f"{30 / np.sum(self.fr_data):.1f} FPS")
        self.fr_last_update = time.time()

        # Check if we are done playing.
        if self.play_limit_reached():
            self.stop_play()

    def play_cb(self):
        if self.playing:
            self.stop_play()
            self.play_srv(False)
        elif self.play_limit_reached():
            print(
                f"Can't play {'forward' if self.play_forward else 'backward'} from time limit."
            )
        else:
            self.start_play()
            self.play_srv(True)

    def start_play(self):
        self.play_btn["text"] = "Pause"
        self.play_btn["bg"] = "red"
        self.playing = True
        self.fr_data = []
        self.fr_last_update = None

    def stop_play(self):
        self.play_btn["text"] = "Play"
        self.play_btn["bg"] = "green"
        self.playing = False

    def play_forward_cb(self):
        if self.play_forward:
            self.play_forward_btn["text"] = "Play: Backward"
            self.play_forward_btn["bg"] = "plum1"
            self.play_forward = False
        else:
            self.play_forward_btn["text"] = "Play: Forward"
            self.play_forward_btn["bg"] = "light blue"
            self.play_forward = True
        self.set_play_forward_srv(self.play_forward)

    def set_scale_cb(self, val):
        # Set the query and or robot time with the scale bar.
        if not USE_ROS:
            return
        self.rt_index = int(val)
        req = SpatioTemporalVisualizerSetStateRequest()
        if self.time_mode != 0:
            self.query_time = self.map_stamps[self.rt_index]
            self.update_query_time()
            req.state.query_time = self.query_time
        if self.time_mode != 1:
            self.robot_time = self.map_stamps[self.rt_index]
            self.update_robot_time()
            req.state.robot_time = self.robot_time
        self.set_state_srv(req)

    def set_time_mode_cb(self, val):
        if val == "Robot":
            self.time_mode = 0
        elif val == "Query":
            self.time_mode = 1
        elif val == "Online":
            self.time_mode = 2
        else:
            print(f"Encountered invalid time mode: {val}")
            return

        self.time_mode = self.set_time_mode_srv(self.time_mode).resulting_time_mode
        self.time_mode_var.set(
            "Time Mode: " + ["Robot", "Query", "Online"][self.time_mode]
        )

    def set_qt_cb(self, _):
        self.read_qt_entry()
        self.update_query_time()
        if self.time_mode == 2:
            self.robot_time = self.query_time
            self.update_robot_time()

        if not USE_ROS:
            return

        req = SpatioTemporalVisualizerSetStateRequest()
        req.state.query_time = self.query_time
        req.state.robot_time = self.robot_time if self.time_mode == 2 else 0
        self.set_state_srv(req)

    def set_qt_unit_cb(self, _):
        self.qt_options = self.compute_time_options(self.qt_unit_var.get())
        self.qt_entry["values"] = self.qt_options
        self.update_query_time()

    def set_rt_cb(self, _):
        self.read_rt_entry()
        self.update_robot_time()
        if self.time_mode == 2:
            self.query_time = self.robot_time
            self.update_query_time()

        if not USE_ROS:
            return

        req = SpatioTemporalVisualizerSetStateRequest()
        req.state.robot_time = self.robot_time
        req.state.query_time = self.query_time if self.time_mode == 2 else 0
        self.set_state_srv(req)

    def set_rt_unit_cb(self, _):
        self.rt_options = self.compute_time_options(self.rt_unit_var.get())
        self.rt_entry["values"] = self.rt_options
        self.update_robot_time()

    def read_rt_entry(self):
        # Set the robot_time to what is set in the robot time entry.
        self.robot_time = self.time_from_string(
            self.rt_entry_var.get(), self.rt_unit_var.get()
        )

    def read_qt_entry(self):
        # Set the query_time to what is set in the query time entry.
        self.query_time = self.time_from_string(
            self.qt_entry_var.get(), self.qt_unit_var.get()
        )

    def update_robot_time(self):
        # Update the visuals to reflect the internal robot time and compute the rt_index.
        if self.robot_time < self.map_stamps[0]:
            self.robot_time = self.map_stamps[0]
        elif self.robot_time > self.map_stamps[-1]:
            self.robot_time = self.map_stamps[-1]

        # Compute the rt_index
        for i, v in enumerate(self.map_stamps):
            if v <= self.robot_time:
                self.rt_index = i
            else:
                break

        self.rt_entry_var.set(
            self.time_to_string(self.robot_time, self.rt_unit_var.get(), self.rt_index)
        )

        progress = (self.robot_time - self.map_stamps[0]) / (
            self.map_stamps[-1] - self.map_stamps[0]
        )
        self.rt_bar_var.set(progress)

        # Update the scale bar.
        return
        # NOTE(lschmid): This unfortunately re-triggers the scale bar callback so disable it for now.
        self.scale_bar.configure(command=None)
        self.scale_bar.set(self.rt_index)
        self.scale_bar.configure(command=self.set_scale_cb)

    def update_query_time(self):
        # Update the visuals of to reflect the internal query time.
        if self.query_time < self.map_stamps[0]:
            self.query_time = self.map_stamps[0]
        elif self.query_time > self.map_stamps[-1]:
            self.query_time = self.map_stamps[-1]

        self.qt_entry_var.set(
            self.time_to_string(self.query_time, self.qt_unit_var.get())
        )

        progress = (self.query_time - self.map_stamps[0]) / (
            self.map_stamps[-1] - self.map_stamps[0]
        )
        self.qt_bar_var.set(progress)

    def play_limit_reached(self):
        if self.play_forward:
            if self.time_mode == 1:
                return self.query_time >= self.robot_time
            return self.robot_time >= self.map_stamps[-1]
        else:
            if self.time_mode == 1:
                return self.query_time <= self.map_stamps[0]
            return self.robot_time <= self.map_stamps[0]

    def compute_time_options(self, unit):
        # Update the options of the robot time entry
        if unit == "idx":
            return [i for i in range(len(self.map_stamps))]
        elif unit == "ns":
            return self.map_stamps
        else:
            return [f"{self.toSecs(i):.1f}" for i in self.map_stamps]

    def time_to_string(self, stamp, unit, index=None):
        if unit == "s":
            return f"{self.toSecs(stamp):.1f}"
        if unit == "ns":
            return f"{stamp:d}"

        if index is None:
            for i, v in enumerate(self.map_stamps):
                if v <= stamp:
                    index = i
                else:
                    break

        if stamp == self.map_stamps[index]:
            return str(index)

        fraction = (stamp - self.map_stamps[index]) / (
            self.map_stamps[index + 1] - self.map_stamps[index]
        )
        return f"{self.rt_index + fraction:.2f}"

    def time_from_string(self, string, unit):
        if unit == "s":
            return int(float(string) * 1e9) + self.map_stamps[0]

        if unit == "ns":
            return int(self.qt_entry_var.get())

        index = int(np.floor(float(string)))
        index = max(0, min(index, len(self.map_stamps) - 1))
        decimal = float(string) - index
        value = self.map_stamps[index]
        if index < len(self.map_stamps) - 1:
            value += int(decimal * (self.map_stamps[index + 1] - value))
        return value

    def toSecs(self, stamp):
        if stamp < self.map_stamps[0]:
            return self.map_stamps[0]
        elif stamp > self.map_stamps[-1]:
            return self.map_stamps[-1]
        else:
            return (stamp - self.map_stamps[0]) / 1e9


def on_shutdown(app: SpatioTemporalVisualizerGUI):
    # Force tk shutdown.
    print("[SpatioTemporalVisualizerGUI] Shutting down.")
    app.master.quit()
    # app.master.destroy()


def main():
    if USE_ROS:
        rospy.init_node("spatio_temporal_visualizer_gui")
    root = tk.Tk()
    app = SpatioTemporalVisualizerGUI(master=root)
    if USE_ROS:
        rospy.on_shutdown(lambda: on_shutdown(app))
    app.mainloop()


if __name__ == "__main__":
    main()
