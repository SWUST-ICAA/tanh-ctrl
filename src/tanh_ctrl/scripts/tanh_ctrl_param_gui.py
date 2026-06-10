#!/usr/bin/env python3

import argparse
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters


DOUBLE_PARAMETERS = [
    "model.mass",
    "model.gravity",
    "model.max_collective_thrust",
    "model.diagonal_wheelbase_m",
    "model.moment_to_thrust_ratio_m",
    "position.horizontal.M_P",
    "position.vertical.M_P",
    "position.horizontal.K_P",
    "position.vertical.K_P",
    "position.horizontal.M_V",
    "position.vertical.M_V",
    "position.horizontal.K_V",
    "position.vertical.K_V",
    "position.horizontal.K_Acceleration",
    "position.vertical.K_Acceleration",
    "position.horizontal.observer.P_V",
    "position.vertical.observer.P_V",
    "position.horizontal.observer.L_V",
    "position.vertical.observer.L_V",
    "position.max_tilt_deg",
    "attitude.tilt.M_Angle",
    "attitude.yaw.M_Angle",
    "attitude.tilt.K_Angle",
    "attitude.yaw.K_Angle",
    "attitude.tilt.M_AngularVelocity",
    "attitude.yaw.M_AngularVelocity",
    "attitude.tilt.K_AngularVelocity",
    "attitude.yaw.K_AngularVelocity",
    "attitude.tilt.K_AngularAcceleration",
    "attitude.yaw.K_AngularAcceleration",
    "attitude.tilt.observer.P_AngularVelocity",
    "attitude.yaw.observer.P_AngularVelocity",
    "attitude.tilt.observer.L_AngularVelocity",
    "attitude.yaw.observer.L_AngularVelocity",
    "filters.velocity_disturbance_cutoff_hz",
    "filters.angular_velocity_disturbance_cutoff_hz",
]

DOUBLE_ARRAY_PARAMETERS = [
    "model.inertia_diag",
]

PARAMETERS = DOUBLE_PARAMETERS + DOUBLE_ARRAY_PARAMETERS


class TanhCtrlParamGui:
    def __init__(self, ros_node, target_node):
        self.ros_node = ros_node
        self.target_node = target_node.rstrip("/")
        self.get_client = ros_node.create_client(GetParameters, f"{self.target_node}/get_parameters")
        self.set_client = ros_node.create_client(SetParameters, f"{self.target_node}/set_parameters")
        self.entries = {}

        self.root = tk.Tk()
        self.root.title("tanh_ctrl parameters")
        self.root.geometry("760x860")
        self.status_var = tk.StringVar(value=f"Target: {self.target_node}")

        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build_widgets(self):
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill=tk.BOTH, expand=True)

        toolbar = ttk.Frame(outer)
        toolbar.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(toolbar, text="Refresh", command=self.refresh).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="Apply", command=self.apply).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(toolbar, textvariable=self.status_var).pack(side=tk.LEFT, padx=(16, 0))

        canvas = tk.Canvas(outer, highlightthickness=0)
        scrollbar = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        self.form = ttk.Frame(canvas)
        self.form.bind("<Configure>", lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.form, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        row = 0
        for name in PARAMETERS:
            ttk.Label(self.form, text=name).grid(row=row, column=0, sticky=tk.W, padx=(0, 10), pady=3)
            entry = ttk.Entry(self.form, width=36)
            entry.grid(row=row, column=1, sticky=tk.EW, pady=3)
            self.entries[name] = entry
            row += 1

        self.form.columnconfigure(1, weight=1)

    def _wait_for_services(self):
        if not self.get_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"Service not available: {self.target_node}/get_parameters")
        if not self.set_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"Service not available: {self.target_node}/set_parameters")

    def _spin_future(self, future):
        rclpy.spin_until_future_complete(self.ros_node, future, timeout_sec=2.0)
        if not future.done():
            raise TimeoutError("ROS parameter service timed out")
        result = future.result()
        if result is None:
            raise RuntimeError("ROS parameter service returned no result")
        return result

    def refresh(self):
        try:
            self._wait_for_services()
            request = GetParameters.Request()
            request.names = PARAMETERS
            result = self._spin_future(self.get_client.call_async(request))
            for name, value in zip(PARAMETERS, result.values):
                entry = self.entries[name]
                entry.delete(0, tk.END)
                entry.insert(0, self._value_to_text(value))
            self.status_var.set("Parameters refreshed")
        except Exception as exc:
            self.status_var.set("Refresh failed")
            messagebox.showerror("Refresh failed", str(exc))

    def apply(self):
        try:
            self._wait_for_services()
            request = SetParameters.Request()
            request.parameters = [self._make_parameter(name, self.entries[name].get()) for name in PARAMETERS]
            result = self._spin_future(self.set_client.call_async(request))
            failures = [item.reason for item in result.results if not item.successful]
            if failures:
                raise RuntimeError("; ".join(failures))
            self.status_var.set("Parameters applied")
        except Exception as exc:
            self.status_var.set("Apply failed")
            messagebox.showerror("Apply failed", str(exc))

    def _value_to_text(self, value):
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return f"{value.double_value:.12g}"
        if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return ", ".join(f"{item:.12g}" for item in value.double_array_value)
        return ""

    def _make_parameter(self, name, text):
        parameter = Parameter()
        parameter.name = name
        if name in DOUBLE_ARRAY_PARAMETERS:
            values = [float(item.strip()) for item in text.split(",") if item.strip()]
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=values)
        else:
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(text))
        return parameter

    def run(self):
        self.root.after(200, self.refresh)
        self.root.mainloop()

    def close(self):
        self.root.destroy()


def parse_args():
    parser = argparse.ArgumentParser(description="tanh_ctrl runtime parameter GUI")
    parser.add_argument("--target-node", default="/tanh_ctrl", help="Target ROS2 node name")
    args, ros_args = parser.parse_known_args()
    return args, ros_args


def main():
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    ros_node = rclpy.create_node("tanh_ctrl_param_gui")
    try:
        app = TanhCtrlParamGui(ros_node, args.target_node)
        app.run()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
