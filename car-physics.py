import time
import math
import threading
import tkinter as tk
import matplotlib.pyplot as plt
from pyModbusTCP.server import ModbusServer


def clamp(value, min_value, max_value):
    """Limit a numeric value to the closed interval [min_value, max_value]."""
    return max(min_value, min(value, max_value))


class ElectricVehiclePlant:
    """
    Simplified longitudinal EV plant model for HIL testing.
    """

    def __init__(self):
        self.mass = 1500.0
        self.wheel_radius = 0.3
        self.wheel_inertia = 1.2
        self.drag_coeff = 0.3
        self.frontal_area = 2.2
        self.air_density = 1.225
        self.gravity = 9.81

        self.wheelbase = 2.7
        self.cg_height = 0.55
        self.driven_wheels = 2
        self.static_driven_axle_load_ratio = 0.50
        self.rolling_resistance_coeff = 0.015

        self.road_mu_scale = 1.0 
        self.low_speed_reference = 0.5

        self.B = 10.0
        self.C = 1.9
        self.D = 1.0
        self.E = 0.97

        self.velocity = 0.0
        self.wheel_omega = 0.0
        self.slip_ratio = 0.0
        self.longitudinal_accel = 0.0
        self.driven_axle_load = self.mass * self.gravity * self.static_driven_axle_load_ratio

    def calculate_driven_axle_load(self):
        static_load = self.mass * self.gravity * self.static_driven_axle_load_ratio
        dynamic_transfer = (self.mass * self.longitudinal_accel * self.cg_height) / self.wheelbase
        driven_axle_load = static_load + dynamic_transfer
        return clamp(driven_axle_load, 0.20 * self.mass * self.gravity, 0.90 * self.mass * self.gravity)

    def calculate_slip_ratio(self, wheel_linear_speed):
        reference_speed = max(abs(self.velocity), abs(wheel_linear_speed), self.low_speed_reference)
        return (wheel_linear_speed - self.velocity) / reference_speed

    def calculate_magic_formula_mu(self, slip_ratio):
        slip = clamp(slip_ratio, -1.5, 1.5)
        base_mu = self.D * math.sin(
            self.C * math.atan(self.B * slip - self.E * (self.B * slip - math.atan(self.B * slip)))
        )
        return self.road_mu_scale * base_mu

    def calculate_physics_step(self, motor_torque, dt):
        if self.velocity < 0.01 and self.wheel_omega < 0.01 and abs(motor_torque) < 1e-6:
            self.velocity = 0.0
            self.wheel_omega = 0.0
            self.slip_ratio = 0.0
            self.longitudinal_accel = 0.0
            return

        wheel_linear_speed = self.wheel_omega * self.wheel_radius
        self.slip_ratio = self.calculate_slip_ratio(wheel_linear_speed)

        self.driven_axle_load = self.calculate_driven_axle_load()
        load_per_driven_wheel = self.driven_axle_load / self.driven_wheels

        mu = self.calculate_magic_formula_mu(self.slip_ratio)
        tire_force_per_wheel = mu * load_per_driven_wheel
        total_tire_force = tire_force_per_wheel * self.driven_wheels

        aero_drag = 0.5 * self.air_density * self.drag_coeff * self.frontal_area * (self.velocity ** 2)
        rolling_resistance = self.rolling_resistance_coeff * self.mass * self.gravity

        wheel_alpha = (motor_torque - (tire_force_per_wheel * self.wheel_radius)) / self.wheel_inertia
        self.wheel_omega = max(0.0, self.wheel_omega + wheel_alpha * dt)

        vehicle_accel = (total_tire_force - aero_drag - rolling_resistance) / self.mass
        self.longitudinal_accel = vehicle_accel
        self.velocity = max(0.0, self.velocity + vehicle_accel * dt)

def start_weather_dashboard(ev_plant):
    """Runs a small GUI in a background thread to change weather live."""
    def set_asphalt():
        ev_plant.road_mu_scale = 1.0
        print("\n\n>>> [WEATHER UPDATE] Road surface changed to DRY ASPHALT (100% Grip) <<<\n")

    def set_rain():
        ev_plant.road_mu_scale = 0.6
        print("\n\n>>> [WEATHER UPDATE] Road surface changed to WET RAIN (60% Grip) <<<\n")

    def set_ice():
        ev_plant.road_mu_scale = 0.2
        print("\n\n>>> [WEATHER UPDATE] Road surface changed to SOLID ICE (20% Grip) <<<\n")

    root = tk.Tk()
    root.title("Weather Control")
    root.geometry("250x160")
    root.attributes("-topmost", True) # Forces window to stay on top of other apps

    tk.Label(root, text="Environmental Settings", font=("Helvetica", 11, "bold")).pack(pady=5)
    tk.Button(root, text="Dry Asphalt (\u03bc = 1.0)", bg="darkgray", fg="white", font=("Arial", 10, "bold"), command=set_asphalt).pack(fill=tk.X, padx=10, pady=2)
    tk.Button(root, text="Wet Rain (\u03bc = 0.6)", bg="royalblue", fg="white", font=("Arial", 10, "bold"), command=set_rain).pack(fill=tk.X, padx=10, pady=2)
    tk.Button(root, text="Solid Ice (\u03bc = 0.2)", bg="lightblue", font=("Arial", 10, "bold"), command=set_ice).pack(fill=tk.X, padx=10, pady=2)

    root.mainloop()

def run_hil_plant():
    print("--- Upgraded HIL Vehicle Plant Starting ---")
    ev = ElectricVehiclePlant()

    gui_thread = threading.Thread(target=start_weather_dashboard, args=(ev,), daemon=True)
    gui_thread.start()

    log_time = []
    log_torque = []
    log_velocity = []
    log_wheel_speed = []
    log_slip = []
    log_axle_load = []

    server = ModbusServer(host="0.0.0.0", port=5020, no_block=True)
    try:
        server.start()
        print("Modbus Server Online (Port 5020). Waiting for PLC torque command...")
        print(">>> PRESS 'Ctrl+C' IN THIS TERMINAL TO STOP SIMULATION AND GENERATE GRAPH <<<")
    except Exception as exc:
        print(f"Modbus Boot Failed: {exc}")
        return

    dt = 0.02
    sim_start_time = time.time()

    try:
        while True:
            loop_start = time.time()
            elapsed_time = time.time() - sim_start_time

            raw_torque = server.data_bank.get_holding_registers(0, 1)
            plc_motor_torque = float(raw_torque[0]) if raw_torque else 0.0

            ev.calculate_physics_step(motor_torque=plc_motor_torque, dt=dt)

            scaled_velocity = int(ev.velocity * 100)
            scaled_wheel_speed = int((ev.wheel_omega * ev.wheel_radius) * 100)
            scaled_slip = int(ev.slip_ratio * 1000)
            scaled_axle_load = int(ev.driven_axle_load)
            server.data_bank.set_holding_registers(
                1, [scaled_velocity, scaled_wheel_speed, scaled_slip, scaled_axle_load]
            )

            log_time.append(elapsed_time)
            log_torque.append(plc_motor_torque)
            log_velocity.append(ev.velocity)
            log_wheel_speed.append(ev.wheel_omega * ev.wheel_radius)
            log_slip.append(ev.slip_ratio)
            log_axle_load.append(ev.driven_axle_load)

            status = "ACCELERATING" if plc_motor_torque > 0 else "COASTING    "
            print(
                f"[{status}] Torque: {plc_motor_torque:6.1f} Nm | Veh Spd: {ev.velocity:5.2f} m/s "
                f"| Whl Spd: {ev.wheel_omega * ev.wheel_radius:5.2f} m/s | Slip: {ev.slip_ratio:6.3f} "
                f"| Axle Load: {ev.driven_axle_load:7.1f} N | Grip: {ev.road_mu_scale*100:3.0f}%  ",
                end="\r",
            )

            sleep_time = dt - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nSimulation Stopped. Generating Thesis Graphs...")
    finally:
        server.stop()

        if len(log_time) > 0:
            plt.figure(figsize=(10, 10))

            plt.subplot(4, 1, 1)
            plt.plot(log_time, log_torque, "k-", linewidth=2, label="Motor Torque (Nm)")
            plt.ylabel("Torque [Nm]")
            plt.title("HIL Plant Response: Dynamic Weather Intervention")
            plt.legend(loc="upper left")
            plt.grid(True)

            plt.subplot(4, 1, 2)
            plt.plot(log_time, log_wheel_speed, "r--", linewidth=2, label="Wheel Linear Speed (m/s)")
            plt.plot(log_time, log_velocity, "b-", linewidth=2, label="Vehicle Speed (m/s)")
            plt.ylabel("Speed [m/s]")
            plt.legend(loc="upper left")
            plt.grid(True)

            plt.subplot(4, 1, 3)
            plt.plot(log_time, log_slip, "g-", linewidth=2, label="Slip Ratio")
            plt.axhline(y=0.15, color="r", linestyle=":", linewidth=2, label="Reference Grip Zone")
            plt.ylabel("Slip Ratio")
            plt.legend(loc="upper right")
            plt.grid(True)

            plt.subplot(4, 1, 4)
            plt.plot(log_time, log_axle_load, "m-", linewidth=2, label="Driven Axle Load (N)")
            plt.ylabel("Load [N]")
            plt.xlabel("Time [Seconds]")
            plt.legend(loc="upper left")
            plt.grid(True)

            plt.tight_layout()
            plt.show()

if __name__ == "__main__":
    run_hil_plant()
