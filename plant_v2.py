import csv
import math
from pyModbusTCP.client import ModbusClient
import random
import time
import matplotlib.pyplot as plt
import sys

plc = ModbusClient(host="192.168.100.17", port=502, auto_open=True)

TC_ON=False
DRIVER_REQUEST=1500

class VehiclePlant:
    def __init__(self,road_scale=1.0):
        self.t=0.0
        self.tau_motor=0.08
        self.tau_tyre=0.035

        self.m=1805
        self.g=9.81
        self.L=2.77
        self.h=0.55
        self.lf=self.L/2
        self.W_total=self.m*self.g

        self.p_air_density=1.225
        self.C_drag=0.267
        self.A=2.36
        self.C_rr=0.015

        self.r=0.347
        self.J=1.20
        self.B=10.0
        self.C=1.9
        self.D_base=1.00
        self.road_scale=road_scale
        self.D=self.D_base*self.road_scale
        self.E=0.97

        self.v_car=0.0
        self.omega=0.0

        self.T_applied=400.0

        self.P_max = 75000.0

        self.F_z_static=(self.m*self.g*self.lf)/self.L
        self.F_z_dynamic=0.0
        self.F_z=self.F_z_static/2.0

        self.slip_ratio=0.0
        self.mu=0.0
        self.F_x=0.0
        self.F_drag=0.0
        self.F_roll=0.0
        self.T_actual=0.0

        self.T_resisting=0.0
        self.T_net=0.0
        self.a_angular=0.0
        self.a_linear=0.0

    def step(self,dt):
        self.t+=dt

        self.T_applied=max(0.0,min(3000.0,self.T_applied))

        T_motor = self.T_applied

        if self.omega > 0.5:
            T_power_limit = self.P_max / self.omega
            T_motor = min(self.T_applied, T_power_limit)

        self.T_actual += (dt / (self.tau_motor + dt)) * (T_motor - self.T_actual)

        v_wheel=self.r*self.omega
        raw_slip=(v_wheel-self.v_car)/max(abs(self.v_car),2.0)
        self.slip_ratio=max(-1.0,min(1.0,raw_slip))

        self.F_drag=0.5*self.p_air_density*self.C_drag*self.A*(self.v_car**2)
        self.F_roll=self.C_rr*self.m*self.g if self.v_car>0.1 else 0.0

        self.F_z_dynamic=(self.m*self.a_linear*self.h)/self.L
        rear_axle_load=self.F_z_static+self.F_z_dynamic
        clamped_axle=max(0.20*self.W_total,min(0.90*self.W_total,rear_axle_load))
        self.F_z=clamped_axle/2.0

        self.D=self.D_base*self.road_scale
        bx=self.B*self.slip_ratio
        self.mu=self.D*math.sin(self.C*math.atan(bx-self.E*(bx-math.atan(bx))))
        F_x_target=self.mu*self.F_z

        self.F_x+=(dt/(self.tau_tyre+dt))*(F_x_target-self.F_x)

        self.T_resisting=self.F_x*self.r
        self.T_net=self.T_actual-self.T_resisting
        self.a_angular=self.T_net/self.J

        self.a_linear=(2.0*self.F_x-self.F_drag-self.F_roll)/self.m

        if self.v_car<=0.0 and self.a_linear<0.0:
            self.a_linear=0.0

        self.omega=max(0.0,self.omega+self.a_angular*dt)
        self.v_car=max(0.0,self.v_car+self.a_linear*dt)

plant=VehiclePlant(road_scale=1.0)
dt=0.02
dt1=dt/5
steps=1000

t_hist = []
v_car_hist = []
v_wheel_hist = []
slip_hist = []
T_applied_hist = []
T_actual_hist = []
NOISE = 0.02

with (open("final_A2.csv",mode="w",newline="") as f):
    writer=csv.writer(f)
    writer.writerow(["t","v_car","omega","v_wheel","slip_ratio","T_applied","T_actual","F_x","F_z","a_linear","a_angular"])

    mode = str(input("Choose your environment (dry, ice, wet): ")).strip().lower()
    start_time = time.perf_counter()
    plot_title = mode
    if mode == "dry":
        plant.road_scale = 1.0
    elif mode == "wet":
        plant.road_scale = 0.6
    elif mode == "ice":
        plant.road_scale = 0.15
    else:
        print("unknown road")
        sys.exit()

    for i in range(steps):
        if mode=="dry" and plant.t>=10:
            plant.road_scale=0.15
            plot_title="DRY to ICE transition"

        if TC_ON:
            regs_read=plc.read_input_registers(0,1)
            if regs_read:
                plant.T_applied=float(regs_read[0])/10
            else:
                plant.T_applied=0
        else:
            plant.T_applied=DRIVER_REQUEST

        for k in range(5):
            plant.step(dt1)

        v_wheel=(plant.r*plant.omega)

        v_wheel_meas=v_wheel+random.gauss(0, NOISE)
        slip_meas=(v_wheel_meas-plant.v_car)/max(plant.v_car, 2.0)

        v_int=int(plant.v_car*100) & 0xFFFF
        wheel_int=int(v_wheel_meas*100) & 0xFFFF
        slip_int=int(slip_meas*1000) & 0xFFFF
        driver_int=int(DRIVER_REQUEST)

        plc.write_multiple_registers(1,[v_int, wheel_int, slip_int, driver_int, i+1])

        writer.writerow([f"{plant.t:.4f}", f"{plant.v_car:.4f}", f"{plant.omega:.4f}",
                         f"{v_wheel:.4f}", f"{plant.slip_ratio:.4f}", f"{plant.T_applied:.2f}",
                         f"{plant.T_actual:.2f}", f"{plant.F_x:.2f}", f"{plant.F_z:.2f}",
                         f"{plant.a_linear:.4f}", f"{plant.a_angular:.4f}"])

        t_hist.append(plant.t)
        v_car_hist.append(plant.v_car)
        v_wheel_hist.append(v_wheel)
        slip_hist.append(plant.slip_ratio)
        T_applied_hist.append(plant.T_applied)
        T_actual_hist.append(plant.T_actual)

    end_time=time.perf_counter()
    duration=end_time-start_time

    print(f"Simulation completed in {duration:.4f} seconds")

    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    axs[0].plot(t_hist, v_car_hist, label='Car Speed (m/s)', linewidth=2, color='blue')
    axs[0].plot(t_hist, v_wheel_hist, label='Wheel Speed (m/s)', linestyle='--', linewidth=2, color='orange')
    axs[0].set_ylabel('Speed (m/s)')
    axs[0].set_title(f'Traction Dynamics ({plot_title.upper()} Road)')
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(t_hist, slip_hist, label='Slip Ratio', color='red', linewidth=2)
    axs[1].set_ylabel('Slip Ratio')
    axs[1].axhline(y=0.15, color='gray', linestyle=':', label='Peak Grip Target (~15%)')
    axs[1].legend()
    axs[1].grid(True)

    axs[2].plot(t_hist, T_applied_hist, label='Commanded Torque (SoftPLC)', linestyle='--', color='purple')
    axs[2].plot(t_hist, T_actual_hist, label='Actual Motor Torque', color='green', linewidth=2)
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Torque (Nm)')
    axs[2].legend()
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()