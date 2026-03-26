import math

def clamp(value, min_value, max_value):
    """Limit a numeric value to the closed interval [min_value, max_value]."""
    return max(min_value, min(value, max_value))

class ElectricVehiclePlant:
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
