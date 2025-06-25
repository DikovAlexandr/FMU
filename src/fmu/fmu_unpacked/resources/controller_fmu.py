from pythonfmu import (
    Fmi2Variability,
    Fmi2Causality,
    Fmi2Slave,
    Real
)

try:
    import numpy as np
    import roboticstoolbox as rtb
except ImportError:
    np, rtb = None, None


class ControllerFMU(Fmi2Slave):
    author = "Alexandr Dikov"
    description = "FMU controller"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.robot = rtb.models.DH.Puma560()
        self.n = self.robot.n
        
        default_kp = 3e3
        default_kd = 10.0

        for i in range(self.n):
            name = f"angle_{i}"
            self.register_variable(
                Real(
                    name, 
                    causality=Fmi2Causality.input,
                    variability=Fmi2Variability.continuous,
                    start=0.0
                )
            )
            setattr(self, name, 0.0)

            name = f"angle_target_{i}"
            self.register_variable(
                Real(name, 
                     causality=Fmi2Causality.input,
                     variability=Fmi2Variability.continuous,
                     start=0.0)
            )
            setattr(self, name, 0.0)

            name = f"velocity_{i}"
            self.register_variable(
                Real(name,
                     causality=Fmi2Causality.input,
                     variability=Fmi2Variability.continuous,
                     start=0.0)
            )
            setattr(self, name, 0.0)

            name = f"velocity_target_{i}"
            self.register_variable(
                Real(name,
                     causality=Fmi2Causality.input,
                     variability=Fmi2Variability.continuous,
                     start=0.0)
            )
            setattr(self, name, 0.0)

            name = f"acceleration_target_{i}"
            self.register_variable(
                Real(name,
                     causality=Fmi2Causality.input,
                     variability=Fmi2Variability.continuous,
                     start=0.0)
            )
            setattr(self, name, 0.0)

            name = f"torque_{i}"
            self.register_variable(
                Real(name,
                     causality=Fmi2Causality.output,
                     variability=Fmi2Variability.continuous,
                     initial="calculated")
            )

        name = "kp"
        self.register_variable(
            Real(name,
                 causality=Fmi2Causality.parameter,
                 variability=Fmi2Variability.tunable,
                 start=default_kp
            )
        )
        setattr(self, name, float(default_kp))

        name = "kd"
        self.register_variable(
            Real(name,
                 causality=Fmi2Causality.parameter,
                 variability=Fmi2Variability.tunable,
                 start=default_kd
            )
        )
        setattr(self, name, float(default_kd))

    def do_step(self, current_time, step_size):
        kp = getattr(self, "kp")
        kd = getattr(self, "kd")

        angles = np.array([
            getattr(self, f"angle_{i}") for i in range(self.n)
        ])
        angle_targets = np.array([
            getattr(self, f"angle_target_{i}") for i in range(self.n)
        ])
        velocities = np.array([
            getattr(self, f"velocity_{i}") for i in range(self.n)
        ])
        velocity_targets = np.array([
            getattr(self, f"velocity_target_{i}") for i in range(self.n)
        ])
        acceleration_targets = np.array([
            getattr(self, f"acceleration_target_{i}") for i in range(self.n)
        ])

        mass = self.robot.inertia(angles)
        coriolis = self.robot.coriolis(angles, velocities)
        weight = self.robot.gravload(angles)

        kp_matrix = np.diag([kp] * self.n)
        kd_matrix = np.diag([kd] * self.n)
        error_angles = angle_targets - angles
        error_velocities = velocity_targets - velocities
        correction_accelerations = (
            kp_matrix @ error_angles + kd_matrix @ error_velocities
        )

        torques = (
            mass @ (acceleration_targets + correction_accelerations)
            + coriolis @ velocities
            + weight
        )

        for i in range(self.n):
            setattr(self, f"torque_{i}", float(torques[i]))
        
        return True