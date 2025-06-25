from pythonfmu import (
    Fmi2Variability,
    Fmi2Causality,
    Fmi2Slave,
    Real
)

import numpy as np
import roboticstoolbox as rtb


class DynamicsFMU(Fmi2Slave):
    author = "Alexandr Dikov"
    description = "FMU dynamics"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.robot = rtb.models.DH.Puma560()
        self.n = self.robot.n
        
        default_dt = 0.05

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

            name = f"velocity_{i}"
            self.register_variable(
                Real(
                    name, 
                    causality=Fmi2Causality.input,
                    variability=Fmi2Variability.continuous,
                    start=0.0
                )
            )
            setattr(self, name, 0.0)

            name = f"torque_{i}"
            self.register_variable(
                Real(
                    name,
                    causality=Fmi2Causality.input,
                    variability=Fmi2Variability.continuous,
                    start=0.0
                )
            )
            setattr(self, name, 0.0)

            name = f"new_angle_{i}"
            self.register_variable(
                Real(
                    name,
                    causality=Fmi2Causality.output,
                    variability=Fmi2Variability.discrete,
                    initial="calculated"
                )
            )

            name = f"new_velocity_{i}"
            self.register_variable(
                Real(
                    name, 
                    causality=Fmi2Causality.output,
                    variability=Fmi2Variability.discrete,
                    initial="calculated"
                )
            )

            name = f"acceleration_{i}"
            self.register_variable(
                Real(
                    name, 
                    causality=Fmi2Causality.output,
                    variability=Fmi2Variability.discrete, 
                    initial="calculated"
                )
            )

        self.register_variable(
            Real(
                "dt", 
                causality=Fmi2Causality.parameter,
                variability=Fmi2Variability.tunable,
                start=default_dt
            )
        )
        setattr(self, "dt", default_dt)

    def do_step(self, current_time, step_size):
        angles = np.array([
            getattr(self, f"angle_{i}") for i in range(self.n)
        ])
        velocities = np.array([
            getattr(self, f"velocity_{i}") for i in range(self.n)
        ])
        torques = np.array([
            getattr(self, f"torque_{i}") for i in range(self.n)
        ])

        dt = step_size if step_size is not None else getattr(self, "dt")

        accelerations = self.robot.accel(angles, velocities, torques)
        new_velocities = velocities + accelerations * dt
        new_angles = (
            angles
            + velocities * dt 
            + 0.5 * accelerations * dt ** 2
        )

        for i in range(self.n):
            setattr(self, f"new_angle_{i}", float(new_angles[i]))
            setattr(self, f"new_velocity_{i}", float(new_velocities[i]))
            setattr(self, f"acceleration_{i}", float(accelerations[i]))

        return True