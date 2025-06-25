from pythonfmu import (
    Fmi2Variability,
    Fmi2Causality,
    Fmi2Slave,
    Integer,
    Real
)

import numpy as np
import roboticstoolbox as rtb


class TrajectoryFMU(Fmi2Slave):
    author = "Alexandr Dikov"
    description = "FMU trajectory"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.robot = rtb.models.DH.Puma560()
        self.n = self.robot.n

        self.start = np.zeros(self.n)
        self.end = np.zeros(self.n)

        default_t_start = 0.0
        default_t_end = 5.0
        default_n_points = 101
        default_index = 0

        for i in range(self.n):
            name = f"start_{i}"
            self.register_variable(
                Real(
                    f"start_{i}",
                    causality=Fmi2Causality.input,
                    variability=Fmi2Variability.continuous,
                    start=0.0
                )
            )
            setattr(self, name, float(self.start[i]))

            name = f"end_{i}"
            self.register_variable(
                Real(
                    name,
                    causality=Fmi2Causality.input,
                    variability=Fmi2Variability.continuous,
                    start=0.0
                )
            )
            setattr(self, name, float(self.end[i]))

        self.register_variable(
            Real(
                "t_start",
                causality=Fmi2Causality.input,
                variability=Fmi2Variability.continuous,
                start=default_t_start
            )
        )
        setattr(self, "t_start", default_t_start)

        self.register_variable(
            Real(
                "t_end",
                causality=Fmi2Causality.input,
                variability=Fmi2Variability.continuous,
                start=default_t_end
            )
        )
        setattr(self, "t_end", default_t_end)

        self.register_variable(
            Integer(
                "n_points",
                causality=Fmi2Causality.input,
                variability=Fmi2Variability.discrete,
                start=default_n_points
            )
        )
        setattr(self, "n_points", default_n_points)

        self.register_variable(
            Integer(
                "index",
                causality=Fmi2Causality.input,
                variability=Fmi2Variability.discrete,
                start=default_index
            )
        )
        setattr(self, "index", default_index)

        for i in range(self.n):
            self.register_variable(
                Real(
                    f"q_{i}",
                    causality=Fmi2Causality.output,
                    variability=Fmi2Variability.discrete,
                    initial="calculated"
                )
            )
            self.register_variable(
                Real(
                    f"qd_{i}",
                    causality=Fmi2Causality.output,
                    variability=Fmi2Variability.discrete,
                    initial="calculated"
                )
            )
            self.register_variable(
                Real(
                    f"qdd_{i}",
                    causality=Fmi2Causality.output,
                    variability=Fmi2Variability.discrete,
                    initial="calculated"
                )
            )

        self._traj = None
        self._prev_params = None

    def do_step(self, current_time, step_size):
        for i in range(self.n):
            self.start[i] = getattr(self, f"start_{i}")
            self.end[i] = getattr(self, f"end_{i}")
        self.t_start = getattr(self, "t_start")
        self.t_end = getattr(self, "t_end")
        self.n_points = getattr(self, "n_points")
        self.index = getattr(self, "index")

        params = (
            tuple(self.start),
            tuple(self.end),
            self.t_start,
            self.t_end,
            self.n_points,
        )
        if self._prev_params != params:
            t_vec = np.linspace(self.t_start, self.t_end, self.n_points)
            self._traj = rtb.jtraj(self.start, self.end, t_vec)
            self._prev_params = params

        idx = min(max(self.index, 0), self.n_points - 1)
        for i in range(self.n):
            setattr(self, f"q_{i}",   float(self._traj.q[idx, i]))
            setattr(self, f"qd_{i}",  float(self._traj.qd[idx, i]))
            setattr(self, f"qdd_{i}", float(self._traj.qdd[idx, i]))

        return True