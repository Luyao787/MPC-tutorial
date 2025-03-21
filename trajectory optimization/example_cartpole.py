import numpy as np
from ilqr import run_ilqr
import matplotlib.pyplot as plt
from dataclasses import dataclass
from model_cartpole import CartPoleModel
from visualization import visualize_cartpole

@dataclass
class PlannerParams:
    Q: np.ndarray = np.diag([1., 20., 1., 1.])
    R: np.ndarray = np.diag([0.1])
    Qf: np.ndarray = 10 * Q
    max_iter: int = 500
    # max_iter: int = 2
    regu_init: float = 100.
    max_regu: float = 1e4
    min_regu: float = 0.01
    N : int = 100

    xf: np.ndarray = np.array([0., np.pi, 0., 0.])
    uf: np.ndarray = np.array([0.])

if __name__ == "__main__":
    # Setup problem and call iLQR
    x0 = np.array([0., 0., 0., 0.])
    params = PlannerParams()
    model = CartPoleModel(dt=0.05)
    q_trj, u_trj, cost_trace, regu_trace, redu_ratio_trace, redu_trace, _, _ = run_ilqr(params, model, x0)

    # plt.figure(figsize=(9.5,8))
    # # Plot circle
    # theta = np.linspace(0, 2*np.pi, 100)
    # plt.plot(r*np.cos(theta), r*np.sin(theta), linewidth=5)
    # ax = plt.gca()

    # Plot resulting trajecotry of car
    # plt.plot(x_trj[:,0], x_trj[:,1], '-o')
    # plt.plot(x_trj[:,2], '-o')
    # plt.plot(x_trj[:,0], '-o')
    # plt.plot(x_trj[:,1], '-o')

    # plt.axis("equal")

    visualize_cartpole(q_trj, dt=0.05, L=1.)
