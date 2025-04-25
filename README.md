# Tutorial materials designed for the MPC course at TU Delft

This repository is actively being developed.

## Required background knowledge
- State space model 

- Discretization

- Stability

- Controllability & Observability

We recommend that students read the following sections in the textbook: Chapters 1.2, 1.3.5, 1.4.5, and 2.4.1.

Rawlings, J. B., Mayne D. Q., Diehl, M., Model Predictive Control: Theory, Computation, and Design, 2nd Edition, Nobhill Publishing.

## Reading

## Textbooks

- Rawlings, J. B., Mayne D. Q., Diehl, M., Model Predictive Control: Theory, Computation, and Design, 2nd Edition, Nobhill Publishing.

- Borrelli, Francesco et al. “Predictive Control for Linear and Hybrid Systems.”

## Papers & Books

Here are some interesting papers and books you might find valuable. Reading them is optional for this course.

### Numerical methods 

We categorize various methods for solving linear MPC problems according to how they handle inequality constraints. In the papers listed below, the system dynamics are treated as equality constraints, resulting in a banded sparse linear system that can be efficiently solved using Riccati recursion or sparse $LDL^\top$ factorization.

#### Interior point methods

- C. V. Rao, S. J. Wright, and J. B. Rawlings, “Application of Interior-Point Methods to Model Predictive Control,” Journal of Optimization Theory and Applications, vol. 99, no. 3, pp. 723–757, Dec. 1998.

- Y. Wang and S. Boyd, “Fast Model Predictive Control Using Online Optimization,” IEEE Transactions on Control Systems Technology, vol. 18, no. 2, pp. 267–278, Mar. 2010.

- A. G. Pandala, Y. Ding, and H.-W. Park, “qpSWIFT: A Real-Time Sparse Quadratic Program Solver for Robotic Applications,” IEEE Robotics and Automation Letters, vol. 4, no. 4, pp. 3355–3362, Oct. 2019.

- G. Frison and M. Diehl, “HPIPM: a high-performance quadratic programming framework for model predictive control,” Jun. 06, 2020.

#### Augmented Lagrangian methods

- K. F. Lowenstein, D. Bernardini, and P. Patrinos, “QPALM-OCP: A Newton-Type Proximal Augmented Lagrangian Solver Tailored for Quadratic Programs Arising in Model Predictive Control,” IEEE Control Syst. Lett., pp. 1–1, 2024.


Below is a list of learning materials on Quadratic Programming.

#### QP

- Bemporad, A., "Quadratic programming and explicit MPC", http://cse.lab.imtlucca.it/~bemporad/teaching/mpc/imt/3-qp_explicit.pdf.

- https://qpsolvers.github.io/qpsolvers/references.html#schwan2023

- D. Kouzoupis, G. Frison, A. Zanelli, and M. Diehl, “Recent Advances in Quadratic Programming Algorithms for Nonlinear Model Predictive Control,” Vietnam J. Math., vol. 46, no. 4, pp. 863–882, Dec. 2018, doi: 10.1007/s10013-018-0311-1.

- Gros, S., Zanon, M., Quirynen, R., Bemporad, A., & Diehl, M. (2016). From linear to nonlinear MPC: bridging the gap via the real-time iteration. International Journal of Control, 93(1), 62–80.

#### Trajectory Optimization

- Betts, John T., "Practical Methods for Optimal Control Using Nonlinear Programming, Third Edition".

### MPC and RL

- D. P. Bertsekas, “Model Predictive Control and Reinforcement Learning: A Unified Framework Based on Dynamic Programming,” Jun. 30, 2024.

