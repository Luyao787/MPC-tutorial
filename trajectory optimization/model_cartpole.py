import casadi as ca
import numpy as np
class CartPoleModel:    
    def __init__(self, dt, L=1., m1=1., m2=1., g=9.81):        
        self.num_state = 4
        self.num_input = 1

        self.dt = dt
        self._L = L
        self._m1 = m1
        self._m2 = m2
        self._g = g

        self._create_model()

    def _create_model(self):
        L = self._L 
        m1 = self._m1
        m2 = self._m2
        g = self._g

        q = ca.SX.sym("q", self.num_state)
        u = ca.SX.sym("u", self.num_input)

        # q1: cart horizontal position
        # q2: pole angle
        # q3: cart horizontal velocity
        # q4: pole angular rate
        q1, q2, q3, q4 = ca.vertsplit(q, 1)
        
        q1_dot = q3
        q2_dot = q4
        q3_dot =  (L*m2*ca.sin(q2)*q4*q4 + u + m2*g*ca.cos(q2)*ca.sin(q2)) / (m1 + m2*(1-ca.cos(q2)**2))
        q4_dot = -(L*m2*ca.cos(q2)*ca.sin(q2)*q4*q4 + u*ca.cos(q2) + (m1+m2)*g*ca.sin(q2)) / (L*m1 + L*m2*(1-ca.cos(q2)**2))

        q_dot = ca.vertcat(q1_dot, q2_dot, q3_dot, q4_dot)
        self.dyn_dt_fun = ca.Function("dynamics_dt_fun", [q, u], [q + self.dt * q_dot])
        
        jac_dyn_q = ca.jacobian(q + self.dt * q_dot, q)
        jac_dyn_u = ca.jacobian(q + self.dt * q_dot, u)
        
        self.jac_dyn_q_fun = ca.Function("jac_dyn_q_fun", [q, u], [jac_dyn_q]) 
        self.jac_dyn_u_fun = ca.Function("jac_dyn_u_fun", [q, u], [jac_dyn_u])
    
    def get_dynamics_matrices(self, q, u):
        A = self.jac_dyn_q_fun(q, u)
        B = self.jac_dyn_u_fun(q, u)
        return A.full(), B.full()
    
    def compute_next_state(self, q, u):
        q_next = self.dyn_dt_fun(q, u)
        return np.squeeze(q_next.full())
    
    # def compute_next_state_approx(self, x, u, x_r, u_r):
    #     dfdx = self.jac_dyn_x_fun(x_r, u_r) 
    #     dfdu = self.jac_dyn_u_fun(x_r, u_r)
    #     x_next = self.compute_next_state(x_r, u_r) + dfdx @ (x - x_r) + dfdu @ (u - u_r)
    #     return x_next