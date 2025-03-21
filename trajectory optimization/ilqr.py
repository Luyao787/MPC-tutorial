import numpy as np

def stage_cost(params, x, u, k):
    Q = params.Q
    R = params.R
    # dx = x - params.x_trj_ref[:,k]
    # du = u - params.u_trj_ref[:,k]  
    dx = x - params.xf
    du = u - params.uf  
    return 0.5 * dx.T @ Q @ dx + 0.5 * du.T @ R @ du

def final_cost(params, x):
    N = params.N
    Qf = params.Qf
    # dx = x - params.x_trj_ref[:,N]
    dx = x - params.xf
    return 0.5 * dx.T @ Qf @ dx

def stage_cost_expansion(params, x, u, k):
    Q = params.Q
    R = params.R
    # dx = x - params.x_trj_ref[:,k]
    # du = u - params.u_trj_ref[:,k] 
    dx = x - params.xf
    du = u - params.uf 
    # l_xx, l_ux, l_uu, l_x, l_u
    return Q, np.zeros((R.shape[0], Q.shape[0])), R, Q @ dx, R @ du 

def final_cost_expansion(params, x):
    N = params.N
    Qf = params.Qf
    # dx = x - params.x_trj_ref[:,N]
    dx = x - params.xf
    # lf_xx, lf_x
    return Qf, Qf @ dx

def trajectory_cost(params, x_trj, u_trj):
    N = params.N
    cost = 0.
    for k in range(N):
        cost += stage_cost(params, x_trj[k, :], u_trj[k, :], k)
    cost += final_cost(params, x_trj[N, :])
    return cost

def backward_pass(params, model, x_trj, u_trj, regu):
    N = params.N
    K_trj = np.zeros([u_trj.shape[0], u_trj.shape[1], x_trj.shape[1]])
    d_trj = np.zeros([u_trj.shape[0], u_trj.shape[1]])
    expected_cost_redu = 0.

    # final cost expansion
    V_xx, V_x = final_cost_expansion(params, x_trj[N, :])

    for k in range(N-1, -1, -1):
        # dynamics jacobians
        A, B = model.get_dynamics_matrices(x_trj[k, :], u_trj[k, :])

        # stage cost expansion
        l_xx, l_ux, l_uu, l_x, l_u = stage_cost_expansion(params, x_trj[k, :], u_trj[k, :], k)

        # Q function expansion
        Q_x  = l_x + A.T @ V_x
        Q_u  = l_u + B.T @ V_x
        Q_xx = l_xx + A.T @ V_xx @ A
        Q_uu = l_uu + B.T @ V_xx @ B
        Q_ux = l_ux + B.T @ V_xx @ A

        # add regularization to ensure that Q_uu is invertible and well conditioned
        # Q_uu_regu = Q_uu + np.eye(Q_uu.shape[0]) * regu
        # Q_uu_regu = Q_uu + np.eye(Q_uu.shape[0]) * 0.09
        Q_uu_regu = Q_uu    # without regularization

        Q_uu_regu_inv = np.linalg.inv(Q_uu_regu)
        K = - Q_uu_regu_inv @ Q_ux
        d = - Q_uu_regu_inv @ Q_u
        # K = np.linalg.solve(Q_uu_regu, -Q_ux)
        # d = np.linalg.solve(Q_uu_regu, -Q_u)
        K_trj[k, :, :] = K
        d_trj[k, :]    = d
        
        # cost-to-go
        V_xx = Q_xx + K.T @ Q_uu @ K + K.T @ Q_ux + Q_ux.T @ K
        V_x  = Q_x  + K.T @ Q_uu @ d + K.T @ Q_u  + Q_ux.T @ d

        # expected cost reduction
        expected_cost_redu += -Q_u.T @ d - 0.5 * d.T @ Q_uu @ d

    return K_trj, d_trj, expected_cost_redu

def forward_pass(params, model, 
                 x_trj, u_trj, K_trj, d_trj, 
                 cost):
    x_trj_new = np.zeros(x_trj.shape)
    x_trj_new[0, :] = x_trj[0, :]
    u_trj_new = np.zeros(u_trj.shape)
    
    alpha = 1.

    # rollout
    for k in range(u_trj.shape[0]):
        u_trj_new[k, :] = u_trj[k, :] + K_trj[k, :, :] @ (x_trj_new[k, :] - x_trj[k, :]) + alpha * d_trj[k, :]
        x_trj_new[k+1, :] = model.compute_next_state(x_trj_new[k, :], u_trj_new[k, :])
   
    cost_new = trajectory_cost(params, x_trj_new, u_trj_new)
    
    # # do line search
    # while cost_new > cost:
    #     alpha *= 0.5
    #     for k in range(u_trj.shape[0]):
    #         u_trj_new[k, :] = u_trj[k, :] + K_trj[k, :, :] @ (x_trj_new[k, :] - x_trj[k, :]) + alpha * d_trj[k, :]
    #         x_trj_new[k+1, :] = model.compute_next_state(x_trj_new[k, :], u_trj_new[k, :])
    #     cost_new = trajectory_cost(params, x_trj_new, u_trj_new)

    return x_trj_new, u_trj_new

def rollout(model, x0, u_trj):
    x_trj = np.zeros((u_trj.shape[0]+1, x0.shape[0]))
    x_trj[0, :] = x0
    for k in range(u_trj.shape[0]):
        x_trj[k+1, :] = model.compute_next_state(x_trj[k, :], u_trj[k, :])
    return x_trj

def run_ilqr(params, model, x0):

    N = params.N
    num_state = model.num_state
    num_input = model.num_input
    # u_trj = np.random.randn(N, num_input)*0.001
    u_trj = np.random.randn(N, num_input)*0.
    x_trj = rollout(model, x0, u_trj)
    x_trj_hist = [x_trj]
    u_trj_hist = [u_trj]
    total_cost = trajectory_cost(params, x_trj, u_trj)
    regu = params.regu_init

    regu = 0.

    max_regu = params.max_regu
    min_regu = params.min_regu
    max_iter = params.max_iter

    # setup traces
    cost_trace = [total_cost]
    expected_cost_redu_trace = []
    redu_ratio_trace = [1]
    redu_trace = []
    regu_trace = [regu]

    # run main loop
    for it in range(max_iter):
        print(f"num of iter: {it}")
        # backward pass
        K_trj, d_trj, expected_cost_redu = backward_pass(params, model, x_trj, u_trj, regu)
        # forward pass
        x_trj_new, u_trj_new = forward_pass(params, model, x_trj, u_trj, K_trj, d_trj, cost_trace[-1])

        x_trj_hist.append(x_trj_new)
        u_trj_hist.append(u_trj_new)

        # evaluate new trajectory
        total_cost = trajectory_cost(params, x_trj_new, u_trj_new)
        cost_redu = cost_trace[-1] - total_cost
        redu_ratio = cost_redu / abs(expected_cost_redu)
        # # accept or reject iteration
        # if cost_redu > 0:
        #     redu_ratio_trace.append(redu_ratio)
        #     cost_trace.append(total_cost)
        #     x_trj = x_trj_new
        #     u_trj = u_trj_new
        #     regu *= 0.7
        # else:
        #     print("do regularization!")
        #     regu *= 2.0
        #     cost_trace.append(cost_trace[-1])
        #     redu_ratio_trace.append(0)
        
        redu_ratio_trace.append(redu_ratio)
        cost_trace.append(total_cost)
        x_trj = x_trj_new
        u_trj = u_trj_new
        
        # regu = min(max(regu, min_regu), max_regu)
        
        regu_trace.append(regu)
        redu_trace.append(cost_redu)


        # early termination if expected improvement is small
        if expected_cost_redu <= 1e-6:
            break

    return x_trj, u_trj, cost_trace, regu_trace, redu_ratio_trace, redu_trace, x_trj_hist, u_trj_hist
