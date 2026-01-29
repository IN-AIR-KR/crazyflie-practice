from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import numpy as np


def setup_acados_solver():
    model = AcadosModel()
    p = ca.SX.sym('p', 3)
    v = ca.SX.sym('v', 3)
    yaw = ca.SX.sym('yaw')
    x = ca.vertcat(p, v, yaw)
    u = ca.SX.sym('u', 4)  # [phi, theta, psi_dot, thrust]
    m = 0.028
    g = 9.81

    # 비선형 동역학
    ax = (u[3]/m) * (ca.cos(x[6])*ca.sin(u[1])*ca.cos(u[0]) + ca.sin(x[6])*ca.sin(u[0]))
    ay = (u[3]/m) * (ca.sin(x[6])*ca.sin(u[1])*ca.cos(u[0]) - ca.cos(x[6])*ca.sin(u[0]))
    az = (u[3]/m) * (ca.cos(u[1])*ca.cos(u[0])) - g

    model.f_expl_expr = ca.vertcat(v, ax, ay, az, u[2])
    model.x, model.u, model.name = x, u, 'cf_flatness'

    ocp = AcadosOcp()
    ocp.model = model
    N = 20
    Tf = 0.2
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = Tf

    nx = 7
    nu = 4
    ny = 11
    ny_e = 7

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_0 = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    # 가중치
    Q = np.array([80, 80, 100, 10, 10, 10, 100])
    R = np.array([1, 1, 5, 0.1])
    W = np.diag(np.concatenate([Q, R]))

    ocp.cost.W = W
    ocp.cost.W_0 = W
    ocp.cost.W_e = np.diag(Q * 2.0)

    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_0 = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_0 = np.zeros((ny, nx))
    ocp.cost.Vx_0[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vu_0 = np.zeros((ny, nu))
    ocp.cost.Vu_0[nx:, :nu] = np.eye(nu)

    # 제약 조건
    max_ang = np.radians(55)
    ocp.constraints.lbu = np.array([-max_ang, -max_ang, -4.0, 0.0])
    ocp.constraints.ubu = np.array([max_ang, max_ang, 4.0, 0.8])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.x0 = np.zeros(nx)

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.integrator_type = 'ERK'

    return AcadosOcpSolver(ocp, json_file='acados_ocp.json')
