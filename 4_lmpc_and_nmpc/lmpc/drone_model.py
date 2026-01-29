from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import numpy as np


def setup_acados_solver():
    model = AcadosModel()

    # 상태 변수: 7차 px, py, pz, vx, vy, vz, yaw
    p = ca.SX.sym('p', 3)
    v = ca.SX.sym('v', 3)
    yaw = ca.SX.sym('yaw')
    x = ca.vertcat(p, v, yaw)

    # 제어 입력: 4차 ax, ay, az, yaw_rate
    u = ca.SX.sym('u', 4)

    # 동역학: Double Integrator + Yaw Rate
    model.f_expl_expr = ca.vertcat(v, u[:3], u[3])
    model.x, model.u, model.name = x, u, "cf_model"

    ocp = AcadosOcp()
    ocp.model = model
    N = 20
    Tf = 0.4
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = Tf

    # 가중치 튜닝
    nx = 7
    nu = 4
    ny = 11  # nx + nu
    # Q: [px, py, pz, vx, vy, vz, yaw]
    Q_diag = np.array([15, 15, 45, 1, 1, 1, 30])  # Yaw에 30 정도 가중치를 주어 방향 고수
    # R: [ax, ay, az, yaw_rate]
    R_diag = np.array([10, 10, 10, 5])
    W = np.diag(np.concatenate([Q_diag, R_diag]))

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_0 = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.W = W
    ocp.cost.W_0 = W
    ocp.cost.W_e = np.diag(Q_diag)

    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx
    ocp.cost.Vx_0 = Vx
    Vu = np.zeros((ny, nu))
    Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vu = Vu
    ocp.cost.Vu_0 = Vu
    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_0 = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(nx)

    # 제약 조건: 안전 가속도 및 Yaw Rate 제한
    max_acc = 1.5
    max_yaw_rate = 1.0  # rad/s
    ocp.constraints.lbu = np.array([-max_acc, -max_acc, -max_acc, -max_yaw_rate])
    ocp.constraints.ubu = np.array([max_acc, max_acc, max_acc, max_yaw_rate])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.x0 = np.zeros(nx)

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.integrator_type = 'ERK'

    return AcadosOcpSolver(ocp, json_file='acados_ocp.json')
