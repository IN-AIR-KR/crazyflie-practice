from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import numpy as np


def setup_acados_solver():
    model = AcadosModel()

    # [상태 변수: 7차]
    p = ca.SX.sym('p', 3)
    v = ca.SX.sym('v', 3)
    yaw = ca.SX.sym('yaw')
    x = ca.vertcat(p, v, yaw)

    # 제어 입력: 4차
    # phi: roll, theta: pitch, psi_dot: yaw_rate, thrust: Newton
    u = ca.SX.sym('u', 4)
    phi, theta, psi_dot, thrust = u[0], u[1], u[2], u[3]

    m = 0.028
    g = 9.81

    # 비선형 동역학 식
    ax = (thrust/m) * (ca.cos(yaw)*ca.sin(theta)*ca.cos(phi) + ca.sin(yaw)*ca.sin(phi))
    ay = (thrust/m) * (ca.sin(yaw)*ca.sin(theta)*ca.cos(phi) - ca.cos(yaw)*ca.sin(phi))
    az = (thrust/m) * (ca.cos(theta)*ca.cos(phi)) - g

    model.f_expl_expr = ca.vertcat(v, ax, ay, az, psi_dot)
    model.x, model.u, model.name = x, u, 'cf_nmpc_model'

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = 20
    ocp.solver_options.tf = 0.3

    # 가중치 설정
    nx, nu, ny = 7, 4, 11
    Q_diag = np.array([700, 700, 700, 20, 20, 20, 1000])
    R_diag = np.array([0.6, 0.6, 0.6, 0.01])  # Thrust 페널티를 낮춰 양력 확보
    W = np.diag(np.concatenate([Q_diag, R_diag]))

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_0 = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.W = W
    ocp.cost.W_0 = W
    ocp.cost.W_e = np.diag(Q_diag*2.0)

    # 매핑 및 초기화
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_0 = np.zeros((ny, nx))
    ocp.cost.Vx_0[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vu_0 = np.zeros((ny, nu))
    ocp.cost.Vu_0[nx:, :nu] = np.eye(nu)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_0 = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(nx)

    # 제약 조건
    max_angle = np.radians(70)
    ocp.constraints.lbu = np.array([-max_angle, -max_angle, -4.5, 0.0])
    ocp.constraints.ubu = np.array([max_angle, max_angle, 4.5, 0.9])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.x0 = np.zeros(nx)

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.integrator_type = 'ERK'

    return AcadosOcpSolver(ocp, json_file='acados_ocp.json')
