import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import matplotlib as mpl
for k in list(mpl.rcParams.keys()):
    if k.startswith("keymap."):
        mpl.rcParams[k] = []


def clamp_norm(v, max_norm):
    n = np.linalg.norm(v)
    if n < 1e-12:
        return v
    if n > max_norm:
        return v * (max_norm / n)
    return v


def rot2(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s],
                     [s,  c]], dtype=float)


def heading_from_dir(d, fallback_theta=0.0):
    n = np.linalg.norm(d)
    if n < 1e-9:
        return fallback_theta
    return np.arctan2(d[1], d[0])


def perp90(v):
    return np.array([-v[1], v[0]], dtype=float)

# =====================
# Formation slots (FOLLOWERS only)  (Leader+2 drones=3 vehicles)
# =====================


def offsets_line_follow_body(spacing=3.2, n_followers=2):
    return np.array([[-spacing*(i+1), 0.0] for i in range(n_followers)], dtype=float)


def offsets_v_apex_body(spacing=3.2, n_followers=2):
    if n_followers == 1:
        return np.array([[-spacing, 0.0]], dtype=float)
    return np.array([[-spacing, +spacing],
                     [-spacing, -spacing]], dtype=float)


def formation_slots_world(leader_pos, leader_heading, mode, spacing, n_followers):
    offsets = offsets_v_apex_body(
        spacing, n_followers) if mode == "v" else offsets_line_follow_body(spacing, n_followers)
    R = rot2(leader_heading)
    return leader_pos + offsets @ R.T

# =====================
# Simple CBBA (1 slot per drone)
# =====================


class CBBA:
    def __init__(self, n_agents, n_tasks):
        self.N = n_agents
        self.K = n_tasks
        self.s = np.full((self.N, self.K), -1, dtype=int)
        self.y = np.full((self.N, self.K), -np.inf, dtype=float)
        self.ts = np.zeros((self.N, self.K), dtype=int)
        self.bundle = [[] for _ in range(self.N)]
        self.iter_counter = 0

    def score(self, agent_pos, task_pos):
        return -np.linalg.norm(agent_pos - task_pos)

    def bundle_build(self, i, agent_pos, task_positions):
        if len(self.bundle[i]) >= 1:
            return
        best_k, best_b = None, -np.inf
        for k in range(self.K):
            if k in self.bundle[i]:
                continue
            b = self.score(agent_pos, task_positions[k])
            if b > self.y[i, k] and b > best_b:
                best_b, best_k = b, k
        if best_k is not None:
            self.bundle[i].append(best_k)
            self.s[i, best_k] = i
            self.y[i, best_k] = best_b
            self.ts[i, best_k] = self.iter_counter

    def consensus(self):
        self.iter_counter += 1
        global_s = np.full(self.K, -1, dtype=int)
        global_y = np.full(self.K, -np.inf, dtype=float)
        global_ts = np.full(self.K, -1, dtype=int)

        for k in range(self.K):
            best_w, best_b, best_t = -1, -np.inf, -1
            for i in range(self.N):
                w, b, t = self.s[i, k], self.y[i, k], self.ts[i, k]
                if (t > best_t) or (t == best_t and b > best_b):
                    best_w, best_b, best_t = w, b, t
            global_s[k], global_y[k], global_ts[k] = best_w, best_b, best_t

        for i in range(self.N):
            self.s[i, :] = global_s
            self.y[i, :] = global_y
            self.ts[i, :] = global_ts

        for i in range(self.N):
            self.bundle[i] = [k for k in self.bundle[i] if global_s[k] == i]

    def get_assignment(self):
        assign = np.full(self.N, -1, dtype=int)
        for i in range(self.N):
            if self.bundle[i]:
                assign[i] = self.bundle[i][0]
        return assign

# =====================
# Hard separation (ABSOLUTE min distance)
# =====================


def enforce_hard_separation(drones, leader_pos, d_dd, d_dl, iters=10, eps=1e-6):
    n = drones.shape[0]
    for _ in range(iters):
        changed = False

        for i in range(n):
            for j in range(i + 1, n):
                diff = drones[i] - drones[j]
                dist = np.linalg.norm(diff)
                if dist < eps:
                    dirv = np.random.uniform(-1.0, 1.0, size=2)
                    dirv = dirv / (np.linalg.norm(dirv) + eps)
                    dist = eps
                else:
                    dirv = diff / dist
                if dist < d_dd:
                    push = 0.5 * (d_dd - dist)
                    drones[i] += dirv * push
                    drones[j] -= dirv * push
                    changed = True

        for i in range(n):
            diff = drones[i] - leader_pos
            dist = np.linalg.norm(diff)
            if dist < eps:
                dirv = np.random.uniform(-1.0, 1.0, size=2)
                dirv = dirv / (np.linalg.norm(dirv) + eps)
                dist = eps
            else:
                dirv = diff / dist
            if dist < d_dl:
                push = (d_dl - dist)
                drones[i] += dirv * push
                changed = True

        if not changed:
            break


# =====================
# Simulation config
# =====================
np.random.seed()

N_DRONES = 2
dt = 0.05

formation_mode = "line"
spacing = 3.2

min_distance = 0.55 * spacing
avoid_buffer = 0.20 * spacing
rep_gain = 10.0
rep_cap = 4.0

leader_min_distance = 0.45 * spacing
leader_buffer = 0.20 * spacing
leader_rep_gain = 12.0
leader_rep_cap = 2.2

d_hard_dd = min(min_distance * 1.05, 0.80 * spacing)
d_hard_dl = min(leader_min_distance * 1.05, 0.80 * spacing)

drones = np.random.uniform(-8.0, 8.0, size=(N_DRONES, 2))
kp = 1.9
vmax = 2.0

# side-step robust
side_step_on = True
side_step_gain = 1.6
side_step_cap = 1.4
side_step_range = 2.8 * d_hard_dd
headon_cos = 0.55

leader_side_step_gain = 1.2
leader_side_step_cap = 1.2
leader_side_step_range = 2.8 * d_hard_dl
leader_headon_cos = 0.55

leader_yield_radius = 2.2 * d_hard_dl
leader_return_scale_min = 0.15

# Leader command position (x) + heading from last move dir
leader_cmd_pos = np.array([0.0, 0.0], dtype=float)
last_cmd_dir = np.array([1.0, 0.0], dtype=float)
leader_cmd_heading = 0.0
leader_cmd_speed = 2.2

# Actual leader (★) follows cmd + avoids
leader_pos = leader_cmd_pos.copy()
leader_vel = np.zeros(2, dtype=float)
leader_return_gain = 6.0
leader_return_cap = 2.7

# CBBA
cbba_replan_period = 0.35
cbba_iters_each_time = 7
cbba = CBBA(N_DRONES, N_DRONES)
assignment = np.full(N_DRONES, -1, dtype=int)
time_since_cbba = 0.0

# =====================
# Key state + robust normalization (IMPORTANT)
# =====================
DEBUG_KEYS = False

pressed = set()
idle_clear_sec = 0.25
time_since_key_event = 0.0

last_vel = np.zeros((N_DRONES, 2), dtype=float)
slot_targets = np.zeros((N_DRONES, 2), dtype=float)

# WASD mapping (now safe because Matplotlib default keymaps are disabled above)
KEY_ALIAS = {
    "w": "up",
    "a": "left",
    "s": "down",
    "d": "right",
}


def normalize_key(k):
    if k is None:
        return None
    k = str(k).lower().strip()
    if "+" in k:
        k = k.split("+")[-1]
    return KEY_ALIAS.get(k, k)


def reset_random_positions():
    return np.random.uniform(-8.0, 8.0, size=(N_DRONES, 2))

# =====================
# Repulsion (soft)
# =====================


def repulsion_from_point(pi, pj, d_min, buffer, gain):
    dvec = pi - pj
    dist = np.linalg.norm(dvec)
    if dist < 1e-9:
        return np.random.uniform(-1.0, 1.0, size=2)

    d0 = d_min + buffer
    if dist >= d0:
        return np.zeros(2, dtype=float)

    scale = (d0 - dist) / (d0 - d_min + 1e-9)
    scale = np.clip(scale, 0.0, 1.0)
    mag = gain * scale * (1.0 / max(dist, 1e-3) - 1.0 / d0)
    return (dvec / dist) * mag


def repulsion_velocity_drone(i):
    vi = np.zeros(2, dtype=float)
    pi = drones[i]
    for j in range(N_DRONES):
        if j == i:
            continue
        vi += repulsion_from_point(pi, drones[j], min_distance, avoid_buffer, rep_gain)
    vi += repulsion_from_point(pi, leader_pos, leader_min_distance, leader_buffer, leader_rep_gain)
    return clamp_norm(vi, rep_cap)


def repulsion_velocity_leader():
    v = np.zeros(2, dtype=float)
    for j in range(N_DRONES):
        v += repulsion_from_point(leader_pos, drones[j], leader_min_distance, leader_buffer, leader_rep_gain)
    return clamp_norm(v, leader_rep_cap)

# =====================
# Side-step (robust)
# =====================


def side_step_velocity_drone(i):
    if not side_step_on:
        return np.zeros(2, dtype=float)

    vi = np.zeros(2, dtype=float)
    pi = drones[i]

    dir_i = slot_targets[i] - pi
    if np.linalg.norm(dir_i) < 1e-6:
        dir_i = last_vel[i]
    if np.linalg.norm(dir_i) < 1e-9:
        dir_i = np.array([1.0, 0.0], dtype=float)
    dir_i = dir_i / (np.linalg.norm(dir_i) + 1e-12)

    for j in range(N_DRONES):
        if j == i:
            continue
        r = drones[j] - pi
        dist = np.linalg.norm(r)
        if dist < 1e-9 or dist > side_step_range:
            continue
        r_hat = r / dist
        if abs(np.dot(dir_i, r_hat)) < headon_cos:
            continue
        lat = perp90(r_hat) if i < j else -perp90(r_hat)
        w = (side_step_range - dist) / (side_step_range + 1e-9)
        vi += lat * np.clip(w, 0.0, 1.0) * side_step_gain

    rL = leader_pos - pi
    distL = np.linalg.norm(rL)
    if distL > 1e-9 and distL <= side_step_range:
        r_hat = rL / distL
        if abs(np.dot(dir_i, r_hat)) >= headon_cos:
            forward = np.array([np.cos(leader_cmd_heading), np.sin(leader_cmd_heading)], dtype=float)
            if np.linalg.norm(forward) < 1e-9:
                forward = last_cmd_dir.copy()
            forward = forward / (np.linalg.norm(forward) + 1e-12)

            preferred_side = perp90(forward)
            lat = perp90(r_hat)
            if np.dot(lat, preferred_side) < 0:
                lat = -lat

            w = (side_step_range - distL) / (side_step_range + 1e-9)
            vi += lat * np.clip(w, 0.0, 1.0) * side_step_gain

    return clamp_norm(vi, side_step_cap)


def side_step_velocity_leader():
    if not side_step_on:
        return np.zeros(2, dtype=float)

    v = np.zeros(2, dtype=float)
    forward = np.array([np.cos(leader_cmd_heading), np.sin(leader_cmd_heading)], dtype=float)
    if np.linalg.norm(forward) < 1e-9:
        forward = last_cmd_dir.copy()
    forward = forward / (np.linalg.norm(forward) + 1e-12)

    for j in range(N_DRONES):
        r = drones[j] - leader_pos
        dist = np.linalg.norm(r)
        if dist < 1e-9 or dist > leader_side_step_range:
            continue
        r_hat = r / dist
        if abs(np.dot(forward, r_hat)) < leader_headon_cos:
            continue

        traffic_side = -perp90(forward)
        lat = perp90(r_hat)
        if np.dot(lat, traffic_side) < 0:
            lat = -lat

        w = (leader_side_step_range - dist) / (leader_side_step_range + 1e-9)
        v += lat * np.clip(w, 0.0, 1.0) * leader_side_step_gain

    return clamp_norm(v, leader_side_step_cap)

# =====================
# Input handling
# =====================


def on_key_press(event):
    global formation_mode, cbba, time_since_key_event
    key = normalize_key(event.key)
    if DEBUG_KEYS:
        print("PRESS:", event.key, "->", key)

    if key in ('up', 'down', 'left', 'right'):
        pressed.add(key)
        time_since_key_event = 0.0
        return

    if key == '1':
        formation_mode = "line"
    elif key == 'v':
        formation_mode = "v"
    elif key == 'r':
        drones[:] = reset_random_positions()
        cbba = CBBA(N_DRONES, N_DRONES)


def on_key_release(event):
    global time_since_key_event
    key = normalize_key(event.key)
    if DEBUG_KEYS:
        print("RELEASE:", event.key, "->", key)

    if key in pressed:
        pressed.remove(key)
    time_since_key_event = 0.0


def on_leave(_event):
    pressed.clear()


def on_close(_event):
    pressed.clear()

# =====================
# Leader command update (POSITION command)
# =====================


def update_leader_command():
    global leader_cmd_pos, last_cmd_dir, leader_cmd_heading

    d = np.zeros(2, dtype=float)
    if 'left' in pressed:
        d[0] -= 1.0
    if 'right' in pressed:
        d[0] += 1.0
    if 'up' in pressed:
        d[1] += 1.0
    if 'down' in pressed:
        d[1] -= 1.0

    if np.linalg.norm(d) > 1e-9:
        d = d / np.linalg.norm(d)
        leader_cmd_pos[:] = leader_cmd_pos + d * leader_cmd_speed * dt
        last_cmd_dir[:] = d
        leader_cmd_heading = heading_from_dir(last_cmd_dir, fallback_theta=leader_cmd_heading)
    else:
        leader_cmd_heading = heading_from_dir(last_cmd_dir, fallback_theta=leader_cmd_heading)

# =====================
# Leader actual update
# =====================


def update_leader_actual():
    global leader_pos, leader_vel

    min_d = 1e9
    for j in range(N_DRONES):
        d = np.linalg.norm(drones[j] - leader_pos)
        if d < min_d:
            min_d = d

    if min_d < leader_yield_radius:
        s = np.clip(min_d / leader_yield_radius, leader_return_scale_min, 1.0)
    else:
        s = 1.0

    v_return = (leader_return_gain * s) * (leader_cmd_pos - leader_pos)
    v_return = clamp_norm(v_return, leader_return_cap)

    v_rep = repulsion_velocity_leader()
    v_side = side_step_velocity_leader()

    leader_vel[:] = v_return + v_rep + v_side
    leader_pos[:] = leader_pos + leader_vel * dt


# =====================
# Visualization
# =====================
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-12, 12)
ax.set_ylim(-12, 12)

drone_scatter = ax.scatter(drones[:, 0], drones[:, 1], s=120)
slot_scatter = ax.scatter([], [], s=90, alpha=0.5)

leader_point, = ax.plot([leader_pos[0]], [leader_pos[1]], marker='*', markersize=18, linestyle='None')
leader_cmd_marker, = ax.plot([leader_cmd_pos[0]], [leader_cmd_pos[1]], marker='x',
                             markersize=10, linestyle='None', alpha=0.6)

texts = [ax.text(0, 0, "") for _ in range(N_DRONES)]

drone_circles = [plt.Circle((0, 0), d_hard_dd, fill=False, alpha=0.18) for _ in range(N_DRONES)]
for c in drone_circles:
    ax.add_patch(c)

leader_circle = plt.Circle((leader_pos[0], leader_pos[1]), d_hard_dl, fill=False, alpha=0.14, linestyle='--')
ax.add_patch(leader_circle)

fig.canvas.mpl_connect('key_press_event', on_key_press)
fig.canvas.mpl_connect('key_release_event', on_key_release)
fig.canvas.mpl_connect('figure_leave_event', on_leave)
fig.canvas.mpl_connect('close_event', on_close)


def update(_):
    global time_since_cbba, cbba, assignment, last_vel, slot_targets, time_since_key_event

    # fail-safe: stop cmd drift if key-release missed
    time_since_key_event += dt
    if (not pressed) and (time_since_key_event > idle_clear_sec):
        pressed.clear()

    update_leader_command()
    update_leader_actual()

    leader_heading = leader_cmd_heading
    slot_positions = formation_slots_world(leader_pos, leader_heading, formation_mode, spacing, N_DRONES)

    # CBBA
    time_since_cbba += dt
    if time_since_cbba >= cbba_replan_period:
        cbba = CBBA(N_DRONES, N_DRONES)
        for _ in range(cbba_iters_each_time):
            for i in range(N_DRONES):
                cbba.bundle_build(i, drones[i], slot_positions)
            cbba.consensus()
        assignment = cbba.get_assignment()
        time_since_cbba = 0.0

    new_last_vel = np.zeros_like(last_vel)

    for i in range(N_DRONES):
        k = assignment[i]
        if k == -1:
            k = int(np.argmin(np.linalg.norm(slot_positions - drones[i], axis=1)))

        target = slot_positions[k]
        slot_targets[i] = target

        v_goal = kp * (target - drones[i])
        v_rep = repulsion_velocity_drone(i)
        v_side = side_step_velocity_drone(i)

        v = clamp_norm(v_goal + v_rep + v_side, vmax)
        drones[i] += v * dt
        new_last_vel[i] = v

    last_vel = new_last_vel

    enforce_hard_separation(drones, leader_pos, d_hard_dd, d_hard_dl, iters=10, eps=1e-6)

    drone_scatter.set_offsets(drones)
    slot_scatter.set_offsets(slot_positions)

    leader_point.set_data([leader_pos[0]], [leader_pos[1]])
    leader_cmd_marker.set_data([leader_cmd_pos[0]], [leader_cmd_pos[1]])
    leader_circle.center = (leader_pos[0], leader_pos[1])

    for i in range(N_DRONES):
        drone_circles[i].center = (drones[i, 0], drones[i, 1])
        texts[i].set_position((drones[i, 0] + 0.25, drones[i, 1] + 0.25))
        texts[i].set_text(f"UAV{i} -> S{assignment[i]}" if assignment[i] != -1 else f"UAV{i} -> (no slot)")

    ax.set_title(
        "Controls: WASD (default Matplotlib shortcuts disabled)\n"
        "1=LINE | v=V | r=RESET | ★ leader avoids/returns | x = desired cmd pos"
    )

    return (drone_scatter, slot_scatter, leader_point, leader_cmd_marker, leader_circle, *texts, *drone_circles)


ani = FuncAnimation(fig, update, interval=int(dt * 1000), blit=True)
plt.show()
