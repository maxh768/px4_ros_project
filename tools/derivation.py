import sympy as sm
from sympy.algebras.quaternion import Quaternion

# --- parameters -----------------------------------------------------------
m, g_use            = sm.symbols('m g_use', positive=True)
Ixx, Iyy, Izz   = sm.symbols('I_xx I_yy I_zz', positive=True)
kT, kM          = sm.symbols('k_T k_M', positive=True)   # thrust, moment constants
kd = sm.symbols('k_d', positive=True)

# rotor geometry -- per-rotor
rx = sm.symbols('r_x0:4', real=True)
ry = sm.symbols('r_y0:4', real=True)
rz = sm.symbols('r_z0:4', real=True)
sg = sm.symbols('sigma0:4', real=True)   # spin direction, +/-1

# --- state ----------------------------------------------------------------
px, py, pz      = sm.symbols('p_x p_y p_z', real=True)        # NED position
vx, vy, vz      = sm.symbols('v_x v_y v_z', real=True)        # NED velocity
q0, q1, q2, q3  = sm.symbols('q_0 q_1 q_2 q_3', real=True)    # body -> NED (quaternion state)
wx, wy, wz      = sm.symbols('omega_x omega_y omega_z', real=True)  # body rates

# --- inputs ---------------------------------------------------------------
T = sm.symbols('T_0:4', nonnegative=True)   # per-rotor thrust, Newtons

# state and input vector
x = sm.Matrix([px, py, pz, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz])
u = sm.Matrix(T)

omega = sm.Matrix([wx, wy, wz])
v = sm.Matrix([vx, vy, vz])
J = sm.Matrix([[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]])

# gravity
g = sm.Matrix([0, 0, g_use])

# quaternion rotation matrix
R = Quaternion(q0, q1, q2, q3).to_rotation_matrix()
# ensure norm
R = sm.simplify(R.subs(q0**2, 1 - q1**2 - q2**2 - q3**2))

assert R.subs({q0: 1, q1: 0, q2: 0, q3: 0}) == sm.eye(3), 'R must be I at the identity quaternion'


# allocation matrix
B = sm.Matrix([[1, 1, 1, 1],
               [-ry[i]    for i in range(4)],
               [ rx[i]    for i in range(4)],
               [ sg[i]*kM for i in range(4)]])
drivers = B*u

# and get T_tot and torques
T_tot = drivers[0]
tau = drivers[1:4, 0]

# for translational dynamics
thrust = sm.Matrix([0, 0, -T_tot])

# H force
P = sm.diag(1, 1, 0) # projection to drop z
v_body = R.T * v # velocity in body frame

f_H = sm.zeros(3,1)
tau_H = sm.zeros(3,1)
for i in range(4):
    r_i = sm.Matrix([rx[i], ry[i], rz[i]])
    w_i = sm.sqrt(T[i] / kT) # rotational speed of rotor
    vperp = P * (v_body + omega.cross(r_i))
    f_i = -kd * w_i * vperp
    f_H += f_i
    tau_H += r_i.cross(f_i)

f_body = thrust + f_H

# assemble system
p_dot = v
v_dot = g + (R*f_body)/m
Q4 = sm.Matrix([[-q1, -q2, -q3],
                [ q0, -q3,  q2],
                [ q3,  q0, -q1],
                [-q2,  q1,  q0]])
q_dot = sm.Rational(1, 2) * Q4 * omega

w_dot = J.inv() * (tau + tau_H - omega.cross(J*omega))

f = sm.Matrix.vstack(p_dot, v_dot, q_dot, w_dot)
assert f.shape == (13,1)



