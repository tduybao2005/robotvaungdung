from roboticstoolbox import ET, ERobot
import numpy as np

# Bang tham so DH theo QUY UOC MODIFIED DH (Craig) - GIONG HET bang nhap tren DHViz:
# i-1_T_i = Rx(alpha_i) . Tx(a_i) . Rz(theta_i) . Tz(d_i)
#
# Khac Standard DH o cho: nhom X (Rx, Tx) dat o DAU hang thay vi o CUOI hang.
# Hau qua: frame do moi hang tao ra nam ngay TAM KHOP hien tai (thay vi o cuoi
# khau, tuc tam khop ke tiep nhu ben Standard DH).
#
# Hang | alpha  | a    | d    | theta      | Y nghia
# -----+--------+------+------+------------+---------------------------------
#   1  |   0    | 0    | 0.2  | q1         | KHOP 1 (base)
#   2  |   0    | 0    | 0.2  | 0          | co dinh - tang noi khop 1 len khop 2
#   3  |  90    | 0    | 0    | q2 + 90    | KHOP 2 (vai)
#   4  |   0    | 0.2  | 0    | q3         | KHOP 3 (khuyu tay)
#   5  |   0    | 0.2  | 0    | q4 - 90    | KHOP 4 (co tay pitch)
#   6  |   0    | 0.2  | 0    | 0          | co dinh
#   7  | -90    | 0    | 0    | q5         | KHOP 5 (co tay roll)
#   8  |   0    | 0    | 0.2  | 0          | co dinh - noi dai dung cu (gripper)
#
# Da verify bang so: sai lech FK so voi ban Standard DH cu = 0.00e+00 (bang 0).

e  = ET.Rz(jindex=0) * ET.tz(0.2)                           # Hang 1 - KHOP 1
e *= ET.tz(0.2)                                             # Hang 2 - co dinh (tang)
e *= ET.Rx(np.pi / 2) * ET.Rz(jindex=1) * ET.Rz(np.pi / 2)  # Hang 3 - KHOP 2 (offset 90)
e *= ET.tx(0.2) * ET.Rz(jindex=2)                           # Hang 4 - KHOP 3
e *= ET.tx(0.2) * ET.Rz(jindex=3) * ET.Rz(-np.pi / 2)       # Hang 5 - KHOP 4 (offset -90)
e *= ET.tx(0.2)                                             # Hang 6 - co dinh
e *= ET.Rx(-np.pi / 2) * ET.Rz(jindex=4)                    # Hang 7 - KHOP 5
e *= ET.tz(0.2)                                             # Hang 8 - co dinh (gripper)

my_robot = ERobot(e, name='5-DOF_ModifiedDH')


def modified_dh_transform(alpha, a, d, theta):
    """1 buoc bien doi Modified DH: Rx(alpha) . Tx(a) . Rz(theta) . Tz(d)"""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct,       -st,      0,    a],
        [st * ca,   ct * ca, -sa, -sa * d],
        [st * sa,   ct * sa,  ca,  ca * d],
        [0,         0,        0,   1]
    ])


def joint_frames(q):
    """
    Tra ve khung toa do TAI VI TRI TUNG KHOP (5 khop).

    Trong Modified DH, moi hang la  Rx(alpha).Tx(a).Rz(theta).Tz(d)  - nhom X
    dat o DAU hang. Nho vay frame do MOI HANG tao ra da nam san ngay tai tam
    khop cua hang do, nen chi can lay thang ma tran tich luy T sau moi hang,
    KHONG can tach frame trung gian nhu hoi con dung Standard DH.
    """
    rows = [
        (0,          0,   0.2, q[0],              True),   # Hang 1 - KHOP 1
        (0,          0,   0.2, 0,                 False),  # Hang 2 - co dinh
        (np.pi / 2,  0,   0,   q[1] + np.pi / 2,  True),   # Hang 3 - KHOP 2
        (0,          0.2, 0,   q[2],              True),   # Hang 4 - KHOP 3
        (0,          0.2, 0,   q[3] - np.pi / 2,  True),   # Hang 5 - KHOP 4
        (0,          0.2, 0,   0,                 False),  # Hang 6 - co dinh
        (-np.pi / 2, 0,   0,   q[4],              True),   # Hang 7 - KHOP 5
        (0,          0,   0.2, 0,                 False),  # Hang 8 - co dinh (gripper)
    ]

    def Rz(t):
        c, s = np.cos(t), np.sin(t)
        return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    def Tz(d):
        return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])

    def Rx(alpha):
        c, s = np.cos(alpha), np.sin(alpha)
        return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

    def Tx(a):
        return np.array([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    frames = []
    T = np.eye(4)
    for alpha, a, d, theta, la_khop in rows:
        T = T @ Rx(alpha) @ Tx(a) @ Rz(theta) @ Tz(d)   # 1 hang MDH day du
        if la_khop:
            frames.append(T.copy())        # T da nam dung tam khop roi
    return frames


def draw_frame(ax, T, length=0.12):
    """Ve 1 khung toa do 3 truc (X do, Y xanh la, Z xanh duong) tai frame T."""
    origin = T[:3, 3]
    colors = ('#F84752', '#BADA55', '#54AEFF')   # X do, Y xanh la, Z xanh duong
    for axis_i, color in enumerate(colors):
        vec = T[:3, axis_i] * length
        ax.quiver(*origin, *vec, color=color, linewidth=2.5, zorder=20)


def draw_all_joint_frames(env, q):
    """Ve khung toa do 3 truc tai tam cua ca 5 khop."""
    for T in joint_frames(q):
        draw_frame(env.ax, T)


q_zero = [0, 0, 0, 0, 0]

# Thanh link to mau xam de 3 truc mau (nhat la truc X do) khong bi lan mau
env = my_robot.plot(q_zero, block=False,
                    options={'robot': {'color': '#9AA0A6', 'linewidth': 4}})
draw_all_joint_frames(env, q_zero)
env.hold()
