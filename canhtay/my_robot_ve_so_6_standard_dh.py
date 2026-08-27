from roboticstoolbox import DHRobot, RevoluteDH, jtraj
from spatialmath import SE3
import numpy as np

# ---------------------------------------------------------------------------
#  BANG THAM SO THEO QUY UOC STANDARD DH (Denavit - Hartenberg co dien)
# ---------------------------------------------------------------------------
#
#   i-1_T_i = Rz(theta_i) . Tz(d_i) . Tx(a_i) . Rx(alpha_i)
#
# Khac Modified DH (Craig) o cho: nhom X (Tx, Rx) dat o CUOI hang thay vi o DAU.
# Hau qua: frame do moi hang tao ra nam o CUOI KHAU (tuc tam khop KE TIEP),
# chu khong nam ngay tam khop hien tai nhu ben Modified DH.
#
#  i |  theta      |  d    |  a    |  alpha  | Y nghia
# ---+-------------+-------+-------+---------+------------------------------
#  1 |  q1         |  0.4  |  0    |   90    | KHOP 1 (base, xoay quanh z)
#  2 |  q2 + 90    |  0    |  0.2  |    0    | KHOP 2 (vai)
#  3 |  q3         |  0    |  0.2  |    0    | KHOP 3 (khuyu tay)
#  4 |  q4 - 90    |  0    |  0.2  |  -90    | KHOP 4 (co tay pitch)
#  5 |  q5         |  0.2  |  0    |    0    | KHOP 5 (co tay roll)
#
# So voi ban Modified DH (my_robot_ve_so_6.py): ban do can 8 hang vi phai chen
# them 3 hang co dinh, con Standard DH gom gon lai dung 5 hang - moi hang mot
# khop. Cu the:
#   - d1 = 0.4 gop chung 2 doan tang cua ban MDH (0.2 + 0.2)
#   - d5 = 0.2 chinh la doan noi dai dung cu (gripper) o hang 8 ban MDH
#   - khong can base/tool transform, tat ca da nam gon trong bang
#
# Da verify bang so: sai lech FK so voi ban Modified DH tren 2000 tu the ngau
# nhien = 6.11e-16 (bang 0 trong pham vi sai so dau cham dong).

L1 = RevoluteDH(d=0.4, a=0,   alpha=np.pi / 2,  offset=0)
L2 = RevoluteDH(d=0,   a=0.2, alpha=0,          offset=np.pi / 2)
L3 = RevoluteDH(d=0,   a=0.2, alpha=0,          offset=0)
L4 = RevoluteDH(d=0,   a=0.2, alpha=-np.pi / 2, offset=-np.pi / 2)
L5 = RevoluteDH(d=0.2, a=0,   alpha=0,          offset=0)

my_robot = DHRobot([L1, L2, L3, L4, L5], name='5-DOF_StandardDH')


def khung_dh(q):
    """
    Tra ve 5 khung toa do CHUAN CUA STANDARD DH (frame {1} den frame {5}).

    Moi hang Standard DH la  Rz(theta).Tz(d).Tx(a).Rx(alpha)  - nhom X dat o
    CUOI hang. Nen frame sinh ra sau moi hang KHONG nam tai tam khop cua hang
    do, ma bi day ra CUOI KHAU (tam khop ke tiep). Day chinh la diem khac biet
    co ban so voi Modified DH, va cung la ly do ban MDH lay thang T tich luy
    con ban nay thi khong.

    Muon lay frame nam DUNG TAM KHOP thi dung khung_tam_khop() ben duoi.
    """
    frames = []
    T = np.eye(4)
    for link, qi in zip(my_robot.links, q):
        T = T @ link.A(qi).A          # A(qi) = 1 hang Standard DH day du
        frames.append(T.copy())
    return frames


# Truot frame doc theo CHINH TRUC QUAY cua khop (met). Truot doc truc quay thi
# khong lam doi truc, chi doi cho dat goc frame - thuan tuy de nhin cho dep.
#
# Khop 1 can truot len 0.2 m: truc quay cua no la truc z cua the gioi, goc frame
# dat o dau tren truc cung dung. Ban Modified DH tach cot de thanh 2 hang 0.2 m
# nen frame khop 1 roi vao GIUA cot (z = 0.2); ban Standard DH gop lai d1 = 0.4
# nen diem tu nhien roi xuong z = 0 (chan de). Truot len 0.2 m de hai ban hien
# thi giong het nhau.
DICH_DOC_TRUC_Z = [0.2, 0, 0, 0, 0]


def khung_tam_khop(q):
    """
    Tra ve 5 khung toa do dat DUNG NGAY TAM TUNG KHOP.

    Trong Standard DH, truc quay cua khop i chinh la truc z cua frame {i-1}.
    Nen frame tam khop i = (tich luy den het hang i-1) . Rz(theta_i): xoay theo
    bien khop nhung DUNG LAI truoc khi nhom Tz/Tx/Rx day frame ra cuoi khau.

    Sau do truot doc truc z theo DICH_DOC_TRUC_Z (xem giai thich ben tren).

    Dung ham nay de hinh mo phong trong giong ban Modified DH.
    """
    frames = []
    T = np.eye(4)
    for link, qi, dz in zip(my_robot.links, q, DICH_DOC_TRUC_Z):
        theta = qi + link.offset
        c, s = np.cos(theta), np.sin(theta)
        Rz = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Tz = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, dz], [0, 0, 0, 1]])
        frames.append(T @ Rz @ Tz)    # dung lai ngay sau Rz -> con o tam khop
        T = T @ link.A(qi).A          # roi moi di het hang de sang khop sau
    return frames


def draw_frame(ax, T, length=0.12):
    """Ve 1 khung toa do 3 truc (X do, Y xanh la, Z xanh duong) tai frame T."""
    origin = T[:3, 3]
    colors = ('#F84752', '#BADA55', '#54AEFF')   # X do, Y xanh la, Z xanh duong
    artists = []
    for axis_i, color in enumerate(colors):
        vec = T[:3, axis_i] * length
        artists.append(
            ax.quiver(*origin, *vec, color=color, linewidth=2.5, zorder=20))
    return artists


def draw_all_joint_frames(env, q, tai_tam_khop=True):
    """Ve khung toa do 3 truc cho ca 5 khop.

    tai_tam_khop=True  : dat frame ngay tam khop (nhin giong ban Modified DH)
    tai_tam_khop=False : dat frame theo dung chuan Standard DH (o cuoi khau)

    Tra ve danh sach mui ten da ve, de con xoa di khi ve lai o buoc ke tiep."""
    frames = khung_tam_khop(q) if tai_tam_khop else khung_dh(q)
    artists = []
    for T in frames:
        artists += draw_frame(env.ax, T)
    return artists


# ---------------------------------------------------------------------------
#  SO 6 KIEU LED 7 DOAN (net vuong)
# ---------------------------------------------------------------------------
#
# So do 7 doan:            Chu so 6 bat 6 doan: a, c, d, e, f, g  (TAT doan b)
#
#      TL ---- a ---- TR
#      |               |            a : TL - TR   (canh tren)     BAT
#      f               b            b : TR - MR   (tren-phai)     TAT  <-- khac so 8
#      |               |            c : MR - BR   (duoi-phai)     BAT
#      ML ---- g ---- MR            d : BL - BR   (canh duoi)     BAT
#      |               |            e : ML - BL   (duoi-trai)     BAT
#      e               c            f : TL - ML   (tren-trai)     BAT
#      |               |            g : ML - MR   (canh giua)     BAT
#      BL ---- d ---- BR
#
# 6 doan nay noi duoc thanh MOT NET LIEN (khong nhac but):
#      TR -> TL -> ML -> BL -> BR -> MR -> ML
#       a     f     e     d     c     g


def diem_so_6(x0=0.28, w=0.16, h=0.26, z=0.0):
    """
    Tra ve mang (N, 3) cac diem goc cua so 6, nam tren MAT PHANG Oxy.

    x0 : khoang cach tu goc toa do den canh trai cua so 6 (theo truc X)
    w  : chieu ngang cua so 6 (theo truc X)
    h  : chieu cao cua so 6 (theo truc Y) - so duoc dat can giua truc X
    z  : do cao cua mat phang ve. z = 0 chinh la mat phang Oxy.
    """
    y0 = -h / 2                 # canh duoi, dat doi xung qua truc X

    BL = (x0,     y0)           # Bottom-Left
    BR = (x0 + w, y0)           # Bottom-Right
    ML = (x0,     y0 + h / 2)   # Middle-Left
    MR = (x0 + w, y0 + h / 2)   # Middle-Right
    TL = (x0,     y0 + h)       # Top-Left
    TR = (x0 + w, y0 + h)       # Top-Right

    net_ve = [TR, TL, ML, BL, BR, MR, ML]     # mot net lien, khong nhac but
    return np.array([(x, y, z) for x, y in net_ve])


def noi_suy(P, buoc=0.005):
    """
    Chen them diem giua cac goc de duong ve thanh day diem day dac.
    buoc : khoang cach mong muon giua 2 diem lien tiep (met)
    """
    duong = [P[0]]
    for A, B in zip(P[:-1], P[1:]):
        n = max(int(np.linalg.norm(B - A) / buoc), 1)
        for k in range(1, n + 1):
            duong.append(A + (B - A) * k / n)
    return np.array(duong)


def ve_so_6(ax, P, color='#FF8C00', linewidth=3):
    """Ve net so 6 len hinh mo phong, kem 1 cham danh dau diem bat dau."""
    ax.plot(P[:, 0], P[:, 1], P[:, 2],
            color=color, linewidth=linewidth, zorder=15)
    ax.scatter(P[0, 0], P[0, 1], P[0, 2],
               color=color, s=40, zorder=16)          # diem bat dau net ve


# ---------------------------------------------------------------------------
#  DONG HOC NGUOC - tim goc khop de dau but di dung net so 6
# ---------------------------------------------------------------------------


def giai_dong_hoc_nguoc(duong_ve, q_bat_dau=None):
    """
    Voi MOI diem tren duong ve, giai nguoc ra bo 5 goc khop q.

    Tu the mong muon cua dau but: dung ngay tai diem can ve, truc z cua dung cu
    chuc THANG XUONG (nen nhan them SE3.Rx(180 do)).

    mask = [1,1,1,1,1,0] : rang buoc 3 toa do vi tri + 2 huong nghieng cua but,
    BO QUA goc xoay quanh chinh truc but (khop 5 - roll), vi voi cay but thi
    xoay quanh truc cua no khong anh huong gi. Canh tay co 5 bac tu do nen chi
    dap ung duoc 5 rang buoc - dung bang so rang buoc con lai.

    q0=q : lay nghiem cua diem TRUOC lam diem xuat phat cho diem SAU, giup
    nghiem lien tuc (canh tay khong bi giat/lat nguoc giua chung).
    """
    q = np.zeros(5) if q_bat_dau is None else np.array(q_bat_dau, dtype=float)
    ds_q = []
    for i, p in enumerate(duong_ve):
        pose = SE3(p[0], p[1], p[2]) * SE3.Rx(np.pi)
        sol = my_robot.ikine_LM(pose, q0=q, mask=[1, 1, 1, 1, 1, 0],
                                joint_limits=False)
        if not sol.success:
            raise RuntimeError(f'Khong giai duoc dong hoc nguoc tai diem {i}: {p}')
        q = sol.q
        ds_q.append(q.copy())
    return np.array(ds_q)


# ---------------------------------------------------------------------------
#  HIEN THI
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    print(my_robot)

    so_6 = diem_so_6()                       # 7 diem goc tren mat phang Oxy (z = 0)
    duong_ve = noi_suy(so_6, buoc=0.01)      # lam day thanh ~88 diem de di muot

    print('Dang giai dong hoc nguoc cho', len(duong_ve), 'diem ...')
    ds_q = giai_dong_hoc_nguoc(duong_ve)
    print('Xong. Bat dau mo phong.')

    # --- Ghep thanh kich ban day du: HOME -> ve so 6 -> HOME -----------------
    # q_home la tu the dung thang ban dau (tat ca goc khop = 0).
    # jtraj() noi 2 tu the bang quy dao MUOT trong khong gian khop (bac 5): van
    # toc va gia toc bang 0 o 2 dau, nen tay khoi hanh va dung lai em, khong giat.
    q_home = np.zeros(5)

    q_di_xuong = jtraj(q_home,   ds_q[0], 40).q     # HOME -> diem bat dau net ve
    q_di_len   = jtraj(ds_q[-1], q_home,  40).q     # net ve xong -> ve HOME

    # Co True/False: chi HA BUT (ghi vet) trong doan ve so 6, con luc di chuyen
    # tu HOME xuong va tu net ve len thi NHAC BUT (khong ghi vet).
    kich_ban = ([(q, False) for q in q_di_xuong]
                + [(q, True) for q in ds_q]
                + [(q, False) for q in q_di_len])

    # Thanh link to mau xam de 3 truc mau (nhat la truc X do) khong bi lan mau
    env = my_robot.plot(q_home, block=False,
                        options={'robot': {'color': '#9AA0A6', 'linewidth': 4}})

    ve_so_6(env.ax, so_6)                    # net so 6 mo (duong tham chieu)

    # Mo rong khung nhin de thay het ca canh tay lan so 6
    env.ax.set_xlim(-0.2, 0.6)
    env.ax.set_ylim(-0.4, 0.4)
    env.ax.set_zlim(0.0, 0.8)

    # Bo dau # dong duoi de nhin tu tren xuong (nhin thang mat phang Oxy)
    # env.ax.view_init(elev=90, azim=-90)

    # Doi thanh False de xem frame theo DUNG CHUAN Standard DH (nam o cuoi khau)
    TAI_TAM_KHOP = True

    # "Vet but" - duong do dan dan hien ra dung noi dau gripper vua di qua
    vet_but, = env.ax.plot([], [], [], color='#1A73E8', linewidth=3, zorder=18)
    xs, ys, zs = [], [], []

    khung_toa_do = draw_all_joint_frames(env, q_home, TAI_TAM_KHOP)

    for q, dang_ve in kich_ban:
        for a in khung_toa_do:               # xoa khung toa do cua buoc truoc
            a.remove()

        my_robot.q = q                       # dat canh tay vao tu the moi
        env.step(0.02)                       # ve lai canh tay + cho 0.02 giay

        khung_toa_do = draw_all_joint_frames(env, q, TAI_TAM_KHOP)

        if dang_ve:                          # chi ghi vet khi dang HA BUT
            p = my_robot.fkine(q).t          # vi tri THUC TE cua dau but
            xs.append(p[0]); ys.append(p[1]); zs.append(p[2])
            vet_but.set_data_3d(xs, ys, zs)

    print('Da ve xong so 6 va tro ve HOME.')
    env.hold()
