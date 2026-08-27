# Kiến thức bảng DH và động học cánh tay 5 bậc

Tài liệu này tổng hợp lại toàn bộ kiến thức bảng DH đã dùng trong dự án, bám sát code thật đang chạy là `my_robot_ve_doan_thang.py`.

Mọi ma trận và con số trong tài liệu đều là **kết quả chạy thật**, không phải viết tay ước lượng.

## Mục lục

1. [Bảng DH là gì](#1-bảng-dh-là-gì)
2. [Từ bảng DH ra ma trận A, rồi ra T](#2-từ-bảng-dh-ra-ma-trận-a-rồi-ra-t)
3. [Standard DH vs Modified DH](#3-standard-dh-vs-modified-dh)
4. [Cái gì cố định, cái gì đổi giữa 2 quy ước](#4-cái-gì-cố-định-cái-gì-đổi-giữa-2-quy-ước)
5. [Ví dụ trực quan cả cánh tay 8 hàng](#5-ví-dụ-trực-quan-cả-cánh-tay-8-hàng)
6. [Quy tắc chuyển đổi giữa 2 bảng](#6-quy-tắc-chuyển-đổi-giữa-2-bảng)
7. [Động học ngược](#7-động-học-ngược)
8. [Giải thích từng hàm trong code](#8-giải-thích-từng-hàm-trong-code)
9. [Phần chạy chính và tra cứu nhanh](#9-phần-chạy-chính-và-tra-cứu-nhanh)

---

## 1. Bảng DH là gì

Bảng DH là cách mô tả một cánh tay robot bằng **4 con số cho mỗi khâu**. Mỗi hàng của bảng mô tả phép biến đổi từ hệ trục của khâu trước sang hệ trục của khâu sau.

| Tham số | Tên gọi | Trả lời câu hỏi | Trục liên quan |
|---|---|---|---|
| `theta` | góc khớp | xoay quanh z bao nhiêu? | z |
| `d` | độ lệch khâu | trượt dọc z bao nhiêu? | z |
| `a` | chiều dài khâu | trượt dọc x bao nhiêu? | x |
| `alpha` | góc xoắn khâu | xoay quanh x bao nhiêu? | x |

### Chia làm 2 nhóm

Cách chia này là **chìa khoá** để hiểu toàn bộ phần sau của tài liệu:

- **Nhóm Z**: `Rz(theta)` và `Tz(d)` — đều tác động lên trục z.
- **Nhóm X**: `Rx(alpha)` và `Tx(a)` — đều tác động lên trục x.

Với khớp xoay thì `theta` là **biến** (thay đổi khi robot cử động), 3 tham số còn lại là **hằng số** (kích thước cơ khí, đúc sẵn không đổi).

Đây cũng là lý do vector `q` trong code chỉ có 5 phần tử cho 5 khớp: mỗi `q[i]` chỉ thay vào đúng vị trí `theta` của hàng khớp tương ứng.

### 4 ma trận cơ bản

```
Rz(theta) = [ ct  -st   0   0 ]      Tz(d) = [ 1   0   0   0 ]
            [ st   ct   0   0 ]              [ 0   1   0   0 ]
            [ 0    0    1   0 ]              [ 0   0   1   d ]
            [ 0    0    0   1 ]              [ 0   0   0   1 ]

Rx(alpha) = [ 1    0    0   0 ]      Tx(a) = [ 1   0   0   a ]
            [ 0    ca  -sa  0 ]              [ 0   1   0   0 ]
            [ 0    sa   ca  0 ]              [ 0   0   1   0 ]
            [ 0    0    0   1 ]              [ 0   0   0   1 ]

(ct = cos(theta), st = sin(theta), ca = cos(alpha), sa = sin(alpha))
```

Chú ý: `Rz` giữ nguyên hàng/cột thứ 3 (`[0,0,1]`) vì xoay quanh trục z thì trục z không đổi. Tương tự `Rx` giữ nguyên hàng/cột thứ 1.

### Thứ tự cột KHÔNG phải thứ tự nhân

Đây là chỗ nhầm phổ biến nhất. Bảng DH chỉ là **kho chứa 4 con số**. Xếp cột theo `alpha, a, d, theta` hay `a, alpha, d, theta` đều được — ma trận ra y hệt nhau.

Còn thứ tự **nhân ma trận** thì cố định theo quy ước, và chính nó phân biệt Standard DH với Modified DH (xem Chương 3).

---

## 2. Từ bảng DH ra ma trận A, rồi ra T

### Cấu trúc ma trận 4x4

Mọi ma trận biến đổi trong tài liệu này đều là 4x4, đọc như sau:

```
[ R11 R12 R13  x ]     3 cot dau  = huong 3 truc X, Y, Z
[ R21 R22 R23  y ]     cot cuoi   = vi tri (x, y, z)
[ R31 R32 R33  z ]     hang cuoi  = [0 0 0 1] (luon co dinh)
[  0   0   0   1 ]
```

Trong code, `T[:3, 3]` lấy cột cuối (vị trí), `T[:3, 0]` lấy cột 0 (trục X), `T[:3, 1]` lấy trục Y, `T[:3, 2]` lấy trục Z.

### Mỗi hàng DH sinh ra một ma trận A

Điểm quan trọng: **`A` là ma trận 4x4**, không phải một hàng số. Bảng 8 hàng thì sinh ra **8 ma trận A**, mỗi ma trận đều 4x4.

Với quy ước Modified DH (dùng trong dự án này):

```
A_i = Rx(alpha_i) . Tx(a_i) . Rz(theta_i) . Tz(d_i)
```

### Nhân dồn ra T

```
T = A1 · A2 · A3 · A4 · A5 · A6 · A7 · A8
```

Nhân **từ trái sang phải**, thứ tự **không đổi được** (nhân ma trận không giao hoán). Ma trận `T` này chính là **phương trình động học thuận**: thay `q` vào, ra vị trí và hướng của đầu gripper.

### Số liệu minh hoạ

Với tư thế `q = [0.3, -0.5, 0.8, 0.2, -0.4]` (radian):

```
A1 (hang 1 - KHOP 1) =
[[ 0.955 -0.296  0.     0.   ]
 [ 0.296  0.955 -0.    -0.   ]
 [ 0.     0.     1.     0.2  ]
 [ 0.     0.     0.     1.   ]]

A3 (hang 3 - KHOP 2) =
[[ 0.479 -0.878  0.     0.   ]
 [ 0.     0.    -1.    -0.   ]
 [ 0.878  0.479  0.     0.   ]
 [ 0.     0.     0.     1.   ]]

T = A1.A2.A3.A4.A5.A6.A7.A8 =
[[ 0.887  0.054 -0.458  0.111]
 [-0.133  0.981 -0.142  0.034]
 [ 0.442  0.187  0.878  1.038]
 [ 0.     0.     0.     1.   ]]

T == robot.fkine(q) ? True
```

Đọc ma trận `T` cuối cùng: cột cuối `[0.111, 0.034, 1.038]` là **vị trí đầu gripper**, còn khối 3x3 là **hướng** của nó.

Dòng `True` ở cuối chứng minh: công thức nhân tay 8 ma trận A cho ra kết quả **trùng khớp tuyệt đối** với hàm `fkine()` của thư viện roboticstoolbox. Tức cách hiểu ở trên là đúng.

---

## 3. Standard DH vs Modified DH

### Hai công thức đặt cạnh nhau

Khác biệt **duy nhất** là vị trí của nhóm X:

```
Standard DH:  A = Rz(theta) . Tz(d) . Rx(alpha) . Tx(a)     <- nhom X o CUOI
Modified DH:  A = Rx(alpha) . Tx(a) . Rz(theta) . Tz(d)     <- nhom X o DAU
```

Modified DH còn được gọi là **quy ước Craig**. Đây là quy ước mà web **DHViz** sử dụng, và cũng là quy ước dự án này đang dùng.

### Dạng khai triển chữ

```
A_standard =
[ ct   -st*ca    st*sa   a*ct ]
[ st    ct*ca   -ct*sa   a*st ]
[ 0     sa       ca      d    ]
[ 0     0        0       1    ]

A_modified =
[ ct       -st      0    a     ]
[ st*ca     ct*ca  -sa  -sa*d  ]
[ st*sa     ct*sa   ca   ca*d  ]
[ 0         0       0    1     ]
```

### Mẹo nhận diện nhanh — nhìn vào cột cuối

Cột cuối là cột vị trí, và nó tố cáo ngay quy ước nào:

| | `a` nằm đâu | `d` nằm đâu |
|---|---|---|
| **Standard DH** | dính `ct`/`st` (bị `Rz` xoay) | đứng trơ một mình |
| **Modified DH** | đứng trơ một mình | dính `sa`/`ca` (bị `Rx` xoay) |

Lý do: trong Standard DH, `Rz(theta)` nhân **trước** `Tx(a)` nên phép tịnh tiến `a` bị xoay theo; còn `Tz(d)` đứng ngay sau `Rz` nên trục z chưa bị `Rx` đụng tới. Modified DH thì ngược lại hoàn toàn.

### Ví dụ số — cùng 4 tham số, hai kết quả khác nhau

Lấy `alpha = 90°`, `a = 0.2`, `d = 0.3`, `theta = 30°`:

```
A_standard = Rz.Tz.Rx.Tx =
[[ 0.866  -0.      0.5     0.1732]
 [ 0.5     0.     -0.866   0.1   ]
 [ 0.      1.      0.      0.3   ]
 [ 0.      0.      0.      1.    ]]

A_modified = Rx.Tx.Rz.Tz =
[[ 0.866 -0.5    0.     0.2  ]
 [ 0.     0.    -1.    -0.3  ]
 [ 0.5    0.866  0.     0.   ]
 [ 0.     0.     0.     1.   ]]

Bang nhau ? False
```

Đối chiếu với mẹo nhận diện ở trên:

- Bản Standard: cột cuối là `[0.1732, 0.1, 0.3]`. Số `0.1732 = 0.2 × cos(30°)` và `0.1 = 0.2 × sin(30°)` — đúng là `a` bị `theta` xoay. Còn `0.3` chính là `d` đứng trơ.
- Bản Modified: cột cuối là `[0.2, -0.3, 0]`. Số `0.2` chính là `a` đứng trơ. Còn `-0.3 = -0.3 × sin(90°)` và `0 = 0.3 × cos(90°)` — đúng là `d` bị `alpha` xoay.

**Kết luận quan trọng:** cùng 4 con số đầu vào nhưng ra hai ma trận **hoàn toàn khác nhau**. Nên một bảng DH luôn phải đi kèm thông tin nó thuộc quy ước nào — thiếu thông tin đó thì bảng vô nghĩa.

### Hệ quả hình học quan trọng nhất

Vì nhóm X nằm ở hai đầu khác nhau của phép nhân, gốc hệ trục do mỗi hàng tạo ra rơi vào hai chỗ khác nhau:

- **Modified DH**: gốc frame nằm ở **đầu** khâu → **đúng ngay tâm khớp** đang xét.
- **Standard DH**: gốc frame nằm ở **cuối** khâu → rơi vào **tâm khớp kế tiếp**.

Chương 5 sẽ chứng minh điều này bằng số cụ thể.

---

## 4. Cái gì cố định, cái gì đổi giữa 2 quy ước

### Bảng giao hoán

Muốn biết cái gì đổi chỗ được, cái gì không, phải kiểm tra tính giao hoán của từng cặp. Kết quả chạy thật:

| Cặp | Đổi chỗ được? | Lý do |
|---|---|---|
| `Rz · Tz` ↔ `Tz · Rz` | ✅ True | cùng tác động trục z |
| `Rx · Tx` ↔ `Tx · Rx` | ✅ True | cùng tác động trục x |
| `Tz · Tx` ↔ `Tx · Tz` | ✅ True | hai phép tịnh tiến thuần |
| `Rz · Rx` ↔ `Rx · Rz` | ❌ False | hai phép xoay khác trục |
| `Rz · Tx` ↔ `Tx · Rz` | ❌ False | xoay z làm đổi hướng trục x |
| `Rx · Tz` ↔ `Tz · Rx` | ❌ False | xoay x làm đổi hướng trục z |

Output gốc:

```
Rz.Tz == Tz.Rz : True
Rx.Tx == Tx.Rx : True
Tz.Tx == Tx.Tz : True
Rz.Rx == Rx.Rz : False
Rz.Tx == Tx.Rz : False
Rx.Tz == Tz.Rx : False
```

### Cái gì CỐ ĐỊNH giữa 2 quy ước

1. **Bản thân 4 ma trận con** `Rz(theta)`, `Tz(d)`, `Rx(alpha)`, `Tx(a)` — định nghĩa **y hệt nhau**, không đổi một chữ. Nhìn vào 4 hàm con trong `joint_frames()` sẽ thấy chúng giống hệt phiên bản Standard DH cũ.

2. **Thứ tự nội bộ trong nhóm Z**: `Rz` với `Tz` đổi chỗ thoải mái. Viết `Rz(theta).Tz(d)` hay `Tz(d).Rz(theta)` đều ra cùng một ma trận.

3. **Thứ tự nội bộ trong nhóm X**: `Rx` với `Tx` cũng đổi chỗ thoải mái.

4. **Cách nhân dồn ra `T`**: luôn là `T = A1 · A2 · ... · An`, trái sang phải, ở cả hai quy ước.

5. **Ma trận `T` cuối cùng của cùng một cánh tay**: **luôn bằng nhau** ở cả hai quy ước (chứng minh ở Chương 5). Đây là điều hợp lý — cánh tay vật lý chỉ có một, không thể có hai vị trí đầu gripper khác nhau.

### Cái gì THAY ĐỔI

Chỉ đúng **một** thứ:

> **Vị trí tương đối giữa nhóm Z và nhóm X trong một hàng.**

Standard đặt nhóm X **sau**, Modified đặt nhóm X **trước**. Vì bảng trên cho thấy `Rz·Rx ≠ Rx·Rz` và `Rz·Tx ≠ Tx·Rz`, việc hoán đổi hai nhóm này chắc chắn ra kết quả khác.

Từ thay đổi duy nhất đó kéo theo 2 hệ quả:

- **Các frame trung gian nằm ở chỗ khác nhau** (Chương 5).
- **Giá trị 4 tham số trên mỗi hàng phải xếp lại** khi chuyển bảng (Chương 6).

---

## 5. Ví dụ trực quan cả cánh tay 8 hàng

### Hai bảng DH của cùng một cánh tay

Cả hai bảng dưới đây mô tả **chính xác cùng một cánh tay 5 bậc** trong dự án (tại tư thế home `q = 0`, góc tính bằng độ):

```
MODIFIED DH (dung trong code, khop DHViz)      STANDARD DH (doi chieu)
Hang | alpha |  a  |  d  | theta               Hang | alpha |  a  |  d  | theta
  1  |   0   |  0  | 0.2 |   0                   1  |   0   |  0  | 0.2 |   0
  2  |   0   |  0  | 0.2 |   0                   2  |  90   |  0  | 0.2 |   0
  3  |  90   |  0  |  0  |  90                   3  |   0   | 0.2 |  0  |  90
  4  |   0   | 0.2 |  0  |   0                   4  |   0   | 0.2 |  0  |   0
  5  |   0   | 0.2 |  0  | -90                   5  |   0   | 0.2 |  0  | -90
  6  |   0   | 0.2 |  0  |   0                   6  | -90   |  0  |  0  |   0
  7  | -90   |  0  |  0  |   0                   7  |   0   |  0  |  0  |   0
  8  |   0   |  0  | 0.2 |   0                   8  |   0   |  0  | 0.2 |   0
```

So sánh hai bảng:

- Cột `d` và `theta`: **giống nhau từng hàng một**, không đổi gì.
- Cặp `alpha` / `a`: bị **lệch đúng một hàng**. Ví dụ `alpha = 90` ở hàng 3 bên MDH thì nhảy lên hàng 2 bên SDH; `alpha = -90` ở hàng 7 bên MDH thì nhảy lên hàng 6 bên SDH.

### Vị trí frame sau từng hàng — minh hoạ đắt giá nhất

Chạy cả hai bảng, in ra vị trí gốc hệ trục sau mỗi hàng:

```
Hang |   MDH frame        |   SDH frame
  1  | [0.  0.  0.2]      | [0.  0.  0.2]
  2  | [0.  0.  0.4]      | [0.  0.  0.4]
  3  | [0.  0.  0.4]      | [0.  0.  0.6]
  4  | [0.  0.  0.6]      | [0.  0.  0.8]
  5  | [0.  0.  0.8]      | [0.2 0.  0.8]
  6  | [0.2 0.  0.8]      | [0.2 0.  0.8]
  7  | [0.2 0.  0.8]      | [0.2 0.  0.8]
  8  | [0.2 0.  1. ]      | [0.2 0.  1. ]

T cuoi giong nhau ? True -> gripper: [0.2 0.  1. ]
```

Phân tích từng đoạn:

- **Hàng 1, 2**: hai bên trùng nhau, chưa thấy khác biệt.
- **Hàng 3 → 5**: **SDH luôn đi trước MDH đúng một khớp**. Hàng 3, MDH còn ở `0.4` thì SDH đã nhảy tới `0.6`. Hàng 4, MDH ở `0.6` thì SDH đã ở `0.8`. Hàng 5, MDH ở `0.8` thì SDH đã vòng sang `[0.2, 0, 0.8]`.
- **Hàng 8**: hai bên gặp lại nhau ở `[0.2, 0, 1.0]`.

### Kết luận

Dòng `T cuoi giong nhau ? True` xác nhận: hai quy ước mô tả **cùng một cánh tay vật lý**, chỉ khác cách đặt hệ trục trung gian. **Không có quy ước nào "đúng hơn"** quy ước nào — chọn cái nào là tuỳ thói quen, tuỳ thầy dạy, tuỳ phần mềm dùng.

### Hệ quả thực tế khi lập trình

Đây chính là **bug đã gặp trong dự án này**, nên phải nhớ kỹ:

- Với **MDH**: muốn lấy frame tại tâm khớp thì lấy thẳng `T` sau mỗi hàng. Đơn giản, một dòng.

- Với **SDH**: làm y hệt sẽ **vẽ lệch một khớp** — cụ thể là khớp cuối bị vẽ trùng chỗ, còn một khớp ở giữa bị bỏ trống hoàn toàn (chính là hiện tượng "khớp 2 không có trục X, Y" đã gặp). Phải tách **frame trung gian**: dừng lại sau `Rz·Tz`, **trước** `Rx·Tx`:

```python
# Ban Standard DH cu - phai tach lam 2 nua
M = T @ Rz(theta) @ Tz(d)          # frame trung gian - tai tam khop
if la_khop:
    frames.append(M.copy())
T = M @ Rx(alpha) @ Tx(a)          # hoan tat hang, sang khau ke tiep
```

```python
# Ban Modified DH hien tai - gon hon han, khong can bien trung gian
T = T @ Rx(alpha) @ Tx(a) @ Rz(theta) @ Tz(d)   # 1 hang MDH day du
if la_khop:
    frames.append(T.copy())        # T da nam dung tam khop roi
```

---

## 6. Quy tắc chuyển đổi giữa 2 bảng

### Quy tắc hai chiều

```
Standard -> Modified:
    Modified hang i . (d, theta)  =  Standard hang i . (d, theta)        <- giu nguyen
    Modified hang i . (alpha, a)  =  Standard hang (i-1) . (alpha, a)    <- lay cua hang TRUOC
    Modified hang 1 . (alpha, a)  =  (0, 0)                              <- phia truoc khong con hang nao

Modified -> Standard:
    Standard hang i . (d, theta)  =  Modified hang i . (d, theta)        <- giu nguyen
    Standard hang i . (alpha, a)  =  Modified hang (i+1) . (alpha, a)    <- lay cua hang SAU
```

Tóm gọn: **cặp `(alpha, a)` trượt một hàng, cặp `(d, theta)` đứng yên.**

### Lý do trực giác

MDH đặt nhóm X ở **đầu** hàng, SDH đặt ở **cuối** hàng. Cùng một phép xoay/tịnh tiến vật lý nhưng bị "gán" vào hai hàng khác nhau tuỳ quy ước — nên cặp `(alpha, a)` lệch đúng một hàng.

Còn cặp `(d, theta)` thì luôn gắn chặt với bản thân khớp (góc quay và độ trượt của chính khớp đó), nên không đi đâu cả.

### Bằng chứng số

Chạy cả hai bảng trên **500 tư thế ngẫu nhiên**:

```
Sai lech FK qua 500 tu the ngau nhien: 0.00e+00
```

Sai lệch bằng **0 tuyệt đối** (không phải "rất nhỏ", mà đúng bằng 0) chứng minh quy tắc chuyển đổi đúng — và đúng ở mọi tư thế, không phải chỉ tình cờ đúng ở tư thế home.

### Lưu ý khi nhập vào DHViz

DHViz dùng **Modified DH**. Nên bảng nhập lên web chính là cột bên trái ở Chương 5:

```
Hang 1:  alpha=0     a=0     d=0.2   theta=0      -> KHOP 1 (base)
Hang 2:  alpha=0     a=0     d=0.2   theta=0      -> co dinh (tang)
Hang 3:  alpha=90    a=0     d=0     theta=90     -> KHOP 2 (vai)
Hang 4:  alpha=0     a=0.2   d=0     theta=0      -> KHOP 3 (khuyu)
Hang 5:  alpha=0     a=0.2   d=0     theta=-90    -> KHOP 4 (co tay pitch)
Hang 6:  alpha=0     a=0.2   d=0     theta=0      -> co dinh
Hang 7:  alpha=-90   a=0     d=0     theta=0      -> KHOP 5 (roll)
Hang 8:  alpha=0     a=0     d=0.2   theta=0      -> co dinh (gripper)
```

Dấu hiệu nhập đúng: đầu gripper ở độ cao **`z = 1.0`**, tại `[0.2, 0, 1.0]`. Nếu ra `z = 0.8` thì hàng 8 bị thiếu; nếu ra `x = 0.4, z = 0.8` thì số `0.2` của hàng 8 bị gõ nhầm vào cột `a` thay vì cột `d`.

---

## 7. Động học ngược

### Thuận vs ngược

```
Dong hoc THUAN  :  q (5 goc khop)      -->  T (vi tri + huong dau but)    de, chi nhan ma tran
Dong hoc NGUOC  :  T (vi tri mong muon) --> q (5 goc khop)                kho, phai giai lap
```

Thuận thì chỉ cần nhân 8 ma trận là xong, luôn có đúng một kết quả. Ngược thì khó hơn nhiều: có thể **vô nghiệm** (điểm nằm ngoài tầm với), có thể **vô số nghiệm** (nhiều tư thế cùng chạm được một điểm), và không có công thức tổng quát cho mọi cánh tay.

### Cách tính: thuật toán Levenberg–Marquardt

Code dùng `robot.ikine_LM()`. Nguyên lý giải lặp:

1. Đoán một giá trị `q` ban đầu.
2. Tính `fkine(q)` xem đầu bút đang ở đâu.
3. So với `T` mục tiêu, tính ra sai số.
4. Dùng **Jacobian** để biết nên chỉnh `q` theo hướng nào thì sai số giảm nhanh nhất.
5. Lặp lại đến khi sai số đủ nhỏ.

Số vòng lặp thực tế khi vẽ đoạn AB:

```
q tai A (do): [ 153.43 -161.24 -105.71   86.96   -1.54]  iter: 41
q tai B (do): [-153.43 -161.24 -105.71   86.96    0.26]  iter: 5
fkine(qA).t = [ 0.3  -0.15  0.  ]  fkine(qB).t = [0.3  0.15 0.  ]
```

Điểm A mất **41 vòng lặp** vì xuất phát từ `q = 0` (xa mục tiêu). Điểm B chỉ mất **5 vòng lặp** vì được cho điểm xuất phát tốt — chính là nghiệm của điểm trước đó.

Hai dòng `fkine(qA).t` và `fkine(qB).t` là bước kiểm tra ngược: thay nghiệm tìm được vào động học thuận, ra đúng toạ độ A và B ban đầu.

### Xây pose mục tiêu

```python
pose = SE3(p[0], p[1], p[2]) * SE3.Rx(np.pi)
```

- `SE3(x, y, z)` đặt **vị trí** cần tới.
- `SE3.Rx(np.pi)` lật 180° quanh trục x, để **trục z của dụng cụ chúc thẳng xuống** — như cầm bút viết trên bàn.

Ma trận pose mục tiêu tại điểm A:

```
[[ 1.    0.    0.    0.3 ]
 [ 0.   -1.   -0.   -0.15]
 [ 0.    0.   -1.    0.  ]
 [ 0.    0.    0.    1.  ]]
```

Nhìn phần tử ở vị trí `[2][2] = -1`: đó chính là thành phần z của cột trục Z, bằng `-1` nghĩa là **trục Z chỉ thẳng xuống dưới**. Cột cuối `[0.3, -0.15, 0]` là toạ độ điểm A.

### Ý nghĩa `mask=[1,1,1,1,1,0]`

`mask` có 6 phần tử ứng với `[x, y, z, roll, pitch, yaw]`. Số `1` = bắt buộc phải khớp, số `0` = bỏ qua.

Ở đây bỏ qua phần tử cuối — góc xoay quanh **chính trục cây bút**. Lý do: với cây bút thì xoay quanh trục của nó không ảnh hưởng gì tới nét vẽ.

**Ghi chú trung thực:** với riêng bài toán này, `mask=[1,1,1,1,1,1]` (ràng buộc đủ 6) **cũng chạy được `31/31` điểm**, vì hướng "chúc thẳng xuống" nằm trong tầm với của cấu hình 5 bậc này. Dùng 5 ràng buộc là lựa chọn **an toàn tổng quát hơn**, không phải vì 6 ràng buộc chắc chắn thất bại.

Ngược lại, nếu chỉ ràng buộc vị trí (`mask=[1,1,1,0,0,0]`) thì hướng bút thành:

```
mask chi vi tri -> huong but = [ 0.599 -0.3   -0.743]
```

Không còn là `[0, 0, -1]` nữa — bút bị nghiêng lệch, không chúc thẳng xuống.

### Ý nghĩa `q0=q` — nối tiếp nghiệm

Lấy nghiệm của điểm **trước** làm điểm xuất phát cho điểm **sau**. Hai lợi ích:

1. Hội tụ nhanh hơn hẳn: `41` → `5` vòng lặp.
2. Nghiệm liên tục nên cánh tay không bị giật hay lật ngược giữa chừng.

**Ghi chú trung thực về bước nhảy 2π:**

```
Buoc nhay lon nhat: diem 14->15, khop 1 = 6.2499 rad
  q[14]=[ 3.108  1.7    1.955 -0.513  0.137]
  q[15]=[-3.142  1.7    1.955 -0.514  0.137]
  tru di 2pi con lai: 0.0333 rad
```

Nhìn con số `6.2499 rad` (≈ 358°) thì tưởng cánh tay quay một vòng lớn. Nhưng thực ra `3.108` và `-3.142` chỉ cách nhau **`0.033 rad`** về mặt vật lý — đây là hiện tượng **quấn vòng 2π**. Vì `sin` và `cos` tuần hoàn `2π`, góc `3.108` và `-3.142` cho ra **cùng một tư thế**, cánh tay thực tế **không di chuyển gì cả**.

Không ảnh hưởng nét vẽ, bằng chứng là sai lệch toàn tuyến:

```
Sai lech vi tri toan tuyen: 7.19e-08
```

### Động học ngược có khác nhau giữa 2 quy ước DH không?

**KHÔNG.** Đây là điểm quan trọng cần nhớ:

- Động học ngược chỉ làm việc với **ma trận `T`** và **Jacobian**. Nó **không đọc bảng DH**.
- Chương 5 đã chứng minh `T` của hai quy ước **bằng nhau tuyệt đối**.
- Nên cùng một mục tiêu sẽ cho **cùng một nghiệm `q`**, bất kể bảng được viết theo Standard hay Modified.

Quy ước DH chỉ ảnh hưởng tới **cách viết bảng** và **vị trí frame trung gian khi vẽ hình**, hoàn toàn không ảnh hưởng tới bài toán động học ngược.

---

## 8. Giải thích từng hàm trong code

Toàn bộ phần này bám theo file `my_robot_ve_doan_thang.py`.

### 8.1 Khối dựng robot (`e` và `my_robot`)

```python
e  = ET.Rz(jindex=0) * ET.tz(0.2)                           # Hang 1 - KHOP 1
e *= ET.tz(0.2)                                             # Hang 2 - co dinh (tang)
e *= ET.Rx(np.pi / 2) * ET.Rz(jindex=1) * ET.Rz(np.pi / 2)  # Hang 3 - KHOP 2 (offset 90)
e *= ET.tx(0.2) * ET.Rz(jindex=2)                           # Hang 4 - KHOP 3
e *= ET.tx(0.2) * ET.Rz(jindex=3) * ET.Rz(-np.pi / 2)       # Hang 5 - KHOP 4 (offset -90)
e *= ET.tx(0.2)                                             # Hang 6 - co dinh
e *= ET.Rx(-np.pi / 2) * ET.Rz(jindex=4)                    # Hang 7 - KHOP 5
e *= ET.tz(0.2)                                             # Hang 8 - co dinh (gripper)

my_robot = ERobot(e, name='5-DOF_ModifiedDH')
```

**8 dòng `e` ứng 1-1 với 8 hàng bảng DH.** `ET` là viết tắt của Elementary Transform — chuỗi các phép biến đổi cơ bản.

- **`ET.Rz(jindex=0)`** nghĩa là "xoay quanh trục z, góc lấy từ `q[0]`". `jindex` là **chỉ số vào vector `q`**, mỗi khớp phải có một chỉ số **duy nhất** (0, 1, 2, 3, 4 cho 5 khớp). Đây là cách thư viện biết phép xoay nào là biến, phép nào là hằng.
- **Các `ET` không có `jindex`** (như `ET.tz(0.2)`, `ET.Rx(np.pi/2)`, `ET.Rz(np.pi/2)`) là **hằng số cố định**, không phụ thuộc `q`.
- **`ET.Rz(jindex=1) * ET.Rz(np.pi/2)`** ở hàng 3 chính là `theta = q2 + 90°` trong bảng — vì hai phép xoay cùng trục z cộng góc lại được (`Rz(a)·Rz(b) = Rz(a+b)`).
- **Thứ tự trong dòng**: chú ý hàng 3 viết `ET.Rx(...) * ET.Rz(...)` và hàng 4 viết `ET.tx(0.2) * ET.Rz(...)` — nhóm X luôn đứng **trước**. Đó chính là dấu hiệu Modified DH (nối lại Chương 3). Nếu là Standard DH thì phải viết ngược lại, nhóm X ở cuối.

Dấu `*` giữa các `ET` là **phép nhân ma trận**, còn `*=` là viết tắt của `e = e * ...` để nối tiếp vào chuỗi.

### 8.2 `joint_frames(q)`

Hàm này trả về khung toạ độ tại tâm của cả 5 khớp, dùng cho việc vẽ 3 mũi tên X/Y/Z.

**Phần 1 — `rows`:**

```python
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
```

Đây là **bảng DH viết dưới dạng dữ liệu Python**. Về kiểu dữ liệu: `rows` là một **`list`**, mỗi phần tử bên trong là một **`tuple`** 5 phần tử `(alpha, a, d, theta, la_khop)`. Dùng `tuple` vì mỗi hàng DH là dữ liệu cố định, không cần sửa từng phần tử.

Cờ **`la_khop`** ở cột thứ 5 **không thuộc bảng DH** — đó là cờ tự thêm để đánh dấu:
- `True` — hàng 1, 3, 4, 5, 7 là **5 khớp thật**, cần lưu frame để vẽ.
- `False` — hàng 2, 6, 8 chỉ là **hình học cố định** (tầng nối, xoay chuẩn bị hướng, đoạn nối gripper), bỏ qua khi vẽ.

**Phần 2 — 4 hàm con:**

```python
def Rz(t):     # t  <-> theta
def Tz(d):     # d  <-> d
def Rx(alpha): # alpha <-> alpha
def Tx(a):     # a  <-> a
```

Bốn ma trận cơ bản đã trình bày ở Chương 1. Bảng ánh xạ tên tham số trong code ↔ cột trong bảng DH:

| Hàm | Tham số | Cột DH |
|---|---|---|
| `Rz(t)` | `t` | `theta` (viết tắt cho gọn) |
| `Tz(d)` | `d` | `d` |
| `Rx(alpha)` | `alpha` | `alpha` |
| `Tx(a)` | `a` | `a` |

**Phần 3 — vòng lặp:**

```python
frames = []
T = np.eye(4)
for alpha, a, d, theta, la_khop in rows:
    T = T @ Rx(alpha) @ Tx(a) @ Rz(theta) @ Tz(d)   # 1 hang MDH day du
    if la_khop:
        frames.append(T.copy())        # T da nam dung tam khop roi
```

- `T = np.eye(4)` là **gốc thế giới** `(0,0,0)`. Đây là ma trận đơn vị, khởi tạo đúng một lần trước vòng lặp.
- Mỗi vòng lặp nhân thêm **một hàng DH đầy đủ** vào `T`. Sau hàng 1 thì `T` chỉ tới khớp 1, sau hàng 2 thì tới hàng 2, cứ thế.
- `if la_khop` chỉ lưu lại 5 frame ứng với 5 khớp thật.
- `.copy()` để lưu lại **giá trị hiện tại** của `T`, vì `T` còn tiếp tục bị gán lại ở vòng sau.

**Tại sao không cần biến trung gian?** Vì Modified DH cho frame nằm sẵn tại tâm khớp (nối lại Chương 5). Bản Standard DH cũ phải tách làm 2 nửa với biến `M` ở giữa — bản này chỉ cần **một dòng**, gọn hơn hẳn và cũng là lý do bug "khớp 2 mất trục X, Y" không còn khả năng xảy ra.

### 8.3 `draw_frame(ax, T, length=0.12)`

```python
def draw_frame(ax, T, length=0.12):
    origin = T[:3, 3]
    colors = ('#F84752', '#BADA55', '#54AEFF')   # X do, Y xanh la, Z xanh duong
    artists = []
    for axis_i, color in enumerate(colors):
        vec = T[:3, axis_i] * length
        artists.append(
            ax.quiver(*origin, *vec, color=color, linewidth=2.5, zorder=20))
    return artists
```

Vẽ 3 mũi tên biểu diễn 3 trục của một frame.

- **`T[:3, 3]`** — cột cuối = **vị trí gốc** của frame.
- **`T[:3, axis_i]`** — cột thứ `axis_i` = **vector hướng của trục**. Cột 0 = X (đỏ), cột 1 = Y (xanh lá), cột 2 = Z (xanh dương).
- **`* length`** — các cột của ma trận xoay đều có độ dài 1, nhân với `0.12` để mũi tên dài `0.12 m` thay vì `1 m` (quá to so với cánh tay).
- **`ax.quiver(*origin, *vec, ...)`** — lệnh vẽ mũi tên 3D, cần 6 số `(x, y, z, dx, dy, dz)`. Dấu `*` để "bung" tuple 3 phần tử thành 3 tham số riêng lẻ.
- **`zorder=20`** — ưu tiên vẽ đè lên trên thanh link của cánh tay, để mũi tên không bị che khuất.
- **`return artists`** — trả về danh sách 3 mũi tên vừa vẽ, để bước sau còn gọi `.remove()` xoá đi khi hoạt hình chuyển sang tư thế mới. Không giữ tham chiếu thì mũi tên cũ sẽ chồng đống lên nhau qua từng khung hình.

### 8.4 `draw_all_joint_frames(env, q)`

```python
def draw_all_joint_frames(env, q):
    artists = []
    for T in joint_frames(q):
        artists += draw_frame(env.ax, T)
    return artists
```

Gọi `joint_frames(q)` để lấy 5 frame, rồi vẽ từng cái. Toán tử `+=` trên list là **nối danh sách**, nên `artists` cuối cùng là một danh sách **phẳng** gồm 15 mũi tên (5 khớp × 3 trục), xoá một lượt được.

Chú ý biến vòng lặp đặt tên là `T` chỉ vì quy ước ký hiệu chung cho "một ma trận biến đổi", không liên quan gì tới biến `T` cục bộ bên trong `joint_frames`.

`env.ax` là trục vẽ 3D của matplotlib mà roboticstoolbox đang dùng — nhờ nó mà mũi tên vẽ đè được lên đúng hình mô phỏng cánh tay.

### 8.5 `doan_thang_AB(A, huong, do_dai)`

```python
def doan_thang_AB(A=(0.30, -0.15, 0.0), huong=(0.0, 1.0, 0.0), do_dai=0.30):
    A = np.array(A, dtype=float)
    u = np.array(huong, dtype=float)
    u = u / np.linalg.norm(u)              # chuan hoa ve vector don vi
    B = A + u * do_dai
    return np.array([A, B])
```

Trọng tâm là dòng **`u = u / np.linalg.norm(u)`**: chuẩn hoá vector hướng về **độ dài 1** trước khi nhân với `do_dai`.

Nhờ vậy `|AB|` **luôn đúng bằng `do_dai`**, bất kể vector `huong` người dùng đưa vào dài hay ngắn. Nếu bỏ dòng chuẩn hoá này, đưa `huong=(0,2,0)` sẽ ra đoạn dài gấp đôi mong muốn.

`z = 0` để đoạn thẳng nằm đúng trên **mặt phẳng Oxy**.

Kết quả với tham số mặc định: `A = [0.3, -0.15, 0]`, `B = [0.3, 0.15, 0]`, độ dài đo lại đúng `0.3000 m`.

### 8.6 `noi_suy(P, buoc)`

```python
def noi_suy(P, buoc=0.005):
    duong = [P[0]]
    for A, B in zip(P[:-1], P[1:]):
        n = max(int(np.linalg.norm(B - A) / buoc), 1)
        for k in range(1, n + 1):
            duong.append(A + (B - A) * k / n)
    return np.array(duong)
```

Chèn thêm điểm vào giữa các đầu mút để đường vẽ thành dãy điểm dày đặc — cần thiết vì động học ngược phải giải cho **từng điểm một**.

- `zip(P[:-1], P[1:])` ghép từng cặp điểm liền kề `(P0,P1), (P1,P2), ...`
- `n = max(int(norm(B-A)/buoc), 1)` — số đoạn cần chia. Hàm `max(..., 1)` đảm bảo **tối thiểu 1 đoạn**, tránh chia cho 0 khi hai điểm trùng nhau.
- `A + (B-A) * k/n` — nội suy tuyến tính, `k` chạy từ 1 tới `n`.

Với `buoc=0.01` và `|AB|=0.3` thì ra **31 điểm** (1 điểm đầu + 30 điểm chèn).

### 8.7 `ve_doan_thang(ax, P)`

```python
def ve_doan_thang(ax, P, color='#FF8C00', linewidth=3):
    ax.plot(P[:, 0], P[:, 1], P[:, 2],
            color=color, linewidth=linewidth, zorder=15)
    for diem, ten in ((P[0], 'A'), (P[-1], 'B')):
        ax.scatter(*diem, color=color, s=45, zorder=16)
        ax.text(diem[0], diem[1], diem[2] + 0.04, ten,
                color=color, fontsize=13, fontweight='bold', zorder=17)
```

Vẽ **đường tham chiếu** màu cam `#FF8C00`, kèm chấm tròn và nhãn chữ `A`, `B` ở hai đầu (`+0.04` để nhãn nhô lên trên chấm, khỏi bị đè).

Quan trọng: đây **chỉ là hình vẽ tham chiếu** cho biết đường mong muốn nằm ở đâu — **không phải** vệt bút thật. Vệt bút thật là đường xanh dương, lấy từ động học thuận (xem Chương 9).

### 8.8 `giai_dong_hoc_nguoc(duong_ve, q_bat_dau)`

```python
def giai_dong_hoc_nguoc(duong_ve, q_bat_dau=None):
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
```

Hàm này nối lại toàn bộ Chương 7:

- Duyệt qua **từng điểm** trên đường vẽ.
- Dựng `pose` mục tiêu: vị trí điểm đó, bút chúc thẳng xuống.
- Gọi `ikine_LM` với `mask=[1,1,1,1,1,0]` và `q0=q` (nghiệm điểm trước).
- **`raise RuntimeError`** nếu thất bại — thường là do điểm nằm ngoài tầm với. Báo lỗi rõ ràng kèm chỉ số điểm và toạ độ, để dễ tìm chỗ sai.
- `q = sol.q` cập nhật cho vòng sau — chính là cơ chế nối tiếp nghiệm.

Trả về mảng kích thước **`(N, 5)`**: N điểm, mỗi điểm 5 góc khớp.

Tầm với để tham khảo: tính từ khớp 2 ở `(0, 0, 0.4)`, cánh tay với xa nhất được `0.2 × 3 + 0.2 = 0.8 m`. Đoạn AB mặc định cách khớp 2 khoảng `0.52 m` nên nằm gọn trong tầm với.

---

## 9. Phần chạy chính và tra cứu nhanh

### Kịch bản HOME → vẽ AB → HOME

```python
q_home = np.zeros(5)

q_di_xuong = jtraj(q_home,   ds_q[0], 40).q      # HOME -> diem A
q_di_len   = jtraj(ds_q[-1], q_home,  40).q      # diem B -> ve HOME

kich_ban = ([(q, False) for q in q_di_xuong]     # HOME -> A : NHAC but
            + [(q, True)  for q in ds_q]         # A -> B    : HA but
            + [(q, False) for q in q_di_len])    # B -> HOME : NHAC but
```

**`jtraj()`** nối 2 tư thế bằng quỹ đạo **mượt bậc 5** trong không gian khớp: vận tốc và gia tốc bằng 0 ở cả hai đầu, nên cánh tay khởi hành và dừng lại êm, không giật. Nếu nhảy thẳng từ `q_home` sang `ds_q[0]` thì sẽ bị giật một cái rất xấu.

**Cơ chế nhấc bút / hạ bút** — cờ `True`/`False` ở phần tử thứ hai:

```python
if dang_ve:                          # chi ghi vet khi dang HA BUT
    p = my_robot.fkine(q).t
    xs.append(p[0]); ys.append(p[1]); zs.append(p[2])
    vet_but.set_data_3d(xs, ys, zs)
```

Nếu ghi vệt suốt cả 3 giai đoạn thì sẽ có **2 đường thừa** vẽ từ trên không trung xuống, làm hỏng hình. Nên chỉ giai đoạn giữa mới ghi.

Kiểm chứng: vệt bút đúng **31 điểm**, khớp chính xác số điểm của đoạn AB — không dính điểm thừa nào từ 2 đoạn di chuyển.

### Vệt bút lấy từ đâu

Dòng quan trọng nhất:

```python
p = my_robot.fkine(q).t          # vi tri THUC TE cua dau but
```

Vệt xanh dương lấy từ **`fkine(q)`** — tức vị trí **thực tế** mà động học thuận tính ra từ nghiệm động học ngược. **Không phải** vẽ lại đường mong muốn.

Chính vì vậy, việc vệt xanh **trùng khít** đường cam mới là **bằng chứng** động học ngược giải đúng. Nếu vẽ lại đường mong muốn thì nó luôn trùng, chẳng chứng minh được gì.

Kết quả chạy thật:

```
Doan thang: A = [ 0.3  -0.15  0.  ]  B = [0.3  0.15 0.  ]  | do dai = 0.300 m
Dang giai dong hoc nguoc cho 31 diem ...
Da ve xong doan AB va tro ve HOME.
HOME: True
diem: 31 / 31
sai lech: 8.11e-08
do dai: 0.3000
```

### Vòng lặp hoạt hình

```python
for q, dang_ve in kich_ban:
    for a in khung_toa_do:               # xoa khung toa do cua buoc truoc
        a.remove()
    my_robot.q = q                       # dat canh tay vao tu the moi
    env.step(0.02)                       # ve lai canh tay + cho 0.02 giay
    khung_toa_do = draw_all_joint_frames(env, q)
```

Mỗi bước: xoá 15 mũi tên cũ, đặt tư thế mới, vẽ lại, rồi vẽ 15 mũi tên mới. Đây là lý do `draw_frame` phải trả về `artists` (mục 8.3).

### Bảng tra cứu nhanh

| Câu hỏi | Trả lời |
|---|---|
| Nhóm Z gồm gì? | `Rz(theta)`, `Tz(d)` |
| Nhóm X gồm gì? | `Rx(alpha)`, `Tx(a)` |
| Standard DH | `A = Rz·Tz·Rx·Tx` — nhóm X ở cuối |
| Modified DH | `A = Rx·Tx·Rz·Tz` — nhóm X ở đầu |
| Frame nằm đâu (MDH)? | đầu khâu = đúng tâm khớp |
| Frame nằm đâu (SDH)? | cuối khâu = tâm khớp kế tiếp |
| `T` hai quy ước có bằng nhau? | Có, bằng tuyệt đối |
| Chuyển SDH → MDH | `(alpha, a)` tụt xuống 1 hàng, `(d, theta)` giữ nguyên |
| Chuyển MDH → SDH | `(alpha, a)` lấy của hàng sau, `(d, theta)` giữ nguyên |
| IK có phụ thuộc quy ước? | Không |
| Nhận diện MDH qua ma trận | `a` đứng trơ ở cột cuối, `d` dính `sa`/`ca` |
| Nhận diện SDH qua ma trận | `a` dính `ct`/`st`, `d` đứng trơ |
| Cặp nào đổi chỗ được? | `Rz↔Tz`, `Rx↔Tx`, `Tz↔Tx` |
| Cặp nào KHÔNG đổi chỗ được? | `Rz↔Rx`, `Rz↔Tx`, `Rx↔Tz` |
| DHViz dùng quy ước nào? | Modified DH |
| Tầm với tối đa | `0.8 m` tính từ khớp 2 ở `(0, 0, 0.4)` |
| Vị trí gripper khi `q = 0` | `[0.2, 0, 1.0]` |

### Lệnh chạy

```bash
cd /home/ncd/learnspaces/robot/canhtay
venv/bin/python my_robot_ve_doan_thang.py
```

Bỏ dấu `#` ở dòng `env.ax.view_init(elev=90, azim=-90)` để nhìn thẳng từ trên xuống mặt phẳng Oxy.

### Các file trong dự án

| File | Nội dung |
|---|---|
| `my_robot_test.py` | Hiển thị cánh tay tĩnh + 5 khung toạ độ (Modified DH) |
| `my_robot_ve_so_6.py` | Vẽ số 6 kiểu LED 7 đoạn (Modified DH) |
| `my_robot_ve_doan_thang.py` | Vẽ đoạn thẳng AB (Modified DH) — file tham chiếu của tài liệu này |
| `my_robot.py` | Bản cũ dùng `DHRobot` + `RevoluteMDH` |
| `CAI_DAT_WINDOWS.md` | Hướng dẫn cài đặt trên Windows |
