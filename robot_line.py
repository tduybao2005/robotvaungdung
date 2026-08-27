import re
import time
import serial

PORT_NAME = '/dev/ttyACM0'  # Đổi thành '/dev/ttyTHS1' nếu chạy trực tiếp trên Jetson Xavier
BAUD_RATE = 115200

# Board RTrobot có buffer nhận UART giới hạn. Lệnh init trong buoi4_robot dài 258 byte
# nên phần đuôi (chính là các kênh #25-#28 đang dùng) có nguy cơ bị cắt mất.
# Tách mỗi lệnh thành các cụm ngắn hơn ngưỡng này trước khi gửi.
MAX_PACKET_LEN = 60

# 13 lệnh từ buoi4_robot, Group id="0".
# Bước 5-8 (nét sổ xuống bên trái của số 6) đã được nội suy tuyến tính:
# giữ nguyên 2 điểm đầu/cuối, rải đều 2 điểm giữa để nét thẳng, hết lượn sóng.
trajectory_commands = [
    "#1P1500#2P1500#3P1500#4P1500#5P1500#6P1500#7P1500#8P1500#9P1500#10P1500#11P1500#12P1500#13P1500#14P1500#15P1500#16P1500#17P1500#18P1500#19P1500#20P1500#21P1500#22P1500#23P1500#24P1500#25P1500#26P2065#27P1561#28P1561#29P1503#30P1500#31P1500#32P1500T1000D500",
    "#26P1873#27P1197#28P2288T1000D500",
    "#26P1490T1000D500",
    "#25P1647T1000D500",
    "#25P1656#26P1601#27P1076#28P2354T1000D500",
    "#25P1677#26P1681#27P975#28P2399T1000D500",
    "#25P1699#26P1760#27P874#28P2445T1000D500",
    "#25P1720#26P1840#27P773#28P2490T1000D500",
    "#25P1410T1000D500",
    "#25P1475#26P1820#28P2307T1000D500",
    "#25P1492#26P1787#28P2172T1000D500",
    "#25P1542#26P1787T1000D500",
    "#25P1673#28P2175T1000D500"
]

COMMAND_RE = re.compile(r'^((?:#\d+P\d+)+)(T\d+D\d+)?$')


def split_command(cmd, max_len=MAX_PACKET_LEN):
    """Tách 1 lệnh nhiều kênh thành danh sách lệnh ngắn, mỗi lệnh giữ nguyên hậu tố T/D."""
    m = COMMAND_RE.match(cmd)
    if not m:
        return [cmd]
    channels = re.findall(r'#\d+P\d+', m.group(1))
    suffix = m.group(2) or ''

    packets, current = [], ''
    for ch in channels:
        if current and len(current) + len(ch) + len(suffix) > max_len:
            packets.append(current + suffix)
            current = ch
        else:
            current += ch
    if current:
        packets.append(current + suffix)
    return packets


def wait_time(cmd):
    """Thời gian chờ = T + D lấy từ chính lệnh, cộng 50ms dự phòng."""
    m = re.search(r'T(\d+)D(\d+)$', cmd)
    if not m:
        return 1.05
    return (int(m.group(1)) + int(m.group(2))) / 1000.0 + 0.05


# Khởi tạo kết nối UART
try:
    ser = serial.Serial(port=PORT_NAME, baudrate=BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"Đã mở cổng {PORT_NAME} thành công!")
except Exception as e:
    print("Lỗi kết nối:", e)
    exit()

print("Bắt đầu chạy quỹ đạo buoi4_robot...")

for i, cmd in enumerate(trajectory_commands):
    packets = split_command(cmd)
    label = f"[{i + 1}/{len(trajectory_commands)}]"
    if len(packets) > 1:
        print(f"{label} Lệnh dài {len(cmd)} byte -> tách thành {len(packets)} gói")

    for j, packet in enumerate(packets):
        ser.write((packet + "\r\n").encode('ascii'))
        ser.flush()
        suffix = f" (gói {j + 1}/{len(packets)})" if len(packets) > 1 else ""
        print(f"{label}{suffix} Gửi lệnh: {packet}")
        time.sleep(wait_time(packet))

ser.close()
print("Hoàn thành chương trình vẽ!")
