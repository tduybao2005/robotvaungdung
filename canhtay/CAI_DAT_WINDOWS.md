# Cài robotics-toolbox-python trên Windows (VSCode + venv)

Mở thư mục dự án trong VSCode → `Ctrl` + `` ` `` để mở terminal → chạy lần lượt:

```powershell
# 0. Xem máy có sẵn Python bản nào
py --list
```

```powershell
# 1. Cài thêm Python 3.12 (numpy 1.26.4 không cài được trên 3.13+)
#    Không cần gỡ 3.13, hai bản chạy song song.
#    Hoặc tải thủ công tại: https://www.python.org/downloads/release/python-31210/
winget install Python.Python.3.12
```

```powershell
# 2. Tạo môi trường ảo bằng đúng Python 3.12
py -3.12 -m venv venv
```

```powershell
# 3. Kích hoạt (thành công thì đầu dòng lệnh hiện "(venv)")
.\venv\Scripts\Activate.ps1
```

```powershell
# 3b. Chỉ chạy nếu bước 3 báo "running scripts is disabled on this system"
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\venv\Scripts\Activate.ps1
```

```powershell
# 4. Nâng cấp pip
python -m pip install --upgrade pip
```

```powershell
# 5. Cài robotics-toolbox-python (~150MB, để chạy tới khi xong)
pip install roboticstoolbox-python
```

```powershell
# 6. Hạ numpy + scipy về đúng bản — PHẢI chạy chung 1 lệnh
pip install "numpy==1.26.4" "scipy==1.13.1"
```

```powershell
# 7. Kiểm tra — đúng thì in ra: 1.26.4 1.13.1 kèm bảng DH của Puma 560
python -c "import numpy, scipy, roboticstoolbox as rtb; print(numpy.__version__, scipy.__version__); print(rtb.models.DH.Puma560())"
```

```
# 8. Trỏ VSCode vào venv:
#    Ctrl + Shift + P  →  Python: Select Interpreter  →  chọn bản trong thư mục venv
```

---

```powershell
# Mỗi lần mở terminal mới đều phải kích hoạt lại
.\venv\Scripts\Activate.ps1
```
