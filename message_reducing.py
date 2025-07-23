from dronekit import connect
import time

# --- 定義訊息監聽器 ---
# 這是用來處理 RAW_IMU 訊息的回調函式
def handle_raw_imu(self, name, msg):
    """
    處理 RAW_IMU 訊息的回調函式。
    """
    print(f"接收到 RAW_IMU 訊息 - XAcc: {msg.xacc}, YAcc: {msg.yacc}, ZAcc: {msg.zacc}")

# --- 連接無人機 ---
print("正在連接無人機...")
try:
    vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True, timeout=60)
    print("無人機已連接！")
except Exception as e:
    print(f"連接無人機失敗: {e}")
    exit()

# --- 1. 添加 RAW_IMU 訊息監聽器 ---
print("\n正在添加 RAW_IMU 訊息監聽器...")
vehicle.add_message_listener('RAW_IMU', handle_raw_imu)

print("在嘗試停止前，監聽 RAW_IMU 訊息 5 秒鐘...")
print("(如果飛控正在發送 RAW_IMU，您應該會看到輸出)")
time.sleep(5) # 觀察一段時間，確認是否有收到 RAW_IMU 訊息

# --- 2. 透過設定 SR1_RAW_SENS 參數來停止 RAW_IMU 訊息 ---
# 因為 TCP 5762 對應到 SERIAL1，所以我們設定 SR1_ 參數
# 將 SR1_RAW_SENS 設為 0，會停止 RAW_IMU 及其他原始感測器數據的傳送
print("\n正在設定 SR1_RAW_SENS 參數為 0 (停止傳送 RAW_IMU 及其他原始感測器訊息)...")
try:
    print(f"R1_RAW_SENS 參數:{vehicle.parameters['SR0_RAW_SENS']}")
    vehicle.parameters['SR0_RAW_SENS'] = 0
    if vehicle.parameters['SR0_RAW_SENS']!=0:
        vehicle.parameters['SR0_RAW_SENS'] = 0
        print("SR1_RAW_SENS 參數設定中")
        time.sleep(1)
   
    print("SR1_RAW_SENS 參數設定命令已發送。")
    time.sleep(2) # 給飛控一點時間處理參數更新
except Exception as e:
    print(f"設定 SR1_RAW_SENS 參數失敗: {e}")

print("等待 10 秒鐘，確認 RAW_IMU 訊息是否已停止...")
# 如果成功，這裡應該不會再看到 "接收到 RAW_IMU 訊息..." 的輸出
time.sleep(20)

# --- (可選) 調整其他 SR1_ 參數以降低整體頻寬 ---
# 例如，將 SR1_EXTRA1 (包含 ATTITUDE, AHRS2 等) 的頻率設為 5Hz
# print("\n正在設定 SR1_EXTRA1 參數為 5 (約 5Hz)...")
# vehicle.parameters['SR1_EXTRA1'] = 5
# time.sleep(2)

# --- 清理與關閉 ---
vehicle.remove_message_listener('RAW_IMU', handle_raw_imu)
print("RAW_IMU 訊息監聽器已移除。")

vehicle.close()
print("無人機連接已關閉。")