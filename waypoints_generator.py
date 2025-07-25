import math
from dronekit import connect, Command, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil  # Import mavutil to use MAV_FRAME and MAV_CMD constants
import time
import os

# --- 1. 定義隊形參數 ---
#       u3
#   u2      u4
# u1            u5
# 設定隊形相關參數
formation_params = {
    "num_drones": 5,
    "lead_drone_id": 3,  # 隊形中心點對應的無人機，通常在V字頂點 (U3)
    "spacing_x": 10,   # 無人機在隊形橫向（左右）的間距 (米)
    "spacing_y": 10,   # 無人機在隊形縱向（前後）的間距 (米)
    "altitude": 30     # 編隊的預設飛行高度 (米)
}

# 定義每台無人機相對於隊形中心點的偏移量 (dx_body, dy_body, dz)
# 這裡的 dx_body 和 dy_body 是在隊形自身的局部座標系中定義的。
# dx_body: 沿著隊形橫向軸的偏移 (向右為正)
# dy_body: 沿著隊形縱向軸的偏移 (向前為正，向後為負)
drone_offsets_body_frame = {
    1: (-2 * formation_params["spacing_x"], -2 * formation_params["spacing_y"], 0), # U1: 最左後
    2: (-1 * formation_params["spacing_x"], -1 * formation_params["spacing_y"], 0), # U2: 左後
    3: (0, 0, 0),                                                                  # U3: 中心 (領頭機)
    4: (1 * formation_params["spacing_x"], -1 * formation_params["spacing_y"], 0),  # U4: 右後
    5: (2 * formation_params["spacing_x"], -2 * formation_params["spacing_y"], 0)   # U5: 最右後
}

# --- 2. 定義虛擬中心航線 (Waypoint Path for the formation center) ---
# 這些是隊形中心點將會經過的航點。
# 格式: (緯度, 經度, 高度)
# virtual_center_waypoints_coords = [
#     (23.000000, 120.000000, formation_params["altitude"]), # 航點 1 (起點)
#     (23.000100, 120.000100, formation_params["altitude"]), # 航點 2
#     (23.000200, 120.000000, formation_params["altitude"]), # 航點 3
#     (23.000100, 120.000200, formation_params["altitude"]), # 航點 4
#     (23.000000, 120.000000, formation_params["altitude"])  # 航點 5 (回到起點)
# ]

# --- 新增：從 .waypoints 檔案讀取航點 ---
def load_waypoints_from_file(filepath):
    """
    從 Mission Planner .waypoints 檔案讀取航點座標。
    檔案格式預期為：QGC WPL 110 等開頭，然後每行包含航點數據。
    我們主要提取緯度、經度和高度。
    """
    waypoints = []
    # 檢查檔案是否存在
    if not os.path.exists(filepath):
        print(f"錯誤：航點檔案 '{filepath}' 不存在。請檢查路徑或確保檔案在程式碼相同目錄下。")
        return []

    try:
        with open(filepath, 'r') as f:
            for i, line in enumerate(f):
                if i == 0: # 跳過檔案頭 (QGC WPL 110)
                    if not line.strip().startswith("QGC WPL 110"):
                        print("警告：這可能不是一個標準的 Mission Planner .waypoints 檔案。")
                    continue # 繼續處理下一行

                parts = line.strip().split('\t') # 假設是 Tab 分隔，如果不是請改為 ' '
                
                # 至少要有 11 個欄位 (索引 0-10) 來獲取 Lat, Lon, Alt
                if len(parts) < 11:
                    print(f"警告：航點檔案行 {i+1} 格式不正確，跳過：{line.strip()}")
                    continue

                try:
                    # 預期格式為: Index Current WP Frame Command Param1 Param2 Param3 Param4 X Y Z Autocontinue
                    # 我們需要 X (緯度), Y (經度), Z (高度)
                    lat = float(parts[8])
                    lon = float(parts[9])
                    alt = float(parts[10])

                    # 排除 Command 22 (MAV_CMD_NAV_TAKEOFF) 的經緯度，因為它通常是 (0,0) 並會被忽略
                    # 但保留其高度作為虛擬航點的目標高度
                    command_id = int(parts[3])
                    if command_id == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                        # 對於 takeoff 命令，我們只關心目標高度，經緯度可能不是實際位置
                        # 或者你可以選擇直接跳過此類命令，如果你的虛擬航線只關注飛行路徑。
                        # 在這裡，我們將使用前一個有效航點的經緯度，或者使用預設中心點，
                        # 但以 takeoff 的目標高度作為該虛擬航點的高度。
                        # 簡單處理：為 takeoff 命令創建一個 "虛擬" 航點，使用前一個航點的經緯度
                        # 但如果它是第一個真實航點，則使用 0,0，並在之後的邏輯中處理
                        if waypoints: # 如果已經有航點了，使用上一個航點的經緯度
                            waypoints.append((waypoints[-1][0], waypoints[-1][1], alt))
                        else: # 如果這是第一個命令，用0,0 (這個情況應該發生在第一個非Home點)
                            waypoints.append((lat, lon, alt)) # 這裡的lat/lon對TAKE_OFF可能無意義
                    else:
                        waypoints.append((lat, lon, alt))

                except (ValueError, IndexError) as e:
                    print(f"解析航點檔案行 {i+1} 時的數據轉換錯誤：{line.strip()} - {e}")
                    continue
    except IOError as e:
        print(f"無法打開或讀取檔案 '{filepath}': {e}")
        return []

    print(f"成功從 '{filepath}' 載入 {len(waypoints)} 個航點。")
    return waypoints

# 設定你的 .waypoints 檔案路徑
# 請確保這個檔案存在於你的程式碼相同目錄下，或者提供完整的路徑
# 檔案名稱要與你 Mission Planner 導出的檔案名稱一致
WAYPOINT_FILE = "2.waypoints"

# 將讀取到的航點存入 virtual_center_waypoints_coords
virtual_center_waypoints_coords = load_waypoints_from_file(WAYPOINT_FILE)

# 如果檔案沒有提供航點，或者載入失敗，則使用預設值
if not virtual_center_waypoints_coords:
    print("由於檔案中沒有航點或載入失敗，將使用預設虛擬航點。")
    virtual_center_waypoints_coords = [
        # 這些預設值應替換為你希望的起點附近座標
        (22.999516, 120.225574, formation_params["altitude"]), # 初始位置 (台南市歸仁區)
        (22.999616, 120.225674, formation_params["altitude"]),
        (22.999716, 120.225574, formation_params["altitude"]),
        (22.999616, 120.225774, formation_params["altitude"]),
        (22.999516, 120.225574, formation_params["altitude"])
    ]

# ... (其餘的 get_bearing, get_location_metres 函數保持不變) ...

# ... (生成每台無人機的實際航線部分保持不變) ...

# ... (連接與上傳航線到多個 SITL 無人機部分保持不變) ...

print("\n--- 航線規劃完成 ---")
print("每個無人機的航線已生成，包含根據飛行方向旋轉的隊形偏移、偏航角，以及航點停留 5 秒的設定。")
print("您可以將這些航點載入到各自的無人機中。")
print("請注意，實際的多機控制需要獨立的連接和更強大的任務管理策略。")
print("請確保您已經運行了足夠數量的 SITL 實例來對應 'num_drones'。")
# --- 輔助函數：計算兩點之間的偏航角 (航向角) ---
def get_bearing(loc1, loc2):
    """
    計算從 loc1 到 loc2 的偏航角 (bearing)。
    Args:
        loc1 (LocationGlobalRelative): 起始位置
        loc2 (LocationGlobalRelative): 結束位置
    Returns:
        float: 偏航角 (0-360 度，北為 0，順時針增加)
    """
    lat1 = math.radians(loc1.lat)
    lon1 = math.radians(loc1.lon)
    lat2 = math.radians(loc2.lat)
    lon2 = math.radians(loc2.lon)

    delta_lon = lon2 - lon1

    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon))

    bearing_rad = math.atan2(x, y)
    bearing_deg = math.degrees(bearing_rad)
    
    # 確保角度在 0 到 360 之間
    bearing_deg = (bearing_deg + 360) % 360
    
    return bearing_deg

# 這個函數將地理座標和旋轉後的相對偏移量結合起來
def get_location_metres(original_location, dNorth, dEast, dDown=0):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth`, `dEast`
    of the `original_location`. The `dDown` is optional and defaults to 0.
    This method is accurate for short distances (up to a few hundred meters).
    """
    earth_radius = 6378137.0 # WGS84 earth radius in metres

    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.radians(original_location.lat)))

    # New position in decimal degrees
    newlat = original_location.lat + math.degrees(dLat)
    newlon = original_location.lon + math.degrees(dLon)
    
    newalt = original_location.alt - dDown 

    return LocationGlobalRelative(newlat, newlon, newalt)

# 儲存所有無人機的航線規劃
all_drone_missions = {i: [] for i in range(1, formation_params["num_drones"] + 1)}

# 將虛擬航點轉換為 LocationGlobalRelative 物件
virtual_waypoints_obj = [LocationGlobalRelative(lat, lon, alt) 
                         for lat, lon, alt in virtual_center_waypoints_coords]

# --- 3. 根據虛擬航線和隊形偏移，生成每台無人機的實際航線 ---
# 為每個航點計算其目標偏航角 (從當前航點指向下一個航點)
waypoint_bearings = []
for i in range(len(virtual_waypoints_obj)):
    current_wp = virtual_waypoints_obj[i]
    # 下一個航點，如果是最後一個，則循環到第一個
    next_wp = virtual_waypoints_obj[(i + 1) % len(virtual_waypoints_obj)] 
    
    if len(virtual_waypoints_obj) == 1:
        waypoint_bearings.append(None) # 自動偏航，如果只有一個航點
    else:
        bearing = get_bearing(current_wp, next_wp)
        waypoint_bearings.append(bearing)

# 遍歷虛擬中心航線的每個航點
for i, virtual_waypoint in enumerate(virtual_waypoints_obj):
    # 獲取這個航點應該設定的偏航角 (隊形的目標方向)
    target_yaw_degrees = waypoint_bearings[i]
    target_yaw_radians = math.radians(target_yaw_degrees) if target_yaw_degrees is not None else None

    # 對於每台無人機
    for drone_id in range(1, formation_params["num_drones"] + 1):
        dx_body, dy_body, dz = drone_offsets_body_frame[drone_id]
        
        # 根據目標偏航角旋轉隊形局部偏移量，得到地球座標系下的偏移量 (dEast, dNorth)
        if target_yaw_radians is not None:
            dEast_rotated = dx_body * math.cos(target_yaw_radians) - dy_body * math.sin(target_yaw_radians)
            dNorth_rotated = dx_body * math.sin(target_yaw_radians) + dy_body * math.cos(target_yaw_radians)
        else:
            # 如果沒有明確的偏航角（例如只有一個航點），則不旋轉，使用原始偏移
            dEast_rotated = dx_body
            dNorth_rotated = dy_body

        # 計算實際航點位置
        actual_waypoint_location = get_location_metres(
            virtual_waypoint, 
            dNorth_rotated,  # dNorth 偏移量
            dEast_rotated,   # dEast 偏移量
            dz               # dDown 偏移量 (通常為 0)
        )
        
        # 創建 MAVLink 導航航點命令
        cmd = Command(
            0,                                     # target_system (can be 0 for default)
            0,                                     # target_component (can be 0 for default)
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame - This is the correct position
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # command
            0,                                     # current - Set to 0 for subsequent waypoints
            1,                                     # autocontinue - Set to 1 to proceed to the next waypoint
            5,                                     # param1: Hold time (seconds)
            0,                                     # param2: Acceptance radius (meters)
            0,                                     # param3: Pass through radius (meters)
            target_yaw_degrees if target_yaw_degrees is not None else float('nan'), # param4: Yaw angle (degrees)
            actual_waypoint_location.lat,          # x (latitude)
            actual_waypoint_location.lon,          # y (longitude)
            actual_waypoint_location.alt           # z (altitude)
        )
        all_drone_missions[drone_id].append(cmd)

# --- 4. 印出每台無人機的航線規劃 ---
print("\n--- 無人機航線規劃總覽 (包含偏航角、隊形旋轉和 5 秒停留) ---")
for drone_id, mission_commands in all_drone_missions.items():
    print(f"\n無人機 {drone_id} 的航線:")
    for i, cmd in enumerate(mission_commands):
        # 顯示 param1 (延遲時間) 和 param4 (偏航角)
        print(f"  航點 {i+1}: Lat={cmd.x:.7f}, Lon={cmd.y:.7f}, Alt={cmd.z:.2f}, Delay={cmd.param1}s, Yaw={cmd.param4:.2f} (Cmd: {cmd.command})")
"""
# --- 連接與上傳航線到多個 SITL 無人機 ---
print("\n--- 正在連接到多個 SITL 無人機並上傳任務 ---")

# 儲存所有連接的無人機物件
vehicles = {}

for i in range(1, formation_params["num_drones"] + 1):
    # SITL TCP 連接埠通常從 5760 開始，每個實例遞增 10
    # 例如：5760 (vehicle 1), 5770 (vehicle 2), 5780 (vehicle 3)...
    connection_string = f'tcp:127.0.0.1:{5760 + (i - 1) * 10}'
    print(f"嘗試連接到無人機 {i} ({connection_string})...")
    try:
        vehicle = connect(connection_string, wait_ready=True, timeout=60)
        vehicles[i] = vehicle
        print(f"成功連接到無人機 {i}！")

        # 清空現有任務
        cmds = vehicle.commands
        cmds.clear()
        cmds.upload() # 確保清除命令生效
        print(f"已清除無人機 {i} 現有任務。")
        time.sleep(0.5) # 給一點時間讓 MAVLink 處理

        # 為這台無人機添加其規劃好的航點
        # 注意：DroneKit 會自動為 MAV_CMD_NAV_WAYPOINT 添加 HOME 點 (MAV_CMD_NAV_LOITER_UNLIMITED) 作為第一個命令
        # 如果你的航點包含 HOME 點 (通常是航點列表的第一個)，則需要處理重複添加的問題
        # 一般來說，任務會從 1 號命令開始，0 號命令是 HOME。
        # 因此，這裡直接添加所有生成的航點。
        for cmd in all_drone_missions[i]:
            cmds.add(cmd)

        print(f"正在上傳無人機 {i} 的新航線...")
        cmds.upload()
        print(f"無人機 {i} 航線已成功上傳！")
        
        # 等待任務上傳完成
        vehicle.commands.wait_ready() 
        print(f"無人機 {i} 的任務中有 {len(vehicle.commands)} 個航點 (包含 HOME 點)。")

    except Exception as e:
        print(f"連接或上傳任務到無人機 {i} 失敗: {e}")
        if i in vehicles:
            vehicles[i].close() # 關閉可能的部分連接

print("\n--- 所有無人機任務上傳完成 ---")

# 您可以在此處添加 ARMing 和 MODE SETTING 的邏輯
# 例如，讓所有無人機進入 AUTO 模式
# for drone_id, vehicle in vehicles.items():
#     if vehicle.is_armed and vehicle.mode.name == "GUIDED":
#         print(f"設置無人機 {drone_id} 為 'AUTO' 模式...")
#         vehicle.mode = VehicleMode("AUTO")
#         while not vehicle.mode.name == "AUTO":
#             print(f" 等待無人機 {drone_id} 進入 AUTO 模式...")
#             time.sleep(1)
#         print(f"無人機 {drone_id} 已進入 AUTO 模式。")
#     else:
#         print(f"無人機 {drone_id} 未準備好進入 AUTO 模式 (可能未ARM或不在GUIDED模式)。")

# 保持連接一段時間，以便您在 Mission Planner 中觀察
# print("\n保持所有連接 60 秒，您可以檢查地面站 (Mission Planner)...")
# time.sleep(60)

# 關閉所有連接
# print("\n關閉所有無人機連接...")
# for drone_id, vehicle in vehicles.items():
#     if vehicle is not None:
#         vehicle.close()
#         print(f"無人機 {drone_id} 連接已關閉。")

print("\n--- 程式執行結束 ---")
print("請確保您已經運行了足夠數量的 SITL 實例來對應 'num_drones'。")
print("例如，如果您有 5 台無人機，您需要運行 5 個 SITL 實例，分別監聽 5760, 5770, 5780, 5790, 5800。")
print("通常可以使用 `sim_vehicle.py -v ArduCopter -f quad --mav10 --console --map -L TNGC_Campus --instance 0` (然後 -instance 1, 2, ...) 來啟動 SITL。")
"""