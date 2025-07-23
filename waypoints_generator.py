import math
from dronekit import connect, Command, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil # 導入 mavutil 以使用 MAV_FRAME 和 MAV_CMD 常數
import time

# --- 1. 定義隊形參數 ---
#      u3
#   u2     u4
# u1          u5
# 設定隊形相關參數
formation_params = {
    "num_drones": 5,
    "lead_drone_id": 3, # 隊形中心點對應的無人機，通常在V字頂點 (U3)
    "spacing_x": 10,  # 無人機在隊形橫向（左右）的間距 (米)
    "spacing_y": 15,  # 無人機在隊形縱向（前後）的間距 (米)
    "altitude": 30    # 編隊的預設飛行高度 (米)
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
virtual_center_waypoints_coords = [
    (23.000000, 120.000000, formation_params["altitude"]), # 航點 1 (起點)
    (23.000100, 120.000100, formation_params["altitude"]), # 航點 2
    (23.000200, 120.000000, formation_params["altitude"]), # 航點 3
    (23.000100, 120.000200, formation_params["altitude"]), # 航點 4
    (23.000000, 120.000000, formation_params["altitude"])  # 航點 5 (回到起點)
]

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
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            5,                   # param1: 延遲時間 (秒) - 這裡設定為 5 秒
            0,                   # param2: 接受半徑 (米)
            0,                   # param3: 傳遞半徑 (米)
            target_yaw_degrees if target_yaw_degrees is not None else float('nan'), # param4: 偏航角 (度)
            actual_waypoint_location.lat,
            actual_waypoint_location.lon,
            actual_waypoint_location.alt
        )
        all_drone_missions[drone_id].append(cmd)

# --- 4. 印出每台無人機的航線規劃 ---
print("\n--- 無人機航線規劃總覽 (包含偏航角、隊形旋轉和 5 秒停留) ---")
for drone_id, mission_commands in all_drone_missions.items():
    print(f"\n無人機 {drone_id} 的航線:")
    for i, cmd in enumerate(mission_commands):
        # 顯示 param1 (延遲時間) 和 param4 (偏航角)
        print(f"  航點 {i+1}: Lat={cmd.x:.7f}, Lon={cmd.y:.7f}, Alt={cmd.z:.2f}, Delay={cmd.param1}s, Yaw={cmd.param4:.2f} (Cmd: {cmd.command})")

# --- 連接與上傳航線 (僅為示範，實際應用需要多個無人機連接實例) ---
# 此處的代碼僅用於示範生成航點。
# 實際的多機控制需要獨立的連接和更強大的任務管理策略。

# # 以下是連接並上傳航線到單個無人機的範例
# # 假設您連接到第一台無人機 (Drone 1)
# print("\n正在連接到第一台無人機 (用於上傳任務)...")
# try:
#     # 通常實際連接會是不同的 IP/Port 或序列埠
#     vehicle_drone1 = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)
#     print("已連接到第一台無人機！")

#     # 清空現有任務
#     cmds = vehicle_drone1.commands
#     cmds.clear()
#     cmds.upload() # 確保清除命令生效
#     print("已清除無人機現有任務。")
#     time.sleep(1)

#     # 為這台無人機添加其規劃好的航點
#     for cmd in all_drone_missions[1]: # 以 Drone 1 的航線為例
#         cmds.add(cmd)

#     print("正在上傳無人機 1 的新航線...")
#     cmds.upload()
#     print("無人機 1 航線已成功上傳！")

#     # 等待任務上傳完成
#     vehicle_drone1.commands.wait_ready() 
#     print(f"無人機 1 的任務中有 {len(vehicle_drone1.commands)} 個航點。")

#     # 設置無人機模式為 AUTO，準備執行任務
#     # print("將無人機 1 模式設定為 'AUTO'...")
#     # vehicle_drone1.mode = VehicleMode("AUTO")
#     # print("模式已設定為 AUTO。")

#     # 等待一段時間，讓您檢查狀態
#     # print("保持連接 30 秒，您可以檢查地面站...")
#     # time.sleep(30)

#     # vehicle_drone1.close()
#     # print("第一台無人機連接已關閉。")

# except Exception as e:
#     print(f"連接或上傳任務到無人機失敗: {e}")

print("\n--- 航線規劃完成 ---")
print("每個無人機的航線已生成，包含根據飛行方向旋轉的隊形偏移、偏航角，以及航點停留 5 秒的設定。")
print("您可以將這些航點載入到各自的無人機中。")
print("請注意，實際的多機控制需要獨立的連接和更強大的任務管理策略。")