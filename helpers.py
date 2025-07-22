from dronekit import LocationGlobalRelative
from geopy import distance
from geopy.distance import geodesic
import math
import numpy as np

def calculate_distance_lla(pos1:LocationGlobalRelative, pos2:LocationGlobalRelative) ->float:
    # Calculate the distance between two points in meters
    return distance.distance((pos1.lat, pos1.lon), (pos2.lat, pos2.lon)).m

def calculate_desired_positions_global(formation_center:LocationGlobalRelative, offsets:np.array) -> list:
    """
    formation_center is a LocationGlobalRelative(lat, lon , 0) 
    return a list of LocationGlobalRelative(lat, lon , 0)
    """
    desired_positions = []
    for offset in offsets:
        # Calculate the new latitude and longitude based on the offset
        new_position = geodesic(meters=offset[0]).destination((formation_center.lat, formation_center.lon), 0)  # Offset in the north direction
        new_lat = new_position.latitude
        new_position = geodesic(meters=offset[1]).destination((new_lat, formation_center.lon), 90)  # Offset in the east direction
        new_lon = new_position.longitude
        desired_positions.append(LocationGlobalRelative(new_lat, new_lon, 0))
    return desired_positions

def interpolate_waypoints(waypoints: list, num_points: int)->list: 
    '''
    Interpolate between the waypoints given in the list to generate a new waypoint_list.
    waypoints: List of waypoints(LocationGlobalRelative(lat, lon , 0)) to interpolate between.
    num_points: Number of points to interpolate between two consecutive waypoints.
    '''
    interpolated_waypoints = []
    for i in range(len(waypoints) - 1):
        lat_diff = (waypoints[i + 1].lat - waypoints[i].lat) / num_points
        lon_diff = (waypoints[i + 1].lon - waypoints[i].lon) / num_points
        for j in range(num_points):
            interpolated_waypoints.append(LocationGlobalRelative(waypoints[i].lat + j * lat_diff, waypoints[i].lon + j * lon_diff, 0))
   
    interpolated_waypoints.append(waypoints[-1])
    return interpolated_waypoints

def calculate_yaw_angle(current_pos: LocationGlobalRelative, center_pos: LocationGlobalRelative) -> float:
    delta_lat = center_pos.lat - current_pos.lat
    delta_lon = center_pos.lon - current_pos.lon
    yaw_rad = math.atan2(delta_lon, delta_lat)  # 注意經度放在前面
    yaw_deg = math.degrees(yaw_rad)
    return (yaw_deg + 360) % 360  # 確保角度範圍是 [0, 360)

def extract_waypoints(filepath):
    """
    從Mission Planner的航點檔案中提取航點資訊。

    Args:
        filepath (str): 航點檔案的路徑。

    Returns:
        list: 包含字典的列表，每個字典代表一個航點，
              鍵包括 'latitude', 'longitude', 'altitude' 等。
              如果檔案格式不符或讀取失敗，則返回空列表。
    """
    waypoints = []
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()

        # Mission Planner 航點檔案通常會有一行標頭，例如 "QGC WPL 110"
        # 並且航點資訊從第二行開始
        if not lines or not lines[0].strip().startswith("QGC WPL"):
            print("錯誤：檔案格式可能不是Mission Planner的航點檔案。")
            return []

        for i, line in enumerate(lines[1:]):  # 從第二行開始解析航點
            parts = line.strip().split('\t')  # Mission Planner 通常使用 Tab 分隔

            if len(parts) < 12:  # 檢查是否包含足夠的列（根據QGC WPL 110標準）
                print(f"警告：第 {i+2} 行的格式不完整，跳過。")
                continue

            try:
                # 根據 QGC WPL 110 標準，常用的航點資訊位於以下索引：
                # 0: 序列號
                # 1: 當前（0）/非當前（1）
                # 2: 座標系 (例如：3代表 WGS84)
                # 3: 任務命令 (例如：16代表 WAYPOINT)
                # 4-6: 參數 1-3
                # 7: 緯度 (latitude)
                # 8: 經度 (longitude)
                # 9: 高度 (altitude)
                # 10: 自動重複 (0/1)
                # 11: 航點類型 (例如：0代表 普通航點，1代表 起飛，2代表 降落)

                waypoint = {
                    'sequence': int(parts[0]),
                    'current': int(parts[1]),
                    'coordinate_frame': int(parts[2]),
                    'command': int(parts[3]),
                    'param1': float(parts[4]),
                    'param2': float(parts[5]),
                    'param3': float(parts[6]),
                    'latitude': float(parts[7]),
                    'longitude': float(parts[8]),
                    'altitude': float(parts[9]),
                    'autocontinue': int(parts[10]),
                    'mission_type': int(parts[11])
                }
                waypoints.append(waypoint)
            except ValueError as e:
                print(f"錯誤：解析第 {i+2} 行數據時發生錯誤：{e}。跳過此行。")
                continue

    except FileNotFoundError:
        print(f"錯誤：找不到檔案 '{filepath}'。請檢查路徑是否正確。")
    except Exception as e:
        print(f"讀取或處理檔案時發生未知錯誤：{e}")

    return waypoints

# 示例用法
if __name__ == "__main__":
    # 假設您的航點檔案名為 'mission.waypoints' 或 'mission.txt'
    # 請將 'mission.waypoints' 替換為您的實際檔案路徑
    file_path = 'mission.waypoints'

    # 創建一個模擬的航點檔案，以便運行範例
    # 在實際應用中，您會直接讀取 Mission Planner 導出的檔案
    with open(file_path, 'w') as f:
        f.write("QGC WPL 110\n")
        f.write("0\t1\t3\t16\t0.000000\t0.000000\t0.000000\t22.999000\t120.213000\t50.000000\t1\t0\n")
        f.write("1\t0\t3\t16\t0.000000\t0.000000\t0.000000\t22.998500\t120.213500\t60.000000\t1\t0\n")
        f.write("2\t0\t3\t16\t0.000000\t0.000000\t0.000000\t22.998000\t120.214000\t70.000000\t1\t0\n")
        f.write("3\t0\t3\t21\t0.000000\t0.000000\t0.000000\t0.000000\t0.000000\t0.000000\t1\t0\n") # 範例：RTL (Return to Launch)

    extracted_waypoints = extract_waypoints(file_path)

    if extracted_waypoints:
        print(f"\n成功提取 {len(extracted_waypoints)} 個航點：")
        for wp in extracted_waypoints:
            print(f"  序列號: {wp['sequence']}, 緯度: {wp['latitude']:.6f}, 經度: {wp['longitude']:.6f}, 高度: {wp['altitude']:.2f}m, 命令: {wp['command']}")
    else:
        print("\n未能提取任何航點。")