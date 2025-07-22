import numpy as np

def generate_waypoints(center, spacing):
    # UAV 1 (頂端)
    uav1_offset = np.array([0, spacing])
    
    # UAV 2, UAV 3 (中間層)
    uav2_offset = np.array([-spacing/2, 0])  # 左側
    uav3_offset = np.array([spacing/2, 0])   # 右側
    
    # UAV 4, UAV 5 (底層)
    uav4_offset = np.array([-spacing, -spacing])  # 左下
    uav5_offset = np.array([spacing, -spacing])   # 右下
    
    # 四個中心點：依次是 (0, 0), (10, 10), (0, 20), (-10, 10)
    center_points = [np.array([0, 0]), np.array([10, 10]), np.array([0, 20]), np.array([-10, 10])]
    
    # 生成每個航點時，無人機的具體位置
    waypoints = {}
    for i, delta in enumerate([uav1_offset, uav2_offset, uav3_offset, uav4_offset, uav5_offset]):
        uav_name = f"UAV {i + 1}"
        uav_waypoints = []
        
        for center in center_points:  # 迭代每個航點的中心
            # 計算每個航點的無人機位置
            waypoint = center + delta
            uav_waypoints.append(waypoint)
        
        waypoints[uav_name] = uav_waypoints
    
    return waypoints

# 假設中心點在 (x, y)
center_point = np.array([0, 0])  # 這個變量不再需要，因為四個航點已經定義
spacing = 10  # UAV 之間的距離

# 生成航點
waypoints = generate_waypoints(center_point, spacing)

# 顯示每台無人機的航點
for uav, wp in waypoints.items():
    print(f"{uav} waypoints:")
    for i, point in enumerate(wp):
        print(f"  {i+1}: {point}")