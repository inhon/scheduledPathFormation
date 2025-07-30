# --- 1. 定義隊形參數 ---
#       u3
#   u2      u4
# u1            u5
# 設定隊形相關參數
formation_params = {
    "num_drones": 5,
    "lead_drone_id": 3,  # 隊形中心點對應的無人機，通常在倒V字頂點 (U3)
    "spacing_x": 10,   # 無人機在隊形橫向（左右）的間距 (米)
    "spacing_y": 10,   # 無人機在隊形縱向（前後）的間距 (米)
    "altitude": 30     # 編隊的預設飛行高度 (米)
}

# 定義每台無人機相對於隊形中心點的偏移量 (dx_body, dy_body, dz) (FRD: xyz)
# 這裡的 dx_body 和 dy_body 是在隊形自身的局部座標系(FRD)中定義的。
# dx_body: 沿著隊形縱向軸的偏移 (向前為正，向後為負)
# dy_body: 沿著隊形橫向軸的偏移 (向右為正)
drone_offsets_body_frame = {
    1: (-2 * formation_params["spacing_x"], -2 * formation_params["spacing_y"], 0), # U1: 最左後
    2: (-1 * formation_params["spacing_x"], -1 * formation_params["spacing_y"], 0), # U2: 左後
    3: (0, 0, 0),                                                                  # U3: 中心 (領頭機)
    4: (-1 * formation_params["spacing_x"], formation_params["spacing_y"], 0),  # U4: 右後
    5: (-2 * formation_params["spacing_x"], 2 * formation_params["spacing_y"], 0)   # U5: 最右後
}

waypoint_file = '2.waypoints'
uav_speed=5 #m/sec
takeoff_altitude=20 # meter
rtl_alt=20 #meter


