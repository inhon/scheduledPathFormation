from dronekit import LocationGlobalRelative, LocationGlobal
from Drone import Drone
import numpy as np
import helpers
#from helpers import calculate_desired_positions_global, calculate_yaw_angle, interpolate_waypoints, save_all_drone_missions
import time
from geopy.distance import geodesic
import formation_setting


class FormationFlying(object):
    def __init__(self):
        self.num_uavs = formation_setting.formation_params["num_drones"]        
        self.drones = {
            i: Drone(f'tcp:localhost:{formation_setting.connection_port + 10 * (i-1)}')
            for i in range(1, formation_setting.formation_params['num_drones'] + 1)
        }
        self.takeoff_altitude = formation_setting.takeoff_altitude # meter
        self.speed = formation_setting.uav_speed  # m/sec
        self.rtl_alt=formation_setting.rtl_alt #cm
   
    def set_rtl_alt_all(self): ##設定RTL高度
       for i, drone in self.drones.items():               
            if (drone.set_rtl_alt(self.rtl_alt)==True):
                print(f"set the UAV {i} RTL_ALT successful")
    
    def initialize_formation(self, waypoints: list[LocationGlobalRelative]): # 紀錄home點、設定guided 模式、解鎖、起飛，飛到第1個航點排列隊形 
        print("Starting Mission!")
        self.home=[]
        for i in range(1, self.num_uavs+1):
            home=self.drones[i].get_home_location()
            print(f"UAV {i} home location set: {home.lat}, {home.lon}, {home.alt}")
            self.home.append(home)          
 
        while(input("\033[93m {}\033[00m" .format("Change UAVs to GUIDED mode and takeoff? y/n\n")) != "y"):
            pass

        for i in range(1,self.num_uavs+1): # change drone to GUIDED mode and arm
            self.drones[i].set_guided_and_arm()
            print(f"UAV {i} changed mode to GUIDED and armed successfully!")
            self.drones[i].takeoff(self.takeoff_altitude)
            print(f"UAV {i} took off successfully!") 
        
        while(input("\033[93m {}\033[00m" .format("Make sure All UAVs in the air! ? y/n\n")) != "y"):
            pass
        
        print("Initializing Formation!")
        
        for i in range(1,self.num_uavs+1):
            desired_pos=waypoints[i-1]#LocationGlobalRelative(lat, lon , virtual_waypoint中的高度)
            self.drones[i].fly_to_point_non_blocking(desired_pos,self.speed)
        
        forming_complete = True       
        while forming_complete:
            forming_complete = False
            for i in range(1,self.num_uavs+1):
                desired_pos=waypoints[i-1] #
                current_pos=self.drones[i].read_global_position() #drone: 編號從0開始 global_relative_frame
                distance_to_formation = geodesic((current_pos.lat, current_pos.lon), (desired_pos.lat, desired_pos.lon)).meters
                print(f"Drone {i} Distance to Formation: {distance_to_formation}")
                if distance_to_formation > formation_setting.wp_radius:
                    forming = True
                time.sleep(1) 
        
        print("Initial Formation Achieved! Proceeding to Waypoints")
        while(input("\033[93m {}\033[00m" .format("continue ? y/n\n")) != "y"):
            pass

    def waypoint_following(self, waypoints: list[LocationGlobalRelative]):
        for i in range(1,self.num_uavs+1):
            desired_pos=waypoints[i-1] #LocationGlobalRelative(lat, lon , alt)
            self.drones[i].fly_to_point_non_blocking(desired_pos,self.speed) #drones是一個dict，key 由1開始
 
        forming_complete = True       
        while forming_complete:
            forming_complete = False
            for i in range(1,self.num_uavs+1):
                desired_pos=waypoints[i-1] 
                current_pos=self.drones[i].read_global_position() #drone: 編號從1開始 global_relative_frame
                distance_to_formation = geodesic((current_pos.lat, current_pos.lon), (desired_pos.lat, desired_pos.lon)).meters
                print(f"Drone {i} Distance to Formation: {distance_to_formation}")
                if distance_to_formation > formation_setting.wp_radius:
                    forming = True
                time.sleep(1) 
        print("Formation Achieved! Proceeding to Waypoints")
        while(input("\033[93m {}\033[00m" .format("continue ? y/n\n")) != "y"):
            pass

    def rtl_all(self):
        for i in range(1,self.num_uavs+1):
            self.drones[i].rtl() #drones是一個dict，key 由1開始
            time.sleep(1)

if __name__ == "__main__":
   
    all_drone_missions = helpers.save_all_drone_missions() #取得航線dict, {id_1:[(lat, lon, alt),()], is_2:[(lat, lon, alt),()]...} 
    transposed_all_drone_missions=helpers.transpose_to_location_relative(all_drone_missions)
    formation_flying = FormationFlying()
    formation_flying.set_rtl_alt_all()
    #紀錄home點、設定guided 模式、解鎖、起飛，飛到第1個航點排列隊形 
    formation_flying.initialize_formation(transposed_all_drone_missions[1]) # 1是waypoint id 
    #執行任務
    for waypoint_id, waypoints in transposed_all_drone_missions.items():
        if waypoint_id==1 :
            continue
        formation_flying.waypoint_following(waypoints)

    #return to home locations
    formation_flying.rtl_all()
    
    print("Mission Completed!")

    