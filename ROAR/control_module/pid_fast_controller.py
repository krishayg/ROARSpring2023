from operator import truediv
from matplotlib.pyplot import close
from pydantic import BaseModel, Field
from ROAR.control_module.controller import Controller
from ROAR.utilities_module.vehicle_models import VehicleControl, Vehicle
import keyboard

from ROAR.utilities_module.data_structures_models import Transform, Location, Rotation
from collections import deque
import numpy as np
import math
import logging
from ROAR.agent_module.agent import Agent
from typing import Tuple
import json
from pathlib import Path
from ROAR.planning_module.mission_planner.waypoint_following_mission_planner import WaypointFollowingMissionPlanner
from ROAR.perception_module.lane_detector import LaneDetector
import json

class PIDFastController(Controller):
    def __init__(self, agent, steering_boundary: Tuple[float, float],
                 throttle_boundary: Tuple[float, float], **kwargs):
        super().__init__(agent, **kwargs)
        self.max_speed = self.agent.agent_settings.max_speed
        throttle_boundary = throttle_boundary
        self.steering_boundary = steering_boundary
        self.config = json.load(Path(agent.agent_settings.pid_config_file_path).open(mode='r'))
        self.turn_number=0
        self.region = 1
        self.brake_counter = 0

        with open('ROAR\\configurations\\carla\\carla_agent_configuration.json', 'r') as file:
            carla_agent_config = json.load(file)

        starting_point = carla_agent_config['spawn_point_id']
        if starting_point<=3:
            self.region=1
        else:
            self.region=2
        
        self.waypoint_queue_region = []
        self.waypoint_queue_turning=[]
        with open("ROAR\\control_module\\region_list.txt") as f:
            for line in f:
                raw = line.split(",")
                waypoint = Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]), rotation=Rotation(pitch=0, yaw=0, roll=0))
                self.waypoint_queue_region.append(waypoint)
            if starting_point>3:
                closest_waypoint = self.waypoint_queue_region[0]
                for waypoint in self.waypoint_queue_region:
                    cur_dist = self.agent.vehicle.transform.location.distance(waypoint.location)
                    closest_dist = self.agent.vehicle.transform.location.distance(closest_waypoint.location)
                    if  cur_dist < closest_dist:
                        closest_waypoint = waypoint
                while self.waypoint_queue_region[0] != closest_waypoint:
                    print("DELETING")
                    self.waypoint_queue_region.pop(0)
        self.waypoint_queue_braking = []
        self.waypoint_queue_shifting=[]

        with open("ROAR\\control_module\\braking_list_mod.txt") as f:
            for line in f:
                raw = line.split(",")
                waypoint = [Transform(location=Location(x=raw[1], y=raw[2], z=raw[3]), rotation=Rotation(pitch=0, yaw=0, roll=0)),raw[0]]
                self.waypoint_queue_braking.append(waypoint)
            print(starting_point)
            if starting_point>3:
                closest_waypoint = self.waypoint_queue_braking[0]
                for waypoint in self.waypoint_queue_braking:
                    cur_dist = self.agent.vehicle.transform.location.distance(waypoint[0].location)
                    closest_dist = self.agent.vehicle.transform.location.distance(closest_waypoint[0].location)
                    if  cur_dist < closest_dist:
                        closest_waypoint = waypoint
                while self.waypoint_queue_braking[0] != closest_waypoint:
                    print("DELETING")
                    self.waypoint_queue_braking.pop(0)
            while starting_point>=10 and len(self.waypoint_queue_braking)>2:
                print(len(self.waypoint_queue_braking))
                self.waypoint_queue_braking.pop(0)
        with open("ROAR\\control_module\\turning_list.txt") as f:
            for line in f:
                raw = line.split(",")
                waypoint = Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]), rotation=Rotation(pitch=0, yaw=0, roll=0))
                self.waypoint_queue_turning.append(waypoint)
            if starting_point>=3:
        
                for i in range(4):
                    self.waypoint_queue_turning.pop(0)
                self.turn_number=4
            with open("ROAR\\control_module\\waypoint_shift_list.txt") as f:
                for line in f:
                    raw = line.split(",")
                    waypoint = Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]), rotation=Rotation(pitch=0, yaw=0, roll=0))
                    self.waypoint_queue_shifting.append(waypoint)
        self.waypoint_queue_sub_region=[]
        with open("ROAR\\control_module\\sub_region_list.txt") as f:
            for line in f:
                raw = line.split(",")
                waypoint = Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]), rotation=Rotation(pitch=0, yaw=0, roll=0))
                self.waypoint_queue_sub_region.append(waypoint)
        self.lat_pid_controller = LatPIDController(
            agent=agent,
            config=self.config["latitudinal_controller"],
            steering_boundary=steering_boundary
        )
        self.logger = logging.getLogger(__name__)
        if starting_point>=6:
            self.sub_region=1
            self.waypoint_queue_sub_region.pop(0)
        else:
            self.sub_region=0

    def run_in_series(self, previous_waypoint,next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform, close_waypoint_next:Transform,close_waypoint_track:Transform,**kwargs) -> VehicleControl:
        if previous_waypoint:
            pass
        else:
            previous_waypoint=next_waypoint
            
        current_speed = Vehicle.get_speed(self.agent.vehicle)
        waypoint=self.waypoint_queue_shifting[0]
        dist = self.agent.vehicle.transform.location.distance(waypoint.location)
        shift_manual=0
        if dist<=5:
            self.waypoint_queue_shifting.pop(0)
            shift_manual=1
        
        waypoint=self.waypoint_queue_sub_region[0]
        dist = self.agent.vehicle.transform.location.distance(waypoint.location)
        if dist<=5:
            self.waypoint_queue_sub_region.pop(0)
            self.sub_region+=1
        # run lat pid controller
        steering, error, wide_error, sharp_error,track_error = self.lat_pid_controller.run_in_series(sub_region=self.sub_region,shift_manual=shift_manual,current_speed=current_speed,region=self.region, previous_waypoint=previous_waypoint,next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint,close_waypoint_next=close_waypoint_next,close_waypoint_track=close_waypoint_track,turn_number=self.turn_number)
        
        self.lane_detector =LaneDetector(agent=self.agent)
        
        # get errors from lat pid
        error = abs(round(error, 3))
        wide_error = abs(round(wide_error, 3))
        sharp_error = abs(round(sharp_error, 3))
        # calculate change in pitch
        pitch = float(next_waypoint.record().split(",")[4])
        hand_brake=0
        turn_point=self.waypoint_queue_turning[0]
        disttoturn=self.agent.vehicle.transform.location.distance(turn_point.location)
        if disttoturn<=30 and ((sharp_error<0.20 and self.region==1) or  (self.region==3 and sharp_error<0.4)):
            self.waypoint_queue_turning.pop(0)
            self.turn_number+=1
        sharp_error_threshold=0.68
        speed_threshold=90
        #Uses a different error threshold value for each turn. Each turn is slightly different so this is more efficient than using a generalized formula.
        if self.turn_number==0:
            sharp_error_threshold=0.73
        elif self.turn_number==1:
            sharp_error_threshold=0.82
            speed_threshold=108
        elif self.turn_number==2:
            sharp_error_threshold=0.63
            speed_threshold=80
        elif self.turn_number==3:
            sharp_error_threshold=0.74
            speed_threshold=110
        elif self.turn_number ==4:
            sharp_error_threshold=0.68
            speed_threshold=90
        elif self.turn_number==5:
            sharp_error_threshold=0.83
            speed_threshold=108
        elif self.turn_number==6:
            sharp_error_threshold=0.63
            speed_threshold=75
        print(self.region)
        if self.region==1:
            if sharp_error < sharp_error_threshold or current_speed <= speed_threshold:
                throttle = 1
                brake = 0
                hand_brake=0
            else:
                throttle = -1
                brake = 1
                hand_brake=1
        #Regions 2 and regions 3 are essentially the same, except they have slightly different parameters
        elif self.region ==2:
            waypoint = self.waypoint_queue_braking[0][0] 
            dist = self.agent.vehicle.transform.location.distance(waypoint.location)
            if dist <= 5:
                self.brake_counter = 1
                self.waypoint_queue_braking.pop(0)
            if self.brake_counter > 0:
                throttle = -1
                brake = 1
                hand_brake=0 
                self.brake_counter += 1
                if self.brake_counter >= int(self.waypoint_queue_braking[0][1]):
                    self.brake_counter = 0
            elif sharp_error >= 0.67 and current_speed > 81: 
                throttle = 0
                brake =current_speed/400
            elif wide_error > 0.09 and current_speed > 95: 
                speed_multiplier=0.00256
                if self.sub_region in [0,2]:
                    speed_multiplier=0.00256
                elif self.sub_region==1:
                    speed_multiplier=0.00225
                if self.sub_region==3:
                    speed_multiplier=0.00233
                elif self.sub_region==4:
                    speed_multiplier=0.0025
                elif self.sub_region==5:
                    speed_multiplier=0.00232
                throttle = max(0, 1 - 6*pow(wide_error*0.9 + current_speed*speed_multiplier, 6)) 
                brake = 0
            else:
                throttle = 1
                brake = 0
        elif self.region == 3:
            if sharp_error > sharp_error_threshold and current_speed >= speed_threshold:
                throttle = -1
                brake = 1
                #Hand brake is also used to allow the car to brake faster. This allows it to travel fast for longer before it needs to brake.
                hand_brake=1
            elif sharp_error >= 0.7 and current_speed > 115: 
                throttle = 0
                brake =0.4
            elif wide_error > 0.09 and current_speed > 100: # wide turn  
                speed_multiplier=0
                if self.turn_number<=6:
                    speed_multiplier=0.0015
                else:
                    speed_multiplier=0.0023
                throttle = max(0, 1 - 6*pow(wide_error*1 + current_speed*speed_multiplier, 6))
                brake = 0
            else:
                throttle = 1
                brake = 0
                hand_brake=0
            


        elif self.region==4:
            waypoint = self.waypoint_queue_braking[0][0] 
            dist = self.agent.vehicle.transform.location.distance(waypoint.location)
            if dist <= 5:
                self.brake_counter = 1
                self.waypoint_queue_braking.pop(0)
            if self.brake_counter > 0:
                throttle = -1
                brake = 1
                hand_brake=0
                self.brake_counter += 1
                if self.brake_counter >= 10:
                    self.brake_counter = 0
            elif sharp_error >=0.45 and current_speed > 75:
                throttle = 0
                brake = 0.7
            elif wide_error > 0.09 and current_speed > 92:
                throttle = max(0, 1 - 6*pow(wide_error + current_speed*0.003, 6))
                brake = 0
            else:
                throttle = 1
                brake = 0
      
        gear = max(1, (int)((current_speed - 2*pitch) / 60))
        if throttle == -1:
            gear = -1
        
        waypoint = self.waypoint_queue_region[0]
        dist = self.agent.vehicle.transform.location.distance(waypoint.location)
        if dist <= 30:
            self.region += 1
            self.waypoint_queue_region.pop(0)
        
        if keyboard.is_pressed("space"):
             print(self.agent.vehicle.transform.record())
        
        return VehicleControl(throttle=throttle, steering=steering, brake=brake, gear=gear,hand_brake=hand_brake)

    @staticmethod
    def find_k_values(vehicle: Vehicle, config: dict) -> np.array:
        current_speed = Vehicle.get_speed(vehicle=vehicle)
        k_p, k_d, k_i = 1, 0, 0
        for speed_upper_bound, kvalues in config.items():
            speed_upper_bound = float(speed_upper_bound)
            if current_speed < speed_upper_bound:
                k_p, k_d, k_i = kvalues["Kp"], kvalues["Kd"], kvalues["Ki"]
                break
        return np.array([k_p, k_d, k_i])
    #Allows the car to have more stable parameters
    def find_k_values_formula(vehicle: Vehicle, config: dict) -> np.array: 
        #uses a sigmoid function to calculate k values
        e=2.71828
        current_speed = Vehicle.get_speed(vehicle=vehicle)
        k_p=-0.0225343+(0.6858377+0.0225343)/(1+(current_speed/160.9069)**5.880651)
        k_d=-0.01573431+(0.2335526+0.01573431)/(1+(current_speed/113.8959)**3.660237)
        k_i=0.01
        #print(k_p,k_d,k_i)
        return np.array([k_p, k_d, k_i])

class LatPIDController(Controller):
    def __init__(self, agent, config: dict, steering_boundary: Tuple[float, float],
                 dt: float = 0.03, **kwargs):
        super().__init__(agent, **kwargs)
        self.config = config
        self.steering_boundary = steering_boundary
        self._error_buffer = deque(maxlen=10)
        self._error_buffer_center = deque(maxlen=10)
        self._dt = dt
        self.lane_detector =LaneDetector(agent=self.agent)
        self.to_shift=0
        self.after_shift=0
    @staticmethod
    def shift_waypoint(point1,point2,direction,amt):
        if (point1==point2).all():
            return point2
        vector_between_points = point2 - point1
        length = np.linalg.norm(vector_between_points)
        unit_vector = vector_between_points / length
        translation_length = amt*direction

        translation_vector = np.array([-unit_vector[2], 0, unit_vector[0],0,0,0]) * translation_length
        point2 = point2 + translation_vector
        return point2
    def calculate_error(self,previous_waypoint: Transform,next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform,close_waypoint_next:Transform,close_waypoint_track:Transform):
        v_begin = self.agent.vehicle.transform.location.to_array()
        direction_vector = np.array([-np.sin(np.deg2rad(self.agent.vehicle.transform.rotation.yaw)),
                                     0,
                                     -np.cos(np.deg2rad(self.agent.vehicle.transform.rotation.yaw))])
        v_end = v_begin + direction_vector

        v_vec = np.array([(v_end[0] - v_begin[0]), 0, (v_end[2] - v_begin[2])])
        
        w_vec = np.array(
            [
                next_waypoint.location.x - v_begin[0],
                0,
                next_waypoint.location.z - v_begin[2],
            ]
        )

        v_vec_normed = v_vec / np.linalg.norm(v_vec)
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) 
        cross = np.cross(v_vec_normed, w_vec_normed)

        if cross[1] > 0:
            error *= -1
        w_vec = np.array(
            [
                close_waypoint.location.x - v_begin[0],
                0,
                close_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        wide_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1))
        cross=np.cross(v_vec_normed, w_vec_normed)

        w_vec = np.array(
            [
                far_waypoint.location.x - v_begin[0],
                0,
                far_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        sharp_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive
        cur_waypoints_vec=np.array(
            [
                next_waypoint.location.x - previous_waypoint.location.x,
                0,
                next_waypoint.location.z - previous_waypoint.location.z,
            ]
        )
        next_waypoints_vec=np.array(
            [
                close_waypoint_next.location.x - close_waypoint_track.location.x,
                0,
                close_waypoint_next.location.z - close_waypoint_track.location.z,
            ]
        )
        next_waypoints_vec = next_waypoints_vec / np.linalg.norm(next_waypoints_vec)
        cur_waypoints_vec=cur_waypoints_vec/np.linalg.norm(cur_waypoints_vec)
        #track error corresponds to the angle between the vectors formed by the 2 closest waypoints and 2 consecutive look-ahead waypoints. 
        # The purpose is to measure a definite value of how large the turn is, as opposed to sharp_error which also factors in how close the turn is.  
        track_error=np.arccos(min(max(cur_waypoints_vec @ next_waypoints_vec.T, -1), 1))
        cross=np.cross(cur_waypoints_vec,next_waypoints_vec)
        return error,sharp_error,wide_error,cross,track_error
    def run_in_series(self, sub_region,shift_manual,current_speed,region,turn_number,previous_waypoint:Transform, next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform,close_waypoint_next:Transform, close_waypoint_track:Transform,**kwargs) -> float:
        """
        Calculates a vector that represent where you are going.
        Args:
            next_waypoint ():
            **kwargs ():

        Returns:
            lat_control
        """
        
        error,sharp_error,wide_error,cross,track_error=self.calculate_error(previous_waypoint=previous_waypoint,next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint,close_waypoint_next=close_waypoint_next,close_waypoint_track=close_waypoint_track)
        toshiftstart=int((0.9*round(pow(current_speed, 2)*0.002 + 0.7*current_speed))) # number of timesteps to shift the waypoints
        if region==2 or turn_number!=5:
            max_shift=max(2.5,1/(current_speed+1e-16)*225) #if the speed is higher, the shift in waypoints is less; higher speed causes larger overshoot which can lead to crash. So, we are reducing the shift amount
        elif turn_number==5:
            max_shift=1.5
        else:
            max_shift=0
        if ((track_error>=0.75) and region in [2,3] and self.to_shift==0) or shift_manual: 
            if self.to_shift==0:
                self.track_error_start=track_error
                self.after_shift=10 
                self.max_shift_saved=max_shift
                self.toshiftstart_saved=toshiftstart+1e-10
                self.to_shift=int(toshiftstart)
                shiftamt=(-1*max_shift*4)/(self.toshiftstart_saved**2)*(self.to_shift-self.toshiftstart_saved/2)**2+max_shift #at first shift very less and gradually increase the shift
            else:
                shiftamt=self.toshiftstart_saved

            self.shift_direction=0

            if cross[1]>0:
                self.shift_direction=-1 #opposite direction of turn
            else:
                self.shift_direction=1
            next_waypoint=Transform.from_array(self.shift_waypoint(previous_waypoint.to_array(),next_waypoint.to_array(),self.shift_direction,shiftamt))
        elif self.to_shift>0:
            shiftamt=((-1*self.max_shift_saved*4)/(self.toshiftstart_saved**2)*(self.to_shift-self.toshiftstart_saved/2)**2+self.max_shift_saved)
            if cross[1]>0:
                next_waypoint=Transform.from_array(self.shift_waypoint(previous_waypoint.to_array(),next_waypoint.to_array(),self.shift_direction,shiftamt))
            else:
                next_waypoint=Transform.from_array(self.shift_waypoint(previous_waypoint.to_array(),next_waypoint.to_array(),self.shift_direction,shiftamt))
            self.to_shift-=1
        
        error,_,_,_,_=self.calculate_error(previous_waypoint=previous_waypoint,next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint,close_waypoint_next=close_waypoint_next,close_waypoint_track=close_waypoint_track)
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        k_p, k_d,_= PIDFastController.find_k_values_formula(config=self.config, vehicle=self.agent.vehicle)
        if region==1:
            _,_,k_i=PIDFastController.find_k_values(config=self.config, vehicle=self.agent.vehicle)
        else:
            _,_,k_i=PIDFastController.find_k_values(config=self.config, vehicle=self.agent.vehicle)
        if self.to_shift>0 and self.to_shift<self.toshiftstart_saved:
           if sub_region not in [2,5]:
                k_p*=2           
                k_d=k_d/2    
           else:
                k_p*=1.6        
                k_d=k_d/2.6   
        elif self.to_shift==0 and self.after_shift>0:
            k_p=k_p/2
            self.after_shift-=1
        lat_control = float(
                np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), self.steering_boundary[0], self.steering_boundary[1])
            )
        return lat_control, error, wide_error, sharp_error,track_error
    