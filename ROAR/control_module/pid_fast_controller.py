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

class PIDFastController(Controller):
    def __init__(self, agent, steering_boundary: Tuple[float, float],
                 throttle_boundary: Tuple[float, float], **kwargs):
        super().__init__(agent, **kwargs)
        self.max_speed = self.agent.agent_settings.max_speed
        throttle_boundary = throttle_boundary
        self.steering_boundary = steering_boundary
        self.config = json.load(Path(agent.agent_settings.pid_config_file_path).open(mode='r'))
        
        # useful variables
        self.turn_number=0
        self.region = 1 #################################################################3################CHANGE THIS!!!!!!!!!!!!!!!!!!!!!!!!!
        self.brake_counter = 0

        self.waypoint_queue_region = []
        self.waypoint_queue_turning=[]
        with open("ROAR\\control_module\\region_list.txt") as f:
            for line in f:
                raw = line.split(",")
                waypoint = Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]), rotation=Rotation(pitch=0, yaw=0, roll=0))
                self.waypoint_queue_region.append(waypoint)

        self.waypoint_queue_braking = []
        with open("ROAR\\control_module\\braking_list_mod.txt") as f:
            for line in f:
                raw = line.split(",")
                waypoint = Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]), rotation=Rotation(pitch=0, yaw=0, roll=0))
                self.waypoint_queue_braking.append(waypoint)
        with open("ROAR\\control_module\\turning_list.txt") as f:
            for line in f:
                raw = line.split(",")
                waypoint = Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]), rotation=Rotation(pitch=0, yaw=0, roll=0))
                self.waypoint_queue_turning.append(waypoint)
        self.lat_pid_controller = LatPIDController(
            agent=agent,
            config=self.config["latitudinal_controller"],
            steering_boundary=steering_boundary
        )
        self.logger = logging.getLogger(__name__)

    def run_in_series(self, previous_waypoint,next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform, **kwargs) -> VehicleControl:
        if previous_waypoint:
            pass
            #print(previous_waypoint.location.to_array())
        else:
            previous_waypoint=next_waypoint
            
        #print(self.region)
        current_speed = Vehicle.get_speed(self.agent.vehicle)

        # run lat pid controller
        steering, error, wide_error, sharp_error = self.lat_pid_controller.run_in_series(current_speed=current_speed,region=self.region, previous_waypoint=previous_waypoint,next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint)
        
        self.lane_detector =LaneDetector(agent=self.agent)
        #print(self.lane_detector.run_in_series())
        
        # get errors from lat pid
        error = abs(round(error, 3))
        wide_error = abs(round(wide_error, 3))
        sharp_error = abs(round(sharp_error, 3))
        #print(error, wide_error, sharp_error)
        #print(wide_error)
        # calculate change in pitch
        pitch = float(next_waypoint.record().split(",")[4])
        hand_brake=0
        turn_point=self.waypoint_queue_turning[0]
        disttoturn=self.agent.vehicle.transform.location.distance(turn_point.location)
        if disttoturn<=30 and sharp_error<0.20:
            self.waypoint_queue_turning.pop(0)
            self.turn_number+=1
        #print(self.turn_number)
        sharp_error_threshold=0.68
        speed_threshold=90
        if self.turn_number==0:
            sharp_error_threshold=0.73
        elif self.turn_number==1:
            sharp_error_threshold=0.82
            speed_threshold=111
        elif self.turn_number==2:
            sharp_error_threshold=0.63
            speed_threshold=80
        else:
            sharp_error_threshold=0.74
            speed_threshold=110
        #print(sharp_error_threshold)
        if self.region == 1:
            #if sharp_error < sharp_error_threshold or current_speed <= speed_threshold:#0.68
            if sharp_error < sharp_error_threshold or current_speed <= speed_threshold:#0.68
                throttle = 1
                brake = 0
                hand_brake=0
            else:
                throttle = -1
                brake = 1
                hand_brake=1
        elif self.region ==2:
            waypoint = self.waypoint_queue_braking[0] # 5012 is weird bump spot
            dist = self.agent.vehicle.transform.location.distance(waypoint.location)
            if dist <= 5:
                self.brake_counter = 1
                # print(self.waypoint_queue_braking[0])
                self.waypoint_queue_braking.pop(0)
            if self.brake_counter > 0:
                print("BRAKING")
                throttle = -1
                brake = 1
                hand_brake=0 #KG
                self.brake_counter += 1
                if self.brake_counter >= 5:#4: #KG
                    self.brake_counter = 0
            elif sharp_error >= 0.70 and current_speed > 93: #originally 0.65, 80
                throttle = 0
                brake = min(0.4,sharp_error/2.3)#0.4
            elif wide_error > 0.13 and current_speed > 96: # wide turn  #originally 0.09,92
                throttle = max(0, 1 - 6*pow(wide_error + current_speed*0.003, 6))
                brake = 0
            else:
                throttle = 1
                brake = 0
        elif self.region==3:
            waypoint = self.waypoint_queue_braking[0] # 5012 is weird bump spot
            dist = self.agent.vehicle.transform.location.distance(waypoint.location)
            if dist <= 5:
                self.brake_counter = 1
                # print(self.waypoint_queue_braking[0])
                self.waypoint_queue_braking.pop(0)
            if self.brake_counter > 0:
                throttle = -1
                brake = 1
                hand_brake=0 #KG
                self.brake_counter += 1
                if self.brake_counter >= 4:#4: #KG
                    self.brake_counter = 0
            elif sharp_error >= 0.65 and current_speed > 80: #originally 0.65, 80
                throttle = 0
                brake = min(0.4,sharp_error/2.3)#0.4
            elif wide_error > 0.10 and current_speed > 91: # wide turn  #originally 0.09,92
                throttle = max(0, 1 - 6*pow(wide_error + current_speed*0.003, 6))
                brake = 0
            else:
                throttle = 1
                brake = 0
        elif self.region==4: #KG switching back to city
            if sharp_error < 0.68 or current_speed <= 90:#0.68
                throttle = 1
                brake = 0
                hand_brake=0
            else:
                throttle = -1
                brake = 1
                hand_brake=1
        
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
    def find_k_values_formula(vehicle: Vehicle, config: dict) -> np.array:  ##KG
        #uses a sigmoid function to calculate k values
        e=2.71828
        current_speed = Vehicle.get_speed(vehicle=vehicle)
        k_p=-0.0225343+(0.6858377+0.0225343)/(1+(current_speed/160.9069)**5.880651)#min(1,1.8*(1-(1/(1+e**(current_speed*-1/100)))))
        k_d=-0.01573431+(0.2335526+0.01573431)/(1+(current_speed/113.8959)**3.660237)#1*(1-(1/(1+e**(current_speed*-1/100))))
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
        self._error_buffer_center=deque(maxlen=10)
        self._dt = dt
        self.lane_detector =LaneDetector(agent=self.agent)
        self.to_shift=0
    @staticmethod
    def shift_waypoint(point1,point2,direction,amt):
        if (point1==point2).all():
            #print(point2)
            return point2
        vector_between_points = point2 - point1
        #print( vector_between_points )
        # Calculate the length of the vector
        length = np.linalg.norm(vector_between_points)
        
        # Calculate the unit vector along the vector between the points
        unit_vector = vector_between_points / length

        # Specify the translation length
        translation_length = amt*direction

        # Calculate the translation vector orthogonally to the unit vector
        translation_vector = np.array([-unit_vector[2], 0, unit_vector[0],0,0,0]) * translation_length
        #print(translation_vector)
        # Translate point2 orthogonally to the vector between the points by the specified length
        point2 = point2 + translation_vector
        #print(point2)
        return point2
    def calculate_error(self,next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform):
        v_begin = self.agent.vehicle.transform.location.to_array()
        direction_vector = np.array([-np.sin(np.deg2rad(self.agent.vehicle.transform.rotation.yaw)),
                                     0,
                                     -np.cos(np.deg2rad(self.agent.vehicle.transform.rotation.yaw))])
        v_end = v_begin + direction_vector

        v_vec = np.array([(v_end[0] - v_begin[0]), 0, (v_end[2] - v_begin[2])])
        
        # calculate error projection
        w_vec = np.array(
            [
                next_waypoint.location.x - v_begin[0],
                0,
                next_waypoint.location.z - v_begin[2],
            ]
        )

        v_vec_normed = v_vec / np.linalg.norm(v_vec)
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        #error = np.arccos(v_vec_normed @ w_vec_normed.T)
        error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive
        cross = np.cross(v_vec_normed, w_vec_normed)

        if cross[1] > 0:
            error *= -1
        # calculate close error projection
        w_vec = np.array(
            [
                close_waypoint.location.x - v_begin[0],
                0,
                close_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        #wide_error = np.arccos(v_vec_normed @ w_vec_normed.T)
        wide_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive
        cross=np.cross(v_vec_normed, w_vec_normed)

        # calculate far error projection
        w_vec = np.array(
            [
                far_waypoint.location.x - v_begin[0],
                0,
                far_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        #sharp_error = np.arccos(v_vec_normed @ w_vec_normed.T)
        sharp_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive

        return error,sharp_error,wide_error,cross
    def run_in_series(self, current_speed,region,previous_waypoint:Transform, next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform, **kwargs) -> float:
        """
        Calculates a vector that represent where you are going.
        Args:
            next_waypoint ():
            **kwargs ():

        Returns:
            lat_control
        """
        # calculate a vector that represent where you are going
        # v_begin = self.agent.vehicle.transform.location.to_array()
        # direction_vector = np.array([-np.sin(np.deg2rad(self.agent.vehicle.transform.rotation.yaw)),
        #                              0,
        #                              -np.cos(np.deg2rad(self.agent.vehicle.transform.rotation.yaw))])
        # v_end = v_begin + direction_vector

        # v_vec = np.array([(v_end[0] - v_begin[0]), 0, (v_end[2] - v_begin[2])])
        
        # # calculate error projection
        # w_vec = np.array(
        #     [
        #         next_waypoint.location.x - v_begin[0],
        #         0,
        #         next_waypoint.location.z - v_begin[2],
        #     ]
        # )

        # v_vec_normed = v_vec / np.linalg.norm(v_vec)
        # w_vec_normed = w_vec / np.linalg.norm(w_vec)
        # #error = np.arccos(v_vec_normed @ w_vec_normed.T)
        # error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive
        # _cross = np.cross(v_vec_normed, w_vec_normed)

        # # calculate close error projection
        # w_vec = np.array(
        #     [
        #         close_waypoint.location.x - v_begin[0],
        #         0,
        #         close_waypoint.location.z - v_begin[2],
        #     ]
        # )
        # w_vec_normed = w_vec / np.linalg.norm(w_vec)
        # #wide_error = np.arccos(v_vec_normed @ w_vec_normed.T)
        # wide_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive

        # # calculate far error projection
        # w_vec = np.array(
        #     [
        #         far_waypoint.location.x - v_begin[0],
        #         0,
        #         far_waypoint.location.z - v_begin[2],
        #     ]
        # )
        # w_vec_normed = w_vec / np.linalg.norm(w_vec)
        # #sharp_error = np.arccos(v_vec_normed @ w_vec_normed.T)
        # sharp_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive

        # if _cross[1] > 0:
        #     error *= -1
        disttoright,disttoleft=self.lane_detector.run_in_series()
        #print(disttoright,disttoleft)
        error,sharp_error,wide_error,cross=self.calculate_error(next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint)
        toshiftstart=int(0.8*(1*round(pow(current_speed, 2)*0.002 + 0.7*current_speed)))

        max_shift=2 #originally 1
        #print(sharp_error,wide_error,region,self.to_shift)
        if (sharp_error>=0.43) and region==2 and self.to_shift==0:
            print("HERE")
            if self.to_shift==0:
                self.toshiftstart_saved=toshiftstart+1e-10
                self.to_shift=int(toshiftstart)
                shiftamt=(-1*max_shift*4)/(self.toshiftstart_saved**2)*(self.to_shift-self.toshiftstart_saved/2)**2+max_shift
            else:
                shiftamt=self.toshiftstart_saved

            self.shift_direction=0

            if cross[1]>0:
                # error-=0.3
                self.shift_direction=-1
                #next_waypoint=Transform.from_array(self.shift_waypoint(previous_waypoint.to_array(),next_waypoint.to_array(),1,0.1))
            else:
                self.shift_direction=1
                # error+=0.3
            print(self.shift_direction)
            next_waypoint=Transform.from_array(self.shift_waypoint(previous_waypoint.to_array(),next_waypoint.to_array(),self.shift_direction,shiftamt))
        elif self.to_shift>0:
            shiftamt=((-1*max_shift*4)/(self.toshiftstart_saved**2)*(self.to_shift-self.toshiftstart_saved/2)**2+max_shift)
            print(self.to_shift,shiftamt,self.shift_direction)
            if cross[1]>0:
                # error-=0.3
                next_waypoint=Transform.from_array(self.shift_waypoint(previous_waypoint.to_array(),next_waypoint.to_array(),self.shift_direction,shiftamt))
            else:
                # error+=0.3
                next_waypoint=Transform.from_array(self.shift_waypoint(previous_waypoint.to_array(),next_waypoint.to_array(),self.shift_direction,shiftamt))
            #print("ERROR UPDATED")
            # if wide_error<=0.23:
            self.to_shift-=1
        error,_,_,_=self.calculate_error(next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint)
        self._error_buffer.append(error)
        #disttocenter=self.lane_detector.run_in_series()
        # if disttocenter is None:
        #     disttocenter=0
        # self._error_buffer_center.append(disttocenter)
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            #_de_center=(self._error_buffer_center[-1] - self._error_buffer_center[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
            #_ie_center=sum(self._error_buffer_center) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        k_p, k_d,_= PIDFastController.find_k_values_formula(config=self.config, vehicle=self.agent.vehicle)
        #print(region)
        if region==1:
            _,_,k_i=PIDFastController.find_k_values(config=self.config, vehicle=self.agent.vehicle)
        else:
            #_,_,k_i=PIDFastController.find_k_values(config=self.config, vehicle=self.agent.vehicle)
            _,_,k_i=PIDFastController.find_k_values(config=self.config, vehicle=self.agent.vehicle)
        if self.to_shift>0:
            k_p*=2
            k_d/=2
            k_i=0
        if region <1000: #KG
            lat_control = float(
                np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), self.steering_boundary[0], self.steering_boundary[1])
            )
        else:
            lat_control = float(
                np.clip((k_p * disttocenter) + (k_d * _de_center) + (k_i * _ie_center), self.steering_boundary[0], self.steering_boundary[1])
            )
        # if self.to_shift>0:
        #     lat_control/=2
        #     lat_control = float(
        #         np.clip(lat_control,-0.4, 0.4)
        #     )
        return lat_control, error, wide_error, sharp_error
    