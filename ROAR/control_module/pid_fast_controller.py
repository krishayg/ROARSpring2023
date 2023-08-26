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
        
        # useful variables
        self.turn_number=0
        self.region = 2 #################################################################3################CHANGE THIS!!!!!!!!!!!!!!!!!!!!!!!!!
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
            #print(len(self.waypoint_queue_braking))
            while starting_point>=10 and len(self.waypoint_queue_braking)>2:
                print(len(self.waypoint_queue_braking))
                self.waypoint_queue_braking.pop(0)
        with open("ROAR\\control_module\\turning_list.txt") as f:
            for line in f:
                raw = line.split(",")
                waypoint = Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]), rotation=Rotation(pitch=0, yaw=0, roll=0))
                self.waypoint_queue_turning.append(waypoint)
            if starting_point>=3:
            #     closest_waypoint = self.waypoint_queue_turning[0]
            #     for waypoint in self.waypoint_queue_turning:
            #         cur_dist = self.agent.vehicle.transform.location.distance(waypoint.location)
            #         closest_dist = self.agent.vehicle.transform.location.distance(closest_waypoint.location)
            #         if  cur_dist < closest_dist:
            #             closest_waypoint = waypoint
            #     while self.waypoint_queue_turning[0] != closest_waypoint:
            #         print("DELETING TURN")
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
            #print(previous_waypoint.location.to_array())
        else:
            previous_waypoint=next_waypoint
            
        current_speed = Vehicle.get_speed(self.agent.vehicle)
        waypoint=self.waypoint_queue_shifting[0]
        dist = self.agent.vehicle.transform.location.distance(waypoint.location)
        shift_manual=0
        #print(dist)
        if dist<=5:
            self.waypoint_queue_shifting.pop(0)
            shift_manual=1
        
        waypoint=self.waypoint_queue_sub_region[0]
        dist = self.agent.vehicle.transform.location.distance(waypoint.location)
        if dist<=5:
            print("SWITCHING SUBREGION")
            self.waypoint_queue_sub_region.pop(0)
            self.sub_region+=1
        # run lat pid controller
        steering, error, wide_error, sharp_error,track_error = self.lat_pid_controller.run_in_series(sub_region=self.sub_region,shift_manual=shift_manual,current_speed=current_speed,region=self.region, previous_waypoint=previous_waypoint,next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint,close_waypoint_next=close_waypoint_next,close_waypoint_track=close_waypoint_track,turn_number=self.turn_number)
        
        self.lane_detector =LaneDetector(agent=self.agent)
        #print(self.lane_detector.run_in_series())
        
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
        #print(self.region)
        sharp_error_threshold=0.68
        speed_threshold=90
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
        #print(len(self.waypoint_queue_braking))
        if self.region ==1:
            #if sharp_error < sharp_error_threshold or current_speed <= speed_threshold:#0.68
            if sharp_error < sharp_error_threshold or current_speed <= speed_threshold:#0.68
                throttle = 1
                brake = 0
                hand_brake=0
            else:
                throttle = -1
                brake = 1
                hand_brake=1
        elif self.region == 3:
            #if sharp_error < sharp_error_threshold or current_speed <= speed_threshold:#0.68
            if sharp_error > sharp_error_threshold and current_speed >= speed_threshold:#0.68
                throttle = -1
                brake = 1
                hand_brake=1
                print("TURNING")
            elif sharp_error >= 0.7 and current_speed > 115: #originally 0.65, 80
                throttle = 0
                brake =0.4#current_speed/400#0.4
            elif wide_error > 0.09 and current_speed > 100: # wide turn  #originally 0.09,92
                #print(wide_error/track_error)
                #throttle = max(0, 1 - 6*pow(track_error*0.27+wide_error*0.53 + current_speed*0.0027, 6))
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
            
        elif self.region ==2:
            waypoint = self.waypoint_queue_braking[0][0] # 5012 is weird bump spot
            dist = self.agent.vehicle.transform.location.distance(waypoint.location)
            if dist <= 5:
                self.brake_counter = 1
                self.waypoint_queue_braking.pop(0)
            if self.brake_counter > 0:
                print("BRAKING")
                throttle = -1
                brake = 1
                hand_brake=0 #KG
                self.brake_counter += 1
                if self.brake_counter >= int(self.waypoint_queue_braking[0][1]):#4: #KG
                    self.brake_counter = 0
            elif sharp_error >= 0.67 and current_speed > 81: #originally 0.65, 80
                throttle = 0
                brake =current_speed/400
            elif wide_error > 0.09 and current_speed > 95: # wide turn  #originally 0.09,92
                #print(wide_error/track_error)
                #throttle = max(0, 1 - 6*pow(track_error*0.27+wide_error*0.53 + current_speed*0.0027, 6))
                speed_multiplier=0.00256
                if self.sub_region in [0,2]:
                    speed_multiplier=0.00258
                elif self.sub_region==1:
                    speed_multiplier=0.00225
                if self.sub_region==3:
                    speed_multiplier=0.00233
                elif self.sub_region==4:
                    speed_multiplier=0.0025
                elif self.sub_region==5:
                    speed_multiplier=0.0023
                throttle = max(0, 1 - 6*pow(wide_error*0.9 + current_speed*speed_multiplier, 6)) #0.92 and 0.00274 #0.00264 works #0.00256
                brake = 0
            else:
                throttle = 1
                brake = 0

        elif self.region==4:
            waypoint = self.waypoint_queue_braking[0][0] # 5012 is weird bump spot
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
                if self.brake_counter >= 10:#4: #KG
                    self.brake_counter = 0
            elif sharp_error >=0.45 and current_speed > 75: #originally 0.65, 80
                throttle = 0
                brake = 0.7#min(0.4,sharp_error/2.3)#0.4
            elif wide_error > 0.09 and current_speed > 92: # wide turn  #originally 0.09,92
                throttle = max(0, 1 - 6*pow(wide_error + current_speed*0.003, 6))
                brake = 0
            else:
                throttle = 1
                brake = 0
        elif self.region==1000: #KG switching back to city
            if sharp_error < 0.68 or current_speed <= 90:#0.68
                throttle = 1
                brake = 0
                hand_brake=0
            else:
                throttle = -1
                brake = 1
                hand_brake=1
        #print(self.turn_number)
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
        self.after_shift=0
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
    def calculate_error(self,previous_waypoint: Transform,next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform,close_waypoint_next:Transform,close_waypoint_track:Transform):
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
        cur_waypoints_vec=np.array(
            [
                next_waypoint.location.x - previous_waypoint.location.x,
                0,
                next_waypoint.location.z - previous_waypoint.location.z,
            ]
        )
        #close_waypoint=close_waypoint_track
        next_waypoints_vec=np.array(
            [
                close_waypoint_next.location.x - close_waypoint_track.location.x,
                0,
                close_waypoint_next.location.z - close_waypoint_track.location.z,
            ]
        )
        next_waypoints_vec = next_waypoints_vec / np.linalg.norm(next_waypoints_vec)
        cur_waypoints_vec=cur_waypoints_vec/np.linalg.norm(cur_waypoints_vec)
        track_error=np.arccos(min(max(cur_waypoints_vec @ next_waypoints_vec.T, -1), 1))
        #print(track_error)
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
        error,sharp_error,wide_error,cross,track_error=self.calculate_error(previous_waypoint=previous_waypoint,next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint,close_waypoint_next=close_waypoint_next,close_waypoint_track=close_waypoint_track)
        toshiftstart=int((0.9*round(pow(current_speed, 2)*0.002 + 0.7*current_speed)))
        #print(turn_number)
        if region==2 or turn_number!=5:
            max_shift=max(2.5,1/(current_speed+1e-16)*225) #3.0,175 #originally 1
        elif turn_number==5:
            max_shift=1.5#max(2,1/(current_speed+1e-16)*60)
        else:
            max_shift=0
        #print(sharp_error,wide_error,region,self.to_shift)
        #print(shift_manual)
        if ((track_error>=0.75) and region in [2,3] and self.to_shift==0) or shift_manual: #originally 0.7
            print("HERE")
            if self.to_shift==0:
                self.track_error_start=track_error
                self.after_shift=10 #10
                self.max_shift_saved=max_shift
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
            #print(self.shift_direction)
            next_waypoint=Transform.from_array(self.shift_waypoint(previous_waypoint.to_array(),next_waypoint.to_array(),self.shift_direction,shiftamt))
        elif self.to_shift>0:
            shiftamt=((-1*self.max_shift_saved*4)/(self.toshiftstart_saved**2)*(self.to_shift-self.toshiftstart_saved/2)**2+self.max_shift_saved)
            #print(self.to_shift,shiftamt,self.shift_direction)
            if cross[1]>0:
                # error-=0.3
                next_waypoint=Transform.from_array(self.shift_waypoint(previous_waypoint.to_array(),next_waypoint.to_array(),self.shift_direction,shiftamt))
            else:
                # error+=0.3
                next_waypoint=Transform.from_array(self.shift_waypoint(previous_waypoint.to_array(),next_waypoint.to_array(),self.shift_direction,shiftamt))
            #print("ERROR UPDATED")
            # if wide_error<=0.23:
            self.to_shift-=1
        
        error,_,_,_,_=self.calculate_error(previous_waypoint=previous_waypoint,next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint,close_waypoint_next=close_waypoint_next,close_waypoint_track=close_waypoint_track)
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
        if self.to_shift>0 and self.to_shift<self.toshiftstart_saved:
           # print(self.track_error_start)
           if sub_region not in [2,5]:
                k_p*=2 #1.65          #self.track_error_start*2
                k_d=k_d/2        #(self.track_error_start*2)
                #k_i=0
           else:
                k_p*=1.6         #self.track_error_start*2
                k_d=k_d/2.6   
        elif self.to_shift==0 and self.after_shift>0:
            k_p=k_p/2#(self.track_error_start*3)
            self.after_shift-=1
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
        return lat_control, error, wide_error, sharp_error,track_error
    