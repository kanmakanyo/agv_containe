#!/usr/bin/env python3

import rospy
import time
import sys
import numpy as np
#from pymodbus.client import ModbusTcpClient as ModbusClient    # Python >= 3.8 
from pymodbus.client.sync import ModbusTcpClient as ModbusClient    # Python < 3.8 
from geometry_msgs.msg import PoseWithCovarianceStamped

# -------DEFINE CONSTANTS---------
# PLC ADDRESSES
PLC_IP = '192.168.1.10'
# PLC_IP = '192.168.0.20'
PLC_PORT = 502
UNIT = 0x1
# CONTROL SIGNAL BOUNDARIES
STEER_MIN = np.degrees(-0.6)    
STEER_MAX = np.degrees(0.6)
THROTTLE_MAX = 100  
THROTTLE_MIN = 0 
THROTTLE_BIT_MAX = 4095
THROTTLE_BIT_MIN = 0
ENC_BIT_MIN = 0 
ENC_BIT_MAX = 1023
BRAKE_MAX = 100 
BRAKE_MIN = 0 
# DESIRED_FREQ = 20 #HZ
# SLEEP_PERIOD = 1/DESIRED_FREQ
# LOW LEVEL CONTROL SIGNAL OUTPUTS
TURN_LEFT = 1           # Steering code accepted by PID 
TURN_RIGHT = 10    
IDLE = 0
# BRAKE_ON = 1            # Braking code accepted by PID
# BRAKE_OFF = 10          # Brake using Handbrake
# BRAKE_IDLE = 0
BRAKE_ON = 1 
BRAKE_OFF = 0           # Brake using Brake Regen
BRAKE_TIME = 5          # Linear motor working time
# Propultion : 0 - 100 %
CONTAINER_LOCK_ON = True    
CONTAINER_LOCK_OFF = False
# EMERGENCY_ON = 1
# EMERGENCY_OFF = 0
PROP_FORWARD = 0 
PROP_BACKWARD = 1
CHANGE_DIR_ON = 11 
CHANGE_DIR_OFF = BRAKE_OFF 
# PLC MEMORY ADDRESSES
STEER_PLC_ADR = 2 
BRAKE_PLC_ADR = 3      # PLC Variable Address
CHANGE_DIR_PLC_ADR = BRAKE_PLC_ADR
PROP_PLC_ADR = 4
CONT_LOCK_PLC_ADR = 5
ENC_PLC_ADR = 6
TACHO_PLC_ADR = 7 
EMERGENCY_PLC_ADR = 8

class PLCGateway():
    def __init__(self, PLC_IP, PLC_PORT, UNIT):
        self.PLC_IP = PLC_IP
        self.PLC_PORT = PLC_PORT
        self.client = ModbusClient(self.PLC_IP, self.PLC_PORT)
        self.UNIT = UNIT
        print('PLC Gateway initialized : client created')

    def connect_client(self):
        self.client.connect()
        print('\t\tclient connected')

    def close_client(self):
        self.client.close()
        print('\t\tclient closed')

    # def read_encoder(self):
    #     """
    #     Choose one of the encoding method below 
    #     if gray code, use the enc_gray_converted variable
    #     if binary code, use the enc_binary variable
    #     """
    #     # test_write = self.client.write_registers(6, 5643, unit=UNIT)
    #     self.connect_client()
    #     # binary
    #     enc_read = self.client.read_holding_registers(6, 1, unit=UNIT)
    #     enc_binary = enc_read.registers[0]
    #     # gray code 
    #     # enc_gray_converted = gray_convert(enc_read)
    #     print(enc_binary)
    #     print('read encoder finished')
    #     self.close_client()
    #     return enc_binary
    
    def read_encoder_v2(self):
        """
        Choose one of the encoding method below 
        if gray code, use the enc_gray_converted variable
        if binary code, use the enc_binary variable
        """
        self.connect_client()
        enc_read = self.client.read_holding_registers(ENC_PLC_ADR, 1, unit=self.UNIT)
        enc_binary = enc_read.registers[0]
        print("\tEncoder Value :",enc_binary)
        self.close_client()
        return enc_binary
    
    def read_emergency_status(self):
        """
        Monitor emergeny status from PLC
        """
        self.connect_client()
        emergency = self.client.read_holding_registers(EMERGENCY_PLC_ADR, 1, unit=self.UNIT)
        emergency_val = bool(emergency.registers[0])
        print('Emergency Status :', emergency_val)
        self.close_client()
        return emergency_val
    
    # def send_steer_cmd_v1(self,steer_cmd,adr=2):
    #     """
    #     Convert the steer command to the acceptable form
    #     steer_cmd variable is a string which can be 'L' or 'R' or 'I'
    #         L stands for Left Turn, R for Right Turn, I or Idle
    #     The acceptable form of command to plc are : 
    #          10 for left turn || 01 or just 1 for right turn || 00 or 0 for idle
    #     """
    #     if steer_cmd not in ['R', 'L', 'I']:
    #         raise Exception('steer command is not valid, use either "R", "L", or "I"')
        
    #     self.connect_client()
        
    #     if steer_cmd == 'R':
    #         write = self.client.write_registers(adr, TURN_RIGHT, unit=self.UNIT)
    #         print('turn right sent')
    #     elif steer_cmd == "L":
    #         write = self.client.write_registers(adr, TURN_LEFT, unit=self.UNIT)
    #         print('turn left sent')
    #     else : 
    #         write = self.client.write_registers(adr, IDLE, unit=self.UNIT)
    #         print('idle command sent')

    #     self.close_client()

    def send_steer_cmd_v2(self,steer_cmd,adr=STEER_PLC_ADR):
        """
        The acceptable form of command to plc are : 
             10 for left turn || 01 or just 1 for right turn || 00 or 0 for idle
        commands already defined as constants
        """
        if steer_cmd not in [TURN_LEFT, TURN_RIGHT, IDLE]:
            s = '''Steer command is not valid\nUse either TURN_LEFT, TURN_RIGHT, or IDLE'''
            raise Exception(s)
        self.connect_client()
        write = self.client.write_registers(adr, steer_cmd, unit=self.UNIT)
        if steer_cmd == TURN_LEFT :
            print(f"\tsteer command [TURN_LEFT] sent") 
        elif steer_cmd == TURN_RIGHT:
            print(f"\tsteer command [TURN_RIGHT] sent")
        else : 
            print(f"\tsteer command [IDLE] sent")
        self.close_client()

    # def send_propultion_cmd_v1(self,prop_cmd,adr=4):
    #     if prop_cmd < 0 or prop_cmd > 1023 : 
    #         raise Exception('propulsion command is invalid, range must between 0-1024')
    #     self.connect_client()
    #     write = self.client.write_registers(adr, prop_cmd, unit=UNIT)
    #     print('propultion command sent')
    #     self.close_client()

    def send_propultion_cmd_v2(self,prop_cmd,adr=PROP_PLC_ADR):
        if prop_cmd < THROTTLE_BIT_MIN or prop_cmd > THROTTLE_BIT_MAX:
            raise Exception(f'Propulsion command is invalid, range must between {THROTTLE_BIT_MIN}-{THROTTLE_BIT_MAX}')
        self.connect_client()
        write = self.client.write_registers(adr, prop_cmd, unit=self.UNIT)
        print(f'\tpropultion command {prop_cmd} sent')
        self.close_client()

    # def send_brake_cmd_v1(self,brake_cmd,adr=3):
    #     brake_cmd = bool(brake_cmd)
    #     self.connect_client()
    #     write = self.client.write_registers(adr, brake_cmd, unit=UNIT)
    #     print('brake command sent')
    #     self.close_client()

    # def send_brake_cmd_v2(self,brake_cmd,adr=BRAKE_PLC_ADR):
    #     if brake_cmd not in [BRAKE_OFF, BRAKE_ON, BRAKE_IDLE]:
    #         raise Exception('Brake command invalid\nUse either BRAKE_OFF or BRAKE_ON or BRAKE_IDLE')
    #     self.connect_client()
    #     write = self.client.write_registers(adr, brake_cmd, unit=self.UNIT)
    #     if brake_cmd == BRAKE_ON :
    #         print(f"\tbrake command [BRAKE_ON] sent") 
    #     elif brake_cmd == BRAKE_OFF :
    #         print(f"\tbrake command [BRAKE_OFF] sent")
    #     else : 
    #         print(f"\tbrake command [BRAKE_IDLE] sent")
    #     self.close_client()

    def send_brake_cmd_v3(self,brake_cmd,adr=BRAKE_PLC_ADR):
        if brake_cmd not in [BRAKE_OFF, BRAKE_ON]:
            raise Exception('Brake command invalid\nUse either BRAKE_OFF or BRAKE_ON')
        self.connect_client()
        write = self.client.write_registers(adr, brake_cmd, unit=self.UNIT)
        if brake_cmd == BRAKE_ON :
            print(f"\tbrake command [BRAKE_ON] sent") 
        else :
            print(f"\tbrake command [BRAKE_OFF] sent")
        self.close_client()

    def send_change_dir_cmd_v1(self,adr=CHANGE_DIR_PLC_ADR):
        self.connect_client()
        write = self.client.write_registers(adr, CHANGE_DIR_ON, unit=self.UNIT)
        print("Sending change direction to propultion... ")
        time.sleep(6)
        write = self.client.write_registers(adr, CHANGE_DIR_OFF, unit=self.UNIT)
        self.close_client()
    
    # def send_container_lock_cmd_v1(self,cont_cmd,adr=5):
    #     cont_cmd = bool(cont_cmd)
    #     self.connect_client()
    #     write = self.client.write_registers(adr, cont_cmd, unit=UNIT)
    #     print('container lock command sent')
    #     self.close_client()

    def send_container_lock_cmd_v2(self,cont_cmd,adr = CONT_LOCK_PLC_ADR):
        if cont_cmd not in [CONTAINER_LOCK_ON, CONTAINER_LOCK_OFF]:
            raise Exception('Container lock command invalid\nUse either CONTAINER_LOCK_ON or CONTAINER_LOCK_OFF')
        write = self.client.write_registers(adr, cont_cmd, unit=self.UNIT)
        if cont_cmd == CONTAINER_LOCK_ON :
            print(f"\tbrake command [CONTAINER_LOCK_ON] sent") 
        else:
            print(f"\tbrake command [CONTAINER_LOCK_OFF] sent")
        self.close_client()

    def __str__(self):
        s = f"""PLC_IP:{self.PLC_IP}\nPLC_PORT:{self.PLC_PORT}\nUNIT:{self.UNIT}"""
        return s

class ControllerLow():
    def __init__(self,gateway,enc_center=0):
        self.gateway = gateway
        if enc_center < ENC_BIT_MIN or enc_center > ENC_BIT_MAX : 
            raise Exception('encoder center identifier is not valid, must be between 0-1023')
        self.enc_center = enc_center
        self.enc_val = self.gateway.read_encoder_v2()
        self.enc_deg = self.bit_to_deg(self.enc_val)
        self.steer_cmd_log = IDLE
        self.prop_cmd_log = 0
        self.handbrake_now = 0 
        self.brake_cmd_log = BRAKE_OFF
        self.brake_start_time = None
        self.prop_direction = PROP_FORWARD
        print('Low Controller Initialized')

    # def control_steer_v1(self, steer_setpoint, pass_band=2):
    #     """
    #     Low level control for steering 
    #     The input are constant angular velocity to either direction or idle
    #     Encoder is used as feedback 
    #     Conventions are : 
    #          - Clockwise rotation is positive
    #          - Counterlockwise rotation is negative
    #          - Turn Left is clockwise
    #          - Turn Right is counterclockwise
    #     """
    #     while(True):
    #         enc_val = self.gateway.read_encoder()
    #         enc_deg = self.counter_to_deg(enc_val)
    #         print(enc_deg)
    #         err = steer_setpoint - enc_deg 
            
    #         if err > 0 and err < pass_band : 
    #             self.gateway.send_steer_cmd_v1(IDLE)
    #             # time.sleep(0.5)
    #             print('Setpoint Reached')
    #             break
    #         elif err > pass_band : 
    #             self.gateway.send_steer_cmd_v1(TURN_LEFT)
    #             time.sleep(0.05)
    #         elif err < -pass_band : 
    #             self.gateway.send_steer_cmd_v1(TURN_RIGHT)
    #             time.sleep(0.05)

    def control_steer_v2(self,steer_setpoint,pass_band=0.5):
        """
        Low level control for steering 
        The input are constant angular velocity to either direction or idle
        Encoder is used as feedback 
        Conventions are : 
             - Clockwise rotation is positive
             - Counterlockwise rotation is negative
             - Turn Left is clockwise
             - Turn Right is counterclockwise
        """
        # while(True): # ROS already provide while loop
        self.enc_val = self.gateway.read_encoder_v2()
        self.enc_deg = self.bit_to_deg(self.enc_val)
        print("Steer Actual :", self.enc_deg)
        err = steer_setpoint - self.enc_deg 
        print("Error Steer :", err)
        if err >= 0 and err <= pass_band : 
            self.gateway.send_steer_cmd_v2(IDLE)
            # time.sleep(0.5)
            print('\t\tSETPOINT REACHED')
            #break
            self.steer_cmd_log = IDLE
        elif err > pass_band : 
            self.gateway.send_steer_cmd_v2(TURN_LEFT)
            #time.sleep(SLEEP_PERIOD) # rospy.Rate already done the job
            self.steer_cmd_log = TURN_LEFT
        elif err < -pass_band : 
            self.gateway.send_steer_cmd_v2(TURN_RIGHT)
            #time.sleep(SLEEP_PERIOD)
            self.steer_cmd_log = TURN_RIGHT

    def control_steer_vTEST(self, steer_setpoint, pass_band=0.5):
        """
        Low level control for steering 
        The input are constant angular velocity to either direction or idle
        Encoder is used as feedback 
        Conventions are : 
             - Clockwise rotation is positive
             - Counterlockwise rotation is negative
             - Turn Left is clockwise
             - Turn Right is counterclockwise
        """
        emergency_status = 0 
        while(True):
            if emergency_status == 0 : 
                # emergency_status = self.gateway.read_emergency_status()
                enc_val = self.gateway.read_encoder_v2()
                enc_deg = self.bit_to_deg(enc_val)
                print(enc_deg)
                err = steer_setpoint - enc_deg 
                
                if err > 0 and err < pass_band : 
                    self.gateway.send_steer_cmd_v2(IDLE)
                    # time.sleep(0.5)
                    print('\t\tSetpoint Reached')
                    break
                elif err > pass_band : 
                    self.gateway.send_steer_cmd_v2(TURN_LEFT)
                    # time.sleep(0.1)
                elif err < -pass_band : 
                    self.gateway.send_steer_cmd_v2(TURN_RIGHT)
                    # time.sleep(0.1)
            else : 
                self.control_emergency()
            
    # def counter_to_deg(self, counter):
    #     delta = counter - self.enc_center
    #     if delta > 512 : # 512 is a half of positive full turn 
    #         delta = delta - 1024
    #     elif delta < -512 : # -512 is a half of negative full turn 
    #         delta = delta + 1024
    #     elif delta == -512 : # because -512 and 512 is equal, for precaution only
    #         delta = 512
    #     deg = delta/1024 * 360 
    #     return deg
    
    def bit_to_deg(self,bit):
        delta = bit - self.enc_center
        if delta > 512 : # 512 is a half of positive full turn 
            delta = delta - 1024
        elif delta < -512 : # -512 is a half of negative full turn 
            delta = delta + 1024
        elif delta == -512 : # because -512 and 512 is equal, for precaution only
            delta = 512
        deg = delta/1024 * 360 
        return deg

    # def control_propultion_v1(self, power_level):
    #     """
    #     Low level open loop controller for propultion
    #     The controlled variable is propultion power percentage
    #     controller input must be between 0-100
    #     Controller output ranges from 0 - 1024 (10 bit integer)
    #     0-1024 will be converted to 0-5V input for motor driver by PLC 
    #     """
    #     if power_level < 0 or power_level > 100 : 
    #         raise Exception('low controller input is invalid must be between 0-100')
    #     prop_cmd = round(power_level/100*4096)
    #     self.gateway.send_propultion_cmd_v1(prop_cmd)

    def control_propultion_v2(self,power_level):
        prop_cmd = round(power_level/THROTTLE_MAX*THROTTLE_BIT_MAX)
        self.gateway.send_propultion_cmd_v2(prop_cmd)
        self.prop_cmd_log = prop_cmd

    def control_propultion_vTEST(self,power_level,limit):
        prop_cmd = round(power_level/THROTTLE_MAX*THROTTLE_BIT_MAX*limit)
        self.gateway.send_propultion_cmd_v2(prop_cmd)
        self.prop_cmd_log = prop_cmd
    
    # def control_brake_v1(self, brake_cmd):
    #     self.gateway.send_brake_cmd_v1(brake_cmd)

    def control_brake_v2(self,handbrake_setpoint): 
        if handbrake_setpoint > 50 : 
            self.gateway.send_brake_cmd_v3(BRAKE_ON)
            time.sleep(5)
            # self.gateway.send_brake_cmd_v2(BRAKE_IDLE)
        else : 
            self.gateway.send_brake_cmd_v3(BRAKE_OFF)
            time.sleep(5)
            # self.gateway.send_brake_cmd_v2(BRAKE_IDLE)

    # def control_brake_v3(self,handbrake_setpoint):
    #     if self.brake_not_yet_start() or time.time() - self.brake_start_time > BRAKE_TIME: 
    #         self.gateway.send_brake_cmd_v2(BRAKE_IDLE)
    #         self.brake_cmd_log = BRAKE_IDLE
    #     if self.handbrake_now != handbrake_setpoint : 
    #         if handbrake_setpoint > 50 : 
    #             self.brake_start_time = time.time()
    #             self.gateway.send_brake_cmd_v2(BRAKE_ON)
    #             self.handbrake_now = handbrake_setpoint
    #             self.brake_cmd_log = BRAKE_ON
    #         else :
    #             self.brake_start_time = time.time()
    #             self.gateway.send_brake_cmd_v2(BRAKE_OFF)
    #             self.handbrake_now = handbrake_setpoint
    #             self.brake_cmd_log = BRAKE_OFF
    # def brake_not_yet_start(self):
    #     return self.brake_start_time == None

    def control_brake_v4(self,handbrake_setpoint):
        if handbrake_setpoint > 50 : 
            self.gateway.send_brake_cmd_v3(BRAKE_ON)
        else : 
            self.gateway.send_brake_cmd_v3(BRAKE_OFF)

    def control_change_prop_direction_v1(self):
        if self.prop_direction == PROP_FORWARD: 
            self.prop_direction == PROP_BACKWARD
        else : 
            self.prop_direction == PROP_FORWARD
        self.gateway.send_change_dir_cmd_v1()

    # def control_container_lock_v1(self,cont_cmd):
    #     self.gateway.send_container_lock_cmd_v1(cont_cmd)
    def control_emergency(self):
        self.gateway.send_propultion_cmd_v2(0)
        self.gateway.send_steer_cmd_v2(IDLE)
        self.prop_cmd_log = 0
        self.steer_cmd_log = IDLE
        self.brake_cmd_log = BRAKE_OFF

    def control_container_lock_v2(self,cont_cmd):
        self.gateway.send_container_lock_cmd_v2(cont_cmd)

# def test_input_steering(gateway):
#     for _ in range(3): 
#         gateway.send_steer_cmd_v1("L")
#         time.sleep(5)
#         gateway.send_steer_cmd_v1("I")
#         time.sleep(3)
#         gateway.send_steer_cmd_v1("R")
#         time.sleep(5)
#         gateway.send_steer_cmd_v1("I")
#         time.sleep(3)

# -------------[ROS NODE INITIALIZATION]--------------------
# rospy.init_node('low_controller')
# enc_center = rospy.get_param('~enc_center', 84) # value of 84 gained from testing
# prop_limit = rospy.get_param('~prop_limit', 1) 
# freq = rospy.get_param('~freq', 20)
# STEER_MIN = rospy.get_param('~steer_min',-0.6)
# STEER_MAX = rospy.get_param('~steer_max',0.6)
# operation_mode = rospy.get_param("~operation_mode",0)
# # Operation mode : 
# # 0 = Path Following 
# # 1 = Steering Test (MONEV)
# # 2 = Propultion Test (MONEV)
# print('ROS parameters initialized')
# pub = rospy.Publisher('logging_PLC', PoseWithCovarianceStamped, queue_size=1)
# rate = rospy.Rate(freq) # Hz
# if operation_mode == 0 :
#     rospy.Subscriber('/control_signal', PoseWithCovarianceStamped, callback_cs)
# print('ROS node Initialized')
#-----------------------------------------------------------


class LLRuntime():
    def __init__(self):
        # VARIABLE INITIALIZATION 
        self.steer_setpoint = 0 
        self.throttle_setpoint = THROTTLE_MIN
        self.hanbrake_setpoint = 0
        self.cont_cmd = CONTAINER_LOCK_OFF
        self.emergency_status = 0

        #---------ROS SETUP---------------
        rospy.init_node('low_controller')
        rospy.Subscriber('/control_signal', PoseWithCovarianceStamped, self.callback_cs)
        self.pub = rospy.Publisher('logging_PLC', PoseWithCovarianceStamped, queue_size=1)
        print('ROS node Initialized')
        self.enc_center = rospy.get_param('~enc_center', 84) # value of 84 gained from testing 
        self.steer_pass_band = rospy.get_param('~steer_pass_band',0.5)
        freq = rospy.get_param('~freq', 20)
        self.rate = rospy.Rate(freq) # Hz
        print('ROS parameters initialized')

        #------- LOW LEVEL CONTROLLER SETUP--------
        self.PLCgateway = PLCGateway(PLC_IP=PLC_IP, PLC_PORT=PLC_PORT, UNIT=UNIT)
        self.LLC = ControllerLow(self.PLCgateway, enc_center=self.enc_center)

        self.pub_msg = PoseWithCovarianceStamped()
        self.pub_msg.header.frame_id = 'Logging PLC'
        self.pub_msg.header.seq = 0
        self.pub_msg.header.stamp = rospy.Time.now()

        while not rospy.is_shutdown():
    
            if self.emergency_status == 0 : 

                self.LLC.control_steer_v2(steer_setpoint=self.steer_setpoint,pass_band=self.steer_pass_band)
                self.LLC.control_propultion_v2(power_level=self.throttle_setpoint)
                self.LLC.control_brake_v4(brake_cmd=self.handbrake_setpoint)
                self.emergency_status = self.PLCgateway.read_emergency_status()
                # LLC.control_container_lock_v2(cont_cmd=cont_cmd) # hasn't been implemented
            else : 
                self.LLC.control_emergency()
        
            ### Send the message
            # Header
            self.pub_msg.header.stamp = rospy.Time.now()
            self.pub_msg.header.seq += 1
            self.pub_msg.pose.covariance[0] = np.radians(self.steer_setpoint) # rad
            self.pub_msg.pose.covariance[1] = self.LLC.steer_cmd_log          # 1 or 10 or 0 
            self.pub_msg.pose.covariance[2] = self.LLC.enc_val                # 0 - 1023 
            self.pub_msg.pose.covariance[3] = np.radians(self.LLC.enc_deg)    # rad
            self.pub_msg.pose.covariance[4] = self.enc_center                 # encoder value when steering 0 radian
            self.pub_msg.pose.covariance[5] = self.steer_pass_band            # pass_band to prevent jitter
            self.pub_msg.pose.covariance[6] = self.throttle_setpoint          # 0 - 100 %
            self.pub_msg.pose.covariance[7] = self.LLC.prop_cmd_log           # 0 - 4095
            self.pub_msg.pose.covariance[8] = self.handbrake_setpoint         # 0 or 100; handbrake control signal 
            self.pub_msg.pose.covariance[9] = self.LLC.handbrake_now          # 0 or 100; handbrake state now
            self.pub_msg.pose.covariance[10] = self.LLC.brake_log_now         # 1 or 10 or 0; linear actuator state
            self.pub_msg.pose.covariance[11] = self.emergency_status          # 1 or 0 

            self.rate.sleep()

    def callback_cs(self,sub_msg):
        self.steer_setpoint = np.degrees(sub_msg.pose.covariance[0])
        self.throttle_setpoint = sub_msg.pose.covariance[1]
        self.handbrake_setpoint = sub_msg.pose.covariance[2]

        self.speed_control_mode = sub_msg.pose.covariance[8]
        self.speed_setpoint = sub_msg.pose.covariance[9]


class RuntimeTestingWithoutROS():
    def __init__(self,steer_pass_band=0.5,enc_center=84):
        self.enc_center = enc_center # value of 84 gained from testing 
        self.steer_pass_band = steer_pass_band
        self.PLCgateway = PLCGateway(PLC_IP=PLC_IP, PLC_PORT=PLC_PORT, UNIT=UNIT)
        self.LLC = ControllerLow(self.PLCgateway, enc_center=self.enc_center)

    def runtime_test(self): 
        self.steer_max_min()
        time.sleep(5)
        self.propulsion_max_min_brake()

    def runtime_uji_steer_control(self):
        print('[STEER CONTROL TEST START]')
        print('Turning right to 20 degrees position...')
        self.LLC.control_steer_vTEST(-20)
        time.sleep(2)
        print('Turning left 20 degrees position...')
        self.LLC.control_steer_vTEST(20)
        time.sleep(2)
        print('Turn right to 0 degree position...')
        self.LLC.control_steer_vTEST(0)
        print('[STEER CONTROL FINISHED]')

    def runtime_uji_propultion(self):
        print('[PROPULTION CONTROL TEST START]')
        self.LLC.control_propultion_vTEST(power_level=THROTTLE_MAX,limit=0.5)
        print('AGV move forward...')
        time.sleep(3)
        self.LLC.control_propultion_vTEST(power_level=0,limit=0.5)
        self.LLC.control_brake_v4(handbrake_setpoint=100) 
        print('Stopped by brake...')
        time.sleep(5)
        self.LLC.control_brake_v2(handbrake_setpoint=0)
        # self.LLC.control_change_prop_direction_v1() 
        # print('Gear set to reverse...')
        # self.LLC.control_propultion_vTEST(power_level=THROTTLE_MAX,limit=0.25)
        # print('AGV move backward...')
        # time.sleep(3)
        # self.LLC.control_propultion_vTEST(power_level=0,limit=0.25)
        # self.LLC.control_brake_v2(handbrake_setpoint=100) 
        # print('Stopped by brake...')
        # time.sleep(2)
        # self.LLC.control_brake_v2(handbrake_setpoint=0)
        # self.LLC.control_change_prop_direction_v1() 
        # print('Gear set to forward...')
        print('[PROPULTION CONTROL TEST FINISHED]')

    def steer_max_min(self):
        self.LLC.control_steer_v1(steer_setpoint=STEER_MIN,pass_band=self.steer_pass_band)
        time.sleep(1)
        self.LLC.control_steer_v1(steer_setpoint=STEER_MAX,pass_band=self.steer_pass_band)
        time.sleep(1)

    def propulsion_max_min_brake(self):
        self.LLC.control_propultion_vTEST(power_level=THROTTLE_MAX,limit=0.25)
        time.sleep(3)
        self.LLC.control_propultion_vTEST(power_level=0,limit=0.25)
        self.LLC.control_brake_v2(handbrake_setpoint=100) 
        self.LLC.control_brake_v2(handbrake_setpoint=0) 

    def turn_steer_one_time(self,cmd): 
        print(f"TURN ONE TIME TO {cmd}")
        self.LLC.control_steer_vTEST(cmd)

    def throttle_one_time(self,target,scale):
        print(f"Propulsion {target}% scaled to {scale*100}%")
        self.LLC.control_propultion_vTEST(power_level=target, limit=scale)
        time.sleep(5)
        self.LLC.control_propultion_vTEST(power_level=0,limit=scale)
        print("Propultion stop")



# TESTING
runtime_testing = RuntimeTestingWithoutROS(enc_center=104)
print('=============[TEST]====================')
# runtime_testing.runtime_uji_steer_control()
# runtime_testing.turn_steer_one_time(0)
# runtime_testing.PLCgateway.read_encoder_v2()
# runtime_testing.throttle_one_time(100,0.25)
# runtime_testing.runtime_uji_propultion()
# runtime_testing.PLCgateway.send_brake_cmd_v3(BRAKE_OFF)
print("================================================")
# print('=============[PROPULTION TEST]==================')
# runtime_testing.runtime_uji_propultion()
# print("================================================")
# RUNTIME 
# runtime = LLRuntime()