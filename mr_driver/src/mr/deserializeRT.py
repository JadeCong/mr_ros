#!/usr/bin/env python

from __future__ import print_function
import struct
import copy


class JointPos(object):
    __slots__ = ['cmd_type', 
                 'pos_joint1', 'pos_joint2', 'pos_joint3', 'pos_joint4', 'pos_joint5']
    
    @staticmethod
    def unpack(buf):
        offset = 0
        
        protocol_type = struct.unpack_from('!H', buf, offset)
        offset+=2
        print(protocol_type)
        if protocol_type != 'CO':
            print("Wrong ProtocolType: ", protocol_type)
            raise Exception("Could not unpack JointPos packet: invalid protocol type")
        
        massage_size = struct.unpack_from("!H", buf, offset)
        offset+=1
        if massage_size != 16:
            print("Wrong massage_size: ", massage_size)
            raise Exception("Could not unpack JointPos packet: invalid massage size")
        
        jp = JointPos()
        # cmd_type: 2 byte
        
        jp.cmd_type = struct.unpack_from('!', buf, 0)[0]
        offset += 1


class JointVel(object):
    __slots__ = ['cmd_type', 
                 'vel_joint1', 'vel_joint2', 'vel_joint3', 'vel_joint4', 'vel_joint5']
    
    @staticmethod
    def unpack(buf):
        offset = 0
        massage_size = struct.unpack_from()



class JointAcc(object):
    __slots__ = ['cmd_type', 
                 'acc_joint1', 'acc_joint2', 'acc_joint3', 'acc_joint4', 'acc_joint5']


class JointCur(object):
    __slots__ = ['cmd_type', 
                 'cur_joint1', 'cur_joint2', 'cur_joint3', 'cur_joint4', 'cur_joint5']


class JointErr(object):
    __slots__ = ['cmd_type', 
                 'err_joint1', 'err_joint2', 'err_joint3', 'err_joint4', 'err_joint5']


class JointStateRT(object):
    __slots__ = ['header', 'name']
    
    @staticmethod
    def unpack(buf):
        header, name, position, velocity, effort, koro_header = struct.unpack('<L16s16f16f16f', buf)
        return JointStateRT(header, name, position, velocity, effort, koro_header)


class RobotStateRT(object):
    __slots__ = ['q_target', 'qd_target', 'qdd_target', 'i_target', 'm_target', 
                 'q_actual', 'qd_actual', 'i_actual', 'i_control', 
                 'tool_vector_actual', 'tcp_speed_actual', 'tcp_force', 
                 'tool_vector_target', 'tcp_speed_target', 
                 'digital_input_bits', 'motor_temperatures', 'controller_timer', 
                 'test_value', 
                 'robot_mode', 'joint_modes', 'safety_mode']
    
    @staticmethod
    def unpack(buf):
        offset = 0
        message_size = struct.unpack_from("!i", buf, offset)[0]
        offset+=4
        if message_size != len(buf):
            print(("MessageSize: ", message_size, "; BufferSize: ", len(buf)))
            raise Exception("Could not unpack RobotStateRT packet: length field is incorrect")
        
        rs = RobotStateRT()
        #time: 1x double (1x 8byte)
        rs.time = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #q_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.q_target = copy.deepcopy(all_values)
        
        #qd_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.qd_target = copy.deepcopy(all_values)
        
        #qdd_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.qdd_target = copy.deepcopy(all_values)
        
        #i_target: 6x double (6x 8byte) 
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.i_target = copy.deepcopy(all_values)
        
        #m_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.m_target = copy.deepcopy(all_values)
        
        #q_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.q_actual = copy.deepcopy(all_values)
        
        #qd_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.qd_actual = copy.deepcopy(all_values)
        
        #i_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.i_actual = copy.deepcopy(all_values)
        
        #i_control: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.i_control = copy.deepcopy(all_values)
        
        #tool_vector_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tool_vector_actual = copy.deepcopy(all_values)
        
        #tcp_speed_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tcp_speed_actual = copy.deepcopy(all_values)
        
        #tcp_force: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tcp_force = copy.deepcopy(all_values)
        
        #tool_vector_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tool_vector_target = copy.deepcopy(all_values)
        
        #tcp_speed_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tcp_speed_target = copy.deepcopy(all_values)
        
        #digital_input_bits: 1x double (1x 8byte) ?
        rs.digital_input_bits = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #motor_temperatures: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.motor_temperatures = copy.deepcopy(all_values)
        
        #controller_timer: 1x double (1x 8byte)
        rs.controller_timer = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #test_value: 1x double (1x 8byte)
        rs.test_value = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #robot_mode: 1x double (1x 8byte)
        rs.robot_mode = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #joint_modes: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.joint_modes = copy.deepcopy(all_values)
        
        #safety_mode: 1x double (1x 8byte)
        rs.safety_mode = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #unused: 6x double (6x 8byte)
        offset+=48
        
        #tool_acc_values: 3x double (3x 8byte)
        all_values = list(struct.unpack_from("!ddd",buf, offset))
        offset+=3*8
        rs.tool_acc_values = copy.deepcopy(all_values)
        
        #unused: 6x double (6x 8byte)
        offset+=48
        
        #speed_scaling: 1x double (1x 8byte)
        rs.speed_scaling = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #linear_momentum_norm: 1x double (1x 8byte)
        rs.linear_momentum_norm = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #unused: 2x double (2x 8byte)
        offset+=16
        
        #v_main: 1x double (1x 8byte)
        rs.v_main = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #v_robot: 1x double (1x 8byte)
        rs.v_robot = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #i_robot: 1x double (1x 8byte)
        rs.i_robot = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #v_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.v_actual = copy.deepcopy(all_values)
        
        return rs
