#!/usr/bin/env python

from __future__ import print_function
import struct
import copy


class RobotModeData(object):
    __slots__ = ['timestamp', 'robot_connected', 'real_robot_enabled',
                 'power_on_robot', 'emergency_stopped',
                 'security_stopped', 'program_running', 'program_paused',
                 'robot_mode', 'speed_fraction']
    @staticmethod
    def unpack(buf):
        rmd = RobotModeData()
        (rmd.timestamp, rmd.robot_connected, rmd.real_robot_endabled,
         rmd.power_on_robot, rmd_emergency_stopped,
         rmd.security_stopped, rmd.program_running, rmd.program_paused,
         rmd.robot_mode, rmd.speed_fraction) = struct.unpack_from("IBQ???????Bd", buf)
        return rmd

class JointData(object):
    __slots__ = ['timestamp', 'q_actual', 'q_target', 'qd_actual', 'i_actual',
                 'tool_vector_actual', 'tcp_speed_actual', 'p_gain', 'i_gain',
                 'd_gain', 'v_gain', 'a_gain', 'q_p_gain', 'q_i_gain',
                 'q_d_gain', 'qd_p_gain', 'qd_i_gain', 'qd_d_gain',
                 'i_clamp_max_max', 'i_clamp_max_min', 'i_clamp_min_max',
                 'i_clamp_min_min', 'v_pid_max', 'a_pid_max', 'v_i_max',
                 'a_i_max', 'v_i_min', 'a_i_min', 'v_d_max', 'a_d_max',
                 'v_d_min', 'a_d_min', 'v_max', 'a_max', 'v_min', 'a_min',
                 'tcp_max_speed', 'tcp_max_acc', 'speed_fraction',
                 'speed_scaling', 'speed_joint_type', 'speed_joint_names']
    @staticmethod
    def unpack(buf):
        jd = JointData()
        (jd.timestamp, jd.q_actual, jd.q_target, jd.qd_actual, jd.i_actual,
         jd.tool_vector_actual, jd.tcp_speed_actual, jd.p_gain, jd.i_gain,
         jd.d_gain, jd.v_gain, jd.a_gain, jd.q_p_gain, jd.q_i_gain,
         jd.q_d_gain, jd.qd_p_gain, jd.qd_i_gain, jd.qd_d_gain) = struct.unpack_from("IBQ???????Bd", buf)

class ToolData(object):
    __slots__ = ['analog_input_range2', 'analog_input_range3',
                 'analog_input2', 'analog_input3',
                 'tool_voltage_48V', 'tool_output_voltage', 'tool_current',
                 'tool_temperature', 'tool_mode']
    @staticmethod
    def unpack(buf):
        td = ToolData()
        (d.analog_input_range2, td.analog_input_range3,
         td.analog_input2, td.analog_input3,
         td.tool_voltage_48V, td.tool_output_voltage, td.tool_current,
         td.tool_temperature, td.tool_mode) = struct.unpack_from("!IBbbddfBffB", buffer)
        return td

class MasterBoardData(object):
    __slots__ = ['digital_input_bits', 'digital_output_bits',
                 'analog_input_range0', 'analog_input_range1',
                 'analog_input0', 'analog_input1',
                 'master_board_temperature', 'robot_vcc']
    @staticmethod
    def unpack(buf):
        mbd = MasterBoardData()
        (mbd.digital_input_bits, mbd.digital_output_bits,
         mbd.analog_input_range0, mbd.analog_input_range1,
         mbd.analog_input0, mbd.analog_input1,
         mbd.master_board_temperature, mbd.robot_vcc) = struct.unpack_from("!IBbbddfBffB", buffer)
        return mbd

class JointLimitData(object):
    __slots__ = ['joint_limit_lower', 'joint_limit_upper']
    @staticmethod
    def unpack(buf):
        jld = JointLimitData()
        (jld.joint_limit_lower, jld.joint_limit_upper) = struct.unpack_from("!dd", buffer)
        return jld

class RobotState(object):
    __slots__ = ['robot_mode_data', 'joint_data', 'tool_data',
                 'masterboard_data', 'cartesian_info',
                 'kinematics_info', 'configuration_data',
                 'force_mode_data', 'additional_info',
                 'unknown_ptypes']

    def __init__(self):sudo
    @staticmethod
    def unpack(buf):
        pass
