# ROS Packages for Conarobot MR

## 1. Package mr_description

> 功能：对MR进行运动学建模，并输出其模型描述urdf文件。<br>
> 启动：roslaunch mr_description mr_display.launch

## 2. Package mr_msgs

> 功能：针对MR生成特定的类型的消息格式(msg/srv/action)，为MR及其他设备提供特定的消息类型。
> 启动：

## 3. Package mr_moveit

> 功能：使用moveit对MR进行运动规划及仿真。<br>
> 启动：roslaunch mr_moveit demo.launch

## 4. Package mr_gazebo

> 功能：在gazebo中利用mr_control功能包对MR进行运动仿真，其中基于sdf版本的存在问题（已弃用）。<br>
> 启动：

## 5. Package mr_driver

> 功能：通过串口与MR机器人底层控制器通讯，以实现机器人控制与规划。<br>
> 启动：

## 6. Package mr_interface

> 功能：为MR机器人提供上层的运动控制及规划的Python/C++接口。<br>
> 启动：

## 7. Package mr_control

> 功能：配置生成MR的运动控制器及rqt窗口配置文件。<br>
> 启动：roslaunch mr_control mr_control.launch

## 8. Package mr_controllers

> 功能：针对MR机器人的自定义控制器。<br>
> 启动：

## 9. Package mr_bringup

> 功能：启动上位机并连接MR下位机，从而控制真实MR机器人。<br>
> 启动：roslaunch mr_bringup mr_bringup.launch

## 10. Package mr_perception

> 功能：作为MR的力感知模块，主要具备以下三项功能：（1）对MR机器人中的触觉感知组件Palm进行初始化标定；（2）对MR机器人中的触觉感知组件Palm的感知功能进行测试并对结果进行可视化；（3）利用Palm的触觉感知组件来感知Palm与人体的交互接触力并进行输出。<br>
> 启动：
>> (1)Palm感知的触觉力的可视化（单独功能）：roslaunch mr_perception palm_force_display.launch <br>
>> (2)Palm感知并发布触觉力（单独功能）：roslaunch mr_perception palm_force_publisher.launch

## 11. Package mr_tasks

> 功能：在仿真器Rviz中加载虚拟人体模型，针对模型进行简单的按摩动作运动规划、控制及仿真。<br>
> 启动：
>> (1)MR机器人进行按摩测试（单独功能）：cd "$(rospack find mr_tasks)/launch/tests" && sh mr_massage_test.sh <br>
>> (2)MR机器人进行按摩Demo（单独功能）：cd "$(rospack find mr_tasks)/launch/tasks" && sh mr_massage_demo.sh

## 12. Package Dependency: Package moveit_task_constructor/mtc_pour

> 功能：
>> (1)moveit_task_constructor: 用以将多个子任务构造成一个复合任务，从而使MR机器人能一次性完成一整套按摩动作。
>> (2)mtc_pour: 利用moveit_task_constructor功能包实现向杯子中倒水的任务合集。
