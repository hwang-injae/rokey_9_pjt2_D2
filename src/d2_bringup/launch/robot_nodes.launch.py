# -*- coding: utf-8 -*-
"""로봇 PC 노드 4개를 띄운다: 정지(safety_stop) · 그리퍼(gripper) · 장면 관리(scene_manager) · 집기·놓기(pick_place).

브링업(real_moveit.launch.py)을 다른 터미널에서 먼저 띄운 뒤 쓴다.
  ros2 launch d2_bringup robot_nodes.launch.py                # 가상
  ros2 launch d2_bringup robot_nodes.launch.py mode:=real     # 실기
recipe 를 비우면 장면 관리가 share/d2_bringup/recipes/ 의 레시피를 모두 읽는다 (어느 것을 쌓을지는 run_recipe 에서 고름).
세트 배치(레시피 여러 개를 나란히)는 recipe:=<a>,<b> 로 run_recipe 와 같은 목록을 준다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """노드 4개를 띄우는 LaunchDescription. mode 가 real 이 아니면 그리퍼 노드를 가상 그리퍼용으로 켠다."""
    virtual = PythonExpression(["'", LaunchConfiguration('mode'), "' != 'real'"])
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='virtual', description='virtual | real'),
        DeclareLaunchArgument('recipe', default_value='', description='레시피 파일, 여럿이면 쉼표로 = 세트 배치 (장면 관리가 블록 크기·자리를 읽음). 비우면 설치된 레시피 전부(하나씩)'),
        Node(package='d2_safety', executable='safety_stop', output='screen'),
        Node(package='d2_gripper', executable='gripper', output='screen',
             parameters=[{'virtual': ParameterValue(virtual, value_type=bool)}]),
        Node(package='d2_motion', executable='scene_manager', output='screen',
             parameters=[{'recipe': LaunchConfiguration('recipe')}]),
        Node(package='d2_motion', executable='pick_place', output='screen'),
    ])
