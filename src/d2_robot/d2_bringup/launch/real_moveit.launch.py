# -*- coding: utf-8 -*-
"""팀 브링업: M0609 + RG2 로봇 드라이버·궤적 컨트롤러·그리퍼 드라이버·MoveIt2 move_group 을 띄운다.

수업 브링업(m0609_rg2_bringup bringup.launch.py)은 MoveIt 과 MoveIt 용 궤적 컨트롤러를 띄우지 않아서
MoveIt 으로 로봇을 움직이는 우리 노드에는 이 브링업을 쓴다. 둘을 같이 띄우지 않는다.

  - 로봇: dsr_hardware2 (mode:=real 이면 실제 컨트롤러, virtual 이면 에뮬레이터)
  - 컨트롤러: joint_state_broadcaster -> dsr_controller2 -> dsr_moveit_controller (서기 궤적도 여기로 보낸다)
  - 그리퍼: real = OnRobot RG2 드라이버 (Modbus), virtual = 가상 그리퍼. 둘 다 /onrobot/sendCommand
            실기 손가락 각은 /onrobot_joint_states 로 이름을 바꿔 내보낸다 (로봇 joint_states 와 섞이지 않게)
  - MoveIt: 모델 = description/m0609_rg2_tcp.urdf.xacro (수업 모델 + 손가락 끝 rg2_tcp, 플랜지에서 228 mm)
  - 카메라 노드는 띄우지 않는다 (비전 파트가 띄운다).

실행:
  ros2 launch d2_bringup real_moveit.launch.py                                  # 가상 (에뮬레이터)
  ros2 launch d2_bringup real_moveit.launch.py mode:=real host:=192.168.1.100  # 실기
"""
import json
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

SHARE = get_package_share_directory('d2_bringup')
XACRO = os.path.join(SHARE, 'description', 'm0609_rg2_tcp.urdf.xacro')
SRDF = os.path.join(SHARE, 'description', 'm0609_rg2.srdf')
RVIZ = os.path.join(SHARE, 'launch', 'real_demo.rviz')
CFG_PKG = 'dsr_moveit_config_m0609'
RG2_BASE_OFFSET_M = 0.004   # 플랜지 -> rg2_base_link 원점 (수업 모델 m0609_with_rg2_camera)


def load_tcp():
    """config/tcp.json (제어기에 등록된 활성 TCP GripperDA_v1, 플랜지 기준 mm · posx 축) + robot.yaml finger.center_offset_m
    (손가락 가운데 − 그 TCP, rg2_tcp 축 m) 으로 MoveIt 손가락 가운데 rg2_tcp 의 rg2_base_link 기준 위치 (x, y, z m) 를 만든다.

    rg2_tcp 축은 posx 를 손목 Z 로 -90° 돌린 틀이라 x_rg2 = -y_posx, y_rg2 = +x_posx (10/7 W134 FK · posx 비교, motion_math 머리말).
    10/8: 손가락 가운데가 제어기 TCP(J6 축 위)에서 비켜 있어 손목을 돌려 놓는 블록이 어긋났다 → MoveIt 쪽만 손가락 가운데로 옮긴다
    (제어기 TCP 는 v1 그대로 — 손목 카메라 보정 · 펜던트 기준이 안 바뀜).
    회전(A · B · C)이 0 이 아니거나 기준이 플랜지가 아니면 모델에 그대로 옮길 수 없으므로 ValueError 로 브링업을 멈춘다.
    """
    with open(os.path.join(SHARE, 'config', 'tcp.json')) as f:
        tcp = json.load(f)
    with open(os.path.join(SHARE, 'config', 'robot.yaml')) as f:
        dx, dy = yaml.safe_load(f)['finger']['center_offset_m']
    x, y, z, a, b, c = tcp['pos']
    if tcp['reference_frame'] != 'FLANGE' or any(abs(v) > 1e-6 for v in (a, b, c)):
        raise ValueError(f'tcp.json {tcp["name"]}: 회전 없는 플랜지 기준 TCP 만 쓸 수 있다 ({tcp["pos"]})')
    return round(-y / 1000.0 + dx, 6), round(x / 1000.0 + dy, 6), round(z / 1000.0 - RG2_BASE_OFFSET_M, 6)


def generate_launch_description():
    """브링업 노드를 순서대로 띄우는 LaunchDescription 을 만든다.

    컨트롤러는 joint_state_broadcaster -> dsr_controller2 -> dsr_moveit_controller 순서로 하나씩 켠다
    (앞 것이 끝나야 다음 것을 켤 수 있다). move_group·RViz 는 dsr_moveit_controller 가 켜진 뒤에 띄운다.
    """
    tcp_x, tcp_y, tcp_z = load_tcp()
    args = [
        DeclareLaunchArgument('mode', default_value='virtual', description='virtual | real'),
        DeclareLaunchArgument('host', default_value='127.0.0.1', description='로봇 IP (real 이면 192.168.1.100)'),
        DeclareLaunchArgument('port', default_value='12345'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50'),
        DeclareLaunchArgument('tcp_z', default_value=str(tcp_z), description='rg2_base_link -> 손가락 끝 (m), 기본 = config/tcp.json'),
        DeclareLaunchArgument('gripper_ip', default_value='192.168.1.1'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]
    is_real = PythonExpression(["'", LaunchConfiguration('mode'), "' == 'real'"])
    is_virtual = PythonExpression(["'", LaunchConfiguration('mode'), "' != 'real'"])
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', XACRO,
        ' host:=', LaunchConfiguration('host'), ' port:=', LaunchConfiguration('port'),
        ' rt_host:=', LaunchConfiguration('rt_host'), ' mode:=', LaunchConfiguration('mode'),
        ' model:=m0609 update_rate:=100 tcp_z:=', LaunchConfiguration('tcp_z'), f' tcp_x:={tcp_x} tcp_y:={tcp_y}',
    ]), value_type=str)
    with open(SRDF) as f:
        srdf = f.read()
    moveit_config = (
        MoveItConfigsBuilder('m0609', 'robot_description', CFG_PKG)
        .robot_description(file_path='config/m0609.urdf.xacro')
        .robot_description_semantic(file_path='config/dsr.srdf.xacro')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .joint_limits(file_path=os.path.join(SHARE, 'config', 'joint_limits.yaml'))   # 6번 관절 속도를 제어기에 맞춤
        .planning_pipelines(pipelines=['ompl', 'chomp', 'pilz_industrial_motion_planner'],
                            default_planning_pipeline='ompl', load_all=False)
        .to_moveit_configs()
    )
    # 두산 MoveIt 설정의 모델 대신 손가락 끝(rg2_tcp)이 든 우리 모델을 쓴다
    override = {'robot_description': robot_description, 'robot_description_semantic': srdf}

    emulator = Node(
        package='dsr_bringup2', executable='run_emulator', namespace='',
        parameters=[{'name': '', 'rate': 100, 'standby': 5000, 'command': True,
                     'host': LaunchConfiguration('host'), 'port': LaunchConfiguration('port'),
                     'mode': LaunchConfiguration('mode'), 'model': 'm0609', 'gripper': 'none',
                     'mobile': 'none', 'rt_host': LaunchConfiguration('rt_host')}],
        output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='both',
               parameters=[{'robot_description': robot_description, 'publish_frequency': 100.0}])
    control = Node(
        package='controller_manager', executable='ros2_control_node', output='both',
        parameters=[{'robot_description': robot_description},
                    os.path.join(get_package_share_directory('dsr_controller2'), 'config', 'dsr_controller2.yaml')])

    def spawner(name):
        """컨트롤러 하나를 켜는 spawner 노드를 만든다. 실기 연결이 느릴 수 있어 120 s 까지 기다린다."""
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '-c', 'controller_manager', '--controller-manager-timeout', '120'])
    jsb, dsr, moveit_ctrl = spawner('joint_state_broadcaster'), spawner('dsr_controller2'), spawner('dsr_moveit_controller')

    grip_virtual = Node(package='m0609_rg2_bringup', executable='gripper_virtual_node.py',
                        name='gripper_virtual_node', output='screen', condition=IfCondition(is_virtual))
    grip_real = Node(
        package='onrobot_rg_control', executable='OnRobotRGControllerServer', name='OnRobotRGControllerServer',
        output='screen', condition=IfCondition(is_real),
        parameters=[{'/onrobot/control': 'modbus', '/onrobot/ip': LaunchConfiguration('gripper_ip'),
                     '/onrobot/port': 502, '/onrobot/changer_addr': 65, '/onrobot/gripper': 'rg2',
                     '/onrobot/offset': 5}],
        remappings=[('/joint_states', '/onrobot_joint_states')])
    grip_js = Node(package='m0609_rg2_bringup', executable='gripper_joint_state_publisher.py',
                   name='gripper_joint_state_publisher', remappings=[('gripper_joint_states', 'joint_states')])

    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                      parameters=[moveit_config.to_dict(), override])
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='log', arguments=['-d', RVIZ],
                parameters=[moveit_config.planning_pipelines, moveit_config.robot_description_kinematics,
                            moveit_config.joint_limits, override],
                condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription(args + [
        LogInfo(msg='>> [D2 브링업] M0609 + RG2 + MoveIt2'),
        emulator, rsp, control, jsb, grip_virtual, grip_real, grip_js,
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[dsr])),
        RegisterEventHandler(OnProcessExit(target_action=dsr, on_exit=[moveit_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=moveit_ctrl, on_exit=[
            LogInfo(msg='>> dsr_moveit_controller 활성. move_group + RViz 시작'), move_group, rviz])),
    ])
