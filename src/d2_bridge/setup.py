from setuptools import find_packages, setup

package_name = 'd2_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='황인재',
    maintainer_email='il1282113@gmail.com',
    description='D2 MQTT 다리 — 로봇 PC ROS 2 ↔ 웹 PC MQTT 브로커 변환(IRD 10장) + 가짜 다리 mock_bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge = d2_bridge.bridge_node:main',
            'mock_bridge = d2_bridge.mock_bridge:main',
        ],
    },
)
