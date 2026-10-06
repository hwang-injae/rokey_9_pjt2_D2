from setuptools import find_packages, setup

package_name = 'd2_safety'

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
    maintainer='박진용',
    maintainer_email='pjy12110@gmail.com',
    description='D2 정지 노드 — 정지 판단·서기 궤적·잠금, 공통 정지 기능 SafeStop',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'safety_stop = d2_safety.safety_stop_node:main',
        ],
    },
)
