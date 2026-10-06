from setuptools import find_packages, setup

package_name = 'd2_task'

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
    maintainer='한석형',
    maintainer_email='gkstjrgud19@gmail.com',
    description='D2 작업 관리자 — 작업 판단(진행표 · 다음 블록), 상태표, 검사 묶음, 기록',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'task = d2_task.task_node:main',
        ],
    },
)
