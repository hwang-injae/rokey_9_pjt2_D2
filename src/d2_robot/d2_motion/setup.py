from setuptools import find_packages, setup

package_name = 'd2_motion'

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
    description='D2 집기·놓기(+ MoveIt2 실행기)와 장면 관리 노드',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pick_place = d2_motion.pick_place_node:main',
            'scene_manager = d2_motion.scene_manager_node:main',
            'run_recipe = d2_motion.run_recipe:main',
        ],
    },
)
