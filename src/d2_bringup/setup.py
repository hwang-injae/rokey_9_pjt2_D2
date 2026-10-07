from glob import glob

from setuptools import find_packages, setup

package_name = 'd2_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 확장자로 골라 설치한다: 'launch/*' 는 src 경로로 런치를 실행하면 생기는 __pycache__ 폴더까지 집어 빌드가 깨진다 (10/7)
        ('share/' + package_name + '/launch', glob('launch/*.launch.py') + glob('launch/*.rviz')),
        ('share/' + package_name + '/description', glob('description/*.xacro') + glob('description/*.srdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.json')),
        # 레시피(한세교 recipe_manager, COLCON_IGNORE)를 함께 설치 — run_recipe 번호 선택·장면 관리가 이 폴더를 읽는다
        ('share/' + package_name + '/recipes', glob('../recipe_manager/recipes/*.recipe.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='박진용',
    maintainer_email='pjy12110@gmail.com',
    description='D2 팀 브링업 — 두산 M0609 + RG2 + MoveIt2, 설정 파일 robot.yaml',
    license='Apache-2.0',
)
