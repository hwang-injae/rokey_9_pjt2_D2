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
        # 레시피(src/recipe_manager/recipes)는 패키지 안 링크 recipes/ 로 읽는다. '../..' 같은 패키지 밖 경로를 주면
        # colcon --symlink-install 이 build/d2_bringup 기준으로 따라가 작업 공간 맨 위에 recipe_manager/ 링크를 만든다 (10/7)
        ('share/' + package_name + '/recipes', glob('recipes/*_recipe.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='박진용',
    maintainer_email='pjy12110@gmail.com',
    description='D2 팀 브링업 — 두산 M0609 + RG2 + MoveIt2, 설정 파일 robot.yaml',
    license='Apache-2.0',
)
