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
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/description', glob('description/*')),
        # robot.yaml(W031, 황인재)이 config/ 에 생기면 그대로 설치된다
        ('share/' + package_name + '/config', glob('config/*')),
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
