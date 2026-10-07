from setuptools import find_packages, setup

package_name = 'd2_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',                                      # 손목 hand-eye 보정값 + 근거 (민범진)
         ['config/T_gripper2camera.npy', 'config/T_gripper2camera.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='한석형',
    maintainer_email='gkstjrgud19@gmail.com',
    description='D2 비전 — 웹캠 사람 감지, 손목 손 찾기, 손목 블록 인식 (+ 가짜 노드 mock_*)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 실행 이름은 처음부터 다 적어 둔다 — 각자 .py 파일만 추가 (파일이 생기기 전에는 실행 안 됨)
            # 한석형
            'webcam_human = d2_vision.webcam_human:main',
            'wrist_hand = d2_vision.wrist_hand:main',
            'mock_webcam_human = d2_vision.mock_webcam_human:main',
            # 민범진
            'wrist_block = d2_vision.wrist_block:main',
            'mock_wrist_block = d2_vision.mock_wrist_block:main',
            'capture_scene = d2_vision.capture_scene:main',          # W114 촬영 저장 도구 (컬러 · 깊이 · 자세)
        ],
    },
)
