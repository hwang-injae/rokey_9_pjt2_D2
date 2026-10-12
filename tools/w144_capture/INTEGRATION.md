# 박진용 객체 인식 조립 코드에 추가할 촬영 기능

공급칸 실행기(build_capture/run.sh)는 사용하지 않는다. 기존 one_pick 객체 인식·고르기·집기·p1·재시도 정책을 대체하지 않는다.

1. 시작 때 PlacementRecorder(folder, move_observe, stop_robot)를 만든다.
2. move_observe 콜백은 기존 MoveTo(target='observe')를 사용하고 실제 도착 성공 bool을 반환한다. 기존 시간 제한·Ctrl+C·먼저 안전정지 처리를 유지한다. stop_robot는 기존 안전정지 함수다.
3. 최종 설계 자리에 배치 성공한 뒤 after_place(block_id, target_center_m, target_quaternion_xyzw)를 호출한다. p1 중간 배치에는 호출하지 않는다. 넘기는 좌표는 실제 블록의 설계 배치 중심/방향이며 틀기·옮김 그리퍼 목표가 아니다.
4. after_place 실패는 위로 전파해 다음 블록을 집지 않는다. 정지 후 재시작/알람해제는 기존 사람 조건을 유지한다.
5. 실행 finally에서 finish(completed)를 호출하고 ZIP경로를 출력한다. 중단자료도압축한다. 완료True는기존조립성공조건과전체11촬영이모두성공한경우만.
6. 기존 실행기가 사용한 robot.yaml/레시피/실제 TCP→카메라 보정 파일·단위·버전도 결과폴더에 복사해 주세요. 키/.env는 넣지 않는다.

결과: 최종배치11개마다observe하나만,사진11장. 블록마다 RGB PNG 한 장과 같은 시점의 정렬 깊이 한 장·카메라실측내참수·촬영시각TF. placements.json은목표,관측자료는실제좌표추정용. ZIP하나전달.

촬영기는표준RealSense /camera/camera 접두사용,다르면capture.py CLI --prefix에기존접두를넘기도록통합. 정렬깊이/색/내참수크기·시각불일치거절. 카메라TF없어도누락기록하며실제handeye쌍확인후분석. ROS환경과카메라기동은기존실행기책임.

검증: recorder pytest3통과(11이동→11촬영→ZIP,observe실패시정지/촬영금지/실패ZIP). 이동·촬영은대역. capture.py import/문법/CLI와기존observe가시성시뮬은확인했지만박진용실행기에통합한실기/모션시험은미완료. 연결후기존가상환경에서pytest/조립·촬영개수/정지실패검증결과와수정코드를팀코로반환.
