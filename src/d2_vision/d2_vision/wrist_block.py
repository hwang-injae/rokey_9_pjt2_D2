"""손목 블록 인식 노드 (wrist_block) — IRD 4.2 `/d2/vision/check_progress` · `scan_capture` · `scan_infer` · `find_blocks`
· 4.1 `camera_status/1` · `wrist_image` (W041 · W140 E-52 → E-69 · W115 · W086 · W148).

작업 관리자가 관측 자세에서 `check_progress`(design_id · part_ids)를 부르면:
  손목 깊이 영상 10장(중앙값) + 로봇 자세(TF base_link → rg2_tcp, 10/9 E-70 ③) + hand-eye 보정값(rg2_tcp 틀) → base 점군 → BlockChecker(block_checker.py)
  → 블록마다 state(present·absent·occluded·unknown) · top_z_m · dz_m (dx·dy 는 1차 NaN) 로 답한다.
카메라 연결 신호 `/d2/vision/camera_status` 를 2 Hz 로 낸다(마지막 프레임 시각).
스캔 · 흩뿌림 찾기도 이 노드 하나가 맡는다(IRD 84줄 '노드를 나눌지는 비전이 정함' → 카메라 구독 · 로봇 자세 · 보정값을 같이 쓰려고 한 노드, 10/8).
  `/d2/vision/scan_capture`(JsonQuery) {"pose_id","run_id"} → 요청 뒤 깊이 scan_frames 장 평균(구멍 0 은 빼고) + 지금 TF 자세 × 보정
      + 요청 뒤 컬러 한 장의 YOLO 블록 마스크(답하기 전에 — 스캔 추론기 v12g 가 윗면 근거로 쓴다, 10/12 W115)
      → StructureScanner.capture_pose(structure_scanner.py) → {"ok":true,"points":n}. run_id 마다 따로 모으고, 같은 run_id 에 같은
      pose_id 가 다시 오면 덮어쓴다. 그때의 컬러 한 장도 둔다(scan_infer 의 사진). 컬러가 없거나 YOLO 를 안 읽었으면(yolo_fallback)
      마스크 없이 넣고 경고만 — scan_infer 의 confidence.masks_missing 에 남는다(마스크가 없으면 일치율이 떨어질 수 있다).
      점군 미리보기(10/10 E-82): 답은 바로 하고, 그 뒤 따로 스레드가 지금까지 모은 점군을 `<scan_dir>/<run_id>/preview_<index>.ply`
      (save_cloud — 복셀 3 mm · ≤ 2 MB)로 저장해 `/d2/vision/scan_preview`(std_msgs/String, scan_preview/1.0 JSON, 기본 QoS)로 알린다.
      index = 이번 run 의 scan_capture 순번(1부터). 파일마다 그 자세까지 모은 점 전부(누적)라 화면은 index 가 가장 큰(최신) 파일로
      바꿔 그린다(쌓으면 앞 자세 점이 겹침). 보기 전용 — 작업 관리자는 안 받는다. 저장 · 발행 실패는 경고 로그만.
      자세 자료(10/11): 같은 스레드가 이어서 `<scan_dir>/<run_id>/<pose_id>_color.png · _depth.png(평균 깊이 mm, uint16) · _pose.json`
      (capture_scene 과 같은 칸 — intrinsics_px · depth_unit · T_base2cam_m(m) · tcp + run_id · index)을 남긴다. 같은 자세 다시 찍으면 덮어씀.
      컬러는 요청 뒤 들어온 것이 있을 때만(답 시간이 늘지 않게 기다리지 않음). 실패는 경고 로그만. 스캔이 저절로 구조물 사진 자료가 되게.
  `/d2/vision/scan_infer`(JsonQuery) {"run_id"} → infer() → `<scan_dir>/<run_id>/` 에 color.png · cloud.ply(≤ 2 MB) · blocks.json
      → {"ok":true,"blocks":blocks/2,"inferred_count","image_path","cloud_path"}. 임시 설계 이름 scan_<family>_<번호>(저장 때 웹이 V 번호로 바꾼다) —
      family 는 recipe_dir 의 기본 설계(이름에 CHAIR · DESK) 중 블록 수 · 외곽이 가장 가까운 것, 못 구하면 unknown.
      번호 = scan_dir 에 이미 있는 blocks.json 수 + 1(노드를 다시 켜도 이름이 겹치지 않게). 추론이 끝난 run_id 는 메모리에서 지운다.
  `/d2/vision/find_blocks`(JsonQuery) {"run_id"} → 요청 뒤 컬러 · 깊이(scan_frames 장 평균) + TF 자세 × 보정 → BlockFinder(block_finder.py)
      → {"ok":true,"blocks":[…],"image_seq":n}(IRD 4.2 칸, 윗면 높이 순). 블록 0개도 ok true(빈 목록 — 작업 관리자가 WAIT_SUPPLY).
      image_seq(10/14 22시 PL) = 공급 영역 찾기(area 없음)를 계산할 때마다 1 늘리는 그림 번호(노드가 켜질 때 0 → 첫 답 1). 그림을 못 냈어도 싣는다 —
      작업 관리자가 고른 블록을 state/2 pick.image_seq 에 그대로 넣으면 이 노드가 그 그림에 번호를 덧그린다(아래 wrist_image). area spare · assembly 답에는 없다.
      마스크: YOLO seg(본 방법 E-38 — 모델은 켤 때 읽는다, 아래 '켤 때 YOLO'). 예비(엣지 + 깊이, E-47)는 yolo_fallback:=true 일 때만.
      선택 칸(10/13 — 없으면 위와 똑같이 답한다. 값 검사는 mock_scan.find_options, 틀리면 SCAN_FAILED):
        "height_map": true → 응답에 "height_map" {"x0_m","y0_m","res_m","nx","ny","z_m"} 를 더한다 — 같은 깊이 · 자세의 base 점으로
            공급 영역(robot.yaml supply_area_m)을 손가락 여유(가장 넓게 연 폭/2 + 손가락 두께)만큼 넓힌 영역의 셀(height_grid.HEIGHT_MAP_RES_M = 2 mm)마다
            최고 z(m, 소수 4자리), 빈 셀 null,
            z_m[iy·nx + ix]. 작업 관리자가 잡기 후보의 손가락 자리에 높은 셀이 있는지 본다.
        "area": "spare" → YOLO · 마스크 없이 받침(robot.yaml spare_spots[0]) 위 블록 하나를 깊이로만 잰다(spare_measure.measure_on_spare)
            → {"ok":true,"blocks":[b]}(b = 위 칸 + "base_z_m" 받침 윗면) 또는 빈 받침 {"ok":true,"blocks":[]}. 카메라는 작업 관리자가
            먼저 받침 위로 옮겨 둔다. 재기는 깊이만 쓴다. 요청 뒤 컬러가 있으면 wrist_image 에 잰 블록 · 목표 자리를 그려 낸다(그림만, 10/14 HMI · PL).
            이때 height_map 은 받침 둘레(받침 가운데 ± 블록 길이/2 + 손가락 여유) 지도다.
      기울어진 블록(tilted true)에만 "center_top_z_m"(윗면 가운데 평면 높이 m) · "normal"(윗면 단위 법선, 위쪽)을 더한다(block_finder).
        "area": "assembly" + "design_id" · "part_id"(10/14 W144 — 한석형 · 민범진) → 조립 자리에 방금 놓은 블록 하나를 깊이로만 잰다
            (placed_measure.measure_placed — 설계 자리 둘레, YOLO 없음) → {"ok":true,"part_id","blocks":[b] 또는 [],"rejected","detail","measured_stamp"}.
            못 재면(옆 블록과 붙음 · 덮임 · 세운 블록 · 기울어짐 · 점 부족 · 크기 다름 · 없음) blocks [] + rejected 코드 — 요청 오류(SCAN_FAILED)와 다르다.
            placed_method:=mask(기본 — 10/15 저녁 PL, 다시 집기 기본 켬 #194 과 함께)면 같은 깊이로 measure_placed 를 먼저 보고(기울어짐 · 덮임 · 세움 · 점 부족 · 없음은
            그대로), 크기 · 위치 · yaw 는 요청 뒤 컬러 한 장의 YOLO 마스크 + 광선으로 다시 잰다(placed_mask.measure_placed_mask —
            깊이가 윗면 가장자리를 처지게 재 크기가 작게 · 가운데가 흔들리는 것을 피하려고). 응답 칸은 같다 — 어느 방법으로 쟀는지는 로그에만.
            컬러가 없거나 · YOLO 를 안 읽었거나(yolo_fallback) · 마스크 쪽 예외면 깊이 결과 그대로 + 경고.
  `/d2/vision/wrist_image`(sensor_msgs/CompressedImage, JPEG 품질 70, 640×480 그대로) — find_blocks 때마다 검출 결과
      (윤곽 · 윤곽 위 작은 글자 'FLAT/EDGE/STAND[ TILT n]')를 그린 그림 한 장(IRD E-67 '때만 내도 됨'). 매 프레임 내지 않는다.
      고른 뒤 덧그림(10/14 22시 PL): `/d2/task/state`(std_msgs/String state/2 JSON, task 와 같은 QoS — RELIABLE · TRANSIENT_LOCAL · KEEP_LAST 1)를
      받아 선택 칸 "pick" {"image_seq","index"(1부터),"ranks"[…](1부터, 우선순위 차례),"reason"} 의 image_seq 가 마지막 공급 찾기 번호와 같고
      (image_seq, index, ranks)가 지난번과 다를 때만 그 찾기 그림에 우선순위 번호 동그라미 · 고른 블록 채우기를 그려 한 장 더 낸다
      (block_finder.draw_found ranks · pick, 판단은 pick_redraw_key). state 가 올 때마다 그리지 않는다. 꼴이 틀린 state · pick 은 종류마다 경고 한 번만.
      area spare 때는 잰 블록 윗면 · 받침 목표 자리 상자 · 가운데 차이(base dx · dy mm + 방향 화살표)를
      그린다(block_finder.draw_spare, 컬러가 없으면 안 냄). 받침 단계에는 이 그림만 낸다.
      작업대 그림(박진용 안 ②, 10/14 22시 PL — block_finder.draw_workbench): check_progress 답 때(시작 확인 · 블록을 놓은 뒤) 물은 부품의
      설계 윗면 사각형을 판정 색(present 초록 · occluded · unknown 주황 · absent 빨강)과 짧은 이름('LEG 1-3')으로, find_blocks area assembly
      (A4 다시 재기) 답 때 설계 자리(빨간 굵은 상자) + 잰 블록(초록) + 차이 · 거절 코드 글자로 한 장. 요청 뒤 컬러가 없으면 안 낸다(답은 그대로).

설계 · 블록 이름 (E-52, 10/7)
  요청 design_id 칸으로 설계를 고른다(블록 이름에서 잘라 내지 않는다). part_ids 는 전체 블록 이름
  `<DESIGN_ID>_<역할>[_<옵션>]_<그룹 3자리>_<부품 2자리>`(예 001_CHAIR_BENCH_V000_LEG_001_01, 10/13 용어 B)를 그대로 받아 레시피가 만든 이름과 맞춘다.
  design_id 칸이 비면 설계를 고를 수 없어 실패로 답한다(success=false, reason=ERROR — 아래 '실패 때').
  한 요청은 한 설계다 — 그 설계 레시피에 없는 블록은 그 블록만 unknown.

설계 읽기 — task 와 같은 파라미터 design_source (10/7 민범진 · 한석형 합의). **task 와 손목에 같은 design_source 값을 준다**
  (다르면 task 는 웹 설계로 쌓는데 손목은 파일 설계로 보는 식으로 어긋난다).
  remote(기본)  설계를 늘 `/d2/hmi/get_design`(JsonQuery, 요청 {"design_id"} → 응답 design/4.0)으로 받는다 — 생성 · 스캔 설계도 같은 길.
                design/4.0 의 recipe(구조) + placements(조립 방법)를 load_recipe 와 같은 모양으로 붙여(block_checker.recipe_from_design)
                팀 공용 `d2_motion.motion_math.recipe_parts()` 로 base 블록 목록을 만든다. 설계마다 · 판(run)마다 받아 캐시(실패는 캐시 안 함).
                제한 시간 robot.yaml timeout.service_s. 실패하면 로컬 파일로 대신하지 않고 check_progress 를 실패로 답한다(아래).
  local         파일만(웹 없이 개발할 때 명시): `<recipe_dir>/<design_id>_recipe.json`(구조, E-69 — 같은 폴더 `_placements.csv` 도 읽음)을
                `load_recipe()` · `recipe_parts()` 로(설계마다 · 판마다 캐시).
  캐시는 한 판(run) 안에서만(E-55 ① 10/7 PL — 기본 설계는 같은 design_id 로 다시 등록될 수 있다): check_progress 요청의 run_id 가
  지난 요청과 다르면 새 판으로 보고 그 설계를 다시 읽는다(E-60 ②, 10/10 W157). 같은 run_id 면 블록을 몇 개 묻든(전부를 묻는 시작 확인도)
  읽어 둔 설계를 쓴다 — '블록 전부를 묻는 요청 = 새 판'이라고 짐작하지 않는다. 다시 읽기가 실패하면 지난 판의 설계로 답하지 않는다.
  run_id 가 빈 값이면 판 구분 없음(CheckProgress.srv): 모두 이름 없는 한 판으로 보아 빈 값끼리는 캐시를 쓰고, 이름 있는 판과 오가면 다시 읽는다.
  (판 계산은 design_cache.RunDesignCache — mock_wrist_block 과 같은 규칙.)

입력(파라미터)
  design_source  'remote'(기본) · 'local' — task 노드와 같은 이름 · 기본값 · 뜻. 'local' 이 아니면 모두 remote 로 본다(task 와 같음)
  recipe_dir   레시피 폴더(task 노드와 같은 파라미터, 예 src/recipe_manager/recipes). design_source:=local 일 때 check_progress 설계,
               그리고 scan_infer 의 family 고르기(기본 설계 4개)에 쓴다 — 비우면 스캔 family 는 unknown
  calib_path   T_rg2tcp2camera.npy(카메라 → rg2_tcp 4x4, mm — 10/10 W156). 비우면 이 패키지 share/config 의 것
  cam_prefix   realsense 토픽 접두 (Jazzy 기본 /camera/camera) — 깊이 · camera_info · 컬러(<접두>/color/image_raw)
  tcp_pose_mm  시험용 rg2_tcp 자세 7개(x · y · z mm + 쿼터니언 x · y · z · w, **실수로**). 모두 0 이면 TF(robot.yaml frame_id → tcp_link)로 읽는다
  n_frames     check_progress 가 중앙값 낼 깊이 프레임 수(10 — V-17 경험: 정지 상태 10장이면 ±0.5 mm)
  scan_frames  scan_capture · find_blocks 가 평균 낼 깊이 프레임 수(10 — 30 fps 면 0.33초, task 의 3초 제한 안. 45장까지 1.5초 안)
  scan_dir     스캔 결과 폴더(비우면 ~/d2_data/scan) — <run_id>/color.png · cloud.ply · blocks.json · preview_<n>.ply ·
               <pose_id>_color.png · _depth.png · _pose.json(자세 자료)
  yolo_model   YOLO seg 모델 파일(.pt) 경로. 기본 = 이 패키지 share/models 에 설치된 판(DEFAULT_YOLO_MODEL, E-86)
  yolo_fallback  true 면 YOLO 를 읽지 않고 예비 마스크(엣지 + 깊이, E-47)만 쓴다(기본 false — 모델 · ultralytics 가 없는 시험 · CI 용)
  yolo_conf    YOLO 검출 신뢰도 문턱(0.55 — 10/8 확정 모델 26s판: 검증 장면 재현율 ≥ 0.95 를 지키는 가장 높은 값, 0.5 보다 헛것만 줄고 놓침은 같음)
  placed_method  find_blocks area assembly 재는 방법: 'mask'(기본 — YOLO 마스크 + 광선으로 크기 · 위치, 기울기 · 덮임은 깊이) · 'depth'(깊이만,
               measure_placed — 끌 때만 -p placed_method:=depth). 'mask' 가 아니면 모두 depth. 요청마다 읽는다(ros2 param set 으로 바로 바꿀 수 있다).
               기본을 mask 로 둔 까닭(10/15 저녁 PL): 다시 집기가 기본 켜짐(#194)이라 깊이 값(−y 끝이 약 5 mm 짧게 잡힘)으로 재면 멀쩡한 블록을 다시 집는다
  placed_mask_shift_px  mask 방법에서 YOLO 마스크를 옮길 (du, dv) 픽셀(정수 · 실수 둘 다 됨 — 정수 픽셀로 반올림, 기본 [0, 0]) — 컬러 ↔ 깊이 정렬 어긋남 보정.
               10/15 흉내 시험에서 dy −2.6 mm 치우침 · 마스크를 깊이 윗면에 맞추려면 u 약 −5 px, 같은 15장의 가운데 dy 를 0 으로 하는 값은
               약 [−3, 0](eval_mask) — 값은 실기에서 정한다. 요청마다 읽는다
robot.yaml(d2_bringup)에서 assembly_origin · assembly_area_half_m · block_size_m · block_actual_m · timeout.service_s ·
  table_z_m · find.min_gap_mm · grasp_depth_m · finger.width_m · supply_area_m · spare_spots(area spare 때) 를 읽는다.

바깥 영향: 서비스 답 · camera_status · wrist_image · scan_preview 발행 · /d2/task/state 구독(그림만) · get_design 조회(remote, 설계마다 · 판마다 한 번) ·
  scan_capture 미리보기 PLY · 자세 자료(사진 · 깊이 · 자세) · scan_infer 의 파일 쓰기(scan_dir 아래)뿐. 로봇·카메라·그리퍼에 명령을 보내지 않는다(관측 · 촬영 자세 이동은 작업 관리자가 move_to 로).
  placed_method:=mask 면 find_blocks area assembly 때 YOLO 를 한 번 더 돌린다(답 시간 + YOLO 한 번 — 이 PC GPU 실측 약 0.02초 · CPU 면 더 길다). 작업대 그림은 두 방법 다 같은 조건(요청 뒤 컬러가 있을 때)으로 낸다.
답할 때: 요청이 온 **뒤에** 들어온 깊이 프레임 n_frames 장(최대 1.5초 기다림)의 중앙값을 쓴다 — 로봇이 막 멈춘 직후의 움직이던 프레임을 섞지 않으려고.
  remote 에서 처음 보는 설계이거나 run_id 가 바뀌었으면 그 앞에 get_design 을 최대 timeout.service_s(3초) 기다린다.
켤 때 YOLO(10/11 E-86): yolo_fallback 이 false(기본)인데 ultralytics 가 없거나 · 모델 파일이 없거나 · 읽기 · 미리 돌리기가 실패하면
  오류 로그 한 줄을 남기고 노드가 켜지지 않는다(종료 코드 1) — 조용히 예비로 돌아 실기 결과를 잘못 읽지 않으려고.
  돌다가 한 번의 추론이 실패하면 그 find_blocks 만 예비 마스크로 간다(그대로).
실패 때: 새 프레임이 시간 안에 안 오거나 TF 자세(브링업 robot_state_publisher)를 못 받으면 success=false, reason=TIMEOUT.
         get_design 이 timeout.service_s 안에 답하지 않으면 success=false, reason=TIMEOUT (task_node.get_design 과 같은 코드).
         get_design 서버 없음 · success=false · 응답 형식 오류(옛 design/1 포함), 보정값·레시피를 못 읽음, design_id 칸이 빔
         → 노드는 뜬 채 success=false, reason=ERROR.
         (배열은 모두 요청 길이, state unknown, 값 NaN.)
스캔 · 찾기 실패 때(JsonQuery 두 층 — mock_wrist_block 과 같음. 응답 JSON 은 늘 {"ok":false,"reason","detail"}):
  요청 JSON 이 틀림(run_id · pose_id 없음 · 폴더 이름으로 못 쓰는 글자) → success=false, SCAN_FAILED.
  보정값 없음 · 계산 예외 → success=false, ERROR. TF 자세 못 받음 · 요청 뒤 새 깊이(find_blocks 는 컬러도)가 시간 안에 안 옴 → success=false, TIMEOUT.
  촬영 안 한 run_id · 추론 신뢰도 미달(점 설명률 · 추정 블록 비율 · 자세 수 — StructureScanner ⑧) → success=true + ok:false SCAN_FAILED.
  PLY 저장만 실패하면 cloud_path 칸을 빼고 ok:true 로 답하고 경고 로그(IRD 4.2 cloud_path 선택 칸, 10/8 PL).
NaN 규칙(CheckProgress.srv · IRD 5장 W121 C-5): dx·dy 는 1차 늘 NaN. dz_m 은 present 일 때만 값.
  top_z_m 은 present · occluded(설계 밖 물체 윗면, 참고값) 일 때 값. absent·unknown · 위 블록에 가려진 present 는 dz·top_z 둘 다 NaN.
로봇 자세는 MoveIt 모델(관절값 + config/tcp.json 으로 만든 rg2_tcp)에서 나오므로 펜던트 활성 TCP 와 상관없다(10/10 W156 — 예전 posx × T_gripper2camera 길은 뺌).

실행 (저장소 맨 위에서). **task 와 같은 design_source 값을 준다:**
  웹 · 다리와 함께(기본 remote — task 도 기본 remote):
    ros2 run d2_vision wrist_block
  웹 없이 파일로(task 도 -p design_source:=local -p recipe_dir:=… 로 띄운다):
    ros2 run d2_vision wrist_block --ros-args -p design_source:=local -p recipe_dir:=src/recipe_manager/recipes
시험 호출 (design_id 칸 + 전체 블록 이름):
  ros2 service call /d2/vision/check_progress d2_interfaces/srv/CheckProgress "{design_id: 001_CHAIR_BENCH_V000, part_ids: [001_CHAIR_BENCH_V000_LEG_001_01, 001_CHAIR_BENCH_V000_SEAT_001_03]}"
스캔(촬영 자세마다 scan_capture → 마지막에 scan_infer) · 흩뿌림 찾기 · 검출 그림 보기:
  ros2 run d2_vision wrist_block --ros-args -p recipe_dir:=src/recipe_manager/recipes          (YOLO = 설치된 기본 모델)
  (ultralytics · 모델 없이 시험할 때만: … -p yolo_fallback:=true — 예비 마스크)
  ros2 service call /d2/vision/scan_capture d2_interfaces/srv/JsonQuery "{request_json: '{\\"pose_id\\": \\"observe_front\\", \\"run_id\\": \\"R1\\"}'}"
  ros2 service call /d2/vision/scan_infer d2_interfaces/srv/JsonQuery "{request_json: '{\\"run_id\\": \\"R1\\"}'}"
  ros2 service call /d2/vision/find_blocks d2_interfaces/srv/JsonQuery "{request_json: '{\\"run_id\\": \\"R1\\"}'}"
  ros2 topic echo --once /d2/vision/wrist_image --field format     (find_blocks 를 부른 뒤 'jpeg' 이 한 번 온다)
"""
import json
import threading
import time
import warnings
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from d2_interfaces.srv import CheckProgress, JsonQuery
from d2_motion.motion_math import RECIPE_SUFFIXES, load_recipe, recipe_parts
from d2_vision.block_checker import BlockChecker, depth_to_base_points, mean_depth_mm, recipe_from_design, recipe_path
from d2_vision.block_finder import (BlockFinder, base_image, draw_found, draw_spare, draw_workbench, finder_cfg, masks_from_yolo,
                                   pick_redraw_key, placed_note)
from d2_vision.design_cache import RunDesignCache
# 요청 검사(run_id 를 폴더 이름으로 써도 되는지까지) · 레시피 두 파일 → blocks/2 는 가짜 노드와 같은 함수를 쓴다(ROS 없는 계산, 시험 있음)
from d2_vision.height_grid import height_grid, spare_map_area, supply_map_area
from d2_vision.mock_scan import assembly_target, find_options, load_recipe_files, parse_request, structure_to_blocks
from d2_vision.placed_mask import measure_placed_mask_detail
from d2_vision.placed_measure import measure_placed
from d2_vision.spare_measure import ideal_spare_block, measure_on_spare
from d2_vision.structure_scanner import StructureScanner, family_of, save_capture, save_preview, scan_response
from d2_vision.tcp_pose import TcpPose, camera_pose_mm, default_calib_path, load_calib, load_tilt_shift

NAN = float('nan')
# 설치된 YOLO seg 모델(setup.py 가 share/d2_vision/models 에 넣는다 — 10/8 W086 선정 26s판, 바꿀 때만 새 판 이름으로, E-86)
DEFAULT_YOLO_MODEL = str(Path(get_package_share_directory('d2_vision')) / 'models' / 'block_seg_26s_e150_261008.pt')
QOS_STATUS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
# 작업 관리자 /d2/task/state 와 같은 QoS(task_node LATCHED_QOS — IRD 4.1 E-62): 늦게 떠도 마지막 state 를 받는다
QOS_STATE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
FRESH_WAIT_S = 1.5         # 요청 뒤 새 프레임 n장을 이만큼 기다린다(30 Hz 면 10장에 0.33초 — 여유 포함). 넘으면 TIMEOUT
SERVICE_WAIT_S = 3.0       # TF 자세 · 서비스(get_design 은 timeout.service_s) 기본 기다림 — 10/10 W156 뒤 두산 서비스는 안 부름
MAX_SCAN_RUNS = 5          # 추론 전 run_id 를 이만큼만 기억한다(scan_infer 가 안 온 판이 쌓여 메모리가 늘지 않게 — 한 판 점군 약 2~5 MB)
JPEG_QUALITY = 70          # wrist_image (IRD E-67 — 640×480 · 품질 70 ≈ 65 KB)


def posx_to_matrix(x, y, z, rx, ry, rz):
    """두산 posx(mm, ZYZ deg) → T_base2gripper 4x4 (mm). 노드는 이제 TF 를 쓰고(W156), 이 함수는 저장소 밖 시험 스크립트(fs_rotate_test 등)가 가져다 쓴다."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


class WristBlock(Node):
    """check_progress · scan_capture · scan_infer · find_blocks 에 답하고 camera_status · wrist_image 를 내는 손목 블록 인식 노드."""

    def __init__(self):
        """파라미터 · 설정 · 보정값 · YOLO 모델 · 기본 설계(스캔 family 용)를 읽고 구독 · 서비스 · 클라이언트(get_design) · TF 리스너 ·
        발행 · 타이머를 만든다. check_progress 설계는 요청이 올 때 읽는다(설계마다 한 번). 레시피 폴더 · 보정이 없어도 노드는 뜬다(경고).
        YOLO 모델만은 못 읽으면 SystemExit(1) 로 켜지지 않는다(yolo_fallback:=true 면 읽지 않음 — E-86)."""
        super().__init__('wrist_block')
        self.declare_parameter('recipe_dir', '')           # design_source:=local 설계 · 스캔 family 고르기에 쓰는 레시피 폴더(task 와 같음)
        self.declare_parameter('design_source', 'remote')  # task 와 같은 값: remote = /d2/hmi/get_design(기본) · local = recipe_dir 파일
        self.declare_parameter('calib_path', '')
        self.declare_parameter('cam_prefix', '/camera/camera')
        self.declare_parameter('tcp_pose_mm', [0.0] * 7)
        self.declare_parameter('n_frames', 10)
        self.declare_parameter('scan_frames', 10)
        self.declare_parameter('scan_dir', '')
        self.declare_parameter('yolo_model', DEFAULT_YOLO_MODEL)
        self.declare_parameter('yolo_fallback', False)     # true = YOLO 안 읽고 예비 마스크만(시험 · CI 용 — E-86, 기본 끔)
        self.declare_parameter('yolo_conf', 0.55)
        self.declare_parameter('status_hz', 2.0)
        self.declare_parameter('placed_method', 'mask')           # find_blocks area assembly: mask(기본 — 마스크 + 광선, 10/15 저녁 PL) · depth(깊이만)
        # mask 때 마스크를 옮길 (du, dv) 픽셀 — 컬러 ↔ 깊이 정렬 보정(실기에서 정함). 형을 안 묶는다(dynamic_typing): [-3, 0] 처럼 정수로 넣어도
        # 노드가 죽지 않게(DOUBLE_ARRAY 로 선언하면 INTEGER_ARRAY 를 넣을 때 InvalidParameterTypeException 으로 안 켜진다 — 10/15 검토)
        self.declare_parameter('placed_mask_shift_px', [0.0, 0.0], ParameterDescriptor(dynamic_typing=True))

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = []                      # (받은 시각 s, 깊이 mm float32) 최근 max(n_frames, scan_frames) 개
        self.color = None                     # (받은 시각 s, 컬러 BGR uint8, frame_id) 마지막 한 장
        self.intr = None                      # (fx, fy, ppx, ppy) 픽셀 단위
        self.last_frame_t = 0.0
        self.n_frames = int(self.get_parameter('n_frames').value)
        self.scan_frames = int(self.get_parameter('scan_frames').value)
        self.keep_frames = max(self.n_frames, self.scan_frames)
        self.scan_dir = Path(self.get_parameter('scan_dir').value or Path.home() / 'd2_data' / 'scan')
        self.tcp_info = self._load_tcp_info()     # 자세 자료 pose.json 의 tcp 칸(capture_scene 과 같음) — 켤 때 한 번
        self.scans = OrderedDict()            # run_id → {'scanner': StructureScanner, 'color': BGR 또는 None} — 추론 전 판만
        self.scan_lock = threading.Lock()
        self.find_lock = threading.Lock()     # BlockFinder 가 마지막 마스크를 들고 있어 한 번에 하나씩
        self.design_lock = threading.Lock()   # 설계 캐시(RunDesignCache)를 check_progress · find_blocks(assembly)가 같이 쓴다 — 동시에 부르면 안전하지 않다
        self.image_seq = 0                    # 공급 find_blocks 그림 번호(응답 image_seq, 1부터) — 작업 관리자 state pick.image_seq 가 어느 그림을 고른 것인지 맞춘다
        self.find_ctx = None                  # 마지막 공급 find 그림 재료 dict(seq · color · blocks · masks · mask_index · frame_id) — 한 번에 통째로 바꾼다
        self.pick_drawn = None                # 지난번 pick 으로 다시 그린 열쇠(pick_redraw_key) — on_state 만 쓴다(자기 그룹이라 차례로 돈다)
        self.warned = set()                   # 한 번만 남긴 경고 종류(state JSON · pick 꼴 · 그림) — state 는 자주 와서 같은 경고가 쌓이지 않게

        self.cfg = self._load_robot_yaml()
        self.T_tcp2c = self._load_calib()
        self.tcp = TcpPose(self, self.cfg.get('frame_id', 'base_link'), self.cfg.get('tcp_link', 'rg2_tcp'))
        self.finder = BlockFinder(finder_cfg(self.cfg))
        self.yolo = self._load_yolo()
        self.bases = self._load_bases()
        self.service_s = float(self.cfg['timeout']['service_s'])   # get_design 한 번의 제한 시간(task 와 같은 값)
        self.designs = RunDesignCache()       # (design_source, design_id) → BlockChecker — 설계마다 · 판(run_id)마다 한 번만 읽는다
        if self.get_parameter('design_source').value == 'local':
            self.get_logger().info('설계 읽기: local — recipe_dir 파일만 (task 도 design_source:=local 이어야 한다)')
            if not self.get_parameter('recipe_dir').value:
                self.get_logger().warn('recipe_dir 파라미터가 비었다 — check_progress 는 ERROR 로 답한다')
        else:
            self.get_logger().info('설계 읽기: remote — /d2/hmi/get_design (제한 %.1f초, task 도 remote 여야 한다)' % self.service_s)

        group = ReentrantCallbackGroup()
        prefix = self.get_parameter('cam_prefix').value
        self.create_subscription(Image, f'{prefix}/aligned_depth_to_color/image_raw', self.on_depth, 10, callback_group=group)
        self.create_subscription(CameraInfo, f'{prefix}/color/camera_info', self.on_info, 10, callback_group=group)
        self.create_subscription(Image, f'{prefix}/color/image_raw', self.on_color, 10, callback_group=group)
        self.cli_design = self.create_client(JsonQuery, '/d2/hmi/get_design', callback_group=group)
        # check_progress 요청끼리만 차례로 처리한다 — 설계 캐시(RunDesignCache)는 동시에 부르면 안전하지 않다(같은 판 요청 둘이 겹치면 get_design 을 두 번 부르고,
        # 느린 지난 판의 답이 새 판 캐시를 덮는다). 대기 중인 요청은 executor 스레드를 잡지 않는다(잠금으로 막으면 잡는다 — 스레드가 모자라 get_design 응답이
        # 못 돌아 TIMEOUT 이 난다). get_design 응답 · 카메라 · TF 구독은 다른 그룹이라 안 막힌다. 새 서비스를 이 캐시에 붙이면 같은 그룹에 둔다.
        self.create_service(CheckProgress, '/d2/vision/check_progress', self.on_check, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(JsonQuery, '/d2/vision/scan_capture', self.on_scan_capture, callback_group=group)
        self.create_service(JsonQuery, '/d2/vision/scan_infer', self.on_scan_infer, callback_group=group)
        self.create_service(JsonQuery, '/d2/vision/find_blocks', self.on_find_blocks, callback_group=group)
        # 작업 관리자 state/2 의 pick(고른 블록 · 우선순위) → 마지막 공급 find 그림에 번호를 덧그려 한 번 더 낸다(10/14 22시 PL).
        # 자기 그룹 — state 콜백끼리 차례로 돌아 pick_drawn 을 잠금 없이 쓴다
        self.create_subscription(String, '/d2/task/state', self.on_state, QOS_STATE, callback_group=MutuallyExclusiveCallbackGroup())
        self.pub_status = self.create_publisher(String, '/d2/vision/camera_status', QOS_STATUS)
        self.pub_image = self.create_publisher(CompressedImage, '/d2/vision/wrist_image', 10)   # IRD E-67 QoS 기본(VOLATILE)
        self.pub_preview = self.create_publisher(String, '/d2/vision/scan_preview', 10)         # IRD E-82 QoS 기본(VOLATILE) — 보기 전용
        self.create_timer(1.0 / self.get_parameter('status_hz').value, self.publish_status, callback_group=group)
        self.get_logger().info('wrist_block 시작: check_progress · scan_capture · scan_infer · find_blocks 대기 (카메라 %s, 스캔 폴더 %s)'
                               % (prefix, self.scan_dir))

    # ---------- 설정 읽기 ----------
    def _load_tcp_info(self):
        """d2_bringup config/tcp.json → {'name','pos','verified_at'}(자세 자료 pose.json 의 tcp 칸, capture_scene 과 같음). 못 읽으면 None(경고 없음 — 기록용)."""
        try:
            t = json.loads((Path(get_package_share_directory('d2_bringup')) / 'config' / 'tcp.json').read_text(encoding='utf-8'))
            return {'name': t.get('name'), 'pos': t.get('pos'), 'verified_at': t.get('verified_at')}
        except Exception:
            return None

    def _load_robot_yaml(self):
        """d2_bringup config/robot.yaml → dict. 없으면 예외(설정 없이 돌지 않는다)."""
        p = Path(get_package_share_directory('d2_bringup')) / 'config' / 'robot.yaml'
        return yaml.safe_load(p.read_text(encoding='utf-8'))

    def _load_calib(self):
        """hand-eye 4x4(mm, 카메라 → rg2_tcp). calib_path 가 비면 d2_vision share/config. 못 읽으면 None(모든 답 unknown)."""
        p = self.get_parameter('calib_path').value or str(default_calib_path())
        self.tilt_shift = load_tilt_shift(p)          # 기울기 밀림 고침(mm, 보정 json — 10/10 E-83), 없으면 0
        try:
            T = load_calib(p)
            self.get_logger().info('보정값 %s (카메라 위치 mm %s · 기울기 밀림 %.1f mm)' % (p, T[:3, 3].round(1).tolist(), self.tilt_shift))
            return T
        except (OSError, ValueError) as e:
            self.get_logger().error('보정값을 못 읽음 %s: %s — 모든 블록을 unknown 으로 답한다' % (p, e))
            return None

    def _load_yolo(self):
        """yolo_model 파라미터의 YOLO seg 모델을 읽어 돌려준다. yolo_fallback 이 true 면 읽지 않고 None(예비 마스크) — 로그 한 번.
        켤 때 읽는 이유: 처음 읽기는 몇 초 걸려 find_blocks 의 3초 제한(timeout.service_s) 안에 넣지 않으려고.
        읽은 뒤 빈 640×480 그림으로 한 번 미리 돌린다 — 첫 추론 준비(10/8 실측 약 0.9초)를 첫 find_blocks 에서 빼려고.
        실패 때: ultralytics 없음 · 파일 없음 · 읽기 · 미리 돌리기 실패면 오류 로그 후 SystemExit(1) — 노드가 켜지지 않는다(E-86)."""
        if self.get_parameter('yolo_fallback').value:
            self.get_logger().warn('find_blocks 마스크: 예비(엣지 + 깊이, E-47) — yolo_fallback:=true (YOLO 안 읽음)')
            return None
        path = self.get_parameter('yolo_model').value
        try:
            from ultralytics import YOLO      # 로봇 PC 에서 노드를 돌리는 python 에 있어야 한다 — 없으면 노드가 안 켜짐
            if not Path(path).is_file():
                raise FileNotFoundError(path)
            model = YOLO(path)
            t0 = time.monotonic()
            model(np.zeros((480, 640, 3), np.uint8), imgsz=640, conf=float(self.get_parameter('yolo_conf').value), verbose=False,
                  retina_masks=True)             # _yolo_masks 와 같은 인자로 미리 한 번
            warm_s = time.monotonic() - t0
        except Exception as e:                # ImportError · 파일 없음 · 모델 형식 · 미리 돌리기 실패 — 무엇이든 켜지 않는다
            self.get_logger().error('YOLO 모델을 못 읽어 wrist_block 을 켜지 않는다(%s: %s, yolo_model=%r) — '
                                    'ultralytics · 모델 파일을 확인하거나, 예비 마스크로 시험하려면 -p yolo_fallback:=true (E-86)'
                                    % (type(e).__name__, e, path))
            raise SystemExit(1)
        self.get_logger().info('find_blocks 마스크: YOLO seg %s (conf %.2f) · 미리 돌리기 %.2f초' % (
            path, self.get_parameter('yolo_conf').value, warm_s))
        return model

    def _load_bases(self):
        """recipe_dir 의 기본 설계(이름에 CHAIR · DESK, 레시피 두 파일 `_recipe.json` + `_placements.csv`, E-69) → blocks/2 목록. scan_infer 가 family 를 고르는 데 쓴다(nearest_base).
        폴더가 비었거나 못 읽은 설계는 빼고 경고 — 하나도 없으면 스캔 family 는 unknown."""
        recipe_dir = self.get_parameter('recipe_dir').value
        if not recipe_dir:
            self.get_logger().warn('recipe_dir 이 비었다 — scan_infer 의 family 는 unknown 으로 답한다')
            return []
        bases = []
        for r_path in sorted(Path(recipe_dir).glob('*_recipe.json')):
            design_id = r_path.name[:-len('_recipe.json')]
            if family_of(design_id) == 'unknown':
                continue
            try:
                pair = load_recipe_files(recipe_dir, design_id)
                if pair is None:                  # 조립 방법 파일이 없다 — 후보에서 뺀다
                    continue
                bases.append(structure_to_blocks(*pair, design_id, family_of(design_id), 0))
            except (OSError, KeyError, TypeError, ValueError, IndexError) as e:
                self.get_logger().warn('기본 설계 %s 를 blocks/2 로 못 바꿈: %s — 스캔 family 후보에서 뺀다' % (design_id, e))
        self.get_logger().info('스캔 family 후보(기본 설계): %s' % [b['design_id'] for b in bases])
        return bases

    def checker_for(self, design_id, run_id):
        """설계 이름 → (그 설계의 BlockChecker 또는 None, 실패 코드). design_source 에 따라 원격(기본) 또는 로컬 파일 하나만 쓴다.

        입력: design_id · run_id(check_progress 요청의 run_id — 같은 판이면 읽어 둔 설계를 쓰고, 지난 요청과 다르면 새 판이라 다시 읽는다, E-55 ① · E-60 ②.
        빈 값은 판 구분 없음 — 빈 값끼리는 캐시를 쓰고 이름 있는 판과 오가면 다시 읽는다). 요청한 블록이 몇 개인지는 보지 않는다.
        출력: 성공 (BlockChecker, '') · 실패 (None, 'ERROR' 또는 'TIMEOUT') — 부르는 쪽이 그 코드로 check_progress 를 답한다.
        바깥 영향: 로그, remote 면 get_design 조회(설계마다 · 판마다 한 번). 캐시 열쇠는 (design_source, design_id) —
        돌리는 중에 design_source 를 바꿔도 다른 길로 읽은 설계를 쓰지 않는다. 다시 읽기가 실패하면 지난 판의 설계도 버린다(옛 설계로 답하지 않는다).
        실패는 캐시하지 않는다 — 웹 · 다리가 늦게 떠도, 레시피 파일을 고쳐도 다음 요청에 다시 읽는다."""
        if not design_id:                     # design_id 칸이 비었다 — 블록 이름에서 설계를 잘라 내지 않는다(E-52 ④). 로그는 on_check 가 남긴다
            return None, 'ERROR'
        source = 'local' if self.get_parameter('design_source').value == 'local' else 'remote'
        load = self._remote_checker if source == 'remote' else self._local_checker
        return self.designs.get((source, design_id), run_id, lambda: load(design_id))

    def _local_checker(self, design_id):
        """design_source:=local — `<recipe_dir>/<design_id>_recipe.json`(구조 — load_recipe 가 옆 `_placements.csv` 도 붙임)
        → recipe_parts(robot.yaml, 레시피) → BlockChecker. part_id 는 조립 방법의 part_id 칸('<design_id 대문자>_<부품 이름>').
        출력: (BlockChecker, '') 또는 (None, 'ERROR') — 파일 없음 · 이름 문자 이상 · 조립 방법 파일 없음 · 형식 틀림(옛 cad_* 포함) ·
        recipe_sha256 다름 · 블록 크기 다름. 바깥 영향: 파일 읽기 · 로그."""
        recipe_dir = self.get_parameter('recipe_dir').value
        path = recipe_path(recipe_dir, design_id, RECIPE_SUFFIXES)
        if path is None:
            self.get_logger().error('레시피 %r 파일을 못 찾음(recipe_dir=%r, 끝 %s)' % (design_id, recipe_dir, ' · '.join(RECIPE_SUFFIXES)))
            return None, 'ERROR'
        try:
            chk = self._make_checker(load_recipe(str(path)))
        except (OSError, KeyError, TypeError, ValueError) as e:
            self.get_logger().error('레시피 %s 를 못 읽음(%s): %s' % (design_id, path, e))
            return None, 'ERROR'
        self.get_logger().info('레시피 %s (%s): 블록 %d개' % (design_id, path.name, len(chk.parts)))
        return chk, ''

    def _make_checker(self, recipe):
        """레시피 dict(load_recipe 모양, mm) → 팀 공용 recipe_parts(robot.yaml) → BlockChecker. 실패는 recipe_parts 의 예외 그대로."""
        o = self.cfg['assembly_origin']
        return BlockChecker(recipe_parts(self.cfg, recipe), (o['x_m'], o['y_m']),
                            float(self.cfg['assembly_area_half_m']), self.cfg['block_actual_m'])

    def _remote_checker(self, design_id):
        """/d2/hmi/get_design 으로 design/4.0 을 받아 BlockChecker 를 만든다. 출력: (BlockChecker, '') 또는 (None, 코드).

        요청 {"design_id"} → 최대 timeout.service_s 기다림(서비스 콜백 안이라 _call — 멀티스레드 실행기 + 재진입 그룹).
        실패 코드는 task_node.get_design 과 맞춘다: 서버 없음(지금 안 보임) → ERROR · 시간 초과(요청을 치움) → TIMEOUT ·
        success=false → ERROR(서버 이유는 로그에만 — 10/7 합의) · JSON · design/4.0 형식(옛 design/1 은 거절) · 짝 해시 · 레시피 내용 오류 → ERROR.
        로컬 파일로 몰래 대신하지 않는다. 바깥 영향: get_design 요청 하나 · 로그."""
        if not self.cli_design.service_is_ready():
            self.get_logger().error('get_design 서버가 없다(다리 · 웹?) — 설계 %s 를 못 받음 → ERROR' % design_id)
            return None, 'ERROR'
        req = JsonQuery.Request()
        req.request_json = json.dumps({'design_id': design_id}, ensure_ascii=False)
        res = self._call(self.cli_design, req, self.service_s)
        if res is None:
            self.get_logger().error('get_design 이 %.1f초 안에 답하지 않음 — 설계 %s → TIMEOUT' % (self.service_s, design_id))
            return None, 'TIMEOUT'
        if not res.success:
            self.get_logger().error('get_design 실패(%r) — 설계 %s → ERROR' % (res.reason, design_id))
            return None, 'ERROR'
        try:
            chk = self._make_checker(recipe_from_design(json.loads(res.response_json), design_id))
        except (AttributeError, IndexError, KeyError, TypeError, ValueError) as e:   # JSON 깨짐(ValueError) · design/4.0 형식 · 레시피 내용 — 바깥 입력이라 넓게 잡아 콜백이 죽지 않게
            self.get_logger().error('get_design 답으로 설계 %s 를 못 읽음: %s → ERROR' % (design_id, e))
            return None, 'ERROR'
        self.get_logger().info('설계 %s (get_design): 블록 %d개' % (design_id, len(chk.parts)))
        return chk, ''

    # ---------- 카메라 ----------
    def on_depth(self, msg):
        """정렬된 깊이 프레임을 mm float 로 모은다(최근 max(n_frames, scan_frames) 개)."""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
        if msg.encoding != '16UC1':           # 32FC1 이면 m
            img *= 1000.0
        img[img <= 0] = np.nan                # 깊이 0 = 구멍. 중앙값 · 평균에 섞이지 않게 NaN
        with self.lock:
            self.frames.append((self.now(), img))
            del self.frames[:-self.keep_frames]
            self.last_frame_t = self.frames[-1][0]

    def on_color(self, msg):
        """컬러 프레임 마지막 한 장을 BGR 로 둔다(scan_infer 사진 · find_blocks 입력 · wrist_image 바탕). 깊이는 이 영상에 맞춰 정렬돼 있다."""
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.color = (self.now(), img, msg.header.frame_id)

    def on_info(self, msg):
        """camera_info → 내부 파라미터 (fx, fy, ppx, ppy) 픽셀 단위."""
        self.intr = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def now(self):
        """노드 시계의 지금 시각(초)."""
        return self.get_clock().now().nanoseconds / 1e9

    def publish_status(self):
        """`/d2/vision/camera_status`(camera_status/1) 2 Hz. last_frame_stamp 는 마지막 깊이 프레임을 받은 시각(초)."""
        msg = {'schema': 'camera_status/1', 'stamp': self.now(), 'node': 'wrist_block', 'last_frame_stamp': self.last_frame_t}
        self.pub_status.publish(String(data=json.dumps(msg)))

    # ---------- 로봇 ----------
    def _call(self, cli, req, wait_s=SERVICE_WAIT_S):
        """서비스를 부르고 답을 기다린다(멀티스레드 실행기 + 재진입 그룹이라 콜백 중에도 됨). 서버 기다림 wait_s + 답 기다림 wait_s
        (초 — get_design 은 timeout.service_s, 기본 SERVICE_WAIT_S)만큼 막힌다. 시간 안에 못 받으면 보낸 요청을 치우고 None."""
        if not cli.wait_for_service(timeout_sec=wait_s):
            return None
        done = threading.Event()
        fut = cli.call_async(req)
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(wait_s):
            cli.remove_pending_request(fut)
            return None
        return fut.result()

    def read_tcp(self):
        """지금 rg2_tcp 자세 T_base←rg2_tcp 4x4 (mm). 파라미터 tcp_pose_mm 가 0 이 아니면 그것(시험용), 아니면 TF(최대 SERVICE_WAIT_S).
        못 받으면 None(브링업 · robot_state_publisher 없음). 바깥 영향 없음(/tf 구독뿐)."""
        p = list(self.get_parameter('tcp_pose_mm').value)
        if any(abs(v) > 1e-9 for v in p):
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat(p[3:7]).as_matrix()
            T[:3, 3] = p[:3]
            return T
        return self.tcp.matrix_mm(SERVICE_WAIT_S)

    def fresh_frames(self, t_req, n=None):
        """t_req 뒤에 들어온 깊이 프레임이 n 장(기본 n_frames) 모일 때까지(최대 FRESH_WAIT_S) 기다렸다가 최근 n 장을 돌려준다. 부족하면 None."""
        n = self.n_frames if n is None else n
        deadline = t_req + FRESH_WAIT_S
        while True:
            with self.lock:
                fresh = [img for t, img in self.frames if t > t_req]
            if len(fresh) >= n:
                return fresh[-n:]
            if self.now() > deadline:
                return None
            threading.Event().wait(0.02)

    # ---------- 서비스 ----------
    def _fill(self, res, ids, state='unknown', success=False, reason=''):
        """결과 배열을 요청 길이로 채운다(모두 같은 길이 — IRD 5장)."""
        n = len(ids)
        res.part_ids = list(ids)
        res.states = [state] * n
        res.dx_m, res.dy_m, res.dz_m, res.top_z_m = [NAN] * n, [NAN] * n, [NAN] * n, [NAN] * n
        res.success, res.reason = success, reason
        return res

    def on_check(self, req, res):
        """check_progress 콜백. 입력 req.design_id(설계 이름 — 빈 값이면 ERROR) · req.run_id(조립 한 판의 ID — 바뀌면 설계를 다시 읽음, 빈 값 = 판 구분 없음) ·
        req.part_ids(전체 블록 이름). 출력 res(배열은 모두 요청 길이).

        보정값 확인 → 설계 고르기 · 읽기(checker_for — remote 면 처음 보는 설계 · run_id 가 바뀐 판만 get_design) → 요청 뒤 새 깊이 프레임 n장 중앙값
        + TF 자세 → 점군 → BlockChecker → 답. 바깥 영향: get_design · 로그 · wrist_image 작업대 그림 한 장(요청 뒤 컬러가 있을 때만,
        설계 윗면을 판정 색으로 — _publish_bench_image. 그림 실패는 경고만, 답은 그대로).
        실패: 보정값 없음 · design_id 칸이 빔 · 설계를 못 읽음 → ERROR(get_design 시간 초과만 TIMEOUT), 프레임 · TF 자세 없음 → TIMEOUT
        (모두 state unknown, 값 NaN)."""
        ids = list(req.part_ids)
        if self.T_tcp2c is None:              # 보정값이 없으면 설계를 받아도 답할 수 없다 — get_design 을 부르지 않는다
            self.get_logger().error('보정값이 없어 답할 수 없다 → ERROR')
            return self._fill(res, ids, reason='ERROR')
        # 설계 = design_id 칸만(E-52 ④ — 블록 이름에서 잘라 내지 않는다. BACK · BASE · BEAM 에도 '_B' 가 있다).
        # 한 요청은 한 설계라고 본다 — 그 설계에 없는 블록은 BlockChecker 가 그 블록만 unknown 으로 답한다
        design_id = req.design_id
        if not design_id:
            self.get_logger().error('check_progress 요청의 design_id 칸이 비었다(블록 %s …) — 설계를 고를 수 없다 → ERROR' % ids[:1])
            return self._fill(res, ids, reason='ERROR')
        with self.design_lock:
            checker, reason = self.checker_for(design_id, req.run_id)
        if checker is None:
            self.get_logger().error('설계 %r 를 못 읽어 답할 수 없다 → %s' % (design_id, reason))
            return self._fill(res, ids, reason=reason)
        t_req = self.now()
        T_b2t = self.read_tcp()                                           # 로봇은 멈춰 있으니 프레임 모으기와 순서는 무관
        if T_b2t is None:
            self.get_logger().error('TF base_link → rg2_tcp 를 못 받음(브링업?) → TIMEOUT')
            return self._fill(res, ids, reason='TIMEOUT')
        frames = self.fresh_frames(t_req)
        if frames is None or self.intr is None:
            self.get_logger().error('요청 뒤 새 깊이 프레임이 %.1f초 안에 %d장 안 모임(카메라 끊김?) → TIMEOUT' % (FRESH_WAIT_S, self.n_frames))
            return self._fill(res, ids, reason='TIMEOUT')

        with warnings.catch_warnings():                                    # 10장 모두 구멍인 화소는 numpy 가 'All-NaN slice' 를 알리지만 결과(NaN → 점에서 빠짐)는 의도한 것
            warnings.simplefilter('ignore', RuntimeWarning)
            depth_m = np.nanmedian(np.stack(frames), axis=0) / 1000.0    # 구멍(NaN)은 빼고 중앙값. 전부 구멍이면 NaN → 점에서 빠짐
        T_b2c = camera_pose_mm(T_b2t, self.T_tcp2c, self.tilt_shift)
        T_b2c[:3, 3] /= 1000.0                                            # 노드 안은 m
        pts = depth_to_base_points(depth_m, self.intr, T_b2c, stride=2)
        results = checker.check(pts, ids)

        res.part_ids = ids
        res.states = [r['state'] for r in results]
        res.dx_m = [r['dx_m'] for r in results]
        res.dy_m = [r['dy_m'] for r in results]
        res.dz_m = [r['dz_m'] for r in results]
        # IRD 5장: absent·unknown 은 top_z 도 NaN (BlockChecker 는 absent 때 보이는 면 높이를 주지만 서비스 답에서는 뺀다)
        res.top_z_m = [r['top_z_m'] if r['state'] in ('present', 'occluded') else NAN for r in results]
        res.success, res.reason = True, ''
        # 로그에는 absent 때 보이는 면(작업면) 높이도 적는다 — 보정 기울기 진단용(10/7 박진용 실기: 윗면 −2.8 mm). 서비스 답은 IRD 5장대로 NaN
        self.get_logger().info('check_progress %s %d개 (점 %d, 손끝 z %.0f mm) → %s' % (
            design_id, len(ids), len(pts), T_b2t[2, 3], {r['part_id']: (r['state'], None if np.isnan(r['top_z_m']) else round(r['top_z_m'] * 1000, 1)) for r in results}))
        # 작업대 그림(박진용 안 ②, 10/14 22시 PL): 물은 부품의 설계 윗면을 판정 색으로. 설계에 없는 part_id 는 자리를 몰라 뺀다
        parts = [{'part_id': r['part_id'], 'center': checker.parts[r['part_id']]['center'], 'rot': checker.parts[r['part_id']]['rot'],
                  'state': r['state']} for r in results if r['part_id'] in checker.parts]
        self._publish_bench_image(t_req, parts, T_b2c)
        return res

    # ---------- 스캔 · 흩뿌림 찾기 (JsonQuery) ----------
    @staticmethod
    def _fail(success, code, detail):
        """실패 응답 (success, reason, {"ok":false,"reason","detail"}). success=True 는 '답은 냈지만 결과가 나쁨'(mock_scan 과 같은 두 층)."""
        return success, ('' if success else code), {'ok': False, 'reason': code, 'detail': detail}

    def _answer(self, res, name, out, t0):
        """(success, reason, 응답 dict) → JsonQuery 응답 res. NaN 은 보내지 않는다(allow_nan=False — 걸리면 ERROR).
        바깥 영향: 로그 한 줄(처리 시간 s — task 는 timeout.service_s 만 기다린다)."""
        success, reason, body = out
        try:
            text = json.dumps(body, ensure_ascii=False, allow_nan=False)
        except (TypeError, ValueError) as e:
            success, reason, body = self._fail(False, 'ERROR', '응답을 JSON 으로 못 바꿈: %s' % e)
            text = json.dumps(body, ensure_ascii=False)
        res.success, res.reason, res.response_json = success, reason, text
        dt = time.monotonic() - t0
        if success and body.get('ok') is True:
            self.get_logger().info('%s 답 (%.2f초)' % (name, dt))
        elif success:                         # 답은 냈지만 결과가 나쁨(다시 스캔 안내 등) — 노드 고장은 아니다
            self.get_logger().warn('%s 결과 실패 (%.2f초): %s' % (name, dt, text[:300]))
        else:
            self.get_logger().error('%s 실패 (%.2f초): reason=%s %s' % (name, dt, reason, text[:300]))
        return res

    def _snapshot(self, n, need_color):
        """요청 뒤 새 깊이 n 장 평균 + 지금 TF 자세 × 손목 보정 + (need_color 면) 요청 뒤 컬러 한 장. on_check 와 같은 규칙.

        출력: ((깊이 (H, W) mm 평균 · 0 = 없음, 컬러 BGR 또는 None, frame_id, T_base2cam 4x4 m), None) 또는 (None, 실패 응답 셋).
        실패: 보정값 없음 → ERROR · TF 자세 못 받음 · 새 프레임(컬러)이 FRESH_WAIT_S 안에 안 옴 → TIMEOUT. 바깥 영향 없음."""
        if self.T_tcp2c is None:
            return None, self._fail(False, 'ERROR', '손목 보정값이 없다')
        t_req = self.now()
        T_b2t = self.read_tcp()                                           # 로봇은 멈춰 있으니 프레임 모으기와 순서는 무관
        if T_b2t is None:
            return None, self._fail(False, 'TIMEOUT', 'TF base_link → rg2_tcp 를 못 받음(브링업?)')
        frames = self.fresh_frames(t_req, n)
        if frames is None or self.intr is None:
            return None, self._fail(False, 'TIMEOUT', '요청 뒤 새 깊이 프레임이 %.1f초 안에 %d장 안 모임(카메라 끊김?)' % (FRESH_WAIT_S, n))
        with self.lock:
            color = self.color
        if color is not None and color[0] <= t_req:
            color = None                                                  # 요청 앞의 컬러(로봇이 움직이던 때일 수 있다)는 쓰지 않는다
        if need_color and color is None:
            return None, self._fail(False, 'TIMEOUT', '요청 뒤 새 컬러 프레임이 없다(카메라 끊김?)')
        T_b2c = camera_pose_mm(T_b2t, self.T_tcp2c, self.tilt_shift)
        T_b2c[:3, 3] /= 1000.0                                            # 노드 안은 m
        img, frame_id = (color[1], color[2]) if color is not None else (None, '')
        return (mean_depth_mm(frames), img, frame_id, T_b2c), None

    def on_scan_capture(self, req, res):
        """scan_capture 콜백: {"pose_id","run_id"} → 깊이 scan_frames 장 평균 + 컬러 YOLO 마스크 → StructureScanner.capture_pose → {"ok":true,"points"}.

        run_id 마다 StructureScanner 하나(최근 MAX_SCAN_RUNS 개만), 같은 pose_id 가 다시 오면 그 자세 촬영을 바꾼다. 컬러 한 장도 둔다.
        YOLO 는 답하기 전에 한 번 돈다(답 시간 +YOLO 한 번 — 마지막 자세 마스크가 scan_infer 전에 꼭 붙게). find_lock 으로 find_blocks 와
        번갈아 쓰고 scan_lock 밖에서 돈다. 컬러가 없거나 yolo_fallback 이면 마스크 없이 넣고 경고 로그만(실패로 하지 않음).
        답한 뒤 따로 스레드가 점군 미리보기를 낸다(_send_preview — 촬영 잠금 안에서 뜬 복사본 · 순번으로, 답 시간 3초 제한에 안 걸리게).
        실패는 맨 위 '스캔 · 찾기 실패 때'. 바깥 영향: 로그 · 미리보기 스레드 하나."""
        t0 = time.monotonic()
        try:
            body = parse_request(req.request_json, ('pose_id', 'run_id'))
        except ValueError as e:
            return self._answer(res, 'scan_capture', self._fail(False, 'SCAN_FAILED', str(e)), t0)
        snap, err = self._snapshot(self.scan_frames, need_color=False)
        if snap is None:
            return self._answer(res, 'scan_capture', err, t0)
        depth_mm, color, _, T = snap
        rid, pose_id = body['run_id'], body['pose_id']
        masks, yolo_ms = [], 0.0
        if color is None or self.yolo is None:
            self.get_logger().warn('scan_capture %s %s: %s — 마스크 없이 넣는다(스캔 추론 정확도가 떨어질 수 있다)' % (
                rid, pose_id, '요청 뒤 컬러 없음' if color is None else 'YOLO 안 읽음(yolo_fallback)'))
        else:
            t_y = time.monotonic()
            with self.find_lock:
                masks, _ = self._yolo_masks(color)
            masks = masks or []
            yolo_ms = (time.monotonic() - t_y) * 1000
        try:
            with self.scan_lock:
                run = self.scans.get(rid)
                if run is None:
                    run = self.scans[rid] = {'scanner': StructureScanner(self.cfg), 'color': None, 'n_capture': 0}
                self.scans.move_to_end(rid)
                while len(self.scans) > MAX_SCAN_RUNS:
                    old, _ = self.scans.popitem(last=False)
                    self.get_logger().warn('scan_infer 가 안 온 run_id %s 의 촬영을 버린다(최근 %d판만 기억)' % (old, MAX_SCAN_RUNS))
                sc = run['scanner']
                n = sc.capture_pose(pose_id, depth_mm, self.intr, T, masks=masks)   # 같은 자세 다시 찍음 → 덮어쓰기
                n_mask = len(sc.captures[-1]['masks'])
                run['n_capture'] += 1
                index, snap = run['n_capture'], sc.snapshot()                    # 미리보기용 — 잠금 밖 스레드가 이 복사본만 읽는다
                if color is not None:
                    run['color'] = color
                poses = [c['pose_id'] for c in sc.captures]
        except ValueError as e:
            return self._answer(res, 'scan_capture', self._fail(True, 'SCAN_FAILED', '점군을 못 만듦: %s' % e), t0)
        self.get_logger().info('scan_capture %s %s: 점 %d · 마스크 %d (YOLO %.0f ms) (모은 자세 %s, 카메라 z %.0f mm)' % (
            rid, pose_id, n, n_mask, yolo_ms, poses, T[2, 3] * 1000))
        threading.Thread(target=self._after_capture, args=(snap, rid, pose_id, index, depth_mm, color, T), daemon=True).start()
        return self._answer(res, 'scan_capture', (True, '', {'ok': True, 'points': int(n)}), t0)

    def _after_capture(self, snap, rid, pose_id, index, depth_mm, color, T):
        """(scan_capture 답 뒤 스레드) 점군 미리보기(_send_preview) → 자세 자료(_save_capture_files) 차례로. 둘은 따로 — 하나가 실패해도 다른 하나는 한다."""
        self._send_preview(snap, rid, pose_id, index)
        self._save_capture_files(rid, pose_id, index, depth_mm, color, T)

    def _save_capture_files(self, rid, pose_id, index, depth_mm, color, T):
        """(답 뒤 스레드) 이 자세의 사진 · 평균 깊이 · 자세를 <scan_dir>/<run_id>/<pose_id>_* 로 남긴다(structure_scanner.save_capture — capture_scene 과 같은 칸).
        입력: depth_mm = scan_frames 장 평균(mm) · color = 요청 뒤 컬러 또는 None · T = T_base2cam(m). 바깥 영향: 파일 3개(컬러가 없으면 2개) · 로그.
        실패(폴더 · 파일 · cv2): 경고 로그만 — 답 · 미리보기 · scan_infer 와 무관."""
        try:
            files = save_capture(self.scan_dir / rid, pose_id, depth_mm, color, self.intr, T,
                                 {'run_id': rid, 'index': int(index), 'tcp': self.tcp_info})
        except Exception as e:                # 자료 모으기 — 무엇이 실패해도 스캔은 그대로 간다
            self.get_logger().warn('scan_capture %s #%d (%s) 자세 자료를 못 남김: %r' % (rid, index, pose_id, e))
            return
        self.get_logger().info('scan_capture %s #%d (%s) 자세 자료: %s%s' % (
            rid, index, pose_id, ' · '.join(f.name for f in files), '' if color is not None else ' (요청 뒤 컬러 없음 — 사진 건너뜀)'))

    def _send_preview(self, snap, rid, pose_id, index):
        """(미리보기 스레드) 지금까지 모은 점군 → <scan_dir>/<run_id>/preview_<index>.ply → /d2/vision/scan_preview 발행(E-82, IRD 6장 scan_preview/1.0).
        입력: snap = 촬영 잠금 안에서 뜬 StructureScanner 복사본 · index = 이번 run 의 scan_capture 순번(1부터).
        바깥 영향: 파일 쓰기 · 토픽 한 번 · 로그. 실패(폴더 · 파일 · 발행): 경고 로그만 — scan_capture 답은 이미 나갔고 scan_infer 와도 무관."""
        t0 = time.monotonic()
        try:
            msg = save_preview(snap, self.scan_dir / rid, rid, pose_id, index)
            self.pub_preview.publish(String(data=json.dumps(msg, ensure_ascii=False)))
        except Exception as e:                # 보기 전용 — 무엇이 실패해도 스캔은 그대로 간다
            self.get_logger().warn('scan_preview %s #%d (%s) 를 못 냄: %r — 화면 미리보기만 빠진다' % (rid, index, pose_id, e))
            return
        size = Path(msg['cloud_path']).stat().st_size if Path(msg['cloud_path']).exists() else 0
        self.get_logger().info('scan_preview %s #%d (%s): 점 %d · %.0f KB · %.0f ms' % (
            rid, index, pose_id, msg['points'], size / 1024, (time.monotonic() - t0) * 1000))

    def _next_scan_number(self):
        """스캔 임시 이름 번호 = scan_dir 에 이미 저장된 blocks.json 수 + 1 (scan_<family>_<번호> — 노드를 다시 켜도 안 겹치게)."""
        try:
            return len(list(self.scan_dir.glob('*/blocks.json'))) + 1
        except OSError:
            return 1

    def on_scan_infer(self, req, res):
        """scan_infer 콜백: {"run_id"} → StructureScanner.infer(기본 설계로 family) → 파일 저장 → scan_response.

        저장: <scan_dir>/<run_id>/color.png(마지막 촬영 컬러) · cloud.ply(save_cloud, ≤ 2 MB) · blocks.json(번호 세기 · 기록).
        PLY 저장이 실패하면 cloud_path 를 빼고 ok:true + 경고(IRD 4.2 선택 칸, 10/8 PL). 사진을 못 쓰면 image_path ''.
        추론이 끝난 run_id 는 성공 · 실패 모두 메모리에서 지운다(다시 스캔 = 새 run_id). 바깥 영향: 파일 쓰기 · 로그."""
        t0 = time.monotonic()
        try:
            rid = parse_request(req.request_json, ('run_id',))['run_id']
        except ValueError as e:
            return self._answer(res, 'scan_infer', self._fail(False, 'SCAN_FAILED', str(e)), t0)
        with self.scan_lock:
            run = self.scans.pop(rid, None)
        if run is None:
            return self._answer(res, 'scan_infer', self._fail(True, 'SCAN_FAILED', 'run_id %s 로 촬영한 자세가 없다(scan_capture 먼저)' % rid), t0)
        sc = run['scanner']
        try:
            result = sc.infer(bases=self.bases or None)
        except Exception as e:                # 계산 예외로 노드가 죽지 않게 — 원인은 로그로
            self.get_logger().error('scan_infer %s 계산 예외: %r' % (rid, e))
            return self._answer(res, 'scan_infer', self._fail(False, 'ERROR', '추론 계산 예외: %s' % e), t0)
        conf = result.get('confidence', {})
        self.get_logger().info('scan_infer %s: ok=%s %s · 블록 %d (추정 %d) · 설명률 %s · 가까운 기본 설계 %s' % (
            rid, result['ok'], result['reason'], len(result['blocks']['blocks']), result['inferred_count'],
            conf.get('explained'), result.get('nearest_base')))
        if not result['ok']:
            return self._answer(res, 'scan_infer', scan_response(result, '', None), t0)
        family = result['blocks'].get('family') or 'unknown'
        result['blocks']['family'] = family
        result['blocks']['design_id'] = 'scan_%s_%02d' % (family, self._next_scan_number())
        out_dir = self.scan_dir / rid
        image_path, cloud_path = '', None
        try:
            out_dir.mkdir(parents=True, exist_ok=True)
            if run['color'] is not None and cv2.imwrite(str(out_dir / 'color.png'), run['color']):
                image_path = str(out_dir / 'color.png')
            else:
                self.get_logger().warn('scan_infer %s: 촬영 때 컬러가 없어(또는 못 써서) image_path 를 비운다' % rid)
        except OSError as e:
            self.get_logger().warn('scan_infer %s: 폴더 · 사진을 못 씀 %s — image_path 를 비운다' % (rid, e))
        try:
            # IRD E-67: PLY 가 없으면 화면 점군 창만 빈다 — 블록 결과는 그대로 넘긴다(cloud_path 칸 뺌). cloud_path 는 선택 칸(IRD 4.2, 10/8 PL)
            n_pts = sc.save_cloud(str(out_dir / 'cloud.ply'))
            cloud_path = str(out_dir / 'cloud.ply')
            self.get_logger().info('scan_infer %s: 점군 %d점 · %.0f KB (복셀 %.1f mm)' % (
                rid, n_pts, (out_dir / 'cloud.ply').stat().st_size / 1024, sc.cloud_voxel_m * 1000))
        except (OSError, ValueError) as e:
            self.get_logger().warn('scan_infer %s: 점군 PLY 를 못 씀 %s — cloud_path 를 빼고 답한다(선택 칸 — 화면 점군 창만 빈다)' % (rid, e))
        out = scan_response(result, image_path, cloud_path)
        try:
            (out_dir / 'blocks.json').write_text(json.dumps(out[2]['blocks'], ensure_ascii=False, indent=1), encoding='utf-8')
        except OSError as e:
            self.get_logger().warn('scan_infer %s: blocks.json 을 못 씀 %s (응답은 그대로)' % (rid, e))
        return self._answer(res, 'scan_infer', out, t0)

    def _yolo_masks(self, color):
        """YOLO seg 로 블록 마스크 · 신뢰도(find_blocks · scan_capture 공용, find_lock 안에서 부른다).
        yolo_fallback 이라 모델을 안 읽었거나 추론 예외면 (None, None) → find_blocks 는 예비 마스크(엣지 + 깊이), scan_capture 는 마스크 없이."""
        if self.yolo is None:
            return None, None
        try:
            r = self.yolo(color, imgsz=640, conf=float(self.get_parameter('yolo_conf').value), verbose=False,
                          retina_masks=True)[0]                          # retina_masks: 마스크를 원본 크기(640×480)로 받는다
            return masks_from_yolo(r, color.shape[:2])
        except Exception as e:                # 추론이 실패해도 찾기는 예비로 · 스캔은 마스크 없이 계속한다
            self.get_logger().error('YOLO 추론 실패(%r) — 이번 find_blocks 는 예비 마스크로 · scan_capture 는 마스크 없이' % (e,))
            return None, None

    def publish_wrist_image(self, color, blocks, frame_id):
        """find_blocks 검출 결과를 그린 그림 한 장을 `/d2/vision/wrist_image`(JPEG 70, 640×480 그대로)로 낸다. 실패는 경고만."""
        try:
            img = draw_found(color, blocks, self.finder.last_masks, self.finder.last_info.get('mask_index', []))
        except (ValueError, IndexError, cv2.error) as e:
            self.get_logger().warn('wrist_image 를 못 만듦: %s' % e)
            return
        self._publish_jpeg(img, frame_id)

    def _publish_jpeg(self, img, frame_id):
        """다 그린 BGR 그림 한 장을 `/d2/vision/wrist_image`(JPEG 70, 크기 그대로)로 낸다. 인코딩 실패는 경고만(발행 안 함)."""
        try:
            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ok:
                raise ValueError('JPEG 인코딩 실패')
        except (ValueError, cv2.error) as e:
            self.get_logger().warn('wrist_image 를 못 만듦: %s' % e)
            return
        msg = CompressedImage(format='jpeg', data=buf.tobytes())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        self.pub_image.publish(msg)

    def on_find_blocks(self, req, res):
        """find_blocks 콜백: {"run_id"[, "height_map"][, "area"]} → 요청 뒤 컬러 · 깊이(scan_frames 장 평균) → BlockFinder
        → {"ok":true,"blocks":[…],"image_seq":n[, "height_map":{…}]} + wrist_image 한 장. area spare 면 _find_spare(받침 위 블록 하나, 깊이만) ·
        area assembly 면 _find_assembly(둘 다 image_seq 없음).
        image_seq = 공급 찾기를 계산할 때마다 1 늘리는 번호(find_lock 안) — 그 찾기의 컬러 · 블록 · 마스크를 find_ctx 로 기억해 두었다가
        작업 관리자 state pick 이 이 번호를 가리키면 on_state 가 번호를 덧그린다.

        블록 0개도 ok true(빈 목록 → 작업 관리자 WAIT_SUPPLY). 선택 칸 값이 틀리면 SCAN_FAILED. 계산 예외 → success=false ERROR.
        나머지 실패는 맨 위. 바깥 영향: wrist_image 발행 · 로그. run_id 는 기록용(로그)."""
        t0 = time.monotonic()
        try:
            body = parse_request(req.request_json, ('run_id',))
            want_map, area = find_options(body)
            target = assembly_target(body) if area == 'assembly' else None
        except ValueError as e:
            return self._answer(res, 'find_blocks', self._fail(False, 'SCAN_FAILED', '요청 오류: %s' % e), t0)
        rid = body['run_id']
        if area == 'spare':
            return self._find_spare(res, rid, t0, want_map)
        if area == 'assembly':
            return self._find_assembly(res, rid, target, t0)
        snap, err = self._snapshot(self.scan_frames, need_color=True)
        if snap is None:
            return self._answer(res, 'find_blocks', err, t0)
        depth_mm, color, frame_id, T = snap
        try:
            with self.find_lock:
                self.image_seq += 1                                       # find_lock 안 — 번호와 기억한 그림 재료가 같은 찾기 것이게
                seq = self.image_seq
                masks, scores = self._yolo_masks(color)
                blocks = self.finder.find(color, depth_mm / 1000.0, self.intr, T, masks=masks, scores=scores)
                self.publish_wrist_image(color, blocks, frame_id)
                info = self.finder.last_info
                # 작업 관리자가 고른 뒤(state pick) 번호 · 고른 블록을 덧그릴 재료 — 복사하지 않고 참조만(다음 find 가 새 목록을 만든다)
                self.find_ctx = {'seq': seq, 'color': color, 'blocks': blocks, 'masks': self.finder.last_masks,
                                 'mask_index': list(info.get('mask_index', [])), 'frame_id': frame_id}
            out = {'ok': True, 'blocks': blocks, 'image_seq': seq}   # 그림을 못 냈어도 번호는 싣는다(작업 관리자가 그대로 pick 에 넣음)
            if want_map:
                # 찾기와 같은 깊이 · 자세로 다시 만든다(blocks_from_masks 안의 점을 꺼내 두지 않으려고 — 640×480 에 수십 ms)
                out['height_map'] = height_grid(base_image(depth_mm / 1000.0, self.intr, T), supply_map_area(self.finder.cfg['area_m'], self.cfg))
        except Exception as e:                # 계산 예외로 노드가 죽지 않게 — 원인은 로그로
            self.get_logger().error('find_blocks %s 계산 예외: %r' % (rid, e))
            return self._answer(res, 'find_blocks', self._fail(False, 'ERROR', '찾기 계산 예외: %s' % e), t0)
        self.get_logger().info('find_blocks %s (%s): 블록 %d · 작업면 %.1f mm · 뺀 마스크 %d → %s' % (
            rid, 'YOLO' if masks is not None else '예비', len(blocks), info.get('table_z_m', NAN) * 1000, len(info.get('skipped', [])),
            [(b['up'][0], b['yaw_deg'], b['overlap'], b['confidence']) for b in blocks]))
        return self._answer(res, 'find_blocks', (True, '', out), t0)

    def _find_spare(self, res, rid, t0, want_map=False):
        """find_blocks area spare: 요청 뒤 깊이(scan_frames 장 평균) + TF 자세 → base 점 → measure_on_spare(받침 spare_spots[0]).
        출력: JsonQuery 응답 res — {"ok":true,"blocks":[블록 하나]} 또는 빈 받침 {"ok":true,"blocks":[]}. YOLO 안 씀 · 재기는 깊이만.
        요청 뒤 컬러가 있으면 wrist_image 한 장(잰 블록 윗면 · 목표 자리 상자 · 가운데 차이, block_finder.draw_spare) — 그림이 실패하거나
        컬러가 없으면 그림만 건너뛴다(답 · 측정값은 그대로).
        want_map 이면 같은 깊이로 받침 둘레 높이 지도(spare_area)를 "height_map" 칸에 더한다 — 다시 잡기 전 손가락 자리 확인(one_pick p1_to_recipe, #20).
        실패: 보정값 없음 → ERROR · TF · 깊이 시간 초과 → TIMEOUT(_snapshot) · spare_spots 없음 · 계산 예외 → ERROR. 바깥 영향: 로그 · wrist_image."""
        snap, err = self._snapshot(self.scan_frames, need_color=False)   # 컬러는 있으면 그림에만 — 없다고 실패하지 않는다
        if snap is None:
            return self._answer(res, 'find_blocks(spare)', err, t0)
        depth_mm, color, frame_id, T = snap
        try:
            pts = depth_to_base_points(depth_mm / 1000.0, self.intr, T, stride=1)
            b = measure_on_spare(pts, self.cfg['spare_spots'][0], self.cfg['block_actual_m'])
        except Exception as e:                # robot.yaml spare_spots 없음 · 계산 예외로 노드가 죽지 않게 — 원인은 로그로
            self.get_logger().error('find_blocks(spare) %s 예외: %r' % (rid, e))
            return self._answer(res, 'find_blocks(spare)', self._fail(False, 'ERROR', '받침 재기 예외: %s' % e), t0)
        self.get_logger().info('find_blocks(spare) %s: 점 %d → %s' % (
            rid, len(pts), '빈 받침' if b is None else (b['up'], b['x_m'], b['y_m'], b['yaw_deg'])))
        if color is not None:
            self._publish_spare_image(color, b, T, frame_id)
        out = {'ok': True, 'blocks': [b] if b else []}
        if want_map:
            out['height_map'] = height_grid(pts, spare_map_area(self.cfg))
        return self._answer(res, 'find_blocks(spare)', (True, '', out), t0)

    def _publish_spare_image(self, color, b, T, frame_id):
        """받침 다시 재기 그림(잰 블록 b 또는 None · 목표 = spare_spots[0] 눕힘)을 wrist_image 로 낸다. 그림 예외는 경고만 — 답은 그대로."""
        try:
            size = self.cfg['block_actual_m']
            img = draw_spare(color, b, ideal_spare_block(self.cfg['spare_spots'][0], size), self.intr, T, size)
        except Exception as e:                # 그림 때문에 받침 재기 답이 실패하면 안 된다
            self.get_logger().warn('wrist_image(spare) 를 못 만듦: %r' % (e,))
            return
        self._publish_jpeg(img, frame_id)

    def _publish_bench_image(self, t_req, parts, T, measured=None, note='', color=None):
        """작업대 그림(설계 자리 윗면 · 판정 색 — block_finder.draw_workbench)을 wrist_image 한 장으로 낸다(박진용 안 ②, 10/14 22시 PL).

        입력: t_req = 요청 받은 시각(s) — color 가 없으면 그 뒤에 들어온 마지막 컬러를 쓴다(앞 장은 로봇이 움직이던 때일 수 있어 안 씀) ·
              parts = draw_workbench 의 부품 목록 · T = T_base2cam 4x4 m(그 요청의 자세) · measured · note = draw_workbench 그대로 ·
              color = (BGR, frame_id) 이미 받은 컬러(find_blocks assembly — _snapshot 이 고른 장).
        컬러 · camera_info 가 없으면 조용히 건너뛴다(draw_spare 와 같게 — 답 · 측정은 그대로). 그림 예외는 경고 한 번만. 바깥 영향: wrist_image 발행."""
        if color is None:
            with self.lock:
                last = self.color
            if last is None or last[0] <= t_req:
                return
            color = (last[1], last[2])
        if self.intr is None or not parts:
            return
        try:
            img = draw_workbench(color[0], parts, self.intr, T, self.cfg['block_actual_m'], measured=measured, note=note)
        except Exception as e:                # 그림 때문에 check_progress · 다시 재기 답이 실패하면 안 된다
            self._warn_once('bench_draw', 'wrist_image(작업대) 를 못 만듦: %r — 그림만 건너뜀(이 경고는 한 번만)' % (e,))
            return
        self._publish_jpeg(img, color[1])

    def _warn_once(self, kind, text):
        """같은 종류(kind) 경고는 노드가 뜬 동안 한 번만 남긴다 — state 는 자주 와서 같은 꼴 오류가 로그를 덮지 않게."""
        if kind not in self.warned:
            self.warned.add(kind)
            self.get_logger().warn(text)

    def on_state(self, msg):
        """/d2/task/state(state/2 JSON) 콜백 — 'pick' 칸이 마지막 공급 find 그림(image_seq)을 가리키고 지난번과 다르면
        그 그림에 우선순위 번호(ranks) · 고른 블록(index) 채우기를 덧그려 wrist_image 를 한 장 더 낸다(10/14 22시 PL).

        판단은 block_finder.pick_redraw_key(시험 있음): pick 없음 · image_seq null · 다른 그림 번호 · 지난번과 같음이면 아무것도 안 한다.
        state 는 다른 칸이 바뀌어도 다시 오므로 같은 pick 으로 다시 그리지 않는다. JSON 이 아님 · pick 꼴 틀림 · 그림 실패는 종류마다 경고 한 번만
        (콜백이 죽지 않는다). 바깥 영향: wrist_image 발행 · 로그."""
        try:
            st = json.loads(msg.data)
        except ValueError:
            self._warn_once('state_json', '/d2/task/state 를 JSON 으로 못 읽음 — pick 그림을 건너뜀(이 경고는 한 번만)')
            return
        if not isinstance(st, dict):
            self._warn_once('state_json', '/d2/task/state 가 JSON 객체가 아님 — pick 그림을 건너뜀(이 경고는 한 번만)')
            return
        ctx = self.find_ctx                   # find 콜백이 통째로 바꾸므로 한 번 읽어 둔다
        try:
            key = pick_redraw_key(ctx['seq'] if ctx else None, self.pick_drawn, st.get('pick'))
        except ValueError as e:
            self._warn_once('pick', '/d2/task/state pick 칸 꼴이 틀림(%s) — 그림을 건너뜀(이 경고는 한 번만)' % e)
            return
        if key is None:
            return
        self.pick_drawn = key                 # 그림이 실패해도 같은 pick 으로 다시 시도하지 않는다(같은 실패가 반복되지 않게)
        try:
            img = draw_found(ctx['color'], ctx['blocks'], ctx['masks'], ctx['mask_index'], ranks=list(key[2]), pick=key[1])
        except Exception as e:                # 보기 전용 — 무엇이 실패해도 노드 · 작업은 그대로
            self._warn_once('pick_draw', 'wrist_image(pick) 를 못 만듦: %r — 그림만 건너뜀(이 경고는 한 번만)' % (e,))
            return
        self._publish_jpeg(img, ctx['frame_id'])
        self.get_logger().info('wrist_image: 그림 #%d 에 고른 블록 %d · 순위 %s 를 덧그림' % (key[0], key[1], list(key[2])))


    def _find_assembly(self, res, rid, target, t0):
        """find_blocks area assembly: 조립 자리에 방금 놓은 블록 하나(design_id · part_id) → 설계 자리(check_progress 와 같은 설계 캐시)
        둘레 깊이(scan_frames 장 평균) + TF 자세 → measure_placed. W144(한석형) 측정 입력 — 응답 꼴은 10/14 한석형 · 민범진.
        출력: {"ok":true, "part_id", "blocks":[블록 하나] 또는 [], "rejected": null 또는 거절 코드(placed_measure.REJECTS), "detail",
              "measured_stamp": 쓴 깊이 중 마지막 장을 받은 시각(초, 노드 시계)}. 측정 거절 · 블록 없음(NO_PART)은 ok true(이번엔 못 잼 — 다시 집지 않음).
        실패(success false): 설계에 없는 part_id → SCAN_FAILED '요청 오류' · 설계를 못 읽음 → ERROR(get_design 시간 초과 TIMEOUT) ·
        보정값 없음 · 계산 예외 → ERROR · TF · 깊이 시간 초과 → TIMEOUT. 바깥 영향: 로그(remote 면 get_design — 보통 check_progress 로 이미 캐시) ·
        요청 뒤 컬러가 있으면 wrist_image 작업대 그림 한 장(설계 자리 빨간 상자 + 잰 블록 초록 + placed_note 글자, 박진용 안 ②).
        placed_method:=mask 면 깊이 결과 뒤에 _placed_by_mask 가 크기 · 위치를 마스크로 다시 잰다(응답 꼴 같음, 거기서 실패하면 깊이 결과 그대로)."""
        design_id, part_id = target
        with self.design_lock:
            checker, reason = self.checker_for(design_id, rid)
        if checker is None:
            return self._answer(res, 'find_blocks(assembly)', self._fail(False, reason, '설계 %s 를 못 읽음' % design_id), t0)
        part = checker.parts.get(part_id)
        if part is None:
            return self._answer(res, 'find_blocks(assembly)', self._fail(False, 'SCAN_FAILED', '요청 오류: 설계 %s 에 없는 part_id %s' % (design_id, part_id)), t0)
        snap, err = self._snapshot(self.scan_frames, need_color=False)
        if snap is None:
            return self._answer(res, 'find_blocks(assembly)', err, t0)
        with self.lock:
            stamp = self.last_frame_t                                     # _snapshot 이 쓴 마지막 깊이 장(바로 뒤 새 장이 왔으면 그 장 — 33 ms 안)
        depth_mm, color, frame_id, T = snap
        try:
            pts = depth_to_base_points(depth_mm / 1000.0, self.intr, T, stride=1)
            b, why = measure_placed(pts, {'part_id': part_id, 'center': part['center'], 'rot': part['rot']}, self.cfg['block_actual_m'])
        except Exception as e:                # 계산 예외로 노드가 죽지 않게 — 원인은 로그로
            self.get_logger().error('find_blocks(assembly) %s %s 예외: %r' % (rid, part_id, e))
            return self._answer(res, 'find_blocks(assembly)', self._fail(False, 'ERROR', '놓은 블록 재기 예외: %s' % e), t0)
        if self.get_parameter('placed_method').value == 'mask':
            b, why = self._placed_by_mask(rid, part_id, part, depth_mm, color, T, b, why)
        out = {'ok': True, 'part_id': part_id, 'blocks': [b] if b else [], 'rejected': None if b else why[0],
               'detail': '' if b else why[1], 'measured_stamp': round(float(stamp), 3)}
        c = part['center']
        self.get_logger().info('find_blocks(assembly) %s %s: 점 %d → %s' % (
            rid, part_id, len(pts), '거절 %s — %s' % why if b is None else
            '잼 dx %.1f · dy %.1f mm · yaw %.1f° · 윗면 %s mm' % ((b['x_m'] - c[0]) * 1000, (b['y_m'] - c[1]) * 1000, b['yaw_deg'], b['size_mm'])))
        if color is not None:                 # 작업대 그림: 설계 자리(빨간 목표 상자) + 잰 블록(초록) + 차이 · 거절 코드 글자(박진용 안 ②)
            self._publish_bench_image(None, [{'part_id': part_id, 'center': c, 'rot': part['rot'], 'state': None}], T,
                                      measured=b, note=placed_note(part_id, b, c, out['rejected']), color=(color, frame_id))
        return self._answer(res, 'find_blocks(assembly)', (True, '', out), t0)

    def _placed_by_mask(self, rid, part_id, part, depth_mm, color, T, b, why):
        """_find_assembly 의 placed_method:=mask 갈래 — 깊이 결과 (b, why) 를 YOLO 마스크 + 광선 결과(placed_mask.measure_placed_mask)로 바꾼다.

        입력: depth_mm = 평균 깊이 (H, W) mm · color = 요청 뒤 컬러 BGR 또는 None · T = 카메라 → base 4x4 m · (b, why) = measure_placed 결과.
        출력: (블록 dict 또는 None, 거절 (코드, 설명) 또는 None) — measure_placed 와 같은 꼴. 쓴 방법 · 까닭은 로그 한 줄에만(응답 칸 그대로).
        실패 때: 컬러 없음 · YOLO 안 읽음(yolo_fallback) · 추론 실패 · 계산 예외면 받은 깊이 결과를 그대로 돌려주고 경고 로그(노드는 계속).
        바깥 영향: YOLO 한 번(find_lock 안 — find_blocks · scan_capture 와 차례로) · 로그."""
        if color is None:
            self.get_logger().warn('find_blocks(assembly) %s %s: placed_method mask 인데 요청 뒤 컬러가 없다 — 깊이 결과로' % (rid, part_id))
            return b, why
        try:
            raw = self.get_parameter('placed_mask_shift_px').value
            try:
                shift = [float(v) for v in raw]
                if len(shift) != 2:
                    raise ValueError
            except (TypeError, ValueError):
                self._warn_once('mask_shift', 'placed_mask_shift_px 가 [du, dv] 두 수가 아니다(%r) — 옮기지 않음(이 경고는 한 번만)' % (raw,))
                shift = [0.0, 0.0]
            with self.find_lock:
                masks, _ = self._yolo_masks(color)
            if masks is None:
                self.get_logger().warn('find_blocks(assembly) %s %s: YOLO 마스크가 없다(yolo_fallback · 추론 실패) — 깊이 결과로' % (rid, part_id))
                return b, why
            P = base_image(depth_mm / 1000.0, self.intr, T)
            bm, wm, info = measure_placed_mask_detail(P, masks, {'part_id': part_id, 'center': part['center'], 'rot': part['rot']},
                                                      self.cfg['block_actual_m'], self.intr, T, shift)
        except Exception as e:                # 마스크 쪽 예외로 노드가 죽거나 답을 못 하지 않게 — 깊이 결과로 돌아간다
            self.get_logger().warn('find_blocks(assembly) %s %s: 마스크 재기 예외 %r — 깊이 결과로' % (rid, part_id, e))
            return b, why
        self.get_logger().info('find_blocks(assembly) %s %s: 방법 %s%s · 마스크 %d개 · 픽셀 %d · 옮김 %s px · 깊이 %s' % (
            rid, part_id, info['method'], ' (%s)' % info['reason'] if info['reason'] else '', len(masks), info['mask_px'], info['shift_px'],
            info['depth_reject'] or 'OK %s mm' % info['depth_size_mm']))
        return bm, wm


def main(args=None):
    """노드를 멀티스레드 실행기로 돌린다(서비스 콜백 안에서 get_design 서비스 · TF 를 기다리기 위해). 로봇에 명령이 없어 서기 처리는 없다."""
    rclpy.init(args=args)
    node = WristBlock()
    exe = MultiThreadedExecutor(num_threads=4)
    exe.add_node(node)
    try:
        exe.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
