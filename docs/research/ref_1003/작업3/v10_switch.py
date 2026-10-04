#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""작업 3 (V-10): MoveIt2 이동과 두산 순응 하강·상승을 번갈아 10번 한다.

한 회차:
  1) MoveIt2로 놓을 자리 위(A 또는 C, 번갈아)로 간다.   [Enter로 계획, 경로 확인 뒤 y로 실행]
  2) 두산 명령                                           [Enter로 시작]
     현재 TCP 위치를 읽는다 -> task_compliance_ctrl() -> movel로 50 mm 아래(절대 목표)
     -> release_compliance_ctrl() -> movel로 처음 위치(절대 목표)로 돌아온다.
     movel은 베이스 기준, 속도 30 mm/s, 가속도 60 mm/s^2.
     움직인 뒤 실제 높이 변화를 읽어 50 mm(±2 mm)가 아니면 실패로 적는다.
  상대 이동(DR_MV_MOD_REL)은 쓰지 않는다. 가상 모드에서 MoveIt2 이동 직후 상대 이동을 보내면
  두산 제어기가 틀린 기준 위치로 목표를 계산해 'NOT REACHABLE' 알람을 내고, 반환값은 0(성공)으로
  돌아오는 것을 확인했다.
결과는 화면과 v10_log_시각.csv 파일(실행한 폴더)에 남는다.

먼저 할 것: source <내 워크스페이스>/install/setup.bash, MoveIt2 브링업이 떠 있어야 한다.
scene_setup.py로 책상과 그리퍼 대신 원기둥을 먼저 넣는다. move_test.py와 같은 폴더에 둔다.
--ns는 브링업 name 인자와 같아야 한다. MoveIt2 브링업 기본값은 빈 값이다 ('dsr01'이 아니다).
틀리면 시작할 때 '두산 서비스를 찾지 못했다'가 나오고 멈춘다.

예)
  python3 v10_switch.py                 # 실기: 10회, Enter·y로 한 단계씩
  python3 v10_switch.py --cycles 2      # 오전 가상 모드에서 미리 2회 돌려 보기
"""
import argparse
import csv
import sys
import time

import rclpy
import DR_init

from move_test import MoveItHelper, GOALS, fmt, ask_yes

ROBOT_MODEL = 'm0609'
MIN_TCP_Z = 40.0   # 내려간 뒤 TCP 높이(베이스 기준 mm)가 이보다 낮아지면 시작하지 않는다
DZ_TOL = 2.0       # 높이 변화 허용 차이 (mm)


def wait_enter(prompt):
    try:
        return input(prompt).strip().lower() != 'q'
    except EOFError:
        return False


def main():
    ap = argparse.ArgumentParser(description='작업 3 V-10: MoveIt2 <-> 두산 명령 번갈아 쓰기')
    ap.add_argument('--ns', default='', help='브링업 name 인자와 같은 값 (기본: 빈 값)')
    ap.add_argument('--cycles', type=int, default=10, help='회차 수 (기본 10)')
    ap.add_argument('--down', type=float, default=50.0, help='내려갈 거리 mm (기본 50, 최대 60)')
    ap.add_argument('--vel', type=float, default=30.0, help='movel 속도 mm/s (기본 30, 최대 50)')
    ap.add_argument('--acc', type=float, default=60.0, help='movel 가속도 mm/s^2 (기본 60, 최대 100)')
    ap.add_argument('--scale', type=float, default=0.1, help='MoveIt2 속도·가속도 비율 (기본 0.1, 최대 0.2)')
    args = ap.parse_args()
    if not (0 < args.down <= 60 and 0 < args.vel <= 50 and 0 < args.acc <= 100 and 0 < args.scale <= 0.2):
        print('[오류] 값이 허용 범위를 넘는다. --help를 본다.')
        return 1

    DR_init.__dsr__id = args.ns
    DR_init.__dsr__model = ROBOT_MODEL
    rclpy.init()
    node = rclpy.create_node('task3_v10_switch', namespace=args.ns)
    DR_init.__dsr__node = node  # DSR_ROBOT2를 불러오기 전에 정해야 한다
    try:
        import DSR_ROBOT2 as dr
        from DSR_ROBOT2 import (task_compliance_ctrl, release_compliance_ctrl, movel, posx,
                                get_current_posx, DR_BASE)
    except Exception as e:  # noqa: BLE001
        print(f'[오류] DSR_ROBOT2를 불러오지 못했다: {e}')
        print('       워크스페이스를 source 했는지 본다.')
        return 1

    def tcp_now():
        pos, _sol = get_current_posx(DR_BASE)
        return None if pos is None else [float(v) for v in pos]

    log_rows = []
    try:
        # 두산 서비스 이름 확인 (틀리면 이후 호출이 끝없이 기다리므로 여기서 멈춘다)
        for cli in (dr._ros2_movel, dr._ros2_task_compliance_ctrl, dr._ros2_release_compliance_ctrl,
                    dr._ros2_get_current_posx):
            if not cli.wait_for_service(timeout_sec=5.0):
                print(f'[오류] 두산 서비스를 찾지 못했다: {cli.srv_name}')
                print('       ros2 service list | grep move_line 으로 실제 이름을 보고, 앞부분이 다르면')
                print('       --ns를 브링업 name과 같게 준다 (MoveIt2 브링업 기본은 빈 값).')
                return 1
        print(f'두산 서비스 확인: {dr._ros2_movel.srv_name}')

        mv = MoveItHelper(node)
        if not mv.wait_ready():
            return 1

        spots = ['A', 'C']
        print(f'회차 {args.cycles}번. 놓을 자리: A, C 번갈아. 하강 {args.down:.0f} mm, '
              f'movel {args.vel:.0f} mm/s. MoveIt2 속도 비율 {args.scale}.')
        print('각 단계 전에 조작자가 펜던트를 들고 비상정지를 바로 누를 수 있는지 확인한다. q 입력 시 끝낸다.')
        n_ok = 0
        for i in range(1, args.cycles + 1):
            spot = spots[(i - 1) % 2]
            row = {'cycle': i, 'spot': spot, 'moveit': '', 'compliance_on': '', 'down': '',
                   'compliance_off': '', 'up': '', 'result': '', 'note': ''}
            log_rows.append(row)
            # 1) MoveIt2 이동
            if not wait_enter(f'\n[{i}/{args.cycles}] MoveIt2로 {spot} 로 이동. 계획하려면 Enter (q=끝): '):
                row['note'] = '사용자가 멈춤'
                break
            res, status, ptime = mv.plan(GOALS[spot], 'ompl', args.scale)
            if res is None:
                row['moveit'] = f'plan:{status}'
                row['result'] = 'FAIL'
                print(f'  [계획 실패] {status}. 같은 회차를 다시 하려면 이 스크립트를 다시 실행한다.')
                if not ask_yes('  다음 회차로 갈까요? y: '):
                    break
                continue
            print(f'  계획 {ptime:.3f} s. RViz에서 경로를 끝까지 본다.')
            if not ask_yes('  실행할까요? 펜던트를 손에 들고(비상정지 가능), 작업 영역에 손 없음이면 y: '):
                row['moveit'] = 'skipped'
                row['note'] = '실행 안 함'
                break
            status = mv.execute(res)
            row['moveit'] = status
            print(f'  MoveIt2 실행: {status}')
            if status != 'SUCCESS':
                row['result'] = 'FAIL'
                if not ask_yes('  실패를 기록했다. 다음 회차로 갈까요? y: '):
                    break
                continue
            # 2) 두산 순응 하강·상승 (절대 목표)
            if not wait_enter(f'  두산 명령: 순응 켜기 -> {args.down:.0f} mm 하강 -> 순응 끄기 -> 처음 높이로. '
                              f'Enter (q=끝): '):
                row['note'] = '사용자가 멈춤'
                break
            t0 = time.time()
            start = tcp_now()
            if start is None:
                row['result'] = 'FAIL'
                row['note'] = 'get_current_posx 실패'
                print('  [멈춤] 두산 현재 위치를 읽지 못했다.')
                break
            if start[2] - args.down < MIN_TCP_Z:
                row['result'] = 'FAIL'
                row['note'] = f'너무 낮음 z={start[2]:.0f}'
                print(f'  [멈춤] 지금 TCP 높이 {start[2]:.0f} mm. 내려가면 {MIN_TCP_Z:.0f} mm보다 낮아진다. '
                      f'목표 자세와 펜던트 TCP를 확인한다.')
                break
            down_target = list(start)
            down_target[2] -= args.down
            row['compliance_on'] = task_compliance_ctrl()
            row['down'] = 'skip'
            if row['compliance_on'] == 0:  # 순응 제어가 켜졌을 때만 내려간다
                ret = movel(posx(down_target), vel=args.vel, acc=args.acc, ref=DR_BASE)
                now = tcp_now()
                dz = None if now is None else now[2] - start[2]
                ok_dz = dz is not None and abs(dz + args.down) <= DZ_TOL
                row['down'] = ret if ok_dz else f'{ret}(dz={dz})'
            row['compliance_off'] = release_compliance_ctrl()
            ret = movel(posx(start), vel=args.vel, acc=args.acc, ref=DR_BASE)  # 처음 위치로
            now = tcp_now()
            dz_back = None if now is None else now[2] - start[2]
            row['up'] = ret if (dz_back is not None and abs(dz_back) <= DZ_TOL) else f'{ret}(dz={dz_back})'
            codes = [row['compliance_on'], row['down'], row['compliance_off'], row['up']]
            ok = all(c == 0 for c in codes)
            row['result'] = 'OK' if ok else 'FAIL'
            n_ok += 1 if ok else 0
            print(f'  두산 결과 (0=성공): 순응 켜기 {codes[0]}, 하강 {codes[1]}, 순응 끄기 {codes[2]}, '
                  f'복귀 {codes[3]}  ({time.time() - t0:.1f} s)  -> {row["result"]}')
            print(f'  현재 관절 [{fmt(mv.current_deg(), 2)}]')
            if not ok and not ask_yes('  실패를 기록했다. 다음 회차로 갈까요? y: '):
                break
        done = len([r for r in log_rows if r['result']])
        print(f'\n결과: {n_ok}/{args.cycles} 성공 (끝까지 간 회차 {done})')
        for r in log_rows:
            if r['result'] == 'FAIL':
                print(f'  실패 회차 {r["cycle"]}: MoveIt2={r["moveit"]} 순응켜기={r["compliance_on"]} '
                      f'하강={r["down"]} 순응끄기={r["compliance_off"]} 복귀={r["up"]} {r["note"]}')
        return 0
    except KeyboardInterrupt:
        print('\n중단했다. 로봇이 움직이고 있으면 비상정지.')
        return 4
    finally:
        if log_rows:
            name = time.strftime('v10_log_%H%M%S.csv')
            with open(name, 'w', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=list(log_rows[0].keys()))
                w.writeheader()
                w.writerows(log_rows)
            print(f'기록 파일: {name}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
