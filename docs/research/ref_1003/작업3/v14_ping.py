#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""작업 3 (여유 V-14): 두 PC 사이 ROS 2 토픽의 왕복 시간을 잰다. 시계를 맞출 필요가 없다.

PC B (또는 컨테이너):  python3 v14_ping.py pong
PC A:                 python3 v14_ping.py ping --rate 50 --seconds 60

ping 쪽이 /v14/ping 에 보낸 메시지를 pong 쪽이 /v14/pong 으로 바로 돌려보낸다.
ping 쪽은 5초마다 받은 수, 잃은 수, 지연(왕복 / 2)의 평균·최대를 출력한다.
두 PC는 같은 ROS_DOMAIN_ID, 같은 ROS 2 배포판이어야 한다. ROS만 있으면 되고 워크스페이스는 필요 없다.
"""
import argparse
import sys
import time

import rclpy
from std_msgs.msg import Header


def run_pong(node):
    pub = node.create_publisher(Header, 'v14/pong', 10)
    node.create_subscription(Header, 'v14/ping', pub.publish, 10)
    print('pong: /v14/ping 을 받아 /v14/pong 으로 돌려보낸다. 끝내려면 Ctrl+C')
    rclpy.spin(node)


def run_ping(node, rate, seconds):
    sent = {}
    rtts = []
    pub = node.create_publisher(Header, 'v14/ping', 10)

    def on_pong(msg):
        t_sent = sent.pop(msg.frame_id, None)
        if t_sent is not None:
            rtts.append(time.monotonic() - t_sent)

    node.create_subscription(Header, 'v14/pong', on_pong, 10)
    print(f'ping: {rate} Hz로 {seconds} s 동안 보낸다. pong 쪽을 먼저 띄운다.')
    seq = 0
    total_sent = 0
    period = 1.0 / rate
    t_end = time.monotonic() + seconds
    next_report = time.monotonic() + 5.0
    next_send = time.monotonic()
    all_rtts = []
    while time.monotonic() < t_end:
        now = time.monotonic()
        if now >= next_send:
            seq += 1
            msg = Header()
            msg.frame_id = str(seq)
            msg.stamp = node.get_clock().now().to_msg()
            sent[msg.frame_id] = time.monotonic()
            pub.publish(msg)
            total_sent += 1
            next_send += period
        rclpy.spin_once(node, timeout_sec=0.001)
        if time.monotonic() >= next_report:
            if rtts:
                half = [r * 500.0 for r in rtts]  # 왕복 / 2, ms
                print(f'  최근 5 s: 받음 {len(rtts)}, 지연 평균 {sum(half) / len(half):.1f} ms, '
                      f'최대 {max(half):.1f} ms')
            else:
                print('  최근 5 s: 받은 것 없음 (pong이 떠 있는지, ROS_DOMAIN_ID가 같은지 본다)')
            all_rtts += rtts
            rtts.clear()
            next_report += 5.0
    end_wait = time.monotonic() + 1.0
    while time.monotonic() < end_wait:
        rclpy.spin_once(node, timeout_sec=0.01)
    all_rtts += rtts
    lost = total_sent - len(all_rtts)
    print(f'\n결과: 보냄 {total_sent}, 받음 {len(all_rtts)}, 잃음 {lost}')
    if all_rtts:
        half = sorted(r * 500.0 for r in all_rtts)
        p95 = half[int(0.95 * (len(half) - 1))]
        print(f'지연(왕복/2): 평균 {sum(half) / len(half):.1f} ms, 95% {p95:.1f} ms, 최대 {half[-1]:.1f} ms')
        print('통과 기준: 잃음 0 (끊김 없음), 지연 50 ms 이하')


def main():
    ap = argparse.ArgumentParser(description='작업 3 V-14: PC 사이 왕복 지연')
    ap.add_argument('role', choices=['ping', 'pong'])
    ap.add_argument('--rate', type=float, default=50.0, help='보내는 횟수/초 (기본 50)')
    ap.add_argument('--seconds', type=float, default=60.0, help='재는 시간 s (기본 60)')
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node(f'v14_{args.role}')
    try:
        if args.role == 'pong':
            run_pong(node)
        else:
            run_ping(node, args.rate, args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
