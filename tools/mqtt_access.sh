#!/usr/bin/env bash
# Docker 공개 MQTT 접근을 로봇 PC 한 곳으로 제한한다. apply는 방화벽 변경, check는 조회만 한다.
set -euo pipefail
mode="${1:?apply 또는 check}"
robot_ip="${2:?로봇 PC IPv4}"
web_ip="${3:?웹 PC Wi-Fi IPv4}"
python3 - "$robot_ip" "$web_ip" <<'PY'
import ipaddress,sys
for value in sys.argv[1:]:
    address=ipaddress.ip_address(value)
    if address.version != 4 or address.is_unspecified or address.is_loopback:
        raise SystemExit('실제 PC의 IPv4 주소가 필요합니다')
PY
[[ "$mode" == apply || "$mode" == check ]] || exit 2
# DNAT 전 목적지는 응답에도 남는다. ORIGINAL만 검사하여 SYN-ACK 응답을 거부하지 않는다.
legacy=(-p tcp -m conntrack --ctorigdst "$web_ip" --ctorigdstport 1883)
base=("${legacy[@]}" --ctdir ORIGINAL)
allow=("${base[@]}" -s "$robot_ip" -j RETURN)
deny=("${base[@]}" -j DROP)
if [[ "$mode" == check ]]; then
    iptables -w -C DOCKER-USER "${allow[@]}"
    iptables -w -C DOCKER-USER "${deny[@]}"
    # 허용이 거부보다 앞인지도 확인한다.
    iptables -w -S DOCKER-USER | python3 -c '
import sys
rules=[r for r in sys.stdin.read().splitlines() if r.startswith("-A DOCKER-USER ")];robot,web=sys.argv[1:]
a=[i for i,r in enumerate(rules) if "--ctorigdst "+web in r and "--ctorigdstport 1883" in r and "-s "+robot+"/32" in r and "-j RETURN" in r]
d=[i for i,r in enumerate(rules) if "--ctorigdst "+web in r and "--ctorigdstport 1883" in r and "-j DROP" in r]
if len(a)!=1 or len(d)!=1 or a[0]!=0 or d[0]!=1 or any("--ctdir ORIGINAL" not in rules[i] for i in a+d):raise SystemExit("규칙 순서/중복을 확인해야 합니다")
' "$robot_ip" "$web_ip"
    exit 0
fi
# 먼저 거부를 넣고 허용을 추가하여 적용 중 전체 공개되는 틈을 만들지 않는다.
iptables -w -S DOCKER-USER >/dev/null
if ! iptables -w -C DOCKER-USER "${deny[@]}" 2>/dev/null; then
    iptables -w -I DOCKER-USER 1 "${deny[@]}"
fi
if ! iptables -w -C DOCKER-USER "${allow[@]}" 2>/dev/null; then
    iptables -w -I DOCKER-USER 1 "${allow[@]}"
fi
# 새 요청방향 규칙을 먼저 설치한 뒤 이 버전에서 만든 옛 양방향 규칙만 제거한다.
# 다른 관리자의 규칙이나 전체 체인을 지우지 않는다.
for action in allow deny; do
    old=("${legacy[@]}")
    if [[ "$action" == allow ]]; then old+=(-s "$robot_ip" -j RETURN); else old+=(-j DROP); fi
    while iptables -w -C DOCKER-USER "${old[@]}" 2>/dev/null; do
        iptables -w -D DOCKER-USER "${old[@]}"
    done
done
bash "$0" check "$robot_ip" "$web_ip"
