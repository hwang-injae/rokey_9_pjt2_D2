#!/usr/bin/env bash
# 웹 PC에서 사용자가 sudo로 실행: 방화벽 적용 및 재부팅 자동 적용 설치. 브로커는 기동하지 않는다.
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo 'sudo로 실행해야 합니다.' >&2; exit 1; }
robot_ip="${1:?로봇 PC IPv4}"
web_ip="${2:?웹 PC Wi-Fi IPv4}"
source_script="$(dirname -- "$(realpath -- "$0")")/mqtt_access.sh"
if [[ -n "$(docker ps --filter publish=1883 --format '{{.ID}}')" ]]; then
    echo '먼저 MQTT 브로커를 정지해야 합니다.' >&2
    exit 1
fi
bash "$source_script" apply "$robot_ip" "$web_ip"
install -d /usr/local/libexec
install -m 755 "$source_script" /usr/local/libexec/d2-mqtt-access.sh
cat > /etc/d2-mqtt-access.env <<EOF
D2_ROBOT_IPV4=$robot_ip
D2_WEB_IPV4=$web_ip
EOF
cat > /etc/systemd/system/d2-mqtt-access.service <<'EOF'
[Unit]
Description=D2 MQTT access restriction
Requires=docker.service
After=docker.service network-online.target
Wants=network-online.target
PartOf=docker.service

[Service]
Type=oneshot
EnvironmentFile=/etc/d2-mqtt-access.env
ExecStart=/bin/bash /usr/local/libexec/d2-mqtt-access.sh apply ${D2_ROBOT_IPV4} ${D2_WEB_IPV4}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable d2-mqtt-access.service
systemctl restart d2-mqtt-access.service
bash /usr/local/libexec/d2-mqtt-access.sh check "$robot_ip" "$web_ip"
systemctl is-enabled d2-mqtt-access.service
systemctl is-active d2-mqtt-access.service
iptables -w -S DOCKER-USER
# 명시한 경우에만 규칙 check 뒤 브로커를 기동한다. 웹 프로세스는 sudo 호출자의 UID/GID로 실행한다.
if [[ "${3:-}" == --start ]]; then
    D2_WEB_IPV4="$web_ip" WEB_UID="${SUDO_UID:?sudo 호출자 UID 필요}" WEB_GID="${SUDO_GID:?sudo 호출자 GID 필요}" \
        docker compose -f "$(dirname -- "$source_script")/../web/compose.yaml" up -d
fi
