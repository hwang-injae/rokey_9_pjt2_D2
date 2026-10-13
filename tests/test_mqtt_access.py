"""MQTT 방화벽 규칙 생성·확인 시험. iptables는 대역이며 실제 차단 검증은 아니다."""
import json
import os
from pathlib import Path
import subprocess

SCRIPT=Path(__file__).parents[1]/'tools/mqtt_access.sh'


def firewall(tmp_path):
    """iptables 대역과 규칙 저장파일을 만들어 실제 호스트 방화벽을 변경하지 않는다."""
    binary=tmp_path/'iptables'
    binary.write_text('''#!/usr/bin/env python3
import sys,json,os
from pathlib import Path
p=Path(os.environ['D2_RULES_FILE']);rules=json.loads(p.read_text()) if p.exists() else []
a=sys.argv[1:];a=a[1:] if a[0]=='-w' else a
op=a[0];args=a[2:]
if op=='-C':sys.exit(0 if args in rules else 1)
if op=='-D':rules.remove(args);p.write_text(json.dumps(rules))
if op=='-I':rules.insert(int(args[0])-1,args[1:]);p.write_text(json.dumps(rules))
if op=='-S':
 for rule in rules:
  r=list(rule)
  if '-s' in r:r[r.index('-s')+1]+='/32'
  print('-A DOCKER-USER '+' '.join(r))
''')
    binary.chmod(0o755)
    env=dict(os.environ,PATH=str(tmp_path)+os.pathsep+os.environ['PATH'],D2_RULES_FILE=str(tmp_path/'rules.json'))
    return env,tmp_path/'rules.json'


def test_apply_is_idempotent_and_deny_follows_allow(tmp_path):
    """규칙 적용 반복에도 중복이 없고 로봇 허용 뒤 다른 출발지 거부 순서가 유지된다."""
    env,path=firewall(tmp_path)
    for _ in range(2):
        subprocess.run(['bash',str(SCRIPT),'apply','172.24.0.137','172.24.2.213'],env=env,check=True)
    rules=json.loads(path.read_text())
    assert len(rules)==2
    assert rules[0][-1]=='RETURN' and '-s' in rules[0]
    assert rules[1][-1]=='DROP' and '-s' not in rules[1]
    assert '--ctorigdst' in rules[1] and '--ctorigdstport' in rules[1]


def test_check_rejects_rule_that_bypasses_restriction(tmp_path):
    """기존 무조건 RETURN이 앞에 있어 차단을 우회하면 확인을 실패시킨다."""
    env,path=firewall(tmp_path)
    subprocess.run(['bash',str(SCRIPT),'apply','172.24.0.137','172.24.2.213'],env=env,check=True)
    rules=json.loads(path.read_text());rules.insert(0,['-j','RETURN']);path.write_text(json.dumps(rules))
    assert subprocess.run(['bash',str(SCRIPT),'check','172.24.0.137','172.24.2.213'],env=env,capture_output=True).returncode!=0


def test_invalid_address_changes_nothing(tmp_path):
    """IPv6/잘못된 주소를 IPv4 공개규칙에 넣지 않고 거절한다."""
    env,path=firewall(tmp_path)
    for address in ('bad','::1','127.0.0.1'):
        assert subprocess.run(['bash',str(SCRIPT),'apply',address,'172.24.2.213'],env=env,capture_output=True).returncode!=0
    assert not path.exists()


def packet_result(rules,source,direction):
    """저장된 규칙의 출발지/원본방향 조건을 평가한다. 실제 kernel 대신 명시 모델이다."""
    for rule in rules:
        if '--ctdir' in rule and rule[rule.index('--ctdir')+1]!=direction:continue
        if '-s' in rule and rule[rule.index('-s')+1]!=source:continue
        return rule[-1]
    return 'CONTINUE'


def test_robot_request_passes_and_reply_is_not_dropped(tmp_path):
    """허용로봇SYN은통과하고응답SYN-ACK은거부규칙에서제외,다른요청만차단한다."""
    env,path=firewall(tmp_path)
    subprocess.run(['bash',str(SCRIPT),'apply','172.24.0.137','172.24.2.213'],env=env,check=True)
    rules=json.loads(path.read_text())
    assert packet_result(rules,'172.24.0.137','ORIGINAL')=='RETURN'
    assert packet_result(rules,'172.24.2.119','ORIGINAL')=='DROP'
    assert packet_result(rules,'172.18.0.2','REPLY')=='CONTINUE'


def test_legacy_bidirectional_rules_are_removed(tmp_path):
    """이전양방향DROP이새규칙뒤에남아응답을막지않도록정확히이전두규칙만제거한다."""
    env,path=firewall(tmp_path)
    base=['-p','tcp','-m','conntrack','--ctorigdst','172.24.2.213','--ctorigdstport','1883']
    path.write_text(json.dumps([base+['-s','172.24.0.137','-j','RETURN'],base+['-j','DROP']]))
    subprocess.run(['bash',str(SCRIPT),'apply','172.24.0.137','172.24.2.213'],env=env,check=True)
    rules=json.loads(path.read_text())
    assert len(rules)==2 and all('--ctdir' in rule for rule in rules)
    assert packet_result(rules,'172.18.0.2','REPLY')=='CONTINUE'
