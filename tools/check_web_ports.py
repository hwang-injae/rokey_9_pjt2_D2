"""현재 PC에서 웹 PC의 MQTT·화면 TCP 접속을 확인한다. 명령 발행·로봇 이동 없음."""
import argparse
import json
import socket


def main():
    """웹 IPv4로 각 포트에 연결 후 즉시 닫고 출발지·성공·실패를 JSON 출력한다."""
    parser=argparse.ArgumentParser()
    parser.add_argument('web_ip')
    args=parser.parse_args()
    results=[]
    for port in (1883,8000):
        sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        sock.settimeout(3)
        row=dict(destination=args.web_ip,port=port)
        try:
            sock.connect((args.web_ip,port))
            row.update(source_ip=sock.getsockname()[0],connected=True)
        except OSError as exc:
            row.update(source_ip=sock.getsockname()[0],connected=False,error=str(exc))
        finally:sock.close()
        results.append(row)
    print(json.dumps(results,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
