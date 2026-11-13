# policy_bridge — Franka FR3 Robot Server ↔ Policy Server 브릿지

- `robot_server.py`: FR3 + RealSense 2대를 WebSocket(JSON)으로 노출하는 Robot Server. 1 kHz 제어는 별도 프로세스
  (pylibfranka joint position control + `reference_filter.py`의 jerk 제한 필터)
- `robot_client2.py`: Robot Server ↔ Policy Server(msgpack, SSH 터널 지원) 브릿지 클라이언트
- `mock_servers.py`: 로봇/정책 서버 없이 테스트하는 가상 서버

## 설치 (새 PC)

```bash
git clone <this repo> ~/policy_bridge
python3 -m venv ~/venvs/franka          && ~/venvs/franka/bin/pip install -r ~/policy_bridge/requirements-server.txt
python3 -m venv ~/venvs/policy_bridge   && ~/venvs/policy_bridge/bin/pip install -r ~/policy_bridge/requirements.txt pillow
# 그리퍼 드라이버: ~/fr3_bridge 를 같이 복사 (다른 위치면 FR3_BRIDGE_DIR=<경로>)
sudo apt install ethtool linux-tools-$(uname -r)    # setup_rt.sh / rt_check.sh 용
```

필수 조건: PREEMPT_RT 커널, 사용자에게 rtprio 99 허용(`/etc/security/limits.conf`의 `@realtime` 그룹 등),
로봇 전용 유선 랜카드가 `172.16.0.x` 대역으로 FCI(`172.16.0.2`)에 직결.

## 실행

```bash
./run_server.sh                     # 터미널 1 (sudo 비밀번호 1회)
./run_client.sh --max-decisions 5   # 터미널 2 (SSH 비밀번호 입력)
```

`run_server.sh` 순서: 기존 서버 종료(SIGINT) → 카메라 점유 확인 → `sudo setup_rt.sh` → 카메라 하드웨어 리셋 → 서버 시작.
로그: `server.log`, `client.log`. 반사(reflex) 발생 시 직전 0.5 s 제어 데이터가 `fault_dumps/fault_*.npz`에 저장됨.

환경 변수: `ROBOT_IP`(기본 172.16.0.2), `CONTROL_CPU`(제어 CPU 고정), `FRANKA_VENV`, `CLIENT_VENV`, `FR3_BRIDGE_DIR`.

## 실시간 설정 (`setup_rt.sh`, root)

1 kHz FCI 링크는 PC가 수 ms만 멈춰도 `joint_motion_generator_*_discontinuity` 반사로 정지한다. 노트북(HP 245 G8)에서
실측으로 찾은 원인과 대응:

| 원인 (측정 방법) | 대응 |
|---|---|
| CPU 깊은 idle 상태 복귀 ~3 ms (`rtla timerlat`) | latency > 1 µs인 cpuidle 상태 비활성화 |
| Realtek Wi-Fi(rtw89) PCIe 레지스터 읽기 2.9 ms 정지 (`rtla timerlat` 스택) | rtw89 ASPM/CLKREQ/절전 끔 (`/etc/modprobe.d/rtw89-realtime.conf`) |
| 로봇 NIC 수신 softirq가 일반 우선순위 (반사 직전 기록) | NIC IRQ 전부 한 CPU로, 그 CPU의 ksoftirqd/IRQ 스레드 FIFO 90, 제어 프로세스도 그 CPU에 단독 고정 |
| NIC 절전 | EEE, flow control, PCIe ASPM 끔 |

점검: `sudo ./rt_check.sh` (60초, 1 ms 넘는 지연이 생기면 원인 스택 출력 → `rtla.log`).
Franka는 Intel 유선 랜카드를 권장한다. Realtek 유선(r8169)에서는 위 설정 후에도 명령이 연속 유실된 사례가 있다.
