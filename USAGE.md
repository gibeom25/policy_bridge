# Action chunk 실행 방식 (`robot_client2.py`) 사용법

정책이 주는 (16, 8) chunk를 openpi 방식으로 실행한다. 120 Hz 실행 루프와 정책 호출 루프가 따로 돌고,
chunk는 **관측 시점** 기준으로 하나의 15 Hz step 축(타임라인)에 놓인다.

- 추론하는 동안 이미 지나간 step은 버리고, 지금 실행 중인 step은 바꾸지 않는다.
- 새 chunk의 앞 `--blend-steps`개 step은 이전 chunk와 섞어서 이어 붙인다.
- 서버 필터 때문에 측정 관절값은 명령보다 약 0.25 s 늦다(`--obs-lag`). 이를 보정하지 않으면 chunk가 바뀔 때마다 로봇이 뒤로 끌려간다.
- 실행할 행동이 바닥나면 마지막 목표를 유지(underrun)하고, 다음 chunk가 오면 이어서 움직인다.

```bash
./run_client.sh --max-decisions 5                      # 기본: async (움직이면서 추론)
./run_client.sh --exec-mode sync                       # 정지 → 안정화 → 추론 → 8 step 실행 (이전 방식)
./run_client.sh --replan-steps 6 --obs-lag 0.3         # 튜닝 예
```

| 옵션 | 기본값 | 의미 |
|---|---|---|
| `--exec-mode` | `async` | `async`: 로봇이 움직이는 동안 추론. `sync`: openpi `ActionChunkBroker`처럼 멈추고 추론 |
| `--replan-steps` | 8 | chunk당 실행할 step 수(openpi `action_horizon`). async에서는 버퍼가 응답 전에 바닥날 것 같으면 더 일찍 호출 |
| `--blend-steps` | 3 | async: chunk 경계에서 섞는 step 수(0 = 바로 교체) |
| `--obs-lag` | 0.25 | async: 측정값이 명령값보다 늦는 시간(s). 서버 필터 wn=8 기준 2/wn |
| `--settle-tol`, `--settle-timeout` | 0.01, 0.4 | 정책 호출 전 안정화 대기. sync에서는 매번, async에서는 첫 호출만 |
| `--max-dq` | 0.25 | 한 step에 허용하는 최대 관절 변화(rad). 넘으면 안전 정지 |

**로그 보는 법** (`client.log`, 결정마다 한 줄):

```
[Decision 18] async | chunk@step 123, exec step 129: dropped=7 blended=2 used=9 seam=0.0005 rad | latency=161ms ... | underruns=0 overruns=0
```

| 항목 | 의미 / 대응 |
|---|---|
| `dropped` | 지연 때문에 버린 앞쪽 행동 수. 대략 (지연 + obs-lag) / 66.7 ms |
| `seam` | 경계에서 새 chunk가 이전 계획과 다른 정도. 계속 0.01 rad를 넘으면 `--obs-lag` 조정(로봇이 뒤로 끌리면 키우고, 앞으로 튀면 줄임) |
| `underruns` | 행동이 바닥나 멈춘 횟수. async에서 계속 늘면 `--replan-steps`를 줄임(예: 6). sync에서는 매 결정마다 1씩 느는 게 정상 |
| `overruns` | 120 Hz 전송이 밀린 횟수. 늘면 PC 부하 확인 |
| `latency` | 관측부터 chunk 반영까지 걸린 시간 |

이상하면 `--exec-mode sync`로 바로 이전 방식으로 돌아갈 수 있다.

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
