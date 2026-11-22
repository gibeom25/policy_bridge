# CaCoFlow 정책 서버 연결 (`cacoflow_bridge.py`)

협업자(tanvir)의 CaCoFlow GPU 서버를 우리 Robot Server(`robot_server.py`)에 연결한다.
협업자가 준 `cacoflow_client.py`는 **수정하지 않고** 그대로 쓴다. 프로토콜, 오래된 관측 거부,
행동 해석, 30 Hz 루프는 모두 그 파일의 `run_trial()`이 처리한다. `cacoflow_bridge.py`는 그 파일의
`RobotInterface`를 Robot Server WebSocket 위에 구현한 것뿐이다.

## 기존 정책 서버(`robot_client2.py`)와의 차이

| | 기존 (`robot_client2.py`) | CaCoFlow (`cacoflow_bridge.py`) |
|---|---|---|
| 통신 | WebSocket + msgpack | HTTP + JSON, `Authorization: Bearer <token>` |
| 입력 | 이미지 224, 관절, 그리퍼, prompt | 전체 해상도 RGB PNG 2장, 관절 7, `gripper_opening`, 관측 시각 |
| 출력 | `(16, 8)` **절대** 관절 목표, 15 Hz | `(10, 8)` **delta**, 30 Hz: `q_target = 지금 측정한 q + actions[k, :7]` |
| 추론 | async (타임라인) / sync | **동기식만** (추론 중에는 마지막 목표 유지) |
| 시행 기록 | 없음 | `/reset`(trial_id) → `/end_trial`(성공 여부) |

delta가 매 step마다 **그 순간 측정한 관절값** 기준이라서 `robot_client2.py`의 async 타임라인 방식은 적용하지 않는다.
협업자도 동기식 실행을 전제로 했다.

## 준비

```bash
~/venvs/policy_bridge/bin/pip install -r requirements.txt     # numpy, websockets, paramiko, pillow
```

협업자에게 받아야 할 것:
- SSH 계정, 포트, 키 등록 (`tanvir@155.230.12.123`, 포트는 확인 필요)
- `CACOFLOW_API_TOKEN` (파일이나 명령 기록에 남기지 말 것. `run_cacoflow.sh`가 입력을 받는다)
- **그리퍼 1의 의미**(닫힘/열림): 핸드오프 문서에 적혀 있지 않다 → `--gripper-one-means`로 반드시 지정
- 카메라 crop/zoom: 핸드오프에 "agent zoom 1.2, wrist x-offset 31"이 있으나 확인 필요. 현재는 `--agent-zoom`만 지원

## 실행

```bash
./run_server.sh                                   # 터미널 1: Robot Server (기존과 동일)
./run_cacoflow.sh --info-only                     # 터미널 2: 서버 확인만 (로봇 안 움직임)
./run_cacoflow.sh --instruction "put the blue mug in the bowl" \
    --gripper-one-means closed --trial-id blue_bowl_p01 --placement p01
```

- `--info-only` 출력에서 `mock: false`, `action_shape: [10, 8]`, `sampling_steps: 10`, 작업 목록을 확인한다.
  mock 서버면 시행을 거부한다.
- SSH 터널은 내장 터널이 **빈 로컬 포트**를 자동으로 쓴다. Robot Server가 이미 `127.0.0.1:8000`을 쓰기 때문에
  핸드오프 문서의 `ssh -L 127.0.0.1:8000:...` 명령은 그대로 쓰면 **포트가 충돌한다**. 직접 터널을 열 때는
  `ssh -N -L 127.0.0.1:8100:127.0.0.1:8000 tanvir@155.230.12.123` 후 `--policy-url http://127.0.0.1:8100`.
- 시행이 끝나면(최대 `--max-steps`, 기본 600 step = 20 s, 또는 Ctrl+C) 성공 여부, 다른 물체 잡음 여부, 메모를 물어보고
  `/end_trial`로 보낸다. Enter는 "미결정"(성공으로 처리되지 않음).
- 서버/SSH 설정은 환경 변수로 바꿀 수 있다: `CACOFLOW_SSH_HOST`, `CACOFLOW_SSH_PORT`, `CACOFLOW_SSH_USER`. 로그는 `cacoflow.log`.

작업 문장(서버 `/info` 기준, 핸드오프 시점):
`put the blue mug in the bowl`, `put the blue mug on the plate`, `put the bowl on the plate`,
`put the red mug in the bowl`, `put the red mug on the plate`, `stack the blue mug on the red mug`,
`stack the red mug on the blue mug`

## 안전 / 주의

- 클라이언트 쪽 보호는 delta ±0.30 rad 클리핑, Panda∩FR3 관절 한계, 오래된 관측(0.5 s) 거부뿐이다.
  비상 정지와 Robot Server의 반사(reflex) 보호가 실제 안전장치다.
- **컨트롤러 일치 여부를 반드시 확인할 것.** 행동은 `목표 - 측정값`으로 학습됐다. 그래서 데이터 수집 때의 컨트롤러와
  우리 Robot Server의 reference filter(wn=8, 측정값이 목표보다 약 0.25 s 늦음)가 다르면 로봇이 학습 때와 다른
  속도로 움직인다. 시뮬레이션에서 매 step +0.01 rad delta를 보냈을 때 로봇은 의도한 0.9 rad 중 0.105 rad만
  움직였다(추론 대기 포함). 첫 실제 시행 전에 협업자에게 수집 때 쓴 컨트롤러와 필터 설정을 물어보고, 낮은 위치에서 짧게
  (`--max-steps 60`) 시험한다.
- Robot Server는 이미지 촬영 시각을 보내지 않는다. 그래서 수신 시각을 촬영 시각으로 쓰고, 실제 관측은 그보다 조금 오래됐다.
- 실패(연결 끊김, 오래된 관측 등) 후에는 세션이 무효가 된다. 새 시행으로 다시 시작하고, 서버 쪽 시행 기록 정리는 협업자와 맞춘다.
