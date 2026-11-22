#!/usr/bin/env bash
# CaCoFlow client 패키지를 zip으로 만든다 -> ~/policy_bridge_cacoflow_client_<날짜>.zip
#   ./make_client_zip.sh [--email] [출력_경로.zip]   (기본: 홈 디렉터리)
#   --email: Gmail은 zip 안의 .js/.sh 를 "보안상의 이유로 차단"한다. 이 옵션은 그 파일들에 .txt 를 붙이고
#            restore_names.py 를 넣는다 (받는 쪽이 `python3 restore_names.py` 로 되돌림).
# 넣는 것: CaCoFlow client/bridge/GUI 코드, 실행 스크립트, 의존성 목록, 문서.
# 안 넣는 것: Robot Server(robot_server.py, run_server.sh, setup_rt.sh), 로그, 캐시, 토큰/비밀번호/설정 파일.
cd "$(dirname "$(readlink -f "$0")")" || exit 1
NAME="policy_bridge_cacoflow_client"
EMAIL=0; [ "$1" = "--email" ] && { EMAIL=1; shift; }
SUFFIX=""; [ "$EMAIL" = 1 ] && SUFFIX="_email"
OUT="${1:-$HOME/${NAME}${SUFFIX}_$(date +%Y%m%d).zip}"
FILES=(cacoflow_client.py cacoflow_bridge.py cacoflow_gui.py robot_client2.py
       run_cacoflow.sh run_cacoflow_gui.sh requirements.txt requirements-gui.txt CACOFLOW.md)
for f in "${FILES[@]}" gui_static; do [ -e "$f" ] || { echo "missing: $f"; exit 1; }; done
STAGE=$(mktemp -d) || exit 1
trap 'rm -rf "$STAGE"' EXIT
mkdir -p "$STAGE/$NAME"
cp -r "${FILES[@]}" gui_static "$STAGE/$NAME/"
cat > "$STAGE/$NAME/README_CLIENT.md" <<'EOT'
# CaCoFlow client (policy_bridge)

로봇 PC의 Robot Server(WebSocket, 기본 ws://127.0.0.1:8000)와 협업자의 CaCoFlow 정책 서버(HTTP, SSH 터널)를 잇는 client.
이 패키지에는 Robot Server가 들어 있지 않다. 자세한 내용은 CACOFLOW.md.

## 설치 (Python 3.10+)
    python3 -m venv ~/venvs/policy_bridge
    ~/venvs/policy_bridge/bin/pip install -r requirements.txt -r requirements-gui.txt

## 실행
    ./run_cacoflow_gui.sh                       # 웹 GUI: http://127.0.0.1:8080/
    ./run_cacoflow.sh --info-only               # CLI: 서버 확인만 (로봇 안 움직임)
    ./run_cacoflow.sh --instruction "put the blue mug in the bowl" --gripper-one-means closed --trial-id t01

## 필요한 것 (협업자에게 받기)
- SSH 계정/포트/키, API 토큰 (`CACOFLOW_API_TOKEN`; 파일에 저장하지 말 것)
- 그리퍼 값 1의 의미(`closed`/`open`) -- 핸드오프 문서에 없으니 확인 필요

venv 경로가 다르면 `CLIENT_VENV=<경로>`로 지정한다. Robot Server는 로봇 PC에서 따로 실행한다(여기엔 없음).
EOT
chmod +x "$STAGE/$NAME"/*.sh
if [ "$EMAIL" = 1 ]; then
  (cd "$STAGE/$NAME" && for f in $(find . -name '*.js' -o -name '*.sh'); do mv "$f" "$f.txt"; done)
  cat > "$STAGE/$NAME/restore_names.py" <<'EOT'
"""이메일용으로 붙인 .txt 를 떼어 원래 파일명으로 되돌린다: python3 restore_names.py"""
import pathlib
import stat

root = pathlib.Path(__file__).resolve().parent
for p in list(root.rglob("*.js.txt")) + list(root.rglob("*.sh.txt")):
    target = p.with_suffix("")
    p.rename(target)
    if target.suffix == ".sh":
        target.chmod(target.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP)
    print("restored", target.relative_to(root))
EOT
  sed -i 's|^## 설치 (Python 3.10+)|## 먼저 (이메일용 zip): `python3 restore_names.py` (.js.txt/.sh.txt 이름을 되돌림)\n\n## 설치 (Python 3.10+)|' "$STAGE/$NAME/README_CLIENT.md"
fi
find "$STAGE" -name __pycache__ -prune -exec rm -rf {} +
mkdir -p "$(dirname "$OUT")"
rm -f "$OUT"
OUT_ABS="$(readlink -f "$(dirname "$OUT")")/$(basename "$OUT")"; (cd "$STAGE" && zip -qr -X "$OUT_ABS" "$NAME")
echo "created: $(readlink -f "$OUT") ($(du -h "$OUT" | cut -f1))"
unzip -l "$OUT" | tail -n +4 | awk '{print $4}' | sed '/^$/d'
