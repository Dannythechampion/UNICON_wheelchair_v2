# route2_intern 경로 DWA 자율주행 절차

처음 휠체어를 돌리는 사람이 이 문서 하나만 보고 route2_intern 지도 위의 D5 → D4 → A 경로를
DWA로 주행할 수 있게 정리했다. 지도·경로를 만든 방법과 좌표는
[`../2026-10-28-route2-intern-map-ko.md`](../2026-10-28-route2-intern-map-ko.md)에 있다.

> [!IMPORTANT]
> 이 경로는 **아직 한 번도 자율주행하지 않은 초안**이다. 모든 주행은 사람이 탑승해서
> 조이스틱에 손을 올린 상태로, 저속으로 한다. 조이스틱을 움직이면 수동 모드로 넘어가고
> 팔로워는 한 제어 주기 안에 멈춘다. `/cmd_vel*`이나 `/wheel_cmd`에 직접 publish하지 않는다.

## 한눈에 보기

| 항목 | 값 |
| --- | --- |
| 지도 | `maps/route2_intern_20261026/route2_intern.pcd` (GLIM, Git LFS), 원점 = A 애지문 |
| 경로 | `routes/route2_manual_20261031/<키>_*` (2026-10-31 수동 경로), A 애지문 · D4 정몽구 · D5 제2공학관 사이 6개 구간 (아래 표). 기본 키 `D5_to_A`, 260.4 m |
| 출발 자세 | `D5_to_A`: 지도 좌표 (189.2, 49.6), 진행 방향 약 174° (서쪽, 제2공학관 앞에서 D4 쪽을 본다). 다른 키는 아래 표 |
| 제어기 | DWA - `person_bypass_dwa_follower.py` (RTX/CuPy 가속, 정지한 사람 우회 포함) |
| 기동 | NUC에서 `~/start_route2_intern.sh` |
| 주행 / 정지 | `~/go.sh` / `~/stop.sh` (또는 조이스틱) |
| 점검 | `tools/route2_intern_check.sh pre` / `running <키>` (읽기만 함) |

## 왜 DWA인가, 어디서 정해지나

`tools/start_wheelchair_localization.sh`만 단독으로 실행하면 기본 제어기는 pure pursuit
(`PROFILE=pursuit`)다. NUC에서는 그 스크립트를 직접 부르지 않는다. 홈 디렉터리 래퍼가
하이브리드 경로를 거치고, 거기서 DWA로 고정된다.

```text
~/start_route2_intern.sh
  -> <deploy>/tools/start_route2_intern.sh        route2_intern_env.sh로 지도·경로·밴드·마스크 선택
  -> ~/start_wheelchair_localization.sh            (앱 [로컬 켜기]도 여기로 온다)
  -> ~/hybrid_current.sh start                     current 배포본, OFFLINE_VERIFIED 확인
  -> <deploy>/tools/hybrid.sh start
       start_hybrid_avoidance.sh                   PROFILE=dwa 고정, gpu_dwa_follower.py
       activate_person_bypass.sh                   person_bypass_dwa_follower.py로 교체
```

그래서 NUC 래퍼로 띄우면 `PROFILE`을 따로 주지 않아도 항상 DWA다. 기동 로그에 나오는
`PROFILE=dwa` 줄은 그 결과다. 띄운 뒤 실제로 DWA인지는 아래 4단계의 `running` 점검이
`person_bypass_dwa_follower.py` 프로세스와 `/waypoint_follower/route` 파라미터로 확인한다.

## 구간 고르기

NUC에서 키 없이 띄우면 출발지와 목적지를 번호로 묻는다.

```bash
~/start_route2_intern.sh
```

```
(지난번에 띄운 경로의 도착지: D4 정몽구)
출발지 - 휠체어가 지금 있는 곳
  1) A  애지문
  2) D4 정몽구   [Enter]
  3) D5 제2공학관
> 
목적지
  1) A  애지문
  2) D5 제2공학관
> 2
D4 정몽구 -> D5 제2공학관 (D4_to_D5) 로 띄웁니다. [Enter=예, n=취소]
```

- 출발지의 [Enter] 기본값은 **지난번에 띄운 경로의 도착지**(`~/.route2_last_destination`)다. 그 주행을
  끝까지 마쳤다면 휠체어가 있을 곳이라는 추측일 뿐 측정이 아니다. 실제 위치는 4단계 점검의
  `chair is ... m from the route origin`이 잰다.
- 물어보지 않고 바로 띄우려면 `~/start_route2_intern.sh D4 D5` 또는 `~/start_route2_intern.sh D4_to_D5`.
- 터미널이 아닌 곳(앱의 [로컬 켜기] 등)에서 띄우면 묻지 않고 `D5_to_A`로 뜬다.

6개 구간은 iamshy95가 2026-10-31에 지도 위에서 직접 찍은 수동 경로(`routes/route2_manual_20261031/`)이고,
마스크는 6개가 함께 쓰는 `common_drivable_mask` 하나다.

| 키 | 출발 → 도착 | 길이 | 출발할 때 세우는 방향 |
| --- | --- | --- | --- |
| `D5_to_A` | D5 → A (D4 통과) | 260.4 m | 서쪽 (D4 쪽) |
| `A_to_D5` | A → D5 (D4 통과) | 260.4 m | 동쪽 (J 쪽) |
| `D5_to_D4` | D5 → D4 | 88.4 m | 서쪽 |
| `D4_to_D5` | D4 → D5 | 88.4 m | 북동쪽 |
| `D4_to_A` | D4 → A | 175.2 m | 남남서쪽 |
| `A_to_D4` | A → D4 | 175.2 m | 동쪽 |

- 세 지점의 좌표: A (1.25, -0.11), D4 (102.84, 57.23), D5 (189.21, 49.56). 같은 지점에서 출발하거나
  그 지점에 도착하는 구간은 모두 같은 점을 쓴다.
- 수동 경로의 safety band는 사람이 찍은 기하 마스크에서 나왔다(`UNVERIFIED: geometric mask, NOT physical
  curb clearance`). 연석까지의 실제 거리는 재지 않았으므로 처음 달리는 구간은 조이스틱에 손을 올리고 달린다.
- `<키>_smooth`(예: `D4_to_D5_smooth`)는 10-28 선을 스무딩해 D4에서 자른 이전 6개 구간이다
  (`routes/20261031_route2_*`, `tools/build_route2_intern_segments.py`). 이 구간의 D4 정지점은 (105.86, 59.75)로,
  수동 경로의 D4와 3.8 m 떨어져 있다.
- D4에서 **반대 방향으로 다시 출발할 때**(예: `A_to_D4` 다음 `D4_to_A`) 휠체어는 제자리 회전을 하지 않는다(바퀴 데드밴드).
  0.35 m/s, 반경 약 0.7 m 원호로 돌아서 옆으로 약 1.4 m, 앞으로 약 0.7 m를 쓴다. 앞이 막혀 있으면
  safety gate가 출발을 막으므로, 조이스틱으로 다음 구간 방향으로 돌려 세운 뒤 출발하는 것이 확실하다.
- `D5_to_A_raw`, `A_to_D5_raw`는 스무딩 전 10-28 선과 10-28 마스크다. 10-30 주행과 비교할 때만 쓴다.

## 0. 준비물

- 노트북에서 NUC SSH 접속 (사용자 `mprp3`, NUC IP는 연결한 핫스팟/공유기에서 확인).
- 휠체어 배터리 커넥터 연결, Livox MID-360 전원.
- 탑승자 1명(조이스틱), 옆에서 따라가는 사람 1명 권장.
- 주행 시작 지점: D5 제2공학관 앞. 위 표의 출발 자세에 맞춰 세운다.

## 1. 배포본 확인 (NUC, 한 번)

NUC는 저장소 체크아웃을 직접 돌리지 않는다. `~/wheelchair_deploys/current`가 가리키는
배포본 하나를 돌린다.

```bash
readlink -f ~/wheelchair_deploys/current
bash ~/wheelchair_deploys/current/source-linux/tools/route2_intern_check.sh pre
```

`ALL CHECKS PASSED (pre)`가 나오면 2단계로 간다. 점검 스크립트 자체가 없다는 오류가 나면
current 배포본이 이 문서보다 오래된 것이므로 아래처럼 main에서 새로 만든다.

```bash
rm -rf /tmp/v2 && GIT_LFS_SKIP_SMUDGE=1 git clone -q --depth 1 \
  https://github.com/Dannythechampion/UNICON_wheelchair_v2 /tmp/v2
bash /tmp/v2/tools/make_nuc_deploy.sh origin/main   # 마지막에 DEPLOY_READY <name> 이 나와야 함
bash ~/use_deploy.sh <name>
```

`/tmp/v2`는 스크립트를 꺼내 쓰는 용도일 뿐이다. 배포본은 `make_nuc_deploy.sh`가
`~/wheelchair_deploys/<name>`에 따로 클론한다.

`make_nuc_deploy.sh`는 커밋을 클론하고 LFS 지도만 받고, PCD 해시를 확인하고, 빌드와
패키지 테스트를 통과해야 `OFFLINE_VERIFIED`를 쓴다. `current`는 바꾸지 않으므로
`use_deploy.sh`는 직접 실행한다. 되돌리려면 이전 이름으로 `use_deploy.sh`를 다시 실행한다.

`pre` 점검이 `~/<파일> missing` 또는 `differs`로 실패하면 홈 래퍼가 저장소와 다르다.
`diff`로 내용을 먼저 본 뒤, 의도한 차이가 아니면 저장소 사본으로 바꾼다.

```bash
R=~/wheelchair_deploys/current/source-linux
diff ~/go.sh $R/tools/nuc_home/go.sh
install -m 755 $R/tools/nuc_home/*.sh ~/
```

| `pre` 실패 메시지 | 뜻 / 조치 |
| --- | --- |
| `no OFFLINE_VERIFIED` | 빌드나 테스트를 통과하지 않은 배포본. start/go가 거부한다. 다른 배포본 선택 |
| `no tools/route2_intern_env.sh` | 오래된 배포본. **이대로 띄우면 v9 경로(다른 길)로 달린다.** 새 배포본 생성 |
| `Git LFS pointer` | PCD가 실제 파일이 아님. 배포본의 `source-linux`에서 `git lfs pull --include="maps/**"` |
| `SHA-256 mismatch` | 다른 PCD. 새 배포본 생성 |
| `ROUTE_SET=... is exported` | 셸에 `ROUTE_SET`이 남아 있음. `unset ROUTE_SET` |

## 2. 휠체어 앞에서 안전 확인

1. 탑승, 조이스틱에 손.
2. D5에서 출발 방향(서쪽, D4 쪽)을 보도록 세운다.
3. 비상정지 확인은 ROS가 떠야 볼 수 있으므로 3단계 기동 직후에 한다(4단계 첫 항목).

## 3. 기동 (아직 움직이지 않음)

```bash
~/start_route2_intern.sh
```

출발지·목적지를 고르면(위 "구간 고르기") 배너에 `route2_intern <키>`, `map route2_intern_20261026`,
`profile dwa`가 나온다.
팔로워는 **PAUSED** 상태로 뜬다. 앱의 [로컬 켜기]를 눌러도 같은 것이 뜬다.

자동 초기화는 지도 전체를 탐색한다(`AUTO_INIT_GLOBAL_ONLY=true`). `TRACKING`이 될 때까지
움직이지 말고 기다린다. 앞의 지연 보상은 기본 0.55 s다. 10-16 주행과 비교하려면
`LATENCY_S=0 ~/start_route2_intern.sh`로 띄우고, 어느 값으로 달렸는지 기록한다.

## 4. 주행 전 점검

먼저 비상정지: 다른 터미널에서 `rostopic echo /wheel_status`를 띄우고 E-stop을 눌렀다
놓으며 `data[1]`이 65 ↔ 77로 바뀌는지 본다. 바뀌지 않으면 주행하지 않는다.

그다음 스택 점검:

```bash
bash ~/wheelchair_deploys/current/source-linux/tools/route2_intern_check.sh running <키>
```

`<키>`는 `~/start_route2_intern.sh`에 붙인 것과 같은 키다(예: `running D4_to_D5`). 기동 스크립트가
export한 키는 그 프로세스 안에만 있어서, 새 SSH 창에서는 다시 알려 줘야 한다. 키를 빼면 지금
돌고 있는 follower의 경로에서 키를 읽어 `[route key]` 줄에 `(from the running follower)`로 보여 준다.
이것은 지금 무엇이 떠 있는지 보여 줄 뿐, 의도한 구간인지는 확인하지 못한다.

아래가 모두 `OK`여야 한다.

- `follower is person_bypass_dwa_follower.py (DWA)` - pure pursuit가 아니라 DWA
- `follower route is <키>_waypoints.json`
- `localization TRACKING`
- `chair is ... m from the route origin (<출발지>)` - 휠체어가 고른 출발지에서 3 m 안에 있다.
  FAIL이면 출발지를 잘못 고른 것이다. 예를 들어 D4에 있는데 `D5_to_A`로 띄우면 follower는 D4 근처
  지점에 붙어서 아무도 고르지 않은 D4→A를 달린다. `~/stop.sh` 후 지금 있는 곳에서 출발하는 구간으로 다시 띄운다.
- `object tracking publishing`

`follower status`는 `PAUSED`류가 정상이다. 하나라도 FAIL이면 주행하지 않는다.

## 5. 주행

```bash
~/go.sh
```

`go.sh`는 팔로워, 객체 추적, 위치 추정 `TRACKING`, GPU DWA, 사람 우회 프리플라이트를 모두
확인한 다음에만 자동 모드로 바꾸고 출발한다. 하나라도 빠지면 `REFUSING TO START`로 끝나고
아무것도 움직이지 않는다. 앱의 [주행 시작]도 같은 스크립트다.

도착 지점을 기록하려면 (선택):

```bash
~/mark.py D4
~/mark.py A
```

## 6. 정지

```bash
~/stop.sh
```

또는 조이스틱을 움직인다. `stop.sh`는 아무것도 확인하지 않고 바로 멈춘다.
주행 중 상태는 `rostopic echo /waypoint_follower/status`로 본다. `HOLD:...`는 가드가
멈춰 세운 것이다. 움직이는 사람처럼 원인이 사라지면 스스로 다시 출발하는 것도 있지만,
10-16에는 `HOLD:DWA_GATE_TRAJECTORY`에서 계속 멈춰 있었다. 오래 멈춰 있으면 `~/stop.sh`
후 조이스틱으로 경로 위로 되돌리고, 상태 문자열을 기록한다.

## 7. 기록

블랙박스가 세션 전체를 `~/localization_trials/blackbox_*.bag`에 남긴다. 주행 후
파일 이름, 사용한 배포본(`readlink -f ~/wheelchair_deploys/current`), `LATENCY_S` 값,
멈춘 위치와 `HOLD` 사유를 함께 적어 둔다. bag 다루는 법은 [`../debug_bags.md`](../debug_bags.md).

## 첫 주행에서 볼 곳

- J 모서리 바깥쪽 (89, -21): 마스크가 연석 선에 붙어 있다.
- A → D4 오르막(waypoint #566–776): 동쪽 연석에서 0.3 m 띄워 놓은 구간.
- D4 굽은 곳 (108.6, 59.8): 모서리를 최대 0.43 m 둥글게 깎았다.
- 0.25 m 미만의 낮은 턱·낙차는 지도로 확인되지 않았다. 사람이 본다.

## 이전 지도로 돌아가기

같은 배포본에서 merged 0707/0725 지도 + v9 경로로 달리려면 두 명령 모두에 `ROUTE_SET=v9`를 준다.

```bash
ROUTE_SET=v9 ~/start_wheelchair_localization.sh
ROUTE_SET=v9 ~/go.sh
```
