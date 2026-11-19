## 빠른 실행 (DWA / PID)

모든 명령은 **NUC 터미널**에서 실행합니다 (노트북에서 `ssh nuc`). 새 터미널을 열면 그 터미널에서도 먼저 NUC에 접속하세요.

> [!WARNING]
> - 주행 중에는 탑승자가 항상 조이스틱에 손을 올려 둡니다. 조이스틱을 움직이면 수동 모드로 바뀝니다.
> - 출발 전 E-STOP 확인: `rostopic echo /wheel_status`의 `data[1]`이 65(자동) ↔ 77(수동)로 바뀌는지 봅니다.
> - **DWA와 PID는 동시에 띄우지 않습니다.** 한쪽 스택을 완전히 내린 뒤 다른 쪽을 시작하세요.

경로 이름: `A_to_D4`, `A_to_D5`, `D4_to_A`, `D4_to_D5`, `D5_to_A`, `D5_to_D4`
(A = 애지문, D4 = 정몽구센터, D5 = 제2공학관)

### DWA로 주행

`~/wheelchair_deploys/current`에 선택된 배포본(이 저장소 `main`)으로 실행합니다. 배포본 바꾸기: `bash ~/use_deploy.sh <배포본 이름>`.

```bash
# 0) 점검 (읽기 전용): 배포본 검증, 지도 해시, ~/ 래퍼 일치
bash ~/wheelchair_deploys/current/source-linux/tools/route2_intern_check.sh pre

# 1) 스택 시작: 출발지/도착지 지정 (예: A → D5). 추종기는 PAUSED 상태로 올라옵니다
~/start_route2_intern.sh A D5

# 2) 점검 (읽기 전용): DWA 추종기, 경로, 위치추정 TRACKING, 경로 시작점 3 m 이내인지
bash ~/wheelchair_deploys/current/source-linux/tools/route2_intern_check.sh running A_to_D5

# 3) 주행 시작
~/go.sh

# 4) 주행 정지 (조이스틱을 움직여도 정지)
~/stop.sh

# 5) 스택 전체 종료 (그날 주행을 모두 마친 뒤)
~/stop_stack.sh
```

- 다음 구간은 1)부터 다시 (예: `~/start_route2_intern.sh D5 A`). 두 번째 구간부터는 위치추정을 유지한 채 경로만 바뀝니다. 거부되면 `KEEP_LOCALIZATION=0 ~/start_route2_intern.sh D5 A`.
- bag은 스택 시작과 함께 `~/localization_trials/`에 자동 기록됩니다: `blackbox_*.bag`, `hybrid_*.bag`, `person_bypass_*.bag`, 그리고 `45c6449` 이후 배포본부터 전체 토픽 `all_topics_*.bag` (끄기: `ALL_TOPICS_BAG=0`).
- 지연 보상 바꾸기: `LATENCY_MODE=lead ~/start_route2_intern.sh A D5` (기본값 `smith`).

### PID(FF-PID)로 주행

`~/wheelchair_deploys/ffpid`에 배포된 `feature/ffpid` 브랜치 코드로 실행합니다 (`main`에는 없습니다).

```bash
P="$HOME/wheelchair_deploys/ffpid/source-linux/src/pid/scripts"

# 1) READY 시작: 센서, 위치추정, 인지, 휠 베이스를 올립니다 (자동 주행은 아직 시작 안 함)
bash "$P/pid_ready.sh" start

# 2) READY 상태 확인
bash "$P/pid_ready.sh" status

# 3) 주행 시작: 경로 이름 지정. 이 터미널은 주행 중 계속 쓰이니 닫지 마세요
bash "$P/pid_drive.sh" A_to_D5

# 4) 주행 정지: 새 터미널에서 NUC에 접속한 뒤 실행 (조이스틱을 움직여도 정지)
bash "$HOME/wheelchair_deploys/ffpid/source-linux/src/pid/scripts/pid_stop.sh"

# 5) READY 종료: 모든 주행을 마친 뒤. bag도 이때 닫힙니다
bash "$P/pid_ready.sh" stop
```

모니터링 (각각 새 터미널):

```bash
bash "$HOME/wheelchair_deploys/ffpid/source-linux/src/pid/scripts/pid_ready.sh" watch
source /opt/ros/noetic/setup.bash && python3 -u "$HOME/wheelchair_deploys/ffpid/source-linux/src/pid/scripts/pid_safety_watch.py"
```

- 다음 구간은 READY를 내리지 않고 3)만 다시 실행합니다 (예: `bash "$P/pid_drive.sh" D5_to_A`).
- bag은 READY 시작과 함께 `~/pid_ready_logs/<세션>/`에 자동 기록됩니다.
  - `ready_topics_*.bag`: 주요 토픽만 담은 경량 bag. READY 종료 후 주행별로 잘려 `~/pid_drive_bags/`에 정리됩니다.
  - `all_topics_*.bag`: 전체 토픽 (`rosbag record -a --lz4`, 2 GB/15분 분할). 빈 공간이 10 GB 미만이면 건너뜁니다. 끄기: `PID_BAG_ALL=0 bash "$P/pid_ready.sh" start`
- 녹화 상태: `bash "$P/pid_ready_bag.sh" status`

---

<p align="center">
  <img src="docs/assets/hero-route.png" alt="Hanyang University Unicon Lab autonomous wheelchair" width="100%" />
</p>

<h1 align="center">Unicon Lab Autonomous Wheelchair</h1>

<p align="center">
  Hanyang University · Unicon Lab, Robotics<br />
  A fail-closed ROS 1 Noetic stack for map-based localization, corridor-bounded route
  following, and independent motion-safety supervision on a real powered wheelchair.
</p>

<p align="center">
  <a href="http://wiki.ros.org/noetic"><img src="https://img.shields.io/badge/ROS-Noetic-22314E?style=for-the-badge&logo=ros" alt="ROS Noetic" /></a>
  <a href="https://releases.ubuntu.com/20.04/"><img src="https://img.shields.io/badge/Ubuntu-20.04-E95420?style=for-the-badge&logo=ubuntu" alt="Ubuntu 20.04" /></a>
  <img src="https://img.shields.io/badge/LiDAR-Livox%20MID--360-00A8E1?style=for-the-badge" alt="Livox MID-360" />
  <img src="https://img.shields.io/badge/localization-field%20validated-16a34a?style=for-the-badge" alt="Localization field validated" />
  <img src="https://img.shields.io/badge/operation-supervised%20only-f59e0b?style=for-the-badge" alt="Supervised operation only" />
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache%202.0-f59e0b?style=for-the-badge" alt="Apache 2.0" /></a>
</p>

<p align="center">
  <a href="#two-stacks-in-one-repository">Two stacks</a> ·
  <a href="#what-has-been-driven">Field results</a> ·
  <a href="#the-vehicle">Vehicle</a> ·
  <a href="#routes-and-the-safety-band">Routes and band</a> ·
  <a href="#command-path">Command path</a> ·
  <a href="#running-it">Running it</a> ·
  <a href="#verification">Verification</a> ·
  <a href="#safety-boundary">Safety boundary</a>
</p>

> [!IMPORTANT]
> The chair drives itself, and only with an operator aboard holding the joystick. Moving the
> joystick drops the base out of auto mode and the follower holds within one control cycle.
> No unsupervised operation and no passenger transport is authorized; every gate in
> [`contracts/wp0/A16-release-authority.yaml`](contracts/wp0/A16-release-authority.yaml)
> remains unapproved and no field result below changes that.

## Two stacks in one repository

Not knowing this costs hours, so it comes first.

| | `src/static_livox_localization/` | `src/wheelchair_*/` |
| --- | --- | --- |
| Status | **Drives the chair** | Designed architecture, not deployed |
| Localization | FAST-LIO odometry + GICP against a prior map | Adapter, confidence guard, topology guard |
| Command owner | `waypoint_follower.py` | `move_base` |
| Deployed by `tools/push_to_nuc.sh` | Yes | No |
| Frozen WP0 contracts | Not bound to them | Bound and hash-checked |

The `wheelchair_*` packages hold the contract and guard design and most of the test suite. The
`static_livox_localization` package grew in the field against a real chair on a real campus
footpath, and it is the one on the vehicle. Both are maintained; only the second moves wheels.

## What has been driven

Two complete autonomous runs of the 0727 route on **2026-07-31**, three recorded sessions on
**2026-08-02**. Localization was the open question and is now closed.

| Measured while driving, both 07-31 runs pooled | Value | Gate |
| --- | --- | --- |
| Samples | 154 | — |
| Tracking state | `TRACKING` for every one | — |
| `inlier_ratio` | 0.9666 – 1.0000 (mean 0.9949) | > 0.20 |
| `fitness` | 0.0137 – 0.0329 (mean 0.0219) | < 0.28 |
| Localization holds, published or suppressed | 0 | — |

Both runs initialized with **no manual seed**, reaching `TRACKING` through
`WAITING_INITIALIZATION` → `VERIFYING`, with FAST-LIO stationary drift of 0.016 m and 0.014 m
against a 0.15 m health limit. The two runs agree to within 0.0006 on both means, which is what
makes this a measurement and not one lucky lap. The same gates were set from 07-29 figures where
the inlier ratio ran 0.44 – 0.79; the worst 07-31 sample beats that day's best on the same route
and map. The 08-02 sessions held `TRACKING` throughout as well.

One excursion is recorded rather than smoothed over. Parked at the goal twenty minutes after the
second run ended, the fix fell to inlier 0.124 – 0.262 and crossed the gate four times with
fitness still inside its ceiling. Nothing was driving. It is the only time all evening the fix
came near failing, so the end of that route is its weakest geometry.

Recordings live in [`blackbox/`](blackbox/) under Git LFS with per-session manifests, verified by
`tests/test_blackbox_archive.py`. A truncated bag opens fine and plays short, which is the failure
that check exists for.

## The vehicle

A powered wheelchair with a Livox MID-360 on the front of the left armrest.

| Measurement | Value | Source |
| --- | --- | --- |
| Sensor forward of the wheel axle | 0.500 m | Operator measurement, 2026-07-31 |
| Sensor left of the wheel axle | 0.200 m | Operator measurement, 2026-07-31 |
| Sensor height above ground | 0.725 m | Operator measurement; the 0727 map independently gives 0.775 m ± 0.018 |
| Ground-blind radius | ~5.9 m | `0.725 / tan(7°)`, from the MID-360's lower field of view |
| Chair half width | 0.35 m | Used to inset the drawn corridor |

That blind radius is the reason drop safety cannot come from the live scan: kerbs and drop-offs
sit inside it. The scan is used for what the sensor can see - obstacles and people - and the
ground is handled by the band described below.

Frames follow REP-103 in
[`wheelchair_hardware.urdf.xacro`](src/wheelchair_description/urdf/wheelchair_hardware.urdf.xacro):
`base_footprint` at the axle midpoint on the ground, `imu_link`/`body` at FAST-LIO's IMU origin,
and `lidar_link` derived from it through the built-in extrinsic. One physical mount, one number
each, pinned across every consumer by `tests/test_mount_geometry.py` - written after that height
lived in six files as an unmeasured 0.30 m placeholder, which silently hid everything shorter
than 57 cm from the obstacle guard.

## Routes and the safety band

A route is the centreline; the band is the lateral limit at each station. The follower is
constrained by the band, not merely advised by it.

| | `20260727_chair_centred_*` | `20260812_route_v6_v8_*` | `20260814_route_algorithm_*` |
| --- | --- | --- | --- |
| Waypoints | 1,446 (0.2 m) | 1,900 (0.2 m) | 1,897 (0.2 m) |
| Length | 383.4 m | 379.2 m | 376.2 m |
| Band stations | 381 at 1.0 m | 761 at 0.5 m | 1,897 at 0.2 m |
| Origin | Resampled from a recorded drive | Previous v6 preferred line inside the v8 drivable mask | Dense-map measured curb corridor and algorithmic route search |
| Shipped by the bringup | No | Previous default | **Yes (2026-08-14)** |
| Localization validated on it | **Yes** (07-31) | Previous field route | Offline audit passed; field validation pending |

The v6/v8 route preserves the preferred v6 line while treating the complete v8 map as a hard
drivable boundary. Route contents, band, mask image and mask geometry metadata are hash-bound,
and the route is smoothed before resampling so raster steps do not command alternating steering.

The `20260814_route_algorithm_*` route was promoted on 2026-08-14; the bringup default has
since moved to `20260816_route_v9_clearance_*`. The algorithm route is generated from the
dense-map curb corridor, uses a 0.2 m station spacing, and is hash-bound to its safety band and
drivable mask. The promotion keeps the existing localization and follower validation gates; it
does not claim field localization validation until a replay and a controlled outdoor run are
completed. The previous v6/v8 pair remains in `routes/` for comparison and rollback.

The route audit passed bilateral centre, wheel-line and rotated full-footprint checks against
dense-map measured curb boundaries. The two OMO implementation lines, commit provenance,
algorithms, evidence and promotion gates are recorded in
[`docs/route-implementation-status-ko.md`](docs/route-implementation-status-ko.md).

> [!WARNING]
> **Most shipped-band edges are drawn, not measured.** Every one of the 0727 band's 762 edges
> carried a measured verdict - `step_up` 266, `drop` 203, `lip` 104, `open` 177, `unscanned` 12.
> The v6/v8 rebuild retains measured v6 edge semantics at the nearest stations but otherwise
> relies on the operator-drawn v8 boundary. This remains a load-bearing assumption that requires
> re-measurement before passenger operation.

Where both exist, the drawing only ever narrows. `safety_band.corridor_limit` takes a `min()`
against the measured limit, insets the chair half width because the corridor was drawn for the
chair rather than for the point it turns about, and never lets the corridor term go negative on
the driven line - a drawing that excludes the path someone actually drove is a drawing error.
Written into `left_corridor_m` / `right_corridor_m` rather than over `left_m`, because telling
the speed policy that a 2.45 m kerb is 0.80 m away would pace the whole route for a fall that
was never near.

### route2_intern (2026-10-28) - the NUC's route since 2026-10-30

A second map, separate from the merged 0707/0725 one: the 2026-10-16 route2 mapping drive
(A 애지문 → D4 정몽구 → D5 제2공학관 → D4 → A) reprocessed with GLIM. Since 2026-10-30 it is what
the chair drives: `~/hybrid_current.sh` takes the map, route, band and mask from
[`tools/route2_intern_env.sh`](tools/route2_intern_env.sh) on any deploy that ships it, so the
operator app's [로컬 켜기] / [주행 시작], `~/start_wheelchair_localization.sh`,
`~/start_route2_intern.sh` and `~/go.sh` all use the same set.

| | `maps/route2_intern_20261026/` + `routes/20261028_route2_D5_to_A_*` |
| --- | --- |
| Map | `route2_intern.pcd`, 3.87 M points (Git LFS), origin at A |
| Route | D5 → D4 → A, 1,193 waypoints (0.2 m), 267.6 m, `chair_centre`, `builtin` |
| Band / mask | 536 stations at 0.5 m; mask ±1.5 m to D4, ±2.0 m to A |
| Driven autonomously | **No** - draft, first run aboard, joystick, low speed |
| Map identity | `MAP_ID=route2_intern_20261026`, PCD SHA-256 `e0809f29…f74432`, checked before start |
| Started by | `~/start_wheelchair_localization.sh` or `~/start_route2_intern.sh` on the NUC |
| Repo default | `tools/start_wheelchair_localization.sh` itself still defaults to the merged map and v9; the NUC wrappers override it |

Destination coordinates, how the scans were registered to the map, and what has not been
checked are in [`docs/2026-10-28-route2-intern-map-ko.md`](docs/2026-10-28-route2-intern-map-ko.md).

## Command path

Deliberately narrow, each stage able to stop the chair when the one above it misbehaves:

```text
waypoint_follower.py     route following, band containment, obstacle policy
  -> /cmd_vel_raw
safety_gate.py           independent stopping envelope and swept footprint
  -> /cmd_vel_gated
tip_guard.py             rate limiter and staleness fail-safe, last stage
  -> /cmd_vel
wheel_cmd / uart         UART-level watchdog
```

If the follower dies the gate stops the chair; if the gate dies, tip_guard's staleness check
does; if that dies, the UART watchdog does.

### Obstacle policy

`obstacle_clusters.py` clusters the accumulated scan, excludes the rider by an explicit box, and
tracks each object **in the odom frame** - whether something is moving is only a question in a
frame that does not move with the chair. The follower then answers two situations differently:

- **Watched standing still** → driven around, decided 5 m out so the chair drifts past instead of
  arriving and stopping. Every sidestep is vetted against the band.
- **Moving, or not yet watched long enough** → waited out where it stands, resuming on its own
  once the corridor clears, with no timer to reset. Stepping around a person is a manoeuvre into
  where they are about to be.

Distance to an object is measured from **its own returns**, sliced laterally, not from its
bounding box. On 2026-07-31 a wall crossing the scan diagonally was published as a box whose near
face read 0.69 m dead ahead while its nearest return inside the corridor was 2.13 m; 0.69 m sits
inside the 0.9 m floor on the stop radius, and a stopped chair's envelope stays at that floor, so
nothing could ever move a static object out of a distance it was never at. Sixteen minutes of
hold on evidence that did not exist. `cluster_guard.profile_reach` is the fix and
`test_object_profile.py` reconstructs that wall from the recorded box to pin both numbers.

### Diagnostic mode

`SAFETY_POLICIES=false` switches off everything that is a judgement about the world - band
containment, the raw corridor scan, the localization hold, the geofence, the slope limit - so one
run can measure one thing without another guard ending the measurement first and looking, from
outside a stationary chair, exactly like the thing it was meant to detect.

What never switches off is the joystick override and what it rests on: `MANUAL_MODE` is the
override and `BASE_STALE` is whether the channel reporting it is still alive. Suppressed policies
are still evaluated and published as `WOULD_HOLD:` on `/waypoint_follower/status`, because that
run is the only place their thresholds can be calibrated against the real thing.

## Running it

### NUC code (2026-10-30)

The NUC does not run a checkout of this repository in place. It runs one deploy directory,
`~/wheelchair_deploys/current` → `~/wheelchair_deploys/<name>`, switched with
`bash ~/use_deploy.sh <name>`; the home wrappers `~/start_wheelchair_localization.sh`, `~/go.sh`
and `~/stop.sh` all go through `~/hybrid_current.sh` into that deploy's `tools/hybrid.sh`.
Copies of those wrappers are kept in [`tools/nuc_home/`](tools/nuc_home/).

`main` now carries the code the NUC drove on until 2026-10-30: deploy
`person-bypass-steering-d88f367-20260830` (`d88f367`, static-person bypass steering) plus the
edits made on the NUC on 2026-08-31 and never committed until now (bicycle / person bypass
hotfix in `person_bypass_dwa_follower.py`, `cluster_guard.py`, `person_bypass_policy.py`,
`dwa_follower.py`, `activate_person_bypass.sh` and their tests).

To deploy a commit of `main`, on the NUC:

```bash
bash tools/make_nuc_deploy.sh <commit> [name]   # from any checkout; clones <commit> into the deploy
bash ~/use_deploy.sh <name>                      # then stop and start the stack from the app
```

`make_nuc_deploy.sh` clones the commit, pulls only the LFS maps, checks the route2_intern PCD
hash, builds `static_livox_localization` (Release, CUDA fast_gicp), runs the package tests, and
writes `OFFLINE_VERIFIED` only if they pass. It never switches `current` itself. Rolling back is
`bash ~/use_deploy.sh person-bypass-steering-d88f367-20260830`; a deploy without
`tools/route2_intern_env.sh` falls back to the merged map and the v9 route.

### 1. Deploy (laptop, older flow)

From a checkout of `main` with the merged-map volume attached:

```bash
./tools/push_to_nuc.sh /Volumes/<map-volume>/merged_0707_0725_v1
```

It verifies the map bundle byte for byte, syncs and builds the workspace with whichever catkin
tool owns the build space, and installs the field scripts into `$HOME` with checksum
verification. Those scripts live outside the checkout, which is how a stale copy once kept
launching the previous week's route.

The route2_intern map is in Git LFS, not on the volume. `make_nuc_deploy.sh` pulls it; for a
plain checkout run `git lfs pull` in it. `route2_intern_env.sh` checks the PCD's SHA-256 and
refuses an LFS pointer, so a missed pull stops the bring-up, not the stack after it is up.

### 2. At the chair (NUC)

Step by step for a first-time operator, in Korean, with read-only checks before and after
bring-up: [`docs/runbooks/route2-intern-dwa-ko.md`](docs/runbooks/route2-intern-dwa-ko.md).

Operator aboard with a hand on the joystick for every step that can move the chair. Check the
E-stop first: `/wheel_status` `data[1]` should toggle 65 ↔ 77.

| Step | Command | What it does |
| --- | --- | --- |
| Check | `bash ~/wheelchair_deploys/current/source-linux/tools/route2_intern_check.sh pre` | Read-only: current deploy is verified, ships route2_intern, PCD hash and `~/` wrappers match |
| Bring up | `~/start_route2_intern.sh` (asks from/to among A, D4, D5; or `~/start_route2_intern.sh D4 D5`; `~/start_wheelchair_localization.sh` = D5 → A) | route2_intern map + the 2026-10-31 manual route (`routes/route2_manual_20261031/`), guards on. Always DWA: the hybrid bring-up pins `PROFILE=dwa` and ends on `person_bypass_dwa_follower.py`. Leaves the follower **PAUSED** |
| Check | `bash .../tools/route2_intern_check.sh running <KEY>` | Read-only: follower is the DWA one, on route `<KEY>` (the one given to `start_route2_intern.sh`; omitted, it is read from the follower), localization `TRACKING`, chair within 3 m of the route origin |
| Drive | `~/go.sh` | Starts the drive |
| Mark | `~/mark.py <label>` | Optional: marks an arrival in the black box and `~/field_marks.log` |
| Stop | `~/stop.sh` | Stops it, as does moving the joystick |

The actuation lag is covered by the Smith predictor (`LATENCY_MODE=smith`, the default since
2026-11-04, `scripts/actuation_lag.py`). `LATENCY_MODE=lead ~/start_route2_intern.sh` brings back
the old 0.55 s `LATENCY_S` lead, and `LATENCY_MODE=off` turns compensation off.

The route starts at D5 (제2공학관), and auto-init searches the whole map (`AUTO_INIT_GLOBAL_ONLY`).
To run on the previous map (merged 0707/0725 with the v9 route) on the same deploy:

```bash
ROUTE_SET=v9 PROFILE=dwa ~/start_wheelchair_localization.sh
ROUTE_SET=v9 ~/go.sh
```

`go.sh` refuses rather than starts if the follower is absent, the object tracker is silent, or
localization is not `TRACKING`, and it checks all three before commanding anything - a check
placed after the first command runs while the chair is already moving. `stop.sh` checks nothing,
because a stop that refuses on a failed precondition is not a stop.

A black box records pose, diagnostics, commands, wheel status, follower state, the object
summary and `/field_mark` for the whole session, to `~/localization_trials/blackbox_*.bag`. See
[`docs/debug_bags.md`](docs/debug_bags.md). `mark.py` waits for the recorder to subscribe before
it publishes: on 10-16 a bare `rostopic pub -1 /field_mark` reached no bag.

## Verification

```bash
python3 -m pytest -q                               # host suite
python3 scripts/validate_wp0_contracts.py --root .  # frozen WP0 contracts
```

Pinned Ubuntu 20.04 / Noetic:

```bash
docker build -f tools/noetic/Dockerfile -t wheelchair-noetic-validation .
docker run --rm --network none wheelchair-noetic-validation bash -lc \
  'source /opt/ros/noetic/setup.bash && catkin_make && catkin_make run_tests && python3 -m pytest -q'
```

About 700 host tests pass. A handful of suites are evidence-bound and fail without the external
dataset - a missing GLIM artifact or full-bag record stays blocked rather than being replaced by
a simulated claim.

The tests worth reading are the ones written against something that actually went wrong:

| Test | Written because |
| --- | --- |
| `test_mount_geometry.py` | One sensor height lived in six files as a placeholder |
| `test_localization_field_envelope.py` | Gates must sit outside the measured envelope, both ways |
| `test_object_profile.py`, `test_cluster_guard.py` | A bounding-box corner is not an obstacle |
| `test_drive_policy.py` | The joystick override must survive every switched-off policy |
| `test_route_corridor.py`, `test_route_v4.py`, `test_route_v5.py` | A drawn corridor may narrow but never widen |
| `test_blackbox_archive.py` | A recording nobody can repeat, and truncation that reads as valid |

## Repository layout

```text
src/
├── static_livox_localization/   # THE DEPLOYED STACK: FAST-LIO + GICP localization, follower,
│                                #   safety gate, tip guard, clustering, field scripts
├── wheelchair_interfaces/       # frozen ROS messages, actions, and ABI contracts
├── wheelchair_perception/       # canonical sensor products; no motion permission
├── wheelchair_navigation/       # localizer adapter, route manager, move_base integration
├── wheelchair_route_safety/     # independent route and geofence authority
├── wheelchair_decision/         # deterministic mission FSM and ExecuteRoute action
├── wheelchair_safety/           # gate plus collision/slope/localization/topology authorities
├── wheelchair_hardware/         # exact driver contract; disabled by default
├── wheelchair_bringup/          # explicit sim/replay/shadow/hardware profiles
├── wheelchair_description/      # Gazebo and NUC-derived URDFs, not interchangeable
└── wheelchair_gazebo/           # simulation-only scenarios and evidence collectors
```

| Path | Purpose |
| --- | --- |
| `routes/` | Route waypoints, safety bands, corridor masks and previews |
| `blackbox/` | Field recordings (Git LFS) with per-session manifests |
| `contracts/wp0/` | Frozen ownership, schema, evidence, hazard, and authority contracts |
| `data/` | Committed candidate map, metadata, and directional routes |
| `tools/` | Deployment, field startup, route and band generation, offline analysis |
| `docs/` | Operator, mapping, interfaces, simulator, safety, and field reports |

`wheelchair_description` keeps two non-interchangeable entry points: the Gazebo-only
`wheelchair.urdf.xacro` and the NUC-derived `wheelchair_hardware.urdf.xacro`. Hardware launch and
calibration must never fall back to the simulation description.

## Safety boundary

Field operation to date is **supervised**: an operator rides the chair with a hand on the
joystick for the whole run, on a known route, on a campus footpath, with no passenger other than
the operator. That is the only mode any result here supports.

The formal release gates are unchanged and remain unapproved:

```yaml
software_release_candidate_authorized: false
hardware_motion_authorized: false
passenger_operation_authorized: false
campus_operation_authorized: false
physical_authority: false
```

They stay false while any of these is unknown or unmeasured: the exact real-driver topic, type,
MD5, sign, units, rate, timeout, mode and watchdog behaviour; physical e-stop and joystick
priority and latency; footprint, payload, battery, surface, braking and stopping envelope;
fingerprinted target-NUC resource, thermal and endurance qualification; inert HIL repetitions and
segregated closed-course no-passenger evidence; surveyed route, corridor and exclusions with
written campus approval; and a separately reviewed passenger-operation protocol.

Known open items, kept here rather than in a tracker nobody reads:

- Most shipped v6/v8 band edges still lack measured drop semantics (see the warning above).
- The localization envelope was measured on the 0727 route; v6/v8 has not been re-measured.
- Live target-NUC shadow QA is required before this route is eligible for a field start.

See [`contracts/wp0/A16-release-authority.yaml`](contracts/wp0/A16-release-authority.yaml),
[`contracts/wp0/A14-hazard-log.yaml`](contracts/wp0/A14-hazard-log.yaml), and
[`docs/safety_case.md`](docs/safety_case.md) for the complete claim boundary.

## Documentation

- [Operator runbook](docs/operator_runbook.md)
- [2026-08-02 field report](docs/2026-08-02-field-report.md)
- [route2_intern map and D5 → A route (2026-10-28)](docs/2026-10-28-route2-intern-map-ko.md)
- [route2_intern DWA drive, step by step (Korean)](docs/runbooks/route2-intern-dwa-ko.md)
- [Debug and black-box bags](docs/debug_bags.md)
- [Replay and offline mapping](docs/replay_and_mapping.md)
- [Interfaces and ownership](docs/interfaces.md)
- [Simulator fidelity](docs/simulator_fidelity.md)
- [Safety case](docs/safety_case.md)

## License

Apache License 2.0. See [LICENSE](LICENSE).
