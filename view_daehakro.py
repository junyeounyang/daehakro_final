"""대학로(이화사거리 ~ 혜화동로터리) 도로 확인 스크립트.

env/daehakro_config.py 를 고친 뒤 학습 전에 도로와 교통 흐름을 확인하는 용도.

사용법:
    python view_daehakro.py              # sumo-gui로 열기 (▶ 버튼으로 재생, 긴급차 보조 포함)
    python view_daehakro.py --plain      # sumo-gui만 열기 (긴급차 보조 없이 SUMO 기본 동작만)
    python view_daehakro.py --netedit    # netedit로 도로망 열기
    python view_daehakro.py --nogui      # 화면 없이 1시간 시뮬레이션 후 점검표 출력
    python view_daehakro.py --plot       # 도로망 그림(PNG) 저장 (env/sumo/daehakro/daehakro_*.png)

--nogui 점검표 항목
    [차량]   투입/도착, 텔레포트, 충돌(차-차 / 차-보행자), 평균 속도, 오래 멈춘 차, 투입 대기
    [버스전용차로] 실선 구간에 들어간 일반 차(0이어야 함), 점선 구간을 잠깐 쓴 일반 차와 그 이유
    [유턴]   이화사거리·D·A 등 유턴한 차량 수
    [부모 차량] 어린이 보호구역 구역1(이화장길 입구 ~ T자 서쪽 횡단보도) 도로 위에 멈춰 아이를 태운 차량 수
    [긴급차량] 1500초 출동한 소방차·구급차(사이렌)의 걸린 시간·멈춘 시간·버스차로 주행 거리,
             길을 비켜 준 차(버스차로에 잠깐 들어간 차 포함), 우선신호 횟수, 긴급차 충돌(0이어야 함)
    [보행자] 횡단보도별로 건넌 사람 수 (어린이 보호구역 포함)

긴급차(사이렌) 보조: env/daehakro_emergency.py 를 TraCI 로 매 스텝 실행한다 (GUI·--nogui 모두).
  SUMO 기본 차로변경 모델은 차가 차로 안에서 옆으로 비킬 수 없어서, 길 비켜 주기·우선신호·
  횡단보도 서행을 이 보조가 맡는다. --plain 으로 열면 이 보조 없이 SUMO 기본 동작만 보인다.
"""
import argparse
import collections
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

BASE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE)

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
else:  # 수업 env와 같은 방식으로 SUMO_HOME 자동 감지
    try:
        from env.sumo_env import SumoHighwayEnv  # noqa: F401
    except Exception:
        pass
    if "SUMO_HOME" in os.environ:
        sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import sumolib  # noqa: E402
from sumolib import checkBinary  # noqa: E402
import traci  # noqa: E402
import traci.constants as tc  # noqa: E402

from env import daehakro_builder  # noqa: E402
from env import daehakro_config as cfg  # noqa: E402
from env.daehakro_emergency import EmergencyAssist  # noqa: E402

# 횡단보도 위치 이름 (노드 → 설명)
PLACE = {
    "IH": "이화사거리", "IWJ": "이화장길 입구(어린이 보호구역)", "SZ_X": "이화장길 T자 서쪽(어린이 보호구역)",
    "M": "M", "O": "O·K", "I": "옆길 I", "H": "H", "G": "옆길 G", "F": "F(마로니에공원)",
    "E": "E(혜화역)", "D": "옆길 D", "C": "C", "C_s": "옆길 C", "B": "옆길 B", "A": "옆길 A", "RS": "로터리 남쪽",
    "J_NE": "로터리 북동", "J_SW": "로터리 남서", "J_NW": "로터리 혜화로", "J_N": "로터리 창경궁로35길",
}


ZONE1 = ("sz0_", "szT_")   # 어린이 보호구역 구역1 (이화장길 입구 ~ T자) — 부모 차량이 멈추는 곳
STUCK_SEC = 180   # 이보다 오래 계속 멈춘 차가 있으면 "점검 필요" (신호 주기 150초 + 여유)
EM_NAMES = {"FS": "소방서", "S": "이화사거리 남쪽", "NW": "로터리 혜화로", "NE": "로터리 창경궁로",
            "W": "이화사거리 서쪽"}


def lane_kinds(net):
    """차로 ID → 'solid'(버스만) / 'dash'(버스+승용, 점선) / 'gen'."""
    kind = {}
    for e in net.getEdges(withInternal=True):
        for ln in e.getLanes():
            if ln.allows("bus") and not ln.allows("passenger"):
                kind[ln.getID()] = "solid" if not ln.allows("pedestrian") else "walk"
            elif ln.allows("bus") and ln.allows("passenger") and not ln.allows("truck"):
                kind[ln.getID()] = "dash"
            else:
                kind[ln.getID()] = "gen"
    return kind


def validate(sumocfg, seconds, seed):
    out_dir = os.path.dirname(sumocfg)
    net = sumolib.net.readNet(os.path.join(out_dir, "daehakro.net.xml"), withInternal=True,
                              withPedestrianConnections=True)
    kind = lane_kinds(net)
    to_node = {e.getID(): e.getToNode().getID() for e in net.getEdges()}
    crossing_place = {}
    for el in ET.parse(os.path.join(out_dir, "daehakro.net.xml")).getroot().iter("edge"):
        if el.get("function") == "crossing":
            eid = el.get("id")
            node = eid[1:eid.rindex("_c")]
            ces = el.get("crossingEdges", "").split()
            crosses_main = any(c.startswith(("nb_", "sb_")) for c in ces)
            label = PLACE.get(node, node)
            if node == "IH":                              # 이화사거리 4갈래
                leg = ("북쪽" if crosses_main else "동쪽" if any(c.startswith("E_") for c in ces)
                       else "서쪽" if any(c.startswith("W_") for c in ces) else "남쪽")
                label += " " + leg
            elif node in ("O", "M") and not crosses_main:
                label = {"O": "옆길 O 입구", "M": "옆길 N 입구"}[node]
            crossing_place[eid] = label

    col_file = tempfile.NamedTemporaryFile(prefix="daehakro_col_", suffix=".xml", delete=False).name
    cmd = [checkBinary("sumo"), "-c", sumocfg, "--no-step-log", "true", "--seed", str(seed),
           "--end", str(seconds), "--no-warnings", "true", "--duration-log.statistics", "false",
           "--collision-output", col_file]
    traci.start(cmd)
    assist = EmergencyAssist(net_file=os.path.join(out_dir, "daehakro.net.xml"))
    em_ids = {vid: (kind, o, d) for vid, kind, o, d, _ in cfg.EMERGENCY_DISPATCH}
    em = {vid: {"dep": None, "arr": None, "stop": 0.0, "bus": 0.0, "vmax": 0.0} for vid in em_ids}
    em_normal = set()                  # 사이렌 없는 소방차·구급차
    em_normal_bus = set()              # 그중 버스전용차로(실선)에 들어간 차 (0이어야 함)
    step = cfg.SIM["step_length"]
    warm = min(cfg.SIM["warmup"], seconds / 3)
    t = 0.0
    inserted = arrived = teleports = 0
    p_dep = p_arr = 0
    long_wait = {}
    halting = collections.Counter()
    speed_sum = speed_n = 0.0
    n_veh_max = 0
    VARS = [tc.VAR_LANE_ID, tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION, tc.VAR_VEHICLECLASS]
    state = {}
    uturn = collections.Counter()
    solid_cars = set()
    dash = collections.Counter()
    dash_dist = collections.defaultdict(list)
    ped_seen = {}
    ped_cross = collections.Counter()
    parked = set()                     # 구역1 도로 위에 멈춰 아이를 태운 부모 차량
    park_t = {}                        # 부모 차량: [멈춘 것을 처음 본 시각, 마지막 본 시각]
    park_max = 0
    parent_dep = 0
    while t < seconds:
        traci.simulationStep()
        assist.step()
        t = traci.simulation.getTime()
        for v in traci.simulation.getDepartedIDList():
            traci.vehicle.subscribe(v, VARS)
            if v.startswith("em_"):
                em_normal.add(v)
        for v in traci.simulation.getArrivedIDList():
            if v in em and em[v]["dep"] is not None:
                em[v]["arr"] = t
        for p in traci.simulation.getDepartedPersonIDList():
            traci.person.subscribe(p, [tc.VAR_ROAD_ID])
        parent_dep += sum(1 for v in traci.simulation.getDepartedIDList() if v.startswith("parent_"))
        if int(t / step) % int(2 / step) == 0:
            occ = 0
            for v in traci.vehicle.getIDList():
                if (v.startswith("parent_") and traci.vehicle.getRoadID(v).startswith(ZONE1)
                        and traci.vehicle.getStopState(v) & 1 and traci.vehicle.getSpeed(v) < 0.1):
                    parked.add(v)
                    occ += 1
                    park_t.setdefault(v, [t, t])[1] = t
            park_max = max(park_max, occ)
        inserted += traci.simulation.getDepartedNumber()
        arrived += traci.simulation.getArrivedNumber()
        teleports += traci.simulation.getStartingTeleportNumber()
        p_dep += len(traci.simulation.getDepartedPersonIDList())
        p_arr += traci.simulation.getArrivedPersonNumber()
        # 차량: 유턴, 버스전용차로 사용
        for v, r in traci.vehicle.getAllSubscriptionResults().items():
            lane, road, pos = r[tc.VAR_LANE_ID], r[tc.VAR_ROAD_ID], r[tc.VAR_LANEPOSITION]
            if v in em:                                        # 사이렌 켠 출동 차량
                E = em[v]
                if E["dep"] is None:
                    E["dep"] = t
                sp = traci.vehicle.getSpeed(v)
                E["vmax"] = max(E["vmax"], sp)
                E["stop"] += step if sp < 0.1 else 0.0
                if kind.get(lane) in ("solid", "dash"):
                    E["bus"] += sp * step
            elif v in em_normal and kind.get(lane) == "solid":
                em_normal_bus.add(v)
            if road.startswith(":"):
                continue
            st = state.setdefault(v, {"edge": None, "lane": None, "dash": None})
            if st["edge"] and st["edge"] != road:
                a, b = st["edge"], road
                if (a[:3], b[:3]) in (("sb_", "nb_"), ("nb_", "sb_")):
                    uturn[to_node.get(a, "?")] += 1
            st["edge"] = road
            if r[tc.VAR_VEHICLECLASS] != "passenger":
                continue
            k = kind.get(lane, "gen")
            if k == "solid":
                solid_cars.add(v)
            prev = st["lane"]
            if k == "dash":
                d = st["dash"]
                if d is None:
                    if prev is None:
                        how = "출발"
                    elif prev.rsplit("_", 1)[0] == road:
                        how = "차로변경"
                    elif prev.startswith(("nb_", "S_in")):
                        how = "앞 교차로 지나"
                    else:
                        how = "옆길·교차로 우회전"
                    st["dash"] = {"how": how, "dist": 0.0, "pos": pos, "road": road}
                else:
                    d["dist"] += max(0.0, pos - d["pos"]) if road == d["road"] else max(0.0, pos)
                    d["pos"], d["road"] = pos, road
            elif st["dash"] is not None:
                d = st["dash"]
                if road == d["road"]:
                    out = "차로변경으로 나감"
                elif road.startswith("nb_"):
                    out = "그대로 직진"
                else:
                    out = "우회전(옆길·교차로)"
                dash[(d["how"], out)] += 1
                dash_dist[(d["how"], out)].append(d["dist"])
                st["dash"] = None
            st["lane"] = lane
        # 보행자: 건넌 횡단보도
        for p, r in traci.person.getAllSubscriptionResults().items():
            road = r[tc.VAR_ROAD_ID]
            if road in crossing_place and ped_seen.get(p) != road:
                ped_cross[crossing_place[road]] += 1
            ped_seen[p] = road
        if int(t / step) % int(10 / step) == 0:
            ids = traci.vehicle.getIDList()
            n_veh_max = max(n_veh_max, len(ids))
            for v in ids:
                w = traci.vehicle.getWaitingTime(v)
                if w > 90:
                    long_wait[v] = (w, traci.vehicle.getRoadID(v))
                if t > warm:
                    speed_sum += traci.vehicle.getSpeed(v)
                    speed_n += 1
            if t > warm:
                for e in traci.edge.getIDList():
                    if not e.startswith(":"):
                        halting[e] += traci.edge.getLastStepHaltingNumber(e)
    pending_ids = traci.simulation.getPendingVehicles()     # 출발 시각이 됐지만 도로가 막혀 못 들어온 차
    backlog = len(pending_ids)
    pending = collections.Counter(v.split(".")[0] for v in pending_ids)
    # 신호 한 주기(150초)를 넘겨 계속 멈춰 있는 차 (옆길은 적색이 약 125초라 그보다 넉넉히)
    stuck_now = [(v, traci.vehicle.getWaitingTime(v), traci.vehicle.getRoadID(v))
                 for v in traci.vehicle.getIDList() if traci.vehicle.getWaitingTime(v) > STUCK_SEC]
    traci.close()
    col_vv = col_vp = 0
    em_col = []                         # 사이렌 켠 긴급차가 낀 충돌 (0이어야 함)
    try:
        for c in ET.parse(col_file).getroot().findall("collision"):
            if c.get("type") == "crossing" or c.get("victimType", "").startswith("ped"):
                col_vp += 1
            else:
                col_vv += 1
            if c.get("collider") in em or c.get("victim") in em:
                em_col.append((c.get("time"), c.get("collider"), c.get("victim")))
        os.remove(col_file)
    except Exception:
        pass
    samples = max(1, int((seconds - warm) / 10))

    print("─" * 66)
    print(f" 대학로 점검 ({seconds:.0f}초, step {step}s, seed {seed})")
    print("─" * 66)
    print(f" [차량] 투입 {inserted}대 / 도착 {arrived}대 / 동시 최대 {n_veh_max}대 / 투입 대기 {backlog}대")
    if pending:
        print("        투입 대기가 많은 흐름:", ", ".join(f"{k} {n}" for k, n in pending.most_common(5)))
    print(f"        텔레포트 {teleports}회 / 충돌 {col_vv + col_vp}건 (차-차 {col_vv}, 차-보행자 접촉 {col_vp})")
    print(f"        평균 속도 {speed_sum / max(1, speed_n) * 3.6:.1f} km/h (앞 {warm:.0f}초 제외)")
    print(f"        90초 넘게 멈춘 적 있는 차 {len(long_wait)}대 (이화사거리 주기 150초라 적색 대기만으로도 생김), "
          f"지금 {STUCK_SEC}초 넘게 계속 멈춘 차 {len(stuck_now)}대")
    print(f" [버스전용차로] 실선 구간에 들어간 일반 차: {len(solid_cars)}대 (0이어야 정상)")
    n_dash = sum(dash.values())
    print(f"        점선 구간을 잠깐 쓴 일반 차: {n_dash}대")
    for (how, out), n in dash.most_common():
        ds = sorted(dash_dist[(how, out)])
        med = ds[len(ds) // 2] if ds else 0.0
        print(f"          {how} → {out}: {n}대 (버스차로 안에서 달린 거리 중앙값 {med:.0f}m)")
    print(" [유턴] " + (", ".join(f"{PLACE.get(k, k)} {n}대" for k, n in uturn.most_common()) or "없음"))
    stop_s = sorted(last - first + 2 for v, (first, last) in park_t.items() if last < t - 2)
    print(f" [부모 차량] 출발 {parent_dep}대 / 구역1 도로 위에 멈춰 아이를 태움 {len(parked)}대 "
          f"(동시에 최대 {park_max}대, 멈춘 시간 중앙값 {stop_s[len(stop_s) // 2] if stop_s else 0:.0f}초)")
    a = assist.summary()
    print(f" [꼬리물기 방지] 직진·우회전이 갈라지는 차로에서 옆 갈래 차가 교차로 입구를 막고 있어 "
          f"정지선 앞에서 기다린 차 {a.get('box_hold', 0)}대")
    for vid, (kind_, o, d) in em_ids.items():
        E = em[vid]
        name = {"fire": "소방차", "ambulance": "구급차"}[kind_]
        if E["dep"] is None:
            print(f" [긴급차량] {name} {vid}: 아직 출발 전 (출동 시각 이전까지만 시뮬레이션)")
            continue
        took = (E["arr"] if E["arr"] is not None else t) - E["dep"]
        print(f" [긴급차량] {name} {vid} ({EM_NAMES.get(o, o)} → {EM_NAMES.get(d, d)}, 사이렌 "
              f"{'켬' if cfg.SIREN_ON else '끔'}): {'걸린 시간' if E['arr'] is not None else '아직 가는 중'} "
              f"{took:.0f}초, 멈춘 시간 {E['stop']:.0f}초, 버스차로 {E['bus']:.0f}m, 최고 {E['vmax'] * 3.6:.0f} km/h")
    print(f"        길 비켜 준 차 {a['make_way']}대 (그중 버스전용차로에 잠깐 들어간 차 {a['bus_lane_cars']}대), "
          f"우선신호 {a['priority_signal']}회, 횡단보도 사람 때문에 서행 {a['ped_slow']}회")
    print(f"        사이렌 없는 소방차·구급차 {len(em_normal)}대 (버스전용 실선 구간 진입 {len(em_normal_bus)}대, 0이어야 정상)"
          f" / 긴급차가 낀 충돌 {len(em_col)}건 (0이어야 정상)")
    print(f" [보행자] 출발 {p_dep}명 / 도착 {p_arr}명. 횡단보도별 건넌 사람(명):")
    items = sorted(ped_cross.items(), key=lambda x: -x[1])
    for i in range(0, len(items), 4):
        print("          " + "  ".join(f"{k} {n}" for k, n in items[i:i + 4]))
    print(" [정지 차량이 많은 곳] (10초 간격 평균)")
    for e, h in halting.most_common(6):
        print(f"          {e:20s} {h / samples:5.1f}대")
    if stuck_now:
        print("  오래 멈춘 차량 예시:", stuck_now[:5])
    rare = (col_vv + col_vp) <= max(3, round(5 * seconds / 3600))
    ok = (teleports == 0 and len(stuck_now) == 0 and len(solid_cars) == 0 and rare
          and not em_col and not em_normal_bus)
    print(" 판정:", "정상 (텔레포트·장시간 정체·버스 실선 침범·긴급차 충돌 없음, 충돌 기록 드묾)" if ok else "점검 필요")
    if em_col:
        print("   긴급차 충돌:", em_col[:5])
    if col_vv + col_vp:
        print("   ※ 충돌 기록은 SUMO 모델의 한계로 드물게(시간당 0~3건) 생기는 '접촉' 판정입니다: 옆길 입구 횡단보도에서\n"
              "     막 들어선 보행자와 차의 모서리, 긴 버스 꽁무니가 교차로에 걸친 채 서 있을 때 옆길에서 우회전해 나오는 차.\n"
              "     (collision.action=warn 이라 차·보행자는 그대로 진행하고 시뮬레이션에는 영향 없음)")
    return ok


def run_gui(sumocfg, seed):
    """sumo-gui 를 TraCI 로 열고 매 스텝 긴급차 보조(길 비켜 주기·우선신호·횡단보도 서행)를 실행.
    창을 닫으면 끝난다. (TraCI 로 연결돼 있어 GUI 의 '다시 불러오기'는 쓰지 말고 다시 실행)"""
    traci.start([checkBinary("sumo-gui"), "-c", sumocfg, "--seed", str(seed)])
    assist = EmergencyAssist(net_file=os.path.join(os.path.dirname(sumocfg), "daehakro.net.xml"))
    try:
        while traci.simulation.getTime() < cfg.SIM["end"]:
            traci.simulationStep()
            assist.step()
        traci.close()
    except (traci.exceptions.FatalTraCIError, traci.exceptions.TraCIException):
        pass                                   # 사용자가 창을 닫음
    finally:
        try:
            traci.close()
        except Exception:
            pass
    s = assist.summary()
    print(f"긴급차 보조: 길 비켜 준 차 {s['make_way']}대 (버스차로에 잠깐 들어간 차 {s['bus_lane_cars']}대), "
          f"우선신호 {s['priority_signal']}회")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="대학로 도로를 GUI로 확인하거나 점검")
    ap.add_argument("--plain", action="store_true", help="긴급차 보조 없이 sumo-gui 만 열기")
    ap.add_argument("--netedit", action="store_true")
    ap.add_argument("--nogui", action="store_true")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--seconds", type=int, default=cfg.SIM["end"])
    ap.add_argument("--seed", type=int, default=cfg.SIM["seed"])
    args = ap.parse_args()

    sumocfg = daehakro_builder.build()
    net_file = os.path.join(os.path.dirname(sumocfg), "daehakro.net.xml")
    if args.plot:
        subprocess.run([sys.executable, os.path.join(BASE, "utils", "plot_daehakro.py")], check=False)
    elif args.netedit:
        subprocess.run([checkBinary("netedit"), net_file])
    elif args.nogui:
        validate(sumocfg, args.seconds, args.seed)
    elif args.plain:
        print("sumo-gui 실행 중 (긴급차 보조 없음)... ▶ 버튼을 누르면 차량이 움직입니다.")
        subprocess.run([checkBinary("sumo-gui"), "-c", sumocfg])
    else:
        print("sumo-gui 실행 중... ▶ 버튼을 누르면 차량이 움직입니다. (1500초에 소방차·구급차 출동)")
        run_gui(sumocfg, args.seed)
