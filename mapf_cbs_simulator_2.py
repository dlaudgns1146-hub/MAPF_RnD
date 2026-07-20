"""
MAPF CBS Simulator
==================
Multi-Agent Path Finding using Conflict-Based Search (CBS)

변경:
  - 로봇 대수 제한 해제 (최대 99대, Spinbox 직접 입력 가능)
  - 자동 모드: 각 로봇이 목적지 도착 즉시 독립적으로 새 목적지 설정 후 이동
  - 전체 완료 대기 없이 개별 로봇 단위로 재계획
"""

import tkinter as tk
from tkinter import ttk
import heapq
import time
import random
from copy import deepcopy
from dataclasses import dataclass, field

# ──────────────────────────────────────────────
# 설정
# ──────────────────────────────────────────────
GRID_ROWS = 16
GRID_COLS = 22
CELL_SIZE = 46

COLORS = {
    "bg":        "#1e1e2e",
    "grid_bg":   "#2a2a3e",
    "wall":      "#444466",
    "empty":     "#2a2a3e",
    "grid_line": "#3a3a5e",
    "text":      "#ffffff",
    "panel":     "#16213e",
    "btn":       "#0f3460",
    "btn_active":"#e94560",
    "status_ok": "#2ecc71",
    "status_err":"#e74c3c",
    "auto_btn":  "#1a6b4a",
    "robot": [
        "#e74c3c","#2ecc71","#f39c12","#9b59b6",
        "#1abc9c","#e67e22","#3498db","#ff6b9d",
        "#00cec9","#fdcb6e","#6c5ce7","#a29bfe",
        "#fd79a8","#55efc4","#fab1a0","#74b9ff",
    ],
    "goal": [
        "#ff6b6b","#6bff8a","#ffd36b","#c56bff",
        "#6bffd3","#ffb36b","#6bc4ff","#ffb6d9",
        "#80fffb","#ffe08b","#a08bff","#c8c4ff",
        "#ffaad0","#88ffde","#ffd4c4","#aad6ff",
    ],
}

# ──────────────────────────────────────────────
# A* (제약 조건 포함)
# ──────────────────────────────────────────────
def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar_with_constraints(grid, start, goal, constraints, max_t=300):
    rows, cols = len(grid), len(grid[0])
    open_heap = []
    heapq.heappush(open_heap, (heuristic(start, goal), 0, start, 0, [start]))
    visited = {}
    while open_heap:
        f, g, pos, t, path = heapq.heappop(open_heap)
        if pos == goal and t >= heuristic(start, goal):
            return path
        state = (pos, t)
        if state in visited:
            continue
        visited[state] = True
        if t > max_t:
            continue
        for npos in [pos,
                     (pos[0]-1, pos[1]),
                     (pos[0]+1, pos[1]),
                     (pos[0],   pos[1]-1),
                     (pos[0],   pos[1]+1)]:
            r, c = npos
            if not (0 <= r < rows and 0 <= c < cols):
                continue
            if grid[r][c] == 1:
                continue
            nt = t + 1
            if (r, c, nt) in constraints:
                continue
            if (pos[0], pos[1], r, c, nt) in constraints:
                continue
            if (npos, nt) in visited:
                continue
            heapq.heappush(open_heap,
                (g+1+heuristic(npos, goal), g+1, npos, nt, path+[npos]))
    return None

# ──────────────────────────────────────────────
# CBS
# ──────────────────────────────────────────────
@dataclass(order=True)
class CBSNode:
    cost: int
    constraints: dict = field(compare=False)
    paths:        dict = field(compare=False)

def find_first_conflict(paths):
    agents = list(paths.keys())
    max_t  = max(len(p) for p in paths.values())
    def pos_at(path, t):
        return path[min(t, len(path)-1)]
    for t in range(max_t):
        seen = {}
        for aid in agents:
            p = pos_at(paths[aid], t)
            if p in seen:
                return ("vertex", t, p, seen[p], aid)
            seen[p] = aid
        if t + 1 < max_t:
            for i in range(len(agents)):
                for j in range(i+1, len(agents)):
                    a1, a2 = agents[i], agents[j]
                    p1t,  p1t1 = pos_at(paths[a1],t), pos_at(paths[a1],t+1)
                    p2t,  p2t1 = pos_at(paths[a2],t), pos_at(paths[a2],t+1)
                    if p1t == p2t1 and p1t1 == p2t:
                        return ("edge", t, p1t, p1t1, a1, a2)
    return None

def cbs(grid, agents, max_iter=8000):
    n = len(agents)
    init_c = {i: set() for i in range(n)}
    init_p = {}
    for i, (s, g) in enumerate(agents):
        p = astar_with_constraints(grid, s, g, set())
        if p is None:
            return None
        init_p[i] = p
    root = CBSNode(sum(len(p) for p in init_p.values()), init_c, init_p)
    heap = [root]
    for _ in range(max_iter):
        if not heap:
            break
        node = heapq.heappop(heap)
        conflict = find_first_conflict(node.paths)
        if conflict is None:
            return node.paths
        if conflict[0] == "vertex":
            _, t, pos, a1, a2 = conflict
            for ai in [a1, a2]:
                nc = deepcopy(node.constraints)
                nc[ai].add((pos[0], pos[1], t))
                np_ = deepcopy(node.paths)
                s, g = agents[ai]
                p = astar_with_constraints(grid, s, g, nc[ai])
                if p is None: continue
                np_[ai] = p
                heapq.heappush(heap, CBSNode(sum(len(x) for x in np_.values()), nc, np_))
        else:
            _, t, p1, p2, a1, a2 = conflict
            for ai, (fr, to) in [(a1,(p1,p2)), (a2,(p2,p1))]:
                nc = deepcopy(node.constraints)
                nc[ai].add((fr[0], fr[1], to[0], to[1], t+1))
                np_ = deepcopy(node.paths)
                s, g = agents[ai]
                p = astar_with_constraints(grid, s, g, nc[ai])
                if p is None: continue
                np_[ai] = p
                heapq.heappush(heap, CBSNode(sum(len(x) for x in np_.values()), nc, np_))
    return None

# ──────────────────────────────────────────────
# GUI
# ──────────────────────────────────────────────
class MAPFApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MAPF CBS Simulator")
        self.root.configure(bg=COLORS["bg"])
        self.root.resizable(True, True)

        self.grid = [[0]*GRID_COLS for _ in range(GRID_ROWS)]

        # 수동 모드용
        self.robots = []
        self.paths  = {}
        self.anim_t = 0
        self.anim_running = False
        self.anim_id = None
        self.anim_speed = 150

        # 자동 모드용 — 로봇별 독립 상태
        self.auto_mode = False
        self.auto_robot_count = tk.IntVar(value=4)
        # {aid: {"pos":(r,c), "goal":(r,c), "path":[(r,c),...], "step":int}}
        self.auto_agents = {}
        self.auto_total_arrivals = 0

        self.mode = tk.StringVar(value="wall")
        self.pending_start = None

        self._build_ui()
        self._draw_grid()

    # ────────────────────────────────────────────
    # UI
    # ────────────────────────────────────────────
    def _build_ui(self):
        left  = tk.Frame(self.root, bg=COLORS["bg"])
        right = tk.Frame(self.root, bg=COLORS["panel"], width=250)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8, pady=8)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(0,8), pady=8)
        right.pack_propagate(False)

        cw = GRID_COLS * CELL_SIZE + 2
        ch = GRID_ROWS * CELL_SIZE + 2
        self.canvas = tk.Canvas(left, width=cw, height=ch,
                                bg=COLORS["grid_bg"], highlightthickness=0,
                                cursor="crosshair")
        self.canvas.pack(pady=4)
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<B1-Motion>", self._on_drag)

        self.status_var = tk.StringVar(value="준비됨 — 모드를 선택하고 그리드를 편집하세요")
        tk.Label(left, textvariable=self.status_var,
                 bg=COLORS["bg"], fg=COLORS["status_ok"],
                 font=("Segoe UI", 10), anchor="w").pack(fill=tk.X, padx=4)

        # ── 오른쪽 패널 ──
        self._ptitle(right, "MAPF CBS 시뮬레이터")

        # 자동 모드 박스
        af = tk.LabelFrame(right, text=" 자동 모드 ",
                           bg=COLORS["panel"], fg="#aaaacc",
                           font=("Segoe UI", 9, "bold"), relief=tk.GROOVE, bd=1)
        af.pack(fill=tk.X, padx=8, pady=(4,2))

        row = tk.Frame(af, bg=COLORS["panel"])
        row.pack(fill=tk.X, padx=8, pady=6)
        tk.Label(row, text="로봇 대수:", bg=COLORS["panel"], fg=COLORS["text"],
                 font=("Segoe UI", 10)).pack(side=tk.LEFT)

        # Spinbox: 1~99, 직접 타이핑 가능
        vcmd = (self.root.register(lambda s: s.isdigit() and 1 <= int(s) <= 99
                                   if s else True), "%P")
        self.spin = tk.Spinbox(
            row, from_=1, to=99, width=5,
            textvariable=self.auto_robot_count,
            bg=COLORS["btn"], fg=COLORS["text"],
            buttonbackground=COLORS["btn"],
            font=("Segoe UI", 11, "bold"), relief=tk.FLAT,
            validate="key", validatecommand=vcmd,
        )
        self.spin.pack(side=tk.RIGHT)

        tk.Button(af, text="랜덤 배치 후 자동 시작",
                  command=self._auto_place_and_start,
                  bg=COLORS["auto_btn"], fg="white",
                  font=("Segoe UI", 10, "bold"),
                  relief=tk.FLAT, cursor="hand2", pady=6
                  ).pack(fill=tk.X, padx=8, pady=(0,6))

        self.auto_label = tk.Label(af, text="자동 모드: OFF",
                                   bg=COLORS["panel"], fg="#888899",
                                   font=("Segoe UI", 9, "bold"))
        self.auto_label.pack(pady=(0,4))

        ttk.Separator(right, orient="horizontal").pack(fill=tk.X, padx=8, pady=6)

        # 수동 편집 모드
        self._ptitle(right, "편집 모드", small=True)
        for label, val in [("장애물 그리기","wall"),("로봇 시작점","robot_start"),
                            ("로봇 목표점","robot_goal"),("지우기","erase")]:
            tk.Radiobutton(right, text=label, variable=self.mode, value=val,
                           bg=COLORS["panel"], fg=COLORS["text"],
                           selectcolor=COLORS["btn_active"],
                           activebackground=COLORS["panel"],
                           font=("Segoe UI", 10), anchor="w"
                           ).pack(fill=tk.X, padx=12, pady=1)

        ttk.Separator(right, orient="horizontal").pack(fill=tk.X, padx=8, pady=6)

        self._ptitle(right, "애니메이션 속도", small=True)
        self.speed_var = tk.IntVar(value=150)
        tk.Scale(right, from_=30, to=800, orient=tk.HORIZONTAL,
                 variable=self.speed_var, bg=COLORS["panel"],
                 fg=COLORS["text"], troughcolor=COLORS["btn"],
                 highlightthickness=0, label="ms/step",
                 command=lambda v: setattr(self,"anim_speed",int(v))
                 ).pack(fill=tk.X, padx=12)

        ttk.Separator(right, orient="horizontal").pack(fill=tk.X, padx=8, pady=6)

        for label, cmd, color in [
            ("CBS 경로 계획",   self._plan,       COLORS["btn_active"]),
            ("애니메이션 시작", self._start_anim, "#27ae60"),
            ("정지",            self._stop_anim,  "#e67e22"),
            ("처음으로",        self._reset_anim, "#8e44ad"),
            ("전체 초기화",     self._clear_all,  "#c0392b"),
        ]:
            tk.Button(right, text=label, command=cmd,
                      bg=color, fg="white", font=("Segoe UI", 10, "bold"),
                      relief=tk.FLAT, cursor="hand2",
                      activebackground=COLORS["btn"], pady=6
                      ).pack(fill=tk.X, padx=12, pady=3)

        ttk.Separator(right, orient="horizontal").pack(fill=tk.X, padx=8, pady=6)

        self._ptitle(right, "로봇 목록", small=True)
        f = tk.Frame(right, bg=COLORS["panel"])
        f.pack(fill=tk.BOTH, expand=True, padx=8)
        self.robot_list = tk.Text(f, bg="#0d1b2a", fg=COLORS["text"],
                                  font=("Consolas", 9), relief=tk.FLAT,
                                  state=tk.DISABLED, height=8)
        self.robot_list.pack(fill=tk.BOTH, expand=True)

        ttk.Separator(right, orient="horizontal").pack(fill=tk.X, padx=8, pady=6)
        self._ptitle(right, "예제 맵", small=True)
        for label, cmd in [("창고 맵 로드", self._load_warehouse),
                            ("랜덤 장애물",  self._random_walls)]:
            tk.Button(right, text=label, command=cmd,
                      bg=COLORS["btn"], fg="white", font=("Segoe UI", 9),
                      relief=tk.FLAT, cursor="hand2", pady=4
                      ).pack(fill=tk.X, padx=12, pady=2)

    def _ptitle(self, parent, text, small=False):
        font = ("Segoe UI", 10, "bold") if small else ("Segoe UI", 13, "bold")
        tk.Label(parent, text=text, bg=COLORS["panel"], fg=COLORS["text"],
                 font=font, anchor="w").pack(fill=tk.X, padx=10, pady=(6,2))

    # ────────────────────────────────────────────
    # 자동 모드 핵심
    # ────────────────────────────────────────────
    def _free_cells(self):
        return [(r, c) for r in range(GRID_ROWS)
                         for c in range(GRID_COLS)
                         if self.grid[r][c] == 0]

    def _auto_place_and_start(self):
        """랜덤 배치 → 각 로봇 개별 A* → 자동 틱 루프 시작"""
        self._stop_anim()
        n = self._get_robot_count()
        if n is None:
            return
        free = self._free_cells()
        if len(free) < n * 2:
            self._set_status("빈 셀 부족 — 장애물 줄이거나 로봇 수를 줄이세요", error=True)
            return

        self.robots = []
        self.paths  = {}
        self.anim_t = 0
        self.auto_agents = {}
        self.auto_total_arrivals = 0

        chosen = random.sample(free, n * 2)
        starts, goals = chosen[:n], chosen[n:]

        for i in range(n):
            sr, sc = starts[i]
            gr, gc = goals[i]
            self.robots.append((sr, sc, gr, gc))
            path = astar_with_constraints(self.grid, (sr,sc), (gr,gc), set())
            if path is None:
                path = [(sr, sc)]
            self.auto_agents[i] = {
                "pos":  (sr, sc),
                "goal": (gr, gc),
                "path": path,
                "step": 0,
            }

        self.auto_mode = True
        self.auto_label.config(text=f"자동 모드: ON ({n}대)", fg="#2ecc71")
        self._set_status(f"자동 모드 시작 — 로봇 {n}대 독립 이동 중!")
        self._draw_grid()
        self.anim_running = True
        self._auto_tick()

    def _get_robot_count(self):
        try:
            n = int(self.auto_robot_count.get())
            if n < 1:
                raise ValueError
            return n
        except (ValueError, tk.TclError):
            self._set_status("로봇 대수를 올바르게 입력하세요 (1 이상)", error=True)
            return None

    def _pick_new_goal(self, agent_id):
        """새 랜덤 목적지: 현재 위치·다른 로봇 목적지와 겹치지 않게"""
        current_pos  = self.auto_agents[agent_id]["pos"]
        taken_goals  = {v["goal"] for k, v in self.auto_agents.items() if k != agent_id}
        excluded     = {current_pos} | taken_goals
        candidates   = [c for c in self._free_cells() if c not in excluded]
        if not candidates:
            candidates = [c for c in self._free_cells() if c != current_pos]
        return random.choice(candidates) if candidates else current_pos

    def _replan_single(self, agent_id):
        """
        단일 로봇 재계획 (A* + 다른 로봇의 예정 경로를 vertex 제약으로 반영)
        로봇 수가 많을 때 CBS 전체 재계획 대신 빠르게 개별 처리
        """
        agent = self.auto_agents[agent_id]
        start = agent["pos"]
        goal  = agent["goal"]

        # 다른 로봇들의 미래 위치를 제약으로 추출
        constraints = set()
        for oid, oa in self.auto_agents.items():
            if oid == agent_id:
                continue
            path = oa["path"]
            step = oa["step"]
            for dt in range(len(path) - step):
                r, c = path[min(step + dt, len(path)-1)]
                constraints.add((r, c, dt + 1))

        path = astar_with_constraints(self.grid, start, goal, constraints)
        if path is None:
            path = astar_with_constraints(self.grid, start, goal, set())
        if path is None:
            path = [start]

        agent["path"] = path
        agent["step"] = 0

    def _auto_tick(self):
        """매 스텝: 각 로봇 독립적으로 전진, 도착 즉시 새 목적지 배정"""
        if not self.anim_running or not self.auto_mode:
            return

        arrived_ids = []

        for aid, agent in self.auto_agents.items():
            path = agent["path"]
            step = agent["step"]
            next_step = min(step + 1, len(path) - 1)
            agent["step"] = next_step
            agent["pos"]  = path[next_step]

            if agent["pos"] == agent["goal"]:
                arrived_ids.append(aid)

        # 도착한 로봇들 처리 (새 목적지 + 재계획)
        for aid in arrived_ids:
            self.auto_total_arrivals += 1
            new_goal = self._pick_new_goal(aid)
            self.auto_agents[aid]["goal"] = new_goal
            self._replan_single(aid)

        self._draw_grid()

        if arrived_ids:
            self._set_status(
                f"자동 모드 — 누적 도착: {self.auto_total_arrivals}회 | "
                f"로봇 {len(self.auto_agents)}대 이동 중"
            )

        self.anim_id = self.root.after(self.anim_speed, self._auto_tick)

    # ────────────────────────────────────────────
    # 수동 모드 CBS
    # ────────────────────────────────────────────
    def _run_cbs(self, robots):
        agent_list = [((sr,sc),(gr,gc)) for sr,sc,gr,gc in robots]
        self._set_status("CBS 경로 계획 중...")
        self.root.update()
        t0 = time.time()
        result = cbs(self.grid, agent_list)
        elapsed = time.time() - t0
        if result:
            makespan = max(len(p) for p in result.values())
            self._set_status(
                f"CBS 완료 {elapsed:.2f}s | 로봇 {len(robots)}대 | Makespan: {makespan}"
            )
        return result

    def _plan(self):
        if not self.robots:
            self._set_status("먼저 로봇을 배치하세요", error=True)
            return
        self._stop_anim()
        result = self._run_cbs(self.robots)
        if result is None:
            self._set_status("경로를 찾을 수 없습니다 (CBS 실패)", error=True)
            return
        self.paths  = result
        self.anim_t = 0
        self._draw_grid()

    # ────────────────────────────────────────────
    # 애니메이션 (수동 모드)
    # ────────────────────────────────────────────
    def _start_anim(self):
        if self.auto_mode:
            if not self.anim_running:
                self.anim_running = True
                self._auto_tick()
            return
        if not self.paths:
            self._plan()
            if not self.paths:
                return
        self.anim_running = True
        self._manual_tick()

    def _manual_tick(self):
        if not self.anim_running:
            return
        max_t = max(len(p) for p in self.paths.values())
        self._draw_grid()
        if self.anim_t < max_t - 1:
            self.anim_t += 1
            self.anim_id = self.root.after(self.anim_speed, self._manual_tick)
        else:
            self.anim_running = False
            self._set_status("애니메이션 완료!")

    def _stop_anim(self):
        self.anim_running = False
        if self.anim_id:
            self.root.after_cancel(self.anim_id)
            self.anim_id = None

    def _reset_anim(self):
        self._stop_anim()
        self.anim_t = 0
        self._draw_grid()

    def _clear_paths(self):
        self.paths = {}
        self.anim_t = 0
        self._stop_anim()

    def _clear_all(self):
        self._stop_anim()
        self.grid    = [[0]*GRID_COLS for _ in range(GRID_ROWS)]
        self.robots  = []
        self.paths   = {}
        self.anim_t  = 0
        self.pending_start = None
        self.auto_mode = False
        self.auto_agents = {}
        self.auto_total_arrivals = 0
        self.auto_label.config(text="자동 모드: OFF", fg="#888899")
        self.mode.set("wall")
        self._set_status("초기화 완료")
        self._draw_grid()

    # ────────────────────────────────────────────
    # 그리기
    # ────────────────────────────────────────────
    def _draw_grid(self):
        self.canvas.delete("all")
        cs = CELL_SIZE

        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                color = COLORS["wall"] if self.grid[r][c] == 1 else COLORS["empty"]
                self.canvas.create_rectangle(
                    c*cs, r*cs, c*cs+cs, r*cs+cs,
                    fill=color, outline=COLORS["grid_line"], width=1)

        if self.auto_mode:
            self._draw_auto()
        else:
            self._draw_manual()

        if self.pending_start:
            r, c = self.pending_start
            cidx = len(self.robots) % len(COLORS["robot"])
            x, y = c*cs+cs//2, r*cs+cs//2
            self.canvas.create_oval(x-cs//2+4, y-cs//2+4, x+cs//2-4, y+cs//2-4,
                                    fill=COLORS["robot"][cidx], outline="white",
                                    width=2, dash=(4,2))
            self.canvas.create_text(x, y, text="?", fill="white",
                                    font=("Arial",11,"bold"))

        self._update_robot_list()

    def _draw_auto(self):
        cs = CELL_SIZE
        n  = len(self.auto_agents)
        fsize = 8 if n > 20 else 9 if n > 10 else 11

        for aid, agent in self.auto_agents.items():
            cidx   = aid % len(COLORS["robot"])
            rcolor = COLORS["robot"][cidx]
            gcolor = COLORS["goal"][cidx]

            # 잔여 경로 선
            path = agent["path"]
            step = agent["step"]
            for i in range(step, len(path)-1):
                r1,c1 = path[i]; r2,c2 = path[i+1]
                self.canvas.create_line(
                    c1*cs+cs//2, r1*cs+cs//2,
                    c2*cs+cs//2, r2*cs+cs//2,
                    fill=rcolor, width=2, dash=(4,3))

            # 목표 마커
            gr, gc = agent["goal"]
            x, y = gc*cs+cs//2, gr*cs+cs//2
            self.canvas.create_oval(x-cs//3, y-cs//3, x+cs//3, y+cs//3,
                                    fill=gcolor, outline="white", width=2)
            self.canvas.create_text(x, y, text="G", fill="white",
                                    font=("Arial",9,"bold"))

            # 로봇 마커
            pr, pc = agent["pos"]
            x2, y2 = pc*cs+cs//2, pr*cs+cs//2
            self.canvas.create_oval(x2-cs//2+4, y2-cs//2+4,
                                    x2+cs//2-4, y2+cs//2-4,
                                    fill=rcolor, outline="white", width=2)
            self.canvas.create_text(x2, y2, text=str(aid+1), fill="white",
                                    font=("Arial",fsize,"bold"))

    def _draw_manual(self):
        cs = CELL_SIZE
        if self.paths:
            for aid, path in self.paths.items():
                color = COLORS["robot"][aid % len(COLORS["robot"])]
                for i in range(len(path)-1):
                    r1,c1 = path[i]; r2,c2 = path[i+1]
                    self.canvas.create_line(
                        c1*cs+cs//2, r1*cs+cs//2,
                        c2*cs+cs//2, r2*cs+cs//2,
                        fill=color, width=3, dash=(4,2))

        for i, (sr,sc,gr,gc) in enumerate(self.robots):
            cidx   = i % len(COLORS["robot"])
            rcolor = COLORS["robot"][cidx]
            gcolor = COLORS["goal"][cidx]

            x, y = gc*cs+cs//2, gr*cs+cs//2
            self.canvas.create_oval(x-cs//3, y-cs//3, x+cs//3, y+cs//3,
                                    fill=gcolor, outline="white", width=2)
            self.canvas.create_text(x, y, text="G", fill="white",
                                    font=("Arial",10,"bold"))

            if self.paths and i in self.paths:
                pr, pc = self.paths[i][min(self.anim_t, len(self.paths[i])-1)]
            else:
                pr, pc = sr, sc
            x2, y2 = pc*cs+cs//2, pr*cs+cs//2
            self.canvas.create_oval(x2-cs//2+4, y2-cs//2+4,
                                    x2+cs//2-4, y2+cs//2-4,
                                    fill=rcolor, outline="white", width=2)
            self.canvas.create_text(x2, y2, text=str(i+1), fill="white",
                                    font=("Arial",11,"bold"))

    # ────────────────────────────────────────────
    # 클릭 / 드래그 (수동 편집)
    # ────────────────────────────────────────────
    def _cell_from_event(self, event):
        c, r = event.x // CELL_SIZE, event.y // CELL_SIZE
        if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
            return r, c
        return None

    def _on_click(self, event):
        if self.auto_mode:
            return
        cell = self._cell_from_event(event)
        if cell is None:
            return
        r, c = cell
        mode = self.mode.get()
        if mode == "wall":
            self.grid[r][c] = 1; self._clear_paths()
        elif mode == "erase":
            self.grid[r][c] = 0
            self.robots = [(sr,sc,gr,gc) for sr,sc,gr,gc in self.robots
                           if (sr,sc)!=(r,c) and (gr,gc)!=(r,c)]
            self._clear_paths()
        elif mode == "robot_start":
            if self.grid[r][c] == 1:
                self._set_status("장애물 위에 로봇 배치 불가", error=True); return
            self.pending_start = (r, c)
            self._set_status(f"시작점 ({r},{c}) — 이제 목표점 클릭")
            self.mode.set("robot_goal")
        elif mode == "robot_goal":
            if self.pending_start is None:
                self._set_status("먼저 시작점을 설정하세요", error=True)
                self.mode.set("robot_start"); return
            if self.grid[r][c] == 1:
                self._set_status("장애물 위에 목표점 설정 불가", error=True); return
            sr, sc = self.pending_start
            if (r,c) == (sr,sc):
                self._set_status("시작점 = 목표점은 불가", error=True); return
            self.robots.append((sr, sc, r, c))
            self.pending_start = None
            self._clear_paths()
            self._set_status(f"로봇 {len(self.robots)} 추가: ({sr},{sc})→({r},{c})")
            self.mode.set("robot_start")
        self._draw_grid()

    def _on_drag(self, event):
        if self.auto_mode:
            return
        cell = self._cell_from_event(event)
        if cell is None:
            return
        r, c = cell
        mode = self.mode.get()
        if mode == "wall":
            self.grid[r][c] = 1; self._clear_paths(); self._draw_grid()
        elif mode == "erase":
            self.grid[r][c] = 0
            self.robots = [(sr,sc,gr,gc) for sr,sc,gr,gc in self.robots
                           if (sr,sc)!=(r,c) and (gr,gc)!=(r,c)]
            self._clear_paths(); self._draw_grid()

    # ────────────────────────────────────────────
    # 예제 맵
    # ────────────────────────────────────────────
    def _load_warehouse(self):
        self._clear_all()
        for r in range(2, GRID_ROWS-2, 3):
            for c in range(3, GRID_COLS-3, 4):
                for dr in range(2):
                    if r+dr < GRID_ROWS:
                        self.grid[r+dr][c] = 1
                        if c+1 < GRID_COLS:
                            self.grid[r+dr][c+1] = 1
        for sr,sc,gr,gc in [(0,0,GRID_ROWS-1,GRID_COLS-1),
                             (0,GRID_COLS-1,GRID_ROWS-1,0),
                             (GRID_ROWS//2,0,GRID_ROWS//2,GRID_COLS-1),
                             (0,GRID_COLS//2,GRID_ROWS-1,GRID_COLS//2)]:
            if self.grid[sr][sc]==0 and self.grid[gr][gc]==0:
                self.robots.append((sr,sc,gr,gc))
        self._set_status("창고 맵 로드됨")
        self._draw_grid()

    def _random_walls(self):
        self._clear_all()
        for _ in range(GRID_ROWS*GRID_COLS//5):
            r = random.randint(0, GRID_ROWS-1)
            c = random.randint(0, GRID_COLS-1)
            if (r,c) not in [(0,0),(0,1),(1,0),(GRID_ROWS-1,GRID_COLS-1)]:
                self.grid[r][c] = 1
        self._set_status("랜덤 장애물 생성됨")
        self._draw_grid()

    # ────────────────────────────────────────────
    # 유틸
    # ────────────────────────────────────────────
    def _set_status(self, msg, error=False):
        self.status_var.set(msg)

    def _update_robot_list(self):
        self.robot_list.configure(state=tk.NORMAL)
        self.robot_list.delete("1.0", tk.END)
        if self.auto_mode:
            self.robot_list.insert(
                tk.END,
                f"  [자동 | 누적 도착: {self.auto_total_arrivals}]\n\n"
            )
            for aid, agent in self.auto_agents.items():
                pr, pc = agent["pos"]
                gr, gc = agent["goal"]
                remaining = max(0, len(agent["path"]) - agent["step"] - 1)
                self.robot_list.insert(
                    tk.END,
                    f"  R{aid+1}: ({pr},{pc})→({gr},{gc})  [{remaining}step]\n"
                )
        else:
            if not self.robots:
                self.robot_list.insert(tk.END, "  (로봇 없음)\n")
            for i, (sr,sc,gr,gc) in enumerate(self.robots):
                steps = len(self.paths[i]) if i in self.paths else "?"
                self.robot_list.insert(
                    tk.END, f"  R{i+1}: ({sr},{sc})→({gr},{gc})  [{steps}step]\n"
                )
        self.robot_list.configure(state=tk.DISABLED)


# ──────────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    app = MAPFApp(root)
    root.mainloop()