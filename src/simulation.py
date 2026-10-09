#!/usr/bin/env python3
"""AI Autonomous Navigation — pure stdlib (no external libraries needed).
A* path planning + simulation loop, with ASCII grid visualization."""
import math
import heapq
import os

# ─── Config ───
W, H = 800, 600
CELL = 10
START = (100, 500)
GOAL = (700, 100)
OBSTACLES = [(300, 200, 100, 50), (500, 400, 80, 80), (200, 350, 120, 40), (600, 150, 60, 120)]
SPEED = 3.0

# ─── Build occupancy grid (True = obstacle) ───
occ = [[False] * (W // CELL) for _ in range(H // CELL)]
for x, y, w, h in OBSTACLES:
    for cx in range(x // CELL, (x + w) // CELL + 1):
        for cy in range(y // CELL, (y + h) // CELL + 1):
            if 0 <= cx < len(occ) and 0 <= cy < len(occ[0]):
                occ[cx][cy] = True

# ─── Helpers ───
def w2g(p):  # world -> grid row,col
    return int(p[1] // CELL), int(p[0] // CELL)

def g2w(c):  # grid -> world (center)
    return c[1] * CELL + CELL // 2, c[0] * CELL + CELL // 2

def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def neighbors(p):
    x, y = p
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(occ) and 0 <= ny < len(occ[0]) and not occ[nx][ny]:
                yield (nx, ny)

def a_star(start, goal):
    """A* with Euclidean heuristic on the occupancy grid."""
    if occ[start[0]][start[1]] or occ[goal[0]][goal[1]]:
        return None
    open_set = [(0, start)]
    came = {start: None}
    g = {start: 0}
    f = {start: heuristic(start, goal)}
    seen = {start}
    while open_set:
        _, cur = heapq.heappop(open_set)
        seen.discard(cur)
        if cur == goal:
            path = [cur]
            while came[cur] is not None:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        for nb in neighbors(cur):
            cost = g[cur] + (math.sqrt(2) if abs(nb[0] - cur[0]) == 1 and abs(nb[1] - cur[1]) == 1 else 1)
            if nb not in g or cost < g[nb]:
                came[nb] = cur
                g[nb] = cost
                f[nb] = cost + heuristic(nb, goal)
                if nb not in seen:
                    heapq.heappush(open_set, (f[nb], nb))
                    seen.add(nb)
    return None

# ─── ASCII map renderer ───
def render_grid(agent_grid, path):
    rows, cols = len(occ), len(occ[0])
    lines = []
    lines.append(" " + "_" * (cols * 4 + 1))
    for r in range(rows):
        line = str(r).rjust(2)
        for c in range(cols):
            if path and (r, c) == path[0]:
                ch = "P"
            elif (r, c) == agent_grid:
                ch = "@"
            elif (r, c) == w2g(GOAL):
                ch = "G"
            elif (r, c) == w2g(START):
                ch = "S"
            elif occ[r][c]:
                ch = "#"
            else:
                ch = "."
            line += f" {ch}"
        line += " |"
        lines.append(line)
    lines.append(" " + "-" * (cols * 4 + 1))
    return "\n".join(lines)

# ─── Simulation loop ───
agent = tuple(w2g(START))
goal = w2g(GOAL)
path = a_star(agent, goal)
print(f"Initial path: {len(path) if path else 0} waypoints\n")

steps = 0
max_steps = 1000
while path and steps < max_steps:
    if agent == goal:
        break
    agent = path.pop(0)
    steps += 1
    if steps % 10 == 0:
        grid = render_grid(agent, path)
        print(grid)
        print(f"\n[step {steps}] position {g2w(agent)} | remaining {len(path)}")

print("\n" + "=" * 40)
if agent == goal:
    print("SUCCESS: Agent reached the goal!")
else:
    print("WARNING: Simulation ended without reaching goal.")
print(f"Steps taken: {steps}")
print("=" * 40)

# ─── Save ASCII proof to outputs/ ───
output_dir = os.path.join(os.path.dirname(__file__), '..', 'outputs')
os.makedirs(output_dir, exist_ok=True)
proof_path = os.path.join(output_dir, 'simulation_log.txt')
try:
    lines = render_grid(w2g(START), path)
    with open(proof_path, 'w') as f:
        f.write(lines + "\n")
        f.write(f"\nSimulation result: {'Goal reached' if agent == goal else 'Not reached'}\n")
        f.write(f"Steps taken: {steps}\n")
    print(f"\nSaved ASCII proof to {proof_path}")
except Exception as e:
    print(f"\nCould not write proof file: {e}")

# ─── Auto-proof generation ───
proof_dir = os.path.join(os.path.dirname(__file__), '..', 'outputs', 'proofs', 'run_' + __import__('datetime').datetime.now().strftime('%Y%m%d_%H%M%S'))
os.makedirs(proof_dir, exist_ok=True)
with open(os.path.join(proof_dir, 'proof.txt'), 'w') as f:
    f.write(f"Run: {__import__('datetime').datetime.now()}\nPath waypoints: {len(path) if path else 0}\nStatus: {'SUCCESS' if agent == goal else 'RUNNING'}\nSteps: {steps}\n")
print(f"Auto-proof saved: {proof_dir}")
