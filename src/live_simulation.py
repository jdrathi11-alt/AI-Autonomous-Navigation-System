#!/usr/bin/env python3
"""Live window simulation using tkinter (built into Python — no external packages)."""
import math, heapq, tkinter as tk

# Config
W, H = 800, 600
CELL = 10
START = (100, 500)
GOAL = (700, 100)
OBSTACLES = [(300, 200, 100, 50), (500, 400, 80, 80), (200, 350, 120, 40), (600, 150, 60, 120)]
SPEED = 3
FPS_DELAY = 30

# Grid
occ = [[False]*(W//CELL) for _ in range(H//CELL)]
for x,y,w,h in OBSTACLES:
    for cx in range(x//CELL, (x+w)//CELL+1):
        for cy in range(y//CELL, (y+h)//CELL+1):
            if 0<=cx<len(occ) and 0<=cy<len(occ[0]):
                occ[cx][cy]=True

def w2g(p): return int(p[1]//CELL), int(p[0]//CELL)
def g2w(c): return c[1]*CELL+CELL//2, c[0]*CELL+CELL//2

def heuristic(a,b): return math.hypot(a[0]-b[0],a[1]-b[1])

def neighbors(p):
    x,y = p
    for dx in (-1,0,1):
        for dy in (-1,0,1):
            if dx==dy==0: continue
            nx,ny = x+dx,y+dy
            if 0<=nx<len(occ) and 0<=ny<len(occ[0]) and not occ[nx][ny]:
                yield (nx,ny)

def a_star(start,goal):
    if occ[start[0]][start[1]] or occ[goal[0]][goal[1]]: return None
    open_set=[]; heapq.heappush(open_set,(0,start)); came={start:None}; g={start:0}; f={start:heuristic(start,goal)}; seen={start}
    while open_set:
        _,cur=heapq.heappop(open_set)
        seen.discard(cur)
        if cur==goal:
            path=[cur]
            while came[cur] is not None: cur=came[cur]; path.append(cur)
            return path[::-1]
        for nb in neighbors(cur):
            cost=g[cur]+(math.sqrt(2) if abs(nb[0]-cur[0])==1 and abs(nb[1]-cur[1])==1 else 1)
            if nb not in g or cost<g[nb]:
                came[nb]=cur; g[nb]=cost; f[nb]=cost+heuristic(nb,goal)
                if nb not in seen: heapq.heappush(open_set,(f[nb],nb)); seen.add(nb)
    return None

def run():
    root=tk.Tk()
    root.title("AI Autonomous Navigation — Live Simulation")
    canvas=tk.Canvas(root,width=W,height=H,bg="#1a1a2e")
    canvas.pack()
    # Draw static obstacles
    for x,y,w,h in OBSTACLES:
        canvas.create_rectangle(x,y,x+w,y+h,fill="#c03030",outline="#ff4040")
    # Start and goal circles
    s=canvas.create_oval(START[0]-8,START[1]-8,START[0]+8,START[1]+8,fill="#00ff00",outline="#00aa00",width=2)
    g=canvas.create_oval(GOAL[0]-8,GOAL[1]-8,GOAL[0]+8,GOAL[1]+8,fill="#ff0000",outline="#aa0000",width=2)
    # Compute path
    start_grid=w2g(START)
    goal_grid=w2g(GOAL)
    path=a_star(start_grid,goal_grid)
    # Path line
    path_line_items=[]
    if path:
        pts=[]
        for r,c in path:
            x,y=g2w((r,c))
            pts.extend([x,y])
        if len(pts)>=4:
            path_line_items.append(canvas.create_line(pts,fill="#00aaff",width=3,smooth=True))
    # Agent
    agent_pos=list(w2g(START))
    agent=canvas.create_oval(0,0,0,0,fill="#ffff00",outline="#ffaa00",width=2)
    # Label
    label=canvas.create_text(W//2,20,text="AI Autonomous Navigation — A* Path Planning",fill="#ffffff",font=("Arial",14,"bold"))
    status_text=canvas.create_text(W//2,H-20,text=f"Path: {len(path) if path else 0} waypoints — Navigating...",fill="#aaaaaa",font=("Arial",10))

    def update():
        nonlocal agent_pos
        if path and len(path)>0 and tuple(agent_pos)!=w2g(GOAL):
            agent_pos=list(path.pop(0))
            x,y=g2w(tuple(agent_pos))
            canvas.coords(agent, x-8, y-8, x+8, y+8)
            canvas.itemconfig(status_text, text=f"Step: navigating — remaining waypoints: {len(path)}")
        elif tuple(agent_pos)==w2g(GOAL):
            canvas.itemconfig(status_text, text="SUCCESS: Goal reached!")
        root.after(FPS_DELAY, update)

    update()
    root.mainloop()

if __name__=="__main__":
    run()
