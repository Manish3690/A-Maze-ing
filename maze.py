from pyamaze import maze, agent, COLOR
import random
import time
from collections import deque
from queue import PriorityQueue
import argparse


SEED = 69
random.seed(SEED)


def BFS(m, start=None):
    if start is None:
        start = (m.rows,m.cols)
    frontier = deque()
    frontier.append(start)
    bfsPath = {}
    explored = [start]
    bSearch = []
    while len(frontier) > 0:
        currCell = frontier.popleft()
        if currCell==m._goal:
            break
        for d in 'ESNW':
            if m.maze_map[currCell][d]==True:
                if d=='E':
                    childCell = (currCell[0],currCell[1]+1)
                elif d == 'W':
                    childCell = (currCell[0],currCell[1]-1)
                elif d == 'N':
                    childCell = (currCell[0]-1,currCell[1])
                elif d == 'S':
                    childCell = (currCell[0]+1,currCell[1])
                if childCell in explored:
                    continue
                frontier.append(childCell)
                explored.append(childCell)
                bfsPath[childCell] = currCell
                bSearch.append(childCell)
    fwdPath = {}
    cell=m._goal
    while cell!=(m.rows,m.cols):
        fwdPath[bfsPath[cell]]=cell
        cell=bfsPath[cell]
    return bSearch, bfsPath, fwdPath

def DFS(m,start=None):
    if start is None:
        start = (m.rows,m.cols)
    explored = [start]
    frontier = [start]
    dfsPath = {}
    dSearch = []
    while len(frontier) > 0:
        currCell = frontier.pop()
        dSearch.append(currCell)
        if currCell==m._goal:
            break
        poss = 0
        for d in 'ESNW':
            if m.maze_map[currCell][d]==True:
                if d=='E':
                    childCell = (currCell[0],currCell[1]+1)
                elif d == 'W':
                    childCell = (currCell[0],currCell[1]-1)
                elif d == 'N':
                    childCell = (currCell[0]-1,currCell[1])
                elif d == 'S':
                    childCell = (currCell[0]+1,currCell[1])
                if childCell in explored:
                    continue
                poss += 1
                frontier.append(childCell)
                explored.append(childCell)
                dfsPath[childCell] = currCell
        if poss > 1:
            m.markCells.append(currCell)
    fwdPath = {}
    cell=m._goal
    while cell!=start:
        fwdPath[dfsPath[cell]] = cell
        cell = dfsPath[cell]
    return dSearch, dfsPath, fwdPath

def h(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return (abs(x1-x2)+abs(y1-y2))

def aStar(m, start=None):
    if start is None:
        start = (m.rows,m.cols)
    open = PriorityQueue()
    open.put((h(start,m._goal),h(start,m._goal),start))
    aPath = {}
    g_score = {row:float("inf") for row in m.grid}
    g_score[start] = 0
    f_score = {row:float("inf") for row in m.grid}
    f_score[start] = h(start,m._goal)
    searchPath = [start]
    while not open.empty():
        currCell = open.get()[2]
        searchPath.append(currCell)
        if currCell == m._goal:
            break
        for d in 'ESNW':
            if m.maze_map[currCell][d]==True:
                if d=='E':
                    childCell = (currCell[0],currCell[1]+1)
                elif d == 'W':
                    childCell = (currCell[0],currCell[1]-1)
                elif d == 'N':
                    childCell = (currCell[0]-1,currCell[1])
                elif d == 'S':
                    childCell = (currCell[0]+1,currCell[1])
                temp_g_score = g_score[currCell]+1
                temp_f_score = temp_g_score + h(childCell,m._goal)
                if temp_f_score < f_score[childCell]:
                    aPath[childCell] = currCell
                    g_score[childCell] = temp_g_score
                    f_score[childCell] = temp_g_score + h(childCell, m._goal)
                    open.put((f_score[childCell],h(childCell,m._goal),childCell))
    fwdPath = {}
    cell = m._goal
    while cell != start:
        fwdPath[aPath[cell]] = cell
        cell = aPath[cell]
    return searchPath, aPath, fwdPath


if __name__=="__main__":
    parser = argparse.ArgumentParser(description = "Run the maze solver in BFS/DFS/A* mode")
    parser.add_argument("--mode", type=str, default="interactive", choices = ["interactive","BFS","DFS","Astar"],
                        help="Mode to run the maze solver. Options: interactive, BFS, DFS, Astar")
    parser.add_argument("--rows", type=int, default=5,
                        help="Number of rows")
    parser.add_argument("--cols", type=int, default=5,
                        help="Number of cols")
    parser.add_argument("--delay", type=int, default=50,
                        help="Delay in trace path")
    args = parser.parse_args()
    mode = args.mode
    rows = args.rows
    cols = args.cols
    delay = args.delay
    m = maze(rows,cols)
    m.CreateMaze()
    start = time.time()
    if mode == 'interactive':
        a = agent(m,footprints=True,color=COLOR.cyan)
    elif mode == 'BFS':
        bSearch,bfsPath,fwdPath = BFS(m)
        a = agent(m,footprints=True,color=COLOR.cyan,filled=True)
        b = agent(m,footprints=True,color=COLOR.red,filled=True)
        c = agent(m,1,1,footprints=True,color=COLOR.green,filled=True,goal=(m.rows,m.cols))
        m.tracePath({a:bSearch},delay=delay)
        m.tracePath({c:bfsPath},delay=delay)
        m.tracePath({b:fwdPath},delay=delay)
    elif mode == 'DFS':
        dSearch, dfsPath, fwdPath = DFS(m)
        a = agent(m,footprints=True,color=COLOR.cyan,filled=True)
        b = agent(m,footprints=True,color=COLOR.red,filled=True)
        c = agent(m,1,1,footprints=True,color=COLOR.green,filled=True,goal=(m.rows,m.cols))
        m.tracePath({a:dSearch},delay=delay)
        m.tracePath({c:dfsPath},delay=delay)
        m.tracePath({b:fwdPath},delay=delay)
    elif mode == 'Astar':
        searchPath, aPath, fwdPath = aStar(m)
        a = agent(m,footprints=True,color=COLOR.cyan,filled=True)
        b = agent(m,footprints=True,color=COLOR.red,filled=True)
        c = agent(m,1,1,footprints=True,color=COLOR.green,filled=True,goal=(m.rows,m.cols))
        m.tracePath({a:searchPath},delay=delay)
        m.tracePath({c:aPath},delay=delay)
        m.tracePath({b:fwdPath},delay=delay)
    print(f"It took {mode} {round(time.time()-start,7)} seconds")
    m.run()