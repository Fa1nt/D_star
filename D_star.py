from tkinter import *
from tkinter import Tk, Canvas, Frame, BOTH, messagebox
from tkinter import ttk

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.h = 0
        self.k = 0
        self.parent = None
        self.state = "_"
        self.t = "new"
        
    def set_state(self, state):   
        # # obstacle
        # _ new
        # * closed state
        # c current state
        # p parent of the current state
        states = ["#", "_", "*", "c", "p"]
        if state not in states:
            return
        self.state = state

    def cost(self, state):
        if self.state == "#" or state.state == "#":
            return 1000000000000000000
        return ((self.x - state.x)**2 + (self.y - state.y)**2)**0.5

class Graph:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.graph = self.create_graph()

    def create_graph(self):
        final_list = []
        for i in range(self.row):
            x_list = []
            for j in range(self.col):
                x_list.append(Node(i, j))
            final_list.append(x_list)
        return final_list
    
    def add_obstacles(self, coord_list):
        for x, y in coord_list:
            if x >= 0 and x < self.row and y >= 0 and y < self.col:
                self.graph[x][y].set_state("#")

    def get_neighbors(self, state):
        states = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if state.x + i >= 0 and state.x + i < self.row:
                    if state.y + j >= 0 and state.y + j < self.col:
                        if i != 0 or j != 0:
                            states.append(self.graph[state.x + i][state.y + j])
        return states

global_path = []

class D_star:
    def __init__(self, graph):
        self.graph = graph
        self.open_list = set()
        
    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "closed":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def delete(self, state):
        if state.t == "open":
            state.t = "closed"
        self.open_list.remove(state)

    # changes the cost and inserts to the open list
    def modify_cost(self, x):
        if x.t == "closed":
            self.insert(x, x.parent.h + x.cost(x.parent))
    
    # computes the optimal pathcosts to the goal
    def process_state(self):
        x = self.min_state()
        if x == None:
            return -1
        k_old = self.get_kmin()
        self.delete(x)
        if k_old < x.h:
            for y in self.graph.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        elif k_old == x.h:
            for y in self.graph.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.graph.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(y, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) and y.t == "closed" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def run(self, start, goal):
        x = []
        y = []

        self.open_list.add(goal)

        while True:
            self.process_state()
            if start.t == "closed":
                break

        start.set_state("c")
        s = start
        s = s.parent
        s.set_state("p")
        current = start
        
        while current != goal:
            global_path.clear()
            current.set_state("*")
            x.append(current.x)
            y.append(current.y)
            [global_path.append((x[i], y[i])) for i in range(len(y))]
            if current.parent.state == "#":
                self.modify_cost(current)
                while True:
                    k_min = self.process_state()
                    if k_min >= current.h:
                        break
            else:
                current = current.parent
        current.set_state("p")
        global_path.append((goal.x, goal.y))
        #print(global_path)
        return x, y

def main():
    # GUI
    raam = Tk()
    raam.title("D* Visualization")
    raam.resizable(False, False)
    raam.geometry("800x800")
    
    def clear_canvas():
        canvas.delete("rect")
        obstacles.clear()
        canvas.create_line(2, 2, 602, 2, fill='grey')
        canvas.create_line(0, 600, 600, 600, fill='grey')
        for i in range(30):
            canvas.create_line(2, 20*i, 602, 20*i, fill='grey')
            canvas.create_line(20*i, 2, 20*i, 602, fill='grey')
        canvas.create_line(2, 2, 2, 602, fill='grey')
        canvas.create_line(600, 0, 600, 600, fill='grey')
        canvas.coords(npc, 5, 585, 15, 595)
        canvas.coords(shadow, 0, 0, 0, 0)
        trail.clear()

    canvas = Canvas(raam, width=600, height=600, background="white")
    canvas.place(x=10, y=10)
    
    # create a list of obstacles to be added
    obstacles = []
    
    # trail that the pathfinder leaves behind
    shadow = canvas.create_rectangle(0, 0, 0, 0, fill = "yellow")
    trail = []
    # goal
    goal = canvas.create_rectangle(580, 0, 600, 20, fill = "green")
    # create the pathfinder, set its location
    npc = canvas.create_rectangle(5, 585, 15, 595, fill = "red")
    #print([int((canvas.coords(npc)[0]-5)/20), int((canvas.coords(npc)[1]-5)/20)])
    
    def run_d_star():
        graph = Graph(30, 30)
        graph.add_obstacles(obstacles)
        start = graph.graph[int((canvas.coords(npc)[0]-5)/20)][int((canvas.coords(npc)[1]-5)/20)]
        finish = graph.graph[29][0]
        dstar = D_star(graph)
        dstar.run(start, finish)
    
    # create a grid
    # offset = +2
    canvas.create_line(2, 2, 602, 2, fill='grey')
    canvas.create_line(0, 600, 600, 600, fill='grey')
    for i in range(30):
        canvas.create_line(2, 20*i, 602, 20*i, fill='grey')
        canvas.create_line(20*i, 2, 20*i, 602, fill='grey')
    canvas.create_line(2, 2, 2, 602, fill='grey')
    canvas.create_line(600, 0, 600, 600, fill='grey')

    # move the pathfinder and leave a trail underneath and behind
    def move_npc():
        if len(global_path) > 0:
            if len(trail) > 0:
                canvas.create_rectangle(trail[-1][0]*20, trail[-1][1]*20, trail[-1][0]*20+20, trail[-1][1]*20+20, fill = "yellow", tags="rect")
            canvas.coords(shadow, global_path[0][0]*20, global_path[0][1]*20, global_path[0][0]*20+20, global_path[0][1]*20+20)
            canvas.coords(npc, global_path[0][0]*20+5, global_path[0][1]*20+5, global_path[0][0]*20+15, global_path[0][1]*20+15)
            trail.append(global_path[0])
            global_path.pop(0)
        #print([int((canvas.coords(npc)[0]-5)/20), int((canvas.coords(npc)[1]-5)/20)])
    
    b1 = ttk.Button(raam, text ="Next", command = lambda : move_npc())
    b1.place(x=675, y=20,height=25, width=80)
    b2 = ttk.Button(raam, text ="Reset", command = lambda : clear_canvas())
    b2.place(x=675, y=100,height=25, width=80)
    b3 = ttk.Button(raam, text ="D*", command = lambda : run_d_star())
    b3.place(x=675, y=180,height=25, width=80)
    
    # create obstacles
    def create_obstacle(event):
        x = event.x - event.x%20
        y = event.y - event.y%20
        if (x//20, y//20) not in obstacles:
            canvas.create_rectangle(x, y, x+20, y+20, fill = "black", tags="rect")
            obstacles.append((x//20, y//20))
        
    canvas.bind("<Button-1>", create_obstacle)
    
    raam.mainloop()

if __name__ == '__main__':
    main()
