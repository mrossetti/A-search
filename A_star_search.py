import numpy as np
from itertools import product
import pygame
import ctypes



class Node:
    """Some operator overloading was done mainly to work with sets and built-in
       functions / operators simplfying syntax and readability."""
    def __init__(self, x, y, value):
        self.x = x  # node column (j)
        self.y = y  # node row (i)
        self.value = value  # value stored by the node
        self.parent = None  # parent node connected
        self.g = 0.0  # actual distance from start to current node
        self.h = 0.0  # heuristic distance from current node to goal

    def __getitem__(self, idx):
        return (self.x, self.y)[idx]

    def __hash__(self):
        return hash(tuple(self))

    def __eq__(self, other):
        if hasattr(other, '__getitem__'):
            if (self.x == other[0] and self.y == other[1]):
                return True
        return False

    def __repr__(self):
        return 'Node({})'.format(self.value)


class A_star:
    """
    steps = actual_steps(start to current) + estimated_steps(current to goal)
    f = g + h

    The best path is the one from start to goal with as few steps as possible
    A* path = argmin f = argmin (g + h)

    A* search prioritizes the node with the lowest g + h score. The following
    implementation uses the manhattan distance to calculate the heuristic h.
    """
    def __init__(self, array, wall=True):
        # parse the array and build an array of node objects
        rows, cols = array.shape
        node = np.empty((rows, cols), dtype=object)
        for x, y in product(range(cols), range(rows)):
            node[y, x] = Node(x, y, array[y, x])
        self.node = node  # array of nodes
        self.wall = wall  # wall identifier

    @staticmethod
    def manhattan(node, goal):
        # l1 norm
        x1, y1 = node
        x2, y2 = goal
        return abs(y2-y1) + abs(x2-x1)

    @staticmethod
    def euclidean(node, goal):
        # l2 norm
        x1, y1 = node
        x2, y2 = goal
        return ((y2-y1)**2 + (x2-x1)**2)**0.5

    def neighbors(self, node):
        dirs = []
        h, w = [dim-1 for dim in self.node.shape]  # zeroed index
        if node.x > 0: dirs.append((node.x - 1, node.y))  # east
        if node.x < w: dirs.append((node.x + 1, node.y))  # west
        if node.y > 0: dirs.append((node.x, node.y - 1))  # north
        if node.y < h: dirs.append((node.x, node.y + 1))  # south
        nodes = [self.node[y,x] for x,y in dirs if \
                 self.node[y,x].value != self.wall]
        return nodes

    def search(self, start, goal, h='manhattan'):
        # heuristic meausre of remaining dist to the goal
        h = getattr(self, h)
        # openset: to explore, closedset: no further exploration
        openset, closedset = set(), set()
        # exploration begins at start node
        openset.add(self.node[start[1], start[0]])
        # safety break
        iteration = 0
        max_iteration = np.prod(self.node.shape)
        # A*
        while openset:
            # interrupt after max_iteration
            iteration += 1
            if iteration > max_iteration:
                print('No path found')
                break
            # which node is explored first? the one that minimizes g + h
            current = min(openset, key=lambda node: node.g + node.h)
            # if goal reached: A* found a solution
            if current == goal:
                path = [current]
                while current.parent:
                    path.append(current.parent)
                    current = current.parent
                return list(reversed(path))
            # current node has been explored
            openset.remove(current)
            closedset.add(current)
            # keep exploring current node's neighbors
            for node in self.neighbors(current):
                # node_g = current_g + distance(from=current, to=node)
                node_g = current.g + 1
                # already expanded -> skip
                if node in closedset:
                    continue
                # already visited from other paths -> keep node with smaller g
                if node in openset:
                    if node_g < node.g:
                        # update this node (instead of recreating a node object)
                        node.parent = current  # current path better
                        node.g = node_g  # update g value
                # never visited -> add to openset
                else:
                    node.parent = current
                    node.g = node_g
                    node.h = h(node, goal)
                    openset.add(node)
        # no path from start to goal found


class Graphics:
    def __new__(cls, *args, **kwargs):
        # before instantiation
        pygame.init()
        ctypes.windll.user32.SetProcessDPIAware()
        return super(Graphics, cls).__new__(cls, *args, **kwargs)

    def __init__(self, **config):
        # config
        self.wsize = config.get('window_size', (640, 640))
        self.bgcol = config.get('bg_color', (255,255,255))
        self.fps   = config.get('fps', 16)
        self.wait  = config.get('wait_between_updates', 0.0)
        # setup
        self.display = pygame.display.set_mode(self.wsize)
        
    def run(self, src):
        # run graphics from source 'src'
        # src must have .update() and .draw() methods
        clock = pygame.time.Clock()
        pause = 0.0
        running = True

        while running:
            self.display.fill(self.bgcol)
            src.draw(self.display)
            if pause > self.wait:  # wait between upds
                src.update()
                pause = 0.0
            pygame.display.update()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    running = False
            ms_elapsed = clock.tick(self.fps)
            pause += ms_elapsed


class Visualizer:
    colors = {'empty': (255, 255, 255),  # white
              'wall' : (  0,   0,   0),  # black
              'start': (255, 255,   0),  # red
              'goal' : (255,   0,   0),  # purple
              'grid' : (127, 127, 127)}  # gray

    def __init__(self, array, wall=True):
        self.array = array
        self.wall = wall
        self.A_star = A_star(array, wall)

    def A_star_search(self, start, goal):
        self.start, self.goal = start, goal
        self.path = self.A_star.search(start, goal)

    @staticmethod
    def print_array(array, start, goal, wall=True):
        res = np.full(array.shape, '.')
        res[array == wall] = '#'
        res[start[1], start[0]] = 'S'
        res[goal[1], goal[0]] = 'G'
        for i in range(array.shape[0]):
            for j in range(array.shape[1]):
                print(res[i, j], end=' ')
            print()  # \n

    def draw(self, dst):
        # draw on destination surface 'dst'
        rows, cols = self.array.shape
        width, height = dst.get_size()
        cw, ch = width//cols, height//rows
        # cell coloring
        for i, j in product(range(rows), range(cols)):
            key = 'wall' if self.array[i, j] == self.wall else 'empty'
            pygame.draw.rect(dst, self.colors[key], (j*cw, i*ch, cw, ch))
        # cell 'start'
        i, j = self.start
        pygame.draw.rect(dst, self.colors['start'], (j*cw, i*ch, cw, ch))
        # cell 'goal'
        i, j = self.goal
        pygame.draw.rect(dst, self.colors['goal'], (j*cw, i*ch, cw, ch))
        # A* path
        T = len(self.path)
        start_r, start_g, start_b = self.colors['start']
        final_r, final_g, final_b = self.colors['goal']
        # lerp color to obtain gradient
        for t, (i, j) in enumerate(self.path[1:-1]):
            color = (int(start_r + t/T * (final_r - start_r)),
                     int(start_g + t/T * (final_g - start_g)),
                     int(start_b + t/T * (final_b - start_b)))
            pygame.draw.rect(dst, color, (j*cw, i*ch, cw, ch))
        # grid lines
        for x in range(0, width, cw):
             pygame.draw.line(dst, self.colors['grid'], (x, 0), (x, height))
        for y in range(0, height, ch):
            pygame.draw.line(dst, self.colors['grid'], (0, y), (width, y))

    def update(self):
        # Here you can recalculate the search with different 'start' and 'goal'
        pass



def main():
    # inputs
    shape = (32, 32)
    start = (2, 2)
    goal = (29, 26)
    wall = True
    slices = [np.s_[ 8:10, 5:20],  # rows ( 8:10), cols (5:20)
              np.s_[10:20, 5: 7]]  # rows (10:20), cols (5: 7)
    # array
    array = np.zeros(shape)
    for slice in slices:
        array[slice].fill(wall)
    # cli
    Visualizer.print_array(array, start, goal, wall)
    # gui
    gui = Graphics()
    src = Visualizer(array, wall)
    src.A_star_search(start, goal)
    gui.run(src)



if __name__ == '__main__':
    main()
