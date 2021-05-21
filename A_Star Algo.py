import pygame
from queue import PriorityQueue

WINDOW_WIDTH = 800
WINDOW = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


class Node:
    def __init__(self, row, col, width, totalRows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.totalRows = totalRows

    def getNodePos(self):
        return self.row, self.col

    def isClosed(self):
        return self.color == RED

    def isOpen(self):
        return self.color == GREEN

    def isBarrier(self):
        return self.color == BLACK

    def isStart(self):
        return self.color == ORANGE

    def isEnd(self):
        return self.color == TURQUOISE

    def resetNode(self):
        self.color = WHITE

    def makeStart(self):
        self.color = ORANGE

    def makeClosed(self):
        self.color = RED

    def makeOpen(self):
        self.color = GREEN

    def makeBarrier(self):
        self.color = BLACK

    def makeEnd(self):
        self.color = TURQUOISE

    def makePath(self):
        self.color = PURPLE

    def DRAW_NODE(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def updateNeighborsUpgraded(self, grid):
        self.neighbors = []

        # Handles Vertical/Horizontal Movement:
        if self.row < self.totalRows - 1 and not grid[self.row + 1][self.col].isBarrier():  # DOWN
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].isBarrier():  # UP
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.totalRows - 1 and not grid[self.row][self.col + 1].isBarrier():  # RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].isBarrier():  # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

        # Handles Diagonal Movement:
        if self.row < self.totalRows - 1 and self.col < self.totalRows - 1 \
                and not grid[self.row + 1][self.col + 1].isBarrier() \
                and not grid[self.row + 1][self.col].isBarrier() \
                and not grid[self.row][self.col + 1].isBarrier():  # DOWN & RIGHT
            self.neighbors.append(grid[self.row + 1][self.col + 1])

        if self.row < self.totalRows - 1 and self.col > 0 \
                and not grid[self.row + 1][self.col - 1].isBarrier() \
                and not grid[self.row + 1][self.col].isBarrier() \
                and not grid[self.row][self.col - 1].isBarrier():  # DOWN & LEFT
            self.neighbors.append(grid[self.row + 1][self.col - 1])

        if self.row > 0 and self.col < self.totalRows - 1 \
                and not grid[self.row - 1][self.col + 1].isBarrier() \
                and not grid[self.row - 1][self.col].isBarrier() \
                and not grid[self.row][self.col + 1].isBarrier():  # UP & RIGHT
            self.neighbors.append(grid[self.row - 1][self.col + 1])

        if self.row > 0 and self.col > 0 \
                and not grid[self.row - 1][self.col - 1].isBarrier() \
                and not grid[self.row - 1][self.col].isBarrier() \
                and not grid[self.row][self.col - 1].isBarrier():  # UP & LEFT
            self.neighbors.append(grid[self.row - 1][self.col - 1])



def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    c = (((x1 - x2)**2) + ((y1 - y2)**2)) ** (1/2)
    return c


def reconstruct_path(came_from, start, current, draw):
    while current in came_from:
        current = came_from[current]
        if current != start:
            current.makePath()
        draw()


def algorithmUpgraded(draw, grid, start, end):
    count = 0
    openSet = PriorityQueue()  # Initialized "openSet" as the priority queue
    openSet.put((0, count, start))  # Puts the start Node into the priority queue
    came_from = {}  # Keeps track of what nodes came from where (Used for path construction)

    g_score = {spot: float("inf") for row in grid for spot in row}  # Keeps track of shortest distance from the start
                                                                    # node to the current node
    g_score[start] = 0  # Initialized to '0' since we start at the start node

    f_score = {spot: float("inf") for row in grid for spot in row}  # Keeps track of the estimated shortest distance
                                                                    # to the end node
    f_score[start] = h(start.getNodePos(), end.getNodePos())  # Initialized to the distance between the start and end node

    open_set_hash = {start}  # This set is used to keep track if a node is in the "openSet" priority queue.
                             # We use a set to keep track of this since there is no explicit way to check the
                             # priority queue for a specific element...

    while not openSet.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = openSet.get()[2]  # Sets current to node popped from priority queue
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, start, end, draw)
            end.makeEnd()
            return True

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1  # Add '1' to account for the move to the neighbor

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current  # Update the 'came_from' tracker to the current Node
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.getNodePos(), end.getNodePos())

                if neighbor not in open_set_hash:
                    count += 1
                    openSet.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.makeOpen()

        draw()

        if current != start:
            current.makeClosed()

    return False


def makeGrid(rows, windowWidth):
    grid = []
    gap = windowWidth // rows
    for row in range(rows):

        grid.append([])
        for col in range(rows):
            spot = Node(row, col, gap, rows)
            grid[row].append(spot)

    return grid


def drawGrid(window, rows, windowWidth):
    gap = windowWidth // rows
    for i in range(rows):
        pygame.draw.line(window, GREY, (0, i * gap), (windowWidth, i * gap))
        for j in range(rows):
            pygame.draw.line(window, GREY, (j * gap, 0), (j * gap, windowWidth))


def draw(window, grid, rows, windowWidth):
    window.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.DRAW_NODE(window)

    drawGrid(window, rows, windowWidth)
    pygame.display.update()


def getClickedPos(pos, rows, windowWidth):
    gap = windowWidth // rows
    y, x = pos

    row = y // gap
    col = x // gap

    return row, col


def RunAStar(window, windowWidth):
    ROWS = 40
    grid = makeGrid(ROWS, windowWidth)

    start = None
    end = None

    run = True
    while run:
        draw(window, grid, ROWS, windowWidth)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:  # LEFT
                clickPos = pygame.mouse.get_pos()
                row, col = getClickedPos(clickPos, ROWS, windowWidth)
                node = grid[row][col]

                if not start and node != end:
                    start = node
                    start.makeStart()

                elif not end and node != start:
                    end = node
                    end.makeEnd()

                elif node != end and node != start:
                    node.makeBarrier()

            elif pygame.mouse.get_pressed()[2]:  # RIGHT
                clickPos = pygame.mouse.get_pos()
                row, col = getClickedPos(clickPos, ROWS, windowWidth)
                node = grid[row][col]
                node.resetNode()

                if node == start:
                    start = None
                elif node == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for node in row:
                            node.updateNeighborsUpgraded(grid)

                    algorithmUpgraded(lambda: draw(window, grid, ROWS, windowWidth), grid, start, end)

                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = makeGrid(ROWS, windowWidth)
    pygame.quit()


RunAStar(WINDOW, WINDOW_WIDTH)
