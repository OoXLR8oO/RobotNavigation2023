class Cell:
    def __init__(self, x, y, type):
        self.x = x
        self.y = y
        self.type = type

# Note: Cell Types
# 0: Empty
# 1: Wall
# 2: Goals
# 3: Found/Seen
# 4: Final Path (Explored/Tavelled Nodes)
# 7: Start