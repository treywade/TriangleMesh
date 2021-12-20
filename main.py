# # Triangle strips
# #
# # Usage: python main.py file_of_triangles
# #
# # You can press ESC in the window to exit.
# #
# # You'll need Python 3 and must install these packages:
# #
# #   PyOpenGL, GLFW
# import copy
# import sys, os, math
#
# try:  # PyOpenGL
#     from OpenGL.GL import *
# except:
#     print('Error: PyOpenGL has not been installed.')
#     sys.exit(0)
#
# try:  # GLFW
#     import glfw
# except:
#     print('Error: GLFW has not been installed.')
#     sys.exit(0)
#
# # Globals
#
# window = None
#
# windowWidth = 1000  # window dimensions
# windowHeight = 1000
#
# minX = None  # range of vertices
# maxX = None
# minY = None
# maxY = None
#
# r = 0.008  # point radius as fraction of window size
#
# allVerts = []  # all triangle vertices
#
# lastKey = None  # last key pressed
#
# showForwardLinks = True
#
#
# # Triangle
# #
# # A Triangle stores its three vertices and pointers to any adjacent triangles.
# #
# # For debugging, you can set the 'highlight1' and 'highlight2' flags
# # of a triangle.  This will cause the triangle to be highlighted when
# # it's drawn.
#
#
# class Triangle(object):
#     nextID = 0
#
#     def __init__(self, verts):
#
#         self.verts = verts  # 3 vertices.  Each is an index into the 'allVerts' global.
#         self.adjTris = []  # adjacent triangles
#
#         self.nextTri = None  # next triangle on strip
#         self.prevTri = None  # previous triangle on strip
#
#         self.highlight1 = False  # to cause drawing to highlight this triangle in colour 1
#         self.highlight2 = False  # to cause drawing to highlight this triangle in colour 2
#
#         self.centroid = (sum([allVerts[i][0] for i in self.verts]) / len(self.verts),
#                          sum([allVerts[i][1] for i in self.verts]) / len(self.verts))
#
#         self.id = Triangle.nextID
#         Triangle.nextID += 1
#
#         self.stripFlag = 0
#
#     # String representation of this triangle
#
#     def __repr__(self):
#         return 'tri-%d' % self.id
#
#     # Draw this triangle
#
#     def draw(self):
#
#         # Highlight with yellow fill
#
#         if self.highlight1 or self.highlight2:
#
#             if self.highlight1:
#                 glColor3f(0.9, 0.9, 0.4)  # dark yellow
#             else:
#                 glColor3f(1, 1, 0.8)  # light yellow
#
#             glBegin(GL_POLYGON)
#             for i in self.verts:
#                 glVertex2f(allVerts[i][0], allVerts[i][1])
#             glEnd()
#
#         # Outline the triangle
#
#         glColor3f(0, 0, 0)
#         glBegin(GL_LINE_LOOP)
#         for i in self.verts:
#             glVertex2f(allVerts[i][0], allVerts[i][1])
#         glEnd()
#
#     # Draw edges to next and previous triangle on the strip
#
#     def drawPointers(self):
#
#         if not showForwardLinks and self.nextTri:
#             glColor3f(0, 0, 1)
#             drawArrow(self.centroid[0], self.centroid[1],
#                       self.nextTri.centroid[0], self.nextTri.centroid[1])
#
#         if showForwardLinks and self.prevTri:
#             glColor3f(1, 0, 0)
#             drawArrow(self.centroid[0], self.centroid[1],
#                       self.prevTri.centroid[0], self.prevTri.centroid[1])
#
#         if not self.nextTri and not self.prevTri:  # no links.  Draw a dot.
#             if showForwardLinks:
#                 glColor3f(0, 0, 1)
#             else:
#                 glColor3f(1, 0, 0)
#             glBegin(GL_POLYGON)
#             for i in range(100):
#                 theta = 3.14159 * i / 50.0
#                 glVertex2f(self.centroid[0] + 0.5 * r * math.cos(theta), self.centroid[1] + 0.5 * r * math.sin(theta))
#             glEnd()
#
#     # Determine whether this triangle contains a point
#
#     def containsPoint(self, pt):
#
#         return (turn(allVerts[self.verts[0]], allVerts[self.verts[1]], pt) == LEFT_TURN and
#                 turn(allVerts[self.verts[1]], allVerts[self.verts[2]], pt) == LEFT_TURN and
#                 turn(allVerts[self.verts[2]], allVerts[self.verts[0]], pt) == LEFT_TURN)
#
#
# # Draw an arrow between two points.
#
# def drawArrow(x0, y0, x1, y1):
#     d = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
#
#     vx = (x1 - x0) / d  # unit direction (x0,y0) -> (x1,y1)
#     vy = (y1 - y0) / d
#
#     vpx = -vy  # unit direction perpendicular to (vx,vy)
#     vpy = vx
#
#     xa = x0 + 0.15 * r * vx  # arrow tail
#     ya = y0 + 0.15 * r * vy
#
#     xb = x1 - 0.15 * r * vx  # arrow head
#     yb = y1 - 0.15 * r * vy
#
#     xc = xb - 2 * r * vx + 0.5 * r * vpx  # arrow outside left
#     yc = yb - 2 * r * vy + 0.5 * r * vpy
#
#     xd = xb - 2 * r * vx - 0.5 * r * vpx  # arrow outside right
#     yd = yb - 2 * r * vy - 0.5 * r * vpy
#
#     glBegin(GL_LINES)
#     glVertex2f(xa, ya)
#     glVertex2f(0.5 * (xc + xd), 0.5 * (yc + yd))
#     glEnd()
#
#     glBegin(GL_LINE_LOOP)
#     glVertex2f(xb, yb)
#     glVertex2f(xc, yc)
#     glVertex2f(xd, yd)
#     glEnd()
#
#
# # Determine whether three points make a left or right turn
#
# LEFT_TURN = 1
# RIGHT_TURN = 2
# COLLINEAR = 3
#
#
# def turn(a, b, c):
#     det = (a[0] - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (a[1] - c[1])
#
#     if det > 0:
#         return LEFT_TURN
#     elif det < 0:
#         return RIGHT_TURN
#     else:
#         return COLLINEAR
#
# def checkAdj(triangle):
#     #initialize the flag count to -1, -1, -1 each time trying to find how many adjacent triangles
#     flagCount = [-1,-1,-1]
#     #look through all of the adjacent triangles
#     for i in range(0, len(triangle.adjTris)):
#         if len(triangle.adjTris) > 1:
#             #check if the triangles have already been visited
#             flagCount[i] = availableAdj(triangle.adjTris[i])
#         else:
#             #no adjacent triangles
#             return 0;
#     #if there are no available adjacent triangles then set value to 4 so it wont ever be minimum
#     for i in range(0, len(flagCount)):
#         if (flagCount[i] == -1):
#             flagCount[i] = 4
#     leastAdjID = flagCount.index(min(flagCount))
#     triangle.adjTris[leastAdjID].prevTri = triangle
#     #return the index which you find the least amount of available triangles
#     return leastAdjID
#
#
# def availableAdj(triangle):
#     count = 0
#     #check if triangle has yet to be visited
#     if triangle.stripFlag == 0:
#         #scan through adjacent triangles finding unvisited adjacent triangles
#         for i in range(0, len(triangle.adjTris)):
#             if triangle.adjTris[i].stripFlag == 0:
#                 #increment count if found
#                 count = count + 1
#     if count == 0:
#         return -1
#     return count
#
#
#
# # ================================================================
# # ================================================================
# # ================================================================
#
# # Build a set of triangle strips that cover all of the given
# # triangles.  The goal is to make the strips as long as possible
# # (i.e. to have the fewest strip that cover all triangles).
# #
# # Follow the instructions in A2.txt.
# #
# # This function does not return anything.  The strips are formed by
# # modifying the 'nextTri' and 'prevTri' pointers in each triangle.
#
#
# def buildTristrips(triangles):
#     count = 0 # number of strips
#     leastAvailableId = triangles[0]         #keeps count of how many triangles in adjacent triangles have not been added to a strip
#     tri=set(triangles)                      #puts list into set for efficiency
#     exitLoop = 0                            #flag to stop building strip once end is reached
#     triAdded = 0                            #how many triangles have been added to strips in total, used to stop program
#     while triAdded < len(triangles):        # main loop to generate strips
#         availableAdj = 3                    #sets to highest possible number
#         for t in tri:                       #iterate through every triangle, if it has been added to strip, skip it
#             tempAvailableAdj = 0
#             if t.stripFlag == 1:
#                 continue
#             for j in range(0, len(t.adjTris)):              #once a triangle that is not in a strip is found, iterate through all adjacent triangles
#                 if t.adjTris[j].stripFlag == 0:             #if the adjacent triangle has not been added to a strip, increase counter
#                     tempAvailableAdj = tempAvailableAdj + 1
#             if (tempAvailableAdj < availableAdj):           #pick lower of the two
#                 availableAdj = tempAvailableAdj             #update counter
#                 leastAvailableId = t                        #set lowest to current triangle
#
#         currTriangle = leastAvailableId                     #set current and prev triangles
#         prevTriangle = leastAvailableId
#         while exitLoop == 0:  #add triangles to strip if pointers to prev and next dont exist
#
#             x = checkAdj(currTriangle)
#             currTriangle.adjTris[x].prevTri = prevTriangle
#             currTriangle.nextTri = currTriangle.adjTris[x]
#             if ((currTriangle.adjTris[x].nextTri is not None) and (currTriangle.adjTris[x].prevTri is not None)):  # if no more triangles in adjacent array are available
#                 exitLoop = 1                                                                        #exitflag
#                 currTriangle.adjTris[x].prevTri = prevTriangle
#                 currTriangle.nextTri = currTriangle.adjTris[x]
#             else : #check if current triangle has a adjacent member not in the strip
#                 currTriangle.stripFlag = 1
#                 currTriangle.nextTri = currTriangle.adjTris[x]                      #the next triangle pointer is set to the newly found adjacent triangle
#                 currTriangle.prevTri = prevTriangle                                 #set current triangle prev pointer to previously found triangle
#                 prevTriangle = currTriangle                                         #previous triangle is current triangle
#                 currTriangle.adjTris[x].prevTri = prevTriangle
#                 tri.remove(currTriangle)
#                 currTriangle = currTriangle.nextTri                                 #current triangle moves to triangle pointed to by next
#                 triAdded = triAdded + 1
#         exitLoop = 0                                                                    #reset exit vairable
#         if currTriangle.stripFlag == 0:
#             currTriangle.stripFlag = 1
#             currTriangle.adjTris[x].prevTri = currTriangle
#             currTriangle.nextTri = None
#         count = count + 1
#         triAdded = triAdded + 1
#
#     print('Generated %d tristrips' % count)
#
#
# # ================================================================
# # ================================================================
# # ================================================================
#
#
# # Set up the display and draw the current image
#
# windowLeft = None
# windowRight = None
# windowTop = None
# windowBottom = None
#
#
# def display(wait=False):
#     global lastKey, windowLeft, windowRight, windowBottom, windowTop
#
#     # Handle any events that have occurred
#
#     glfw.poll_events()
#
#     # Set up window
#
#     glClearColor(1, 1, 1, 0)
#     glClear(GL_COLOR_BUFFER_BIT)
#     glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
#
#     glMatrixMode(GL_PROJECTION)
#     glLoadIdentity()
#
#     glMatrixMode(GL_MODELVIEW)
#     glLoadIdentity()
#
#     if maxX - minX > maxY - minY:  # wider point spread in x direction
#         windowLeft = -0.1 * (maxX - minX) + minX
#         windowRight = 1.1 * (maxX - minX) + minX
#         windowBottom = windowLeft
#         windowTop = windowRight
#     else:  # wider point spread in y direction
#         windowTop = -0.1 * (maxY - minY) + minY
#         windowBottom = 1.1 * (maxY - minY) + minY
#         windowLeft = windowBottom
#         windowRight = windowTop
#
#     glOrtho(windowLeft, windowRight, windowBottom, windowTop, 0, 1)
#
#     # Draw triangles
#
#     for tri in allTriangles:
#         tri.draw()
#
#     # Draw pointers.  Do this *after* the triangles (above) so that the
#     # triangle drawing doesn't overlay the pointers.
#
#     for tri in allTriangles:
#         tri.drawPointers()
#
#     # Show window
#
#     glfw.swap_buffers(window)
#
#     # Maybe wait until the user presses 'p' to proceed
#
#     if wait:
#
#         sys.stderr.write('Press "p" to proceed ')
#         sys.stderr.flush()
#
#         lastKey = None
#         while lastKey != 80:  # wait for 'p'
#             glfw.wait_events()
#             display()
#
#         sys.stderr.write('\r                     \r')
#         sys.stderr.flush()
#
#
# # Handle keyboard input
#
# def keyCallback(window, key, scancode, action, mods):
#     global lastKey, showForwardLinks
#
#     if action == glfw.PRESS:
#
#         if key == glfw.KEY_ESCAPE:  # quit upon ESC
#             sys.exit(0)
#         elif key == ord('F'):  # toggle forward/backward link display
#             showForwardLinks = not showForwardLinks
#         else:
#             lastKey = key
#
#
# # Handle window reshape
#
#
# def windowReshapeCallback(window, newWidth, newHeight):
#     global windowWidth, windowHeight
#
#     windowWidth = newWidth
#     windowHeight = newHeight
#
#
# # Handle mouse click/release
#
# def mouseButtonCallback(window, btn, action, keyModifiers):
#     if action == glfw.PRESS:
#
#         # Find point under mouse
#
#         x, y = glfw.get_cursor_pos(window)  # mouse position
#
#         wx = (x - 0) / float(windowWidth) * (windowRight - windowLeft) + windowLeft
#         wy = (windowHeight - y) / float(windowHeight) * (windowTop - windowBottom) + windowBottom
#
#         selectedTri = None
#         for tri in allTriangles:
#             if tri.containsPoint([wx, wy]):
#                 selectedTri = tri
#                 break
#
#         # print triangle, toggle its highlight1, and toggle the highlight2s of its adjacent triangles
#
#         if selectedTri:
#             selectedTri.highlight1 = not selectedTri.highlight1
#             print('%s with adjacent %s' % (selectedTri, repr(selectedTri.adjTris)))
#             for t in selectedTri.adjTris:
#                 t.highlight2 = not t.highlight2
#
#
# # Read triangles from a file
#
# def readTriangles(f):
#     global allVerts
#
#     errorsFound = False
#
#     lines = f.readlines()
#
#     # Read the vertices
#
#     numVerts = int(lines[0])
#     allVerts = [[float(c) for c in line.split()] for line in lines[1:numVerts + 1]]
#
#     # Check that the vertices are valid
#
#     for l, v in enumerate(allVerts):
#         if len(v) != 2:
#             print('Line %d: vertex does not have two coordinates.' % (l + 2))
#             errorsFound = True
#
#     # Read the triangles
#
#     numTris = int(lines[numVerts + 1])
#     triVerts = [[int(v) for v in line.split()] for line in lines[numVerts + 2:]]
#
#     # Check that the triangle vertices are valid
#
#     for l, tvs in enumerate(triVerts):
#         if len(tvs) != 3:
#             print('Line %d: triangle does not have three vertices.' % (l + 2 + numVerts))
#             errorsFound = True
#         else:
#             for v in tvs:
#                 if v < 0 or v >= numVerts:
#                     print('Line %d: Vertex index is not in range [0,%d].' % (l + 2 + numVerts, numVerts - 1))
#                     errorsFound = True
#
#     # Build triangles
#
#     tris = []
#
#     for tvs in triVerts:
#         theseVerts = tvs
#         if turn(allVerts[tvs[0]], allVerts[tvs[1]], allVerts[tvs[2]]) != COLLINEAR:
#             tris.append(Triangle(tvs))  # (don't include degenerate triangles)
#
#     # For each triangle, find and record its adjacent triangles
#     #
#     # This would normally take O(n^2) time if done by brute force, so
#     # we'll exploit Python's hashed dictionary keys.
#
#     if False:
#
#         for tri in tris:  # brute force
#             adjTris = []
#             for i in range(3):
#                 v0 = tri.verts[i % 3]
#                 v1 = tri.verts[(i + 1) % 3]
#                 for tri2 in tris:
#                     for j in range(3):
#                         if v1 == tri2.verts[j % 3] and v0 == tri2.verts[(j + 1) % 3]:
#                             adjTris.append(tri2)
#                     if len(adjTris) == 3:
#                         break
#             tri.adjTris = adjTris
#
#     else:  # hashing
#
#         edges = {}
#
#         for tri in tris:
#             for i in range(3):
#                 v0 = tri.verts[i % 3]
#                 v1 = tri.verts[(i + 1) % 3]
#                 key = '%f-%f' % (v0, v1)
#                 edges[key] = tri
#
#         for tri in tris:
#             adjTris = []
#             for i in range(3):
#                 v1 = tri.verts[i % 3]  # find a reversed edge of an adjacent triangle
#                 v0 = tri.verts[(i + 1) % 3]
#                 key = '%f-%f' % (v0, v1)
#                 if key in edges:
#                     adjTris.append(edges[key])
#                 if len(adjTris) == 3:
#                     break
#             tri.adjTris = adjTris
#
#     print('Read %d points and %d triangles' % (numVerts, numTris))
#
#     if errorsFound:
#         return []
#     else:
#         return tris
#
#
# # Initialize GLFW and run the main event loop
#
# def main():
#     global window, allTriangles, minX, maxX, minY, maxY, r
#
#     # Check command-line args
#
#     if len(sys.argv) < 2:
#         print('Usage: %s filename' % sys.argv[0])
#         sys.exit(1)
#
#     args = sys.argv[1:]
#     while len(args) > 1:
#         # if args[0] == '-x':
#         #     pass
#         args = args[1:]
#
#     # Set up window
#
#     if not glfw.init():
#         print('Error: GLFW failed to initialize')
#         sys.exit(1)
#
#     window = glfw.create_window(windowWidth, windowHeight, "Assignment 2", None, None)
#
#     if not window:
#         glfw.terminate()
#         print('Error: GLFW failed to create a window')
#         sys.exit(1)
#
#     glfw.make_context_current(window)
#     glfw.swap_interval(1)
#     glfw.set_key_callback(window, keyCallback)
#     glfw.set_window_size_callback(window, windowReshapeCallback)
#     glfw.set_mouse_button_callback(window, mouseButtonCallback)
#
#     # Read the triangles.  This also fills in the global 'allVerts'.
#
#     with open(args[0], 'rb') as f:
#         allTriangles = readTriangles(f)
#
#     if allTriangles == []:
#         return
#
#     # Get bounding box of points
#
#     minX = min(p[0] for p in allVerts)
#     maxX = max(p[0] for p in allVerts)
#     minY = min(p[1] for p in allVerts)
#     maxY = max(p[1] for p in allVerts)
#
#     # Adjust point radius in proportion to bounding box
#
#     if maxX - minX > maxY - minY:
#         r *= maxX - minX
#     else:
#         r *= maxY - minY
#
#     # Run the code
#
#     buildTristrips(allTriangles)
#
#     # Show result and wait to exit
#
#     display(wait=True)
#
#     glfw.destroy_window(window)
#     glfw.terminate()
#
#
# if __name__ == '__main__':
#     main()
# Convex hull


#
# Usage: python main.py [-d] file_of_points
#
# You can press ESC in the window to exit.
#
# You'll need Python 3 and must install these packages:
#
#   PyOpenGL, GLFW


import sys, os, math

try:  # PyOpenGL
    from OpenGL.GL import *
except:
    print('Error: PyOpenGL has not been installed.')
    sys.exit(0)

try:  # GLFW
    import glfw
except:
    print('Error: GLFW has not been installed.')
    sys.exit(0)

# Globals

window = None

windowWidth = 1000  # window dimensions
windowHeight = 1000

minX = None  # range of points
maxX = None
minY = None
maxY = None

r = 0.01  # point radius as fraction of window size

numAngles = 32
thetas = [i / float(numAngles) * 2 * 3.14159 for i in range(numAngles)]  # used for circle drawing

allPoints = []  # list of points

lastKey = None  # last key pressed

discardPoints = False


# Point
#
# A Point stores its coordinates and pointers to the two points beside
# it (CW and CCW) on its hull.  The CW and CCW pointers are None if
# the point is not on any hull.
#
# For debugging, you can set the 'highlight' flag of a point.  This
# will cause the point to be highlighted when it's drawn.

class Point(object):

    def __init__(self, coords):

        self.x = float(coords[0])  # coordinates
        self.y = float(coords[1])

        self.ccwPoint = None  # point CCW of this on hull
        self.cwPoint = None  # point CW of this on hull

        self.highlight = False  # to cause drawing to highlight this point

    def __repr__(self):
        return 'pt(%g,%g)' % (self.x, self.y)

    def drawPoint(self):

        # Highlight with yellow fill

        if self.highlight:
            glColor3f(0.9, 0.9, 0.4)
            glBegin(GL_POLYGON)
            for theta in thetas:
                glVertex2f(self.x + r * math.cos(theta), self.y + r * math.sin(theta))
            glEnd()

        # Outline the point

        glColor3f(0, 0, 0)
        glBegin(GL_LINE_LOOP)
        for theta in thetas:
            glVertex2f(self.x + r * math.cos(theta), self.y + r * math.sin(theta))
        glEnd()

        # Draw edges to next CCW and CW points.

        if self.ccwPoint:
            glColor3f(0, 0, 1)
            drawArrow(self.x, self.y, self.ccwPoint.x, self.ccwPoint.y)

        if self.ccwPoint:
            glColor3f(1, 0, 0)
            drawArrow(self.x, self.y, self.cwPoint.x, self.cwPoint.y)


# Draw an arrow between two points, offset a bit to the right

def drawArrow(x0, y0, x1, y1):
    d = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))

    vx = (x1 - x0) / d  # unit direction (x0,y0) -> (x1,y1)
    vy = (y1 - y0) / d

    vpx = -vy  # unit direction perpendicular to (vx,vy)
    vpy = vx

    xa = x0 + 1.5 * r * vx - 0.4 * r * vpx  # arrow tail
    ya = y0 + 1.5 * r * vy - 0.4 * r * vpy

    xb = x1 - 1.5 * r * vx - 0.4 * r * vpx  # arrow head
    yb = y1 - 1.5 * r * vy - 0.4 * r * vpy

    xc = xb - 2 * r * vx + 0.5 * r * vpx  # arrow outside left
    yc = yb - 2 * r * vy + 0.5 * r * vpy

    xd = xb - 2 * r * vx - 0.5 * r * vpx  # arrow outside right
    yd = yb - 2 * r * vy - 0.5 * r * vpy

    glBegin(GL_LINES)
    glVertex2f(xa, ya)
    glVertex2f(xb, yb)
    glEnd()

    glBegin(GL_POLYGON)
    glVertex2f(xb, yb)
    glVertex2f(xc, yc)
    glVertex2f(xd, yd)
    glEnd()


# Determine whether three points make a left or right turn

LEFT_TURN = 1
RIGHT_TURN = 2
COLLINEAR = 3


def turn(a, b, c):
    det = (a.x - c.x) * (b.y - c.y) - (b.x - c.x) * (a.y - c.y)

    if det > 0:
        return LEFT_TURN
    elif det < 0:
        return RIGHT_TURN
    else:
        return COLLINEAR


# Build a convex hull from a set of point
#
# Use the method described in class


def buildHull(points):
    if len(points) == 2:  # If function was only called with two points

        points[1].ccwPoint = points[0]
        points[1].cwPoint = points[0]
        points[0].ccwPoint = points[1]
        points[0].cwPoint = points[1]


    elif len(points) == 3:  # If function was only called with three points

        if turn(points[0], points[1], points[2]) == 1: # if turn of three points in order results in left turn
            points[2].ccwPoint = points[0]
            points[2].cwPoint = points[1]

            points[0].cwPoint = points[2]
            points[0].ccwPoint = points[1]

            points[1].cwPoint = points[0]
            points[1].ccwPoint = points[2]



        elif turn(points[0], points[1], points[2]) == 2: #if turn of three points in order results in right turn

            points[2].cwPoint = points[0]
            points[2].ccwPoint = points[1]

            points[0].cwPoint = points[1]
            points[0].ccwPoint = points[2]

            points[1].ccwPoint = points[0]
            points[1].cwPoint = points[2]



        else: # if neither, three points are colinear

            points[0].cwPoint = points[1]
            points[0].ccwPoint = points[1]

            points[1].cwPoint = points[2]
            points[1].ccwPoint = points[0]

            points[2].ccwPoint = points[1]
            points[2].cwPoint = points[1]
    else:
        l1 = None  # These will store index references to the top and bottom most points of both hulls for merging them together
        r1 = None
        l2 = None
        r2 = None

        Arr1 = points[:(int(len(points) / 2))]  # Place left section of list passed into BuildHull into Arr1
        buildHull(Arr1)  # Call buildhull with new smaller list
        Arr2 = points[int(len(points) / 2):]
        buildHull(Arr2)
        Arr3 = []  # Here we will store all the points we want to delete during walk up and walk down
        r = Arr2[0]  # since lists are sorted, first point of right list is leftmost point
        l = Arr1[-1]  # since lists are sort, last point of left list is rightmost point

        if ((len(Arr1) + len(Arr2)) > 3):
            Arr3.append(l)
            Arr3.append(r)

        while (turn(l.ccwPoint, l, r) == 1 or turn(l, r, r.cwPoint) == 1):  # walking up both lists

            if (turn(l.ccwPoint, l, r) == 1):# if left's ccw, left and right points make a left turn, walkup the left list
                l = l.ccwPoint
                if ((len(Arr1) + len(
                        Arr2)) > 3):  # only add points for deletion if the length of both lists is larger than 3
                    Arr3.append(l)
            else:  #this is an else instead of elif to address the case of a colinear return from turn
                r = r.cwPoint
                if ((len(Arr1) + len(Arr2)) > 3):
                    Arr3.append(r)

        if l in Arr3: # remove the edge cases
            Arr3.remove(l)
        if r in Arr3:
            Arr3.remove(r)

        for x in range(0, len(Arr2)):# find what index of the right list matches with the highest right point
            if (r.x == Arr2[x].x and r.y == Arr2[x].y):
                r1 = x

        for i in range(0, len(Arr1)):#Find what index of the left list matches with the highest left point
            if (l.x == Arr1[i].x and l.y == Arr1[i].y):
                l1 = i

        r = Arr2[0] #Set R and L markers to the center of both lists before walkdown procedure
        l = Arr1[-1]
        while ((turn(l.cwPoint, l, r) != 1) or (turn(r.ccwPoint, r, l) != 2)): #same as walkup,
            if turn(l.cwPoint, l, r) == 2: # if clockwise of left , left and right make a right turn, walk down left side
                l = l.cwPoint
                if ((len(Arr1) + len(Arr2)) > 3):# append values as we walkup to list to be later deleted
                    Arr3.append(l)
            else:
                r = r.ccwPoint
                if ((len(Arr1) + len(Arr2)) > 3):
                    Arr3.append(r)

        if l in Arr3:#remove edge cases
            Arr3.remove(l)
        if r in Arr3:
            Arr3.remove(r)

        for x in range(0, len(Arr2)):
            if (r.x == Arr2[x].x and r.y == Arr2[x].y):
                r2 = x
        for i in range(0, len(Arr1)):
            if (l.x == Arr1[i].x and l.y == Arr1[i].y):
                l2 = i

        Arr2[r2].cwPoint = Arr1[l2]# join up lists using indecies found in for loops, in this case start by pointing clockwise of bottom right list to bottom or left list
        Arr1[l1].cwPoint = Arr2[r1]
        Arr1[l2].ccwPoint = Arr2[r2]
        Arr2[r1].ccwPoint = Arr1[l1]
        for x in range(0, len(Arr3)): #this is the deletion, iterate through the list that was appended to, and set all pointers to None
            Arr3[x].cwPoint = None
            Arr3[x].ccwPoint = None

    for p in points:
        p.highlight = True
        display(wait=True)

    #display()


windowLeft = None
windowRight = None
windowTop = None
windowBottom = None


# Set up the display and draw the current image

def display(wait=False):
    global lastKey, windowLeft, windowRight, windowBottom, windowTop

    # Handle any events that have occurred

    glfw.poll_events()

    # Set up window

    glClearColor(1, 1, 1, 0)
    glClear(GL_COLOR_BUFFER_BIT)
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    if maxX - minX > maxY - minY:  # wider point spread in x direction
        windowLeft = -0.1 * (maxX - minX) + minX
        windowRight = 1.1 * (maxX - minX) + minX
        windowBottom = windowLeft
        windowTop = windowRight
    else:  # wider point spread in y direction
        windowTop = -0.1 * (maxY - minY) + minY
        windowBottom = 1.1 * (maxY - minY) + minY
        windowLeft = windowBottom
        windowRight = windowTop

    glOrtho(windowLeft, windowRight, windowBottom, windowTop, 0, 1)

    # Draw points and hull

    for p in allPoints:
        p.drawPoint()

    # Show window

    glfw.swap_buffers(window)

    # Maybe wait until the user presses 'p' to proceed

    if wait:

        sys.stderr.write('Press "p" to proceed ')
        sys.stderr.flush()

        lastKey = None
        while lastKey != 80:  # wait for 'p'
            glfw.wait_events()
            display()

        sys.stderr.write('\r                     \r')
        sys.stderr.flush()


# Handle keyboard input

def keyCallback(window, key, scancode, action, mods):
    global lastKey

    if action == glfw.PRESS:

        if key == glfw.KEY_ESCAPE:  # quit upon ESC
            sys.exit(0)
        else:
            lastKey = key


# Handle window reshape


def windowReshapeCallback(window, newWidth, newHeight):
    global windowWidth, windowHeight

    windowWidth = newWidth
    windowHeight = newHeight


# Handle mouse click/release

def mouseButtonCallback(window, btn, action, keyModifiers):
    if action == glfw.PRESS:

        # Find point under mouse

        x, y = glfw.get_cursor_pos(window)  # mouse position

        wx = (x - 0) / float(windowWidth) * (windowRight - windowLeft) + windowLeft
        wy = (windowHeight - y) / float(windowHeight) * (windowTop - windowBottom) + windowBottom

        minDist = windowRight - windowLeft
        minPoint = None
        for p in allPoints:
            dist = math.sqrt((p.x - wx) * (p.x - wx) + (p.y - wy) * (p.y - wy))
            if dist < r and dist < minDist:
                minDist = dist
                minPoint = p

        # print point and toggle its highlight

        if minPoint:
            minPoint.highlight = not minPoint.highlight
            print(minPoint)


# Initialize GLFW and run the main event loop

def main():
    global window, allPoints, minX, maxX, minY, maxY, r, discardPoints

    # Check command-line args

    if len(sys.argv) < 2:
        print('Usage: %s points1.txt' % sys.argv[0])
        sys.exit(1)

    args = sys.argv[1:]
    while len(args) > 1:
        print(args)
        if args[0] == '-d':
            discardPoints = not discardPoints
        args = args[1:]

    # Set up window

    if not glfw.init():
        print('Error: GLFW failed to initialize')
        sys.exit(1)

    window = glfw.create_window(windowWidth, windowHeight, "Assignment 1", None, None)

    if not window:
        glfw.terminate()
        print('Error: GLFW failed to create a window')
        sys.exit(1)

    glfw.make_context_current(window)
    glfw.swap_interval(1)
    glfw.set_key_callback(window, keyCallback)
    glfw.set_window_size_callback(window, windowReshapeCallback)
    glfw.set_mouse_button_callback(window, mouseButtonCallback)

    # Read the points

    with open(args[0], 'rb') as f:
        allPoints = [Point(line.split(b' ')) for line in f.readlines()]

    # Get bounding box of points

    minX = min(p.x for p in allPoints)
    maxX = max(p.x for p in allPoints)
    minY = min(p.y for p in allPoints)
    maxY = max(p.y for p in allPoints)

    # Adjust point radius in proportion to bounding box

    if maxX - minX > maxY - minY:
        r *= maxX - minX
    else:
        r *= maxY - minY

    # Sort by increasing x.  For equal x, sort by increasing y.

    allPoints.sort(key=lambda p: (p.x, p.y))

    # Run the code

    buildHull(allPoints)

    # Wait to exit

    while not glfw.window_should_close(window):
        glfw.wait_events()

    glfw.destroy_window(window)
    glfw.terminate()


if __name__ == '__main__':
    main()