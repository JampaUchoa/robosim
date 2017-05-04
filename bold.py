from __future__ import division # For correct float division in Python 2
from AriaPy import *
import sys
import numpy
from heapq import *

debug = 0

#
# Aria Parameters
#

Aria_init()
parser = ArArgumentParser(sys.argv)
parser.loadDefaultArguments()
robot = ArRobot()
conn = ArRobotConnector(parser, robot)
sonar = ArSonarDevice()
if not conn.connectRobot():
  print "Could not connect to robot, exiting"
  Aria_exit(1)

if not Aria_parseArgs():
  Aria_logOptions()
  Aria_exit(1)

##########
# Mapa
#########

print "Initializing array"

endGoal = {'x': 9000, 'y': 9000}


tileSize = 500 # 50cm
mapSize = int(45 * 1000 / tileSize) # 30m de largura maxima
mapOffset = mapSize / 2
explored = numpy.zeros(shape=(mapSize,mapSize))

# Inicia o mapa
#for x in xrange(mapSize):
#    explored.append([])
#    for y in xrange(mapSize):
#        explored[x].append(0)

print "Done"

#Xmap = Xreal / Tsize + Offset
#Xreal = (Xmap - Offset) * Tsize (+/- Offset)

# Insere uma "parede"
def exploredInsert(x, y):
    explored[int(x / tileSize + mapOffset)][int(y / tileSize + mapOffset)] = 1

#Recupera uma coordenada do array para real
def getRealCoords(x, y):
    return ((x - mapOffset) * tileSize, (y - mapOffset) * tileSize)

def getArrayCoords(x, y):
    return (int(round(x / tileSize + mapOffset)), int(round(y / tileSize + mapOffset)))

robot.addRangeDevice(sonar)
robot.runAsync(1)

##########
# Actions
##########

#limiterAction = ArActionLimiterForwards("speed limiter near", 300, 600, 250)
#limiterFarAction = ArActionLimiterForwards("speed limiter far", 300, 1100, 400)
#tableLimiterAction = ArActionLimiterTableSensor()
#robot.addAction(tableLimiterAction, 100)
#robot.addAction(limiterAction, 95)
#robot.addAction(limiterFarAction, 90)

recover = ArActionStallRecover()
robot.addAction(recover, 100)

gotoPoseAction = ArActionGoto("goto")
robot.addAction(gotoPoseAction, 50)

stopAction = ArActionStop ("stop")
robot.addAction(stopAction, 40)


robot.enableMotors()

##
##
##

duration = 30000
ArLog.log(ArLog.Normal, "Going to four goals in turn for %d seconds, then cancelling goal and exiting." % (duration/1000))

goalNum = 0
timer = ArTime()
timer.setToNow()

#
# Hey now you are an A*
#

path = []
finishAt = getArrayCoords(endGoal['x'], endGoal['y']);

def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def aStar(start):

    array = explored
    goal = finishAt

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))

    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))

    return False



#
# Main
#

reroute = True

while Aria.getRunning():
    robot.lock()

    poses = sonar.getCurrentBufferAsVector()
    for p in poses:
        try:
            exploredInsert(p.x, p.y)
        except Exception, e:
            print "Error trying to insert x: %d, y: %d -> x: %d, y: %d" % (p.x, p.y, int(p.x / tileSize + mapOffset), int(y / tileSize + mapOffset))
            print e

    if (reroute or gotoPoseAction.haveAchievedGoal()):
        reroute = False
        myPos = robot.getPose()
        path = aStar(getArrayCoords(myPos.x, myPos.y))

        # Se chegarmos no tile final
        if not path:
            ArLog.log(ArLog.Normal, "GOOOOOOAL");
            robot.unlock()
            break

        nextTile = path[-1]
        nextPos = getRealCoords(nextTile[0], nextTile[1]);
        nextPosEase = (nextPos[0] + (myPos.x % tileSize), nextPos[1] + (myPos.x % tileSize))
        gotoPoseAction.setGoal(ArPose(nextPos[0], nextPos[1]));

        ArLog.log(ArLog.Normal, "Going to next goal at %.0f %.0f" % (gotoPoseAction.getGoal().getX(), gotoPoseAction.getGoal().getY()) );

    if (debug and timer.mSecSince() >= 5000):
        print explored
        print "\n\n\n"
        timer.setToNow()

    #print robot.findDistanceTo(ArPose(nextPos[0], nextPos[1]))

    distance = robot.findDistanceTo(ArPose(nextPos[0], nextPos[1]))

    if distance < 250 or explored[nextTile[0]][nextTile[1]] == 1:
        gotoPoseAction.cancelGoal()
        reroute = True
        print distance

#      gotoPoseAction.cancelGoal()
#      robot.unlock()
#      ArUtil.sleep(3000)
#      break

    robot.unlock()
    ArUtil.sleep(100)

Aria_exit(0)
