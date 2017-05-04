from __future__ import division # For correct float division in Python 2
from AriaPy import *
import sys

debug = 1

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

endGoal = {'x': 250, 'y': 0}


tileSize = 500 # 50cm
mapSize = int(30 * 1000 / tileSize) # 30m de largura maxima
mapOffset = mapSize / 2
explored = []

# Inicia o mapa
for x in xrange(mapSize):
    explored.append([])
    for y in xrange(mapSize):
        explored[x].append(0)

print "Done"

#Xmap = Xreal / Tsize + Offset
#Xreal = (Xmap - Offset) * Tsize (+/- Offset)

# Insere uma "parede"
def exploredInsert(x, y):
    explored[int(x / tileSize + mapOffset)][int(y / tileSize + mapOffset)] = 1

#Recupera uma coordenada do array para real
def getRealCoords(x, y):
    return {'x': (x - mapOffset) * tileSize + tileSize / 2.0, 'y': (y - mapOffset) * tileSize + tileSize / 2.0}

def getArrayCoords(x, y):
    return {'x': int(x / tileSize + mapOffset), 'y': int(y / tileSize + mapOffset)}

robot.addRangeDevice(sonar)
robot.runAsync(1)

##########
# Actions
##########

limiterAction = ArActionLimiterForwards("speed limiter near", 300, 600, 250)
limiterFarAction = ArActionLimiterForwards("speed limiter far", 300, 1100, 400)
tableLimiterAction = ArActionLimiterTableSensor()
robot.addAction(tableLimiterAction, 100)
robot.addAction(limiterAction, 95)
robot.addAction(limiterFarAction, 90)

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

first = True
goalNum = 0
timer = ArTime()
timer.setToNow()

#
# Hey now you are an A*
#

path = []
finishAt = getArrayCoords(endGoal['x'], endGoal['y']);

#
# Main
#
gotoPoseAction.setGoal(ArPose(0, 200))

while Aria.getRunning():
    robot.lock()

    poses = sonar.getCurrentBufferAsVector()
    for p in poses:
        try:
            exploredInsert(p.x, p.y)
        except Exception, e:
            print "Error trying to insert x: %d, y: %d -> x: %d, y: %d" % (p.x, p.y, int(p.x / tileSize + mapOffset), int(y / tileSize + mapOffset))
            print e

    if (gotoPoseAction.haveAchievedGoal()):




        # Se chegarmos no tile final
        if (0):
            gotoPoseAction.setGoal(ArPose(endGoal['x'], endGoal['y']))
            ArUtil.sleep(2500)
            robot.unlock()
            break

        ArLog.log(ArLog.Normal, "Going to next goal at %.0f %.0f" % (gotoPoseAction.getGoal().getX(), gotoPoseAction.getGoal().getY()) );

    if (debug and timer.mSecSince() >= 5000):
        print explored
        print "\n\n\n"
        timer.setToNow()

#      gotoPoseAction.cancelGoal()
#      robot.unlock()
#      ArUtil.sleep(3000)
#      break

    robot.unlock()
    ArUtil.sleep(100)

Aria_exit(0)
