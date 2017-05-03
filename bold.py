from __future__ import division # For correct float division in Python 2
from AriaPy import *
import sys
import random

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

goal = {x: 0, y: 0}


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
    return {x: (x - mapOffset) * tileSize + tileSize / 2.0, y: (y - mapOffset) * tileSize + tileSize / 2.0}

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
start = ArTime()
start.setToNow()

while Aria.getRunning():
    robot.lock()
    poses = sonar.getCurrentBufferAsVector()
#    print 'Sonar readings (%d) (Point coordinates in space):' % (len(poses))
    for p in poses:
        #print '    sonar sensed something at point ', p
        try:
            exploredInsert(p.x, p.y)
            if random.random() < 0.0005:
                print explored
                print "\n\n\n"
        except Exception, e:
            print "Error trying to insert x: %d, y: %d -> x: %d, y: %d" % (p.x, p.y, int(p.x / tileSize + mapOffset), int(y / tileSize + mapOffset))
            print e

    if (first or gotoPoseAction.haveAchievedGoal()):
        first = False;
        goalNum = goalNum + 1
        if (goalNum > 4):
            goalNum = 1
        if (goalNum == 1):
            gotoPoseAction.setGoal(ArPose(2500, 0))
        elif (goalNum == 2):
            gotoPoseAction.setGoal(ArPose(2500, 2500))
        elif (goalNum == 3):
            gotoPoseAction.setGoal(ArPose(0, 2500))
        elif (goalNum == 4):
            gotoPoseAction.setGoal(ArPose(0, 0))
        ArLog.log(ArLog.Normal, "Going to next goal at %.0f %.0f" % (gotoPoseAction.getGoal().getX(), gotoPoseAction.getGoal().getY()) );

#    if (start.mSecSince() >= duration):
#      ArLog.log(ArLog.Normal, "%d seconds have elapsed. Cancelling current goal, waiting 3 seconds, and exiting." % (duration/1000))
#      gotoPoseAction.cancelGoal()
#      robot.unlock()
#      ArUtil.sleep(3000)
#      break

    robot.unlock()
    ArUtil.sleep(100)

Aria_exit(0)
