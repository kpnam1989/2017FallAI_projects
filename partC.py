# === Output Specifications ===
#
# `next_move` should return a string in one of the following formats:
#
# 'move {steering} {distance}', where '{steering}' is a floating-point number between
#   -`max_steering` and `max_steering` (inclusive) and '{distance}' is a floating-point
#   number between 0 and `max_distance`
# 
# 'lift {bearing}', where '{bearing}' is replaced by the bearing of the box's center point
#   relative to the robot (with 0 meaning right in front of the robot).
#   If a box's center point is within 0.5 units of the robot's center point and its bearing
#   is within 0.2 radians of the specified bearing, then the box is lifted. Otherwise, nothing
#   happens.
#
# 'down {bearing}', where '{bearing}' is replaced by the relative bearing where the
#   robot wishes to place its box. The robot will attempt to place the box so that its center point
#   is at this bearing at a distance of 0.5 units away. If the box can be so placed without intersecting
#   anything, it does so. Otherwise, the placement fails and the robot continues to hold the box.

import math
import robot
#import helper

MAP_X = -5.5
MAP_Y = 4.5

class Helper:
    
    # Assuming input map is of size 31*31 at most
    BOX_INDEX_ZERO = 15 # at 0.0, the box index is 25
    BOX_SIZE = 1.0
    
    def Adjacent(self, box1, box2):
        return abs(box1[0]-box2[0]) + abs(box1[1] - box2[1]) == 1
    
    def Init(self):
        # Return a sample map
        # current position: 0.0 is at the center of the map
        # let's say map is 31*31
        # 0.0 is at the center of box [14, 14]
        grid = [[' ' for x in range(Helper.BOX_INDEX_ZERO * 2)] for y in range(Helper.BOX_INDEX_ZERO * 2)]
        return grid
    
	# Some codes have been removed 
    def FindBoxCenter(self, orientation, x, y):
        # given the position of the landmark
        # and the orientation
        # output the center of the box

        # 0 is N
        # 1 is S
        # 2 is W
        # 3 is E
        if orientation == 0:
            return [x, y - 0.1]
        elif orientation == 1:
            return [x,y + 0.1]
        elif orientation == 2:
            return [x + 0.1, y]
        else:
            return [x - 0.1, y]

    def Dist_box_warehouse(self, x, orientation, y_coord = False):
        # Orientation of a wall is different from orientation of a box
        if orientation == 0:
            orientation_ = 1
        elif orientation == 1:
            orientation_ = 0
        elif orientation == 2:
            orientation_ = 3
        elif orientation == 3:
            orientation_ = 2
        
        return self.Dist_box_wall(x, orientation_, y_coord)
              
    def Dist_box_wall(self, x, orientation, y_coord = False, boxSize = BOX_SIZE):
        # Find the center of the box
        center = x
        if y_coord:
            if orientation == 0:
                center = x - boxSize/2
            elif orientation == 1:
                center = x + boxSize/2
        else:
            if orientation == 2: # W
                center = x + boxSize/2
            elif orientation == 3: # E
                center = x - boxSize/2
        
        if boxSize == Helper.BOX_SIZE:
            # if this is a Wall or Warehouse
            res = int(round(center,0))
        else:
            # if we are dealing with a box for pick up
            res = round(2 * center, 0) /2
        
        if y_coord:
            res = -res
        return res
        
    def ShiftTarget(self, xy1,xy2, targetDistance = 0.48):
        # given input coordinates
        # shift the destination
        
        # calculate the slope and the current distance
        # make sure that the next distance is 4.99
        currentDistance = self.distance_between_points(xy1[0], xy1[1], xy2[0], xy2[1])
        x_new = xy1[0] * targetDistance / currentDistance + xy2[0] * (1.0 - targetDistance/currentDistance)
        y_new = xy1[1] * targetDistance / currentDistance + xy2[1] * (1.0 - targetDistance/currentDistance)
        return [x_new, y_new]
        
    def distance_between_points(self, x1, y1, x2, y2):
        res = (x1-x2)**2 + (y1 - y2)**2
        res = math.sqrt(res)
        return res
    
class RefineMeasurement:
    def __init__(self):
        self.box = False
        self.ActualCoord = [0.0, 0.0]
        self.dist = 0.0
        self.bearing = 0.0
        
class OnlineDeliveryPlanner:

    DELTA = [[-1, 0],
             [0,1],
             [1,0],
             [0,-1]]
    STEP_LIMIT = 2
    MOVE = 'move'
    LIFT = 'lift'
    DOWN = 'down'
    
    RAW_INPUT = False
    
    MEAS_LIMIT = 2.8
    DISTANCE_LIMIT_LIFT = 0.40
    DISTANCE_LIMIT_DROP = 0.65
    MOVE_RELIFT = 0.04
    
    DISTANCE_MOVE_DROP = 0.55
    DISTANCE_TARGET = 1.05
    
    ANGLE_LIMIT = 0.2
    
    def __init__(self, todo, max_distance, max_steering, verbose=False):
        self.todo = todo   #Integer number of boxes in the warehouse
        self.boxCounter = 0
        self.max_distance = max_distance   #Maximum movement per move/turn
        self.max_steering = max_steering   #Maximum steering angle per move/turn
        
        
        self.verbose = verbose   #Optional flag you can toggle from the testing suite.
        
        self.robot = robot.Robot(max_distance = max_distance, max_steering = max_steering)
        self.Helper = Helper()
        self.Grid = self.Helper.Init()
        self.Grid[self.Helper.BOX_INDEX_ZERO][self.Helper.BOX_INDEX_ZERO] = '@'

        self.ExploredGrid = self.Helper.Init_explored()

        self.DropOff = [0.0, 0.0]
        self.MapDimensions = {}
        self.BoxPositions = []
        self.CurrentBox = []
        self.TargetBox = None
        
        self.robot_has_box = None
        self.robot_is_crashed = None
        self.boxes_delivered = None
        self.data = None
        
        self.FoundBox = False
        self.error_x = []
        self.error_y = []
        self.RefineMeasurements = []
        self.start = True
        self.firstBox = True
        
        self.AngleTry = []
        
        self.lastX = self.robot.x
        self.lastY = self.robot.y
        self.lastBearing = self.robot.bearing
        
        self.realX = 0.0
        self.realY = 0.0
        self.realbearing = 0.0
        
        self.Relift = False
        self.LastMove = None
        self.LastMove_processed = None
        self.exploring = True
        
        self.MoveList = []
        
        self.count = 0
        self.recursion = 0

        # Known map size
        # To update value functions later 
        self.MapDimensions['min_x'] = self.Helper.BOX_INDEX_ZERO * 2
        self.MapDimensions['max_x'] = 0
        self.MapDimensions['min_y'] = self.Helper.BOX_INDEX_ZERO * 2
        self.MapDimensions['max_y'] = 0

    def RecordLastPosition(self):
        self.lastX = self.robot.x
        self.lastY = self.robot.y
        self.lastBearing = self.robot.bearing
        
    def process_measurement(self, data, verbose = False):
        self.count += 1
        self.data = data
            
        self.error_x = []
        self.error_y = []
        self.RefineMeasurements = []

        currentX = self.Helper.Dist_to_box(self.robot.y, True) + self.Helper.BOX_INDEX_ZERO
        currentY = self.Helper.Dist_to_box(self.robot.x) + self.Helper.BOX_INDEX_ZERO
        
        currentX = max(0, min(currentX, self.Helper.BOX_INDEX_ZERO * 2 - 1))
        currentY = max(0, min(currentY, self.Helper.BOX_INDEX_ZERO * 2 - 1))
        self.CurrentBox = [currentX, currentY]

        for meas in data:
            self.process(meas)

        self.Localize()

        if self.verbose:
            self.Print_map()
        self.FillExplored()
        
                
    def process(self, meas):
    #
    # given the x,y coordinates compared to current position
    # decide which box the xy should be in
    #
        if meas[2] >= OnlineDeliveryPlanner.MEAS_LIMIT and meas[0] == 'box':
            # if the object is too far away from the current box
            # ignore that measurement
            return
        
        xy = self.Transform_xy(meas)
        
        outOfBound = meas[2] >= OnlineDeliveryPlanner.MEAS_LIMIT
        
        if meas[0] == 'warehouse':
            boxIndex = [self.Helper.BOX_INDEX_ZERO + self.Helper.Dist_box_warehouse(xy[1], meas[1], True),
                            self.Helper.BOX_INDEX_ZERO + self.Helper.Dist_box_warehouse(xy[0], meas[1])]
            for i in range(len(boxIndex)):
                boxIndex[i] = max(0, boxIndex[i])
                boxIndex[i] = min(boxIndex[i], self.Helper.BOX_INDEX_ZERO * 2 - 1)
            
            if not outOfBound:
                self.Grid[boxIndex[0]][boxIndex[1]] = 'x'
                actualCoord = self.Helper.Get_Coord_Marker(boxIndex, meas[1])
                
                tmp = RefineMeasurement()
                tmp.ActualCoord = actualCoord
                tmp.dist = meas[2]
                tmp.bearing = meas[3]

                self.RefineMeasurements.append(tmp)
                
                self.error_x.append((actualCoord[0] - xy[0])/(meas[2]**2))
                self.error_y.append((actualCoord[1] - xy[1])/(meas[2]**2))
                
            self.Update_map_size(boxIndex, outOfBound)

        # Some codes have been removed
                    
                    
    
    def Localize(self):
        # Localized the robot based on noisy measurements

    def FillExplored(self):
        # Fill the horizontal and vertical as explored
        # Explore 3 cells around the current box
        
        maxExplored = 4
        for x in range(maxExplored):
            # only explored up to an obstacle
            x_row = min(self.CurrentBox[0] + x, self.Helper.BOX_INDEX_ZERO *2 - 1)
            if self.Grid[x_row][self.CurrentBox[1]] in [' ','@']:
                self.ExploredGrid[x_row][self.CurrentBox[1]] = True
            else:
                break

        for x in range(maxExplored):
            # only explored up to an obstacle
            row = max(self.CurrentBox[0] - x,0)
            if self.Grid[row][self.CurrentBox[1]] in [' ','@']:
                self.ExploredGrid[row][self.CurrentBox[1]] = True
            else:
                break

		# Some codes have been removed
            
    def Print_map(self):
        currentX = self.CurrentBox[0]
        currentY = self.CurrentBox[1]
        
        tmp1 = self.Grid[currentX][currentY]
        if tmp1 == ' ':
            self.Grid[currentX][currentY] = 'c'
            
        for x in range(max(0,self.MapDimensions['min_x']-1), self.MapDimensions['max_x']):
            tmp = self.Grid[x][max(0, self.MapDimensions['min_y']-1):(self.MapDimensions['max_y'])+1]
            print ''.join(tmp)
        
        if tmp1 == ' ':
            self.Grid[currentX][currentY] = tmp1
        
    def Transform_xy(self, measurement):
        # 0 is box, wall, warehouse
        # 1 is Mark (Direction)
        # 2 is Dist
        # 3 is bearing in Radian

        newAngle = robot.truncate_angle(measurement[3] + self.robot.bearing)
        x = self.robot.x + measurement[2] * math.cos(newAngle)
        y = self.robot.y + measurement[2] * math.sin(newAngle)        
        
        return [x,y]

    def next_move(self, verbose = False):
        if self.count > 200:
            return []
    
        if self.exploring:
            if len(self.BoxPositions) > 0:
                self.exploring = False
                
                self.TargetBox = self.next_box()
                # Move to within 1.1 of the box
                # if box is in the middle of a box grid => move to the center of the next grid
                # if box is on the edge of a grid => move to the center of that grid
                self.plan_move_point(self.TargetBox, OnlineDeliveryPlanner.DISTANCE_TARGET)
            elif len(self.MoveList) == 0:
                self.explore()
            else: 
                # Just in case we plan ahead to far and run into a wall
                # We have this problem in test2
                tmp = self.MoveList[-1]['target']
                if self.MoveList[-1]['type'] == OnlineDeliveryPlanner.MOVE and self.Grid[tmp[0]][tmp[1]] == 'x':
                    self.explore()
                
		# Some codes have been removed

        res = self.return_next_move()
        
        if res == None or self.Crash_next_move(res):
            
            # Current Target Box most likely a wrong observation
            # Remove it
            self.TargetBox = None
            self.exploring = True
            
            # Hopefully no overflow here
            self.recursion += 1
            if self.recursion >= 5:
                return []
            
            return self.next_move(self.verbose)
        
        self.recursion = 0
        
        self.LastMove_processed = res
        
        self.RecordLastPosition()
        if len(res) > 2:
            self.robot.move(res[1], res[2])
        
        res_str = []
        res_str.append(res[0])
        
        for i in range(1, len(res)):
            res_str.append(str(res[i]))
        
        
        return ' '.join(res_str)

    def return_next_move(self):
        # return the next move according to move list
        # move list may be of simple format
        # need some extra re-formatting
        
        if len(self.MoveList) > 0:
            res = self.MoveList.pop()
        else:
            if self.verbose:
                print "Move List is empty"
            return None
        
        # Copy the last move
        self.LastMove = res
        
        # res should be a dictionary
        # res has type, target
        # target is the index of the box

        distance =      0.0
        turningAngle =  0.0
        
        if res['type'] == OnlineDeliveryPlanner.MOVE:
            nextPosition = self.Helper.Get_Coord_box(res['target'])
            
            distance, turningAngle = self.robot.measure_distance_and_bearing_to(nextPosition)
            
            tmp_res = self.move_to_point(nextPosition, res)
            
        elif res['type'] == OnlineDeliveryPlanner.DOWN:
            distance, turningAngle = self.robot.measure_distance_and_bearing_to(res['target'])
            
            nextDist = 0.0
            shiftMove = False
            
            if distance > OnlineDeliveryPlanner.DISTANCE_LIMIT_DROP:                
                nextDist = OnlineDeliveryPlanner.DISTANCE_MOVE_DROP
                nextPosition = self.Helper.ShiftTarget([self.robot.x, self.robot.y], res['target'], nextDist)                
                nextDist1 = robot.compute_distance(nextPosition, res['target'])
                
                if self.verbose:
                    print "Need to move from current position to", nextPosition, " new distance = ", nextDist1
                    print "Current distance", distance
                
                tmp_res = self.move_to_point(nextPosition, res)
            else:
                tmp_res = [res['type'], turningAngle]
                
        elif res['type'] == OnlineDeliveryPlanner.LIFT:
            distance, turningAngle = self.robot.measure_distance_and_bearing_to(res['target'])
                
            nextDist = 0.0
            shiftMove = False
            
            # TO DO:
            if self.FoundBox == False:
                if self.verbose:
                    print "Box not found", res['target']
                self.TargetBox = None
                self.exploring = True
                self.MoveList = []
                return None
                
            if self.verbose: 
                if x != None:
                    print "Spotted box - dist-bearing", x.ActualCoord, x.dist, x.bearing
                    print "Max steering", self.robot.max_steering
                
            if self.Relift == True:
                # Codes have been removed
            else:
                # Refine the measurement, just in case we are too far off
                if x == None and self.FoundBox == True:
                    # Some times we found the box, but when we move closer, the box disappears
                    tmp_res = [OnlineDeliveryPlanner.LIFT, turningAngle]
                else:
                    distance = x.dist
                    sign = 1
                    if x.bearing < 0:
                        sign = -1
                    
                    if distance > OnlineDeliveryPlanner.DISTANCE_LIMIT_LIFT:
                        nextDist = distance - OnlineDeliveryPlanner.DISTANCE_LIMIT_LIFT
                        self.MoveList.append(res)
                        
                        if abs(x.bearing) > self.robot.max_steering:
                            tmp_res = [OnlineDeliveryPlanner.MOVE, sign * min(abs(x.bearing), self.robot.max_steering), 0.0]
                        else:
                            tmp_res = [OnlineDeliveryPlanner.MOVE, sign * min(abs(x.bearing), self.robot.max_steering), nextDist]
                    else:
                        tmp_res = [OnlineDeliveryPlanner.LIFT, x.bearing]
                                            
        return tmp_res

    def plan_move_point(self, target, distance, dropoff = False):
        moveList = []
        
        if self.verbose:
            print "Planning move to", target
        
        # Decide whether to move back to center first
        dist_center = self.distance_box_point(self.CurrentBox, [self.robot.x, self.robot.y])
        
        distance_target = robot.compute_distance([self.robot.x, self.robot.y], target)
        
        if distance_target < 0.8:
            # if we are too close to the target, no need to plan anything else
            pass
        else:
            if dist_center > 0.3:
                # Move back to the center 
                if self.verbose:
                    print "current distance to the center is ", dist_center
                moveList.append({'type': OnlineDeliveryPlanner.MOVE, 'target': self.CurrentBox})
            
            ValueGrid, actions = self.CreateValue(target, distance)
            
            # follow the value list
            i, j = self.CurrentBox
            
            count = 0
            while ValueGrid[i][j] > 0 and count < 30:
                count += 1
                move = actions[i][j]
                if move > len(OnlineDeliveryPlanner.DELTA):
                    if self.verbose:
                        print "move value not yet initiated at cell",i, j
                    return
                else:
                    i = i + OnlineDeliveryPlanner.DELTA[move][0]
                    j = j + OnlineDeliveryPlanner.DELTA[move][1]
                    moveList.append({'type': OnlineDeliveryPlanner.MOVE, 'target': [i, j]})

        # Add the last action
        if not self.exploring:
            # if only exploring, we don't need a last action
            if dropoff:
                last_action = OnlineDeliveryPlanner.DOWN
            else:
                last_action = OnlineDeliveryPlanner.LIFT
            moveList.append({'type': last_action, 'target': target})

        if self.verbose:
            self.PrintMoveList(moveList)
        
        moveList.reverse()
        self.MoveList = moveList
        
        return
    
    def CreateValue(self, target, distance):
        change = True
        ValueGrid = [[99 for x in range(self.Helper.BOX_INDEX_ZERO * 2)] for y in range(self.Helper.BOX_INDEX_ZERO * 2)]
        actions = [[5 for x in range(self.Helper.BOX_INDEX_ZERO * 2)] for y in range(self.Helper.BOX_INDEX_ZERO * 2)]
        
#        print "Find move to", target,"at distance", distance
        
        count = 0
        while change and count < 150:
            change = False
            count += 1
            for i in range(self.MapDimensions['min_x'], min(self.MapDimensions['max_x'] + 1, self.Helper.BOX_INDEX_ZERO * 2)):
                for j in range(self.MapDimensions['min_y'], min(self.MapDimensions['max_y'] + 1,self.Helper.BOX_INDEX_ZERO * 2)):
                    if self.Grid[i][j] != 'x':
                        if self.distance_box_point([i,j], target) < distance:
                            if ValueGrid[i][j] != 0:
                                change = True
                                ValueGrid[i][j] = 0
                        else:
                            for move in range(len(OnlineDeliveryPlanner.DELTA)):
                                i2 = i + OnlineDeliveryPlanner.DELTA[move][0]
                                j2 = j + OnlineDeliveryPlanner.DELTA[move][1]
                                
                                if self.Grid[i2][j2] != 'x' and i2 <=  self.MapDimensions['max_x'] and i2 >= self.MapDimensions['min_x'] and j2 >= self.MapDimensions['min_y'] and j2 <= self.MapDimensions['max_y']:
                                    cost = ValueGrid[i2][j2] + 1
                                
                                    # Add additional cost if we have not explorer this cell
                                    if self.ExploredGrid[i2][j2] == False:
                                        cost += 3
                                    
                                    # Add extra cost if we are not going straight
                                    if actions[i][j] != actions[i2][j2]:
                                        cost += 1
                                        
                                    if cost < ValueGrid[i][j]:
                                        change = True
                                        ValueGrid[i][j] = cost
                                        actions[i][j] = move                  
                            
        return (ValueGrid, actions)
    
    def next_box(self):
        if self.firstBox:
            self.firstBox = False
            if len(self.BoxPositions) > 1:
                min_dist = 100
                select = -1
                for i in range(len(self.BoxPositions)):
                    tmp_dist = robot.compute_distance([self.robot.x, self.robot.y], self.BoxPositions[i])
                    if tmp_dist <= min_dist:
                        select = i
                        min_dist = tmp_dist

                if select >= 0:
                    return self.BoxPositions.pop(select)
            else:
                return self.BoxPositions.pop()
                
        else:
            if len(self.BoxPositions) > 0:
                return self.BoxPositions.pop(0)
            # self.BoxPositions = self.BoxPositions[1:]
    
    def distance_box_point(self, thisBox, thatBox, box = False):
        # Codes removed

    def ValidNeighbor(self, currentCell, explored):
        # return the valid neighbor of a cell
        res = []
        for i in OnlineDeliveryPlanner.DELTA:
            x2 = currentCell[0] + i[0]
            y2 = currentCell[1] + i[1]
            if [x2,y2] not in explored and  self.Grid[x2][y2] == ' ':
                res.append([x2, y2])

                # the explored variable will also be changed
                explored.append([x2, y2])
                
        return res
    
    def explore(self):
        # If there are no more boxes to find
        if self.todo < self.boxCounter + 1:
            return

        # Use a BFS to find the next cell to explorer
        count = 0
        toExplore = [self.CurrentBox]
        explored = []
        while (len(toExplore) > 0) and count < 50:
            count += 1
            tmp = toExplore.pop()
            validNeighbor = self.ValidNeighbor(tmp, explored)

            for i in validNeighbor:
                if self.ExploredGrid[i[0]][i[1]] == False:
                    
                    target = self.Helper.Get_Coord_box(i)
                    self.plan_move_point(target, 0.5, dropoff = False)
                  
                    return
                else:
                    toExplore.append(i)
            
        return
    
    def set_robot_state(self, robot_has_box, robot_is_crashed, boxes_delivered, verbose = False):
        
        #######################################################################################
        #  This function is called from testing_suite_partC to let you know if the
        # robot is holding a box,
        # robot is crashed,
        # a list of the boxes delivered so far
        # You do not need to modify this method for a correct solution
        # (although you MAY add your own code to it if you wish)
        #######################################################################################      
        if robot_is_crashed:
            if self.verbose:
                print "Robot crashed"
                self.PrintMeasurements(self.data)
            if OnlineDeliveryPlanner.RAW_INPUT:
                x = raw_input("Crashed")
        
        if robot_has_box != None and self.robot_has_box == None:
            # Picked up a box, so change the map
            self.TargetBox = None
            self.MoveList = []
            self.LastMove_processed = None
            self.FoundBox = False
            self.Relift = False
            
            if self.verbose:
                print "Picked up a box at", self.LastMove['target']
                print "boxes left", self.BoxPositions
            for i in range(len(self.BoxPositions)):
                if self.BoxPositions[i] == self.LastMove['target']:
                    self.BoxPositions.pop(i)
                    
            # Clear Move list, especially when we are trying different way to lift
            self.MoveList = []
            self.AngleTry = []
        elif robot_has_box == None and self.robot_has_box != None:
            if self.verbose:
                print "Drop off a box at", self.LastMove['target']
            self.MoveList = []
            self.exploring = True
        elif robot_has_box == None and self.LastMove_processed[0] == OnlineDeliveryPlanner.LIFT:
            if self.Relift == True:
                self.Relift = False
                self.TargetBox = None
                self.MoveList = []
            else:
                if self.verbose:
                    print "Lift again"
                self.Relift = True

        self.robot_has_box = robot_has_box
        self.robot_is_crashed = robot_is_crashed
        self.boxes_delivered = boxes_delivered
        
        if verbose:
            print "Robot has box: {},  Robot is crashed: {}".format(robot_has_box, robot_is_crashed)
            print "Boxes delivered: {}".format(boxes_delivered)
    
    def set_coord(self, x,y, bearing):
        # Used in debugging
        self.realX = x
        self.realY = y
        self.realbearing = bearing
