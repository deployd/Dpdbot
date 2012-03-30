#
# create.py
#
# Python interface for the iRobot Create
#
# Zach Dodds   dodds@cs.hmc.edu
# updated for SIGCSE 3/9/07
#

import serial
import math
import time
import thread
import threading

# some module-level definitions for the robot commands
START = chr(128)    # already converted to bytes...
BAUD = chr(129)     # + 1 byte
CONTROL = chr(130)  # deprecated for Create
SAFE = chr(131)
FULL = chr(132)
POWER = chr(133)
SPOT = chr(134)     # Same for the Roomba and Create
CLEAN = chr(135)    # Clean button - Roomba
COVER = chr(135)    # Cover demo - Create
MAX = chr(136)      # Roomba
DEMO = chr(136)     # Create
DRIVE = chr(137)    # + 4 bytes
MOTORS = chr(138)   # + 1 byte
LEDS = chr(139)     # + 3 bytes
SONG = chr(140)     # + 2N+2 bytes, where N is the number of notes
PLAY = chr(141)     # + 1 byte
SENSORS = chr(142)  # + 1 byte
FORCESEEKINGDOCK = chr(143)  # same on Roomba and Create
# the above command is called "Cover and Dock" on the Create
DRIVEDIRECT = chr(145)       # Create only
STREAM = chr(148)       # Create only
QUERYLIST = chr(149)       # Create only
PAUSERESUME = chr(150)       # Create only

# the four SCI modes
# the code will try to keep track of which mode the system is in,
# but this might not be 100% trivial...
OFF_MODE = 0
PASSIVE_MODE = 1
SAFE_MODE = 2
FULL_MODE = 3

# the sensors
BUMPS_AND_WHEEL_DROPS = 7
WALL_IR_SENSOR = 8
CLIFF_LEFT = 9
CLIFF_FRONT_LEFT = 10
CLIFF_FRONT_RIGHT = 11
CLIFF_RIGHT = 12
VIRTUAL_WALL = 13
LSD_AND_OVERCURRENTS = 14
INFRARED_BYTE = 17
BUTTONS = 18
DISTANCE = 19
ANGLE = 20
CHARGING_STATE = 21
VOLTAGE = 22
CURRENT = 23
BATTERY_TEMP = 24
BATTERY_CHARGE = 25
BATTERY_CAPACITY = 26
WALL_SIGNAL = 27
CLIFF_LEFT_SIGNAL = 28
CLIFF_FRONT_LEFT_SIGNAL = 29
CLIFF_FRONT_RIGHT_SIGNAL = 30
CLIFF_RIGHT_SIGNAL = 31
CARGO_BAY_DIGITAL_INPUTS = 32
CARGO_BAY_ANALOG_SIGNAL = 33
CHARGING_SOURCES_AVAILABLE = 34
OI_MODE = 35
SONG_NUMBER = 36
SONG_PLAYING = 37
NUM_STREAM_PACKETS = 38
REQUESTED_VELOCITY = 39
REQUESTED_RADIUS = 40
REQUESTED_RIGHT_VELOCITY = 41
REQUESTED_LEFT_VELOCITY = 42
# others just for easy access to particular parts of the data
POSE = 100
LEFT_BUMP = 101
RIGHT_BUMP = 102
LEFT_WHEEL_DROP = 103
RIGHT_WHEEL_DROP = 104
CENTER_WHEEL_DROP = 105
LEFT_WHEEL_OVERCURRENT = 106
RIGHT_WHEEL_OVERCURRENT = 107
ADVANCE_BUTTON = 108
PLAY_BUTTON = 109

#                    0 1 2 3 4 5 6 7 8 9101112131415161718192021222324252627282930313233343536373839404142
SENSOR_DATA_WIDTH = [0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,2,2,1,2,2,1,2,2,2,2,2,2,2,1,2,1,1,1,1,1,2,2,2,2]


# for printing the SCI modes
def modeStr( mode ):
    """ prints a string representing the input SCI mode """
    if mode == OFF_MODE: return 'OFF_MODE'
    if mode == PASSIVE_MODE: return 'PASSIVE_MODE'
    if mode == SAFE_MODE: return 'SAFE_MODE'
    if mode == FULL_MODE: return 'FULL_MODE'
    print 'Warning: unknown mode', mode, 'seen in modeStr'
    return 'UNKNOWN_MODE'

#
# some module-level functions for dealing with bits and bytes
#
def bytesOfR( r ):
    """ for looking at the raw bytes of a sensor reply, r """
    print 'raw r is', r
    for i in range(len(r)):
        print 'byte', i, 'is', ord(r[i])
    print 'finished with formatR'

def bitOfByte( bit, byte ):
    """ returns a 0 or 1: the value of the 'bit' of 'byte' """
    if bit < 0 or bit > 7:
        print 'Your bit of', bit, 'is out of range (0-7)'
        print 'returning 0'
        return 0
    return ((byte >> bit) & 0x01)

def toBinary( val, numbits ):
    """ prints numbits digits of val in binary """
    if numbits == 0:  return
    toBinary( val>>1 , numbits-1 )
    print (val & 0x01),  # print least significant bit


def fromBinary( s ):
    """ s is a string of 0's and 1's """
    if s == '': return 0
    lowbit = ord(s[-1]) - ord('0')
    return lowbit + 2*fromBinary( s[:-1] )


def twosComplementInt1byte( byte ):
    """ returns an int of the same value of the input
        int (a byte), but interpreted in two's
        complement
        the output range should be -128 to 127
    """
    # take everything except the top bit
    topbit = bitOfByte( 7, byte )
    lowerbits = byte & 127
    if topbit == 1:
        return lowerbits - (1 << 7)
    else:
        return lowerbits


def twosComplementInt2bytes( highByte, lowByte ):
    """ returns an int which has the same value
        as the twosComplement value stored in
        the two bytes passed in
        
        the output range should be -32768 to 32767
        
        chars or ints can be input, both will be
        truncated to 8 bits
    """
    # take everything except the top bit
    topbit = bitOfByte( 7, highByte )
    lowerbits = highByte & 127
    unsignedInt = lowerbits << 8 | (lowByte & 0xFF)
    if topbit == 1:
        # with sufficient thought, I've convinced
        # myself of this... we'll see, I suppose.
        return unsignedInt - (1 << 15)
    else:
        return unsignedInt


def toTwosComplement2Bytes( value ):
    """ returns two bytes (ints) in high, low order
        whose bits form the input value when interpreted in
        two's complement
    """
    # if positive or zero, it's OK
    if value >= 0:
        eqBitVal = value
    # if it's negative, I think it is this
    else:
        eqBitVal = (1<<16) + value
    
    return ( (eqBitVal >> 8) & 0xFF, eqBitVal & 0xFF )

def poseDeltaFromVelRadSec( vel_mm_sec, ROC, sec ):
    """ returns the pose change (dx,dy,dthr) in (mm,mm,radians)
        undergone by a differential-drive robot
        with a wheelspan of 258mm that is traveling with
        a "velocity" of vel_mm_sec, along a radius of
        ROC_mm, for sec seconds
        
        NOTE that the pose change is represented in the canonical
        "robot-centric" coordinate system:
              
              Hooray for ASCII art!
                            
                            | +x         aligned to robot's heading
                            |
                        ^   |   ^
                        |   |   |
                 +y <---WL--+--WR--- -y  perp to robot's heading
                            |
                            |            DELTA = 1/2 distance from WL to WR
                            | -x
             
             vel_mm_sec is the average of the velocities of WL and WR
             it is positive when the robot is moving forward
             
             the center of the robot's circular arc is at (0,ROC)
             positive ROC => turning to the left
             negative ROC => turning to the right
        
        Special cases: ROC ==  1    => counterclockwise
                       ROC == -1    => clockwise
                       ROC == 32768 => straight
    """
    # the robot moves along the arc of a circle
    #
    #     the robot's position after the arc is
    #     (0,ROC) + (ROC*sin(thr),-ROC*cos(thr))
    #
    # so we first find thr
    #
    # handle special cases
    #
    
    DELTA = 258.0/2.0   # there are 258 mm between the roomba's wheels
    
    if ROC == 32768:
        # going straight, so the wheels have equal velocities
        # and there is no angular change
        thr_delta = 0.0
        x_delta = vel_mm_sec * sec   # D = RT in action!
        y_delta = 0.0
    
    elif ROC == 1 or ROC == 0:
        # turning in place counterclockwise = positive thr_delta
        x_delta = 0.0
        y_delta = 0.0
        # to do - check if the sign of vel_mm_sec matters!
        thr_delta = (vel_mm_sec * sec)/float(DELTA)
    
    elif ROC == -1:
        # turning in place counterclockwise = positive thr_delta
        x_delta = 0.0
        y_delta = 0.0
        # to do - check if the sign of vel_mm_sec matters!
        thr_delta = - ( (vel_mm_sec * sec)/float(DELTA) )
    
    else:
        # general case
        # how much distance did the center travel
        # the specification is not 100% clear whether vel_mm_sec
        # is the average velocity (signed) or speed (unsigned)
        # of the wheels... we need to test this!
        
        # we assume average speed (unsigned) for now...
        # we handle the case where vel_mm_sec and ROC are both > 0
        #    and then check for signs later...
        pos_vel_mm_sec = math.fabs(vel_mm_sec)
        pos_ROC = math.fabs(ROC)
        
        # need to switch the sign of the left wheel when the ROC < DELTA
        if DELTA <= pos_ROC:
            # both wheels are going the same direction
            # center is traveling at pos_vel_mm_sec
            # center is traveling at a radians/second
            a =  pos_vel_mm_sec / pos_ROC
        else:
            # wheels going in opposite directions
            # center is traveling at a radians/second
            a = pos_vel_mm_sec / DELTA
    
    # we find the total (positive) angle traveled, pos_thr
    pos_thr = a * sec
    
    # we handle four different cases
    
    # case 1: ROC >= 0 and vel_mm_sec >= 0  (forward left)
    if ROC >= 0 and vel_mm_sec >= 0:
        thr_delta = pos_thr
            # (0,ROC) + (ROC*sin(thr_delta),-ROC*cos(thr_delta))
        x_delta = 0.0 + ROC*math.sin(thr_delta)
        y_delta = ROC - ROC*math.cos(thr_delta)
    
    # case 2: ROC <  0 and vel_mm_sec >= 0  (forward right)
    if ROC <  0 and vel_mm_sec >= 0:
        thr_delta = -pos_thr
            # (0,ROC) + (ROC*sin(thr_delta),-ROC*cos(thr_delta))
        x_delta = 0.0 + ROC*math.sin(thr_delta)
        y_delta = ROC - ROC*math.cos(thr_delta)
    
    # case 3: ROC >= 0 and vel_mm_sec <  0  (backward left)
    if ROC >= 0 and vel_mm_sec <  0:
        thr_delta = -pos_thr
            # (0,ROC) + (ROC*sin(thr_delta),-ROC*cos(thr_delta))
        x_delta = 0.0 + ROC*math.sin(thr_delta)
        y_delta = ROC - ROC*math.cos(thr_delta)
    
    # case 4: ROC <  0 and vel_mm_sec <  0  (backward right)
    if ROC <  0 and vel_mm_sec <  0:
        thr_delta = pos_thr
            # (0,ROC) + (ROC*sin(thr_delta),-ROC*cos(thr_delta))
        x_delta = 0.0 + ROC*math.sin(thr_delta)
        y_delta = ROC - ROC*math.cos(thr_delta)

    
    return (x_delta, y_delta, thr_delta)


#
# this class represents a snapshot of the robot's data
#
class SensorFrame:
    """ the sensorFrame class is really a struct whose
        fields are filled in by sensorStatus
    """
    
    def __init__(self):
        """ constructor -- set all fields to 0
            
            see interpretSensorString for details
            on all of these fields
        """
        self.casterDrop = 0
        self.leftWheelDrop = 0
        self.rightWheelDrop = 0
        self.leftBump = 0
        self.rightBump = 0
        self.wallSensor = 0
        self.leftCliff = 0
        self.frontLeftCliff = 0
        self.frontRightCliff = 0
        self.rightCliff = 0
        self.virtualWall = 0
        self.driveLeft = 0
        self.driveRight = 0
        self.mainBrush = 0
        self.vacuum = 0
        self.sideBrush = 0
        self.leftDirt = 0
        self.rightDirt = 0
        self.remoteControlCommand = 0
        self.powerButton = 0
        self.spotButton = 0
        self.cleanButton = 0
        self.maxButton = 0
        self.distance = 0
        self.rawAngle = 0
        self.angleInRadians = 0
        self.chargingState = 0
        self.voltage = 0
        self.current = 0
        self.temperature = 0
        self.charge = 0
        self.capacity = 0
    
    def __str__(self):
        """ returns a string with the information
            from this SensorFrame
        """
        # there's probably a more efficient way to do this...
        # perhaps just making it all + instead of the separate
        # += would be more efficient
        #
        # actually, we should make a list and call ''.join(list)
        # not that we will...
        #
        s = ''
        s += 'casterDrop: ' + str(self.casterDrop) + '\n'
        s += 'leftWheelDrop: ' + str(self.leftWheelDrop) + '\n'
        s += 'rightWheelDrop: ' + str(self.rightWheelDrop) + '\n'
        s += 'leftBump: ' + str(self.leftBump) + '\n'
        s += 'rightBump: ' + str(self.rightBump) + '\n'
        s += 'wallSensor: ' + str(self.wallSensor) + '\n'
        s += 'leftCliff: ' + str(self.leftCliff) + '\n'
        s += 'frontLeftCliff: ' + str(self.frontLeftCliff) + '\n'
        s += 'frontRightCliff: ' + str(self.frontRightCliff) + '\n'
        s += 'rightCliff: ' + str(self.rightCliff) + '\n'
        s += 'virtualWall: ' + str(self.virtualWall) + '\n'
        s += 'driveLeft: ' + str(self.driveLeft) + '\n'
        s += 'driveRight: ' + str(self.driveRight) + '\n'
        s += 'mainBrush: ' + str(self.mainBrush) + '\n'
        s += 'vacuum: ' + str(self.vacuum) + '\n'
        s += 'sideBrush: ' + str(self.sideBrush) + '\n'
        s += 'leftDirt: ' + str(self.leftDirt) + '\n'
        s += 'rightDirt: ' + str(self.rightDirt) + '\n'
        s += 'remoteControlCommand: ' + str(self.remoteControlCommand) + '\n'
        s += 'powerButton: ' + str(self.powerButton) + '\n'
        s += 'spotButton: ' + str(self.spotButton) + '\n'
        s += 'cleanButton: ' + str(self.cleanButton) + '\n'
        s += 'maxButton: ' + str(self.maxButton) + '\n'
        s += 'distance: ' + str(self.distance) + '\n'
        s += 'rawAngle: ' + str(self.rawAngle) + '\n'
        s += 'angleInRadians: ' + str(self.angleInRadians) + '\n'
        # no data member needed for this next line
        s += 'angleInDegrees: ' + str(math.degrees(self.angleInRadians)) + '\n'
        s += 'chargingState: ' + str(self.chargingState) + '\n'
        s += 'voltage: ' + str(self.voltage) + '\n'
        s += 'current: ' + str(self.current) + '\n'
        s += 'temperature: ' + str(self.temperature) + '\n'
        s += 'charge: ' + str(self.charge) + '\n'
        s += 'capacity: ' + str(self.capacity) + '\n'
        return s
    
    def toBinaryString(self):
        """ this converts the calling SensorFrame into a 26-byte
            string of the format the roomba sends back
        """
        # todo: handle the different subsets (frames) of sensor data
        
        # here are the 26 bytes in list form
        slist = [0]*26
        
        # First Frame
        
        # byte 0: bumps and wheeldrops
        slist[0] = self.casterDrop << 4 | \
                   self.leftWheelDrop << 3 | \
                   self.rightWheelDrop << 2 | \
                   self.leftBump << 1 | \
                   self.rightBump
        
        # byte 1: wall data
        slist[1] = self.wallSensor
        
        # byte 2: cliff left
        slist[2] = self.leftCliff
        # byte 3: cliff front left
        slist[3] = self.frontLeftCliff
        # byte 4: cliff front right
        slist[4] = self.frontRightCliff
        # byte 5: cliff right
        slist[5] = self.rightCliff
        
        # byte 6: virtual wall
        slist[6] = self.virtualWall
        
        # byte 7: motor overcurrents
        slist[7] = self.driveLeft << 4 | \
                   self.driveRight << 3 | \
                   self.mainBrush << 2 | \
                   self.vacuum << 1 | \
                   self.sideBrush
        
        # byte 8: dirt detector left
        slist[8] = self.leftDirt
        # byte 9: dirt detector left
        slist[9] = self.rightDirt
        
        # Second Frame
        
        # byte 10: remote control command
        slist[10] = self.remoteControlCommand
        
        # byte 11: buttons
        slist[11] = self.powerButton << 3 | \
                    self.spotButton << 2 | \
                    self.cleanButton << 1 | \
                    self.maxButton
        
        # bytes 12, 13: distance
        highVal, lowVal = toTwosComplement2Bytes( self.distance )
        slist[12] = highVal
        slist[13] = lowVal
        
        # bytes 14, 15: angle
        highVal, lowVal = toTwosComplement2Bytes( self.rawAngle )
        slist[14] = highVal
        slist[15] = lowVal
        
        # Third Frame
        
        # byte 16: charging state
        slist[16] = self.chargingState
        
        # bytes 17, 18: voltage
        slist[17] = (self.voltage >> 8) & 0xFF
        slist[18] = self.voltage & 0xFF
        
        # bytes 19, 20: current
        highVal, lowVal = toTwosComplement2Bytes( self.current )
        slist[19] = highVal
        slist[20] = lowVal
        
        # byte 21: temperature
        slist[21] = self.temperature
        
        # bytes 22, 23: charge
        slist[22] = (self.charge >> 8) & 0xFF
        slist[23] = self.charge & 0xFF
        
        # bytes 24, 25: capacity
        slist[24] = (self.capacity >> 8) & 0xFF
        slist[25] = self.capacity & 0xFF
        
        # convert to a string
        s = ''.join([ chr(x) for x in slist ])
        
        return s



#
# the robot class
#
class Create:
    """ the Create class is an abstraction of the iRobot Create's
        SCI interface, including communication and a bit
        of processing of the strings passed back and forth
        
        when you create an object of type Create, the code
        will try to open a connection to it - so, it will fail
        if it's not attached!
    """
    # to do: check if we can start in other modes...
    
    def __init__(self, PORT, startingMode=SAFE_MODE):
        """ the constructor which tries to open the
            connection to the robot at port PORT
        """
        # to do: find the shortest safe serial timeout value...
        # to do: use the timeout to do more error checking than
        #        is currently done...
        #
        # the -1 here is because windows starts counting from 1
        # in the hardware control panel, but not in pyserial, it seems
        
        # if PORT is the string 'simulated' (or any string for the moment)
        # we use our SRSerial class
        print 'PORT is', PORT
        if type(PORT) == type('string'):
            if PORT == 'sim':
                print 'In simulated mode...'
                self.ser = 'sim'; # SRSerial('mapSquare.txt')
            else:
                # for Mac/Linux - use whole port name
                # print 'In Mac/Linux mode...'
                self.ser = serial.Serial(PORT, baudrate=57600, timeout=0.5)
        # otherwise, we try to open the numeric serial port...
        else:
            # print 'In Windows mode...'
            self.ser = serial.Serial(PORT-1, baudrate=57600, timeout=0.5)
        
        # did the serial port actually open?
        if self.ser != 'sim' and self.ser.isOpen():
            print 'Serial port did open, presumably to a roomba...'
        else:
            print 'Serial port did NOT open, check the'
            print '  - port number'
            print '  - physical connection'
            print '  - baud rate of the roomba (it\'s _possible_, if unlikely,'
            print '              that it might be set to 19200 instead'
            print '              of the default 57600 - removing and'
            print '              reinstalling the battery should reset it.'
        
        # our OI mode
        self.sciMode = OFF_MODE

        # our sensor dictionary, currently empty
        self.sensord = {}
        
        # here are the variables that constitute the robot's
        # estimated odometry, thr is theta in radians...
        # these are updated by integrateNextOdometricStep
        # which is called in interpretSensorString
        self.xPose =   0.0
        self.yPose =   0.0
        self.thrPose = 0.0
        
        time.sleep(0.3)
        self.start()  # go to passive mode - want to do this
        # regardless of the final mode we'd like to be in...
        time.sleep(0.3)
        
        if (startingMode == SAFE_MODE):
            print 'Putting the robot into safe mode...'
            self.toSafeMode()
        
        if (startingMode == FULL_MODE):
            print 'Putting the robot into full mode...'
            self.toSafeMode()
            time.sleep(0.3)
            self.toFullMode()
        
        # We need to read the angle and distance sensors so that
        # their values clear out!
        time.sleep(0.25)
        #self.sensors(6) # read all sensors to establish the sensord dictionary
        self.setPose(0,0,0)
            

    def sendRawBytes(self, arr):
        for x in arr:
            self.ser.write(chr(int(x)))
        return "Data processed"

    def getPose(self, dist='cm', angle='deg'):
        """ getPose returns the current estimate of the
            robot's global pose
            dist may be 'cm' or 'mm'
            angle may be 'deg' or 'rad'
        """
        x = 0; y = 0; th = 0
        if dist == 'cm':
            x = self.xPose/10.0; y = self.yPose/10.0
        else:
            x = self.xPose; y = self.yPose
            
        if angle == 'deg':
            th = math.degrees(self.thrPose)
        else:
            th = self.thrPose
            
        return (x,y,th)
            
            
    def setPose(self, x, y, th, dist='cm', angle='deg'):
        """ setPose sets the internal odometry to the input values
              x: global x in mm
              y: global y in mm
              th: global th in radians
              dist: 'cm' or 'mm' for x and y
              angle: 'deg' or 'rad' for th
        """
        if dist == 'cm':
            self.xPose = x*10.0; self.yPose = y*10.0
        else:
            self.xPose = x; self.yPose = y
            
        if angle == 'deg':
            self.thrPose = math.radians(th)
        else:
            self.thrPose = th
    
    
    def resetPose(self):
        """ resetPose simply sets the internal odometry to 0,0,0
        """
        self.setPose(0.0,0.0,0.0)
    
    def integrateNextOdometricStepCreate(self, distance, rawAngle):
        """ integrateNextOdometricStep adds the reported inputs
              distance in mm
              rawAngle in degrees
            to the estimate of the robot's global pose
        """
        # OK, so this _should_ be easy
        # distance is, supposedly, the arc length that the center
        #              of the robot has traveled (the average of
        #              the two wheel's linear distances)
        #
        # rawAngle is, supposedly, the (RightWheel-LeftWheel)/2.0
        #
        # the distance (diameter) between the two wheels is 258mm
        #     keep in mind that the robot's physical diameter is larger ~
        #
        # 0.5*258 == 129mm radius
        #
        # perhaps there's nothing to do...
        if distance == 0 and rawAngle == 0:
            return
        
        # then again, mayber there is something to do...
        dthr = math.radians(rawAngle)  # angle traveled
        d = distance              # distance traveled
        # compute offsets in the local coordinate system,
        # with the x-axis pointing in the direction the robot was
        # initially facing (for this move) and the y-axis pointing
        # perpendicularly to the left (when facing forward)
        #
        # first, the special case when the angle is zero...
        if rawAngle == 0:
            dx = float(d)
            dy = 0.0
        # or if the distance is zero...
        elif distance == 0:
            dx = 0.0
            dy = 0.0
        # or when neither is zero...
        else:
            # finite radius of curvature
            ROC = float(d)/dthr   # remember, this is signed!
            dx = ROC*math.sin(dthr)       # because ROC is signed,
            dy =  ROC-ROC*math.cos(dthr)  # we don't need two cases
        #
        # we need to add dx, dy, and dthr to the global pose
        # and so we need to do so in the global direction in
        # which the robot was facing at the start of this movement
        #
        # here is the unit vector describing that direction
        unitForwardX = math.cos( self.thrPose )
        unitForwardY = math.sin( self.thrPose )
        # here is the unit vector perpendicular to the left
        unitPerpX = math.cos( self.thrPose + math.pi/2.0 )
        unitPerpY = math.sin( self.thrPose + math.pi/2.0 )
        # now we compute our global offsets
        dx_global = dx*unitForwardX + dy*unitPerpX
        dy_global = dx*unitForwardY + dy*unitPerpY
        ##print 'distance and rawAngle', distance, rawAngle
        ##print 'local offsets, x, y, thd', dx, dy, math.degrees(dthr)
        ##print 'global offsets, x, y, thd', dx_global, dy_global, math.degrees(dthr)
        # and we add them all in...
        self.xPose += dx_global
        self.yPose += dy_global
        self.thrPose += dthr
        #print 'final pose', self.xPose, self.yPose, self.thrPose
        return
    
    def integrateNextOdometricStepRoomba(self, distance, rawAngle):
        """ integrateNextOdometricStep adds the reported inputs
              distance in mm
              rawAngle in mm
            to the estimate of the robot's global pose
        """
        # OK, so this _should_ be easy
        # distance is, supposedly, the arc length that the center
        #              of the robot has traveled (the average of
        #              the two wheel's linear distances)
        #
        # rawAngle is, supposedly, the (RightWheel-LeftWheel)/2.0
        #
        # the distance (diameter) between the two wheels is 258mm
        #     keep in mind that the robot's physical diameter is larger ~
        #
        # 0.5*258 == 129mm radius
        #
        # perhaps there's nothing to do...
        if distance == 0 and rawAngle == 0:
            return
        
        # then again, mayber there is something to do...
        dthr = rawAngle / 129.0   # angle traveled in radians
        d = distance              # distance traveled
        # compute offsets in the local coordinate system,
        # with the x-axis pointing in the direction the robot was
        # initially facing (for this move) and the y-axis pointing
        # perpendicularly to the left (when facing forward)
        #
        # first, the special case when the angle is zero...
        if rawAngle == 0:
            dx = float(d)
            dy = 0.0
        # or if the distance is zero...
        elif distance == 0:
            dx = 0.0
            dy = 0.0
        # or when neither is zero...
        else:
            # finite radius of curvature
            ROC = float(d)/dthr   # remember, this is signed!
            dx = ROC*math.sin(dthr)       # because ROC is signed,
            dy =  ROC-ROC*math.cos(dthr)  # we don't need two cases
        #
        # we need to add dx, dy, and dthr to the global pose
        # and so we need to do so in the global direction in
        # which the robot was facing at the start of this movement
        #
        # here is the unit vector describing that direction
        unitForwardX = math.cos( self.thrPose )
        unitForwardY = math.sin( self.thrPose )
        # here is the unit vector perpendicular to the left
        unitPerpX = math.cos( self.thrPose + math.pi/2.0 )
        unitPerpY = math.sin( self.thrPose + math.pi/2.0 )
        # now we compute our global offsets
        dx_global = dx*unitForwardX + dy*unitPerpX
        dy_global = dx*unitForwardY + dy*unitPerpY
        ##print 'distance and rawAngle', distance, rawAngle
        ##print 'local offsets, x, y, thd', dx, dy, math.degrees(dthr)
        ##print 'global offsets, x, y, thd', dx_global, dy_global, math.degrees(dthr)
        # and we add them all in...
        self.xPose += dx_global
        self.yPose += dy_global
        self.thrPose += dthr
        #print 'final pose', self.xPose, self.yPose, self.thrPose
        return
    
    def setWheelVelocities( self, left_cm_sec, right_cm_sec ):
        """ sends velocities of each wheel independently
               left_cm_sec:  left  wheel velocity in cm/sec (capped at +- 50)
               right_cm_sec: right wheel velocity in cm/sec (capped at +- 50)
        """
        if left_cm_sec < -50: left_cm_sec = -50;
        if left_cm_sec > 50:  left_cm_sec = 50;
        if right_cm_sec < -50: right_cm_sec = -50;
        if left_cm_sec > 50: left_cm_sec = 50;
        # convert to mm/sec, ensure we have integers
        leftHighVal, leftLowVal = toTwosComplement2Bytes( int(left_cm_sec*10) )
        rightHighVal, rightLowVal = toTwosComplement2Bytes( int(right_cm_sec*10) )
        # send these bytes and set the stored velocities
        self.ser.write( DRIVEDIRECT )
        self.ser.write( chr(rightHighVal) )
        self.ser.write( chr(rightLowVal) )
        self.ser.write( chr(leftHighVal) )
        self.ser.write( chr(leftLowVal) )
    
    def Stop(self):
        """ Stop calls Go(0,0) """
        self.Go(0,0)
    
    def stop(self):
        """ stop calls go(0,0) """
        self.Stop()
    
    def go(self, cm_per_sec=0, deg_per_sec=0 ):
        """ go(cmpsec, degpsec) sets the robot's velocity to
               cmpsec centimeters per second
               degpsec degrees per second
            go() is equivalent to go(0,0)
        """
        self.Go( cm_per_sec, deg_per_sec )
    
    def Go( self, cm_per_sec=0, deg_per_sec=0 ):
        """ Go(cmpsec, degpsec) sets the robot's velocity to
               cmpsec centimeters per second
               degpsec degrees per second
            Go() is equivalent to go(0,0)
        """
        # need to convert to the roomba's drive parameters
        #
        # for now, just one or the other...
        if cm_per_sec == 0:
            # just handle rotation
            # convert to radians
            rad_per_sec = math.radians(deg_per_sec)
            # make sure the direction is correct
            if rad_per_sec >= 0:  dirstr = 'CCW'
            else: dirstr = 'CW'
            # compute the velocity, given that the robot's
            # radius is 258mm/2.0
            vel_mm_sec = math.fabs(rad_per_sec) * (258.0/2.0)
            # send it off to the robot
            self.drive( vel_mm_sec, 0, dirstr )
        
        elif deg_per_sec == 0:
            # just handle forward/backward translation
            vel_mm_sec = 10.0*cm_per_sec
            big_radius = 32767
            # send it off to the robot
            self.drive( vel_mm_sec, big_radius )
        
        else:
            # move in the appropriate arc
            rad_per_sec = math.radians(deg_per_sec)
            vel_mm_sec = 10.0*cm_per_sec
            radius_mm = vel_mm_sec / rad_per_sec
            # check for extremes
            if radius_mm > 32767: radius_mm = 32767
            if radius_mm < -32767: radius_mm = -32767
            self.drive( vel_mm_sec, radius_mm )
        
        return
    
    def start(self):
        """ changes from OFF_MODE to PASSIVE_MODE """
        self.ser.write( START )
        # they recommend 20 ms between mode-changing commands
        time.sleep(0.25)
        # change the mode we think we're in...
        return
    
    def turnOnRooToothPower(self):
        """ the RooTooth Bluetooth radio has its PIO pin 6
            attached to the RTS serial line for waking the
            roomba up...
        """
        self.ser.write('+++\r')  # go to AT mode
        self.ser.readline()      # reads the '\r\n'
        self.ser.readline()      # reads the 'OK\r\n'
        self.ser.write('ATSW22,6,1,1\r')  # set pin 6 to output
        self.ser.readline()      # reads the '\r\n'
        self.ser.readline()      # reads the 'OK\r\n'
        self.ser.write('ATSW23,6,0,1\r')  # set pin 6 to output
        self.ser.readline()      # reads the '\r\n'
        self.ser.readline()      # reads the 'OK\r\n'
        self.ser.write('ATSW23,6,1,1\r')  # set pin 6 to output
        self.ser.readline()      # reads the '\r\n'
        self.ser.readline()      # reads the 'OK\r\n'
        self.ser.write('ATMD\r')  # set pin 6 to output
        self.ser.readline()      # reads the '\r\n'
        self.ser.readline()      # reads the 'OK\r\n'
    
    def close(self):
        """ tries to shutdown the robot as kindly as possible, by
            clearing any remaining odometric data
            going to passive mode
            closing the serial port
        """
        # is there other clean up to be done?
        # let's get rid of any lingering odometric data
        # we don't call getSensorList, because we don't want to integrate the odometry...
        self.getRawSensorDataAsList( [19,20] )
        time.sleep(0.1)
        self.start()       # send Create back to passive mode
        time.sleep(0.1)
        self.ser.close()
        return
    
    def _closeSer(self):
        """ just disconnects the serial port """
        self.ser.close()
        return
    
    def _openSer(self):
        """ opens the port again """
        self.ser.open()
        return

    def drive(self, roomba_mm_sec, roomba_radius_mm, turn_dir='CCW'):
        """ implements the drive command as specified
            the turn_dir should be either 'CW' or 'CCW' for
            clockwise or counterclockwise - this is only
            used if roomba_radius_mm == 0 (or rounds down to 0)
            other drive-related calls are available
        """
        # first, they should be ints
        #   in case they're being generated mathematically
        if type(roomba_mm_sec) != type(42):
            roomba_mm_sec = int(roomba_mm_sec)
        if type(roomba_radius_mm) != type(42):
            roomba_radius_mm = int(roomba_radius_mm)
        
        # we check that the inputs are within limits
        # if not, we cap them there
        if roomba_mm_sec < -500:
            roomba_mm_sec = -500
        if roomba_mm_sec > 500:
            roomba_mm_sec = 500
        
        # if the radius is beyond the limits, we go straight
        # it doesn't really seem to go straight, however...
        if roomba_radius_mm < -2000:
            roomba_radius_mm = 32768
        if roomba_radius_mm > 2000:
            roomba_radius_mm = 32768
        
        # get the two bytes from the velocity
        # these come back as numbers, so we will chr them
        velHighVal, velLowVal = toTwosComplement2Bytes( roomba_mm_sec )
        
        # get the two bytes from the radius in the same way
        # note the special cases
        if roomba_radius_mm == 0:
            if turn_dir == 'CW':
                roomba_radius_mm = -1
            else: # default is 'CCW' (turning left)
                roomba_radius_mm = 1
        radiusHighVal, radiusLowVal = toTwosComplement2Bytes( roomba_radius_mm )
        
        #print 'bytes are', velHighVal, velLowVal, radiusHighVal, radiusLowVal
        
        # send these bytes and set the stored velocities
        self.ser.write( DRIVE )
        self.ser.write( chr(velHighVal) )
        self.ser.write( chr(velLowVal) )
        self.ser.write( chr(radiusHighVal) )
        self.ser.write( chr(radiusLowVal) )

    
    def setLEDs(self, power_color, power_intensity, play, advance ):
        """ The setLEDs method sets each of the three LEDs, from left to right:
            the power LED, the play LED, and the status LED.
            The power LED at the left can display colors from green (0) to red (255)
            and its intensity can be specified, as well. Hence, power_color and
            power_intensity are values from 0 to 255. The other two LED inputs
            should either be 0 (off) or 1 (on).
        """
        # make sure we're within range...
        if advance != 0: advance = 1
        if play != 0: play = 1
        try:
            power = int(power_intensity)
            powercolor = int(power_color)
        except TypeError:
            power = 128
            powercolor = 128
            print 'Type excpetion caught in setAbsoluteLEDs in roomba.py'
            print 'Your power_color or power_intensity was not of type int.'
        if power < 0: power = 0
        if power > 255: power = 255
        if powercolor < 0: powercolor = 0
        if powercolor > 255: powercolor = 255
        # create the first byte
        #firstByteVal = (status << 4) | (spot << 3) | (clean << 2) | (max << 1) | dirtdetect
        firstByteVal =  (advance << 3) | (play << 1) 
        
        # send these as bytes
        # print 'bytes are', firstByteVal, powercolor, power
        self.ser.write( LEDS )
        self.ser.write( chr(firstByteVal) )
        self.ser.write( chr(powercolor) )
        self.ser.write( chr(power) )
        
        return

    
    #
    # DO NOT CALL THIS FUNCTION!
    #    call readSensors instead - it will integrate odometry
    #    for what that's worth, admittedly...
    #    if you call this without integrating odometry, the
    #    distance and rawAngle reported will be lost...
    #
    def getRawSensorFrameAsList(self, packetnumber):
        """ gets back a raw string of sensor data
            which then can be used to create a SensorFrame
        """
        if type(packetnumber) != type(1):
            packetnumber = 6
        
        if packetnumber < 0 or packetnumber > 6:
            packetnumber = 6
        
        self.ser.write( SENSORS )
        self.ser.write( chr(packetnumber) )
        
        if packetnumber == 0:
            r = self.ser.read(size=26)
        if packetnumber == 1:
            r = self.ser.read(size=10)
        if packetnumber == 2:
            r = self.ser.read(size=6)
        if packetnumber == 3:
            r = self.ser.read(size=10)
        if packetnumber == 4:
            r = self.ser.read(size=14)
        if packetnumber == 5:
            r = self.ser.read(size=12)
        if packetnumber == 6:
            r = self.ser.read(size=52)
        
        r = [ ord(c) for c in r ]   # convert to ints
        return r
        
    
    def getRawSensorDataAsList(self, listofsensors):
        """ gets the chosen sensors
            and returns the raw bytes, as a string
            needs to be converted to integers...
        """
        numberOfSensors = len(listofsensors)
        self.ser.write( QUERYLIST )
        self.ser.write( chr(numberOfSensors) )
        resultLength = 0
        for sensornum in listofsensors:
            self.ser.write( chr(sensornum) )
            resultLength += SENSOR_DATA_WIDTH[sensornum]

        r = self.ser.read(size=resultLength)
        r = [ ord(c) for c in r ]   # convert to ints
        #print 'r is ', r
        return r

    
    def seekDock(self):
        """ sends the force-seeking-dock signal
        """
        self.demo(1)

    
    def demo(self, demoNumber=-1):
        """ runs one of the built-in demos for Create
            if demoNumber is
              <omitted> or
              -1 stop current demo
               0 wander the surrounding area
               1 wander and dock, when the docking station is seen
               2 wander a more local area
               3 wander to a wall and then follow along it
               4 figure 8
               5 "wimp" demo: when pushed, move forward
                 when bumped, move back and away
               6 home: will home in on a virtual wall, as
                 long as the back and sides of the IR receiver
                 are covered with tape
               7 tag: homes in on sequential virtual walls
               8 pachelbel: plays the first few notes of the canon in D
               9 banjo: plays chord notes according to its cliff sensors
                 chord key is selected via the bumper
        """
        if (demoNumber < -1 or demoNumber > 9):
            demoNumber = -1 # stop current demo
        
        self.ser.write( DEMO )
        if demoNumber < 0 or demoNumber > 9:
            # invalid values are equivalent to stopping
            self.ser.write( chr(255) ) # -1
        else:
            self.ser.write( chr(demoNumber) )

    
    def setSong(self, songNumber, songDataList):
        """ this stores a song to roomba's memory to play later
            with the playSong command
            
            songNumber must be between 0 and 15 (inclusive)
            songDataList is a list of (note, duration) pairs (up to 16)
                         note is the midi note number, from 31 to 127
                              (outside this range, the note is a rest)
                         duration is from 0 to 255 in 1/64ths of a second
        """
        # any notes to play?
        if type(songDataList) != type([]) and type(songDataList) != type(()):
            print 'songDataList was', songDataList
            return 
            
        if len(songDataList) < 1:
            print 'No data in the songDataList'
            return
        
        if songNumber < 0: songNumber = 0
        if songNumber > 15: songNumber = 15
        
        # indicate that a song is coming
        self.ser.write( SONG )
        self.ser.write( chr(songNumber) )
        
        L = min(len(songDataList), 16)
        self.ser.write( chr(L) )
        
        # loop through the notes, up to 16
        for note in songDataList[:L]:
            # make sure its a tuple, or else we rest for 1/4 second
            if type(note) == type( () ):
                #more error checking here!
                self.ser.write( chr(note[0]) )  # note number
                self.ser.write( chr(note[1]) )  # duration
            else:
                self.ser.write( chr(30) )   # a rest note
                self.ser.write( chr(16) )   # 1/4 of a second
                
        return


    def playSong(self, list_of_notes):
        """ The input to <tt>playSong</tt> should be specified as a list
            of pairs of [ note_number, note_duration ] format. Thus, 
            r.playSong( [(60,8),(64,8),(67,8),(72,8)] ) plays a quick C chord.
        """
        # implemented by setting song #1 to the notes and then playing it
        self.setSong(1, list_of_notes)
        self.playSongNumber(1)

    
    def playSongNumber(self, songNumber):
        """ plays song songNumber """
        if songNumber < 0: songNumber = 0
        if songNumber > 15: songNumber = 15
        
        self.ser.write( PLAY )
        self.ser.write( chr(songNumber) )
        
    
    def playNote(self, noteNumber, duration, songNumber=0):
        """ plays a single note as a song (at songNumber)
            duration is in 64ths of a second (1-255)
            the note number chart is on page 12 of the open interface manual
        """
        # set the song
        self.setSong(songNumber, [(noteNumber,duration)])
        self.playSongNumber(songNumber)
     
    
    def _getLower5Bits( self, r ):
        """ r is one byte as an integer """
        return [ bitOfByte(4,r), bitOfByte(3,r), bitOfByte(2,r), bitOfByte(1,r), bitOfByte(0,r) ]
    
    def _getOneBit( self, r ):
        """ r is one byte as an integer """
        if r == 1:  return 1
        else:       return 0
    
    def _getOneByteUnsigned( self, r ):
        """ r is one byte as an integer """
        return r
    
    def _getOneByteSigned( self, r ):
        """ r is one byte as a signed integer """
        return twosComplementInt1byte( r )
    
    def _getTwoBytesSigned( self, r1, r2 ):
        """ r1, r2 are two bytes as a signed integer """
        return twosComplementInt2bytes( r1, r2 )
    
    def _getTwoBytesUnsigned( self, r1, r2 ):
        """ r1, r2 are two bytes as an unsigned integer """
        return r1 << 8 | r2
    
    def _getButtonBits( self, r ):
        """ r is one byte as an integer """
        return [ bitOfByte(2,r), bitOfByte(0,r) ]
        
        
    def setNextDataFrame(self):
        """ This function _asks_ the robot to collect ALL of
            the sensor data into the next packet to send back.
        """
        self.ser.write( SENSORS )
        self.ser.write( chr(6) )
        
    def getNextDataFrame(self):
        """ This function then gets back ALL of
            the sensor data and organizes it into the sensor 
            dictionary, sensord.
        """
        r = self.ser.read(size=52)
        r = [ ord(c) for c in r ]
        #return self.readSensorList(r)
    
    def _rawSend( self, listofints ):
        for x in listofints:
            self.ser.write( chr(x) )
    
    def _rawRecv( self ):
        nBytesWaiting = self.ser.inWaiting()
        #print 'nBytesWaiting is', nBytesWaiting
        r = self.ser.read(size=nBytesWaiting)
        r = [ ord(x) for x in r ]
        #print 'r is', r
        return r
    
    def _rawRecvStr( self ):
        nBytesWaiting = self.ser.inWaiting()
        #print 'nBytesWaiting is', nBytesWaiting
        r = self.ser.read(size=nBytesWaiting)
        return r
        
    def sensors( self, list_of_sensors_to_poll=6 ):
        """ this function updates the robot's currently maintained
            state of its robot sensors for those sensors requested
            If none are requested, then all of the sensors are updated
            (which takes a bit more time...)
        """
        if type(list_of_sensors_to_poll) == type([]):
            # first, we change any pieces of sensor values to
            # the single digit that is required here
            distangle = 0
            if POSE in list_of_sensors_to_poll:
                list_of_sensors_to_poll.remove(POSE)
                # should check if they're already there
                list_of_sensors_to_poll.append(DISTANCE)
                list_of_sensors_to_poll.append(ANGLE)
                
            if LEFT_BUMP in list_of_sensors_to_poll:
                list_of_sensors_to_poll.remove(LEFT_BUMP)
                if BUMPS_AND_WHEEL_DROPS not in list_of_sensors_to_poll:
                    list_of_sensors_to_poll.append(BUMPS_AND_WHEEL_DROPS)

            if RIGHT_BUMP in list_of_sensors_to_poll:
                list_of_sensors_to_poll.remove(RIGHT_BUMP)
                if BUMPS_AND_WHEEL_DROPS not in list_of_sensors_to_poll:
                    list_of_sensors_to_poll.append(BUMPS_AND_WHEEL_DROPS)
                    
            if RIGHT_WHEEL_DROP in list_of_sensors_to_poll:
                list_of_sensors_to_poll.remove(RIGHT_WHEEL_DROP)
                if BUMPS_AND_WHEEL_DROPS not in list_of_sensors_to_poll:
                    list_of_sensors_to_poll.append(BUMPS_AND_WHEEL_DROPS)
                    
            if LEFT_WHEEL_DROP in list_of_sensors_to_poll:
                list_of_sensors_to_poll.remove(LEFT_WHEEL_DROP)
                if BUMPS_AND_WHEEL_DROPS not in list_of_sensors_to_poll:
                    list_of_sensors_to_poll.append(BUMPS_AND_WHEEL_DROPS) 
                       
            if CENTER_WHEEL_DROP in list_of_sensors_to_poll:
                list_of_sensors_to_poll.remove(LEFT_WHEEL_DROP)
                if BUMPS_AND_WHEEL_DROPS not in list_of_sensors_to_poll:
                    list_of_sensors_to_poll.append(BUMPS_AND_WHEEL_DROPS)
                    
            if LEFT_WHEEL_OVERCURRENT in list_of_sensors_to_poll:
                list_of_sensors_to_poll.remove(LEFT_WHEEL_OVERCURRENT)
                if LSD_AND_OVERCURRENTS not in list_of_sensors_to_poll:
                    list_of_sensors_to_poll.append(LSD_AND_OVERCURRENTS)
                    
            if RIGHT_WHEEL_OVERCURRENT in list_of_sensors_to_poll:
                list_of_sensors_to_poll.remove(RIGHT_WHEEL_OVERCURRENT)
                if LSD_AND_OVERCURRENTS not in list_of_sensors_to_poll:
                    list_of_sensors_to_poll.append(LSD_AND_OVERCURRENTS)
                    
            if ADVANCE_BUTTON in list_of_sensors_to_poll:
                list_of_sensors_to_poll.remove(ADVANCE_BUTTON)
                if BUTTONS not in list_of_sensors_to_poll:
                    list_of_sensors_to_poll.append(BUTTONS)
            
            if PLAY_BUTTON in list_of_sensors_to_poll:
                list_of_sensors_to_poll.remove(PLAY_BUTTON)
                if BUTTONS not in list_of_sensors_to_poll:
                    list_of_sensors_to_poll.append(BUTTONS)
                    
            r = self.getRawSensorDataAsList(list_of_sensors_to_poll)

        else:
            # if it's an integer, its a frame number
            r = self.getRawSensorFrameAsList( list_of_sensors_to_poll )
            # now, we set list_of_sensors_to_poll
            frameNumber = list_of_sensors_to_poll
            if frameNumber == 0:
                list_of_sensors_to_poll = range(7,27)
            elif frameNumber == 1:
                list_of_sensors_to_poll = range(7,17)
            elif frameNumber == 2:
                list_of_sensors_to_poll = range(17,21)
            elif frameNumber == 3:
                list_of_sensors_to_poll = range(21,27)
            elif frameNumber == 4:
                list_of_sensors_to_poll = range(27,35)
            elif frameNumber == 5:
                list_of_sensors_to_poll = range(35,43)
            else:
                list_of_sensors_to_poll = range(7,43)
         
        # change our dictionary
        self.readSensorList(list_of_sensors_to_poll, r)      
        return self.sensord
        
    def printSensors(self, d=None):
        """ convenience function to show sensed data in d 
            if d is None, the current self.sensord is used instead
        """
        if not d:
            d = self.sensord
            
        pose = d[POSE]
            
        print '                   LEFT_BUMP:', d[LEFT_BUMP]
    	print '                  RIGHT_BUMP:', d[RIGHT_BUMP]
    	print '             LEFT_WHEEL_DROP:', d[LEFT_WHEEL_DROP]
    	print '            RIGHT_WHEEL_DROP:', d[RIGHT_WHEEL_DROP]
    	print '           CENTER_WHEEL_DROP:', d[CENTER_WHEEL_DROP]
    	print '              WALL_IR_SENSOR:', d[WALL_IR_SENSOR]
    	print '                  CLIFF_LEFT:', d[CLIFF_LEFT]
    	print '            CLIFF_FRONT_LEFT:', d[CLIFF_FRONT_LEFT]
    	print '           CLIFF_FRONT_RIGHT:', d[CLIFF_FRONT_RIGHT]
    	print '                 CLIFF_RIGHT:', d[CLIFF_RIGHT]
    	print '                VIRTUAL_WALL:', d[VIRTUAL_WALL]
    	print '      LEFT_WHEEL_OVERCURRENT:', d[LEFT_WHEEL_OVERCURRENT]
    	print '     RIGHT_WHEEL_OVERCURRENT:', d[RIGHT_WHEEL_OVERCURRENT]
    	print '               INFRARED_BYTE:', d[INFRARED_BYTE]
    	print '                 PLAY_BUTTON:', d[PLAY_BUTTON]
    	print '              ADVANCE_BUTTON:', d[ADVANCE_BUTTON]
    	print '                 POSE X (cm):', pose[0]
    	print '                 POSE Y (cm):', pose[1]
    	print '               POSE TH (deg):', pose[2]
    	print '              CHARGING_STATE:', d[CHARGING_STATE]
    	print '                     VOLTAGE:', d[VOLTAGE]
    	print '                     CURRENT:', d[CURRENT]
    	print '                BATTERY_TEMP:', d[BATTERY_TEMP]
    	print '              BATTERY_CHARGE:', d[BATTERY_CHARGE]
    	print '            BATTERY_CAPACITY:', d[BATTERY_CAPACITY]
    	print '                 WALL_SIGNAL:', d[WALL_SIGNAL]
    	print '           CLIFF_LEFT_SIGNAL:', d[CLIFF_LEFT_SIGNAL]
    	print '     CLIFF_FRONT_LEFT_SIGNAL:', d[CLIFF_FRONT_LEFT_SIGNAL]
    	print '    CLIFF_FRONT_RIGHT_SIGNAL:', d[CLIFF_FRONT_RIGHT_SIGNAL]
    	print '          CLIFF_RIGHT_SIGNAL:', d[CLIFF_RIGHT_SIGNAL]
    	print '                     OI_MODE:', d[OI_MODE]
    	print '                 SONG_NUMBER:', d[SONG_NUMBER]
    	print '                SONG_PLAYING:', d[SONG_PLAYING]
    	print '  CHARGING_SOURCES_AVAILABLE:', d[CHARGING_SOURCES_AVAILABLE]

            
        
    
    def readSensorList(self, sensor_data_list, r):
        """ this returns the latest values from the particular
            sensors requested in the listofvalues
        """
        
        if len(sensor_data_list) == 0:
            print 'No data was read in readSensorList.'
            return self.sensord
        
        sensorDataInterpreter = [ None, # 0
                                  None, # 1
                                  None, # 2
                                  None, # 3
                                  None, # 4
                                  None, # 5
                                  None, # 6
                                  self._getLower5Bits, # 7 BUMPS_AND_WHEEL_DROPS
                                  self._getOneBit, # 8 WALL_IR_SENSOR
                                  self._getOneBit, # 9 CLIFF_LEFT = 9
                                  self._getOneBit, # 10 CLIFF_FRONT_LEFT = 10
                                  self._getOneBit, # 11 CLIFF_FRONT_RIGHT = 11
                                  self._getOneBit, # 12 CLIFF_RIGHT = 12
                                  self._getOneBit, # 13 VIRTUAL_WALL
                                  self._getLower5Bits, # 14 LSD_AND_OVERCURRENTS
                                  self._getOneBit, # 15 unused
                                  self._getOneBit, # 16 unused
                                  self._getOneByteUnsigned, # 17 INFRARED_BYTE
                                  self._getButtonBits, # 18 BUTTONS
                                  self._getTwoBytesSigned, # 19 DISTANCE
                                  self._getTwoBytesSigned, # 20 ANGLE
                                  self._getOneByteUnsigned, # 21 CHARGING_STATE
                                  self._getTwoBytesUnsigned, # 22 VOLTAGE
                                  self._getTwoBytesSigned, # 23 CURRENT
                                  self._getOneByteSigned, # 24 BATTERY_TEMP
                                  self._getTwoBytesUnsigned, # 25 BATTERY_CHARGE
                                  self._getTwoBytesUnsigned, # 26 BATTERY_CAPACITY
                                  self._getTwoBytesUnsigned, # 27 WALL_SIGNAL
                                  self._getTwoBytesUnsigned, # 28 CLIFF_LEFT_SIGNAL
                                  self._getTwoBytesUnsigned, # 29 CLIFF_FRONT_LEFT_SIGNAL
                                  self._getTwoBytesUnsigned, # 30 CLIFF_FRONT_RIGHT_SIGNAL
                                  self._getTwoBytesUnsigned, # 31 CLIFF_RIGHT_SIGNAL
                                  self._getLower5Bits, # 32 CARGO_BAY_DIGITAL_INPUTS
                                  self._getTwoBytesUnsigned, # 33 CARGO_BAY_ANALOG_SIGNAL
                                  self._getOneByteUnsigned, # 34 CHARGING_SOURCES_AVAILABLE
                                  self._getOneByteUnsigned, # 35 OI_MODE
                                  self._getOneByteUnsigned, # 36 SONG_NUMBER
                                  self._getOneByteUnsigned, # 37 SONG_PLAYING
                                  self._getOneByteUnsigned, # 38 NUM_STREAM_PACKETS
                                  self._getTwoBytesSigned, # 39 REQUESTED_VELOCITY
                                  self._getTwoBytesSigned, # 40 REQUESTED_RADIUS
                                  self._getTwoBytesSigned, # 41 REQUESTED_RIGHT_VELOCITY
                                  self._getTwoBytesSigned, # 42 REQUESTED_LEFT_VELOCITY
                                  None # only 42 as of right now
                                  ]
        
        startofdata = 0
        distance = 0
        angle = 0
        for sensorNum in sensor_data_list:
            width = SENSOR_DATA_WIDTH[sensorNum]
            dataGetter = sensorDataInterpreter[sensorNum]
            interpretedData = 0
            
            if (width == 1):
                interpretedData = dataGetter(r[startofdata])
            if (width == 2):
                interpretedData = dataGetter(r[startofdata], r[startofdata+1] )
                
            # add to our dictionary
            self.sensord[sensorNum] = interpretedData
            
            # POSE = 100 - later
            
            #LEFT_BUMP = 101
            #RIGHT_BUMP = 102
            #LEFT_WHEEL_DROP = 103
            #RIGHT_WHEEL_DROP = 104
            #CENTER_WHEEL_DROP = 105
            if sensorNum == BUMPS_AND_WHEEL_DROPS:
                self.sensord[CENTER_WHEEL_DROP] = interpretedData[0]
                self.sensord[LEFT_WHEEL_DROP] = interpretedData[1]
                self.sensord[RIGHT_WHEEL_DROP] = interpretedData[2]
                self.sensord[LEFT_BUMP] = interpretedData[3]
                self.sensord[RIGHT_BUMP] = interpretedData[4]
                
            #LEFT_WHEEL_OVERCURRENT = 106
            #RIGHT_WHEEL_OVERCURRENT = 107
            if sensorNum == LSD_AND_OVERCURRENTS:
                self.sensord[LEFT_WHEEL_OVERCURRENT] = interpretedData[0]
                self.sensord[RIGHT_WHEEL_OVERCURRENT] = interpretedData[1]
                
            #ADVANCE_BUTTON = 108
            #PLAY_BUTTON = 109
            if sensorNum == BUTTONS:
                self.sensord[ADVANCE_BUTTON] = interpretedData[0]
                self.sensord[PLAY_BUTTON] = interpretedData[1]
            
            # handle special cases
            if (sensorNum == DISTANCE):
                distance = interpretedData
            if (sensorNum == ANGLE):
                angle = interpretedData
                
            #resultingValues.append(interpretedData)
            # update index for next sensor...
            startofdata = startofdata + width
        
        if (distance != 0 or angle != 0):
            self.integrateNextOdometricStepCreate(distance,angle)
            
        self.sensord[POSE] = self.getPose(dist='cm',angle='deg')



    def getData(self):
        """ for class """
        s = self.readSensors()
        return (s.distance, math.degrees(s.angleInRadians), s.leftBump, s.rightBump)
        
    
    def toFullMode(self):
        """ changes the state to FULL_MODE
        """
        self.start()
        time.sleep(0.03)
        self.toSafeMode()
        time.sleep(0.03)
        self.ser.write( FULL )
        time.sleep(0.03)
        self.sciMode = FULL_MODE
        
        return

    
    def toSafeMode(self):
        """ changes the state (from PASSIVE_MODE or FULL_MODE)
            to SAFE_MODE
        """
        self.start()
        time.sleep(0.03)
        # now we're in PASSIVE_MODE, so we repeat the above code...
        self.ser.write( SAFE )
        # they recommend 20 ms between mode-changing commands
        time.sleep(0.03)
        # change the mode we think we're in...
        self.sciMode = SAFE_MODE
        # no response here, so we don't get any...
        return


    
    def getMode(self):
        """ returns one of OFF_MODE, PASSIVE_MODE, SAFE_MODE, FULL_MODE """
        # but how right is it?
        return self.sciMode

    
    def setBaudRate(self, baudrate=10):
        """ sets the communications rate to the desired value """
        # check for OK value
        #baudcode = 10  # 57600, the default
        if baudrate == 300: baudcode = 0
        elif baudrate == 600: baudcode = 1
        elif baudrate == 1200: baudcode = 2
        elif baudrate == 2400: baudcode = 3
        elif baudrate == 4800: baudcode = 4
        elif baudrate == 9600: baudcode = 5
        elif baudrate == 14400: baudcode = 6
        elif baudrate == 19200: baudcode = 7
        elif baudrate == 28800: baudcode = 8
        elif baudrate == 38400: baudcode = 9
        elif baudrate == 57600: baudcode = 10
        elif baudrate == 115200: baudcode = 11
        else:
            print 'The baudrate of', baudrate, 'in setBaudRate'
            print 'was not recognized. Not sending anything.'
            return
        # otherwise, send off the message
        self.ser.write( START )
        self.ser.write( chr(baudcode) )
        # the recommended pause
        time.sleep(0.1)
        # change the mode we think we're in...
        self.sciMode = PASSIVE_MODE
        # no response here, so we don't get any...
        return

    
    def interpretSensorString( self, r ):
        """ This returns a sensorFrame object with its fields
            filled in from the raw sensor return string, r, which
            has to be the full 3-packet (26-byte) string.
            
            r is obtained by writing [142][0] to the serial port.
        """
        # check length
        # we should save a bit of time by handling each sub-string
        # appropriately, but we don't do this yet...
        if len(r) != 26:
            #print 'You have input an incorrectly formatted string to'
            #print 'sensorStatus. It needs to have 26 bytes (full sensors).'
            #print 'The input is', r
            return
        
        s = SensorFrame()
        
        # convert r so that it is a list of 26 ints instead of 26 chrs
        r = [ ord(c) for c in r ]
        
        # packet number 1 (10 bytes)
        
        # byte 0: bumps and wheeldrops
        s.casterDrop = bitOfByte( 4, r[0] )
        s.leftWheelDrop = bitOfByte( 3, r[0] )
        s.rightWheelDrop = bitOfByte( 2, r[0] )
        s.leftBump = bitOfByte( 1, r[0] )
        s.rightBump = bitOfByte( 0, r[0] )
        
        # byte 1: wall sensor, the IR looking to the right
        s.wallSensor = bitOfByte( 0, r[1] )
        
        # byte 2: left cliff sensor
        s.leftCliff = bitOfByte( 0, r[2] )
        
        # byte 3: front left cliff sensor
        s.frontLeftCliff = bitOfByte( 0, r[3] )
        
        # byte 4: front right cliff sensor
        s.frontRightCliff = bitOfByte( 0, r[4] )
        
        # byte 5: right cliff sensor
        s.rightCliff = bitOfByte( 0, r[5] )
        
        # byte 6: virtual wall detector (the separate unit)
        s.virtualWall = bitOfByte( 0, r[6] )
        
        # byte 7: motor overcurrents byte
        s.driveLeft = bitOfByte( 4, r[7] )
        s.driveRight = bitOfByte( 3, r[7] )
        s.mainBrush = bitOfByte( 2, r[7] )
        s.vacuum = bitOfByte( 1, r[7] )
        s.sideBrush = bitOfByte( 0, r[7] )
        
        # byte 8: dirt detector left
        # the dirt-detecting sensors are acoustic impact sensors
        # basically, they hear the dirt (or don't) going by toward the back
        # this value ranges from 0 (no dirt) to 255 (lots of dirt)
        s.leftDirt = r[8]
        
        # byte 9: dirt detector right
        # some roomba's don't have the right dirt detector
        # the dirt detectors are metallic disks near the brushes
        s.rightDirt = r[9]
        
        # packet number 2 (6 bytes)
        
        # byte 10: remote control command
        # this is the value of the remote control command currently
        # being seen by the roomba, it is 255 if there is no command
        # not all roombas have a remote control...
        s.remoteControlCommand = r[10]
        
        # byte 11: button presses
        s.powerButton = bitOfByte( 3, r[11] )
        s.spotButton = bitOfByte( 2, r[11] )
        s.cleanButton = bitOfByte( 1, r[11] )
        s.maxButton = bitOfByte( 0, r[11] )
        
        # bytes 12 and 13: distance
        # the distance that roomba has traveled, in mm, since the
        # last time this data was requested (not from a SensorFrame,
        # but from the roomba)
        # It will stay at the max or min (32767 or -32768) if
        # not polled often enough, i.e., it then means "a long way"
        # It is the sum of the two drive wheels' distances, divided by 2
        s.distance = twosComplementInt2bytes( r[12], r[13] )
        
        # bytes 14 and 15: angle
        s.rawAngle = twosComplementInt2bytes( r[14], r[15] )
        # the distance between the wheels is 258 mm
        s.angleInRadians = 2.0 * s.rawAngle / 258.0
        
        # packet number 3 (10 bytes)
        
        # byte 16: charging state
        # 0 == not charging
        # 1 == charging recovery
        # 2 == charging
        # 3 == trickle charging
        # 4 == waiting
        # 5 == charging error
        s.chargingState = r[16]
        
        # bytes 17 and 18: voltage in millivolts
        # this is unsigned, so we don't use two's complement
        s.voltage = r[17] << 8 | r[18]
        # check this for byte order!!
        
        # bytes 19 and 20: current in milliamps
        # this is signed, from -32768 to 32767
        # negative currents are flowing out of the battery
        # positive currents are flowing into the battery (charging)
        s.current = twosComplementInt2bytes( r[19], r[20] )
        
        # byte 21: temperature of the battery
        # this is in degrees celsius
        s.temperature = twosComplementInt1byte( r[21] )
        
        # bytes 22 and 23: charge of the battery in milliamp-hours
        # this is two unsigned bytes
        s.charge = r[22] << 8 | r[23]
        
        # bytes 24 and 25: estimated capacity of the roomba's battery
        # in units of milliamp-hours
        # when the charge reaches this value, the battery is
        # considered fully charged
        s.capacity = r[24] << 8 | r[25]
        
        # OK, here we call a function to integrate the odometric
        # step taken here (unless distance and rawAngle are 0)
        self.integrateNextOdometricStepCreate(s.distance,s.rawAngle)
        
        return s

    def moveServo(self, degree):
        self.ser.write(chr(144))
        self.ser.write(chr(degree))
        self.ser.write(chr(0))
        self.ser.write(chr(0))
        return "Moving Servo"

    def text(self):
        return "Correct classese"


class RoombaRobot:
    """ this class contains a Roomba object to handle all of the
        communications, but this class actually starts to reason
        about the robot, e.g., where it is right now...
    """
    
    def __init__(self, PORT, mapfilename=None):
        """ the constructor creates the robot object
            and tracks its position
        """
        self.roomba = Roomba(PORT)
        if mapfilename != None:
            self.roomba.ser.setMap(mapfilename)
        # in cm and radians
        self.x = 0.0
        self.y = 0.0
        self.thr = 0.0
    
    def SetPose(self, x, y, thr):
        """ set's the robot's global pose to x, y, thr
        """
        self.x = x
        self.y = y
        self.thr = thr
        # now set the simulator's pose there, too!
        self.roomba.ser.x_real = x*10.0
        self.roomba.ser.y_real = y*10.0
        self.roomba.ser.thr_real = thr
    
    def SetCollisions(self, doWeCrashIntoWalls):
        """ sets the simulator to use or ignore walls
        """
        self.roomba.ser.doWeCrashIntoWalls = doWeCrashIntoWalls
    
    def GetPose(self):
        """ cheats by providing the internal pose... """
        s = self.roomba.readSensors() # updates the simulator's pose
        self.x = self.roomba.ser.x_real/10.0 # and then grabs it...
        self.y = self.roomba.ser.y_real/10.0
        self.thr = self.roomba.ser.thr_real
        # get the two bump sensors
        lb = s.leftBump
        rb = s.rightBump
        return (self.x, self.y, self.thr, lb, rb)
    
    def GetPoseReal(self):
        """ GetPose updates the pose and returns it """
        # s is a SensorFrame
        s = self.roomba.readSensors()
        
        # we have the distance and angle
        d = s.distance/10.0  # the distance the center traveled in cm
        thr = s.angleInRadians  # the angle the robot changed
        
        # this means we know the arc of the circle it's traveled...
        # in robot-local coordinates here
        if thr == 0.0 or math.fabs(d/thr) > 10000:
            # just went straight
            x_delta = d
            y_delta = 0.0
        else:
            ROC = d/float(thr)
            x_delta = 0.0 + ROC*math.sin(thr)
            y_delta = ROC - ROC*math.cos(thr)
        
        x_from_deltax = math.cos(self.thr) * x_delta
        y_from_deltax = math.sin(self.thr) * x_delta
        x_from_deltay = math.cos(self.thr + math.pi/2.0) * y_delta
        y_from_deltay = math.sin(self.thr + math.pi/2.0) * y_delta
        
        self.x = self.x + x_from_deltax + x_from_deltay
        self.y = self.y + y_from_deltax + y_from_deltay
        self.thr += thr
        
        # find our deltas in a canonical coordinate system
        return (self.x, self.y, self.thr, s.leftBump, s.rightBump)

    
    def Go( self, cm_per_sec, deg_per_sec, sleepyTime ):
        """ Go sets the velocity in cm/sec and deg/sec """
        # need to convert to the roomba's drive parameters
        #
        # for now, just one or the other...
        if cm_per_sec == 0:
            # just handle rotation
            # convert to radians
            rad_per_sec = math.radians(deg_per_sec)
            # make sure the direction is correct
            if rad_per_sec >= 0:  dirstr = 'CCW'
            else: dirstr = 'CW'
            # compute the velocity, given that the robot's
            # radius is 258mm/2.0
            vel_mm_sec = math.fabs(rad_per_sec) * (258.0/2.0)
            # send it off to the robot
            self.roomba.drive( vel_mm_sec, 0, dirstr )
        
        elif deg_per_sec == 0:
            # just handle forward/backward translation
            vel_mm_sec = 10.0*cm_per_sec
            big_radius = 30000
            # send it off to the robot
            self.roomba.drive( vel_mm_sec, big_radius )

        
        else:
            #
            print 'Go in RoombaRobot not implemented yet'
        return
        
