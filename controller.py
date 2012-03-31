import time
import create
import urllib2
from urllib2 import HTTPError
from urllib2 import URLError
import subprocess
import json
from messages import Messages
from collections import deque

#Constants
SERVER = "http://dpdbot.deploydapp.com"
# SERVER = "http://localhost/~jcross"
global bot
global current_program

# subprocess.call(['say', Messages.failure()])

def say(something):
	subprocess.call(['say', something])

def robot_data(data):
	print 'The robot is telling me something', data

def start_game ():
	print 'Let the games begin.'
	global current_program
	current_program = load_program()
	start_program()

def stop_bot ():
    bot.command('stop');

def start_program ():
    #initialize the driver
    global bot
    global current_program
    bot = create.Create('/dev/tty.usbserial-ftDII8Z7')
    bot.toFullMode()
    
    print "current_program"
    print current_program
    did_finish = False
    did_hit_wall = False
    first_time = True
    steps = current_program['stepsJson']
    steps = json.loads(steps)
    steps = deque(steps)
    print 'steps deque'
    print steps
    
    bot.Go(-10)
    time.sleep(1)
    bot.Go(0, 90)
    time.sleep(2)
    bot.stop()
    
    def drive_forward (step):
        print "drive_forward"
        bot.Go(500)
        return step['seconds']

    def drive_backward (step):
        print "drive_backward"
        bot.Go(-500)
        return step['seconds']
    
    def turn_right (step):
        print "turn_right"
        bot.Go(0, 90)
        # return time to sleep
        return step['degrees'] / 90

    def turn_left (step):
        print "turn_left"
        bot.Go(0, -90)
        #return time to sleep
        return step['degrees'] / 90
        	
    commands = {
		'driveForward' : drive_forward,
		'driveBackward': drive_backward,
		'turnRight' : turn_right,
		'turnLeft'	: turn_left
	}
	
    while 1:
        try:
            step = steps.popleft()
            print step
        except (Exception):
            print "No more steps"
            break
		
        if did_hit_wall:
            "Hit a wall"
            break
		
        milliseconds = commands.get(step['type'])(step)
        print "milliseconds returned"
        print milliseconds
        milliseconds = milliseconds * 1000
        
        while milliseconds > 0:
            #Loops 10 times a second
            #Doesn't care what's running right now.
            milliseconds -= 15
            sensors = bot.sensors()
            print sensors[7]
            print sensors[13]
            #Check sensors for bumps or finish
            if not first_time and 1 in sensors[7]:
                did_hit_wall = True
                break
            if not first_time and sensors[13] == 1:
                did_finish = True
                break
            first_time = False

            #Sleep for 100ms
            time.sleep(0.015)
	
    print "stopping bot"
    bot.stop()
    print "seeking doc"
    bot.seekDock()
    
    if did_finish == True:
        mission_success()
    elif did_hit_wall:
        mission_crashed()
    else:
        mission_failed()

def get_twitter_bio (sn):
    try:
        response = urllib2.urlopen("http://api.twitter.com/1/users/show.json?screen_name="+sn)
        data = response.read()
        user = json.loads(data)
        print user['description']
        return user['description']
    except (Exception):
        return False

def load_program ():
    #Load the top program from the server.
    #TODO: Check again later if no program is loaded
    say('Loading program.')
	
    try:
        reqUrl = SERVER+'/programs?q='+urllib2.quote('{"executed":false}')
        print reqUrl
        program = urllib2.urlopen(reqUrl)
        program = program.read()
        print 'program retrieved'
        print program
	
    	program = json.loads(program)
    	print 'json parsed'
    	print program
	
    	program = program[0]
    	print 'program from index'
    	print program
	
    	say('Program loaded')
    	#Load the user's Twitter bio if possible.
    	if program.has_key('twitterHandle'):
    		handle = program['twitterHandle'].replace('@', '')
    		bio = get_twitter_bio(program['twitterHandle'])
    		if bio:
    			say(Messages.greeting(program['twitterHandle'], bio))
    		else:
    			say(Messages.greeting(program['twitterHandle']))
    	else:
    		say('say', 'Oops, could not find twitter handle.')
	
    	return program
    except Exception as e:
        print "No program to load"
        print e

def start_after_dock():
    global bot
    global current_program
    try:
        print "About to post score"
        #TODO: add distance and time
        valueObject = json.dumps({'twitterHandle': current_program['twitterHandle'], 'robotSecurityKey': 'DPDSECRET123', 'completed':False, 'time': 0, 'distance': 0})
        print valueObject
        route = SERVER+'/scores'
        score = urllib2.urlopen(route, valueObject)
        score = score.read()
        print score
        scoreId = json.loads(score)['_id']

        opener = urllib2.build_opener(urllib2.HTTPHandler)
        programs_route = SERVER+'/programs/' + current_program['_id']
        current_program['executed'] = True
        current_program['scoreId'] = scoreId
        print programs_route
        program_json = json.dumps(current_program)
        print "program_json"
        print program_json
        request = urllib2.Request(programs_route, data=program_json)
        request.add_header('Content-Type', 'application/json')
        request.get_method = lambda: 'PUT'
        print "executing programs request"
        program = opener.open(request)
    except HTTPError as e:
        print "HTTPError"
        print e
        print e.code
    except URLError as e:
        print "HttpError"
        print e.reason
    
    countdown = 2
    while countdown > 0:
        countdown -= 0.25
        #TODO: Set timeout and warn user to put back on dock
        print bot.sensors()[21]
        if bot.sensors()[21] == 2:
            break
        time.sleep(0.25)
    say("You have five seconds to put me back on the dock.")
    time.sleep(5)
    bot.close()
    start_game()
    
def mission_success ():
    print "mission_success"
    say(Messages.success())
    start_after_dock()

def mission_crashed():
    print "mission_crashed"
    say(Messages.failure())
    start_after_dock()

def mission_failed ():
    print "mission_failed"
    say("You disappoint me")
    start_after_dock()

start_game()

# say(Messages.greeting('voodootikigod', get_twitter_bio('voodootikigod')))
