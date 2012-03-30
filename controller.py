import time
import create
import urllib2
import subprocess
import json
from messages import Messages
from collections import deque

#Constants
# SERVER = "http://dpdbot.deploydapp.com"
SERVER = "http://localhost/~jcross"

# subprocess.call(['say', Messages.failure()])

#initialize the driver
bot = create.Create('/dev/tty.usbserial-ftDII8Z7')
bot.toFullMode()

def say(something):
	subprocess.call(['say', something])

def robot_data(data):
	print 'The robot is telling me something', data

def start_game ():
	print 'Let the games begin.'
	program = load_program()
	start_program(program)

def stop_bot ():
  bot.command('stop');

def start_program (program):
    did_finish = False
    did_hit_wall = False
    first_time = True
    steps = program['stepsJson']
    steps = json.loads(steps)
    steps = deque(steps)
    print 'steps deque'
    print steps
    
    bot.Go(-10)
    time.sleep(1)
    bot.Go(0, 100)
    time.sleep(2.22)
    bot.stop()
    
    def drive_forward (step):
        bot.Go(500)
        return step['seconds']
    
    def turn_right (step):
        bot.Go(0, 90)
        # return time to sleep
        return step['degrees'] / 90

    def turn_left (step):
        bot.Go(0, -90)
        #return time to sleep
        return step['degrees'] / 90
	
    commands = {
		'driveForward' : drive_forward,
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
			if not first_time and 1 in sensors[7]:
			  did_hit_wall = True
			  break
			first_time = False
			#Check sensors for bumps
			#Sleep for 100ms
			time.sleep(0.015)
	
	bot.stop()
	bot.seekDock()
	
    if did_finish == True:
        mission_success()
    elif did_hit_wall:
        mission_crashed()
    else:
        mission_failed()

def execute_step (speed, degrees, rotation, time):
    bot.Go(speed,degrees,rotation)

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
	say('Loading program.')
	
	program = urllib2.urlopen(SERVER+'/programs')
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

def mission_success ():
	say(Messages.success())

def mission_crashed():
  say(Messages.failure())

def mission_failed ():
	say("You disappoint me")


start_game()
# say(Messages.greeting('voodootikigod', get_twitter_bio('voodootikigod')))
