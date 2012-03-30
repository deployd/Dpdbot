import time
import create
import urllib2
import subprocess
import json
from messages import Messages

#Constants
SERVER = "http://dpdbot.deploydapp.com"

subprocess.call(['say', Messages.failure()])

#initialize the driver
bot = create.Create('/dev/tty.usbserial-ftDII8Z7')

def robot_data(data):
	print 'The robot is telling me something', data

def start_game ():
  print 'Let the games begin.'
  load_program()

def stop_bot ():
  bot.command('stop');

def start_program (program):
	steps = json.loads(program.steps)
	print ("steps", steps)
  #Start the current program

def execute_step (speed, degrees, rotation, time):
  bot.Go(speed,degrees,rotation)

def get_twitter_bio (sn, next):
	response = urllib2.urlopen("http://api.twitter.com/1/users/show.json?screen_name="+sn)
	data = response.read()
	user = json.loads(data)
	print user.bio
	return user.bio

def load_program ():
  #Load the top program from the server.
	program = urllib2.urlopen(SERVER+'/programs')
	program = program[0]
	#Load the user's Twitter bio if possible.
	if program.username:
		bio = get_twitter_bio(program.username)
		if bio:
			subprocess.call(['say', Messages.greeting(program.username, bio)])
   	else:
			subprocess.call(['say', Messages.greeting(program.username)])
	start_program(program)

def mission_failed ():
	bot.stop()
	bot.seekDock()
	subprocess.call(['say', Messages.failure()])