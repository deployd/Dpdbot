import math
import random

class Messages:
	global failures
	failures = [
	  "You drive like a Java developer."
	  , "You crash more than Internet Explorer."
	  , "Clear your cookies and try again."
	  , "That tickles."
	  , "I'm going home."
	  , "[[rate 30]] Ouch."
	  , "You broke my back bone."
	  , "Your program needs better regression testing."
	  , "You are a baby developer."
	  , "That's a blocker."
	  , "Oops."
	  , "Who put this wall here!"
	  , "I enjoyed our drive together."
	  , "We should do this again sometime not."
	  , "It's my turn to drive you now."
	  , "That was exhausting."
	]

	global successes
	successes = [
	  "I never doubted you."
	  , "You should program me more often"
	  , "Best ride of my life."
	  , "I couldn't have done it without you. Actually, I could."
	]

	#User either gets a personal greeting or standard greeting
	global greeting_begins
	greeting_begins = {
	  'with_bio': "Hello <name>. I just red your twitter bio, "
	  , 'standard': "Let's do this, <name>"
	}

	#These are appended to greetings, presumably of twitter handles.
	global greeting_ends
	greeting_ends = [
	  "You make me sick"
	  , "I'm so proud of you."
		,	"[[rate 90]] tell me more?"
	  , "That's a damn good bio."
	  , "You're okay in my book."
	  , "I wish I had a fist to punch you."
	  , "I would so fall oh you."
	  , "140 characters have never been more wasted"
	  , "You are so [[rate 50]] boring."
	]
	
	global get_random
	def get_random(collection):
	  return collection[int(round((random.random() * (len(collection) - 1))))]
	@staticmethod
	def failure ():
		return get_random(failures)
	
	@staticmethod
	def success ():
	  return get_random(successes)

	@staticmethod
	def greeting (name, bio=False):
		if bio is False:
			return greeting_begins['standard'].replace('<name>', name)
		else:
			bio.replace('.', '?')
			return greeting_begins['with_bio'].replace('<name>', name) + bio + '? '+ get_random(greeting_ends)