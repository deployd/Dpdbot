var failures = [
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
  , "That was exhausting."
];

var successes = [
  "I never doubted you."
  , "You should program me more often"
  , "Best ride of my life."
  , "It's my turn to drive you now."
  , "I couldn't have done it without you. Actually, I could."
];

//User either gets a personal greeting or standard greeting
var greeting_begins = {
  with_bio: "Hello <name>. I just red your twitter bio."
  , standard: "Let's do this."
};

//These are appended to greetings, presumably of twitter handles.
var greeting_ends = [
  "You make me sick"
  , "I'm so proud of you."
  , "That's a damn good bio."
  , "You're okay in my buk."
  , "I wish I had a fist to punch you."
  , "I would so fall oh you."
  , "140 characters have never been more wasted"
  , "You are so [[rate 50]] boring."
];

function get_random(collection) {
  return collection[Math.round((Math.random() * (collection.length - 1)))];
}

exports.failure = function () {
	return get_random(failures);
};

exports.success = function () {
  return get_random(successes);
};

exports.greeting = function (name, bio) {
  if (!bio) return greeting_begins.standard;
  return greeting_begins.bio.replace('<name>', name) + bio + get_random(greeting_ends);
};