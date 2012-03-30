var exec_python = require('eval.py').eval
  , say = require('say')
  , messages = require('./lib/messages')
  , SERVER = 'http://dpdbot.deploydapp.com'
  , program = require('mdoq').require('mdoq-http').use(SERVER+'/programs')
  , bot
  , arguments = process.argv.splice(2)
  , usb = arguments[0] || '/dev/tty.usbserial-ftDII8Z7';
;

//Example commands: Go(speed, degrees, rotation), printSensors(), seekDock(), playNote(number, duration)
//bot.command('Go', [100,1000], function (err, data){ console.log(data); })
bot = {
  init: function (usb) {
    exec_python("import create");
    exec_python("import time");
    exec_python("bot = create.Create('"+usb+"')");
  },
  command: function(method, args, callback) {
    var cmd = "bot."+method+'(', arg, first_loop=true;
    while (arg = args.shift()) {
      if (!first_loop) cmd += ",";
      cmd += arg;
      first_loop=false;
    }
    cmd+=")";
    exec_python(cmd, callback);
  }
};

bot.init();

function stop_bot () {
  bot.command('stop');
}
function execute_step (speed, degrees, rotation, time) {
  bot.command('Go', [degrees, rotation]);
}
function get_current_program () {
  program.get(function(err, res){
    console.log('response');
  });
}
function mission_failed () {
  bot.command('stop');
  bot.command('seekDock')
  say.speak(messages.failure());
}

//Loop to load the next program from the server, and check if I should abandon this current program
//We really just want one program at a time from the server...let it worry about the queue
