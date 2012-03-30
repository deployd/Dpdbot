var MAX_SECONDS = 30;

this.timestamp = new Date();

if (this.twitterHandle.indexOf('@') !== 0) {
    this.twitterHandle = '@' + this.twitterHandle;
}

if (this.twitterHandle.length < 2) {
    error('twitterHandle', 'Twitter handle is required');
}

var totalSeconds = 0;
var steps = JSON.parse(this.stepsJson);

steps.forEach(function(step, i) {
    
    i += 1;
    
    if (step.type === 'turnLeft' || step.type === 'turnRight') {
        step.seconds = 1;
        if (step.degrees > 180 || step.degrees < 1) {
            error('stepsJson' + i, 'Step ' + i + 
                ' must be between 1 and 180 degrees');
        }
    }
    
    totalSeconds += step.seconds;
    
    if (step.seconds < 1) {
        error('stepsJson' + i, 'Step ' + i + 
            ' must be at least 1 second');
    }
    
    
});

if (totalSeconds > MAX_SECONDS) {
    error('stepsJson', 'Program must not be longer than 30 seconds');
}