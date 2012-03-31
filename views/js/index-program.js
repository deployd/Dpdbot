
function Step(type) {
  this.type = ko.observable(type);
  this.typeId = ko.observable();
  this.typeTag = ko.observable();

  this.seconds = ko.observable(1);

  ko.computed(function() {
    if (this.type().tag === 'turn') {
      if (!this.degrees) {
        this.degrees = ko.observable(90);
      }
      this.seconds(1);  
    }

    this.typeId(this.type().typeId);
    this.typeTag(this.type().tag);

  }, this);
  
  this.toJS = (function() {
    if (this.type().tag === 'turn') {
      return {
        degrees: this.degrees(),
        type: this.typeId()
      }
    } else if (this.type().tag === 'drive') {
      return {
        seconds: this.seconds(),
        type: this.typeId()
      };  
    }
  }).bind(this);
};

var stepTypes = [{
  typeId: 'driveForward',
  typeName: 'Drive Forward',
  tag: 'drive'
}, {
  typeId: 'driveBackward',
  typeName: 'Drive Backward',
  tag: 'drive'
}, {
  typeId: 'turnLeft',
  typeName: 'Turn Left',
  tag: 'turn'
}, {
  typeId: 'turnRight',
  typeName: 'Turn Right',
  tag: 'turn'
}];

var programViewModel = {
  stepTypes: stepTypes,
  maxSeconds: 30,

  steps: ko.observableArray(),
  twitterHandle: ko.observable('')
};

programViewModel.totalSeconds = ko.computed(function() {
  var total = 0;

  this.steps().forEach(function(step) {
    total += step.seconds();
  });

  return total;
}, programViewModel);

programViewModel.isValid = ko.computed(function() {
  if (this.steps().length <= 0 ||
      this.twitterHandle().length <= 0 ||
      this.totalSeconds() > this.maxSeconds) {
    return false;
  }

  return true
}, programViewModel);

programViewModel.addStep = (function() {
  this.steps.push(new Step(stepTypes[0]));
}).bind(programViewModel);

programViewModel.removeStep = (function(step) {
  this.steps.remove(step);
}).bind(programViewModel);

programViewModel.changeStep = (function(step) {
  var index = this.steps.indexOf(step);
  this.steps.splice(index, 1, new step.type());
}).bind(programViewModel);

programViewModel.submit = (function() {
  if (!this.isValid()) { return; }

  var json = {
    twitterHandle: this.twitterHandle(),
    stepsJson: JSON.stringify(this.steps().map(function(step) {
      return step.toJS();
    }))
  };

  $.ajax(url('/programs'), {
    type: 'POST',
    contentType: 'application/json',
    data: JSON.stringify(json),
    success: function(result) {
      viewModel.queue(createQueueViewModel(result._id));
    },
    error: function(xhr) {
      try {
        var json = JSON.parse(xhr.responseText); 
        var errors = "Could not save program:\n\n";
        Object.each(json.errors, function(key, value) {
          errors += value + "\n";
        });
        alert(errors);
      } catch (err) {
        alert("An error occurred");  
      }
    }
  })

}).bind(programViewModel);

programViewModel.addStep();