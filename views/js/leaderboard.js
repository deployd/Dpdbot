var scoreMapping = {
  'scores': {
      create: function(options) {
        return makeScore(options.data);
      }
    }
};

var queueMapping = {
  'queue': {
      create: function(options) {
        return makeProgram(options.data);
      }
    }
};


function makeScore(data) {
  var self = ko.mapping.fromJS(data);

  self.place = ko.computed(function() {
    return viewModel.scores.indexOf(self) + 1;
  }, self);

  self.pointsFormatted = ko.computed(function() {
    return Math.floor(self.score()) + "pts";
  }, self);

  return self;
}

function makeProgram(data) {
  var self = ko.mapping.fromJS(data);
  self.number = ko.computed(function() {
    return viewModel.queue.indexOf(self) + 1;
  });
  return self;
}

var viewModel = {
  scores: ko.observableArray(),
  queue: ko.observableArray()
};

viewModel.refreshScores = (function() {
  getLeaderboard(scoreMapping, viewModel);
}).bind(viewModel);

viewModel.refreshQueue = (function() {
  var query = {
    executed: false,
    $orderby: {
      timestamp: 1
    }
  };
  $.get(url('/programs?q=' + JSON.stringify(query)), function(res) {
      res = res || [];
      ko.mapping.fromJS({queue: res}, queueMapping, viewModel);
  });
}).bind(viewModel);

viewModel.refreshScores();
viewModel.refreshQueue();

setInterval(viewModel.refreshScores, 5000);
setInterval(viewModel.refreshQueue, 5000);

ko.applyBindings(viewModel);