var scoreMapping = {
  'scores': {
      key: function(data) {
        return ko.utils.unwrapObservable(data._id);
      },
      create: function(options) {
        return makeScore(options.data);
      }
    }
};

function makeScore(data) {
  var self = ko.mapping.fromJS(data);

  self.place = ko.computed(function() {
    return leaderboardViewModel.scores.indexOf(self) + 1;
  }, self);

  self.pointsFormatted = ko.computed(function() {
    return Math.floor(self.score()) + "pts";
  }, self);

  self.isOwned = ko.computed(function() {
    return viewModel.result() && viewModel.result().ownedId == this._id();
  }, self);

  return self;
}

var leaderboardViewModel = {
  scores: ko.observableArray()
};

leaderboardViewModel.topThreeScores = ko.computed(function() {
  return this.scores.slice(0, 3);
}, leaderboardViewModel);

leaderboardViewModel.isVisible = ko.computed(function() {
  return viewModel.queue();
}, leaderboardViewModel);

leaderboardViewModel.updateScores = (function() {
  getLeaderboard(scoreMapping, leaderboardViewModel);
}).bind(leaderboardViewModel);

leaderboardViewModel.updateScores();

setInterval(function() {
  if (leaderboardViewModel.isVisible()) {
    leaderboardViewModel.updateScores();  
  }
}, 5000);

viewModel.leaderboard = leaderboardViewModel;