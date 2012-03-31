function createResultViewModel(id) {
  var resultViewModel = {
    ownedId: id,
    score: ko.observable(0),
    completed: ko.observable(false),
    distance: ko.observable(0),
    time: ko.observable(0)
  };

  $.get(url('/scores/' + id), function(res) {
    ko.mapping.fromJS(res, {}, resultViewModel);
  });

  resultViewModel.close = (function() {
    viewModel.result(null);
  }).bind(resultViewModel);

  return resultViewModel;
}