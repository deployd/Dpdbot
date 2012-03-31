viewModel = {
  program: programViewModel,
  queue: ko.observable(),
  result: ko.observable(),

  _lastQueue: null
};

ko.computed(function() {
  if (this._lastQueue) {
    this._lastQueue.unload();
  }

  this._lastQueue = viewModel.queue();
}, viewModel);

viewModel.status = ko.computed(function() {
  if (!this.queue() && !this.result()) {
    return "Programming";
  } else if (this.result()) {
    return "Results"
  } else if (this.queue() && this.queue().isWaiting()) {
    return "Waiting";
  } else {
    return "Executing";
  } 
}, viewModel);

$('.container').show();