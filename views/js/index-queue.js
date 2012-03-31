function createQueueViewModel(id) {

  function makeQueuedProgram(data) {
    var self = ko.mapping.fromJS(data);

    self.number = ko.computed(function() {
      return queueViewModel.queuedPrograms().indexOf(self) + 1;
    }, self);

    self.isOwned = ko.computed(function() {
      return self._id() === id;
    });

    return self;
  };

  var queuedProgramsMapping = {
    'queuedPrograms': {
      key: function(data) {
        return ko.utils.unwrapObservable(data._id);
      },
      create: function(options) {
        return makeQueuedProgram(options.data);
      }
    }
  };

  var queueViewModel = {
    ownedId: id, 
    queuedPrograms: ko.observableArray()
  };

  queueViewModel.isWaiting = ko.computed(function() {
    if (this.queuedPrograms()[0] && this.queuedPrograms()[0].isOwned()) {
      return false;
    } else {
      return true;
    }
  }, queueViewModel);

  queueViewModel.ownedProgram = ko.computed(function() {
    return this.queuedPrograms().find(function(program) {
      return program.isOwned();
    })
  }, queueViewModel);

  queueViewModel.updateQueue = (function() {
    var query = {
      executed: false,
      $orderby: {
        timestamp: 1
      }
    };
    $.get(url('/programs?q=' + JSON.stringify(query)), function(res) {
        res = res || [];
        ko.mapping.fromJS({queuedPrograms: res}, queuedProgramsMapping, queueViewModel);
    });
  }).bind(queueViewModel);

  queueViewModel.checkOwned = (function() {
    $.get(url('/programs/' + id), function(result) {
      if (result.executed) {
        viewModel.queue(null);
        viewModel.result(createResultViewModel(result.scoreId));
      }
    })
  }).bind(queueViewModel);

  queueViewModel.cancel = (function() {
    var owned = this.ownedProgram();
    if (this.isWaiting()) {
      $.ajax(url('/programs/' + id), {
        type: 'DELETE',
        success: function() {
          viewModel.queue(null);
        }
      })
    } else {
      viewModel.queue(null);
    }
      
  }).bind(queueViewModel);

  queueViewModel.unload = (function() {
    clearInterval(interval);
  }).bind(queueViewModel)

  queueViewModel.updateQueue();

  var interval = setInterval(function() {
    queueViewModel.updateQueue();
    queueViewModel.checkOwned(); 
  }, 1000);


  return queueViewModel;

};