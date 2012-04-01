function url(path) {
  return 'http://dpdbot.deploydapp.com' + path;
}

ko.bindingHandlers.numberValue = {
  init: function(element, valueAccessor) {
    var prop = valueAccessor();
    $(element).on('input', function() {
      prop(parseFloat($(this).val()));
    });
  },
  update: function(element, valueAccessor) {
    var prop = valueAccessor();
    if (isNaN(prop())) {
      $(element).val("");
    } else {
      $(element).val(prop());
    }
  }
};

ko.bindingHandlers.jqSlider = {
  init: function(element, valueAccessor) {
    var options = valueAccessor();
    var val = ko.utils.unwrapObservable(options.value);
    $(element).slider({
      range: 'min',
      value: val,
      min: options.min || 0,
      max: options.max,
      step: options.step || 1,
      slide: function(event, ui) {
        options.value(ui.value)
      }
    });
  }, update: function(element, valueAccessor) {
    var options = valueAccessor();
    var val = ko.utils.unwrapObservable(options.value);
    $(element).slider('value', val);
  }
}

function getLeaderboard(mapping, viewModel) {
  var query = {
    $orderby: {
      score: -1,
      programLength: 1
    }
  };
  $.get(url('/scores?q=' + JSON.stringify(query)), function(res) {
    res = res || [];
    ko.mapping.fromJS({scores: res}, mapping, viewModel);
  });
}