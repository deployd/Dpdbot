function url(path) {
  return 'http://localhost:2403' + path;
}

ko.bindingHandlers.numberValue = {
  init: function(element, valueAccessor) {
    var prop = valueAccessor();
    $(element).on('input', function() {
      prop(parseInt($(this).val()));
    });
  },
  update: function(element, valueAccessor) {
    var prop = valueAccessor();
    $(element).val(prop());
  }
};