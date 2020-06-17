var _regeneratorRuntime = require('@babel/runtime/regenerator');

require('regenerator-runtime/runtime');

var _awaitAsyncGenerator = require('@babel/runtime/helpers/awaitAsyncGenerator');

var _wrapAsyncGenerator = require('@babel/runtime/helpers/wrapAsyncGenerator');

function test() {
  return _test.apply(this, arguments);
}

function _test() {
  _test = _wrapAsyncGenerator(
    /*#__PURE__*/ _regeneratorRuntime.mark(function _callee() {
      var i;
      return _regeneratorRuntime.wrap(function _callee$(_context) {
        while(1)
          switch ((_context.prev = _context.next)) {
            case 0:
              i = 0;

            case 1:
              if(!(i < 10)) {
                _context.next = 7;
                break;
              }

              _context.next = 4;
              return i;

            case 4:
              i++;
              _context.next = 1;
              break;

            case 7:
            case 'end':
              return _context.stop();
          }
      }, _callee);
    })
  );
  return _test.apply(this, arguments);
}
