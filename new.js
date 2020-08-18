/** concatenanted 'preact-context/dist/esm/context.js' ***/
import { h, Component } from 'lib/dom/preactComponent.js';

function getRenderer(props) {
  var child = getOnlyChildAndChildren(props).child;
  return child || ('render' in props && props.render);
}
var MAX_SIGNED_31_BIT_INT = 1073741823;

var defaultBitmaskFactory = function() {
  return MAX_SIGNED_31_BIT_INT;
};
var ids = 0;

function _createContext(value, bitmaskFactory) {
  var key = '_preactContextProvider-' + ids++;

  var Provider = (function(_super) {
    __extends(Provider, _super);

    function Provider(props) {
      var _this = _super.call(this, props) || this;
      _this._emitter = createEmitter(props.value, bitmaskFactory || defaultBitmaskFactory);
      return _this;
    }

    Provider.prototype.getChildContext = function() {
      var _a;
      return (_a = ({}, (_a.key = (this._emitter, _a))));
    };

    Provider.prototype.componentDidUpdate = function() {
      this._emitter.val(this.props.value);
    };

    Provider.prototype.render = function() {
      var _a = getOnlyChildAndChildren(this.props),
        child = _a.child,
        children = _a.children;

      if(child) {
        return child;
      }

      return h('span', null, children);
    };

    return Provider;
  })(Component);

  var Consumer = (function(_super) {
    __extends(Consumer, _super);

    function Consumer(props, ctx) {
      var _this = _super.call(this, props, ctx) || this;

      _this._updateContext = function(value, bitmask) {
        var unstable_observedBits = _this.props.unstable_observedBits;
        var observed = unstable_observedBits === undefined || unstable_observedBits === null ? MAX_SIGNED_31_BIT_INT : unstable_observedBits;
        observed = observed | 0;

        if((observed & bitmask) === 0) {
          return;
        }

        _this.setState({ value });
      };

      _this.state = { value: _this._getEmitter().val() || value };
      return _this;
    }

    Consumer.prototype.componentDidMount = function() {
      this._getEmitter().register(this._updateContext);
    };

    Consumer.prototype.shouldComponentUpdate = function(nextProps, nextState) {
      return this.state.value !== nextState.value || getRenderer(this.props) !== getRenderer(nextProps);
    };

    Consumer.prototype.componentWillUnmount = function() {
      this._getEmitter().unregister(this._updateContext);
    };

    Consumer.prototype.componentDidUpdate = function(_, __, prevCtx) {
      var previousProvider = prevCtx.key;

      if(previousProvider === this.context.key) {
        return;
      }

      (previousProvider || noopEmitter).unregister(this._updateContext);
      this.componentDidMount();
    };

    Consumer.prototype.render = function() {
      var render = 'render' in this.props && this.props.render;
      var r = getRenderer(this.props);

      if(render && render !== r) {
        console.warn('Both children and a render function are defined. Children will be used');
      }

      if(typeof r === 'function') {
        return r(this.state.value);
      }

      console.warn("Consumer is expecting a function as one and only child but didn't find any");
    };

    Consumer.prototype._getEmitter = function() {
      return this.context.key || noopEmitter;
    };

    return Consumer;
  })(Component);

  return { Provider, Consumer };
}
export default _createContext;
var createContext = _createContext;
