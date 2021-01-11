import { h, render, Component } from './lib/dom/preactComponent.js';

export class SlotContent {
  apply(slot, content, fireChange) {
    let { slots } = this.context;
    if(slot) {
      slots.named[slot] = content;
      if(fireChange) {
        for(let i = 0; i < slots.onChange.length; i++) {
          slots.onChange[i]();
        }
      }
    }
  }

  componentWillMount() {
    this.apply(this.props.slot, this.props.children[0], true);
  }

  componentWillReceiveProps({ slot, children }) {
    if(slot !== this.props.slot) {
      this.apply(this.props.slot, null, false);
      this.apply(slot, children[0], true);
    } else if(children[0] !== this.props.children[0]) {
      this.apply(slot, children[0], true);
    }
  }

  componentWillUnmount() {
    this.apply(this.props.slot, null, true);
  }

  render() {
    return null;
  }
}

export class SlotProvider {
  getChildContext() {
    return {
      slots: {
        named: {},
        onChange: []
      }
    };
  }

  render(props) {
    return props.children /*[0]*/;
  }
}

export class Slot extends Component {
  state = {};

  constructor(props, context) {
    super(props, context);
    this.update();
  }
  update = () => {
    let content = this.context.slots.named[this.props.name];
    if(content != this.state.content) {
      this.setState({ content });
    }
  };
  componentDidMount = () => {
    this.context.slots.onChange.push(this.update);
  };
  componentWillUnmount = () => {
    this.context.slots.onChange.push(this.update);
  };
}

//(Slot.prototype = new Component()).constructor = Slot;
Slot.prototype.render = function() {
  let child = ReactComponent.toChildArray(this.props.children)[0];
  return typeof child === 'function' ? child(this.state.content) : this.state.content || child;
};

export function withSlot(name, alias) {
  return Child => props =>
    h(Slot, { name }, content => {
      let childProps = {};
      childProps[alias || name] = content;
      for(let i in props) childProps[i] = props[i];
      return h(Child, childProps);
    });
}
