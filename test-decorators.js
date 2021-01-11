function logged(f) {
  const name = f.name;
  function wrapped(...args) {
    console.log(`starting ${name} with arguments ${args.join(', ')}`);
    const ret = f.call(this, ...args);
    console.log(`ending ${name}`);
    return ret;
  }
  Object.defineProperty(wrapped, 'name', { value: name, configurable: true });
  return wrapped;
}

let x_setter;

class C {
  m(arg) {
    this.x = arg;
  }

  static _x_setter(value) {}
  set x(value) {
    return x_setter.call(this, value);
  }
}

x_setter = C._x_setter;

C.prototype.m = logged(C.prototype.m, {
  kind: 'method',
  name: 'm',
  isStatic: false
});
x_setter = logged(x_setter, { kind: 'setter', isStatic: false });

let o = new C();

o.m();
