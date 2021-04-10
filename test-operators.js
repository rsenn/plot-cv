'use strict';

/* operators overloading with Operators.create() */
function test_operators_create() {
  class Vec2 {
    constructor(x, y) {
      this.x = x;
      this.y = y;
    }
    static mul_scalar(p1, a) {
      var r = new Vec2();
      r.x = p1.x * a;
      r.y = p1.y * a;
      return r;
    }
    toString() {
      return 'Vec2(' + this.x + ',' + this.y + ')';
    }
  }

  Vec2.prototype[Symbol.operatorSet] = Operators.create({
      '+'(p1, p2) {
        var r = new Vec2();
        r.x = p1.x + p2.x;
        r.y = p1.y + p2.y;
        return r;
      },
      '-'(p1, p2) {
        var r = new Vec2();
        r.x = p1.x - p2.x;
        r.y = p1.y - p2.y;
        return r;
      },
      '=='(a, b) {
        return a.x == b.x && a.y == b.y;
      },
      '<'(a, b) {
        var r;
        /* lexicographic order */
        if(a.x == b.x) r = a.y < b.y;
        else r = a.x < b.x;
        return r;
      },
      '++'(a) {
        var r = new Vec2();
        r.x = a.x + 1;
        r.y = a.y + 1;
        return r;
      },
      '--'(a) {
        var r = new Vec2();
        r.x = a.x - 1;
        r.y = a.y - 1;
        return r;
      },
      '^'(p1,p2) {
        var r =  Math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2);
        return r;
      }
    },
    {
      left: Number,
      '*'(a, b) {
        return Vec2.mul_scalar(b, a);
      }
    },
    {
      right: Number,
      '*'(a, b) {
        return Vec2.mul_scalar(a, b);
      }
    }
  );

 
  var a = new Vec2(10, 5);
  var b = new Vec2(110, 105);
  var r;

console.log(`a * 2 + 3 * b = `, r = a * 2 + 3 * b);
console.log(`b - a = `, b -a );
console.log(`b ^ a = `, b ^a );
  a++;
  r = ++a;
}

/* operators overloading thru inheritance */
function test_operators() {
  var Vec2;

  function mul_scalar(p1, a) {
    var r = new Vec2();
    r.x = p1.x * a;
    r.y = p1.y * a;
    return r;
  }

  var vec2_ops = Operators({
      '+'(p1, p2) {
        var r = new Vec2();
        r.x = p1.x + p2.x;
        r.y = p1.y + p2.y;
        return r;
      },
      '-'(p1, p2) {
        var r = new Vec2();
        r.x = p1.x - p2.x;
        r.y = p1.y - p2.y;
        return r;
      },
      '=='(a, b) {
        return a.x == b.x && a.y == b.y;
      },
      '<'(a, b) {
        var r;
        /* lexicographic order */
        if(a.x == b.x) r = a.y < b.y;
        else r = a.x < b.x;
        return r;
      },
      '++'(a) {
        var r = new Vec2();
        r.x = a.x + 1;
        r.y = a.y + 1;
        return r;
      },
      '--'(a) {
        var r = new Vec2();
        r.x = a.x - 1;
        r.y = a.y - 1;
        return r;
      },
      '^'(p1,p2) {
        var r =  Math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2);
        return r;
      }
    },
    {
      left: Number,
      '*'(a, b) {
        return mul_scalar(b, a);
      }
    },
    {
      right: Number,
      '*'(a, b) {
        return mul_scalar(a, b);
      }
    }
  );

  Vec2 = class Vec2 extends vec2_ops {
    constructor(x, y) {
      super();
      this.x = x;
      this.y = y;
    }
    toString() {
      return 'Vec2(' + this.x + ',' + this.y + ')';
    }
  };

  var a = new Vec2(10, 5);
  var b = new Vec2(110, 105);
  var r;

console.log(`a * 2 + 3 * b = `, r = a * 2 + 3 * b);
console.log(`b - a = `, b -a );
console.log(`b ^ a = `, b ^a );
  a++;
  r = ++a;
}

function test_default_op() {}

test_operators_create();
test_operators();
test_default_op();
