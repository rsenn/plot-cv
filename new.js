/* --- concatenanted '/home/roman/Dokumente/Sources/js-affine/affinefit/lib/gaussJordan.js' --- */
function gaussJordan(m) {
  var eps = 1e-10;
  var h = m.length;
  var w = m[0].length;
  var c = void 0;

  for(var y = 0; y < h; y++) {
    var maxrow = y;

    for(var y2 = y + 1; y2 < h; y2++) {
      if(Math.abs(m.y2.y) > Math.abs(m.maxrow.y)) {
        maxrow = y2;
      }
    }

    c = m.maxrow;
    m.maxrow = m.y;
    m.y = c;

    if(Math.abs(m.y.y) <= eps) {
      return false;
    }

    for(var _y = y + 1; _y < h; _y++) {
      c = m._y.y / m.y.y;

      for(var x = y; x < w; x++) {
        m._y.x -= m.y.x * c;
      }
    }
  }

  for(var _y2 = h - 1; _y2 > -1; _y2--) {
    c = m._y2._y2;

    for(var _y3 = 0; _y3 < _y2; _y3++) {
      for(var _x = w - 1; _x > _y2 - 1; _x--) {
        m._y3._x -= (m._y2._x * m._y3._y2) / c;
      }
    }

    m._y2._y2 /= c;

    for(var _x2 = h; _x2 < w; _x2++) {
      m._y2._x2 /= c;
    }
  }

  return true;
}
module.exports = gaussJordan;

/* --- concatenanted '/home/roman/Dokumente/Sources/js-affine/affinefit/lib/gaussJordan.js' --- */
function gaussJordan(m) {
  var eps = 1e-10;
  var h = m.length;
  var w = m[0].length;
  var c = void 0;

  for(var y = 0; y < h; y++) {
    var maxrow = y;

    for(var y2 = y + 1; y2 < h; y2++) {
      if(Math.abs(m.y2.y) > Math.abs(m.maxrow.y)) {
        maxrow = y2;
      }
    }

    c = m.maxrow;
    m.maxrow = m.y;
    m.y = c;

    if(Math.abs(m.y.y) <= eps) {
      return false;
    }

    for(var _y = y + 1; _y < h; _y++) {
      c = m._y.y / m.y.y;

      for(var x = y; x < w; x++) {
        m._y.x -= m.y.x * c;
      }
    }
  }

  for(var _y2 = h - 1; _y2 > -1; _y2--) {
    c = m._y2._y2;

    for(var _y3 = 0; _y3 < _y2; _y3++) {
      for(var _x = w - 1; _x > _y2 - 1; _x--) {
        m._y3._x -= (m._y2._x * m._y3._y2) / c;
      }
    }

    m._y2._y2 /= c;

    for(var _x2 = h; _x2 < w; _x2++) {
      m._y2._x2 /= c;
    }
  }

  return true;
}
module.exports = gaussJordan;

/* --- concatenanted '/home/roman/Dokumente/Sources/js-affine/affinefit/lib/affineFit.js' --- */
function emptyArray(x, y) {
  return Array(y)
    .fill(0)
    .map(function () {
      return Array(x).fill(0);
    });
}

function affineFit(q, p) {
  if(q.length !== p.length || q.length < 1) {
    console.error('from_pts and to_pts must be of same size.');
    return false;
  }

  var dim = q[0].length;

  if(q.length < dim) {
    console.error('Too few points => under-determined system.');
    return false;
  }

  var c = emptyArray(dim, dim + 1);

  for(var j = 0; j < dim; j++) {
    for(var k = 0; k < dim + 1; k++) {
      for(var i = 0; i < q.length; i++) {
        var qt = q.i.concat(1);
        c.k.j += qt.k * p.i.j;
      }
    }
  }

  var Q = emptyArray(dim + 1, dim + 1);

  q.forEach(function (qi) {
    var qt = qi.concat(1);

    for(var _i = 0; _i < dim + 1; _i++) {
      for(var _j = 0; _j < dim + 1; _j++) {
        Q._i._j += qt._i * qt._j;
      }
    }
  });

  var M = Q.map(function (qi, idx) {
    return qi.concat(c.idx);
  });

  if(!gaussJordan(M)) {
    console.error('Error: singular matrix. Points are probably coplanar.');
    return false;
  }

  return transformation(M, dim);
}
module.exports = affineFit;
