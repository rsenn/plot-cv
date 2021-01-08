function pick(p) {
  var r = Math.random();
  for (var i = 0; i < p.length; i++) {
    if (r < p[i]) return i;
  }
  return p.length - 1;
}
// the iterated function system (IFS) "code" for Barnsley's fern.
// consists of four matrices and a probability for each.
var fern_M = [
  [
    [0, 0, 0],
    [0, 0.16, 0],
    [0, 0, 1]
  ],
  [
    [0.85, 0.04, 0],
    [-0.04, 0.85, 1.6],
    [0, 0, 1]
  ],
  [
    [0.2, -0.26, 0],
    [0.23, 0.22, 1.6],
    [0, 0, 1]
  ],
  [
    [-0.15, 0.28, 0],
    [0.26, 0.24, 0.44],
    [0, 0, 1]
  ]
];
var fern_P = cumsum([0.01, 0.85, 0.07, 0.07]);
function ifs(M, p, niter) {
  var pt = [0.5, 0.5, 1]; // start at an arbitrary point
  for (var i = 1; i < niter + 10; i++) {
    pt = mul(M[pick(p)], pt);
    if (i > 10) {
      // wait 10 iterations to make sure we approach the attractor
      point((pt[0] + 4) / 14, 1 - pt[1] / 10.2);
    }
  }
}
ctx.fillStyle = 'black';
function point(x, y) {
  ctx.fillRect(x * W, y * H, 1, 1);
}
ifs(fern_M, fern_P, 16000);
function cumsum(a) {
  var r = [];
  var s = 0;
  for (var i = 0; i < a.length; i++) {
    s += a[i];
    r.push(s);
  }
  return r;
}
function mul(m, p) {
  var r = [];
  for (var j = 0; j < m.length; j++) {
    var s = 0;
    for (var i = 0; i < p.length; i++) {
      s += p[i] * m[j][i];
    }
    r.push(s);
  }
  return r;
}
