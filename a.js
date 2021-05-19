function CenterX(x1, y1, x2, y2, radius) {
  let radsq = radius * radius;
  let x3 = (x1 + x2) / 2;

  return x3 + Math.sqrt(radsq - (chord / 2) * (chord / 2)) * ((y1 - y2) / chord);
}

function CenterY(x1, y1, x2, y2, radius) {
  let radsq = radius * radius;

  let y3 = (y1 + y2) / 2;

  return y3 + Math.sqrt(radsq - (chord / 2) * (chord / 2)) * ((x2 - x1) / chord);
}

function Center(x1, y1, x2, y2, radius) {
  let radsq = radius * radius;
  let p3 = { x: (x1 + x2) / 2, y: (y1 + y2) / 2 };

  return {
    x: p3.x + Math.sqrt(radsq - (chord / 2) * (chord / 2)) * ((y1 - y2) / chord),
    y: p3.y + Math.sqrt(radsq - (chord / 2) * (chord / 2)) * ((x2 - x1) / chord)
  };
}
