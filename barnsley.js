$(function () {
  var scale = 60;
  var numberOfPoints = 4000 * scale;

  var setupCanvas = function() {
    $('#painthere').empty();
    $('#painthere').append($('<canvas></canvas>'));
    return $('#painthere canvas')[0];
  };
  var canvSize = { width: 10 + scale * 8.4, height: 4 + scale * 7.3 };
  var slant = -0.625;
  function paintfern() {
    //Farnsley
    var canvas = setupCanvas();
    $(canvas).css('background-color', 'whitesmoke');
    $(canvas).css('opacity', 0.8);
    canvas.width = canvSize.width;
    canvas.height = canvSize.height;
    ctx = canvas.getContext('2d');
    ctx.translate(6, canvSize.height + scale * Math.sin(Math.PI * slant));
    ctx.rotate(Math.PI * slant);
    ctx.fillStyle = 'darkgreen';

    x = y = a = b = c = d = f = 0;
    for(i = 0; i < numberOfPoints; i++) {
      let r = Math.random();

      if(r < 0.02) {
        a = b = c = f = 0;
        d = 0.16;
      } else if(r < 0.86) {
        a = d = 0.85;
        b = 0.04;
        c = -0.04;
        f = 1.6;
      } else if(r < 0.93) {
        a = 0.2;
        b = -0.26;
        c = 0.23;
        d = 0.22;
        f = 1.6;
      } else {
        a = -0.15;
        b = 0.28;
        c = 0.26;
        d = 0.24;
        f = 0.44;
      }

      let xs = x;
      x = a * x + b * y;
      y = c * xs + d * y + f;
      ctx.fillRect(x * scale, y * scale, 1, 1);
    }
  }
  paintfern();
  $('#painthere').on('click', 'canvas', paintfern);
});
