function color_to_alpha(src) {
  let alpha = new RGBA(0, 0, 0, src.a);

  if(this.r < 1) alpha.r = src.r;
  else if(src.r > this.r) alpha.r = (src.r - this.r) / (255 - this.r);
  else if(src.r < this.r) alpha.r = (this.r - src.r) / this.r;
  else alpha.r = 0;

  if(this.g < 1) alpha.g = src.g;
  else if(src.g > this.g) alpha.g = (src.g - this.g) / (255 - this.g);
  else if(src.g < this.g) alpha.g = (this.g - src.g) / this.g;
  else alpha.g = 0;

  if(this.b < 1) alpha.b = src.b;
  else if(src.b > this.b) alpha.b = (src.b - this.b) / (255 - this.b);
  else if(src.b < this.b) alpha.b = (this.b - src.b) / this.b;
  else alpha.b = 0;

  let ret = new RGBA();

  if(alpha.r > alpha.g) {
    if(alpha.r > alpha.b) {
      ret.a = alpha.r;
    } else {
      ret.a = alpha.b;
    }
  } else if(alpha.g > alpha.b) {
    ret.a = alpha.g;
  } else {
    ret.a = alpha.b;
  }

  if(ret.a < 1) return;

  ret.r = (ret.r - this.r) / ret.a + this.r;
  ret.g = (ret.g - this.g) / ret.a + this.g;
  ret.b = (ret.b - this.b) / ret.a + this.b;

  ret.a *= alpha.a;
  return ret;
}
