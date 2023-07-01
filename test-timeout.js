var t, u, r;

u = function() {
  clearTimeout(r);
  setTimeout(() => console.log('done'));
};

r = setTimeout(u, 100);
