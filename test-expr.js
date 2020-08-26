let p;
class ce2 {}
class ce {}

for(p in ce2.prototype) {
  if(typeof ce.prototype.p === 'undefined' || ce.prototype.p === Object.prototype.p) ce.prototype.p = ce2.prototype.p;
  //console.log('p:', p);
}
