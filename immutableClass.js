import { defineGetterSetter, fnName, isArrowFunction } from './lib/misc.js';
/* jshint esversion: 6 */

const immutableClass = (orig, ...proto) => {
  let name = Util.fnName(orig).replace(/Mutable/g, '');
  let imName = 'Immutable' + name;
  proto = proto || [];

  let initialProto = proto.map(p =>
    Util.isArrowFunction(p)
      ? p
      : ctor => {
          for(let n in p) ctor.prototype.n = p.n;
        }
  );

  let body = `class ${imName} extends ${name} {
      constructor(...args) {
        super(...args);
        if(new.target === ${imName})
          return Object.freeze(this);
      }
    };
    
    ${imName}.prototype.constructor = ${imName};
    
    return ${imName};`;

  for(let p of initialProto) p(orig);
  let ctor = new Function(name, body)(orig);

  //console.log('immutableClass', { initialProto, body }, orig);

  let species = ctor;

  /* prettier-ignore */ Util.defineGetterSetter(ctor, Symbol.species, (() => species), ((value) => {
      species = value;
    }));

  return ctor;
};
