function getModuleProps(m) {
  let obj = {};
  for(let prop of Object.getOwnPropertyNames(globalThis)
    .filter(n => /getModule/.test(n))
    .map(n => n.slice(9))) {
    if(!/Props|Object/.test(prop)) {
      const value = globalThis['getModule' + prop](m);
      if(value !== undefined) obj[prop.toLowerCase()] = value;
    }
  }
  return obj;
}
