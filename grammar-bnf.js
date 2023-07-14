
function wrap(parser, name) {
  return (str, pos) => {
    let r = parser(str, pos);
    if(r[0] || name.startsWith('direct')) console.log('matched (' + name + ') ' + pos + ' - ' + r[2] + ": '", r[1], "'");
    return r;
  };
}

export default {};