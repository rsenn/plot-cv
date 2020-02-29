export const inspect = (obj, pred = v => true) => (
  "{\n  " +
  Object.entries(obj)
    .map(([key, value]) => {
      if(pred(value,key) == false)
        return '';
      let out = value;
      if(typeof out != "string") {
        try {
          if(typeof out == "object") out = inspect(out, pred);
          else out = out + "";
        } catch(err) {
          out = typeof out;
        }
        if(typeof value == "function") {
          let idx = out.indexOf("{");
          out = out.substring(0, idx).trim();
        }
      } else {
        out = '"' + out + '"';
      }
      out = out.replace(/\n\s*/g, " ");
      if(out.length > 200) out = out.substring(0, 200) + "...";
      return key + ": " + out;
    }).filter(item => item != '')
    .join(",\n  ") +
  "\n}"
);

export default inspect;
