import { EagleEntity } from "./lib/eagle/entity.js";
import { EagleDocument } from "./lib/eagle/document.js";
import { EagleProject } from "./lib/eagle/project.js";
import { EagleLocator } from "./lib/eagle/locator.js";
import Util from "./lib/util.js";
import util from "util";
import deep from "./lib/deep.js";
import deepDiff from "deep-diff";
import { Console } from "console";
import { inspect } from "./lib/eagle/common.js";

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
});
function dump(o, depth = 2, breakLength = 400) {
  let s;
  if(o instanceof EagleEntity) {
    s = inspect(o, undefined, { depth, location: false });
    depth * 4;
  } else s = util.inspect(o, { depth, colors: true, breakLength });
  return s;
}
function xmlize(obj, depth = 2) {
  return obj.toXML ? obj.toXML().replace(/>\s*</g, ">\n    <") : EagleDocument.toXML(obj, depth).split(/\n/g)[0];
}
function testLocator() {
  let testobj = [0, 1, 2, { name: "roman", children: ["x", "y", { id: 1, items: ["a", "b", "c"] }] }];
  let l = new EagleLocator([3, "children", 2, "items", -2]);
  let a = [l.slice(), l.slice()];
  console.log("l:", dump(l));
  console.log("a[0] == a[1]:", a[0] === a[1]);
  a[1][0] = "x";
  console.log("a:", dump(a));
  let b = [l.prevSibling, l.nextSibling, l.parent];
  console.log("b:", dump(b));
  console.log(b[2].parent.parent.up(3));
  console.log("apply:", l.apply(testobj));
}

async function testEagle(filename) {
  let proj = new EagleProject(filename);
  let { board, schematic, libraries } = proj;

  const getPackage = (d, e) => {
    let part;
    if(e.tagName == "instance") {
      part = e.part;
      let deviceset = part.deviceset;
      const device = deviceset.getByName("device", part.attributes.device);
      return device.package;
    }
    return e.package;
  };

  //return;
  /*  console.log("board.location:", board.location);
  console.log("board.owner:", board.owner);*/
  /* const pred = v => (v.tagName == "instance" || v.tagName == "element") && "attributes" in v;
  const tran = ([v, l, d]) => new EagleEntity(d, l, v);*/
  const packages = {
    board: [...proj.board.getAll("package")],
    schematic: [...proj.schematic.getAll("package")]
  };

  for(let element of proj.board.getAll("element")) {
    let instance = proj.schematic.getByName("instance", element.attributes.name, "part");
    //console.log(dump(element,1));
    console.log("owner:", dump(instance.owner, 1));

    const pkgs = [element.package.name, instance.part.device.package.name];
    console.log("pkgs:", dump(pkgs, 1));

    if(pkgs[0] != pkgs[1]) {
      console.log(dump(element, 1));
      console.log(dump(instance.part, 1));

      console.log(dump(pkgs, 1));

      const checkPkg = pkgName => {
        let bpkg = board.getByName("package", pkgName);
        let spkg = schematic.getByName("device", pkgName);

        if(!!(bpkg && spkg)) console.log(["board:     " + dump(bpkg, 1), "schematic: " + dump(spkg, 1)].join("\n"));
        return !!(bpkg && spkg);
      };
      let res = pkgs.map(pkg => checkPkg(pkg));

      let index = res.indexOf(true);

      if(index != -1) {
        instance.part.device = pkgs[index];
        console.log(instance.part.device, pkgs[index]);
        element.package = pkgs[index];
      }
      console.log(res, index);
    }
  }
  console.log(
    "libs:",
    libraries.map(lib => lib.basename)
  );

  for(let pkg of Util.concat(...libraries.map(lib => lib.findAll("package")))) {
    console.log("pkg.name:", pkg.name);
    let other = { board: board.find("package", pkg.name), schematic: schematic.find("package", pkg.name) };
      

      console.log("pkg:", dump(pkg.raw, 10));

    for(let k in other) {
      const o = other[k];
      if(typeof o != "object" || !o) continue;
      
      console.log(`${k}:`, dump(o, 10));

      if(k == 'schematic') {
      let diff = deepDiff(pkg.raw, other.schematic.raw);
        console.log(`diff:`, dump(diff, 10));
      }
    }

  }

  console.log("found:", dump(schematic.find("part", "T1")));
  console.log("found:", [...Util.concat(...libraries.map(lib => lib.findAll("package")))].map(e => [e.owner.basename, e.xpath(), e.toXML()].join("\n  ")).join("\n   "));

  console.log("schematic:", schematic.changes);
  console.log("board:", board.changes);

  /* for(let it of schematic.iterator(["children", "0", "children", "0", "children", "0"], t => t)) console.log("elem:", it);

  console.log("schematic:", schematic.getByName("instance", "T1", "part"));

  console.log("schematic:", schematic.changes);
  console.log("board:", board.xml);

  for(let [value, path] of deep.iterate(schematic.xml, (v, p) => (p.length > 1 ? p[p.length - 2] == "children" : true))) console.log("iterate: ", value, `[${path.map(p => (typeof p == "string" ? `'${p}'` : p)).join(",")}]`);

  let elem = deep.get(schematic.xml, ["0", "children", "0", "children", "0", "children", "3", "children", "6", "children", "0", "children", "3", "children", "11", "children", "0", "children", "2"]);
  console.log("elem: ", elem);*/

  /*    try {
      for(let e of Util.concat(proj.board.getAll("element"), proj.schematic.getAll("instance"))) {
        const part = e.part;
        const deviceset = part && part.deviceset;
        const device = part && part.device;
        const pkg = e.package || device.package; // = getPackage(schematic, e);

        if(part) {
          console.log("device.attributes.package:", device.attributes.package);
          if(device.attributes.package && !pkg) console.log(`Package ${device.attributes.package} not found!`);
          console.log(dump(part, 1));
          console.log(dump(deviceset, 1));
          console.log(dump(device, 1));
        } else {
          console.log(dump(e, 1));
        }
        console.log(dump(pkg, 1));
      }
    } catch(error) {
      const { stack } = error;
      console.log(
        "error:",
        error
          .toString()
          .split(/\n/g)
          .slice(0, 10),
        stack
      );
      throw new Error("err");
    }*/
  return proj.saveTo(".", true);
}
(async () => {
  try {
    await testLocator();
    await testEagle("../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3").then(result => console.log(result));
  } catch(err) {
    const stack = err.stack;
    console.log("err:", err.toString());
    console.log("stack:", stack);
    throw err;
  }
})();
