import { EagleEntity } from "./lib/eagle/entity.js";
import { EagleDocument } from "./lib/eagle/document.js";
import { EagleProject } from "./lib/eagle/project.js";
import { EaglePath } from "./lib/eagle/locator.js";
import Util from "./lib/util.js";
import util from "util";
import deep from "./lib/deep.js";
import DeepDiff from "deep-diff";
import { Console } from "console";
import { inspect, toXML } from "./lib/eagle/common.js";

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
});
function dump(o, depth = 2, breakLength = 400) {
  let s;
  if(o instanceof Array) {
    s = "";
    for(let i of o) {
      if(s.length > 0) s += i instanceof EagleEntity ? ",\n" : ", ";
      s += dump(i, depth - 1, breakLength);
    }
  } else if(o instanceof EagleEntity) {
    s = inspect(o, undefined, { depth, path: false });
    depth * 4;
  } else s = util.inspect(o, { depth, colors: true, breakLength });
  return s;
}
function xmlize(obj, depth = 2) {
  return obj.toXML ? obj.toXML().replace(/>\s*</g, ">\n    <") : EagleDocument.toXML(obj, depth).split(/\n/g)[0];
}
function testLocator() {
  let testobj = [0, 1, 2, { name: "roman", children: ["x", "y", { id: 1, items: ["a", "b", "c"] }] }];
  let l = new EaglePath([3, "children", 2, "items", -2]);
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
  let { board, schematic } = proj;

  const getPackage = (d, e) => {
    let part;
    if(e.tagName == "instance") {
      part = e.part;
      let deviceset = part.deviceset;
      const device = deviceset.get("device", part.attributes.device);
      return device.package;
    }
    return e.package;
  };

  //return;
  /*  console.log("board.path:", board.path);
  console.log("board.owner:", board.owner);*/
  /* const pred = v => (v.tagName == "instance" || v.tagName == "element") && "attributes" in v;
  const tran = ([v, l, d]) => new EagleEntity(d, l, v);*/
  const packages = {
    board: [...proj.board.getAll("package")],
    schematic: [...proj.schematic.getAll("package")]
  };
  let e = proj.board.getByName("element", "T1");
  console.log("e:", e);
  console.log("e.package:", e.package);
  // console.log("elements:", [...proj.board.getAll("element")]);

  for(let element of proj.board.getAll("element", (v, p, o) => v)) {
    console.log("element:", dump(element, 1));
    const packageName = element.attributes.package;
    console.log("packageName:", packageName);
    continue;

    let instances = [...proj.schematic.getAll("instance")];
    let devices = [...proj.schematic.findAll("device")].map(device => ({ names: device.names, device, package: device.package }));
    let parts = [...proj.schematic.getAll("part")];
    console.log("parts: ", proj.schematic.parts);
    console.log("libraries: ", proj.schematic.libraries);
    console.log("parts.length: ", proj.schematic.parts.length);
    console.log("parts.keys(): ", Object.keys(proj.schematic.parts));
    console.log("[...parts]:", [...proj.schematic.parts]);

    let part = parts.find(e => (e.attributes.device.length > 0 && e.attributes.device == packageName.substring(0, e.attributes.device.length)) || e.attributes.deviceset == packageName.substring(0, e.attributes.deviceset.length));
    console.log(
      "devices: ",
      devices.filter(d => !d.package)
    );
    console.log("part: ", part);
    console.log("part.library: ", part.library);
    console.log("part.deviceset: ", part.deviceset);
    console.log("part.device: ", part.device);
    console.log("part.device.names: ", part.device.names);
    console.log("part.library.parentNode: ", part.library.parentNode);
    console.log("part.deviceset.parentNode: ", part.deviceset.parentNode);
    console.log("part.device.parentNode: ", part.device.parentNode);
    /*    console.log("part.library.devicesets: ",  part.library.devicesets);
    console.log("deviceset: ",  part.deviceset);
    console.log("device: ",  part.device);*/
    //  console.log("instances: ", instances );
    let instance = instances.filter(e => e.attributes.part == packageName.substring(0, e.attributes.part.length));
    console.log("instance:", instance); //console.log("owner:", dump(element.owner, 1));

    const pkgs = [element.package, part.device.package];
    console.log("pkgs:", dump(pkgs, 1));

    if(pkgs[0] != pkgs[1]) {
      console.log(dump(element, 1));
      console.log(dump(instance.part, 1));

      console.log(dump(pkgs, 1));

      const checkPkg = pkgName => {
        let bpkg = board.get("package", pkgName);
        let spkg = schematic.get("device", pkgName);

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
    proj.libraries.map(lib => lib.basename)
  );
  let layers = { board: [], schematic: [] };
  let allLayers = {
    schematic: [
      ...schematic.findAll(
        v => v.tagName == "layer",
        ([v, l, d]) => v.attributes
      )
    ],
    board: [
      ...board.findAll(
        v => v.tagName == "layer",
        ([v, l, d]) => v.attributes
      )
    ]
  };
  console.log(`allLayers:`, dump(allLayers, 2));
  console.log("schematic.layers:", schematic.get("layers"));
  console.log("schematic.layers.all:\n" + [...schematic.getAll("layer", l => dump(l))].join("\n"));
  //  console.log("schematic.layer:", schematic.get("layer", 8, "number"));

  console.log("schematic.cache.libraries:", schematic.cache.libraries);
  console.log("schematic.cache.instances:", schematic.cache.instances);
  console.log("board.cache.elements:", board.cache.elements);
  let parts = schematic.parts;
  console.log("schematic.parts:", Util.className(parts));
  console.log("schematic.parts:", [...parts]);
  console.log("schematic.parts.length:", parts.length);
  console.log("schematic.parts[0]:", parts[0]);
  console.log("schematic.parts[1]:", parts[1]);
  // console.log("schematic.get('library','r'):", schematic.getByName('library','r'));
  console.log("schematic.parts[1].library:", parts[1].library);
  let libraries = schematic.libraries;
  console.log("schematic.libraries:", libraries);
  console.log("schematic.libraries:", [...libraries]);
  console.log("schematic.libraries.length:", libraries.length);
  //console.log("schematic.parts.keys:", Reflect.ownKeys(parts).join(", "));
  console.log("schematic.parts.has(T1):", Reflect.has(parts, "T1"));
  console.log("schematic.parts.has(1):", Reflect.has(parts, 1));
  let firstPart = parts[0];
  console.log("firstPart:", firstPart);
  console.log("firstPart.parentNode:" + dump(firstPart.parentNode, 3));
  console.log("firstPart.parentNode.parentNode:" + dump(firstPart.parentNode.parentNode, 3));
  console.log("firstPart.chain:" + dump(firstPart.chain.map(Util.className), 1));
  console.log("firstPart.raw:" + dump(firstPart.raw, 1));

  /* console.log("schematic.deviceset.devices:", Util.className(deviceset.devices));
  console.log("schematic.deviceset.devices:", deviceset.devices);*/
  /*  let device = firstPart.device;
  let deviceset = firstPart.deviceset;*/
  console.log("firstPart.library:", firstPart.library);
  //console.log("firstPart.library.getMap('deviceset') :", firstPart.getMap('deviceset'));
  console.log("firstPart.attributes.library:", firstPart.attributes.library);
  console.log("firstPart.attributes.device:", firstPart.attributes.device);
  console.log("firstPart.attributes.deviceset:", firstPart.attributes.deviceset);
  console.log("firstPart.deviceset:", firstPart.deviceset);
  console.log("firstPart.device:", firstPart.device);
  console.log("firstPart.library.devicesets:", [...firstPart.library.cache.devicesets.children]);
  console.log("firstPart.library.devicesets:", firstPart.library.devicesets.find(firstPart.attributes.deviceset));

  /*  console.log("firstPart.library.get('devicesets') :", firstPart.library.getByName('deviceset', firstPart.attributes.deviceset));
  console.log("firstPart.library.getByName('deviceset') :", firstPart.library.getByName('deviceset'));
  //console.log("firstPart.library.devicesets:", firstPart.library.devicesets);
  // return proj.saveTo(".", true);
  /*
  console.log("schematic.cache:", Object.keys(schematic.cache));
  console.log("schematic.deviceset.cacheFields():", deviceset.cacheFields());
  console.log("schematic.deviceset.gates:", dump(deviceset.gates, 3));
  console.log("board.layers:", dump(board.layers.filter(l => l.visible == "yes")));
  console.log("schematic.layers:", dump(schematic.layers.filter(l => l.active == "yes")));
  //  console.log("schematic.deviceset.devices:", dump(deviceset.devices, 3));
  let devices = deviceset.devices;

  //console.log("get", devices.map(dev => ({ path: dev.path, name: dev.name })));

  for(let pkg of Util.concat(...libraries.map(lib => lib.findAll("package")))) {
    //console.log("pkg.name:", pkg.name);
    let other = { board: board.get("package", pkg.name), schematic: schematic.get("package", pkg.name) };

    //console.log("pkg:", dump(pkg.raw, 10));

    for(let k in other) {
      const o = other[k];

      if(typeof o != "object" || !o) continue;

      let ll = o.children.map(child => child.attributes.layer);
      //  let ll2 = o.children.map(child => child.layer);

      layers[k] = layers[k].concat(ll);
      //console.log(`layers.${k}:`,layers[k].join(','));
      // console.log(`layers.${k}:`,ll2.join(','));

      let children = pkg.children.filter(child => [21, 23, 25, 27, 51].indexOf(child.layer) != -1);
      console.log(`children:`, children);

      console.log(`${k}:`, dump(o, 10));

      /*  if(k == "schematic") {
        let diff = deepDiff(pkg.raw, other.schematic.raw);
        console.log(`diff:`, dump(diff, 10));
      }*/

  //  }

  /* for(let key in layers) {
    //  console.log("key:", key, allLayers[key]);
    layers[key] = Util.unique(layers[key]);
    const layerMap = new Map(
      layers[key]
        .filter(layer => layer !== undefined)
        .map(layerId => proj[key].get("layer", layerId, "number"))
        .map(layer => [parseInt(layer.number), layer.name])
        .sort((a, b) => a[0] - b[0])
    );
    console.log(`${key}.layer names:`, [...layerMap.values()].join(","));
    console.log(`${key}.layer keys:`, layerMap.keys());
  }*/
  proj.updateLibrary("c");

  //console.log("found:", [...Util.concat(...libraries.map(lib => lib.findAll("package")))].map(e => [e.owner.basename, e.xpath(), e].join(" ")).join("\n   "));

/*  console.log("schematic.libraries:", schematic.libraries);
  console.log("schematic.find(part,T1):", dump(schematic.find("part", "T1")));
  // console.log("proj.library.c.getMap(packages):",proj.library.c.getMap('package'));
  console.log("proj.library.c.packages:", proj.library.c.packages["E2,5-6/V"]);

  console.log("schematic.parts.T1:", dump(schematic.parts.T1, 10));

  const changes = DeepDiff.diff(schematic.orig, schematic.root);
  console.log("schematic.orig:", dump(schematic.orig, 1));
  console.log("schematic.root:", dump(schematic.root, 1));*/
 // console.log("diff:", dump(changes));
console.log("board:", dump(board.changes, 10)); 
/*const newXml = changes.reduce((acc,change) => { DeepDiff.applyChange(acc, change); return acc; }, board.root);
  console.log("DeepDiff:", Object.keys(DeepDiff));  
  console.log("DeepDiff.applyDiff:", DeepDiff.applyDiff);  
  console.log("newXml:", toXML(board));  */


  return proj.saveTo(".", true);

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
}
(async () => {
  try {
    await testLocator();
    await testEagle("../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3").then(result => console.log(result));
  } catch(err) {
    const stack = err.stack;
    console.log("err:", err);
    console.log("stack:", stack);
    throw err;
  }
})();
