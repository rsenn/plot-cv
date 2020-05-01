import { EagleElement } from "./lib/eagle/element.js";
import { EagleDocument } from "./lib/eagle/document.js";
import { EagleProject } from "./lib/eagle/project.js";
import { EaglePath } from "./lib/eagle/locator.js";
import Util from "./lib/util.js";
import fs, { promises as fsPromises } from "fs";
import deep from "./lib/deep.js";
import DeepDiff from "deep-diff";
import { Console } from "console";
import { inspect, toXML, dump } from "./lib/eagle/common.js";
import { JsonPointer, JsonReference } from "./lib/json-pointer.js";
import ptr from "./lib/json-ptr.js";

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
});

/**/
function xmlize(obj, depth = 2) {
  return obj.toXML
    ? obj.toXML().replace(/>\s*</g, ">\n    <")
    : EagleDocument.toXML(obj, depth).split(/\n/g)[0];
}
function testLocator() {
  let testobj = [
    0,
    1,
    2,
    { name: "roman", children: ["x", "y", { id: 1, items: ["a", "b", "c"] }] }
  ];
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

function testProxyTree() {
  let tree = Util.proxyTree((path, key, value) => {
    console.log(`: [${dump(path)}].set(`, key, value, `)`);
    return true;
  });
  tree.a.b.c.d("test");
  tree.a.b.c.d.e = 0;
  tree.a.b.c.d.e[0] = 1;
}

function testProxyClone() {
  let obj = {
    blah: [1, 2, 3, 4],
    test: { text: "eruoiewurew", name: "haha" },
    num: 41
  };

  let clone = Util.proxyClone(obj);

  obj.addProp = "1234";
  clone.newProp = "test";

  console.log("obj:", obj);
  console.log("clone:", Object.keys(clone));
  console.log("clone.addProp:", clone.addProp);
  console.log("clone.newProp:", clone.newProp);
  console.log("clone.blah:", clone.blah);
  console.log("clone.blah[0]", clone.blah[0]);
}

function testJsonPointer() {
  var data = {
    legumes: [
      {
        name: "pinto beans",
        unit: "lbs",
        instock: 4
      }, {
        name: "lima beans",
        unit: "lbs",
        instock: 21
      }, {
        name: "black eyed peas",
        unit: "lbs",
        instock: 13
      }, {
        name: "plit peas",
        unit: "lbs",
        instock: 8
      }
    ]
  };
  var pointer = ptr.append(ptr.nil, "legumes", 0);
  var pointer2 = ptr.append(pointer, "name");
  console.log("pointer:", pointer);
  console.log("pointer2:", pointer2);
  console.log("ptr.get:", ptr.get(pointer)(data));

  ptr.assign(pointer2)(data, "test name");

  console.log("ptr.get:", ptr.get(pointer2)(data));

  console.log("JsonPointer.flatten:", JsonPointer.flatten(data));
  console.log("JsonPointer.map:", JsonPointer.map(data));
  /*
  var ref = new JsonReference(ptr.create("/legumes/3"));
  console.log("ref.resolve:",ref.resolve(data));*/
}
const filesystem = {
  readFile(filename) {
    let data = fs.readFileSync(filename).toString();
    console.log(`Read ${filename} ${data.length} bytes`);
    return data;
  }, writeFile(filename, data, overwrite = true) {
    return fs.writeFileSync(filename, data, { flag: overwrite ? "w" : "wx" });
  }, exists(filename) {
    return fs.existsSync(filename);
  }
};

async function testEagle(filename) {
  let proj = new EagleProject(filename, filesystem);
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

  const packages = {
    board: [...proj.board.getAll("package")],
    schematic: [...proj.schematic.getAll("package")]
  };
  let e = proj.board.elements.T1;
  //("element", "T1");
  console.log("proj.board.packages:", proj.board.cache);

  console.log("e:", e);
  console.log("e.package:", e.package);
  /*
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
    console.log("devices: ",
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

    let instance = instances.filter(e => e.attributes.part == packageName.substring(0, e.attributes.part.length));
    console.log("instance:", instance);

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
  console.log("libs:",
    proj.libraries.map(lib => lib.basename)
  );
  let layers = { board: [], schematic: [] };
  let allLayers = {
    schematic: [
      ...schematic.findAll(v => v.tagName == "layer",
        ([v, l, d]) => v.attributes
      )
    ],
    board: [
      ...board.findAll(v => v.tagName == "layer",
        ([v, l, d]) => v.attributes
      )
    ]
  };*/
  //  console.log("schematic.layers.all:\n" + [...schematic.getAll("layer", l => dump(l))].join("\n"));

  console.log("schematic.cache.libraries:", schematic.cache.libraries);
  console.log("schematic.cache.parts:", Util.className(schematic.cache.parts));
  console.log("schematic.cache.instances:", schematic.cache.instances);

  /*
    console.log("schematic.cache.libraries.children:", schematic.cache.libraries.children);
    console.log("schematic.cache.parts.children:", schematic.cache.parts.children);
    console.log("schematic.cache.instances.children:", schematic.cache.instances.children);*/
  /*
  console.log("board.cache.elements:", board.cache.elements);*/
  let parts = schematic.parts;
  console.log("schematic.parts.ref:", parts.ref);
  console.log("schematic.parts.raw:", parts.raw);
  console.log("schematic.parts:", Util.className(schematic.parts));
  console.log("schematic.parts.keys():", schematic.parts.keys());
  console.log("schematic.parts.entries():", schematic.parts.entries());
  console.log("schematic.layers:", schematic.layers);
  /* console.log("schematic.layers.map():", schematic.layers.map());
   console.log("schematic.layers.map():", schematic.layers.map('name'));*/
  console.log("schematic.parts.list:", schematic.parts.list);
  console.log("schematic.parts.list[0]:", schematic.parts.list[0]);
  /*  console.log("schematic.libraries:", schematic.libraries);
  console.log("schematic.libraries.keys():", schematic.libraries.keys());
  console.log("schematic.libraries.d:", schematic.libraries.d);
  console.log("Util.isBrowser():", Util.isBrowser());

  console.log("schematic.parts.size:", parts.size);
  console.log("schematic.parts.keys():", parts.keys());
  console.log("schematic.parts[0]:", parts[0]);
  console.log("schematic.parts[1]:", parts[1]);

  let libraries = schematic.libraries;
  console.log("schematic.libraries:", libraries);
  console.log("schematic.libraries:", [...libraries]);
  console.log("schematic.libraries.length:", libraries.length);

  console.log("schematic.parts.has(T1):", Reflect.has(parts, "T1"));
  console.log("schematic.parts.has(1):", Reflect.has(parts, 1));

  console.log("parts.keys():", parts.keys());
  //console.log("parts.raw:", parts.raw);
  console.log("parts.ref:", parts.ref);
  console.log("parts.list:", parts.list);*/

  let firstPart = parts[0];
  /* console.log("firstPart:", firstPart);
  console.log("firstPart:" + dump(firstPart, 3));*/
  //console.log("firstPart.parentNode.parentNode:" + dump(firstPart.parentNode.parentNode, 3));*/
  /* console.log("firstPart.chain:" + dump(firstPart.chain.map(Util.className), 1));
  console.log("firstPart.raw:" + dump(firstPart.raw, 1));*/

  // console.log("firstPart.raw:", firstPart.raw);

  /* console.log("firstPart.attributes.library:", firstPart.raw.attributes.library);
  console.log("firstPart.attributes.device:", firstPart.attributes.device);
  console.log("firstPart.attributes.deviceset:", firstPart.attributes.deviceset);
  console.log("firstPart.deviceset:", firstPart.deviceset);
  console.log("firstPart.device:", firstPart.device);
  console.log("firstPart.library.devicesets:", [...firstPart.library.cache.devicesets.children]);
  console.log("firstPart.library.devicesets:", firstPart.library.devicesets.find(firstPart.attributes.deviceset));

  console.log(`proj.library.c:`, proj.library.c);
  console.log(`proj.library.c.packages:`, proj.library.c.packages);*/
  //console.log(`proj.library.c.packages[0]:`, proj.library.c.packages[0]);

  proj.updateLibrary("c");

  //console.log("board:", dump(board.changes, 10));

  console.log(`proj.library.c:`, proj.library.c);
  /*
  for(let pad of board.findAll('pad')) {

  }
*/
  /* for(let pad of board.iterator([], ([v,l,d]) => new EagleElement(board,l))) {
    if(pad.tagName === 'pad')
    console.log("pad:", pad.toXML(0));
  }*/

  //console.log(board.toXML());

  for(let lib of board.libraries.list)
    for(let pkg of lib.packages.list)
      for(let pad of pkg.children) {
        if(pad.tagName !== "pad") continue;

        pad.setAttribute("drill", "0.7");
        pad.setAttribute("diameter", "1.778");
        pad.removeAttribute("stop");
        pad.removeAttribute("rot");
        pad.removeAttribute("shape");

        console.log("pad():", pad.toXML());
        /*  console.log("pad:", pad.path.toString());
    console.log("pad:", pad.xpath().split(/\//g).slice(5).join("/"));*/
      }
  let cmds = [];
  for(let elem of board.elements.list) {
    cmds.push(`MOVE ${elem.name} ${elem.pos};`);
    if(elem.rot) cmds.push(`ROTATE ${elem.rot} ${elem.name};`);
  }

  console.log(cmds.join(" "));

  /*testProxyTree();
  testProxyClone();*/
  // testJsonPointer();
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
      console.log("error:",
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
  let args = process.argv.slice(2);
  if(args.length == 0) args.unshift("../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3");

  for(let arg of args) {
    try {
      //testLocator();

      let r = await testEagle(arg).then(result => console.log(result));
      console.log("r:", r);
    } catch(err) {
      const stack = err.stack;
      console.log("err:", err);
      console.log("stack:", stack);
      throw err;
    }
  }
})();
