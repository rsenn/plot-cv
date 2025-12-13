import INIGrammar from './grammar-INI.js';
import { ReadFile, WriteFile } from './io-helpers.js';
import deep from './lib/deep.js';
import { BBox, Point, Rect, Size } from './lib/geom.js';
import { toXML } from './lib/json.js';
import * as path from './lib/path.js';
import tXml from './lib/tXml.js';
import { Console } from 'console';
async function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      compact: 2,
      customInspect: true,
      maxArrayLength: 200
    }
  });

  if(!args.length) args = [path.gethome() + '/Sources/pictest/build/mplab/7segtest-16f876a-xc8-debug.mcp'];

  let xy = new Point();
  let size = new Size(128, 128);
  let maxWidth = 1360;
  let newSize;
  let spacing = 32;
  let count = 0;
  let iconSize, iconAspect;

  for(let filename of args) {
    let src = ReadFile(filename);

    //console.log('src:', src);
    let [done, data, pos] = INIGrammar.ini(src, 0);

    let createMap = entries => /*Object.fromEntries(entries) ||*/ new Map(entries);

    let sections = data[0].reduce((acc, sdata) => {
      console.log('sdata:', sdata);
      return {
        ...acc,
        [sdata[0]]: createMap(sdata[1] || [])
      };
    }, {});

    const flat = deep.flatten(
      sections,
      new Map(),
      k => k.length > 0,
      (k, v) => [k.slice(1), v]
    );
    console.log('flat:', flat);
    if(sections['Desktop Entry']) {
      const desktopEntry = sections['Desktop Entry'];

      /*  const { Exec, Icon, Terminal, Type, Name, GenericName, StartupNotify } = desktopEntry;

      console.log("Desktop Entry:",  { Exec, Icon, Terminal, Type, Name, GenericName, StartupNotify });*/
      //     let r = Rect.bind(new Rect(), { x: () => pos.x, y: () => pos.y, width: () => size.width, height: () => size.height });
      const lnkFile = '/home/roman/mnt/lexy/.idesktop/' + path.basename(filename, '.desktop') + '.lnk';

      const svgFile = '/home/roman/mnt/ubuntu/' + desktopEntry.Icon.replace(/\.[a-z]*$/, '') + '.svg';
      const iconFile = '/home/lexy/.logos/' + path.basename(svgFile, '.svg') + '.png';
      console.log(' :', { svgFile, iconFile });
      let svgData = tXml(ReadFile(svgFile));

      let svg = svgData[0] || { attributes: {} };
      const attr = svg && svg.attributes;
      const viewBoxStr = attr && attr.viewBox;
      const viewCoords = (viewBoxStr && viewBoxStr.split(' ')) || [0, 0, svg.attributes.width, svg.attributes.height];
      const [x1, y1, x2, y2] = viewCoords;
      const viewBox = new Rect(svg.attributes && svg.attributes.viewBox ? { x1, y1, x2, y2 } : ['width', 'height'].map(a => svg.attributes[a]));
      iconSize = viewBox.size; //new Size(viewBox.width, viewBox.height);
      iconAspect = iconSize.aspect();
      const scale = iconAspect > 1 ? size.width / iconSize.width : size.height / iconSize.height;
      newSize = iconSize.prod(scale, scale);

      const { width, height } = newSize.prod(5, 5).round();
      newSize = newSize.round();

      Object.assign(svg.attributes, { width, height });
      weakDefine(svg.attributes, {
        viewBox: new BBox(0, 0, iconSize.width, iconSize.height)
      });
      WriteFile(svgFile, toXML(svgData));

      console.log(' :', {
        attr,
        svgFile,
        scale,
        iconSize,
        viewCoords,
        newSize,
        size,
        scale,
        width,
        height,
        iconSize,
        iconAspect
      });

      if(xy.x + size.width >= maxWidth) {
        xy.x = 0;
        xy.y += size.height + spacing;
      }

      const ideskEntry = makeIDeskEntry({
        ...desktopEntry,
        Icon: iconFile
      });
      WriteFile(lnkFile, ideskEntry);
      console.log(`Wrote '${lnkFile}'.`);
      console.log(`ideskEntry: `, ideskEntry);
      console.log(`pos:`, xy);

      xy.x += size.width + spacing;

      count++;
    } else if(sections.FILE_INFO) {
      let file_keys = sections.FILE_INFO.keys();
      let file_sections = Object.keys(sections).filter(name => /FILE/.test(name));

      let files = [];
      for(let key of file_keys) {
        let file = {};
        for(let sect of file_sections) {
          let value = sections[sect].get(key);
          if(value) file[sect] = value;
        }
        files.push(file);
      }

      for(let sect of file_sections) {
        sections[sect].clear();
      }

      let filenames = files.map(f => f.FILE_INFO);

      files = files.filter(file => !/.*(buffer|comparator|lcd|format|ds18b20|hd44).*/.test(file.FILE_INFO));

      //console.log('data:', file_sections);
      //console.log('files:', filenames);
      let i = 0;
      for(let file of files) {
        for(let field in file) sections[field].set(`file_${(i + '').padStart(3, '0')}`, file[field]);

        i++;
      }
    }

    let out = '';
    for(let section in sections) {
      out += `[${section}]\r\n`;
      for(let [key, value] of entries(sections[section])) {
        out += `${key}=${value}\r\n`;
      }
    }

    //console.log('out:', out);

    WriteFile(filename, out);
  }

  function makeIDeskEntry({ Exec, Icon, Terminal, Type, Name, GenericName, StartupNotify }) {
    return `table Icon
  Caption: ${Name}
  ToolTip.Caption: ${GenericName}
  Icon: ${Icon}
  Width: ${(newSize || size).width}
  Height: ${(newSize || size).height}
  X: ${xy.x}
  Y: ${xy.y + (size.height - newSize.height)}
  Command: ${Exec}
end`;
  }
}

main(...scriptArgs.slice(1));