import { WriteFile } from './io-helpers.js';
import { Line, LineList, MakePolygon, Point } from './lib/geom.js';
import SvgPath from './lib/svg/path.js';
import { Console } from 'console';
import * as xml from 'xml';
function PathFromPoints(points, pos = new Point(0, 0)) {
  let path = new SvgPath();

  let i = 0;
  for(let point of points) path[i++ ? 'line' : 'to'](...point.round(0.001));

  path.close();
  return `<path transform="translate(${pos.x} ${pos.y})" fill="none" stroke="black" stroke-width="0.025" d="${path}" />`;
}

function main(...args) {
  globalThis.console = new Console({ inspectOptions: { depth: 10, compact: 1 } });

  const f = 1.082392200292395;
  const center = new Point(5.08, 2.54);
  const octagon = MakePolygon(8, 1.27 * f, 0.5).map(p => p.add(center));
  const octagon2 = MakePolygon(8, 1.27 * f, 0).map(p => p.add(center));
  console.log('octagon', octagon);
  console.log('octagon.boundingRect()', octagon.boundingRect());
  //console.log('xml.write', xml.write([{ tagName: 'g', attributes: { stroke: '#ccc', 'stroke-width': 0.025 } }]));

  const lines = new LineList(octagon2.map(p => new Line(center, p)));

  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="10.16" height="10.16">
    ${xml.write([
      lines.toSVG(
        (tagName, attributes, children) => (children ? { tagName, attributes, children } : { tagName, attributes }),
        () => ({
          tagName: 'g',
          attributes: { stroke: '#ccc', fill: 'none', 'stroke-width': 0.025 }
        })
      )
    ])}
  ${PathFromPoints(octagon, new Point(0, 0))}
  ${PathFromPoints(MakePolygon(10, 1.27), new Point(5.08, 7.62))}

</svg>`;

  WriteFile('output.svg', svg);
}

main(...scriptArgs.slice(1));