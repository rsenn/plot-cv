import { declare, isNumeric, properties, nonenumerable } from 'util';
import { Entities, DOMException, Prototypes, Factory, Parser, Interface, Node, NODE_TYPES, NodeList, HTMLCollection, NamedNodeMap, Element, Document, Attr, CharacterData, Text, Comment, TokenList, CSSStyleDeclaration, Serializer, MapItems, FindItemIndex, FindItem, ListAdapter, MutationRecord, MutationObserver, URLSearchParams, URL, } from 'dom';
 
export const EagleClasses = {
  Document: class EagleDocument extends Document {
    constructor(obj, factory) {
      super(obj, null, factory);
    }

    /* prettier-ignore */ get eagle() { return this.querySelector('eagle'); }
    /* prettier-ignore */ get drawing() { return this.eagle.drawing; }
    /* prettier-ignore */ get board() { return this.eagle.drawing.board; }
    /* prettier-ignore */ get schematic() { return this.eagle.drawing.schematic; }
    /* prettier-ignore */ get library() { return this.eagle.drawing.library; }
    /* prettier-ignore */ get layers() { return this.eagle.drawing.layers; }
    /* prettier-ignore */ get type() { return this.eagle.drawing.type; }
  },
  Element: class EagleElement extends Element {
    constructor(node, parent) {
      super(node, parent);

      for(let e of Element.hier(this)
        .slice(0, -1)
        .filter(e => e.hasAttribute && (e.hasAttribute('name') || e.tagName == 'sheet'))) {
        const { tagName } = e;

        if(!Reflect.has(this, tagName))
          Reflect.defineProperty(this, tagName, {
            get: () => Element.hier(e).find(e => e.tagName == tagName),
            configurable: true,
          });
      }
    }

    static elements = Prototypes({
      /*
       * Common Eagle Elements
       */
      eagle: class EagleElement extends this {
        /* prettier-ignore */ get drawing() { return this.children[ Node.raw(this).children.findIndex(e => e.tagName == 'drawing')]; }
      },
      drawing: class DrawingElement extends this {
        /* prettier-ignore */ get settings() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'settings')]; }
        /* prettier-ignore */ get grid() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'grid')]; }
        /* prettier-ignore */ get layers() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'layers')]; }
        /* prettier-ignore */ get schematic() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'schematic')]; }
        /* prettier-ignore */ get board() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'board')]; }
        /* prettier-ignore */ get library() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'board')]; }
        /* prettier-ignore */ get type() { return [...this.children].find(e => ['schematic', 'board', 'library'].includes(e.tagName))?.tagName; }
      },
      settings: class SettingsElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return Collection(this);
        }
      },
      setting: class SettingElement extends this {
        /* prettier-ignore */ get name() { return this.attributes[0].name; }
        /* prettier-ignore */ get value() { return this.attributes[0].value; }
      },
      grid: class GridElement extends this {},
      layers: class LayersElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return NamedMap(
            this,
            n => this.querySelector(`layer[name=${n}]`),
            () => [...this.children].reduce((acc, e) => ((acc[+e.getAttribute('number')] = e.getAttribute('name')), acc), []),
          );
        }
      },
      layer: class LayerElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get number() { return +this.getAttribute('number'); }
        /* prettier-ignore */ get color() { return +this.getAttribute('color'); }
        /* prettier-ignore */ get fill() { return +this.getAttribute('fill'); }
        /* prettier-ignore */ get visible() { return this.getAttribute('visible') == 'yes'; }
        /* prettier-ignore */ get active() { return this.getAttribute('active') == 'yes'; }
      },
      description: class DescriptionElement extends this {},
      plain: class PlainElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return Collection(this);
        }

        wires = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'wire');
        texts = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'text');
        dimensions = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'dimension');
      },
      wire: class WireElement extends this {
        /* prettier-ignore */ get x1() { return +this.getAttribute('x1'); }
        /* prettier-ignore */ get y1() { return +this.getAttribute('y1'); }
        /* prettier-ignore */ get x2() { return +this.getAttribute('x2'); }
        /* prettier-ignore */ get y2() { return +this.getAttribute('y2'); }
        /* prettier-ignore */ get layer() { return this.ownerDocument.layers[this.getAttribute('layer')]; }
        /* prettier-ignore */ get width() { return +this.getAttribute('width'); }
      },
      libraries: class LibrariesElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      library: class LibraryElement extends this {
        /* prettier-ignore */ get description() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'description')]; }
        /* prettier-ignore */ get packages() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'packages')]; }
        /* prettier-ignore */ get symbols() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'symbols')]; }
        /* prettier-ignore */ get devicesets() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'devicesets')]; }
      },
      packages: class PackagesElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      package: class PackageElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return Collection(this);
        }

        /* prettier-ignore */ get description() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'description')]; }
        pads = new NamedNodeMap(
          {
            get: name => [...this.children].find(e => e.tagName == 'pad' && e.getAttribute('name') == name),
            keys: () =>
              [...this.children]
                .filter(e => e.tagName == 'pad')
                .map(e => e.getAttribute('name'))
                .reduce((acc, e) => ((acc[e] = e), acc), []),
          },
          this,
        );
        wires = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'wire');
        texts = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'text');
      },
      pad: class PadElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
        /* prettier-ignore */ get drill() { return +this.getAttribute('drill'); }
        /* prettier-ignore */ get shape() { return this.getAttribute('shape'); }
      },
      text: class TextElement extends this {
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
        /* prettier-ignore */ get size() { return +this.getAttribute('size'); }
        /* prettier-ignore */ get layer() { return this.ownerDocument.layers[this.getAttribute('layer')]; }
        /* prettier-ignore */ get ratio() { return +this.getAttribute('ratio'); }
      },
      rectangle: class RectangleElement extends this {
        /* prettier-ignore */ get x1() { return +this.getAttribute('x1'); }
        /* prettier-ignore */ get y1() { return +this.getAttribute('y1'); }
        /* prettier-ignore */ get x2() { return +this.getAttribute('x2'); }
        /* prettier-ignore */ get y2() { return +this.getAttribute('y2'); }
        /* prettier-ignore */ get layer() { return this.ownerDocument.layers[this.getAttribute('layer')]; }
        /* prettier-ignore */ get rot() { return this.getAttribute('rot'); }
      },
      circle: class CircleElement extends this {
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
        /* prettier-ignore */ get radius() { return +this.getAttribute('radius'); }
        /* prettier-ignore */ get width() { return +this.getAttribute('width'); }
        /* prettier-ignore */ get layer() { return this.ownerDocument.layers[this.getAttribute('layer')]; }
      },
      attributes: class AttributesElement extends this {},
      variantdefs: class VariantdefsElement extends this {},
      classes: class ClassesElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'number');
        }
      },
      class: class ClassElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return Collection(this);
        }
        /* prettier-ignore */ get number() { return +this.getAttribute('number'); }
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get width() { return +this.getAttribute('width'); }
        /* prettier-ignore */ get drill() { return +this.getAttribute('drill'); }

        clearances = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'clearance');
      },

      /*
       * Eagle Schematic Elements
       */
      schematic: class SchematicElement extends this {
        /* prettier-ignore */ get description() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'description')]; }
        /* prettier-ignore */ get libraries() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'libraries')]; }
        /* prettier-ignore */ get attributes() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'attributes')]; }
        /* prettier-ignore */ get variantdefs() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'variantdefs')]; }
        /* prettier-ignore */ get classes() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'classes')]; }
        /* prettier-ignore */ get parts() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'parts')]; }
        /* prettier-ignore */ get sheets() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'sheets')]; }
        /* prettier-ignore */ get modules() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'modules')]; }
      },
      symbols: class SymbolsElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      symbol: class SymbolElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return Collection(this);
        }
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }

        pins = new NamedNodeMap(
          {
            get: name => [...this.children].find(e => e.tagName == 'pin' && e.getAttribute('name') == name),
            keys: () => [...this.children].filter(e => e.tagName == 'pin').map(e => e.getAttribute('name')),
          },
          this,
        );
        wires = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'wire');
        texts = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'text');
      },
      pin: class PinElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
        /* prettier-ignore */ get visible() { return this.getAttribute('visible'); }
      },
      devicesets: class DevicesetsElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      deviceset: class DevicesetElement extends this {
        /* prettier-ignore */ get description() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'description')]; }
        /* prettier-ignore */ get gates() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'gates')]; }
        /* prettier-ignore */ get devices() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'devices')]; }
      },
      gates: class GatesElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name', () => []);
        }
      },
      gate: class GateElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get symbol() { return this.library.symbols[this.getAttribute('symbol')]; }
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
      },
      devices: class DevicesElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      device: class DeviceElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get package() { return this.library.packages[this.getAttribute('package')]; }
        /* prettier-ignore */ get connects() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'connects')]; }
        /* prettier-ignore */ get technologies() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'technologies')]; }
      },
      connects: class ConnectsElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return Collection(this);
        }
      },
      connect: class ConnectElement extends this {
        /* prettier-ignore */ get gate() { return this.deviceset.gates[this.getAttribute('gate')]; }
        /* prettier-ignore */ get pin() { return this.gate.symbol.pins[this.getAttribute('pin')]; }
        /* prettier-ignore */ get pad() { return this.device.package.pads[this.getAttribute('pad')]; }
      },
      technologies: class TechnologiesElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      technology: class TechnologyElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
      },
      parts: class PartsElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      part: class PartElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get library() { return this.ownerDocument.schematic.libraries[this.getAttribute('library')]; }
        /* prettier-ignore */ get deviceset() { return this.library.devicesets[this.getAttribute('deviceset')]; }
        /* prettier-ignore */ get device() { return this.deviceset.devices[this.getAttribute('device')]; }
        /* prettier-ignore */ get value() { return this.getAttribute('value'); }
      },
      sheets: class SheetsElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return Collection(this);
        }
      },
      sheet: class SheetElement extends this {
        /* prettier-ignore */ get plain() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'plain')]; }
        /* prettier-ignore */ get instances() { return this.children[ Node.raw(this).children.findIndex(e => e.tagName == 'instances')]; }
        /* prettier-ignore */ get busses() { return this.children[ Node.raw(this).children.findIndex(e => e.tagName == 'busses')]; }
        /* prettier-ignore */ get nets() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'nets')]; }
        /* prettier-ignore */ get moduleinsts() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'moduleinsts')]; }
      },
      instances: class InstancesElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return Collection(this);
        }
      },
      instance: class InstanceElement extends this {
        /* prettier-ignore */ get part() { return (this.module ?? this.ownerDocument.schematic).parts[this.getAttribute('part')]; }
        /* prettier-ignore */ get gate() { return this.part.deviceset.gates[this.getAttribute('gate')]; }
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
        /* prettier-ignore */ get smashed() { return this.getAttribute('smashed') == 'yes'; }
      },
      busses: class BussesElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      bus: class BusElement extends this {
        segments = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'segment');
      },
      nets: class NetsElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      net: class NetElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get class() { return this.ownerDocument.schematic.classes[this.getAttribute('class')]; }

        segments = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'segment');
      },
      segment: class SegmentElement extends this {
        pinrefs = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'pinref');
        wires = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'wire');
        labels = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'label');
        junctions = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'junction');
      },
      label: class LabelElement extends this {
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
        /* prettier-ignore */ get size() { return +this.getAttribute('size'); }
        /* prettier-ignore */ get layer() { return this.ownerDocument.layers[this.getAttribute('layer')]; }
      },
      pinref: class PinrefElement extends this {
        /* prettier-ignore */ get part() { return this.ownerDocument.schematic.parts[this.getAttribute('part')]; }
        /* prettier-ignore */ get gate() { return this.part.deviceset.gates[this.getAttribute('gate')]; }
        /* prettier-ignore */ get pin() { return this.gate.symbol.pins[this.getAttribute('pin')]; }
      },
      junction: class JunctionElement extends this {
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
      },

      /* board */
      board: class BoardElement extends this {
        /* prettier-ignore */ get description() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'description')]; }
        /* prettier-ignore */ get plain() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'plain')]; }
        /* prettier-ignore */ get libraries() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'libraries')]; }
        /* prettier-ignore */ get attributes() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'attributes')]; }
        /* prettier-ignore */ get variantdefs() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'variantdefs')]; }
        /* prettier-ignore */ get classes() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'classes')]; }
        /* prettier-ignore */ get designrules() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'designrules')]; }
        /* prettier-ignore */ get autorouter() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'autorouter')]; }
        /* prettier-ignore */ get elements() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'elements')]; }
        /* prettier-ignore */ get signals() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'signals')]; }
      },
      designrules: class DesignrulesElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return Collection(this);
        }
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        params = new NamedNodeMap(
          {
            get: name => [...this.children].find(e => e.tagName == 'param' && e.getAttribute('name') == name),
            keys: () => [...this.children].filter(e => e.tagName == 'param').map(e => e.getAttribute('name')),
          },
          this,
        );
      },
      param: class ParamElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get value() { return this.getAttribute('value'); }
      },
      autorouter: class AutorouterElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return Collection(this);
        }
        passes = new NamedNodeMap(
          {
            get: name => [...this.children].find(e => e.tagName == 'pass' && e.getAttribute('name') == name),
            keys: () => [...this.children].filter(e => e.tagName == 'pass').map(e => e.getAttribute('name')),
          },
          this,
        );
      },
      pass: class PassElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return Collection(this);
        }

        /* prettier-ignore */ get name() { return this.getAttribute('name'); }

        params = new NamedNodeMap(
          {
            get: name => [...this.children].find(e => e.tagName == 'param' && e.getAttribute('name') == name),
            keys: () => [...this.children].filter(e => e.tagName == 'param').map(e => e.getAttribute('name')),
          },
          this,
        );
      },
      elements: class ElementsElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      element: class ElementElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get value() { return this.getAttribute('value'); }
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
        /* prettier-ignore */ get library() { return this.ownerDocument.board.libraries[this.getAttribute('library')]; }
        /* prettier-ignore */ get package() { return this.library.packages[this.getAttribute('package')]; }
      },
      signals: class SignalsElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      signal: class SignalElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }

        contactrefs = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'contactref');
        wires = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'wire');
        vias = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'via');
      },
      contactref: class ContactrefElement extends this {
        /* prettier-ignore */ get element() { return this.ownerDocument.board.elements[this.getAttribute('element')]; }
        /* prettier-ignore */ get pad() { return this.element.package.pads[this.getAttribute('pad')]; }
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
      },
      via: class ViaElement extends this {
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
        /* prettier-ignore */ get extent() { return this.getAttribute('extent'); }
        /* prettier-ignore */ get drill() { return +this.getAttribute('drill'); }
        /* prettier-ignore */ get shape() { return this.getAttribute('shape'); }
      },
      approved: class ApprovedElement extends this {
        /* prettier-ignore */ get hash() { return this.getAttribute('hash'); }
      },
      attribute: class AttributeElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
        /* prettier-ignore */ get size() { return +this.getAttribute('size'); }
        /* prettier-ignore */ get layer() { return this.ownerDocument.layers[this.getAttribute('layer')]; }
      },
      clearance: class ClearanceElement extends this {
        /* prettier-ignore */ get class() { return this.classes[this.getAttribute('class')]; }
        /* prettier-ignore */ get value() { return this.getAttribute('value'); }
      },
      compatibility: class CompatibilityElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return Collection(this);
        }
        notes = new HTMLCollection(Node.raw(this).children, this, e => e.tagName == 'note');
      },

      note: class NoteElement extends this {},

      errors: class ErrorsElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return Collection(this);
        }
      },
      module: class ModuleElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get prefix() { return this.getAttribute('prefix'); }
        /* prettier-ignore */ get dx() { return +this.getAttribute('dx'); }
        /* prettier-ignore */ get dy() { return +this.getAttribute('dy'); }

        /* prettier-ignore */ get ports() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'ports')]; }
        /* prettier-ignore */ get variantdefs() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'variantdefs')]; }
        /* prettier-ignore */ get parts() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'parts')]; }
        /* prettier-ignore */ get sheets() { return this.children[Node.raw(this).children.findIndex(e => e.tagName == 'sheets')]; }
      },
      moduleinst: class ModuleinstElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get module() { return this.ownerDocument.schematic.modules[this.getAttribute('module')]; }
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
      },
      moduleinsts: class ModuleinstsElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      modules: class ModulesElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },

      dimension: class DimensionElement extends this {
        /* prettier-ignore */ get x1() { return +this.getAttribute('x1'); }
        /* prettier-ignore */ get y1() { return +this.getAttribute('y1'); }
        /* prettier-ignore */ get x2() { return +this.getAttribute('x2'); }
        /* prettier-ignore */ get y2() { return +this.getAttribute('y2'); }
        /* prettier-ignore */ get x3() { return +this.getAttribute('x3'); }
        /* prettier-ignore */ get y3() { return +this.getAttribute('y3'); }
        /* prettier-ignore */ get textsize() { return +this.getAttribute('textsize'); }
        /* prettier-ignore */ get visible() { return this.getAttribute('visible') == 'yes'; }
        /* prettier-ignore */ get layer() { return this.ownerDocument.layers[this.getAttribute('layer')]; }
      },
      frame: class FrameElement extends this {
        /* prettier-ignore */ get x1() { return +this.getAttribute('x1'); }
        /* prettier-ignore */ get y1() { return +this.getAttribute('y1'); }
        /* prettier-ignore */ get x2() { return +this.getAttribute('x2'); }
        /* prettier-ignore */ get y2() { return +this.getAttribute('y2'); }
        /* prettier-ignore */ get columns() { return +this.getAttribute('columns'); }
        /* prettier-ignore */ get rows() { return +this.getAttribute('rows'); }
        /* prettier-ignore */ get layer() { return this.ownerDocument.layers[this.getAttribute('layer')]; }
      },
      hole: class HoleElement extends this {
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
        /* prettier-ignore */ get drill() { return +this.getAttribute('drill'); }
      },

      polygon: class PolygonElement extends this {
        constructor(node, parent) {
          super(node, parent);

          return Collection(this);
        }
        /* prettier-ignore */ get layer() { return this.ownerDocument.layers[this.getAttribute('layer')]; }
        /* prettier-ignore */ get width() { return +this.getAttribute('width'); }
      },
      port: class PortElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get side() { return this.getAttribute('side'); }
        /* prettier-ignore */ get coord() { return +this.getAttribute('coord'); }
        /* prettier-ignore */ get direction() { return this.getAttribute('direction'); }
      },
      portref: class PortrefElement extends this {
        /* prettier-ignore */ get moduleinst() { return this.sheet.moduleinsts[this.getAttribute('moduleinst')]; }
        /* prettier-ignore */ get port() { return this.moduleinst.module.ports[this.getAttribute('port')]; }
      },
      ports: class PortsElement extends this {
        constructor(node, parent) {
          super(node, parent);
          return NamedMap(this, 'name');
        }
      },
      smd: class SmdElement extends this {
        /* prettier-ignore */ get name() { return this.getAttribute('name'); }
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
        /* prettier-ignore */ get dx() { return +this.getAttribute('dx'); }
        /* prettier-ignore */ get dy() { return +this.getAttribute('dy'); }
        /* prettier-ignore */ get layer() { return this.ownerDocument.layers[this.getAttribute('layer')]; }
      },
      vertex: class VertexElement extends this {
        /* prettier-ignore */ get x() { return +this.getAttribute('x'); }
        /* prettier-ignore */ get y() { return +this.getAttribute('y'); }
      },
    });
  },
};

export class EagleFactory extends Factory {
  constructor() {
    super(Prototypes(EagleClasses));
  }
}

declare(EagleFactory.prototype, { [Symbol.toStringTag]: 'EagleFactory' });

export class EagleParser extends Parser {
  constructor() {
    super(new EagleFactory());
  }
}

declare(EagleParser.prototype, { [Symbol.toStringTag]: 'EagleParser' });
