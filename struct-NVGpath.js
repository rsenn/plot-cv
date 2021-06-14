class NVGpath extends ArrayBuffer {
  constructor(obj = {}) {
    super(56);
    Object.assign(this, obj);
  }
  /* prettier-ignore */ get [Symbol.toStringTag]() { return `[struct NVGpath @ ${this} ]`; }

  /* 0: int first@4 */
  /* prettier-ignore */ set first(v) { new Int32Array(this, 0)[0] = v; }
  /* prettier-ignore */ get first() { return new Int32Array(this, 0)[0]; }

  /* 4: int count@4 */
  /* prettier-ignore */ set count(v) { new Int32Array(this, 4)[0] = v; }
  /* prettier-ignore */ get count() { return new Int32Array(this, 4)[0]; }

  /* 8: unsigned char closed@1 */
  /* prettier-ignore */ set closed(v) { new Uint8Array(this, 8)[0] = v; }
  /* prettier-ignore */ get closed() { return new Uint8Array(this, 8)[0]; }

  /* 12: int nbevel@4 */
  /* prettier-ignore */ set nbevel(v) { new Int32Array(this, 12)[0] = v; }
  /* prettier-ignore */ get nbevel() { return new Int32Array(this, 12)[0]; }

  /* 16: NVGvertex * fill@8 */
  /* prettier-ignore */ set fill(v) { new BigUint64Array(this, 16)[0] = BigInt(v); }
  /* prettier-ignore */ get fill() { return new BigUint64Array(this, 16)[0]; }

  /* 24: int nfill@4 */
  /* prettier-ignore */ set nfill(v) { new Int32Array(this, 24)[0] = v; }
  /* prettier-ignore */ get nfill() { return new Int32Array(this, 24)[0]; }

  /* 32: NVGvertex * stroke@8 */
  /* prettier-ignore */ set stroke(v) { new BigUint64Array(this, 32)[0] = BigInt(v); }
  /* prettier-ignore */ get stroke() { return new BigUint64Array(this, 32)[0]; }

  /* 40: int nstroke@4 */
  /* prettier-ignore */ set nstroke(v) { new Int32Array(this, 40)[0] = v; }
  /* prettier-ignore */ get nstroke() { return new Int32Array(this, 40)[0]; }

  /* 44: int winding@4 */
  /* prettier-ignore */ set winding(v) { new Int32Array(this, 44)[0] = v; }
  /* prettier-ignore */ get winding() { return new Int32Array(this, 44)[0]; }

  /* 48: int convex@4 */
  /* prettier-ignore */ set convex(v) { new Int32Array(this, 48)[0] = v; }
  /* prettier-ignore */ get convex() { return new Int32Array(this, 48)[0]; }

  toString() {
    const { first, count, closed, nbevel, fill, nfill, stroke, nstroke, winding, convex } = this;
    return `struct NVGpath {\n\t.first = ${first},\n\t.count = ${count},\n\t.closed = ${closed},\n\t.nbevel = ${nbevel},\n\t.fill = 0x${fill.toString(
      16
    )},\n\t.nfill = ${nfill},\n\t.stroke = 0x${stroke.toString(
      16
    )},\n\t.nstroke = ${nstroke},\n\t.winding = ${winding},\n\t.convex = ${convex}\n}`;
  }
}
