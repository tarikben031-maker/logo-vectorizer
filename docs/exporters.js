/* Vector exporters: SVG, PDF, EPS, DXF, PNG from the vectorizer's layered model.
 *
 * model = { width, height, background: '#rrggbb' | null, layers: [{ color: '#rrggbb', d: 'M…L…C…Z' }] }
 * style = 'filled' (solid shapes) | 'outline' (thin strokes, for cutting / engraving)
 * group = true -> one named group per colour (easy editing)
 */
(function (root) {
  'use strict';

  const PT = 0.75;                 // 1 px (96 dpi) = 0.75 pt

  // ---------- path parsing: M, L, C, Z (absolute) -> [{ start, segs: [['L', x, y] | ['C', x1, y1, x2, y2, x, y]] }]
  function parsePath(d) {
    const subs = [];
    let cur = null;
    const re = /([MLCZ])([^MLCZ]*)/g;
    let m;
    while ((m = re.exec(d))) {
      const n = (m[2].match(/-?\d*\.?\d+(?:e-?\d+)?/g) || []).map(Number);
      if (m[1] === 'M') { cur = { start: [n[0], n[1]], segs: [] }; subs.push(cur); }
      else if (m[1] === 'L') cur.segs.push(['L', n[0], n[1]]);
      else if (m[1] === 'C') cur.segs.push(['C', n[0], n[1], n[2], n[3], n[4], n[5]]);
    }
    return subs;
  }

  const hexRGB = (h) => [1, 3, 5].map((i) => parseInt(h.slice(i, i + 2), 16) / 255);
  const f = (x) => { const s = (+x.toFixed(3)).toString(); return s === '-0' ? '0' : s; };

  // ---------- SVG
  function toSVG(model, style, group) {
    const { width: W, height: H } = model;
    let body = '';
    if (model.background && style === 'filled') body += `<rect width="${W}" height="${H}" fill="${model.background}"/>`;
    model.layers.forEach((l, i) => {
      const paint = style === 'outline'
        ? `fill="none" stroke="#000000" stroke-width="1" vector-effect="non-scaling-stroke"`
        : `fill="${l.color}" fill-rule="evenodd"`;
      const path = `<path ${paint} d="${l.d}"/>`;
      body += group ? `<g id="couleur-${i + 1}" data-color="${l.color}">${path}</g>` : path;
    });
    return `<?xml version="1.0" encoding="UTF-8"?>\n<svg xmlns="http://www.w3.org/2000/svg" version="1.1" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}">${body}</svg>`;
  }

  // ---------- PDF (vector, one page, 1 px = 0.75 pt)
  function pdfOps(model, style) {
    const { width: W, height: H } = model;
    const out = [`${PT} 0 0 ${-PT} 0 ${f(H * PT)} cm`];
    if (model.background && style === 'filled') {
      const [r, g, b] = hexRGB(model.background);
      out.push(`${f(r)} ${f(g)} ${f(b)} rg 0 0 ${W} ${H} re f`);
    }
    if (style === 'outline') out.push(`0 0 0 RG ${f(1 / PT)} w 1 j`);
    for (const l of model.layers) {
      if (style === 'filled') { const [r, g, b] = hexRGB(l.color); out.push(`${f(r)} ${f(g)} ${f(b)} rg`); }
      const ops = [];
      for (const s of parsePath(l.d)) {
        ops.push(`${f(s.start[0])} ${f(s.start[1])} m`);
        for (const g of s.segs) ops.push(g[0] === 'L' ? `${f(g[1])} ${f(g[2])} l` : `${f(g[1])} ${f(g[2])} ${f(g[3])} ${f(g[4])} ${f(g[5])} ${f(g[6])} c`);
        ops.push('h');
      }
      ops.push(style === 'filled' ? 'f*' : 'S');
      out.push(ops.join('\n'));
    }
    return out.join('\n');
  }

  async function deflate(bytes) {
    if (typeof CompressionStream === 'undefined') return null;
    try {
      const cs = new CompressionStream('deflate');
      const buf = await new Response(new Blob([bytes]).stream().pipeThrough(cs)).arrayBuffer();
      return new Uint8Array(buf);
    } catch (e) { return null; }
  }

  async function toPDF(model, style) {
    const enc = new TextEncoder();
    const W = model.width * PT, H = model.height * PT;
    const raw = enc.encode(pdfOps(model, style));
    const z = await deflate(raw);
    const stream = z || raw;
    const parts = [];
    const offsets = [];
    let len = 0;
    const push = (x) => { const b = typeof x === 'string' ? enc.encode(x) : x; parts.push(b); len += b.length; };
    push('%PDF-1.4\n%\xE2\xE3\xCF\xD3\n');
    const obj = (n, body) => { offsets[n] = len; push(`${n} 0 obj\n${body}\nendobj\n`); };
    obj(1, '<< /Type /Catalog /Pages 2 0 R >>');
    obj(2, '<< /Type /Pages /Kids [3 0 R] /Count 1 >>');
    obj(3, `<< /Type /Page /Parent 2 0 R /MediaBox [0 0 ${f(W)} ${f(H)}] /Contents 4 0 R /Resources << >> >>`);
    offsets[4] = len;
    push(`4 0 obj\n<< /Length ${stream.length}${z ? ' /Filter /FlateDecode' : ''} >>\nstream\n`);
    push(stream);
    push('\nendstream\nendobj\n');
    obj(5, `<< /Producer (Vectoriseur de logos) /Creator (Vectoriseur de logos) >>`);
    const xref = len;
    let x = `xref\n0 6\n0000000000 65535 f \n`;
    for (let i = 1; i <= 5; i++) x += String(offsets[i]).padStart(10, '0') + ' 00000 n \n';
    push(x + `trailer\n<< /Size 6 /Root 1 0 R /Info 5 0 R >>\nstartxref\n${xref}\n%%EOF\n`);
    return new Blob(parts, { type: 'application/pdf' });
  }

  // ---------- EPS (PostScript, 1 px = 0.75 pt)
  function toEPS(model, style) {
    const { width: W, height: H } = model;
    const L = [
      '%!PS-Adobe-3.0 EPSF-3.0',
      `%%BoundingBox: 0 0 ${Math.ceil(W * PT)} ${Math.ceil(H * PT)}`,
      `%%HiResBoundingBox: 0 0 ${f(W * PT)} ${f(H * PT)}`,
      '%%Creator: Vectoriseur de logos',
      '%%LanguageLevel: 2',
      '%%EndComments',
      'gsave',
      `0 ${f(H * PT)} translate ${PT} ${-PT} scale`,
    ];
    if (model.background && style === 'filled') {
      const [r, g, b] = hexRGB(model.background);
      L.push(`${f(r)} ${f(g)} ${f(b)} setrgbcolor newpath 0 0 moveto ${W} 0 lineto ${W} ${H} lineto 0 ${H} lineto closepath fill`);
    }
    if (style === 'outline') L.push(`0 0 0 setrgbcolor ${f(1 / PT)} setlinewidth 1 setlinejoin`);
    for (const l of model.layers) {
      if (style === 'filled') { const [r, g, b] = hexRGB(l.color); L.push(`${f(r)} ${f(g)} ${f(b)} setrgbcolor`); }
      L.push('newpath');
      for (const s of parsePath(l.d)) {
        L.push(`${f(s.start[0])} ${f(s.start[1])} moveto`);
        for (const g of s.segs) L.push(g[0] === 'L' ? `${f(g[1])} ${f(g[2])} lineto` : `${f(g[1])} ${f(g[2])} ${f(g[3])} ${f(g[4])} ${f(g[5])} ${f(g[6])} curveto`);
        L.push('closepath');
      }
      L.push(style === 'filled' ? 'eofill' : 'stroke');
    }
    L.push('grestore', 'showpage', '%%EOF');
    return new Blob([L.join('\n') + '\n'], { type: 'application/postscript' });
  }

  // ---------- DXF (R12 ASCII, closed polylines, one layer per colour; curves flattened to 0.05 px)
  const ACI = [[1, [255, 0, 0]], [2, [255, 255, 0]], [3, [0, 255, 0]], [4, [0, 255, 255]], [5, [0, 0, 255]],
    [6, [255, 0, 255]], [7, [0, 0, 0]], [8, [128, 128, 128]], [9, [192, 192, 192]], [30, [255, 127, 0]],
    [40, [255, 191, 0]], [150, [0, 127, 255]], [200, [127, 0, 255]], [240, [255, 0, 63]], [34, [127, 63, 0]]];
  function nearestACI(hex) {
    const c = hexRGB(hex).map((v) => v * 255);
    let best = 7, bd = Infinity;
    for (const [i, rgb] of ACI) { const d = (c[0] - rgb[0]) ** 2 + (c[1] - rgb[1]) ** 2 + (c[2] - rgb[2]) ** 2; if (d < bd) { bd = d; best = i; } }
    return best;
  }
  function flattenCubic(p0, p1, p2, p3, tol, out, depth = 0) {
    const dx = p3[0] - p0[0], dy = p3[1] - p0[1], n = Math.hypot(dx, dy) || 1e-9;
    const d1 = Math.abs((p1[0] - p3[0]) * dy - (p1[1] - p3[1]) * dx) / n;
    const d2 = Math.abs((p2[0] - p3[0]) * dy - (p2[1] - p3[1]) * dx) / n;
    if (depth > 12 || d1 + d2 <= tol) { out.push(p3); return; }
    const m = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    const p01 = m(p0, p1), p12 = m(p1, p2), p23 = m(p2, p3), p012 = m(p01, p12), p123 = m(p12, p23), mid = m(p012, p123);
    flattenCubic(p0, p01, p012, mid, tol, out, depth + 1);
    flattenCubic(mid, p123, p23, p3, tol, out, depth + 1);
  }
  function toDXF(model) {
    const H = model.height;
    const layers = model.layers.map((l, i) => ({ name: `COULEUR_${i + 1}_${l.color.slice(1).toUpperCase()}`, aci: nearestACI(l.color), d: l.d }));
    const o = [];
    const g = (code, v) => o.push(String(code), String(v));
    g(0, 'SECTION'); g(2, 'HEADER');
    g(9, '$ACADVER'); g(1, 'AC1009');
    g(9, '$EXTMIN'); g(10, 0); g(20, 0);
    g(9, '$EXTMAX'); g(10, model.width); g(20, H);
    g(0, 'ENDSEC');
    g(0, 'SECTION'); g(2, 'TABLES');
    g(0, 'TABLE'); g(2, 'LAYER'); g(70, layers.length);
    for (const l of layers) { g(0, 'LAYER'); g(2, l.name); g(70, 0); g(62, l.aci); g(6, 'CONTINUOUS'); }
    g(0, 'ENDTAB'); g(0, 'ENDSEC');
    g(0, 'SECTION'); g(2, 'ENTITIES');
    for (const l of layers) {
      for (const s of parsePath(l.d)) {
        const pts = [s.start];
        let cur = s.start;
        for (const seg of s.segs) {
          if (seg[0] === 'L') { cur = [seg[1], seg[2]]; pts.push(cur); }
          else { const p3 = [seg[5], seg[6]]; flattenCubic(cur, [seg[1], seg[2]], [seg[3], seg[4]], p3, 0.05, pts); cur = p3; }
        }
        if (pts.length > 1 && Math.hypot(pts[0][0] - pts[pts.length - 1][0], pts[0][1] - pts[pts.length - 1][1]) < 1e-6) pts.pop();
        if (pts.length < 2) continue;
        g(0, 'POLYLINE'); g(8, l.name); g(66, 1); g(10, 0); g(20, 0); g(30, 0); g(70, 1);
        for (const p of pts) { g(0, 'VERTEX'); g(8, l.name); g(10, f(p[0])); g(20, f(H - p[1])); g(30, 0); }
        g(0, 'SEQEND'); g(8, l.name);
      }
    }
    g(0, 'ENDSEC'); g(0, 'EOF');
    return new Blob([o.join('\r\n') + '\r\n'], { type: 'application/dxf' });
  }

  // ---------- PNG (rendered from the SVG in the browser)
  async function toPNG(svgString, width, height, scale) {
    const img = new Image();
    const url = URL.createObjectURL(new Blob([svgString], { type: 'image/svg+xml' }));
    img.src = url;
    await img.decode();
    const c = document.createElement('canvas');
    c.width = Math.round(width * scale); c.height = Math.round(height * scale);
    c.getContext('2d').drawImage(img, 0, 0, c.width, c.height);
    URL.revokeObjectURL(url);
    return new Promise((res) => c.toBlob(res, 'image/png'));
  }

  const api = { parsePath, toSVG, toPDF, toEPS, toDXF, toPNG, pdfOps };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  root.LogoExport = api;
})(typeof self !== 'undefined' ? self : this);
