/* Background removal for logos, in the browser (no AI, no upload).
 *
 * Works on solid backgrounds and on "fake transparent" backgrounds (a checkerboard baked into
 * a JPG/PNG): the background colours are read from the image border, everything connected to
 * the border that looks like background is removed, plus enclosed checkerboard pockets.
 * Inner parts of the logo that share the background colour (white text inside a red badge…)
 * are kept because they are not connected to the border.
 *
 * BgRemove.removeBackground(rgba, W, H, { tolerance: 'low'|'normal'|'high' })
 *   -> { alpha: Uint8ClampedArray(W*H), checker: bool, bgColors: [[r,g,b]…], bbox }
 * BgRemove.compose(rgba, W, H, alpha, scale, crop) -> HTMLCanvasElement (transparent PNG)
 * BgRemove.flatten(rgba, W, H, alpha, color) -> Uint8ClampedArray (bg replaced by a solid colour)
 */
(function (root) {
  'use strict';

  const TOL = { low: 12, normal: 18, high: 30 };

  // k-means on a small set of colours
  function kmeans(px, k, iters) {
    const n = px.length / 3;
    const C = [];
    for (let j = 0; j < k; j++) { const i = Math.floor(((j + 0.5) / k) * n); C.push([px[i * 3], px[i * 3 + 1], px[i * 3 + 2]]); }
    const lab = new Uint8Array(n);
    for (let it = 0; it < iters; it++) {
      const S = C.map(() => [0, 0, 0, 0]);
      for (let i = 0; i < n; i++) {
        let b = 0, bd = Infinity;
        for (let j = 0; j < k; j++) { const d = (px[i * 3] - C[j][0]) ** 2 + (px[i * 3 + 1] - C[j][1]) ** 2 + (px[i * 3 + 2] - C[j][2]) ** 2; if (d < bd) { bd = d; b = j; } }
        lab[i] = b; S[b][0] += px[i * 3]; S[b][1] += px[i * 3 + 1]; S[b][2] += px[i * 3 + 2]; S[b][3]++;
      }
      for (let j = 0; j < k; j++) if (S[j][3]) C[j] = [S[j][0] / S[j][3], S[j][1] / S[j][3], S[j][2] / S[j][3]];
    }
    const cnt = new Array(k).fill(0); for (let i = 0; i < n; i++) cnt[lab[i]]++;
    return C.map((c, j) => ({ c, share: cnt[j] / n }));
  }

  // background colours from a 2-px ring around the image
  function borderColors(rgba, W, H) {
    const px = [];
    const push = (x, y) => { const o = (y * W + x) * 4; if (rgba[o + 3] > 128) px.push(rgba[o], rgba[o + 1], rgba[o + 2]); };
    for (let r = 0; r < 2; r++) {
      for (let x = 0; x < W; x++) { push(x, r); push(x, H - 1 - r); }
      for (let y = 0; y < H; y++) { push(r, y); push(W - 1 - r, y); }
    }
    if (px.length < 9) return [];
    let cl = kmeans(px, 4, 8).filter((c) => c.share > 0.06);
    // merge near-duplicates
    const out = [];
    for (const c of cl.sort((a, b) => b.share - a.share)) {
      const m = out.find((o) => Math.hypot(o.c[0] - c.c[0], o.c[1] - c.c[1], o.c[2] - c.c[2]) < 12);
      if (m) m.share += c.share; else out.push({ c: c.c.slice(), share: c.share });
    }
    return out;
  }

  function removeBackground(rgba, W, H, opts) {
    const N = W * H;
    const tol = TOL[(opts && opts.tolerance) || 'normal'] || TOL.normal;
    const alpha = new Uint8ClampedArray(N);
    // already transparent image: keep its alpha
    let transp = 0; for (let i = 0; i < N; i++) if (rgba[i * 4 + 3] < 20) transp++;
    const cols = borderColors(rgba, W, H);
    const light = (c) => Math.min(...c) > 170 && Math.max(...c) - Math.min(...c) < 24;
    const checker = cols.length >= 2 && light(cols[0].c) && light(cols[1].c) && cols[1].share > 0.15 &&
      Math.abs(cols[0].c[0] - cols[1].c[0]) > 8;
    const lightMin = checker ? Math.min(cols[0].c[0], cols[1].c[0], cols[0].c[1], cols[1].c[1]) - tol * 0.6 : 0;

    // 1) background-like pixels
    const like = new Uint8Array(N);
    const near = new Uint8Array(N);          // index+1 of nearest border colour (for checker pockets)
    for (let i = 0; i < N; i++) {
      const o = i * 4;
      if (rgba[o + 3] < 20) { like[i] = 1; continue; }
      const r = rgba[o], g = rgba[o + 1], b = rgba[o + 2];
      let bd = Infinity, bj = 0;
      for (let j = 0; j < cols.length; j++) { const c = cols[j].c; const d = Math.hypot(r - c[0], g - c[1], b - c[2]); if (d < bd) { bd = d; bj = j; } }
      if (bd < tol) { like[i] = 1; near[i] = bj + 1; }
      else if (checker) {
        const mn = Math.min(r, g, b), mx = Math.max(r, g, b);
        if (mn > lightMin && mx - mn < 20 + tol * 0.2) { like[i] = 1; near[i] = bj + 1; }
      }
    }

    // 2) connected components of background-like pixels (4-connectivity)
    const comp = new Int32Array(N).fill(-1);
    const stack = new Int32Array(N);
    const bgMask = new Uint8Array(N);
    let nc = 0;
    for (let s = 0; s < N; s++) {
      if (!like[s] || comp[s] >= 0) continue;
      let sp = 0, size = 0, edge = false, t0 = 0, t1 = 0;
      stack[sp++] = s; comp[s] = nc;
      const members = [];
      while (sp) {
        const i = stack[--sp]; members.push(i); size++;
        const x = i % W, y = (i - x) / W;
        if (x === 0 || y === 0 || x === W - 1 || y === H - 1) edge = true;
        if (near[i] === 1) t0++; else if (near[i] === 2) t1++;
        if (x > 0 && like[i - 1] && comp[i - 1] < 0) { comp[i - 1] = nc; stack[sp++] = i - 1; }
        if (x < W - 1 && like[i + 1] && comp[i + 1] < 0) { comp[i + 1] = nc; stack[sp++] = i + 1; }
        if (y > 0 && like[i - W] && comp[i - W] < 0) { comp[i - W] = nc; stack[sp++] = i - W; }
        if (y < H - 1 && like[i + W] && comp[i + W] < 0) { comp[i + W] = nc; stack[sp++] = i + W; }
      }
      // enclosed checkerboard pocket: both checker tones present
      const pocket = !edge && checker && size >= 40 && t0 / size > 0.2 && t1 / size > 0.2;
      if (edge || pocket) for (const i of members) bgMask[i] = 1;
      nc++;
    }

    // 3) foreground = not background; drop tiny specks, choke 1 px (JPEG halo)
    let fg = new Uint8Array(N);
    for (let i = 0; i < N; i++) fg[i] = bgMask[i] ? 0 : 1;
    fg = dropSmall(fg, W, H, Math.max(12, Math.round(N * 0.00002)));
    const eroded = new Uint8Array(N);
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      const i = y * W + x;
      if (!fg[i]) continue;
      const o = i * 4, dark = Math.max(rgba[o], rgba[o + 1], rgba[o + 2]) < 110;
      const edge = (x > 0 && !fg[i - 1]) || (x < W - 1 && !fg[i + 1]) || (y > 0 && !fg[i - W]) || (y < H - 1 && !fg[i + W]);
      eroded[i] = edge && !dark ? 0 : 1;
    }
    fg = dropSmall(eroded, W, H, Math.max(12, Math.round(N * 0.00002)));

    // 4) soft edge: 3x3 box blur of the mask, remapped for a crisp but antialiased edge
    let minX = W, minY = H, maxX = -1, maxY = -1;
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      const i = y * W + x;
      let s = 0, n = 0;
      for (let dy = -1; dy <= 1; dy++) { const yy = y + dy; if (yy < 0 || yy >= H) continue; for (let dx = -1; dx <= 1; dx++) { const xx = x + dx; if (xx < 0 || xx >= W) continue; s += fg[yy * W + xx]; n++; } }
      const a = Math.max(0, Math.min(1, (s / n - 0.25) / 0.5));
      let v = Math.round(a * 255);
      if (transp && rgba[i * 4 + 3] < 255) v = Math.min(v, rgba[i * 4 + 3]);
      alpha[i] = v;
      if (v > 8) { if (x < minX) minX = x; if (x > maxX) maxX = x; if (y < minY) minY = y; if (y > maxY) maxY = y; }
    }
    if (maxX < 0) throw new Error('Aucun logo trouvé après suppression du fond. Essayez une tolérance plus faible.');
    return { alpha, checker, bgColors: cols.map((c) => c.c.map(Math.round)), bbox: { x: minX, y: minY, w: maxX - minX + 1, h: maxY - minY + 1 } };
  }

  function dropSmall(m, W, H, minSize) {
    const N = W * H, seen = new Uint8Array(N), stack = new Int32Array(N), out = m.slice();
    for (let s = 0; s < N; s++) {
      if (!m[s] || seen[s]) continue;
      let sp = 0; stack[sp++] = s; seen[s] = 1; const mem = [];
      while (sp) {
        const i = stack[--sp]; mem.push(i);
        const x = i % W;
        if (x > 0 && m[i - 1] && !seen[i - 1]) { seen[i - 1] = 1; stack[sp++] = i - 1; }
        if (x < W - 1 && m[i + 1] && !seen[i + 1]) { seen[i + 1] = 1; stack[sp++] = i + 1; }
        if (i >= W && m[i - W] && !seen[i - W]) { seen[i - W] = 1; stack[sp++] = i - W; }
        if (i < N - W && m[i + W] && !seen[i + W]) { seen[i + W] = 1; stack[sp++] = i + W; }
      }
      if (mem.length < minSize) for (const i of mem) out[i] = 0;
    }
    return out;
  }

  // push logo colours a few pixels outward under transparent areas, so upscaling never
  // bleeds the old background colour into the edge
  function bleed(rgba, W, H, alpha, passes) {
    const px = new Uint8ClampedArray(rgba);
    let known = new Uint8Array(W * H);
    for (let i = 0; i < W * H; i++) known[i] = alpha[i] > 200 ? 1 : 0;
    for (let p = 0; p < passes; p++) {
      const nk = known.slice();
      for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
        const i = y * W + x; if (known[i]) continue;
        let r = 0, g = 0, b = 0, n = 0;
        for (const j of [x > 0 ? i - 1 : -1, x < W - 1 ? i + 1 : -1, y > 0 ? i - W : -1, y < H - 1 ? i + W : -1]) {
          if (j >= 0 && known[j]) { r += px[j * 4]; g += px[j * 4 + 1]; b += px[j * 4 + 2]; n++; }
        }
        if (n) { px[i * 4] = r / n; px[i * 4 + 1] = g / n; px[i * 4 + 2] = b / n; nk[i] = 1; }
      }
      known = nk;
    }
    return px;
  }

  // transparent PNG canvas, cropped to the logo (with a small margin) and upscaled
  function compose(rgba, W, H, alpha, scale, bbox) {
    const pad = Math.round(Math.max(bbox.w, bbox.h) * 0.01) + 2;
    const cx = Math.max(0, bbox.x - pad), cy = Math.max(0, bbox.y - pad);
    const cw = Math.min(W, bbox.x + bbox.w + pad) - cx, ch = Math.min(H, bbox.y + bbox.h + pad) - cy;
    const col = bleed(rgba, W, H, alpha, 4);
    const src = document.createElement('canvas'); src.width = cw; src.height = ch;
    const sctx = src.getContext('2d');
    const cimg = sctx.createImageData(cw, ch);
    const msk = document.createElement('canvas'); msk.width = cw; msk.height = ch;
    const mctx = msk.getContext('2d');
    const mimg = mctx.createImageData(cw, ch);
    for (let y = 0; y < ch; y++) for (let x = 0; x < cw; x++) {
      const i = (y + cy) * W + (x + cx), o = (y * cw + x) * 4;
      cimg.data[o] = col[i * 4]; cimg.data[o + 1] = col[i * 4 + 1]; cimg.data[o + 2] = col[i * 4 + 2]; cimg.data[o + 3] = 255;
      mimg.data[o] = mimg.data[o + 1] = mimg.data[o + 2] = alpha[i]; mimg.data[o + 3] = 255;
    }
    sctx.putImageData(cimg, 0, 0); mctx.putImageData(mimg, 0, 0);
    const k = Math.max(1, Math.min(scale, Math.floor(Math.sqrt(64e6 / (cw * ch)) * 100) / 100, 16000 / Math.max(cw, ch)));
    const OW = Math.round(cw * k), OH = Math.round(ch * k);
    const up = (c) => {
      const d = document.createElement('canvas'); d.width = OW; d.height = OH;
      const x = d.getContext('2d', { willReadFrequently: true }); x.imageSmoothingEnabled = true; x.imageSmoothingQuality = 'high';
      x.drawImage(c, 0, 0, OW, OH); return x.getImageData(0, 0, OW, OH);
    };
    const big = up(src), bm = up(msk);
    const soft = k > 1.5 ? 1 : 0;
    for (let i = 0, n = OW * OH; i < n; i++) {
      let a = bm.data[i * 4];
      if (soft) a = Math.max(0, Math.min(255, (a - 96) * 255 / 96));   // re-sharpen the upscaled edge
      big.data[i * 4 + 3] = a;
    }
    const out = document.createElement('canvas'); out.width = OW; out.height = OH;
    out.getContext('2d').putImageData(big, 0, 0);
    return out;
  }

  // for vectorisation: replace the removed background by a solid colour absent from the logo
  function flatten(rgba, W, H, alpha) {
    const cands = [[255, 0, 255], [0, 255, 0], [0, 255, 255], [255, 255, 0], [0, 0, 255]];
    let best = cands[0], bd = -1;
    for (const c of cands) {
      let md = Infinity;
      for (let i = 0; i < W * H; i += 7) {
        if (alpha[i] < 128) continue;
        const d = Math.hypot(rgba[i * 4] - c[0], rgba[i * 4 + 1] - c[1], rgba[i * 4 + 2] - c[2]);
        if (d < md) md = d;
      }
      if (md > bd) { bd = md; best = c; }
    }
    const out = new Uint8ClampedArray(W * H * 4);
    for (let i = 0; i < W * H; i++) {
      const a = alpha[i] / 255;
      for (let c = 0; c < 3; c++) out[i * 4 + c] = rgba[i * 4 + c] * a + best[c] * (1 - a);
      out[i * 4 + 3] = 255;
    }
    return { rgba: out, key: best };
  }

  root.BgRemove = { removeBackground, compose, flatten };
})(typeof self !== 'undefined' ? self : this);
