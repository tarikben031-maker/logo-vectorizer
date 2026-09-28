/* Logo vectorizer engine (JavaScript port of vectorizer.py).
 * Two-colour raster logo -> clean SVG: sharp corners, straight stems, exact circles, smooth curves.
 *
 *   LogoVectorizer.vectorizeRGBA(rgba: Uint8ClampedArray|Uint8Array, width, height, onProgress?)
 *     -> { svg, svg_transparent, width, height, fg, bg, shapes, arcs, ms }
 *
 * Pipeline: colour separation -> coverage-aware subpixel mask -> Gaussian smoothing -> border
 * following (Suzuki) -> Douglas-Peucker polygon -> corner detection + merging of antialias-split
 * corners -> exact straight lines (axis snap) -> exact circular arcs -> Schneider Bezier fitting
 * -> per-shape area safety check.
 */
(function (root) {
  'use strict';

  const LINE_TOL = 0.2, AXIS_SNAP = 2.5, MERGE_LEN = 0.9, MASK_BLUR = 0.35;
  let ARC_RMAX = 25.0, BIG_ARCS = false;
  // precision profile: 'logo' (flat colours, maximum fidelity) or 'illus' (gradients, smoother curves)
  let DP_EPS = 0.2, FIT_TOL = 0.25, SMOOTH_MAX = 0.75;
  let CLEAN_PX = (typeof process !== 'undefined' && process.env && process.env.VCP) ? +process.env.VCP : 0;
  let CUR_T = null, CUR_W = 0, CUR_H = 0;
  let TIP_REBUILD = !((typeof process !== 'undefined' && process.env && process.env.VNOTIP));
  let LAYER_BLUR = (typeof process !== 'undefined' && process.env && process.env.VLB) ? +process.env.VLB : 0.7;
  function setProfile(illus, style) {
    if (illus) { DP_EPS = 0.45; FIT_TOL = 0.5; SMOOTH_MAX = 2.0; ARC_RMAX = 5000; BIG_ARCS = true; }
    else if (style === 'clean') {
      // idealised shapes (vectorizer.ai-like): smoother curves, large exact circles, fewer points
      DP_EPS = +(ENV('VDP') || 0.26); FIT_TOL = +(ENV('VFT') || 0.34); SMOOTH_MAX = +(ENV('VSM') || 1.1); ARC_RMAX = 25.0; BIG_ARCS = false;
    }
    else { DP_EPS = 0.2; FIT_TOL = 0.25; SMOOTH_MAX = 0.75; ARC_RMAX = 25.0; BIG_ARCS = false; }
  }
  function ENV(k) { return (typeof process !== 'undefined' && process.env) ? process.env[k] : undefined; }
  const MAX_SUBPIXELS = 110e6;       // memory guard for the supersampled mask
  const DEG = 180 / Math.PI;

  let S = 8;                         // supersampling factor (lowered for very large images)
  let ARCS = 0;
  let USE_ARCS = true;

  // ---------------------------------------------------------------- vector helpers
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1]];
  const add = (a, b) => [a[0] + b[0], a[1] + b[1]];
  const mul = (a, k) => [a[0] * k, a[1] * k];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1];
  const cross = (a, b) => a[0] * b[1] - a[1] * b[0];
  const norm = (a) => Math.hypot(a[0], a[1]);
  const unit = (a) => { const n = norm(a); return n > 1e-12 ? [a[0] / n, a[1] / n] : [a[0], a[1]]; };
  const angleBetween = (a, b) => Math.atan2(Math.abs(cross(a, b)), dot(a, b)) * DEG;
  const sang = (a, b) => Math.atan2(cross(a, b), dot(a, b)) * DEG;
  const mod = (a, m) => ((a % m) + m) % m;
  const clip = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
  function fmt(x) {
    let s = x.toFixed(2);
    s = s.replace(/0+$/, '').replace(/\.$/, '');
    return s === '-0' ? '0' : s;
  }
  const P2 = (p) => fmt(p[0]) + ' ' + fmt(p[1]);

  function fitline(pts) {
    let cx = 0, cy = 0;
    for (const p of pts) { cx += p[0]; cy += p[1]; }
    cx /= pts.length; cy /= pts.length;
    let a = 0, b = 0, c = 0;
    for (const p of pts) { const dx = p[0] - cx, dy = p[1] - cy; a += dx * dx; b += dx * dy; c += dy * dy; }
    const th = 0.5 * Math.atan2(2 * b, a - c);
    return [[cx, cy], [Math.cos(th), Math.sin(th)]];
  }
  function intersect(c1, d1, c2, d2) {
    const den = cross(d1, d2);
    if (Math.abs(den) < 1e-6) return null;
    return add(c1, mul(d1, cross(sub(c2, c1), d2) / den));
  }
  function lineDev(pts, a, b) {
    if (!pts.length) return 0;
    const v = sub(b, a), n = norm(v);
    let m = 0;
    if (n < 1e-9) { for (const p of pts) m = Math.max(m, norm(sub(p, a))); return m; }
    for (const p of pts) m = Math.max(m, Math.abs(cross(v, sub(p, a))) / n);
    return m;
  }
  function unwrap(a) {
    const out = new Float64Array(a.length);
    if (!a.length) return out;
    out[0] = a[0];
    let corr = 0;
    for (let i = 1; i < a.length; i++) {
      const d = a[i] - a[i - 1];
      let dm = mod(d + Math.PI, 2 * Math.PI) - Math.PI;
      if (dm === -Math.PI && d > 0) dm = Math.PI;
      let ph = dm - d;
      if (Math.abs(d) < Math.PI) ph = 0;
      corr += ph;
      out[i] = a[i] + corr;
    }
    return out;
  }
  function ptp(arr, s, e) { let lo = Infinity, hi = -Infinity; for (let i = s; i < e; i++) { if (arr[i] < lo) lo = arr[i]; if (arr[i] > hi) hi = arr[i]; } return hi - lo; }

  // ---------------------------------------------------------------- Bezier fitting (Schneider)
  function bez(p, u) {
    const m = 1 - u, a = m * m * m, b = 3 * m * m * u, c = 3 * m * u * u, d = u * u * u;
    return [a * p[0][0] + b * p[1][0] + c * p[2][0] + d * p[3][0], a * p[0][1] + b * p[1][1] + c * p[2][1] + d * p[3][1]];
  }
  function chord(pts) {
    const d = new Float64Array(pts.length);
    for (let i = 1; i < pts.length; i++) d[i] = d[i - 1] + norm(sub(pts[i], pts[i - 1]));
    const L = d[d.length - 1];
    if (L > 0) for (let i = 0; i < d.length; i++) d[i] /= L;
    return d;
  }
  function gen(pts, u, t1, t2) {
    const p0 = pts[0], p3 = pts[pts.length - 1];
    let c00 = 0, c01 = 0, c11 = 0, x0 = 0, x1 = 0;
    for (let i = 0; i < pts.length; i++) {
      const ui = u[i], m = 1 - ui;
      const a1 = [t1[0] * 3 * m * m * ui, t1[1] * 3 * m * m * ui];
      const a2 = [t2[0] * 3 * m * ui * ui, t2[1] * 3 * m * ui * ui];
      c00 += dot(a1, a1); c01 += dot(a1, a2); c11 += dot(a2, a2);
      const b0 = m * m * m + 3 * m * m * ui, b1 = 3 * m * ui * ui + ui * ui * ui;
      const tmp = [pts[i][0] - (b0 * p0[0] + b1 * p3[0]), pts[i][1] - (b0 * p0[1] + b1 * p3[1])];
      x0 += dot(a1, tmp); x1 += dot(a2, tmp);
    }
    const det = c00 * c11 - c01 * c01, seg = norm(sub(p3, p0));
    let al1, al2;
    if (Math.abs(det) > 1e-12) { al1 = (x0 * c11 - x1 * c01) / det; al2 = (c00 * x1 - c01 * x0) / det; }
    else { al1 = al2 = seg / 3; }
    if (!(1e-6 * seg < al1 && al1 < 2 * seg) || !(1e-6 * seg < al2 && al2 < 2 * seg)) al1 = al2 = seg / 3;
    return [p0, add(p0, mul(t1, al1)), add(p3, mul(t2, al2)), p3];
  }
  function reparam(b, pts, u) {
    const d1 = [mul(sub(b[1], b[0]), 3), mul(sub(b[2], b[1]), 3), mul(sub(b[3], b[2]), 3)];
    const d2 = [mul(sub(d1[1], d1[0]), 2), mul(sub(d1[2], d1[1]), 2)];
    const out = new Float64Array(u.length);
    for (let i = 0; i < u.length; i++) {
      const ui = u[i], m = 1 - ui;
      const q = sub(bez(b, ui), pts[i]);
      const q1 = [m * m * d1[0][0] + 2 * m * ui * d1[1][0] + ui * ui * d1[2][0], m * m * d1[0][1] + 2 * m * ui * d1[1][1] + ui * ui * d1[2][1]];
      const q2 = [m * d2[0][0] + ui * d2[1][0], m * d2[0][1] + ui * d2[1][1]];
      const num = dot(q, q1), den = dot(q1, q1) + dot(q, q2);
      let v = Math.abs(den) > 1e-12 ? ui - num / den : ui;
      if (!Number.isFinite(v)) v = ui;
      out[i] = clip(v, 0, 1);
    }
    return out;
  }
  function fit(pts, t1, t2, err, depth = 0) {
    const n = pts.length;
    if (n <= 3) {
      const s = norm(sub(pts[n - 1], pts[0])) / 3;
      return [[pts[0], add(pts[0], mul(t1, s)), add(pts[n - 1], mul(t2, s)), pts[n - 1]]];
    }
    let u = chord(pts), b, dd;
    for (let it = 0; it < 10; it++) {
      b = gen(pts, u, t1, t2);
      dd = new Float64Array(n); let mx = 0;
      for (let i = 0; i < n; i++) { const e = sub(bez(b, u[i]), pts[i]); dd[i] = dot(e, e); if (dd[i] > mx) mx = dd[i]; }
      if (mx < err * err || depth > 10) return [b];
      u = reparam(b, pts, u);
    }
    let im = 0; for (let i = 1; i < n; i++) if (dd[i] > dd[im]) im = i;
    const i = clip(im, 2, n - 3);
    const tc = unit(sub(pts[i - 2], pts[i + 2]));
    return fit(pts.slice(0, i + 1), t1, tc, err, depth + 1).concat(fit(pts.slice(i), mul(tc, -1), t2, err, depth + 1));
  }

  function smoothOpen(pts, k) {
    const n = pts.length;
    if (n < 5 || k < 1) return pts;
    const out = pts.slice();
    for (let i = 1; i < n - 1; i++) {
      const r = Math.min(k, i, n - 1 - i);
      let sx = 0, sy = 0;
      for (let j = i - r; j <= i + r; j++) { sx += pts[j][0]; sy += pts[j][1]; }
      out[i] = [sx / (2 * r + 1), sy / (2 * r + 1)];
    }
    return out;
  }
  function smoothClosed(p, k) {
    const n = p.length, out = new Array(n), w = 2 * k + 1;
    let sx = 0, sy = 0;
    for (let j = -k; j <= k; j++) { const q = p[mod(j, n)]; sx += q[0]; sy += q[1]; }
    for (let i = 0; i < n; i++) {
      out[i] = [sx / w, sy / w];
      const a = p[mod(i - k, n)], b = p[mod(i + k + 1, n)];
      sx += b[0] - a[0]; sy += b[1] - a[1];
    }
    return out;
  }

  // ---------------------------------------------------------------- circles / arcs
  function fitCircle(pts) {
    const n = pts.length;
    let mx = 0, my = 0;
    for (const p of pts) { mx += p[0]; my += p[1]; }
    mx /= n; my /= n;
    // least squares for 2x*cx + 2y*cy + c = x^2 + y^2 (centred coordinates)
    const M = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], v = [0, 0, 0];
    for (const p of pts) {
      const x = p[0] - mx, y = p[1] - my, row = [2 * x, 2 * y, 1], bb = x * x + y * y;
      for (let r = 0; r < 3; r++) { v[r] += row[r] * bb; for (let c = 0; c < 3; c++) M[r][c] += row[r] * row[c]; }
    }
    const s = solve3(M, v);
    if (!s) return null;
    const r = Math.sqrt(Math.max(s[2] + s[0] * s[0] + s[1] * s[1], 0));
    const c = [s[0] + mx, s[1] + my];
    const res = new Float64Array(n);
    for (let i = 0; i < n; i++) res[i] = Math.abs(Math.hypot(pts[i][0] - c[0], pts[i][1] - c[1]) - r);
    return { c, r, res };
  }
  function solve3(M, v) {
    const A = M.map((row, i) => row.concat([v[i]]));
    for (let col = 0; col < 3; col++) {
      let piv = col;
      for (let r = col + 1; r < 3; r++) if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
      if (Math.abs(A[piv][col]) < 1e-12) return null;
      [A[col], A[piv]] = [A[piv], A[col]];
      for (let r = 0; r < 3; r++) {
        if (r === col) continue;
        const f = A[r][col] / A[col][col];
        for (let c = col; c < 4; c++) A[r][c] -= f * A[col][c];
      }
    }
    return [A[0][3] / A[0][0], A[1][3] / A[1][1], A[2][3] / A[2][2]];
  }
  const maxOf = (a) => { let m = -Infinity; for (const x of a) if (x > m) m = x; return m; };
  const rmsOf = (a) => { let s = 0; for (const x of a) s += x * x; return Math.sqrt(s / a.length); };
  function angles(pts, c) { const a = new Float64Array(pts.length); for (let i = 0; i < pts.length; i++) a[i] = Math.atan2(pts[i][1] - c[1], pts[i][0] - c[0]); return unwrap(a); }
  function monotone(ang, tol) {
    let up = true, down = true;
    for (let i = 1; i < ang.length; i++) { const d = ang[i] - ang[i - 1]; if (d < -tol) up = false; if (d > tol) down = false; }
    return up || down;
  }

  function arcBeziers(c, r, a0, a1) {
    const sweep = a1 - a0, nseg = Math.max(1, Math.ceil(Math.abs(sweep) / (Math.PI / 2) - 1e-6)), out = [];
    for (let i = 0; i < nseg; i++) {
      const t0 = a0 + sweep * i / nseg, t1 = a0 + sweep * (i + 1) / nseg, k = 4 / 3 * Math.tan((t1 - t0) / 4);
      const p0 = [c[0] + r * Math.cos(t0), c[1] + r * Math.sin(t0)], p3 = [c[0] + r * Math.cos(t1), c[1] + r * Math.sin(t1)];
      const p1 = [p0[0] - k * r * Math.sin(t0), p0[1] + k * r * Math.cos(t0)];
      const p2 = [p3[0] + k * r * Math.sin(t1), p3[1] - k * r * Math.cos(t1)];
      out.push([p0, p1, p2, p3]);
    }
    return out;
  }
  const bzStr = (bz) => 'C' + P2(bz[1]) + ' ' + P2(bz[2]) + ' ' + P2(bz[3]);

  function tryArc(pts, closed = false) {
    const n = pts.length;
    if (n < 12) return null;
    const core = closed ? pts : pts.slice(Math.floor(n / 6), n - Math.floor(n / 6));
    const f = fitCircle(core);
    if (!f) return null;
    const { c, r, res } = f;
    if (r < 1.2 || r > ARC_RMAX) return null;
    if (maxOf(res) > Math.min(0.6, Math.max(0.25, 0.035 * r)) || rmsOf(res) > Math.min(0.2, Math.max(0.1, 0.013 * r))) return null;
    if (!closed) {
      let m = 0; for (const p of pts) m = Math.max(m, Math.abs(Math.hypot(p[0] - c[0], p[1] - c[1]) - r));
      if (m > Math.min(1.2, Math.max(0.9, 0.12 * r))) return null;
    }
    const ang = angles(pts, c);
    if (!closed && !monotone(ang, 0.02)) return null;
    const sweep = ang[n - 1] - ang[0];
    if (!closed && Math.abs(sweep) < 100 / DEG) return null;
    return { c, r, a0: ang[0], a1: ang[n - 1] };
  }

  const RUN_FRACS = (() => {
    const f = [];
    for (let k = 0; 0.95 - 0.05 * k > 0.3 + 1e-9; k++) f.push(0.95 - 0.05 * k);
    for (let k = 0; 0.3 - 0.02 * k > 0.04 + 1e-9; k++) f.push(0.3 - 0.02 * k);
    return f;
  })();

  function findArcRun(pts, minSweep = 170 / DEG) {
    const N = pts.length;
    if (N < 40) return null;
    const stepd = Math.max(1, Math.floor(N / 160));
    const q = []; for (let i = 0; i < N; i += stepd) q.push(pts[i]);
    const M = q.length;
    const thr = new Float64Array(M - 1);
    for (let i = 0; i < M - 1; i++) thr[i] = Math.atan2(q[i + 1][1] - q[i][1], q[i + 1][0] - q[i][0]);
    const th = unwrap(thr);
    if (!BIG_ARCS && ptp(th, 0, th.length) < minSweep * 0.9) return null;
    const lengths = [...new Set(RUN_FRACS.map((f) => Math.floor(M * f)))].sort((a, b) => b - a);
    let best = null;
    for (const L of lengths) {
      if (L < 14) break;
      const step = Math.max(1, Math.floor(L / 8));
      for (let st = 0; st <= M - L; st += step) {
        if (!BIG_ARCS && ptp(th, st, Math.min(th.length, st + L - 1)) < minSweep * 0.85) continue;
        const seg = q.slice(st, st + L);
        const f = fitCircle(seg);
        if (!f) continue;
        const { c, r, res } = f;
        if (r < 1.5 || r > ARC_RMAX) continue;
        if (maxOf(res) > Math.min(0.6, Math.max(0.25, 0.035 * r)) || rmsOf(res) > Math.min(0.2, Math.max(0.09, 0.012 * r))) continue;
        const ang = angles(seg, c);
        const sweepNeed = (BIG_ARCS && r > 20) ? Math.min(minSweep, 40 / r) : minSweep;
        if (Math.abs(ang[ang.length - 1] - ang[0]) < sweepNeed) continue;
        if (!monotone(ang, 0.03)) continue;
        best = { i0: st * stepd, i1: Math.min(N - 1, (st + L - 1) * stepd), c, r };
        break;
      }
      if (best) break;
    }
    if (!best) return null;
    const { c, r } = best, tol = Math.min(0.6, Math.max(0.25, 0.035 * r));
    const off = (p) => Math.abs(Math.hypot(p[0] - c[0], p[1] - c[1]) - r);
    while (best.i0 > 0 && off(pts[best.i0 - 1]) < tol) best.i0--;
    while (best.i1 < N - 1 && off(pts[best.i1 + 1]) < tol) best.i1++;
    return best;
  }

  function bzPath(ptsq, tA, tB) {
    let plen = 0;
    for (let i = 1; i < ptsq.length; i++) plen += norm(sub(ptsq[i], ptsq[i - 1]));
    if (ptsq.length < 3 || plen < 0.25) return 'L' + P2(ptsq[ptsq.length - 1]);
    return fit(ptsq, tA, tB, FIT_TOL).map(bzStr).join('');
  }
  function endDir(seq) {
    const q = seq.slice(0, Math.max(3, Math.min(seq.length, Math.floor(1.0 * S))));
    if (q.length >= 3) {
      let d = unit(fitline(q)[1]);
      if (dot(d, sub(q[q.length - 1], q[0])) < 0) d = mul(d, -1);
      return d;
    }
    return unit(sub(seq[seq.length - 1], seq[0]));
  }

  function emitCurve(rawp, sp, pa, pb, t1, t2, defaultPts, depth = 0) {
    const run = (depth < 4 && USE_ARCS) ? findArcRun(rawp) : null;
    if (!run) return bzPath(defaultPts, t1, t2);
    const { i0, i1, c, r } = run;
    let ea = add(c, mul(unit(sub(rawp[i0], c)), r)), eb = add(c, mul(unit(sub(rawp[i1], c)), r));
    if (i0 <= 2 && Math.abs(norm(sub(pa, c)) - r) < 0.4) ea = pa;
    if (i1 >= rawp.length - 3 && Math.abs(norm(sub(pb, c)) - r) < 0.4) eb = pb;
    const seg = rawp.slice(i0, i1 + 1);
    const ang = angles(seg, c);
    const a0 = Math.atan2(ea[1] - c[1], ea[0] - c[0]), a1 = Math.atan2(eb[1] - c[1], eb[0] - c[0]);
    let sw = a1 - a0;
    const want = ang[ang.length - 1] - ang[0];
    while (sw - want > Math.PI) sw -= 2 * Math.PI;
    while (want - sw > Math.PI) sw += 2 * Math.PI;
    // verify: the exact arc must follow the original points
    const arcpts = [];
    for (let k = 0; k < 64; k++) { const t = a0 + sw * k / 63; arcpts.push([c[0] + r * Math.cos(t), c[1] + r * Math.sin(t)]); }
    const seg4 = seg.filter((_, k) => k % 4 === 0);
    let dA = 0, dB = 0;
    for (const a of arcpts) { let m = Infinity; for (const s of seg4) m = Math.min(m, norm(sub(a, s))); dA = Math.max(dA, m); }
    for (const s of seg4) { let m = Infinity; for (const a of arcpts) m = Math.min(m, norm(sub(a, s))); dB = Math.max(dB, m); }
    if (Math.max(dA, dB) > 0.8 || Math.abs(sw - want) > 0.6) return bzPath(defaultPts, t1, t2);
    ARCS++;
    const sgn = sw > 0 ? 1 : -1;
    const tan0 = [-sgn * Math.sin(a0), sgn * Math.cos(a0)];
    const tan1 = [-sgn * Math.sin(a0 + sw), sgn * Math.cos(a0 + sw)];
    let o = '';
    if (norm(sub(ea, pa)) > 0.05) {
      const partA = i0 > 1 ? [pa].concat(sp.slice(1, i0), [ea]) : [pa, ea];
      let tAend = endDir(partA.slice().reverse());
      if (angleBetween(mul(tAend, -1), tan0) < 25) tAend = mul(tan0, -1);
      o += emitCurve(rawp.slice(0, i0 + 1), sp.slice(0, i0 + 1), pa, ea, t1, tAend, partA, depth + 1);
    }
    o += arcBeziers(c, r, a0, a0 + sw).map(bzStr).join('');
    if (norm(sub(pb, eb)) > 0.05) {
      const partB = i1 < sp.length - 2 ? [eb].concat(sp.slice(i1 + 1, sp.length - 1), [pb]) : [eb, pb];
      let tBst = endDir(partB);
      if (angleBetween(tBst, tan1) < 25) tBst = tan1;
      o += emitCurve(rawp.slice(i1), sp.slice(i1), eb, pb, tBst, t2, partB, depth + 1);
    }
    return o;
  }

  // ---------------------------------------------------------------- polygon (Douglas-Peucker, closed)
  function approxPolyClosed(P, eps) {
    const n = P.length;
    const d2 = (a, b) => (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2;
    let f1 = 0, best = -1;
    for (let i = 0; i < n; i++) { const d = d2(P[i], P[0]); if (d > best) { best = d; f1 = i; } }
    let f0 = 0; best = -1;
    for (let i = 0; i < n; i++) { const d = d2(P[i], P[f1]); if (d > best) { best = d; f0 = i; } }
    const keep = new Set([f0, f1]);
    const stack = [[f0, f1], [f1, f0]];
    while (stack.length) {
      const [a, b] = stack.pop();
      const len = mod(b - a, n);
      if (len < 2) continue;
      const A = P[a], B = P[b], v = sub(B, A), vn = norm(v);
      let far = -1, dmax = -1;
      for (let k = 1; k < len; k++) {
        const i = (a + k) % n;
        const d = vn < 1e-12 ? norm(sub(P[i], A)) : Math.abs(cross(v, sub(P[i], A))) / vn;
        if (d > dmax) { dmax = d; far = i; }
      }
      if (dmax > eps) { keep.add(far); stack.push([a, far], [far, b]); }
    }
    return [...keep].sort((x, y) => x - y);
  }

  // ---------------------------------------------------------------- tracing one border
  function trace(raw) {
    const n = raw.length;
    const P = smoothClosed(raw, 2);
    const idx = approxPolyClosed(P, DP_EPS);
    if (idx.length < 3) return smoothLoop(P);
    let VV = idx.map((i) => ({ p: P[i].slice(), iin: i, iout: i }));
    let changed = true;
    while (changed && VV.length > 4) {
      changed = false;
      const m = VV.length;
      let best = null;
      for (let k = 0; k < m; k++) {
        for (const j of [1, 2, 3, 4, 5]) {
          if (j + 3 > m) break;
          const ids = []; for (let q = 0; q <= j; q++) ids.push((k + q) % m);
          const A = VV[mod(k - 1, m)].p, D = VV[(k + j + 1) % m].p;
          const pts_ = ids.map((i) => VV[i].p);
          let inner = 0; for (let q = 0; q < j; q++) inner += norm(sub(pts_[q + 1], pts_[q]));
          const B = pts_[0], E = pts_[pts_.length - 1];
          const la = norm(sub(B, A)), ld = norm(sub(D, E));
          const tipTot = Math.abs(sang(sub(B, A), sub(D, E)));
          let tipMode = false;
          if (j > 3 || inner > MERGE_LEN + 0.55 * (j - 1)) {
            // wider antialias/noise split: only between long straight-ish edges
            const okCorner = j <= 3 && inner <= 1.8 + 0.4 * (j - 1) && Math.min(la, ld) >= Math.max(3.5, 3.5 * inner);
            // blurred wedge tip (lines crossing, a colour thinning to a point): rebuild the sharp tip
            tipMode = TIP_REBUILD && tipTot > 125 && inner <= 5 && Math.min(la, ld) >= Math.max(5, 2.2 * inner);
            // blurred corner (e.g. the crotch where two strokes cross)
            if (!tipMode && TIP_REBUILD && tipTot >= 70 && inner <= 3.5 && Math.min(la, ld) >= Math.max(4, 1.8 * inner)) tipMode = true;
            if (!okCorner && !tipMode) continue;
          }
          if (j > 1 && (Math.min(la, ld) < 1.0 || Math.min(la, ld) < 1.5 * inner)) continue;
          const seq = [A].concat(pts_, [D]);
          let pos = true, neg = true;
          for (let q = 0; q < seq.length - 2; q++) {
            const t = sang(sub(seq[q + 1], seq[q]), sub(seq[q + 2], seq[q + 1]));
            if (!(t > 0)) pos = false; if (!(t < 0)) neg = false;
          }
          if (!pos && !neg) continue;
          const tot = Math.abs(sang(sub(B, A), sub(D, E)));
          if (tot < 55 || tot > 178) continue;
          const x = intersect(A, unit(sub(B, A)), D, unit(sub(D, E)));
          if (!x) continue;
          const lim = tipMode ? (tot > 125 ? Math.min(4.5, Math.max(1.4, 0.4 * Math.min(la, ld))) : Math.min(2.2, Math.max(0.9, 0.35 * Math.min(la, ld)))) : (tot <= 150 ? 0.75 : 1.4);
          let md = Infinity; for (const p of pts_) md = Math.min(md, norm(sub(p, x)));
          if (md > lim) continue;
          if (tipMode && CUR_T) {
            // the rebuilt tip must lie on pixels that really carry this colour (no spikes into other colours)
            const mid = [(B[0] + E[0]) / 2, (B[1] + E[1]) / 2];
            let acc = 0, n = 0;
            for (let q = 1; q <= 6; q++) {
              const px = mid[0] + (x[0] - mid[0]) * q / 6, py = mid[1] + (x[1] - mid[1]) * q / 6;
              const ix = Math.min(CUR_W - 1, Math.max(0, Math.floor(px))), iy = Math.min(CUR_H - 1, Math.max(0, Math.floor(py)));
              acc += CUR_T[iy * CUR_W + ix]; n++;
            }
            if (acc / n < 0.3) continue;
          }
          const score = inner / j;
          if (!best || score < best.score) best = { score, ids, x };
        }
      }
      if (best) {
        const merged = { p: best.x, iin: VV[best.ids[0]].iin, iout: VV[best.ids[best.ids.length - 1]].iout };
        const drop = new Set(best.ids), NV = [];
        for (let i = 0; i < VV.length; i++) {
          if (i === best.ids[0]) NV.push(merged);
          else if (!drop.has(i)) NV.push(VV[i]);
        }
        VV = NV; changed = true;
      }
    }
    const m = VV.length;
    const V = VV.map((v) => v.p);
    const Lprev = V.map((v, k) => norm(sub(v, V[mod(k - 1, m)])));
    const Lnext = V.map((_, k) => Lprev[(k + 1) % m]);
    const ang = V.map((v, k) => angleBetween(sub(v, V[mod(k - 1, m)]), sub(V[(k + 1) % m], v)));
    const corner = new Array(m).fill(false);
    for (let k = 0; k < m; k++) {
      const a = ang[k];
      if (a >= 60) corner[k] = true;
      else if (a >= 40 && Math.min(Lprev[k], Lnext[k]) >= 1.3) corner[k] = true;
      else if (a >= 33 && a >= 2.0 * Math.max(ang[mod(k - 1, m)], ang[(k + 1) % m]) && Math.min(Lprev[k], Lnext[k]) >= 1.0) corner[k] = true;
    }
    const ci = []; corner.forEach((c, k) => { if (c) ci.push(k); });
    if (!ci.length) return smoothLoop(P);
    const Cpos = new Map(ci.map((k) => [k, V[k].slice()]));
    const cornerSet = new Set(ci);
    const idxIn = VV.map((v) => v.iin), idxOut = VV.map((v) => v.iout);
    const range = (a, b, wrapIfLE) => {                       // arange(a, b + (n if cond else 0) + 1) % n
      const end = b + ((wrapIfLE ? b <= a : b < a) ? n : 0);
      const r = []; for (let i = a; i <= end; i++) r.push(i % n); return r;
    };
    const segs = ci.map((ka, j) => { const kb = ci[(j + 1) % ci.length]; return { ka, kb, rng: range(idxOut[ka], idxIn[kb], true) }; });
    const kinds = segs.map(({ ka, kb, rng }) => {
      const pts = rng.map((i) => P[i]);
      const inner = rng.length > 6 ? pts.slice(2, pts.length - 2) : pts;
      return lineDev(inner, Cpos.get(ka), Cpos.get(kb)) < LINE_TOL ? 'L' : 'C';
    });
    segs.forEach(({ ka, kb }, s) => {
      if (kinds[s] !== 'L') return;
      const a = Cpos.get(ka), b = Cpos.get(kb), d = sub(b, a);
      if (norm(d) < 0.6) return;
      const g = Math.atan2(Math.abs(d[1]), Math.abs(d[0])) * DEG;
      if (g < AXIS_SNAP && Math.abs(a[1] - b[1]) < 0.7) { const y = (a[1] + b[1]) / 2; a[1] = b[1] = y; }
      else if (g > 90 - AXIS_SNAP && Math.abs(a[0] - b[0]) < 0.7) { const x = (a[0] + b[0]) / 2; a[0] = b[0] = x; }
    });
    const pos = (k) => (Cpos.has(k) ? Cpos.get(k) : V[k]);
    let out = 'M' + P2(Cpos.get(segs[0].ka));
    segs.forEach(({ ka, kb }, s) => {
      if (kinds[s] === 'L') { out += 'L' + P2(Cpos.get(kb)); return; }
      // long straight edges inside this curved run (stems, flats)
      const chain = [ka]; for (let q = 0; q < mod(kb - ka - 1, m); q++) chain.push((ka + 1 + q) % m); chain.push(kb);
      const lines = [];
      const longLine = (c0, c1) => {
        const u = chain[c0], w = chain[c1], pu = pos(u), pw = pos(w), L = norm(sub(pw, pu));
        if (L < 8) return false;
        for (let q = c0 + 1; q < c1; q++) {             // intermediate vertices must barely turn
          const k = chain[q];
          if (angleBetween(sub(V[k], V[mod(k - 1, m)]), sub(V[(k + 1) % m], V[k])) > 20) return false;
        }
        // every polygon edge in between must run in the line's direction (no wrapping round a tip)
        const dir = unit(sub(pw, pu));
        let turnSum = 0;
        for (let q = c0; q < c1; q++) {
          const e = sub(pos(chain[q + 1]), pos(chain[q]));
          if (norm(e) > 0.8 && (dot(unit(e), dir) < Math.cos(15 / DEG))) return false;
          if (q > c0) turnSum += sang(sub(pos(chain[q]), pos(chain[q - 1])), e);
        }
        if (Math.abs(turnSum) > 25) return false;
        const rr = range(idxOut[u], idxIn[w], false);
        const cut = Math.min(Math.floor(rr.length / 10), S);
        const pts = rr.slice(cut, rr.length - cut).map((i) => P[i]);
        if (pts.length < 3) return false;
        // noise tolerance, but a real curve (radius < 150 px) stays a curve
        const tol = Math.min(0.5, Math.max(0.35, (L * L) / (8 * 400)));
        let pos_ = 0, neg_ = 0;
        const v = sub(pw, pu);
        for (const p of pts) {
          const d = cross(v, sub(p, pu)) / L;
          if (Math.abs(d) > tol) return false;
          if (d > 0.05) pos_++; else if (d < -0.05) neg_++;
        }
        // systematic bulge on one side = gentle curve, not a straight edge
        if (Math.max(pos_, neg_) > 0.6 * pts.length && lineDev(pts, pu, pw) > 0.1) return false;
        return true;
      };
      for (let c = 0; c < chain.length - 1; c++) {
        let e = -1;
        for (let c1 = chain.length - 1; c1 >= c + 2; c1--) if (longLine(c, c1)) { e = c1; break; }
        if (e > 0) { lines.push([chain[c], chain[e]]); c = e - 1; continue; }
        const u = chain[c], w = chain[c + 1];
        const pu = pos(u), pw = pos(w), L = norm(sub(pw, pu));
        if (L < 2.5) continue;
        const rr = range(idxOut[u], idxIn[w], false);
        const cut = Math.floor(rr.length / 8);
        const inner = rr.slice(cut, rr.length - cut).map((i) => P[i]);
        if (inner.length < 3) continue;
        const dv_ = sub(pw, pu), g = Math.atan2(Math.abs(dv_[1]), Math.abs(dv_[0])) * DEG;
        const axis = g < 4 || g > 86;
        const dv = lineDev(inner, pu, pw);
        const nb = Math.max(norm(sub(V[u], V[mod(u - 1, m)])), norm(sub(V[(w + 1) % m], V[w])));
        const su = cross(sub(V[u], V[mod(u - 1, m)]), sub(V[(u + 1) % m], V[u]));
        const sw = cross(sub(V[w], V[mod(w - 1, m)]), sub(V[(w + 1) % m], V[w]));
        const extremum = !cornerSet.has(u) && !cornerSet.has(w) && su * sw > 0;
        if (extremum && dv > 0.03) continue;
        if (!axis && dv > 0.06) {                    // gentle arc: points all on one side of the chord
          let ps = 0, ng = 0;
          for (const p of inner) { const d = cross(dv_, sub(p, pu)) / L; if (d > 0.04) ps++; else if (d < -0.04) ng++; }
          if (Math.max(ps, ng) > 0.65 * inner.length) continue;
        }
        if (dv < Math.min(0.3, Math.max(0.1, 0.015 * L)) && (dv < 0.045 || L >= 1.8 * nb) && (axis || L >= 6)) lines.push([u, w]);
      }
      const pieces = []; let cur = ka;
      for (const [u, w] of lines) { if (u !== cur) pieces.push(['c', cur, u]); pieces.push(['l', u, w]); cur = w; }
      if (cur !== kb || !pieces.length) pieces.push(['c', cur, kb]);
      for (const [kind, u, w] of pieces) {
        if (kind !== 'l') continue;
        const d = sub(pos(w), pos(u)), g = Math.atan2(Math.abs(d[1]), Math.abs(d[0])) * DEG;
        if (!Cpos.has(u)) Cpos.set(u, V[u].slice());
        if (!Cpos.has(w)) Cpos.set(w, V[w].slice());
        const cu = Cpos.get(u), cw = Cpos.get(w);
        if (g < AXIS_SNAP && Math.abs(cu[1] - cw[1]) < 0.7) { const y = (cu[1] + cw[1]) / 2; cu[1] = cw[1] = y; }
        else if (g > 90 - AXIS_SNAP && Math.abs(cu[0] - cw[0]) < 0.7) { const x = (cu[0] + cw[0]) / 2; cu[0] = cw[0] = x; }
      }
      pieces.forEach(([kind, u, w], pi) => {
        const pa = pos(u), pb = pos(w);
        if (kind === 'l') { out += 'L' + P2(pb); return; }
        const rr = range(idxOut[u], idxIn[w], true);
        const rawSeg = rr.map((i) => raw[i]);
        let seglen = 0; for (let i = 1; i < rawSeg.length; i++) seglen += norm(sub(rawSeg[i], rawSeg[i - 1]));
        const kk = Math.trunc(clip(seglen / 10, 0.25, SMOOTH_MAX) * S);
        const sm = smoothOpen(rawSeg, kk);
        const trim = (cornerSet.has(u) || cornerSet.has(w)) ? Math.min(Math.trunc(0.35 * S), Math.floor(sm.length / 4)) : 0;
        const body = sm.length > 2 * trim + 4 ? sm.slice(trim, sm.length - trim) : sm.slice(1, sm.length - 1);
        const pts = [pa].concat(body, [pb]);
        let t1 = endDir(pts), t2 = endDir(pts.slice().reverse());
        if (pi > 0 && pieces[pi - 1][0] === 'l') t1 = unit(sub(pos(pieces[pi - 1][2]), pos(pieces[pi - 1][1])));
        if (pi + 1 < pieces.length && pieces[pi + 1][0] === 'l') t2 = mul(unit(sub(pos(pieces[pi + 1][2]), pos(pieces[pi + 1][1]))), -1);
        const arc = USE_ARCS ? tryArc(rawSeg) : null;
        if (arc) {
          ARCS++;
          const { c, r } = arc;
          const ea = add(c, mul(unit(sub(pa, c)), r)), eb = add(c, mul(unit(sub(pb, c)), r));
          const a0 = Math.atan2(ea[1] - c[1], ea[0] - c[0]), a1 = Math.atan2(eb[1] - c[1], eb[0] - c[0]);
          let sw = a1 - a0; const want = arc.a1 - arc.a0;
          while (sw - want > Math.PI) sw -= 2 * Math.PI;
          while (want - sw > Math.PI) sw += 2 * Math.PI;
          if (Cpos.has(u)) { const cu = Cpos.get(u); cu[0] = ea[0]; cu[1] = ea[1]; }
          if (norm(sub(ea, pa)) > 1e-6) out += 'L' + P2(ea);
          out += arcBeziers(c, r, a0, a0 + sw).map(bzStr).join('');
          if (Cpos.has(w)) { const cw = Cpos.get(w); cw[0] = eb[0]; cw[1] = eb[1]; }
          return;
        }
        out += emitCurve(rawSeg, smoothOpen(rawSeg, kk), pa, pb, t1, t2, pts);
      });
    });
    return out + 'Z';
  }

  function smoothLoop(P) {
    const n = P.length;
    const arc = USE_ARCS ? tryArc(P.concat([P[0]]), true) : null;
    if (arc) {
      const sw = (arc.a1 - arc.a0) > 0 ? 2 * Math.PI : -2 * Math.PI;
      const bzs = arcBeziers(arc.c, arc.r, 0, sw);
      return 'M' + P2(bzs[0][0]) + bzs.map(bzStr).join('') + 'Z';
    }
    let L = 0; for (let i = 0; i < n; i++) L += norm(sub(P[(i + 1) % n], P[i]));
    const k = Math.trunc(clip(L / 24, 0.2, 1.5) * S);
    const ps = smoothClosed(P, Math.max(1, k));
    const h = Math.floor(n / 2);
    const tang = (i) => unit(sub(ps[(i + 1) % n], ps[mod(i - 1, n)]));
    let s = 'M' + P2(ps[0]);
    for (const [a, b] of [[0, h], [h, n]]) {
      const seg = []; for (let i = a; i <= b; i++) seg.push(ps[i % n]);
      s += fit(seg, tang(a), mul(tang(b % n), -1), FIT_TOL).map(bzStr).join('');
    }
    return s + 'Z';
  }

  function pathArea(d) {
    const pts = []; let cur = null;
    const re = /([MLCZ])([^MLCZ]*)/g; let m;
    while ((m = re.exec(d))) {
      const nums = (m[2].match(/-?\d+(?:\.\d+)?(?:e-?\d+)?/g) || []).map(Number);
      if (m[1] === 'M' || m[1] === 'L') { cur = [nums[0], nums[1]]; pts.push(cur); }
      else if (m[1] === 'C') {
        const p = [cur, [nums[0], nums[1]], [nums[2], nums[3]], [nums[4], nums[5]]];
        for (let k = 1; k < 10; k++) pts.push(bez(p, k / 9));
        cur = p[3];
      }
    }
    if (pts.length < 3) return 0;
    let a = 0;
    for (let i = 0; i < pts.length; i++) { const p = pts[i], q = pts[(i + 1) % pts.length]; a += p[0] * q[1] - q[0] * p[1]; }
    return Math.abs(a / 2);
  }

  // ---------------------------------------------------------------- image processing
  function gaussKernel(sigma) {
    const k = (Math.round(sigma * 4 * 2 + 1)) | 1, h = (k - 1) / 2, w = new Float64Array(k);
    let s = 0; for (let i = 0; i < k; i++) { w[i] = Math.exp(-((i - h) ** 2) / (2 * sigma * sigma)); s += w[i]; }
    for (let i = 0; i < k; i++) w[i] /= s;
    return w;
  }
  const reflect101 = (i, n) => { if (n === 1) return 0; while (i < 0 || i >= n) { if (i < 0) i = -i; if (i >= n) i = 2 * n - 2 - i; } return i; };
  function blurFloat(src, W, H, sigma) {
    const w = gaussKernel(sigma), h = (w.length - 1) / 2, tmp = new Float32Array(W * H), out = new Float32Array(W * H);
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      let s = 0; for (let k = 0; k < w.length; k++) s += w[k] * src[y * W + reflect101(x + k - h, W)];
      tmp[y * W + x] = s;
    }
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      let s = 0; for (let k = 0; k < w.length; k++) s += w[k] * tmp[reflect101(y + k - h, H) * W + x];
      out[y * W + x] = s;
    }
    return out;
  }
  function sobel(src, W, H) {
    const gx = new Float32Array(W * H), gy = new Float32Array(W * H);
    const at = (x, y) => src[reflect101(y, H) * W + reflect101(x, W)];
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      gx[y * W + x] = (at(x + 1, y - 1) + 2 * at(x + 1, y) + at(x + 1, y + 1)) - (at(x - 1, y - 1) + 2 * at(x - 1, y) + at(x - 1, y + 1));
      gy[y * W + x] = (at(x - 1, y + 1) + 2 * at(x, y + 1) + at(x + 1, y + 1)) - (at(x - 1, y - 1) + 2 * at(x, y - 1) + at(x + 1, y - 1));
    }
    return [gx, gy];
  }
  function cubicW(x) {
    const A = -0.75;
    const c0 = ((A * (x + 1) - 5 * A) * (x + 1) + 8 * A) * (x + 1) - 4 * A;
    const c1 = ((A + 2) * x - (A + 3)) * x * x + 1;
    const c2 = ((A + 2) * (1 - x) - (A + 3)) * (1 - x) * (1 - x) + 1;
    return [c0, c1, c2, 1 - c0 - c1 - c2];
  }

  // Supersampled binary mask: bicubic upscale + coverage-accurate subpixels on clean edge pixels
  function coverageMask(t, W, H) {
    const BW = W * S, BH = H * S, mask = new Uint8Array(BW * BH);
    // x taps for the bicubic upscale
    const xi = new Int32Array(BW * 4), xw = new Float32Array(BW * 4);
    for (let dx = 0; dx < BW; dx++) {
      let fx = (dx + 0.5) / S - 0.5; const sx = Math.floor(fx); fx -= sx;
      const w = cubicW(fx);
      for (let k = 0; k < 4; k++) { xi[dx * 4 + k] = clip(sx - 1 + k, 0, W - 1); xw[dx * 4 + k] = w[k]; }
    }
    const rowCache = new Map();
    const hrow = (sy) => {
      if (rowCache.has(sy)) return rowCache.get(sy);
      const r = new Float32Array(BW), base = sy * W;
      for (let dx = 0; dx < BW; dx++) {
        const o = dx * 4;
        r[dx] = xw[o] * t[base + xi[o]] + xw[o + 1] * t[base + xi[o + 1]] + xw[o + 2] * t[base + xi[o + 2]] + xw[o + 3] * t[base + xi[o + 3]];
      }
      rowCache.set(sy, r);
      if (rowCache.size > 8) rowCache.delete(rowCache.keys().next().value);
      return r;
    };
    for (let dy = 0; dy < BH; dy++) {
      let fy = (dy + 0.5) / S - 0.5; const sy = Math.floor(fy); fy -= sy;
      const w = cubicW(fy);
      const r0 = hrow(clip(sy - 1, 0, H - 1)), r1 = hrow(clip(sy, 0, H - 1)), r2 = hrow(clip(sy + 1, 0, H - 1)), r3 = hrow(clip(sy + 2, 0, H - 1));
      const o = dy * BW;
      for (let dx = 0; dx < BW; dx++) mask[o + dx] = (w[0] * r0[dx] + w[1] * r1[dx] + w[2] * r2[dx] + w[3] * r3[dx]) > 0.5 ? 1 : 0;
    }
    // coverage-accurate subpixels
    const ts = blurFloat(t, W, H, 0.8);
    const [gx, gy] = sobel(ts, W, H);
    const T = (x, y) => t[clip(y, 0, H - 1) * W + clip(x, 0, W - 1)];
    const off = []; for (let j = 0; j < S; j++) off.push((j + 0.5) / S - 0.5);
    let seed = 12345;
    const rnd = () => { seed = (seed * 1103515245 + 12345) >>> 0; return seed / 4294967296; };
    const SS = S * S, proj = new Float64Array(SS), order = new Array(SS);
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      const tv = t[y * W + x];
      if (!(tv > 0.03 && tv < 0.97)) continue;
      const gnx = gx[y * W + x], gny = gy[y * W + x], gn = Math.hypot(gnx, gny) + 1e-9;
      if (!(gn > 0.05)) continue;
      const ux = Math.round(gnx / gn), uy = Math.round(gny / gn);
      if (!(T(x + ux, y + uy) > 0.85 && T(x - ux, y - uy) < 0.15 && T(x + 2 * ux, y + 2 * uy) > 0.85 && T(x - 2 * ux, y - 2 * uy) < 0.15)) continue;
      const nx = gnx / gn, ny = gny / gn;
      for (let j = 0; j < S; j++) for (let i = 0; i < S; i++) { proj[j * S + i] = nx * off[i] + ny * off[j] + 1e-4 * rnd(); order[j * S + i] = j * S + i; }
      order.sort((a, b) => proj[b] - proj[a]);
      const k = Math.round(tv * SS);
      for (let r = 0; r < SS; r++) {
        const s = order[r], si = s % S, sj = (s / S) | 0;
        mask[(y * S + sj) * BW + x * S + si] = r < k ? 1 : 0;
      }
    }
    return mask;
  }

  // Gaussian blur of a binary mask + threshold at 0.5, computed only near edges (fast, low memory)
  function blurThreshold(mask, BW, BH, sigma) {
    const w = gaussKernel(sigma), h = (w.length - 1) / 2;
    const tmp = new Uint8Array(BW * BH);
    // horizontal pass -> tmp (0..255)
    for (let y = 0; y < BH; y++) {
      const o = y * BW;
      for (let x = 0; x < BW; x++) tmp[o + x] = mask[o + x] * 255;
      let last = -1e9;
      const edges = [];
      for (let x = 1; x < BW; x++) if (mask[o + x] !== mask[o + x - 1]) edges.push(x);
      for (const e of edges) {
        const x0 = Math.max(Math.max(0, e - h - 1), last + 1), x1 = Math.min(BW - 1, e + h);
        for (let x = x0; x <= x1; x++) {
          let s = 0; for (let k = 0; k < w.length; k++) s += w[k] * mask[o + reflect101(x + k - h, BW)];
          tmp[o + x] = Math.round(s * 255);
        }
        last = Math.max(last, x1);
      }
    }
    // vertical pass -> mask (0/1)
    const col = new Uint8Array(BH), done = new Uint8Array(BH);
    for (let x = 0; x < BW; x++) {
      for (let y = 0; y < BH; y++) col[y] = tmp[y * BW + x];
      done.fill(0);
      for (let y = 0; y < BH; y++) mask[y * BW + x] = col[y] > 127 ? 1 : 0;
      for (let y = 1; y < BH; y++) {
        if (col[y] === col[y - 1]) continue;
        const y0 = Math.max(0, y - h - 1), y1 = Math.min(BH - 1, y + h);
        for (let yy = y0; yy <= y1; yy++) {
          if (done[yy]) continue; done[yy] = 1;
          let s = 0; for (let k = 0; k < w.length; k++) s += w[k] * col[reflect101(yy + k - h, BH)];
          mask[yy * BW + x] = s > 127.5 ? 1 : 0;
        }
      }
    }
  }

  function maxFilter(src, W, H, r) {
    const tmp = new Float32Array(W * H), out = new Float32Array(W * H);
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      let m = 0; const o = y * W;
      for (let k = Math.max(0, x - r); k <= Math.min(W - 1, x + r); k++) if (src[o + k] > m) m = src[o + k];
      tmp[o + x] = m;
    }
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      let m = 0;
      for (let k = Math.max(0, y - r); k <= Math.min(H - 1, y + r); k++) if (tmp[k * W + x] > m) m = tmp[k * W + x];
      out[y * W + x] = m;
    }
    return out;
  }

  // Binary morphology on the supersampled mask (square element, separable, O(n))
  function morph(mask, BW, BH, r, dilate) {
    const tmp = new Uint8Array(BW * BH), row = new Int32Array(Math.max(BW, BH) + 1);
    const pass = (src, dst, len, count, stride, lineStride) => {
      for (let l = 0; l < count; l++) {
        const base = l * lineStride;
        row[0] = 0;
        for (let k = 0; k < len; k++) row[k + 1] = row[k] + src[base + k * stride];
        for (let k = 0; k < len; k++) {
          const a = Math.max(0, k - r), b = Math.min(len, k + r + 1), sum = row[b] - row[a];
          dst[base + k * stride] = dilate ? (sum > 0 ? 1 : 0) : (sum === b - a ? 1 : 0);
        }
      }
    };
    pass(mask, tmp, BW, BH, 1, BW);
    pass(tmp, mask, BH, BW, BW, 1);
  }

  // Border following (Suzuki & Abe 1985) -> list of closed borders (outer + holes), pixel coordinates
  function findContours(mask, BW, BH) {
    // states: 0 bg, 1 fg unvisited, 2 visited (positive), 3 visited with right neighbour bg (negative)
    const dx = [1, 1, 0, -1, -1, -1, 0, 1], dy = [0, 1, 1, 1, 0, -1, -1, -1];   // E SE S SW W NW N NE (clockwise)
    const get = (x, y) => (x < 0 || y < 0 || x >= BW || y >= BH) ? 0 : mask[y * BW + x];
    const dirOf = (cx, cy, nx, ny) => { for (let d = 0; d < 8; d++) if (cx + dx[d] === nx && cy + dy[d] === ny) return d; return 0; };
    const contours = [];
    for (let y = 0; y < BH; y++) {
      for (let x = 0; x < BW; x++) {
        const v = mask[y * BW + x];
        if (v === 0) continue;
        let fx, fy, hole = false;
        if (v === 1 && get(x - 1, y) === 0) { fx = x - 1; fy = y; }
        else if ((v === 1 || v === 2) && get(x + 1, y) === 0) { fx = x + 1; fy = y; hole = true; }
        else continue;
        // 3.1 clockwise search from (fx, fy)
        const d0 = dirOf(x, y, fx, fy);
        let found = -1;
        for (let k = 0; k < 8; k++) { const d = (d0 + k) % 8; if (get(x + dx[d], y + dy[d]) !== 0) { found = d; break; } }
        if (found < 0) { mask[y * BW + x] = 3; contours.push([[x, y]]); continue; }
        const x1 = x + dx[found], y1 = y + dy[found];
        let x2 = x1, y2 = y1, x3 = x, y3 = y;
        const pts = [];
        for (let guard = 0; guard < 4 * BW * BH; guard++) {
          pts.push([x3, y3]);
          const d2 = dirOf(x3, y3, x2, y2);
          let d4 = -1, eastZero = false;
          for (let k = 1; k <= 8; k++) {                  // 3.3 counter-clockwise search
            const d = (d2 - k + 16) % 8;
            if (get(x3 + dx[d], y3 + dy[d]) !== 0) { d4 = d; break; }
            if (d === 0) eastZero = true;
          }
          const i3 = y3 * BW + x3;
          if (eastZero) mask[i3] = 3; else if (mask[i3] === 1) mask[i3] = 2;
          const x4 = x3 + dx[d4], y4 = y3 + dy[d4];
          if (x4 === x && y4 === y && x3 === x1 && y3 === y1) break;
          x2 = x3; y2 = y3; x3 = x4; y3 = y4;
        }
        pts.hole = hole;
        contours.push(pts);
      }
    }
    return contours;
  }

  function median(arr) {
    if (!arr.length) return 0;
    const a = Float64Array.from(arr).sort(), n = a.length;
    return n % 2 ? a[(n - 1) / 2] : (a[n / 2 - 1] + a[n / 2]) / 2;
  }
  const hex = (c) => '#' + c.map((v) => Math.round(v).toString(16).padStart(2, '0')).join('');

  // ---------------------------------------------------------------- palette (multi-colour)
  // k-means on "smooth" pixels (antialiased edge pixels are ignored). With colours = 'auto' the
  // number of colours grows until almost every smooth pixel is close to a palette colour.
  function choosePalette(rgb, W, H, colors) {
    const N = W * H, smooth = [];
    for (let y = 1; y < H - 1; y++) for (let x = 1; x < W - 1; x++) {
      const i = y * W + x; let dmax = 0;
      for (const j of [i - 1, i + 1, i - W, i + W, i - W - 1, i - W + 1, i + W - 1, i + W + 1]) {
        const d = Math.abs(rgb[i * 3] - rgb[j * 3]) + Math.abs(rgb[i * 3 + 1] - rgb[j * 3 + 1]) + Math.abs(rgb[i * 3 + 2] - rgb[j * 3 + 2]);
        if (d > dmax) dmax = d;
      }
      if (dmax <= 48) smooth.push(i);
    }
    if (smooth.length < 50) for (let i = 0; i < N; i++) smooth.push(i);
    // deterministic sample
    const MAXS = 24000, step = Math.max(1, smooth.length / MAXS), n = Math.min(MAXS, smooth.length);
    const X = new Float32Array(n * 3);
    for (let k = 0; k < n; k++) { const i = smooth[Math.floor(k * step)]; X[k * 3] = rgb[i * 3]; X[k * 3 + 1] = rgb[i * 3 + 1]; X[k * 3 + 2] = rgb[i * 3 + 2]; }
    let seed = 7;
    const rnd = () => { seed = (seed * 1103515245 + 12345) >>> 0; return seed / 4294967296; };
    const lab = new Int32Array(n), dist = new Float32Array(n);
    function kmeans(k) {
      // k-means++ init
      const C = [[X[0], X[1], X[2]]];
      const d2 = new Float64Array(n).fill(Infinity);
      while (C.length < k) {
        const c = C[C.length - 1]; let sum = 0;
        for (let p = 0; p < n; p++) { const d = (X[p * 3] - c[0]) ** 2 + (X[p * 3 + 1] - c[1]) ** 2 + (X[p * 3 + 2] - c[2]) ** 2; if (d < d2[p]) d2[p] = d; sum += d2[p]; }
        if (sum <= 0) break;
        let r = rnd() * sum, p = 0;
        for (; p < n - 1; p++) { r -= d2[p]; if (r <= 0) break; }
        C.push([X[p * 3], X[p * 3 + 1], X[p * 3 + 2]]);
      }
      for (let it = 0; it < 20; it++) {
        const acc = C.map(() => [0, 0, 0, 0]); let moved = 0;
        for (let p = 0; p < n; p++) {
          let best = 0, bd = Infinity;
          for (let c = 0; c < C.length; c++) { const d = (X[p * 3] - C[c][0]) ** 2 + (X[p * 3 + 1] - C[c][1]) ** 2 + (X[p * 3 + 2] - C[c][2]) ** 2; if (d < bd) { bd = d; best = c; } }
          if (lab[p] !== best) moved++;
          lab[p] = best; dist[p] = Math.sqrt(bd);
          const a = acc[best]; a[0]++; a[1] += X[p * 3]; a[2] += X[p * 3 + 1]; a[3] += X[p * 3 + 2];
        }
        acc.forEach((a, c) => { if (a[0]) C[c] = [a[1] / a[0], a[2] / a[0], a[3] / a[0]]; });
        if (it > 0 && moved < n * 0.001) break;
      }
      const cnt = new Int32Array(C.length); for (let p = 0; p < n; p++) cnt[lab[p]]++;
      return C.filter((_, c) => cnt[c] > Math.max(3, n * 0.0005));
    }
    const outliers = () => { let o = 0; for (let p = 0; p < n; p++) if (dist[p] > 30) o++; return o / n; };
    let C;
    if (colors === 'auto' || !colors) {
      for (let k = 2; k <= 20; k++) { C = kmeans(k); if (outliers() <= 0.0025) break; }
    } else {
      C = kmeans(Math.max(2, Math.min(20, +colors + 1)));   // +1: the background
    }
    // rare but clearly distinct colours (thin strokes): pixels that are not a mix of two palette colours
    if (colors === 'auto' || !colors) {
      const stepAll = Math.max(1, Math.floor(N / 80000));
      let far = [];
      const mixErr = (r, g, b, pal) => {
        let best = Infinity;
        for (let u = 0; u < pal.length; u++) for (let v = u; v < pal.length; v++) {
          const A = pal[u], B = pal[v], d0 = A[0] - B[0], d1 = A[1] - B[1], d2 = A[2] - B[2], dd = d0 * d0 + d1 * d1 + d2 * d2;
          const vx = r - B[0], vy = g - B[1], vz = b - B[2];
          const tt = dd > 0 ? clip((vx * d0 + vy * d1 + vz * d2) / dd, 0, 1) : 0;
          const ex = vx - tt * d0, ey = vy - tt * d1, ez = vz - tt * d2;
          // hue errors count double: a green can't be explained by mixing pinks, golds and black
          const gm = (ex + ey + ez) / 3, cx = ex - gm, cy = ey - gm, cz = ez - gm;
          const e = 3 * gm * gm + 4 * (cx * cx + cy * cy + cz * cz);
          if (e < best) best = e;
        }
        return Math.sqrt(best);
      };
      for (let i = 0; i < N; i += stepAll) {
        const r = rgb[i * 3], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
        if (mixErr(r, g, b, C) > 28) far.push([r, g, b]);
      }
      const minN = Math.max(12, (N / stepAll) * 0.0004);
      for (let round = 0; round < 4 && far.length >= minN; round++) {
        let seedIdx = 0, bestN = -1;
        for (let a2 = 0; a2 < far.length; a2 += Math.max(1, Math.floor(far.length / 300))) {
          let cnt = 0; for (const q of far) if (Math.hypot(q[0] - far[a2][0], q[1] - far[a2][1], q[2] - far[a2][2]) < 28) cnt++;
          if (cnt > bestN) { bestN = cnt; seedIdx = a2; }
        }
        if (bestN < minN) break;
        const sd = far[seedIdx], m = [0, 0, 0]; let c = 0;
        for (const q of far) if (Math.hypot(q[0] - sd[0], q[1] - sd[1], q[2] - sd[2]) < 28) { m[0] += q[0]; m[1] += q[1]; m[2] += q[2]; c++; }
        const nc = [m[0] / c, m[1] / c, m[2] / c];
        C.push(nc);
        far = far.filter((q) => mixErr(q[0], q[1], q[2], [nc].concat(C)) > 28);
      }
    }
    // small flat areas of their own colour (e.g. light dashes): real colours even if rare and even if
    // their colour looks like a mix of two others. Only flat pixels count (edges are blends).
    if (colors === 'auto' || !colors) {
      const nearest = (r, g, b) => { let m = Infinity; for (const c of C) m = Math.min(m, Math.hypot(r - c[0], g - c[1], b - c[2])); return m; };
      let pool = [];
      for (let y = 1; y < H - 1; y++) for (let x = 1; x < W - 1; x++) {
        const i = y * W + x; let dmax = 0;
        for (const j of [i - 1, i + 1, i - W, i + W]) {
          const d = Math.abs(rgb[i * 3] - rgb[j * 3]) + Math.abs(rgb[i * 3 + 1] - rgb[j * 3 + 1]) + Math.abs(rgb[i * 3 + 2] - rgb[j * 3 + 2]);
          if (d > dmax) dmax = d;
        }
        if (dmax > 20) continue;
        const r = rgb[i * 3], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
        if (nearest(r, g, b) > 32) pool.push([r, g, b]);
      }
      const minFlat = Math.max(40, N * 0.00012);
      for (let round = 0; round < 6 && pool.length >= minFlat && C.length < 20; round++) {
        let seed = null, bestN = -1;
        const stp = Math.max(1, Math.floor(pool.length / 400));
        for (let a = 0; a < pool.length; a += stp) {
          let cnt = 0; for (let q = 0; q < pool.length; q += stp) if (Math.hypot(pool[q][0] - pool[a][0], pool[q][1] - pool[a][1], pool[q][2] - pool[a][2]) < 22) cnt++;
          if (cnt > bestN) { bestN = cnt; seed = pool[a]; }
        }
        const m = [0, 0, 0]; let c = 0;
        for (const q of pool) if (Math.hypot(q[0] - seed[0], q[1] - seed[1], q[2] - seed[2]) < 22) { m[0] += q[0]; m[1] += q[1]; m[2] += q[2]; c++; }
        if (c < minFlat) break;
        const nc = [m[0] / c, m[1] / c, m[2] / c];
        C.push(nc);
        pool = pool.filter((q) => Math.hypot(q[0] - nc[0], q[1] - nc[1], q[2] - nc[2]) > 32 && nearest(q[0], q[1], q[2]) > 32);
      }
    }
    // merge near-identical colours
    let out = [];
    for (const c of C) if (!out.some((o) => Math.hypot(o[0] - c[0], o[1] - c[1], o[2] - c[2]) < 18)) out.push(c);
    if (colors === 'auto' || !colors) out = mergeInterleaved(rgb, W, H, out);
    return out;
  }

  // Two close colours whose pixels are interleaved (noise / compression) are one colour.
  // Real colours (even close shades of a gradient) occupy separate, compact areas.
  function mergeInterleaved(rgb, W, H, pal) {
    const N = W * H;
    for (let round = 0; round < 12 && pal.length > 2; round++) {
      const K = pal.length, lbl = new Uint8Array(N), area = new Float64Array(K);
      for (let i = 0; i < N; i++) {
        let best = 0, bd = Infinity;
        for (let c = 0; c < K; c++) { const d = (rgb[i * 3] - pal[c][0]) ** 2 + (rgb[i * 3 + 1] - pal[c][1]) ** 2 + (rgb[i * 3 + 2] - pal[c][2]) ** 2; if (d < bd) { bd = d; best = c; } }
        lbl[i] = best; area[best]++;
      }
      const bnd = new Float64Array(K * K);
      for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
        const i = y * W + x, a = lbl[i];
        if (x + 1 < W) { const b = lbl[i + 1]; if (a !== b) { bnd[a * K + b]++; bnd[b * K + a]++; } }
        if (y + 1 < H) { const b = lbl[i + W]; if (a !== b) { bnd[a * K + b]++; bnd[b * K + a]++; } }
      }
      let best = null;
      for (let a = 0; a < K; a++) for (let b = a + 1; b < K; b++) {
        const d = Math.hypot(pal[a][0] - pal[b][0], pal[a][1] - pal[b][1], pal[a][2] - pal[b][2]);
        if (d > 45) continue;
        const small = Math.min(area[a], area[b]);
        if (small < 1) { best = { a, b, score: 99 }; break; }
        // shared border relative to the smaller colour's area: a compact region has a small ratio
        const ratio = bnd[a * K + b] / small;
        const score = ratio * (1 - d / 60);
        if (ratio > 1.2 && (!best || score > best.score)) best = { a, b, score };
      }
      if (!best) break;
      const { a, b } = best, wa = area[a], wb = area[b], wt = Math.max(1, wa + wb);
      const merged = [0, 1, 2].map((k) => (pal[a][k] * wa + pal[b][k] * wb) / wt);
      pal = pal.filter((_, i) => i !== a && i !== b).concat([merged]);
    }
    return pal;
  }

  // Each pixel = mix of two palette colours: returns colour indices + mixing weight
  function decompose(rgb, N, pal) {
    const K = pal.length, ia = new Uint8Array(N), ib = new Uint8Array(N), al = new Float32Array(N);
    const pairs = [];
    for (let a = 0; a < K; a++) for (let b = a + 1; b < K; b++) {
      const d = [pal[a][0] - pal[b][0], pal[a][1] - pal[b][1], pal[a][2] - pal[b][2]];
      pairs.push([a, b, d, d[0] * d[0] + d[1] * d[1] + d[2] * d[2]]);
    }
    for (let i = 0; i < N; i++) {
      const r = rgb[i * 3], g = rgb[i * 3 + 1], bl = rgb[i * 3 + 2];
      let best = Infinity;
      for (const [a, b, d, dd] of pairs) {
        const pb = pal[b], vx = r - pb[0], vy = g - pb[1], vz = bl - pb[2];
        const t = dd > 0 ? clip((vx * d[0] + vy * d[1] + vz * d[2]) / dd, 0, 1) : 0;
        const ex = vx - t * d[0], ey = vy - t * d[1], ez = vz - t * d[2], e = ex * ex + ey * ey + ez * ez;
        if (e < best) { best = e; ia[i] = a; ib[i] = b; al[i] = t; }
      }
    }
    return { ia, ib, al };
  }

  // Colour patches smaller than minSize pixels (slivers where 3 colours meet) take the colour around them
  function absorbSpecks(lbl, W, H, K, minSize) {
    const N = W * H, comp = new Int32Array(N).fill(-1), stack = new Int32Array(N), members = [];
    const nb = new Int32Array(K);
    for (let s0 = 0; s0 < N; s0++) {
      if (comp[s0] >= 0) continue;
      const c = lbl[s0]; let sp = 0, cnt = 0; stack[sp++] = s0; comp[s0] = s0; members.length = 0;
      let big = false;
      while (sp) {
        const i = stack[--sp]; if (!big) members.push(i); cnt++;
        if (cnt >= minSize) big = true;
        const x = i % W, y = (i / W) | 0;
        if (x > 0 && comp[i - 1] < 0 && lbl[i - 1] === c) { comp[i - 1] = s0; stack[sp++] = i - 1; }
        if (x < W - 1 && comp[i + 1] < 0 && lbl[i + 1] === c) { comp[i + 1] = s0; stack[sp++] = i + 1; }
        if (y > 0 && comp[i - W] < 0 && lbl[i - W] === c) { comp[i - W] = s0; stack[sp++] = i - W; }
        if (y < H - 1 && comp[i + W] < 0 && lbl[i + W] === c) { comp[i + W] = s0; stack[sp++] = i + W; }
      }
      if (big) continue;
      nb.fill(0);
      for (const i of members) {
        const x = i % W, y = (i / W) | 0;
        if (x > 0 && lbl[i - 1] !== c) nb[lbl[i - 1]]++;
        if (x < W - 1 && lbl[i + 1] !== c) nb[lbl[i + 1]]++;
        if (y > 0 && lbl[i - W] !== c) nb[lbl[i - W]]++;
        if (y < H - 1 && lbl[i + W] !== c) nb[lbl[i + W]]++;
      }
      let m = -1, mv = 0; for (let k = 0; k < K; k++) if (nb[k] > mv) { mv = nb[k]; m = k; }
      if (m >= 0) for (const i of members) lbl[i] = m;
    }
    return lbl;
  }

  // Like decompose(), but each pixel may only mix colours present around it (5x5, after denoising):
  // prevents fringes of a third colour along edges and noise specks inside light areas.
  function decomposeLocal(rgb, W, H, pal) {
    const N = W * H, K = pal.length;
    let lbl = new Uint8Array(N);
    for (let i = 0; i < N; i++) {
      let best = 0, bd = Infinity;
      for (let c = 0; c < K; c++) { const d = (rgb[i * 3] - pal[c][0]) ** 2 + (rgb[i * 3 + 1] - pal[c][1]) ** 2 + (rgb[i * 3 + 2] - pal[c][2]) ** 2; if (d < bd) { bd = d; best = c; } }
      lbl[i] = best;
    }
    const cnt = new Int32Array(K);
    for (let pass = 0; pass < 2; pass++) {
      const out = lbl.slice();
      for (let y = 1; y < H - 1; y++) for (let x = 1; x < W - 1; x++) {
        const i = y * W + x; cnt.fill(0);
        for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) cnt[lbl[i + dy * W + dx]]++;
        let m = lbl[i]; for (let c = 0; c < K; c++) if (cnt[c] > cnt[m]) m = c;
        if (cnt[m] >= 5) out[i] = m;
      }
      lbl = out;
    }
    lbl = absorbSpecks(lbl, W, H, K, 8);
    const ia = new Uint8Array(N), ib = new Uint8Array(N), al = new Float32Array(N);
    const present = new Uint8Array(K);
    // "flat" pixels (not on an edge): only their colours are real candidates. Blended edge pixels
    // can look like a third colour (e.g. white+red looks peach) and must not add it.
    const flat = new Uint8Array(N);
    for (let y = 1; y < H - 1; y++) for (let x = 1; x < W - 1; x++) {
      const i = y * W + x; let dmax = 0;
      for (const j of [i - 1, i + 1, i - W, i + W]) {
        const d = Math.abs(rgb[i * 3] - rgb[j * 3]) + Math.abs(rgb[i * 3 + 1] - rgb[j * 3 + 1]) + Math.abs(rgb[i * 3 + 2] - rgb[j * 3 + 2]);
        if (d > dmax) dmax = d;
      }
      flat[i] = dmax <= 30 ? 1 : 0;
    }
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      const i = y * W + x;
      present.fill(0);
      let nflat = 0;
      for (let dy = -3; dy <= 3; dy++) { const yy = y + dy; if (yy < 0 || yy >= H) continue;
        for (let dx = -3; dx <= 3; dx++) { const xx = x + dx; if (xx < 0 || xx >= W) continue;
          const j = yy * W + xx; if (flat[j]) { present[lbl[j]] = 1; nflat++; } } }
      if (!nflat) {
        for (let dy = -2; dy <= 2; dy++) { const yy = y + dy; if (yy < 0 || yy >= H) continue;
          for (let dx = -2; dx <= 2; dx++) { const xx = x + dx; if (xx < 0 || xx >= W) continue; present[lbl[yy * W + xx]] = 1; } }
      }
      // the pixel must be explainable by the candidates; otherwise add its own colour and neighbours'
      {
        const r = rgb[i * 3], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
        let e = Infinity;
        const cl0 = []; for (let c = 0; c < K; c++) if (present[c]) cl0.push(c);
        for (let u = 0; u < cl0.length; u++) for (let v = u; v < cl0.length; v++) {
          const A = pal[cl0[u]], B = pal[cl0[v]], d0 = A[0] - B[0], d1 = A[1] - B[1], d2 = A[2] - B[2], dd = d0 * d0 + d1 * d1 + d2 * d2;
          const vx = r - B[0], vy = g - B[1], vz = b - B[2];
          const t = dd > 0 ? clip((vx * d0 + vy * d1 + vz * d2) / dd, 0, 1) : 1;
          e = Math.min(e, (vx - t * d0) ** 2 + (vy - t * d1) ** 2 + (vz - t * d2) ** 2);
        }
        if (e > 30 * 30) {
          for (let dy = -2; dy <= 2; dy++) { const yy = y + dy; if (yy < 0 || yy >= H) continue;
            for (let dx = -2; dx <= 2; dx++) { const xx = x + dx; if (xx < 0 || xx >= W) continue; present[lbl[yy * W + xx]] = 1; } }
        }
      }
      const cl = []; for (let c = 0; c < K; c++) if (present[c]) cl.push(c);
      if (cl.length === 1) { ia[i] = ib[i] = cl[0]; al[i] = 1; continue; }
      const r = rgb[i * 3], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
      let best = Infinity;
      for (let u = 0; u < cl.length; u++) for (let v = u + 1; v < cl.length; v++) {
        const A = pal[cl[u]], B = pal[cl[v]], d0 = A[0] - B[0], d1 = A[1] - B[1], d2 = A[2] - B[2], dd = d0 * d0 + d1 * d1 + d2 * d2;
        const vx = r - B[0], vy = g - B[1], vz = b - B[2];
        const t = dd > 0 ? clip((vx * d0 + vy * d1 + vz * d2) / dd, 0, 1) : 1;
        const e = (vx - t * d0) ** 2 + (vy - t * d1) ** 2 + (vz - t * d2) ** 2;
        if (e < best) { best = e; ia[i] = cl[u]; ib[i] = cl[v]; al[i] = t; }
      }
      // pixels deep inside a denoised area keep that colour (noise in light areas)
      if (flat[i] && cl.length === 2 && lbl[i] !== ia[i] && lbl[i] !== ib[i] && present[lbl[i]]) { ia[i] = ib[i] = lbl[i]; al[i] = 1; }
    }
    return { ia, ib, al };
  }

  // Traces one coverage map t (0 = outside, 1 = inside) into SVG path data
  function traceLayer(t, W, H, progress, label, minArea, fillGaps, cleanR = 0) {
    CUR_T = t; CUR_W = W; CUR_H = H;
    const BW = W * S, BH = H * S;
    const mask = coverageMask(t, W, H);
    blurThreshold(mask, BW, BH, S * MASK_BLUR);
    if (cleanR > 0) {                 // remove burrs (opening) and nicks (closing) < ~2*cleanR subpixels
      const r = cleanR;
      for (let i = 0; i < mask.length; i++) mask[i] = mask[i] ? 1 : 0;
      morph(mask, BW, BH, r, false); morph(mask, BW, BH, r, true);
      morph(mask, BW, BH, r, true); morph(mask, BW, BH, r, false);
    }
    const contours = findContours(mask, BW, BH).filter((c) => c.length >= 3);
    const paths = [];
    let done = 0;
    for (const c of contours) {
      done++;
      let a2 = 0;
      for (let i = 0; i < c.length; i++) { const p = c[i], q = c[(i + 1) % c.length]; a2 += p[0] * q[1] - q[0] * p[1]; }
      const areaPx = Math.abs(a2 / 2);
      if (areaPx < S * S * minArea) continue;
      if (fillGaps && c.hole) {
        // gap filler: tiny or very thin background slivers between colours (JPEG halos)
        let per = 0;
        for (let i = 0; i < c.length; i++) { const p = c[i], q = c[(i + 1) % c.length]; per += Math.hypot(p[0] - q[0], p[1] - q[1]); }
        const aPx = areaPx / S / S, width = 2 * aPx / Math.max(1e-9, per / S);
        if (aPx < 6 || (aPx < 60 && width < 1.3)) continue;
      }
      const a0 = areaPx / S / S;
      const raw = c.map((p) => [(p[0] + 0.5) / S, (p[1] + 0.5) / S]);
      let d = null;
      try { d = trace(raw); } catch (e) { d = null; }
      if (d === null || Math.abs(pathArea(d) - a0) / a0 > 0.05) {
        USE_ARCS = false;
        let d2 = null;
        try { d2 = trace(raw); } catch (e) { d2 = null; } finally { USE_ARCS = true; }
        if (d2 !== null && (d === null || Math.abs(pathArea(d2) - a0) < Math.abs(pathArea(d) - a0))) d = d2;
      }
      if (d) paths.push(d);
      if (done % 10 === 0) progress(`${label}${Math.round(100 * done / contours.length)} %`);
    }
    return paths;
  }

  // ---------------------------------------------------------------- main entry
  function vectorizeRGBA(rgba, W, H, onProgress, opts) {
    const t0 = Date.now();
    const progress = (msg) => { if (onProgress) onProgress(msg); };
    const colorsOpt = (opts && opts.colors) || 'auto';
    const styleOpt = (opts && opts.style) || ENV('VSTYLE') || 'clean';
    const N = W * H;
    // background colour from the border (white for transparent images)
    const bord = [[], [], []];
    const pushB = (x, y) => { const o = (y * W + x) * 4; for (let c = 0; c < 3; c++) bord[c].push(rgba[o + c]); };
    for (let x = 0; x < W; x++) { pushB(x, 0); pushB(x, H - 1); }
    for (let y = 0; y < H; y++) { pushB(0, y); pushB(W - 1, y); }
    let bgc = [median(bord[0]), median(bord[1]), median(bord[2])];
    let amean = 0; for (let i = 0; i < N; i++) amean += rgba[i * 4 + 3];
    amean /= N * 255;
    const topBottomAlpha = []; for (let x = 0; x < W; x++) { topBottomAlpha.push(rgba[x * 4 + 3], rgba[((H - 1) * W + x) * 4 + 3]); }
    if (amean < 0.999 && median(topBottomAlpha) < 128) bgc = [255, 255, 255];
    const rgb = new Float32Array(N * 3);
    for (let i = 0; i < N; i++) {
      const a = rgba[i * 4 + 3] / 255;
      for (let c = 0; c < 3; c++) rgb[i * 3 + c] = rgba[i * 4 + c] * a + bgc[c] * (1 - a);
    }
    progress('Détection des couleurs…');
    let pal = choosePalette(rgb, W, H, colorsOpt);
    // put the background colour first (closest palette colour to the border colour)
    let bi = 0, bd = Infinity;
    pal.forEach((c, i) => { const d = Math.hypot(c[0] - bgc[0], c[1] - bgc[1], c[2] - bgc[2]); if (d < bd) { bd = d; bi = i; } });
    if (bd > 40) { pal.unshift(bgc.slice()); bi = 0; }
    pal = [pal[bi]].concat(pal.filter((_, i) => i !== bi));
    if (pal.length < 2) throw new Error("L'image ne contient pas de logo visible (une seule couleur).");
    const K0 = pal.length;
    const illus = K0 > 5;                         // many colours: gradients / illustration
    setProfile(illus, styleOpt);
    let ia, ib, al, tsil = null;
    if (illus) {
      progress('Simplification des couleurs…');
      let lbl = new Uint8Array(N);
      const smoothPx = new Uint8Array(N);
      for (let i = 0; i < N; i++) {
        let best = 0, bd = Infinity;
        for (let c = 0; c < K0; c++) { const d = (rgb[i * 3] - pal[c][0]) ** 2 + (rgb[i * 3 + 1] - pal[c][1]) ** 2 + (rgb[i * 3 + 2] - pal[c][2]) ** 2; if (d < bd) { bd = d; best = c; } }
        lbl[i] = best;
      }
      for (let y = 1; y < H - 1; y++) for (let x = 1; x < W - 1; x++) {
        const i = y * W + x; let dmax = 0;
        for (const j of [i - 1, i + 1, i - W, i + W]) {
          const d = Math.abs(rgb[i * 3] - rgb[j * 3]) + Math.abs(rgb[i * 3 + 1] - rgb[j * 3 + 1]) + Math.abs(rgb[i * 3 + 2] - rgb[j * 3 + 2]);
          if (d > dmax) dmax = d;
        }
        smoothPx[i] = dmax <= 30 ? 1 : 0;
      }
      // edge (blended) pixels: explain the pixel as a mix of two colours found among nearby smooth pixels
      const fixed = lbl.slice();
      const edgeA = new Uint8Array(N), edgeB = new Uint8Array(N), edgeT = new Float32Array(N), isEdge = new Uint8Array(N);
      for (let y = 2; y < H - 2; y++) for (let x = 2; x < W - 2; x++) {
        const i = y * W + x;
        if (smoothPx[i]) continue;
        const cand = new Set();
        for (let dy = -2; dy <= 2; dy++) for (let dx = -2; dx <= 2; dx++) { const j = i + dy * W + dx; if (smoothPx[j]) cand.add(lbl[j]); }
        cand.add(0);                                   // the background is always a candidate
        const cl = [...cand], r = rgb[i * 3], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
        let bestE = Infinity, bestL = lbl[i], bu = lbl[i], bv = lbl[i], bt = 1;
        for (let u = 0; u < cl.length; u++) for (let v = u; v < cl.length; v++) {
          const A = pal[cl[u]], B = pal[cl[v]], d = [A[0] - B[0], A[1] - B[1], A[2] - B[2]], dd = d[0] ** 2 + d[1] ** 2 + d[2] ** 2;
          const vx = r - B[0], vy = g - B[1], vz = b - B[2];
          const tt = dd > 0 ? clip((vx * d[0] + vy * d[1] + vz * d[2]) / dd, 0, 1) : 1;
          const e = (vx - tt * d[0]) ** 2 + (vy - tt * d[1]) ** 2 + (vz - tt * d[2]) ** 2;
          if (e < bestE) { bestE = e; bestL = tt >= 0.5 ? cl[u] : cl[v]; bu = cl[u]; bv = cl[v]; bt = tt; }
        }
        fixed[i] = bestL;
        edgeA[i] = bu; edgeB[i] = bv; edgeT[i] = bt; isEdge[i] = 1;
      }
      lbl = fixed;
      // 3x3 majority filter (twice) removes isolated pixels and speckles
      const cnt = new Int32Array(K0);
      for (let pass = 0; pass < 2; pass++) {
        const out = lbl.slice();
        for (let y = 1; y < H - 1; y++) for (let x = 1; x < W - 1; x++) {
          const i = y * W + x; cnt.fill(0);
          for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) cnt[lbl[i + dy * W + dx]]++;
          let m = lbl[i]; for (let c = 0; c < K0; c++) if (cnt[c] > cnt[m]) m = c;
          if (cnt[m] >= 5) out[i] = m;
        }
        lbl = out;
      }
      // silhouette (logo vs background) from antialiasing: pixel = mix(background, one logo colour)
      tsil = new Float32Array(N);
      const B0 = pal[0];
      for (let i = 0; i < N; i++) {
        const vx = rgb[i * 3] - B0[0], vy = rgb[i * 3 + 1] - B0[1], vz = rgb[i * 3 + 2] - B0[2];
        let best = Infinity, ba = 0, bcol = 1;
        for (let c = 1; c < K0; c++) {
          const d0 = pal[c][0] - B0[0], d1 = pal[c][1] - B0[1], d2 = pal[c][2] - B0[2], dd = d0 * d0 + d1 * d1 + d2 * d2;
          const tt = dd > 0 ? clip((vx * d0 + vy * d1 + vz * d2) / dd, 0, 1) : 0;
          const e = (vx - tt * d0) ** 2 + (vy - tt * d1) ** 2 + (vz - tt * d2) ** 2;
          if (e < best) { best = e; ba = tt; bcol = c; }
        }
        tsil[i] = ba;
        // pixels inside the silhouette must carry a logo colour (otherwise the bottom layer shows through)
        if (lbl[i] === 0 && ba > 0.1) lbl[i] = bcol;
      }
      ia = lbl.slice(); ib = lbl.slice(); al = new Float32Array(N).fill(1);
      for (let i = 0; i < N; i++) if (isEdge[i] && edgeA[i] !== edgeB[i]) {
        // keep the soft antialiasing only where the majority filter agreed with one of the two colours
        if (lbl[i] === edgeA[i] || lbl[i] === edgeB[i]) { ia[i] = edgeA[i]; ib[i] = edgeB[i]; al[i] = edgeT[i]; }
      }
    } else {
      ({ ia, ib, al } = K0 > 2 ? decomposeLocal(rgb, W, H, pal) : decompose(rgb, N, pal));
    }
    // stacking order: largest colours at the bottom, smaller details on top
    const tsilSmooth = illus ? blurFloat(tsil, W, H, 0.6) : null;
    const K = pal.length, weight = new Float64Array(K);
    for (let i = 0; i < N; i++) { weight[ia[i]] += al[i]; weight[ib[i]] += 1 - al[i]; }
    let order = [...Array(K).keys()].slice(1).sort((p, q) => weight[q] - weight[p]);
    let minGap = Infinity;
    for (let a = 1; a < K; a++) for (let b = a + 1; b < K; b++)
      minGap = Math.min(minGap, Math.hypot(pal[a][0] - pal[b][0], pal[a][1] - pal[b][1], pal[a][2] - pal[b][2]));
    // clearly distinct colours (not shades of one gradient): own-colour layers, thin strokes on top
    const flatStack = !illus && K > 2 && minGap > 70;
    if (flatStack) {
      // flat colours: thick fills at the bottom, thin strokes on top (their own edges stay visible)
      const area = new Float64Array(K), per = new Float64Array(K);
      const hard = (i) => (al[i] >= 0.5 ? ia[i] : ib[i]);
      for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
        const i = y * W + x, c = hard(i); area[c]++;
        if (x + 1 < W) { const d = hard(i + 1); if (d !== c) { per[c]++; per[d]++; } }
        if (y + 1 < H) { const d = hard(i + W); if (d !== c) { per[c]++; per[d]++; } }
      }
      const thick = (c) => area[c] / Math.max(1, per[c]);
      order = order.slice().sort((p, q) => thick(q) - thick(p));
    }
    // supersampling factor under the memory guard
    S = pal.length > 7 ? 6 : 8;                // many colours: slightly lower supersampling (speed)
    while (S > 3 && N * S * S > MAX_SUBPIXELS) S--;
    ARCS = 0; USE_ARCS = true;
    const layers = [];
    const rank = new Int32Array(K).fill(-1);
    order.forEach((c, r) => { rank[c] = r; });
    const t = new Float32Array(N);
    for (let L = 0; L < order.length; L++) {
      // layer L covers its own colour and every colour stacked above it (no gaps between colours)
      for (let i = 0; i < N; i++) t[i] = al[i] * (rank[ia[i]] >= L ? 1 : 0) + (1 - al[i]) * (rank[ib[i]] >= L ? 1 : 0);
      if (illus) {
        if (L === 0) t.set(tsilSmooth);
        else { const tb = blurFloat(t, W, H, 0.9); for (let i = 0; i < N; i++) t[i] = Math.min(tb[i], tsilSmooth[i]); }
      } else if (K0 > 2) {
        if (!flatStack && LAYER_BLUR > 0) t.set(blurFloat(t, W, H, LAYER_BLUR));     // removes burrs from colour noise
        if (flatStack) {
          // own colour only, then gap filler: extend by <= 2 px into upper colours / the halo next to them
          const own = new Float32Array(N), up = new Float32Array(N);
          for (let i = 0; i < N; i++) {
            own[i] = al[i] * (rank[ia[i]] === L ? 1 : 0) + (1 - al[i]) * (rank[ib[i]] === L ? 1 : 0);
            up[i] = al[i] * (rank[ia[i]] > L ? 1 : 0) + (1 - al[i]) * (rank[ib[i]] > L ? 1 : 0);
          }
          const ob = LAYER_BLUR > 0 ? blurFloat(own, W, H, LAYER_BLUR) : own;
          if (L < order.length - 1) {
            const mine = maxFilter(ob, W, H, 2), near = L === 0 ? maxFilter(up, W, H, 1) : up;
            for (let i = 0; i < N; i++) t[i] = Math.max(ob[i], Math.min(mine[i], near[i]));
          } else t.set(ob);
        }
      }
      const label = order.length > 1 ? `Couleur ${L + 1}/${order.length} — ` : 'Tracé des courbes… ';
      progress(label);
      const paths = traceLayer(t, W, H, progress, label, illus ? 4 : 0.8, L === 0 && order.length > 1, K0 > 2 && !illus ? Math.round(S * CLEAN_PX) : 0);
      if (paths.length) layers.push({ color: hex(pal[order[L]]), paths });
    }
    if (!layers.length) throw new Error("L'image ne contient pas de logo visible (une seule couleur).");
    const bg = hex(pal[0]);
    const body = layers.map((l) => `<path fill="${l.color}" fill-rule="evenodd" d="${l.paths.join('')}"/>`).join('');
    const head = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}">`;
    return {
      svg: head + `<rect width="${W}" height="${H}" fill="${bg}"/>` + body + '</svg>',
      svg_transparent: head + body + '</svg>',
      width: W, height: H, bg, fg: layers[0].color, colors: layers.map((l) => l.color),
      shapes: layers.reduce((n, l) => n + l.paths.length, 0), arcs: ARCS, ms: Date.now() - t0,
    };
  }

  const api = { vectorizeRGBA };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  root.LogoVectorizer = api;
})(typeof self !== 'undefined' ? self : this);
