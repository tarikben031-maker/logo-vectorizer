"""Logo vectorizer engine: two-colour raster logo -> clean SVG.

vectorize(image_bytes) -> dict(svg, svg_transparent, width, height, fg, bg, shapes, arcs, ms)

Pipeline: colour separation -> coverage-aware 8x subpixel mask -> polygon (Douglas-Peucker)
-> corner detection + merging of antialias-split corners -> exact straight lines (axis snap)
-> exact circular arcs -> Schneider Bezier fitting -> per-shape area safety check.
"""
import io, re as _re, threading, time
import numpy as np, cv2
from PIL import Image

S = 8               # supersampling factor
DP_EPS = 0.2        # px, polygon tolerance
FIT_TOL = 0.25      # px, bezier tolerance
LINE_TOL = 0.2      # px
AXIS_SNAP = 2.5     # deg
MERGE_LEN = 0.9     # px
SMOOTH_MAX = 0.75   # px
ARC_RMAX = 25.0     # px
MASK_BLUR = 0.35    # px
MAX_SIDE = 3000     # larger images are downscaled first

_LOCK = threading.Lock()
ARCS = []
USE_ARCS = True

def coverage_mask(t, S):
    # each edge pixel gets round(coverage*S*S) foreground subpixels, arranged as a
    # half-plane perpendicular to the local gradient -> sub-pixel accurate edges
    ts = cv2.GaussianBlur(t, (0, 0), 0.8)
    gx = cv2.Sobel(ts, cv2.CV_32F, 1, 0, ksize=3); gy = cv2.Sobel(ts, cv2.CV_32F, 0, 1, ksize=3)
    Hh, Ww = t.shape
    big = cv2.resize(t, (Ww * S, Hh * S), interpolation=cv2.INTER_CUBIC)
    mask = (big > 0.5).astype(np.uint8)
    ys, xs = np.nonzero((t > 0.03) & (t < 0.97))
    # keep only clean single-edge pixels: fg one step along the gradient, bg one step against it
    gnx = gx[ys, xs]; gny = gy[ys, xs]; gn = np.hypot(gnx, gny) + 1e-9
    ux = np.rint(gnx / gn).astype(int); uy = np.rint(gny / gn).astype(int)
    xa = np.clip(xs + ux, 0, Ww-1); ya = np.clip(ys + uy, 0, Hh-1)
    xb = np.clip(xs - ux, 0, Ww-1); yb = np.clip(ys - uy, 0, Hh-1)
    xa2 = np.clip(xs + 2*ux, 0, Ww-1); ya2 = np.clip(ys + 2*uy, 0, Hh-1)
    xb2 = np.clip(xs - 2*ux, 0, Ww-1); yb2 = np.clip(ys - 2*uy, 0, Hh-1)
    ok = (t[ya, xa] > 0.85) & (t[yb, xb] < 0.15) & (gn > 0.05) & (t[ya2, xa2] > 0.85) & (t[yb2, xb2] < 0.15)
    ys, xs = ys[ok], xs[ok]
    off = (np.arange(S) + 0.5) / S - 0.5
    DX, DY = np.meshgrid(off, off)
    DX = DX.ravel(); DY = DY.ravel()
    nx = gx[ys, xs]; ny = gy[ys, xs]; nn = np.hypot(nx, ny) + 1e-9
    nx /= nn; ny /= nn
    proj = nx[:, None] * DX[None] + ny[:, None] * DY[None]          # larger = toward fg
    proj += 1e-4 * np.random.RandomState(0).rand(*proj.shape)
    k = np.rint(t[ys, xs] * S * S).astype(int)
    order = np.argsort(-proj, axis=1)
    rank = np.empty_like(order); rows = np.arange(len(ys))[:, None]
    rank[rows, order] = np.arange(S * S)[None]
    sub = (rank < k[:, None]).astype(np.uint8).reshape(-1, S, S)
    for i in range(len(ys)):
        mask[ys[i]*S:(ys[i]+1)*S, xs[i]*S:(xs[i]+1)*S] = sub[i]
    return mask


# ---------------- helpers ----------------
def unit(u):
    n = np.linalg.norm(u); return u / n if n > 1e-12 else u
def cross2(a, b): return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]
def angle_between(a, b):
    return np.degrees(np.arctan2(abs(cross2(a, b)), np.dot(a, b)))
def fmt(x): return f"{x:.2f}".rstrip("0").rstrip(".")
def fitline(pts):
    c = pts.mean(0); _, _, vt = np.linalg.svd(pts - c); return c, vt[0]
def intersect(c1, d1, c2, d2):
    den = cross2(d1, d2)
    if abs(den) < 1e-6: return None
    return c1 + (cross2(c2 - c1, d2) / den) * d1
def line_dev(pts, a, b):
    v = b - a; n = np.linalg.norm(v)
    if n < 1e-9: return np.linalg.norm(pts - a, axis=1).max() if len(pts) else 0
    return np.abs(cross2(v[None], pts - a)).max() / n if len(pts) else 0

# bezier fitting (Schneider)
def bez(p, u):
    m = 1 - u
    return (m**3)[:, None]*p[0] + (3*m*m*u)[:, None]*p[1] + (3*m*u*u)[:, None]*p[2] + (u**3)[:, None]*p[3]
def chord(pts):
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    return d / d[-1] if d[-1] > 0 else d
def gen(pts, u, t1, t2):
    p0, p3 = pts[0], pts[-1]
    A1 = (3*(1-u)**2*u)[:, None]*t1; A2 = (3*(1-u)*u*u)[:, None]*t2
    C = np.array([[np.sum(A1*A1), np.sum(A1*A2)], [np.sum(A1*A2), np.sum(A2*A2)]])
    tmp = pts - bez(np.array([p0, p0, p3, p3]), u)
    X = np.array([np.sum(A1*tmp), np.sum(A2*tmp)])
    det = C[0, 0]*C[1, 1] - C[0, 1]**2; seg = np.linalg.norm(p3 - p0)
    if abs(det) > 1e-12:
        a1 = (X[0]*C[1, 1] - X[1]*C[0, 1]) / det; a2 = (C[0, 0]*X[1] - C[0, 1]*X[0]) / det
    else: a1 = a2 = seg / 3
    if not (1e-6*seg < a1 < 2*seg) or not (1e-6*seg < a2 < 2*seg): a1 = a2 = seg / 3
    return np.array([p0, p0 + a1*t1, p3 + a2*t2, p3])
def reparam(b, pts, u):
    d1 = 3*np.array([b[1]-b[0], b[2]-b[1], b[3]-b[2]]); d2 = 2*np.array([d1[1]-d1[0], d1[2]-d1[1]])
    q = bez(b, u) - pts; m = 1-u
    q1 = (m*m)[:, None]*d1[0] + (2*m*u)[:, None]*d1[1] + (u*u)[:, None]*d1[2]
    q2 = m[:, None]*d2[0] + u[:, None]*d2[1]
    num = np.sum(q*q1, 1); den = np.sum(q1*q1, 1) + np.sum(q*q2, 1)
    with np.errstate(all='ignore'):
        return np.clip(u - np.where(np.abs(den) > 1e-12, num/den, 0), 0, 1)
def fit(pts, t1, t2, err, depth=0):
    if len(pts) <= 3:
        s = np.linalg.norm(pts[-1]-pts[0]) / 3
        return [np.array([pts[0], pts[0]+t1*s, pts[-1]+t2*s, pts[-1]])]
    u = chord(pts)
    for _ in range(10):
        b = gen(pts, u, t1, t2)
        dd = np.sum((bez(b, u) - pts)**2, 1)
        if dd.max() < err*err or depth > 10: return [b]
        u = reparam(b, pts, u)
    i = int(np.clip(np.argmax(dd), 2, len(pts)-3))
    tc = unit(pts[i-2] - pts[i+2])
    return fit(pts[:i+1], t1, tc, err, depth+1) + fit(pts[i:], -tc, t2, err, depth+1)

def smooth_open(pts, k):
    if len(pts) < 5 or k < 1: return pts
    out = pts.copy(); n = len(pts)
    for i in range(1, n-1):
        r = min(k, i, n-1-i)
        out[i] = pts[i-r:i+r+1].mean(0)
    return out
def smooth_closed(p, k):
    n = len(p); c = np.cumsum(np.r_[p[-k-1:], p, p[:k]], 0)
    return (c[2*k+1:] - c[:-2*k-1])[:n] / (2*k+1)


# ---------------- circle / arc detection ----------------
def fit_circle(pts):
    x, y = pts[:, 0], pts[:, 1]
    A = np.c_[2*x, 2*y, np.ones(len(x))]; bb = x*x + y*y
    try: sol = np.linalg.lstsq(A, bb, rcond=None)[0]
    except Exception: return None
    cx, cy = sol[0], sol[1]; r = np.sqrt(max(sol[2] + cx*cx + cy*cy, 0))
    res = np.abs(np.hypot(x - cx, y - cy) - r)
    return np.array([cx, cy]), r, res

def arc_beziers(c, r, a0, a1):
    """cubic beziers for arc from angle a0 to a1 (radians, signed sweep)"""
    sweep = a1 - a0; nseg = max(1, int(np.ceil(abs(sweep) / (np.pi / 2) - 1e-6)))
    out = []
    for i in range(nseg):
        t0 = a0 + sweep * i / nseg; t1 = a0 + sweep * (i + 1) / nseg
        k = 4/3 * np.tan((t1 - t0) / 4)
        p0 = c + r*np.array([np.cos(t0), np.sin(t0)]); p3 = c + r*np.array([np.cos(t1), np.sin(t1)])
        p1 = p0 + k*r*np.array([-np.sin(t0), np.cos(t0)]); p2 = p3 - k*r*np.array([-np.sin(t1), np.cos(t1)])
        out.append(np.array([p0, p1, p2, p3]))
    return out

def try_arc(pts, closed=False):
    """return (center, r, a0, a1) if pts lie on a circle, else None"""
    if len(pts) < 12: return None
    core = pts if closed else pts[len(pts)//6: len(pts) - len(pts)//6]
    f = fit_circle(core)
    if f is None: return None
    c, r, res = f
    if r < 1.2 or r > ARC_RMAX: return None
    if res.max() > max(0.25, 0.035 * r) or np.sqrt(np.mean(res**2)) > max(0.1, 0.013 * r): return None
    if not closed:
        allres = np.abs(np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1]) - r)
        if allres.max() > max(0.9, 0.12 * r): return None
    ang = np.unwrap(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))
    d = np.diff(ang)
    if not closed and not (np.all(d >= -0.02) or np.all(d <= 0.02)): return None
    sweep = ang[-1] - ang[0]
    if not closed and abs(sweep) < np.radians(100): return None
    return c, r, ang[0], ang[-1]

def find_arc_run(pts, min_sweep=np.radians(170)):
    """longest sub-run of pts lying on a circle with a large sweep -> (i0, i1, c, r)"""
    N = len(pts)
    if N < 40: return None
    # quick reject: total turning must be large enough
    stepd = max(1, N // 160)
    q = pts[::stepd]; M = len(q)
    dd = np.diff(q, axis=0); th = np.unwrap(np.arctan2(dd[:, 1], dd[:, 0]))
    if np.max(np.abs(th - th[0])) < min_sweep * 0.9 and abs(th[-1] - th[0]) < min_sweep * 0.9:
        if np.ptp(th) < min_sweep * 0.9: return None
    best = None
    lengths = sorted(set(int(M * f) for f in np.r_[np.arange(0.95, 0.3, -0.05), np.arange(0.3, 0.04, -0.02)]), reverse=True)
    for L in lengths:
        if L < 14: break
        for st in range(0, M - L + 1, max(1, L // 8)):
            seg = q[st:st + L]
            # cheap pre-check: segment direction must turn >= min_sweep
            if np.ptp(th[st:st + L - 1]) < min_sweep * 0.85: continue
            f = fit_circle(seg)
            if f is None: continue
            c, r, res = f
            if r < 1.5 or r > ARC_RMAX: continue
            if res.max() > max(0.25, 0.035 * r) or np.sqrt(np.mean(res**2)) > max(0.09, 0.012 * r): continue
            ang = np.unwrap(np.arctan2(seg[:, 1] - c[1], seg[:, 0] - c[0]))
            if abs(ang[-1] - ang[0]) < min_sweep: continue
            d = np.diff(ang)
            if not (np.all(d >= -0.03) or np.all(d <= 0.03)): continue
            best = (st * stepd, min(N - 1, (st + L - 1) * stepd), c, r); break
        if best is not None: break
    if best is None: return None
    # extend the run point-by-point while points stay on the circle
    i0, i1, c, r = best
    tol = max(0.25, 0.035 * r)
    while i0 > 0 and abs(np.hypot(*(pts[i0-1] - c)) - r) < tol: i0 -= 1
    while i1 < N-1 and abs(np.hypot(*(pts[i1+1] - c)) - r) < tol: i1 += 1
    return i0, i1, c, r

def _bz(ptsq, tA, tB):
    plen = np.sum(np.linalg.norm(np.diff(ptsq, axis=0), axis=1)) if len(ptsq) > 1 else 0
    if len(ptsq) < 3 or plen < 0.25:
        return f"L{fmt(ptsq[-1][0])} {fmt(ptsq[-1][1])}"
    o = ""
    for bz in fit(ptsq, tA, tB, FIT_TOL):
        o += "C" + " ".join(f"{fmt(bz[i][0])} {fmt(bz[i][1])}" for i in (1, 2, 3))
    return o

def _end_dir(seq):
    q = seq[:max(3, min(len(seq), int(1.0*S)))]
    if len(q) >= 3:
        c0, d0 = fitline(q); d0 = unit(d0)
        if np.dot(d0, q[-1]-q[0]) < 0: d0 = -d0
        return d0
    return unit(seq[-1] - seq[0])

USE_ARCS = True
def emit_curve(rawp, sp, pa, pb, t1, t2, default_pts, depth=0):
    """curve from pa to pb following rawp; circular sub-runs become exact arcs"""
    run = find_arc_run(rawp) if (depth < 4 and USE_ARCS) else None
    if run is None:
        return _bz(default_pts, t1, t2)
    i0, i1, c_, r_ = run
    ea = c_ + r_ * unit(rawp[i0] - c_); eb = c_ + r_ * unit(rawp[i1] - c_)
    if i0 <= 2: ea = pa if abs(np.hypot(*(pa - c_)) - r_) < 0.4 else ea
    if i1 >= len(rawp) - 3: eb = pb if abs(np.hypot(*(pb - c_)) - r_) < 0.4 else eb
    seg = rawp[i0:i1+1]
    ang = np.unwrap(np.arctan2(seg[:, 1] - c_[1], seg[:, 0] - c_[0]))
    a0_ = np.arctan2(ea[1]-c_[1], ea[0]-c_[0])
    a1_ = np.arctan2(eb[1]-c_[1], eb[0]-c_[0])
    sw = a1_ - a0_; want = ang[-1] - ang[0]
    while sw - want > np.pi: sw -= 2*np.pi
    while want - sw > np.pi: sw += 2*np.pi
    # verify: sampled arc must follow the original points
    tt = np.linspace(a0_, a0_ + sw, 64)
    arcpts = c_ + r_ * np.c_[np.cos(tt), np.sin(tt)]
    dmax = max(np.min(np.linalg.norm(arcpts[:, None] - seg[None, ::4], axis=2), axis=1).max(),
               np.min(np.linalg.norm(seg[::4, None] - arcpts[None], axis=2), axis=1).max())
    if dmax > 0.8 or abs(sw - want) > 0.6:
        return _bz(default_pts, t1, t2)
    ARCS.append((c_, r_))
    sgn = 1 if sw > 0 else -1
    tan0 = sgn * np.array([-np.sin(a0_), np.cos(a0_)])
    tan1 = sgn * np.array([-np.sin(a0_ + sw), np.cos(a0_ + sw)])
    o = ""
    # part before the arc
    if np.linalg.norm(ea - pa) > 0.05:
        partA = np.r_[[pa], sp[1:i0], [ea]] if i0 > 1 else np.array([pa, ea])
        tAend = _end_dir(partA[::-1])
        if angle_between(-tAend, tan0) < 25: tAend = -tan0
        o += emit_curve(rawp[:i0+1], sp[:i0+1], pa, ea, t1, tAend, partA, depth+1)
    for bz in arc_beziers(c_, r_, a0_, a0_ + sw):
        o += "C" + " ".join(f"{fmt(bz[i][0])} {fmt(bz[i][1])}" for i in (1, 2, 3))
    if np.linalg.norm(pb - eb) > 0.05:
        partB = np.r_[[eb], sp[i1+1:-1], [pb]] if i1 < len(sp) - 2 else np.array([eb, pb])
        tBst = _end_dir(partB)
        if angle_between(tBst, tan1) < 25: tBst = tan1
        o += emit_curve(rawp[i1:], sp[i1:], eb, pb, tBst, t2, partB, depth+1)
    return o

# ---------------- tracing ----------------
def trace(c):
    raw = (c[:, 0, :].astype(np.float64) + 0.5) / S          # 1/8 px staircase
    n = len(raw)
    # de-staircase lightly (±0.25px) for geometry measurements
    P = smooth_closed(raw, 2)
    poly_idx = cv2.approxPolyDP((P * S).astype(np.float32).reshape(-1, 1, 2), DP_EPS * S, True)
    # map polygon vertices back to indices
    tree = {tuple(np.round(q * S, 3)): i for i, q in enumerate(P)}
    idx = []
    for q in poly_idx[:, 0, :]:
        j = int(np.argmin(np.sum((P * S - q) ** 2, 1))); idx.append(j)
    idx = sorted(set(idx))
    if len(idx) < 3: return smooth_loop(P)
    # vertices: [pos, idx_in, idx_out]
    VV = [[P[i].copy(), i, i] for i in idx]
    def sang(a, b): return np.degrees(np.arctan2(cross2(a, b), np.dot(a, b)))
    changed = True
    while changed and len(VV) > 4:
        changed = False
        m = len(VV)
        best = None
        for k in range(m):
            for j in (1, 2, 3):
                if j + 3 > m: break
                ids = [(k + q) % m for q in range(j + 1)]           # vertices to merge
                A = VV[(k - 1) % m][0]; D = VV[(k + j + 1) % m][0]
                pts_ = [VV[i][0] for i in ids]
                inner = sum(np.linalg.norm(pts_[q+1] - pts_[q]) for q in range(j))
                if inner > MERGE_LEN + 0.55 * (j - 1): continue
                B, E = pts_[0], pts_[-1]
                la, ld = np.linalg.norm(B - A), np.linalg.norm(D - E)
                if j > 1 and (min(la, ld) < 1.0 or min(la, ld) < 1.5 * inner): continue
                seq = [A] + pts_ + [D]
                turns = [sang(seq[q+1] - seq[q], seq[q+2] - seq[q+1]) for q in range(len(seq) - 2)]
                if not (all(t_ > 0 for t_ in turns) or all(t_ < 0 for t_ in turns)): continue
                tot = abs(sang(B - A, D - E))
                if tot < 55 or tot > 178: continue
                x = intersect(A, unit(B - A), D, unit(D - E))
                if x is None: continue
                lim = 0.75 if tot <= 150 else 1.4
                if np.min(np.linalg.norm(np.array(pts_) - x, axis=1)) > lim: continue
                score = inner / j
                if best is None or score < best[0]: best = (score, ids, x)
        if best:
            _, ids, x = best
            merged = [x, VV[ids[0]][1], VV[ids[-1]][2]]
            drop = set(ids)
            NV = []
            for i in range(len(VV)):
                if i == ids[0]: NV.append(merged)
                elif i not in drop: NV.append(VV[i])
            VV = NV
            changed = True
    m = len(VV)
    V = np.array([q[0] for q in VV])
    L_prev = np.linalg.norm(V - np.roll(V, 1, 0), axis=1)
    L_next = np.roll(L_prev, -1)
    ang = np.array([angle_between(V[k] - V[k-1], V[(k+1) % m] - V[k]) for k in range(m)])
    corner = np.zeros(m, bool)
    for k in range(m):
        a = ang[k]
        if a >= 60: corner[k] = True
        elif a >= 40 and min(L_prev[k], L_next[k]) >= 1.3: corner[k] = True
        elif a >= 33 and a >= 2.0 * max(ang[k-1], ang[(k+1) % m]) and min(L_prev[k], L_next[k]) >= 1.0: corner[k] = True
    ci = np.nonzero(corner)[0]
    if len(ci) == 0: return smooth_loop(P)
    Cpos = {k: V[k].copy() for k in ci}
    corner_set = set(int(k) for k in ci)

    idx_in = [q[1] for q in VV]; idx_out = [q[2] for q in VV]
    # segments between consecutive corners
    cks = list(ci)
    segs = []
    for j in range(len(cks)):
        ka, kb = cks[j], cks[(j+1) % len(cks)]
        ia, ib = idx_out[ka], idx_in[kb]
        rng = np.arange(ia, ib + (n if ib <= ia else 0) + 1) % n
        segs.append((ka, kb, rng))
    # classify & snap
    kinds = []
    for ka, kb, rng in segs:
        a, b = Cpos[ka], Cpos[kb]
        inner = P[rng][2:-2] if len(rng) > 6 else P[rng]
        kinds.append('L' if line_dev(inner, a, b) < LINE_TOL else 'C')
    for (ka, kb, rng), kd in zip(segs, kinds):
        if kd != 'L': continue
        a, b = Cpos[ka], Cpos[kb]; d = b - a; L = np.linalg.norm(d)
        if L < 0.6: continue
        g = np.degrees(np.arctan2(abs(d[1]), abs(d[0])))
        if g < AXIS_SNAP: y = (a[1]+b[1])/2; a[1] = b[1] = y
        elif g > 90-AXIS_SNAP: x = (a[0]+b[0])/2; a[0] = b[0] = x
    out = f"M{fmt(Cpos[segs[0][0]][0])} {fmt(Cpos[segs[0][0]][1])}"
    for (ka, kb, rng), kd in zip(segs, kinds):
        a, b = Cpos[ka], Cpos[kb]
        if kd == 'L':
            out += f"L{fmt(b[0])} {fmt(b[1])}"; continue
        # --- find long straight edges inside this curved run (stems, flats) ---
        ks = [(ka + 1 + q) % m for q in range((kb - ka - 1) % m)]      # intermediate vertices
        chain = [ka] + ks + [kb]
        lines = []
        for u, w in zip(chain[:-1], chain[1:]):
            pu = Cpos[u] if u in Cpos else V[u]; pw = Cpos[w] if w in Cpos else V[w]
            L = np.linalg.norm(pw - pu)
            if L < 2.5: continue
            iu, iw = idx_out[u], idx_in[w]
            rr = np.arange(iu, iw + (n if iw < iu else 0) + 1) % n
            inner = P[rr][len(rr)//8: len(rr) - len(rr)//8]
            if len(inner) < 3: continue
            dvec = pw - pu
            g = np.degrees(np.arctan2(abs(dvec[1]), abs(dvec[0])))
            axis = g < 4 or g > 86
            dv = line_dev(inner, pu, pw)
            nb = max(np.linalg.norm(V[u] - V[u-1]), np.linalg.norm(V[(w+1) % m] - V[w]))
            su = cross2(V[u] - V[u-1], V[(u+1) % m] - V[u]); sw = cross2(V[w] - V[w-1], V[(w+1) % m] - V[w])
            extremum = (u not in corner_set) and (w not in corner_set) and su * sw > 0
            if extremum and dv > 0.03: continue
            if dv < min(0.3, max(0.1, 0.015 * L)) and (dv < 0.045 or L >= 1.8 * nb) and (axis or L >= 6):
                lines.append((u, w))
        # build pieces
        pieces = []; cur = ka
        for u, w in lines:
            if u != cur: pieces.append(('c', cur, u))
            pieces.append(('l', u, w)); cur = w
        if cur != kb or not pieces: pieces.append(('c', cur, kb))
        pos = lambda k_: Cpos[k_] if k_ in Cpos else V[k_]
        # snap axis lines
        for kind, u, w in pieces:
            if kind != 'l': continue
            pu, pw = pos(u), pos(w); d = pw - pu
            g = np.degrees(np.arctan2(abs(d[1]), abs(d[0])))
            if u not in Cpos: Cpos[u] = V[u].copy()
            if w not in Cpos: Cpos[w] = V[w].copy()
            if g < AXIS_SNAP: y = (Cpos[u][1] + Cpos[w][1]) / 2; Cpos[u][1] = Cpos[w][1] = y
            elif g > 90 - AXIS_SNAP: x = (Cpos[u][0] + Cpos[w][0]) / 2; Cpos[u][0] = Cpos[w][0] = x
        for pi, (kind, u, w) in enumerate(pieces):
            pa, pb = pos(u), pos(w)
            if kind == 'l':
                out += f"L{fmt(pb[0])} {fmt(pb[1])}"; continue
            iu, iw = idx_out[u], idx_in[w]
            rr = np.arange(iu, iw + (n if iw <= iu else 0) + 1) % n
            pts = raw[rr]
            seglen = np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))
            kk = int(np.clip(seglen / 10, 0.25, SMOOTH_MAX) * S)
            pts = smooth_open(pts, kk)
            trim = min(int(0.35 * S), len(pts) // 4) if (u in corner_set or w in corner_set) else 0
            body = pts[trim: len(pts) - trim] if len(pts) > 2*trim + 4 else pts[1:-1]
            pts = np.r_[[pa], body, [pb]]
            def end_dir(seq):
                q = seq[:max(3, min(len(seq), int(1.0*S)))]
                if len(q) >= 3:
                    c0, d0 = fitline(q); d0 = unit(d0)
                    if np.dot(d0, q[-1]-q[0]) < 0: d0 = -d0
                    return d0
                return unit(seq[-1] - seq[0])
            t1 = end_dir(pts); t2 = end_dir(pts[::-1])
            if pi > 0 and pieces[pi-1][0] == 'l':
                t1 = unit(pos(pieces[pi-1][2]) - pos(pieces[pi-1][1]))
            if pi + 1 < len(pieces) and pieces[pi+1][0] == 'l':
                t2 = -unit(pos(pieces[pi+1][2]) - pos(pieces[pi+1][1]))
            arc = try_arc(raw[rr]) if USE_ARCS else None
            if arc is not None:
                ARCS.append(arc)
                c_, r_, a0_, a1_ = arc
                # snap both ends onto the circle (keeps the joint with neighbouring lines)
                ea = c_ + r_ * unit(pa - c_); eb = c_ + r_ * unit(pb - c_)
                a0_ = np.arctan2(*(ea - c_)[::-1]); a1_ = np.arctan2(*(eb - c_)[::-1])
                sw = a1_ - a0_
                want = arc[3] - arc[2]
                while sw - want > np.pi: sw -= 2*np.pi
                while want - sw > np.pi: sw += 2*np.pi
                if u in Cpos: Cpos[u][:] = ea
                if np.linalg.norm(ea - pa) > 1e-6:
                    out += f"L{fmt(ea[0])} {fmt(ea[1])}"
                for bz in arc_beziers(c_, r_, a0_, a0_ + sw):
                    out += "C" + " ".join(f"{fmt(bz[i][0])} {fmt(bz[i][1])}" for i in (1, 2, 3))
                if w in Cpos: Cpos[w][:] = eb
                continue
            out += emit_curve(raw[rr], smooth_open(raw[rr], kk), pa, pb, t1, t2, pts)
            continue
            for bz in fit(pts, t1, t2, FIT_TOL):
                out += "C" + " ".join(f"{fmt(bz[i][0])} {fmt(bz[i][1])}" for i in (1, 2, 3))
    return out + "Z"

def smooth_loop(P):
    n = len(P)
    arc = try_arc(np.r_[P, P[:1]], closed=True) if USE_ARCS else None
    if arc is not None:
        c_, r_, a0_, a1_ = arc
        sw = 2*np.pi if (a1_ - a0_) > 0 else -2*np.pi
        bzs = arc_beziers(c_, r_, 0.0, sw)
        s_ = f"M{fmt(bzs[0][0][0])} {fmt(bzs[0][0][1])}"
        for bz in bzs:
            s_ += "C" + " ".join(f"{fmt(bz[i][0])} {fmt(bz[i][1])}" for i in (1, 2, 3))
        return s_ + "Z"
    L = np.sum(np.linalg.norm(np.diff(np.r_[P, P[:1]], axis=0), axis=1))
    k = int(np.clip(L / 24, 0.2, 1.5) * S)
    ps = smooth_closed(P, max(1, k))
    h = n // 2
    tang = lambda i: unit(ps[(i+1) % n] - ps[(i-1) % n])
    s = f"M{fmt(ps[0][0])} {fmt(ps[0][1])}"
    for a, b in ((0, h), (h, n)):
        seg = ps[np.arange(a, b+1) % n]
        for bz in fit(seg, tang(a), -tang(b % n), FIT_TOL):
            s += "C" + " ".join(f"{fmt(bz[i][0])} {fmt(bz[i][1])}" for i in (1, 2, 3))
    return s + "Z"

import re as _re
def path_area(d):
    pts = []; cur = None
    for cmd, args in _re.findall(r'([MLCZ])([^MLCZ]*)', d):
        nums = [float(x) for x in _re.findall(r'-?\d+(?:\.\d+)?(?:e-?\d+)?', args)]
        if cmd in 'ML': cur = np.array(nums[:2]); pts.append(cur)
        elif cmd == 'C':
            p = np.array([cur, nums[0:2], nums[2:4], nums[4:6]], float)
            for u_ in np.linspace(0, 1, 10)[1:]: pts.append(bez(p, np.array([u_]))[0])
            cur = p[3]
    if len(pts) < 3: return 0.0
    q = np.array(pts); return abs(0.5 * np.sum(q[:, 0]*np.roll(q[:, 1], -1) - np.roll(q[:, 0], -1)*q[:, 1]))

def find_arc_run_off(*a, **k): return None

def path_area(d):
    pts = []; cur = None
    for cmd, args in _re.findall(r'([MLCZ])([^MLCZ]*)', d):
        nums = [float(x) for x in _re.findall(r'-?\d+(?:\.\d+)?(?:e-?\d+)?', args)]
        if cmd in 'ML': cur = np.array(nums[:2]); pts.append(cur)
        elif cmd == 'C':
            p = np.array([cur, nums[0:2], nums[2:4], nums[4:6]], float)
            for u_ in np.linspace(0, 1, 10)[1:]: pts.append(bez(p, np.array([u_]))[0])
            cur = p[3]
    if len(pts) < 3: return 0.0
    q = np.array(pts); return abs(0.5 * np.sum(q[:, 0]*np.roll(q[:, 1], -1) - np.roll(q[:, 0], -1)*q[:, 1]))

def _hex(c): return "#%02x%02x%02x" % tuple(int(round(x)) for x in c)

def vectorize(data: bytes) -> dict:
    """data: bytes of a PNG/JPG/WebP image. Returns SVG strings and stats."""
    global USE_ARCS
    t0 = time.time()
    im = Image.open(io.BytesIO(data))
    im.load()
    im = im.convert("RGBA")
    if max(im.size) > MAX_SIDE:
        k = MAX_SIDE / max(im.size)
        im = im.resize((max(1, round(im.width * k)), max(1, round(im.height * k))), Image.LANCZOS)
    rgb0 = np.array(im).astype(np.float32)
    al = rgb0[..., 3:4] / 255
    border = np.concatenate([rgb0[0, :, :3], rgb0[-1, :, :3], rgb0[:, 0, :3], rgb0[:, -1, :3]])
    bgc = np.median(border, 0)
    if al.mean() < 0.999 and np.median(np.concatenate([rgb0[0, :, 3], rgb0[-1, :, 3]])) < 128:
        bgc = np.array([255.0, 255.0, 255.0])          # transparent input -> white background
    rgb = rgb0[..., :3] * al + bgc * (1 - al)
    dist = np.linalg.norm(rgb - bgc, axis=2)
    if dist.max() < 30:
        raise ValueError("L'image ne contient pas de logo visible (une seule couleur).")
    fgc = np.median(rgb[dist > 0.8 * dist.max()], 0)
    v = fgc - bgc
    t = np.clip(((rgb - bgc) @ v) / (v @ v), 0, 1).astype(np.float32)
    H, W = t.shape
    with _LOCK:
        ARCS.clear()
        mf = coverage_mask(t, S).astype(np.float32)
        mf = cv2.GaussianBlur(mf, (0, 0), S * MASK_BLUR)
        mask = (mf > 0.5).astype(np.uint8) * 255
        contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        paths = []
        for c in contours:
            if cv2.contourArea(c) < S*S*0.8: continue
            a0 = cv2.contourArea(c) / S / S
            try:
                d = trace(c)
            except Exception:
                d = None
            if d is None or abs(path_area(d) - a0) / a0 > 0.05:
                USE_ARCS = False
                try:
                    d2 = trace(c)
                finally:
                    USE_ARCS = True
                if d is None or abs(path_area(d2) - a0) < abs(path_area(d) - a0): d = d2
            paths.append(d)
        n_arcs = len(ARCS)
    fg, bg = _hex(fgc), _hex(bgc)
    body = f'<path fill="{fg}" fill-rule="evenodd" d="{"".join(paths)}"/>'
    head = f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}">'
    return {
        "svg": head + f'<rect width="{W}" height="{H}" fill="{bg}"/>' + body + "</svg>",
        "svg_transparent": head + body + "</svg>",
        "width": W, "height": H, "fg": fg, "bg": bg,
        "shapes": len(paths), "arcs": n_arcs, "ms": int((time.time() - t0) * 1000),
    }
