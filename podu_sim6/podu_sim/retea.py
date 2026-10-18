# -*- coding: utf-8 -*-
"""
Citeste reteaua SUMO (retea/podu_ros.net.xml) direct, fara SUMO:
benzi, legaturi intre benzi, matricile de prioritate din fiecare intersectie (cine cedeaza cui),
semafoare, treceri de pietoni, trotuare. Rezultatul se pastreaza intr-un cache (.pkl).
"""
import os, math, pickle, heapq
import xml.etree.ElementTree as ET
from collections import defaultdict

CACHE_VERSION = 9
VCLASSES = ('passenger', 'tram', 'pedestrian', 'emergency')
ALL_CLASSES = set(VCLASSES)


def parse_shape(s):
    return [tuple(float(v) for v in p.split(',')[:2]) for p in s.split()]


def poly_len(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def cumlen(pts):
    out = [0.0]
    for a, b in zip(pts, pts[1:]):
        out.append(out[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return out


def point_at(pts, cum, s):
    """punctul si directia la distanta s de-a lungul poliliniei"""
    if s <= 0:
        a, b = pts[0], pts[1] if len(pts) > 1 else pts[0]
        return a[0], a[1], math.atan2(b[1] - a[1], b[0] - a[0])
    n = len(pts)
    if s >= cum[-1]:
        a, b = pts[-2] if n > 1 else pts[-1], pts[-1]
        return b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0])
    lo, hi = 0, n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if cum[mid] <= s:
            lo = mid
        else:
            hi = mid
    a, b = pts[lo], pts[lo + 1]
    seg = (cum[lo + 1] - cum[lo]) or 1e-9
    t = (s - cum[lo]) / seg
    return a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, math.atan2(b[1] - a[1], b[0] - a[0])


def seg_intersect(p1, p2, p3, p4):
    """intersectia a doua segmente -> (t, u) sau None"""
    d = (p2[0] - p1[0]) * (p4[1] - p3[1]) - (p2[1] - p1[1]) * (p4[0] - p3[0])
    if abs(d) < 1e-12:
        return None
    t = ((p3[0] - p1[0]) * (p4[1] - p3[1]) - (p3[1] - p1[1]) * (p4[0] - p3[0])) / d
    u = ((p3[0] - p1[0]) * (p2[1] - p1[1]) - (p3[1] - p1[1]) * (p2[0] - p1[0])) / d
    if 0 <= t <= 1 and 0 <= u <= 1:
        return t, u
    return None


def poly_intersect(a, b):
    """prima intersectie a doua polilinii -> (pozitia pe a, pozitia pe b) sau None"""
    ca, cb = cumlen(a), cumlen(b)
    best = None
    for i in range(len(a) - 1):
        for j in range(len(b) - 1):
            r = seg_intersect(a[i], a[i + 1], b[j], b[j + 1])
            if r:
                pa = ca[i] + r[0] * (ca[i + 1] - ca[i])
                pb = cb[j] + r[1] * (cb[j + 1] - cb[j])
                if best is None or pa < best[0]:
                    best = (pa, pb)
    return best


def dist_point_seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1])


class Lane(object):
    __slots__ = ('id', 'edge', 'index', 'length', 'speed', 'width', 'shape', 'cum', 'allow', 'kind',
                 'links', 'next_internal', 'link', 'vehs', 'peds', 'num')

    def __init__(self):
        self.links = []           # Link-uri care pleaca de pe aceasta banda (doar benzi normale)
        self.next_internal = None  # pentru benzi interne: urmatoarea banda
        self.link = None          # pentru benzi interne: Link-ul caruia ii apartine
        self.vehs = []            # vehicule pe banda, ordonate dupa pozitie (crescator)
        self.peds = []


class Edge(object):
    __slots__ = ('id', 'func', 'frm', 'to', 'name', 'lanes', 'length', 'speed', 'crossing_edges', 'num')


class Link(object):
    __slots__ = ('num', 'frm', 'to', 'via', 'via_len', 'junction', 'jidx', 'tl', 'tl_index', 'dir', 'state',
                 'resp', 'foes', 'conf', 'is_ped', 'crossing', 'occ', 'appr', 'appr_prev', 'over_crossing',
                 'vclass_ok', 'speed', 'node')

    def __init__(self):
        self.resp, self.foes, self.conf = [], [], {}
        self.occ = {}             # vehicule (sau pietoni pentru treceri) aflate pe legatura (ordonat)
        self.appr, self.appr_prev = {}, {}   # vehicule care se apropie: obj -> (t_sosire, t_plecare, viteza)
        self.over_crossing = None


class Junction(object):
    __slots__ = ('id', 'type', 'x', 'y', 'links', 'shape')


class Crossing(object):
    __slots__ = ('id', 'lane', 'node', 'width', 'shape', 'length', 'ped_links', 'car_links', 'crossed',
                 'cars', 'name', 'zona', 'tl', 'priority', 'ends', 'num')


class Network(object):
    """toata reteaua, gata de simulat"""

    def lane_xy(self, lane, pos):
        return point_at(lane.shape, lane.cum, pos)


def _allow(lane_el):
    a, d = lane_el.get('allow'), lane_el.get('disallow')
    if a is not None:
        s = set(a.split())
        out = {c for c in VCLASSES if c in s}
        if 'passenger' in s:
            out.add('emergency')
        return frozenset(out)
    if d is not None:
        s = set(d.split())
        return frozenset(c for c in VCLASSES if c not in s)
    return frozenset(VCLASSES)


def load(net_path, names=None, log=print):
    cache = net_path + '.podu_sim.pkl'
    try:
        if os.path.getmtime(cache) >= os.path.getmtime(net_path):
            with open(cache, 'rb') as f:
                net = pickle.load(f)
            if getattr(net, 'version', 0) == CACHE_VERSION:
                return net
    except Exception:
        pass
    log('Citesc reteaua (o singura data, apoi din cache) ...')
    net = build(net_path, names or {})
    try:
        with open(cache, 'wb') as f:
            pickle.dump(net, f, protocol=pickle.HIGHEST_PROTOCOL)
    except Exception:
        pass
    return net


def build(net_path, names):
    root = ET.parse(net_path).getroot()
    net = Network()
    net.version = CACHE_VERSION
    net.edges, net.lanes, net.links, net.junctions, net.tls, net.crossings = {}, {}, [], {}, {}, {}
    loc = root.find('location')
    net.offset = [float(v) for v in loc.get('netOffset').split(',')]
    net.bounds = [float(v) for v in loc.get('convBoundary').split(',')]
    for e in root.findall('edge'):
        ed = Edge()
        ed.id, ed.func = e.get('id'), e.get('function', '')
        ed.frm, ed.to, ed.name = e.get('from'), e.get('to'), e.get('name', '')
        ed.crossing_edges = e.get('crossingEdges', '').split()
        ed.lanes = []
        for l in e.findall('lane'):
            ln = Lane()
            ln.id, ln.edge, ln.index = l.get('id'), ed, int(l.get('index'))
            ln.length = max(0.1, float(l.get('length')))
            ln.speed = float(l.get('speed'))
            ln.width = float(l.get('width', '3.2'))
            ln.shape = parse_shape(l.get('shape'))
            if len(ln.shape) == 1:
                ln.shape.append(ln.shape[0])
            ln.cum = cumlen(ln.shape)
            ln.allow = _allow(l)
            ln.kind = ed.func or 'normal'
            ed.lanes.append(ln)
            net.lanes[ln.id] = ln
        ed.length = ed.lanes[0].length if ed.lanes else 0
        ed.speed = max(l.speed for l in ed.lanes) if ed.lanes else 13.89
        net.edges[ed.id] = ed
    jreq = {}
    net.intj = {}
    net.roundabout = set()
    for rb in root.findall('roundabout'):
        net.roundabout |= set(rb.get('edges', '').split())
    for j in root.findall('junction'):
        if j.get('type') == 'internal':
            # viraj in doua etape: vehiculul care intra pe banda interna j cedeaza celor de pe intLanes
            net.intj[j.get('id')] = j.get('intLanes', '').split()
            continue
        J = Junction()
        J.id, J.type = j.get('id'), j.get('type')
        J.x, J.y = float(j.get('x')), float(j.get('y'))
        J.shape = parse_shape(j.get('shape')) if j.get('shape') else []
        J.links = {}
        net.junctions[J.id] = J
        intl = j.get('intLanes', '').split()
        reqs = {}
        for q in j.findall('request'):
            reqs[int(q.get('index'))] = (q.get('response')[::-1], q.get('foes')[::-1])
        jreq[J.id] = ({ln: i for i, ln in enumerate(intl)}, reqs)
    # benzi interne: continuari
    cont = {}
    for c in root.findall('connection'):
        f = c.get('from')
        if f.startswith(':') and net.edges[f].func == 'internal':
            lf = '%s_%s' % (f, c.get('fromLane'))
            nxt = c.get('via') or '%s_%s' % (c.get('to'), c.get('toLane'))
            cont[lf] = nxt
    for lf, nxt in cont.items():
        net.lanes[lf].next_internal = net.lanes.get(nxt)
    # legaturi
    walk_conn = []
    for c in root.findall('connection'):
        f, t = c.get('from'), c.get('to')
        ef, et = net.edges.get(f), net.edges.get(t)
        if ef is None or et is None:
            continue
        if ef.func in ('walkingarea', 'crossing') or et.func in ('walkingarea', 'crossing'):
            walk_conn.append((f, t, c))
            if not (ef.func == 'walkingarea' and et.func == 'crossing'):
                continue
        if ef.func == 'internal':
            continue
        lk = Link()
        lk.num = len(net.links)
        lk.frm = net.lanes['%s_%s' % (f, c.get('fromLane'))]
        lk.to = net.lanes['%s_%s' % (t, c.get('toLane'))]
        lk.is_ped = ef.func == 'walkingarea'
        lk.dir, lk.state = c.get('dir'), c.get('state')
        lk.tl, lk.tl_index = c.get('tl'), (int(c.get('linkIndex')) if c.get('linkIndex') else None)
        via = []
        v = c.get('via')
        while v and v in net.lanes and len(via) < 6:
            vl = net.lanes[v]
            via.append(vl)
            nx = vl.next_internal
            if nx is None or nx.kind != 'internal':
                break
            v = nx.id
        lk.via = via
        lk.via_len = sum(x.length for x in via)
        for vl in via:
            vl.link = lk
        lk.node = ef.to if not lk.is_ped else t[1:].rsplit('_', 1)[0]
        lk.junction = net.junctions.get(lk.node)
        lk.speed = min([x.speed for x in via] or [lk.to.speed])
        lk.crossing = t if lk.is_ped else None
        lk.jidx = None
        if lk.junction is not None:
            idxmap = jreq[lk.junction.id][0]
            key = lk.to.id if lk.is_ped else (via[0].id if via else None)
            lk.jidx = idxmap.get(key)
            if lk.jidx is not None:
                lk.junction.links[lk.jidx] = lk
        net.links.append(lk)
        if not lk.is_ped:
            lk.frm.links.append(lk)
    # matricile de prioritate
    for J in net.junctions.values():
        reqs = jreq[J.id][1]
        for i, lk in J.links.items():
            resp, foes = reqs.get(i, ('', ''))
            lk.resp = [J.links[j] for j, ch in enumerate(resp) if ch == '1' and j in J.links]
            lk.foes = [J.links[j] for j, ch in enumerate(foes) if ch == '1' and j in J.links]
    # puncte de conflict (unde se intersecteaza drumurile prin intersectie)
    for J in net.junctions.values():
        for lk in J.links.values():
            pa = _path(lk)
            for fo in lk.foes:
                pb = _path(fo)
                if len(pa) < 2 or len(pb) < 2:
                    continue
                r = poly_intersect(pa, pb)
                if r is None:
                    if not lk.is_ped and not fo.is_ped and lk.to is fo.to:
                        r = (poly_len(pa), poly_len(pb))       # se unesc pe aceeasi banda
                    else:
                        continue
                lk.conf[fo.num] = r
    # treceri de pietoni
    for ed in net.edges.values():
        if ed.func != 'crossing':
            continue
        cr = Crossing()
        cr.id, cr.lane = ed.id, ed.lanes[0]
        cr.node = ed.id[1:].rsplit('_', 1)[0]
        cr.width, cr.shape, cr.length = cr.lane.width, cr.lane.shape, cr.lane.length
        cr.crossed = ed.crossing_edges
        cr.ped_links = [lk for lk in net.links if lk.is_ped and lk.to is cr.lane]
        cl = set()
        for pl in cr.ped_links:
            for fo in pl.foes:
                if not fo.is_ped:
                    cl.add(fo)
        cr.car_links = sorted(cl, key=lambda k: k.num)
        for lk in cr.car_links:
            r = lk.conf.get(cr.ped_links[0].num) if cr.ped_links else None
            lk.over_crossing = (cr.id, r[0] if r else 0.0)
        cr.cars = any('passenger' in l.allow for e in cr.crossed if e in net.edges for l in net.edges[e].lanes)
        tls = {pl.tl for pl in cr.ped_links if pl.tl}
        cr.tl = tls.pop() if tls else None
        cr.priority = True
        cr.name, cr.zona = names.get(cr.id, (cr.id, ''))
        net.crossings[cr.id] = cr
    # semafoare
    for t in root.findall('tlLogic'):
        ph = []
        for p in t.findall('phase'):
            ph.append(dict(dur=float(p.get('duration')), state=p.get('state'),
                           minDur=float(p.get('minDur', p.get('duration'))),
                           maxDur=float(p.get('maxDur', p.get('duration')))))
        net.tls[t.get('id')] = dict(id=t.get('id'), type=t.get('type'), offset=float(t.get('offset', '0')),
                                    phases=ph, links=defaultdict(list))
    for lk in net.links:
        if lk.tl in net.tls:
            net.tls[lk.tl]['links'][lk.tl_index].append(lk)
    for t in net.tls.values():
        t['links'] = dict(t['links'])
    # graful pietonilor
    build_ped_graph(net, walk_conn)
    for i, ed in enumerate(net.edges.values()):
        ed.num = i
    return net


def _path(lk):
    if lk.is_ped:
        return list(lk.to.shape)
    pts = []
    for v in lk.via:
        pts += v.shape if not pts else v.shape[1:]
    return pts


# ----------------------------------------------------------------------------- pietoni
def build_ped_graph(net, walk_conn):
    """noduri = zone de mers (walkingarea) + capete de trotuar; muchii = trotuare si treceri"""
    adj = defaultdict(list)      # nod -> [(nod2, cost, segment)]
    net.ped_lane_ends = {}       # lane id -> (nod la inceput, nod la sfarsit)
    start_node, end_node = {}, {}
    cand = defaultdict(set)
    for f, t, c in walk_conn:
        ef, et = net.edges[f], net.edges[t]
        if ef.func == 'walkingarea' and et.func not in ('crossing', 'walkingarea'):
            cand['%s_%s' % (t, c.get('toLane'))].add(f)
        elif et.func == 'walkingarea' and ef.func not in ('crossing', 'walkingarea'):
            cand['%s_%s' % (f, c.get('fromLane'))].add(t)

    def dist_to_area(p, w):
        sh = net.edges[w].lanes[0].shape
        return min(dist_point_seg(p, a, b) for a, b in zip(sh, sh[1:] + sh[:1]))
    for lid, ws in cand.items():
        ln = net.lanes[lid]
        p0, p1 = ln.shape[0], ln.shape[-1]
        for w in ws:
            d0, d1 = dist_to_area(p0, w), dist_to_area(p1, w)
            if abs(d0 - d1) < 0.5 or ln.length < 2.0:
                continue                      # banda prea scurta: decide sensul conexiunii
            if d0 < d1:
                start_node[lid] = w
            else:
                end_node[lid] = w
    for f, t, c in walk_conn:
        ef, et = net.edges[f], net.edges[t]
        if ef.func == 'walkingarea' and et.func not in ('crossing', 'walkingarea'):
            start_node.setdefault('%s_%s' % (t, c.get('toLane')), f)
        elif et.func == 'walkingarea' and ef.func not in ('crossing', 'walkingarea'):
            end_node.setdefault('%s_%s' % (f, c.get('fromLane')), t)
    for ed in net.edges.values():
        if ed.func:
            continue
        for ln in ed.lanes:
            if 'pedestrian' not in ln.allow:
                continue
            a = start_node.get(ln.id, 'cap0:' + ln.id)
            b = end_node.get(ln.id, 'cap1:' + ln.id)
            net.ped_lane_ends[ln.id] = (a, b)
            adj[a].append((b, ln.length, ('lane', ln.id, 1)))
            adj[b].append((a, ln.length, ('lane', ln.id, -1)))
    for cr in net.crossings.values():
        ws = set()
        for f, t, c in walk_conn:
            if t == cr.id and net.edges[f].func == 'walkingarea':
                ws.add(f)
            if f == cr.id and net.edges[t].func == 'walkingarea':
                ws.add(t)
        p0, p1 = cr.shape[0], cr.shape[-1]
        near = {}
        for w in ws:
            wl = net.edges[w].lanes[0]
            d0 = min(math.hypot(p0[0] - x, p0[1] - y) for x, y in wl.shape)
            d1 = min(math.hypot(p1[0] - x, p1[1] - y) for x, y in wl.shape)
            near[w] = (d0, d1)
        if len(near) < 2:
            continue
        w0 = min(near, key=lambda w: near[w][0])
        w1 = min((w for w in near if w != w0), key=lambda w: near[w][1])
        cr.ends = (w0, w1)
        adj[w0].append((w1, cr.length + 1.0, ('cross', cr.id, 1)))
        adj[w1].append((w0, cr.length + 1.0, ('cross', cr.id, -1)))
    net.ped_adj = dict(adj)


def ped_route(net, from_edge, to_edge):
    """drumul unui pieton: lista de segmente (tip, id, sens, s_start, s_end)"""
    fl = [l for l in net.edges[from_edge].lanes if 'pedestrian' in l.allow] if from_edge in net.edges else []
    tl = [l for l in net.edges[to_edge].lanes if 'pedestrian' in l.allow] if to_edge in net.edges else []
    if not fl or not tl:
        return None
    fl, tl = fl[0], tl[0]
    sm, tm = fl.length / 2, tl.length / 2
    if fl is tl:
        return [('lane', fl.id, 1, sm, sm + 0.1)]
    a0, a1 = net.ped_lane_ends[fl.id]
    b0, b1 = net.ped_lane_ends[tl.id]
    dist, prev = {a0: sm, a1: fl.length - sm}, {a0: None, a1: None}
    first = {a0: ('lane', fl.id, -1, sm, 0.0), a1: ('lane', fl.id, 1, sm, fl.length)}
    pq = [(sm, a0), (fl.length - sm, a1)]
    goals = {b0: tm, b1: tl.length - tm}
    best, best_node = 1e18, None
    while pq:
        d, n = heapq.heappop(pq)
        if d > dist.get(n, 1e18) or d >= best:
            continue
        if n in goals and d + goals[n] < best:
            best, best_node = d + goals[n], n
        for n2, c, seg in net.ped_adj.get(n, ()):
            if seg[0] == 'lane' and seg[1] in (fl.id, tl.id):
                continue
            nd = d + c
            if nd < dist.get(n2, 1e18):
                dist[n2] = nd
                prev[n2] = (n, seg)
                heapq.heappush(pq, (nd, n2))
    if best_node is None:
        return None
    segs = []
    n = best_node
    while prev.get(n):
        p, seg = prev[n]
        kind, sid, sens = seg
        L = net.lanes[sid].length if kind == 'lane' else net.edges[sid].lanes[0].length
        segs.append((kind, sid, sens, 0.0 if sens > 0 else L, L if sens > 0 else 0.0))
        n = p
    segs.append(first[n])
    segs.reverse()
    segs.append(('lane', tl.id, 1, 0.0, tm) if best_node == b0 else ('lane', tl.id, -1, tl.length, tm))
    return segs


# ----------------------------------------------------------------------------- masini
def build_routing(net):
    """graful muchiilor pentru masini/tramvaie: edge -> [(edge urmator, Link-uri)]"""
    g = defaultdict(lambda: defaultdict(list))
    for lk in net.links:
        if lk.is_ped:
            continue
        g[lk.frm.edge.id][lk.to.edge.id].append(lk)
    net.succ = {e: dict(v) for e, v in g.items()}


def car_route(net, frm, to, vclass='passenger', tt=None):
    if frm not in net.edges or to not in net.edges:
        return None
    if frm == to:
        return [frm]
    tt = tt or {}
    dist, prev = {frm: 0.0}, {frm: None}
    pq = [(0.0, frm)]
    while pq:
        d, e = heapq.heappop(pq)
        if e == to:
            break
        if d > dist[e]:
            continue
        for e2, lks in net.succ.get(e, {}).items():
            if not any(vclass in lk.to.allow for lk in lks):
                continue
            ed = net.edges[e2]
            c = tt.get(e2) or ed.length / max(1.0, ed.speed)
            c += lks[0].via_len / max(1.0, lks[0].speed)
            nd = d + c
            if nd < dist.get(e2, 1e18):
                dist[e2] = nd
                prev[e2] = e
                heapq.heappush(pq, (nd, e2))
    if to not in prev:
        return None
    out, e = [], to
    while e is not None:
        out.append(e)
        e = prev[e]
    return out[::-1]
