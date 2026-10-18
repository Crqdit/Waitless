# -*- coding: utf-8 -*-
"""
Controlul trecerilor de pietoni. Fiecare trecere poate fi:
  original    - cum e in retea (zebra sau semaforul din program)
  zebra       - fara semafor, pietonul are prioritate
  semafor     - semafor normal: unde exista deja semafor, programul original; in rest ciclu fix
  inteligent  - sistemul nostru: placi de presiune + senzori + regulile alese
"""
import math, random, copy, zlib
from collections import defaultdict, OrderedDict, deque

MODES = OrderedDict([('original', 'Original'), ('zebra', 'Zebra'), ('semafor', 'Semafor normal'),
                     ('inteligent', 'Sistemul nostru')])

# ----------------------------------------------------------------------------- reguli
RULE_DEFAULTS = OrderedDict([
    # placile de presiune
    ('detectie', 'dale_greutate'),   # perfect | dale_greutate | doar_greutate | doar_dale
    ('dwell', 1.0),                  # cat trebuie sa stai pe placa ca sa fii numarat (s)
    ('kg_minim_obiect', 15.0),       # grupuri mai usoare sunt ignorate (genti, caini mici)
    ('kg_minim_persoana', 20.0),     # un grup nu poate fi mai multe persoane decat kg / asta
    ('kg_om_mediu', 75.0),           # pentru "doar greutate"
    ('placa_coloane', 6), ('placa_randuri', 4), ('dala_m', 0.5),
    # countdown
    ('countdown_start', 25.0), ('secunde_per_persoana', 2.0), ('countdown_min', 10.0),
    # semafor
    ('verde_minim_masini', 12.0), ('galben', 3.0), ('tot_rosu', 1.0), ('walk', 8.0), ('clear', 4.0),
    # senzorii din carosabil
    ('senzor0', True), ('senzor1', True), ('senzor2', True),
    ('senzor1_m', 25.0), ('senzor0_m', 60.0),
    ('senzori_numara', True),        # False = senzorii stiu doar "ocupat / liber"
    ('senzor_rateaza', 0.0),         # probabilitatea ca un senzor sa nu vada o masina
    ('zona_viraj_max_m', 120.0),     # mai departe de atat nu mai e "zona de viraj"
    ('m_per_masina', 7.0),
    # masinile de pe senzori cand se termina countdown-ul
    ('hold', 'cap12'),               # none | cap60 | cap12
    ('hold_cap', 12.0), ('hold_total_cap', 60.0),
    # fix-uri
    ('fix2', False), ('min_walk', 5.0), ('short_clear', 1.0),
    ('fix3', False), ('pair_max_wait', 5.0),
    ('fix4', False), ('gap_needed', 2.5), ('gap_max_wait', 4.0),
    ('fix5', False), ('pull_a_within', 15.0), ('spill_risk_ok', 0.10), ('smart_max_wait', 8.0),
    ('turn_rate_window', 180.0), ('turn_rate_prior', 0.07),
    ('fix6', False), ('empty_road_m', 60.0), ('empty_road_wait', 2.0),
    ('fix7', False), ('smart_clear_min', 1.0), ('smart_clear_max', 10.0),
    ('fix8', False), ('max_wait', 45.0),
])

RULE_LABELS = OrderedDict([
    ('detectie', 'Cum numara placa oamenii'),
    ('dwell', 'Timp pe placa pana esti numarat (s)'),
    ('kg_minim_obiect', 'Ignora obiecte sub (kg)'),
    ('kg_minim_persoana', 'Minim kg pe persoana'),
    ('kg_om_mediu', 'Om mediu (kg) - pentru "doar greutate"'),
    ('placa_coloane', 'Dale pe latime'), ('placa_randuri', 'Dale in adancime'), ('dala_m', 'Marimea dalei (m)'),
    ('countdown_start', 'Countdown de pornire (s)'), ('secunde_per_persoana', 'Minus secunde per persoana'),
    ('countdown_min', 'Countdown minim (s)'),
    ('verde_minim_masini', 'Verde minim pentru masini intre treceri (s)'),
    ('galben', 'Galben (s)'), ('tot_rosu', 'Tot rosu (s)'), ('walk', 'Verde pietoni (s)'),
    ('clear', 'Verde intermitent / eliberare (s)'),
    ('senzor0', 'Senzor 0 (inainte de viraj, 25-60 m)'), ('senzor1', 'Senzor 1 (banda comuna, ultimii 25 m)'),
    ('senzor2', 'Senzor 2 (zona de viraj, pana la trecere)'),
    ('senzor1_m', 'Lungime senzor 1 (m)'), ('senzor0_m', 'Senzor 0 pana la (m)'),
    ('senzori_numara', 'Senzorii numara masinile (altfel doar ocupat/liber)'),
    ('senzor_rateaza', 'Probabilitate ca senzorul sa rateze o masina'),
    ('zona_viraj_max_m', 'Zona de viraj maxima (m)'), ('m_per_masina', 'Metri per masina in coada'),
    ('hold', 'Masinile de pe senzori cand se termina countdown-ul'),
    ('hold_cap', 'Hold maxim (s)'), ('hold_total_cap', 'Hold: asteptare totala maxima (s)'),
    ('fix2', 'Fix 2 - termina verdele devreme daca trecerea e goala'),
    ('min_walk', '   verde minim (s)'), ('short_clear', '   eliberare scurta (s)'),
    ('fix3', "Fix 3 - B asteapta verdele lui A (perechi)"), ('pair_max_wait', '   astepta maxim (s)'),
    ('fix4', 'Fix 4 - asteapta o pauza in trafic (senzor 0)'), ('gap_needed', '   pauza necesara (s)'),
    ('gap_max_wait', '   asteptare maxima (s)'),
    ('fix5', 'Fix 5 - coordonat, tine cont de spatiul de viraj'), ('pull_a_within', '   trage A inainte daca e in (s)'),
    ('spill_risk_ok', '   risc de blocare acceptat'), ('smart_max_wait', '   asteptare maxima (s)'),
    ('turn_rate_window', '   invata rata de viraj din ultimele (s)'), ('turn_rate_prior', '   rata de start (masini/s)'),
    ('fix6', 'Fix 6 - drum gol: verde imediat'), ('empty_road_m', '   nicio masina pe (m)'),
    ('empty_road_wait', '   verde dupa (s)'),
    ('fix7', 'Fix 7 - eliberare inteligenta'), ('smart_clear_min', '   minim (s)'), ('smart_clear_max', '   maxim (s)'),
    ('fix8', 'Fix 8 - asteptare maxima garantata'), ('max_wait', '   nimeni nu asteapta mai mult de (s)'),
])
CHOICES = {
    'detectie': OrderedDict([('perfect', 'Perfect (numar exact)'), ('dale_greutate', 'Dale + greutate'),
                             ('doar_greutate', 'Doar greutate'), ('doar_dale', 'Doar dale')]),
    'hold': OrderedDict([('none', 'Fara hold'), ('cap60', 'Hold, 60 s total'), ('cap12', 'Hold, 12 s maxim')]),
}

SEMAFOR_DEFAULTS = OrderedDict([('verde_masini', 45.0), ('galben', 3.0), ('tot_rosu', 2.0), ('walk', 10.0),
                                ('clear', 5.0), ('la_cerere', False)])

FOOTPRINTS = {   # tip: (dale fata de ancora, kg mediu, +/- kg) - ca in crossing_sim.py
    'Adult': ([(0, 0), (1, 0)], 75, 12),
    'Child': ([(0, 0)], 25, 6),
    'Heavy adult': ([(0, 0), (2, 0)], 150, 25),
    'Bag / object': ([(0, 0)], 8, 3),
    'Dog': ([(0, 0)], 20, 8),
}
PRESETS_RULES = OrderedDict([
    ('Sistemul tau (hold 12 s)', {}),
    ('Sistemul tau + fix 5', dict(fix5=True)),
    ('Sistemul tau + fix 6, 7, 8', dict(fix6=True, fix7=True, fix8=True)),
    ('Fix 5 + 6 + 7 + 8', dict(fix5=True, fix6=True, fix7=True, fix8=True)),
    ('Fara hold', dict(hold='none')),
])
DEFAULT_PAIRS = [[':12043430980_c0', ':6413766796_c0'], [':10616411210_c0', ':10616411211_c0']]


def rules_with(over):
    r = OrderedDict(RULE_DEFAULTS)
    r.update(over or {})
    return r


def countdown_for(n, R):
    return max(R['countdown_min'], R['countdown_start'] - R['secunde_per_persoana'] * n)


# ----------------------------------------------------------------------------- placi de presiune
class Plate(object):
    """o placa cu dale la un capat al trecerii"""

    def __init__(self, cols, rows, size, origin, ux, uy):
        self.cols, self.rows, self.size = cols, rows, size
        self.origin, self.ux, self.uy = origin, ux, uy     # ux = de-a lungul bordurii, uy = spre trotuar
        self.items = []                                     # (obiect, dale, kg, t_sosire, este_persoana)

    def tile_poly(self, c, r, shrink=0.9):
        """cele 4 colturi ale dalei, rotite odata cu trecerea"""
        s = self.size
        ox, oy = self.origin
        a0, a1 = (c - self.cols / 2.0) * s, (c + 1 - self.cols / 2.0) * s
        b0, b1 = r * s, (r + 1) * s
        m = s * (1 - shrink) / 2
        a0, a1, b0, b1 = a0 + m, a1 - m, b0 + m, b1 - m
        return [(ox + self.ux[0] * a + self.uy[0] * b, oy + self.ux[1] * a + self.uy[1] * b)
                for a, b in ((a0, b0), (a1, b0), (a1, b1), (a0, b1))]

    def tile_xy(self, c, r):
        s = self.size
        ox, oy = self.origin
        a = (c + 0.5 - self.cols / 2.0) * s
        b = (r + 0.5) * s
        return ox + self.ux[0] * a + self.uy[0] * b, oy + self.ux[1] * a + self.uy[1] * b

    def corners(self):
        s = self.size
        ox, oy = self.origin
        pts = []
        for a, b in ((-self.cols / 2.0, 0), (self.cols / 2.0, 0), (self.cols / 2.0, self.rows), (-self.cols / 2.0, self.rows)):
            pts.append((ox + (self.ux[0] * a + self.uy[0] * b) * s, oy + (self.ux[1] * a + self.uy[1] * b) * s))
        return pts

    def place(self, owner, kind, kg, t, rng, person=True):
        shape = FOOTPRINTS[kind][0]
        used = {tile for it in self.items for tile in it[1]}
        spots = []
        for r in range(self.rows):
            for c in range(self.cols):
                tiles = [(c + dc, r + dr) for dc, dr in shape]
                if all(0 <= x < self.cols and 0 <= y < self.rows and (x, y) not in used for x, y in tiles):
                    spots.append(tiles)
        if not spots:
            return None
        spots.sort(key=lambda ts: ts[0][1])              # cat mai aproape de bordura
        best_row = spots[0][0][1]
        near = [s for s in spots if s[0][1] <= best_row + 1]
        tiles = rng.choice(near)
        self.items.append((owner, tiles, kg, t, person))
        return tiles

    def remove(self, owner):
        self.items = [it for it in self.items if it[0] is not owner]

    def load(self, t, dwell):
        out = {}
        for owner, tiles, kg, t0, person in self.items:
            if t - t0 < dwell:
                continue
            for tile in tiles:
                out[tile] = out.get(tile, 0.0) + kg / len(tiles)
        return out

    def count(self, t, R):
        mode = R['detectie']
        if mode == 'perfect':
            return sum(1 for it in self.items if it[4] and t - it[3] >= R['dwell'])
        load = self.load(t, R['dwell'])
        if mode == 'doar_greutate':
            kg = sum(load.values())
            return int(kg / R['kg_om_mediu'] + 0.5) if kg >= R['kg_minim_obiect'] else 0
        seen, clusters = set(), []
        for start in load:
            if start in seen:
                continue
            seen.add(start)
            stack, tiles = [start], []
            while stack:
                c, r = stack.pop()
                tiles.append((c, r))
                for dc in (-1, 0, 1):
                    for dr in (-1, 0, 1):
                        nb = (c + dc, r + dr)
                        if nb in load and nb not in seen:
                            seen.add(nb)
                            stack.append(nb)
            clusters.append((tiles, sum(load[x] for x in tiles)))
        if mode == 'doar_dale':
            return len(clusters)
        clusters = [cl for cl in clusters if cl[1] >= R['kg_minim_obiect']]
        n = 0
        for tiles, kg in clusters:
            n += min(max(1, int(len(tiles) / 2 + 0.5)), max(1, int(kg // R['kg_minim_persoana'])))
        singles = [cl for cl in clusters if len(cl[0]) == 1]
        used = set()
        for i in range(len(singles)):
            for j in range(i + 1, len(singles)):
                if i in used or j in used:
                    continue
                (a,), (b,) = singles[i][0], singles[j][0]
                if max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 2 and 40.0 <= singles[i][1] + singles[j][1] <= 250.0:
                    used |= {i, j}
                    n -= 1
        return n


# ----------------------------------------------------------------------------- zonele senzorilor
def sensor_zones(net, cr, R):
    """distanta pana la trecere pentru fiecare banda din amonte; detecteaza punctul de viraj (diverge)"""
    incoming = defaultdict(list)
    for lk in net.links:
        if not lk.is_ped:
            incoming[lk.to.id].append(lk)
    maxd = R['zona_viraj_max_m'] + R['senzor0_m'] + 5
    dist = {}                        # lane id -> distanta de la capatul benzii pana la trecere
    diverge = []
    stack = []
    for lk in cr.car_links:
        cpos = lk.over_crossing[1] if lk.over_crossing else 0.0
        # benzile interne ale legaturii, pana la trecere
        acc = 0.0
        for vl in lk.via:
            dist[vl.id] = min(dist.get(vl.id, 1e9), cpos - acc - vl.length)
            acc += vl.length
        d = cpos
        stack.append((lk.frm, d, lk))
    while stack:
        lane, d, lk_out = stack.pop()
        if d > maxd or d >= dist.get(lane.id, 1e9):
            continue
        dist[lane.id] = d
        others = {l.to.edge.id for l in lane.links} - {lk_out.to.edge.id}
        if others and lane.kind == 'normal':
            diverge.append(d)
        dd = d + lane.length
        for k in incoming.get(lane.id, ()):
            for vl in k.via:
                pass
            acc = 0.0
            for vl in reversed(k.via):
                dist[vl.id] = min(dist.get(vl.id, 1e9), dd + acc)
                acc += vl.length
            stack.append((k.frm, dd + k.via_len, k))
    near = [x for x in diverge if x <= R['zona_viraj_max_m']]
    storage = min(near) if near else None
    return dist, storage


class Zones(object):
    def __init__(self, net, cr, R):
        self.dist, self.storage = sensor_zones(net, cr, R)
        self.lanes = {lid: net.lanes[lid] for lid in self.dist}
        st = self.storage or 0.0
        s1 = R['senzor1_m']
        self.ranges = dict(area=(0.0, st) if self.storage else None, s1=(st, st + s1),
                           s0=(st + s1, st + R['senzor0_m']))
        self.cap = int(st // R['m_per_masina']) + 1 if self.storage else 0

    def read(self, which, miss=0.0, t=0.0):
        rg = self.ranges.get(which)
        if rg is None:
            return set()
        a, b = rg
        out = set()
        for lid, d in self.dist.items():
            lane = self.lanes[lid]
            L = lane.length
            for v in lane.vehs:
                # distanta pana la trecere: fata si spatele vehiculului
                front = d + (L - v.pos)
                rear = front + v.L
                if rear >= a and front <= b:
                    if miss > 0 and (zlib.crc32(('%s|%s' % (v.id, which)).encode()) % 1000) / 1000.0 < miss:
                        continue
                    out.add(v)
        return out


# ----------------------------------------------------------------------------- controlere
class Base(object):
    mode = 'original'

    def __init__(self, sim, cr):
        self.sim, self.cr = sim, cr
        self.state = 'GREEN'
        self.detected = 0
        self.waiting = set()
        self.events = deque(maxlen=60)

    def log(self, msg):
        if not self.events or self.events[-1][1] != msg:
            self.events.append((self.sim.t, msg))

    def ped_arrives(self, p):
        self.waiting.add(p)

    def ped_leaves(self, p):
        self.waiting.discard(p)

    def update(self, t):
        pass

    def apply(self, overlay, ped_overlay):
        pass

    def text(self):
        return MODES[self.mode]


class Original(Base):
    mode = 'original'


class Zebra(Base):
    mode = 'zebra'

    def apply(self, overlay, ped_overlay):
        ped_overlay[self.cr.id] = 'Z'
        if self.cr.tl is not None:
            for lk in self.cr.car_links:
                overlay[lk.num] = 'g'


class Fixed(Base):
    """semafor normal cu ciclu fix (doar unde nu exista deja semafor)"""
    mode = 'semafor'

    def __init__(self, sim, cr, P, offset=0.0):
        Base.__init__(self, sim, cr)
        self.P = P
        self.t0 = -offset
        self.state = 'GREEN'
        self.requested = False

    def update(self, t):
        P = self.P
        el = t - self.t0
        if self.waiting:
            self.requested = True
        seq = [('GREEN', P['verde_masini']), ('AMBER', P['galben']), ('ALLRED', P['tot_rosu']),
               ('WALK', P['walk']), ('CLEAR', P['clear'])]
        d = dict(seq)
        if el >= d[self.state]:
            if self.state == 'GREEN' and P['la_cerere'] and not self.requested:
                return
            nxt = {'GREEN': 'AMBER', 'AMBER': 'ALLRED', 'ALLRED': 'WALK', 'WALK': 'CLEAR', 'CLEAR': 'GREEN'}[self.state]
            if nxt == 'WALK':
                self.requested = False
            self.state, self.t0 = nxt, t
            self.log({'GREEN': 'verde masini', 'AMBER': 'galben (ciclu fix)', 'ALLRED': 'tot rosu',
                      'WALK': 'VERDE pietoni', 'CLEAR': 'eliberare'}[nxt])

    def apply(self, overlay, ped_overlay):
        car = {'AMBER': 'y', 'ALLRED': 'r', 'WALK': 'r', 'CLEAR': 'r'}.get(self.state, 'G')
        if car == 'G' and self.sim.cross_peds.get(self.cr.id):
            car = 'g'
        for lk in self.cr.car_links:
            overlay[lk.num] = car
        ped_overlay[self.cr.id] = 'G' if self.state == 'WALK' else 'r'

    def text(self):
        return 'Semafor: ' + {'GREEN': 'masini', 'AMBER': 'galben', 'ALLRED': 'tot rosu', 'WALK': 'VERDE pietoni',
                              'CLEAR': 'eliberare'}[self.state]


class Smart(Base):
    """sistemul nostru (regulile din crossing_sim.py / smart_crossing.py)"""
    mode = 'inteligent'
    TEXT = {'GREEN': 'verde masini', 'SMARTWAIT': 'asteapta moment sigur', 'PAIRWAIT': 'asteapta A',
            'GAPWAIT': 'asteapta pauza', 'HOLD': 'hold pentru masini', 'AMBER': 'galben', 'ALLRED': 'tot rosu',
            'WALK': 'VERDE pietoni', 'CLEAR': 'eliberare'}

    def __init__(self, sim, cr, R, rng):
        Base.__init__(self, sim, cr)
        self.R, self.rng = R, rng
        self.t_state, self.wait_start, self.last_end = 0.0, None, -1e9
        self.committed, self.short_clear = set(), False
        self.partner = self.b_of = None
        self.zones = Zones(sim.net, cr, R)
        self.turn_log, self.seen = [], set()
        self.area, self.s1, self.s0 = set(), set(), set()
        self.last_s0 = -1e9
        self.first_wait = None
        self.st = defaultdict(float)
        self.plates = self.make_plates()
        self.actual = 0

    # --- placi la ambele capete ale trecerii
    def make_plates(self):
        """cate o placa la fiecare capat; pe refugiile inguste placa e mai scurta ca sa nu intre pe trecerea vecina"""
        R = self.R
        sh = self.cr.shape
        (x0, y0), (x1, y1) = sh[0], sh[-1]
        L = math.hypot(x1 - x0, y1 - y0) or 1.0
        dx, dy = (x1 - x0) / L, (y1 - y0) / L
        px, py = -dy, dx
        out = []
        for (ox, oy), (ux, uy) in (((x0, y0), (-dx, -dy)), ((x1, y1), (dx, dy))):
            rows = int(R['placa_randuri'])
            for k in range(1, rows + 1):
                d = (k - 0.15) * R['dala_m']
                q = (ox + ux * d, oy + uy * d)
                if self._on_other_crossing(q):
                    rows = k - 1
                    break
            out.append(Plate(R['placa_coloane'], max(1, rows), R['dala_m'], (ox, oy), (px, py), (ux, uy)))
        return out

    def _on_other_crossing(self, q):
        from retea import dist_point_seg
        for cid, cr in self.sim.net.crossings.items():
            if cid == self.cr.id:
                continue
            a, b = cr.shape[0], cr.shape[-1]
            if dist_point_seg(q, a, b) < cr.width / 2.0 + 0.1:
                return True
        return False

    def plate_for(self, p):
        seg = p.segs[p.si + 1] if p.si + 1 < len(p.segs) else None
        sens = seg[2] if seg else 1
        return self.plates[0] if sens > 0 else self.plates[1]

    def ped_arrives(self, p):
        Base.ped_arrives(self, p)
        pl = self.plate_for(p)
        tiles = pl.place(p, p.kind, p.kg, self.sim.t, self.rng, True)
        p.spot = (pl, tiles)
        if p.obj and tiles:
            pl.place(('obj', p), p.obj[0], p.obj[1], self.sim.t, self.rng, False)

    def ped_leaves(self, p):
        Base.ped_leaves(self, p)
        for pl in self.plates:
            pl.remove(p)
            pl.items = [it for it in pl.items if not (isinstance(it[0], tuple) and it[0][1] is p)]
        p.spot = None

    # --- senzori
    def sense(self, t):
        R, z = self.R, self.zones
        miss = R['senzor_rateaza']
        self.area = z.read('area', miss) if R['senzor2'] else set()
        self.s1 = z.read('s1', miss) if R['senzor1'] else set()
        self.s0 = z.read('s0', miss) if R['senzor0'] else set()
        if self.s0:
            self.last_s0 = t
        for v in self.area:
            if v.id not in self.seen:
                self.seen.add(v.id)
                self.turn_log.append(t)
        if len(self.seen) > 5000:
            self.seen = set(list(self.seen)[-500:])
        if z.storage and len(self.area) >= z.cap:
            self.st['area_full_s'] += 0.5
        stopped_turners = [v for v in self.area | self.s1 if v.v < 0.3]
        self.st['turners_stopped_s'] += 0.5 * len([v for v in self.area if v.v < 0.3])
        # masini drepte blocate in spatele unei masini care vireaza si sta
        if z.storage:
            for v in self.s1:
                if v.v < 0.3 and v not in self.area:
                    pass
            if any(v.v < 0.3 for v in self.s1) and len(self.area) >= z.cap - 1 and any(v.v < 0.3 for v in self.area):
                self.st['spill_s'] += 0.5

    def storage_count(self):
        if not self.R['senzori_numara']:
            return self.zones.cap if self.area else 0
        return len(self.area)

    def road_empty(self):
        m = self.R['empty_road_m']
        for lid, d in self.zones.dist.items():
            if d > m:
                continue
            lane = self.zones.lanes[lid]
            for v in lane.vehs:
                if d + lane.length - v.pos <= m:
                    return False
        return True

    # --- regulile
    def go_time(self):
        R = self.R
        cd = countdown_for(self.detected, R)
        if R['fix6'] and self.road_empty():
            return self.wait_start + R['empty_road_wait']
        t = max(self.wait_start + cd, self.last_end + R['verde_minim_masini'])
        if R['fix8'] and self.first_wait is not None:
            t = min(t, max(self.first_wait + R['max_wait'], self.last_end + R['galben']))
        return t

    def overdue(self):
        return self.R['fix8'] and self.first_wait is not None and self.sim.t - self.first_wait >= self.R['max_wait']

    def is_b(self):
        return bool(self.zones.storage)

    def turn_rate(self):
        R, t = self.R, self.sim.t
        w = R['turn_rate_window']
        recent = sum(1 for x in self.turn_log if x > t - w)
        return (recent + R['turn_rate_prior'] * 30.0) / (min(w, t) + 30.0)

    def spill_risk(self):
        R = self.R
        room = self.zones.cap - self.storage_count()
        if room < 0:
            return 1.0
        lam = self.turn_rate() * (R['galben'] + R['tot_rosu'] + R['min_walk'] + R['short_clear'] + 2.0)
        return 1.0 - sum(math.exp(-lam) * lam ** k / math.factorial(k) for k in range(room + 1))

    LOGTXT = {'GREEN': 'verde masini', 'SMARTWAIT': 'fix 5: asteapta un moment sigur', 'PAIRWAIT': 'fix 3: asteapta verdele lui A',
              'GAPWAIT': 'fix 4: asteapta o pauza in trafic', 'HOLD': 'HOLD', 'AMBER': 'galben pentru masini',
              'ALLRED': 'tot rosu', 'WALK': 'VERDE pietoni', 'CLEAR': 'eliberare (nimeni nu mai porneste)'}

    def go(self, s, why=''):
        if s != self.state:
            self.log(self.LOGTXT.get(s, s) + ((' - ' + why) if why else ''))
        self.state, self.t_state = s, self.sim.t

    def smart_decide(self, waited):
        R, A, t = self.R, self.partner, self.sim.t
        if self.overdue():
            return True
        if A is not None:
            if A.state in ('AMBER', 'ALLRED'):
                return True
            if A.state == 'GREEN' and A.wait_start is not None and A.go_time() - t <= R['pull_a_within']:
                if t >= A.last_end + A.R['verde_minim_masini']:
                    A.start_walk(join=False)
                    A.log('fix 5: pornit mai devreme, impreuna cu B')
                    self.log('fix 5: A (%s) porneste odata cu B' % A.cr.name)
                    self.st['shared_walks'] += 1
                    return True
                if waited < R['smart_max_wait']:
                    return False
        if not self.is_b():
            return True
        if self.spill_risk() <= R['spill_risk_ok']:
            return True
        if self.storage_count() == 0 and not self.s1:
            return True
        return waited >= R['smart_max_wait']

    def start_walk(self, join=True):
        if self.wait_start is None:
            self.wait_start = self.sim.t
        self.go('AMBER')
        self.st['walks'] += 1
        B = self.b_of
        if join and B is not None and (B.R['fix3'] or B.R['fix5']) and B.state in ('GREEN', 'SMARTWAIT', 'PAIRWAIT') \
                and B.detected > 0:
            if B.wait_start is None:
                B.wait_start = self.sim.t
            if B.R['fix5']:
                B.b_check()
            elif B.state == 'GREEN':
                B.b_after_pair()

    def b_begin(self):
        R, A = self.R, self.partner
        if R['fix5']:
            if self.smart_decide(0.0):
                self.b_check()
            else:
                self.go('SMARTWAIT', 'risc de blocare %.0f %%, zona de viraj %d/%d' % (
                    100 * self.spill_risk(), self.storage_count(), self.zones.cap))
        elif R['fix3'] and A is not None and A.state == 'GREEN' and A.wait_start is not None \
                and A.go_time() - self.sim.t <= R['pair_max_wait']:
            self.go('PAIRWAIT')
        else:
            self.b_after_pair()

    def b_after_pair(self):
        if self.R['fix4'] and not self.overdue():
            self.go('GAPWAIT')
        else:
            self.b_check()

    def b_check(self):
        cands = self.area | self.s1
        if self.R['hold'] == 'none' or not cands or self.overdue():
            self.go('AMBER', 'countdown gata, ' + ('nicio masina pe senzori' if not cands else
                                                  'fara hold' if self.R['hold'] == 'none' else 'fix 8: asteptare maxima'))
            self.st['walks'] += 1
            return
        self.committed = set(cands)
        na = len(cands & self.area)
        self.go('HOLD', '%d masini pe senzori (%d in zona de viraj, %d pe senzorul 1) trec intai' % (
            len(cands), na, len(cands) - na))
        self.st['holds'] += 1
        self.st['walks'] += 1

    def hold_done(self):
        R = self.R
        self.committed &= (self.area | self.s1)
        if not self.committed or self.overdue():
            return True
        if R['hold'] == 'cap60':
            return self.sim.t - (self.wait_start or self.t_state) >= R['hold_total_cap']
        return self.sim.t - self.t_state >= R['hold_cap']

    def gap_ok(self):
        return not self.s0 and self.sim.t - self.last_s0 >= self.R['gap_needed']

    def update(self, t):
        R = self.R
        self.sense(t)
        self.detected = sum(pl.count(t, R) for pl in self.plates)
        self.actual = len(self.waiting)
        fw = [p.wait_start for p in self.waiting if p.wait_start is not None]
        self.first_wait = min(fw) if fw else None
        walkers = len(self.sim.cross_peds.get(self.cr.id, ()))
        el = t - self.t_state
        s = self.state
        if s == 'GREEN':
            if self.detected != getattr(self, '_last_det', 0):
                if self.detected > 0:
                    self.log('placile: %d %s -> countdown %.0f s' % (
                        self.detected, 'persoana' if self.detected == 1 else 'persoane', countdown_for(self.detected, R)))
                self._last_det = self.detected
            if self.detected > 0:
                if self.wait_start is None:
                    self.wait_start = t
                if t >= self.go_time():
                    if self.is_b() or self.partner is not None:
                        self.b_begin()
                    else:
                        self.start_walk()
            else:
                self.wait_start = None
        elif s == 'SMARTWAIT':
            if self.smart_decide(el):
                self.b_check()
        elif s == 'PAIRWAIT':
            A = self.partner
            if A is None or A.state in ('AMBER', 'ALLRED', 'WALK') or el >= R['pair_max_wait'] or self.overdue():
                self.b_after_pair()
        elif s == 'GAPWAIT':
            if self.gap_ok() or el >= R['gap_max_wait'] or self.overdue():
                self.b_check()
        elif s == 'HOLD':
            if self.hold_done():
                self.st['hold_time'] += el
                self.go('AMBER', 'hold terminat dupa %.0f s (%s)' % (el, 'au trecut toate' if not self.committed else 'limita'))
        elif s == 'AMBER' and el >= R['galben']:
            self.go('ALLRED')
        elif s == 'ALLRED' and el >= R['tot_rosu']:
            self.go('WALK')
        elif s == 'WALK':
            early = R['fix2'] or (R['fix5'] and (self.is_b() or self.b_of is not None))
            empty = self.detected == 0 and walkers == 0
            if early and el >= R['min_walk'] and empty:
                self.short_clear = True
                self.log('trecerea e goala, verdele se termina devreme')
            elif R['fix5'] and self.is_b() and el >= R['min_walk'] and self.storage_count() >= self.zones.cap:
                self.short_clear = False
                self.st['spill_cuts'] += 1
                self.log('fix 5: zona de viraj plina, se opreste verdele pietonilor')
            elif el >= R['walk']:
                self.short_clear = False
            else:
                return
            self.go('CLEAR')
        elif s == 'CLEAR':
            if R['fix7']:
                done = (el >= R['smart_clear_min'] and walkers == 0) or el >= R['smart_clear_max']
            else:
                done = el >= (R['short_clear'] if self.short_clear else R['clear'])
            if done:
                self.go('GREEN')
                self.last_end, self.wait_start = t, None

    def apply(self, overlay, ped_overlay):
        car = {'AMBER': 'y', 'ALLRED': 'r', 'WALK': 'r', 'CLEAR': 'r'}.get(self.state, 'G')
        if car == 'G' and self.sim.cross_peds.get(self.cr.id):
            car = 'g'
        for lk in self.cr.car_links:
            overlay[lk.num] = car
        ped_overlay[self.cr.id] = 'G' if self.state == 'WALK' else 'r'

    def text(self):
        s = self.state
        if s == 'GREEN' and self.wait_start is not None:
            return 'countdown %.0f s' % max(0.0, self.go_time() - self.sim.t)
        return self.TEXT.get(s, s)


def install(sim, setup, seed=1):
    """pune controlerele pe treceri dupa setup:
       setup = {'treceri': {id: {'mod':..., 'reguli': nume}}, 'reguli': {nume: {...}}, 'perechi': [[A,B],..],
                'semafor': {...}, 'nerabdare': [40, 0.05] sau None, 'fara_faza_pietoni': True}"""
    rng = random.Random(7000 + seed)
    rules = {k: rules_with(v) for k, v in setup.get('reguli', {}).items()}
    if not rules:
        rules = {'Sistemul tau (hold 12 s)': rules_with({})}
    first = next(iter(rules))
    SP = OrderedDict(SEMAFOR_DEFAULTS)
    SP.update(setup.get('semafor', {}))
    sim.crossing_ctl = {}
    smart = {}
    for cid, cr in sim.net.crossings.items():
        cfg = setup.get('treceri', {}).get(cid, {})
        mode = cfg.get('mod', 'original')
        if mode == 'zebra':
            ctl = Zebra(sim, cr)
        elif mode == 'semafor':
            ctl = Original(sim, cr) if cr.tl is not None else Fixed(sim, cr, SP, offset=rng.uniform(0, 30))
            ctl.mode = 'semafor'
        elif mode == 'inteligent':
            R = rules.get(cfg.get('reguli', first), rules[first])
            ctl = Smart(sim, cr, R, random.Random(rng.random()))
            smart[cid] = ctl
        else:
            ctl = Original(sim, cr)
        sim.crossing_ctl[cid] = ctl
    for a, b in setup.get('perechi', DEFAULT_PAIRS):
        if a in smart and b in smart:
            smart[b].partner, smart[a].b_of = smart[a], smart[b]
    # semafoarele care au toate trecerile controlate altfel nu mai au nevoie de faza exclusiva pentru pietoni
    for tc in sim.tls_ctl.values():
        tc.skip_ped_only = False
    if setup.get('fara_faza_pietoni', True):
        for tid, tc in sim.tls_ctl.items():
            crs = [c for c in sim.net.crossings.values() if c.tl == tid]
            if crs and all(sim.crossing_ctl[c.id].mode in ('zebra', 'inteligent') for c in crs):
                tc.skip_ped_only = True
    sim.ped_impatience = tuple(setup['nerabdare']) if setup.get('nerabdare') else None
    return sim.crossing_ctl


def summary_smart(sim):
    tot = defaultdict(float)
    for ctl in sim.crossing_ctl.values():
        if isinstance(ctl, Smart):
            for k, v in ctl.st.items():
                tot[k] += v
    return tot
