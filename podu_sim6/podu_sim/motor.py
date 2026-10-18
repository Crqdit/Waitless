# -*- coding: utf-8 -*-
"""
Motorul simularii (fara SUMO). Reproduce modelele implicite SUMO acolo unde sunt documentate:
  * urmarire: Krauss (viteza sigura + "neatentia" sigma), pas de 0.5 s
  * intersectii: matricile de prioritate din podu_ros.net.xml (cine cedeaza cui), semafoare,
    "nu intra in intersectie daca nu ai loc sa iesi", cedeaza pietonilor pe zebre
  * schimbarea benzii: strategic (banda buna pentru traseu) + mers pe dreapta + castig de viteza
  * tramvaie cu statii, vehicule de urgenta care trec pe rosu cu viteza redusa
  * pietoni pe trotuare / zone de mers / treceri; asteapta la rosu
  * teleportare dupa timpul din nivel, rerutare (30 % din masini, la 120 s)
"""
import math, random, bisect, heapq
from collections import defaultdict

import retea as R

DT = 0.5
RANK = {'r': 0, 'R': 0, 'u': 0, 'y': 1, 'Y': 1, 'o': 2, 'O': 2, 'g': 2, 'm': 2, '=': 2, 'G': 3, 'M': 3, 's': 2}
INF = 1e18


def vsafe(gap, v_lead, b, tau):
    """viteza sigura Krauss"""
    if gap <= 0:
        return 0.0
    bt = b * tau
    return -bt + math.sqrt(bt * bt + v_lead * v_lead + 2.0 * b * gap)


class Veh(object):
    __slots__ = ('id', 'kind', 'vt', 'vclass', 'L', 'mingap', 'a', 'b', 'be', 'tau', 'sigma', 'vmaxv', 'sf',
                 'route', 'ri', 'lane', 'pos', 'v', 'link', 'via_off', 'good', 'goodfor', 'wait', 'wait_total',
                 'depart', 'inserted', 'timeloss', 'dist', 'stops', 'dwell_until', 'brk_until', 'lc_cd',
                 'emergency', 'reroute', 'next_rr', 'dest', 'color', 'line', 'tele', 'want', 'planned', 'loops')

    def __init__(self):
        self.link, self.via_off, self.good, self.goodfor = None, 0.0, None, None
        self.wait = self.wait_total = self.timeloss = self.dist = 0.0
        self.stops, self.dwell_until, self.brk_until, self.lc_cd = [], -1.0, -1.0, 0.0
        self.emergency = self.reroute = False
        self.next_rr, self.tele, self.want, self.planned = INF, 0, 0.0, None
        self.loops = 0


class Ped(object):
    __slots__ = ('id', 'segs', 'si', 's', 'speed', 'state', 'wait_start', 'waited', 'depart', 'timeloss',
                 'kind', 'kg', 'xy', 'cross', 'spot', 'obj', 'red', 'cpos', 'dist', 'impatient_ok')


class OSet(dict):
    """multime care pastreaza ordinea (ca simularea sa fie identica la acelasi seed)"""
    def add(self, x):
        self[x] = None

    def discard(self, x):
        self.pop(x, None)


class Sim(object):
    def __init__(self, net, demand, seed=1, end=4200.0, ped_impatience=None, log=print):
        self.net, self.dem, self.end = net, demand, end
        self.rng = random.Random(seed)
        self.t = 0.0
        self.vehs, self.peds = [], []
        self.pending = list(demand['cars'])
        self.pending_peds = list(demand['peds'])
        self.pending_trams = list(demand['trams'])
        self.insert_q = []
        self.vt = demand['vtypes']
        self.teleport_time = demand.get('teleport', 600)
        self.ped_impatience = ped_impatience        # None sau (dupa_s, probabilitate_pe_s)
        self.tls_state = {}
        self.tls_ctl = {}
        self.overlay = {}                            # link.num -> stare impusa de controlerul trecerii
        self.ped_overlay = {}                        # crossing id -> 'G' / 'r'
        self.crossing_ctl = {}                       # crossing id -> controler (treceri.py)
        self.cross_peds = defaultdict(OSet)
        self.active_lanes = OSet()
        self.occ_links = OSet()
        self.good_cache = {}
        self.route_cache = {}
        self.edge_tt = {}
        self.stats = defaultdict(float)
        self.trip = []                               # vehicule ajunse: (durata, asteptare, timp pierdut, intarziere)
        self.ped_trip = []
        self.collided = {}
        self.ped_waits = defaultdict(list)           # crossing -> [secunde asteptate]
        self.series = []                             # pentru grafice: (t, masini in retea, pietoni care asteapta)
        self.log = log
        R.build_routing(net)
        for tid, t in net.tls.items():
            self.tls_ctl[tid] = TLSProgram(self, t)
        self.skip_ped_phase = set()

    # ---------------------------------------------------------------- semnale
    def lstate(self, lk):
        ov = self.overlay.get(lk.num)
        if lk.tl is not None:
            s = self.tls_state.get(lk.tl)
            st = s[lk.tl_index] if s and lk.tl_index < len(s) else 'G'
        else:
            st = lk.state
        if ov is not None and RANK.get(ov, 3) < RANK.get(st, 3):
            return ov
        return st

    def ped_state(self, cr):
        ov = self.ped_overlay.get(cr.id)
        if ov is not None:
            return ov
        if cr.tl is not None and cr.ped_links:
            return self.lstate(cr.ped_links[0])
        return 'Z'                                    # zebra: pietonul are prioritate

    # ---------------------------------------------------------------- benzi bune pentru traseu
    def good_lanes(self, v):
        key = (v.vclass,) + tuple(v.route[v.ri:v.ri + 80])
        if v.goodfor == (v.ri, id(v.route)):
            return v.good
        g = self.good_cache.get(key)
        if g is None:
            net = self.net
            edges, tot = [], 0.0
            for e in v.route[v.ri:]:
                edges.append(e)
                tot += net.edges[e].length
                if tot > 600 and len(edges) > 1:
                    break
            sets = [None] * len(edges)
            last = net.edges[edges[-1]]
            sets[-1] = {l.id for l in last.lanes if v.vclass in l.allow}
            for k in range(len(edges) - 2, -1, -1):
                ed = net.edges[edges[k]]
                nxt = edges[k + 1]
                s = set()
                anyl = set()
                for l in ed.lanes:
                    if v.vclass not in l.allow:
                        continue
                    for lk in l.links:
                        if lk.to.edge.id == nxt:
                            anyl.add(l.id)
                            if lk.to.id in sets[k + 1]:
                                s.add(l.id)
                sets[k] = s or anyl or {l.id for l in ed.lanes if v.vclass in l.allow}
            g = sets
            self.good_cache[key] = g
        v.good, v.goodfor = g, (v.ri, id(v.route))
        return g

    def choose_link(self, v, lane, ri):
        if ri + 1 >= len(v.route):
            return None
        nxt = v.route[ri + 1]
        cands = [lk for lk in lane.links if lk.to.edge.id == nxt and v.vclass in lk.to.allow]
        if not cands:
            return None
        if len(cands) == 1:
            return cands[0]
        g = v.good
        k = ri + 1 - v.ri
        if g is not None and 0 <= k < len(g):
            for lk in cands:
                if lk.to.id in g[k]:
                    return lk
        return cands[0]

    # ---------------------------------------------------------------- permisiunea de a intra
    def permit(self, v, lk, dist, margin=0.0):
        st = self.lstate(lk)
        stop_d = v.v * v.v / (2.0 * v.b)
        if st in 'rRu':
            if v.emergency:
                st = 'm'
            else:
                return False
        elif st in 'yY':
            if v.emergency:
                st = 'm'
            elif dist > stop_d + 0.5:
                return False
        if not lk.foes and not lk.resp:
            return True
        # pietonii: franeaza si de urgenta daca e nevoie
        hard_d = v.v * v.v / (2.0 * v.be)
        if dist > hard_d:
            for fo in lk.foes:
                if fo.is_ped and (fo in lk.resp or st not in 'GM'):
                    cp = lk.conf.get(fo.num)
                    for p in fo.occ:
                        if cp is None or abs(p.cpos - cp[1]) < 4.0 or fo in lk.resp:
                            return False
                    if fo in lk.resp:
                        for p, rec in fo.appr_prev.items():
                            if rec[2]:
                                return False
        committed = dist < stop_d - 1.0
        if committed:
            return True
        # cineva e deja in punctul de conflict
        for fo in lk.foes:
            if not fo.occ:
                continue
            cp = lk.conf.get(fo.num)
            if cp is None:
                continue
            if fo.is_ped:
                if fo.occ and lk.resp and fo in lk.resp:
                    return False
                if fo.occ:
                    for p in fo.occ:
                        if abs(p.cpos - cp[1]) < 4.0:
                            return False
                continue
            for o in fo.occ:
                if o is v:
                    continue
                along = o.via_off + o.pos
                if along < cp[1] + o.L + 0.5:
                    if lk.to is fo.to and cp[1] >= fo.via_len - 0.01:
                        # se unesc pe aceeasi banda: urmarirea rezolva, ceda doar cui e inainte
                        if along > 0.5:
                            return False
                        continue
                    return False
        if st not in 'GM' or (st == 'M' and lk.resp):
            vin = max(min(v.v, lk.speed), 2.0)
            t_arr = dist / max(v.v, 1.0)
            t_leave = t_arr + (lk.via_len + v.L) / vin
            for fo in lk.resp:
                if fo.is_ped:
                    for p, rec in fo.appr_prev.items():
                        if rec[2]:
                            return False
                    continue
                fst = self.lstate(fo)
                if fst in 'rRu':
                    continue
                mutual = lk in fo.resp
                for o, (ta, tl_, wp) in fo.appr_prev.items():
                    if o is v:
                        continue
                    if mutual:
                        if ta < t_arr - 0.2 or (abs(ta - t_arr) <= 0.2 and o.id < v.id):
                            if wp or ta < 3.0:
                                return False
                    elif wp and ta < t_leave + 1.0 + margin:
                        return False
        # loc dupa intersectie
        if lk.foes and lk.via:
            free = self.space_after(v, lk, v.L + v.mingap + 0.5)
            if free < v.L + v.mingap * 0.5:
                return False
        return True

    def intj_blocked(self, v, jid, dist):
        """viraj la stanga in doua etape: asteapta la jumatatea intersectiei daca vine cineva cu prioritate"""
        if dist < v.v * v.v / (2.0 * v.be) - 0.5:
            return False                      # nu mai poate opri
        for fid in self.net.intj[jid]:
            fl = self.net.lanes.get(fid)
            if fl is None:
                continue
            for o in fl.vehs:
                if o is not v and o.pos < fl.length - 0.5:
                    return True
            lk = fl.link
            if lk is not None:
                if self.lstate(lk) in 'rRu':
                    continue
                for o, (ta, tl_, wp) in lk.appr_prev.items():
                    if o is not v and wp and ta < 4.0:
                        return True
                for o in lk.occ:
                    if o is not v and o.lane is not fl and o.lane in lk.via and lk.via.index(o.lane) < lk.via.index(fl):
                        return True
        return False

    def space_after(self, v, lk, need):
        """spatiul liber dupa intersectie, pe drumul vehiculului, pana la primul vehicul lent"""
        acc = 0.0
        cur, ri = lk.to, v.ri + 1
        for _ in range(30):
            if cur.vehs:
                o = cur.vehs[0]
                if o.v > 1.0:
                    return INF
                return acc + o.pos - o.L
            acc += cur.length
            if acc >= need:
                return INF
            if cur.kind == 'internal':
                nx = cur.next_internal
                if nx is not None and nx.kind != 'internal':
                    ri += 1
            else:
                nk = self.choose_link(v, cur, ri)
                if nk is None:
                    return INF
                if nk.via:
                    nx = nk.via[0]
                else:
                    nx = nk.to
                    ri += 1
            if nx is None:
                return INF
            cur = nx
        return INF

    # ---------------------------------------------------------------- planificarea vitezei
    def plan(self, v, rnd):
        dt = DT
        lane = v.lane
        vlim = min(lane.speed * v.sf, v.vmaxv)
        vn = min(v.v + v.a * dt, vlim)
        if v.brk_until > self.t:
            return 0.0
        if v.dwell_until > self.t:
            return 0.0
        look = v.v * v.v / (2 * v.b) + v.v * 2.0 + 25.0
        vs = lane.vehs
        leader_found = False
        stopd = None                 # unde s-ar opri fata vehiculului (daca in fata e ceva oprit)
        lead_info = None             # (distanta pana la cel din fata, viteza lui)
        czones = []                  # trecerile de pietoni de pe drum: (inceput, sfarsit) fata de bara
        idx = vs.index(v) if len(vs) > 1 else 0
        if idx + 1 < len(vs):
            o = vs[idx + 1]
            gap = o.pos - o.L - v.pos - v.mingap
            vn = min(vn, vsafe(gap, o.v, v.b, v.tau))
            leader_found = True
            lead_info = (gap, o.v)
            if o.v < 2.0:
                stopd = gap
        if lane.kind == 'internal' and lane.link is not None and lane.link.over_crossing:
            cid, cp = lane.link.over_crossing
            w = self.net.crossings[cid].width / 2.0 + 0.5
            along = v.via_off + v.pos
            if cp - w - along > 0:
                czones.append((cp - w - along, cp + w - along))
        dist = lane.length - v.pos
        cur, ri = lane, v.ri
        steps = 0
        stops = v.stops
        if stops and stops[0][0] == lane.id and v.pos < stops[0][1] + 0.5:
            d = stops[0][1] - v.pos
            vn = min(vn, vsafe(d, 0.0, v.b, v.tau))
        while dist < look and steps < 60 and not leader_found:
            steps += 1
            if cur.kind == 'internal':
                nx = cur.next_internal
                if nx is None:
                    break
                if nx.kind == 'internal' and nx.id in self.net.intj and self.intj_blocked(v, nx.id, dist):
                    vn = min(vn, vsafe(dist - 0.2, 0.0, v.b, v.tau))
                    stopd = dist if stopd is None else min(stopd, dist)
                    break
                if nx.kind != 'internal':
                    ri += 1
                cur = nx
            else:
                if ri + 1 >= len(v.route):
                    break
                lk = self.choose_link(v, cur, ri)
                if lk is None:
                    vn = min(vn, vsafe(dist - 0.3, 0.0, v.b, v.tau))
                    stopd = dist if stopd is None else min(stopd, dist)
                    break
                strict = 0.0
                for a_, b_ in czones:
                    if a_ > 0.3 and dist < b_ + v.L + 1.0:
                        strict = 3.0          # cedezi imediat dupa zebra: intra pe ea doar cu marja
                        break
                ok = self.permit(v, lk, dist, strict)
                if lk.foes or lk.resp:
                    t_arr = dist / max(v.v, 1.0)
                    lk.appr[v] = (t_arr, t_arr + (lk.via_len + v.L) / max(min(v.v, lk.speed), 2.0), ok)
                if not ok:
                    vn = min(vn, vsafe(dist - 0.2, 0.0, v.b, v.tau))
                    stopd = dist if stopd is None else min(stopd, dist)
                    break
                if lk.over_crossing:
                    cid, cp = lk.over_crossing
                    w = self.net.crossings[cid].width / 2.0 + 0.5
                    czones.append((dist + cp - w, dist + cp + w))
                if v.emergency and self.lstate(lk) in 'rRuyY':
                    vn = min(vn, math.sqrt(5.56 ** 2 + 2 * v.b * max(0.0, dist)))
                if lk.via:
                    cur = lk.via[0]
                else:
                    cur = lk.to
                    ri += 1
            if cur.vehs:
                o = cur.vehs[0]
                gap = dist + o.pos - o.L - v.mingap
                vn = min(vn, vsafe(gap, o.v, v.b, v.tau))
                leader_found = True
                lead_info = (gap, o.v)
                if o.v < 2.0:
                    stopd = gap if stopd is None else min(stopd, gap)
                break
            if stops and cur.id == stops[0][0]:
                vn = min(vn, vsafe(dist + stops[0][1], 0.0, v.b, v.tau))
            dist += cur.length
        # vehiculul anunta mereu urmatoarea intersectie, chiar daca are pe cineva in fata
        if lane.kind == 'normal' and lane.length - v.pos < 120:
            nl = self.choose_link(v, lane, v.ri)
            d0 = lane.length - v.pos
            hops = 0
            while nl is not None and not (nl.foes or nl.resp) and hops < 6 and nl.to.kind == 'normal':
                d0 += nl.via_len + nl.to.length
                nxt_l = nl.to
                nl = self.choose_link(v, nxt_l, v.ri + 1 + hops)
                hops += 1
            if nl is not None and (nl.foes or nl.resp) and v not in nl.appr:
                t_arr = d0 / max(v.v, 1.0)
                nl.appr[v] = (t_arr, t_arr + (nl.via_len + v.L) / max(min(v.v, nl.speed), 2.0),
                              self.lstate(nl) not in 'rRu' and v.v > 1.0)
        # nu intra pe trecerea de pietoni daca nu ai loc s-o eliberezi complet:
        # daca in fata e coada (sau trebuie sa cedezi imediat dupa zebra), opreste inaintea ei
        if czones:
            hard = v.v * v.v / (2.0 * v.be)
            for a, b in czones:
                if a <= 0.3:
                    continue
                blocked = False
                if stopd is not None and stopd > a and stopd - v.L < b:
                    blocked = True                     # m-as opri pe zebra
                elif lead_info is not None:
                    gap, lv = lead_info
                    if lv < 5.0 and gap < b + v.L:
                        blocked = True                 # cel din fata incetineste si nu am loc dupa zebra
                if blocked and a - 0.3 >= hard:
                    vn = min(vn, vsafe(a - 0.3, 0.0, v.b, v.tau))
                    break
        # limita de franare
        vmin = max(0.0, v.v - v.be * dt)
        if vn < vmin:
            vn = vmin
        # neatentia soferului (Krauss)
        if v.sigma > 0 and vn > 0:
            vn = max(0.0, vn - rnd * v.sigma * v.a * dt)
        return vn

    # ---------------------------------------------------------------- miscare
    def move(self, v, vn):
        dt = DT
        v.v = vn
        s = vn * dt
        v.pos += s
        v.dist += s
        lane = v.lane
        while v.pos > lane.length:
            over = v.pos - lane.length
            if lane.kind == 'internal':
                nx = lane.next_internal
                if nx is None:
                    self.arrive(v)
                    return
                if nx.kind == 'internal':
                    v.via_off += lane.length
                else:
                    lk = lane.link
                    if lk is not None:
                        lk.occ.discard(v)
                        if not lk.occ:
                            self.occ_links.discard(lk)
                    v.link, v.via_off = None, 0.0
                    v.ri += 1
            else:
                if v.ri + 1 >= len(v.route):
                    self.arrive(v)
                    return
                lk = self.choose_link(v, lane, v.ri)
                if lk is None:
                    v.pos, v.v = lane.length, 0.0
                    return
                if lk.via:
                    nx = lk.via[0]
                    lk.occ.add(v)
                    self.occ_links.add(lk)
                    v.link, v.via_off = lk, 0.0
                else:
                    nx = lk.to
                    v.ri += 1
            lane.vehs.remove(v)
            if not lane.vehs:
                self.active_lanes.discard(lane)
            v.lane = nx
            v.pos = over
            bisect.insort(nx.vehs, v, key=lambda o: o.pos)
            self.active_lanes.add(nx)
            lane = nx
            if nx.kind == 'normal':
                self.good_lanes(v)
                if v.stops and v.stops[0][0] != nx.id and v.stops[0][0] not in [l.id for l in nx.edge.lanes]:
                    pass
        if v.ri == len(v.route) - 1 and lane.kind == 'normal' and v.pos >= lane.length - 0.5:
            self.arrive(v)

    def arrive(self, v):
        if v.lane is not None:
            if v in v.lane.vehs:
                v.lane.vehs.remove(v)
                if not v.lane.vehs:
                    self.active_lanes.discard(v.lane)
        if v.link is not None:
            v.link.occ.discard(v)
            if not v.link.occ:
                self.occ_links.discard(v.link)
        v.lane = None
        if v in self._alive:
            self._alive.discard(v)
        if v.kind != 'tram':
            self.trip.append((self.t - v.depart, v.wait_total, v.timeloss, v.inserted - v.depart, v.kind))

    # ---------------------------------------------------------------- schimbarea benzii
    def lane_change(self, v):
        lane = v.lane
        if lane.kind != 'normal' or lane.length < 3.0 or v.dwell_until > self.t:
            return
        ed = lane.edge
        g = v.good[0] if v.good else None
        cur_ok = g is None or lane.id in g
        target = None
        urgent = False
        if not cur_ok:
            idxs = [l.index for l in ed.lanes if l.id in g and v.vclass in l.allow]
            if idxs:
                best = min(idxs, key=lambda k: abs(k - lane.index))
                d = 1 if best > lane.index else -1
                target = self._lane_by_index(ed, lane.index + d)
                urgent = True
        elif v.lc_cd <= 0:
            v.lc_cd = 2.0
            right = self._lane_by_index(ed, lane.index - 1)
            if right is not None and v.vclass in right.allow and (g is None or right.id in g):
                lo = self._neighbor(right, v.pos)
                if lo[0] is None or (lo[0].pos - lo[0].L - v.pos > 40):
                    target = right
            if target is None:
                vs = lane.vehs
                idx = vs.index(v)
                if idx + 1 < len(vs):
                    ld = vs[idx + 1]
                    desired = min(lane.speed * v.sf, v.vmaxv)
                    if ld.pos - ld.L - v.pos < 30 and ld.v < desired - 2.0:
                        for d in (1, -1):
                            nl = self._lane_by_index(ed, lane.index + d)
                            if nl is None or v.vclass not in nl.allow or (g is not None and nl.id not in g):
                                continue
                            lo = self._neighbor(nl, v.pos)
                            if lo[0] is None or lo[0].v > ld.v + 1.0:
                                target = nl
                                break
        if target is None or v.vclass not in target.allow:
            return
        lead, foll = self._neighbor(target, v.pos)
        if lead is not None:
            gap = lead.pos - lead.L - v.pos
            if gap < v.mingap * (0.5 if urgent else 1.0) or v.v > vsafe(gap - v.mingap * 0.5, lead.v, v.b, v.tau) + 1.0:
                return
        if foll is not None:
            gap = v.pos - v.L - foll.pos
            need = foll.mingap * (0.5 if urgent else 1.0)
            if gap < need or foll.v > vsafe(gap - need, v.v, foll.b, foll.tau) + (1.5 if urgent else 0.3):
                return
        lane.vehs.remove(v)
        if not lane.vehs:
            self.active_lanes.discard(lane)
        v.lane = target
        bisect.insort(target.vehs, v, key=lambda o: o.pos)
        self.active_lanes.add(target)
        self.stats['lane_changes'] += 1

    @staticmethod
    def _lane_by_index(ed, i):
        if 0 <= i < len(ed.lanes):
            return ed.lanes[i]
        return None

    @staticmethod
    def _neighbor(lane, pos):
        vs = lane.vehs
        k = bisect.bisect_left([o.pos for o in vs], pos)
        lead = vs[k] if k < len(vs) else None
        foll = vs[k - 1] if k > 0 else None
        return lead, foll

    # ---------------------------------------------------------------- inserare
    def make_vehicle(self, spec, kind):
        vt = dict(self.vt.get(spec['type']) or self.vt.get('masina') or {})
        if not vt:
            from cerere import DEFAULTS
            vt = dict(DEFAULTS['passenger'], vClass='passenger')
        v = Veh()
        v.id, v.kind, v.vt = spec['id'], kind, vt
        v.vclass = vt.get('vClass', 'passenger')
        v.emergency = v.vclass == 'emergency'
        if v.emergency:
            v.vclass = 'passenger'          # circula pe aceleasi benzi
        v.L, v.mingap = float(vt.get('length', 5.0)), float(vt.get('minGap', 2.5))
        v.a, v.b = float(vt.get('accel', 2.6)), float(vt.get('decel', 4.5))
        v.be = float(vt.get('emergencyDecel', 9.0))
        v.tau, v.sigma = max(0.5, float(vt.get('tau', 1.0))), float(vt.get('sigma', 0.5))
        from cerere import sample_dist
        v.sf = sample_dist(vt.get('speedFactor', '1.0'), self.rng)
        v.vmaxv = float(vt.get('maxSpeed', 55.56))
        v.depart = spec['depart']
        v.color = vt.get('color')
        v.line = spec.get('line', '')
        v.dest = spec.get('to')
        if kind == 'tram':
            v.route = list(spec['edges'])
            v.stops = [(st['lane'], st['end'], dur) for st, dur in spec.get('stops', [])]
        else:
            key = (spec['frm'], spec['to'])
            r = self.route_cache.get(key)
            if r is None:
                r = R.car_route(self.net, spec['frm'], spec['to'], 'passenger')
                self.route_cache[key] = r or []
            if not r:
                return None
            v.route = list(r)
            v.reroute = self.rng.random() < 0.3
            v.next_rr = self.t + 120.0
        v.ri = 0
        return v

    def try_insert(self, v):
        ed = self.net.edges[v.route[0]]
        self.good_lanes(v)
        g = v.good[0]
        lanes = [l for l in ed.lanes if v.vclass in l.allow and (l.id in g or v.kind == 'tram')]
        if not lanes:
            lanes = [l for l in ed.lanes if v.vclass in l.allow]
        best, bfree = None, -1.0
        for l in lanes:
            free = (l.vehs[0].pos - l.vehs[0].L) if l.vehs else INF
            if free > bfree:
                best, bfree = l, free
        if best is None:
            return False
        need = min(v.L, best.length) + v.mingap
        if bfree < need:
            return False
        v.lane = best
        v.pos = min(v.L, best.length)
        vl = min(best.speed * v.sf, v.vmaxv)
        if best.vehs:
            o = best.vehs[0]
            vl = min(vl, vsafe(o.pos - o.L - v.pos - v.mingap, o.v, v.b, v.tau))
        v.v = max(0.0, vl)
        bisect.insort(best.vehs, v, key=lambda o: o.pos)
        self.active_lanes.add(best)
        v.inserted = self.t
        return True

    # ---------------------------------------------------------------- pietoni
    def make_ped(self, spec):
        segs = R.ped_route(self.net, spec['frm'], spec['to'])
        if not segs:
            return None
        net = self.net
        out = []
        prev_end = None
        for kind, sid, sens, s0, s1 in segs:
            ln = net.lanes[sid] if kind == 'lane' else net.edges[sid].lanes[0]
            x0, y0, _ = R.point_at(ln.shape, ln.cum, s0)
            if prev_end is not None:
                gap = math.hypot(x0 - prev_end[0], y0 - prev_end[1])
                if gap > 0.3:
                    out.append(('wa', None, 0, prev_end, (x0, y0), gap))
            out.append((kind, sid, sens, s0, s1, abs(s1 - s0)))
            x1, y1, _ = R.point_at(ln.shape, ln.cum, s1)
            prev_end = (x1, y1)
        p = Ped()
        p.id, p.segs, p.si, p.s = spec['id'], out, 0, 0.0
        vt = self.vt.get(spec['type'], {})
        from cerere import sample_dist
        sf = sample_dist(vt.get('speedFactor', '1.0'), self.rng)
        r = self.rng.random()
        if r < 0.70:
            p.kind, p.kg = 'Adult', self.rng.uniform(63, 87)
        elif r < 0.85:
            p.kind, p.kg = 'Child', self.rng.uniform(19, 31)
            sf *= 0.85
        else:
            p.kind, p.kg = 'Heavy adult', self.rng.uniform(125, 175)
            sf *= 0.9
        p.speed = float(vt.get('maxSpeed', 1.39)) * sf
        p.obj = None
        if self.rng.random() < 0.08:
            p.obj = ('Bag / object', self.rng.uniform(5, 11)) if self.rng.random() < 0.7 else ('Dog', self.rng.uniform(12, 30))
        p.state, p.wait_start, p.waited, p.depart, p.timeloss = 'walk', None, 0.0, spec['depart'], 0.0
        p.cross, p.spot, p.red, p.cpos, p.dist = None, None, False, 0.0, 0.0
        p.xy = (0.0, 0.0)
        return p

    def ped_xy(self, p):
        seg = p.segs[p.si]
        if seg[0] == 'wa':
            (x0, y0), (x1, y1), L = seg[3], seg[4], seg[5]
            f = p.s / L if L > 0 else 0
            return x0 + (x1 - x0) * f, y0 + (y1 - y0) * f
        kind, sid, sens, s0 = seg[0], seg[1], seg[2], seg[3]
        ln = self.net.lanes[sid] if kind == 'lane' else self.net.edges[sid].lanes[0]
        x, y, _ = R.point_at(ln.shape, ln.cum, s0 + sens * p.s)
        return x, y

    def car_on_crossing(self, cr):
        """o masina care tocmai trece peste zebra (intrata deja in intersectie)"""
        if not cr.ped_links:
            return False
        pn = cr.ped_links[0].num
        w = cr.width / 2.0 + 0.5
        for lk in cr.car_links:
            cp = lk.conf.get(pn)
            if cp is None:
                continue
            for o in lk.occ:
                along = o.via_off + o.pos
                if along - o.L < cp[0] + w and o.v > 0.5 and along > cp[0] - w - 6.0:
                    return True
        return False

    def ped_may_cross(self, p, cr):
        st = self.ped_state(cr)
        if st in 'Gg':
            return not self.car_on_crossing(cr)
        if st == 'Z':
            for lk in cr.car_links:
                cp = lk.conf.get(cr.ped_links[0].num) if cr.ped_links else None
                w = cr.width / 2.0 + 0.5
                for o in lk.occ:
                    along = o.via_off + o.pos
                    if cp is None:
                        return False
                    front_to = cp[0] - w - along            # cat mai are fata pana la zebra
                    if along > cp[0] - w and along - o.L < cp[0] + w:
                        if o.v > 0.5 or o.wait < 3.0:
                            return False                   # trece acum peste zebra
                        continue                           # sta blocata pe zebra: pietonii o ocolesc
                    if front_to > 0 and o.v > 1.0 and front_to < o.v * o.v / (2 * 0.6 * o.b) + o.v * 1.0:
                        return False                       # vine si nu mai poate opri linistit
                for o, (ta, tl_, wp) in lk.appr_prev.items():
                    d = o.v * ta
                    if o.v > 1.0 and d < o.v * o.v / (2 * 0.6 * o.b) + o.v * 1.0:
                        return False                       # nu mai poate opri
            return True
        # rosu: pietonii nerabdatori pot traversa daca nu vine nimic
        imp = self.ped_impatience
        if imp and p.wait_start is not None and self.t - p.wait_start > imp[0]:
            if self.rng.random() < imp[1] * DT:
                for lk in cr.car_links:
                    if lk.occ:
                        return False
                    for o, (ta, tl_, wp) in lk.appr_prev.items():
                        if ta < 4.0:
                            return False
                p.red = True
                return True
        return False

    def step_ped(self, p):
        dt = DT
        seg = p.segs[p.si]
        if p.state == 'wait':
            cr = self.net.crossings[p.cross]
            if self.ped_may_cross(p, cr):
                w = self.t - p.wait_start
                self.ped_waits[cr.id].append(w)
                p.waited += w
                ctl = self.crossing_ctl.get(cr.id)
                if ctl is not None:
                    ctl.ped_leaves(p)
                if p.red:
                    self.stats['ped_red'] += 1
                p.state, p.wait_start, p.red = 'walk', None, False
                p.si += 1
                p.s = 0.0
                self._enter_seg(p)
            else:
                p.timeloss += dt
                if self.ped_state(cr) == 'Z' and cr.ped_links:
                    cr.ped_links[0].appr[p] = (0.0, 5.0, True)    # masinile trebuie sa opreasca
                return
        s = p.s + p.speed * dt
        p.dist += p.speed * dt
        while True:
            seg = p.segs[p.si]
            L = seg[5]
            if s < L:
                p.s = s
                break
            s -= L
            self._leave_seg(p)
            if p.si + 1 >= len(p.segs):
                self.ped_arrive(p)
                return
            nxt = p.segs[p.si + 1]
            if nxt[0] == 'cross':
                cr = self.net.crossings.get(nxt[1])
                if cr is not None:
                    pl = cr.ped_links[0] if cr.ped_links else None
                    if not self.ped_may_cross(p, cr):
                        p.s = L
                        p.state, p.wait_start, p.cross = 'wait', self.t, cr.id
                        if self.ped_state(cr) == 'Z' and pl is not None:
                            pl.appr[p] = (0.0, 5.0, True)
                        ctl = self.crossing_ctl.get(cr.id)
                        if ctl is not None:
                            ctl.ped_arrives(p)
                        return
                    if pl is not None:
                        pl.appr[p] = (0.0, 5.0, True)
            p.si += 1
            self._enter_seg(p)
        if seg[0] == 'cross':
            p.cpos = seg[3] + seg[2] * p.s

    def _enter_seg(self, p):
        seg = p.segs[p.si]
        if seg[0] == 'cross':
            cr = self.net.crossings[seg[1]]
            self.cross_peds[cr.id].add(p)
            p.cpos = seg[3]
            for pl in cr.ped_links:
                pl.occ.add(p)

    def _leave_seg(self, p):
        seg = p.segs[p.si]
        if seg[0] == 'cross':
            cr = self.net.crossings[seg[1]]
            self.cross_peds[cr.id].discard(p)
            for pl in cr.ped_links:
                pl.occ.discard(p)

    def ped_arrive(self, p):
        self._palive.discard(p)
        self.ped_trip.append((self.t - p.depart, p.waited, p.timeloss, p.dist))

    # ---------------------------------------------------------------- pasul principal
    def setup(self):
        self._alive, self._palive = OSet(), OSet()
        for tid, tc in self.tls_ctl.items():
            self.tls_state[tid] = tc.state()

    def step(self):
        t = self.t
        dt = DT
        rng = self.rng
        # 1. semafoare si controlerele trecerilor
        for tid, tc in self.tls_ctl.items():
            tc.update(t)
            self.tls_state[tid] = tc.state()
        self.overlay.clear()
        self.ped_overlay.clear()
        for ctl in self.crossing_ctl.values():
            ctl.update(t)
            ctl.apply(self.overlay, self.ped_overlay)
        # 2. aparitii
        while self.pending and self.pending[0]['depart'] <= t:
            spec = self.pending.pop(0)
            kind = 'emergency' if 'urgenta' in spec['id'] else 'car'
            v = self.make_vehicle(spec, kind)
            if v is None:
                self.stats['no_route'] += 1
                continue
            self.insert_q.append(v)
        while self.pending_trams and self.pending_trams[0]['depart'] <= t:
            v = self.make_vehicle(self.pending_trams.pop(0), 'tram')
            if v is not None:
                self.insert_q.append(v)
        if self.insert_q:
            left = []
            used_edges = set()
            for v in self.insert_q:
                e0 = v.route[0]
                if e0 not in used_edges and self.try_insert(v):
                    self._alive.add(v)
                    self.stats['inserted'] += 1
                else:
                    left.append(v)
                used_edges.add(e0)
            self.insert_q = left
        while self.pending_peds and self.pending_peds[0]['depart'] <= t:
            p = self.make_ped(self.pending_peds.pop(0))
            if p is not None:
                self._palive.add(p)
                self.stats['peds_inserted'] += 1
        # 3. apropierile de la pasul anterior
        for lk in self._appr_links if hasattr(self, '_appr_links') else ():
            lk.appr_prev = lk.appr
            lk.appr = {}
        # 4. viteze
        alive = list(self._alive)
        plans = []
        for v in alive:
            plans.append(self.plan(v, rng.random()))
        self._appr_links = {lk for lk in self.net.links if lk.appr} | {lk for lk in self.net.links if lk.appr_prev}
        # 5. miscare
        for v, vn in zip(alive, plans):
            if v.lane is None:
                continue
            vlim = min(v.lane.speed * v.sf, v.vmaxv)
            if vlim > 0:
                v.timeloss += dt * max(0.0, 1.0 - vn / vlim)
            if vn < 0.1:
                v.wait += dt
                v.wait_total += dt
            else:
                v.wait = 0.0
            self.move(v, vn)
        for lane in list(self.active_lanes):
            vs = lane.vehs
            if len(vs) > 1:
                vs.sort(key=lambda o: o.pos)
        # 6. tramvaie in statie, schimbari de banda, teleportare, rerutare
        for v in list(self._alive):
            if v.lane is None:
                continue
            if v.stops:
                sl, se, dur = v.stops[0]
                if v.lane.id == sl and v.pos >= se - 2.0 and v.v < 0.2:
                    if v.dwell_until < 0:
                        v.dwell_until = t + dur
                    elif t >= v.dwell_until:
                        v.stops.pop(0)
                        v.dwell_until = -1.0
                elif v.lane.id == sl and v.pos > se + 1.0:
                    v.stops.pop(0)
            if v.kind != 'tram':
                v.lc_cd -= dt
                if v.lane.kind == 'normal':
                    self.lane_change(v)
                if v.reroute and t >= v.next_rr and v.lane.kind == 'normal':
                    v.next_rr = t + 120.0
                    self.do_reroute(v)
            if v.kind != 'tram' and v.wait > 6.0 and (v.lane.kind != 'normal' or v.lane.length - v.pos < 5.0):
                self.unstick(v)
            if self.teleport_time > 0 and v.wait > self.teleport_time and v.dwell_until < 0 and v.brk_until < t:
                self.teleport(v)
        # 7. pietoni
        for p in list(self._palive):
            self.step_ped(p)
            p.xy = None
        # 8. coliziuni
        self.check_collisions()
        if int(t / DT) % 60 == 0:
            self.update_tt()
            nwait = sum(1 for p in self._palive if p.state == 'wait')
            self.series.append((t, len(self._alive), nwait))
        self.t = t + dt

    def unstick(self, v):
        """ce face un sofer real ajuns pe o banda care nu duce unde vrea (de ex. pe o bucata de strada
           de 0.2 m unde nu poate schimba banda): o ia pe unde poate si isi recalculeaza drumul"""
        lane = v.lane
        rb = self.net.roundabout
        ri = v.ri
        if lane.kind == 'internal':
            if lane.link is None:
                return
            lane, ri = lane.link.to, v.ri + 1          # banda pe care ajunge dupa intersectie
            # urmeaza benzile scurte fara alegere
            for _ in range(6):
                if ri + 1 >= len(v.route) or lane.length > 6.0:
                    break
                nk = self.choose_link(v, lane, ri)
                if nk is None or nk.via:
                    break
                lane, ri = nk.to, ri + 1
        if ri + 1 >= len(v.route):
            return
        lk = self.choose_link(v, lane, ri)
        alt = None
        if lk is None and v.wait > 10.0:
            alt = [k for k in lane.links if v.vclass in k.to.allow]
            why = 'banda'
        if not alt:
            return
        for k in alt:
            if self.space_after(v, k, v.L + v.mingap) < v.L + v.mingap * 0.5:
                continue
            r = R.car_route(self.net, k.to.edge.id, v.route[-1], 'passenger', self.edge_tt)
            if r:
                v.route = v.route[:ri + 1] + r
                v.goodfor = None
                self.good_lanes(v)
                v.loops += 1
                self.stats['unstick_' + why] += 1
                return

    def update_tt(self):
        sp = defaultdict(list)
        for v in self._alive:
            if v.lane is not None and v.lane.kind == 'normal':
                sp[v.lane.edge.id].append(v.v)
        for e, vs in sp.items():
            ed = self.net.edges[e]
            m = sum(vs) / len(vs)
            tt = ed.length / max(0.5, m)
            old = self.edge_tt.get(e, ed.length / max(1.0, ed.speed))
            self.edge_tt[e] = 0.5 * old + 0.5 * tt

    def do_reroute(self, v):
        r = R.car_route(self.net, v.route[v.ri], v.route[-1], 'passenger', self.edge_tt)
        if r and r != v.route[v.ri:]:
            v.route = v.route[:v.ri] + r
            v.goodfor = None
            self.good_lanes(v)
            self.stats['reroutes'] += 1

    def teleport(self, v):
        """ca SUMO: vehiculul blocat sare pe urmatoarea muchie din traseu unde are loc"""
        self.stats['teleports'] += 1
        old = v.lane
        if v in old.vehs:
            old.vehs.remove(v)
            if not old.vehs:
                self.active_lanes.discard(old)
        if v.link is not None:
            v.link.occ.discard(v)
            if not v.link.occ:
                self.occ_links.discard(v.link)
            v.link = None
            v.via_off = 0.0
        for k in range(v.ri + 1, len(v.route)):
            ed = self.net.edges[v.route[k]]
            for l in ed.lanes:
                if v.vclass in l.allow and l.length >= v.L and (not l.vehs or l.vehs[0].pos - l.vehs[0].L > v.L + v.mingap):
                    v.ri, v.lane, v.pos, v.v, v.wait = k, l, v.L, 0.0, 0.0
                    v.goodfor = None
                    self.good_lanes(v)
                    bisect.insort(l.vehs, v, key=lambda o: o.pos)
                    self.active_lanes.add(l)
                    return
        v.lane = None
        self._alive.discard(v)
        self.trip.append((self.t - v.depart, v.wait_total, v.timeloss, v.inserted - v.depart, v.kind))

    def check_collisions(self):
        for lk in list(self.occ_links):
            for fo in lk.foes:
                if fo.num < lk.num and not fo.is_ped:
                    continue
                if not fo.occ:
                    continue
                cp = lk.conf.get(fo.num)
                if cp is None or (lk.to is fo.to and not fo.is_ped):
                    continue
                for a in lk.occ:
                    al = a.via_off + a.pos
                    if not (cp[0] - 0.3 <= al <= cp[0] + a.L):
                        continue
                    if fo.is_ped and a.v < 0.5:
                        continue             # masina oprita: pietonul o ocoleste, nu e lovire
                    for b in fo.occ:
                        if fo.is_ped:
                            if abs(b.cpos - cp[1]) > 1.0:
                                continue
                        else:
                            bl = b.via_off + b.pos
                            if not (cp[1] - 0.3 <= bl <= cp[1] + b.L):
                                continue
                        key = (a.id, b.id)
                        if self.t - self.collided.get(key, -1e9) > 20:
                            self.collided[key] = self.t
                            self.stats['collisions'] += 1
                            if fo.is_ped:
                                self.stats['collisions_ped'] += 1

    # ---------------------------------------------------------------- rezultate
    def results(self):
        trips = [x for x in self.trip if x[4] != 'tram']
        n = len(trips)
        avg = lambda i: sum(x[i] for x in trips) / n if n else 0.0
        pn = len(self.ped_trip)
        pavg = lambda i: sum(x[i] for x in self.ped_trip) / pn if pn else 0.0
        waits = [w for ws in self.ped_waits.values() for w in ws]
        for p in self._palive:
            if p.state == 'wait' and p.wait_start is not None:
                waits.append(self.t - p.wait_start)
        return dict(
            car_time_loss=avg(2), car_waiting=avg(1), car_duration=avg(0), depart_delay=avg(3),
            cars_finished=n, cars_unfinished=len([v for v in self._alive if v.kind != 'tram']) + len(self.insert_q),
            teleports=self.stats['teleports'], collisions=self.stats['collisions'],
            collisions_ped=self.stats['collisions_ped'],
            ped_time_loss=pavg(2), ped_duration=pavg(0), peds_finished=pn,
            ped_wait_crossings=(sum(waits) / len(waits)) if waits else 0.0,
            ped_wait_max=max(waits) if waits else 0.0,
            ped_waited_over_45s=sum(1 for w in waits if w > 45.0),
            ped_crossed_on_red=self.stats['ped_red'],
        )


class TLSProgram(object):
    """programul unui semafor din retea (static sau actuat, ca in SUMO)"""

    def __init__(self, sim, t):
        self.sim, self.t = sim, t
        self.phases = t['phases']
        self.i, self.t0 = 0, -t.get('offset', 0.0)
        self.actuated = t['type'] == 'actuated'
        self.skip_ped_only = False
        n = len(self.phases[0]['state'])
        ped_idx = set()
        for idx, lks in t['links'].items():
            if all(lk.is_ped for lk in lks):
                ped_idx.add(idx)
        self.ped_idx = ped_idx
        self.car_idx = [i for i in range(n) if i not in ped_idx]
        self.approach = {}
        for idx, lks in t['links'].items():
            for lk in lks:
                if not lk.is_ped:
                    self.approach.setdefault(idx, set()).add(lk.frm)

    def ped_only(self, ph):
        st = ph['state']
        return all(st[i] in 'rRu' for i in self.car_idx) and any(st[i] in 'Gg' for i in self.ped_idx)

    def update(self, t):
        ph = self.phases[self.i]
        el = t - self.t0
        dur = ph['dur']
        if self.skip_ped_only and self.ped_only(ph):
            dur = 0.0
        if self.actuated and ph['minDur'] < ph['maxDur'] and any(c in 'Gg' for c in ph['state']):
            if el < ph['minDur']:
                return
            if el < ph['maxDur'] and self.demand(ph):
                return
            self.next(t)
            return
        if el >= dur:
            self.next(t)

    def demand(self, ph):
        """se prelungeste verdele cat timp vin masini spre benzile cu verde (detectoare la 30 m)"""
        st = ph['state']
        for idx, lanes in self.approach.items():
            if idx < len(st) and st[idx] in 'Gg':
                for l in lanes:
                    for v in l.vehs:
                        if l.length - v.pos < 30 and v.v > 0.5:
                            return True
        return False

    def next(self, t):
        for _ in range(len(self.phases)):
            self.i = (self.i + 1) % len(self.phases)
            if not (self.skip_ped_only and self.ped_only(self.phases[self.i])):
                break
        self.t0 = t

    def state(self):
        return self.phases[self.i]['state']


# ----------------------------------------------------------------------------- din fereastra
def inject_ped(sim, cr, side, kind):
    """un pieton pus direct la o trecere (butonul "Adauga persoana")"""
    sh = cr.shape
    (x0, y0), (x1, y1) = (sh[0], sh[-1]) if side == 0 else (sh[-1], sh[0])
    L = math.hypot(x1 - x0, y1 - y0) or 1.0
    ux, uy = (x1 - x0) / L, (y1 - y0) / L
    a = (x0 - ux * 1.5, y0 - uy * 1.5)
    b = (x1 + ux * 2.0, y1 + uy * 2.0)
    p = Ped()
    p.id = 'manual_%d' % int(sim.stats['manual'])
    sim.stats['manual'] += 1
    sens = 1 if side == 0 else -1
    s0, s1 = (0.0, cr.length) if side == 0 else (cr.length, 0.0)
    p.segs = [('wa', None, 0, a, (x0, y0), 1.5), ('cross', cr.id, sens, s0, s1, cr.length),
              ('wa', None, 0, (x1, y1), b, 2.0)]
    p.si, p.s = 0, 0.0
    kinds = {'Adult': (63, 87, 1.0), 'Child': (19, 31, 0.85), 'Heavy adult': (125, 175, 0.9)}
    lo, hi, f = kinds.get(kind, kinds['Adult'])
    p.kind, p.kg = kind, sim.rng.uniform(lo, hi)
    p.speed = 1.3 * f
    p.obj = None
    p.state, p.wait_start, p.waited, p.depart, p.timeloss = 'walk', None, 0.0, sim.t, 0.0
    p.cross, p.spot, p.red, p.cpos, p.dist, p.xy = None, None, False, 0.0, 0.0, (x0, y0)
    sim._palive.add(p)
    return p


def breakdown(sim, cr, seconds=45.0):
    """opreste prima masina care vine spre trecere (butonul "Masina in pana")"""
    best, bd = None, 1e9
    for lk in cr.car_links:
        for v in list(lk.occ):
            return _brk(sim, v, seconds)
        lane, d = lk.frm, 0.0
        for v in reversed(lane.vehs):
            dd = lane.length - v.pos
            if dd < bd and v.kind != 'tram':
                best, bd = v, dd
    if best is not None:
        return _brk(sim, best, seconds)
    return None


def _brk(sim, v, seconds):
    v.brk_until = sim.t + seconds
    sim.stats['breakdowns'] += 1
    return v
