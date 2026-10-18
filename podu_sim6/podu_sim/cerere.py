# -*- coding: utf-8 -*-
"""Traficul: aceleasi fisiere ca SUMO (trafic/*.xml), citite direct."""
import os, re, math, random
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, 'proiect.txt')      # unde e folderul PoduRos_SUMO (se tine minte)


def is_project(d):
    return bool(d) and os.path.isfile(os.path.join(d, 'retea', 'podu_ros.net.xml')) \
        and os.path.isdir(os.path.join(d, 'trafic'))


def find_root():
    """cauta folderul proiectului (cu retea\\podu_ros.net.xml si trafic\\) in jurul programului"""
    try:
        with open(CFG, encoding='utf-8') as f:
            d = f.read().strip()
        if is_project(d):
            return d
    except OSError:
        pass
    anc, p = [], HERE
    for _ in range(4):
        anc.append(p)
        p = os.path.dirname(p)
    for d in anc:
        if is_project(d):
            return d
    for base in anc[:3]:
        try:
            subs = sorted(os.listdir(base))
        except OSError:
            continue
        for n in subs:
            d = os.path.join(base, n)
            if not os.path.isdir(d):
                continue
            if is_project(d):
                return d
            try:
                for n2 in sorted(os.listdir(d)):
                    d2 = os.path.join(d, n2)
                    if os.path.isdir(d2) and is_project(d2):
                        return d2
            except OSError:
                pass
    return None


def set_root(d):
    global ROOT, TRAFIC, RETEA
    ROOT, TRAFIC, RETEA = d, os.path.join(d, 'trafic'), os.path.join(d, 'retea')
    try:
        with open(CFG, 'w', encoding='utf-8') as f:
            f.write(d)
    except OSError:
        pass


ROOT = find_root() or os.path.dirname(HERE)
TRAFIC = os.path.join(ROOT, 'trafic')
RETEA = os.path.join(ROOT, 'retea')

# valorile implicite SUMO pentru fiecare clasa de vehicul
DEFAULTS = {
    'passenger': dict(length=5.0, minGap=2.5, accel=2.6, decel=4.5, emergencyDecel=9.0, maxSpeed=55.56,
                      tau=1.0, sigma=0.5, speedFactor='1.0'),
    'emergency': dict(length=6.5, minGap=2.5, accel=2.6, decel=4.5, emergencyDecel=9.0, maxSpeed=41.67,
                      tau=1.0, sigma=0.5, speedFactor='1.0'),
    'tram': dict(length=22.0, minGap=2.5, accel=1.0, decel=3.0, emergencyDecel=7.0, maxSpeed=22.22,
                 tau=1.0, sigma=0.5, speedFactor='1.0'),
    'pedestrian': dict(length=0.215, width=0.478, maxSpeed=1.39, speedFactor='1.0'),
}

LEVELS = ['liber', 'relativ_liber', 'semi_aglomerat', 'aglomerat', 'ora_de_varf', 'ambuteiaj']
LEVEL_NAMES = {'liber': 'Liber', 'relativ_liber': 'Relativ liber', 'semi_aglomerat': 'Semi-aglomerat',
               'aglomerat': 'Aglomerat', 'ora_de_varf': 'Ora de varf', 'ambuteiaj': 'Ambuteiaj'}
TELEPORT = {'liber': 300, 'relativ_liber': 300, 'semi_aglomerat': 300, 'aglomerat': 400,
            'ora_de_varf': 600, 'ambuteiaj': 900}


def sample_dist(spec, rng):
    """'normc(medie,abatere,min,max)' sau un numar"""
    spec = str(spec).replace(' ', '')
    m = re.match(r'normc\(([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)\)', spec)
    if m:
        mu, sd, lo, hi = [float(x) for x in m.groups()]
        for _ in range(20):
            v = rng.gauss(mu, sd)
            if lo <= v <= hi:
                return v
        return min(hi, max(lo, mu))
    m = re.match(r'norm\(([-\d.]+),([-\d.]+)\)', spec)
    if m:
        return max(0.1, rng.gauss(float(m.group(1)), float(m.group(2))))
    return float(spec)


def read_vtypes(paths):
    out = {}
    for p in paths:
        if not os.path.isfile(p):
            continue
        for v in ET.parse(p).getroot().iter('vType'):
            vc = v.get('vClass', 'passenger')
            d = dict(DEFAULTS.get(vc, DEFAULTS['passenger']))
            d['vClass'] = vc
            for k, val in v.attrib.items():
                if k in ('id', 'vClass', 'color', 'guiShape'):
                    continue
                try:
                    d[k] = float(val)
                except ValueError:
                    d[k] = val
            if 'color' in v.attrib:
                d['color'] = v.get('color')
            out[v.get('id')] = d
    return out


def _scaled(items, scale, rng):
    if abs(scale - 1.0) < 1e-9:
        return items
    out = []
    for it in items:
        n = int(scale) + (1 if rng.random() < scale - int(scale) else 0)
        for k in range(n):
            if k == 0:
                out.append(it)
            else:
                d = dict(it)
                d['id'] = '%s.%d' % (it['id'], k)
                d['depart'] = it['depart'] + rng.uniform(0, 5)
                out.append(d)
    out.sort(key=lambda d: d['depart'])
    return out


def load_level(level, scale=1.0, seed=1):
    """toata cererea pentru un nivel: vehicule, pietoni, tramvaie, urgente"""
    rng = random.Random(1000 + seed)
    vt = read_vtypes([os.path.join(TRAFIC, 'tipuri_masini_%s.add.xml' % level),
                      os.path.join(TRAFIC, 'tipuri_pietoni_%s.add.xml' % level),
                      os.path.join(TRAFIC, 'tipuri_tramvai.add.xml'),
                      os.path.join(TRAFIC, 'tipuri_urgenta.add.xml')])
    cars = []
    for f in ('masini_%s.trips.xml' % level, 'urgente_%s.rou.xml' % level):
        p = os.path.join(TRAFIC, f)
        if not os.path.isfile(p):
            continue
        for t in ET.parse(p).getroot().iter('trip'):
            cars.append(dict(id=t.get('id'), type=t.get('type', 'masina'), depart=float(t.get('depart')),
                             frm=t.get('from'), to=t.get('to'), via=t.get('via', '').split()))
    cars.sort(key=lambda d: d['depart'])
    peds = []
    p = os.path.join(TRAFIC, 'pietoni_%s.trips.xml' % level)
    if os.path.isfile(p):
        for pe in ET.parse(p).getroot().iter('person'):
            w = pe.find('walk')
            if w is None:
                continue
            peds.append(dict(id=pe.get('id'), type=pe.get('type', 'pieton'), depart=float(pe.get('depart')),
                             frm=w.get('from'), to=w.get('to')))
    trams = []
    p = os.path.join(TRAFIC, 'tramvaie_%s.rou.xml' % level)
    stops = read_stops(os.path.join(RETEA, 'statii_tramvai.add.xml'))
    if os.path.isfile(p):
        r = ET.parse(p).getroot()
        routes = {}
        for ro in r.iter('route'):
            st = [(stops[s.get('busStop')], float(s.get('duration', '20'))) for s in ro.findall('stop')
                  if s.get('busStop') in stops]
            routes[ro.get('id')] = (ro.get('edges').split(), st)
        for fl in r.iter('flow'):
            b, e = float(fl.get('begin', '0')), float(fl.get('end', '3600'))
            per = float(fl.get('period', '600'))
            t, k = b, 0
            while t < e:
                trams.append(dict(id='%s.%d' % (fl.get('id'), k), type=fl.get('type', 'tramvai'), depart=t,
                                  edges=routes[fl.get('route')][0], stops=routes[fl.get('route')][1],
                                  line=fl.get('line', '')))
                t += per
                k += 1
        trams.sort(key=lambda d: d['depart'])
    cars = _scaled(cars, scale, rng)
    peds = _scaled(peds, scale, rng)
    return dict(vtypes=vt, cars=cars, peds=peds, trams=trams, teleport=TELEPORT.get(level, 600))


def read_stops(path):
    out = {}
    if os.path.isfile(path):
        for b in ET.parse(path).getroot().iter('busStop'):
            out[b.get('id')] = dict(lane=b.get('lane'), start=float(b.get('startPos', '0')),
                                    end=float(b.get('endPos', '0')), name=b.get('name', ''))
    return out


def crossing_names(net_crossings=None):
    """nume prietenoase pentru treceri, din treceri_pietoni.json (zona) + strada traversata"""
    import json
    names = {}
    p = os.path.join(ROOT, 'treceri_pietoni.json')
    if os.path.isfile(p):
        J = json.load(open(p, encoding='utf-8'))
        for t in J.get('treceri', []):
            for m in t.get('muchii', []):
                names.setdefault(t['nod'], t['zona'])
    return names
