# -*- coding: utf-8 -*-
"""Ruleaza o simulare fara fereastra (folosit de comparatii, si din linia de comanda):

  python rulare.py --setup setup.json --nivel aglomerat --minute 30 --seed 1 --out rezultat.json
"""
import os, sys, json, time, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import retea, cerere, motor, treceri   # noqa: E402


METRICS = [
    ('car_time_loss', 'Masini: timp pierdut mediu (s)', True),
    ('car_waiting', 'Masini: timp stat pe loc (s)', True),
    ('cars_finished', 'Masini: calatorii terminate', False),
    ('cars_unfinished', 'Masini: ramase in retea la final', True),
    ('teleports', 'Masini: teleportari (blocaje)', True),
    ('collisions', 'Coliziuni (total)', True),
    ('collisions_ped', 'Coliziuni cu pietoni', True),
    ('ped_wait_crossings', 'Pietoni: asteptare medie la treceri (s)', True),
    ('ped_wait_max', 'Pietoni: cea mai lunga asteptare (s)', True),
    ('ped_waited_over_45s', 'Pietoni: asteptari peste 45 s', True),
    ('ped_crossed_on_red', 'Pietoni: traversari pe rosu', True),
    ('ped_time_loss', 'Pietoni: timp pierdut per drum (s)', True),
    ('turners_stopped_s', 'Masini oprite in zona de viraj (masina-s)', True),
    ('spill_s', 'Coada de viraj ajunsa pe banda comuna (s)', True),
    ('area_full_s', 'Zona de viraj plina (s)', True),
    ('holds', 'Hold-uri', None),
    ('hold_time', 'Timp de hold (s)', None),
    ('shared_walks', 'Verde comun A+B (fix 5)', None),
    ('spill_cuts', 'Verde oprit: zona de viraj plina (fix 5)', None),
    ('walks', 'Faze de verde pentru pietoni', None),
]


def load_net():
    names = cerere.crossing_names()
    net = retea.load(os.path.join(cerere.RETEA, 'podu_ros.net.xml'))
    for cid, cr in net.crossings.items():
        z = names.get(cr.node, '')
        streets = sorted({net.edges[e].name for e in cr.crossed if e in net.edges and net.edges[e].name})
        tram = not cr.cars
        cr.zona = z
        cr.name = '%s %s' % (z, ', '.join(streets) or 'linie tramvai') if streets or tram else '%s %s' % (z, cid)
    seen = {}
    for cid in sorted(net.crossings):
        cr = net.crossings[cid]
        seen.setdefault(cr.name, []).append(cr)
    for name, crs in seen.items():
        if len(crs) > 1:
            for k, cr in enumerate(crs, 1):
                cr.name = '%s #%d' % (name, k)
    return net


def make_sim(net, setup, level, minutes, seed, scale, log=print):
    dem = cerere.load_level(level, scale, seed)
    sim = motor.Sim(net, dem, seed=seed, end=minutes * 60.0, log=log)
    reset_net(net)
    treceri.install(sim, setup, seed)
    sim.setup()
    return sim


def reset_net(net):
    for l in net.lanes.values():
        l.vehs = []
        l.peds = []
    for lk in net.links:
        lk.occ = motor.OSet()
        lk.appr, lk.appr_prev = {}, {}


def run(setup, level, minutes, seed=1, scale=1.0, progress=None, stop=None):
    net = load_net()
    sim = make_sim(net, setup, level, minutes, seed, scale)
    n = 0
    while sim.t < sim.end:
        if stop is not None and stop():
            break
        sim.step()
        n += 1
        if progress and n % 120 == 0:
            progress(sim.t / sim.end)
    res = sim.results()
    res.update(treceri.summary_smart(sim))
    pc = {}
    for cid, ws in sim.ped_waits.items():
        cr = net.crossings[cid]
        pc[cr.name] = (sum(ws) / len(ws), len(ws))
    res['per_crossing_wait'] = pc
    res['series'] = sim.series
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--setup', required=True, help='fisier JSON cu setarile (din fereastra)')
    ap.add_argument('--nivel', default='aglomerat', choices=cerere.LEVELS)
    ap.add_argument('--minute', type=float, default=30)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--scale', type=float, default=1.0)
    ap.add_argument('--out')
    a = ap.parse_args()
    with open(a.setup, encoding='utf-8') as f:
        setup = json.load(f)
    t0 = time.time()
    res = run(setup, a.nivel, a.minute, a.seed, a.scale)
    res['durata_calcul_s'] = round(time.time() - t0, 1)
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(res, f)
    else:
        print(json.dumps({k: v for k, v in res.items() if k != 'series'}, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
