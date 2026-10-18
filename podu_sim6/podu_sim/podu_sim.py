#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PODU ROS - simulare standalone (fara SUMO) cu trecerile de pietoni inteligente.

Porneste:  python podu_sim\\podu_sim.py     (sau dublu-click pe podu_sim.bat)

Taburi:
  Simulare  - harta live: start / pauza / viteza, click pe o trecere ca s-o vezi in detaliu,
              "Adauga persoana" si "Masina in pana" la trecerea selectata
  Treceri   - pentru fiecare trecere: Original / Zebra / Semafor normal / Sistemul nostru (+ ce reguli)
  Reguli    - seturile de reguli pentru sistemul nostru (placi, senzori, hold, fix-uri)
  Altele    - semaforul normal, pietonii nerabdatori, faza exclusiva pentru pietoni
  Comparare - mai multe configuratii, pe cate minute vrei, cu repetari; tabel + grafic + CSV/TXT
"""
import os, sys, json, math, time, copy, csv, queue, threading, subprocess, statistics, datetime
from collections import OrderedDict
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import retea, cerere, motor, treceri, rulare   # noqa: E402

SETARI = os.path.join(HERE, 'setari.json')
EIGHT = [':6413766796_c0', ':12043430980_c0', ':10616411211_c0', ':10616411210_c0', ':3086446449_c0',
         ':12080013339_c0', ':12159533179_c0', ':6413766794_c0']
SPEEDS = OrderedDict([('0.5x', 0.5), ('1x', 1), ('2x', 2), ('5x', 5), ('10x', 10), ('30x', 30), ('Maxim', 1e9)])
STATE_COLOR = {'G': '#22c55e', 'g': '#86efac', 'y': '#facc15', 'Y': '#facc15', 'r': '#ef4444', 'R': '#ef4444',
               'u': '#f97316', 'o': '#facc15', 'O': '#facc15'}


def rez_dir():
    return os.path.join(cerere.ROOT, 'rezultate', 'podu_sim')


def default_setup():
    first = next(iter(treceri.PRESETS_RULES))
    return dict(treceri={c: dict(mod='inteligent', reguli=first) for c in EIGHT}, reguli=OrderedDict((k, dict(v)) for k, v in treceri.PRESETS_RULES.items()),
                perechi=[list(p) for p in treceri.DEFAULT_PAIRS], semafor=dict(treceri.SEMAFOR_DEFAULTS),
                nerabdare=None, nerabdare_dupa=40.0, nerabdare_prob=0.05, fara_faza_pietoni=True)


def setup_for_run(S):
    s = copy.deepcopy(S)
    s['nerabdare'] = [S.get('nerabdare_dupa', 40.0), S.get('nerabdare_prob', 0.05)] if S.get('nerabdare_on') else None
    return s


def describe(S, net):
    modes = {}
    for cid in net.crossings:
        m = S['treceri'].get(cid, {}).get('mod', 'original')
        modes[m] = modes.get(m, 0) + 1
    parts = ['%d %s' % (n, treceri.MODES[m]) for m, n in modes.items()]
    rs = sorted({S['treceri'][c].get('reguli', '') for c in S['treceri'] if S['treceri'][c].get('mod') == 'inteligent'})
    return ', '.join(parts) + ((' | reguli: ' + ', '.join(rs)) if rs else '')


class App(object):
    def __init__(self, root):
        self.root = root
        root.title('Podu Ros - simulare cu treceri inteligente')
        root.geometry('1400x900')
        self.net = rulare.load_net()
        self.load_settings()
        self.q = queue.Queue()
        self.sim = None
        self.running = False
        self.sel = None
        self.items = {}
        self.cmp_stop = None
        self.sc, self.ox, self.oy = 2.0, 0.0, 0.0
        pw = ttk.PanedWindow(root, orient='horizontal')
        pw.pack(fill='both', expand=True)
        left = ttk.Frame(pw)
        right = ttk.Frame(pw, padding=4)
        pw.add(left, weight=3)
        pw.add(right, weight=2)
        root.after(300, lambda: pw.sashpos(0, 820))
        self.canvas = tk.Canvas(left, bg='#e8e6e1', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.status = ttk.Label(left, text='', anchor='w')
        self.status.pack(fill='x')
        self.nb = ttk.Notebook(right)
        self.nb.pack(fill='both', expand=True)
        for name, fn in (('Simulare', self.tab_sim), ('Treceri', self.tab_cross), ('Reguli', self.tab_rules),
                         ('Altele', self.tab_other), ('Comparare', self.tab_cmp)):
            f = ttk.Frame(self.nb, padding=6)
            self.nb.add(f, text=name)
            fn(f)
        self.canvas.bind('<Configure>', lambda e: self.fit() if not self.items else None)
        self.canvas.bind('<ButtonPress-1>', self.on_press)
        self.canvas.bind('<B1-Motion>', self.on_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_release)
        self.canvas.bind('<MouseWheel>', self.on_wheel)
        self.canvas.bind('<Button-4>', lambda e: self.zoom(1.2, e.x, e.y))
        self.canvas.bind('<Button-5>', lambda e: self.zoom(1 / 1.2, e.x, e.y))
        root.protocol('WM_DELETE_WINDOW', self.on_close)
        root.after(100, self.first_draw)
        root.after(50, self.loop)
        root.after(200, self.poll)

    # ================================================================== setari
    def load_settings(self):
        self.S = default_setup()
        self.compare_list = []
        try:
            with open(SETARI, encoding='utf-8') as f:
                d = json.load(f)
            self.S.update(d.get('setup', {}))
            self.compare_list = d.get('comparare', [])
            if d.get('versiune', 1) < 2 and not any(c.get('mod') not in (None, 'original') for c in self.S['treceri'].values()):
                self.S['treceri'] = default_setup()['treceri']      # versiunea veche pornea fara sistemul nostru
        except Exception:
            pass
        if not self.S['reguli']:
            self.S['reguli'] = {'Sistemul tau (hold 12 s)': {}}

    def save_settings(self):
        try:
            with open(SETARI, 'w', encoding='utf-8') as f:
                json.dump(dict(versiune=2, setup=self.S, comparare=self.compare_list), f, indent=1, ensure_ascii=False)
        except Exception:
            pass

    def on_close(self):
        self.save_settings()
        if self.cmp_stop is not None:
            self.cmp_stop.set()
        self.root.destroy()

    # ================================================================== tab Simulare
    def tab_sim(self, f):
        g = ttk.LabelFrame(f, text='Trafic', padding=6)
        g.pack(fill='x')
        self.level = tk.StringVar(value='aglomerat')
        ttk.Label(g, text='Nivel').grid(row=0, column=0, sticky='w')
        ttk.Combobox(g, textvariable=self.level, values=cerere.LEVELS, state='readonly', width=15).grid(row=0, column=1)
        ttk.Label(g, text=' Trafic x').grid(row=0, column=2)
        self.scale = tk.DoubleVar(value=1.0)
        ttk.Spinbox(g, from_=0.3, to=3.0, increment=0.05, textvariable=self.scale, width=5).grid(row=0, column=3)
        ttk.Label(g, text=' Seed').grid(row=0, column=4)
        self.seed = tk.IntVar(value=1)
        ttk.Spinbox(g, from_=1, to=999, textvariable=self.seed, width=4).grid(row=0, column=5)
        ttk.Label(g, text='Minute').grid(row=1, column=0, sticky='w')
        self.minutes = tk.IntVar(value=70)
        ttk.Spinbox(g, from_=1, to=240, textvariable=self.minutes, width=6).grid(row=1, column=1, sticky='w')
        b = ttk.Frame(f)
        b.pack(fill='x', pady=6)
        ttk.Button(b, text='▶ Start / Continua', command=self.start).pack(side='left')
        ttk.Button(b, text='❚❚ Pauza', command=self.pause).pack(side='left', padx=4)
        ttk.Button(b, text='⟲ De la inceput', command=self.reset).pack(side='left')
        r = ttk.Frame(f)
        r.pack(fill='x')
        ttk.Label(r, text='Viteza').pack(side='left')
        self.speed = tk.StringVar(value='2x')
        ttk.Combobox(r, textvariable=self.speed, values=list(SPEEDS), state='readonly', width=7).pack(side='left', padx=4)
        self.show_plates = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text='Placi', variable=self.show_plates).pack(side='left', padx=6)
        ttk.Label(r, text='Senzori:').pack(side='left')
        self.show_sensors = tk.StringVar(value='trecerea selectata')
        cb = ttk.Combobox(r, textvariable=self.show_sensors, values=['trecerea selectata', 'toate', 'deloc'],
                          state='readonly', width=16)
        cb.pack(side='left')
        cb.bind('<<ComboboxSelected>>', lambda e: self.redraw())
        self.show_legend = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text='Legenda', variable=self.show_legend, command=self.redraw).pack(side='left', padx=4)
        self.show_labels = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text='Etichete', variable=self.show_labels, command=self.redraw).pack(side='left', padx=6)
        r = ttk.Frame(f)
        r.pack(fill='x', pady=(6, 0))
        ttk.Label(r, text='Mergi la trecerea').pack(side='left')
        self.goto = tk.StringVar()
        self.goto_cb = ttk.Combobox(r, textvariable=self.goto, state='readonly', width=42)
        self.goto_cb.pack(side='left', padx=4)
        self.goto_cb.bind('<<ComboboxSelected>>', lambda e: self.goto_crossing())
        ttk.Button(r, text='Vezi tot', command=lambda: (self.fit(), self.draw_static())).pack(side='left')
        self.active_lbl = ttk.Label(f, text='', foreground='#5b21b6', wraplength=500, justify='left')
        self.active_lbl.pack(fill='x', pady=4)
        self.live = ttk.Label(f, text='Apasa Start.', justify='left', font=('TkFixedFont', 9))
        self.live.pack(fill='x', pady=2)
        g = ttk.LabelFrame(f, text='Trecerea selectata (click pe ea pe harta, sau "Mergi la")', padding=6)
        g.pack(fill='both', expand=True)
        self.selinfo = ttk.Label(g, text='-', justify='left', wraplength=500, font=('TkFixedFont', 9))
        self.selinfo.pack(fill='x')
        ttk.Label(g, text='Ce s-a intamplat (cel mai nou sus):').pack(anchor='w', pady=(4, 0))
        self.evlist = tk.Listbox(g, height=9, font=('TkFixedFont', 9))
        self.evlist.pack(fill='both', expand=True)
        r = ttk.Frame(g)
        r.pack(fill='x', pady=4)
        self.addkind = tk.StringVar(value='Adult')
        ttk.Combobox(r, textvariable=self.addkind, values=['Adult', 'Child', 'Heavy adult'], state='readonly',
                     width=11).pack(side='left')
        ttk.Button(r, text='Adauga persoana (partea 1)', command=lambda: self.add_person(0)).pack(side='left', padx=2)
        ttk.Button(r, text='(partea 2)', command=lambda: self.add_person(1)).pack(side='left')
        ttk.Button(g, text='Masina in pana 45 s inainte de trecere', command=self.breakdown).pack(anchor='w')
        self.refresh_goto()

    def refresh_goto(self):
        names = []
        for cid, cr in sorted(self.net.crossings.items(), key=lambda kv: kv[1].name):
            m = self.S['treceri'].get(cid, {}).get('mod', 'original')
            names.append(('0' if m == 'inteligent' else '1') + cr.name + '   [' + treceri.MODES[m] + ']')
        names.sort()
        self.goto_cb['values'] = [n[1:] for n in names]
        smart = [self.net.crossings[c].name for c, cfg in self.S['treceri'].items()
                 if cfg.get('mod') == 'inteligent' and c in self.net.crossings]
        self.active_lbl.config(text=('Sistemul nostru e pe %d treceri: %s' % (len(smart), ', '.join(sorted(smart))))
                               if smart else 'Sistemul nostru nu e pus pe nicio trecere (tabul Treceri).')

    def goto_crossing(self):
        name = self.goto.get().split('   [')[0]
        cid = self.name_to_id(name)
        if cid is None:
            return
        cr = self.net.crossings[cid]
        mx = sum(p[0] for p in cr.shape) / len(cr.shape)
        my = sum(p[1] for p in cr.shape) / len(cr.shape)
        cw, ch = max(200, self.canvas.winfo_width()), max(200, self.canvas.winfo_height())
        self.sc = 7.0
        self.ox, self.oy = mx - cw / 2 / self.sc, my + ch / 2 / self.sc
        self.sel = cid
        self.draw_static()
        self.show_sel()

    def redraw(self):
        self.draw_static()

    def apply_live(self):
        """schimbarile din Treceri / Reguli / Altele se aplica imediat simularii care ruleaza"""
        if getattr(self, '_ap', None):
            self.root.after_cancel(self._ap)
        self._ap = self.root.after(400, self._apply_now)

    def _apply_now(self):
        self._ap = None
        self.refresh_goto()
        if self.sim is None:
            return
        for ctl in self.sim.crossing_ctl.values():
            for p in list(ctl.waiting):
                ctl.ped_leaves(p)
        treceri.install(self.sim, setup_for_run(self.S), int(self.seed.get()))
        for p in self.sim._palive:
            if p.state == 'wait' and p.cross in self.sim.crossing_ctl:
                self.sim.crossing_ctl[p.cross].ped_arrives(p)
        self.draw_static()

    def make_sim(self):
        S = setup_for_run(self.S)
        self.sim = rulare.make_sim(self.net, S, self.level.get(), float(self.minutes.get()), int(self.seed.get()),
                                   float(self.scale.get()))
        self.t_wall, self.t_acc = time.time(), 0.0
        self.draw_static()

    def start(self):
        if self.sim is None or self.sim.t >= self.sim.end:
            self.make_sim()
        self.running = True
        self.t_wall = time.time()

    def pause(self):
        self.running = False

    def reset(self):
        self.running = False
        self.clear_dyn()
        self.make_sim()
        self.running = True
        self.t_wall = time.time()

    def add_person(self, side):
        if self.sim is None or self.sel is None:
            messagebox.showinfo('', 'Porneste simularea si selecteaza o trecere (click pe ea).')
            return
        motor.inject_ped(self.sim, self.net.crossings[self.sel], side, self.addkind.get())

    def breakdown(self):
        if self.sim is None or self.sel is None:
            messagebox.showinfo('', 'Porneste simularea si selecteaza o trecere (click pe ea).')
            return
        v = motor.breakdown(self.sim, self.net.crossings[self.sel])
        if v is None:
            messagebox.showinfo('', 'Nicio masina nu se apropie acum de trecere.')

    # ================================================================== bucla live
    def loop(self):
        try:
            if self.running and self.sim is not None:
                mult = SPEEDS.get(self.speed.get(), 5)
                now = time.time()
                dtw = min(0.25, now - self.t_wall)
                self.t_wall = now
                if mult >= 1e8:
                    t_end = time.time() + 0.04
                    while time.time() < t_end and self.sim.t < self.sim.end:
                        self.sim.step()
                else:
                    self.t_acc += dtw * mult
                    n = 0
                    while self.t_acc >= motor.DT and self.sim.t < self.sim.end and n < 400:
                        self.sim.step()
                        self.t_acc -= motor.DT
                        n += 1
                if self.sim.t >= self.sim.end:
                    self.running = False
                self.draw_dyn()
                self.update_live()
        except Exception as e:
            self.running = False
            messagebox.showerror('Eroare in simulare', repr(e))
            raise
        self.root.after(40, self.loop)

    def update_live(self):
        s = self.sim
        r = s.results()
        wait = sum(1 for p in s._palive if p.state == 'wait')
        txt = ('Timp %02d:%02d / %d min   masini in retea %d   pietoni %d (asteapta %d)\n'
               'Masini terminate %d, timp pierdut mediu %.1f s   teleportari %d   coliziuni %d\n'
               'Pietoni: asteptare medie la treceri %.1f s, maxim %.0f s, pe rosu %d' % (
                   s.t // 60, s.t % 60, s.end / 60, len(s._alive), len(s._palive), wait, r['cars_finished'],
                   r['car_time_loss'], r['teleports'], r['collisions'], r['ped_wait_crossings'], r['ped_wait_max'],
                   r['ped_crossed_on_red']))
        self.live.config(text=txt)
        if self.sel:
            self.show_sel()

    def show_sel(self):
        cr = self.net.crossings[self.sel]
        lines = [cr.name, 'id %s' % cr.id]
        if self.sim is None:
            self.selinfo.config(text='\n'.join(lines))
            return
        ctl = self.sim.crossing_ctl.get(cr.id)
        mode = ctl.mode if ctl else 'original'
        lines.append('Mod: %s   stare: %s' % (treceri.MODES[mode], ctl.text() if ctl else '-'))
        if cr.ped_links:
            lines.append('Semafor pietoni: %s' % {'Z': 'zebra (prioritate pietoni)', 'G': 'VERDE', 'g': 'verde',
                                                'r': 'rosu'}.get(self.sim.ped_state(cr), self.sim.ped_state(cr)))
        waiting = [p for p in self.sim._palive if p.state == 'wait' and p.cross == cr.id]
        lines.append('Asteapta: %d oameni   pe trecere: %d' % (len(waiting), len(self.sim.cross_peds.get(cr.id, ()))))
        if isinstance(ctl, treceri.Smart):
            z = ctl.zones
            lines.append('Placi: detectati %d (in realitate %d)' % (ctl.detected, ctl.actual))
            if z.storage:
                lines.append('Zona de viraj: %.0f m, %d/%d masini   senzor 1: %d   senzor 0: %d' % (
                    z.storage, len(ctl.area), z.cap, len(ctl.s1), len(ctl.s0)))
                lines.append('Rata de viraj invatata: %.1f masini/min   risc de blocare: %.0f %%' % (
                    ctl.turn_rate() * 60, 100 * ctl.spill_risk()))
            else:
                lines.append('Fara zona de viraj (trecere tip A)   senzor 1: %d   senzor 0: %d' % (len(ctl.s1), len(ctl.s0)))
            if ctl.partner:
                lines.append('Pereche cu A: %s' % ctl.partner.cr.name)
            if ctl.b_of:
                lines.append('Pereche cu B: %s' % ctl.b_of.cr.name)
        self.selinfo.config(text='\n'.join(lines))
        self.evlist.delete(0, 'end')
        if ctl is not None and getattr(ctl, 'events', None):
            for t0, msg in reversed(ctl.events):
                self.evlist.insert('end', '%02d:%02d  %s' % (t0 // 60, t0 % 60, msg))
        elif ctl is not None:
            self.evlist.insert('end', '(trecerea nu e controlata de sistemul nostru sau de un semafor normal)')

    # ================================================================== harta
    def W(self, x, y):
        return (x - self.ox) * self.sc, (self.oy - y) * self.sc

    def fit(self):
        b = self.net.bounds
        cw, ch = max(200, self.canvas.winfo_width()), max(200, self.canvas.winfo_height())
        cx, cy = 340.0, 380.0          # giratoriul
        self.sc = min(cw / 560.0, ch / 420.0)
        self.ox, self.oy = cx - cw / 2 / self.sc, cy + ch / 2 / self.sc

    def first_draw(self):
        self.fit()
        self.draw_static()

    def draw_static(self):
        c = self.canvas
        c.delete('all')
        self.items = {'x': 1}
        sc = self.sc
        ASF, ASF_RB, SIDE = '#6b7280', '#5b616b', '#d6d3d1'
        # zone de mers si intersectii
        for ln in self.net.lanes.values():
            if ln.kind == 'walkingarea' and len(ln.shape) >= 3:
                c.create_polygon([v for p in ln.shape for v in self.W(*p)], fill=SIDE, outline='')
        for J in self.net.junctions.values():
            if len(J.shape) > 2 and J.type != 'dead_end':
                c.create_polygon([v for p in J.shape for v in self.W(*p)], fill=ASF, outline='')
        rb = self.net.roundabout
        # benzile
        tram_only = []
        for ln in self.net.lanes.values():
            if ln.kind != 'normal':
                continue
            pts = [v for p in ln.shape for v in self.W(*p)]
            if len(pts) < 4:
                continue
            if ln.length < 1.0 and 'passenger' not in ln.allow:
                continue                         # bucatele de 0.2 m de trotuar: le acopera zona de mers
            if 'passenger' in ln.allow:
                col = ASF_RB if ln.edge.id in rb else ASF
            elif 'tram' in ln.allow:
                col = '#a8a29e'
                tram_only.append(ln)
            elif 'pedestrian' in ln.allow:
                col = SIDE
            else:
                col = '#cbd5e1'
            c.create_line(pts, fill=col, width=max(1, ln.width * sc + 0.5), capstyle='butt')
        # sinele de tramvai
        for ln in tram_only:
            for off in (-0.72, 0.72):
                pts = self.offset_line(ln, off)
                if pts:
                    c.create_line(pts, fill='#44403c', width=1)
        for ln in self.net.lanes.values():
            if ln.kind == 'normal' and 'passenger' in ln.allow and 'tram' in ln.allow:
                pts = [v for p in ln.shape for v in self.W(*p)]
                c.create_line(pts, fill='#78716c', width=1, dash=(2, 3))
        # linii intre benzi (punctat alb)
        if sc >= 1.5:
            for ed in self.net.edges.values():
                if ed.func or ed.lanes[0].length < 4:
                    continue
                car = [l for l in ed.lanes if 'passenger' in l.allow]
                for a, b in zip(car, car[1:]):
                    if b.index != a.index + 1:
                        continue
                    pts = self.offset_line(b, -b.width / 2.0)
                    if pts:
                        c.create_line(pts, fill='#f8fafc', width=1, dash=(6, 6))
        # sensul de mers in giratoriu: sageti
        if sc >= 1.2:
            done = 0.0
            paths = []
            for eid in rb:
                ed = self.net.edges.get(eid)
                if ed is None:
                    continue
                for ln in ed.lanes:
                    if 'passenger' in ln.allow and ln.length >= 5:
                        paths.append(ln)
            for lk in self.net.links:
                if not lk.is_ped and lk.frm.edge.id in rb and lk.to.edge.id in rb:
                    for vl in lk.via:
                        if vl.length >= 5:
                            paths.append(vl)
            placed = []
            for ln in paths:
                k = max(1, int(ln.length // 12))
                for n in range(k):
                    s0 = ln.length * (n + 0.5) / k
                    x, y, _ = retea.point_at(ln.shape, ln.cum, s0)
                    if any((x - a) ** 2 + (y - b) ** 2 < 49.0 for a, b in placed):
                        continue
                    placed.append((x, y))
                    self.arrow(ln, s0, '#f1f5f9')
        # trecerile
        self.stripes, self.frames = {}, {}
        for cid, cr in self.net.crossings.items():
            (x0, y0), (x1, y1) = cr.shape[0], cr.shape[-1]
            L = math.hypot(x1 - x0, y1 - y0) or 1
            ux, uy = (x1 - x0) / L, (y1 - y0) / L
            px, py = -uy * cr.width / 2, ux * cr.width / 2
            n = max(2, int(L / 0.6))
            its = []
            for k in range(0, n, 2):
                a, b = k / n * L, (k + 1) / n * L
                pts = [self.W(x0 + ux * a + px, y0 + uy * a + py), self.W(x0 + ux * b + px, y0 + uy * b + py),
                       self.W(x0 + ux * b - px, y0 + uy * b - py), self.W(x0 + ux * a - px, y0 + uy * a - py)]
                its.append(c.create_polygon([v for p in pts for v in p], fill='white', outline=''))
            self.stripes[cid] = its
            fr = [self.W(x0 + px, y0 + py), self.W(x1 + px, y1 + py), self.W(x1 - px, y1 - py), self.W(x0 - px, y0 - py)]
            self.frames[cid] = c.create_polygon([v for p in fr for v in p], fill='', outline='', width=max(2, 0.4 * sc))
        self.dyn, self.sig, self.plate_items, self.badges = {}, {}, {}, {}
        self.zone_items, self.bars = [], []
        if self.sim is not None:
            self.draw_overlays()
        self.draw_legend()
        self.draw_dyn()

    def offset_line(self, ln, off):
        """linia benzii deplasata lateral cu off metri (stanga = pozitiv)"""
        sh = ln.shape
        out = []
        for k, (x, y) in enumerate(sh):
            a = sh[max(0, k - 1)]
            b = sh[min(len(sh) - 1, k + 1)]
            dx, dy = b[0] - a[0], b[1] - a[1]
            d = math.hypot(dx, dy) or 1.0
            out += list(self.W(x - dy / d * off, y + dx / d * off))
        return out if len(out) >= 4 else None

    def arrow(self, ln, s0, col):
        x, y, a = retea.point_at(ln.shape, ln.cum, s0)
        L = 2.2
        ca, sa = math.cos(a), math.sin(a)
        tip = self.W(x + ca * L, y + sa * L)
        l = self.W(x - ca * L * 0.3 - sa * 1.0, y - sa * L * 0.3 + ca * 1.0)
        r = self.W(x - ca * L * 0.3 + sa * 1.0, y - sa * L * 0.3 - ca * 1.0)
        self.canvas.create_line(l[0], l[1], tip[0], tip[1], r[0], r[1], fill=col, width=max(1, 0.35 * self.sc))

    ZONE_COL = {'area': ('#c4b5fd', '#6d28d9'), 's1': ('#fdba74', '#ea580c'), 's0': ('#fde047', '#a16207')}

    def draw_overlays(self):
        """senzorii (pe carosabil) si liniile de oprire ale masinilor la trecerile controlate"""
        c, sc, sim = self.canvas, self.sc, self.sim
        mode = self.show_sensors.get()
        if mode != 'deloc':
            for cid, ctl in sim.crossing_ctl.items():
                if not isinstance(ctl, treceri.Smart):
                    continue
                if mode == 'trecerea selectata' and cid != self.sel:
                    continue
                z = ctl.zones
                for zn in ('s0', 's1', 'area'):
                    rg = z.ranges.get(zn)
                    if rg is None or not ctl.R[{'s0': 'senzor0', 's1': 'senzor1', 'area': 'senzor2'}[zn]]:
                        continue
                    a, b = rg
                    for lid, d in z.dist.items():
                        ln = z.lanes[lid]
                        L = ln.length
                        p0, p1 = max(0.0, L - (b - d)), min(L, L - (a - d))
                        if p1 - p0 < 0.3:
                            continue
                        pts = self.subline(ln, p0, p1)
                        if len(pts) < 4:
                            continue
                        it = c.create_line(pts, fill=self.ZONE_COL[zn][0], width=max(2, ln.width * sc * 0.55),
                                           capstyle='butt', stipple='gray50')
                        self.zone_items.append((it, ctl, zn))
        # linia de oprire pentru fiecare banda care are semafor (al intersectiei sau al trecerii)
        by_lane = {}
        for cid, cr in self.net.crossings.items():
            ctl = sim.crossing_ctl.get(cid)
            controlled = isinstance(ctl, (treceri.Smart, treceri.Fixed)) or (cr.tl is not None and not isinstance(ctl, treceri.Zebra))
            if controlled:
                for lk in cr.car_links:
                    by_lane.setdefault(lk.frm.id, set()).add(lk)
        for lk in self.net.links:
            if lk.tl is not None and not lk.is_ped:
                by_lane.setdefault(lk.frm.id, set()).add(lk)
        for lid, lks in by_lane.items():
            ln = self.net.lanes[lid]
            x, y, a = retea.point_at(ln.shape, ln.cum, ln.length)
            hx, hy = -math.sin(a) * ln.width / 2, math.cos(a) * ln.width / 2
            X0, Y0 = self.W(x + hx, y + hy)
            X1, Y1 = self.W(x - hx, y - hy)
            it = c.create_line(X0, Y0, X1, Y1, fill='#22c55e', width=max(3, 0.6 * sc))
            self.bars.append((it, list(lks)))

    def subline(self, ln, p0, p1):
        pts = [retea.point_at(ln.shape, ln.cum, p0)[:2]]
        for k, cl in enumerate(ln.cum):
            if p0 < cl < p1:
                pts.append(ln.shape[k])
        pts.append(retea.point_at(ln.shape, ln.cum, p1)[:2])
        return [v for p in pts for v in self.W(*p)]

    def draw_legend(self):
        c = self.canvas
        if not self.show_legend.get():
            return
        rows = [('#7c3aed', 'eticheta mov = SISTEMUL NOSTRU'), ('#1d4ed8', 'albastru = semafor normal'),
                ('#e5e7eb', 'gri deschis = zebra'), ('#6b7280', 'gri = semafor original'),
                ('#16a34a', 'contur trecere: verde = pietonii trec'), ('#dc2626', 'contur trecere: rosu = pietonii asteapta'),
                ('#e5e7eb', 'sageti = sensul in giratoriu'),
                ('#7c3aed', 'banda mov = senzor 2 / zona de viraj'), ('#ea580c', 'banda portocalie = senzor 1'),
                ('#ca8a04', 'banda galbena = senzor 0'), ('#22c55e', 'linie la capatul benzii = semaforul masinilor'),
                ('#f97316', 'punct portocaliu = pieton care asteapta'), ('#2563eb', 'albastru = masina'),
                ('#dc2626', 'rosu = tramvai')]
        y = 8
        c.create_rectangle(6, 4, 290, 10 + 15 * len(rows), fill='white', outline='#9ca3af', tags='legend')
        for col, txt in rows:
            c.create_rectangle(12, y + 2, 24, y + 12, fill=col, outline='#6b7280', tags='legend')
            c.create_text(30, y + 7, text=txt, anchor='w', font=('TkDefaultFont', 8), tags='legend')
            y += 15

    def clear_dyn(self):
        for it in list(self.dyn.values()):
            self.canvas.delete(it)
        self.dyn = {}

    def draw_dyn(self):
        c, sim = self.canvas, self.sim
        if sim is None:
            return
        sc = self.sc
        seen = set()
        for v in sim._alive:
            if v.lane is None:
                continue
            x, y, a = retea.point_at(v.lane.shape, v.lane.cum, v.pos)
            L, w = v.L, (2.4 if v.kind == 'tram' else 1.8)
            ca, sa = math.cos(a), math.sin(a)
            hx, hy = -sa * w / 2, ca * w / 2
            bx, by = x - ca * L, y - sa * L
            pts = [self.W(x + hx, y + hy), self.W(x - hx, y - hy), self.W(bx - hx, by - hy), self.W(bx + hx, by + hy)]
            flat = [q for p in pts for q in p]
            key = ('v', v.id)
            seen.add(key)
            it = self.dyn.get(key)
            fx, fy = x - ca * min(1.4, L * 0.3), y - sa * min(1.4, L * 0.3)
            ws = [self.W(fx + hx * 0.8, fy + hy * 0.8), self.W(fx - hx * 0.8, fy - hy * 0.8)]
            wflat = [q for p in ws for q in p]
            if it is None:
                col = '#dc2626' if v.kind == 'tram' else ('#ffffff' if v.emergency else '#3b82f6')
                it = c.create_polygon(flat, fill=col, outline='#991b1b' if v.emergency else '#1e3a8a')
                self.dyn[key] = it
                self.dyn[('w', v.id)] = c.create_line(*wflat, fill='#dbeafe', width=max(1, 0.4 * sc))
            else:
                c.coords(it, *flat)
                c.coords(self.dyn[('w', v.id)], *wflat)
            seen.add(('w', v.id))
            if v.brk_until > sim.t:
                c.itemconfig(it, fill='#000000')
        r = max(1.5, 0.35 * sc)
        for p in sim._palive:
            if p.state == 'wait' and p.spot and p.spot[1] and self.show_plates.get():
                pl, tiles = p.spot
                xs = [pl.tile_xy(cc, rr) for cc, rr in tiles]
                x, y = sum(q[0] for q in xs) / len(xs), sum(q[1] for q in xs) / len(xs)
            else:
                x, y = sim.ped_xy(p)
            X, Y = self.W(x, y)
            key = ('p', p.id)
            seen.add(key)
            it = self.dyn.get(key)
            col = '#f97316' if p.state == 'wait' else '#db2777'
            if it is None:
                it = c.create_oval(X - r, Y - r, X + r, Y + r, fill=col, outline='')
                self.dyn[key] = it
            else:
                c.coords(it, X - r, Y - r, X + r, Y + r)
                c.itemconfig(it, fill=col)
        for key in list(self.dyn):
            if key not in seen:
                c.delete(self.dyn.pop(key))
        # senzorii se aprind cand vad o masina
        for it, ctl, zn in self.zone_items:
            on = bool({'area': ctl.area, 's1': ctl.s1, 's0': ctl.s0}[zn])
            c.itemconfig(it, fill=self.ZONE_COL[zn][1 if on else 0], stipple='' if on else 'gray50')
        for it, lks in self.bars:
            st = min((sim.lstate(lk) for lk in lks), key=lambda q: motor.RANK.get(q, 3))
            c.itemconfig(it, fill=STATE_COLOR.get(st, '#22c55e'))
        self._label_pos = []
        order = sorted(self.net.crossings.items(), key=lambda kv: (kv[0] != self.sel,
                                                                   not isinstance(sim.crossing_ctl.get(kv[0]), treceri.Smart)))
        for cid, cr in order:
            self.draw_crossing_state(cid, cr)
        c.tag_raise('legend')
        self.status.config(text='zoom %.1f px/m   (placile se vad de la zoom 5)' % sc)

    def draw_crossing_state(self, cid, cr):
        c, sim, sc = self.canvas, self.sim, self.sc
        ctl = sim.crossing_ctl.get(cid)
        mx = sum(p[0] for p in cr.shape) / len(cr.shape)
        my = sum(p[1] for p in cr.shape) / len(cr.shape)
        X, Y = self.W(mx, my)
        smart = isinstance(ctl, treceri.Smart)
        if isinstance(ctl, (treceri.Smart, treceri.Fixed)):
            ped = 'WALK' if ctl.state == 'WALK' else ('CLEAR' if ctl.state == 'CLEAR' else 'RED')
        else:
            st = sim.ped_state(cr)
            ped = 'ZEBRA' if st == 'Z' else ('WALK' if st in 'Gg' else 'RED')
        frame = {'WALK': '#16a34a', 'CLEAR': '#f97316', 'RED': '#dc2626', 'ZEBRA': ''}[ped]
        fr = self.frames.get(cid)
        if fr is not None:
            c.itemconfig(fr, outline=frame)
        if smart and self.show_plates.get() and sc >= 5:
            self.draw_plates(cid, ctl)
        elif cid in self.plate_items:
            for i2 in self.plate_items.pop(cid):
                c.delete(i2)
        # eticheta
        if smart:
            txt, bg, fg = 'SISTEM: ' + ctl.text(), '#7c3aed', 'white'
            if ctl.state == 'GREEN' and ctl.wait_start is None:
                txt = 'SISTEM: verde masini'
            if ctl.detected:
                txt += '  (%d pe placi)' % ctl.detected
        elif isinstance(ctl, treceri.Fixed):
            txt, bg, fg = ctl.text(), '#1d4ed8', 'white'
        elif isinstance(ctl, treceri.Zebra):
            txt, bg, fg = 'zebra', '#e5e7eb', '#111827'
        elif cr.tl is not None:
            txt, bg, fg = 'semafor original: ' + ('VERDE pietoni' if ped == 'WALK' else 'rosu pietoni'), '#6b7280', 'white'
        else:
            txt, bg, fg = 'zebra (original)', '#e5e7eb', '#111827'
        show = self.show_labels.get() and (smart or sc >= 6 or cid == self.sel)
        if show and sc < 2.5 and cid != self.sel:
            txt = '%s %s' % (cr.zona, ctl.text() if smart else '')
        if show and cid != self.sel:
            for (lx, ly) in self._label_pos:
                if abs(lx - X) < 150 and abs(ly - Y) < 16:
                    show = False
                    break
        if show:
            self._label_pos.append((X, Y))
        b = self.badges.get(cid)
        if not show:
            if b:
                c.delete(b[0]); c.delete(b[1])
                del self.badges[cid]
            return
        if cid == self.sel:
            bg = '#facc15' if not smart else '#5b21b6'
            fg = '#111827' if not smart else 'white'
        if b is None:
            t_it = c.create_text(X + 10, Y - 10, text=txt, anchor='sw', fill=fg, font=('TkDefaultFont', 8, 'bold'))
            r_it = c.create_rectangle(c.bbox(t_it), fill=bg, outline='#111827')
            c.tag_lower(r_it, t_it)
            self.badges[cid] = (r_it, t_it)
        else:
            r_it, t_it = b
            c.coords(t_it, X + 10, Y - 10)
            c.itemconfig(t_it, text=txt, fill=fg)
            bb = c.bbox(t_it)
            c.coords(r_it, bb[0] - 2, bb[1] - 1, bb[2] + 2, bb[3] + 1)
            c.itemconfig(r_it, fill=bg)
            c.tag_raise(r_it)
            c.tag_raise(t_it)

    def draw_plates(self, cid, ctl):
        c = self.canvas
        t = self.sim.t
        its = self.plate_items.get(cid)
        tiles = []
        for pl in ctl.plates:
            load = pl.load(t, 0)
            for rr in range(pl.rows):
                for cc in range(pl.cols):
                    kg = load.get((cc, rr), 0.0)
                    col = '#ede9fe' if kg <= 0 else ('#fde047' if kg < 30 else '#f59e0b' if kg < 70 else '#b45309')
                    tiles.append(([v for p in pl.tile_poly(cc, rr) for v in self.W(*p)], col))
        if its is None or len(its) != len(tiles):
            if its:
                for i2 in its:
                    c.delete(i2)
            its = [c.create_polygon(0, 0, 0, 0, 0, 0, outline='#a78bfa') for _ in tiles]
            self.plate_items[cid] = its
        for it, (pts, col) in zip(its, tiles):
            c.coords(it, *pts)
            c.itemconfig(it, fill=col)

    def on_press(self, e):
        self.drag = (e.x, e.y, e.x, e.y)

    def on_drag(self, e):
        x0, y0, sx, sy = self.drag
        dx, dy = e.x - x0, e.y - y0
        self.canvas.move('all', dx, dy)
        self.canvas.move('legend', -dx, -dy)
        self.ox -= dx / self.sc
        self.oy += dy / self.sc
        self.drag = (e.x, e.y, sx, sy)

    def on_release(self, e):
        x0, y0, sx, sy = self.drag
        if abs(e.x - sx) + abs(e.y - sy) > 4:
            return
        best, bd = None, 18
        for cid, cr in self.net.crossings.items():
            mx = sum(p[0] for p in cr.shape) / len(cr.shape)
            my = sum(p[1] for p in cr.shape) / len(cr.shape)
            X, Y = self.W(mx, my)
            d = math.hypot(X - e.x, Y - e.y)
            if d < bd:
                best, bd = cid, d
        self.sel = best
        if best and (e.state & 0x0004):
            self.toggle_check(best)
        if best:
            self.show_sel()
            self.select_in_table(best)
            if self.show_sensors.get() == 'trecerea selectata':
                self.draw_static()

    def on_wheel(self, e):
        self.zoom(1.2 if e.delta > 0 else 1 / 1.2, e.x, e.y)

    def zoom(self, f, x, y):
        nsc = max(0.5, min(40.0, self.sc * f))
        f = nsc / self.sc
        wx, wy = self.ox + x / self.sc, self.oy - y / self.sc
        self.sc = nsc
        self.ox, self.oy = wx - x / self.sc, wy + y / self.sc
        self.draw_static()

    # ================================================================== tab Treceri
    def tab_cross(self, f):
        self.checked = set()
        ttk.Label(f, text='Click pe un rand il bifeaza / debifeaza. Poti bifa oricate, apoi schimbi modul sau '
                          'regulile la toate deodata. Pe harta: Ctrl + click pe o trecere o bifeaza.',
                  foreground='#5b21b6', wraplength=500).pack(anchor='w')
        cols = ('mod', 'reguli')
        self.ct = ttk.Treeview(f, columns=cols, height=13, selectmode='none')
        self.ct.heading('#0', text='Trecere')
        self.ct.heading('mod', text='Mod')
        self.ct.heading('reguli', text='Reguli')
        self.ct.column('#0', width=250)
        self.ct.column('mod', width=105)
        self.ct.column('reguli', width=140)
        self.ct.tag_configure('on', background='#ede9fe')
        self.ct.pack(fill='both', expand=True)
        self.ct.bind('<Button-1>', self.on_ct_click)
        g = ttk.Frame(f)
        g.pack(fill='x', pady=(4, 0))
        ttk.Label(g, text='Bifeaza:').pack(side='left')
        ttk.Button(g, style='S.TButton', text='Tot', command=lambda: self.check_where(lambda c, m: True)).pack(side='left', padx=1)
        ttk.Button(g, style='S.TButton', text='Nimic', command=lambda: self.check_where(lambda c, m: False)).pack(side='left', padx=1)
        ttk.Button(g, style='S.TButton', text='Cele 8 din SUMO', command=lambda: self.check_where(lambda c, m: c in EIGHT, add=True)).pack(side='left', padx=1)
        ttk.Button(g, style='S.TButton', text='Doar cu masini', command=lambda: self.check_where(
            lambda c, m: c in self.checked and self.net.crossings[c].cars)).pack(side='left', padx=1)
        g = ttk.Frame(f)
        g.pack(fill='x', pady=(2, 0))
        ttk.Label(g, text='Bifeaza tot ce e:').pack(side='left')
        short = {'original': 'Original', 'zebra': 'Zebra', 'semafor': 'Semafor', 'inteligent': 'Sistemul nostru'}
        for m, lab in treceri.MODES.items():
            ttk.Button(g, style='S.TButton', text=short[m], command=lambda m=m: self.check_where(lambda c, mm, m=m: mm == m, add=True)).pack(side='left', padx=1)
        g = ttk.Frame(f)
        g.pack(fill='x', pady=2)
        ttk.Label(g, text='Bifeaza zona:').pack(side='left')
        zones = sorted({cr.zona for cr in self.net.crossings.values() if cr.zona})
        for z in zones:
            ttk.Button(g, style='S.TButton', text=z, width=3, command=lambda z=z: self.check_where(
                lambda c, m, z=z: self.net.crossings[c].zona == z, add=True)).pack(side='left', padx=1)
        g = ttk.LabelFrame(f, text='Pentru trecerile bifate', padding=6)
        g.pack(fill='x', pady=4)
        self.ct_count = ttk.Label(g, text='0 bifate', font=('TkDefaultFont', 9, 'bold'))
        self.ct_count.grid(row=0, column=0, columnspan=3, sticky='w')
        ttk.Label(g, text='Mod').grid(row=1, column=0, sticky='w')
        self.ct_mode = tk.StringVar(value=treceri.MODES['inteligent'])
        ttk.Combobox(g, textvariable=self.ct_mode, values=list(treceri.MODES.values()), state='readonly',
                     width=16).grid(row=1, column=1, padx=4, sticky='w')
        ttk.Button(g, style='S.TButton', text='Schimba modul', command=self.apply_mode).grid(row=1, column=2, sticky='w')
        ttk.Label(g, text='Reguli').grid(row=2, column=0, sticky='w')
        self.ct_rules = tk.StringVar()
        self.ct_rules_cb = ttk.Combobox(g, textvariable=self.ct_rules, state='readonly', width=22)
        self.ct_rules_cb.grid(row=2, column=1, padx=4, sticky='w')
        ttk.Button(g, style='S.TButton', text='Schimba regulile', command=self.apply_rules).grid(row=2, column=2, sticky='w')
        ttk.Label(g, text='"Schimba regulile" pune si sistemul nostru pe trecerile bifate care nu il aveau.',
                  foreground='gray').grid(row=3, column=0, columnspan=3, sticky='w')
        g = ttk.LabelFrame(f, text='Perechi A (drumul principal) - B (unde intra masinile care vireaza)', padding=4)
        g.pack(fill='x', pady=4)
        self.pairs_lb = tk.Listbox(g, height=3)
        self.pairs_lb.pack(fill='x')
        r = ttk.Frame(g)
        r.pack(fill='x')
        self.pa, self.pb = tk.StringVar(), tk.StringVar()
        names = sorted(cr.name for cr in self.net.crossings.values())
        ttk.Combobox(r, textvariable=self.pa, values=names, width=19, state='readonly').pack(side='left')
        ttk.Combobox(r, textvariable=self.pb, values=names, width=19, state='readonly').pack(side='left', padx=2)
        ttk.Button(r, text='Adauga', command=self.add_pair).pack(side='left')
        ttk.Button(r, text='Sterge', command=self.del_pair).pack(side='left', padx=2)
        ttk.Label(f, text='Schimbarile se aplica imediat pe harta si in comparatii.', foreground='gray').pack(anchor='w')
        self.refresh_ct()

    def refresh_ct(self):
        self.ct.delete(*self.ct.get_children())
        for cid, cr in sorted(self.net.crossings.items(), key=lambda kv: kv[1].name):
            cfg = self.S['treceri'].get(cid, {})
            m = cfg.get('mod', 'original')
            on = cid in self.checked
            self.ct.insert('', 'end', iid=cid, text=('☑ ' if on else '☐ ') + cr.name + ('  [semafor]' if cr.tl else ''),
                           values=(treceri.MODES[m], cfg.get('reguli', '') if m == 'inteligent' else ''),
                           tags=('on',) if on else ())
        self.ct_rules_cb['values'] = list(self.S['reguli'])
        if self.ct_rules.get() not in self.S['reguli'] and self.S['reguli']:
            self.ct_rules.set(next(iter(self.S['reguli'])))
        self.ct_count.config(text='%d %s' % (len(self.checked), 'bifata' if len(self.checked) == 1 else 'bifate'))
        self.pairs_lb.delete(0, 'end')
        for a, b in self.S['perechi']:
            na = self.net.crossings[a].name if a in self.net.crossings else a
            nb = self.net.crossings[b].name if b in self.net.crossings else b
            self.pairs_lb.insert('end', 'A: %s   ->   B: %s' % (na, nb))
        self.save_settings()
        if hasattr(self, 'goto_cb'):
            self.apply_live()

    def refresh_checks(self):
        """doar bifele (fara sa reaplice setarile)"""
        for cid in self.ct.get_children():
            cr = self.net.crossings[cid]
            on = cid in self.checked
            self.ct.item(cid, text=('☑ ' if on else '☐ ') + cr.name + ('  [semafor]' if cr.tl else ''),
                         tags=('on',) if on else ())
        self.ct_count.config(text='%d %s' % (len(self.checked), 'bifata' if len(self.checked) == 1 else 'bifate'))
        if hasattr(self, 'rs_apply_lbl'):
            self.rs_apply_lbl.config(text='(%d treceri bifate in tabul Treceri)' % len(self.checked))

    def on_ct_click(self, e):
        cid = self.ct.identify_row(e.y)
        if not cid:
            return 'break'
        self.toggle_check(cid)
        self.sel = cid
        cfg = self.S['treceri'].get(cid, {})
        if cfg.get('reguli') in self.S['reguli']:
            self.ct_rules.set(cfg['reguli'])
        return 'break'

    def toggle_check(self, cid):
        if cid in self.checked:
            self.checked.discard(cid)
        else:
            self.checked.add(cid)
        self.refresh_checks()

    def check_where(self, fn, add=False):
        new = {cid for cid in self.net.crossings
               if fn(cid, self.S['treceri'].get(cid, {}).get('mod', 'original'))}
        self.checked = (self.checked | new) if add else new
        self.refresh_checks()

    def select_in_table(self, cid):
        try:
            self.ct.see(cid)
        except tk.TclError:
            pass

    def mode_key(self, label):
        return [k for k, v in treceri.MODES.items() if v == label][0]

    def _need_checked(self):
        if not self.checked:
            messagebox.showinfo('Nimic bifat', 'Bifeaza intai trecerile (click pe randuri, sau butoanele "Bifeaza").')
            return False
        return True

    def apply_mode(self):
        if not self._need_checked():
            return
        m = self.mode_key(self.ct_mode.get())
        for cid in self.checked:
            cfg = dict(self.S['treceri'].get(cid, {}))
            cfg['mod'] = m
            if m == 'inteligent' and cfg.get('reguli') not in self.S['reguli']:
                cfg['reguli'] = self.ct_rules.get()
            self.S['treceri'][cid] = cfg
        self.refresh_ct()

    def apply_rules(self, name=None):
        if not self._need_checked():
            return
        name = name or self.ct_rules.get()
        for cid in self.checked:
            self.S['treceri'][cid] = dict(mod='inteligent', reguli=name)
        self.refresh_ct()

    def name_to_id(self, n):
        for cid, cr in self.net.crossings.items():
            if cr.name == n:
                return cid
        return None

    def add_pair(self):
        a, b = self.name_to_id(self.pa.get()), self.name_to_id(self.pb.get())
        if a and b and a != b:
            self.S['perechi'].append([a, b])
            self.refresh_ct()

    def del_pair(self):
        for i in sorted(self.pairs_lb.curselection(), reverse=True):
            del self.S['perechi'][i]
        self.refresh_ct()

    # ================================================================== tab Reguli
    def tab_rules(self, f):
        self.loading = True
        r = ttk.Frame(f)
        r.pack(fill='x')
        ttk.Label(r, text='Set de reguli').pack(side='left')
        self.rs_name = tk.StringVar(value=next(iter(self.S['reguli'])))
        self.rs_cb = ttk.Combobox(r, textvariable=self.rs_name, values=list(self.S['reguli']), state='readonly', width=28)
        self.rs_cb.pack(side='left', padx=4)
        self.rs_cb.bind('<<ComboboxSelected>>', lambda e: self.load_rules_editor())
        ttk.Button(r, text='Nou', command=self.rs_new).pack(side='left')
        ttk.Button(r, text='Copiaza', command=self.rs_copy).pack(side='left', padx=2)
        ttk.Button(r, text='Redenumeste', command=self.rs_rename).pack(side='left')
        ttk.Button(r, text='Sterge', command=self.rs_delete).pack(side='left', padx=2)
        r = ttk.Frame(f)
        r.pack(fill='x', pady=(4, 0))
        ttk.Button(r, text='Pune acest set pe trecerile bifate',
                   command=lambda: self.apply_rules(self.rs_name.get())).pack(side='left')
        self.rs_apply_lbl = ttk.Label(r, text='(0 treceri bifate in tabul Treceri)', foreground='gray')
        self.rs_apply_lbl.pack(side='left', padx=6)
        outer = ttk.Frame(f)
        outer.pack(fill='both', expand=True, pady=4)
        cv = tk.Canvas(outer, highlightthickness=0)
        sb = ttk.Scrollbar(outer, orient='vertical', command=cv.yview)
        self.rs_frame = ttk.Frame(cv)
        self.rs_frame.bind('<Configure>', lambda e: cv.configure(scrollregion=cv.bbox('all')))
        cv.create_window((0, 0), window=self.rs_frame, anchor='nw')
        cv.configure(yscrollcommand=sb.set)
        cv.pack(side='left', fill='both', expand=True)
        sb.pack(side='right', fill='y')
        self.rs_vars = {}
        row = 0
        sections = {'detectie': 'Placile de presiune', 'countdown_start': 'Countdown', 'verde_minim_masini': 'Semafor',
                    'senzor0': 'Senzorii din carosabil',
                    'hold': 'Masinile de pe senzori cand se termina countdown-ul', 'fix2': 'Fix-uri'}
        for k, label in treceri.RULE_LABELS.items():
            if k in sections:
                ttk.Label(self.rs_frame, text=sections[k], font=('TkDefaultFont', 10, 'bold')).grid(
                    row=row, column=0, columnspan=2, sticky='w', pady=(8, 2))
                row += 1
            dv = treceri.RULE_DEFAULTS[k]
            ttk.Label(self.rs_frame, text=label).grid(row=row, column=0, sticky='w')
            if k in treceri.CHOICES:
                var = tk.StringVar()
                w = ttk.Combobox(self.rs_frame, textvariable=var, values=list(treceri.CHOICES[k].values()),
                                 state='readonly', width=20)
            elif isinstance(dv, bool):
                var = tk.BooleanVar()
                w = ttk.Checkbutton(self.rs_frame, variable=var)
            else:
                var = tk.StringVar()
                w = ttk.Entry(self.rs_frame, textvariable=var, width=10)
            w.grid(row=row, column=1, sticky='w', padx=6)
            var.trace_add('write', lambda *a, k=k: self.rs_changed(k))
            self.rs_vars[k] = var
            row += 1
        ttk.Label(f, text='Fix 1 (mutarea trecerii B mai departe pe strada laterala) cere schimbarea strazii, nu '
                          'e aici. Schimbarile se aplica imediat pe harta si in comparatii.', foreground='gray',
                  wraplength=480).pack(anchor='w')
        self.loading = False
        self.load_rules_editor()

    def load_rules_editor(self):
        self.loading = True
        R = treceri.rules_with(self.S['reguli'].get(self.rs_name.get(), {}))
        for k, var in self.rs_vars.items():
            v = R[k]
            if k in treceri.CHOICES:
                var.set(treceri.CHOICES[k].get(v, v))
            else:
                var.set(v)
        self.loading = False

    def rs_changed(self, k):
        if self.loading:
            return
        name = self.rs_name.get()
        rs = self.S['reguli'].setdefault(name, {})
        var = self.rs_vars[k]
        dv = treceri.RULE_DEFAULTS[k]
        try:
            if k in treceri.CHOICES:
                lab = var.get()
                val = [kk for kk, vv in treceri.CHOICES[k].items() if vv == lab][0]
            elif isinstance(dv, bool):
                val = bool(var.get())
            elif isinstance(dv, int):
                val = int(float(var.get()))
            else:
                val = float(var.get())
        except (ValueError, IndexError, tk.TclError):
            return
        if val == dv:
            rs.pop(k, None)
        else:
            rs[k] = val
        self.save_settings()
        self.apply_live()

    def rs_refresh(self):
        self.rs_cb['values'] = list(self.S['reguli'])
        self.ct_rules_cb['values'] = list(self.S['reguli'])
        self.load_rules_editor()
        self.save_settings()

    def rs_new(self):
        n = simpledialog.askstring('Set nou', 'Nume:')
        if n and n not in self.S['reguli']:
            self.S['reguli'][n] = {}
            self.rs_name.set(n)
            self.rs_refresh()

    def rs_copy(self):
        n = simpledialog.askstring('Copiaza', 'Nume pentru copie:')
        if n and n not in self.S['reguli']:
            self.S['reguli'][n] = dict(self.S['reguli'].get(self.rs_name.get(), {}))
            self.rs_name.set(n)
            self.rs_refresh()

    def rs_rename(self):
        old = self.rs_name.get()
        n = simpledialog.askstring('Redenumeste', 'Nume nou:', initialvalue=old)
        if n and n != old and n not in self.S['reguli']:
            self.S['reguli'] = OrderedDict((n if k == old else k, v) for k, v in self.S['reguli'].items())
            for cfg in self.S['treceri'].values():
                if cfg.get('reguli') == old:
                    cfg['reguli'] = n
            self.rs_name.set(n)
            self.rs_refresh()
            self.refresh_ct()

    def rs_delete(self):
        if len(self.S['reguli']) <= 1:
            return
        n = self.rs_name.get()
        del self.S['reguli'][n]
        self.rs_name.set(next(iter(self.S['reguli'])))
        self.rs_refresh()

    # ================================================================== tab Altele
    def tab_other(self, f):
        g = ttk.LabelFrame(f, text='Semafor normal (la trecerile care nu au deja semafor)', padding=6)
        g.pack(fill='x')
        self.sp_vars = {}
        labels = dict(verde_masini='Verde masini (s)', galben='Galben (s)', tot_rosu='Tot rosu (s)',
                      walk='Verde pietoni (s)', clear='Eliberare (s)', la_cerere='Doar la cerere (buton)')
        for i, (k, dv) in enumerate(treceri.SEMAFOR_DEFAULTS.items()):
            ttk.Label(g, text=labels[k]).grid(row=i, column=0, sticky='w')
            var = tk.BooleanVar(value=self.S['semafor'].get(k, dv)) if isinstance(dv, bool) else \
                tk.StringVar(value=self.S['semafor'].get(k, dv))
            w = ttk.Checkbutton(g, variable=var) if isinstance(dv, bool) else ttk.Entry(g, textvariable=var, width=8)
            w.grid(row=i, column=1, sticky='w', padx=6)
            var.trace_add('write', lambda *a, k=k: self.sp_changed(k))
            self.sp_vars[k] = var
        ttk.Label(g, text='Unde exista deja semafor (pod, Nicolina), "Semafor normal" foloseste programul original.',
                  foreground='gray', wraplength=460).grid(row=10, column=0, columnspan=2, sticky='w')
        g = ttk.LabelFrame(f, text='Pietoni', padding=6)
        g.pack(fill='x', pady=6)
        self.imp_on = tk.BooleanVar(value=self.S.get('nerabdare_on', False))
        ttk.Checkbutton(g, text='Pietoni nerabdatori (traverseaza pe rosu cand nu vine nimic)',
                        variable=self.imp_on, command=self.other_changed).grid(row=0, column=0, columnspan=2, sticky='w')
        self.imp_after = tk.StringVar(value=self.S.get('nerabdare_dupa', 40.0))
        self.imp_prob = tk.StringVar(value=self.S.get('nerabdare_prob', 0.05))
        ttk.Label(g, text='   dupa (s)').grid(row=1, column=0, sticky='w')
        ttk.Entry(g, textvariable=self.imp_after, width=8).grid(row=1, column=1, sticky='w')
        ttk.Label(g, text='   probabilitate pe secunda').grid(row=2, column=0, sticky='w')
        ttk.Entry(g, textvariable=self.imp_prob, width=8).grid(row=2, column=1, sticky='w')
        for v in (self.imp_after, self.imp_prob):
            v.trace_add('write', lambda *a: self.other_changed())
        g = ttk.LabelFrame(f, text='Semafoarele existente', padding=6)
        g.pack(fill='x')
        self.skip_ped = tk.BooleanVar(value=self.S.get('fara_faza_pietoni', True))
        ttk.Checkbutton(g, text='Sari peste faza exclusiva pentru pietoni cand toate trecerile acelui semafor sunt '
                                'zebre sau sistemul nostru', variable=self.skip_ped, command=self.other_changed).pack(anchor='w')

    def sp_changed(self, k):
        try:
            dv = treceri.SEMAFOR_DEFAULTS[k]
            self.S['semafor'][k] = bool(self.sp_vars[k].get()) if isinstance(dv, bool) else float(self.sp_vars[k].get())
            self.save_settings()
            self.apply_live()
        except (ValueError, tk.TclError):
            pass

    def other_changed(self):
        try:
            self.S['nerabdare_on'] = bool(self.imp_on.get())
            self.S['nerabdare_dupa'] = float(self.imp_after.get())
            self.S['nerabdare_prob'] = float(self.imp_prob.get())
            self.S['fara_faza_pietoni'] = bool(self.skip_ped.get())
            self.save_settings()
            self.apply_live()
        except (ValueError, tk.TclError):
            pass

    # ================================================================== tab Comparare
    def tab_cmp(self, f):
        g = ttk.LabelFrame(f, text='Configuratii de comparat', padding=4)
        g.pack(fill='x')
        self.cl = tk.Listbox(g, height=6, selectmode='extended')
        self.cl.pack(fill='x')
        r = ttk.Frame(g)
        r.pack(fill='x', pady=2)
        ttk.Button(r, text='Adauga configuratia curenta', command=self.cmp_add).pack(side='left')
        ttk.Button(r, text='Adauga modele', command=self.cmp_presets).pack(side='left', padx=2)
        ttk.Button(r, text='Incarca in editor', command=self.cmp_load).pack(side='left')
        ttk.Button(r, text='Sterge', command=self.cmp_del).pack(side='left', padx=2)
        g = ttk.Frame(f)
        g.pack(fill='x', pady=4)
        ttk.Label(g, text='Minute simulate').grid(row=0, column=0, sticky='w')
        self.c_min = tk.IntVar(value=30)
        ttk.Spinbox(g, from_=1, to=240, textvariable=self.c_min, width=5).grid(row=0, column=1, sticky='w')
        ttk.Label(g, text=' Repetari').grid(row=0, column=2)
        self.c_rep = tk.IntVar(value=5)
        ttk.Spinbox(g, from_=1, to=50, textvariable=self.c_rep, width=4).grid(row=0, column=3)
        ttk.Label(g, text=' In paralel').grid(row=0, column=4)
        self.c_par = tk.IntVar(value=max(1, (os.cpu_count() or 2) - 1))
        ttk.Spinbox(g, from_=1, to=64, textvariable=self.c_par, width=4).grid(row=0, column=5)
        ttk.Label(g, text='Nivelul de trafic si "Trafic x" se iau din tabul Simulare.', foreground='gray').grid(
            row=1, column=0, columnspan=6, sticky='w')
        r = ttk.Frame(f)
        r.pack(fill='x')
        self.c_go = ttk.Button(r, text='▶ Compara', command=self.cmp_run)
        self.c_go.pack(side='left')
        ttk.Button(r, text='■ Opreste', command=lambda: self.cmp_stop and self.cmp_stop.set()).pack(side='left', padx=4)
        ttk.Button(r, text='Deschide folderul cu rezultate', command=self.open_folder).pack(side='right')
        self.c_prog = ttk.Progressbar(f, mode='determinate')
        self.c_prog.pack(fill='x', pady=3)
        self.c_res = ttk.Treeview(f, show='headings', height=12)
        xs = ttk.Scrollbar(f, orient='horizontal', command=self.c_res.xview)
        self.c_res.configure(xscrollcommand=xs.set)
        self.c_res.pack(fill='both', expand=True)
        xs.pack(fill='x')
        r = ttk.Frame(f)
        r.pack(fill='x')
        ttk.Label(r, text='Grafic:').pack(side='left')
        self.c_plot = tk.StringVar(value='Masini in retea')
        cb = ttk.Combobox(r, textvariable=self.c_plot, values=['Masini in retea', 'Pietoni care asteapta'],
                          state='readonly', width=20)
        cb.pack(side='left')
        cb.bind('<<ComboboxSelected>>', lambda e: self.plot())
        self.c_saved = ttk.Label(r, text='', foreground='gray')
        self.c_saved.pack(side='left', padx=6)
        self.c_canvas = tk.Canvas(f, height=170, bg='white', highlightthickness=0)
        self.c_canvas.pack(fill='x')
        self.cmp_data = None
        self.cmp_refresh()

    def cmp_refresh(self):
        self.cl.delete(0, 'end')
        for i, it in enumerate(self.compare_list):
            self.cl.insert('end', '[%d] %s  -  %s' % (i + 1, it['nume'], describe(it['setup'], self.net)))
        self.save_settings()

    def cmp_add(self):
        n = simpledialog.askstring('Nume', 'Nume pentru configuratie:', initialvalue='Configuratia %d' % (len(self.compare_list) + 1))
        if n:
            self.compare_list.append(dict(nume=n, setup=copy.deepcopy(self.S)))
            self.cmp_refresh()

    def cmp_presets(self):
        first = next(iter(self.S['reguli']))
        def with_modes(fn):
            s = copy.deepcopy(self.S)
            s['treceri'] = {cid: fn(cid) for cid in self.net.crossings}
            return s
        presets = [
            ('Original (ca in SUMO)', with_modes(lambda c: dict(mod='original'))),
            ('Toate zebre', with_modes(lambda c: dict(mod='zebra'))),
            ('Semafoare normale', with_modes(lambda c: dict(mod='semafor'))),
        ]
        for rname in list(self.S['reguli'])[:4]:
            presets.append(('Sistemul nostru (8 treceri): ' + rname,
                            with_modes(lambda c, r=rname: dict(mod='inteligent', reguli=r) if c in EIGHT else dict(mod='original'))))
        for n, s in presets:
            self.compare_list.append(dict(nume=n, setup=s))
        self.cmp_refresh()

    def cmp_load(self):
        sel = self.cl.curselection()
        if sel:
            self.S = copy.deepcopy(self.compare_list[sel[0]]['setup'])
            self.refresh_ct()
            self.rs_name.set(next(iter(self.S['reguli'])))
            self.rs_refresh()

    def cmp_del(self):
        for i in sorted(self.cl.curselection(), reverse=True):
            del self.compare_list[i]
        self.cmp_refresh()

    def cmp_run(self):
        if self.cmp_stop is not None or not self.compare_list:
            return
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        outdir = os.path.join(rez_dir(), 'comparatie_%s' % stamp)
        os.makedirs(outdir, exist_ok=True)
        jobs = []
        for i, it in enumerate(self.compare_list):
            sp = os.path.join(outdir, 'setup%d.json' % i)
            with open(sp, 'w', encoding='utf-8') as f:
                json.dump(setup_for_run(it['setup']), f, ensure_ascii=False)
            for s in range(1, int(self.c_rep.get()) + 1):
                jobs.append((i, s, sp, os.path.join(outdir, 'rez%d_seed%d.json' % (i, s))))
        meta = dict(level=self.level.get(), scale=float(self.scale.get()), minutes=float(self.c_min.get()),
                    reps=int(self.c_rep.get()), names=[it['nume'] for it in self.compare_list],
                    descr=[describe(it['setup'], self.net) for it in self.compare_list], stamp=stamp, outdir=outdir)
        self.cmp_stop = threading.Event()
        self.c_go.state(['disabled'])
        self.c_prog.config(maximum=len(jobs), value=0)
        self.c_saved.config(text='Ruleaza %d simulari ...' % len(jobs))
        threading.Thread(target=self.cmp_worker, args=(jobs, meta, int(self.c_par.get())), daemon=True).start()

    def cmp_worker(self, jobs, meta, par):
        script = os.path.join(HERE, 'rulare.py')
        flags = 0x08000000 if os.name == 'nt' else 0
        pending, running, res, errors = list(jobs), [], {}, []
        while (pending or running) and not self.cmp_stop.is_set():
            while pending and len(running) < par:
                i, s, sp, out = pending.pop(0)
                cmd = [sys.executable, script, '--setup', sp, '--nivel', meta['level'], '--minute', str(meta['minutes']),
                       '--seed', str(s), '--scale', str(meta['scale']), '--out', out]
                pr = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, creationflags=flags)
                running.append((pr, i, s, out))
            time.sleep(0.3)
            for item in list(running):
                pr, i, s, out = item
                if pr.poll() is None:
                    continue
                running.remove(item)
                try:
                    with open(out, encoding='utf-8') as f:
                        res.setdefault(i, []).append(json.load(f))
                except Exception:
                    errors.append('[%d] seed %d: %s' % (i + 1, s, pr.stderr.read().decode(errors='replace')[-600:]))
                self.q.put(('prog', len(jobs) - len(pending) - len(running)))
        for pr, *_ in running:
            pr.kill()
        self.q.put(('done', res, meta, errors, self.cmp_stop.is_set()))

    def cmp_show(self, res, meta, errors, stopped):
        names = meta['names']
        t = self.c_res
        t.delete(*t.get_children())
        cols = ['m'] + ['c%d' % i for i in range(len(names))]
        t.configure(columns=cols)
        t.heading('m', text='Masura')
        t.column('m', width=290, anchor='w', stretch=False)
        for i, n in enumerate(names):
            t.heading('c%d' % i, text='[%d] %s' % (i + 1, n[:18]))
            t.column('c%d' % i, width=125, anchor='e', stretch=False)
        table = []
        for key, label, lower in rulare.METRICS:
            cells = []
            for i in range(len(names)):
                vals = [r.get(key, 0.0) for r in res.get(i, []) if isinstance(r.get(key, 0.0), (int, float))]
                m = statistics.mean(vals) if vals else None
                sd = statistics.stdev(vals) if len(vals) > 1 else 0.0
                cells.append((m, sd))
            ok = [m for m, _ in cells if m is not None]
            best = None
            if lower is not None and len(ok) > 1 and max(ok) != min(ok):
                best = min(ok) if lower else max(ok)
            row = [label]
            for m, sd in cells:
                row.append('-' if m is None else (('★ ' if best is not None and m == best else '') + '%.1f' % m +
                                                  (' ±%.1f' % sd if sd else '')))
            t.insert('', 'end', values=row)
            table.append((label, cells))
        self.cmp_data = (res, meta)
        self.plot()
        base = os.path.join(rez_dir(), 'comparatie_%s' % meta['stamp'])
        with open(base + '.csv', 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['masura'] + sum([[n + ' medie', n + ' +/-'] for n in names], []))
            for label, cells in table:
                w.writerow([label] + sum([['' if m is None else round(m, 3), round(sd, 3)] for m, sd in cells], []))
        lines = ['PODU ROS - comparatie %s' % meta['stamp'],
                 'Nivel: %s   trafic x%.2f   %d minute simulate   %d repetari%s' % (
                     meta['level'], meta['scale'], meta['minutes'], meta['reps'], '   (OPRIT DEVREME)' if stopped else ''), '']
        for i, n in enumerate(names):
            lines.append('  [%d] %s : %s  (%d rulari)' % (i + 1, n, meta['descr'][i], len(res.get(i, []))))
        lines.append('')
        wl = max(len(l) for l, _ in table) + 2
        lines.append(' ' * wl + ''.join('%16s' % ('[%d]' % (i + 1)) for i in range(len(names))))
        for label, cells in table:
            lines.append(label.ljust(wl) + ''.join('%16s' % ('-' if m is None else '%.1f' % m + (' ±%.1f' % sd if sd else ''))
                                                   for m, sd in cells))
        lines += ['', 'Diferentele mai mici decat ± sunt probabil intamplatoare: mai multe repetari.']
        if errors:
            lines += ['', 'RULARI ESUATE:'] + errors
        with open(base + '.txt', 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')
        self.c_saved.config(text='Salvat: %s.txt / .csv' % os.path.relpath(base, cerere.ROOT))
        if errors:
            messagebox.showwarning('Unele rulari au esuat', '\n\n'.join(errors[:4]))

    def plot(self):
        c = self.c_canvas
        c.delete('all')
        if not self.cmp_data:
            return
        res, meta = self.cmp_data
        k = 1 if self.c_plot.get() == 'Masini in retea' else 2
        W, H = max(300, c.winfo_width()), 170
        series = {}
        ymax, tmax = 1.0, 1.0
        for i, rs in res.items():
            acc = {}
            for r in rs:
                for row in r.get('series', []):
                    acc.setdefault(row[0], []).append(row[k])
            pts = sorted((t, sum(v) / len(v)) for t, v in acc.items())
            series[i] = pts
            if pts:
                ymax = max(ymax, max(p[1] for p in pts))
                tmax = max(tmax, pts[-1][0])
        cols = ['#2563eb', '#dc2626', '#16a34a', '#9333ea', '#ea580c', '#0891b2', '#4b5563', '#be185d']
        X = lambda t: 40 + (W - 50) * t / tmax
        Y = lambda y: H - 20 - (H - 30) * y / ymax
        c.create_line(40, H - 20, W - 10, H - 20)
        c.create_line(40, 10, 40, H - 20)
        c.create_text(36, 12, text='%.0f' % ymax, anchor='e', font=('TkDefaultFont', 7))
        c.create_text(W - 10, H - 8, text='%d min' % (tmax / 60), anchor='e', font=('TkDefaultFont', 7))
        for i, pts in series.items():
            if len(pts) > 1:
                c.create_line([v for t, y in pts for v in (X(t), Y(y))], fill=cols[i % len(cols)], width=2)
                c.create_text(45 + 30 * i, 14, text='[%d]' % (i + 1), fill=cols[i % len(cols)], anchor='w',
                              font=('TkDefaultFont', 8, 'bold'))

    def open_folder(self):
        d = rez_dir()
        os.makedirs(d, exist_ok=True)
        if os.name == 'nt':
            os.startfile(d)
        elif sys.platform == 'darwin':
            subprocess.Popen(['open', d])
        else:
            subprocess.Popen(['xdg-open', d])

    def poll(self):
        try:
            while True:
                m = self.q.get_nowait()
                if m[0] == 'prog':
                    self.c_prog.config(value=m[1])
                elif m[0] == 'done':
                    self.cmp_stop = None
                    self.c_go.state(['!disabled'])
                    self.cmp_show(*m[1:])
        except queue.Empty:
            pass
        self.root.after(250, self.poll)


def main():
    root = tk.Tk()
    if not cerere.is_project(cerere.ROOT):
        root.withdraw()
        messagebox.showinfo('Unde e proiectul?', 'Nu gasesc folderul PoduRos_SUMO (cel cu "retea" si "trafic").\n'
                                                 'Alege-l in fereastra urmatoare. Il tin minte pentru data viitoare.')
        while True:
            d = filedialog.askdirectory(title='Alege folderul PoduRos_SUMO (cu retea si trafic)')
            if not d:
                root.destroy()
                return
            d = os.path.normpath(d)
            if cerere.is_project(d):
                break
            sub = os.path.join(d, 'PoduRos_SUMO')
            if cerere.is_project(sub):
                d = sub
                break
            messagebox.showwarning('Nu e folderul bun', 'In folderul ales nu exista retea\\podu_ros.net.xml si trafic\\.')
        cerere.set_root(d)
        root.deiconify()
    st = ttk.Style()
    try:
        st.theme_use('vista' if os.name == 'nt' else 'clam')
    except tk.TclError:
        pass
    st.configure('S.TButton', padding=(3, 1), width=0)
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
