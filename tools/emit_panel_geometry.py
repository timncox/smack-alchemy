#!/usr/bin/env python3
"""
Derive the Alchemy Lab V2 panel geometry for docs/index.html from the SDK's
KiCad front-panel template, so the site's drawing is never hand-placed.

    python3 tools/emit_panel_geometry.py            # print the JS block
    python3 tools/emit_panel_geometry.py --check    # fail if docs/ is stale
    python3 tools/emit_panel_geometry.py --write    # splice into docs/

Source: lib/alchemy-sdk/panel/front-panel-template.kicad_pcb (Edge.Cuts for
the plate, holes, mounting slots and the USB-C slot; F.Mask arcs for the LED
ring radius; F.Cu text for label offsets and the IN / OUT legends).

Everything is in millimetres, front view, origin at the plate's top-left
corner, y down. The template says nothing about which of the six middle
jacks is J3..J8; they are numbered in reading order here (top row J3 J4 J5,
bottom row J6 J7 J8) and the site says so. J1/J2 sit under the "IN" legend
on the left, J9/J10 under "OUT" on the right, which the SDK's codec mapping
agrees with.
"""
import json, math, re, sys, os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB  = os.path.join(ROOT, "lib/alchemy-sdk/panel/front-panel-template.kicad_pcb")
HTML = os.path.join(ROOT, "docs/index.html")
BEGIN = "/* == BEGIN GENERATED GEOMETRY -- tools/emit_panel_geometry.py == */"
END   = "/* == END GENERATED GEOMETRY == */"


def top_level_items(s):
    items, d, start, i, n = [], 0, None, 0, len(s)
    while i < n:
        c = s[i]
        if c == '"':
            j = s.find('"', i + 1)
            while s[j - 1] == '\\':
                j = s.find('"', j + 1)
            i = j + 1
            continue
        if c == '(':
            d += 1
            if d == 2:
                start = i
        elif c == ')':
            if d == 2 and start is not None:
                items.append(s[start:i + 1])
                start = None
            d -= 1
        i += 1
    return items


def layer(it):
    m = re.search(r'\(layer\s+"([^"]+)"', it)
    return m.group(1) if m else None


def pts(it):
    return {k: (float(x), float(y))
            for k, x, y in re.findall(r'\((start|end|mid|center|at)\s+([-\d.]+)\s+([-\d.]+)', it)}


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def main():
    s = open(PCB).read()
    items = top_level_items(s)
    edge = [it for it in items if layer(it) == "Edge.Cuts"]
    mask = [it for it in items if layer(it) == "F.Mask"]
    cu   = [it for it in items if layer(it) == "F.Cu"]

    # Plate: the one rect on Edge.Cuts.
    rect = next(pts(it) for it in edge if it.startswith("(gr_rect"))
    ox, oy = rect["start"]
    W, H = rect["end"][0] - ox, rect["end"][1] - oy

    def P(p):
        return (round(p[0] - ox, 3), round(p[1] - oy, 3))

    circles = []
    for it in edge:
        if it.startswith("(gr_circle"):
            q = pts(it)
            circles.append((P(q["center"]), round(dist(q["center"], q["end"]), 3)))

    def by_radius(lo, hi):
        return sorted([c for c, r in circles if lo <= r <= hi], key=lambda c: (round(c[1]), c[0]))

    pots    = by_radius(3.55, 3.65)   # 7.2 mm
    buttons = by_radius(3.05, 3.15)   # 6.2 mm
    jacks   = by_radius(3.15, 3.25)   # 6.4 mm
    assert len(pots) == 6 and len(buttons) == 3 and len(jacks) == 10, (len(pots), len(buttons), len(jacks))

    # Arcs on Edge.Cuts: semicircles whose chord midpoint is the centre.
    arcs = []
    for it in edge:
        if it.startswith("(gr_arc"):
            q = pts(it)
            c = ((q["start"][0] + q["end"][0]) / 2, (q["start"][1] + q["end"][1]) / 2)
            arcs.append((P(c), round(dist(q["start"], q["end"]) / 2, 3)))

    def slots(radius_lo, radius_hi):
        ends = [a for a in arcs if radius_lo <= a[1] <= radius_hi]
        out, used = [], set()
        for i, (c, r) in enumerate(ends):
            if i in used:
                continue
            for j in range(i + 1, len(ends)):
                if j in used:
                    continue
                c2, r2 = ends[j]
                if abs(c[1] - c2[1]) < 0.05 and dist(c, c2) < 12:
                    used |= {i, j}
                    x0, x1 = sorted([c[0], c2[0]])
                    out.append({"x": round(x0 - r, 3), "y": round(c[1] - r, 3),
                                "w": round(x1 - x0 + 2 * r, 3), "h": round(2 * r, 3), "r": r})
                    break
        return sorted(out, key=lambda o: (o["y"], o["x"]))

    mount = slots(1.65, 1.75)
    usb   = slots(1.95, 2.10)
    assert len(mount) == 4 and len(usb) == 1, (len(mount), len(usb))

    # LED ring radius: F.Mask arcs whose chord midpoint sits near a pot.
    ring_r = []
    for it in mask:
        if it.startswith("(gr_arc"):
            q = pts(it)
            m = P(q["mid"])
            for pc in pots:
                cx = (q["start"][0] + q["end"][0]) / 2 - ox
                if abs(cx - pc[0]) < 1.0 and abs(m[1] - pc[1]) < 14:
                    ring_r.append(dist(m, pc))
    ring_r = round(sum(ring_r) / len(ring_r), 2)

    # Label offsets and legends from the template's placeholder text.
    texts = []
    for it in cu:
        if it.startswith("(gr_text"):
            t = re.search(r'\(gr_text\s+"([^"]*)"', it).group(1)
            texts.append((t, P(pts(it)["at"])))
    # A label belongs to the control above it in its own column, not to
    # whichever hole is nearest (the pot below a mid-row label is closer).
    def above(holes, at):
        cands = [h for h in holes if abs(h[0] - at[0]) < 3.0 and h[1] < at[1]]
        return max(cands, key=lambda h: h[1])
    knob_dy = round(sum(t[1][1] - above(pots, t[1])[1]
                        for t in texts if t[0] == "KNOB LABEL") / 6, 2)
    btn_dy  = round(sum(t[1][1] - above(buttons, t[1])[1]
                        for t in texts if t[0] == "BUTTON LABEL") / 3, 2)
    legends = {t[0]: t[1] for t in texts if t[0] in ("IN", "OUT")}
    rails = sorted({round(pts(it)["start"][1] - oy, 2) for it in cu
                    if it.startswith("(gr_line") and abs(pts(it)["start"][0] - pts(it)["end"][0]) > 50})

    # Jack numbering: two rows of five. Left column IN (J1 top, J2 bottom),
    # right column OUT (J9 top, J10 bottom), middle six in reading order.
    rows = sorted({round(j[1]) for j in jacks})
    assert len(rows) == 2
    top = sorted([j for j in jacks if round(j[1]) == rows[0]])
    bot = sorted([j for j in jacks if round(j[1]) == rows[1]])
    jack_ids = {"J1": top[0], "J2": bot[0], "J3": top[1], "J4": top[2], "J5": top[3],
                "J6": bot[1], "J7": bot[2], "J8": bot[3], "J9": top[4], "J10": bot[4]}

    geom = {
        "source": "lib/alchemy-sdk/panel/front-panel-template.kicad_pcb",
        "units": "mm, front view, origin top-left of the plate, y down",
        "plate": {"w": round(W, 2), "h": round(H, 2), "hp": round(W / 5.08, 1)},
        "mount": mount,
        "rails": rails,
        "pots": [{"id": "P%d" % (i + 1), "x": p[0], "y": p[1], "hole": 3.6} for i, p in enumerate(pots)],
        "ring": {"r": ring_r, "leds": 16, "start_hour": 7.5, "step_hours": 0.75, "arc_leds": 13},
        "buttons": [{"id": "B%d" % (i + 1), "x": b[0], "y": b[1], "hole": 3.1, "halo": 4.0}
                    for i, b in enumerate(buttons)],
        "usb": usb[0],
        "jacks": [{"id": k, "x": v[0], "y": v[1], "hole": 3.2} for k, v in jack_ids.items()],
        "jack_order_assumed": "J3..J8 in reading order; the printed panel is the authority",
        "labels": {"knob_dy": knob_dy, "button_dy": btn_dy,
                   "in": legends["IN"], "out": legends["OUT"]},
    }
    block = BEGIN + "\nconst PANEL = " + json.dumps(geom, indent=1) + ";\n" + END
    return block


def splice(block, write):
    html = open(HTML).read()
    a, b = html.find(BEGIN), html.find(END)
    if a < 0 or b < 0:
        sys.exit("docs/index.html has no geometry block")
    current = html[a:b + len(END)]
    if current == block:
        print("docs/index.html geometry is current")
        return 0
    if not write:
        print("docs/index.html geometry is STALE; run with --write", file=sys.stderr)
        return 1
    open(HTML, "w").write(html[:a] + block + html[b + len(END):])
    print("docs/index.html geometry updated")
    return 0


if __name__ == "__main__":
    blk = main()
    if "--check" in sys.argv:
        sys.exit(splice(blk, False))
    elif "--write" in sys.argv:
        sys.exit(splice(blk, True))
    else:
        print(blk)
