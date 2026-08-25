#!/usr/bin/env python3
"""Rasterise the world maps — land, borders, rivers, cities — from Natural Earth.

Every raster is equirectangular, row-major, x = lon -180..180 and
y = lat +90..-90, so the 3D globe and the flat FT8/APRS maps place a coastline
at the same coordinates and a QTH marker lands on the same shoreline in both.

    land.png     8 192 x 4 096, 8-bit  — *coverage*: the fraction of each texel
                                          that is land, 0 = open ocean or lake,
                                          255 = solid ground
    borders.png  4 320 x 2 160, 8-bit  — antialiased coverage of the
                                          international boundary lines
    rivers.png   4 320 x 2 160, 8-bit  — the same, for river and lake
                                          centrelines, each drawn at the width
                                          Natural Earth itself ranks it at
    cities.png   4 320 x 2 160, 8-bit  — coverage of the built-up urban areas:
                                          the shape a city actually has, which
                                          is what the globe lights at night
    cities.bin   ~180 kB               — the populated-places *table* (name,
                                          position, population), for the flat
                                          maps, which have room to label them
    lines.bin    ~1.6 MB                — the borders and rivers again, as the
                                          *polylines* they were digitised as,
                                          simplified into five levels of
                                          detail. What the flat maps draw:
                                          a line from its own geometry is one
                                          dot wide at any zoom, which no
                                          threshold on a raster can manage

Coverage, not a 1-bit mask, is the whole point of the land map. The shader
draws the shoreline as the field's half-way contour and strokes it a fixed
number of *pixels* wide, so where that contour falls inside a texel is what the
eye reads as sharpness. Thresholding would round it to the texel grid and put a
staircase on every coast; keeping the supersampled coverage places it to a
fraction of a texel and the same contour comes out smooth at any zoom. The line
layers keep their coverage for the same reason from the other end: a river a
third of a texel wide is a third-strength texel rather than a dropped one, so
it thins out as the map zooms away from it instead of flickering.

The land grid is 1/22.75 deg, ~4.9 km at the equator — four times the flat
maps' CPU grid in each axis, because the globe's camera can fly down to the
surface and a panel-sized rectangle's grid runs out long before it does. The
line and city layers stay at 1/12 deg: they are drawn as thin lines and small
blobs rather than as one continuous filled region, so resolution buys them far
less than it buys the coast, and they cost the same memory per texel.

Cities arrive twice because the two views need different things from them. The
globe wants a *shape* to light — a texture it can add to the night side, where
Tokyo is a sprawl and Reykjavik is a dot — and that is `cities.png`, rasterised
from the urban-area polygons. The flat maps want a *name* to draw next to a
point, which no texture can carry, so they get `cities.bin`: every populated
place with its position and population, biggest first, so a map can simply take
as many as fit and stop.

Sources (all public domain, Natural Earth 1:10m):

    ne_10m_land, ne_10m_lakes, ne_10m_admin_0_boundary_lines_land,
    ne_10m_rivers_lake_centerlines_scale_rank, ne_10m_urban_areas,
    ne_10m_populated_places

Run from anywhere; it downloads into a temporary directory and writes the
outputs next to itself. Peak memory is a few hundred MB — the land pass
supersamples to 32768x16384 before averaging down.

    python3 make_earth_maps.py
"""

import io
import os
import struct
import sys
import unicodedata
import urllib.request
import zipfile

from PIL import Image, ImageDraw

LAND_W, LAND_H = 8192, 4096
# Borders, rivers and urban areas share one grid: they are all thin marks over
# the land rather than the land itself, and sharing lets the three be sampled
# with one set of level arithmetic.
LINE_W, LINE_H = 4320, 2160
# Supersampling factors. Every output keeps its coverage as an 8-bit alpha, so
# these decide how finely a shoreline can be placed inside a texel and how
# readable a sub-pixel line is. Land gets the finest grid because its coverage
# is what the shader's contour is reconstructed from: SS^2 + 1 distinct levels,
# and the contour wobbles by about half a level's worth of a texel.
LAND_SS = 4
LINE_SS = 3
# Urban areas are filled blobs, not lines: their edges are soft in the source
# data to begin with, and 3x would cost 80 MB of intermediate for an edge
# nobody can see.
CITY_SS = 2

CACHE = os.environ.get("NE_CACHE", "/tmp/naturalearth")
BASE = "https://naciscdn.org/naturalearth/10m"
LAYERS = {
    "land": f"{BASE}/physical/ne_10m_land.zip",
    "lakes": f"{BASE}/physical/ne_10m_lakes.zip",
    "borders": f"{BASE}/cultural/ne_10m_admin_0_boundary_lines_land.zip",
    # The `_scale_rank` cut of the rivers carries `strokeweig`, which is
    # Natural Earth's own answer to "how big a river is this" — worth the
    # extra 300 kB of download over guessing from the geometry.
    "rivers": f"{BASE}/physical/ne_10m_rivers_lake_centerlines_scale_rank.zip",
    "urban": f"{BASE}/cultural/ne_10m_urban_areas.zip",
    "places": f"{BASE}/cultural/ne_10m_populated_places.zip",
}


def fetch(name: str, url: str, cache: str, ext: str = "shp") -> bytes:
    """One member of a Natural Earth zip, downloaded once and cached.

    `ext` picks the member: "shp" for the geometry, "dbf" for the attribute
    table beside it. Both are cached, so asking for the second one does not
    fetch the zip again.
    """
    path = os.path.join(cache, f"{name}.{ext}")
    if os.path.exists(path):
        return open(path, "rb").read()
    print(f"fetching {url}", file=sys.stderr)
    with urllib.request.urlopen(url, timeout=180) as r:
        blob = r.read()
    with zipfile.ZipFile(io.BytesIO(blob)) as z:
        for want in ("shp", "dbf"):
            member = next((n for n in z.namelist() if n.endswith("." + want)), None)
            if member is not None:
                open(os.path.join(cache, f"{name}.{want}"), "wb").write(z.read(member))
    return open(path, "rb").read()


def shapes(shp: bytes):
    """Yield each record's rings as lists of (lon, lat).

    Handles the two shape types Natural Earth uses here: 3 (polyline) and
    5 (polygon). Both store the same parts/points layout, so one reader does.
    Records come out in file order, which is the order the .dbf rows are in.
    """
    pos = 100  # file header
    while pos + 8 <= len(shp):
        _, words = struct.unpack_from(">ii", shp, pos)
        pos += 8
        end = pos + words * 2
        (kind,) = struct.unpack_from("<i", shp, pos)
        if kind in (3, 5):
            n_parts, n_points = struct.unpack_from("<ii", shp, pos + 36)
            parts = struct.unpack_from(f"<{n_parts}i", shp, pos + 44)
            off = pos + 44 + n_parts * 4
            pts = struct.unpack_from(f"<{2 * n_points}d", shp, off)
            bounds = list(parts) + [n_points]
            rings = []
            for k in range(n_parts):
                a, b = bounds[k], bounds[k + 1]
                rings.append([(pts[2 * i], pts[2 * i + 1]) for i in range(a, b)])
            yield rings
        pos = end


def dbf(blob: bytes, want: set):
    """Yield one dict per .dbf record, holding only the `want`ed fields.

    A minimal reader for what dBASE III actually is: a header of fixed-width
    field descriptors, then fixed-width records. Every value comes back as the
    stripped string it is stored as — the callers know which are numbers.
    """
    _, _, _, _, n_recs, hdr_len, rec_len = struct.unpack_from("<BBBBIHH", blob, 0)
    fields, off, pos = [], 0, 32
    while blob[pos] != 0x0D:
        name = blob[pos : pos + 11].split(b"\0")[0].decode("latin1")
        length = blob[pos + 16]
        if name in want:
            fields.append((name, off, length))
        off += length
        pos += 32
    for i in range(n_recs):
        # The leading byte of each record is the deletion flag.
        base = hdr_len + i * rec_len + 1
        yield {
            name: blob[base + o : base + o + ln].decode("utf-8", "replace").strip()
            for name, o, ln in fields
        }


def project(ring, size):
    """Lon/lat degrees to supersampled pixel coordinates for one output grid."""
    sw, sh = size
    return [
        (((lon + 180.0) / 360.0) * sw, ((90.0 - lat) / 180.0) * sh) for lon, lat in ring
    ]


def split_dateline(ring):
    """Break a line wherever it steps across the antimeridian.

    Natural Earth splits its own geometry at +-180 nearly everywhere, but a
    single pair of points that does step across would otherwise be drawn as a
    line straight back through the whole map — one horizontal scar over three
    continents. Cutting the run there costs nothing when there is nothing to
    cut, which is the usual case.
    """
    out, run = [], [ring[0]]
    for prev, cur in zip(ring, ring[1:]):
        if abs(cur[0] - prev[0]) > 180.0:
            out.append(run)
            run = []
        run.append(cur)
    out.append(run)
    return [r for r in out if len(r) >= 2]


def signed_area(ring) -> float:
    """Shoelace area in lon/lat. Negative is clockwise, which in the shapefile
    convention is an outer ring; positive rings are holes."""
    a = 0.0
    for (x0, y0), (x1, y1) in zip(ring, ring[1:] + ring[:1]):
        a += x0 * y1 - x1 * y0
    return a * 0.5


def rasterise_polygons(shp: bytes, img: Image.Image, fill: int):
    """Fill every outer ring, then punch out every hole."""
    draw = ImageDraw.Draw(img)
    holes = []
    for rings in shapes(shp):
        for ring in rings:
            if len(ring) < 3:
                continue
            if signed_area(ring) > 0:
                holes.append(ring)
            else:
                draw.polygon(project(ring, img.size), fill=fill)
    for ring in holes:
        draw.polygon(project(ring, img.size), fill=0)


def rasterise_lines(shp: bytes, img: Image.Image, width):
    """Stroke every line. `width` is either a constant or a per-record list,
    in the file order `shapes` yields."""
    draw = ImageDraw.Draw(img)
    for i, rings in enumerate(shapes(shp)):
        w = width if isinstance(width, int) else width[i]
        for ring in rings:
            if len(ring) < 2:
                continue
            for run in split_dateline(ring):
                draw.line(project(run, img.size), fill=255, width=w, joint="curve")


def build_land() -> Image.Image:
    land = Image.new("L", (LAND_W * LAND_SS, LAND_H * LAND_SS), 0)
    rasterise_polygons(fetch("land", LAYERS["land"], CACHE), land, 255)
    # Inland water is not land: without this the Caspian and the Great Lakes
    # read as continent, which on a globe is immediately wrong.
    rasterise_polygons(fetch("lakes", LAYERS["lakes"], CACHE), land, 0)
    # BOX is an exact box filter over the SS x SS subsamples, so every texel
    # comes out holding the fraction of itself that is land — which is what the
    # shader's contour needs.
    return land.resize((LAND_W, LAND_H), Image.BOX)


def build_borders() -> Image.Image:
    borders = Image.new("L", (LINE_W * LINE_SS, LINE_H * LINE_SS), 0)
    # One output pixel wide once downsampled — any thicker and the borders
    # dominate the coastline they sit next to.
    rasterise_lines(fetch("borders", LAYERS["borders"], CACHE), borders, LINE_SS)
    return borders.resize((LINE_W, LINE_H), Image.BOX)


def river_width(stroke: float) -> int:
    """Supersampled stroke for one river, from Natural Earth's own weight.

    The source ranks its rivers from 0.15 to 2.0, and that ranking is the whole
    reason this layer is worth having: drawn at one width the Amazon and a
    Norwegian creek are the same mark, and the map says nothing about which
    rivers matter. Mapped here to roughly 2/3 of an output pixel at the bottom
    and two pixels at the top — the small ones deliberately land *under* a
    pixel, so the box filter hands them back as a partial-coverage hairline
    that fades out on its own as the map zooms away.
    """
    return max(1, round(LINE_SS * (0.45 + 0.8 * stroke)))


def build_rivers() -> Image.Image:
    shp = fetch("rivers", LAYERS["rivers"], CACHE)
    rows = list(dbf(fetch("rivers", LAYERS["rivers"], CACHE, "dbf"), {"strokeweig"}))
    widths = [river_width(float(r["strokeweig"] or 0.2)) for r in rows]
    rivers = Image.new("L", (LINE_W * LINE_SS, LINE_H * LINE_SS), 0)
    rasterise_lines(shp, rivers, widths)
    return rivers.resize((LINE_W, LINE_H), Image.BOX)


def build_cities() -> Image.Image:
    cities = Image.new("L", (LINE_W * CITY_SS, LINE_H * CITY_SS), 0)
    rasterise_polygons(fetch("urban", LAYERS["urban"], CACHE), cities, 255)
    return cities.resize((LINE_W, LINE_H), Image.BOX)


CITY_MAGIC = b"SDXCITY1"


def ascii_name(name: str) -> bytes:
    """A place name with its accents decomposed away, or the original if that
    leaves nothing (a name written in a script with no Latin form at all)."""
    stripped = unicodedata.normalize("NFKD", name).encode("ascii", "ignore")
    return (stripped or name.encode("utf-8"))[:255]


def build_places() -> bytes:
    """The populated-places table, biggest first.

    Sorted by population because that is how a map has to read it: there is no
    room for seven thousand labels, so a view takes them in order and stops
    when it has enough — which makes "which cities show" a function of how far
    in you are zoomed, with no threshold to pick.

    ASCII names (`NAMEASCII`, the field Natural Earth keeps for exactly this)
    rather than the local spelling: the UI ships one Latin font, and a name
    that renders as a row of empty boxes is worse than a transliterated one.
    Natural Earth lets the odd accent through that field anyway — Utqiagvik is
    spelled with a g-dot — so the accents are decomposed off here and the
    promise the name makes is one the file actually keeps.
    """
    rows = list(
        dbf(
            fetch("places", LAYERS["places"], CACHE, "dbf"),
            {"NAMEASCII", "NAME", "LATITUDE", "LONGITUDE", "POP_MAX", "ADM0CAP"},
        )
    )
    out = []
    for r in rows:
        name = ascii_name(r["NAMEASCII"] or r["NAME"])
        if not name:
            continue
        # Natural Earth writes -99 where it has no figure, which is a place
        # that exists and is simply unsized — it belongs at the bottom of the
        # order, not off the map.
        pop = max(0, int(float(r["POP_MAX"] or 0)))
        flags = 1 if r["ADM0CAP"] == "1" else 0
        out.append(
            (
                pop,
                struct.pack(
                    "<iiIBB",
                    round(float(r["LATITUDE"]) * 1e5),
                    round(float(r["LONGITUDE"]) * 1e5),
                    pop,
                    flags,
                    len(name),
                )
                + name,
            )
        )
    out.sort(key=lambda p: -p[0])
    return CITY_MAGIC + struct.pack("<I", len(out)) + b"".join(rec for _, rec in out)


# ── The line layers, as lines ───────────────────────────────────────────────

LINE_MAGIC = b"SDXLINE1"

# Simplification tolerance per level, in degrees, coarsest last. Each is four
# times the one before, and a map takes the level whose tolerance is under half
# a dot cell — so the simplification is always finer than the grid it is drawn
# on and never something the eye can find. The five span the whole zoom range,
# from a one-degree view of a valley to the whole world in a panel.
LINE_LEVELS = [0.003, 0.012, 0.05, 0.2, 0.8]

# One delta step, in degrees. 11 m: far finer than the finest tolerance above,
# and it leaves an i16 a range of ±3.2°, which all but a handful of simplified
# segments fit inside.
LINE_STEP = 1e-4


def douglas_peucker(pts, eps: float):
    """Drop every vertex that sits within `eps` of the line it lies on.

    Iterative rather than recursive: a river with forty thousand vertices in
    one part will blow a recursion limit, and this reads no worse.
    """
    if len(pts) < 3:
        return pts
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        a, b = stack.pop()
        (x0, y0), (x1, y1) = pts[a], pts[b]
        dx, dy = x1 - x0, y1 - y0
        norm = (dx * dx + dy * dy) ** 0.5 or 1e-12
        far, at = eps, None
        for i in range(a + 1, b):
            x, y = pts[i]
            d = abs(dy * (x - x0) - dx * (y - y0)) / norm
            if d > far:
                far, at = d, i
        if at is not None:
            keep[at] = True
            stack.append((a, at))
            stack.append((at, b))
    return [p for p, k in zip(pts, keep) if k]


def encode_part(pts, rank: int) -> bytes:
    """One polyline: an absolute first point and 16-bit steps after it.

    A step of 11 m leaves an i16 a range of 3.2°, which every segment of the
    finer levels is well inside; the coarse ones do produce the odd longer one,
    and those are subdivided rather than split off into a part of their own —
    the extra vertices are exactly on the line they came from, so nothing about
    the geometry changes and the reader stays a straight loop.
    """
    if len(pts) < 2:
        return b""
    reach = 30000 * LINE_STEP
    dense = [pts[0]]
    for (lat0, lon0), (lat1, lon1) in zip(pts, pts[1:]):
        steps = int(max(abs(lat1 - lat0), abs(lon1 - lon0)) / reach) + 1
        for k in range(1, steps + 1):
            f = k / steps
            dense.append((lat0 + (lat1 - lat0) * f, lon0 + (lon1 - lon0) * f))
    lat0, lon0 = dense[0]
    body = struct.pack("<BHii", rank, len(dense), round(lat0 * 1e5), round(lon0 * 1e5))
    prev = (round(lat0 / LINE_STEP), round(lon0 / LINE_STEP))
    for lat, lon in dense[1:]:
        cur = (round(lat / LINE_STEP), round(lon / LINE_STEP))
        body += struct.pack("<hh", cur[0] - prev[0], cur[1] - prev[1])
        prev = cur
    return body


def build_lines() -> bytes:
    """Borders and rivers as polylines, simplified into levels.

    The rasters these come from stay — the globe is textured with them — but a
    flat map cannot use one. A border is a line one texel wide, and once a map
    is zoomed in far enough for a texel to cover several of its dots, *no*
    threshold on the interpolated coverage recovers a line: set it low and the
    border comes out a band four dots wide, set it high and the same border
    breaks into fragments wherever it happens to fall between two texels. The
    geometry has no such problem — a line drawn from the vertices it was
    digitised from is one dot wide at every zoom, and in the right place.

    Cheaper, too: the two rasters cost 12 MB of CPU pyramids for detail they
    could not deliver, and the vectors are under a megabyte and a half for
    detail bounded only by Natural Earth's own.
    """
    layers = []
    for name, ranked in (("borders", False), ("rivers", True)):
        shp = fetch(name, LAYERS[name], CACHE)
        if ranked:
            rows = list(dbf(fetch(name, LAYERS[name], CACHE, "dbf"), {"strokeweig"}))
            # 0.15…2.0 of stroke weight over the sixteen ranks a nibble holds.
            ranks = [
                min(15, max(0, round((float(r["strokeweig"] or 0.2) - 0.15) * 8.0)))
                for r in rows
            ]
        else:
            ranks = None
        parts = []
        for i, rings in enumerate(shapes(shp)):
            rank = ranks[i] if ranks else 0
            for ring in rings:
                if len(ring) >= 2:
                    for run in split_dateline([(lat, lon) for lon, lat in ring]):
                        parts.append((rank, run))
        levels = []
        for eps in LINE_LEVELS:
            encoded, count = [], 0
            for rank, pts in parts:
                # The coarser levels are for a map showing a continent at a
                # time, where a creek is a smudge and a border is the point.
                if eps >= 0.05 and rank and rank < (2 if eps < 0.2 else 5):
                    continue
                # A part smaller than the tolerance is a dot at this level, and
                # nine thousand of them are what a coarse level would otherwise
                # spend most of its bytes on: the borders alone leave three
                # thousand islands and enclaves behind at continental zoom.
                span = max(
                    max(p[0] for p in pts) - min(p[0] for p in pts),
                    max(p[1] for p in pts) - min(p[1] for p in pts),
                )
                if span < 2.0 * eps:
                    continue
                blob = encode_part(douglas_peucker(pts, eps), rank)
                if blob:
                    encoded.append(blob)
                    count += 1
            # The tolerance travels with the level: what picks a level at
            # draw time is "which of these is finer than half a dot cell", and
            # a reader that had to be told the list separately could be told a
            # stale one.
            levels.append(
                struct.pack("<fI", eps, count) + b"".join(encoded)
            )
            print(
                f"  {name} @{eps}°: {count} parts, {len(levels[-1]) / 1024:.0f} kB",
                file=sys.stderr,
            )
        layers.append(struct.pack("<B", len(levels)) + b"".join(levels))
    return LINE_MAGIC + struct.pack("<B", len(layers)) + b"".join(layers)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    os.makedirs(CACHE, exist_ok=True)
    only = set(sys.argv[1:])
    outputs = (
        ("land.png", build_land),
        ("borders.png", build_borders),
        ("rivers.png", build_rivers),
        ("cities.png", build_cities),
        ("cities.bin", build_places),
        ("lines.bin", build_lines),
    )
    for name, build in outputs:
        if only and name not in only:
            continue
        path = os.path.join(here, name)
        built = build()
        if isinstance(built, bytes):
            open(path, "wb").write(built)
        else:
            built.save(path, optimize=True)
        print(f"{name}: {os.path.getsize(path) / 1024:.0f} kB", file=sys.stderr)


if __name__ == "__main__":
    main()
