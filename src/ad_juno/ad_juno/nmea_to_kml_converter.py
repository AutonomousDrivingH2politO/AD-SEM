#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nmea_speed.py
-------------
Reads an NMEA log file (GGA sentences) and computes speed between
consecutive fixes from position + timestamp. Produces three outputs:

  1) A CSV table (time, lat, lon, altitude, fix quality, speed).
  2) A KML track colored by speed, openable in Google Earth.
  3) A printed summary (distance, duration, average / max speed).

Speed is derived from the haversine distance between consecutive GGA
positions divided by the time difference of their timestamps. This is
needed because GGA does not carry a speed field (unlike RMC/VTG).

Uses only the Python standard library, no extra installation required.

Usage:
    python3 nmea_speed.py input.nmea
    python3 nmea_speed.py input.nmea --smooth 5     # moving-average smoothing
    python3 nmea_speed.py input.nmea -o myname      # output base name
"""

import argparse
import math
import os
import sys

FIX_LABEL = {
    0: "No fix", 1: "GPS", 2: "DGPS", 3: "PPS",
    4: "RTK Fixed", 5: "RTK Float", 6: "Estimated",
}

# Speed color scale, slow -> fast (KML ABGR).
SPEED_COLORS = [
    "ff00ff00",  # green  (slowest band)
    "ff00ffff",  # yellow
    "ff00a5ff",  # orange
    "ff0050ff",  # orange-red
    "ff0000ff",  # red    (fastest band)
]


def validate_checksum(sentence):
    """Validate the *XX checksum. True if absent/unparseable (lenient)."""
    if "*" not in sentence:
        return True
    body, _, cksum = sentence.partition("*")
    cksum = cksum.strip()[:2]
    if len(cksum) < 2:
        return True
    calc = 0
    for ch in body[1:]:
        calc ^= ord(ch)
    try:
        return calc == int(cksum, 16)
    except ValueError:
        return True


def to_decimal(value, hemisphere):
    """Convert ddmm.mmmm / dddmm.mmmm to decimal degrees."""
    if not value:
        return None
    try:
        v = float(value)
    except ValueError:
        return None
    deg = int(v / 100)
    dec = deg + (v - deg * 100) / 60.0
    if hemisphere in ("S", "W"):
        dec = -dec
    return dec


def parse_time(s):
    """hhmmss.ss -> seconds since midnight (UTC), or None."""
    if not s or len(s) < 6:
        return None
    try:
        return int(s[0:2]) * 3600 + int(s[2:4]) * 60 + float(s[4:])
    except ValueError:
        return None


def haversine(lat1, lon1, lat2, lon2):
    """Great-circle distance in meters between two lat/lon points."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def iter_sentences(line):
    """A log line may contain junk and several glued NMEA sentences
    (e.g. '$GNTXT,01,01,$GNGGA,...'). Split on '$' and yield each
    reconstructed candidate sentence."""
    for frag in line.split("$"):
        frag = frag.strip()
        if frag:
            yield "$" + frag


def read_points(path):
    """Read GGA fixes -> list of dicts {t, lat, lon, alt, q}.
    Robust to noisy logs where GGA sentences are glued to other
    messages on the same line, or surrounded by unknown messages."""
    pts = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            for s in iter_sentences(line.strip()):
                if "GGA" not in s[:7] or not validate_checksum(s):
                    continue
                fields = s.split("*")[0].split(",")
                if len(fields) < 10:
                    continue
                lat = to_decimal(fields[2], fields[3])
                lon = to_decimal(fields[4], fields[5])
                t = parse_time(fields[1])
                if lat is None or lon is None or t is None:
                    continue
                try:
                    q = int(fields[6]) if fields[6] else 0
                except ValueError:
                    q = 0
                if q == 0:
                    continue
                try:
                    alt = float(fields[9]) if fields[9] else 0.0
                except ValueError:
                    alt = 0.0
                pts.append({"t": t, "lat": lat, "lon": lon,
                            "alt": alt, "q": q})
    return pts


def compute_speeds(pts):
    """Fill each point with dt (s), dist (m), speed (m/s). First point = 0."""
    for idx, p in enumerate(pts):
        if idx == 0:
            p["dt"] = 0.0
            p["dist"] = 0.0
            p["v"] = 0.0
            continue
        prev = pts[idx - 1]
        dt = p["t"] - prev["t"]
        if dt < 0:           # midnight wrap
            dt += 86400.0
        dist = haversine(prev["lat"], prev["lon"], p["lat"], p["lon"])
        p["dt"] = dt
        p["dist"] = dist
        p["v"] = (dist / dt) if dt > 0 else 0.0


def smooth_speeds(pts, window):
    """Centered moving average over the speed values, window >= 1."""
    if window <= 1:
        return
    n = len(pts)
    raw = [p["v"] for p in pts]
    half = window // 2
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        chunk = raw[lo:hi]
        pts[i]["v"] = sum(chunk) / len(chunk)


def write_csv(pts, path):
    """Write the per-point table."""
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write("index,utc_seconds,latitude,longitude,altitude_m,"
                "fix,dt_s,dist_m,speed_ms,speed_kmh\n")
        for i, p in enumerate(pts):
            f.write("{},{:.2f},{:.8f},{:.8f},{:.2f},{},{:.2f},{:.3f},"
                    "{:.3f},{:.2f}\n".format(
                        i, p["t"], p["lat"], p["lon"], p["alt"],
                        FIX_LABEL.get(p["q"], "?"),
                        p["dt"], p["dist"], p["v"], p["v"] * 3.6))


def speed_band(v_kmh, vmax):
    """Map a speed to a color-band index 0..len(SPEED_COLORS)-1."""
    if vmax <= 0:
        return 0
    frac = v_kmh / vmax
    idx = int(frac * len(SPEED_COLORS))
    return max(0, min(len(SPEED_COLORS) - 1, idx))


def write_kml(pts, path, vmax_kmh):
    """Write a track split into segments colored by speed."""
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<kml xmlns="http://www.opengis.net/kml/2.2">',
           '<Document>', '  <name>Speed-colored track</name>']

    # One style per color band.
    for b, color in enumerate(SPEED_COLORS):
        out.append('  <Style id="s{}"><LineStyle><color>{}</color>'
                   '<width>4</width></LineStyle></Style>'.format(b, color))

    # Group consecutive segments that share the same band into one LineString
    # to keep the KML small even for long logs.
    out.append('  <Folder><name>Track by speed</name>')
    i = 1
    n = len(pts)
    while i < n:
        band = speed_band(pts[i]["v"] * 3.6, vmax_kmh)
        coords = ["{:.8f},{:.8f},{:.2f}".format(
            pts[i - 1]["lon"], pts[i - 1]["lat"], pts[i - 1]["alt"])]
        # extend while band stays the same
        while i < n and speed_band(pts[i]["v"] * 3.6, vmax_kmh) == band:
            coords.append("{:.8f},{:.8f},{:.2f}".format(
                pts[i]["lon"], pts[i]["lat"], pts[i]["alt"]))
            i += 1
        out.append('    <Placemark><styleUrl>#s{}</styleUrl>'
                   '<LineString><tessellate>1</tessellate>'
                   '<altitudeMode>clampToGround</altitudeMode>'
                   '<coordinates>{}</coordinates></LineString></Placemark>'
                   .format(band, " ".join(coords)))
    out.append('  </Folder>')
    out.append('</Document></kml>')
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(out))


def main():
    ap = argparse.ArgumentParser(
        description="Compute speed from an NMEA GGA log; "
                    "output CSV, speed-colored KML, and a summary.")
    ap.add_argument("input", help="Input NMEA file")
    ap.add_argument("-o", "--output", help="Output base name "
                    "(default: input name). Produces .csv and .kml")
    ap.add_argument("--smooth", type=int, default=1,
                    help="Moving-average window for speed (default 1 = none)")
    args = ap.parse_args()

    if not os.path.isfile(args.input):
        print("Error: file not found:", args.input)
        sys.exit(1)

    pts = read_points(args.input)
    if len(pts) < 2:
        print("Warning: need at least 2 valid GGA fixes to compute speed.")
        sys.exit(1)

    compute_speeds(pts)
    smooth_speeds(pts, args.smooth)

    base = args.output or os.path.splitext(args.input)[0]
    csv_path = base + ".csv"
    kml_path = base + "_speed.kml"

    speeds_kmh = [p["v"] * 3.6 for p in pts]
    vmax = max(speeds_kmh)
    total_dist = sum(p["dist"] for p in pts)
    total_time = sum(p["dt"] for p in pts)
    avg_kmh = (total_dist / total_time * 3.6) if total_time > 0 else 0.0

    write_csv(pts, csv_path)
    write_kml(pts, kml_path, vmax if vmax > 0 else 1.0)

    print("Fixes used      :", len(pts))
    print("Total distance  : {:.1f} m ({:.3f} km)".format(
        total_dist, total_dist / 1000.0))
    print("Duration        : {:.1f} s".format(total_time))
    print("Average speed   : {:.2f} km/h".format(avg_kmh))
    print("Max speed       : {:.2f} km/h".format(vmax))
    print("CSV written     :", csv_path)
    print("KML written     :", kml_path)


if __name__ == "__main__":
    main()