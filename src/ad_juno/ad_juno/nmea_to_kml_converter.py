#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#TO RUN THIS CODE python3 nmea_to_kml_converter.py location of the nmea file (for example: /home/bylogix/Downloads/nmea_msgs.nmea)

"""
nmea_to_kml.py
--------------
Reads an NMEA log file (e.g. the output of QGIS "Log NMEA sentences") and
produces a KML file that you can open in Google Earth.

- Parses $..GGA sentences (the message sent by the ArduSimple rover).
- Writes the travelled path as a line (track / LineString).
- Optionally adds each position as a point (Placemark), colored by RTK
  fix status.

Uses only the Python standard library, no extra installation required.

Usage:
    python nmea_to_kml.py input.nmea
    python nmea_to_kml.py input.nmea -o output.kml
    python nmea_to_kml.py input.nmea --points          # also add points
    python nmea_to_kml.py input.nmea --only-rtk         # RTK fixes only
"""

import argparse
import sys
import os

# GGA fix-quality codes -> human-readable label and point color (KML ABGR).
FIX_QUALITY = {
    0: ("No fix",        "ff0000ff"),  # red
    1: ("GPS (single)",  "ff00a5ff"),  # orange
    2: ("DGPS",          "ff00ffff"),  # yellow
    3: ("PPS",           "ff00ffff"),
    4: ("RTK Fixed",     "ff00ff00"),  # green
    5: ("RTK Float",     "ff00ffaa"),  # light green
    6: ("Estimated (DR)","ffaaaaaa"),  # gray
}


def validate_checksum(sentence):
    """Validate the *XX checksum. Returns True if no checksum is present."""
    if "*" not in sentence:
        return True
    body, _, cksum = sentence.partition("*")
    cksum = cksum.strip()[:2]
    if len(cksum) < 2:
        return True
    calc = 0
    for ch in body[1:]:  # skip the leading '$'
        calc ^= ord(ch)
    try:
        return calc == int(cksum, 16)
    except ValueError:
        return True


def nmea_to_decimal(value, hemisphere):
    """Convert ddmm.mmmm / dddmm.mmmm to decimal degrees."""
    if not value:
        return None
    try:
        v = float(value)
    except ValueError:
        return None
    degrees = int(v / 100)
    minutes = v - degrees * 100
    dec = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        dec = -dec
    return dec


def parse_gga(fields):
    """Return (lon, lat, alt, fix_quality) from GGA fields, or None if invalid."""
    # GGA: 0:type 1:time 2:lat 3:N/S 4:lon 5:E/W 6:fixquality 7:sats 8:hdop 9:alt ...
    if len(fields) < 10:
        return None
    lat = nmea_to_decimal(fields[2], fields[3])
    lon = nmea_to_decimal(fields[4], fields[5])
    if lat is None or lon is None:
        return None
    try:
        quality = int(fields[6]) if fields[6] else 0
    except ValueError:
        quality = 0
    try:
        alt = float(fields[9]) if fields[9] else 0.0
    except ValueError:
        alt = 0.0
    return (lon, lat, alt, quality)


def read_points(path, only_rtk=False):
    """Read the NMEA file and return a list of valid positions."""
    points = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            idx = line.find("$")
            if idx < 0:
                continue
            sentence = line[idx:]
            if "GGA" not in sentence[:7]:
                continue
            if not validate_checksum(sentence):
                continue
            fields = sentence.split("*")[0].split(",")
            parsed = parse_gga(fields)
            if parsed is None:
                continue
            lon, lat, alt, quality = parsed
            if quality == 0:
                continue  # skip positions with no fix
            if only_rtk and quality not in (4, 5):
                continue
            points.append((lon, lat, alt, quality))
    return points


def build_kml(points, with_points=False):
    """Build the KML text from a list of positions."""
    out = []
    out.append('<?xml version="1.0" encoding="UTF-8"?>')
    out.append('<kml xmlns="http://www.opengis.net/kml/2.2">')
    out.append('<Document>')
    out.append('  <name>Rover GPS Track</name>')

    # Track line
    coords = " ".join("{:.8f},{:.8f},{:.2f}".format(lon, lat, alt)
                      for (lon, lat, alt, q) in points)
    out.append('  <Placemark>')
    out.append('    <name>Track</name>')
    out.append('    <Style><LineStyle>'
               '<color>ff0000ff</color><width>3</width>'
               '</LineStyle></Style>')
    out.append('    <LineString>')
    out.append('      <tessellate>1</tessellate>')
    out.append('      <altitudeMode>clampToGround</altitudeMode>')
    out.append('      <coordinates>{}</coordinates>'.format(coords))
    out.append('    </LineString>')
    out.append('  </Placemark>')

    # Optional points (colored by fix status)
    if with_points:
        # define one style per fix type
        used = sorted({q for (_, _, _, q) in points})
        for q in used:
            label, color = FIX_QUALITY.get(q, ("Unknown", "ffffffff"))
            out.append('  <Style id="q{}">'.format(q))
            out.append('    <IconStyle><color>{}</color><scale>0.5</scale>'
                       '<Icon><href>http://maps.google.com/mapfiles/kml/'
                       'shapes/placemark_circle.png</href></Icon></IconStyle>'
                       .format(color))
            out.append('  </Style>')
        out.append('  <Folder><name>Points</name>')
        for i, (lon, lat, alt, q) in enumerate(points, 1):
            label, _ = FIX_QUALITY.get(q, ("Unknown", ""))
            out.append('    <Placemark>')
            out.append('      <name>{}</name>'.format(i))
            out.append('      <description>{}</description>'.format(label))
            out.append('      <styleUrl>#q{}</styleUrl>'.format(q))
            out.append('      <Point><coordinates>{:.8f},{:.8f},{:.2f}'
                       '</coordinates></Point>'.format(lon, lat, alt))
            out.append('    </Placemark>')
        out.append('  </Folder>')

    out.append('</Document>')
    out.append('</kml>')
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(
        description="Convert an NMEA (GGA) log file to KML.")
    ap.add_argument("input", help="Input NMEA file")
    ap.add_argument("-o", "--output", help="Output KML file "
                    "(default: input name + .kml)")
    ap.add_argument("--points", action="store_true",
                    help="Add points in addition to the track line")
    ap.add_argument("--only-rtk", action="store_true",
                    help="Keep only RTK Fixed/Float positions")
    args = ap.parse_args()

    if not os.path.isfile(args.input):
        print("Error: file not found:", args.input)
        sys.exit(1)

    points = read_points(args.input, only_rtk=args.only_rtk)
    if not points:
        print("Warning: no valid positions found. "
              "Check that the file contains GGA sentences with a fix.")
        sys.exit(1)

    output = args.output
    if not output:
        base = os.path.splitext(args.input)[0]
        output = base + ".kml"

    kml = build_kml(points, with_points=args.points)
    with open(output, "w", encoding="utf-8") as f:
        f.write(kml)

    # Summary
    counts = {}
    for (_, _, _, q) in points:
        counts[q] = counts.get(q, 0) + 1
    print("Total positions:", len(points))
    for q in sorted(counts):
        label, _ = FIX_QUALITY.get(q, ("Unknown", ""))
        print("  {:<14} : {}".format(label, counts[q]))
    print("KML written:", output)


if __name__ == "__main__":
    main()