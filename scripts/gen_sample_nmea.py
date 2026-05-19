#!/usr/bin/env python3
"""Generate test/data/sample_nmea_100.txt with valid NMEA checksums."""

from pathlib import Path


def nmea_checksum(body: str) -> str:
    cs = 0
    for c in body:
        cs ^= ord(c)
    return f"{cs:02X}"


def main() -> None:
    out = Path(__file__).resolve().parents[1] / "test" / "data" / "sample_nmea_100.txt"
    out.parent.mkdir(parents=True, exist_ok=True)

    first_lat, first_lon, first_alt = 3000.0001, 12000.0000, 100.0
    lines = []
    for i in range(100):
        t = f"0900{i * 0.10:05.2f}"
        if i == 99:
            lat, lon, alt = first_lat, first_lon, first_alt
        else:
            lat = first_lat + (i + 1) * 0.01
            lon = first_lon
            alt = first_alt + i * 0.01
        body = (
            f"GNGGA,{t},{lat:010.4f},N,{lon:011.4f},E,4,12,0.8,{alt:.1f},M,0.0,M,,"
        )
        lines.append(f"${body}*{nmea_checksum(body)}")

    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {len(lines)} lines to {out}")


if __name__ == "__main__":
    main()
