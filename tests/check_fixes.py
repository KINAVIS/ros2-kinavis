#!/usr/bin/env python3
"""Fixes echoed from /fix (`ros2 topic echo --csv`) against a reference CSV.

Matched by the receiver time of their stamp; latitude, longitude, altitude
and status must agree, and at least 90 % of the reference epochs must have
arrived (the first may go before discovery).
"""

import math
import sys


def main(echoed, reference):
    expected = {}
    with open(reference) as rows:
        next(rows)
        for row in rows:
            seconds, status, latitude, longitude, altitude = row.strip().split(",")
            expected[round(float(seconds), 3)] = (int(status), float(latitude),
                                                  float(longitude), float(altitude))
    received = 0
    with open(echoed) as rows:
        for row in rows:
            fields = row.strip().split(",")
            if len(fields) < 8:
                continue
            sec, nanosec, _frame, status, _service, latitude, longitude, altitude = fields[:8]
            key = round(int(sec) % 86400 + int(nanosec) * 1e-9, 3)
            if key not in expected:
                sys.exit(f"fix at {key}: no reference epoch")
            want = expected[key]
            got = (int(status), float(latitude), float(longitude), float(altitude))
            same = got[0] == want[0] and all(
                (math.isnan(a) and math.isnan(b)) or abs(a - b) <= 1e-9
                for a, b in zip(got[1:], want[1:]))
            if not same:
                sys.exit(f"fix at {key}: {got}, reference {want}")
            received += 1
    print(f"{received} of {len(expected)} fixes, all as the reference")
    if received < 0.9 * len(expected):
        sys.exit("too few fixes")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
