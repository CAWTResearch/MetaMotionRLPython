#!/usr/bin/env python3
import re
import sys
import statistics

def parse_log_lines(lines):
    latencies = []
    deltas = []

    # Matches “[inference] Latency: 0.2073s” and “[inference] DeltaT: 0.4907s”
    latency_re = re.compile(r'Latency:\s*([\d.]+)s')
    delta_re   = re.compile(r'DeltaT:\s*([\d.]+)s')

    for line in lines:
        lat_m = latency_re.search(line)
        if lat_m:
            latencies.append(float(lat_m.group(1)))
        dt_m = delta_re.search(line)
        if dt_m:
            deltas.append(float(dt_m.group(1)))

    return latencies, deltas

def main():
    if len(sys.argv) > 1:
        # read from file
        with open(sys.argv[1], 'r') as f:
            lat, dt = parse_log_lines(f)
    else:
        # read from stdin
        lat, dt = parse_log_lines(sys.stdin)

    if not lat or not dt:
        print("No latency or DeltaT entries found.", file=sys.stderr)
        sys.exit(1)

    avg_lat = statistics.mean(lat)
    avg_dt  = statistics.mean(dt)
    min_lat = min(lat); max_lat = max(lat)
    min_dt  = min(dt);  max_dt  = max(dt)

    # Count how often thresholds are exceeded
    count_lat_over = sum(1 for l in lat if l > 0.24)
    count_dt_over  = sum(1 for d in dt  if d > 0.519)
    count_dt_under = sum(1 for d in dt if d <0.480)

    print(f"Parsed {len(lat)} latency entries, {len(dt)} ΔT entries")
    print(f"Average Latency: {avg_lat:.4f}s  (min: {min_lat:.4f}s, max: {max_lat:.4f}s)")
    print(f"Average ΔT:      {avg_dt:.4f}s  (min: {min_dt:.4f}s, max: {max_dt:.4f}s)")
    print(f"Latency > 0.24s: {count_lat_over} times")
    print(f"ΔT      > 0.519s: {count_dt_over} times")
    print(f"ΔT      < 0.480s: {count_dt_under} times")
    

if __name__ == "__main__":
    main()
