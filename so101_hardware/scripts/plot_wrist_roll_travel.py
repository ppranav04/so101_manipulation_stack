import csv

import matplotlib.pyplot as plt

t, raw, unw = [], [], []
with open("wrist_roll_travel_run1.csv") as f:
    for row in csv.DictReader(f):
        t.append(float(row["t_s"]))
        raw.append(int(row["raw"]))
        unw.append(int(row["unwrapped"]))

plt.plot(t, raw, label="raw (wraps)")
plt.plot(t, unw, label="unwrapped")
plt.axhline(2048, ls="--", c="gray", label="calibration centre")
plt.axhline(0, ls=":", c="red", label="wrap at -180 deg")
plt.axhline(4096, ls=":", c="red", label="wrap at +180 deg")
plt.xlabel("time [s]")
plt.ylabel("counts")
plt.legend()
plt.show()
