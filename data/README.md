# Daten

Alle Aufnahmen stammen vom RPLIDAR A2M12 auf dem Scout Mini. Sie liegen als rosbag
(MCAP, `metadata.yaml` + `.mcap`) vor, zum Teil zusätzlich als NumPy-Export (`scans.npz`,
erzeugt mit `scripts/bag_to_npz.py`).

## `lidar_raw_static/` – Rohdaten, Roboter stehend (27.11.2026)

**Zweck:** Referenz, um synthetische Lidar-Daten an den echten Sensor anzupassen
(Strahlzahl, Winkelraster, Rauschen, Ausfälle).

**Aufnahme:** Nur der Lidar-Treiber lief, ohne Fahrgestell und ohne SLAM. Roboter
stand still, statische Laborumgebung, 61,4 s.

**Treiber-Einstellungen** (abweichend vom Normalbetrieb in `scoutmini_launch/config/rplidar.yaml`):

| Parameter | Hier | Normalbetrieb |
|---|---|---|
| `interpolated_rays` | **false** (echte Messpunkte, nur auf das LaserScan-Raster einsortiert) | true (auf 1450 feste Strahlen interpoliert) |
| `publish_point_cloud` | true | – |
| `scan_mode` | Standard | Standard |
| `angle_compensate` | true | true |
| Motordrehzahl | 600 rpm (Standard) | 600 rpm |

**Topics:** `/scan` (`sensor_msgs/LaserScan`, 553 Scans), `/cloud`
(`sensor_msgs/PointCloud2`, Felder x, y, z, rgb, 553 Wolken). Frame: `laser_frame`.

**Kennwerte** (aus `scans.npz` berechnet):

| Größe | Wert |
|---|---|
| Scanrate | ~9,0 Hz (Periode 0,111 s ± 0,005 s) |
| Strahlen pro Scan | schwankt, 1544–1590, im Mittel ~1569. Ein Ausreißer-Scan hat 3164 Strahlen (vermutlich zwei Umdrehungen zusammen) |
| Winkelschritt | ≈ 0,23° (`angle_increment` je Scan unterschiedlich) |
| `angle_min` | 0° … 0,2°, Startwinkel variiert von Scan zu Scan |
| Gemessene Distanzen | 1,17 m … 5,51 m (Median 2,0 m) in dieser Umgebung; `range_max` 12 m |
| Keine Rückmeldung (`inf`) | 9,7 % aller Strahlen. In 25 von 360 1°-Sektoren fehlt mehr als die Hälfte der Werte |
| Rauschen (Std. über die Zeit je 1°-Sektor, statische Szene) | 0–2 m: ~5 mm · 2–4 m: ~12 mm · 4–6 m: ~16 mm |
| Intensitäten | nicht vorhanden |

Hinweise:
- Ein Strahl ohne Echo ist `inf`, nicht 0. Im `.npz` sind die Arrays nach `n_rays` mit `NaN` aufgefüllt.
- Die Rausch-Werte je Sektor enthalten an Kanten auch Sprünge zwischen Vorder- und
  Hintergrund. Der Median ist deshalb aussagekräftiger als der Maximalwert.
- Fürs Training relevant: Im Normalbetrieb (mit SLAM) sieht der Roboter interpolierte
  Scans mit genau 1450 Strahlen über 0…2π. Siehe `scan_sample/`.

## `scan_sample/` – Normalbetrieb, kurz (27.11.2026)

11,5 s mit vollständigem `scoutlaunch` (interpolierte Scans, 1450 Strahlen, ~9,2 Hz).
Topics: `/scan`, `/odom`, `/tf`, `/tf_static`. Bewegung des Roboters während der
Aufnahme nicht geprüft.

## Lesen

```bash
ros2 bag info data/lidar_raw_static
python3 scripts/bag_to_npz.py data/lidar_raw_static      # erzeugt scans.npz
```

```python
import numpy as np
d = np.load("data/lidar_raw_static/scans.npz")
k = 0
n = d["n_rays"][k]
r = d["ranges"][k, :n]
a = d["angle_min"][k] + np.arange(n) * d["angle_increment"][k]
```
