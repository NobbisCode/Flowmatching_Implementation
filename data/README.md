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
  Scans mit genau 1450 Strahlen über 0…2π. Siehe `lidar_processed_driving/`.

## `lidar_processed_driving/` – Normalbetrieb, Fahrt per Fernsteuerung (27.11.2026)

**Zweck:** Genau die Scans, die im Betrieb auf `/scan` publiziert werden und auf die
später das Netz hört, während einer Fahrt.

**Aufnahme:** Vollständiger `scoutlaunch` (Fahrgestell, Lidar, slam_toolbox). Roboter
per Fernsteuerung gefahren (`control_mode = 3` die ganze Zeit), 121 s, Laborumgebung.

**Treiber-Einstellungen:** wie im Normalbetrieb, `interpolated_rays: true`,
`computed_ray_count: 1450`, `scan_mode: Standard`.

**Topics:** `/scan` (1081), `/odom` (~50 Hz), `/tf`, `/tf_static`, `/scout_status`,
`/rc_status`, `/map` (606), `/pose` (87). Die `scout_msgs`-Topics lassen sich nur
deserialisieren, wenn `scout_msgs` installiert ist (auf dem Roboter ja, auf dem PC nicht).

**Kennwerte:**

| Größe | Wert |
|---|---|
| Scanrate | ~8,9 Hz (Periode 0,112 s) |
| Strahlen pro Scan | **immer 1450** |
| Winkel | `angle_min` = 0, `angle_increment` = 0,24828° (2π/1450), also 0° … 359,75° |
| Keine Rückmeldung (`inf`) | 12,2 % insgesamt; je Scan 4,9 % … 20 % (Median 12,5 %) |
| Gemessene Distanzen | 0,17 m … 6,1 m (1 %-Quantil 0,37 m, Median 1,9 m, 99 %-Quantil 4,7 m); `range_max` 12 m |
| Nächstes Hindernis je Scan | Median 0,67 m, minimal 0,17 m |
| Fahrt (Odometrie) | 23,2 m Weg in einem Bereich von ca. 3,6 m × 3,3 m; v = −0,79 … 0,78 m/s, ω = −0,97 … 0,95 rad/s; 59 % der Zeit in Bewegung |

`scans.npz` enthält zu jedem Scan zusätzlich die nächstgelegene Odometrie (`odom_x`,
`odom_y`, `odom_yaw`, `odom_v`, `odom_w`, zeitlicher Abstand `odom_dt` ≤ 10 ms).

## `lidar_processed_short/` – Normalbetrieb, überwiegend stehend (27.11.2026)

61,5 s, vollständiger `scoutlaunch`, gleiche Einstellungen wie oben. Bei ca. 5–10 s
wurde der Roboter per Fernsteuerung ~1,65 m rückwärts gefahren, sonst stand er.
Topics: `/scan` (550, je 1450 Strahlen), `/odom`, `/tf`, `/tf_static`, `/scout_status`.
`inf`-Anteil 12,6 %, Distanzen 0,19 m … 6,6 m.

## `scan_sample/` – Normalbetrieb, kurz (27.11.2026)

11,5 s mit vollständigem `scoutlaunch` (interpolierte Scans, 1450 Strahlen, ~9,2 Hz).
Topics: `/scan`, `/odom`, `/tf`, `/tf_static`. Ob sich der Roboter bewegt hat, wurde nicht geprüft.

## Rohdaten vs. vorverarbeitete Daten

| | Roh (`lidar_raw_static`) | Vorverarbeitet (`lidar_processed_*`) |
|---|---|---|
| Strahlen/Scan | 1544–1590, schwankend | 1450, fest |
| Winkelraster | ≈ 0,23°, Startwinkel schwankt | 0,24828°, Start immer 0 |
| Entstehung | echte Messpunkte | Strahlen eines festen Rasters mit der Verbindungslinie zweier benachbarter Messpunkte geschnitten |
| Wofür | Rauschen und Ausfälle des Sensors verstehen | Format, das SLAM und das Netz im Betrieb bekommen |

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
