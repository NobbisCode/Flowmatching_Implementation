# Flowmatching

Flow-Matching-basierte Pfadplanung für den AgileX Scout Mini (ROS 2 lyrical).

Wie Roboter, Sensoren, ROS und PC zusammenspielen, steht in [SYSTEM.md](SYSTEM.md).

## Inhalt

| Pfad | Inhalt |
|---|---|
| `scripts/scout_control.py` | Roboter per Tastatur oder mit festem Befehl fahren (`/cmd_vel`) |
| `scripts/pure_pursuit.py` | Pfadfolger: folgt Pfaden auf `/fm_path` oder eingebauten Testpfaden |
| `data/` | Aufgezeichnete rosbags (MCAP) |

## Wo das Repo liegt

| Rechner | Pfad |
|---|---|
| Entwicklungs-PC (WSL) | `~/Flowmatching/src/Flowmatching_Implementation` |
| Roboter `scoutmini` | `~/fm_ws/src/Flowmatching_Implementation` |

Auf dem Roboter aktualisieren:

```bash
cd ~/fm_ws/src/Flowmatching_Implementation && git pull
```

## Skripte starten (auf dem Roboter, während `scoutlaunch` läuft)

```bash
cd ~/fm_ws/src/Flowmatching_Implementation
python3 scripts/scout_control.py                # Tastatursteuerung
python3 scripts/pure_pursuit.py --test circle   # Testpfad abfahren
```
