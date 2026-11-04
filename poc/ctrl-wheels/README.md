# ctrl-wheels (placeholder)

This directory is planned for the bridge, relay, CSV log and Webots side of the twin-wheel balancing robot simulator. It is simulation-only. It is the twin of `poc/ctrl-wheel`, which is the single-wheel version: this directory is intended as a copy of that tree with the names changed, not a shared module.

## Status

Placeholder. There is no code yet. Nothing has been run. Nothing is validated on hardware. Simulation-only.

## Plan and tracking

- Plan: https://github.com/pluto-atom-4/balancing-robot-controller/discussions/266
- Earlier strategy: https://github.com/pluto-atom-4/freecad-workspace/discussions/481
- Tracking: issue #482 (umbrella) and issue #483 (this directory).

## Planned contents (not present yet)

- WHL2 line parser
- UDP v2 transport: command on port 5018, echo on port 5017
- CSV log
- Bridge
- Fake wheels
- `webots/` world and controller
- `CONTRACT.md`

## Notes

The single-wheel `poc/ctrl-wheel` stays untouched. Nothing in this directory changes its behaviour or its files.
