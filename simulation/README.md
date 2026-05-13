# CSP Simulation

This repository contains a differential-drive robot simulation for cooperative sequential pursuit and constant spacing platooning (CSP), including an enhanced DSR-based controller with trajectory feedforward.

## Overview

- `csp_simulation.ipynb` runs the simulation for a leader-follower robot convoy.
- The notebook compares three cases:
  1. CSP with LOS orientation tracking
  2. CSP with path curvature feedforward
  3. DSR-augmented CSP with path curvature feedforward
- The animation output is written to `simulation.gif`.

## Preview

If `simulation.gif` is in this directory, GitHub will render the animated preview automatically.

![Simulation](simulation.gif)

## Usage

1. Open `csp_simulation.ipynb` in Jupyter or VS Code.
2. Run the notebook cells from top to bottom.
3. The notebook generates `simulation.gif` in the same directory.

## Notes

- `simulation.gif` can be previewed directly in VS Code.
- After committing and pushing to GitHub, the GIF will animate in the repository view.

## Files

- `csp_simulation.ipynb` — main simulation notebook
- `source_trajectory/path_lab_field_shortened.npy` — leader path data
- `README.md` — this file
- `simulation.gif` — generated output animation (if present)

