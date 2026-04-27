# TA_APID-control (APID + RAPID)

MATLAB reference implementations for two controllers for EF-responsive T4 regulation:

- **APID**: windowed adaptive PID with **band-lock** holding.
- **RAPID**: robust adaptive PID with **multi-perturbation robustness** (CVaR/mean blending).

## Repository layout

```text
TA_APID-control/
├── apid/
│   ├── run_apid_bandlock_smooth.m
│   ├── src/                # APID-only helpers
│   └── examples/
├── rapid/
│   ├── run_rapid_clean5.m
│   ├── src/                # RAPID-only helpers
│   └── examples/
├── common/
│   ├── plant/              # nominal plant + plant parameter helpers
│   ├── timing/             # EF burst timing helpers
│   └── utils/              # clamp, rate_limit, mappings, etc.
├── experiments/            # comparisons / sweeps (scripts)
└── results/                # generated plots, saved runs, exports
```

## Quick start

From the repo root in MATLAB:

```matlab
addpath(genpath(pwd));

OUT_apid  = run_apid_bandlock_smooth(25);
OUT_rapid = run_rapid_clean5(25);
```

### APID

```matlab
OUT = run_apid_bandlock_smooth(25, struct(
    'dt_seconds', 0.1, ...
    'sim_hours', 80, ...
    'make_plots', true, ...
    'verbose', true));
```

### RAPID

```matlab
OUT = run_rapid_clean5(25, struct(
    'rng_seed', 7, ...
    'sim_hours', 80, ...
    'do_plots', true, ...
    'verbose', true));
```

## Notes

- `common/` contains shared pieces (nominal plant, timing, small utilities).
- `apid/src` contains APID-specific helpers (band-lock update, APID one-window cost, etc.).
- `rapid/src` contains the RAPID refactor helpers (robust cost, perturbation sampling, etc.).
- Put any *comparison* scripts (e.g., APID vs RAPID plots) in `experiments/`, and write outputs to `results/`.

## Reproducibility

- RAPID uses explicit local `RandStream` objects (seeded via `opts.rng_seed`) to keep runs repeatable.

