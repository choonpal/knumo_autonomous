# Slim Official Slalom — 2-D Revalidation

- Seeds: `20260821`, `20260822`, `20260823`, `20260824`
- Random trials: 30 per pattern, per random group, per seed
- Distinct scenario executions: `518`
- Geometry: 1.40 × 0.65 m vehicle, 0.50 × 0.90 m boxes, 2.50 m face gap, 3.00 m centre spacing, boxes flush to lane divider.
- Controller: 20 Hz; LiDAR: 200° model at 10 Hz; locked speed 0.50–0.70 m/s; |ω| ≤ 1.00 rad/s.

> These are deterministic simulation scenarios, not a measured probability of real-vehicle success.

## Results

| Group | Trials | Passes | Failures | Min inflated box margin | Min road margin | Mean command speed |
|---|---:|---:|---:|---:|---:|---:|
| nominal | 2 | 2 | 0 | 0.192 m | 0.163 m | 0.693 m/s |
| exact_uncertainty | 240 | 240 | 0 | 0.143 m | 0.099 m | 0.682 m/s |
| placement_stress | 240 | 240 | 0 | 0.083 m | 0.081 m | 0.677 m/s |
| boundary_stress | 36 | 36 | 0 | 0.058 m | 0.057 m | 0.661 m/s |

## Modeled uncertainty

`exact_uncertainty` varies initial lateral/yaw error, actual yaw response, angular/linear lag, 0–100 ms command delay, speed scale, and localization bias while keeping boxes at nominal coordinates.

`placement_stress` additionally varies each box by up to ±0.08 m longitudinally and ±0.04 m laterally.

`boundary_stress` moves obstacle rows 0.04 m toward the selected pass line and combines endpoint values of the plant-delay and pose-error assumptions.

## Interpretation

All modeled scenarios completed without physical contact, inflated-footprint contact, or road-corridor violation. The minimum modeled margin in the harsh deterministic set remains about 0.058 m, so real-world errors totaling several centimetres can still consume the remaining margin.
