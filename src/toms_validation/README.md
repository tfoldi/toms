# toms_validation

Grasp and placement validators with structured failure types.

## Failure types
`empty_grasp`, `slip`, `dropped`, `placement_failed`, `uncertain`

## Contents
- `grasp_validator.py` – width + effort + lift + vision signals
- `placement_validator.py` – release check + zone check + stability check

## TODO (hardware)
- Implement lift-test signal (requires pre-lift pose snapshot).
- Implement post-retreat stability perception check.
- Calibrate all thresholds in `planning.yaml`.
