# toms_world

Canonical world state manager.  Single source of truth for all modules.

## Contents
- `world_state_manager.py` – thread-safe `WorldStateManager`

## Rules
- Only `WorldStateManager` mutates `WorldState`.
- All other packages call methods here; they do not modify `WorldState` directly.
