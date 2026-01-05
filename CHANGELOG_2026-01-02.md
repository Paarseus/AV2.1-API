# Changes Made - January 2, 2026

## Summary

This document tracks all changes made to the UTM Navigator codebase on 2026-01-02 during a code review and improvement session.

---

## Changes That REMAIN in Codebase

### 1. Thread Safety - `perception/occupancy_grid.py`

**What was added:** RLock for thread-safe concurrent access

**Lines changed:**
- Import added at top: `from threading import RLock`
- In `__init__`: `self._lock = RLock()`
- Wrapped `update()`, `get_cost()`, `to_probability_grid()`, `clear()`, `get_obstacle_points()` with `with self._lock:`

**How to revert:**
```python
# Remove the import
from threading import RLock  # DELETE THIS LINE

# Remove from __init__
self._lock = RLock()  # DELETE THIS LINE

# Remove all "with self._lock:" blocks and unindent their contents
```

---

### 2. Thread Safety - `perception/costmap.py`

**What was added:** RLock for thread-safe concurrent access

**Same pattern as occupancy_grid.py** - wrapped `update()`, `get_cost()`, `check_collision()`, `check_path()`, `get_obstacle_points()`, `to_rgb()`

**How to revert:** Same as occupancy_grid.py

---

### 3. Error Handling - `sensors/xsens_receiver.py` (Line 41)

**Before:**
```python
assert len(self._buf) > 0
```

**After:**
```python
if len(self._buf) == 0:
    raise RuntimeError("No packet available in buffer")
```

**How to revert:** Change back to `assert len(self._buf) > 0`

---

### 4. New File - `requirements.txt` (project root)

**Content:** Core dependencies (numpy, scipy, matplotlib, opencv-python, osmnx, networkx, pyproj, pyserial)

**How to revert:** Delete the file

---

### 5. New File - `config/default.yaml`

**Content:** Centralized vehicle configuration parameters

**How to revert:** Delete the file (and `config/` directory if empty)

---

### 6. New File - `utils/logging_config.py`

**Content:** Logging setup utilities

**How to revert:** Delete the file

---

### 7. Modified - `utils/__init__.py`

**Added:**
```python
from .logging_config import setup_logging, get_logger

__all__ = [
    'ControlLogger',
    'setup_logging',
    'get_logger',
]
```

**How to revert:** Remove the new imports and restore original `__all__`

---

### 8. New Test Files

| File | Description |
|------|-------------|
| `tests/test_pure_pursuit.py` | 15 test cases for Pure Pursuit |
| `tests/test_dwa.py` | 20 test cases for DWA planners |
| `tests/test_perception.py` | 20 test cases for perception thread safety |

**How to revert:** Delete these files

---

## Changes That Were REVERTED (No Action Needed)

These were over-engineered changes that were already reverted during the session:

| File | Change | Status |
|------|--------|--------|
| `planning/dwa.py` | Epsilon in obstacle cost | REVERTED |
| `planning/dwa.py` | Goal position check | REVERTED |
| `planning/ackermann_dwa.py` | Steering clamp before tan() | REVERTED |
| `planning/ackermann_dwa.py` | Epsilon in obstacle cost | REVERTED |
| `planning/ackermann_dwa.py` | Goal position check | REVERTED |
| `control/pure_pursuit.py` | min_safe_lookahead guard | REVERTED |
| `control/pure_pursuit.py` | NaN guard on steering | REVERTED |
| `control/pure_pursuit.py` | Fallback to nearest point | REVERTED |

---

## Quick Revert Commands

If you need to revert everything quickly:

```bash
# Delete new files
rm requirements.txt
rm config/default.yaml
rm utils/logging_config.py
rm tests/test_pure_pursuit.py
rm tests/test_dwa.py
rm tests/test_perception.py

# Then manually revert:
# - perception/occupancy_grid.py (remove RLock)
# - perception/costmap.py (remove RLock)
# - sensors/xsens_receiver.py (restore assert)
# - utils/__init__.py (remove logging imports)
```

---

## Risk Assessment

| Category | Files Changed | Risk Level |
|----------|---------------|------------|
| Thread Safety | 2 files | Low - adds protection, doesn't change behavior |
| Error Handling | 1 file | Low - only affects error messages |
| New Infrastructure | 4 files | None - new files, don't affect existing code |
| New Tests | 3 files | None - only run when explicitly invoked |

The thread safety changes in perception are the most impactful. If you see any performance issues or deadlocks during testing, those would be the first to investigate.

---

## Testing Notes

Before physical testing with `runner_dwa.py`:

1. **Verify steering sign convention:** Send `set_steer_deg(10)` and confirm wheels turn RIGHT
2. **Point car in intended test direction** before starting (obstacles placed relative to initial heading)
3. **Check E-stop works** before testing

---

## Run Tests

```bash
# Run all new tests
pytest tests/test_perception.py tests/test_dwa.py tests/test_pure_pursuit.py -v
```
