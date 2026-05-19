# Drone Mapper

A C++20 simulator for an autonomous 3D building mapping drone, submitted as Assignment 1 for TAU Advanced Topics in Programming (Semester B 2026).

## Overview

The drone flies inside a 3D building, uses a simulated LiDAR sensor to scan its surroundings, and builds a sparse 3D voxel map. When mapping is complete it writes the result to file and scores it against the ground-truth input map.

The simulation runs in software only — all hardware (LiDAR, position sensor, movement driver) is replaced by mock implementations that are hidden from the drone's algorithm. The drone interacts exclusively through abstract interfaces and has no knowledge of the underlying simulation.

## Architecture

```
include/
  interfaces/     – pure abstract interfaces (ILidarSensor, IMovementDriver,
                    IPositionSensor, IMap3D, IMapBuilder)
  drone/          – Drone, MapAlgorithm, MapBuilder
  simulation/     – mock sensors, movement driver, voxel map, cell map
  IO/             – config parser, map I/O, error logger, scorer
  types/          – strong-typed units (Units.h), map values, commands
src/              – implementations of the above
scenario*/        – ready-made input files for test runs
```

The `drone/` and `interfaces/` layers know nothing about the simulation. The `simulation/` layer implements those interfaces using the ground-truth voxel map. All three mock objects (`MockLidarSensor`, `MockPositionSensor`, `MockMovementDriver`) share a single `SimulationState` struct — the movement driver writes to it and the two sensors read from it, keeping state consistent without coupling to each other.

All physical values in the codebase use the **mp-units** library (`cm`, `m`, `deg`) for compile-time unit safety. X, Y, and Z lengths are separate quantity specs (`x_extent`, `y_extent`, `z_extent`) so they cannot be accidentally mixed. Horizontal and altitude angles are also distinct types.

## Building

**Requirements:** CMake ≥ 3.20, C++20 compiler, [vcpkg](https://vcpkg.io) with the `mp-units` package.

```bash
# configure (vcpkg toolchain path must be set, or use the CMakePresets)
cmake --preset default        # or: cmake -B build -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
cmake --build build
```

The build compiles all drone, IO, and simulation sources into a static library `drone_core`, then links it into the `drone_mapper` executable. The library boundary makes it straightforward to add GTest-based unit tests against individual components without touching `main.cpp`.

The build produces a single executable: `build/drone_mapper`.

## Running

```
drone_mapper [<input_output_files_path>]
```

If the path argument is omitted the current working directory is used. The program reads three files from that directory:

| File | Purpose |
|------|---------|
| `drone_config.txt` | Drone capabilities (physical size, movement limits, LiDAR config) |
| `mission_config.txt` | Mission parameters (boundaries, start position, map resolution) |
| `map_input.txt` | Ground-truth building map (used only by the mock LiDAR, never by the drone) |

It writes one output file:

| File | Purpose |
|------|---------|
| `map_output.txt` | Drone's built map, same format as `map_input.txt` |
| `input_errors.txt` | Created only when recoverable config errors are detected |

The mapping score (0–100) is printed to stdout at the end of each run.

### Example — run scenario 1

```bash
./build/drone_mapper scenario1
```

## Startup sequence

When launched, `main` performs these steps in order:

1. Resolve the I/O path (CLI arg or CWD).
2. Parse `drone_config.txt` and `mission_config.txt`; log recoverable errors.
3. Parse `map_input.txt` into a `DenseVoxelMap` (ground truth).
4. Validate that `map_resolution_cm` in the mission config matches the actual grid step detected in the input map — mismatch is a fatal error.
5. Construct `SimulationState` and the three mock objects.
6. Construct the `Drone` and `MappingAlgorithm`, then call `algorithm.Run()`.
7. Augment the drone's output with `-1` / `-2` entries for every ground-truth cell the drone did not explicitly record.
8. Write `map_output.txt` and print the score.
9. Flush any logged input errors to `input_errors.txt` (only if errors exist).

## File Formats

### `drone_config.txt`

```
min_pass_width_cm  = 20       # minimum opening the drone can pass through
min_pass_length_cm = 20
min_pass_height_cm = 20
max_rotate_deg     = 45       # max rotation per single Rotate command
max_advance_cm     = 10       # max distance per single Advance command
max_elevate_cm     = 10       # max distance per single Elevate command
lidar_beam_min_cm       = 5   # Z-min: too close to measure accurately
lidar_beam_max_cm       = 120  # Z-max: beyond this range nothing is detected
lidar_circle_spacing_cm = 5   # D: spacing between beam circles at Z-min
lidar_fov_circles       = 4   # FOVC: number of beam circles (0 = single centre beam)
```

### `mission_config.txt`

```
boundary_polygon     = (0,0),(200,0),(200,200),(0,200)  # XY bounding rectangle (cm)
min_height_cm        = 0
max_height_cm        = 100
start_x_cm           = 100   # drone centre at launch
start_y_cm           = 100
start_height_cm      = 50
output_resolution_xy = 0     # decimal places (kept for backwards compatibility)
output_resolution_h  = 0
map_resolution_cm    = 5     # grid cell size; must match the input map
```

### `map_input.txt` / `map_output.txt`

```
# drone_mapper map file
x_min=0 x_max=200 y_min=0 y_max=200 z_min=0 z_max=100
# x y z value
0 0 0 1
0 0 5 1
...
```

The header line declares the bounding box of the map. Each data line is `x y z value` (all in cm, integer coordinates) where value is:

| Value | Meaning |
|-------|---------|
| `1` | Occupied (wall / obstacle / ceiling / floor) |
| `0` | Empty (traversable space) |
| `-1` | Not mapped — inside mission boundaries but drone could not reach it |
| `-2` | Outside mapping boundaries |

Only cells with non-default values need to appear; any position absent from the file is implicitly not mapped.

## Mapping Algorithm

The drone uses a **BFS-based frontier exploration** across horizontal slices. At each altitude level it maintains a visited set and a frontier queue of grid cells. The core steps are:

1. Scan the current position with the LiDAR (`ScanAndUpdate`) — in all horizontal directions and at several elevation angles.
2. Pop the next unvisited frontier cell, find a collision-free path to it (`FindPath`), and move there step by step.
3. When no more reachable frontier cells exist at the current altitude, elevate to the next height level (`ElevateToNextHeight`) and repeat.
4. Declare `Finished` once all altitude levels have been exhausted.

Navigation helpers (`RotateToFace`, `AdjustHeight`, `MoveToCell`) ensure the drone never commands a movement that would exceed the per-command limits from `drone_config.txt`. The `HasClearance` check uses the drone's minimum pass-through dimensions to avoid entering openings that are too narrow.

## Scoring

The score (0–100) is computed cell-by-cell against the ground-truth map:

| Drone output for a GT cell | Effect |
|----------------------------|--------|
| Correct (`0` or `1` matching GT) | +1 correct |
| Wrong classification | −0.5 penalty |
| `-1` (not mapped, inside mission) | −0.5 penalty |
| `-2` (outside mission boundaries) | excluded from score entirely |
| Missing from output | −0.5 penalty (treated as not mapped) |

```
score = max(0, correct − penalty) / total × 100
```

Only cells that are definitively `Occupied` or `Empty` in the ground truth count toward `total`. A score of 100 is achievable when the drone correctly classifies every reachable cell; inaccessible areas lower the theoretical maximum.

## Scenarios

| Scenario | Description |
|----------|-------------|
| `scenario1` | Small 200×200 cm empty room, height 0–100 cm |
| `scenario2` | 200×200×100 cm room with obstacles |
| `scenario3` | 200×200 cm room with a complex layout |

Each scenario directory contains `drone_config.txt`, `mission_config.txt`, `map_input.txt`, and an `original_output/` folder with a reference `map_output.txt` and a visualisation PNG.

## Error Handling

- Bad or missing config values are replaced by safe defaults and written to `input_errors.txt` (created only when errors occur).
- A missing or unparseable `map_input.txt`, or a resolution mismatch between the mission config and the input map, is a fatal error — a message is printed to `stderr` and the program returns a non-zero exit code.
- The program never calls `exit()` or similar; all code paths return through `main`.
- The program will not crash on any valid or invalid input.

## Design Notes

- The drone is modelled as a perfect sphere (Ex1 simplification) for collision and clearance checks.
- Battery is assumed infinite; no recharging positions are used in Ex1.
- All decisions in the mapping algorithm are deterministic — no randomness.
- The `drone_core` static library boundary keeps drone logic testable in isolation from `main.cpp`.

See [HLD.pdf](HLD.pdf) for the full UML class diagram, sequence diagram, and design rationale.
