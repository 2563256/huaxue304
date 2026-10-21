# Fleet State Lattice control sets

The wheeled robots load `hetero_fleet_navigation::SweptLattice`, the fleet's
Nav2 GlobalPlanner implementation. It searches a finite `(cell x, cell y,
heading)` lattice with weighted A*. Every edge uses one control-set primitive;
there is no Hybrid A* or analytic-expansion fallback. Search collision checks
include the filled rectangle, occupied-cell areas and the entire primitive,
including stationary rotations. Masks also include Nav2's Bresenham footprint
boundary cells at every published pose, keeping IsPathValid consistent with
the search. The map is locked and reread on each request,
so an obstacle added or removed between plans changes the result.

`diff_64.json` (1,952 primitives) and `omni_64.json` (2,152 primitives) were
produced using [Navigation2's Jazzy lattice generator](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_smac_planner/lattice_primitives),
an Apache-2.0 utility by Matthew Booker. The generator configuration is:

```json
{"motion_model": "diff or omni", "turning_radius": 0.5,
 "grid_resolution": 0.05, "stopping_threshold": 5, "num_of_headings": 64}
```

Generation uses separate `diff` and `omni` runs, followed by:

```bash
python3 tools/prepare_lattice.py --diff-source /path/to/diff/output.json \
  --omni-source /path/to/omni/output.json \
  --output-dir src/hetero_fleet_bringup/config/lattice
```

The preparation includes `(0,0,start_heading)` where the generated samples
omit it. It fixes small stationary rotations whose last generated sample
retains the starting angle despite the declared end heading. For omni it
adds explicit negative translations while preserving yaw and retains moving
curves. Duplicate primitives are removed and groups sorted by start heading.
The diff planner mirrors translations, with yaw unchanged, for reverse
expansion; omni reverse moves are already explicit in its control set.

`lattice_filepath`, `allow_reverse_expansion`, `swept_footprint`, and
`swept_footprint_padding` select the model and physical body. The last two must
match the local and global costmap geometry. `tolerance` bounds the sub-cell
goal connector; start connectors are bounded by the map resolution. Every
connector also receives the full swept-body check. `allow_unknown: false`
blocks unknown cells and map boundaries. Swept masks use corner-travel steps
of resolution/8 plus half-step padding to cover continuous motion.

`heuristic_weight: 1.5` biases the obstacle-distance heuristic; soft inflation
costs favour clearance. `cost_penalty: 2.0`, `reverse_penalty: 2.0`, and
`rotation_penalty: 2.0` tune travel, backwards travel and stationary rotation.
Each state is expanded once; costs are not guaranteed globally optimal.
`max_planning_time: 8.0` and `max_iterations: 1000000` bound search. Parameters
and body masks are read at lifecycle configuration; reconfigure/restart after
changing them. The map origin and dimensions may change between requests,
but changing its resolution requires reconfiguration and a matching control set.

Build, source the overlay, then run `tools/verify_navigation.py` for both task
scenarios and `tools/verify_lattice.py` for native motion and obstacle tests.
These checks cover geometric feasibility, not Gazebo tracking or fleet traffic.
