# Scout fence margin proposal

The original proposal enlarged the default controller fence. Current XGC2 instead
freezes the authored Experiment `controlBounds` in the Session and materializes
those exact XY bounds in every controller parameter file. Changing the default
YAML does not enlarge a configured Session and must not silently enlarge an
unconfigured one.

`scout_fence_margin.yaml` preserves the optional 0.5 m proposal for the historical
planner box x=[-12,12], y=[-7,7]. It is not referenced by any launch file. Keep the
normal profile and its current HEALTH timeout; use the Experiment's authored
boundary for a real run. The actual robot footprint, localization uncertainty
and stopping envelope must fit the physical venue. If they do not, reduce or
redesign the planning envelope instead of expanding the venue boundary.

The regression test loads the current production materializer with the real
Scout profile and candidate, verifies that a smaller frozen Session overrides
all candidate endpoints, and verifies that an explicit absent override retains
the unchanged default. This is configuration-path acceptance, not field or
stopping-distance validation.

Run `python3 unicycle_ugv_controller/test/test_scout_fence_contract.py -v` and
`python3 ugv_reset_safety/test/world_boundary_launch_test.py -v`.
