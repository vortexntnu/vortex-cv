# Behavior trees

Mission trees for BehaviorTree.CPP v4, one folder per competition.

```
behavior_trees/
└── robosub/
    ├── main.xml          Main (safety, ReactiveSequence), Mission (task order,
    │                     TaskSlot per task), SafeStop; includes tasks/*.xml
    └── tasks/            One subtree per task, BehaviorTree ID = task name
        ├── gate.xml          Gate
        ├── slalom.xml        Slalom
        ├── bins.xml          Bins
        ├── torpedo.xml       Torpedo
        ├── octagon.xml       Octagon
        └── return_home.xml   ReturnHome
```

The values the trees read (`{gate_depth}`, time budgets, course areas, ...)
are in `config/mission/robosub/mission.yaml`, loaded onto the blackboard by
`LoadMissionConfig` at the start of `Mission`. Numbers in the XML are start
values; course numbers are measured on site and go in the yaml.

## Conventions

- **One task, one file.** A task file is a complete `<root BTCPP_format="4">`
  with a single `BehaviorTree` whose `ID` is the task name. `main.xml` pulls
  them in with `<include path="tasks/<task>.xml"/>` (relative to `main.xml`).
- **Subtrees share the blackboard** through `_autoremap="true"`.
- **Every motion has a `Timeout`**, and a task is wrapped in a `TaskSlot` in
  `main.xml` (time budget, reserve for what comes after, always SUCCESS so one
  task never stops the run).
- **Fallbacks in XML, not in the nodes.** Nodes do one thing; searches, blind
  passes and backup plans are written in the tree.
- **No fixed x/y of the course.** Transitions and blind passes use the course
  frame (`GoToCourse`, `MoveCourse`), which follows the gate in
  `landmark_server`.
- **Landmark targets** go through `ApproachLandmark` (the `landmark_targets`
  library), all motion through `waypoint_manager`.

## Status

The trees are the reference design from the RoboSub course tree plan
(sections 1-7). The custom nodes (`PoseFeeder`, `MapFeeder`, `TaskSlot`,
`SelectGatePanel`, `MatchPipes`, `ApproachLandmark`, `GoToCourse`, ...) are
specified in section 8 of the plan and are not implemented yet, so the trees
do not load until those nodes are registered. `launch/mission/robosub/` is
where the launch file for the tree runner goes.

Adding a task: write `tasks/<task>.xml` with `BehaviorTree ID="<Task>"`, add
the `include` and a `TaskSlot` in `main.xml`, and its values in
`mission.yaml`.
