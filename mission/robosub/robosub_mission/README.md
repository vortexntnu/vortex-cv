# robosub_mission

Runner and trees for RoboSub. The nodes are in `vortex_bt_nodes`.

```
src/main.cpp        loads trees/root.xml and ticks it (10 Hz)
trees/root.xml      Main → Mission (task order) → SafeStop
trees/<task>.xml    one subtree per task, BehaviorTree ID = task name
```

Launch: `perception_setup/launch/mission/robosub/robosub_mission.launch.py`.
Values: `perception_setup/config/mission/robosub/mission.yaml`; add your
task's values there and read them as `{key}`.

| Task | Owner |
|---|---|
| Gate | Amélie |
| Slalom | André |
| Bins | Karol |
| Torpedo | Johannes |
| Octagon | Ashish |

A task tree is done when:
- every move and wait has a `Timeout`;
- the numbers come from `mission.yaml`, and moves between tasks are in the
  course frame;
- every way the task can fail has a fallback, and a failed task never stops
  the run;
- it succeeds in the sim.

Add it with `<include path="<task>.xml"/>` and a
`<SubTree ID="<Task>" _autoremap="true"/>` in `root.xml`.
