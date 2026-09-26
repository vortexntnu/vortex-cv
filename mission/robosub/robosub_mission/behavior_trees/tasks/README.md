# Task subtrees

One file per task, `<task>.xml`, each a complete document with one tree whose
`ID` is the task name:

```xml
<?xml version="1.0"?>
<root BTCPP_format="4">
  <BehaviorTree ID="Gate">
    ...
  </BehaviorTree>
</root>
```

To add a task: write the file here, add `<include path="tasks/<task>.xml"/>`
at the top of `../main.xml`, and `<SubTree ID="<Task>" _autoremap="true"/>` in
`Mission`. Its values go in
`perception_setup/config/mission/robosub/mission.yaml`.
