
```
roslaunch openni2_launch openni2.launch  # for xtion camera
```

```
rosrun gambit_perception interactive_mask
```

```
rosrun gambit_perception learn_basetable
```

```
rosrun gambit_perception event_main_simple_blocksworld
```

If something goes wrong, check

1. `img_dep_sync_node.h` L46-L48
2. `img_dep_sync_node.h` L113
3. paths of the param files (e.g. in interactive_mask src code)
