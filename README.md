# human_stand_dynamic_goal
Simple example of how to add a humanoid task to MJPC 

The files in the humanoid_stand_dynamic_goal folder are modifications of the original humanoid stand task in the MJPC folder mujoco_mpc/mjpc/tasks/humanoid_stand

To build the `tasks.cc` file in the mujoco_mpc/tasks folder needs to be modified, add 

```
#include "mjpc/tasks/humanoid/stand_dynamic_goal/stand_dynamic_goal.h"
```
and

```
std::make_shared<humanoid_dynamic_goal::StandDynamicGoal>(),
```

And also the `CMakeList.txt` in the mujoco_mpc/mjpc folder needs modifying with,

```
tasks/humanoid/stand_dynamic_goal/stand_dynamic_goal.cc
tasks/humanoid/stand_dynamic_goal/stand_dynamic_goal.h
```

The build can then be done using clang, with

```
~/Python_Projects/mujoco_mpc/build$ cmake ..
~/Python_Projects/mujoco_mpc/build$ cmake --build .
```

Finally, launch mjpc using,

```
~/Python_Projects/mujoco_mpc/build$ ~/Python_Projects/mujoco_mpc/build/bin/mjpc
```
