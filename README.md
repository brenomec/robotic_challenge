# Robotic Challenge - 2021

This repository has the solution of the Senai's selective process challenge, with the edit of the controller available by the selective process and some codes meshed.

## Files

- controllers: Robot controller code
- worlds: Simulation world

## Solution

The solution follow two requirements, that is a good navigation and the detection of the goal. Then all the used acessories are the default distance sensors and an additional RGB camera on top of the robot.

### Navigation

The default navigation code that was available by the selective process works good to execute the mission, but there's still a risk to get stuck even with the detection of the sensor like the image above. So, to unstuck in this kind of situation a condition was added to move the robot backward to try to avoid the obstacle.

The condition is, if the sensor get the max value, it's because the robot is too close to the wall to move forward, then it will move backward:

```
else if (sensor_value >= MAX_SENSOR_VALUE)
    stuck = 1;
...
        } else if (stuck == 1) {
          speed[0] = -MAX_SPEED;
          speed[1] = -MAX_SPEED;
          state = BACKWARD;
...
      case BACKWARD:
        if (stuck == 1) {
          speed[0] = -MAX_SPEED;
          speed[1] = -MAX_SPEED;
          state = BACKWARD;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break;
```

![stuck_robot](https://github.com/brenomec/robotic_challenge/blob/master/stuck_robot.png)

### Object Detection

A RGB camera was added on top of the robot and directed upward and forward. In the controller code, all the requisites were added and the objects numbereds. To be sure that the goal got recognized and not mistaken by another object, the recognize system of the Webots can distinguish the object and have sure what model it is seeing as the image above show.

![data](https://github.com/brenomec/robotic_challenge/blob/master/recognition_data.png)

With the possibility of filtering the objects, it could be possible to make the stop as the robot reach the goal and detect the sign, as shown on the image above.

![sign](https://github.com/brenomec/robotic_challenge/blob/master/recognited_sign.png)

After get all the information and detection, the condition was implemented to just finish the mission, as seen in the code above.

```
char stop[] = "stop panel";
...
      if (strcmp (stop,objects[i].model) == 0){
        state = GOAL;
      }
...
        case GOAL:
          speed[0] = 0;
          speed[1] = 0;
        break;
```

A MP4 file with the challenge being completed is available in the repository named as **robot_challenge.mp4**
