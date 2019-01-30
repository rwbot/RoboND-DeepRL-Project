[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

#### RoboND DeepRL Project Writeup: [Markdown](https://github.com/rwbot/RoboND-DeepRL-Project/blob/master/WRITEUP.md)  or  [PDF](https://github.com/rwbot/RoboND-DeepRL-Project/blob/master/WRITEUP.pdf)

# Deep Reinforcement Learning Arm Manipulation

This project is based on the Nvidia open source project "jetson-reinforcement" developed by [Dustin Franklin](https://github.com/dusty-nv). The goal of the project is to create a DQN agent and define reward functions to teach a robotic arm to carry out two primary objectives:

1. Have any part of the robot arm touch the object of interest, with at least a 90% accuracy.
2. Have only the gripper base of the robot arm touch the object, with at least a 80% accuracy.


## Setup
See below for environment setup. In the method `ArmPlugin::Load()`, subscriber nodes were defined to facilitate camera communication and collision detection through Gazebo:
```cpp
// Camera subscriber
cameraSub = cameraNode->Subscribe("/gazebo/arm_world/camera/link/camera/image",&ArmPlugin::onCameraMsg, this);
// Collision Message subscriber
collisionSub = collisionNode->Subscribe("/gazebo/arm_world/tube/tube_link/my_contact",&ArmPlugin::onCollisionMsg, this);
```

In the method `ArmPlugin::Create()`, the DQN Agent was created using the parameters defined at the top of **Armplugin.cpp**:
```cpp
agent = dqnAgent::Create(INPUT_WIDTH, INPUT_HEIGHT, INPUT_CHANNELS,
    DOF*2, OPTIMIZER, LEARNING_RATE, REPLAY_MEMORY, BATCH_SIZE, GAMMA,
    EPS_START, EPS_END, EPS_DECAY, USE_LSTM, LSTM_SIZE, ALLOW_RANDOM, DEBUG_DQN);
```

The output of the DQN agent is mapped to particular actions of the robotic arm. The method `ArmPlugin::updateAgent()` is responsible for executing the action selected by the DQN agent. Two types of joint control were implemented: velocity control and position control. Both are dependent upon whether the issued action is an odd or even number.  Odd-numbered actions decrease the joint position/velocity, while even-numbered actions increase the joint position/velocity. Additionally, the current delta of the position/values is used:
```cpp
// Evaluating if action is even or odd
const int actionSign = 1 - 2 * (action % 2);
// Set joint position
float joint = ref[action/2] + actionSign * actionJointDelta;
// Set joint velocity
const float velocity = vel[action/2] + actionSign * actionVelDelta;
```

## Reward Function
<!--- Explain the reward functions that you created: Brief explanation of each reward function and associated reward values. The writeup should also include what type of joint control was implemented. -->

The reward function is a crucial component in guiding the DQN agent through the process of learning to manipulate the arm. After every end of episode (EOE), a particular reward is issued, depending on the triggering event.

#### `REWARD_LOSS` is issued when the current episode exceeds 100 steps or when the arm contacts the ground.

The `REWARD_LOSS` issued when the arm contacts the ground is detected using the `GetBoundingBox()` function from the Gazebo API which provides the min/max XYZ values of the arm. Comparing this with the Z value of the ground, collisions can be detected.
```cpp
const bool checkGroundContact = (gripBBox.min.z <= groundContact || gripBBox.max.z <= groundContact);

if(checkGroundContact){
    rewardHistory = REWARD_LOSS;
    ...
}
```

#### `REWARD_WIN` is issued when any part of the arm contacts the object (Objective 1) and when only the gripper contacts the object (Objective 2).
The reward function for arm and object collision first checks for a collision between any part of the arm and the object. Depending on which objective was being attempted, the boolean `GRIP_ONLY` controls whether the arm-object collision is a win or loss.

If `GRIP_ONLY == 0`, the rules of Objective 1 are applied, and the arm-object collision is considered a win. Additionally, if that part happens to be the gripper, the reward is multiplied.

If `GRIP_ONLY == 1`, the rules of Objective 2 are applied, and the arm-object collision is considered a loss by default. However, if that part happens to be the gripper, the collision is issued a multiplied reward.

```cpp
// Define whether ARM or GRIPPER only is considered a win
#define GRIP_ONLY 0

// Check if the collision is between any part of the arm and the object
bool collisionItemCheck = ( strcmp(contacts->contact(i).collision1().c_str(), COLLISION_ITEM) == 0 );

if (collisionItemCheck){
    // Reward for any collision with arm given only when GRIP_ONLY == 0
    rewardHistory = GRIP_ONLY ? REWARD_LOSS : REWARD_WIN;

    // Check if the collision is between only the gripper and the object
    bool collisionPointCheck = ( strcmp(contacts->contact(i).collision2().c_str(), COLLISION_POINT) == 0 );
    if (collisionPointCheck)
        rewardHistory = rewardHistory * 5.0f;

    // Update state
    newReward  = true;
    endEpisode = true;
    return;
}
```

### Interim Reward
To reduce the necessary training time, an interim reward function was implemented which would help guide the arm towards the object. The reward is proportional to the change in distance between the arm and the object. If the arm is moving away from the object, the interim reward is negative. Conversely, if the arm is moving towards the object, the interim reward is positive. The raw values cause the arm to have a jerking movement, so they are smoothed by taking a moving average. Finally, due to the agent's tendency to remain still, a time penalty was introduced to force the agent to finish as quickly as possible.
```cpp
// The current distance to the goal
const float distGoal = BoxDistance(gripBBox,propBBox);
// The current change in distance from the last position
const float distDelta  = lastGoalDistance - distGoal;
// Smoothing the values using a moving average
avgGoalDelta  = (avgGoalDelta * ALPHA) + (distDelta * (1.0f - ALPHA));
// Time penalty to reduce stand-still and completion time
rewardHistory = REWARD_INTERIM * avgGoalDelta - TIME_PENALTY;
```

## Hyperparameters
<!--- Specify the hyperparameters that you selected for each objective, and explain the reasoning behind the selection. Student should explain the choice of hyperparameters for both objectives. -->

Below are all the relevant parameters used. Between both objectives, only 3 parameters were changed.

In Q-Learning, epsilon-Greedy is a typical exploration method used to encourage an agent to explore more of the state-action space. The larger the value, the more frequently a random action is chosen instead of one with the highest q-value.
`EPS_START` defines the initial value.  **0.9f** ensures the agent is exposed to many state-action spaces.
`EPS_END` defines the final value. **0.0f** ensures that only learned actions with the highest q-value is used once it has been sufficiently exposed.
`EPS_DECAY` defines the rate by which the value decays from initial to final over time. For objective 2, this was increased to **250**. Touching the gripper to the object was less likely to happen than the whole arm. Preferably, the agent needed enough exploration to encounter that situation, but not too much that the agent falls into a "best loss" policy.
```cpp
// DQN API Settings
#define INPUT_CHANNELS 3
#define ALLOW_RANDOM true
#define DEBUG_DQN false
#define GAMMA 0.9f
#define EPS_START 0.9f
#define EPS_END 0.0f
#define EPS_DECAY 250            // Obj1: 200
```

The input dimensions were set to match that of the image captured by the camera: 64 x 64 pixels. `"RMSprop"` was used as the optimizer. `LSTM` was set to **true** to enable the agent to consider previously encountered states. `LSTM_SIZE` was set to **256**, increasing the agent's ability to consider more complex moves. A `LEARNING_RATE` of **0.1f** was sufficient for the first objective, but not for the second as it took much too long learn. It was increased to **0.9f** because it was high enough that the time reduced dramatically and low enough to ensure it would actually converge. Similarly, the `BATCH_SIZE` was increased from **64** to **128**.
```cpp
// Learning Hyperparameters
#define INPUT_WIDTH   64
#define INPUT_HEIGHT  64
#define OPTIMIZER "RMSprop"
#define LEARNING_RATE 0.9f        // Obj1: 0.1f
#define REPLAY_MEMORY 10000
#define BATCH_SIZE 128            // Obj1: 64
#define USE_LSTM true
#define LSTM_SIZE 256
```

`REWARD_INTERIM` was the reward incrementally issued to lead the agent towards the object. `ALPHA` was used to calculate a moving average of the delta distances. To deter the agent from standing still, a `TIME_PENALTY` was introduced. All three of these were tuned concurrently to find the values that issued appropriate interim rewards.
```cpp
// Reward Parameters
#define REWARD_WIN  30.0f
#define REWARD_LOSS -30.0f
#define REWARD_INTERIM 5.0f
#define ALPHA 0.6f
#define TIME_PENALTY 0.4f
```

## Results
<!--- Explain the results obtained for both objectives. Include discussion on the DQN agent's performance for both objectives. Include watermarked images, or videos of your results.
Student should describe and briefly explain the results they achieved for both objectives. The discussion should also include their comments on the DQN agent's performance and if there were any shortcomings. Student should include either watermarked images of their results, or attach a video that displays the results and the arm in action. -->

The DQN was able to successfully complete both objectives. Objective 1 parameters were consistent in results of 90% within the range of 100 to 120 episodes. Objective 2 parameters were less consistent. The initial episodes significantly influenced the convergence time. On less ideal runs, it would take the agent 40 episodes before receiving a single win. On more ideal runs, the agent would receive a win within the first 15 episodes. This puts the result of 80% consistent within the range of 100 to 300 episodes.



* Objective 1 - 91% at 100th episode
![Objective 1: 91% at 100th episode](https://github.com/rwbot/RoboND-DeepRL-Project/blob/master/media/arm_91%25100_wm.gif?raw=true)

* Objective 2: 81% at 100th episode
![Objective 2: 81% at 100th episode](https://github.com/rwbot/RoboND-DeepRL-Project/blob/master/media/grip_81%25100_wm.gif?raw=true)


## Future Work
<!--- Briefly discuss how you can improve your current results. Student should discuss on what approaches they could take to improve their results. <!--- -->

With enough GPU time, the performance of the DQN agent could be improved by employing a more extensive approach to tuning. Each set of parameters could be run multiple times to decrease the effect of varying initial episodes.


## Building from Source (Nvidia Jetson TX2)

Run the following commands from terminal to build the project from source:

``` bash
$ sudo apt-get install cmake
$ git clone http://github.com/udacity/RoboND-DeepRL-Project
$ cd RoboND-DeepRL-Project
$ git submodule update --init
$ mkdir build
$ cd build
$ cmake ../
$ make
```

During the `cmake` step, Torch will be installed so it can take awhile. It will download packages and ask you for your `sudo` password during the install.

## Testing the API

To make sure that the reinforcement learners are still functioning properly from C++, a simple example of using the API called [`catch`](samples/catch/catch.cpp) is provided.  Similar in concept to pong, a ball drops from the top of the screen which the agent must catch before the ball reaches the bottom of the screen, by moving it's paddle left or right.

To test the textual [`catch`](samples/catch/catch.cpp) sample, run the following executable from the terminal.  After around 100 episodes or so, the agent should start winning the episodes nearly 100% of the time:  

``` bash
$ cd RoboND-DeepRL-Project/build/aarch64/bin
$ ./catch
[deepRL]  input_width:    64
[deepRL]  input_height:   64
[deepRL]  input_channels: 1
[deepRL]  num_actions:    3
[deepRL]  optimizer:      RMSprop
[deepRL]  learning rate:  0.01
[deepRL]  replay_memory:  10000
[deepRL]  batch_size:     32
[deepRL]  gamma:          0.9
[deepRL]  epsilon_start:  0.9
[deepRL]  epsilon_end:    0.05
[deepRL]  epsilon_decay:  200.0
[deepRL]  allow_random:   1
[deepRL]  debug_mode:     0
[deepRL]  creating DQN model instance
[deepRL]  DQN model instance created
[deepRL]  DQN script done init
[cuda]  cudaAllocMapped 16384 bytes, CPU 0x1020a800000 GPU 0x1020a800000
[deepRL]  pyTorch THCState  0x0318D490
[deepRL]  nn.Conv2d() output size = 800
WON! episode 1
001 for 001  (1.0000)  
WON! episode 5
004 for 005  (0.8000)  
WON! episode 10
007 for 010  (0.7000)  
WON! episode 15
010 for 015  (0.6667)  
WON! episode 20
013 for 020  (0.6500)  13 of last 20  (0.65)  (max=0.65)
WON! episode 25
015 for 025  (0.6000)  11 of last 20  (0.55)  (max=0.65)
LOST episode 30
018 for 030  (0.6000)  11 of last 20  (0.55)  (max=0.65)
LOST episode 35
019 for 035  (0.5429)  09 of last 20  (0.45)  (max=0.65)


Internally, [`catch`](samples/catch/catch.cpp) is using the [`dqnAgent`](c/dqnAgent.h) API from our C++ library to implement the learning.


## Project Environment Setup

To get started with the project environment, run the following:

``` bash
$ cd RoboND-DeepRL-Project/build/aarch64/bin
$ chmod u+x gazebo-arm.sh
$ ./gazebo-arm.sh
```

<img src="https://github.com/dusty-nv/jetson-reinforcement/raw/master/docs/images/gazebo_arm.jpg">

The plugins which hook the learning into the simulation are located in the `gazebo/` directory of the repo. The RL agent and the reward functions are to be defined in [`ArmPlugin.cpp`](gazebo/ArmPlugin.cpp).
