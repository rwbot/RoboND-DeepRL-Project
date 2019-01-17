
# Deep Reinforcement Learning - Arm Manipulation

## Reward Functions
Explain the reward functions that you created: Brief explanation of each reward function and associated reward values. The writeup should also include what type of joint control was implemented.

## Setup
In the method `ArmPlugin::Load()`, subscriber nodes were defined to faciliate camera communication and collision detection through Gazebo:
```cpp
cameraSub = cameraNode->Subscribe("/gazebo/arm_world/camera/link/camera/image",&ArmPlugin::onCameraMsg, this);
collisionSub = collisionNode->Subscribe("/gazebo/arm_world/tube/tube_link/my_contact",&ArmPlugin::onCollisionMsg, this);
```

In the method `ArmPlugin::Create()`, the DQN Agent was created using the parameters defined at the top of **Armplugin.cpp**:
```cpp
agent = dqnAgent::Create(INPUT_WIDTH, INPUT_HEIGHT, INPUT_CHANNELS,
	DOF*2, OPTIMIZER, LEARNING_RATE, REPLAY_MEMORY, BATCH_SIZE, GAMMA,
	EPS_START, EPS_END, EPS_DECAY, USE_LSTM, LSTM_SIZE, ALLOW_RANDOM, DEBUG_DQN);
```

The output of the DQN agent is mapped to a particular actions of the robotic arm. The method `ArmPlugin::updateAgent()` is responsible for executing the action selected by the DQN agent. Two types of joint control were implemented: velocity control and position control. Both are dependent upon whether the issued action is an odd or even number.  Odd-numbered actions decrease the joint position/velocity, while even-numbered actions increase the joint position/velocity. Additionally, the current delta of the position/values is used:
```cpp
// Evaluating if action is even or odd
const int actionSign = 1 - 2 * (action % 2);
// Set joint position
float joint = ref[action/2] + actionSign * actionJointDelta;
// Set joint velocity
const float velocity = vel[action/2] + actionSign * actionVelDelta;
```

## Reward Function
Explain the reward functions that you created: Brief explanation of each reward function and associated reward values. The writeup should also include what type of joint control was implemented.

The reward function is a crucial component in guiding the DQN agent through the process of learning to manipulate the arm. After every end of episode (EOE), a particular reward is issued, depending on the triggering event.

#### `REWARD_LOSS` is issued  when the current episode exceeds 100 steps or when the arm makes contact with the ground.

The `REWARD_LOSS` issued when the arm makes contact with the ground is detected using the `GetBoundingBox()` function from the Gazebo API which provides the min/max XYZ values of the arm. Comparing this with the Z value of the ground, collisions can be detected.
```cpp
const bool checkGroundContact = (gripBBox.min.z <= groundContact || gripBBox.max.z <= groundContact);

if(checkGroundContact){
	rewardHistory = REWARD_LOSS;
	...
}
```

#### `REWARD_WIN` is issued when any part of the arm makes contact with the object (Objective 1) and when only the gripper makes contact with the object (Objective 2).
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
To reduce the necessary training time, an interim reward function was implemented which would help guide the arm towards the object. The reward is proportional to the change in distance between the arm and the object. If the arm is moving away from the object, the interim reward is negative. Conversely, if the arm is moving towards the object, the interim reward is positive. The raw values cause the arm to have a jerking movement, so they are smoothed by taking a moving average. Finally, due to agent's tendency to remain still, a time penalty was introduced to force the agent to finish as quickly as possible.
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
Specify the hyperparameters that you selected for each objective, and explain the reasoning behind the selection. Student should explain the choice of hyperparameters for both objectives.

Since the camera sent a 64 x 64 image, that value used as the width and height of the input.
```cpp
#define INPUT_WIDTH   64
#define INPUT_HEIGHT  64
#define OPTIMIZER "RMSprop"
#define LEARNING_RATE 0.1f
#define REPLAY_MEMORY 10000
#define BATCH_SIZE 64
#define USE_LSTM true
#define LSTM_SIZE 256
// DQN API
#define EPS_START 0.9f
#define EPS_END 0.01f
#define EPS_DECAY 200
```

## Results
Explain the results obtained for both objectives. Include discussion on the DQN agent's performance for both objectives. Include watermarked images, or videos of your results.
Student should describe and briefly explain the results they achieved for both objectives. The discussion should also include their comments on the DQN agent's performance and if there were any shortcomings. Student should include either watermarked images of their results, or attach a video that displays the results and the arm in action.

## Future Work
Briefly discuss how you can improve your current results. Student should discuss on what approaches they could take to improve their results.
