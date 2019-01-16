
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

## Reward Functions
Explain the reward functions that you created: Brief explanation of each reward function and associated reward values. The writeup should also include what type of joint control was implemented.

## Hyperparameters
Specify the hyperparameters that you selected for each objective, and explain the reasoning behind the selection. Student should explain the choice of hyperparameters for both objectives.

## Results
Explain the results obtained for both objectives. Include discussion on the DQN agent's performance for both objectives. Include watermarked images, or videos of your results.
Student should describe and briefly explain the results they achieved for both objectives. The discussion should also include their comments on the DQN agent's performance and if there were any shortcomings. Student should include either watermarked images of their results, or attach a video that displays the results and the arm in action.

## Future Work
Briefly discuss how you can improve your current results. Student should discuss on what approaches they could take to improve their results.





## KaTeX

You can render LaTeX mathematical expressions using [KaTeX](https://khan.github.io/KaTeX/): The *Gamma function* satisfying $\Gamma(n) = (n-1)!\quad\forall n\in\mathbb N$ is via the Euler integral

$$\Gamma(z) = \int_0^\infty t^{z-1}e^{-t}dt\,.$$
> You can find more information about **LaTeX** mathematical expressions [here](http://meta.math.stackexchange.com/questions/5020/mathjax-basic-tutorial-and-quick-reference).
>
