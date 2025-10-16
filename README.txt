This project contains the MATLAB code and Simulink model for adaptive control of an UAV using Reinforcement Learning (RL) to find the optimal gains of the system.

1. Step 1: Open and run create_env.
Run the first part of the script to create the environment for Reinforcement Learning (RL) using the Simulink model.

2. Step 2: MATLAB RL toolbox
In the MATLAB toolboxes, open the Reinforcement Learning Toolbox and import the environment created in the previous step.
Then click on New Agent and select the desired type of agent.
In the next window, you can modify the agent’s parameters as well as advanced settings.
Once everything is configured, click Train to start the training process.
The training process may take some time to complete.

3. Step 3:Open and run second part of create_env.
Save the training session and, if desired, simulate it.
Then run the second part of the create_env script to complete the process.


In the Simulink file, you will find the UAV model divided into several subsystems.
In the Reward Function block, you can modify the range of the function.
In the Weight Calculation section, you can adjust the weights applied to the error norms.
In the New Gains subsystem, it is possible to change the value of the parameter η (eta).
There are several scopes available to monitor the system’s performance.
If you want to implement RL tuning for the UAV’s attitude control as well, you can copy and paste the existing RL subsystem, making sure to modify the Reward Function and other related parameters accordingly.