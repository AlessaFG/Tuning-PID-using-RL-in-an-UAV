% Observation space
obsInfo = rlNumericSpec([12 1]); % column vector
obsInfo.Name = 'observations';
obsInfo.Description = 'position, velocity, position error, velocity error';

% Action space
actInfo = rlNumericSpec([6 1], 'LowerLimit', -1, 'UpperLimit', 1);
actInfo.Name = 'actions';

% Create the reinforcement learning environment from the Simulink model
env = rlSimulinkEnv('rlDronePidTuning', 'rlDronePidTuning/RL Agent', obsInfo, actInfo);

%% My Agent
% Load the Session of Agent
session = load('ReinforcementLearningDesignerSessionDDPG.mat');
myAgent = session.RLDesignerSession.Data.Agents(2).Data;

