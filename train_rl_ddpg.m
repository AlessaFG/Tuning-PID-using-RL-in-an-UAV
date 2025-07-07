env = UAV_PID_Env;
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

% === Rete Attore ===
actorLayerSizes = [32 32];
actorNet = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization','none', 'Name','actor_input')
    fullyConnectedLayer(actorLayerSizes(1), 'Name','actor_fc1')
    reluLayer('Name','actor_relu1')
    fullyConnectedLayer(actorLayerSizes(2), 'Name','actor_fc2')
    reluLayer('Name','actor_relu2')
    fullyConnectedLayer(actInfo.Dimension(1), 'Name','actor_fc3')
    tanhLayer('Name','actor_tanh')
    scalingLayer('Scale', actInfo.UpperLimit, 'Name','actor_scaling')
];

actorOpts = rlRepresentationOptions('LearnRate',1e-5);
actor = rlDeterministicActorRepresentation(actorNet, obsInfo, actInfo, ...
    'Observation', {'actor_input'}, 'Action', {'actor_scaling'}, actorOpts);

% === Rete Critico ===
statePath = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization','none', 'Name','critic_state_input')
    fullyConnectedLayer(64, 'Name','critic_state_fc')
    reluLayer('Name','critic_state_relu')
];

actionPath = [
    featureInputLayer(actInfo.Dimension(1), 'Normalization','none', 'Name','critic_action_input')
    fullyConnectedLayer(64, 'Name','critic_action_fc')
];

commonPath = [
    additionLayer(2, 'Name','critic_add')
    reluLayer('Name','critic_common_relu1')
    fullyConnectedLayer(64, 'Name','critic_common_fc')
    reluLayer('Name','critic_common_relu2')
    fullyConnectedLayer(1, 'Name','critic_output')
];

criticNet = layerGraph();
criticNet = addLayers(criticNet, statePath);
criticNet = addLayers(criticNet, actionPath);
criticNet = addLayers(criticNet, commonPath);

% Collegamenti chiave
criticNet = connectLayers(criticNet, 'critic_state_relu', 'critic_add/in1');
criticNet = connectLayers(criticNet, 'critic_action_fc', 'critic_add/in2');

criticOpts = rlRepresentationOptions('LearnRate',1e-3);
critic = rlQValueRepresentation(criticNet, obsInfo, actInfo, ...
    'Observation', {'critic_state_input'}, 'Action', {'critic_action_input'}, criticOpts);

noise = rl.option.OrnsteinUhlenbeckActionNoise(...
    'Mean', 0, ...
    'StandardDeviation', 0.3, ...
    'StandardDeviationDecayRate', 1);

% === Agente DDPG ===
agentOpts = rlDDPGAgentOptions(...
    'SampleTime', env.Ts, ...
    'ExperienceBufferLength', 1e6, ...
    'MiniBatchSize', 256, ...
    'TargetSmoothFactor', 1e-3, ...
    'DiscountFactor', 0.99, ...
    'NoiseOptions', noise);


agent = rlDDPGAgent(actor, critic, agentOpts);

% === Addestramento ===
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 100, ...
    'MaxStepsPerEpisode', env.MaxSteps, ...
    'Verbose', true, ...
    'Plots', 'training-progress', ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', -0.1 ...
);

trainingStats = train(agent, env, trainOpts);
