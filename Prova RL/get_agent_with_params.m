

reinforcementLearningDesigner('ReinforcementLearningDesignerSession.mat')

%%
actor = getActor(agent1_Trained)

parameters = getLearnableParameters(actor);

gains = parameters{5}'
% params = getLearnableParameters(getActor(agent1_Trained));
% gains = params{5}'

% Kp_pos_x = (gains(:,1))*200
% Kp_pos_y = (gains(:,2))*200
% Kp_pos_z = (gains(:,3))*200
% 
% Kd_pos_x = (gains(:,4))*50
% Kd_pos_y = (gains(:,5))*50
% Kd_pos_z = (gains(:,6))*50

Kp_mean = abs([mean(Kp_pos_x),mean(Kp_pos_y),mean(Kp_pos_z)])

Kd_mean = abs([mean(Kd_pos_x),mean(Kd_pos_y),mean(Kd_pos_z)])