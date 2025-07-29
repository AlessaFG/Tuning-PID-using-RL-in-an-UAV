% function [InitialObservation, LoggedSignals] = myResetFunction()
%     x0 = zeros(12,1); % inizializza a zero
%     InitialObservation = x0;
% 
%     LoggedSignals.State = x0;
%     LoggedSignals.Time = 0;
% end


% function [InitialObservation, LoggedSignals] = myResetFunction()
% 
%     % === Stato iniziale randomizzato per robustezza ===
%     x0 = zeros(12,1);
%     x0(1:3) = 0.1 * randn(3,1);  % posizione iniziale ±10cm
%     x0(4:6) = 0.1 * randn(3,1);  % velocità iniziali
%     x0(7:9) = 0.05 * randn(3,1); % piccoli angoli iniziali
%     x0(10:12) = 0.05 * randn(3,1); % velocità angolari
% 
%     % === Normalizzazione osservazione iniziale ===
%     max_pos = 10;
%     max_vel = 5;
%     InitialObservation = [x0(1:3)/max_pos;
%                           x0(4:6)/max_vel;
%                           x0(7:9);
%                           x0(10:12)];
% 
%     LoggedSignals.State = x0;
%     LoggedSignals.Time = 0;
% end


%% File: myResetFunction.m
% function [InitialObservation, LoggedSignals] = myResetFunction()
%     % === Stato iniziale randomizzato ===
%     x0 = zeros(12,1);
%     x0(1:3)   = 0.1 * randn(3,1);
%     x0(4:6)   = 0.1 * randn(3,1);
%     x0(7:9)   = 0.05* randn(3,1);
%     x0(10:12) = 0.05* randn(3,1);
% 
%     % === Normalize initial obs in [-1,1] ===
%     max_pos = 10; max_vel = 5;
%     InitialObservation = [x0(1:3)/max_pos;
%                           x0(4:6)/max_vel;
%                           x0(7:9)/(pi);
%                           x0(10:12)/10];
% 
%     LoggedSignals.State            = x0;
%     LoggedSignals.Time             = 0;
%     LoggedSignals.CumulativeReward = 0;
%     LoggedSignals.MaxSteps         = 300;
%     LoggedSignals.StepCount = 0;
% end

%% SOLO GLI ERRORI DI POSIZIONE E VELOCITA'
function [InitialObservation, LoggedSignals] = myResetFunction()
    % === Stato iniziale randomizzato ===
    x0 = zeros(12,1);
    x0(1:3)   = 0.1 * randn(3,1);      % posizione iniziale
    x0(4:6)   = 0.1 * randn(3,1);      % velocità iniziale
    x0(7:9)   = 0.05 * randn(3,1);     % angoli (roll, pitch, yaw)
    x0(10:12) = 0.05 * randn(3,1);     % velocità angolari

    % === Traiettoria di riferimento al tempo iniziale ===
    t0 = 0;
    [des_pos, des_vel] = reference_trajectory(t0);

    % === Calcolo degli errori ===
    pos = x0(1:3);
    vel = x0(4:6);
    e_pos = des_pos - pos;
    e_vel = des_vel - vel;

    % === Osservazione iniziale: errori ===
    InitialObservation = [e_pos; e_vel];

    % === Logged signals ===
    LoggedSignals.State = x0;
    LoggedSignals.Time = 0;
    LoggedSignals.CumulativeReward = 0;
    LoggedSignals.MaxSteps = 300;
    LoggedSignals.StepCount = 0;
end
