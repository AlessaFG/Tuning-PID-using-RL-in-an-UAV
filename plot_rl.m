%% 
% Define the desired position trajectory (3xN matrix)
% Each row corresponds to X, Y, Z components respectively.
pos_d = [RL_tout'; sin(0.5*RL_tout)'; 2 + 0.5*cos(0.5*RL_tout)'];  

% Transpose pos_d to have Nx3 format — each column corresponds to one axis (X, Y, Z)
pos_d = pos_d';

% Define the desired velocity trajectory (3xN matrix)
vel_d = [ ones(size(RL_tout))'; ...
          0.5*cos(0.5*RL_tout)'; ...
         -0.25*sin(0.5*RL_tout)' ];
vel_d = vel_d';

%% Plot position tracking performance
figure(6)
for i=1:3
    subplot(3,1,i)
    % Plot simulated and desired positions over time
    plot(RL_tout, RL_Tuning_Position(:,i),  'LineWidth', 1.5)
    hold on
    plot(RL_tout, pos_d(:,i), '-.', 'LineWidth', 1.5)
    hold off
    grid on
    
    % Label each subplot according to axis
    if i == 1
        ylabel('X [m]')
        title('Position tracking Reinforcement Learning')
    elseif i == 2
        ylabel('Y [m]')
    else
        ylabel('Z [m]')
        xlabel('Time [s]')
    end
    
    % Add legend
    legend('Simulated Position','Desired Position')
end

%% Plot velocity tracking performance
figure(7)
for i=1:3
    subplot(3,1,i)
    % Plot simulated and desired velocities
    plot(RL_tout, RL_Tuning_Velocity(:,i), 'LineWidth', 1.2)
    hold on
    plot(RL_tout, vel_d(:,i), '-.', 'LineWidth', 1.2)
    hold off
    grid on
    
    % Label each subplot according to axis
    if i == 1
        ylabel('X [m/s]')
        title('Velocity tracking Reinforcement Learning')
    elseif i == 2
        ylabel('Y [m/s]')
    else
        ylabel('Z [m/s]')
        xlabel('Time [s]')
    end
    
    % Add legend
    legend('Simulated Velocity','Desired Velocity')
end

%% Plot position tracking error
figure(8)
for i=1:3
    subplot(3,1,i)
    % Plot position error (desired - simulated)
    plot(RL_tout, pos_d(:,i)-RL_Tuning_Position(:,i),  'LineWidth', 1.2)
    hold on
    yline(0,'k--','LineWidth',1.2);  % Reference zero line
    hold off
    grid on
    
    % Label subplots
    if i == 1
        ylabel('X [m]')
        title('Error Position Tracking Reinforcement Learning')
    elseif i == 2
        ylabel('Y [m]')
    else
        ylabel('Z [m]')
        xlabel('Time [s]')
    end
end

%% Plot velocity tracking error
figure(9)
for i=1:3
    subplot(3,1,i)
    % Plot velocity error (desired - simulated)
    plot(RL_tout, vel_d(:,i)-RL_Tuning_Velocity(:,i),  'LineWidth', 1.2)
    hold on
    yline(0,'k--','LineWidth',1.2);  % Reference zero line
    hold off
    grid on
    
    % Label subplots
    if i == 1
        ylabel('X [m/s]')
        title('Error Velocity Tracking Reinforcement Learning')
    elseif i == 2
        ylabel('Y [m/s]')
    else
        ylabel('Z [m/s]')
        xlabel('Time [s]')
    end
end

%% Plot angular error (orientation tracking)
Error_Ang = squeeze(RL_Tuning_Error_Ang)';   % Convert angular error data to 2D (Nx3)
figure(10)
% Plot angular errors for each axis
plot(RL_tout, Error_Ang(:,1),  'LineWidth', 1.2)
hold on
plot(RL_tout, Error_Ang(:,2),  'LineWidth', 1.2)
plot(RL_tout, Error_Ang(:,3),  'LineWidth', 1.2)
hold off
grid on
xlabel('Time [s]')
ylabel('Angular error [rad]')
title('Angular Error Classic PD')
legend('Error X','Error Y','Error Z')

%% Plot proportional gain evolution (K_P)
A = RL_Tuning_Gains.Data;   % Extract gain values
T = RL_Tuning_Gains.Time;   % Extract time vector

% Odd indices correspond to proportional gains (K_P)
odd_indices = 1:2:6;  % 1, 3, 5

figure;
for k = 1:length(odd_indices)
    i = odd_indices(k);
    subplot(length(odd_indices),1,k)
    plot(T, A(:,i), 'LineWidth', 1.2)
    grid on;

    % Label each subplot for K_P gains
    switch i
        case 1
            ylabel('K_P X')
            title('Evolution of the PD’s gains')
        case 3
            ylabel('K_P Y')
        case 5
            ylabel('K_P Z')
            xlabel('Time [s]')
    end
end

%% Plot derivative gain evolution (K_D)
A = RL_Tuning_Gains.Data;
T = RL_Tuning_Gains.Time;

% Even indices correspond to derivative gains (K_D)
even_indices = 2:2:6;  % 2, 4, 6

figure;
for k = 1:length(even_indices)
    i = even_indices(k);
    subplot(length(even_indices),1,k)
    plot(T, A(:,i), 'LineWidth', 1.2)
    grid on;

    % Label each subplot for K_D gains
    switch i
        case 2
            ylabel('K_D X')
            title('Evolution of the PD’s gains')
        case 4
            ylabel('K_D Y')
        case 6
            ylabel('K_D Z')
            xlabel('Time [s]')
    end
end

%% Plot all PD gains together
A = RL_Tuning_Gains.Data;
T = RL_Tuning_Gains.Time;
figure;
% Plot all six PD gains (K_P and K_D for each axis)
plot(T, A(:,1), 'LineWidth', 1.2)
hold on
plot(T, A(:,2), 'LineWidth', 1.2)
plot(T, A(:,3), 'LineWidth', 1.2)
plot(T, A(:,4), 'LineWidth', 1.2)
plot(T, A(:,5), 'LineWidth', 1.2)
plot(T, A(:,6), 'LineWidth', 1.2)
ylabel('Proportional & Derivative Gains')
xlabel('Time [s]')
legend('K_{P,new X}','K_{D,new X}','K_{P,new Y}','K_{D,new Y}','K_{P,new Z}','K_{D,new Z}')
