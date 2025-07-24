function [NextObs, Reward, IsDone, LoggedSignals] = myStepFunction(Action, LoggedSignals)

    % === Parametri fisici ===
    m = 1.5; g = 9.81;
    Ixx = 0.01; Iyy = 0.01; Izz = 0.02; L = 0.3;
    Kp_ang = diag([150 150 100]);
    Kd_ang = diag([50 50 30]);

    % === Stato attuale ===
    x = LoggedSignals.State;
    t = LoggedSignals.Time;

    % === Azione dell’agente: tuning PID z ===
    Kp = diag([Action(1), Action(3), Action(5)]);
    Kd = diag([Action(2), Action(4), Action(6)]);

    % === Integrazione dinamica ===
    dt = 0.1;
    [~, Xout] = ode45(@(t,x) quadrotor_ode(t,x,m,g,Ixx,Iyy,Izz,L,Kp,Kd,Kp_ang,Kd_ang), [0 dt], x);
    x_next = Xout(end,:)';

    % === Traiettoria di riferimento ===
    t_new = t + dt;
    if t_new < 5
        ref = [2*t_new/5; 0; 2];
    elseif t_new < 10
        ref = [2; 2*(t_new-5)/5; 2];
    else
        ref = [2 - 2*(t_new-10)/5; 2; 2];
    end

  sigma_pos = 0.5;
  sigma_vel = 1.0;
    w1 = 0.8;
    w2 = 0.2;

    err = x_next(1:3) - ref;
    vel = x_next(4:6);
    ang = x_next(7:9);
    ang_vel = x_next(10:12);
    r_pos = exp(- (norm(err)/sigma_pos)^2);
    r_vel = exp(- (norm(vel)/sigma_vel)^2);
    r_ang = exp(- norm(ang)^2 / 0.25);
    r_ang_vel = exp(- norm(ang_vel)^2 / 1.0);

    penalty_action = 0.001 * sum(Action.^2);
    penalty_vel = 0.01 * norm(vel);

    Reward = w1 * r_pos + w2 * r_vel + 0.05*r_ang + 0.05*r_ang_vel - penalty_action - penalty_vel;
    eror_norm = [norm(err)]
    % === Fine episodio? ===
    max_err = 5;
    max_altitude = 10;
    IsDone = norm(err) > max_err || abs(x_next(3)) > max_altitude || any(isnan(x_next));

    % === Aggiorna log ===
    LoggedSignals.State = x_next;
    LoggedSignals.Time = t_new;

    % === Normalizzazione osservazione ===
    max_pos = 10;
    max_vel = 5;
    NextObs = [x_next(1:3)/max_pos;
               x_next(4:6)/max_vel;
               x_next(7:9);        % angoli
               x_next(10:12)];     % velocità angolari

    persistent traj_real traj_des

if isempty(traj_real)
    figure(2); clf
    traj_real = animatedline('Color', 'b', 'LineWidth', 2); hold on
    traj_des = animatedline('Color', 'r', 'LineStyle', '--');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    legend('Reale', 'Desiderata');
    title('Tracking Traiettoria');
    axis equal; grid on; view(3);
end

% Stato reale
pos = LoggedSignals.State(1:3);
% Calcolo posizione desiderata (come nel tuo step function)
t = LoggedSignals.StepCount * Ts;
if t < 5
    des_pos = [2*t/5; 0; 2];
elseif t < 10
    des_pos = [2; 2*(t-5)/5; 2];
else
    des_pos = [2 - 2*(t-10)/5; 2; 2];
end

addpoints(traj_real, pos(1), pos(2), pos(3));
addpoints(traj_des, des_pos(1), des_pos(2), des_pos(3));
drawnow limitrate

end

%%

% %% Prova %%
% function [NextObs, Reward, IsDone, LoggedSignals] = myStepFunction(Action, LoggedSignals)
% 
%     % === Parametri fisici ===
%     m = 1.5; g = 9.81;
%     Ixx = 0.01; Iyy = 0.01; Izz = 0.02; L = 0.3;
%     Kp_ang = diag([150 150 100]);
%     Kd_ang = diag([50 50 30]);
% 
%     % === Stato attuale ===
%     x = LoggedSignals.State;
%     t = LoggedSignals.Time;
% 
%     % === Azione dell’agente: tuning PID z ===
%     % Azione normalizzata ∈ [0, 1], riscalata a range utile
%     Kp_vec = rescale(Action(1:2:end), 0, 200);
%     Kd_vec = rescale(Action(2:2:end), 0, 50);
%     Kp = diag(Kp_vec);
%     Kd = diag(Kd_vec);
% 
%     % === Integrazione dinamica ===
%     dt = 0.1;
%     [~, Xout] = ode45(@(t,x) quadrotor_ode(t,x,m,g,Ixx,Iyy,Izz,L,Kp,Kd,Kp_ang,Kd_ang), [0 dt], x);
%     x_next = Xout(end,:)';
% 
%     % === Traiettoria di riferimento ===
%     t_new = t + dt;
%     if t_new < 5
%         ref = [2*t_new/5; 0; 2];
%     elseif t_new < 10
%         ref = [2; 2*(t_new-5)/5; 2];
%     else
%         ref = [2 - 2*(t_new-10)/5; 2; 2];
%     end
% 
%     % === Calcolo reward ===
%     sigma_pos = 0.5;
%     sigma_vel = 1.0;
%     w1 = 0.7;
%     w2 = 0.3;
% 
%     err = x_next(1:3) - ref;
%     vel = x_next(4:6);
%     ang = x_next(7:9);
%     ang_vel = x_next(10:12);
% 
%     r_pos = exp(- (norm(err)/sigma_pos)^2);
%     r_vel = exp(- (norm(vel)/sigma_vel)^2);
%     r_ang = exp(- norm(ang)^2 / 0.25);
%     r_ang_vel = exp(- norm(ang_vel)^2 / 1.0);
% 
%     penalty_action = 0.001 * sum(Action.^2);
%     penalty_vel = 0.01 * norm(vel);
% 
%     Reward = w1 * r_pos + w2 * r_vel + 0.05*r_ang + 0.05*r_ang_vel - penalty_action - penalty_vel;
% 
%     % === Fine episodio? ===
%     max_err = 5;
%     max_altitude = 10;
%     IsDone = norm(err) > max_err || abs(x_next(3)) > max_altitude || any(isnan(x_next));
% 
%     % === Aggiorna log ===
%     LoggedSignals.State = x_next;
%     LoggedSignals.Time = t_new;
% 
%     % === Normalizzazione osservazione ===
%     max_pos = 10;
%     max_vel = 5;
%     NextObs = [x_next(1:3)/max_pos;
%                x_next(4:6)/max_vel;
%                x_next(7:9);        % angoli
%                x_next(10:12)];     % velocità angolari
% end
% 
% %%Prova 2 %%
