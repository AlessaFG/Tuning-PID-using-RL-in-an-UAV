function [NextObs, Reward, IsDone, LoggedSignals] = quadrotorStepFunction(Action, LoggedSignals)
% Step function per tuning PID di un quadrotor via RL
% Action = [Kp_pos, Kd_pos, Kp_ang, Kd_ang]

%% === Parametri costanti dell'ambiente ===
m = 1.0; g = 9.81;
Ixx = 0.01; Iyy = 0.01; Izz = 0.02;
L = 0.25;           % lunghezza del braccio
Ts = 0.1;           % passo temporale
Tfinal = Ts;
MaxSteps = 300;

%% === Estrazione guadagni PID dall'azione ===
Kp_pos = diag(Action(1:3));  % Kp per posizione (x, y, z)
Kd_pos = diag(Action(4:6));  % Kd per velocità
Kp_ang = diag(Action(7:9));  % Kp per angoli (phi, theta, psi)
Kd_ang = diag(Action(10:12));% Kd per velocità angolari

%% === Stato corrente ===
x0 = LoggedSignals.State;    % stato del sistema 12x1
stepCount = LoggedSignals.StepCount;

%% === Simulazione dinamica ===
ode = @(t, x) quadrotor_ode(t, x, m, g, Ixx, Iyy, Izz, L, ...
                            Kp_pos, Kd_pos, Kp_ang, Kd_ang);
[~, x_out] = ode45(ode, [0 Ts], x0);  % simulazione da t a t+Ts
x_next = x_out(end,:)';

%% === Reward ===
% L'obiettivo è seguire una traiettoria desiderata (x_ref, y_ref, z_ref)
t_global = stepCount * Ts;
if t_global < 5
    des_pos = [2*t_global/5; 0; 2];
elseif t_global < 10
    des_pos = [2; 2*(t_global-5)/5; 2];
else
    des_pos = [2 - 2*(t_global-10)/5; 2; 2];
end

pos = x_next(1:3);
vel = x_next(4:6);
ang = x_next(7:9);
omega = x_next(10:12);

% penalizza errori di posizione e orientamento
e_pos = des_pos - pos;
norm_e_pos = norm(e_pos);
reward_pos = exp(-norm_e_pos^2);    % tra 0 (errore grande) e 1 (errore nullo)
reward_ang = exp(-norm(ang)^2);      % idem

Reward = reward_pos + 0.1 * reward_ang

if norm_e_pos < 0.05
    Reward = Reward + 50;
elseif norm_e_pos < 0.1
    Reward = Reward + 20;
elseif norm_e_pos < 0.2
    Reward = Reward + 5;
end

%% === Aggiornamento LoggedSignals ===
LoggedSignals.State = x_next;
LoggedSignals.StepCount = stepCount + 1;
MaxSteps = LoggedSignals.MaxSteps;
%% === Osservazione ===
NextObs = x_next;  % puoi trasformare in osservazione ridotta se vuoi

%% === Termina se troppo lontano o passo massimo superato ===
IsDone = norm(e_pos) > 5 || stepCount >= MaxSteps;

end
