clc; clear; close all;

%% === PARAMETRI UAV ===
params.m = 1.5;
params.g = 9.81;
params.Ixx = 0.01;
params.Iyy = 0.01;
params.Izz = 0.02;
params.L = 0.3;  % lunghezza bracci

% PID gains
params.Kp = diag([4, 4, 10]);
params.Kd = diag([3, 3, 6]);
params.Kp_ang = diag([250, 250, 150]);
params.Kd_ang = diag([80, 80, 50]);

%% === STATO INIZIALE [x y z vx vy vz phi theta psi p q r] ===
x0 = zeros(12,1);

%% === SIMULAZIONE ===
tspan = [0 15];
[t, X] = ode45(@(t,x) quadrotor_ode(t,x,params), tspan, x0);

x_pos = X(:,1); y_pos = X(:,2); z_pos = X(:,3);


%% === ANIMAZIONE DRONE 3D ===
figure('Color','w');
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
view([-40 20]);
title('UAV con controllo stabilizzato');
hold on;


arm1 = plot3([0 0], [0 0], [0 0], 'r', 'LineWidth', 3);
arm2 = plot3([0 0], [0 0], [0 0], 'b', 'LineWidth', 3);
center = plot3(0,0,0,'ko','MarkerSize',6,'MarkerFaceColor','k');
traj = plot3(x_pos, y_pos, z_pos, 'k--');

% Raggio eliche
prop_radius = 0.1;
theta_circle = linspace(0, 2*pi, 30);
xc = prop_radius * cos(theta_circle);
yc = prop_radius * sin(theta_circle);

% Posizioni relative eliche (bracci)
prop_pos_body = [ params.L, 0, 0;
                 -params.L, 0, 0;
                  0, params.L, 0;
                  0,-params.L, 0 ];

% Preallocazione per handle grafici
h_prop = gobjects(4,1);

for i = 1:4
    Xc = xc + prop_pos_body(i,1);
    Yc = yc + prop_pos_body(i,2);
    Zc = zeros(size(xc)) + prop_pos_body(i,3);
    h_prop(i) = fill3(Xc, Yc, Zc, 'c', 'FaceAlpha', 0.5);
end
%% === CICLO DI ANIMAZIONE ===
for k = 1:10:length(t)-1
    pos = X(k,1:3)';
    phi = X(k,7); theta = X(k,8); psi = X(k,9);
    R = eul2rotm([psi, theta, phi]);
    L = params.L;

    bx = L * R * [1; 0; 0];
    by = L * R * [0; 1; 0];

    set(arm1, 'XData', [pos(1)-bx(1), pos(1)+bx(1)], ...
              'YData', [pos(2)-bx(2), pos(2)+bx(2)], ...
              'ZData', [pos(3)-bx(3), pos(3)+bx(3)]);

    set(arm2, 'XData', [pos(1)-by(1), pos(1)+by(1)], ...
              'YData', [pos(2)-by(2), pos(2)+by(2)], ...
              'ZData', [pos(3)-by(3), pos(3)+by(3)]);

    set(center, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
    
    % ===  Aggiorna posizione delle eliche === %
  for i = 1:4
    prop_body = prop_pos_body(i, :)';
    prop_world_center = pos + R * prop_body;

    % Corretto: tutti vettori colonna (30x1)
    Xc = xc';      
    Yc = yc';      
    Zc = zeros(size(Xc));  % stesse dimensioni di Xc e Yc

    circle_points = [Xc, Yc, Zc]';  % 3x30 matrice di punti

    circle_points_rot = R * circle_points;

    X_plot = prop_world_center(1) + circle_points_rot(1, :);
    Y_plot = prop_world_center(2) + circle_points_rot(2, :);
    Z_plot = prop_world_center(3) + circle_points_rot(3, :);

    set(h_prop(i), 'XData', X_plot, 'YData', Y_plot, 'ZData', Z_plot);
end

    

    % visuale dinamica centrata sul drone
    xlim([pos(1)-2 pos(1)+2]);
    ylim([pos(2)-2 pos(2)+2]);
    zlim([pos(3)-1 pos(3)+3]);

    drawnow;
    pause(t(k+1)-t(k));
end


%% === FUNZIONI ===

% function dx = quadrotor_ode(t, x, p)
%     % === Estrai stato ===
%     pos = x(1:3); vel = x(4:6);
%     phi = x(7); theta = x(8); psi = x(9);
%     omega = x(10:12); % [p q r]
% 
%     % === Obiettivo posizione dinamico ===
%     des_pos = [1.5; 1.5; 2 + 1.5*sin(0.3*t)];
%     des_vel = [0; 0; 0];
% 
%     % === Controllore di posizione (PID semplificato) ===
%     e_pos = des_pos - pos;
%     e_vel = des_vel - vel;
%     acc_des = p.Kp * e_pos + p.Kd * e_vel;
%     Fz = p.m * (acc_des(3) + p.g);
% 
%     % === Orientamento desiderato ===
%     phi_des = (1/p.g)*(acc_des(1)*sin(psi) - acc_des(2)*cos(psi));
%     theta_des = (1/p.g)*(acc_des(1)*cos(psi) + acc_des(2)*sin(psi));
%     psi_des = 0;
% 
%     ang_des = [phi_des; theta_des; psi_des];
%     ang = [phi; theta; psi];
%     e_ang = ang_des - ang;
%     e_omega = -omega;
% 
%     % === Controllore angolare PID ===
%     tau = p.Kp_ang * e_ang + p.Kd_ang * e_omega;
% 
%     % === Dinamica ===
%     R = eul2rotm([psi, theta, phi]);
%     acc = (1/p.m)*(R * [0; 0; Fz]) - [0; 0; p.g];
% 
%     omega_dot = [ (tau(1) - (p.Iyy - p.Izz)*omega(2)*omega(3)) / p.Ixx;
%                   (tau(2) - (p.Izz - p.Ixx)*omega(1)*omega(3)) / p.Iyy;
%                   (tau(3) - (p.Ixx - p.Iyy)*omega(1)*omega(2)) / p.Izz ];
% 
%     % === Equazioni cinematiche ===
%     phi_dot = omega(1) + tan(theta)*(omega(2)*sin(phi) + omega(3)*cos(phi));
%     theta_dot = omega(2)*cos(phi) - omega(3)*sin(phi);
%     psi_dot = (omega(2)*sin(phi) + omega(3)*cos(phi))/cos(theta);
% 
%     % === Stato derivato ===
%     dx = zeros(12,1);
%     dx(1:3) = vel;
%     dx(4:6) = acc;
%     dx(7) = phi_dot;
%     dx(8) = theta_dot;
%     dx(9) = psi_dot;
%     dx(10:12) = omega_dot;
% end

function dx = quadrotor_ode(t, x, p)
    % === Estrai stato ===
    pos = x(1:3); vel = x(4:6);
    phi = x(7); theta = x(8); psi = x(9);
    omega = x(10:12); % [p q r]

    % === Nuova traiettoria figura 8 ===
    a = 1.5; % ampiezza asse x
    b = 1;   % ampiezza asse y
    des_pos = [a * sin(t);
               b * sin(2*t);
               2];  % quota costante 2 m

    des_vel = [a * cos(t);
               2*b * cos(2*t);
               0];

    % === Controllore di posizione (PID semplificato) ===
    e_pos = des_pos - pos;
    e_vel = des_vel - vel;
    acc_des = p.Kp * e_pos + p.Kd * e_vel;
    Fz = p.m * (acc_des(3) + p.g);

    % === Orientamento desiderato ===
    phi_des = (1/p.g)*(acc_des(1)*sin(psi) - acc_des(2)*cos(psi));
    theta_des = (1/p.g)*(acc_des(1)*cos(psi) + acc_des(2)*sin(psi));
    psi_des = 0;

    ang_des = [phi_des; theta_des; psi_des];
    ang = [phi; theta; psi];
    e_ang = ang_des - ang;
    e_omega = -omega;

    % === Controllore angolare PID ===
    tau = p.Kp_ang * e_ang + p.Kd_ang * e_omega;

    % === Dinamica ===
    R = eul2rotm([psi, theta, phi]);
    acc = (1/p.m)*(R * [0; 0; Fz]) - [0; 0; p.g];

    omega_dot = [ (tau(1) - (p.Iyy - p.Izz)*omega(2)*omega(3)) / p.Ixx;
                  (tau(2) - (p.Izz - p.Ixx)*omega(1)*omega(3)) / p.Iyy;
                  (tau(3) - (p.Ixx - p.Iyy)*omega(1)*omega(2)) / p.Izz ];

    % === Equazioni cinematiche ===
    phi_dot = omega(1) + tan(theta)*(omega(2)*sin(phi) + omega(3)*cos(phi));
    theta_dot = omega(2)*cos(phi) - omega(3)*sin(phi);
    psi_dot = (omega(2)*sin(phi) + omega(3)*cos(phi))/cos(theta);

    % === Stato derivato ===
    dx = zeros(12,1);
    dx(1:3) = vel;
    dx(4:6) = acc;
    dx(7) = phi_dot;
    dx(8) = theta_dot;
    dx(9) = psi_dot;
    dx(10:12) = omega_dot;
end

% function dx = quadrotor_ode(t, x, p)
%     % === Estrai stato ===
%     pos = x(1:3); vel = x(4:6);
%     phi = x(7); theta = x(8); psi = x(9);
%     omega = x(10:12); % [p q r]
% 
%     % === Parametri traiettoria quadrata ===
%     side = 2;          % lunghezza lato quadrato [m]
%     T_side = 5;        % durata per lato [s]
%     total_time = 4 * T_side; % tempo totale per fare il giro completo
% 
%     % Tempo modulo per ciclo continuo
%     t_mod = mod(t, total_time);
% 
%     % Definisci posizioni vertici quadrato
%     p1 = [0; 0; 2];
%     p2 = [side; 0; 2];
%     p3 = [side; side; 2];
%     p4 = [0; side; 2];
% 
%     % Calcola posizione desiderata e velocità in base alla fase
%     if t_mod < T_side
%         % lato 1: da p1 a p2
%         alpha = t_mod / T_side;
%         des_pos = p1 + alpha * (p2 - p1);
%         des_vel = (p2 - p1) / T_side;
%     elseif t_mod < 2*T_side
%         % lato 2: da p2 a p3
%         alpha = (t_mod - T_side) / T_side;
%         des_pos = p2 + alpha * (p3 - p2);
%         des_vel = (p3 - p2) / T_side;
%     elseif t_mod < 3*T_side
%         % lato 3: da p3 a p4
%         alpha = (t_mod - 2*T_side) / T_side;
%         des_pos = p3 + alpha * (p4 - p3);
%         des_vel = (p4 - p3) / T_side;
%     else
%         % lato 4: da p4 a p1
%         alpha = (t_mod - 3*T_side) / T_side;
%         des_pos = p4 + alpha * (p1 - p4);
%         des_vel = (p1 - p4) / T_side;
%     end
% 
%     % === Controllore di posizione (PID semplificato) ===
%     e_pos = des_pos - pos;
%     e_vel = des_vel - vel;
%     acc_des = p.Kp * e_pos + p.Kd * e_vel;
%     Fz = p.m * (acc_des(3) + p.g);
% 
%     % === Orientamento desiderato ===
%     phi_des = (1/p.g)*(acc_des(1)*sin(psi) - acc_des(2)*cos(psi));
%     theta_des = (1/p.g)*(acc_des(1)*cos(psi) + acc_des(2)*sin(psi));
%     psi_des = 0;
% 
%     ang_des = [phi_des; theta_des; psi_des];
%     ang = [phi; theta; psi];
%     e_ang = ang_des - ang;
%     e_omega = -omega;
% 
%     % === Controllore angolare PID ===
%     tau = p.Kp_ang * e_ang + p.Kd_ang * e_omega;
% 
%     % === Dinamica ===
%     R = eul2rotm([psi, theta, phi]);
%     acc = (1/p.m)*(R * [0; 0; Fz]) - [0; 0; p.g];
% 
%     omega_dot = [ (tau(1) - (p.Iyy - p.Izz)*omega(2)*omega(3)) / p.Ixx;
%                   (tau(2) - (p.Izz - p.Ixx)*omega(1)*omega(3)) / p.Iyy;
%                   (tau(3) - (p.Ixx - p.Iyy)*omega(1)*omega(2)) / p.Izz ];
% 
%     % === Equazioni cinematiche ===
%     phi_dot = omega(1) + tan(theta)*(omega(2)*sin(phi) + omega(3)*cos(phi));
%     theta_dot = omega(2)*cos(phi) - omega(3)*sin(phi);
%     psi_dot = (omega(2)*sin(phi) + omega(3)*cos(phi))/cos(theta);
% 
%     % === Stato derivato ===
%     dx = zeros(12,1);
%     dx(1:3) = vel;
%     dx(4:6) = acc;
%     dx(7) = phi_dot;
%     dx(8) = theta_dot;
%     dx(9) = psi_dot;
%     dx(10:12) = omega_dot;
% end


function R = eul2rotm(eul)
    psi = eul(1); theta = eul(2); phi = eul(3);
    Rz = [cos(psi) -sin(psi) 0;
          sin(psi)  cos(psi) 0;
               0         0   1];
    Ry = [cos(theta) 0 sin(theta);
          0          1     0;
         -sin(theta) 0 cos(theta)];
    Rx = [1    0           0;
          0 cos(phi) -sin(phi);
          0 sin(phi)  cos(phi)];
    R = Rz * Ry * Rx;
end
