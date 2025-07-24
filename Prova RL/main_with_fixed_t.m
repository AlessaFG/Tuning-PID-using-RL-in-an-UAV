clc; clear; close all;

%% === PARAMETRI UAV ===
m = 1.5;
g = 9.81;
Ixx = 0.01;
Iyy = 0.01;
Izz = 0.02;
L = 0.3;

% PID gains
Kp = diag([4.5, 4.5, 11.5]);
Kd = diag([6, 7, 9]);
Kp_ang = diag([150, 150, 100]);
Kd_ang = diag([50, 50, 30]);

%% === STATO INIZIALE ===
x0 = zeros(12,1);

%% === SIMULAZIONE CON TEMPO FISSO ===
t_fixed = linspace(0, 15, 5000);
[t_raw, X_raw] = ode45(@(t,x) quadrotor_ode(t,x,m,g,Ixx,Iyy,Izz,L,Kp,Kd,Kp_ang,Kd_ang), [0 15], x0);
X = interp1(t_raw, X_raw, t_fixed);
t = t_fixed;

x_pos = X(:,1); y_pos = X(:,2); z_pos = X(:,3);

%% === TRAIETTORIA DI RIFERIMENTO ===
t_ref = linspace(0, 15, 000);
ref_traj = zeros(3, length(t_ref));
for i = 1:length(t_ref)
    ti = t_ref(i);
    if ti < 5
        ref_traj(:,i) = [2*ti/5; 0; 2];
    elseif ti < 10
        ref_traj(:,i) = [2; 2*(ti-5)/5; 2];
    else
        ref_traj(:,i) = [2 - 2*(ti-10)/5; 2; 2];
    end
end

%% === ANIMAZIONE 3D ===
figure('Color','w');
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
view([-40 20]);
title('UAV con traiettoria rettilinea');
hold on;

plot3(ref_traj(1,:), ref_traj(2,:), ref_traj(3,:), 'r--', 'LineWidth', 1.5);

arm1 = plot3([0 0], [0 0], [0 0], 'r', 'LineWidth', 3);
arm2 = plot3([0 0], [0 0], [0 0], 'b', 'LineWidth', 3);
center = plot3(0,0,0,'ko','MarkerSize',6,'MarkerFaceColor','k');
traj = plot3(x_pos, y_pos, z_pos, 'k--');

prop_radius = 0.1;
theta_circle = linspace(0, 2*pi, 30);
xc = prop_radius * cos(theta_circle);
yc = prop_radius * sin(theta_circle);
prop_pos_body = [ L, 0, 0;
                 -L, 0, 0;
                  0, L, 0;
                  0,-L, 0 ];
h_prop = gobjects(4,1);
for i = 1:4
    Xc = xc + prop_pos_body(i,1);
    Yc = yc + prop_pos_body(i,2);
    Zc = zeros(size(xc)) + prop_pos_body(i,3);
    h_prop(i) = fill3(Xc, Yc, Zc, 'c', 'FaceAlpha', 0.5);
end

%% === CICLO ANIMAZIONE ===
for k = 1:10:length(t)-1
    pos = X(k,1:3)';
    phi = X(k,7); theta = X(k,8); psi = X(k,9);
    R = eul2rotm([psi, theta, phi]);

    bx = L * R * [1; 0; 0];
    by = L * R * [0; 1; 0];

    set(arm1, 'XData', [pos(1)-bx(1), pos(1)+bx(1)], ...
              'YData', [pos(2)-bx(2), pos(2)+bx(2)], ...
              'ZData', [pos(3)-bx(3), pos(3)+bx(3)]);
    set(arm2, 'XData', [pos(1)-by(1), pos(1)+by(1)], ...
              'YData', [pos(2)-by(2), pos(2)+by(2)], ...
              'ZData', [pos(3)-by(3), pos(3)+by(3)]);
    set(center, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));

    for i = 1:4
        prop_body = prop_pos_body(i, :)';
        prop_world_center = pos + R * prop_body;
        circle_points = [xc; yc; zeros(size(xc))];
        circle_points_rot = R * circle_points;
        X_plot = prop_world_center(1) + circle_points_rot(1,:);
        Y_plot = prop_world_center(2) + circle_points_rot(2,:);
        Z_plot = prop_world_center(3) + circle_points_rot(3,:);
        set(h_prop(i), 'XData', X_plot, 'YData', Y_plot, 'ZData', Z_plot);
    end

    xlim([pos(1)-2 pos(1)+2]);
    ylim([pos(2)-2 pos(2)+2]);
    zlim([pos(3)-1 pos(3)+3]);

    drawnow;
    pause(t(k+1)-t(k));
end

%% === FUNZIONE DINAMICA UAV ===
function dx = quadrotor_ode(t, x, m, g, Ixx, Iyy, Izz, L, Kp, Kd, Kp_ang, Kd_ang)
    pos = x(1:3); vel = x(4:6);
    phi = x(7); theta = x(8); psi = x(9);
    omega = x(10:12);

    if t < 5
        des_pos = [2*t/5; 0; 2];
        des_vel = [2/5; 0; 0];
    elseif t < 10
        des_pos = [2; 2*(t-5)/5; 2];
        des_vel = [0; 2/5; 0];
    else
        des_pos = [2 - 2*(t-10)/5; 2; 2];
        des_vel = [-2/5; 0; 0];
    end

    e_pos = des_pos - pos;
    e_vel = des_vel - vel;
    acc_des = Kp * e_pos + Kd * e_vel;
    Fz = m * (acc_des(3) + g);

    phi_des = (1/g)*(acc_des(1)*sin(psi) - acc_des(2)*cos(psi));
    theta_des = (1/g)*(acc_des(1)*cos(psi) + acc_des(2)*sin(psi));
    psi_des = 0;

    ang_des = [phi_des; theta_des; psi_des];
    ang = [phi; theta; psi];
    e_ang = ang_des - ang;
    e_omega = -omega;

    tau = Kp_ang * e_ang + Kd_ang * e_omega;

    R = eul2rotm([psi, theta, phi]);
    acc = (1/m)*(R * [0; 0; Fz]) - [0; 0; g];

    omega_dot = [ (tau(1) - (Iyy - Izz)*omega(2)*omega(3)) / Ixx;
                  (tau(2) - (Izz - Ixx)*omega(1)*omega(3)) / Iyy;
                  (tau(3) - (Ixx - Iyy)*omega(1)*omega(2)) / Izz ];

    phi_dot = omega(1) + tan(theta)*(omega(2)*sin(phi) + omega(3)*cos(phi));
    theta_dot = omega(2)*cos(phi) - omega(3)*sin(phi);
    psi_dot = (omega(2)*sin(phi) + omega(3)*cos(phi))/cos(theta);

    dx = zeros(12,1);
    dx(1:3) = vel;
    dx(4:6) = acc;
    dx(7) = phi_dot;
    dx(8) = theta_dot;
    dx(9) = psi_dot;
    dx(10:12) = omega_dot;
end

%% === ROTAZIONE DA EULERO A MATRICE ===
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
