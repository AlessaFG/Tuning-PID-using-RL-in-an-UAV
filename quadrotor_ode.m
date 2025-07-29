function dx = quadrotor_ode(t, x, m, g, Ixx, Iyy, Izz, L, Kp_init, Kd_init, Kp_ang, Kd_ang)
    pos = x(1:3); vel = x(4:6);
    phi = x(7); theta = x(8); psi = x(9);
    omega = x(10:12);

    % Traiettoria desiderata
   [des_pos, des_vel] = reference_trajectory(t);


    e_pos = des_pos - pos;
    e_vel = des_vel - vel;

    acc_des = Kp_init * e_pos + Kd_init * e_vel;
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
    acc = (1/m)*(R * [0; 0; Fz]) - [0; 0; g] + [1;1;1];

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



% %% File: quadrotor_ode.m
% function dx = quadrotor_ode(~, x, m, g, Ixx, Iyy, Izz, L, Kp_init, Kd_init, Kp_ang, Kd_ang)
%     pos   = x(1:3);   vel   = x(4:6);
%     phi   = x(7);     theta = x(8);   psi = x(9);
%     omega = x(10:12);
% 
%     % === Controllore posizionale ===
%     % (qui x(1:3) e x(4:6) sono già nel referenziale mondo)
%     % usiamo Kp_init e Kd_init per acc_des
%     des_pos = []; des_vel = [];
%     % (la generazione di ref e vel la fa lo step)
% 
%     % per compatibilità, usiamo piccoli errori (sono già calcolati nello step)
%     acc_des = Kp_init*(0) + Kd_init*(0);
%     Fz = m*(acc_des(3) + g);
% 
%     % === Roll/Pitch/Heading desiderati ===
%     phi_des   = 0;
%     theta_des = 0;
%     psi_des   = 0;
% 
%     e_ang   = [phi_des; theta_des; psi_des] - [phi; theta; psi];
%     e_omega = -omega;
% 
%     tau = Kp_ang*e_ang + Kd_ang*e_omega;
% 
%     R = eul2rotm([psi, theta, phi]);
%     acc = (1/m)*(R*[0;0;Fz]) - [0;0;g];
% 
%     omega_dot = [ (tau(1) - (Iyy-Izz)*omega(2)*omega(3))/Ixx;
%                   (tau(2) - (Izz-Ixx)*omega(1)*omega(3))/Iyy;
%                   (tau(3) - (Ixx-Iyy)*omega(1)*omega(2))/Izz];
% 
%     phi_dot   = omega(1) + tan(theta)*(omega(2)*sin(phi)+omega(3)*cos(phi));
%     theta_dot = omega(2)*cos(phi) - omega(3)*sin(phi);
%     psi_dot   = (omega(2)*sin(phi)+omega(3)*cos(phi))/cos(theta);
% 
%     dx = [vel; acc; phi_dot; theta_dot; psi_dot; omega_dot];
% end
% 
% %% Helper: eul2rotm (ZYX)
% function R = eul2rotm(eul)
%     psi = eul(1); theta = eul(2); phi = eul(3);
%     Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
%     Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
%     Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
%     R  = Rz*Ry*Rx;
% end