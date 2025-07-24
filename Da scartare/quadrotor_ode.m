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