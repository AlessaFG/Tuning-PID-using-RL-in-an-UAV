function p = getDefaulParams()
    p.m = 1.5;
    p.g = 9.81;
    p.Ixx = 0.01;
    p.Iyy = 0.01;
    p.Izz = 0.02;
    p.L = 0.3;  % lunghezza bracci

% PID gains
    p.Kp = diag([4, 4, 10]);
    p.Kd = diag([3, 3, 6]);
    p.Kp_ang = diag([250, 250, 150]);
    p.Kd_ang = diag([80, 80, 50]);
    
end