% function [pos_d, vel_d] = reference_trajectory(t)
%     if t < 5
%         pos_d = [2*t/5; 0; 2];
%         vel_d = [2/5; 0; 0];
%     elseif t < 10
%         pos_d = [2; 2*(t-5)/5; 2];
%         vel_d = [0; 2/5; 0];
%     else
%         pos_d = [2 - 2*(t-10)/5; 2; 2];
%         vel_d = [-2/5; 0; 0];
%     end
% end
% 
% 
function [pos_d, vel_d] = reference_trajectory(t)
    % Durata totale della simulazione
    tspan = [0, 15];
    t_ref = linspace(tspan(1), tspan(2), 300);

    % Traiettoria: x = t_ref, y = sin(0.5*t), z = 2 + 0.5*cos(0.5*t)
    ref_traj = [t_ref;
                sin(0.5 * t_ref);
                2 + 0.5 * cos(0.5 * t_ref)];  % 3xN

    % Interpolazione posizione
    pos_d = interp1(t_ref', ref_traj(1:3,:)', t, 'linear', 'extrap')';

    % Derivata numerica per la velocità
    dt = t_ref(2) - t_ref(1);
    vel_traj = gradient(ref_traj(1:3,:), dt);
    vel_d = interp1(t_ref', vel_traj', t, 'linear', 'extrap')';
end

