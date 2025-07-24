function pos_ref = reference_trajectory(t)
    if t < 5
        pos_ref = [2*t/5; 0; 2];
    elseif t < 10
        pos_ref = [2; 2*(t-5)/5; 2];
    else
        pos_ref = [2 - 2*(t-10)/5; 2; 2];
    end
end
