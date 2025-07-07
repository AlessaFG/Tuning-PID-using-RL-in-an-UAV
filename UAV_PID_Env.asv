classdef UAV_PID_Env < rl.env.MATLABEnvironment
    properties
        Ts = 0.05               % Passo di simulazione
        MaxSteps = 300
        CurrentStep = 0
        Params                 % Parametri dinamici del drone
        State                  % Stato attuale
    end
    
    properties(Access = protected)
        IsDone = false;
    end
    
    methods
        function this = UAV_PID_Env()
            ObservationInfo = rlNumericSpec([12 1]);  % stato drone
            ActionInfo = rlNumericSpec([12 1], 'LowerLimit', 0, 'UpperLimit', 500);
            this = this@rl.env.MATLABEnvironment(ObservationInfo, ActionInfo);
            reset(this);
        end
        
        function [obs, reward, isdone, loggedSignals] = step(this, action)
            this.CurrentStep = this.CurrentStep + 1;

            % Decode action as PID gains
            this.Params.Kp = diag(action(1:3));
            this.Params.Kd = diag(action(4:6));
            this.Params.Kp_ang = diag(action(7:9));
            this.Params.Kd_ang = diag(action(10:12));
            
            % Simula dinamica
            [~, X] = ode45(@(t, x) quadrotor_ode(t, x, this.Params), ...
                           [0 this.Ts], this.State);

            this.State = X(end,:)';
            obs = this.State;

            % Reward = - errore di tracking
            a = 1.5; b = 1;
            des_pos = [a * sin(this.CurrentStep*this.Ts);
                       b * sin(2*this.CurrentStep*this.Ts);
                       2];
            pos = this.State(1:3);
            e = des_pos - pos;
            reward = -norm(e);  % penalizza distanza dalla traiettoria

            isdone = this.CurrentStep >= this.MaxSteps;
            this.IsDone = isdone;
            loggedSignals = [];
        end
        
        function initialObs = reset(this)
            this.CurrentStep = 0;
            this.Params = getDefaulParams();
            this.State = zeros(12,1);
            initialObs = this.State;
            this.IsDone = false;
        end
    end
end
