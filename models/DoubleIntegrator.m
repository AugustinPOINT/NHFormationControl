classdef DoubleIntegrator < handle
    % Class to represent an object with single integrator dynamics
    %

    properties
        state = []; % State vector of the SI
        state_pre = []; % Previous state vector of the SI
        input = []; % Input vector of the SI
        input_pre = []; % Previous input vector of the SI
        N = 0; % Dimension of the system
        dt = 0.05; % Time step
        
        saturate_inputs = false; % Wether the inputs are saturated or not
        sat_max = []; % Max accel and speed values
        sat_min = []; % Min accel and speed values
        flagInputSaturated = false; % Saturation detected on inputs
        flagStateSaturated = false; % Saturation detected on state
        flagInputSet = false; % Input correctly set
    end

    methods
        % Constructor
        function this = DoubleIntegrator(N_, state_ini, input_ini, sat_min_, sat_max_, dt_)
            this.N = N_;
            % Initialize state
            if(~exist("state_ini","var") || isempty(state_ini))
                this.state = zeros(2*N_,1);
            else
                if(any(size(state_ini) ~= [2*N_, 1]))
                    error("SingleIntegrator.SingleIntegrator : "+"Wrong dimensions");
                else
                    this.state = state_ini;
                end
            end
            % Initialize input
            if(~exist("input_ini","var") || isempty(input_ini))
                this.input = zeros(N_,1);
            else
                if(any(size(input_ini) ~= [N_, 1]))
                    error("SingleIntegrator.SingleIntegrator : "+"Wrong dimensions");
                else
                    this.input = input_ini;
                end
            end
            % Initialize saturations on the input
            if(~exist("sat_max_","var") && ~exist("sat_min_","var") || isempty(sat_max_) && isempty(sat_min_)) % No saturations specified
                this.saturate_inputs = false;
            elseif(exist("sat_max_","var") && exist("sat_min_","var"))
                if(size(sat_max_,1) ~= 2*N_ || size(sat_min_,1) ~= 2*N_) % Saturations specified but not the right dims
                    error("SingleIntegrator.SingleIntegrator : "+"Wrong dimensions");
                else % Saturations specified with the right dims
                    this.saturate_inputs = true;
                    this.sat_min = sat_min_;
                    this.sat_max = sat_max_;
                end
            else % Not all saturations specified
                error("SingleIntegrator.SingleIntegrator : "+"Missing parameter");
            end
            % Initialize time step
            if(~exist("dt_","var") || isempty(dt_))
                this.dt = 0.05;
            else
                this.dt = dt_;
            end
        end

        % Commands setter function
        function SetInput(this, new_input)
            if(any(size(new_input) ~= size(this.input)))
                error("SingleIntegrator.SetInput : "+"Wrong dimensions");
            end

            % Reset the saturation check
            this.flagInputSaturated = false;

            % Set the inputs
            this.input_pre = this.input;
            this.input = new_input;
            this.flagInputSet = true;

            % Validate the inputs
            if(this.saturate_inputs)
                if(any(this.input > this.sat_max(this.N+1:end)) || any(this.input < this.sat_min(this.N+1:end)))
                    this.input = max(min(this.input, this.sat_max(this.N+1:end)), this.sat_min(this.N+1:end));
                    this.flagInputSaturated = true;
                end
            end
        end

        % Step function
        function Step(this)
            if(~this.flagInputSet)
                error("SingleIntegrator.Step : "+"No input set");
            end
            % Evolution model
            this.state(1:this.N) = this.state(1:this.N) + this.dt.*this.state(this.N+1:end);
            this.state(this.N+1:end) = this.state(this.N+1:end) + this.dt.*this.input;
            % Validate the state
            if(this.saturate_inputs)
                if(any(this.state(this.N+1:end) > this.sat_max(1:this.N)) || any(this.state(this.N+1:end) < this.sat_min(1:this.N)))
                    this.state(this.N+1:end) = max(min(this.state(this.N+1:end), this.sat_max(1:this.N)), this.sat_min(1:this.N));
                    this.flagStateSaturated = true;
                else
                    this.flagStateSaturated = false;
                end
            end
            % Set the flag back to false
            this.flagInputSet = false;
        end
    end
    
end