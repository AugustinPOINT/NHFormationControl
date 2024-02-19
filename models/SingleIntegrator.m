classdef SingleIntegrator < handle
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
        input_max = []; % Max input value
        input_min = []; % Min input value
        flagInputSaturated = false; % Saturation of the command
        flagInputSet = false; %
    end

    methods
        % Constructor
        function this = SingleIntegrator(N, state_ini, input_ini, input_min_, input_max_, dt_)
            % Initialize state
            if(~exist("state_ini","var") || isempty(state_ini))
                this.state = zeros(N,1);
            else
                if(any(size(state_ini) ~= [N, 1]))
                    error("SingleIntegrator.SingleIntegrator : "+"Wrong dimensions");
                else
                    this.state = state_ini;
                end
            end
            % Initialize input
            if(~exist("input_ini","var") || isempty(input_ini))
                this.input = zeros(N,1);
            else
                if(any(size(input_ini) ~= [N, 1]))
                    error("SingleIntegrator.SingleIntegrator : "+"Wrong dimensions");
                else
                    this.input = input_ini;
                end
            end
            % Initialize saturations on the input
            if(~exist("input_max_","var") && ~exist("input_min_","var") || isempty(input_max_) && isempty(input_min_)) % No saturations specified
                this.saturate_inputs = false;
            elseif(exist("input_max_","var") && exist("input_min_","var"))
                if(size(input_max_,1) ~= N || size(input_min_,1) ~= N) % Saturations specified but not the right dims
                    error("SingleIntegrator.SingleIntegrator : "+"Wrong dimensions");
                else % Saturations specified with the right dims
                    this.saturate_inputs = true;
                    this.input_min = input_min_;
                    this.input_max = input_max_;
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
                if(any(this.input > this.input_max) || any(this.input < this.input_min))
                    this.input = max(min(this.input, this.input_max), this.input_min);
                    this.flagInputSaturated = true;
                end
            end
            % Validate the states
            %
        end

        % Step function
        function Step(this)
            if(~this.flagInputSet)
                error("SingleIntegrator.Step : "+"No input set");
            end
            % Evolution model
            this.state = this.state + this.dt.*this.input;
            % Set the flag back to false
            this.flagInputSet = false;
        end
    end
    
end