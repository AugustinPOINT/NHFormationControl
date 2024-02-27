classdef Unicycle < handle
    % Class to represent an object with Unicycle dynamics
    %

    properties
        state = []; % State vector of the SI
        state_pre = []; % Previous state vector of the SI
        input = []; % Input vector of the SI
        input_pre = []; % Previous input vector of the SI
        dt = 0.05; % Time step
        
        saturate_inputs = false; % Wether the inputs are saturated or not
        v_max = []; % Max x,y speed
        a_max = []; % Max x,y accel
        flagInputSaturated = false; % Saturation of the command
        flagInputSet = false; %
    end

    methods
        % Constructor
        function this = Unicycle(N, state_ini, input_ini, v_max_, a_max_, dt_)
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
            % Initialize saturations on the speed
            if(~exist("v_max_","var") && ~exist("a_max_","var") || isempty(v_max_) && isempty(a_max_)) % No saturations specified
                this.saturate_inputs = false;
            elseif(exist("v_max_","var") && exist("a_max_","var"))
                if(size(v_max_,1) ~= 2 || size(a_max_,1) ~= N) % Saturations specified but not the right dims
                    error("SingleIntegrator.SingleIntegrator : "+"Wrong dimensions");
                else % Saturations specified with the right dims
                    this.saturate_inputs = true;
                    this.v_max = v_max_;
                    this.a_max = a_max_;
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
                % Speed saturation
                if(this.input(1) > this.v_max(1))
                    delta = this.v_max(1)./this.input(1);
                    this.input(1) = this.input(1).*delta;
                    this.flagInputSaturated = true;
                end
                % Ang velocity saturation
                if(abs(this.input(2)) > this.v_max(2))
                    sgn = sign(this.input(2));
                    delta = this.v_max(2)./this.input(2);
                    this.input(2) = sgn*this.input(2).*delta;
                    this.flagInputSaturated = true;
                end

                % Acceleration saturation
                if(any(abs(this.input(1)-this.input_pre(1))/this.dt > this.a_max(1)))
                    sgn = sign(this.input(1)-this.input_pre(1));
                    this.input(1) = sgn.*this.a_max(1)*this.dt + this.input_pre(1);
                end
                % Ang accel saturation
                if(any(abs(this.input(2)-this.input_pre(2))/this.dt > this.a_max(2)))
                    sgn = sign(this.input(2)-this.input_pre(2));
                    this.input(2) = sgn.*this.a_max(2)*this.dt + this.input_pre(2);
                end

                % Speed saturation
                if(this.input(1) > this.v_max(1))
                    delta = this.v_max(1)./this.input(1);
                    this.input(1) = this.input(1).*delta;
                    this.flagInputSaturated = true;
                end
                % Ang velocity saturation
                if(abs(this.input(2)) > this.v_max(2))
                    sgn = sign(this.input(2));
                    delta = this.v_max(2)./this.input(2);
                    this.input(2) = sgn*this.input(2).*delta;
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