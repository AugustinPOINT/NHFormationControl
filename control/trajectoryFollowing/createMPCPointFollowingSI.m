function fun = createMPCPointFollowingSI(x0, Nc, Np, dt)
    % Create a MPC controller given an initial state x0, a prediction
    % horizon Np, a control horizon Nc and a time-step dt.
    %

    %% Single Integrator model
    Ac = zeros(2,2);
    Bc = eye(2,2);
    Cc = eye(2,2);
    [no, n] = size(Cc);
    [n, ni] = size(Bc);
    %% Compute MPC terms
    Dc = zeros(size(Cc,1), size(Bc,2));
    [Ad, Bd, Cd, ~] = c2dm(Ac, Bc, Cc, Dc, dt);
    [Phi_Phi, Phi_F, Phi_Rs, Phi, A, B, C] = mpcterms(Ad, Bd, Cd, Nc, Np);
    %% Compute outputs (each time_step)
    x_pre = x0;
    us = [0; 0];
    us = repmat(us, Nc, 1);

    fun = @MPCPointFollowingSI;
    function u = MPCPointFollowingSI(x, rw, yd)
        % Computes the optimal control given the current state x, the
        % desired output yd and an input cost rw.
        %
        %% Compute augmented system
        xa = [x-x_pre; Cd*x_pre];
        ya = yd;
        %% Express the system as a Quadratic Programming problem
        E = 2*(Phi_Phi+eye(Nc*ni)*rw); % Quadratic term
        if(size(ya,2)==1)
            ya = repmat(yd, Nc, 1);
            F = -2*(Phi_Rs.*ya-Phi_F*xa); % Linear term
        else % We consider we were given the Np desired outputs
            ya = reshape(ya, 2*Np, 1);
            F = -2*(Phi'*ya-Phi_F*xa); % Linear term
        end
        % G = (); % Constant term unused in the optimisation
        %% Computation of the solution
        deltaU = QP(E, F, [], []);
        us = us + deltaU;
        u = us(1:2,1);
        x_pre = x;
    end

end