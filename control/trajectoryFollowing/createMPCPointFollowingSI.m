function fun = createMPCPointFollowingSI(x0, Nc, Np, ulim, dulim, xlim, ylim, dt)
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
    [Phi_Phi, Phi_F, Phi_Rs, Phi, F, A, B, C] = mpcterms(Ad, Bd, Cd, Nc, Np);
    %% Compute the static constraints
    Adu = []; bdu = [];
    if(~isempty(dulim))
        Adu = [-eye(Nc*ni); eye(Nc*ni)];
        bdu = [-dulim(1)*ones(Nc*ni,1); dulim(2)*ones(Nc*ni,1)];
    end
    %% Initialize variables
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
        %% Compute the system-variables dependent constraints
        % Input constraints
        Au = [];
        bu = [];
        if(~isempty(ulim))
            C1 = kron(ones(Nc,1), eye(ni));
            C2 = kron(tril(ones(Nc,Nc)), eye(ni));
            Au = [-C2; C2];
            bu = [-ulim(1)+C1*us(1:2,1); ulim(2)-C1*us(1:2,1)];
        end
        % Output constraints
        Ay = [];
        by = [];
        if(~isempty(ylim))
            Ay = [-Phi; Phi];
            by = [-ylim(1)+F*xa; ylim(2)-F*xa];
        end
        % State constraints
        Ax = [];
        bx = [];
        if(~isempty(xlim))
            Ax = [-B; B];
            bx = [-xlim(1)+A*xa; xlim(1)-A*xa];
        end
        % All constraints
        A = [Adu; Au; Ax; Ay];
        b = [bdu; bu; bx; by];
        %% Express the system as a Quadratic Programming problem
        E = 2*(Phi_Phi+eye(Nc*ni)*rw); % Quadratic term
        if(size(ya,2)==1)
            ya = repmat(yd, Nc, 1);
            f = -2*(Phi_Rs.*ya-Phi_F*xa); % Linear term
        else % We consider we were given the Np desired outputs
            ya = reshape(ya, 2*Np, 1);
            f = -2*(Phi'*ya-Phi_F*xa); % Linear term
        end
        % G = (); % Constant term unused in the optimisation
        %% Creation of the optimisation problem
        pb.H = E;
        pb.f = f;
        pb.Aineq = A;
        pb.bineq = b;
        pb.solver = 'quadprog';
        pb.options = optimoptions('quadprog','Display','off');
        %% Computation of the solution
        [deltaU,fval,exitflag,output,lambda] = quadprog(pb);
        if(exitflag~=1)
            disp("Error solving QP");
            u = [0; 0];
        else
            us = us + deltaU;
            u = us(1:2,1);
        end
        x_pre = x;
    end

end