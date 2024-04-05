function u = NLMPC(x0, u0_ini, I, Weights, Nc, Np, Wo, Wi, Wt, ulim, dulim, xlim, ylim, dt)
    % Computes the optimal controls for a group of N robots using DMPC
    %
    persistent us;

    ni = size(u0_ini, 1);
    no = size(I, 2);
    if(isempty(us))
        us = zeros(ni, Nc);
    end
    %% Computation of the consign
    yd = diag(Weights).^2;

    %% Costs
    % Inputs
    M_input = [];
    gamma_input = [];
    if(~isempty(ulim))
        M_input = [-eye(ni*Nc); eye(ni*Nc)];
        gamma_input = [repmat(-ulim(1), ni*Nc, 1); repmat(ulim(2), ni*Nc, 1)];
    end
    % Input Variation
    M_dinput = [];
    gamma_dinput = [];
    if(~isempty(dulim))
        C1 = kron(ones(Nc,1), eye(ni));
        C2_inv = eye(ni*Nc) - diag(ones(Nc*ni-ni,1),-ni);
        M_dinput = [-C2_inv; C2_inv];
        gamma_dinput = [-dulim(1)-C2_inv*C1*us(:,1); dulim(2)+C2_inv*C1*us(:,1)];
    end
    M = [M_input; M_dinput];
    gamma = [gamma_input; gamma_dinput];

    %% NL-MPC Optimisation
    pb.options = optimoptions('fmincon','Display','none','Algorithm','sqp'); % Ou 'Display','iter'
    pb.solver = 'fmincon';
    pb.objective = @(u)costFunction(u, us(:,1), x0, I, no, yd, Nc, Np, Wo, Wi, Wt, dt);
    pb.Aineq = M;
    pb.bineq = gamma;
    pb.MaxIterations = 10;
    pb.x0 = us;
    us = fmincon(pb);
    % Optimal input
    u = us(:,1);
end

function [y_pred, delta_u] = costTerms(u, u0, x0, I, no, Np, Nc, evol, obs, dt)
    %% Initialize the arrays of predicted states and outputs
    n = size(x0,1);
    x_pred = zeros(n, Np+1);
    x_pred(:,1) = x0;
    y_pred = zeros(no, Np);
    %% Compute the vector of variation of inputs
    u = [u repmat(u(:,Nc), 1, Np-Nc)];
    delta_u = u - [u0 u(:,1:Np-1)];
    %% Compute the vector of predicted outputs
    for k = 1:Np
        x_pred(:,k+1) = evol(x_pred(:,k), u(:,k), dt);
        y_pred(:,k) = obs(x_pred(:,k+1), I); % Prediction of the future output
    end
end

function J = costFunction(u, u0, x0, I, no, yd, Nc, Np, Wo, Wi, Wt, dt)
    % Compute the vectors of variation of inputs and predicted outputs
    [y_pred, delta_u] = costTerms(u, u0, x0, I, no, Np, size(u,2), @evol, @obs, dt);
    %% Compute the cost on the outputs
    J = 0;
    for k = 1:Np
        J = J + (yd-y_pred(:,k))'*Wo*(yd-y_pred(:,k));
    end
    %% Compute the cost on the inputs
    for k = 1:Nc
        J = J + delta_u(:,k)'*Wi*delta_u(:,k);
    end
    %% Compute the terminal cost
    J = J + (yd-y_pred(:,Np))'*Wt*(yd-y_pred(:,Np));
end

function [xkp1, A, B] = evol(x, u, dt)
    n = size(x,1);
    ni = size(u,1);
    Ac = zeros(n);
    Bc = eye(ni);
    xdot = Ac*x + Bc*u; % We compute x-k-plus-1
    xkp1 = x + dt*xdot;
end

function y = obs(x, I)
    n = size(x,1);
    M = n-3;
    y = kron(eye(M),[1 1])*(kron(I',eye(2))*x).^2; % Computation of the square norms of the robots interdistances
end