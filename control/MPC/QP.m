function [x_opt, x_opt_uncons] = QP(E, F, M, gamma)
    % Computes the analytical solution of a QP optimisation problem.
    % The problem is given by :
    %              min J(dU)=1/2*du'*E*dU + dU'F
    % subject to       M*dU <= gamma
    %

    %% Computation of the optimal unconstrained solution
    x_opt_uncons = -E\F;
    %% Computation of the optimal constrained solution
    x_opt_cons = 0;
    if(~isempty(M) && ~isempty(gamma))
        lambda = -(M*(E\M'))\(gamma+M*(E\F));
        x_opt_cons = -E\(M'*lambda);
    end
    %% Computation of the optimal solution
    x_opt = x_opt_cons + x_opt_uncons;
end