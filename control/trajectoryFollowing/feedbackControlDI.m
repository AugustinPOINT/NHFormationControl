function u = feedbackControlDI(x, consign, time_step)
    N = size(x,1)/2;
    %% Initialize integral error
    persistent eI;
    if(isempty(eI)), eI = 0; end
    %% Computation of the consign
    w = consign(:,1);
    wd = consign(:,2);
    wdd = consign(:,3);
    %% Computation of the output
    y = x(1:N);
    yd = x(N+1:end);
    %% Feedback linearization
    A = eye(N);
    B = zeros(2,1);
    %% Command of the linear system
    v = 1*eI + 3*(w-y) + 3*(wd-yd)+ wdd; % Constante de temps de 1s
    %v = 0.125*eI + 0.75*(w-y) + 1.5*(wd-yd)+ wdd; % Constante de temps de 2s
    %% Command of the nonlinear system
    u = A\(v-B);
    %% Update the integral term
    eI = eI + time_step*(w-y);
end