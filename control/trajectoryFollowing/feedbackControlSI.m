function u = feedbackControlSI(x, consign, time_step)
    N = size(x,1);
    %% Initialize integral error
    persistent eI;
    if(isempty(eI)), eI = 0; end
    %% Computation of the consign
    w = consign(:,1);
    wd = consign(:,2);
    %% Computation of the output
    y = x(1:N);
    %% Feedback linearization
    A = eye(N);
    B = zeros(2,1);
    %% Command of the linear system
    v = 4*eI + 4*(w-y) + wd; % Constante de temps de 0.5s
    %v = 1*eI + 2*(w-y) + wd; % Constante de temps de 1s
    %v = 0.25*eI + 1*(w-y) + wd; % Constante de temps de 2s
    %% Command of the nonlinear system
    u = A\(v-B);
    %% Update the integral term
    eI = eI + time_step*(w-y);
end