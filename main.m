%% Load parameters
parameters;

%% Figure
if(~exist("app","var") || ~isvalid(app))
    app = AppDisplay;
end
app.clearAll();

%% Define the trajectory
p1 = [-17; -13];
p2 = [17; 13];
trajectory = [p1 [-12; 0] [-5; 2] [5; 0] [10; -2] [15; 0] p2];
polyline = createPolylineParametrization(trajectory, 2);
% Todo : allow user to change the trajectory on the UI with left-click to add points, right-click to remove

%% Robot(s)
vlim = 7.5;
a_lim = 15;
robots = {};
robots{1} = SingleIntegrator(2, [-16; -14], [], [-vlim; vlim], [a_lim; a_lim], time_step);
robots{2} = SingleIntegrator(2, [-12; -17], [], [-vlim; vlim], [a_lim; a_lim], time_step);
robots{3} = SingleIntegrator(2, [ -8; -17], [], [-vlim; vlim], [a_lim; a_lim], time_step);
robots{4} = SingleIntegrator(2, [-10; -15], [], [-vlim; vlim], [a_lim; a_lim], time_step);
robots{5} = SingleIntegrator(2, [-10; -19], [], [-vlim; vlim], [a_lim; a_lim], time_step);
poses = [robots{1}.state, robots{2}.state, robots{3}.state, robots{4}.state, robots{5}.state];
rcolors = rainbow(N,1);

%% Architecture
d1 = 1;
L = vshapeWeightedGL(N, d1); % Topology matrix representing the formation
topology = L;
[rows, cols] = find(topology < 0);

%% Saved data
% Trajectories
trajectories = cell(N,1);
for r = 1:N
    trajectories{r} = zeros(2,n);
end
% Interdistances
ndist = N*(N-1)/2;
interdistances_mat = -completeVshapeGL5(d1); % Matrix of desired interdistances
interdistances_des = zeros(ndist,1); % Array of desired interdistances
pair = 1;
for i = 1:N
    for j = i+1:N
        interdistances_des(pair) = interdistances_mat(i,j);
        pair = pair + 1;
    end
end
interdistances = inf(ndist, n); % Array of interdistances
segcolors = flip(rainbow(ndist,1));
% Formation distances errors
mFormation1 = inf(1,n); % MSE (Mean Square Error) of the distance errors between each robot and its neighbors
% Computation time
computationTime = inf(1,n);

%% Metrics
formationThresh = 0.33; % Admissible error, equivalent to a mean error of sqrt(0.25)=0.5;

%% Controllers
controls = cell(1,N);
% Reset the integral terms
clear feedbackControlSI;
clear feedbackControlDI;
% Choice of the controller
controller = Controllers.CentralizedConsensus;

%% Loop
for k = 1:iterations
    tic;
    t = k * time_step;
    
    %% Define the trajectory
    w = polyline(t);
    
    %% Compute the controls
    switch(controller)
        case Controllers.CentralizedConsensus
            %
            u = centralizedFormationControl(poses, L);
            uLead = feedbackControlSI(poses(:,1), w, time_step);
            controls = mat2cell(u,2,ones(1,size(u,2)));
            controls(:,1) = {uLead};
        case Controllers.LeaderSIStandalone
            %
            controls(:) = {[0;0]};
            u = feedbackControlSI(poses(:,1), w, time_step);
            controls(:,1) = {u};
        case Controllers.LeaderDIStandalone
            %
            controls(:) = {[0;0]};
            u = feedbackControlDI(poses(:,1), w, time_step);
            controls(:,1) = {u};
        otherwise
            %
            controls(:) = {[0;0]};
    end
    
    %% Set the controls
    for r = 1:N
        robots{r}.SetInput(controls{:,r});
    end

    %% Compute the interdistances
    pair = 1;
    for i = 1:N
        for j = i+1:N
            interdistances(pair,k) = norm(poses(:,i)-poses(:,j));
            pair = pair + 1;
        end
    end

    %% Compute the distance MSE
    mFormation1(1,k) = mean((interdistances_des(:)-interdistances(:,k)).^2);

    %% Update the robot state
    for r = 1:N
        robots{r}.Step();
        poses(:,r) = robots{r}.state(1:2);
        trajectories{r}(:,k) = poses(:,r);
    end

    %% End of simulation detection
    if(norm(poses(:,1)-trajectory(:,end)) < 0.1)
        k_final = k;
        break;
    end
    
    %% Update the display
    % Main
    cla(app.mainAxis);
    drawIntegrator(app.mainAxis, w(:,1), 0.15, 'g');
    drawTrajectory(app.mainAxis, trajectory);
    drawTarget(app.mainAxis, p1, [], [], 1);
    drawTarget(app.mainAxis, p2, [], [], 1);
    for r = 1:N
        drawIntegrator(app.mainAxis, robots{r}.state, 0.3, rcolors(r));
        text(app.mainAxis, robots{r}.state(1)-0.35, robots{r}.state(2)+0.6, sprintf('R%d', r));
    end
    for r = 1:size(rows,1)
        drawArrow(app.mainAxis, poses(:,rows(r)), poses(:,cols(r)), 0.15, 0.3);
    end
    % Interdistances
    cla(app.plotAxis1);
    for i = 1:ndist
        plot(app.plotAxis1, (1:k)*time_step, interdistances_des(i)-interdistances(i,1:k), "Color", segcolors(i), "LineStyle", "--");
    end
    if(mod(k, floor(2/time_step)) == 0) % Update the XLim axis every 2 seconds (in simulation time)
        app.plotAxis1.XLim = app.plotAxis1.XLim + [0 2];
    end
    % Formation error
    plot(app.plotAxis1, (1:k)*time_step, mFormation1(1,1:k), "Color", "r", "LineWidth", 1.5);
    plot(app.plotAxis1, app.plotAxis1.XLim, [0, 0], "Color", "black", "LineStyle", "-", "LineWidth", 1);
    plot(app.plotAxis1, app.plotAxis1.XLim, [formationThresh, formationThresh], "Color", "black", "LineStyle", "-.", "LineWidth", 1);

    ttoc = toc;
    computationTime(1,k) = ttoc;
    delta_t = max(time_step-ttoc,0);
    %pause(delta_t);
    drawnow();
end

mConvergence = find(mFormation1(1:k_final)<formationThresh,1); % Time until first formation
mStability = 100*sum(mFormation1(1:k_final)<formationThresh)/k_final; % Percentage of time spent in formation
mStability2 = 100*sum(mFormation1(mConvergence:k_final)<formationThresh)/(k_final-mConvergence+1); % Percentage of time spent in formation after convergence
mComplexity = mean(computationTime(1:k_final-1));
