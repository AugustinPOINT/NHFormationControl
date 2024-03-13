%% Load parameters
parameters; % Load parameters
k_final = iterations; % Set the default index representing the end of the simulation

%% Figure
if(displayType~=0)
    if(~exist("app","var") || ~isvalid(app))
        app = AppDisplay;
    end
    app.clearAll();
end

%% Define the trajectory
p1 = [-17; -5];
p2 = [17; 13];
trajectory = [p1 [-12; 5] [-5; 7] [0; 3.5] [6; 5] [12; 3.5] [15; 5] p2];
xx = (p1(1):0.1:p2(1));
yy = spline(trajectory(1,:), trajectory(2,:), xx);
spl = [xx; yy];
polyline = createPolylineParametrization(spl, 3);
% Todo : allow user to change the trajectory on the UI with left-click to add points, right-click to remove

%% Robot(s)
vlim = 7.5;
a_lim = 15;
robots = {};
initial_positions = [-14+2*cos(linspace(pi/3,2*pi+pi/3,N+1)); -8+2*sin(linspace(pi/3,2*pi+pi/3,N+1))];
robots{1} = SingleIntegrator(2, initial_positions(:,1), [], [-vlim; vlim], [inf; inf], time_step); %[-16; -14]
robots{2} = SingleIntegrator(2, initial_positions(:,2), [], [-vlim; vlim], [a_lim; a_lim], time_step);
robots{3} = SingleIntegrator(2, initial_positions(:,3), [], [-vlim; vlim], [a_lim; a_lim], time_step);
robots{4} = SingleIntegrator(2, initial_positions(:,4), [], [-vlim; vlim], [a_lim; a_lim], time_step);
robots{5} = SingleIntegrator(2, initial_positions(:,5), [], [-vlim; vlim], [a_lim; a_lim], time_step);
poses = [robots{1}.state, robots{2}.state, robots{3}.state, robots{4}.state, robots{5}.state];
rcolors = rainbow(N);

%% Communication architecture
d1 = 1;
L = vshapeWeightedGL(N, d1); % Topology matrix representing the formation
topology = L;
[rows, cols] = find(topology < 0);

%% Saved data
% Trajectories
trajectories = cell(N,1);
for r = 1:N
    trajectories{r} = zeros(2,iterations);
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
interdistances = inf(ndist, iterations); % Array of interdistances
segcolors = flip(rainbow(ndist,1));
% Formation distances errors
mFormation1 = inf(1,iterations); % MSE (Mean Square Error) of the distance errors between each robot and its neighbors
% Computation time
computationTime = inf(1,iterations);

%% Metrics
formationThresh = 0.33; % Admissible error, equivalent to a mean error of sqrt(0.25)=0.5;

%% Controllers
controls = cell(1,N);
% Reset the integral terms
clear feedbackControlSI;
clear feedbackControlDI;
% Choice of the controller
controller = Controllers.CentralizedConsensusMPC;
if(controller == Controllers.CentralizedConsensusMPC)
    Nc = 5;
    Np = 50;
    R = 1;
    ulim = [-5 5];
    dulim = [-5 5];
    xlim = [];
    ylim = [];
    MPCPointFollowingSI = createMPCPointFollowingSI(poses(:,1), Nc, Np, ulim, dulim, xlim, ylim, time_step);
end

%% Loop
for k = 1:iterations
    tic;
    t = k * time_step;
    
    %% Compute the controls
    switch(controller)
        case Controllers.CentralizedConsensusFeedback
            % Compute Leader controls
            w = polyline(t);
            uLead = feedbackControlSI(poses(:,1), w, time_step);
            % Compute Followers controls
            u = centralizedFormationControl(poses, L);
            % Format controls
            controls = mat2cell(u,2,ones(1,size(u,2)));
            controls(:,1) = {uLead};
        case Controllers.CentralizedConsensusMPC
            % Compute Leader controls
            w = polyline(linspace(t,t+Np*time_step,Np));
            %w = polyline(t);
            uLead = MPCPointFollowingSI(poses(:,1), R, w(:,1:Np));
            % Compute Followers controls
            u = centralizedFormationControl(poses, L);
            % Format controls
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
    mFormation1(1,k) = mean(abs(interdistances_des(:)-interdistances(:,k)));

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

    %% Compute the computation time before the display
    computationTime(1,k) = toc;

    %% Update the display
    if(displayType==1 || (displayType==2 && k == 1))
        % Main
        if(displayType == 2)
            title(app.mainAxis, "Snapshots des positions de la formation le long de la trajectoire");
        end
        cla(app.mainAxis);
        drawIntegrator(app.mainAxis, w(:,1), 0.15, 'g');
        drawTrajectory(app.mainAxis, spl);
        drawTrajectory(app.mainAxis, trajectory, "black", [], 'none', "none");
        drawTarget(app.mainAxis, p1, [], [], 1);
        text(app.mainAxis, p1(1)-3, p1(2)+2, sprintf('Début'), "Color", "green", "FontSize", 20);
        drawTarget(app.mainAxis, p2, [], [], 1);
        text(app.mainAxis, p2(1)-1, p2(2)+2, sprintf('Fin'), "Color", "green", "FontSize", 20);
        for r = 1:N
            drawIntegrator(app.mainAxis, poses(:,r), 0.3, rcolors(r));
            text(app.mainAxis, poses(1,r)-0.35, poses(2,r)+0.6, sprintf('R%d', r));
        end
        % for r = 1:size(rows,1)
        %     drawArrow(app.mainAxis, poses(:,rows(r)), poses(:,cols(r)), 0.15, 0.3);
        % end
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
    end
    if(displayType == 2)
        if(floor(mod(k-1, time_capture/time_step)+1e-13)==0) % Every 5 seconds (simulation time)
            text(app.mainAxis, mean(poses(1,:))-1, max(poses(2,:))+2, sprintf('t=%ds', floor((k+1)*time_step)), "FontSize", 20);
            for r = 1:N
                drawIntegrator(app.mainAxis, poses(:,r), 0.3, rcolors(r));
                text(app.mainAxis, poses(1,r)-0.35, poses(2,r)+0.6, sprintf('R%d', r));
            end
            % for r = 1:size(rows,1)
            %     drawArrow(app.mainAxis, poses(:,rows(r)), poses(:,cols(r)), 0.15, 0.3);
            % end
        end
    end

    ttoc = toc;
    delta_t = max(time_step-ttoc,0);
    %pause(delta_t);
    drawnow();
end

%% Global simulation metrics
mConvergence1 = find(mFormation1(1:k_final)<formationThresh,1); % Time until first formation (in number of iterations)
mConvergence2 = find(mFormation1(1:k_final)<formationThresh,1)*time_step; % Time until first formation (in seconds)
if(isempty(mConvergence1)), mConvergence1 = k_final; mConvergence2 = k_final*time_step; end
mStability1 = 100*sum(mFormation1(1:k_final)<formationThresh)/k_final; % Percentage of time spent in formation
mStability2 = 100*sum(mFormation1(mConvergence1:k_final)<formationThresh)/(k_final-mConvergence1+1); % Percentage of time spent in formation after convergence
mComplexity = mean(computationTime(1:k_final-1)); % Mean computation time each iteration
metrics = [["Convergence time (s)"; num2str(mConvergence2)], ["Stability 1 (% of time)"; num2str(mStability1)], ["Stability 2 (% of time)"; num2str(mStability2)], ["Complexity (s)"; num2str(mComplexity)]];
%disp(metrics);
