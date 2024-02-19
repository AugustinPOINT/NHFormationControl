%% Simulation
time_step = 0.02;
n = 2000;

%% Figure
if(~exist("app","var") || ~isvalid(app))
    app = AppDisplay;
end

%% Define the trajectory
p1 = [-17; -13];
p2 = [17; 13];
trajectory = [p1 [-12; 0] [-5; 2] [5; 0] [10; -2] [15; 0] p2];
polyline = createPolylineParametrization(trajectory, 2);
% Todo : allow user to change the trajectory on the UI with left-click to add points, right-click to remove

%% Robot(s)
N = 5;
robots = {};
robots{1} = SingleIntegrator(2, [-16; -13], [], [], [], time_step); % /!\ Le robot a tendance a aller en diag car c'est sa vmax
robots{2} = SingleIntegrator(2, [-12; -17], [], [-10; -10], [10; 10], time_step);
robots{3} = SingleIntegrator(2, [-8; -17], [], [-10; -10], [10; 10], time_step);
robots{4} = SingleIntegrator(2, [-10; -15], [], [-10; -10], [10; 10], time_step);
robots{5} = SingleIntegrator(2, [-10; -19], [], [-10; -10], [10; 10], time_step);
poses = [robots{1}.state, robots{2}.state, robots{3}.state, robots{4}.state, robots{5}.state];
rcolors = ["#FF0000","#990033","#339900","#009933","#003399"];

%% Architecture
topology = decentralizedGL(N);
[rows, cols] = find(topology < 0);
for i = 1:length(rows)
   %links(i) = line([poses(1,rows(i)), poses(1,cols(i))],[poses(2,rows(i)), poses(2,cols(i))], 'LineWidth', 2, 'Color', 'b'); 
end

%% Saved data
trajectories = cell(N,1);

%% Controllers
controls = cell(1,N);
% Reset the integral terms
clear feedbackControlSI;
clear feedbackControlDI;
% Choice of the controller
controller = Controllers.CentralizedConsensus;

%% Loop
for i = 1:n
    tic;
    t = i * time_step;
    
    %% Define the trajectory
    w = polyline(t);
    
    %% Compute the controls
    switch(controller)
        case Controllers.CentralizedConsensus
            %
            u = centralizedFormationControl(poses, vshapeWeightedGL(N));
            uLead = feedbackControlSI(robots{1}.state, w, time_step);
            controls = mat2cell(u,2,ones(1,size(u,2)));
            controls(:,1) = {uLead};
        case Controllers.LeaderSIStandalone
            %
            controls(:) = {[0;0]};
            u = feedbackControlSI(robots{1}.state, w, time_step);
            controls(:,1) = {u};
        case Controllers.LeaderDIStandalone
            %
            controls(:) = {[0;0]};
            u = feedbackControlDI(robots{1}.state, w, time_step);
            controls(:,1) = {u};
        otherwise
            %
            controls(:) = {[0;0]};
    end
    
    %% Set the controls
    for r = 1:N
        robots{r}.SetInput(controls{:,r});
    end
    %% Update the robot state
    for r = 1:N
        robots{r}.Step();
    end
    poses = [robots{1}.state, robots{2}.state, robots{3}.state, robots{4}.state, robots{5}.state];
    
    %% Update the display
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
        drawArrow(app.mainAxis, poses(:,rows(r)), poses(:,cols(r)), 0.5, 0.3);
    end

    ttoc = toc;
    delta_t = max(time_step-ttoc,0);
    pause(delta_t);
end
