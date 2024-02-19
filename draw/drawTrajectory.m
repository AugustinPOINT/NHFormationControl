function handle = drawTrajectory(axis, points, color, lineWidth, lineStyle)
    % Draws a trajectory to the given figure
    %
    if(isempty(axis)), axis = gca; end
    if(~exist("color", "var") || isempty(color)), color='black'; end
    if(~exist("lineWidth", "var") || isempty(lineWidth)), lineWidth = 1; end
    if(~exist("lineStyle", "var") || isempty(lineStyle)), lineStyle = '-'; end

    handle = plot(axis, points(1,:), points(2,:), "Color", color, "LineStyle", lineStyle, "LineWidth", lineWidth);
end