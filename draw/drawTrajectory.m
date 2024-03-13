function handle = drawTrajectory(axis, points, color, lineWidth, lineStyle, marker)
    % Draws a trajectory to the given figure
    %
    if(isempty(axis)), axis = gca; end
    if(~exist("color", "var") || isempty(color)), color='black'; end
    if(~exist("lineWidth", "var") || isempty(lineWidth)), lineWidth = 1; end
    if(~exist("lineStyle", "var") || isempty(lineStyle)), lineStyle = '-'; end
    if(~exist("marker", "var") || isempty(marker)), marker = 'none'; end

    handle = plot(axis, points(1,:), points(2,:), "Color", color, "LineStyle", lineStyle, "LineWidth", lineWidth, "Marker", marker);
end