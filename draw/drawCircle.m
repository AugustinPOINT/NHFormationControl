function handle = drawCircle(axis, c, r, color, lineWidth, lineStyle)
    % Draws a circle to the given figure
    %
    if(isempty(axis)), axis = gca; end
    if(~exist("color", "var") || isempty(color)), color='black'; end
    if(~exist("lineWidth", "var") || isempty(lineWidth)), lineWidth=1; end
    if(~exist("lineStyle", "var") || isempty(lineStyle)), lineStyle='-'; end
    
    handle = rectangle(axis, 'Position',[c(1)-r c(2)-r 2*r 2*r],'Curvature', 1, 'EdgeColor', color, 'LineWidth', lineWidth, 'LineStyle', lineStyle);
end