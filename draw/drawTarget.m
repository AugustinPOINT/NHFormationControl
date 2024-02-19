function handles = drawTarget(axis, c, color, lineWidth, scale)
    % Draws a target to the given figure
    %
    if(isempty(axis)), axis = gca; end
    if(~exist("color", "var") || isempty(color)), color='green'; end
    if(~exist("lineWidth", "var") || isempty(lineWidth)), lineWidth = 3; end
    if(~exist("scale", "var") || isempty(scale)), scale=1; end

    h1 = plot(axis, [c(1)-scale*0.5 c(1)+scale*0.5], [c(2)-scale*0.5 c(2)+scale*0.5], "Color", color, "LineStyle", "-", "LineWidth", lineWidth);
    h2 = plot(axis, [c(1)+scale*0.5 c(1)-scale*0.5], [c(2)-scale*0.5 c(2)+scale*0.5], "Color", color, "LineStyle", "-", "LineWidth", lineWidth);
    handles = [h1 h2];
end