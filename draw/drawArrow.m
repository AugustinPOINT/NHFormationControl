function handle = drawArrow(axis, p1, p2, scale, delta, color, lineWidth, lineStyle)
    % Draws a circle to the given figure
    %
    if(isempty(axis)), axis = gca; end
    if(~exist("scale", "var") || isempty(scale)), scale=1; end
    if(~exist("delta", "var") || isempty(delta)), delta=0; end
    if(~exist("color", "var") || isempty(color)), color='blue'; end
    if(~exist("lineWidth", "var") || isempty(lineWidth)), lineWidth=1; end
    if(~exist("lineStyle", "var") || isempty(lineStyle)), lineStyle='-'; end
    
    % Erase a small portion of the arrow at both ends
    dir = (p2-p1)/norm(p2-p1);
    if(delta ~= 0)
        p1 = p1+delta*dir;
        p2 = p2-delta*dir;
    end
    d = scale;
    p3 = p2-d*dir;
    handle = line(axis, [p1(1) p2(1) p3(1)+d*dir(2) p2(1) p3(1)-d*dir(2)], [p1(2) p2(2) p3(2)-d*dir(1) p2(2) p3(2)+d*dir(1)], "Color", color, "LineStyle", lineStyle, "LineWidth", lineWidth);
end