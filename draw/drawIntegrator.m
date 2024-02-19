function handle = drawIntegrator(axis, state, radius, color)
    % Draws a integrator robot to the given figure
    %
    if(isempty(axis)), axis = gca; end
    if(~exist("radius", "var") || isempty(radius)), radius = 1; end
    if(~exist("color", "var") || isempty(color)), color = 'r'; end
    
    pos = state(1:2);
    handle = drawCircle(axis, pos, radius, color, 2.5);
end