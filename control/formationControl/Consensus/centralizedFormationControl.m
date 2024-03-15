function [controls, actualL] = centralizedFormationControl(poses, L, u)
    % Computes commands for a group of N robots in a centralized way
    %
    
    N = size(poses,2);
    
    %% Compute the robots controls
    gmax = 10;
    gmin = 1;
    global g;
    %g = 4;
    
    controls(:,:) = zeros(size(poses));
    
    actualL = L;
    for i = 2:N
        controls(:, i) = [0; 0];
        neighbors = topologicalNeighbors(L, i);
        for j = neighbors
            dij = norm(poses(1:2, j) - poses(1:2, i));
            gain = sin(atan(abs(dij^2 - L(i,j)^2)/5))*(gmax-gmin)+(gmax-(gmax-gmin));
            controls(:, i) = controls(:, i) + g * (dij^2 - L(i,j)^2) * (poses(1:2, j) - poses(1:2, i))/dij;
            actualL(i,j) = -dij;
            actualL(j,i) = -dij;
        end
    end
    
end