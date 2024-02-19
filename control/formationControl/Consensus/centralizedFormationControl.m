function controls = centralizedFormationControl(poses, topology)
    % Computes commands for a group of N robots in a centralized way
    %
    
    N = size(poses,2);
    %% Define the formation
    d1 = 0.75;
    L = topology;

    %% Compute the robots controls
    gain = 2;
    controls(:,:) = zeros(size(poses));
    
    for i = 2:N
        controls(:, i) = [0; 0];
        neighbors = topologicalNeighbors(L, i);
        for j = neighbors
            controls(:, i) = controls(:, i) + ...
                gain*(norm(poses(1:2, j) - poses(1:2, i))^2 -  L(i,j)^2)*(poses(1:2, j) - poses(1:2, i));
        end
    end
    
end