function L = lineGL(N)
    % Returns a line graph laplacian as a matrix
    %
    L = 2*eye(N) - diag(ones(1,(N-1)), 1) - diag(ones(1, N-1), -1);
    L(1, 1) = 1; 
    L(N, N) = 1;
end

