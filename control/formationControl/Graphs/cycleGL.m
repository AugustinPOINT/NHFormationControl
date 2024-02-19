function L = cycleGL(N)
    % Returns a cyclic graph laplacian as a matrix
    %
    L = 2*eye(N) - diag(ones(1,(N-1)), 1) - diag(ones(1,(N-1)), -1);
    L(N, 1) = -1;
    L(1, N) = -1;
end

