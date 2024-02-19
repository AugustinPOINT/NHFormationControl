function L = completeGL(N)
    % Returns a complete graph laplacian as a matrix
    %
    L = N * eye(N) - ones(N,N);
end