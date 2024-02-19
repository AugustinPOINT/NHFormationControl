function L = vshapeGL(N)
    % Returns a v-shaped graph laplacian as a matrix
    %

    if(N==3)
        L = completeGL(N);
        return;
    end
    
    L = diag([[2 3] 4*ones(1,N-4) [3 2]]) - diag(ones(1,(N-1)), 1) - diag(ones(1, N-1), -1) - diag(ones(1,(N-2)), 2) - diag(ones(1, N-2), -2);
end