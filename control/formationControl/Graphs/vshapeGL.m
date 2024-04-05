function [L, I, W, C] = vshapeGL(N, d1)
    % Computes the matrices L, I and W representing a v-shaped topology.
    % L is the Laplacian matrix, I the edges matrix, W a weight matrix
    % containing the different edges weights and C a complete graph matrix
    % containing the paiwise desired inter-agent distances.
    %

    %% Edges matrix
    M = 2*N-3;
    I = zeros(N, M);
    col = 1;
    for i = 1:N
        for j = i:i+2
            if(i~=j && j > 0 && j <= N)
                I(i,col) = 1;
                I(j,col) = -1;
                col = col + 1;
            end
        end
    end
    % Weights
    if(~exist("d1","var") || isempty(d1))
        W = eye(M);
    else
        w = zeros(1, M);
        w(1:3) = [1 1 sqrt(2)]; % Leader triangle part
        for i = 1:(N-3)/2 % Trapezoidal parts
            w(4*i) = 1; % one time the d1 length
            w(4*i+1) = sqrt(i^2+(i+1)^2);
            w(4*i+2) = 1; % one time the d1 length
            w(4*i+3) = 2*w(4*i-1); % 2 times the previous trapezoide base length
        end
        W = diag(w);
    end
    %% Laplacian
    L = I*W*I';
    %% Complete graph
    % Fill in the non-diagonal terms
    C = zeros(N, N);
    for i = 1:floor(N/2)
        C = C + diag(i*ones(N-2*i,1),2*i);
        C = C + diag(i*ones(N-2*i,1),-2*i);
    end
    % Fill in the terms for the leader agent
    for i = 1:floor(N/2)
        C(1,2*i) = i;
        C(2*i,1) = i;
    end
    % Fill in the diagonal terms
    for i = 2:N
        for j = i+1:2:N
            id_i = floor(i/2);
            id_j = floor(j/2);
            C(i,j) = sqrt(id_i^2 + id_j^2);
            C(j,i) = sqrt(id_i^2 + id_j^2);
        end
    end
    C = d1*C;
end