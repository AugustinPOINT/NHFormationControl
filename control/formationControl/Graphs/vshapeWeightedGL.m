function L = vshapeWeightedGL(N, d1)
    % Returns a v-shaped graph laplacian as a matrix
    %

    if(~exist("d1", "var") || isempty(d1)), d1=1; end
    % Part 1 : V shape
    L1 = - diag(ones(1,(N-2)), 2) - diag(ones(1, N-2), -2);
    L1(2,1) = -1;
    L1(1,2) = -1;
    L1 = d1*L1; % Distances
    L1 = L1 + 2*eye(N);
    L1(end,end) = 1;
    L1(end-1,end-1) = 1;
    % Part 2 : Trapezoid structure
    M2 = N - mod(N-1,2) - 1;
    L2 = zeros(N);
    L2(2:M2+1,2:M2+1) = L2(2:M2+1,2:M2+1) - diag([repmat([1 0],1,M2/2-1) 1],1) - diag([repmat([1 0],1,M2/2-1) 1],-1);
    L2(2:M2+1,2:M2+1) = L2(2:M2+1,2:M2+1).*max(repmat([1:M2]',1,M2),repmat([1:M2],M2,1))*d1*sqrt(2)/2;
    L2 = L2 + eye(N);
    L2(1,1) = 0;
    L2(end,end) = mod(N,2)*1;
    % Part 3 : Diagonal reinforcements
    M3 = N - mod(N,2) - 2;
    L3 = zeros(N);
    L3(3:M3+2,3:M3+2) = L3(3:M3+2,3:M3+2) - diag([repmat([1 0],1,M3/2-1) 1],1) - diag([repmat([1 0],1,M3/2-1) 1],-1);
    mat = max(repmat([1:M3]',1,M3),repmat([1:M3],M3,1));
    L3(3:M3+2,3:M3+2) = L3(3:M3+2,3:M3+2).*sqrt(2*(mat./2).^2+mat+1)*d1;
    L3 = L3 + eye(N);
    L3(1,1) = 0;
    L3(2,2) = 0;
    L3(end,end) = mod(N-1,2)*1;
    % Final Laplacian matrix
    L = L1 + L2 + L3;
end