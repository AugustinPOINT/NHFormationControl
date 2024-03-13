function [Phi_Phi, Phi_F, Phi_Rs, Phi, F, A, B, C] = mpcterms(Ad, Bd, Cd, Nc, Np)
    %
    %

    %% Compute the augmented state-space model
    % System constants and variables
    [no, n] = size(Cd);
    [n, ni] = size(Bd);
    % Computation of the augmented model
    A = eye(n+no,n+no);
    A(1:n,1:n) = Ad;
    A(n+1:n+no,1:n) = Cd*Ad;
    B = zeros(n+no,ni);
    B(1:n,1:ni) = Bd;
    B(n+1:n+no,1:ni) = Cd*Bd;
    C = zeros(no,n+no);
    C(:,n+1:n+no) = eye(no,no);

    %% Compute the desired matrices
    m = n + no;
    F = zeros(Np*no,m);
    F(1:no,:) = C*A;
    Phi = zeros(Np*no,Nc*ni);
    for kk = no+1:no:Np*no
        F(kk:kk+no-1,:) = F(kk-no:kk-1,:)*A;
    end
    Phi(:,1:ni) = [C*B; F(1:end-no,:)*B];
    for kk = ni+1:ni:Nc*ni
        Phi(:,kk:kk+ni-1) = [zeros(no,ni) ; Phi(1:Np*no-no,kk-ni:kk-1)];
    end
    RsBar = ones(Np*no,1);
    Phi_Phi = Phi'*Phi;
    Phi_F = Phi'*F;
    Phi_Rs = Phi'*RsBar;

end