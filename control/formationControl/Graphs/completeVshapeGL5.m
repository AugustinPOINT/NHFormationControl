function L = completeVshapeGL5(d1)
    % Returns a complete v-shaped graph laplacian as a matrix
    %

    if(~exist("d1", "var") || isempty(d1)), d1=1; end
    s2 = sqrt(2);
    s5 = sqrt(5);
    L = [ 2  -1  -1    -2    -2;
         -1   3 -s2    -1   -s5;
         -1 -s2   4   -s5    -1;
         -2  -1 -s5     3 -2*s2;
         -2 -s5  -1 -2*s2     2];
    L = L * d1;
end