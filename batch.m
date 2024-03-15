global g;
% for batchNum = 1:80
%     g = batchNum/4+1;
samples = [2 3 4 20];
for batchNum = 1:size(samples,2)
    g = samples(batchNum);
    g
    main;
    if(batchNum==1)
        totalMetrics = [["gain"; string(g)], metrics];
    else
        totalMetrics = [totalMetrics; [string(g) metrics(2:end,:)]];
    end
end
disp(totalMetrics);
str2double(totalMetrics(2:end,:))