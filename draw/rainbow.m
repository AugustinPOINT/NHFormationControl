function colors = rainbow(N, skip)
    %
    %
    if(~exist("skip", "var") || isempty(skip)), skip=0; end

    splits = (floor(log(N+skip-1)/log(3))+1);
    colors = strings(1,N);
    RGB = dec2hex((0:splits-1).*floor(255/(splits-1)));
    count = 1;
    for i = 1:splits
        for j = 1:splits
            for k = 1:splits
                if(count > N)
                    return;
                end
                if(skip~=0)
                    skip = skip-1;
                    continue;
                end
                colors(count) = sprintf("#%s%s%s",RGB(k,:),RGB(j,:),RGB(i,:));
                count = count + 1;
            end
        end
    end
end