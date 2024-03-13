function polyline = createPolylineParametrization(points_, rate)
    % Function that returns a parametrization of a polyline
    %
    if(~exist("rate", "var") || isempty(rate)), rate=1; end

    %% Computation of the cumulative path length of each segment
    points = points_;
    segLenghts = vecnorm(points(:,2:end)-points(:,1:end-1))/rate;
    cumSegLenghts = [0 cumsum(segLenghts)];

    polyline = @polyline_;
    function w = polyline_(t)
        %% Computation of the segment index that contains each given t
        N = size(t,2);
        repCumSumLength = repmat(cumSegLenghts, N, 1);
        [maxVal, seg] = max(t < repCumSumLength'); % Index of the segment t belongs to
        seg = seg - 1;
        seg(maxVal == 0) = size(points,2)-1; % If the point crosses the total path length, we associate it the last segment
        %% Computation of an interpolation parameter k, for each t
        k = (t-cumSegLenghts(seg))./segLenghts(seg);
        k = repmat(k,2,1); % Parameter in [0-1] mesuring distance along segments for each t
        %% Point interpolation for each t
        w_ = points(:,seg).*(1-k)+points(:,seg+1).*k;
        %% Formating of the outputs as the concatenation of the positions, speeds and accelerations of the points
        vec = points(:,seg+1)-points(:,seg);
        wd_ = rate*vec/norm(vec);
        wdd_ = zeros(2,N);
        w = [w_ wd_ wdd_];
    end
    
end