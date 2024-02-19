function polyline = createPolylineParametrization(points_, rate)
    % Function that returns a parametrization of a polyline
    %
    if(~exist("rate", "var") || isempty(rate)), rate=1; end

    points = points_;
    segLenghts = vecnorm(points(:,2:end)-points(:,1:end-1))/rate;
    cumSegLenghts = [0 cumsum(segLenghts)];
    polyline = @polyline_;

    function w = polyline_(t)
        N = size(t,2);
        repCumSumLength = repmat(cumSegLenghts, N, 1);
        [maxVal, seg] = max(t < repCumSumLength'); % Index of the segment t belongs to
        seg = seg - 1;
        if(maxVal == 0) % The parameter is higher than the polyline total length
            w = [points(:,end) zeros(2,2)];
        else
            dSeg = cumSegLenghts(seg+1)-cumSegLenghts(seg);
            w_ = points(:,seg) + (t-cumSegLenghts(seg))*(points(:,seg+1)-points(:,seg))./dSeg;
            wd_ = rate*ones(2,1);
            wdd_ = zeros(2,1);
            w = [w_ wd_ wdd_];
        end
    end
    
end