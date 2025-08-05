function [lIndex] = cal_lIndex(position_i,x,y,m)

    dis = zeros(1,m);
    for j = 1:m
        dis(j) = sqrt( (position_i(1)-x(j))^2 + (position_i(2)-y(j))^2 );
    end
    [minVal, lIndex] = min(dis);

end