function [nb_lIndex] = cal_nb_lIndex(nb_position,x,y,lIndex1,lIndex2)

    if lIndex1 < 1
        lIndex1 = 1;
    end
    if lIndex2 > size(x,2)
        lIndex2 = size(x,2);
    end
    dis = zeros(1, lIndex2 - lIndex1 + 1); 
    for i = lIndex1 : lIndex2 
        dis(i-lIndex1+1) = sqrt( (nb_position(1)-x(i))^2 + (nb_position(2)-y(i))^2 );
    end
    [minVal, Index] = min(dis); 
    nb_lIndex = Index + (lIndex1-1); 

end