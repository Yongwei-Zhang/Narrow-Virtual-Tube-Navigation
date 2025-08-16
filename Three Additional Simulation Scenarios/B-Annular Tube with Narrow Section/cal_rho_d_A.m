function A = cal_rho_d_A(radius,min_lIndex,max_lIndex,l,all_lIndex,m,N)

    up_lIndex = [];
    down_lIndex = [];
    for i = 1:N
        if all_lIndex(i) >= 1 && all_lIndex(i) <= (m-1)/2
            up_lIndex(end+1) = all_lIndex(i);
        else
            down_lIndex(end+1) = all_lIndex(i);
        end
    end
    
    %% 
    max_up_lIndex = max(up_lIndex);
    min_down_lIndex = min(down_lIndex);
    
    %%
    lIndex_limit = 7/8 * (m-1); 
    lIndex_dis = max_lIndex - min_lIndex;
    
    % Case 1
    if lIndex_dis > lIndex_limit
        A1 = 0; 
        A2 = 0; 
        for i = 1 : (max_up_lIndex-1)
            delta_l = l(i+1) - l(i);
            delta_A1 = 2 * radius(i)^2 * delta_l;
            A1 = A1 + delta_A1;
        end
        for j = min_down_lIndex : (m-1)
            delta_l = l(j+1) - l(j);
            delta_A2 = 2 * radius(j)^2 * delta_l;
            A2 = A2 + delta_A2;
        end
        A = A1 + A2;
    % Case 2
    else
        A = 0;
        for i = min_lIndex : (max_lIndex-1)
            delta_l = l(i+1) - l(i);
            delta_A = 2 * radius(i)^2 * delta_l;
            A = A + delta_A;
        end
    end

end