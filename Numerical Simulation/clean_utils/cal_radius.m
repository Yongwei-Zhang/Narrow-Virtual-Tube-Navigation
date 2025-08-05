function [r] = cal_radius(arcl_ep1, arcl_ep3, l_i)
    % set tube radius

    l1 = arcl_ep1(1);
    l2 = arcl_ep1(2);
    l3 = arcl_ep1(3);
    l4 = arcl_ep1(4);
    l5 = arcl_ep3(1);
    l6 = arcl_ep3(2);
    l7 = arcl_ep3(3); 
    
    r_max = 5;
    a = 5;
    r_min1 = 0.8;
    r_min2 = 0.8;
    
    if l_i >= l1 && l_i <= l2
        r = r_max;
    elseif l_i > l2 && l_i <= l3
        r = set_radius(l2, l3, r_max, r_min1, l_i); 
    elseif l_i > l3 && l_i <= l4
        r = set_radius(l3, l4, a, r_min1, -l_i+l3+l4);
    elseif l_i > l4 && l_i <= l5
        r = a;
    elseif l_i > l5 && l_i <= l6
        r = set_radius(l5, l6, a, r_min2, l_i); 
    else
        r = set_radius(l6, l7, r_max, r_min2, -l_i+l6+l7);
    end

end