function [dert_radius] = cal_radius_dert(arcl_ep1, arcl_ep3, l_i)

    l1 = arcl_ep1(1);
    l2 = arcl_ep1(2);
    l3 = arcl_ep1(3);
    l4 = arcl_ep1(4);
    l5 = arcl_ep3(1);
    l6 = arcl_ep3(2);
    l7 = arcl_ep3(3); 
    
    r_max = 5;
    r_min = 1;
    a = 5;
    
    if l_i >= l1 && l_i <= l2
        dert_radius = 0;
    elseif l_i > l2 && l_i <= l3
        dert_radius = set_radius_dert1(l2, l3, r_max, r_min, l_i); 
    elseif l_i > l3 && l_i <= l4
        dert_radius = set_radius_dert2(l3, l4, a, r_min, l_i);
    elseif l_i > l4 && l_i <= l5 
        dert_radius = 0;
    elseif l_i > l5 && l_i <= l6
        dert_radius = set_radius_dert1(l5, l6, a, r_min, l_i); 
    else
        dert_radius = set_radius_dert2(l6, l7, r_max, r_min, l_i);
    end

end