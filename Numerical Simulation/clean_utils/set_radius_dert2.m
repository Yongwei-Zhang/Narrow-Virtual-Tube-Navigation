function [dert_radius] = set_radius_dert2(d1,d2,r_max,r_min,l_i)

    A = -2/(d1-d2)^3;
    B = 3*(d1+d2)/(d1-d2)^3;
    C = -6*d1*d2/(d1-d2)^3;
    
    dert_radius = (r_max-r_min)*(-3*A*(-l_i+d1+d2)^2 - 2*B*(-l_i+d1+d2) - C);

end