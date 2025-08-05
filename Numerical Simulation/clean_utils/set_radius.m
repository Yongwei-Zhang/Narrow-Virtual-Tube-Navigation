function [r] = set_radius(d1,d2,r_max,r_min,x)

    A = -2/(d1-d2)^3;
    B = 3*(d1+d2)/(d1-d2)^3;
    C = -6*d1*d2/(d1-d2)^3;
    D = d2^2*(3*d1-d2)/(d1-d2)^3;
    
    r = (r_max-r_min)*(A*x^3 + B*x^2 + C*x + D) + r_min;

end