function [u2] = cal_u2(k2,d_ij,d1,d2,p_ij,epsl_m,rs,x1,x2,epsl_s)

    fm = ((1+epsl_m)*d_ij - 2*rs*s_function(d_ij,rs,x1,x2,epsl_s,0))^2;
    
    fz1 = sigma(d_ij,d1,d2,1) * ( (1+epsl_m)*d_ij-2*rs*s_function(d_ij,rs,x1,x2,epsl_s,0) );
    fz2 = sigma(d_ij,d1,d2,0) * ( (1+epsl_m)-2*rs*s_function(d_ij,rs,x1,x2,epsl_s,1) );
    fz = fz1 - fz2;
    
    term1 = k2*fz/fm;
    
    term2 = 1/d_ij;
    
    term3_1 = p_ij(1);
    term3_2 = p_ij(2);
    
    u2 = -[term1*term2*term3_1, term1*term2*term3_2]; 

end