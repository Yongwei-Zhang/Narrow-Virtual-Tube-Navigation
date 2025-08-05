function [u3] = cal_u3(k3,d_it,d_im,tube_m_x,tube_m_y,position,i,epsl_t,rs,x1,x2,epsl_s,ra)

    d1 = rs;
    d2 = ra;
    
    fm = ( (1+epsl_t)*d_it - rs*s_function2(d_it,rs,x1,x2,epsl_s,epsl_t,0) )^2;
    
    fz1 = sigma2(d_it,d1,d2,1) * ( (1+epsl_t)*d_it-rs*s_function2(d_it,rs,x1,x2,epsl_s,epsl_t,0) );
    fz2 = sigma2(d_it,d1,d2,0) * ( (1+epsl_t)-rs*s_function2(d_it,rs,x1,x2,epsl_s,epsl_t,1) );
    fz = fz1 - fz2;
    
    term1 = k3*fz/fm;
    
    if d_im == 0
        term2_1 = 0;
        term2_2 = 0;
    else
        term2_1 = -( position{i}(1)-tube_m_x )/d_im;
        term2_2 = -( position{i}(2)-tube_m_y )/d_im;
    end
    
    u3 = -[term1*term2_1, term1*term2_2];

end