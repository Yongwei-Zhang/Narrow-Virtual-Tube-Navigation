function [rho_d_grad] = cal_rho_d_grad(A,radius_dert,lIndex,l,position_i,delta,itval,x,y)

    coef = 1/A * radius_dert(lIndex);
    
    lIndex1 = lIndex - itval;
    lIndex2 = lIndex + itval;
    
    nb_x_position = [position_i(1) + delta, position_i(2)];
    nb_x_lIndex = cal_nb_lIndex(nb_x_position, x, y, lIndex1, lIndex2);
    pt_x = (l(nb_x_lIndex) - l(lIndex))/delta;
    
    nb_y_position = [position_i(1), position_i(2)+delta];
    nb_y_lIndex = cal_nb_lIndex(nb_y_position, x, y, lIndex1, lIndex2);
    pt_y = (l(nb_y_lIndex) - l(lIndex))/delta;
    
    rho_d_grad = coef * [pt_x, pt_y];

end
