function [rho_d_grad] = cal_rho_d_grad(A,radius_dert,lIndex,l,position_i,delta,x,y,m)

    C = l(m); % The perimeter of an ellipse
    coef = 1/A * radius_dert(lIndex);
    
    %% neighbor of x-direction
    nb_x_position = [position_i(1) + delta, position_i(2)];
    nb_x_lIndex = cal_lIndex(nb_x_position, x, y, m);
    l_nb_x = l(nb_x_lIndex);
    l_position_i = l(lIndex);
    
    lIndex_limit = 9/10 * (m-1); 
    lIndex_dis_x = abs(nb_x_lIndex - lIndex);
    
    if lIndex_dis_x > lIndex_limit 
        if l_nb_x < C/2 
            pt_x = (C - abs(l_nb_x - l_position_i))/delta;
        else
            pt_x = -(C - abs(l_nb_x - l_position_i))/delta;
        end    
    else 
        pt_x = (l_nb_x - l_position_i)/delta; 
    end
    
    %% neighbor of y-direction
    nb_y_position = [position_i(1), position_i(2)+delta];
    nb_y_lIndex = cal_lIndex(nb_y_position, x, y, m);
    l_nb_y = l(nb_y_lIndex);
    lIndex_dis_y = abs(nb_y_lIndex - lIndex);
    
    if lIndex_dis_y > lIndex_limit
        if l_nb_y < C/2 
            pt_y = (C - abs(l_nb_y - l_position_i))/delta;
        else
            pt_y = -(C - abs(l_nb_y - l_position_i))/delta;
        end    
    else 
        pt_y = (l_nb_y - l_position_i)/delta;
    end
    
    %% return value
    rho_d_grad = coef * [pt_x, pt_y];

end
