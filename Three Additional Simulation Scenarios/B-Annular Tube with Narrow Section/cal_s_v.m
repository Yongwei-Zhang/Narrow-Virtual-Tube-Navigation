function [saturated_velocity] =  cal_s_v(v_x, v_y, v_max)

    saturated_velocity = zeros(1,2); 
    value = sqrt(v_x^2+v_y^2); 
    
    unit_x = v_x/value; 
    unit_y = v_y/value; 
    
    if value >= v_max
        saturated_velocity(1) = v_max*unit_x;
        saturated_velocity(2) = v_max*unit_y;
    else
        saturated_velocity(1) = v_x;
        saturated_velocity(2) = v_y;
    end

end