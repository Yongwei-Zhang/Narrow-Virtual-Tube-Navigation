function [value2] = s_function(x,rs,x1,x2,epsl_s,label)

    if label == 0 
        if x >= 0 && x <= 2*rs*x1
            value2 = x/(2*rs);
        elseif x >= 2*rs*x2
            value2 = 1;
        else
            value2 = (1-epsl_s) + sqrt(epsl_s^2-(x/(2*rs)-x2)^2);
        end
    else
        if x >= 0 && x <= 2*rs*x1
            value2 = 1/(2*rs);
        elseif x >= 2*rs*x2
            value2 = 0;
        else
            value2 = -(x-2*rs*x2)/( 4*rs^2 * sqrt(epsl_s^2-(x/(2*rs)-x2)^2) );
        end
    end
        
end