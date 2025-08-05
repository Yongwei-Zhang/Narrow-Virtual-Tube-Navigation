function [value2] = s_function2(x,rs,x1,x2,epsl_s,epsl_t,label)

    if label == 0 
        if x >= 0 && x <= rs*x1
            value2 = x/rs;
        elseif x >= rs*x2
            value2 = 1;
        else
            value2 = (1-epsl_t) + sqrt(epsl_s^2-(x/rs-x2)^2);
        end
    else
        if x >= 0 && x <= rs*x1
            value2 = 1/rs;
        elseif x >= rs*x2
            value2 = 0;
        else
            value2 = -(x-rs*x2)/( rs^2 * sqrt(epsl_s^2-(x/rs-x2)^2) );
        end
    end

end