function [value1] = sigma(x,d1,d2,label)

    A = -2/(d1-d2)^3;
    B = 3*(d1+d2)/(d1-d2)^3;
    C = -6*d1*d2/(d1-d2)^3;
    D = d2^2*(3*d1-d2)/(d1-d2)^3;
    
    if label == 0
        if x <= d1
            value1 = 1;
        elseif x >= d2
            value1 = 0;
        else
            value1 = A*x^3 + B*x^2 + C*x + D;
        end
    else
        if x <= d1
            value1 = 0;
        elseif x >= d2
            value1 = 0;
        else
            value1 = 3*A*x^2 + 2*B*x + C;
        end
    end

end