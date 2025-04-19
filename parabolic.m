function [a, b, c] = parabolic(points, i, f)
    xi = i(1);
    yi = i(2);
    xf = f(1);
    yf = f(2);
    tolerance = 1e-6;
    for a = -100000:0.1:100000
        A = [xi, 1; xf, 1];
        B = [yi - a*xi^2; yf - a*xf^2];
        coeff = A \ B;
        b = coeff(1);
        c = coeff(2);
        all_fail = true;
        for j = 1:length(points)
            x = points(j, 1);
            y = points(j, 2);
            y_calc = a*x^2+b*x+c;
            if y_calc==y
                all_fail = false;
                break;  
            end
        end
        if all_fail
            return  % Return the valid n
        end
    end
    
error('No suitable n found.')  % This will be triggered if no n is found
end
