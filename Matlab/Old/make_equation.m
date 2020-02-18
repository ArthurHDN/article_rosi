    syms x y t c_x c_y default_r_x default_r_y a b
    r_y = default_r_y + b*t;
    r_x = default_r_x + a*t;
    fi = ((x + c_x)./r_x).^4 + ((y + c_y)./r_y).^4 - 1;
    diff(fi, t)