function gn_n = gn_func(crt_x, crt_y, crt_n_x, crt_n_y, gn, map)
if ((crt_n_x-crt_x == 1) && (crt_n_y-crt_y == 1))||((crt_n_x-crt_x == -1) && (crt_n_y-crt_y == -1)) ||((crt_n_x-crt_x == -1) && (crt_n_y-crt_y == 1))||((crt_n_x-crt_x == 1) && (crt_n_y-crt_y ==-1))
    gn_n= round(gn + map(crt_n_x, crt_n_y) + 0.4, 1);
else
    gn_n=round(gn+map(crt_n_x, crt_n_y), 1);
end