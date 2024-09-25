function [yp1, yp2, yp3, yp4] = dydt(t, y)
    y1 = y(1);
    y2 = y(2);
    y3 = y(3);
    y4 = y(4);
    yp1 = y4;
    yp2 = y3;
    yp3 = -y1;
    yp4 = -y2*y1+t;
end