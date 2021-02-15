function [x_all,y_all] = four_points(x,y,ang,length,width)

a = length;
b = width;
ang = ang/180.*pi;
x1_i = x - 0.5*b * cos(ang);
x2_i = x + 0.5*b * cos(ang);
x3_i = x - a*sin(ang) + 0.5*b * cos(ang);
x4_i = x - a*sin(ang) - 0.5*b * cos(ang);
y1_i = y + 0.5*b * sin(ang);
y2_i = y - 0.5*b * sin(ang);
y3_i = y - a*cos(ang) - 0.5*b * sin(ang);
y4_i = y - a*cos(ang) + 0.5*b * sin(ang);

x_all = [x1_i,x2_i,x3_i,x4_i];
y_all = [y1_i,y2_i,y3_i,y4_i];
end

