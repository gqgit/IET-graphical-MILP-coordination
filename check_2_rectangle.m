function [min_distance] = check_2_rectangle(vehi_pos,vehi_ang,vehj_pos,vehj_ang)

a = 4.;
b = 1.6;
vehi_ang = vehi_ang/180.*pi;
vehj_ang = vehj_ang/180.*pi;
x1_i = vehi_pos(1) - 0.5*b * cos(vehi_ang);
x2_i = vehi_pos(1) + 0.5*b * cos(vehi_ang);
x3_i = vehi_pos(1) - a*sin(vehi_ang) + 0.5*b * cos(vehi_ang);
x4_i = vehi_pos(1) - a*sin(vehi_ang) - 0.5*b * cos(vehi_ang);
y1_i = vehi_pos(2) + 0.5*b * sin(vehi_ang);
y2_i = vehi_pos(2) - 0.5*b * sin(vehi_ang);
y3_i = vehi_pos(2) - a*cos(vehi_ang) - 0.5*b * sin(vehi_ang);
y4_i = vehi_pos(2) - a*cos(vehi_ang) + 0.5*b * sin(vehi_ang);

x1_j = vehj_pos(1) - 0.5*b * cos(vehj_ang);
x2_j = vehj_pos(1) + 0.5*b * cos(vehj_ang);
x3_j = vehj_pos(1) - a*sin(vehj_ang) + 0.5*b * cos(vehj_ang);
x4_j = vehj_pos(1) - a*sin(vehj_ang) - 0.5*b * cos(vehj_ang);
y1_j = vehj_pos(2) + 0.5*b * sin(vehj_ang);
y2_j = vehj_pos(2) - 0.5*b * sin(vehj_ang);
y3_j = vehj_pos(2) - a*cos(vehj_ang) - 0.5*b * sin(vehj_ang);
y4_j = vehj_pos(2) - a*cos(vehj_ang) + 0.5*b * sin(vehj_ang);

coor_i = [x1_i y1_i;
          x2_i y2_i;
          x3_i y3_i;
          x4_i y4_i];
coor_j = [x1_j y1_j;
          x2_j y2_j;
          x3_j y3_j;
          x4_j y4_j];
min_dis = zeros(1,4);
for i = 1:4
    vec1 = coor_j(1,:) - coor_i(i,:);
    vec2 = coor_j(2,:) - coor_i(i,:);
    vec3 = coor_j(3,:) - coor_i(i,:);
    vec4 = coor_j(4,:) - coor_i(i,:);
    
    a1 = acos(  vec1*vec2' / (norm(vec1)*norm(vec2))  );
    a2 = acos(  vec2*vec3' / (norm(vec2)*norm(vec3))  );
    a3 = acos(  vec3*vec4' / (norm(vec3)*norm(vec4))  );
    a4 = acos(  vec4*vec1' / (norm(vec4)*norm(vec1))  );
    
    if abs(a1+a2+a3+a4 - 2*pi) > 0.001*2*pi 
        min_dis(i) = min([norm(vec1),norm(vec2),norm(vec3),norm(vec4)]);
    else
        min_dis(i) = -min([norm(vec1),norm(vec2),norm(vec3),norm(vec4)]);
    end
end

min_distance = min(min_dis);
        


end

