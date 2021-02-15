function [min_dis] = collision_check(info_all)

%IDs = keys(info_all{1});
num = size(info_all,2);

min_dis = 32;

if num <= 1
    min_dis = 32;
else
    for i = 1:num-1
        for j = i+1:num
            vehi_pos = info_all{i}('0x42');    vehi_ang = info_all{i}('0x43');
            vehj_pos = info_all{j}('0x42');    vehj_ang = info_all{j}('0x43');
            dis = check_2_rectangle(vehi_pos,vehi_ang,vehj_pos,vehj_ang);
            if dis <= min_dis
                min_dis = dis;
            end
        end
    end
end
end

