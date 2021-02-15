function [ flag ] = inter_loop_check(t_arrive_current, t_arrive_last, V_current, V_last, index)

global conflict
conflict = [1,3,6,7,8;
            2,3,4,5,8;
            3,1,2,5,8;
            4,2,5,6,7;
            5,2,3,4,7;
            6,1,4,7,8;
            7,1,4,5,6;
            8,1,2,3,6];
global delta_L
delta_L = [pi/2*18,       0.476*18,      0.98*18,     1.095*18,       0.59*18;
           32,              14.97,         22,        32-14.97,          10;  
           pi/2*18,       1.095*18,     0.59*18,      0.476*18,       0.98*18;
           32,                10,        14.97,          22,          32-14.97;
           pi/2*18,        0.98*18,     1.095*18,      0.59*18,       0.476*18;
           32,             32-14.97,       10,          14.97,            22;
           pi/2*18,        0.476*18,    0.98*18,      1.095*18,       0.59*18;
           32,              14.97,         22,        32-14.97,           10;
    ];

flag = 0;

for i = index
    for k = 1:5
        j = conflict(i,k);
        if t_arrive_current(i) + (delta_L(i,k) - 4 )/V_current(i) < ...
                t_arrive_last(j) + (delta_L(j,find(conflict(j,:)==i)) + 4)/V_last(j)
            flag = 1;
            break
        end
    end
    if flag == 1
        break
    end
end

end


