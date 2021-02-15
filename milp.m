function [ result ] = milp( L, Vmin, Vmax1, Vmax2)

%C1 = 2;
M = 100000;
%safe_coef = 2;
A = 4;
W = 1.6;


Le0 = 0.0 * A + 1.0 * W;
Ls0 = 1.0 * A + 1.0 * W;
Le1 = 0.0 * A + 1.5 * W;
Ls1 = 1.0 * A + 1.5 * W;
Le2 = 0.0 * A + 1.5 * W;
Ls2 = 1.0 * A + 1.5 * W;


result = zeros(1,24);
solver = 'matlab';
%solver = 'gurobi';
VL=Vmin;
U1=Vmax1;
U2=Vmax2;

%%
lanewid = 4;
lanenum = 3;
radius = 4;

R = radius + (lanenum+0.5) * lanewid;
stop_dis = radius + lanenum * lanewid;
const1 = radius + (1+0.5) * lanewid;
const2 = stop_dis;
%% 
for i = 1 : length(L)
    if L(i) == 0 
        L(i) = max(L)+500;
    end
end

L1 = L(1); L2 = L(2); L3 = L(3); L4 = L(4); L5 = L(5); L6 = L(6); L7 = L(7); L8 = L(8);
L1_18 = L1 - stop_dis + R * asin(  const1/R  );
L1_17 = L1 - stop_dis + R * asin(  const2/R  );
L1_16 = L1 - stop_dis + R * acos(  const1/R  );
L1_13 = L1 - stop_dis + R * acos(  const2/R  );

L2_28 = L2 - 6;
L2_25 = L2 - stop_dis + (2*stop_dis - sqrt(R^2 - const1^2));
L2_24 = L2 + 6;
L2_23 = L2 - stop_dis + sqrt(R^2 - const1^2);

L3_13 = L3 - stop_dis + R * asin(  const2/R  );
L3_23 = L3 - stop_dis + R * asin(  const1/R  );
L3_38 = L3 - stop_dis + R * acos(  const1/R  );
L3_35 = L3 - stop_dis + R * acos(  const2/R  );

L4_24 = L4 - 6;
L4_47 = L4 - stop_dis + (2*stop_dis - sqrt(R^2 - const1^2));
L4_46 = L4 + 6;
L4_45 = L4 - stop_dis + sqrt(R^2 - const1^2);

L5_25 = L5 - stop_dis + R * acos(  const1/R  );
L5_35 = L5 - stop_dis + R * asin(  const2/R  );
L5_45 = L5 - stop_dis + R * asin(  const1/R  );
L5_57 = L5 - stop_dis + R * acos(  const2/R  );

L6_16 = L6 - stop_dis + (2*stop_dis - sqrt(R^2 - const1^2));
L6_46 = L6 - 6;
L6_67 = L6 - stop_dis + sqrt(R^2 - const1^2);
L6_68 = L6 + 6;

L7_17 = L7 - stop_dis + R * acos(  const2/R  );
L7_47 = L7 - stop_dis + R * acos(  const1/R  );
L7_57 = L7 - stop_dis + R * asin(  const2/R  );
L7_67 = L7 - stop_dis + R * asin(  const1/R  );

L8_18 = L8 - stop_dis + sqrt(R^2 - const1^2);
L8_28 = L8 + 6;
L8_38 = L8 - stop_dis + (2*stop_dis - sqrt(R^2 - const1^2));
L8_68 = L8 - 6;


f = [-1;-1;-1;-1;-1;-1;-1;-1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
      %V1               V2               V3               V4              V5               V6               V7               V8        B18   B17   B16   B13   B28   B25   B24   B23   B38   B35    B47   B46   B45    B57   B67    B68 
A=[   
    Le1- L8_18          0                0                0               0                0                0           Ls1+ L1_18     M     0     0     0     0     0     0     0     0     0      0     0     0      0     0      0;
    Ls1+ L8_18          0                0                0               0                0                0           Le1- L1_18    -M     0     0     0     0     0     0     0     0     0      0     0     0      0     0      0;% veh1 - veh8 conflict
       
    Le2- L7_17          0                0                0               0                0            Ls2+ L1_17           0         0     M     0     0     0     0     0     0     0     0      0     0     0      0     0      0;
    Ls2+ L7_17          0                0                0               0                0            Le2- L1_17           0         0    -M     0     0     0     0     0     0     0     0      0     0     0      0     0      0;% veh1 - veh7 conflict
          
    Le1- L6_16          0                0                0               0            Ls1+ L1_16           0                0         0     0     M     0     0     0     0     0     0     0      0     0     0      0     0      0;
    Ls1+ L6_16          0                0                0               0            Le1- L1_16           0                0         0     0    -M     0     0     0     0     0     0     0      0     0     0      0     0      0;% veh1 - veh6 conflict
          
    Le2- L3_13          0           Ls2+ L1_13            0               0                0                0                0         0     0     0     M     0     0     0     0     0     0      0     0     0      0     0      0;
    Ls2+ L3_13          0           Le2- L1_13            0               0                0                0                0         0     0     0    -M     0     0     0     0     0     0      0     0     0      0     0      0;% veh1 - veh3 conflict
    
    
    
       0           Le0- L8_28            0                0               0                0                0           Ls0+ L2_28     0     0     0     0     M     0     0     0     0     0      0     0     0      0     0      0;
       0           Ls0+ L8_28            0                0               0                0                0           Le0- L2_28     0     0     0     0    -M     0     0     0     0     0      0     0     0      0     0      0;% veh2 - veh8 conflict
       
       0           Le1- L5_25            0                0           Ls1+ L2_25           0                0                0         0     0     0     0     0     M     0     0     0     0      0     0     0      0     0      0;
       0           Ls1+ L5_25            0                0           Le1- L2_25           0                0                0         0     0     0     0     0    -M     0     0     0     0      0     0     0      0     0      0;% veh2 - veh5 conflict
          
       0           Le0- L4_24            0           Ls0+ L2_24           0                0                0                0         0     0     0     0     0     0     M     0     0     0      0     0     0      0     0      0;
       0           Ls0+ L4_24            0           Le0- L2_24           0                0                0                0         0     0     0     0     0     0    -M     0     0     0      0     0     0      0     0      0;% veh2 - veh4 conflict
          
       0           Le1- L3_23        Ls1+ L2_23           0               0                0                0                0         0     0     0     0     0     0     0     M     0     0      0     0     0      0     0      0;
       0           Ls1+ L3_23        Le1- L2_23           0               0                0                0                0         0     0     0     0     0     0     0    -M     0     0      0     0     0      0     0      0;% veh2 - veh3 conflict
    
    
     
       
       0                0            Le1- L8_38           0               0                0                0           Ls1+ L3_38     0     0     0     0     0     0     0     0     M     0      0     0     0      0     0      0;
       0                0            Ls1+ L8_38           0               0                0                0           Le1- L3_38     0     0     0     0     0     0     0     0    -M     0      0     0     0      0     0      0;% veh3 - veh8 conflict
       
       0                0            Le2- L5_35           0          Ls2+ L3_35            0                0                0         0     0     0     0     0     0     0     0     0     M      0     0     0      0     0      0;
       0                0            Ls2+ L5_35           0          Le2- L3_35            0                0                0         0     0     0     0     0     0     0     0     0    -M      0     0     0      0     0      0;% veh3 - veh5 conflict
          
       0                0                0            Le1- L7_47          0                0            Ls1+ L4_47           0         0     0     0     0     0     0     0     0     0     0      M     0     0      0     0      0;
       0                0                0            Ls1+ L7_47          0                0            Le1- L4_47           0         0     0     0     0     0     0     0     0     0     0     -M     0     0      0     0      0;% veh4 - veh7 conflict
          
       0                0                0            Le0- L6_46          0           Ls0+ L4_46            0                0         0     0     0     0     0     0     0     0     0     0      0     M     0      0     0      0;
       0                0                0            Ls0+ L6_46          0           Le0- L4_46            0                0         0     0     0     0     0     0     0     0     0     0      0    -M     0      0     0      0;% veh4 - veh6 conflict
       
       
       
       
       0                0                0            Le1- L5_45     Ls1+ L4_45            0                0                0         0     0     0     0     0     0     0     0     0     0      0     0     M      0     0      0;
       0                0                0            Ls1+ L5_45     Le1- L4_45            0                0                0         0     0     0     0     0     0     0     0     0     0      0     0    -M      0     0      0;% veh4 - veh5 conflict
       
       0                0                0                0          Le2- L7_57            0            Ls2+ L5_57           0         0     0     0     0     0     0     0     0     0     0      0     0     0      M     0      0;
       0                0                0                0          Ls2+ L7_57            0            Le2- L5_57           0         0     0     0     0     0     0     0     0     0     0      0     0     0     -M     0      0;% veh5 - veh7 conflict
          
       0                0                0                0               0           Le1- L7_67        Ls1+ L6_67           0         0     0     0     0     0     0     0     0     0     0      0     0     0      0     M      0;
       0                0                0                0               0           Ls1+ L7_67        Le1- L6_67           0         0     0     0     0     0     0     0     0     0     0      0     0     0      0    -M      0;% veh6 - veh7 conflict
          
       0                0                0                0               0           Le0- L8_68            0            Ls0+ L6_68    0     0     0     0     0     0     0     0     0     0      0     0     0      0     0      M;
       0                0                0                0               0           Ls0+ L8_68            0            Le0- L6_68    0     0     0     0     0     0     0     0     0     0      0     0     0      0     0     -M;% veh6 - veh8 conflict
       
    ];
b  = [ M;  0;  M;  0;  M;  0;  M;  0;  M;  0;  M;  0;  M;  0;  M;  0;  M;  0;  M;  0;  M;  0;  M;  0; M;  0;  M;  0;  M;  0;  M;  0];
lb = [VL; VL; VL; VL; VL; VL; VL; VL;       0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0;  0];
ub = [U1; U2; U1; U2; U1; U2; U1; U2;       1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1;  1];


if strcmp(solver, 'matlab')
    options = optimoptions('intlinprog','Display','off');
    %tic
    result = intlinprog(f, [9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24], A, b, [], [], lb, ub,[],options);
    
    for i = 9:24
        if abs(result(i)-1) < 0.01
            result(i) = 1;
        end
    end
else
    model.obj = f';
    A_lb = zeros(24,24); A_ub = A_lb;
    b_lb = zeros(24,1);b_ub = b_lb;
    for i = 1:8
        A_lb(i,i) =-1;
        A_ub(i,i) = 1;
        b_lb(i) =-lb(i);%-x<=-lower_limit
        b_ub(i) = ub(i);%x<=upper_limit
    end
    for i = 1 : 24
        if i >= 9
            model.vtype(i) = 'B';
        else
            model.vtype(i) = 'C';
        end
    end
    model.A = sparse([A;A_lb;A_ub]);
    model.rhs = [b;b_lb;b_ub];
    model.sense = '<';
    model.modelsense = 'min';
    params.outputflag = 0;
    %tic
    result_gurobi = gurobi(model, params);
    for i = 1 : 24
        result(i) = result_gurobi.x(i);
    end
end

for i = 1:2:7
    if abs(result(i)-U1) < 0.001
        result(i) = U1;
    end
end
for i = 2:2:8
    if abs(result(i)-U2) < 0.001
        result(i) = U2;
    end
end
%toc
end