function [ net_index ] = ad_hoc( X, result,Vmax1,Vmax2 )
%clc; clear;
%return the index of those who are too far to have conflict with the current
%group, i.e. even full acceleration will not collide.

%tic
% Vmax1 = 12;
% Vmax2 = 20;
num = length(X);
interest = zeros(2,num); sum_vmax = 0;
on_circuit = zeros(1,num);
interest(2,:) = ones(1,num)*600;
%prior = zeros(1,num);
R = zeros(1,16); C = zeros(1,16); W = ones(1,16);
names = {'V1' 'V2' 'V3' 'V4' 'V5' 'V6' 'V7' 'V8'};
%sum_father_equal_max = 0;
for i = 1:2:num-1
    if result(i) == Vmax1
        names(i) = cellstr(['V',num2str(i),'==Vmax']);
    end
end
for i = 2:2:num
    if result(i) == Vmax2
        names(i) = cellstr(['V',num2str(i),'==Vmax']);
    end
end
%R(num+1:2*num) = 1:num; C(num+1:2*num) = 1:num;

delete_index = zeros(1,num);
%flag = ones(1,num);
V = result(1:num); B = result(num+1:length(result));
%B14 = B(1); B21 = B(2); B32 = B(3); B43 = B(4);
B18 = B(1);  B17 = B(2);  B16 = B(3);  B13 = B(4);  B28 = B(5); B25 = B(6); B24 = B(7); B23 = B(8); B38 = B(9);  B35 = B(10); B47 = B(11); B46 = B(12); B45 = B(13); B57  = B(14); B67 = B(15); B68 = B(16); 
%B10 = B14; B54 = B14;
%% build the directed graph
if B18 == 1,  R(1) = 1; C(1) = 8; else, R(1) = 8; C(1) = 1; end
if B17 == 1,  R(2) = 1; C(2) = 7; else, R(2) = 7; C(2) = 1; end
if B16 == 1,  R(3) = 1; C(3) = 6; else, R(3) = 6; C(3) = 1; end
if B13 == 1,  R(4) = 1; C(4) = 3; else, R(4) = 3; C(4) = 1; end

if B28 == 1,  R(5) = 2; C(5) = 8; else, R(5) = 8; C(5) = 2; end
if B25 == 1,  R(6) = 2; C(6) = 5; else, R(6) = 5; C(6) = 2; end
if B24 == 1,  R(7) = 2; C(7) = 4; else, R(7) = 4; C(7) = 2; end
if B23 == 1,  R(8) = 2; C(8) = 3; else, R(8) = 3; C(8) = 2; end

if B38 == 1,  R(9) = 3; C(9) = 8; else, R(9) = 8; C(9) = 3; end
if B35 == 1,  R(10) = 3; C(10) = 5; else, R(10) = 5; C(10) = 3; end
if B47 == 1,  R(11) = 4; C(11) = 7; else, R(11) = 7; C(11) = 4; end
if B46 == 1,  R(12) = 4; C(12) = 6; else, R(12) = 6; C(12) = 4; end

if B45 == 1,  R(13) = 4; C(13) = 5; else, R(13) = 5; C(13) = 4; end
if B57 == 1,  R(14) = 5; C(14) = 7; else, R(14) = 7; C(14) = 5; end
if B67 == 1,  R(15) = 6; C(15) = 7; else, R(15) = 7; C(15) = 6; end
if B68 == 1,  R(16) = 6; C(16) = 8; else, R(16) = 8; C(16) = 6; end



Ga=sparse([R,8,1],[C,1,8],[W,0,0]);
Gb=digraph(R,C,[],names);
Gc=flipedge(Gb);

for i = 1:num
    fatherset = predecessors(Gb,i);
    childset = graphtraverse(Ga,i);
    [~,number_of_intersect] = size(intersect(fatherset',childset));
    if number_of_intersect > 0
       on_circuit(i) = 1;
    end
end
    
% view(biograph(Ga,[ ],'ShowW','ON'));
% figure
% plot(Gb)
% set(gcf,'unit','centimeters','position',[15 12 11 10]);
% axis off
%% necessary and sufficient condition of deleting a node

for i = 1:num
%% 
    if mod(i,2)==1
        if V(i) == Vmax1
            interest(1,i)=i;
            interest(2,i)=X(i); 
            sum_vmax = sum_vmax + 1;
        end
    else
        if V(i) == Vmax2
            interest(1,i)=i;
            interest(2,i)=X(i);
            sum_vmax = sum_vmax + 1;
        end
    end
end
    
if sum_vmax ~= 1 

    %% 
    for i = find(on_circuit==0) 
        if interest(1,i) == i
            if indegree(Gb,i) > 0
                delete_index(i)=1;
            end
        end
    end
    %%
    for i = find(on_circuit==0) 
        if interest(1,i) ~= i 
            %fatherset = predecessors(Gb,i);
            fatherset = dfsearch(Gc,i);
            
            sum_father_equal_max = 0;
            for j = fatherset
                if indegree(Gb,j) == 0
                    sum_father_equal_max = sum_father_equal_max + 1;
                end
            end
            
            if sum_father_equal_max > 1
                delete_index(i)=1;
            end
        end
    end
    %% 
    for i = find(on_circuit==1) 
        
        fatherset = predecessors(Gb,i);
        [~,num_of_common] = setdiff(fatherset,find(on_circuit==1));
        if num_of_common ~= 0
            delete_index(i)=1;
        end
    end
    %%
    for i = find(delete_index)
        for j = graphtraverse(Ga,i)
            delete_index(j)=1;
        end
    end
    
    
    
end

net_index = delete_index == 0;


end