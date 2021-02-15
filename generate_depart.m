function [depart] = generate_depart(flux,number)

flux_ratio = 1.;
mean_interval = 3600/flux;
ratio = 5;
interval_direct = exprnd(mean_interval,[4,number*ratio]);
interval_corner = exprnd(mean_interval*flux_ratio,[4,number*ratio]);

depart = zeros(8,number);
intvl_real = zeros(8,number);
for i = 1:4
    depart(i*2,1)=interval_direct(i,1);
    depart(i*2-1,1)=interval_corner(i,1);
    intvl_real(i*2,1)=interval_direct(i,1);
    intvl_real(i*2-1,1)=interval_corner(i,1);
end
k=2*ones(1,4);
for i = 1:4
    for j = 1:number*ratio
        if interval_direct(i,j) >= 1
            depart(i*2,k(i)) = depart(i*2,k(i)-1) + interval_direct(i,j);
            intvl_real(i*2,k(i)) = interval_direct(i,j);
            k(i) = k(i)+1;
            if k(i) >= number+1
                break
            end
        end
    end
end

k=2*ones(1,4);
for i = 1:4
    for j = 1:number*ratio
        if interval_corner(i,j) >= 1
            depart(i*2-1,k(i)) = depart(i*2-1,k(i)-1) + interval_corner(i,j);
            intvl_real(i*2-1,k(i)) = interval_corner(i,j);
            k(i) = k(i)+1;
            if k(i) >= number+1
                break
            end
        end
    end
end

if ismember(0,depart)
    fprintf(2,'--------------wrong depart time!----------\n please increase ratio in generate_depart.m!')
    return;
end


fprintf(1,'---------flux1 is %f---------\n',3600/mean(intvl_real(1,:)));
fprintf(1,'---------flux2 is %f---------\n',3600/mean(intvl_real(2,:)));
fprintf(1,'---------flux3 is %f---------\n',3600/mean(intvl_real(3,:)));
fprintf(1,'---------flux4 is %f---------\n',3600/mean(intvl_real(4,:)));
fprintf(1,'---------flux5 is %f---------\n',3600/mean(intvl_real(5,:)));
fprintf(1,'---------flux6 is %f---------\n',3600/mean(intvl_real(6,:)));
fprintf(1,'---------flux7 is %f---------\n',3600/mean(intvl_real(7,:)));
fprintf(1,'---------flux8 is %f---------\n',3600/mean(intvl_real(8,:)));



