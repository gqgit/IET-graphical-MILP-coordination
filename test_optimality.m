Vmin = 10;
Vmax1 = 20;
Vmax2 = 20;
font_size = 13;


num = 200;
difference = zeros(1,num);
data = zeros(8,num);
for i =1:num
    len = 150 + 50*rand(1,8);
    result = milp(len,Vmin,Vmax1,Vmax2);
    formal_net = ad_hoc(len,result,Vmax1,Vmax2);
    X = len .* formal_net;
    new = milp(X,Vmin,Vmax1,Vmax2);
    data(:,i) = len';
    
    %difference(i) = norm(new(1:8)'.*formal_net-result(1:8)'.*formal_net,Inf);
    difference(i) = sum(new(1:8)'.*formal_net)/sum(formal_net)-sum(result(1:8)'.*formal_net)/sum(formal_net);
%     if difference(i) >= 0.1
%         new(1:8)'.*formal_net
%         result(1:8)'.*formal_net
%     end
end

stem(difference,'filled','Marker','o','MarkerSize',3);
%bar(difference,'EdgeColor','none');
grid on
set(gcf,'unit','centimeters','position',[15 10 20 9]);
ylabel('loss of average optimal velocity (m/s)','FontSize',font_size,'Fontname', 'Times New Roman');
xlabel('simulation times','FontSize',font_size,'Fontname', 'Times New Roman');