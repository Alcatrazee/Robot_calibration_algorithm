norm_1995 = cali_1995;
norm_2009 = cali_2009;
norm_2016 = cali_2016;

l95 = length(norm_1995);
l09 = length(norm_2009);
l16 = length(norm_2016);

num = [l95 l09 l16];
[val,index] = min(num);

new_arr = [norm_1995(1:val);norm_2009(1:val);norm_2016(1:val)]';

%%
close all
figure('Name','algorithm comparison','position',[500 100 1280 800])
bar(new_arr)
for i = 1:val
    text(i-0.35,new_arr(i,1)+5,num2str(new_arr(i,1),'%3.2f'),'fontsize',10);
    text(i-0.09,new_arr(i,2)+5,num2str(new_arr(i,2),'%3.2f'),'fontsize',10);
    text(i+0.1,new_arr(i,3)+5,num2str(new_arr(i,3),'%3.2f'),'fontsize',10);
end
legend('POE result','MPOE result','ACS result')
title('Comparison of three algorithms')