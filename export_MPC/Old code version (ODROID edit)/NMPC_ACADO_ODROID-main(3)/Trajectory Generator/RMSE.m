function [rmse] = RMSE(ref,actual)
%RMSE Summary of this function goes here
%   Detailed explanation goes here

min_int_ref = 0;
min_int_actual = 0;
min_int_ref_save = [];
min_int_actual_save = [];

for i=1:3001
minn = 100;    
for ii=1:3001

compare = ref(i,1) - actual(ii,1);

if abs(compare)<= minn
minn = abs(compare);
min_int_ref = i;
min_int_actual = ii;
end    


end
min_int_ref_save = [min_int_ref_save min_int_ref];
min_int_actual_save = [min_int_actual_save min_int_actual];
end    
    


    
end

