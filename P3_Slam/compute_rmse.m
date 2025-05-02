function rmse_value = compute_rmse(true_path, estimated_path)
[row,n]=size(true_path);
rmse_valuex=0;
rmse_valuey=0;
rmse_valueo=0;
    
for i=1:n
    rmse_valuex=rmse_valuex+(true_path(1,i)-estimated_path(1,i))^2;
    rmse_valuey=rmse_valuey+(true_path(2,i)-estimated_path(2,i))^2;
    rmse_valueo=rmse_valueo+(true_path(3,i)-estimated_path(3,i))^2;

end
rmse_value=[sqrt(rmse_valuex/n), sqrt(rmse_valuey/n),sqrt(rmse_valueo/n)];

end