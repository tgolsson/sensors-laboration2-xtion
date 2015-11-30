close all; clear all; clc;
it = 1;
for i = [50 100 150 200 300]
  figure; 
  title("Error plots for static camera")
  subplot(1,2, 1);hold on;
  i2 = num2str(i);
  load(strcat("mean", i2, "cm.txt"));
  l = 1:length(eval(strcat("mean", i2, "cm"))); 
  plot(l, eval(strcat("mean", i2, "cm"))-i/100, "r-", 'linewidth', 2);
  title("Error")
  subplot(1,2,2);
  load( strcat("stddev", i2, "cm.txt"));
  plot(l, eval(strcat("stddev", i2, "cm")), "b-", 'linewidth', 2);
  

  title("Variance");
endfor


