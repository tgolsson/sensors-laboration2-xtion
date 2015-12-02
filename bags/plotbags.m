close all; clear all; clc;
it = 1;
append = "60x60";
for i = [50 100 150 200 300]
  i2 = num2str(i);
  load(strcat("mean", i2, "cm", append, ".txt"));

  eval(strcat("mean", i2, "cm", " = ","mean", i2, "cm", append, ";")); 
  load( strcat("stddev", i2, "cm", append, ".txt"));
  genvarname(strcat("stddev", i2, "cm")); 
  eval(strcat("stddev", i2, "cm", " = ", "stddev", i2, "cm", append, ";"));
  
endfor

figure; 
subplot(2,3, 1);hold on;
title("Mean error");
mLength = min([length(mean50cm), length(mean100cm), length(mean150cm), length(mean200cm), length(mean300cm)]);
X = 1:mLength;
plot(X, mean50cm(1:mLength)-.5, "r-", 'linewidth', 2);
plot(X, mean100cm(1:mLength)-1, "g-", 'linewidth', 2);
plot(X, mean150cm(1:mLength)-1.5, "b-", 'linewidth', 2);
plot(X, mean200cm(1:mLength)-2, "k-", 'linewidth', 2);
plot(X, mean300cm(1:mLength)-3, "c-", 'linewidth', 2);


legend("50cm", "100cm", "150cm", "200cm", "300cm", "location", "southoutside","orientation", "horizontal");
legend("boxoff");
xlabel("Sample");
ylabel("Error");


subplot(2,3,2);hold on;
title("Variance"); 
plot(X, stddev50cm(1:mLength), "r-",'linewidth', 2);
plot(X, stddev100cm(1:mLength),"g-",'linewidth', 2);
plot(X, stddev150cm(1:mLength), "b-", 'linewidth', 2);
plot(X, stddev200cm(1:mLength), "k-", 'linewidth', 2);
plot(X, stddev300cm(1:mLength), "c-", 'linewidth', 2);
legend("50cm", "100cm", "150cm", "200cm", "300cm", "location", "southoutside","orientation", "horizontal");
legend("boxoff")
xlabel("Sample");
ylabel("Variance");

subplot(2,3,3); hold on;
title("By distance");
means = [mean(mean50cm-.5) mean(mean100cm-1) mean(mean150cm-1.5) mean(mean200cm-2) mean(mean300cm-3)];
stddevs = [mean(stddev50cm) mean(stddev100cm) mean(stddev150cm) mean(stddev200cm) mean(stddev300cm)];
distances = [50 100 150 200 300];
[ax, h1, h2] = plotyy(distances, means, distances, stddevs);
set(h1,  "linewidth", 2)
set(h2,  "linewidth", 2)
legend("Mean", "Stddev");
legend("boxoff")
ylabel("")
xlabel("Distance (cm)");



