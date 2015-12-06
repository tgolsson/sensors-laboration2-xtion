close all; clear all; clc;
it = 1;
append = "40x40";
for i = [300]
  for filter = {"", "G", "B", "M", "tM", "tA"}
    i2 = num2str(i);
    load(strcat("mean", filter{1,1}, i2, "cm", append, ".txt"));

    eval(strcat("mean", filter{1,1}, i2, "cm", " = ","mean",  filter{1,1},i2, "cm", append, ";")); 
    load( strcat("variance",  filter{1,1},i2, "cm", append, ".txt"));
    genvarname(strcat("variance",  filter{1,1},i2, "cm")); 
    eval(strcat("variance",  filter{1,1},i2, "cm", " = ", "variance",  filter{1,1}, i2, "cm", append, ";"));
  endfor
endfor

colors = ["rgbkcm"]
figure; 
subplot(2,3, 1);hold on;
title("Mean error");

mLength = min([length(mean300cm), length(meanG300cm), length(meanB300cm), length(meanM300cm), length(meantM300cm) length(meantA300cm)]);
mLength = min([mLength length(variance300cm), length(varianceG300cm), length(varianceB300cm), length(varianceM300cm), length(variancetM300cm) length(variancetA300cm)]);

filter = {"", "G", "B", "M", "tM", "tA"};
for i = 1:6
    X = 2:mLength;
    plot(X, eval(strcat("mean", filter{1,i}, "300cm(2:mLength)-3")), strcat(colors(i),"-"), 'linewidth', 2);
endfor

legend("Original", "Gaussian", "Bilateral", "Median", "t-Median", "t-Mean", "location", "southoutside","orientation", "horizontal");
legend("boxoff");
xlabel("Sample");
ylabel("Error");


subplot(2,3,2);hold on;
title("Variance"); 
for i = 1:6
    X = 2:mLength;
    plot(X, eval(strcat("variance", filter{1,i}, "300cm(2:mLength)")), strcat(colors(i),"-"), 'linewidth', 2);
endfor

legend("Original", "Gaussian", "Bilateral", "Median", "t-Median", "t-Mean", "location", "southoutside","orientation", "horizontal");
legend("boxoff")
xlabel("Sample");
ylabel("Variance");

subplot(2,3,3); hold on;
title("By Filter");
means = [];
variances = [];
for i = 1:6
  means = [means mean(eval(strcat("mean", filter{1,i}, "300cm(2:mLength)")))];
  variances = [variances mean(eval(strcat("variance", filter{1,i}, "300cm(2:mLength)")))];
endfor

distances = [1 2 3 4 5 6];
[ax, h1, h2] = plotyy(distances, means-3, distances, variances);
set(h1, "linewidth", 2)
set(h2, "linewidth", 2)

set(h1,'linestyle','none','marker','o'); set(h2,'linestyle','none','marker','^');
set(ax, 'XTickLabel', ["Original"; "Gaussian"; "Bilateral"; "Median"; "t-Median"; "t-Mean"]);
legend("Mean error", "Variance");
ylabel("")
xlabel("Filter");



