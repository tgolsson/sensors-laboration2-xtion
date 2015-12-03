close all; clear all; clc;
it = 1;
append = "20x20";
for i = [50]
  for filter = {"", "G", "B", "M", "tM", "tA"}
    i2 = num2str(i);
    load(strcat("mean", filter{1,1}, i2, "cm", append, ".txt"));

    eval(strcat("mean", filter{1,1}, i2, "cm", " = ","mean",  filter{1,1},i2, "cm", append, ";")); 
    load( strcat("stddev",  filter{1,1},i2, "cm", append, ".txt"));
    genvarname(strcat("stddev",  filter{1,1},i2, "cm")); 
    eval(strcat("stddev",  filter{1,1},i2, "cm", " = ", "stddev",  filter{1,1}, i2, "cm", append, ";"));
  endfor
endfor

colors = ["rgbkcm"]
figure; 
subplot(1,2, 1);hold on;
title("Mean error");

mLength = min([length(mean50cm), length(meanG50cm), length(meanB50cm), length(meanM50cm), length(meantM50cm) length(meantA50cm)]);

filter = {"", "G", "B", "M", "tM", "tA"};
for i = 1:6
    X = 1:mLength;
    plot(X(2:end), eval(strcat("mean", filter{1,i}, "50cm(2:mLength)-.5")), strcat(colors(i),"-"), 'linewidth', 2);
endfor

legend("Original", "Gaussian", "Bilateral", "Median", "Temporal Median", "Temporal Mean", "location", "southoutside","orientation", "horizontal");
legend("boxoff");
xlabel("Sample");
ylabel("Error");


subplot(1,2,2);hold on;
title("Variance"); 
for i = 1:6
    X = 1:mLength;
    plot(X(2:end), eval(strcat("stddev", filter{1,i}, "50cm(2:mLength)")), strcat(colors(i),"-"), 'linewidth', 2);
endfor

legend("Original", "Gaussian", "Bilateral", "Median", "Temporal Median", "Temporal Mean", "location", "southoutside","orientation", "horizontal");
legend("boxoff")
xlabel("Sample");
ylabel("Variance");

figure;
hold on;
title("By Filter");
means = [];
stddevs = [];
for i = 1:6
  means = [means mean(eval(strcat("mean", filter{1,i}, "50cm(2:mLength)")))];
  stddevs = [stddevs mean(eval(strcat("stddev", filter{1,i}, "50cm(2:mLength)")))];
endfor

distances = [1 2 3 4 5 6];
[ax, h1, h2] = plotyy(distances, means-.5, distances, stddevs);
set(h1, "linewidth", 2)
set(h2, "linewidth", 2)

set(h1,'linestyle','none','marker','o'); set(h2,'linestyle','none','marker','^');
set(ax, 'XTickLabel', ["Original"; "Gaussian"; "Bilateral"; "Median"; "Temporal Meadian"; "Temporal Mean"]);
legend("Mean", "Stddev");
legend("boxoff")
ylabel("")
xlabel("Distance (cm)");



