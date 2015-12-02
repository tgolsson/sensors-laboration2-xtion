close all; clear all; clc;
it = 1;
for i = {"20x20" "40x40" "60x60" "80x80" "100x100"}
  load(strcat("mean50cm", i{1,1}, ".txt"));
  load(strcat("stddev50cm", i{1,1}, ".txt"));  
endfor

figure; 
subplot(2,3, 1);hold on;
title("Mean error");
mLength = min([length(mean50cm20x20), length(mean50cm40x40), length(mean50cm60x60), length(mean50cm80x80), length(mean50cm100x100)]);
X = 1:mLength;
plot(X, mean50cm20x20(1:mLength)-.5, "r-", 'linewidth', 2);
plot(X, mean50cm40x40(1:mLength)-.5, "g-", 'linewidth', 2);
plot(X, mean50cm60x60(1:mLength)-.5, "b-", 'linewidth', 2);
plot(X, mean50cm80x80(1:mLength)-.5, "k-", 'linewidth', 2);
plot(X, mean50cm100x100(1:mLength)-.5, "c-", 'linewidth', 2);


legend("20x20", "40x40", "60x60", "80x80", "100x100", "location", "southoutside","orientation", "horizontal");
legend("boxoff");
xlabel("Sample");
ylabel("Error");


subplot(2,3,2);hold on;
title("Variance"); 
plot(X, stddev50cm20x20(1:mLength), "r-",'linewidth', 2);
plot(X, stddev50cm40x40(1:mLength),"g-",'linewidth', 2);
plot(X, stddev50cm60x60(1:mLength), "b-", 'linewidth', 2);
plot(X, stddev50cm80x80(1:mLength), "k-", 'linewidth', 2);
plot(X, stddev50cm100x100(1:mLength), "c-", 'linewidth', 2);
legend("20x20", "40x40", "60x60", "80x80", "100x100", "location", "southoutside","orientation", "horizontal");
legend("boxoff")
xlabel("Sample");
ylabel("Variance");

subplot(2,3,3); hold on;
title("By window size");
means = [mean(mean50cm20x20-.5) mean(mean50cm40x40-.5) mean(mean50cm60x60-.5) mean(mean50cm80x80-.5) mean(mean50cm100x100-.5)];
stddevs = [mean(stddev50cm20x20) mean(stddev50cm40x40) mean(stddev50cm60x60) mean(stddev50cm80x80) mean(stddev50cm100x100)];
distances = [20 40 60 80 100];
[ax, h1, h2] = plotyy(distances, means, distances, stddevs);
set(h1,  "linewidth", 2)
set(h2,  "linewidth", 2)
legend("Mean", "Stddev");
legend("boxoff")
ylabel("")
xlabel("Window size (px)");



