clear all;
close all;

data = [49 7 29
    8 19 16
    12 8 14
    19 37 22
    3 43 21
    34 17 17
    20 34 27
    49 14 37
    20 26 21
    31 41 21];

data(:,1) = data(:,1) - mean(data(:,1));
data(:,2) = data(:,2) - mean(data(:,2));
data(:,3) = data(:,3) - mean(data(:,3));



