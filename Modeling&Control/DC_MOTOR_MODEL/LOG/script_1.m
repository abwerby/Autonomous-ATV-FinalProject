%% Clear workspace
clc;
clear variables;
close all;

%% Read csv file
sp = csvread('sp_tic_data.csv');

t = sp(:,1);
t = t - 1912;
t = t*0.1;
s = sp(:,2);

%% Genereate volt input stamp
v = s;
for i=1:length(v)
    if v(i) ~= 0 
        v(i) = 12.08 ;
    end
end

input = [t,v];
% subplot(t,v);
