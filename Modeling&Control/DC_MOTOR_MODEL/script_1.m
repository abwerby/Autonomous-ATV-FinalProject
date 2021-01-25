%% Clear workspace
clc;
clear variables;
close all;

%% Read csv file
sp = csvread('sp_tic_data.csv');

t = sp(:,2);
t = t*0.1;
s = sp(:,3);

%% Genereate volt input stamp
v = sp(:,4);
for i=1:length(v)
    if v(i) ~= 0 
        v(i) = 12.08 ;
    end
end

input = [t,v];
output = [ t,s];
plot(t,[s,v]);
