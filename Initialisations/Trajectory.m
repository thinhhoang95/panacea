clear all;
clc;

t = [0 5 10 15 20 25 30];
x = [0 3 5 9 12 8 4];
y = [0 -1 -5 -9 -15 -12 -7];
z = [0 -3 -5 -6 -6.5 -6.5 -7];

phi = pi/180 * [0 9 12 -1 -8 -12 0];
the = pi/180 * [0 2 -4 10 5 -12 -14];
psi = pi/180 * [0 20 40 45 55 47 40];

f = 0.026; % iPhone camera wide angle lens

globalTs = 1/300; % global sampling time
trackingTs = 1/25; % x fps for tracking algorithm
detectionTs = 3.25; % CNN delay in seconds
numOfLm = 50;

lms = zeros(numOfLm,3);
for i=1:numOfLm
    lms(i,:) = [(5-(-5)).*rand(1,2) + (-5), 0];
end