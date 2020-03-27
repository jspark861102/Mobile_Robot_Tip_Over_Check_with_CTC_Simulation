
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KIMM platform %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%mass
m1 = 100;
m2 = 40 + 10; %add payload
m = m1 + m2;
g = 9.81;

%wheel radius
r = 0.1;
%wheel base
D = 0.6;
%width
L = 0.5;

%z value from global origin to link1 mass center
h1 = 0.40;
%z value from global origin to link2 mass center
h2 = 0.95;
%z value from joint 0 to link1 mass centor
h0c1 = h1;
%z value from joint 1 to link2 mass centor
h1c2 = 0.5;

%x value from global origin to link2 mass center
a = 0.09;
%y value from global origin to link2 mass center
b = 0.00;
%moment of inertia
I = 1/12*m1*(D^2 + L^2) + m2*(a^2+b^2); %m2�� lumped mass�� ����

v_max = 1.00;%1;
road_width = 0.9;%2;
circle_r = (road_width - L*cos(pi/4)) / (1-cos(pi/4));

t = 0 : dt : pi *circle_r/(2*v_max) +0.5;
num = 20;
v_ref = v_max*[-cos(pi .* [0:1/num:1])/2+0.5 1*ones(1,length(t) - 2*(num+1)) +cos(pi .* [0:1/num:1])/2+0.5];
w_ref = 1*(v_max/circle_r)*[-cos(pi .* [0:1/num:1])/2+0.5 1*ones(1,length(t) -2*(num+1)) +cos(pi .* [0:1/num:1])/2+0.5];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
