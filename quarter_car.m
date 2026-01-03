ms = 250; % sprung mass in kg
mu = 30;  % unsprung_mass_in _kg
ks = 20000; %stiffness of suspensiion
kt = 200000; %stiffnes of tire
C_min = 100; % Lower limit for variable damper
C_max = 4000; %Upper limit for variable damper



%as = [ - ks*(xs - xu) - c*(vs - vu) ] / ms

%,au = [   ks*(xs - xu) + c*(vs - vu) - kt*(xu - r) ] / mu
