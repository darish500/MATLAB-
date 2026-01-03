%workinhg with state_space

A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];

con_t = ctrb( A ,B)
rank_con =  rank(con_t)
rank(B)
if rank_con == size(A,1)
    disp("Controllable")
else 
    disp("not controllable")
end

kb = place(A ,B,[-5,-6])%setting the poles 
 %Creating a closed loo equation

 Acl = A - B*kb

 sys = ss(Acl , +B , C ,0)

 step(sys);

 %Coverting the ss to tf 

[num , den]= ss2tf(A,B,C,0)

 real_val = tf(num , den)

 %Getting the state or everything aboutthe system 

 ord = stepinfo(real_val)