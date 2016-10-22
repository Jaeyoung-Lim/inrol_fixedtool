function f_ex=Environment(inputs)
% modeling the environment
pdot = [inputs(1) inputs(2) inputs(3)]';
p = [inputs(4) inputs(5) inputs(6)]';
w = [inputs(7) inputs(8) inputs(9)]';
R = [inputs(10) inputs(11) inputs(12); inputs(13) inputs(14) inputs(15); inputs(16) inputs(17) inputs(18)];
xe=inputs(19);
ke=inputs(20);
 if p(1)<xe
     f_tem=[0 0 0];
 else
     f_tem=[0 0 0];
     %f_tem=-[ke*(p(1)-xe) 0 0];
 end
 

 f_ex=f_tem;