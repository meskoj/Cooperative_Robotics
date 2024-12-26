function [uvms] = UpdateTransforms(uvms)
% the function updates all the transformations

uvms.wTv = [rotation(uvms.p(4), uvms.p(5), uvms.p(6)) uvms.p(1:3); 0 0 0 1]; 
uvms.vTw = inv(uvms.wTv);
uvms.vTg = uvms.vTw*uvms.wTg;
[uvms.bJe, uvms.djdq, uvms.bTe] = JacobianMaris2(uvms.q);
uvms.vTe = uvms.vTb*uvms.bTe;
uvms.vTt = uvms.vTe*uvms.eTt;
uvms.wTt = uvms.wTv*uvms.vTt;

uvms.vTbodyGoal = uvms.vTw * uvms.wTbodyGoal;

w_Kw = [0 0 1]';
v_Kw = uvms.vTw(1:3,1:3) * w_Kw;
uvms.altitude = v_Kw' * [0 0 uvms.sensorDistance]';
