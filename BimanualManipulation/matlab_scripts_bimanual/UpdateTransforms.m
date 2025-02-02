function [pandaArms] = UpdateTransforms(pandaArms, mission)
% the function updates all the transformations

% Left arm transformations
pandaArms.ArmL.bTe = getTransform(pandaArms.ArmL.franka, ...
    [pandaArms.ArmL.q',0,0],'panda_link7');%DO NOT EDIT

% Right arm transformations
pandaArms.ArmR.bTe = getTransform(pandaArms.ArmR.franka, ...
    [pandaArms.ArmR.q',0,0],'panda_link7');%DO NOT EDIT

% <e> to <w>
pandaArms.ArmL.wTe = pandaArms.ArmL.wTb * pandaArms.ArmL.bTe;
pandaArms.ArmR.wTe = pandaArms.ArmR.wTb * pandaArms.ArmR.bTe;

% Transformation matrix from <t> to <w>
pandaArms.ArmL.wTt = pandaArms.ArmL.wTe * pandaArms.ArmL.eTt;
pandaArms.ArmR.wTt = pandaArms.ArmR.wTe * pandaArms.ArmR.eTt;
