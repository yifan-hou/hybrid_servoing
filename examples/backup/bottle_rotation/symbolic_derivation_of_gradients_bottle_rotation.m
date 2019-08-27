% Derive jacobian function for 'flip against corner' example.

clear;clc;
% symbols
q_WO     = sym('q_WO',[4,1],'real');  % object in world frame
p_WO     = sym('p_WO',[3,1],'real');
q_WH     = sym('q_WH',[4,1],'real');  % hand in world frame
p_WH     = sym('p_WH',[3,1],'real');

p_OTC = sym('p_OTC',[3,1],'real');  % table contacts in object frame
p_WTC = sym('p_WTC',[3,1],'real');  % table contacts in world frame

% contacts between the object and the hand
kPointsPerFaceContact = 4;
p_OHC_all = sym('p_OHC_all',[3, kPointsPerFaceContact],'real');
p_HHC_all = sym('p_HHC_all',[3, kPointsPerFaceContact],'real');

holonomic_constraint = sym('Phi', [3*(1+kPointsPerFaceContact), 1], 'real');

% table contact
holonomic_constraint(1:3) = quatOnVec(p_OTC, q_WO) + p_WO - p_WTC;
% contact between the object and the hand
for i = 1:kPointsPerFaceContact
	holonomic_constraint(3*i+1:3*i+3) = ...
        quatOnVec(quatOnVec(p_OHC_all(:,i), q_WO)+p_WO-p_WH, quatInv(q_WH)) ...
			- p_HHC_all(:,i);
end

holonomic_constraint = simplify(holonomic_constraint);
save generated/derivation.mat;
disp('Computing derivatives:');

%% ---------------------------------------------------------------
% 			calculate derivatives
% ---------------------------------------------------------------
% load generated/derivation
deriv_q  = @(f) [
		diff(f,'p_WO1'), diff(f,'p_WO2'), diff(f,'p_WO3'), ...
		diff(f,'q_WO1'), diff(f,'q_WO2'), diff(f,'q_WO3'), diff(f, 'q_WO4'), ...
		diff(f,'p_WH1'), diff(f,'p_WH2'), diff(f,'p_WH3'), ...
		diff(f,'q_WH1'), diff(f,'q_WH2'), diff(f,'q_WH3'), diff(f, 'q_WH4')];

Phi_q  = simplify(deriv_q(holonomic_constraint));
disp('Done. Generating file:');

%% ---------------------------------------------------------------
% 			write to file
% ---------------------------------------------------------------
jac_phi_q  = matlabFunction(Phi_q, 'File', 'generated/jac_phi_q_bottle_rotation', 'vars', ...
	{p_WO, q_WO, p_WH, q_WH, p_OTC, p_WTC, p_OHC_all, p_HHC_all});
save generated/derivation.mat;
disp('All done');
