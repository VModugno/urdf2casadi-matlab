% test forward dyanmics idintree vs urdf2casadi
clear 
clc 
close all

%% Choose a urdf model idyntree
location_tests_folder = pwd;
kuka_urdf        = [location_tests_folder,'/../../URDFs/kr30_ha-identified.urdf'];
twoLink_urdf     = [location_tests_folder,'/../../URDFs/twoLinks.urdf'];
twoLink_env      = [location_tests_folder,'/../../URDFs/twoLinks_env.urdf'];
kuka_kr210       = [location_tests_folder,'/../../URDFs/kuka_kr210.urdf'];
kuka_kr210_env   = [location_tests_folder,'/../../URDFs/kuka_kr210_env.urdf'];
kuka_iiwa        = [location_tests_folder,'/../../URDFs/kuka_iiwa.urdf'];
iCub_r_leg       = [location_tests_folder,'/../../URDFs/iCub_r_leg.urdf'];


%% Choose a urdf model urdf2casai


%% Input urdf file to acquire robot structure
robotURDFModelIDyn = kuka_kr210;
robotURDFmodelID   = kuka_kr210;

%% Constants
gravityModulus = 9.80665;
tol = 1e-10;

%% nominal variables
env_var = [1385.5 -0.83462 710.03 -1.3604 6.3154];

%1385.5 -0.83462

%% Symbolic functions
% Inverse dynamics symbolic function
[symbolicIDFunction, env_var_flag] = symbolicForwardDynamics(robotURDFmodelID,0);
% Compute mass matrix, its derivative and Coriolis matrix with an efficient algorithm
smds = extractSystemModel(robotURDFmodelID);
nrOfJoints = smds.NB;
g = [0;0;-gravityModulus];


%% Prepare variables to store results
nrOfTests = 4000;

e_accID   = zeros(nrOfJoints,nrOfTests);
for i= 1:nrOfTests
    %% Random points
    jointPos = rand(nrOfJoints,1);
    jointVel = rand(nrOfJoints,1);
    jointAcc = rand(nrOfJoints,1);
    tau      = rand(nrOfJoints,1);
    
    %% IDynTree values 
    % Compute mass matrix with IDynTree
    M_IDyn = computeMassMatrixIDynTree(robotURDFModelIDyn,jointPos,jointVel,jointAcc,gravityModulus);
    % Compute gravity torques IDynTree
    %tau_gravity_IDyn = computeGravityTorqueIDynTree(robotURDFModel,jointPos,gravityModulus);
    % Compute the generalized bias force C(q,qd)*qd + g(q)
    genealizedBias_IDyn = computeGeneralizedBiasForceIDynTree(robotURDFModelIDyn,jointPos,jointVel,gravityModulus);
    
    %% compute direct dynamics IDyn
    acc_IDyn = M_IDyn\(-genealizedBias_IDyn  + tau);
    
    %% COmpute direct dynamics symbolic
    if(env_var_flag)
        acc_ID = full(symbolicIDFunction(jointPos,jointVel,g,tau,env_var));
    else
        acc_ID = full(symbolicIDFunction(jointPos,jointVel,g,tau));
    end

    %% Compare iDynTee and symbolic 
    e_accID(:,i) = abs(acc_ID-acc_IDyn);
    
end
%% plot results
plot(e_accID);title('forward dynamics error norm: norm(acc_{symb} - acc_{IDyn})');legend
