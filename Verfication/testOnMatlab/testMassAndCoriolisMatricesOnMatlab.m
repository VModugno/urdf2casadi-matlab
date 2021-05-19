% Test the Mass and the Coriolis matrices
close all 
clear variables
clc
%% Choose a urdf model
location_tests_folder = pwd;
kuka_urdf = [location_tests_folder,'/../../URDFs/kr30_ha-identified.urdf'];
twoLink_urdf = [location_tests_folder,'/../../URDFs/twoLinks.urdf'];
kuka_kr210 = [location_tests_folder,'/../../URDFs/kuka_kr210.urdf'];
kuka_kr210_env = [location_tests_folder,'/../../URDFs/kuka_kr210_env.urdf'];
iCub_r_leg = [location_tests_folder,'/../../URDFs/iCub_r_leg.urdf'];

%% Input urdf file to acquire robot structure
robotModelURDFIDyn = kuka_kr210;
robotModelURDFID   = kuka_kr210_env;

%% nominal variables
env_var = [1385.5 -0.83462 710.03 -1.3604 6.3154];

%% Constants
gravityModulus = 9.80665;
tol = 1e-10;
%% Symbolic functions
% Inverse dynamics symbolic function
[symbolicIDFunction env_var_flag] = symbolicInverseDynamics(robotModelURDFID,0);
% Compute mass matrix, its derivative and Coriolis matrix with an efficient algorithm
smds = extractSystemModel(robotModelURDFID);
nrOfJoints = smds.NB;
g = [0;0;-gravityModulus];


%% Prepare variables to store results
nrOfTests = 20;
e_massMatrix = zeros(nrOfJoints,nrOfJoints,nrOfTests);
e_gravity    = zeros(nrOfJoints,nrOfTests);
e_torqueID   = zeros(nrOfJoints,nrOfTests);
e_genBias    = zeros(nrOfJoints,nrOfTests);
for i= 1:nrOfTests
    %% Random points
    jointPos = rand(6,1);
    jointVel = rand(6,1);
    jointAcc = rand(6,1);
    
    %% IDynTree values 
    % Compute mass matrix with IDynTree
    M_IDyn=computeMassMatrixIDynTree(robotModelURDFIDyn,jointPos,jointVel,jointAcc,gravityModulus);
    % Compute gravity torques IDynTree
    tau_gravity_IDyn = computeGravityTorqueIDynTree(robotModelURDFIDyn,jointPos,gravityModulus);
    % Compute the generalized bias force C(q,qd)*qd + g(q)
    genealizedBias_IDyn = computeGeneralizedBiasForceIDynTree(robotModelURDFIDyn,jointPos,jointVel,gravityModulus);
    
    %% Matrices
    %[H_cell,HDot_cell,C_cell] = computeSymbolicCoriolismatrix(jointPos,jointVel,jointAcc,smds);
    %H = cell2mat_casadi(H_cell);
    %HDot = cell2mat_casadi(HDot_cell);
    %C = cell2mat_casadi(C_cell);
    [HFunction,HDotFunction,CFunction, env_var_flag]= createMassAndCoriolisMatrixFunction(robotModelURDFID,0,'');
    if(env_var_flag)
        H = full(HFunction(jointPos,env_var));
        HDot = full(HDotFunction(jointPos,jointVel,env_var));
        C = full(CFunction(jointPos,jointVel,env_var));
    else
        H = full(HFunction(jointPos));
        HDot = full(HDotFunction(jointPos,jointVel));
        C = full(CFunction(jointPos,jointVel));
    end
    
    % Use CRBA and NE
    %[H_CRBA,C_CRBA] = HandC( smds, jointPos,jointVel, g );
    
    %% Compute gravity torques using the symbolic Inverse Dynamics function
    tau_gravity = full(computeGravityTorque(jointPos,g,symbolicIDFunction,env_var,env_var_flag));
   

    %% Compare generalized bias forces
    generalizedBias = C*jointVel + tau_gravity;

    %% Compute the inverse dynamics with the matrices  
    tau_efficient = H*jointAcc + C*jointVel + tau_gravity;
    
    if(env_var_flag)
        tau_ID = full(symbolicIDFunction(jointPos,jointVel,jointAcc,g,0,env_var));
    else
        tau_ID = full(symbolicIDFunction(jointPos,jointVel,jointAcc,g,0));
    end

    %% Compare iDynTee and symbolic 
    e_massMatrix(:,:,i) = abs(H - M_IDyn);
    e_normMass(:,i) = norm(e_massMatrix(:,:,i)); 
    e_gravity(:,i) = abs(tau_gravity - tau_gravity_IDyn);
    e_torqueID(:,i) = abs(tau_efficient-tau_ID);
    e_genBias(:,i) = abs(generalizedBias-genealizedBias_IDyn);
end
figure
plot(e_normMass');title('Mass matrix error norm: norm(M_{symb} - M_{IDyn})');legend; 
figure
plot(e_genBias');title('Generalized bias error norm: norm(generalizedBias_{symb} - generalizedBias_{IDyn})');
legend;   
