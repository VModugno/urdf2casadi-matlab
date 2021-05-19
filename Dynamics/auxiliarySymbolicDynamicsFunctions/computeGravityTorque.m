function tau_gravity = computeGravityTorque(jointPos,g,tau_symbolic,env_var,env_var_flag)
%Compute the inverse dynamics by setting the acceleration and velocity to zero
% To set the vector/matrix arguments to all zeros in Casadi functions, it is
% sufficient to set them to a scalar zero
qd = 0;
qdd = 0;
F_ext = 0;
if(env_var_flag==0)
    tau_gravity = tau_symbolic(jointPos,qd,qdd,g,F_ext);
else
    tau_gravity = tau_symbolic(jointPos,qd,qdd,g,F_ext,env_var);
end