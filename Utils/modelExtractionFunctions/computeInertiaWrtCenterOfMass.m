function [I_oL, m, com ,rpy, env_var] = computeInertiaWrtCenterOfMass(model, jointIndex, env_var)
%Compute the inertia of body i wrt a frame centered in the center of mass.
% Seethe inertial tag for a link in a urdf:
% http://wiki.ros.org/urdf/XML/link
% Link 1 is the base_link, which is not considered in the array of
% spatial inertias(see http://royfeatherstone.org/spatial/v2/sysmodel.html)

% If a link does not have the inertial tag we'll set default values to zero
if ~isfield(model.robot.link{jointIndex+1}, 'inertial')
    warning('Setting inertial elements(center of mass,mass,Inertia Matix) to zero for link %d', jointIndex);
    com = [0 0 0];
    m = 0;
    I = zeros(3);
    rpy = [0 0 0];
else
    % Get the orientation of the frame (G) wrt the Inertia is computed in the urdf
    % and the local body frame
    if isfield(model.robot.link{1,jointIndex+1}.inertial.origin.Attributes,'rpy')
        str_rpy = model.robot.link{1,jointIndex+1}.inertial.origin.Attributes.rpy;
        str_rpy = split(str_rpy,' ');
        rpy     = casadi.SX.sym('rpy',[1 3]);
        for i = 1:length(str_rpy)
            str_rpy_cur = split(str_rpy{i},'_');
            if(strcmp(str_rpy_cur{1},'envvar'))
                v_name  = ['v_' str_rpy_cur{2}];
                v_rpy   = casadi.SX.sym(v_name);
                rpy(i)  = v_rpy;
                env_var = [env_var; v_rpy];
            else
                rpy(i) = str2num(str_rpy_cur{1});
            end
        end
        %rpy = str2num(model.robot.link{1,jointIndex+1}.inertial.origin.Attributes.rpy);
    else
        warning('Link %d missing orientation of the frame wrt the inertia is computed in the urdf.\n Setting it to [0 0 0]', jointIndex);
        rpy = [0 0 0];
    end
    
    % Center of mass
    str_com = model.robot.link{jointIndex+1}.inertial.origin.Attributes.xyz;
    str_com = split(str_com,' ');
    com     = casadi.SX.sym('rpy',[1 3]);
    for i = 1:length(str_com)
        str_com_cur = split(str_com{i},'_');
        if(strcmp(str_com_cur{1},'envvar'))
            v_name  = ['v_' str_com_cur{2}];
            v_com   = casadi.SX.sym(v_name);
            com(i)  = v_com;
            env_var = [env_var; v_com];
        else
            com(i) = str2num(str_com_cur{1});
        end
    end

    % Mass
    str_mass = model.robot.link{jointIndex+1}.inertial.mass.Attributes.value;
    str_mass  = split(str_mass,'_');
    if(strcmp(str_mass{1},'envvar'))
        v_name = ['v_' str_mass{2}];
        v_m    = casadi.SX.sym(v_name);
        m      = v_m;
        env_var = [env_var; v_m];
    else  
        m = str2num(model.robot.link{jointIndex+1}.inertial.mass.Attributes.value);
    end
    
    % Inertia
    str_ixx = model.robot.link{jointIndex+1}.inertial.inertia.Attributes.ixx;
    str_ixx  = split(str_ixx,'_');
    if(strcmp(str_ixx{1},'envvar'))
        v_name = ['v_' str_ixx{2}];
        v_ixx  = casadi.SX.sym(v_name);
        ixx    = v_ixx;
        env_var = [env_var; v_ixx];
    else
        ixx = str2num(model.robot.link{jointIndex+1}.inertial.inertia.Attributes.ixx);
    end
    
    str_ixy = model.robot.link{jointIndex+1}.inertial.inertia.Attributes.ixy;
    str_ixy  = split(str_ixy,'_');
    if(strcmp(str_ixy{1},'envvar'))
        v_name = ['v_' str_ixy{2}];
        v_ixy  = casadi.SX.sym(v_name);
        ixy    = v_ixy;
        env_var = [env_var; v_ixy];
    else
        ixy = str2num(model.robot.link{jointIndex+1}.inertial.inertia.Attributes.ixy);
    end
    
    str_ixz = model.robot.link{jointIndex+1}.inertial.inertia.Attributes.ixz;
    str_ixz  = split(str_ixz,'_');
    if(strcmp(str_ixz{1},'envvar'))
        v_name = ['v_' str_ixz{2}];
        v_ixz  = casadi.SX.sym(v_name);
        ixz    = v_ixz;
        env_var = [env_var; v_ixz];
    else
        ixz = str2num(model.robot.link{jointIndex+1}.inertial.inertia.Attributes.ixz);
    end
    
    str_iyy = model.robot.link{jointIndex+1}.inertial.inertia.Attributes.iyy;
    str_iyy  = split(str_iyy,'_');
    if(strcmp(str_iyy(1),'envvar'))
        v_name = ['v_' str_iyy{2}];
        v_iyy  = casadi.SX.sym(v_name);
        iyy    = v_iyy;
        env_var = [env_var; v_iyy];
    else
        iyy  = str2num(model.robot.link{jointIndex+1}.inertial.inertia.Attributes.iyy);
    end
    
    str_iyz = model.robot.link{jointIndex+1}.inertial.inertia.Attributes.iyz;
    str_iyz  = split(str_iyz,'_');
    if(strcmp(str_iyz{1},'envvar'))
        v_name = ['v_' str_iyz{2}];
        v_iyz  = casadi.SX.sym(v_name);
        iyz    = v_iyz;
        env_var = [env_var; v_iyz];
    else
        iyz = str2num(model.robot.link{jointIndex+1}.inertial.inertia.Attributes.iyz);
    end
    
    str_izz = model.robot.link{jointIndex+1}.inertial.inertia.Attributes.izz;
    str_izz  = split(str_izz,'_');
    if(strcmp(str_izz{1},'envvar'))
        v_name = ['v_' str_izz{2}];
        v_izz  = casadi.SX.sym(v_name);
        izz    = v_izz;
        env_var = [env_var; v_izz];
    else
        izz = str2num(model.robot.link{jointIndex+1}.inertial.inertia.Attributes.izz);
    end
    
    % TODO I has to became a casadi if one of I are casadi 
    I =[ixx, ixy, ixz;
        ixy, iyy, iyz;
        ixz, iyz, izz]; 

    % Sanity check on Inertia: it must be at least positive semidefinite (only if I is not casadi)
    if(isequal(class(I),'casadi.SX'))
        warning('the inertial matrix is symbolic so we cannot check the positive semidefinite')
    else 
        principalI = eig(I);
        if find(principalI<0)
            warning('Inertial matrix body %d is not positive semi-definite', jointIndex);
        end
    end
end
% Compute the Inertia matrix wrt a frame centered in the center of mass(g) but
% alligned with body local frame (L) orientation 
I_gL = changeReferenceFrameInertiaMatrix(I, [],rpy,[], 'rpy');
% Compute Inertia matrix wrt frame centered in the local frame origin (o) and
% with orintation of the local body frame (L)
I_oL = mcI(m, com, I_gL);   
end