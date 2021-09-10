% Initial Proof of concept for intersection of three spheres
% one or more solutions must exist, or must be within some tolerance
% does not include any correction for error in distance measurements
% does not include any correction for error in anchor positions
% 
% one potential way to calculate a valid location given an uncertainty
% is to minimize the distance from all the anchors to the valid location
% with constraints for each anchor-location distance bounded between 
% [distance-potentialerror, distance+potentialerror]
% After a valid point is calculated, monte carlo simulation can be used in
% a region surrounding this point to see if a better solution can be found.


% clear the workspace and commmand window
clear;
clc;


% a,b,c are three center points for spheres and r is the radii for each,
% where a valid intersection point between these spheres exists
a = [0,0,0];
b = [4,0,0];
c = [2,2,0];
r = [2, 2,2];

% solve a set of nonlinear equations
[out, fval] = fsolveVar(a,b,c,r)



function [out, fval] = fsolveVar(a,b,c,r)
    xyz0 = [0,0,0]; % an initial guess for the solver
    [out, fval] = fsolve(@spheres,xyz0); % actually computes the solution (if exists)
    
    % function definition of the system of nonlinear equations
    function F = spheres(xyz)
        x = xyz(1);
        y = xyz(2);
        z = xyz(3);
        F(1) = x^2 - 2*x*a(1) + a(1)^2 + y^2 - 2*y*a(2) + a(2)^2 + z^2 - 2*z*a(3) + a(3)^2 - r(1)^2;
        F(2) = x^2 - 2*x*b(1) + b(1)^2 + y^2 - 2*y*b(2) + b(2)^2 + z^2 - 2*z*b(3) + b(3)^2 - r(2)^2;
        F(3) = x^2 - 2*x*c(1) + c(1)^2 + y^2 - 2*y*c(2) + c(2)^2 + z^2 - 2*z*c(3) + c(3)^2 - r(3)^2;

    end
end
