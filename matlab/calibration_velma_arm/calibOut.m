function [] = calibOut( name, parent, T )
%CALIBOUT Summary of this function goes here
%   Detailed explanation goes here
rpy = mat332rpy(T(1:3, 1:3));

disp(['<!-- ' parent ' -> ' name ' -->']);
disp(['<property name="cal_' parent '_to_' name '_x" value="' num2str(T(1, 4)) '" />']);
disp(['<property name="cal_' parent '_to_' name '_y" value="' num2str(T(2, 4)) '" />']);
disp(['<property name="cal_' parent '_to_' name '_z" value="' num2str(T(3, 4)) '" />']);

disp(['<property name="cal_' parent '_to_' name '_roll" value="' num2str(rpy(1)) '" />']);
disp(['<property name="cal_' parent '_to_' name '_pitch" value="' num2str(rpy(2)) '" />']);
disp(['<property name="cal_' parent '_to_' name '_yaw" value="' num2str(rpy(3)) '" />']);

end

