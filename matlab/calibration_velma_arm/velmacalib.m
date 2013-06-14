%VELMACALIB

%  Copyright (c) 2013 IAiIS PW
%  All rights reserved
%
%  Author: Konrad Banachowicz <konradb3@gmail.com>
%
%  Redistribution and use in source and binary forms, with or without
%  modification, are permitted provided that the following conditions
%  are met:
%
%   * Redistributions of source code must retain the above copyright
%     notice, this list of conditions and the following disclaimer.
%   * Redistributions in binary form must reproduce the above
%     copyright notice, this list of conditions and the following
%     disclaimer in the documentation and/or other materials provided
%     with the distribution.
%   * Neither the name of IAiIS PW nor the names of any
%     contributors may be used to endorse or promote products derived
%     from this software without specific prior written permission.
%
%  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
%  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
%  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
%  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
%  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
%  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
%  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
%  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
%  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%  POSSIBILITY OF SUCH DAMAGE.
%

load_data;

intrinsic_t = mtk.make_compound('focal_length', @mtk.Rn, 2, ...
                               'offset', @mtk.Rn, 2, ...
                               'distortion', @mtk.Rn, 1);

% create and init i_left, i_right, left2right, left2imu,  
% cam2head, p_hand_l, p_hand_r, left2world_for_img
compute_initial_parameters;

o = mtk.OptimizationProblem();

% add intrinsic parameters
%i_left_id = o.add_random_var(i_left);
%i_right_id = o.add_random_var(i_right);
%i_kinect_left_id = o.add_random_var(i_kinect_left);
%i_kinect_right_id = o.add_random_var(i_kinect_right);

% add transformations
 head2left_id = o.add_random_var(head2left); 
%left2right_id = o.add_random_var(left2right); 
 left2kinect_left_id = o.add_random_var(left2kinect_left);
 left2kinect_right_id = o.add_random_var(left2kinect_right);

% add marker positions on hand 
%p_hand_l_id = o.add_random_var(p_hand_l); 
%p_hand_r_id = o.add_random_var(p_hand_r);

wrist2world_id = o.add_random_var(wrist2world); 
arm2base_id = o.add_random_var(arm2base); 

for i=1:num_images_left        
    o.add_measurement(p_img_l{i}(:), @project_points, {wrist2world_id, arm2base_id, head2left_id}, {mtk.SE3(eye(4)), mtk.SE3(headFK([head_joints_l{i}])), mtk.SE3(armFK([right_arm_joints_l{i}])), i_left, p_world_l{i}(:)}, Sigma_chk);
end

for i=1:num_images_kinect_left
    o.add_measurement(p_img_kl{i}(:), @project_points, {wrist2world_id, arm2base_id, head2left_id, left2kinect_left_id}, {mtk.SE3(headFK([head_joints_kl{i}])), mtk.SE3(armFK([right_arm_joints_kl{i}])), i_kinect_left, p_world_kl{i}(:)}, Sigma_chk);
end

for i=1:num_images_kinect_right
    o.add_measurement(p_img_kr{i}(:), @project_points, {wrist2world_id, arm2base_id, head2left_id, left2kinect_right_id}, {mtk.SE3(headFK([head_joints_kr{i}])), mtk.SE3(armFK([right_arm_joints_kr{i}])), i_kinect_right, p_world_kr{i}(:)}, Sigma_chk);
end

% for i=1:num_images_left        
%     o.add_measurement(p_img_l{i}(:), @project_points, {wrist2world_id, arm2base_id}, {head2left, mtk.SE3(eye(4)), mtk.SE3(headFK([head_joints_l{i}])), mtk.SE3(armFK([right_arm_joints_l{i}])), i_left, p_world_l{i}(:)}, Sigma_chk);
% end
% 
% for i=1:num_images_kinect_left
%     o.add_measurement(p_img_kl{i}(:), @project_points, {wrist2world_id, arm2base_id}, {head2left, left2kinect_left, mtk.SE3(headFK([head_joints_kl{i}])), mtk.SE3(armFK([right_arm_joints_kl{i}])), i_kinect_left, p_world_kl{i}(:)}, Sigma_chk);
% end
% 
% for i=1:num_images_kinect_right
%     o.add_measurement(p_img_kr{i}(:), @project_points, {wrist2world_id, arm2base_id}, {head2left, left2kinect_right, mtk.SE3(headFK([head_joints_kr{i}])), mtk.SE3(armFK([right_arm_joints_kr{i}])), i_kinect_right, p_world_kr{i}(:)}, Sigma_chk);
% end

[X] = nrlm(@o.fun, o.X0, [20, 4, 1e-3]); % solve problem

[r, J, r_orig] = o.fun(X);

chk = reshape(r_orig, 2, []);

index = 0;
chk_size_l = size(p_img_l{1}, 2);
chk_l = chk(:,index+1:index+num_images_left*chk_size_l);
index = index + num_images_left * chk_size_l;
chk_size_kl = size(p_img_kl{1}, 2);
chk_kl = chk(:,index+1:index+num_images_kinect_left * chk_size_kl);
index = index + num_images_kinect_left * chk_size_kl;
chk_size_kr = size(p_img_kr{1}, 2);
chk_kr = chk(:,index+1:index+num_images_kinect_right * chk_size_kr);

figure('Name','Measurment resudual','NumberTitle','off');
plot(chk_l(1,:), chk_l(2,:), '+', chk_kl(1,:), chk_kl(2,:), 'r+', chk_kr(1,:), chk_kr(2,:), 'g+')
legend('Left camera' ,'Left kinect', 'Right kinect')
xlabel('px')
ylabel('py')
axis equal
% RMS Checkerboard measurements

rms_chk = sqrt(sum(r_orig.^2,1) / size(r_orig,1));
disp(['RMS pattern reprojection error:' num2str(rms_chk)])

% print results
%i = X.get_random_var(i_left_id);
%disp(['left camera -> focal length: ' num2str(i.focal_length.vec') ' offset:  ' num2str(i.offset.vec') ' dist: ' num2str(i.distortion.vec) ] );
%i = X.get_random_var(i_right_id);
%disp(['right camera -> focal length: ' num2str(i.focal_length.vec) ' offset:  ' num2str(i.offset.vec') ' dist: ' num2str(i.distortion.vec) ] );
%i = X.get_random_var(i_kinect_left_id);
%disp(['left kinect camera -> focal length: ' num2str(i.focal_length.vec') ' offset:  ' num2str(i.offset.vec') ' dist: ' num2str(i.distortion.vec) ] );
%i = X.get_random_var(i_kinect_right_id);
%disp(['right kinect camera -> focal length: ' num2str(i.focal_length.vec') ' offset:  ' num2str(i.offset.vec') ' dist: ' num2str(i.distortion.vec) ] );


torso2arm = eye(4);
torso2arm(1:3, 4) = [0; -0.118; 0.54];
torso2arm(1:3, 1:3) = quat2mat33([0.35355 -0.35355 0.61237 0.61237]);

disp('arm2base:');
e = X.get_random_var(arm2base_id);
t = torso2arm * e.transform();
disp(['rpy: ' num2str(mat332rpy(t(1:3, 1:3))') ' xyz: ' num2str(t(1:3, 4)')]);

T_opt = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1];

disp('head2left:');
e = X.get_random_var(head2left_id);
t = e.transform() * inv(T_opt);
disp(['rpy: ' num2str(mat332rpy(t(1:3, 1:3))') ' xyz: ' num2str(t(1:3, 4)')]);

%disp('left2right:');
%e = X.get_random_var(left2right_id);
%disp(num2str(e.transform()));

disp('left2kinect_left:');
e = X.get_random_var(left2kinect_left_id);
t = T_opt * e.transform() * inv(T_opt);
disp(['rpy: ' num2str(mat332rpy(t(1:3, 1:3))') ' xyz: ' num2str(t(1:3, 4)')]);

disp('left2kinect_right:');
e = X.get_random_var(left2kinect_right_id);
t = T_opt * e.transform() * inv(T_opt);
disp(['rpy: ' num2str(mat332rpy(t(1:3, 1:3))') ' xyz: ' num2str(t(1:3, 4)')]);

% %disp('left2right:');
% %e = X.get_random_var(left2right_id);
% %disp(num2str(e.transform()));
% 
% disp('head2left:');
% e = X.get_random_var(head2left_id);
% t = e.transform();
% disp(['rpy: ' num2str(mat332rpy(t(1:3, 1:3))') ' xyz: ' num2str(t(1:3, 4)')]);
% 
% disp('left2kinect_left:');
% e = X.get_random_var(left2kinect_left_id);
% t = e.transform();
% disp(['rpy: ' num2str(mat332rpy(t(1:3, 1:3))') ' xyz: ' num2str(t(1:3, 4)')]);
% 
% disp('left2kinect_right:');
% e = X.get_random_var(left2kinect_right_id);
% t = e.transform();
% disp(['rpy: ' num2str(mat332rpy(t(1:3, 1:3))') ' xyz: ' num2str(t(1:3, 4)')]);

plotdata;
