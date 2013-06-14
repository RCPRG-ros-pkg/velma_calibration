% COMPUTE_INITIAL_PARAMETERS

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

% intrinsic parameters for each camera
i_left = intrinsic_t();
i_right = intrinsic_t();
i_kinect_left = intrinsic_t();
i_kinect_right = intrinsic_t();

% transformation between both cameras
head2left = mtk.SE3(eye(4));
head2right = mtk.SE3(eye(4));

left2kinect_left = mtk.SE3(eye(4));
left2kinect_right = mtk.SE3(eye(4));

wrist2world = mtk.SE3(eye(4));
arm2base = mtk.SE3(eye(4));

arm2base.pos = [0.279; -0.1; 0.025];
arm2base.Q = quat2mat33([-0.35355 0.35355 -0.61237 0.61237]);

head2left.pos = T_l;
head2left.Q = quat2mat33(R_l);

z = inv(head2left.transform());

head2left.pos = z(1:3,4);
head2left.Q = z(1:3,1:3);

left2kinect_left.pos = T_kl;
left2kinect_left.Q = quat2mat33(R_kl);

z = inv(head2left.transform()) * inv(left2kinect_left.transform());

left2kinect_left.pos = z(1:3,4);
left2kinect_left.Q = z(1:3,1:3);

left2kinect_right.pos = T_kr;
left2kinect_right.Q = quat2mat33(R_kr);

z = inv(head2left.transform()) * inv(left2kinect_right.transform());

left2kinect_right.pos = z(1:3,4);
left2kinect_right.Q = z(1:3,1:3);

% compute initial intrinsic parameters
stacked_homographies_left = [];
%stacked_homographies_right = [];
stacked_homographies_kinect_left = [];
stacked_homographies_kinect_right = [];

stacked_measured_gs = zeros(3, num_images_left);
for k=1:num_images_left
    eval(['Hp_grid_left' num2str(k) ' = p_world_l{' num2str(k) '};']);
    eval(['Hp_grid_left' num2str(k) '(3,:) = 1;']);
    
    eval(['Hp_image_left' num2str(k) ' = p_img_l{' num2str(k) '};']);
    eval(['Hp_image_left' num2str(k) '(3,:) = 1;']);
     
    eval(['H_left = estimate_homography(Hp_image_left' num2str(k) ', Hp_grid_left' num2str(k) ');']);

    stacked_homographies_left((k-1)*3+1:(k-1)*3+3, 1:3) = H_left;
end;

for k=1:num_images_kinect_left
    eval(['Hp_grid_kinect_left' num2str(k) ' = p_world_kl{' num2str(k) '};']);
    eval(['Hp_grid_kinect_left' num2str(k) '(3,:) = 1;']);
    
    eval(['Hp_image_kinect_left' num2str(k) ' = p_img_kl{' num2str(k) '};']);
    eval(['Hp_image_kinect_left' num2str(k) '(3,:) = 1;']);
    
    eval(['H_kinect_left = estimate_homography(Hp_image_kinect_left' num2str(k) ', Hp_grid_kinect_left' num2str(k) ');']);

    stacked_homographies_kinect_left((k-1)*3+1:(k-1)*3+3, 1:3) = H_kinect_left;
end;

for k=1:num_images_kinect_right
    eval(['Hp_grid_kinect_right' num2str(k) ' = p_world_kr{' num2str(k) '};']);
    eval(['Hp_grid_kinect_right' num2str(k) '(3,:) = 1;']);
    
    eval(['Hp_image_kinect_right' num2str(k) ' = p_img_kr{' num2str(k) '};']);
    eval(['Hp_image_kinect_right' num2str(k) '(3,:) = 1;']);
    
    eval(['H_kinect_right = estimate_homography(Hp_image_kinect_right' num2str(k) ', Hp_grid_kinect_right' num2str(k) ');']);

    stacked_homographies_kinect_right((k-1)*3+1:(k-1)*3+3, 1:3) = H_kinect_right;
end;

% compute and set initial intrinsic parameters
A_left = estimate_initial_intrinsic(num_images_left, stacked_homographies_left);
%A_right = estimate_initial_intrinsic(num_images_right, stacked_homographies_right);
A_kinect_left = estimate_initial_intrinsic(num_images_kinect_left, stacked_homographies_kinect_left);
A_kinect_right = estimate_initial_intrinsic(num_images_kinect_right, stacked_homographies_kinect_right);

A_left(1,1) = 1142.59668;
A_left(2,2) = 1193.55139;
A_left(1,3) = 663.19184;
A_left(2,3) = 486.45521;
i_left.focal_length.vec = [A_left(1,1); A_left(2,2)];
i_left.offset.vec = [A_left(1,3); A_left(2,3)];
i_left.distortion.vec = 0.0;

i_right.focal_length.vec = [524.0; 522.0];
i_right.offset.vec = [293.0; 230.0];
i_right.distortion.vec = 0.0;

A_kinect_left(1,1) = 530.98779296875;
A_kinect_left(2,2) = 533.542846679688;
A_kinect_left(1,3) = 327.953265229211;
A_kinect_left(2,3) = 265.187574867246;

i_kinect_left.focal_length.vec = [A_kinect_left(1,1); A_kinect_left(2,2)];
i_kinect_left.offset.vec = [A_kinect_left(1,3); A_kinect_left(2,3)];
i_kinect_left.distortion.vec = 0.0;

A_kinect_right(1,1) = 601.696044921875;
A_kinect_right(2,2) = 610.256958007812;
A_kinect_right(1,3) = 311.996309254406;
A_kinect_right(2,3) = 245.403162674575;

i_kinect_right.focal_length.vec = [A_kinect_right(1,1); A_kinect_right(2,2)];
i_kinect_right.offset.vec = [A_kinect_right(1,3); A_kinect_right(2,3)];
i_kinect_right.distortion.vec = 0.0;

[R, t] = estimate_initial_extrinsic(stacked_homographies_left(1:3, 1:3), A_left);

T = inv([R t; 0 0 0 1]) * inv(head2left.transform()) * inv(headFK([head_joints_l{1}])) * inv(arm2base.transform()) * armFK(right_arm_joints_l{1});

wrist2world.Q = T(1:3, 1:3);
wrist2world.pos = T(1:3, 4);

