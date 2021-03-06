% PLOTDATA

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
for i=1:num_images_left 
  p_l = project_points(X.get_random_var(armcal_r_id), X.get_random_var(wrist2world_id), X.get_random_var(arm2base_id), X.get_random_var(head2left_id), i_left, mtk.SE3(eye(4)), mtk.SE3(headFK(head_joints_l{i})), right_arm_joints_l{i}, right_arm_joints_trq_l{i}, p_world_l{i});
   
  pm_l = reshape(p_l, 2, []);
  figure('Name','Left camera','NumberTitle','on');
  plot(pm_l(1,:), pm_l(2,:), '*', p_img_l{i}(1,:), p_img_l{i}(2,:), 'r+');
  xlabel('px')
  ylabel('py')
  axis equal
end
p_kl = project_points(X.get_random_var(i_kinect_left_id), X.get_random_var(armcal_r_id), X.get_random_var(wrist2world_id), X.get_random_var(arm2base_id), X.get_random_var(head2left_id), X.get_random_var(left2kinect_left_id), mtk.SE3(headFK(head_joints_kl{1})), right_arm_joints_kl{1}, right_arm_joints_trq_kl{1}, p_world_kl{1});

pm_kl = reshape(p_kl, 2, []);
figure('Name','Left kinect','NumberTitle','off');
plot(pm_kl(1,:), pm_kl(2,:), '*', p_img_kl{1}(1,:), p_img_kl{1}(2,:), 'r+');
xlabel('px')
ylabel('py')
axis equal

p_kr = project_points(X.get_random_var(i_kinect_right_id), X.get_random_var(armcal_r_id), X.get_random_var(wrist2world_id), X.get_random_var(arm2base_id), X.get_random_var(head2left_id), X.get_random_var(left2kinect_right_id), mtk.SE3(headFK(head_joints_kr{1})), right_arm_joints_kr{1}, right_arm_joints_trq_kr{1}, p_world_kr{1});

pm_kr = reshape(p_kr, 2, []);
figure('Name','Right kinect','NumberTitle','off');
plot(pm_kr(1,:), pm_kr(2,:), '*', p_img_kr{1}(1,:), p_img_kr{1}(2,:), 'r+');
xlabel('px')
ylabel('py')
axis equal