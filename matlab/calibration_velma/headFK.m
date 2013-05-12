% HEADFK

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

function [ trans ] = headFK( q )
%HEADFK Summary of this function goes here
%   Detailed explanation goes here
    %T1 = dh2mat44(0.0, 0.0, 0.0, 0.0);
    
    %T2 = dh2mat44(0.0, 0.0, 0.0, -pi/2);
    
    %t1 = dh_setangle(T1, q(1));[-0.32044243812561035, 0.8583459258079529]
    %t2 = dh_setangle(T2, q(2));
    %trans = t1.m * t2.m;
    
    %ca = cos(0);
    %sa = sin(0);
    
%     cb = cos(q(1));
%     sb = sin(q(1));
%     
%     cy = cos(q(2));
%     sy = sin(q(2));
%     
%     m = zeros(4);
%     m(4, 4) = 1;
%     
%     m(1,1) = ca*cb;
%     m(2,1) = sb*sy - sa*cy;
%     m(3,1) = ca*sb*cy + sa*sy;
%     
%     m(2,1) = sa*cb;
%     m(2,2) = sa*sb*sy + ca*cy;
%     m(2,3) = sa*sb*cy - ca*sy;
%     
%     m(3,1) = -sb;
%     m(3,2) = cb*sy;
%     m(3,3) = cb*cy;
%     
%     trans = m;

trans = dh2mat44(0.0, q(1), 0.0, -pi/2) * dh2mat44(0.0, q(2), 0.0, 0.0);
end

