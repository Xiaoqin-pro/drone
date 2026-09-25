function setup
%SETUP 将当前Benchmark20的源代码、实验和审计工具加入MATLAB路径。
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src'));
addpath(fullfile(root,'experiments'));
addpath(fullfile(root,'audit'));
end
