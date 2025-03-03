function h_models = trajectory_dynamic(x, y, z, pitch, roll, yaw, scale_factor, step, V, F, C, color)
%   function h_models = trajectory_dynamic(x, y, z, pitch, roll, yaw, scale_factor, step, V, F, C, color)
%
%   x, y, z             center trajectory (vector)    [m]
%   pitch, roll, yaw    euler's angles                [rad]
%   scale_factor        normalization factor          [scalar]
%   step                attitude sampling factor      [scalar]
%   V                   model vertices                [matrix]
%   F                   model faces                   [matrix]
%   C                   model color data              [matrix]
%   color               model color                  [string]
%
%   OUTPUT:
%   h_models            handles to model patches (cell array)
%
%   NOTICE: This function now only updates models, not the trajectory line.
%
%   *******************************
%   Function Version: Dynamic (based on trajectory2 v2.0)
%   Modified: March 2025
%   *******************************

% 输入验证
if nargin < 11
    disp('  Error:');
    disp('      Error: Invalid Number Inputs! Expected at least 11 arguments.');
    return;
end
if (length(x) ~= length(y)) || (length(x) ~= length(z)) || (length(y) ~= length(z))
    disp('  Error:');
    disp('      Uncorrect Dimension of the center trajectory Vectors. Please Check the size');
    return;
end
if ((length(pitch) ~= length(roll)) || (length(pitch) ~= length(yaw)) || (length(roll) ~= length(yaw)))
    disp('  Error:');
    disp('      Uncorrect Dimension of the euler''s angle Vectors. Please Check the size');
    return;
end
if length(pitch) ~= length(x)
    disp('  Error:');
    disp('      Size mismatch between euler''s angle vectors and center trajectory vectors');
    return;
end
if step >= length(x)
    step = max(1, length(x)); % 动态调整 step，确保不超出窗口长度
elseif step < 1
    step = 1;
end

% 缩放模型
correction = max(abs(V(:,1)));
V = V ./ (scale_factor * correction);

% 初始化模型句柄数组
ii = length(x);
resto = mod(ii, step);
num_models = floor((ii - resto) / step) + 1;
h_models = cell(num_models, 1);
model_idx = 1;

% 绘制模型
for i = 1:step:(ii-resto)
    theta = pitch(i);
    phi = roll(i);
    psi = yaw(i);
    Tbe = [cos(psi)*cos(theta) sin(psi)*cos(theta) -sin(theta);
           ((cos(psi)*sin(theta)*sin(phi))-(sin(psi)*cos(phi))) ((sin(psi)*sin(theta)*sin(phi))+(cos(psi)*cos(phi))) cos(theta)*sin(phi);
           ((cos(psi)*sin(theta)*cos(phi))+(sin(psi)*sin(phi))) ((sin(psi)*sin(theta)*cos(phi))-(cos(psi)*sin(phi))) cos(theta)*cos(phi)];
    Vnew = V * Tbe;
    rif = [x(i) y(i) z(i)];
    X0 = repmat(rif, size(Vnew,1), 1);
    Vnew = Vnew + X0;

    h_models{model_idx} = patch('faces', F, 'vertices', Vnew);
    if strcmpi(color, 'Red')
        set(h_models{model_idx}, 'facec', [1 0 0]);
    elseif strcmpi(color, 'Blue')
        set(h_models{model_idx}, 'facec', [0 0 1]);
    else
        set(h_models{model_idx}, 'facec', [1 0 0]); % 默认红色
    end
    set(h_models{model_idx}, 'FaceVertexCData', C, 'EdgeColor', 'none');
    model_idx = model_idx + 1;
end

% 移除 plot3，不再更新轨迹线
% 设置图形属性
grid on;
daspect([1 1 1]);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
end