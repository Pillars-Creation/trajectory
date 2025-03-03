function [h_traj, h_models] = trajectory_dynamic(x, y, z, pitch, roll, yaw, scale_factor, step, varargin)
%   function [h_traj, h_models] = trajectory_dynamic(x, y, z, pitch, roll, yaw, scale_factor, step, selector, color)
%
%   x, y, z             center trajectory (vector)    [m]
%   pitch, roll, yaw    euler's angles                [rad]
%   scale_factor        normalization factor          [scalar]
%   step                attitude sampling factor      [scalar]
%   OPTIONAL INPUT:
%   selector            select the body model         [string]
%                       (e.g., 'A-10', 'jet', 'shuttle', etc.)
%   color               model color                  [string]
%                       (e.g., 'Red', 'Blue')
%
%   OUTPUT:
%   h_traj              handle to trajectory line
%   h_models            handles to model patches (cell array)
%
%   NOTICE: If selector is omitted, use the stylized body model from trajectory_old.
%           If color is omitted, default to red.
%
%   *******************************
%   Function Version: Dynamic (based on trajectory2 v2.0)
%   Modified: March 2025
%   *******************************

% 输入验证
if nargin < 8
    disp('  Error:');
    disp('      Error: Invalid Number Inputs!');
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
if nargin > 10
    disp('  Error:');
    disp('      Too much input arguments!');
    return;
end

% 处理可选参数
if nargin == 8
    trajectory_old(x, y, z, pitch, roll, yaw, scale_factor, step);
    return;
else
    selector = cell2mat(varargin(1));
    if nargin == 10
        color = cell2mat(varargin(2));
    else
        color = 'Red';
    end
end

% 加载模型
cur_dir = pwd;
model_path = 'D:\workspace\drone\b5763-main\trajectory_vers2\trajectory_vers2'; % 模型文件路径
disp(['尝试加载模型文件: ' fullfile(model_path, [selector '.mat'])]); % 调试路径
if strcmp(selector, 'shuttle')
    data = load(fullfile(model_path, 'shuttle.mat')); % 使用结构体加载，避免变量冲突
    V = data.V;
    F = data.F;
    C = data.C;
    V = [-V(:,2) V(:,1) V(:,3)];
    V(:,1) = V(:,1) - round(sum(V(:,1))/size(V,1));
    V(:,2) = V(:,2) - round(sum(V(:,2))/size(V,1));
    V(:,3) = V(:,3) - round(sum(V(:,3))/size(V,1));
elseif strcmp(selector, 'helicopter')
    data = load(fullfile(model_path, 'helicopter.mat'));
    V = data.V;
    F = data.F;
    C = data.C;
    V = [-V(:,2) V(:,1) V(:,3)];
    V(:,1) = V(:,1) - round(sum(V(:,1))/size(V,1));
    V(:,2) = V(:,2) - round(sum(V(:,2))/size(V,1));
    V(:,3) = V(:,3) - round(sum(V(:,3))/size(V,1));
elseif strcmp(selector, '747')
    data = load(fullfile(model_path, 'boeing_747.mat'));
    V = data.V;
    F = data.F;
    C = data.C;
    V = [V(:,2) V(:,1) V(:,3)];
    V(:,1) = V(:,1) - round(sum(V(:,1))/size(V,1));
    V(:,2) = V(:,2) - round(sum(V(:,2))/size(V,1));
    V(:,3) = V(:,3) - round(sum(V(:,3))/size(V,1));
elseif strcmp(selector, 'biplane')
    data = load(fullfile(model_path, 'biplane.mat'));
    V = data.V;
    F = data.F;
    C = data.C;
    V = [-V(:,2) V(:,1) V(:,3)];
    V(:,1) = V(:,1) - round(sum(V(:,1))/size(V,1));
    V(:,2) = V(:,2) - round(sum(V(:,2))/size(V,1));
    V(:,3) = V(:,3) - round(sum(V(:,3))/size(V,1));
elseif strcmp(selector, 'md90')
    data = load(fullfile(model_path, 'md90.mat'));
    V = data.V;
    F = data.F;
    C = data.C;
    V = [-V(:,1) V(:,2) V(:,3)];
    V(:,1) = V(:,1) - round(sum(V(:,1))/size(V,1));
    V(:,2) = V(:,2) - round(sum(V(:,2))/size(V,1));
    V(:,3) = V(:,3) - round(sum(V(:,3))/size(V,1));
elseif strcmp(selector, 'dc10')
    data = load(fullfile(model_path, 'dc10.mat'));
    V = data.V;
    F = data.F;
    C = data.C;
    V = [V(:,2) V(:,1) V(:,3)];
    V(:,1) = V(:,1) - round(sum(V(:,1))/size(V,1));
    V(:,2) = V(:,2) - round(sum(V(:,2))/size(V,1));
    V(:,3) = V(:,3) - round(sum(V(:,3))/size(V,1));
elseif strcmp(selector, 'ah64')
    data = load(fullfile(model_path, 'ah64.mat'));
    V = data.V;
    F = data.F;
    C = data.C;
    V = [V(:,2) V(:,1) V(:,3)];
    V(:,1) = V(:,1) - round(sum(V(:,1))/size(V,1));
    V(:,2) = V(:,2) - round(sum(V(:,2))/size(V,1));
    V(:,3) = V(:,3) - round(sum(V(:,3))/size(V,1));
elseif strcmp(selector, 'mig')
    data = load(fullfile(model_path, 'mig.mat'));
    V = data.V;
    F = data.F;
    C = data.C;
    V = [V(:,2) V(:,1) V(:,3)];
elseif strcmp(selector, 'tomcat')
    data = load(fullfile(model_path, 'tomcat.mat'));
    V = data.V;
    F = data.F;
    C = data.C;
    V = [-V(:,2) V(:,1) V(:,3)];
    V(:,1) = V(:,1) - round(sum(V(:,1))/size(V,1));
    V(:,2) = V(:,2) - round(sum(V(:,2))/size(V,1));
    V(:,3) = V(:,3) - round(sum(V(:,3))/size(V,1));
elseif strcmp(selector, 'jet')
    jet_file = fullfile(model_path, '80jet.mat');
    if exist(jet_file, 'file')
        data = load(jet_file); % 使用结构体加载，避免变量冲突
        if isfield(data, 'V') && isfield(data, 'F') && isfield(data, 'C')
            V = data.V;
            F = data.F;
            C = data.C;
            V = [-V(:,2) V(:,1) V(:,3)];
        else
            error('80jet.mat 文件格式错误，缺少 V, F 或 C 变量');
        end
    else
        error('无法找到 80jet.mat 文件，请检查路径: %s', jet_file);
    end
elseif strcmp(selector, 'cessna')
    data = load(fullfile(model_path, '83plane.mat'));
    V = data.V;
    F = data.F;
    C = data.C;
    V = [-V(:,2) V(:,1) V(:,3)];
elseif strcmp(selector, 'A-10')
    data = load(fullfile(model_path, 'A-10.mat'));
    V = data.V;
    F = data.F;
    C = data.C;
    V = [V(:,3) V(:,1) V(:,2)];
else
    try
        data = load(fullfile(model_path, [selector '.mat']));
        V = data.V;
        F = data.F;
        C = data.C;
        V(:,1) = V(:,1) - round(sum(V(:,1))/size(V,1));
        V(:,2) = V(:,2) - round(sum(V(:,2))/size(V,1));
        V(:,3) = V(:,3) - round(sum(V(:,3))/size(V,1));
    catch
        disp(['Warning: ' selector ' not found. Default=A-10']);
        data = load(fullfile(model_path, 'A-10.mat'));
        V = data.V;
        F = data.F;
        C = data.C;
        V = [V(:,3) V(:,1) V(:,2)];
    end
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

% 绘制轨迹线
h_traj = plot3(x, y, z, 'b-', 'LineWidth', 2);

% 设置图形属性
%light;
grid on;
view(82.50, 2);
daspect([1 1 1]);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
cd(cur_dir);

% 子函数：trajectory_old
function trajectory_old(x, y, z, pitch, roll, yaw, scale_factor, step)
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
        step = max(1, length(x));
    end
    if step < 1
        step = 1;
    end

    [xxx, yyy, zzz] = miss_shape;
    correction = 10;
    xxx = -xxx / (scale_factor * correction);
    yyy = yyy / (scale_factor * correction);
    zzz = zzz / (scale_factor * correction);

    ii = length(x);
    resto = mod(ii, step);
    for i = 1:step:(ii-resto)
        theta = pitch(i);
        phi = roll(i);
        psi = yaw(i);
        Tbe = [cos(psi)*cos(theta) sin(psi)*cos(theta) -sin(theta);
               ((cos(psi)*sin(theta)*sin(phi))-(sin(psi)*cos(phi))) ((sin(psi)*sin(theta)*sin(phi))+(cos(psi)*cos(phi))) cos(theta)*sin(phi);
               ((cos(psi)*sin(theta)*cos(phi))+(sin(psi)*sin(phi))) ((sin(psi)*sin(theta)*cos(phi))-(cos(psi)*sin(phi))) cos(theta)*cos(phi)];
        x_hat = 0 .* xxx;
        y_hat = 0 .* yyy;
        z_hat = 0 .* zzz;
        for iii = 1:size(xxx,1)
            for jjj = 1:size(xxx,2)
                tmp_b = [xxx(iii,jjj); yyy(iii,jjj); zzz(iii,jjj)];
                tmp_e = Tbe * tmp_b;
                x_hat(iii,jjj) = x(i) + tmp_e(1,1);
                y_hat(iii,jjj) = y(i) + tmp_e(2,1);
                z_hat(iii,jjj) = z(i) + tmp_e(3,1);
            end
        end
        plot3(x_hat, y_hat, z_hat);
        hold on;
        patch(x_hat, y_hat, z_hat, [1 0 0]);
    end
    axis equal;
    hold on;
    plot3(x, y, z);
    %light;
    grid on;
    view(82.50, 2);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
end

% 子函数：miss_shape
function [x, y, z] = miss_shape
    num = 30;
    count = 1;
    theta = [360/2/num:360/num:(360+360/2/num)] * pi/180;
    len = 25.7;
    radius = 1.5/2;
    s_fore = 5;
    thr_len = 1.4;
    rad_throt = 1.3/2;
    l_fore = len - s_fore - thr_len;
    c_g = 14;

    yc_range = radius * sin(theta);
    zc_range = -radius * cos(theta);
    for i = 1:num
        xcraft{i} = [s_fore s_fore s_fore+l_fore s_fore+l_fore] - c_g;
        ycraft{i} = [yc_range(i) yc_range(i+1) yc_range(i+1) yc_range(i)];
        zcraft{i} = [zc_range(i) zc_range(i+1) zc_range(i+1) zc_range(i)];
    end
    count = num + 1;

    yc_range2 = rad_throt * sin(theta);
    zc_range2 = -rad_throt * cos(theta);
    for i = 1:num
        xcraft{count} = [len-thr_len len-thr_len len len] - c_g;
        ycraft{count} = [yc_range(i) yc_range(i+1) yc_range2(i+1) yc_range2(i)];
        zcraft{count} = [zc_range(i) zc_range(i+1) zc_range2(i+1) zc_range2(i)];
        count = count + 1;
    end

    for i = 1:num
        xcraft{count} = [s_fore s_fore 0 s_fore] - c_g;
        ycraft{count} = [yc_range(i) yc_range(i+1) 0 yc_range(i)];
        zcraft{count} = [zc_range(i) zc_range(i+1) 0 zc_range(i)];
        count = count + 1;
    end

    xcraft{count} = [10.2 13.6 14.6 15] - c_g;
    ycraft{count} = [-zc_range(1) -zc_range(1)+1.5 -zc_range(1)+1.5 -zc_range(1)];
    zcraft{count} = [0 0 0 0];
    xcraft{count+1} = xcraft{count};
    ycraft{count+1} = -ycraft{count};
    zcraft{count+1} = zcraft{count};
    count = count + 2;

    xcraft{count} = [22.1 22.9 23.3 23.3] - c_g;
    ycraft{count} = [-zc_range(1) -zc_range(1)+1.1 -zc_range(1)+1.1 -zc_range(1)];
    zcraft{count} = [0 0 0 0];
    xcraft{count+1} = xcraft{count};
    ycraft{count+1} = -ycraft{count};
    zcraft{count+1} = zcraft{count};
    xcraft{count+2} = xcraft{count};
    ycraft{count+2} = zcraft{count};
    zcraft{count+2} = ycraft{count};
    count = count + 2;

    x = []; y = []; z = [];
    roll = [1 0 0; 0 cos(0/180*pi) sin(0/180*pi); 0 -sin(0/180*pi) cos(0/180*pi)];
    for i = 1:count
        x = [x xcraft{i}'];
        y = [y ycraft{i}'];
        z = [z zcraft{i}'];
    end

    for i = 1:4
        dum = [x(i,:); y(i,:); z(i,:)];
        dum = roll * dum;
        x(i,:) = dum(1,:);
        y(i,:) = dum(2,:);
        z(i,:) = dum(3,:);
    end
end
end