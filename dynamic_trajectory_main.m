clear; clc; close all;

% 添加路径
addpath('D:\workspace\drone\b5763-main\trajectory_vers2\trajectory_vers2');
filename = 'control.txt.acmi';

% 读取文件
fid = fopen(filename, 'r');
if fid == -1
    error('无法打开文件 %s', filename);
end

objects = struct();
while ~feof(fid)
    line = fgetl(fid);
    if ischar(line) && ~isempty(line)
        if startsWith(line, 'FileType') || startsWith(line, 'FileVersion') || startsWith(line, '0,ReferenceTime')
            continue;
        end
        if startsWith(line, '#')
            continue;
        end
        tokens = split(line, ',');
        obj_id = tokens{1};
        if ~isfield(objects, obj_id)
            objects.(obj_id) = struct('x', [], 'y', [], 'z', [], 'pitch', [], 'roll', [], 'yaw', [], 'name', '', 'color', '');
        end
        t_data = split(tokens{2}, '=');
        coords = split(t_data{2}, '|');
        objects.(obj_id).x(end+1) = str2double(coords{1});
        objects.(obj_id).y(end+1) = str2double(coords{2});
        objects.(obj_id).z(end+1) = str2double(coords{3});
        objects.(obj_id).pitch(end+1) = deg2rad(str2double(coords{4}));
        objects.(obj_id).roll(end+1) = deg2rad(str2double(coords{5}));
        objects.(obj_id).yaw(end+1) = deg2rad(str2double(coords{6}));
        for i = 3:length(tokens)
            pair = split(tokens{i}, '=');
            if strcmp(pair{1}, 'Name')
                objects.(obj_id).name = pair{2};
            elseif strcmp(pair{1}, 'Color')
                objects.(obj_id).color = pair{2};
            end
        end
    end
end
fclose(fid);

% 加载模型文件（只加载一次）
model_path = 'D:\workspace\drone\b5763-main\trajectory_vers2\trajectory_vers2';
models = struct();
selector_key = 'jet'; % 使用合法字段名
jet_file = fullfile(model_path, '80jet.mat');
if exist(jet_file, 'file')
    disp(['加载模型文件: ' jet_file]);
    data = load(jet_file);
    if isfield(data, 'V') && isfield(data, 'F') && isfield(data, 'C')
        models.(selector_key) = struct('V', data.V, 'F', data.F, 'C', data.C);
        % 预处理模型数据
        models.(selector_key).V = [-data.V(:,2) data.V(:,1) data.V(:,3)];
    else
        error('80jet.mat 文件格式错误，缺少 V, F 或 C 变量');
    end
else
    error('无法找到 80jet.mat 文件，请检查路径: %s', jet_file);
end

% 初始化图形窗口
fig = figure;
hold on;

% 处理每个对象
object_ids = fieldnames(objects);
for i = 1:length(object_ids)
    obj_id = object_ids{i};
    lon = objects.(obj_id).x;
    lat = objects.(obj_id).y;
    z = objects.(obj_id).z;
    pitch = objects.(obj_id).pitch;
    roll = objects.(obj_id).roll;
    yaw = objects.(obj_id).yaw;
    name = objects.(obj_id).name;
    color = objects.(obj_id).color;

    % 转换为米并缩放
    lon_ref = lon(1);
    lat_ref = lat(1);
    x_m = (lon - lon_ref) * 111320 * cosd(lat_ref) * 150; % X 轴放大 150 倍
    y_m = (lat - lat_ref) * 111320 * 50;                   % Y 轴放大 50 倍
    z_m = z * 900;                                          % Z 轴放大 90 倍

    % 参数设置
    scale_factor = 1;
    step = 1;
    trail_length = 3;
    selector = 'jet'; % 与 models 的字段名一致

    % 设置固定坐标轴范围
    xlim([min(x_m) max(x_m)]);
    ylim([min(y_m) max(y_m)]);
    zlim([min(z_m) max(z_m)]);
    title(['Tacview ACMI 数据可视化 (对象: ' obj_id ')']);
    xlabel('X (米)'); ylabel('Y (米)'); zlabel('Z (米)');
    grid on;
    daspect([1 1 1]);
    view(82.50, 2); % 仅在此处设置初始视角

    % 启用 3D 旋转交互
    rotate3d on;

    % 初始化图形对象
    disp(['绘制对象: ' obj_id ', Name: ' name ', Color: ' color]);
    h_traj = plot3(x_m(1), y_m(1), z_m(1), 'b-', 'LineWidth', 2);
    h_models = [];

    % 调试：确认 h_traj 初始化
    disp(['初始化 ' obj_id ' 的 h_traj 类型: ' class(h_traj)]);
    disp(['初始化 ' obj_id ' 的 h_traj 是否有效: ' num2str(isvalid(h_traj))]);

    % 创建 timer 对象
    t = timer('ExecutionMode', 'fixedRate', ...
              'Period', 0.1, ...
              'TasksToExecute', length(x_m), ...
              'UserData', struct('x_m', x_m, 'y_m', y_m, 'z_m', z_m, ...
                                 'pitch', pitch, 'roll', roll, 'yaw', yaw, ...
                                 'scale_factor', scale_factor, 'step', step, ...
                                 'trail_length', trail_length, 'selector', selector, ...
                                 'color', color, 'h_traj', h_traj, 'h_models', h_models, ...
                                 'current_idx', 1, 'models', models), ...
              'TimerFcn', @update_trajectory);

    % 设置窗口关闭回调
    set(fig, 'CloseRequestFcn', @(src, event) stop_and_clean(t));

    % 启动 timer
    start(t);

    % 等待 timer 完成（可选）
    wait(t);
    delete(t);
end
hold off;

% 更新轨迹的回调函数
function update_trajectory(timerObj, ~)
    % 获取 UserData
    data = get(timerObj, 'UserData');
    j = data.current_idx;
    x_m = data.x_m;
    y_m = data.y_m;
    z_m = data.z_m;
    pitch = data.pitch;
    roll = data.roll;
    yaw = data.yaw;
    scale_factor = data.scale_factor;
    step = data.step;
    trail_length = data.trail_length;
    selector = data.selector;
    color = data.color;
    h_traj = data.h_traj;
    h_models = data.h_models;
    models = data.models;

    % 计算当前窗口
    start_idx = max(1, j - trail_length + 1);
    end_idx = j;

    % 提取窗口数据
    x_window = x_m(start_idx:end_idx);
    y_window = y_m(start_idx:end_idx);
    z_window = z_m(start_idx:end_idx);
    pitch_window = pitch(start_idx:end_idx);
    roll_window = roll(start_idx:end_idx);
    yaw_window = yaw(start_idx:end_idx);

    % 检查轨迹线句柄
    if isa(h_traj, 'matlab.graphics.chart.primitive.Line') && isvalid(h_traj)
        set(h_traj, 'XData', x_window, 'YData', y_window, 'ZData', z_window);
    else
        disp(['警告: h_traj 无效或不是图形对象，类型: ' class(h_traj)]);
        return;
    end

    % 删除旧模型
    if ~isempty(h_models)
        valid_models = cellfun(@(x) isa(x, 'matlab.graphics.primitive.Patch') && isvalid(x), h_models);
        delete([h_models{valid_models}]);
    end

    % 绘制新模型并传递预加载的模型数据
    try
        model_data = models.(selector);
        [~, h_models] = trajectory_dynamic(x_window, y_window, z_window, pitch_window, roll_window, yaw_window, ...
                                          scale_factor, step, model_data.V, model_data.F, model_data.C, color);
    catch e
        disp(['绘制模型时出错: ' e.message]);
        return;
    end

    % 更新 UserData
    data.h_models = h_models;
    data.current_idx = j + 1;
    set(timerObj, 'UserData', data);
end

% 清理函数
function stop_and_clean(t)
    if isvalid(t)
        stop(t);
        delete(t);
    end
    delete(gcf);
end

% 辅助函数：根据颜色字符串返回 RGB 值
function rgb = get_line_color(color_str)
    switch lower(color_str)
        case 'red'
            rgb = [1 0 0];
        case 'blue'
            rgb = [0 0 1];
        otherwise
            rgb = [0 0 1]; % 默认蓝色
    end
end