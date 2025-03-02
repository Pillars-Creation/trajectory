clear; clc; close all;

% 添加路径
addpath('D:\workspace\drone\b5763-main\trajectory_vers2\trajectory_vers2');
filename = 'v1.txt.acmi';

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

    % 转换为米
    lon_ref = lon(1);
    lat_ref = lat(1);
    x_m = (lon - lon_ref) * 111320 * cosd(lat_ref)*150;
    y_m = (lat - lat_ref) * 111320*50;
    z_m = z*90;

    % 参数设置
    scale_factor = 1;
    step = 1;
    trail_length = 3;
    selector = 'jet';
    if contains(lower(name), 'f16')
        selector = 'jet';
    end

    % 设置固定坐标轴范围
    xlim([min(x_m) max(x_m)]);
    ylim([min(y_m) max(y_m)]);
    zlim([min(z_m) max(z_m)]);
    title(['Tacview ACMI 数据可视化 (对象: ' obj_id ')']);
    xlabel('X (米)'); ylabel('Y (米)'); zlabel('Z (米)');
    view(82.50, 2);
    grid on;
    daspect([1 1 1]);

    % 启用 3D 旋转交互
    rotate3d on;

    % 初始化图形对象
    disp(['绘制对象: ' obj_id ', Name: ' name ', Color: ' color]);
    h_traj = plot3(x_m(1), y_m(1), z_m(1), 'b-', 'LineWidth', 2);
    h_models = [];

    % 创建 timer 对象
    t = timer('ExecutionMode', 'fixedRate', ...
              'Period', 0.1, ... % 更新间隔 0.1 秒
              'TasksToExecute', length(x_m), ... % 执行次数
              'UserData', struct('x_m', x_m, 'y_m', y_m, 'z_m', z_m, ...
                                 'pitch', pitch, 'roll', roll, 'yaw', yaw, ...
                                 'scale_factor', scale_factor, 'step', step, ...
                                 'trail_length', trail_length, 'selector', selector, ...
                                 'color', color, 'h_traj', h_traj, 'h_models', h_models, ...
                                 'current_idx', 1), ...
              'TimerFcn', @update_trajectory);

    % 启动 timer
    start(t);

    % 等待 timer 完成（可选，若需要顺序执行多个对象）
    wait(t);
    delete(t); % 清理 timer
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

    % 更新轨迹线
    set(h_traj, 'XData', x_window, 'YData', y_window, 'ZData', z_window);

    % 删除旧模型
    if ~isempty(h_models)
        delete([h_models{:}]);
    end

    % 绘制新模型
    [~, h_models] = trajectory_dynamic(x_window, y_window, z_window, pitch_window, roll_window, yaw_window, scale_factor, step, selector, color);

    % 更新 UserData
    data.h_models = h_models;
    data.current_idx = j + 1;
    set(timerObj, 'UserData', data);
end