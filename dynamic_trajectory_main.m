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

% 初始化单一图形窗口
fig = figure;
hold on;

% 处理所有飞机数据
object_ids = fieldnames(objects);
num_objects = length(object_ids);
data = struct(); % 存储每个飞机的绘制数据

% 计算全局坐标轴范围
global_x_min = Inf; global_x_max = -Inf;
global_y_min = Inf; global_y_max = -Inf;
global_z_min = Inf; global_z_max = -Inf;

for i = 1:num_objects
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

    % 初始化轨迹线
    disp(['绘制对象: ' obj_id ', Name: ' name ', Color: ' color]);
    h_traj = plot3(x_m(1), y_m(1), z_m(1), '-', 'LineWidth', 2, ...
                   'Color', get_line_color(color)); % 根据颜色设置轨迹线
    h_models = [];

    % 存储数据到结构体
    data.(obj_id) = struct('x_m', x_m, 'y_m', y_m, 'z_m', z_m, ...
                           'pitch', pitch, 'roll', roll, 'yaw', yaw, ...
                           'scale_factor', scale_factor, 'step', step, ...
                           'trail_length', trail_length, 'selector', selector, ...
                           'color', color, 'h_traj', h_traj, 'h_models', h_models, ...
                           'current_idx', 1, 'total_steps', length(x_m));

    % 更新全局范围
    global_x_min = min(global_x_min, min(x_m));
    global_x_max = max(global_x_max, max(x_m));
    global_y_min = min(global_y_min, min(y_m));
    global_y_max = max(global_y_max, max(y_m));
    global_z_min = min(global_z_min, min(z_m));
    global_z_max = max(global_z_max, max(z_m));
end

% 设置全局坐标轴范围
xlim([global_x_min global_x_max]);
ylim([global_y_min global_y_max]);
zlim([global_z_min global_z_max]);
title('Tacview ACMI 数据可视化 (多飞机轨迹)');
xlabel('X (米)'); ylabel('Y (米)'); zlabel('Z (米)');
view(82.50, 2);
grid on;
daspect([1 1 1]);

% 启用 3D 旋转交互
rotate3d on;

% 创建 timer 对象
% 修复：将 total_steps 收集到一个数组
total_steps_array = zeros(1, num_objects);
for i = 1:num_objects
    total_steps_array(i) = data.(object_ids{i}).total_steps;
end
max_steps = max(total_steps_array); % 取最长轨迹步数
t = timer('ExecutionMode', 'fixedRate', ...
          'Period', 0.1, ... % 更新间隔 0.1 秒
          'TasksToExecute', max_steps, ...
          'UserData', struct('data', data, 'object_ids', {object_ids}), ...
          'TimerFcn', @update_all_trajectories);

% 启动 timer
start(t);

% 等待 timer 完成（可选）
wait(t);
delete(t);
hold off;

% 更新所有轨迹的回调函数
function update_all_trajectories(timerObj, ~)
    % 获取 UserData
    user_data = get(timerObj, 'UserData');
    data = user_data.data;
    object_ids = user_data.object_ids;

    for i = 1:length(object_ids)
        obj_id = object_ids{i};
        obj_data = data.(obj_id);

        % 检查是否还有点需要更新
        if obj_data.current_idx <= obj_data.total_steps
            % 计算当前窗口
            start_idx = max(1, obj_data.current_idx - obj_data.trail_length + 1);
            end_idx = obj_data.current_idx;

            % 提取窗口数据
            x_window = obj_data.x_m(start_idx:end_idx);
            y_window = obj_data.y_m(start_idx:end_idx);
            z_window = obj_data.z_m(start_idx:end_idx);
            pitch_window = obj_data.pitch(start_idx:end_idx);
            roll_window = obj_data.roll(start_idx:end_idx);
            yaw_window = obj_data.yaw(start_idx:end_idx);

            % 更新轨迹线
            set(obj_data.h_traj, 'XData', x_window, 'YData', y_window, 'ZData', z_window);

            % 删除旧模型
            if ~isempty(obj_data.h_models)
                delete([obj_data.h_models{:}]);
            end

            % 绘制新模型
            [~, h_models] = trajectory_dynamic(x_window, y_window, z_window, pitch_window, roll_window, yaw_window, ...
                                               obj_data.scale_factor, obj_data.step, obj_data.selector, obj_data.color);

            % 更新数据
            obj_data.h_models = h_models;
            obj_data.current_idx = obj_data.current_idx + 1;
            data.(obj_id) = obj_data;
        end
    end

    % 更新 UserData
    user_data.data = data;
    set(timerObj, 'UserData', user_data);
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