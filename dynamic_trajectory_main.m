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
selector_key = 'jet';
jet_file = fullfile(model_path, '80jet.mat');
if exist(jet_file, 'file')
    disp(['加载模型文件: ' jet_file]);
    data = load(jet_file);
    if isfield(data, 'V') && isfield(data, 'F') && isfield(data, 'C')
        models.(selector_key) = struct('V', data.V, 'F', data.F, 'C', data.C);
        models.(selector_key).V = [-data.V(:,2) data.V(:,1) data.V(:,3)];
    else
        error('80jet.mat 文件格式错误，缺少 V, F 或 C 变量');
    end
else
    error('无法找到 80jet.mat 文件，请检查路径: %s', jet_file);
end

% 初始化单一图形窗口
fig = figure;
hold on;

% 处理所有飞机数据
object_ids = fieldnames(objects);
num_objects = length(object_ids);
data = struct();

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

    % 转换为米并缩放
    lon_ref = lon(1);
    lat_ref = lat(1);
    x_m = (lon - lon_ref) * 111320 * cosd(lat_ref) * 150;
    y_m = (lat - lat_ref) * 111320 * 50;
    z_m = z * 290;

    % 参数设置
    scale_factor = 9000; % 放大模型尺寸，确保可见
    step = 5;
    trail_length = min(300, length(x_m));
    selector = 'jet';

    % 初始化轨迹线
    disp(['绘制对象: ' obj_id ', Name: ' name ', Color: ' color]);
    h_traj = plot3(x_m(1), y_m(1), z_m(1), '-', 'LineWidth', 2, 'Color', get_line_color(color));
    
    % 初始化单一 patch 对象
    model_data = models.(selector);
    V_init = model_data.V * scale_factor / max(abs(model_data.V(:))); % 调整初始模型大小
    h_model = patch('Faces', model_data.F, 'Vertices', V_init + [x_m(1) y_m(1) z_m(1)], ...
                    'FaceColor', get_line_color(color), 'FaceVertexCData', model_data.C, 'EdgeColor', 'none');

    % 调试：确认初始模型顶点范围
    V_range = [min(V_init) max(V_init)];
    disp(['初始模型 ' obj_id ' 的顶点范围: ' num2str(V_range)]);

    % 存储数据到结构体（h_model 改为单一对象）
    data.(obj_id) = struct('x_m', x_m, 'y_m', y_m, 'z_m', z_m, ...
                           'pitch', pitch, 'roll', roll, 'yaw', yaw, ...
                           'scale_factor', scale_factor, 'step', step, ...
                           'trail_length', trail_length, 'selector', selector, ...
                           'color', color, 'h_traj', h_traj, 'h_model', h_model, ...
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
light('Position', [1 1 1], 'Style', 'infinite');

% 创建单一 timer 对象
total_steps_array = zeros(1, num_objects);
for i = 1:num_objects
    total_steps_array(i) = data.(object_ids{i}).total_steps;
end
max_steps = max(total_steps_array);
t = timer('ExecutionMode', 'fixedRate', ...
          'Period', 0.2, ...
          'TasksToExecute', max_steps, ...
          'UserData', struct('data', data, 'object_ids', {object_ids}, 'fig', fig, 'models', models), ...
          'TimerFcn', @update_all_trajectories, ...
          'StopFcn', @(obj, event) disp('Timer 已停止'), ...
          'BusyMode', 'drop');

% 设置窗口关闭回调
set(fig, 'CloseRequestFcn', @(src, event) stop_and_clean(t));

% 启动 timer
start(t);

% 异步运行，确保 timer 有时间执行
hold off;
pause(5);

% 更新所有轨迹的回调函数
function update_all_trajectories(timerObj, ~)
    user_data = get(timerObj, 'UserData');
    data = user_data.data;
    object_ids = user_data.object_ids;
    fig = user_data.fig;
    models = user_data.models;

    if ~ishandle(fig) || ~isvalid(fig)
        disp('图形窗口已关闭，停止 timer');
        stop(timerObj);
        return;
    end

    for i = 1:length(object_ids)
        obj_id = object_ids{i};
        obj_data = data.(obj_id);

        if obj_data.current_idx <= obj_data.total_steps
            start_idx = max(1, obj_data.current_idx - obj_data.trail_length + 1);
            end_idx = obj_data.current_idx;

            x_window = obj_data.x_m(start_idx:end_idx);
            y_window = obj_data.y_m(start_idx:end_idx);
            z_window = obj_data.z_m(start_idx:end_idx);
            pitch_window = obj_data.pitch(start_idx:end_idx);
            roll_window = obj_data.roll(start_idx:end_idx);
            yaw_window = obj_data.yaw(start_idx:end_idx);

            % 更新轨迹线
            if isa(obj_data.h_traj, 'matlab.graphics.chart.primitive.Line') && isvalid(obj_data.h_traj)
                set(obj_data.h_traj, 'XData', x_window, 'YData', y_window, 'ZData', z_window);
            else
                disp(['警告: ' obj_id ' 的 h_traj 无效或不是图形对象，类型: ' class(obj_data.h_traj)]);
                continue;
            end

            % 更新单一 patch 对象
            if isa(obj_data.h_model, 'matlab.graphics.primitive.Patch') && isvalid(obj_data.h_model)
                model_data = models.(obj_data.selector);
                V_new = update_model_position(model_data.V, obj_data.scale_factor, pitch_window(end), roll_window(end), yaw_window(end), ...
                                             x_window(end), y_window(end), z_window(end));
                set(obj_data.h_model, 'Vertices', V_new);
                % 调试：检查更新后的顶点范围
                V_range = [min(V_new) max(V_new)];
                disp(['更新 ' obj_id ' 模型后，顶点范围: ' num2str(V_range)]);
            else
                disp(['警告: ' obj_id ' 的 h_model 无效或不是图形对象，类型: ' class(obj_data.h_model)]);
                continue;
            end

            % 更新索引
            obj_data.current_idx = obj_data.current_idx + 1;
            data.(obj_id) = obj_data;
        end
    end

    user_data.data = data;
    set(timerObj, 'UserData', user_data);
    drawnow;
end

% 更新模型位置的子函数
function V_new = update_model_position(V, scale_factor, pitch, roll, yaw, x, y, z)
    % 缩放模型
    correction = max(abs(V(:)));
    V_scaled = V * scale_factor / correction;
    
    % 计算旋转矩阵
    Tbe = [cos(yaw)*cos(pitch) sin(yaw)*cos(pitch) -sin(pitch);
           cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll) sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll) cos(pitch)*sin(roll);
           cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll) sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll) cos(pitch)*cos(roll)];
    
    % 应用旋转和位移
    V_new = V_scaled * Tbe + [x y z];
end

% 清理函数
function stop_and_clean(t)
    if isvalid(t)
        disp('窗口关闭，停止并删除 timer');
        stop(t);
        delete(t);
    end
    if ishandle(gcf)
        delete(gcf);
    end
end

% 辅助函数：根据颜色字符串返回 RGB 值
function rgb = get_line_color(color_str)
    switch lower(color_str)
        case 'red'
            rgb = [1 0 0];
        case 'blue'
            rgb = [0 0 1];
        otherwise
            rgb = [0 0 1];
    end
end