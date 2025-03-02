clear; clc; close all;

% 添加路径（确保模型文件可加载）
addpath('D:\workspace\drone\b5763-main\trajectory_vers2\trajectory_vers2');

% 文件路径
filename = 'Copy_of_v1.txt.acmi';

% 读取文件
fid = fopen(filename, 'r');
if fid == -1
    error('无法打开文件 %s', filename);
end

% 初始化存储数据的结构体
objects = struct();

% 解析文件
while ~feof(fid)
    line = fgetl(fid);
    if ischar(line) && ~isempty(line)
        if startsWith(line, 'FileType') || startsWith(line, 'FileVersion') || startsWith(line, '0,ReferenceTime')
            continue;
        end
        if startsWith(line, '#')
            current_time = str2double(strrep(line, '#', ''));
            continue;
        end
        tokens = split(line, ',');
        obj_id = tokens{1};
        if ~isfield(objects, obj_id)
            objects.(obj_id) = struct('x', [], 'y', [], 'z', [], 'pitch', [], 'roll', [], 'yaw', [], 'name', '', 'color', '');
        end
        t_data = split(tokens{2}, '=');
        coords = split(t_data{2}, '|');
        objects.(obj_id).x(end+1) = str2double(coords{1}); % 经度
        objects.(obj_id).y(end+1) = str2double(coords{2}); % 纬度
        objects.(obj_id).z(end+1) = str2double(coords{3}); % 高度
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

% 为每个对象转换坐标并绘制
object_ids = fieldnames(objects);
for i = 1:length(object_ids)
    obj_id = object_ids{i};
    lon = objects.(obj_id).x; % 经度
    lat = objects.(obj_id).y; % 纬度
    z = objects.(obj_id).z;   % 高度
    pitch = objects.(obj_id).pitch;
    roll = objects.(obj_id).roll;
    yaw = objects.(obj_id).yaw;
    name = objects.(obj_id).name;
    color = objects.(obj_id).color;

    % 选择参考点（第一个点）
    lon_ref = lon(1);
    lat_ref = lat(1);

    % 转换为米（简化公式）
    x_m = (lon - lon_ref) * 111320 * cosd(lat_ref)*10; % 经度变化转为米
    y_m = (lat - lat_ref) * 111320;                  % 纬度变化转为米
    z_m = z*10;                                         % 高度保持不变

    % 参数设置
    scale_factor = 1;
    step = 1;
    selector = 'jet';
    if contains(lower(name), 'f16')
        selector = 'jet';
    end

    % 调用 trajectory2
    disp(['绘制对象: ' obj_id ', Name: ' name ', Color: ' color]);
    trajectory2(x_m, y_m, z_m, pitch, roll, yaw, scale_factor, step, selector);
end

% 添加标题和调整视图
title('Tacview ACMI 数据可视化 (单位：米)');
xlabel('X (米)');
ylabel('Y (米)');
zlabel('Z (米)');
view(82.50, 2);