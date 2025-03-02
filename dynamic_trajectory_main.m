clear; clc; close all;

addpath('D:\workspace\drone\b5763-main\trajectory_vers2\trajectory_vers2');
filename = 'Copy_of_v1.txt.acmi';

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

figure;
hold on;

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

    lon_ref = lon(1);
    lat_ref = lat(1);
    x_m = (lon - lon_ref) * 111320 * cosd(lat_ref);
    y_m = (lat - lat_ref) * 111320;
    z_m = z;

    scale_factor = 1;
    step = 1;
    trail_length = 3;
    selector = 'jet';
    if contains(lower(name), 'f16')
        selector = 'jet';
    end

    xlim([min(x_m) max(x_m)]);
    ylim([min(y_m) max(y_m)]);
    zlim([min(z_m) max(z_m)]);
    title(['Tacview ACMI 数据可视化 (对象: ' obj_id ')']);
    xlabel('X (米)'); ylabel('Y (米)'); zlabel('Z (米)');
    view(82.50, 2);
    grid on;
    daspect([1 1 1]);

    disp(['绘制对象: ' obj_id ', Name: ' name ', Color: ' color]);
    h_traj = plot3(x_m(1), y_m(1), z_m(1), 'b-', 'LineWidth', 2);
    h_models = [];

    for j = 1:length(x_m)
        start_idx = max(1, j - trail_length + 1);
        end_idx = j;

        x_window = x_m(start_idx:end_idx);
        y_window = y_m(start_idx:end_idx);
        z_window = z_m(start_idx:end_idx);
        pitch_window = pitch(start_idx:end_idx);
        roll_window = roll(start_idx:end_idx);
        yaw_window = yaw(start_idx:end_idx);

        set(h_traj, 'XData', x_window, 'YData', y_window, 'ZData', z_window);

        if ~isempty(h_models)
            delete([h_models{:}]);
        end

        [~, h_models] = trajectory_dynamic(x_window, y_window, z_window, pitch_window, roll_window, yaw_window, scale_factor, step, selector, color);

        drawnow;
        pause(0.1);
    end
end
hold off;