function main()
    % DH Parameter Calibration - Student TODO Version (Cube Corner Points)
    fprintf('DH Parameter Calibration - 立方体顶点标定\n');
    fprintf('==========================================\n');
    
    % TODO 1: 请输入您的小组编号 (1-8)
    group_number = 4; % 请修改为您的小组编号
    
    if group_number < 1 || group_number > 8
        error('请设置正确的小组编号 (1-8)');
    end
    
    % TODO 2: 请设置立方体中心位置 (x, y, z) 单位：米
    % 建议范围: x: 0.12-0.20, y: -0.05-0.05, z: 0.08-0.12
    cube_center = [0.2, -0.17, -0.10]; % 请设置立方体中心位置
    
    if all(cube_center == 0)
        error('请设置立方体中心位置 (TODO 2)');
    end
    
    % 立方体顶面四个顶点的空间位置 (边长0.08m)
    half_edge = 0.04; % 边长的一半 (0.08/2)
    
    cube_corners = [
        cube_center + [-half_edge, -half_edge, half_edge];  % 顶点1: 左前上
        cube_center + [half_edge,  -half_edge, half_edge];  % 顶点2: 右前上  
        cube_center + [half_edge,   half_edge, half_edge];  % 顶点3: 右后上
        cube_center + [-half_edge,  half_edge, half_edge];  % 顶点4: 左后上
    ];
    
    fprintf('立方体中心: [%.3f, %.3f, %.3f]\n', cube_center);
    fprintf('立方体顶面四个顶点位置:\n');
    for i = 1:4
        fprintf('顶点%d: [%.3f, %.3f, %.3f]\n', i, cube_corners(i,1), cube_corners(i,2), cube_corners(i,3));
    end
    
    % TODO 3: 请输入机械臂到达四个顶点时测量的关节角度
    % 学生需要将机械臂末端移动到每个顶点，并记录关节角度
    measured_joint_angles = zeros(4, 6); % 4个点，每个点6个关节角度
    
    % 顶点1的关节角度
    measured_joint_angles(1, :) = [-0.866, 2.598, -1.511, -0.119, 2.021, 0.0]; % 请输入测量值
    
    % 顶点2的关节角度  
    measured_joint_angles(2, :) = [-0.696, 2.836, -2.055, -0.085, 2.156, 0.0]; % 请输入测量值
    
    % 顶点3的关节角度
    measured_joint_angles(3, :) = [-0.459, 2.734, -1.817, -0.085, 2.293, 0.0]; % 请输入测量值
    
    % 顶点4的关节角度
    measured_joint_angles(4, :) = [-0.662, 2.395, -1.036, 0.0, 1.851, 0.0]; % 请输入测量值
    
    % 检查是否输入了测量数据
    if all(measured_joint_angles(:) == 0)
        error('请输入四个顶点的测量关节角度 (TODO 3)');
    end
    
    fprintf('小组编号: %d\n', group_number);
    fprintf('测量的关节角度:\n');
    for i = 1:4
        fprintf('顶点%d: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', i, measured_joint_angles(i,:));
    end
    
    [dh_initial, dh_calibrated, improvement] = calibrate_dh_with_cube_corners(cube_corners, measured_joint_angles, group_number);
    
    % Save results
    filename = sprintf('calibrated_dh_cube_group_%d.mat', group_number);
    save(filename, 'dh_initial', 'dh_calibrated', 'improvement', 'cube_corners', 'measured_joint_angles');
    fprintf('Results saved to: %s\n', filename);
end

function [dh_initial, dh_calibrated, improvement] = calibrate_dh_with_cube_corners(cube_corners, measured_joint_angles, group_number)
    % 使用立方体顶点数据进行DH参数标定
    
    % Initial DH parameters [a, alpha, d, theta_offset]
    dh_initial = [
        0.0,  pi/2,  0.0,  pi;      % Link 1: a1, α1, d1, θ1_offset
        0.17, 0.0,     0.0,  0.0;       % Link 2: a2, α2, d2, θ2_offset
        0.07, -pi/2, 0.0,  pi/2;    % Link 3: a3, α3, d3, θ3_offset
        0.0,  pi/2,  0.11, 0;       % Link 4: a4, α4, d4, θ4_offset
        0.0,  -pi/2, 0.0,  -pi/2;   % Link 5: a5, α5, d5, θ5_offset
        0.0,  0.0,     0.09, 0.0;       % Link 6: a6, α6, d6, θ6_offset
    ];
    
    % TODO 4: 使用正运动学计算四个顶点的完整位姿
    reference_poses = zeros(4, 12); % 4个位姿，每个位姿12个元素(3位置+9姿态)
    
    fprintf('\n=== 位姿分析 ===\n');
    for i = 1:4
        T_reference = forward_kinematics(dh_initial, measured_joint_angles(i,:));
        
        if isempty(T_reference)
            error('请完成TODO 4: 计算第%d个顶点的参考位姿', i);
        end
        
        % 从变换矩阵提取位置和姿态
        reference_position = T_reference(1:3, 4)';
        reference_orientation = T_reference(1:3, 1:3);
        reference_poses(i, :) = [reference_position, reshape(reference_orientation, 1, 9)];
        
        fprintf('顶点%d:\n', i);
        fprintf('  测量位置: [%.6f, %.6f, %.6f]\n', cube_corners(i,:));
        fprintf('  计算位置: [%.6f, %.6f, %.6f]\n', reference_position);
        fprintf('  位置误差: %.6f m\n', norm(cube_corners(i,:) - reference_position));
    end
    
    % TODO 5: 生成用于标定的随机位姿数据
    [all_joint_configs, all_target_poses] = generate_calibration_data_from_cube_corners(cube_corners, measured_joint_angles, reference_poses, group_number);
    
    % All 24 DH parameters to calibrate
    param_indices = [
        1, 1; 1, 2; 1, 3; 1, 4;  % Link 1: a1, α1, d1, θ_offset1
        2, 1; 2, 2; 2, 3; 2, 4;  % Link 2: a2, α2, d2, θ_offset2
        3, 1; 3, 2; 3, 3; 3, 4;  % Link 3: a3, α3, d3, θ_offset3
        4, 1; 4, 2; 4, 3; 4, 4;  % Link 4: a4, α4, d4, θ_offset4
        5, 1; 5, 2; 5, 3; 5, 4;  % Link 5: a5, α5, d5, θ_offset5
        6, 1; 6, 2; 6, 3; 6, 4;  % Link 6: a6, α6, d6, θ_offset6
    ];
    
    % Iterative calibration
    dh_calibrated = iterative_calibration(dh_initial, all_joint_configs, param_indices, all_target_poses);
    
    % TODO 6: 计算标定改善程度 (完成这部分代码)
    [error_before, error_after] = calculate_pose_errors(dh_initial, dh_calibrated, all_joint_configs, all_target_poses);
    improvement = 100*(error_after-error_before)/error_before; % 请计算改善百分比: improvement = ?
    
    % Display results
    display_results(dh_initial, dh_calibrated, error_before, error_after, improvement);
    
    % Verify with known points
    fprintf('\n=== 验证结果 ===\n');
    for i = 1:4
        T_calibrated = forward_kinematics(dh_calibrated, measured_joint_angles(i,:));
        calibrated_pos = T_calibrated(1:3, 4)';
        fprintf('顶点%d - 测量: [%.6f, %.6f, %.6f], 标定后: [%.6f, %.6f, %.6f], 误差: %.6f m\n', ...
            i, cube_corners(i,:), calibrated_pos, norm(cube_corners(i,:) - calibrated_pos));
    end
end

function [all_joint_configs, all_target_poses] = generate_calibration_data_from_cube_corners(cube_corners, measured_joint_angles, reference_poses, group_number)
    % TODO 4: 基于立方体顶点生成标定数据
    
    % 设置基于小组编号的随机种子
    rng(group_number * 456); % 每个小组有不同的随机种子
    
    all_joint_configs = [];
    all_target_poses = [];
    
    % 1. 使用四个已知顶点数据（用实际测量位置修正）
    for i = 1:4
        all_joint_configs = [all_joint_configs; measured_joint_angles(i,:)];
        
        % 使用实际测量位置 + 计算出的姿态
        reference_orientation = reshape(reference_poses(i, 4:12), 3, 3);
        corrected_pose = [cube_corners(i,:), reshape(reference_orientation, 1, 9)];
        all_target_poses = [all_target_poses; corrected_pose];
    end
    
    % 2. 为每个顶点生成10个变化点
    for corner_idx = 1:4
        for variation_idx = 1:10
            % TODO 4a: 生成关节角度变化 (完成这部分代码)
            % 提示: 在每个顶点的测量关节角度基础上添加随机变化
            % 小幅变化: ±0.05弧度, 中等变化: ±0.10弧度
            
            if variation_idx <= 6  % 前6个点用小幅变化
                joint_variation = (rand(1,6) - 0.5) * 0.10;  % ±0.05弧度
            else  % 后4个点用中等变化
                joint_variation = (rand(1,6) - 0.5) * 0.20;  % ±0.10弧度  
            end

            % 在基础关节角度上添加变化
            varied_config = measured_joint_angles(corner_idx,:) + joint_variation;
        
            % 应用关节限制 (±2.5弧度)
            varied_config = max(-2.5, min(2.5, varied_config));
        
            % 添加到数据集
            all_joint_configs = [all_joint_configs; varied_config];
            
            % TODO 4b: 生成接近该顶点的目标位姿 (完成这部分代码)
            reference_pose = reference_poses(corner_idx, :);
            target_pose = generate_close_target_pose_for_cube(reference_pose, variation_idx, group_number, corner_idx);
            all_target_poses = [all_target_poses; target_pose];
        end
    end 
    
    fprintf('为小组 %d 生成了 %d 个标定配置 (4个顶点 + 40个变化点)\n', group_number, size(all_joint_configs, 1));
end

function target_pose = generate_close_target_pose_for_cube(reference_pose, variation_idx, group_number, corner_idx)
    % 为立方体顶点生成接近的目标位姿
    
    % 使用小组编号、顶点索引和变化索引设置局部随机种子
    rng(group_number * 10000 + corner_idx * 100 + variation_idx);
    
    reference_position = reference_pose(1:3);
    reference_orientation = reshape(reference_pose(4:12), 3, 3);
    
    % TODO 6: 生成位置和姿态变化 (完成这部分代码)
    if variation_idx <= 6  % 小幅变化
        % 提示: pos_variation = 0.003; rot_variation = 0.008;
        pos_variation = 0.003;    % 请修改: 位置变化范围 (建议0.003, 即±3mm)
        rot_variation = 0.008;    % 请修改: 姿态变化范围 (建议0.008, 即±0.008弧度)
    else  % 中等变化  
        % 提示: pos_variation = 0.008; rot_variation = 0.015;
        pos_variation = 0.008;    % 请修改: 位置变化范围 (建议0.008, 即±8mm)
        rot_variation = 0.015;    % 请修改: 姿态变化范围 (建议0.015, 即±0.015弧度)
    end
    
    % TODO 7: 生成位置噪声 (完成这部分代码)
    pos_noise = (rand(1,3) - 0.5) * 2 * pos_variation;
    target_position = reference_position + pos_noise;
    
    % TODO 8: 生成姿态变化 (完成这部分代码)
    angle_variations = (rand(1,3) - 0.5) * 2 * rot_variation;
    
    
    % 创建旋转扰动
    R_perturbation = angle_axis_to_rotation_matrix([1,0,0], angle_variations(1)) * ...
                     angle_axis_to_rotation_matrix([0,1,0], angle_variations(2)) * ...
                     angle_axis_to_rotation_matrix([0,0,1], angle_variations(3));
    
    target_orientation = reference_orientation * R_perturbation;
    
    % 组合成位姿
    target_pose = [target_position, reshape(target_orientation, 1, 9)];
end

function R = angle_axis_to_rotation_matrix(axis, angle)
    % Convert angle-axis to rotation matrix
    if norm(axis) == 0
        R = eye(3);
        return;
    end
    axis = axis / norm(axis);
    K = [0, -axis(3), axis(2); axis(3), 0, -axis(1); -axis(2), axis(1), 0];
    R = eye(3) + sin(angle) * K + (1 - cos(angle)) * K^2;
end

function dh_calibrated = iterative_calibration(dh_initial, joint_configs, param_indices, target_poses)
    % Iterative calibration algorithm
    dh_calibrated = dh_initial;
    max_iterations = 25;
    convergence_threshold = 1e-8;
    
    fprintf('开始标定...\n');
    
    initial_error = calculate_total_error(dh_calibrated, joint_configs, target_poses);
    fprintf('初始误差: %.6f\n', initial_error);
    
    for iter = 1:max_iterations
        [total_jacobian, total_error_vector] = build_pose_calibration_system(dh_calibrated, joint_configs, param_indices, target_poses);
        
        lambda = 1e-8;
        JTJ = total_jacobian' * total_jacobian + lambda * eye(size(total_jacobian, 2));
        JTe = total_jacobian' * total_error_vector;
        
        delta_params = JTJ \ JTe;
        
        % Adaptive step size
        step_size = 0.1;
        if iter > 8, step_size = 0.05; end
        if iter > 15, step_size = 0.01; end

        
        delta_params = delta_params * step_size;
        
        
        % Test update
        dh_test = dh_calibrated;
        for i = 1:length(param_indices)
            link_idx = param_indices(i, 1);
            param_idx = param_indices(i, 2);
            dh_test(link_idx, param_idx) = dh_test(link_idx, param_idx) + delta_params(i);
        end
        
        test_error = calculate_total_error(dh_test, joint_configs, target_poses);
        
        if test_error < initial_error
            dh_calibrated = dh_test;
            initial_error = test_error;
        end
        
        if norm(delta_params) < convergence_threshold
            fprintf('收敛于第 %d 次迭代\n', iter);
            break;
        end
    end
end

function total_error = calculate_total_error(dh_params, joint_configs, target_poses)
    % Calculate total error
    total_error = 0;
    
    for i = 1:size(joint_configs, 1)
        T_current = forward_kinematics(dh_params, joint_configs(i,:));
        current_pos = T_current(1:3, 4)';
        current_R = T_current(1:3, 1:3);
        
        target_pos = target_poses(i, 1:3);
        target_R = reshape(target_poses(i, 4:12), 3, 3);
        
        pos_error = norm(target_pos - current_pos);
        rot_error = norm(target_R - current_R, 'fro');
        
        total_error = total_error + pos_error^2 + 0.01 * rot_error^2;
    end
    
    total_error = sqrt(total_error / size(joint_configs, 1));
end

function [total_jacobian, total_error_vector] = build_pose_calibration_system(dh_params, joint_configs, param_indices, target_poses)
    % Build calibration system
    total_jacobian = [];
    total_error_vector = [];
    
    for i = 1:size(joint_configs, 1)
        J_pos = compute_position_jacobian(dh_params, joint_configs(i,:), param_indices);
        J_rot = compute_orientation_jacobian(dh_params, joint_configs(i,:), param_indices);
        
        T_current = forward_kinematics(dh_params, joint_configs(i,:));
        current_pos = T_current(1:3, 4)';
        current_R = T_current(1:3, 1:3);
        
        target_pos = target_poses(i, 1:3);
        target_R = reshape(target_poses(i, 4:12), 3, 3);
        
        pos_error = (target_pos - current_pos)';
        R_error = target_R * current_R';
        rot_error = rotation_matrix_to_vector(R_error);
        
        J_combined = [J_pos; 0.1 * J_rot];
        error_combined = [pos_error; 0.1 * rot_error];
        
        total_jacobian = [total_jacobian; J_combined];
        total_error_vector = [total_error_vector; error_combined];
    end
end

function rot_vec = rotation_matrix_to_vector(R)
    % Convert rotation matrix to rotation vector
    angle = acos(max(-1, min(1, (trace(R) - 1) / 2)));
    if abs(angle) < 1e-6
        rot_vec = [0; 0; 0];
    else
        axis = [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)] / (2 * sin(angle));
        rot_vec = angle * axis;
    end
end

function J_pos = compute_position_jacobian(dh_params, joint_angles, param_indices)
    % TODO 8: 计算位置雅可比矩阵 (完成这部分函数)
    % 提示: 使用数值微分方法计算每个DH参数对末端位置的影响
    
    J_pos = zeros(3, size(param_indices, 1));
    delta = 1e-6; % 微分步长
    
    % TODO 8a: 计算基准位置 (完成这部分代码)
    pos_base_T = forward_kinematics(dh_params, joint_angles);
    pos_base = pos_base_T(1:3, 4)';

    if isempty(pos_base_T)
        error('请完成TODO 8a: 计算基准位置');
    end
    pos_base = pos_base_T(1:3, 4)';
    
    for i = 1:size(param_indices, 1)
        link_idx = param_indices(i, 1);
        param_idx = param_indices(i, 2);
        
        % TODO 8b: 创建扰动后的DH参数 (完成这部分代码)
        dh_perturbed = dh_params; % 复制原参数
        dh_perturbed(link_idx, param_idx) = dh_perturbed(link_idx, param_idx) + delta;
        
        % TODO 8c: 计算扰动后的位置 (完成这部分代码)
        T_perturbed = forward_kinematics(dh_perturbed, joint_angles);
        if isempty(T_perturbed)
            error('请完成TODO 8c: 计算扰动后位置');
        end
        pos_perturbed = T_perturbed(1:3, 4)';
        
        % TODO 8d: 计算雅可比矩阵元素 (完成这部分代码)
        J_pos(:, i) = (pos_perturbed - pos_base)' / delta;
    end
end

function J_rot = compute_orientation_jacobian(dh_params, joint_angles, param_indices)
    % Compute orientation Jacobian
    J_rot = zeros(3, size(param_indices, 1));
    delta = 1e-6;
    
    T_base = forward_kinematics(dh_params, joint_angles);
    R_base = T_base(1:3, 1:3);
    
    for i = 1:size(param_indices, 1)
        link_idx = param_indices(i, 1);
        param_idx = param_indices(i, 2);
        
        dh_perturbed = dh_params;
        dh_perturbed(link_idx, param_idx) = dh_perturbed(link_idx, param_idx) + delta;
        
        T_perturbed = forward_kinematics(dh_perturbed, joint_angles);
        R_perturbed = T_perturbed(1:3, 1:3);
        
        R_diff = R_perturbed * R_base';
        rot_vec_diff = rotation_matrix_to_vector(R_diff);
        
        J_rot(:, i) = rot_vec_diff / delta;
    end
end

function [error_before, error_after] = calculate_pose_errors(dh_initial, dh_calibrated, joint_configs, target_poses)
    % TODO 7: 计算标定前后的位姿误差 (学生完成这部分函数)
    % 提示: 计算每个关节配置下，理论位姿与目标位姿的差异
    
    pos_error_before_sum = 0;
    pos_error_after_sum = 0;
    rot_error_before_sum = 0;
    rot_error_after_sum = 0;
    
    for i = 1:size(joint_configs, 1)
        target_pos = target_poses(i, 1:3);
        target_R = reshape(target_poses(i, 4:12), 3, 3);
        
        % TODO 7a: 计算标定前的位姿误差 (完成这部分代码)
        T_before = forward_kinematics(dh_initial, joint_configs(i,:));
        if isempty(T_before)
            error('请完成TODO 7a: 计算标定前位姿');
        end
        
        pos_before = T_before(1:3, 4)';
        R_before = T_before(1:3, 1:3);
        
        % TODO 7b: 计算位置和姿态误差 (完成这部分代码)
        % 提示: 位置误差用norm(), 姿态误差用矩阵的Frobenius范数
        pos_error_before = norm(target_pos - pos_before)^2;
        rot_error_before = norm(target_R - R_before, 'fro')^2;
        
        pos_error_before_sum = pos_error_before_sum + pos_error_before;
        rot_error_before_sum = rot_error_before_sum + rot_error_before;
        
        % TODO 7c: 计算标定后的位姿误差 (完成这部分代码)
        T_after = forward_kinematics(dh_calibrated, joint_configs(i,:));
        if isempty(T_after)
            error('请完成TODO 7c: 计算标定后位姿');
        end
        
        pos_after = T_after(1:3, 4)';
        R_after = T_after(1:3, 1:3);
        
        % TODO 7d: 计算标定后的位置和姿态误差 (完成这部分代码)
        pos_error_after = norm(target_pos - pos_after)^2;
        rot_error_after = norm(target_R - R_after, 'fro')^2;
        
        pos_error_after_sum = pos_error_after_sum + pos_error_after;
        rot_error_after_sum = rot_error_after_sum + rot_error_after;
    end
    
    % TODO 7e: 计算RMS误差 (完成这部分代码)
    num_configs = size(joint_configs, 1);
    pos_error_before = sqrt(pos_error_before_sum / num_configs);
    pos_error_after = sqrt(pos_error_after_sum / num_configs);
    rot_error_before = sqrt(rot_error_before_sum / num_configs);
    rot_error_after = sqrt(rot_error_after_sum / num_configs);
    
    % Combined error (position weighted more than orientation)
    error_before = pos_error_before + 0.1 * rot_error_before;
    error_after = pos_error_after + 0.1 * rot_error_after;
end

function display_results(dh_initial, dh_calibrated, error_before, error_after, improvement)
    % Display calibration results
    
    fprintf('\n=== 原始 DH 参数 ===\n');
    fprintf('Link    a(m)      α(deg)    d(m)      θ_offset(deg)\n');
    for i = 1:6
        fprintf('%d    %8.6f  %8.2f  %8.6f  %8.2f\n', i, ...
            dh_initial(i,1), dh_initial(i,2)*180/pi, dh_initial(i,3), dh_initial(i,4)*180/pi);
    end
    
    fprintf('\n=== 标定后 DH 参数 ===\n');
    fprintf('Link    a(m)      α(deg)    d(m)      θ_offset(deg)\n');
    for i = 1:6
        fprintf('%d    %8.6f  %8.2f  %8.6f  %8.2f\n', i, ...
            dh_calibrated(i,1), dh_calibrated(i,2)*180/pi, dh_calibrated(i,3), dh_calibrated(i,4)*180/pi);
    end
    
    fprintf('\n=== 精度提升 ===\n');
    fprintf('标定前误差: %.6f\n', error_before);
    fprintf('标定后误差: %.6f\n', error_after);
    fprintf('提升程度:  %.1f%%\n\n', improvement);
end

function T = forward_kinematics(dh_params, joint_angles)
    % Forward kinematics using DH convention
    T = eye(4);
    
    for i = 1:6
        a = dh_params(i, 1);
        alpha = dh_params(i, 2);
        d = dh_params(i, 3);
        theta = joint_angles(i) + dh_params(i, 4);
        
        T_i = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
               sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
               0,           sin(alpha),             cos(alpha),            d;
               0,           0,                      0,                     1];
        
        T = T * T_i;
    end
end

%% =========
%{

TODO 列表:
数据准备:
- TODO 1: 设置小组编号 (1-8)
- TODO 2: 设置立方体中心位置 (自选位置)
- TODO 3: 输入四个顶点的测量关节角度
- TODO 4: 计算四个顶点的位姿
- TODO 5a: 生成关节角度变化
- TODO 6: 设置变化范围

标定算法:
- TODO 6: 计算标定改善程度
- TODO 7: 实现误差计算函数 (7a-7e)
- TODO 8: 实现位置雅可比计算 (8a-8d)

%}