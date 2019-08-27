function varargout = interface(varargin)
% INTERFACE MATLAB code for interface.fig
%      INTERFACE, by itself, creates a new INTERFACE or raises the existing
%      singleton*.
%
%      H = INTERFACE returns the handle to a new INTERFACE or the handle to
%      the existing singleton*.
%
%      INTERFACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTERFACE.M with the given input arguments.
%
%      INTERFACE('Property','Value',...) creates a new INTERFACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before interface_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to interface_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help interface

% Last Modified by GUIDE v2.5 17-May-2019 22:48:34

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @interface_OpeningFcn, ...
                   'gui_OutputFcn',  @interface_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before interface is made visible.
function [] = interface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to interface (see VARARGIN)

% Choose default command line output for interface
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);
addpath ../derivation
addpath ../derivation/generated
clc

% --- Outputs from this function are returned to the command line.
function varargout = interface_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% ---------------------------------------------------------------------
%       User Defined Functionalities
% ---------------------------------------------------------------------
addpath ../examples/block_tilting
addpath ../examples/bottle_rotation
addpath ../examples/flip_against_corner


% --- Executes on button press in BTN_EXP_Init_ROS.
function BTN_EXP_Init_ROS_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Init_ROS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global reset_client read_obj_pose_client get_robot_pose_client move_tool_client
global move_hybrid_client move_until_touch_client

% rosinit;

reset_client            = rossvcclient('/robot_bridge/reset');
get_robot_pose_client   = rossvcclient('/robot_bridge/get_pose');
move_tool_client        = rossvcclient('/robot_bridge/move_tool');
move_hybrid_client      = rossvcclient('/robot_bridge/move_hybrid');
move_until_touch_client = rossvcclient('/robot_bridge/move_until_touch');

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'on');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
% set(handles.BTN_EXP_Run, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

disp('Initialization is done. Service clients are ready to use.');

% --- Executes on button press in BTN_EXP_Reset.
function BTN_EXP_Reset_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global reset_client
disp('Calling Reset_service:');
call(reset_client);
disp('Reset is done.');

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'off');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'on');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

% --- Executes on button press in BTN_EXP_read_obj_pose.
function BTN_EXP_read_obj_pose_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_read_obj_pose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global read_obj_pose_client inputs

% disp('Calling read_obj_pose service:');
% call(read_obj_pose_client);

% p_WCenter0 is a point on the table surface;
% p_WO is defined as the middle point of the pivoting edge, not the object center
switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
    case 'RBTN_block_tilting'
        inputs.kNumberOfSteps = 15;
        inputs.kTimeStep = 0.3; % s
        inputs.ObjectEdgeLength = 0.075;
        % inputs.zTableTop = 0.259;
        p_WCenter0 = [41.5 398.15 257]'/1000;
        switch get(get(handles.BTNGROUP_block_tilting_direction,'SelectedObject'),'Tag')
            case 'RBTN_X_plus'
                inputs.p_lineCenter = p_WCenter0 + ...
                        [inputs.ObjectEdgeLength/2, 0, 0]';
                inputs.TiltDirection = [1 0 0]';
                inputs.m_project = diag([1 0 1]);
            case 'RBTN_Y_minus'
                inputs.p_lineCenter = p_WCenter0 + ...
                        [inputs.ObjectEdgeLength, -inputs.ObjectEdgeLength/2, 0]';
                inputs.TiltDirection = [0 -1 0]';
                inputs.m_project = diag([0 1 1]);
            case 'RBTN_X_minus'
                inputs.p_lineCenter = p_WCenter0 + ...
                        [inputs.ObjectEdgeLength/2, -inputs.ObjectEdgeLength, 0]';
                inputs.TiltDirection = [-1 0 0]';
                inputs.m_project = diag([1 0 1]);
            case 'RBTN_Y_plus'
                inputs.p_lineCenter = p_WCenter0 + ...
                        [0, -inputs.ObjectEdgeLength/2, 0]';
                inputs.TiltDirection = [0 1 0]';
                inputs.m_project = diag([0 1 1]);
            otherwise
                error('[Error] unknown selected object.');
        end
        inputs.p_WH0 = inputs.p_lineCenter - ...
                0.15*inputs.ObjectEdgeLength*inputs.TiltDirection + ...
                [0 0 inputs.ObjectEdgeLength]';
        inputs.p_WHPrepare_mm = 1000*(inputs.p_WH0 + [0 0 0.2*inputs.ObjectEdgeLength]');
        % write parameters to file
        fp = fopen(['../results/block_tilting/para.txt'],'w');
        fprintf(fp, '%d\n', inputs.kNumberOfSteps);
        fprintf(fp, '%f\n', inputs.kTimeStep); % length of a timestep
        fprintf(fp, '%f\n', 0.5); % object mass
        fprintf(fp, '%f\n', 0.3); % hand mass
        fprintf(fp, '%f\n', 9.8); % gravity constant
        fprintf(fp, '%f\n', 0.8); % friction coef table object
        fprintf(fp, '%f\n', 0.8); % friction coef hand object
        fprintf(fp, '%d\n', 6); % friction polyhedron sides
        fprintf(fp, '%f\n', 10); % min normal force (newton)
        fprintf(fp, '%f\n', inputs.ObjectEdgeLength);
        fprintf(fp, '%f %f %f\n', inputs.p_WH0(1),...
                inputs.p_WH0(2), inputs.p_WH0(3));
        fprintf(fp, '%f %f %f\n', inputs.p_lineCenter(1),...
                inputs.p_lineCenter(2), inputs.p_lineCenter(3));
        fprintf(fp, '%f %f %f\n', inputs.TiltDirection(1),...
                inputs.TiltDirection(2), inputs.TiltDirection(3));
        fclose(fp);

    case 'RBTN_flip_against_corner'
        inputs.p_WHPrepare_mm   = [30 385 268]; % block
%         inputs.p_WHPrepare_mm   = [30 415 260.5]; % cell phone
    case 'RBTN_plane_engaging'
        inputs.p_WHPrepare_mm = [50 400 350];
    otherwise
        error('[Error] unknown selected object.');
end
disp('Parameter file is written.');

set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'on');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

disp('Read obj pose is done.');


% --- Executes on button press in BTN_EXP_Pre_Grasp.
function BTN_EXP_Pre_Grasp_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Pre_Grasp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global move_tool_client get_robot_pose_client inputs

switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
    case 'RBTN_plane_engaging'
%         pose_set = [50 390 420 0 1 0 0];
        pose_set = [60 352 240 0 0.9848 0 0.1736];
    otherwise
        pose_set = [50 400 350 0 0.1951 0.9808 0];
end

fp = fopen('../results/pose_set.txt','w');
fprintf(fp, '%f ', pose_set);
fclose(fp);

disp('Calling move_tool_service:');
call(move_tool_client);


set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'off');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'on');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
    case 'RBTN_plane_engaging'
        set(handles.BTN_EXP_Engage, 'Enable', 'off');
        set(handles.BTN_EXP_Planning, 'Enable', 'on');
end


disp('Pre Grasp is done.');

% --- Executes on button press in BTN_EXP_Engage.
function BTN_EXP_Engage_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Engage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global move_until_touch_client get_robot_pose_client

switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
    case 'RBTN_block_tilting'
        velocity_set = 5*[0 0 -1]'; % mm/s
    case 'RBTN_flip_against_corner'
        velocity_set = 10*[0 -1 0]'; % mm/s
    case 'RBTN_plane_engaging'
        velocity_set = 5*[0 0 -1]'; % mm/s
    otherwise
        error('[Error] unknown selected object.');
end

fp = fopen('../results/velocity_set.txt','w');
fprintf(fp, '%f ', velocity_set);
fclose(fp);

disp('Calling move_until_touch_service:');
call(move_until_touch_client);

% % feedback
% call(get_robot_pose_client);
% fp_feedback = fopen(['../results/pose_feedback.txt'],'r');
% pose_feedback = fscanf(fp_feedback, '%f');
% fclose(fp_feedback);
% pose_feedback(1:3) = pose_feedback(1:3)/1000;

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'off');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'on');
set(handles.BTN_EXP_Planning, 'Enable', 'on');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

disp('Engaging is done.');

% --- Executes on button press in BTN_EXP_Planning.
function BTN_EXP_Planning_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Planning (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc;
global get_robot_pose_client move_hybrid_client move_tool_client inputs

disp('[Planning] Planning begin.');

% Generate state trajectory
% print parameters to file
switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
    case 'RBTN_block_tilting'
        kTotalRotationAngle = 50*pi/180;
        traj.p_WH = zeros(3, inputs.kNumberOfSteps);
        traj.p_WO = zeros(3, inputs.kNumberOfSteps);
        traj.q_WO = zeros(4, inputs.kNumberOfSteps);

        kRotateAxis = cross([0 0 1]', inputs.TiltDirection);

        for i = 1:inputs.kNumberOfSteps
            angle_i = kTotalRotationAngle/inputs.kNumberOfSteps*i;
            traj.p_WO(:, i) = inputs.p_lineCenter;
            traj.q_WO(:, i) = aa2quat(angle_i, kRotateAxis);
            traj.p_WH(:, i) = ...
                    quatOnVec(inputs.p_WH0 - inputs.p_lineCenter, traj.q_WO(:, i)) ...
                    + inputs.p_lineCenter;
        end
        fp_p_WH = fopen(['../results/block_tilting/p_WH.txt'],'w');
        fp_p_WO = fopen(['../results/block_tilting/p_WO.txt'],'w');
        fp_q_WO = fopen(['../results/block_tilting/q_WO.txt'],'w');
        for i = 1:inputs.kNumberOfSteps
            fprintf(fp_p_WH, '%f\n', traj.p_WH(:,i));
            fprintf(fp_p_WO, '%f\n', traj.p_WO(:,i));
            fprintf(fp_q_WO, '%f\n', traj.q_WO(:,i));
        end
        fclose(fp_p_WH);
        fclose(fp_p_WO);
        fclose(fp_q_WO);

        % p_WO = p_WO0;
        % v_C2C0 = p_WH0 - p_LineContact;
        % q_WO = quatBTVec(m_project*v_C2C0, m_project*(p_WH - p_WO));
        % inputs.p_WH = pose_feedback(1:3);
        % p_WO = inputs.p_lineCenter;
        % v_C2C0 = inputs.p_WH0 - inputs.p_lineCenter;
        % angle_rotated = angBTVec(inputs.m_project*v_C2C0, inputs.m_project*(inputs.p_WH - p_WO));
    case 'RBTN_flip_against_corner'
        kTotalRotationAngle = 50*pi/180;
        % obtain orientation
        call(get_robot_pose_client);
        fp_feedback   = fopen('../results/pose_feedback.txt','r');
        pose_feedback = fscanf(fp_feedback, '%f');
        fclose(fp_feedback);

        kGoalTheta            = 45/180*pi;
        kTimeStep             = 0.1; % s
        kGoalRotationVelocity = 0.1;
        kObjectLength         = 75;
        kObjectThickness      = 35;
        kHandHeight0          = 14.5; %7
        p_WH0 = pose_feedback(2:3)/1000; %[400.4 268]'/1000;
        p_WO0 = p_WH0 - [6.5+kObjectLength/2;
                kHandHeight0 - kObjectThickness/2]/1000; %[44; -9.5]

        fp = fopen(['../results/flip_against_corner/para.txt'],'w');
        fprintf(fp, '%f\n', kGoalTheta);
        fprintf(fp, '%f\n', kTimeStep); % length of a timestep
        fprintf(fp, '%f\n', 0.15); % object mass
        fprintf(fp, '%f\n', 0.0); % hand mass
        fprintf(fp, '%f\n', 9.8); % gravity constant
        fprintf(fp, '%f\n', kObjectLength/1000); % 0.075 object length
        fprintf(fp, '%f\n', kObjectThickness/1000); % 0.035 object thickness
        fprintf(fp, '%f\n', 0.5); % friction coef table object
        fprintf(fp, '%f\n', 0.7); % friction coef hand object
        fprintf(fp, '%f\n', 0.5); % friction coef bin object
        fprintf(fp, '%f\n', 4); % min normal force
        fprintf(fp, '%f\n', -5); % min normal force sliding
        fprintf(fp, '%f\n', 100); % max normal force sliding
        fprintf(fp, '%f\n', kGoalRotationVelocity); % Goal rotation velocity
        fprintf(fp, '%f %f\n', p_WH0(1), p_WH0(2));
        fprintf(fp, '%f %f\n', p_WO0(1), p_WO0(2));
        fclose(fp);
     case 'RBTN_plane_engaging'
        % G, b_G
        % [0 0 1 0 0]*Adj_WT*v_T = -0.05
        call(get_robot_pose_client);
        fp_feedback   = fopen('../results/pose_feedback.txt','r');
        pose_feedback_mm = fscanf(fp_feedback, '%f');
        fclose(fp_feedback);
        
        pose_fb = pose_feedback_mm;
        pose_fb(1:3) = pose_fb(1:3)/1000;
        
        R_WT = quat2SO3(pose_fb(4:7));
        vz_T = R_WT'*[0 0 -1]'; % world -Z viewed in tool frame 
        G = [vz_T' 0 0 0];
        b_G = 0.004; %m/s

        % G
        fp = fopen(['../results/plane_engaging/para.txt'],'w');
        fprintf(fp, '%d %d\n', 1, 6);
        fprintf(fp, '%f ', G'); % transpose so that it is row first
        
        % b_G
        fprintf(fp, '\n');
        fprintf(fp, '%f ', b_G');
        fclose(fp);
end

disp('Calling move_hybrid_service:');
call(move_hybrid_client);

% disengage
switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
    case 'RBTN_block_tilting'
        call(get_robot_pose_client);
        fp_feedback   = fopen('../results/pose_feedback.txt','r');
        pose_feedback = fscanf(fp_feedback, '%f');
        fclose(fp_feedback);

        pose_set = pose_feedback;
        pose_set(1:3) = pose_set(1:3) + inputs.TiltDirection*40; % mm
        % pose_set(1:3) = pose_set(1:3)*1000;
        fp = fopen(['../results/pose_set.txt'],'w');
        fprintf(fp, '%f ', pose_set);
        fclose(fp);
        disp('Calling move_tool_service:');
        call(move_tool_client);

        set(handles.BTN_EXP_Planning, 'Enable', 'off');
        set(handles.BTN_EXP_Release_Reset, 'Enable', 'on');
    case 'RBTN_flip_against_corner'
        % call(get_robot_pose_client);
        % fp_feedback   = fopen('../results/pose_feedback.txt','r');
        % pose_feedback = fscanf(fp_feedback, '%f');
        % fclose(fp_feedback);

        % pose_set = pose_feedback;
        % pose_set(3) = pose_set(3) + 30; % mm
        % fp = fopen(['../results/pose_set.txt'],'w');
        % fprintf(fp, '%f ', pose_set);
        % fclose(fp);
        % disp('Calling move_tool_service:');
        % call(move_tool_client);
        set(handles.BTN_EXP_Planning, 'Enable', 'off');
        set(handles.BTN_EXP_Release_Reset, 'Enable', 'on');
    case 'RBTN_plane_engaging'
        % visualization
        f_queue = dlmread('../results/plane_engaging/f_queue.txt');
        v_queue = dlmread('../results/plane_engaging/v_queue.txt');
        f_weights = dlmread('../results/plane_engaging/f_weights.txt');
        v_weights = dlmread('../results/plane_engaging/v_weights.txt');
        % the following two files could be empty
        D = dir('../results/plane_engaging/f_data_filtered.txt');
        if D.bytes > 1
            f_data_filtered = dlmread('../results/plane_engaging/f_data_filtered.txt');
        else
            f_data_filtered = [];
        end
        D = dir('../results/plane_engaging/f_data_selected.txt');
        if D.bytes > 1
            f_data_selected = dlmread('../results/plane_engaging/f_data_selected.txt');
        else
            f_data_selected = [];
        end
        others = dlmread('../results/plane_engaging/process.txt');
        DimV = others(1, 1);
        DimF = others(2, 1);
        v_T = others(3, :);
        force_T = others(4, :);

        disp(['DimV: ' num2str(DimV) ', DimF: ' num2str(DimF)]);
        if get(handles.CB_PE_show_figures,'value')
            figure(1);clf(1); hold on;
            axis equal
            % v_queue
            n = size(v_queue, 1);
            quiver3(zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
                v_queue(:, 1), v_queue(:, 2), v_queue(:, 3), 'g');
            % f_queue
            n = size(f_queue, 1);
            quiver3(zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
                f_queue(:, 1), f_queue(:, 2), f_queue(:, 3), 'b');
            % f_data_filtered
            n = size(f_data_filtered, 1);
            if n > 0
                quiver3(zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
                    f_data_filtered(:, 1), f_data_filtered(:, 2), f_data_filtered(:, 3), 'y');
            end
            % f_data_selected
            n = size(f_data_selected, 1);
            if n > 0
                quiver3(zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
                    f_data_selected(:, 1), f_data_selected(:, 2), f_data_selected(:, 3), 'r', 'linewidth', 10);
            end
            figure(2);clf(2); hold on;
            axis equal
            % v_T
            n = size(v_T, 1);
            v_T = v_T*1000; % m -> mm
            quiver3(zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
                v_T(:, 1), v_T(:, 2), v_T(:, 3), 'b', 'linewidth', 3);
            % force_T
            n = size(force_T, 1);
            quiver3(zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
                force_T(:, 1), force_T(:, 2), force_T(:, 3), 'r', 'linewidth', 3);
            % f_data_selected
            n = size(f_data_selected, 1);
            if n > 0
                quiver3(zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
                    f_data_selected(:, 1), f_data_selected(:, 2), f_data_selected(:, 3), 'g');
                legend('v T','f T', 'N');
            end
            xlabel('X'); ylabel('Y'); zlabel('Z');
            figure(3);clf(3);
            subplot(2,1,1); hold on;
            plot(f_weights, '.r-', 'markersize', 4);
            plot(v_weights, '.b-', 'markersize', 4);
            legend('f weights', 'v weights');
            subplot(2,1,2); hold on;
            plot(normByRow(f_queue), '-r', 'linewidth', 3);
            plot(normByRow(v_queue), '-b', 'linewidth', 3);
            legend('f magnitude', 'v magnitude');
        end
        set(handles.BTN_EXP_Planning, 'Enable', 'on');
        set(handles.BTN_EXP_Release_Reset, 'Enable', 'on');
end



disp('Planning is done.')



% --- Executes on button press in BTN_EXP_Release_Reset.
function BTN_EXP_Release_Reset_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Release_Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global get_robot_pose_client move_tool_client inputs reset_client

% read feedback
call(get_robot_pose_client);
fp_feedback = fopen('../results/pose_feedback.txt','r');
pose_feedback = fscanf(fp_feedback, '%f');
fclose(fp_feedback);

% move up to release
pose_set = pose_feedback;
pose_set(3) = pose_set(3) + 30;
fp = fopen('../results/pose_set.txt','w');
fprintf(fp, '%f ', pose_set);
fclose(fp);

disp('Calling move_tool_service:');
call(move_tool_client);

% call reset
call(reset_client);

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'on');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');
disp('Release_reset is done.');

% --- Executes on button press in RBTN_X_plus.
function RBTN_X_plus_Callback(hObject, eventdata, handles)
% hObject    handle to RBTN_X_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RBTN_X_plus


% --- Executes on button press in CB_PE_show_figures.
function CB_PE_show_figures_Callback(hObject, eventdata, handles)
% hObject    handle to CB_PE_show_figures (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of CB_PE_show_figures
