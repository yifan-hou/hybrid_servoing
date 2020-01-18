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

% Last Modified by GUIDE v2.5 16-Jan-2020 14:26:29

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
clc

% --- Outputs from this function are returned to the command line.
function varargout = interface_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in BTN_EXP_Init_ROS.
function BTN_EXP_Init_ROS_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Init_ROS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global reset_client get_robot_pose_client move_tool_client
global execute_task_client move_until_touch_client
global config
% rosinit;

config = yaml.ReadYaml('../config/task.yaml');

reset_client            = rossvcclient('/robot_bridge/reset');
get_robot_pose_client   = rossvcclient('/robot_bridge/get_pose');
move_tool_client        = rossvcclient('/robot_bridge/move_tool');
move_until_touch_client = rossvcclient('/robot_bridge/move_until_touch');
execute_task_client     = rossvcclient('/robot_bridge/hybrid_servo');

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
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
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'on');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

% --- Executes on button press in BTN_EXP_Pre_Grasp.
function BTN_EXP_Pre_Grasp_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Pre_Grasp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global move_tool_client
global config data

disp('Resetting parameters.');
% Parameters
data.kFrictionH = 1.2;
data.kFrictionE = 0.25;

% list of contact points and contact normals
% in 3D W frame
obj_height = 0.0435;
obj_width = 0.0435;
data.X.obj_height = obj_height;
data.X.obj_width = obj_width;
data.X.CP_W_e = [[obj_width/2; 0; 0], [-obj_width/2; 0; 0]];
data.X.CN_W_e = [[0; 0; 1], [0; 0; 1]];
data.X.CP_H_h = [[obj_width/2; 0; 0], [-obj_width/2; 0; 0]];
data.X.CN_H_h = [[0; 0; -1], [0; 0; -1]];
data.X.CP_O_e = data.X.CP_W_e;

obj_height = 0.0435;
obj_width = 0.0435;
data.Y.obj_height = obj_height;
data.Y.obj_width = obj_width;
data.Y.CP_W_e = [[0; obj_width/2; 0], [0; -obj_width/2; 0]];
data.Y.CN_W_e = [[0; 0; 1], [0; 0; 1]];
data.Y.CP_H_h = [[0; -obj_width/2; 0], [0; obj_width/2; 0]];
data.Y.CN_H_h = [[0; 0; -1], [0; 0; -1]];
data.Y.CP_O_e = data.Y.CP_W_e;

% initial mode
data.mode = 'ffff';

pose_set = cell2mat(config.task.pre_grasp_pose);
fp = fopen('../data/pose_set.txt','w');
fprintf(fp, '%f ', pose_set);
fclose(fp);

disp('Calling move_tool_service:');
call(move_tool_client);

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'on');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

disp('Pre Grasp is done.');

% --- Executes on button press in BTN_EXP_Engage.
function BTN_EXP_Engage_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Engage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global move_until_touch_client

velocity_set = 10*[0 0 -1]'; % mm/s
fp = fopen('../data/velocity_set.txt','w');
fprintf(fp, '%f ', velocity_set);
fclose(fp);
disp('Calling move_until_touch_service:');
call(move_until_touch_client);

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
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
global execute_task_client get_robot_pose_client
global config data

tic;
% 1. Read feedback from robot
call(get_robot_pose_client);
fp_feedback = fopen('../data/pose_feedback.txt','r');
pose_feedback = fscanf(fp_feedback, '%f');
pose_feedback(1:3) = pose_feedback(1:3)/1000;
fclose(fp_feedback);

% 2. compute Hand pose in world frame
% T(Tool) is the same as H(Hand)
SE3_BH = pose2SE3(pose_feedback);
pose_BW = cell2mat(config.task.pose_BW);
pose_BW(1:3) = pose_BW(1:3)/1000;
SE3_BW = pose2SE3(pose_BW);
SE3_WT = SE3inv(SE3_BW)*SE3_BH;
R_WH = SE3_WT(1:3, 1:3);
p_WH = SE3_WT(1:3, 4);

% Compute object pose from hand pose based on current mode
switch data.mode
    case {'ffff', 'sfff', 'fsff', 'rrff', 'llff'}
        switch get(get(handles.RBTGroup_Direction, 'SelectedObject'),'Tag')
            case {'RBT_X_Minus', 'RBT_X_Plus'}
                SE3_TO = [eye(3) [0 0 -data.X.obj_height]';0 0 0 1];
                SE3_WO = SE3_WT*SE3_TO;
            otherwise
                disp('Wrong selection!');
                assert(false);
        end
    case 'sffs'
        switch get(get(handles.RBTGroup_Direction, 'SelectedObject'),'Tag')
            case {'RBT_X_Minus', 'RBT_X_Plus'}
                p_Th1 = data.X.CP_H_h(:, 1);
                p_Wh1 = SE3_WT*p_Th1;
                p_We2 = data.X.CP_W_e(:,2);
                ang_diag0 = atan2(data.X.obj_height, data.X.obj_width);
                ang_diag = angBTVec([1, 0, 0]', p_Wh1 - p_We2);
                rot_angle = ang_diag - ang_diag0;
                assert(rot_angle > -1e-7);
                p_We1 = p_We2 + [data.X.obj_width*cos(rot_angle); 0; data.X.obj_width*sin(rot_angle)];
                p_WO = (p_We1 + p_We2)/2;
                vx = p_We1 - p_We2; vx = vx/norm(vx);
                vy = [0 1 0]';
                vz = cross(vx, vy);
                SE3_WO = [vx, vy, vz, p_WO; 0 0 0 1];
            % case {'RBT_Y_Minus', 'RBT_Y_Plus'}
                % p_Th1 = data.Y.CP_H_h(:, 1);
                % p_Wh1 = SE3_WT*p_Th1;
                % p_We2 = data.Y.CP_W_e(:,2);
                % ang_diag0 = atan2(data.Y.obj_height, data.Y.obj_width);
                % ang_diag = angBTVec([-1, 0]', p_Wh1(2:3) - p_We2(2:3));
                % rot_angle = ang_diag - ang_diag0;
                % assert(rot_angle > -1e-7);
                % p_We1 = p_We2 + [0; -data.Y.obj_width*cos(rot_angle); data.Y.obj_width*sin(rot_angle)];
                % p_WO = (p_We1 + p_We2)/2;
                % vy = p_We2 - p_We1; vy = vy/norm(vy);
                % vx = [0 1 0]';
                % vz = cross(vx, vy);
                % SE3_WO = [vx, vy, vz, p_WO; 0 0 0 1];
            otherwise
                disp('Wrong selection!');
                assert(false);
        end
    otherwise
        disp('unknown mode!!!');
        disp(data.mode);
        assert(false);
end

% compute goal
switch get(get(handles.RBTGroup_Goal,'SelectedObject'),'Tag')
    case 'RBT_Object_Pivot'
        G = [0 0 1 0 0 0];
        switch get(get(handles.RBTGroup_Direction, 'SelectedObject'),'Tag')
            case 'RBT_X_Minus'
                b_G = -0.1;
            case 'RBT_X_Plus'
                b_G = 0.1;
            otherwise
                disp('Wrong selection!');
                assert(false);
        end
    case 'RBT_Object_Slide'
        G = [1 0 0 0 0 0;
             0 0 1 0 0 0];
        switch get(get(handles.RBTGroup_Direction, 'SelectedObject'),'Tag')
            case 'RBT_X_Minus'
                b_G = [0.1; 0];
            case 'RBT_X_Plus'
                b_G = [-0.1; 0];
            otherwise
                disp('Wrong selection!');
                assert(false);
        end
    case 'RBT_Hand_Pivot'
        G = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 1];
        b_G = [0; 0; -0.1];
    otherwise
        disp('Wrong selection!');
        assert(false);
end

% update contact locations
switch data.mode
    case {'ffff', 'sfff', 'fsff', 'sffs'}
        % no change
    case  {'rrff', 'llff'}
        data.X.CP_W_e = SE3OnVec(SE3_WO, data.X.CP_O_e);
    otherwise
        disp('Wrong selection!');
        assert(false);
end
% special care for hand pivoting
if get(handles.RBT_Hand_Pivot,'Value') == true
    if size(data.X.CP_W_e, 2) == 2
        data.X.CP_W_e = data.X.CP_W_e(:, 2);
        data.X.CN_W_e = data.X.CN_W_e(:, 2);
        data.X.CP_H_h = data.X.CP_H_h(:, 1);
        data.X.CN_H_h = data.X.CN_H_h(:, 1);
    end
end

% 4. call wrench space analysis, compute HFVC plan

% switch to 2D
CP_W_e_2 = data.X.CP_W_e([1 3], :);
CN_W_e_2 = data.X.CN_W_e([1 3], :);
CP_H_h_2 = data.X.CP_H_h([1 3], :);
CN_H_h_2 = data.X.CN_H_h([1 3], :);

R_WH_2 = R_WH([1 3], [1 3]);
p_WH_2 = p_WH([1 3]);

nominal_force = config.task.nominal_force;
solution = wrenchSpaceAnalysis_modeSelection(data.kFrictionE, data.kFrictionH, ...
        CP_W_e_2, CN_W_e_2, CP_H_h_2, CN_H_h_2, R_WH_2, p_WH_2, G, b_G, nominal_force);

% 5. send to robot
if isempty(solution)
    disp('No solution found!!');
    return;
end

% go back to 3D
R32 = [1 0 0 0 0 0;
       0 0 1 0 0 0;
       0 0 0 0 -1 0];
n_af   = solution.n_af;
n_av   = solution.n_av + 3;
w_av   = [solution.w_av; 0; 0; 0];
eta_af = solution.eta_af;
R_a    = [solution.R_a*R32;
          0 1 0 0 0 0;
          0 0 0 1 0 0;
          0 0 0 0 0 1];
R_a_inv = R_a^-1;

w_set = [zeros(n_af, 1); w_av];
v_T = R_a_inv*w_set;

% cap at maximum speed
kVMax = config.task.max_veloicty_meter;
scale_rot_to_tran = 0.5;
v_T_scaled = v_T;
v_T_scaled(4:6) = v_T_scaled(4:6)*scale_rot_to_tran;
if norm(v_T_scaled) > kVMax
  scale_safe = kVMax/norm(v_T_scaled);
  v_T = v_T*scale_safe;
end

% compute velocity and displacement in base frame
Adj_WT = SE32Adj(SE3_WT);
v_W = Adj_WT*v_T;
step_time_s = config.task.time_step;
SE3_WT_command = SE3_WT + wedge6(v_W)*SE3_WT*step_time_s;
SE3_BT_command = SE3_BW*SE3_WT_command;
pose_set = SE32pose(SE3_BT_command);
pose_set(1:3) = pose_set(1:3)*1000; % m to mm

% compute force action in base frame
force_set = zeros(6,1);
force_set(1:n_af) = eta_af;
force_T = R_a_inv*force_set;
Adj_TW = SE32Adj(SE3inv(SE3_WT));
force_W = Adj_TW'*force_T;

fprintf("V in world: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", v_W(1), ...
    v_W(2),v_W(3),v_W(4),v_W(5), v_W(6));
fprintf("F in world: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", force_W(1), ...
    force_W(2),force_W(3),force_W(4),force_W(5), force_W(6));
toc;

% write down the control
fp = fopen(['../data/hybrid_action.txt'],'w');
fprintf(fp, '%d %d %f \n', n_af, n_av, step_time_s);
fprintf(fp, '%f \n', pose_set);
fprintf(fp, '%f \n', force_set);
fprintf(fp, '%f ', R_a');
fclose(fp);

disp('Calling move_hybrid_service:');
call(execute_task_client);

data.mode = printModes(solution.eh_mode, false);
if get(handles.RBT_Hand_Pivot,'Value') == true
    assert(data.mode == 'ff');
    data.mode = 'sffs';
end

set(handles.BTN_EXP_Release_Reset, 'Enable', 'on');

disp('Planning is done.')


% --- Executes on button press in BTN_EXP_Release_Reset.
function BTN_EXP_Release_Reset_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Release_Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global get_robot_pose_client move_tool_client inputs reset_client

% read feedback
call(get_robot_pose_client);
fp_feedback = fopen('../data/pose_feedback.txt','r');
pose_feedback = fscanf(fp_feedback, '%f');
fclose(fp_feedback);

% move up to release
pose_set = pose_feedback;
pose_set(3) = pose_set(3) + 30;
fp = fopen('../data/pose_set.txt','w');
fprintf(fp, '%f ', pose_set);
fclose(fp);

disp('Calling move_tool_service:');
call(move_tool_client);

% call reset
call(reset_client);

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');
disp('Release_reset is done.');


% --- Executes on button press in RBT_X_Minus.
function RBT_X_Minus_Callback(hObject, eventdata, handles)
% hObject    handle to RBT_X_Minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RBT_X_Minus


% --- Executes on button press in RBT_Y_Plus.
function RBT_Y_Plus_Callback(hObject, eventdata, handles)
% hObject    handle to RBT_Y_Plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RBT_Y_Plus


% --- Executes on button press in RBT_X_Plus.
function RBT_X_Plus_Callback(hObject, eventdata, handles)
% hObject    handle to RBT_X_Plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RBT_X_Plus


% --- Executes on button press in RBT_Y_Minus.
function RBT_Y_Minus_Callback(hObject, eventdata, handles)
% hObject    handle to RBT_Y_Minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RBT_Y_Minus
