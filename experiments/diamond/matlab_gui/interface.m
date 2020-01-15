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

% Last Modified by GUIDE v2.5 12-Jan-2020 17:51:00

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
global config data
% rosinit;

config = yaml.ReadYaml('../config/task.yaml');

% Parameters
data.kFrictionH = 1.5;
data.kFrictionE = 1.1;

% list of contact points and contact normals
% in 3D W frame
p_W_e1 = [0.0435/2; 0; 0];
p_W_e2 = [-0.0435/2; 0; 0];
n_W_e1 = [0; 0; 1];
n_W_e2 = [0; 0; 1];

p_H_h1 = [0.0435/2; 0; 0];
p_H_h2 = [-0.0435/2; 0; 0];
n_H_h1 = [0; 0; -1];
n_H_h2 = [0; 0; -1];

data.obj_height = 0.0435;
data.obj_width = p_W_e1(1) - p_W_e2(1);
data.CP_W_e = [p_W_e1, p_W_e2];
data.CN_W_e = [n_W_e1, n_W_e2];
data.CP_H_h = [p_H_h1, p_H_h2];
data.CN_H_h = [n_H_h1, n_H_h2];
data.mode = 'ffff';

reset_client            = rossvcclient('/robot_bridge/reset');
get_robot_pose_client   = rossvcclient('/robot_bridge/get_pose');
move_tool_client        = rossvcclient('/robot_bridge/move_tool');
move_until_touch_client = rossvcclient('/robot_bridge/move_until_touch');
execute_task_client     = rossvcclient('/robot_bridge/execute_task');

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
global config

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

velocity_set = 10*[0 -1 0]'; % mm/s
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

% 2. compute poses
% T(Tool) is the same as H(Hand)
SE3_BH = pose2SE3(pose_feedback);
pose_BW = cell2mat(config.task.pose_BW);
pose_BW(1:3) = pose_BW(1:3)/1000;
SE3_BW = pose2SE3(pose_BW);
SE3_WT = SE3inv(SE3_BW)*SE3_BH;
R_WH = SE3_WT(1:3, 1:3);
p_WH = SE3_WT(1:3, 4);

% 3. update goal
switch data.mode
    case {'ffff', 'sfff'}
        SE3_TO = [eye(3) [0 0 -data.obj_height];0 0 0 1];
        SE3_WO = SE3_WT*SE3_TO;
    case 'sffs'
        p_Th1 = data.CP_H_h(:, 1);
        p_Wh1 = SE3_WT*p_Th1;
        p_We2 = data.CP_W_e(:,2);
        ang_diag = angBTVec([1, 0, 0]', p_Wh1 - p_We2);
        ang_diag0 = atan2(data.obj_height, data.obj_width);
        rot_angle = ang_diag - ang_diag0;
        assert(rot_angle > -1e-7);
        p_We1 = p_We2 + [data.obj_width*cos(rot_angle); 0; data.obj_width*sin(rot_angle)];
        p_WO = (p_We1 + p_We2)/2;
        vx = p_We1 - p_We2; vx = vx/norm(vx);
        vy = [0 1 0]';
        vz = cross(vx, vy);
        SE3_WO = [vx, vy, vz, p_WO; 0 0 0 1];
    otherwise
        disp('unknown mode!!!');
        disp(data.mode);
        assert(false);
end
G = [0 0 1 0 0 0];
b_G = [0.1];

% 4. call wrench space analysis, compute HFVC plan

% switch to 2D
CP_W_e_2 = data.CP_W_e([1 3], :);
CN_W_e_2 = data.CN_W_e([1 3], :);
CP_H_h_2 = data.CP_H_h([1 3], :);
CN_H_h_2 = data.CN_H_h([1 3], :);

R_WH_2 = R_WH([1 3], [1 3]);
p_WH_2 = p_WH([1 3]);

solution = wrenchSpaceAnalysis_modeSelection(kFrictionE, kFrictionH, ...
        CP_W_e_2, CN_W_e_2, CP_H_h_2, CN_H_h_2, R_WH_2, p_WH_2, G, b_G)


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
kVMax = 0.002; % m/s,  maximum speed limit
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
pose_set = SE32Pose(SE3_BT_command);
pose_set(1:3) = pose_set(1:3)*1000; % m to mm

% compute force action in base frame
force_set = zeros(6,1);
force_set(1:n_af) = eta_f;
force_T = R_a_inv*force_set;
Adj_TW = SE32Adj(SE3Inv(SE3_WT));
force_W = Adj_TW'*force_T;

fprintf("V in world: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", v_W(1),
    v_W(2),v_W(3),v_W(4),v_W(5), v_W(6));
fprintf("F in world: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", force_W(1),
    force_W(2),force_W(3),force_W(4),force_W(5), force_W(6));
toc;

% write down the control
fp = fopen(['../data/hybrid_action.txt'],'w');
fprintf(fp, '%d %d %f', n_af, n_av, step_time_s);
fprintf(fp, '%f ', pose_set);
fprintf(fp, '%f ', force_set);
fprintf(fp, '%f %f %f %f %f %f %f %f %f ', R_a(1,1), R_a(1,2), R_a(1,3), ...
                                           R_a(2,1), R_a(2,2), R_a(2,3), ...
                                           R_a(3,1), R_a(3,2), R_a(3,3));
fclose(fp);


disp('Calling move_hybrid_service:');
call(execute_task_client);

data.mode = solution.eh_mode;

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
pose_set(2) = pose_set(2) + 30;
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


% --- Executes on button press in CB_PE_show_figures.
function CB_PE_show_figures_Callback(hObject, eventdata, handles)
% hObject    handle to CB_PE_show_figures (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of CB_PE_show_figures


% --- Executes on button press in CB_second_run.
function CB_second_run_Callback(hObject, eventdata, handles)
% hObject    handle to CB_second_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of CB_second_run
