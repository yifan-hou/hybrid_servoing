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

% Last Modified by GUIDE v2.5 22-Oct-2020 21:49:19

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
global reset_client read_obj_pose_client get_robot_pose_client move_tool_client
global execute_task_client move_until_touch_client
global config
% rosinit;

config = yaml.ReadYaml('../config/task.yaml');

% ROS initializations

reset_client            = rossvcclient('/robot_bridge/reset');
get_robot_pose_client   = rossvcclient('/robot_bridge/get_pose');
move_tool_client        = rossvcclient('/robot_bridge/move_tool');
move_until_touch_client = rossvcclient('/robot_bridge/move_until_touch');
execute_task_client      = rossvcclient('/robot_bridge/execute_task');


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
global move_tool_client get_robot_pose_client inputs
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
global move_until_touch_client get_robot_pose_client

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
global get_robot_pose_client execute_task_client move_tool_client inputs

disp('[Planning] Planning begin.');
disp('Calling move_hybrid_service:');
call(execute_task_client);

% disengage
set(handles.BTN_EXP_Planning, 'Enable', 'off');
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


% --- Executes on button press in BTN_Generate.
function BTN_Generate_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_Generate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% global config
% num_of_frames = config.num_of_frames;
% ang_max = config.rotation_angle_deg * pi/180;
% H = config.obj_height_mm;
% l = config.hand_offset_mm; % distance from the edge to the hand contact
% kFrictionE = config.environment_friction;
% kFrictionH = config.hand_friction;
% kObjWeight = config.object_weight;
% kMinContactNormalForce = config.contact_normal_force_min;

num_of_frames = 50; % config.num_of_frames;
ang_max = 50; % config.rotation_angle_deg * pi/180;
H = 76; % config.obj_height_mm;
l = 10; % config.hand_offset_mm; % distance from the edge to the hand contact
kFrictionE = 0.5; % config.environment_friction;
kFrictionH = 0.9; % config.hand_friction;
kObjWeight = 5; % config.object_weight;
kMinContactNormalForce = 8; % config.contact_normal_force_min;

motion_plan = zeros(num_of_frames+2, 17);
motion_plan(1, 1:5) = [-1 0 1 -1 0];

emodes = 1;
hmodes = 1;
for fr = 0:num_of_frames
    theta = ang_max*fr/num_of_frames;
    st = sin(theta);
    ct = cos(theta);
    x = -H*st + l*ct;
    y =  H*ct + l*st;

    % solve hfvc
    p_We = [0; 0];
    n_We = [0; 1];
    p_Hh = [0; 0];
    n_Hh = [st; -ct];

    R_WH = eye(2);
    p_WH = [x; y];

    R_HW = R_WH';
    p_HW = -R_HW*p_WH;
    adj_HW = SE22Adj(R_HW, p_HW);
    adj_WH = SE22Adj(R_WH, p_WH);

    [N_e, T_e, N_h, T_h, ~, ~, ~, ~] = getWholeJacobian(p_We, n_We, ...
        p_Hh, n_Hh, adj_WH, adj_HW, 1, kFrictionE, kFrictionH);

    [N, ~] = getJacobianFromContacts(emodes, hmodes, N_e, N_h, T_e, T_h);

    J_e = [N_e; T_e];
    J_h = [N_h; T_h];
    J_All = [J_e, zeros(2,3) ; -J_h, J_h];

    % goal
    G = [0 0 1 0 0 0;
         0 0 0 0 0 1];
    b_G = [1; 0];

    % guard condition
    A = [-1           0           0  0;
         0            0          -1  0;
         -kFrictionE  1           0  0;
         -kFrictionE -1           0  0;
         0            0 -kFrictionH  1;
         0            0 -kFrictionH -1];
    b = [-kMinContactNormalForce;
         -kMinContactNormalForce;
         0;0;0;0];
    Fg = [0 kObjWeight 0 0 0 0]';

    dims.Actualized = 3;
    dims.UnActualized = 3;
    [action, time] = ochs(dims, N, G, b_G, Fg, A, b, J_All);
    disp(['Frame ' num2str(fr) ' solved in ' num2str(time.velocity*1000+time.force*1000) ' ms.']);
    % [margin, n_af, n_av, R_a (9), eta_af, w_av]
    motion_plan(fr+2, :) = [1, action.n_af, action.n_av, ...
                          action.R_a(1,1), action.R_a(1,2), action.R_a(1,3), ...
                          action.R_a(2,1), action.R_a(2,2), action.R_a(2,3), ...
                          action.R_a(3,1), action.R_a(3,2), action.R_a(3,3), ...
                          action.eta_af', action.w_av', p_WH'];
end

% write to file
writematrix(motion_plan, '../data/traj_block_tilting.csv');
