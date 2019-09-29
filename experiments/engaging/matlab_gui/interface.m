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

% Last Modified by GUIDE v2.5 13-Sep-2019 11:41:38

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
global execute_task_client engage_client
global config
% rosinit;

config = yaml.ReadYaml('../config/abb/task.yaml');

reset_client            = rossvcclient('/robot_bridge/reset');
get_robot_pose_client   = rossvcclient('/robot_bridge/get_pose');
move_tool_client        = rossvcclient('/robot_bridge/move_tool');
engage_client           = rossvcclient('/robot_bridge/compliant_engage');
execute_task_client     = rossvcclient('/robot_bridge/execute_task');

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
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
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
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
set(handles.BTN_EXP_Planning, 'Enable', 'on');
set(handles.BTN_EXP_Engage, 'Enable', 'on');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

disp('Pre Grasp is done.');

% --- Executes on button press in BTN_EXP_Planning.
function BTN_EXP_Planning_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Planning (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc;
global execute_task_client

disp('[Execution] Execution begin.');

fp = fopen('../data/n.txt','w');
n = str2double(get(handles.ET_n, 'String')) ;
disp('n: ');
disp(n);
fprintf(fp, '%d ', n);
fclose(fp);

disp('Calling move_hybrid_service:');
call(execute_task_client);
disp('move_hybrid_service is done:');

%% Drawing

% visualization
f_queue = dlmread('../data/f_queue.txt');
v_queue = dlmread('../data/v_queue.txt');
f_weights = dlmread('../data/f_weights.txt');
v_weights = dlmread('../data/v_weights.txt');
% the following two files could be empty
D = dir('../data/f_data_filtered.txt');
if D.bytes > 1
    f_data_filtered = dlmread('../data/f_data_filtered.txt');
else
    f_data_filtered = [];
end
D = dir('../data/f_data_selected.txt');
if D.bytes > 1
    f_data_selected = dlmread('../data/f_data_selected.txt');
else
    f_data_selected = [];
end
others = dlmread('../data/process.txt');
DimV = others(1, 1);
DimF = others(2, 1);
v_T = others(3, :);
force_T = others(4, :);

disp(['DimV: ' num2str(DimV) ', DimF: ' num2str(DimF)]);
if get(handles.CB_PE_show_figures,'value')
    figure(1);clf(1); hold on;
    axis equal
    % v_queue
%     n = size(v_queue, 1);
%     quiver3(zeros(n, 1), zeros(n, 1), zeros(n, 1), ...
%         v_queue(:, 1), v_queue(:, 2), v_queue(:, 3), 'g');
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
pose_set(1) = pose_set(1) + 30;
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
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');
disp('Release_reset is done.');


% --- Executes on button press in CB_PE_show_figures.
function CB_PE_show_figures_Callback(hObject, eventdata, handles)
% hObject    handle to CB_PE_show_figures (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of CB_PE_show_figures


% --- Executes on button press in btn_exp_engage.
function BTN_EXP_Engage_Callback(hObject, eventdata, handles)
% hObject    handle to btn_exp_engage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc;
global engage_client

disp('[Compliant Engage] Compliant Engage begin.');
disp('Calling engage_service:');
call(engage_client);
disp('engage_service is done:');

set(handles.BTN_EXP_Release_Reset, 'Enable', 'on');



function ET_n_Callback(hObject, eventdata, handles)
% hObject    handle to ET_n (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ET_n as text
%        str2double(get(hObject,'String')) returns contents of ET_n as a double


% --- Executes during object creation, after setting all properties.
function ET_n_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ET_n (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
