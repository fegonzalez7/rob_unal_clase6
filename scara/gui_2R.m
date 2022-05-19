%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
GUI para modelar robot de 4GDL tipo SCARA
Movimiento en el espacio articular y espacio de la tarea
Por: Felipe Gonzalez Roldan
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}
function varargout = gui_2r(varargin)
% GUI_2R MATLAB code for gui_2R.fig
%      GUI_2R, by itself, creates a new GUI_2R or raises the existing
%      singleton*.
%
%      H = GUI_2R returns the handle to a new GUI_2R or the handle to
%      the existing singleton*.
%
%      GUI_2R('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_2R.M with the given input arguments.
%
%      GUI_2R('Property','Value',...) creates a new GUI_2R or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_2R_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_2R_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_2R

% Last Modified by GUIDE v2.5 19-May-2022 08:23:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_2R_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_2R_OutputFcn, ...
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

function gui_2R_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_2R (see VARARGIN)

% Choose default command line output for gui_2R
handles.output = hObject;
handles.q = zeros(4,1);
handles.pos = zeros(3,1);
handles.ori = zeros(3,1);
l = [4 2 2 1];
L(1) = Link('revolute' ,'alpha',0,    'a',0,    'd',l(1)  ,'offset',0, 'modified');
L(2) = Link('revolute' ,'alpha',0,    'a',l(2), 'd',0     ,'offset',0, 'modified');
L(3) = Link('prismatic','alpha',pi,   'a',l(3), 'theta',0 ,'offset',0, 'qlim', [0 4], 'modified');
L(4) = Link('revolute' ,'alpha',0,    'a',0,    'd',0     ,'offset',0, 'modified');
SCARA = SerialLink(L,'name','SCARA');
SCARA.tool = transl(0,0,l(4));
handles.robot = SCARA;
handles.l = l;

set(handles.btnr_ue,'Value',1);
set(handles.btnr_de,'Value',0);
handles.config = 1;

handles.ws = [-8 8 -8 8 0 8];
axes(handles.axes_plot)
SCARA.plot([0 0 0 0],'workspace', handles.ws,'tilesize',5,'noname','lightpos',[0 0 50],...
    'tile1color',[0.5 1 0.5])
hold on
trplot(eye(4),'rgb','arrow','length',2,'frame','0')
[x_cyl, y_cyl, z_cyl] = cylinder(.15,20);
z_cyl = l(1)*z_cyl;
surf(x_cyl,y_cyl,z_cyl,'EdgeColor','none','FaceColor','blue','FaceAlpha',0.95)
axis(handles.ws)

T = round(handles.robot.fkine(handles.q),2);
handles.pos = T(1:3,4);
set(handles.tb_mat, 'data', T);

handles.x_adj = 0.1;
handles.y_adj = 0.1;
handles.z_adj = 0.1;
set(handles.sli_x,'Value',handles.x_adj);
set(handles.txt_x_adj,'String',num2str(handles.x_adj));
set(handles.sli_y,'Value',handles.y_adj);
set(handles.txt_y_adj,'String',num2str(handles.y_adj));
set(handles.sli_z,'Value',handles.z_adj);
set(handles.txt_z_adj,'String',num2str(handles.z_adj));
set(handles.sli_q3,'Min',0);
set(handles.sli_q3,'Max',4);

% Update handles structure
guidata(hObject, handles);

function varargout = gui_2R_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function etxt_x_Callback(hObject, eventdata, handles)
T = handles.robot.fkine(handles.q);
handles.pos = T(1:3,4);
x = str2double(get(hObject,'String'));
handles.pos(1) = x;
T(1:3,4) = handles.pos;
q_inv = inv_scara(T, handles.l, handles.config);
q1 = q_inv(1);
q2 = q_inv(2);
q3 = q_inv(3);
q4 = q_inv(4);
handles.q = [q1 q2 q3 q4];
update_MTH_Panel(hObject, handles)
update_task_Panel(hObject, handles)
update_config_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function etxt_x_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function etxt_y_Callback(hObject, eventdata, handles)
T = handles.robot.fkine(handles.q);
handles.pos = T(1:3,4);
y = str2double(get(hObject,'String'));
handles.pos(2) = y;
T(1:3,4) = handles.pos;
q_inv = inv_scara(T, handles.l, handles.config);
q1 = q_inv(1);
q2 = q_inv(2);
q3 = q_inv(3);
q4 = q_inv(4);
handles.q = [q1 q2 q3 q4];
update_MTH_Panel(hObject, handles)
update_task_Panel(hObject, handles)
update_config_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function etxt_y_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function etxt_z_Callback(hObject, eventdata, handles)
T = handles.robot.fkine(handles.q);
handles.pos = T(1:3,4);
z = str2double(get(hObject,'String'));
handles.pos(3) = z;
T(1:3,4) = handles.pos;
q_inv = inv_scara(T, handles.l, handles.config);
q1 = q_inv(1);
q2 = q_inv(2);
q3 = q_inv(3);
q4 = q_inv(4);
handles.q = [q1 q2 q3 q4];
update_MTH_Panel(hObject, handles)
update_task_Panel(hObject, handles)
update_config_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function etxt_z_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function etxt_R_Callback(hObject, eventdata, handles)

function etxt_R_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function etxt_P_Callback(hObject, eventdata, handles)

function etxt_P_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function etxt_Y_Callback(hObject, eventdata, handles)

function etxt_Y_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function sli_q1_Callback(hObject, eventdata, handles)
q1 = get(hObject,'Value');
q2 = handles.q(2);
q3 = handles.q(3);
q4 = handles.q(4);
handles.q(1) = q1;
handles.etxt_q1.String = num2str(q1);
update_task_Panel(hObject, handles)
update_MTH_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function sli_q1_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function sli_q2_Callback(hObject, eventdata, handles)
q2 = get(hObject,'Value');
q1 = handles.q(1);
q3 = handles.q(3);
q4 = handles.q(4);
handles.q(2) = q2;
handles.etxt_q2.String = num2str(q2);
update_task_Panel(hObject, handles)
update_MTH_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function sli_q2_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function sli_q3_Callback(hObject, eventdata, handles)
q3 = get(hObject,'Value');
q1 = handles.q(1);
q2 = handles.q(2);
q4 = handles.q(4);
handles.q(3) = q3;
handles.etxt_q3.String = num2str(q3);
update_task_Panel(hObject, handles)
update_MTH_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function sli_q3_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function sli_q4_Callback(hObject, eventdata, handles)
q4 = get(hObject,'Value');
q1 = handles.q(1);
q2 = handles.q(2);
q3 = handles.q(3);
handles.q(4) = q4;
handles.etxt_q4.String = num2str(q4);
update_task_Panel(hObject, handles)
update_MTH_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function sli_q4_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function etxt_q1_Callback(hObject, eventdata, handles)
handles.q(1) = str2double(get(hObject,'String'));
handles.sli_q1.Value = handles.q(1);
q1 = handles.q(1);
q2 = handles.q(2);
q3 = handles.q(3);
q4 = handles.q(4);
update_task_Panel(hObject, handles)
update_MTH_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function etxt_q1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function etxt_q2_Callback(hObject, eventdata, handles)
handles.q(2) = str2double(get(hObject,'String'));
handles.sli_q2.Value = handles.q(2);
q1 = handles.q(1);
q2 = handles.q(2);
q3 = handles.q(3);
q4 = handles.q(4);
update_task_Panel(hObject, handles)
update_MTH_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function etxt_q2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function etxt_q3_Callback(hObject, eventdata, handles)
handles.q(3) = str2double(get(hObject,'String'));
handles.sli_q3.Value = handles.q(3);
q1 = handles.q(1);
q2 = handles.q(2);
q3 = handles.q(3);
q4 = handles.q(4);
update_task_Panel(hObject, handles)
update_MTH_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function etxt_q3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function etxt_q4_Callback(hObject, eventdata, handles)
handles.q(4) = str2double(get(hObject,'String'));
handles.sli_q4.Value = handles.q(4);
q1 = handles.q(1);
q2 = handles.q(2);
q3 = handles.q(3);
q4 = handles.q(4);
update_task_Panel(hObject, handles)
update_MTH_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function etxt_q4_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tb_mat_CreateFcn(hObject, eventdata, handles)

function btn_px_Callback(hObject, eventdata, handles)
T = handles.robot.fkine(handles.q);
handles.pos = T(1:3,4);
x = handles.pos(1);
disp(x)
x = x + handles.x_adj;
handles.pos(1) = x;
T(1:3,4) = handles.pos;
q_inv = inv_scara(T, handles.l, handles.config);
q1 = q_inv(1);
q2 = q_inv(2);
q3 = q_inv(3);
q4 = q_inv(4);
handles.q = [q1 q2 q3 q4];
update_MTH_Panel(hObject, handles)
update_task_Panel(hObject, handles)
update_config_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function btn_mx_Callback(hObject, eventdata, handles)
T = handles.robot.fkine(handles.q);
handles.pos = T(1:3,4);
x = handles.pos(1);
disp(x)
x = x - handles.x_adj;
handles.pos(1) = x;
T(1:3,4) = handles.pos;
q_inv = inv_scara(T, handles.l, handles.config);
q1 = q_inv(1);
q2 = q_inv(2);
q3 = q_inv(3);
q4 = q_inv(4);
handles.q = [q1 q2 q3 q4];
update_MTH_Panel(hObject, handles)
update_task_Panel(hObject, handles)
update_config_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function btn_py_Callback(hObject, eventdata, handles)
T = handles.robot.fkine(handles.q);
handles.pos = T(1:3,4);
y = handles.pos(2);
disp(y)
y = y + handles.y_adj;
handles.pos(2) = y;
T(1:3,4) = handles.pos;
q_inv = inv_scara(T, handles.l, handles.config);
q1 = q_inv(1);
q2 = q_inv(2);
q3 = q_inv(3);
q4 = q_inv(4);
handles.q = [q1 q2 q3 q4];
update_MTH_Panel(hObject, handles)
update_task_Panel(hObject, handles)
update_config_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function btn_my_Callback(hObject, eventdata, handles)
T = handles.robot.fkine(handles.q);
handles.pos = T(1:3,4);
y = handles.pos(2);
disp(y)
y = y - handles.y_adj;
handles.pos(2) = y;
T(1:3,4) = handles.pos;
q_inv = inv_scara(T, handles.l, handles.config);
q1 = q_inv(1);
q2 = q_inv(2);
q3 = q_inv(3);
q4 = q_inv(4);
handles.q = [q1 q2 q3 q4];
update_MTH_Panel(hObject, handles)
update_task_Panel(hObject, handles)
update_config_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function btn_pz_Callback(hObject, eventdata, handles)
T = handles.robot.fkine(handles.q);
handles.pos = T(1:3,4);
z = handles.pos(3);
disp(z)
z = z + handles.z_adj;
handles.pos(3) = z;
T(1:3,4) = handles.pos;
q_inv = inv_scara(T, handles.l, handles.config);
q1 = q_inv(1);
q2 = q_inv(2);
q3 = q_inv(3);
q4 = q_inv(4);
handles.q = [q1 q2 q3 q4];
update_MTH_Panel(hObject, handles)
update_task_Panel(hObject, handles)
update_config_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function btn_mz_Callback(hObject, eventdata, handles)
T = handles.robot.fkine(handles.q);
handles.pos = T(1:3,4);
z = handles.pos(3);
disp(z)
z = z - handles.z_adj;
handles.pos(3) = z;
T(1:3,4) = handles.pos;
q_inv = inv_scara(T, handles.l, handles.config);
q1 = q_inv(1);
q2 = q_inv(2);
q3 = q_inv(3);
q4 = q_inv(4);
handles.q = [q1 q2 q3 q4];
update_MTH_Panel(hObject, handles)
update_task_Panel(hObject, handles)
update_config_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function sli_z_Callback(hObject, eventdata, handles)
handles.z_adj = get(hObject,'Value');
handles.txt_z_adj.String = num2str(round(handles.z_adj,2));
guidata(hObject, handles);

function sli_z_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function txt_z_adj_CreateFcn(hObject, eventdata, handles)

function sli_y_Callback(hObject, eventdata, handles)
handles.y_adj = get(hObject,'Value');
handles.txt_y_adj.String = num2str(round(handles.y_adj,2));
guidata(hObject, handles);

function sli_y_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function txt_y_adj_CreateFcn(hObject, eventdata, handles)

function sli_x_Callback(hObject, eventdata, handles)
handles.x_adj = get(hObject,'Value');
handles.txt_x_adj.String = num2str(round(handles.x_adj,2));
guidata(hObject, handles);

function sli_x_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function txt_x_adj_CreateFcn(hObject, eventdata, handles)

function btnr_ue_Callback(hObject, eventdata, handles)
handles.config = 1;
T = handles.robot.fkine(handles.q);
q_inv = inv_scara(T, handles.l, handles.config);
q1 = q_inv(1);
q2 = q_inv(2);
q3 = q_inv(3);
q4 = q_inv(4);
handles.q = [q1 q2 q3 q4];
update_MTH_Panel(hObject, handles)
update_task_Panel(hObject, handles)
update_config_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function btnr_de_Callback(hObject, eventdata, handles)
handles.config = -1;
T = handles.robot.fkine(handles.q);
q_inv = inv_scara(T, handles.l, handles.config);
q1 = q_inv(1);
q2 = q_inv(2);
q3 = q_inv(3);
q4 = q_inv(4);
handles.q = [q1 q2 q3 q4];
update_MTH_Panel(hObject, handles)
update_task_Panel(hObject, handles)
update_config_Panel(hObject, handles)
axes(handles.axes_plot)
handles.robot.plot([q1 q2 q3 q4],'workspace',handles.ws,'noname');
guidata(hObject, handles);

function update_task_Panel(hObject, handles)
T = handles.robot.fkine(handles.q);
handles.pos = T(1:3,4);
handles.etxt_x.String = num2str(round(handles.pos(1),3));
handles.etxt_y.String = num2str(round(handles.pos(2),3));
handles.etxt_z.String = num2str(round(handles.pos(3),3));
handles.ori = tr2rpy(T);
handles.etxt_R.String = num2str(round(rad2deg(handles.ori(1)),1));
handles.etxt_P.String = num2str(round(rad2deg(handles.ori(2)),1));
handles.etxt_Y.String = num2str(round(rad2deg(handles.ori(3)),1));
guidata(hObject, handles);

function update_config_Panel(hObject, handles)
handles.sli_q1.Value = handles.q(1);
handles.sli_q2.Value = handles.q(2);
handles.sli_q3.Value = handles.q(3);
handles.sli_q4.Value = handles.q(4);
handles.etxt_q1.String = num2str(round(handles.q(1),3));
handles.etxt_q2.String = num2str(round(handles.q(2),3));
handles.etxt_q3.String = num2str(round(handles.q(3),3));
handles.etxt_q4.String = num2str(round(handles.q(4),3));
guidata(hObject, handles);

function update_MTH_Panel(hObject, handles)
T = handles.robot.fkine(handles.q);
set(handles.tb_mat, 'data', round(T,2));
guidata(hObject, handles);

function uipanel3_CreateFcn(hObject, eventdata, handles)
