function varargout = GUI(varargin)
% GUI MATLAB code for GUI.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI

% Last Modified by GUIDE v2.5 25-May-2018 14:24:34

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_OutputFcn, ...
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

% --- Executes just before GUI is made visible.
function GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI (see VARARGIN)

% Choose default command line output for GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using GUI.
% if strcmp(get(hObject,'Visible'),'off')
%     plot(rand(3));
% end

% UIWAIT makes GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
        popup_sel_index = get(handles.popupmenu1, 'Value');
        RT=str2double(get(handles.RT,'String'));
        MT=str2double(get(handles.MT,'String'));
        X=str2double(get(handles.X,'String'));
        Y=str2double(get(handles.Y,'String'));
        VX=str2double(get(handles.VX,'String'));
        VY=str2double(get(handles.VY,'String'));
        XA=str2double(get(handles.XA,'String'));
        YA=str2double(get(handles.YA,'String'));
        T=str2double(get(handles.T,'String'));
switch popup_sel_index
    case 1
       [Z_Real,Z_Noise,XCVE,RunTime,MonteCarloTimes,r]=FtestCV_A(T,RT,MT, [X Y]', [VX VY]', [XA YA]');
        
    case 2

        [Z_Real,Z_Noise,XCVE,RunTime,MonteCarloTimes,r]=FtestCV(T,RT,MT, [X Y]', [VX VY]', [XA YA]');
        
    case 3
         [Z_Real,Z_Noise,XCVE,RunTime,MonteCarloTimes,r]=FtestCV_Turn(T,RT,MT, [X Y]', [VX VY]', [XA YA]');
         
    case 4
         XA=-(VX/20);
         YA=-(VY/20);
         [Z_Real,Z_Noise,XCVE,RunTime,MonteCarloTimes,r]=FtestCVA_Turn(T,RT,MT, [X Y]', [VX VY]', [XA YA]');
    case 5
        [Z_Real,Z_Noise,XCVE,RunTime,MonteCarloTimes,r]=FtestCV_Turn2(T,RT,MT, [X Y]', [VX VY]', [XA YA]');
end
        axes(handles.axes1);
        cla;
        %time step-Position figure
        plot(Z_Real(1,1:RunTime),Z_Real(4,1:RunTime),'K');
        hold on
        plot(Z_Noise(1,1:RunTime),Z_Noise(2,1:RunTime), 'G');
        hold on
        plot(XCVE(1,1:RunTime),XCVE(4,1:RunTime),'R');
        xlabel('X_Position (m)');
        ylabel('Y_Position (m)');
        if popup_sel_index==1 || popup_sel_index==3 || popup_sel_index==5
            title('CV Model Kalman Filter');
        else
            title('CVA Model Kalman Filter');
        end
        legend('True', 'Measure', 'Filtered','Location','southeast');
        
        axes(handles.axes2);
        cla;
        %time step-noise compress figure
        plot(sqrt((Z_Noise(1,1:RunTime) - Z_Real(1,1:RunTime)).^2+(Z_Noise(2,1:RunTime) - Z_Real(4,1:RunTime)).^2),'K');
        hold on
        plot(sqrt((XCVE(1,1:RunTime) - Z_Real(1,1:RunTime)).^2+(XCVE(4,1:RunTime) - Z_Real(4,1:RunTime)).^2),'r');
        xlabel('Time Step');
        ylabel('Noise ComPress (m)');
        if popup_sel_index==1 || popup_sel_index==3|| popup_sel_index==5
            title('CV Model Kalman Filter');
        else
            title('CVA Model Kalman Filter');
        end
        legend('Noise', 'Compressed', 'Location','northeast');
        
        if popup_sel_index==1 || popup_sel_index==3|| popup_sel_index==5
            axes(handles.axes10);
            cla;
            set(handles.axes10,'Visible','off');
            axes(handles.axes11);
            cla;
            set(handles.axes11,'Visible','off');
            axes(handles.axes12);
            cla;
            set(handles.axes12,'Visible','off');
            axes(handles.axes13);
            cla;
            set(handles.axes13,'Visible','off');
            axes(handles.axes3);
            cla;
            set(handles.axes3,'Visible','on');
            %time step-velocity figure
            Velocity(1,:) = Z_Real(2,:);
            Velocity(2,:) = Z_Real(5,:);
            plot(1:RunTime,sqrt((Velocity(1,:)).^2+(Velocity(2,:)).^2),'K');
            hold on
            plot(1:RunTime,sqrt((XCVE(2,1:RunTime)).^2+(XCVE(5,1:RunTime)).^2),'r');
            xlabel('Time Step');
            ylabel('Velocity (m/s)');
            title('CV Model Kalman Filter');
            %legend('Model Speed', 'Filter Speed','Location','southeast');
        end
        
        if popup_sel_index==2 ||popup_sel_index==4
            axes(handles.axes3);
            cla;
            set(handles.axes3,'Visible','off');
            axes(handles.axes10);
            cla;
            set(handles.axes10,'Visible','on');
            %time step-velocity figure
            Velocity(1,:) = Z_Real(2,:);
            Velocity(2,:) = Z_Real(5,:);
            plot(1:RunTime,sqrt((Velocity(1,:)).^2+(Velocity(2,:)).^2),'K');
            hold on
            plot(1:RunTime,sqrt((XCVE(2,1:RunTime)).^2+(XCVE(5,1:RunTime)).^2),'r');
            xlabel('Time Step');
            ylabel('Velocity (m/s)');
            title('CVA Model Kalman Filter');
            %legend('Model Speed', 'Filter Speed','Location','southeast');
            
            axes(handles.axes12);
            cla;
            set(handles.axes12,'Visible','on');
            Acceleration(1,:) = Z_Real(3,:);
            Acceleration(2,:) = Z_Real(6,:);
            plot(1:RunTime,sqrt((Acceleration(1,:)).^2+(Acceleration(2,:)).^2),'K');
            hold on
            plot(1:RunTime,sqrt((XCVE(3,1:RunTime)).^2+(XCVE(6,1:RunTime)).^2),'r');
            xlabel('Time Step');
            ylabel('Acceleration(m/s/s)');
            title('CVA Model Kalman Filter');
           % legend('Model const Acceleration', 'Filter Acceleration','Location','southeast');
        end
        
        %statistics
        for i = 1 : RunTime
           Last = ((MonteCarloTimes-1)* RunTime+i);
           X_Mean_ERROR(i)= mean(sqrt(XCVE(1,i:RunTime:Last).^2+XCVE(4,i:RunTime:Last).^2)) - sqrt(Z_Real(1,i)^2+Z_Real(4,i)^2);
           V_Mean_ERROR(i)= mean(sqrt(XCVE(2,i:RunTime:Last).^2+XCVE(5,i:RunTime:Last).^2)) - sqrt(Z_Real(2,i)^2+Z_Real(5,i)^2);
           A_Mean_ERROR(i)= mean(sqrt(XCVE(3,i:RunTime:Last).^2+XCVE(6,i:RunTime:Last).^2)) - sqrt(Z_Real(3,i)^2+Z_Real(6,i)^2);
           
           Z_X_Real = sqrt(Z_Real(1,i)^2+Z_Real(4,i)^2)*ones(1,MonteCarloTimes);
           Z_V_Real = sqrt(Z_Real(2,i)^2+Z_Real(5,i)^2)*ones(1,MonteCarloTimes);
           Z_A_Real = sqrt(Z_Real(3,i)^2+Z_Real(6,i)^2)*ones(1,MonteCarloTimes);
           X_Cov_ERROR(i) = norm(sqrt(XCVE(1,i:RunTime:Last).^2+XCVE(4,i:RunTime:Last).^2)-Z_X_Real)/sqrt(MonteCarloTimes);
           V_Cov_ERROR(i) = norm(sqrt(XCVE(2,i:RunTime:Last).^2+XCVE(5,i:RunTime:Last).^2)-Z_V_Real)/sqrt(MonteCarloTimes);
           A_Cov_ERROR(i) = norm(sqrt(XCVE(3,i:RunTime:Last).^2+XCVE(6,i:RunTime:Last).^2)-Z_A_Real)/sqrt(MonteCarloTimes);
           SNA(i) = std(sqrt(XCVE(1,i:RunTime:Last).^2+XCVE(4,i:RunTime:Last).^2)-Z_X_Real)/r;
        end
        axes(handles.axes4);
        cla;
        plot(X_Cov_ERROR,'r-.');
        xlabel('Time Step');
        ylabel('RMSE');
        title('RMSE -Positon');

        axes(handles.axes6);
        cla;
        plot(V_Cov_ERROR,'r-.');
        xlabel('Time Step');
        ylabel('RMSE');
        title('RMSE -Velocity');

        axes(handles.axes7);
        cla;
        plot(X_Mean_ERROR,'r-.');
        xlabel('Time Step');
        ylabel('Mean ERROR');
        title('Mean ERROR -Positon');

        axes(handles.axes8);
        cla;
        plot(V_Mean_ERROR,'r-.');
        xlabel('Time Step');
        ylabel('Mean Error');
        title('Mean Error -Velocity');
        
        if popup_sel_index==2 ||popup_sel_index==4
            %set(handles.axe3,'Visible','off');
            axes(handles.axes11);
            cla;
            set(handles.axes11,'Visible','on');
            plot(A_Cov_ERROR,'r-.');
            xlabel('Time Step');
            ylabel('RMSE');
            title('RMSE -Acceleration');
            
            axes(handles.axes13);
            cla;
            set(handles.axes13,'Visible','on');
            plot(V_Mean_ERROR,'r-.');
            xlabel('Time Step');
            ylabel('Mean Error');
            title('Mean Error -Acceleration');
        end
        
        axes(handles.axes5);
        cla;
        %SNR
        plot(SNA,'r-.');
        xlabel('Time Step');
        ylabel('Signal Noise Ration');
        if popup_sel_index==1 || popup_sel_index==3 || popup_sel_index==5
            title('CV Model Kalman Filter');
        else
            title('CVA Model Kalman Filter');
        end

% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'testCV', 'testCV_UA','testCV_Turn','testCVA_turn','testCV_Turn2'});



function RT_Callback(hObject, eventdata, handles)
% hObject    handle to RT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of RT as text
%        str2double(get(hObject,'String')) returns contents of RT as a double


% --- Executes during object creation, after setting all properties.
function RT_CreateFcn(hObject, eventdata, handles)
% hObject    handle to RT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function MT_Callback(hObject, eventdata, handles)
% hObject    handle to MT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of MT as text
%        str2double(get(hObject,'String')) returns contents of MT as a double


% --- Executes during object creation, after setting all properties.
function MT_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function X_Callback(hObject, eventdata, handles)
% hObject    handle to X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of X as text
%        str2double(get(hObject,'String')) returns contents of X as a double


% --- Executes during object creation, after setting all properties.
function X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function VX_Callback(hObject, eventdata, handles)
% hObject    handle to VX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of VX as text
%        str2double(get(hObject,'String')) returns contents of VX as a double


% --- Executes during object creation, after setting all properties.
function VX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to VX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function XA_Callback(hObject, eventdata, handles)
% hObject    handle to XA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of XA as text
%        str2double(get(hObject,'String')) returns contents of XA as a double


% --- Executes during object creation, after setting all properties.
function XA_CreateFcn(hObject, eventdata, handles)
% hObject    handle to XA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Y_Callback(hObject, eventdata, handles)
% hObject    handle to Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Y as text
%        str2double(get(hObject,'String')) returns contents of Y as a double


% --- Executes during object creation, after setting all properties.
function Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function VY_Callback(hObject, eventdata, handles)
% hObject    handle to VY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of VY as text
%        str2double(get(hObject,'String')) returns contents of VY as a double


% --- Executes during object creation, after setting all properties.
function VY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to VY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function YA_Callback(hObject, eventdata, handles)
% hObject    handle to YA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of YA as text
%        str2double(get(hObject,'String')) returns contents of YA as a double


% --- Executes during object creation, after setting all properties.
function YA_CreateFcn(hObject, eventdata, handles)
% hObject    handle to YA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T_Callback(hObject, eventdata, handles)
% hObject    handle to T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T as text
%        str2double(get(hObject,'String')) returns contents of T as a double


% --- Executes during object creation, after setting all properties.
function T_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
