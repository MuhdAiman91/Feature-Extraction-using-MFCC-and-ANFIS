function varargout = AplikasiDeteksiArahan(varargin)
% APLIKASIDETEKSIARAHAN MATLAB code for AplikasiDeteksiArahan.fig
%      APLIKASIDETEKSIARAHAN, by itself, creates a new APLIKASIDETEKSIARAHAN or raises the existing
%      singleton*.
%
%      H = APLIKASIDETEKSIARAHAN returns the handle to a new APLIKASIDETEKSIARAHAN or the handle to
%      the existing singleton*.
%
%      APLIKASIDETEKSIARAHAN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in APLIKASIDETEKSIARAHAN.M with the given input arguments.
%
%      APLIKASIDETEKSIARAHAN('Property','Value',...) creates a new APLIKASIDETEKSIARAHAN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before AplikasiDeteksiArahan_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to AplikasiDeteksiArahan_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help AplikasiDeteksiArahan

% Last Modified by GUIDE v2.5 02-Sep-2019 11:30:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @AplikasiDeteksiArahan_OpeningFcn, ...
                   'gui_OutputFcn',  @AplikasiDeteksiArahan_OutputFcn, ...
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


% --- Executes just before AplikasiDeteksiArahan is made visible.
function AplikasiDeteksiArahan_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to AplikasiDeteksiArahan (see VARARGIN)

% Choose default command line output for AplikasiDeteksiArahan
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
movegui(gcf, 'center');

axes(handles.axes1);
image1=imread('aplikasi.jpg');
%imshow(image,'aplikasi.jpg',handles.axes1);
imshow(image1);

axes(handles.axes2);
image2=imread('teknikelektro.jpeg');
imshow(image2);

axes(handles.axes3);
image3=imread('UNLA.jpg');
imshow(image3);



% UIWAIT makes AplikasiDeteksiArahan wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = AplikasiDeteksiArahan_OutputFcn(hObject, eventdata, handles) 
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
close;
datalatih;

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close;
GUI_pengujian;
