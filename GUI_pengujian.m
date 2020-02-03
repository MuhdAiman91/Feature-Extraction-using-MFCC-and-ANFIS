function varargout = GUI_pengujian(varargin)
%GUI_pengujian M-file for GUI_pengujian.fig
%      GUI_pengujian, by itself, creates a new GUI_pengujian or raises the existing
%      singleton*.
%
%      H = GUI_pengujian returns the handle to a new GUI_pengujian or the handle to
%      the existing singleton*.
%
%      GUI_pengujian('Property','Value',...) creates a new GUI_pengujian using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to GUI_pengujian_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      GUI_pengujian('CALLBACK') and GUI_pengujian('CALLBACK',hObject,...) call the
%      local function named CALLBACK in GUI_pengujian.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_pengujian

% Last Modified by GUIDE v2.5 21-Oct-2016 05:40:47

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_pengujian_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_pengujian_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
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


% --- Executes just before GUI_pengujian is made visible.
function GUI_pengujian_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for GUI_pengujian
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI_pengujian wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_pengujian_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in record.
function record_Callback(hObject, eventdata, handles)
% hObject    handle to record (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%global ser
Fs = 16000;
suara = wavrecord(1*Fs, Fs);
%normalization amplitudo
tic
x1 = abs(suara);
xmax = max(x1);
xnorm = suara / xmax;
s = potong(xnorm);
plot(s);
title('SINYAL SUARA');
xlabel('Time');
ylabel('Magnitudo');
num=13;% jumlah ciri sinyal wicara

n=512;              %Number of FFT points
Tf=0.025;           %Frame duration in seconds
N=Fs*Tf;            %Number of samples per frame
fn=24;              %Number of mel filters
l=length(s);        %total number of samples in speech
Ts=0.01;            %Frame step in seconds
FrameStep=Fs*Ts;    %Frame step in samples
a=1;
b=[1, -0.97];       %a and b are high pass filter coefficients

noFrames=floor(l/FrameStep);    %Maximum no of frames in speech sample
FMatrix=zeros(noFrames-2, num); %Matrix to hold cepstral coefficients
lifter=1:num;                   %Lifter vector index
lifter=1+floor((num)/2)*(sin(lifter*pi/num));%raised sine lifter version

if mean(abs(s)) > 0.01
    s=s/max(s);                     %Normalises to compensate for mic vol differences
end

%Segment the signal into overlapping frames and compute MFCC coefficients
for i=1:noFrames-2
    frame=s((i-1)*FrameStep+1:(i-1)*FrameStep+N);  %Holds individual frames
    Ce1=sum(frame.^2);          %Frame energy
    Ce2=max(Ce1,2e-22);         %floors to 2 X 10 raised to power -22
    Ce=log(Ce2);
    framef=filter(b,a,frame);   %High pass pre-emphasis filter
    F=framef.*hamming(N);       %multiplies each frame with hamming window
    FFTo=fft(F,N);              %computes the fft
    melf=melbankm(fn,n,Fs);     %creates 24 filter, mel filter bank
    halfn=1+floor(n/2);    
    spectr1=log10(melf*abs(FFTo(1:halfn)).^2);%result is mel-scale filtered
    spectr=max(spectr1(:),1e-22);
    c=dct(spectr);              %obtains DCT, changes to cepstral domain
    c(1)=Ce;                    %replaces first coefficient
    coeffs=c(1:num);            %retains first num coefficients
    ncoeffs=coeffs.*lifter';    %Multiplies coefficients by lifter value
    FMatrix(i, :)=ncoeffs';  
    koef=ncoeffs'; 
end
% Program pengujian jaringan syaraf tiruan dengan memanggil model JST
K1=koef(1);
K2=koef(2);
K3=koef(3);
K4=koef(4);
K5=koef(5);
K6=koef(6);
K7=koef(7);
K8=koef(8);
K9=koef(9);
K10=koef(10);
K11=koef(11);
K12=koef(12);
K13=koef(13);


set(handles.outciri1,'String',num2str(K1)); 
set(handles.outciri2,'String',num2str(K2)); 
set(handles.outciri3,'String',num2str(K3)); 
set(handles.outciri4,'String',num2str(K4));
set(handles.outciri5,'String',num2str(K5)); 
set(handles.outciri6,'String',num2str(K6));
set(handles.outciri7,'String',num2str(K7));
set(handles.outciri8,'String',num2str(K8));
set(handles.outciri9,'String',num2str(K9));
set(handles.outciri10,'String',num2str(K10));
set(handles.outciri11,'String',num2str(K11));
set(handles.outciri12,'String',num2str(K12));
set(handles.outciri13,'String',num2str(K13));


% memanggil file ANFIS
a=readfis('13coeff_TA'); 
out=evalfis([K1, K2, K3, K4, K5, K6, K7, K8, K9, K10, K11, K12, K13],a);
set(handles.out_sistem,'String',out);

% Program komunikasi serial komputer dan arduino

if out>=-0.5 && out<0.5
%fwrite(handles.ser,'a');
set(handles.karakter,'String','MATI');
elseif out>=0.5 && out<=1.5
%fwrite(handles.ser,'b');
set(handles.karakter,'String','NYALA');
else
%fwrite(handles.ser,'c');
set(handles.karakter,'String','TAK DIKENAL');
end




function com_Callback(hObject, eventdata, handles)
% hObject    handle to com (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of com as text
%        str2double(get(hObject,'String')) returns contents of com as a double


% --- Executes during object creation, after setting all properties.
function com_CreateFcn(hObject, eventdata, handles)
% hObject    handle to com (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function out_sistem_Callback(hObject, eventdata, handles)
% hObject    handle to out_sistem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of out_sistem as text
%        str2double(get(hObject,'String')) returns contents of out_sistem as a double


% --- Executes during object creation, after setting all properties.
function out_sistem_CreateFcn(hObject, eventdata, handles)
% hObject    handle to out_sistem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function karakter_Callback(hObject, eventdata, handles)
% hObject    handle to karakter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of karakter as text
%        str2double(get(hObject,'String')) returns contents of karakter as a double


% --- Executes during object creation, after setting all properties.
function karakter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to karakter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outciri1_Callback(hObject, eventdata, handles)
% hObject    handle to outciri1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri1 as text
%        str2double(get(hObject,'String')) returns contents of outciri1 as a double


% --- Executes during object creation, after setting all properties.
function outciri1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outciri2_Callback(hObject, eventdata, handles)
% hObject    handle to outciri2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri2 as text
%        str2double(get(hObject,'String')) returns contents of outciri2 as a double


% --- Executes during object creation, after setting all properties.
function outciri2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outciri3_Callback(hObject, eventdata, handles)
% hObject    handle to outciri3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri3 as text
%        str2double(get(hObject,'String')) returns contents of outciri3 as a double


% --- Executes during object creation, after setting all properties.
function outciri3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outciri4_Callback(hObject, eventdata, handles)
% hObject    handle to outciri4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri4 as text
%        str2double(get(hObject,'String')) returns contents of outciri4 as a double


% --- Executes during object creation, after setting all properties.
function outciri4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outciri5_Callback(hObject, eventdata, handles)
% hObject    handle to outciri5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri5 as text
%        str2double(get(hObject,'String')) returns contents of outciri5 as a double


% --- Executes during object creation, after setting all properties.
function outciri5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outciri6_Callback(hObject, eventdata, handles)
% hObject    handle to outciri6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri6 as text
%        str2double(get(hObject,'String')) returns contents of outciri6 as a double


% --- Executes during object creation, after setting all properties.
function outciri6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in koneksi.
function koneksi_Callback(hObject, eventdata, handles)
% hObject    handle to koneksi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
delete(instrfindall);
handles.output = hObject;
clear ser;
global ser;
handles.ser=ser;
handles.ser=serial('COM8','BAUD', 9600);
fopen(handles.ser);
guidata(hObject, handles);

%myform=guidata(gcbo);
%aa=get(myform.com,'String');
%ser=serial(aa,'Baudrate',9600,'DataBits',8,'StopBits',1,'InputBufferSize',16000);
%fopen(ser);


% --- Executes on button press in exit.
function exit_Callback(hObject, eventdata, handles)
% hObject    handle to exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ser
fclose(ser);
delete(ser);
clear ser;
close;



function outciri7_Callback(hObject, eventdata, handles)
% hObject    handle to outciri7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri7 as text
%        str2double(get(hObject,'String')) returns contents of outciri7 as a double


% --- Executes during object creation, after setting all properties.
function outciri7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outciri8_Callback(hObject, eventdata, handles)
% hObject    handle to outciri8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri8 as text
%        str2double(get(hObject,'String')) returns contents of outciri8 as a double


% --- Executes during object creation, after setting all properties.
function outciri8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outciri9_Callback(hObject, eventdata, handles)
% hObject    handle to outciri9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri9 as text
%        str2double(get(hObject,'String')) returns contents of outciri9 as a double


% --- Executes during object creation, after setting all properties.
function outciri9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outciri10_Callback(hObject, eventdata, handles)
% hObject    handle to outciri10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri10 as text
%        str2double(get(hObject,'String')) returns contents of outciri10 as a double


% --- Executes during object creation, after setting all properties.
function outciri10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outciri11_Callback(hObject, eventdata, handles)
% hObject    handle to outciri11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri11 as text
%        str2double(get(hObject,'String')) returns contents of outciri11 as a double


% --- Executes during object creation, after setting all properties.
function outciri11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outciri12_Callback(hObject, eventdata, handles)
% hObject    handle to outciri12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri12 as text
%        str2double(get(hObject,'String')) returns contents of outciri12 as a double


% --- Executes during object creation, after setting all properties.
function outciri12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function outciri13_Callback(hObject, eventdata, handles)
% hObject    handle to outciri13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of outciri13 as text
%        str2double(get(hObject,'String')) returns contents of outciri13 as a double


% --- Executes during object creation, after setting all properties.
function outciri13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outciri13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[nama_file nama_folder] = uigetfile({'*.wav'}, 'Ambil File Data Latih');

if (nama_file(1)==0)&&(nama_folder(1)==0)
    errordlg('Silahkan pilih data latih terlebih dahulu!');
end
Fs = 16000;
suara = wavread([nama_folder nama_file]);
[y ,fs] = wavread([nama_folder nama_file]);
t = [1/Fs:1/Fs:length(y)/Fs];
y = resample(y,Fs,fs);
tic
x1 = abs(suara);
xmax = max(x1);
xnorm = suara / xmax;
s = potong(xnorm);
plot(s);
title('SINYAL SUARA');
xlabel('Frequency Sampling');
ylabel('Magnitudo');
num=13;% jumlah ciri sinyal wicara

n=512;              %Number of FFT points
Tf=0.025;           %Frame duration in seconds
N=Fs*Tf;            %Number of samples per frame
fn=24;              %Number of mel filters
l=length(s);        %total number of samples in speech
Ts=0.01;            %Frame step in seconds
FrameStep=Fs*Ts;    %Frame step in samples
a=1;
b=[1, -0.97];       %a and b are high pass filter coefficients

noFrames=floor(l/FrameStep);    %Maximum no of frames in speech sample
FMatrix=zeros(noFrames-2, num); %Matrix to hold cepstral coefficients
lifter=1:num;                   %Lifter vector index
lifter=1+floor((num)/2)*(sin(lifter*pi/num));%raised sine lifter version

if mean(abs(s)) > 0.01
    s=s/max(s);                     %Normalises to compensate for mic vol differences
end

%Segment the signal into overlapping frames and compute MFCC coefficients
for i=1:noFrames-2
    frame=s((i-1)*FrameStep+1:(i-1)*FrameStep+N);  %Holds individual frames
    Ce1=sum(frame.^2);          %Frame energy
    Ce2=max(Ce1,2e-22);         %floors to 2 X 10 raised to power -22
    Ce=log(Ce2);
    framef=filter(b,a,frame);   %High pass pre-emphasis filter
    F=framef.*hamming(N);       %multiplies each frame with hamming window
    FFTo=fft(F,N);              %computes the fft
    melf=melbankm(fn,n,Fs);     %creates 24 filter, mel filter bank
    halfn=1+floor(n/2);    
    spectr1=log10(melf*abs(FFTo(1:halfn)).^2);%result is mel-scale filtered
    spectr=max(spectr1(:),1e-22);
    c=dct(spectr);              %obtains DCT, changes to cepstral domain
    c(1)=Ce;                    %replaces first coefficient
    coeffs=c(1:num);            %retains first num coefficients
    ncoeffs=coeffs.*lifter';    %Multiplies coefficients by lifter value
    FMatrix(i, :)=ncoeffs';  
    koef=ncoeffs'; 
end
% Program pengujian jaringan syaraf tiruan dengan memanggil model JST
K1=koef(1);
K2=koef(2);
K3=koef(3);
K4=koef(4);
K5=koef(5);
K6=koef(6);
K7=koef(7);
K8=koef(8);
K9=koef(9);
K10=koef(10);
K11=koef(11);
K12=koef(12);
K13=koef(13);


set(handles.outciri1,'String',num2str(K1)); 
set(handles.outciri2,'String',num2str(K2)); 
set(handles.outciri3,'String',num2str(K3)); 
set(handles.outciri4,'String',num2str(K4));
set(handles.outciri5,'String',num2str(K5)); 
set(handles.outciri6,'String',num2str(K6));
set(handles.outciri7,'String',num2str(K7));
set(handles.outciri8,'String',num2str(K8));
set(handles.outciri9,'String',num2str(K9));
set(handles.outciri10,'String',num2str(K10));
set(handles.outciri11,'String',num2str(K11));
set(handles.outciri12,'String',num2str(K12));
set(handles.outciri13,'String',num2str(K13));


% memanggil file ANFIS
a=readfis('13coeff_TA'); 
out=evalfis([K1, K2, K3, K4, K5, K6, K7, K8, K9, K10, K11, K12, K13],a);
set(handles.out_sistem,'String',out);

% Program komunikasi serial komputer dan arduino

if out>=-0.5 && out<0.5
%fwrite(handles.ser,'a');
set(handles.karakter,'String','MATI');
elseif out>=0.5 && out<=1.5
%fwrite(handles.ser,'b');
set(handles.karakter,'String','NYALA');
else
%fwrite(handles.ser,'c');
set(handles.karakter,'String','TAK DIKENAL');
end

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close;
AplikasiDeteksiArahan
