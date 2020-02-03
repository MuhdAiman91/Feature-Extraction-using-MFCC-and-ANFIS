function varargout = datalatih(varargin)
% DATALATIH MATLAB code for datalatih.fig
%      DATALATIH, by itself, creates a new DATALATIH or raises the existing
%      singleton*.
%
%      H = DATALATIH returns the handle to a new DATALATIH or the handle to
%      the existing singleton*.
%
%      DATALATIH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DATALATIH.M with the given input arguments.
%
%      DATALATIH('Property','Value',...) creates a new DATALATIH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before datalatih_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to datalatih_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help datalatih

% Last Modified by GUIDE v2.5 17-Oct-2019 14:04:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @datalatih_OpeningFcn, ...
                   'gui_OutputFcn',  @datalatih_OutputFcn, ...
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


% --- Executes just before datalatih is made visible.
function datalatih_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to datalatih (see VARARGIN)

% Choose default command line output for datalatih
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
movegui(gcf, 'center');
% UIWAIT makes datalatih wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = datalatih_OutputFcn(hObject, eventdata, handles) 
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
%mengambil data latih
[nama_file nama_folder] = uigetfile({'*.wav'}, 'Ambil File Data Latih');

if (nama_file(1)==0)&&(nama_folder(1)==0)
    errordlg('Silahkan pilih data latih terlebih dahulu!');
end
Fs = 16000;
suara = wavread([nama_folder nama_file]);
[y ,fs] = wavread([nama_folder nama_file]);
t = [1/Fs:1/Fs:length(y)/Fs];
y = resample(y,Fs,fs);
%wavwrite('DataProses.wav',y,Fs);
%suara = wavread('DataProses.wav');
%normalization amplitudo
tic
x1 = abs(suara);
xmax = max(x1);
xnorm = suara / xmax;
s = potong(xnorm);
%nyimpen data supaya bisa dipanggil di fungsi berikutnya
handles.y=s;
handles.Fs=Fs;
guidata(hObject, handles);
k = s(90:1:100);
%plot grafik
proyek=guidata(gcbo);
axes(proyek.axes1);
plot(t,y,'Color', [0, 1, 0.6]);
set(gca, 'Color', 'None');
set(proyek.axes1,'XColor',[0, 1, 0.6])
set(proyek.axes1,'YColor',[0, 1, 0.6])
set(proyek.axes1,'XMinorTick','on');
set(proyek.axes1,'YMinorTick','on');
grid on;
disp(k);
%dlmwrite('Initial.txt',s);
set(handles.pushbutton2,'visible','on')

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Step 1: Pre-Emphasis
x=handles.y;
fs=handles.Fs;
TSn = 0.01; %Frame Step in Seconds
Ts=TSn;            %Frame step in seconds
Tf=0.025;           %Frame duration in seconds
N=fs*Tf;            %Number of samples per frame
l=length(x);        %total number of samples in signal audio
FrameStep=fs*Ts;    %Frame step in samples
noFrames = floor(l/FrameStep);
xx=length(x)-1;

a=1;
b=[1, -0.97];       %a and b are high pass filter coefficients

for i=1:noFrames-2
z=filter(b,a,x);
end
%for i=2:xx 
%z0(i)=x(i-1);
%end

%for i=1:xx 
%z(i) = x(i) - (alpha*z0(i)); 
%end
f = z(90:1:100);
%ambil nilai
handles.z=z;
handles.fs=fs;
handles.N=N;
handles.noFrames=noFrames;
handles.FrameStep=FrameStep;
guidata(hObject, handles);

%plot grafik
proyek=guidata(gcbo);
axes(proyek.axes2);
t=1:xx;
j=t/fs;
plot(z, 'Color', [1, 0.4, 0.3]);
set(gca, 'Color', 'None');
set(proyek.axes2,'XColor',[1, 0.4, 0.3])
set(proyek.axes2,'YColor',[1, 0.4, 0.3])
set(proyek.axes2,'XMinorTick','on');
set(proyek.axes2,'YMinorTick','on');
xlabel('Time');
ylabel('Amplitude');
grid on;
display(f);
%display(max(z));
%dlmwrite('Preemphasis.txt',z);
set(handles.pushbutton3,'visible','on')

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Step 2: Frame Blocking
s=handles.z;
noFrames=handles.noFrames;
FrameStep=handles.FrameStep;
N=handles.N;

FrameBlocking=[];

if mean(abs(s)) > 0.01
    s=s/max(s);                     %Normalises to compensate for mic vol differences
end

%Segment the signal into overlapping frames

for i=1:noFrames-2
    frame=s((i-1)*FrameStep+1:(i-1)*FrameStep+N);  %Holds individual frames
    FrameBlocking(i,:)=frame';
end
q = frame(90:1:100);
%plot grafik
proyek=guidata(gcbo);
axes(proyek.axes3);
set(handles.edit1,'String',N);
plot(frame,'Color', [1, 0.4, 0.3]);
set(gca, 'Color', 'None');
set(proyek.axes3,'XColor',[1, 0.4, 0.3])
set(proyek.axes3,'YColor',[1, 0.4, 0.3])
xlabel('Samples');
%ylabel('Amplitude');
set(proyek.axes3,'XMinorTick','on');
set(proyek.axes3,'YMinorTick','on');
grid on;

figure,
plot(FrameBlocking);
%ambil nilai
%handles.FrameBlocking=FrameBlocking
handles.N=N;
handles.noFrames=noFrames;
handles.frame=frame;
%display(q);
%dlmwrite('FrameBlocking.txt',frame);
%display(frame);
%display(s);
guidata(hObject, handles);

set(handles.pushbutton4,'visible','on')

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Step 3: Windowing
alpha=handles.frame;
n=handles.fs;
noFrames=handles.noFrames;
N=length(alpha);

for i=1:noFrames-2 
    F = hamming(N);
    q = F.*alpha;
    %h(noFrames) = 0.54-(0.46*cos((2*pi*n)/(N-1)));
    %hx = h(noFrames).*alpha;
end
%h=h(noFrames);
w = q(1:1:10);
%F = F(90:1:100);
%ambil nilai
handles.q=q;
guidata(hObject, handles);

%plot grafik
proyek=guidata(gcbo);
axes(proyek.axes5);
plot(q,'Color', [1, 0.4, 0.3]);
set(gca, 'Color', 'None');
set(proyek.axes5,'XColor',[1, 0.4, 0.3])
set(proyek.axes5,'YColor',[1, 0.4, 0.3])
xlabel('Samples');
%ylabel('Amplitude');
set(proyek.axes5,'XMinorTick','on');
set(proyek.axes5,'YMinorTick','on');
grid on;
%disp(h(noFrames));
%w = hx(90:100);
display(w);
%dlmwrite('curios.txt',F);
%hx(200)=[];
%display(F);
set(handles.pushbutton5,'visible','on')

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Step 4: Fast-Fourier Transform
N=handles.N;
hx=handles.q;
noFrames=handles.noFrames;

for i=1:noFrames-2
	FT=fft(hx,N);              %computes the fft
end
f = FT(1:1:24);
% normalized FFT of signal
%S=(fftshift(fft(hx,512))/(N));
% power spectrum
%Sp=10*log10((abs(S).^2));
%ambil nilai
handles.FT=FT;
guidata(hObject, handles);
%fvals=(0:N-1)/N;
%mag=abs(FT);
%figure;plot(fvals,mag);
%xlabel('Frenquency'); ylabel('P(f)'); title('Power spectrum by FFT method');
%plot grafik
proyek=guidata(gcbo);
axes(proyek.axes6);
plot(FT,'Color', [1, 0.4, 0.3]);
set(gca, 'Color', 'None');
set(proyek.axes6,'XColor',[1, 0.4, 0.3])
set(proyek.axes6,'YColor',[1, 0.4, 0.3])
xlabel('');
set(proyek.axes6,'XMinorTick','on');
set(proyek.axes6,'YMinorTick','on');
grid on;
%dlmwrite('FFT.txt',FT);
display(f);
%display(Sp);
set(handles.pushbutton6,'visible','on')

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, ~, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Step 4: Mel-Frequency Wrapping
FT=handles.FT;
noFrames=handles.noFrames;
Fs=handles.Fs;
fn=24;              %Number of mel filters
n=512;              %Number of FFT points
for i=1:noFrames-2
	melf=melbankm(fn,n,Fs);     %creates 24 filter, mel filter bank
    halfn=1+floor(n/2);    
    spectr1=log10(melf*abs(FT(1:halfn)).^2);%result is mel-scale filtered
    %Sp=(abs(FT(1:halfn)).^2);
end
z = sum(FT);
figure,
plot(melf);
xlabel('Mel filters');
ylabel('Amplitude');
%m = melf(1:1:10);
%ambil nilai
handles.spectr1=spectr1;
guidata(hObject, handles);

%plot grafik
proyek=guidata(gcbo);
axes(proyek.axes7);
plot(spectr1,'Color', [1, 0.4, 0.3]);
set(gca, 'Color', 'None');
set(proyek.axes7,'XColor',[1, 0.4, 0.3])
set(proyek.axes7,'YColor',[1, 0.4, 0.3])
xlabel('Mel Filters');
set(proyek.axes7,'XMinorTick','on');
set(proyek.axes7,'YMinorTick','on');
grid on;
%clcdisplay(halfn);
%display(Sp);
display(spectr1);
%display(melf);
set(handles.pushbutton7,'visible','on')


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%Step 6:Cepstrum%%
frame=handles.frame;
spectr1=handles.spectr1;
noFrames=handles.noFrames;
num = 13;
FMatrix=zeros(noFrames-2, num); %Matrix to hold cepstral coefficients
lifter=1:num;                   %Lifter vector index
lifter=1+floor((num)/2)*(sin(lifter*pi/num));%raised sine lifter version

for i=1:noFrames-2
	Ce1=sum(frame.^2);          %Frame energy
    Ce2=max(Ce1,2e-22);         %floors to 2 X 10 raised to power -22
    Ce=log(Ce2);
    spectr=max(spectr1(:),1e-22);
    c=dct(spectr);              %obtains DCT, changes to cepstral domain
    c(1)=Ce;                    %replaces first coefficient
    coeffs=c(1:num);            %retains first num coefficients
    ncoeffs=coeffs.*lifter';    %Multiplies coefficients by lifter value
    FMatrix(i, :)=ncoeffs';  
    koef=ncoeffs'; 
end
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

set(handles.edit2,'String',num2str(K1));
set(handles.edit3,'String',num2str(K2));
set(handles.edit4,'String',num2str(K3));
set(handles.edit5,'String',num2str(K4));
set(handles.edit6,'String',num2str(K5));
set(handles.edit7,'String',num2str(K6));
set(handles.edit8,'String',num2str(K7));
set(handles.edit9,'String',num2str(K8));
set(handles.edit10,'String',num2str(K9));
set(handles.edit11,'String',num2str(K10));
set(handles.edit12,'String',num2str(K11));
set(handles.edit13,'String',num2str(K12));
set(handles.edit14,'String',num2str(K13));
%plot grafik
proyek=guidata(gcbo);
axes(proyek.axes8);
plot(spectr1,'Color', [1, 0.4, 0.3]);
set(gca, 'Color', 'None');
set(proyek.axes8,'XColor',[1, 0.4, 0.3])
set(proyek.axes8,'YColor',[1, 0.4, 0.3])
xlabel('Mel Filters');
set(proyek.axes8,'XMinorTick','on');
set(proyek.axes8,'YMinorTick','on');
grid on;
%clcdisplay(halfn);
%display(lifter);
%display(x);
display(koef);
%set(handles.pushbutton7,'visible','on')

function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close;
AplikasiDeteksiArahan



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
