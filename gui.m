
function varargout = gui(varargin)
% GUI MATLAB code for gui.fig
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
%      applied to the GUI before gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui

% Last Modified by GUIDE v2.5 18-Jun-2017 23:07:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_OutputFcn, ...
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


% --- Executes just before gui is made visible.
function gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui (see VARARGIN)

% Choose default command line output for gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
threshold_gray = get(hObject,'Value');
set(handles.text4,'string',threshold_gray);
handles.threshold_gray = threshold_gray;
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%���ļ�
clc;
[FileName,PathName,FilterIndex]=uigetfile('*.jpg','ѡ��jpg�ļ�');
im = imread([PathName FileName]);
im = rgb2gray(im);
set(handles.text3,'string',FileName);
axes(handles.axes1) ;
cla reset;
axes(handles.axes1);
imshow(im,[]);
handles.im = im;
handles.FileName = FileName;
guidata(hObject,handles);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%���⡢�˲�
im1 = handles.im;
im1 = adapthisteq(im1);
% im1 = medfilt2(im1);
axes(handles.axes1);
imshow(im1,[]);
handles.im1 = im1;
guidata(hObject,handles);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Ѱ�������ͨ��
im4 = handles.im4;
im = handles.im;
L = bwlabel(im4);
stats = regionprops(L);
Ar = cat(1, stats.Area);
%�ҵ����������ͨ��
[~,position]=sort(Ar,'descend');
ind=position(1:4);
new_stats=stats(ind);
%������ҵ�������
axes(handles.axes1);
for k=1:length(new_stats)
    Area_array=new_stats(k).Area;
end
max_ind=find(max(Area_array));
hold on
new_stats(max_ind).BoundingBox(1)=new_stats(max_ind).BoundingBox(1)-10;
new_stats(max_ind).BoundingBox(2)=new_stats(max_ind).BoundingBox(2)-10;
new_stats(max_ind).BoundingBox(3)=new_stats(max_ind).BoundingBox(3)+25;
new_stats(max_ind).BoundingBox(4)=new_stats(max_ind).BoundingBox(4)+25;
rectangle('Position',[new_stats(max_ind).BoundingBox],'LineWidth',1,'EdgeColor','m') ;
save_area=new_stats(max_ind);

handles.save_area = save_area;
guidata(hObject,handles);


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%��Ե��ȡ
im1 = handles.im1;
im1 = double(im1);
sobelKernelY = [1 2 1;0 0 0; -1 -2 -1];
sobelKernelX = sobelKernelY';
derivativeX = imfilter(im1, sobelKernelX,'replicate');
derivativeY = imfilter(im1, sobelKernelY,'replicate');
gradientMagnitude = sqrt(derivativeX.^2 + derivativeY.^2);
gradientMagnitude = floor((gradientMagnitude./max(max(gradientMagnitude))).*255);%��һ��
gradientMagnitude=medfilt2(gradientMagnitude);%ȥ��
im2 = gradientMagnitude;
axes(handles.axes1) ;
cla reset;
axes(handles.axes1);
imshow(im2,[]);
handles.im2 = im2;
guidata(hObject,handles);


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%��ʴ�����Ͳ���
im3 = handles.im3;
seType = strel('square',2);
rode_image=imopen(im3,seType);
im4 = rode_image;
axes(handles.axes1) ;
cla reset;
axes(handles.axes1);
imshow(im4,[]);
handles.im4 = im4;
guidata(hObject,handles);

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%��ֵ��
im2 = handles.im2;
threshold_gray = handles.threshold_gray;
white_location=find(im2>threshold_gray);
black_location=find(im2<threshold_gray);
gray_image=im2;
gray_image(white_location)=1;
gray_image(black_location)=0;
axes(handles.axes1);
imshow(gray_image,[]);
handles.im3 = gray_image;
guidata(hObject,handles);


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%�ָ�
save_area = handles.save_area;
im = handles.im;
%�ָ�����ȡ�Ķ�ά��ͼ��
img = imcrop(im,save_area.BoundingBox);
img = uint8(imresize(img,[500 500],'bilinear'));
axes(handles.axes1) ;
cla reset;
axes(handles.axes1) ;
imshow(img,[]);
handles.img = img;
guidata(hObject,handles);


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%У��
img = handles.img;
img = imadjust(img);
img = im2uint8(img);
[M,N] = size(img);
dot=ginput();       %ȡ�ĸ��㣬���������ϣ����ϣ����£�����,������ȡ��������ĸ���
w=round(sqrt((dot(1,1)-dot(2,1))^2+(dot(1,2)-dot(2,2))^2));     %��ԭ�ı��λ���¾��ο�
h=round(sqrt((dot(1,1)-dot(3,1))^2+(dot(1,2)-dot(3,2))^2));     %��ԭ�ı��λ���¾��θ�

y=[dot(1,1) dot(2,1) dot(3,1) dot(4,1)];        %�ĸ�ԭ����
x=[dot(1,2) dot(2,2) dot(3,2) dot(4,2)];

%�������µĶ��㣬��ȡ�ľ���,Ҳ����������������״
%�����ԭͼ���Ǿ��Σ���ͼ���Ǵ�dot��ȡ�õĵ���ɵ������ı���.:)
Y=[dot(1,1) dot(1,1) dot(1,1)+h dot(1,1)+h];     
X=[dot(1,2) dot(1,2)+w dot(1,2) dot(1,2)+w];

B=[X(1) Y(1) X(2) Y(2) X(3) Y(3) X(4) Y(4)]';   %�任����ĸ����㣬�����ұߵ�ֵ
%�����ⷽ���飬���̵�ϵ��
A=[x(1) y(1) 1 0 0 0 -X(1)*x(1) -X(1)*y(1);             
   0 0 0 x(1) y(1) 1 -Y(1)*x(1) -Y(1)*y(1);
   x(2) y(2) 1 0 0 0 -X(2)*x(2) -X(2)*y(2);
   0 0 0 x(2) y(2) 1 -Y(2)*x(2) -Y(2)*y(2);
   x(3) y(3) 1 0 0 0 -X(3)*x(3) -X(3)*y(3);
   0 0 0 x(3) y(3) 1 -Y(3)*x(3) -Y(3)*y(3);
   x(4) y(4) 1 0 0 0 -X(4)*x(4) -X(4)*y(4);
   0 0 0 x(4) y(4) 1 -Y(4)*x(4) -Y(4)*y(4)];

fa=inv(A)*B;        %���ĵ���õķ��̵Ľ⣬Ҳ��ȫ�ֱ任ϵ��
a=fa(1);b=fa(2);c=fa(3);
d=fa(4);e=fa(5);f=fa(6);
g=fa(7);h=fa(8);

rot=[d e f;
     a b c;
     g h 1];        %��ʽ�е�һ������x,Matlab��һ����ʾy�������Ҿ���1,2�л�����

pix1=rot*[1 1 1]'/(g*1+h*1+1);  %�任��ͼ�����ϵ�
pix2=rot*[1 N 1]'/(g*1+h*N+1);  %�任��ͼ�����ϵ�
pix3=rot*[M 1 1]'/(g*M+h*1+1);  %�任��ͼ�����µ�
pix4=rot*[M N 1]'/(g*M+h*N+1);  %�任��ͼ�����µ�

height=round(max([pix1(1) pix2(1) pix3(1) pix4(1)])-min([pix1(1) pix2(1) pix3(1) pix4(1)]));     %�任��ͼ��ĸ߶�
width=round(max([pix1(2) pix2(2) pix3(2) pix4(2)])-min([pix1(2) pix2(2) pix3(2) pix4(2)]));      %�任��ͼ��Ŀ��

imgn=255*ones(height,width);

delta_y=round(abs(min([pix1(1) pix2(1) pix3(1) pix4(1)])));            %ȡ��y����ĸ��ᳬ����ƫ����
delta_x=round(abs(min([pix1(2) pix2(2) pix3(2) pix4(2)])));            %ȡ��x����ĸ��ᳬ����ƫ����
inv_rot=inv(rot);

for i = 1-delta_y:height-delta_y                        %�ӱ任ͼ���з���Ѱ��ԭͼ��ĵ㣬������ֿն�������ת�Ŵ�ԭ��һ��
    for j = 1-delta_x:width-delta_x
        pix=inv_rot*[i j 1]';       %��ԭͼ�������꣬��Ϊ[YW XW W]=fa*[y x 1],�������������[YW XW W],W=gy+hx+1;
        pix=inv([g*pix(1)-1 h*pix(1);g*pix(2) h*pix(2)-1])*[-pix(1) -pix(2)]'; %�൱�ڽ�[pix(1)*(gy+hx+1) pix(2)*(gy+hx+1)]=[y x],����һ�����̣���y��x�����pix=[y x];
        
        if pix(1)>=0.5 && pix(2)>=0.5 && pix(1)<=M && pix(2)<=N
            imgn(i+delta_y,j+delta_x)=img(round(pix(1)),round(pix(2)));     %���ڽ���ֵ,Ҳ������˫���Ի�˫������ֵ
        end  
    end
end
imgn = uint8(imgn);
axes(handles.axes1) ;
cla reset;
axes(handles.axes1);
imshow(imgn,[]);
handles.im5 = imgn;
guidata(hObject,handles);


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
im5 = handles.im5;
%��ֵ��
threshold_gray = handles.threshold_gray;
white_location=find(im5>threshold_gray);
black_location=find(im5<threshold_gray);
gray_image2=im5;
gray_image2(white_location)=1;
gray_image2(black_location)=0;
imgn1 = uint8(gray_image2);
%��ʴ����
seType = strel('square',3);
im6 = imopen(imgn1,seType);
axes(handles.axes1) ;
cla reset;
axes(handles.axes1);
imshow(im6,[]);
handles.im6 = im6;
guidata(hObject,handles);


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%%ͶӰ��λ�ٷָ�
im6 = handles.im6;
im6 = touying(im6);
im6 = uint8(imresize(im6,[500 500],'bilinear'));

axes(handles.axes1) ;
cla reset;
% set(handles.axes1,'position',[300,12,500,500]);
axes(handles.axes1) ;
imshow(im6,[]);
handles.im6 = im6;
guidata(hObject,handles);



% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
value_matrix = handles.value;
im6 = handles.im6;
% [filename,pathname]=...
%     uiputfile({'*.bmp','BMPͼ��(*.bmp)';...
%     '*.jpg','JPGͼ��(*.jpg)';'*.gif','GIFͼ��(*.gif)';...
%     '*.tif','TIFͼ��(*.tif)';'*.png','PNGͼ��(*.png)';...
%     '*.*','ALL FILES(*.*)'},'���Ϊ...');
% im6 = uint8(255*im6);
% imwrite(im6,str);

[pathname filename]=uiputfile({
   '*.mat','mat�ļ�(*.mat)';'*.*','All Files(*.*)'},'choose a File');
str=[pathname filename];
% set(value_matrix,'string',str);
% m = get(handles.displayArea,'String');
% fid = fopen(char(str), 'w');
% fwrite(fid, '', 'integer*4')
% fprintf(fid,'%s',m);
% fclose(fid);
save([filename pathname],'value_matrix');
if isequal([filename pathname],[0,0])
    return;
end

guidata(hObject,handles);



% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
im6 = handles.im6;

%%
%������ֵ
[m,n]=size(im6);
% ��ֱͶӰ
for y=1:n
     Sy(y)=sum(im6(:,y));
end
% ��ˮƽͶӰ
for x=1:m
    Sx(x)=sum(im6(x,:));
end
x = find(Sx<max(Sx)-50);
x1 = min(x);
x2 = max(x);

y = find(Sy<max(Sy)-50);
y1 = min(y);
y2 = max(y);

P1=[x1,y1];%P2=[x1,y2];P3=[x2,y1];P4=[x2,y2];
N=11;
y_width=(y2-y1)/N;
x_width=(x2-x1)/N;
value_matrix=zeros(N,N);
for k=1:N
    for j=1:N
       point_y=floor(y_width*(k-0.5)+P1(2));
       point_x=floor(x_width*(j-0.5)+P1(1));
       value_matrix(j,k)=im6(point_x,point_y);
       hold on 
       plot(point_y,point_x,'.','markersize',15,'color','m');
    end
end
value_matrix
handles.value = value_matrix;
guidata(hObject,handles);


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
im6 = handles.im6;
N = handles.N;
%%
%������ֵ
[m,n]=size(im6);
% ��ֱͶӰ
for y=1:n
     Sy(y)=sum(im6(:,y));
end
% ��ˮƽͶӰ
for x=1:m
    Sx(x)=sum(im6(x,:));
end
x = find(Sx<max(Sx)-50);
x1 = min(x);
x2 = max(x);

y = find(Sy<max(Sy)-50);
y1 = min(y);
y2 = max(y);

P1=[x1,y1];%P2=[x1,y2];P3=[x2,y1];P4=[x2,y2];

y_width=(y2-y1)/N;
x_width=(x2-x1)/N;
value_matrix=zeros(N,N);
for k=1:N
    for j=1:N
       point_y=floor(y_width*(k-0.5)+P1(2));
       point_x=floor(x_width*(j-0.5)+P1(1));
       value_matrix(j,k)=im6(point_x,point_y);
       hold on 
       plot(point_y,point_x,'.','markersize',15,'color','m');
    end
end
value_matrix
handles.value = value_matrix;
guidata(hObject,handles);



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double
N=get(handles.edit3,'string');
N = str2num(N);
handles.N = N;
guidata(hObject,handles);

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
