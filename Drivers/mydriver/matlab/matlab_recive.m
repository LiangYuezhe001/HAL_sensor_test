% 释放串口资源
delete(instrfindall);
% 创建串口资源
s = serialport("COM10",9600);
configureTerminator(s,"CR/LF");
flush(s);
s.UserData = struct("x",[],"y",[],"z",[],"Count",1,"i",[]);

configureCallback(s,"terminator",@readSineWaveData);
%delete(s);
%pause(5); 
function readSineWaveData(s,~)
data = readline(s)
tem = str2double(data);
if(tem==250)
    s.UserData.i=0;
    s.UserData.Count = s.UserData.Count + 1;
    plot3(s.UserData.x,s.UserData.y,s.UserData.z)
end

switch(s.UserData.i)
    case 1 
        s.UserData.x(end+1)=tem;
    case 2 
       s.UserData.y(end+1)=tem;
    case 3 
        s.UserData.z(end+1)=tem;
end
s.UserData.i=s.UserData.i+1;

if s.UserData.Count > 200
    configureCallback(s, "off");   
    %mesh(s.UserData.x,s.UserData.y,s.UserData.z)
   % plot(s.UserData.Data(2:end));
   delete(s);
end
end