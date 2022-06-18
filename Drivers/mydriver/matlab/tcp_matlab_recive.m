address = '192.168.3.11';
port = 8080;

ts = tcpip(address,port,'NetworkRole','server');
ts.InputBuffersize=512;
fopen(ts);
try_times=1000;
for i=1:try_times
    pause(0.005);%每次读取之前等待0.02s，随意设置
    try     %因为fread（）在缓冲区没有数据的时候读取会报错，因此用try—catch语句忽略这种错误，直到读取到数据。
        dataReceive=fread(ts, ts.BytesAvailable);
    catch
       gsz_ValuesReceived = ts.ValuesReceived
       %查看读取出的数据数量，如果没有读到，返回0；
    end
end