#import siyia2;
import os,sys,time;

from ctypes import *;

#lib=CDLL("./xianfei.so");  #或者 
lib = cdll.LoadLibrary("./xianfei.so");

#lib.setCenter(); #居中
time.sleep(1);

#lib.setAngle(c_float(-22.1), c_float(1.5), c_float(2.5)); #设置角度yaw,roll, pitch


#获取云台姿态
def getAngle():
   m_str = create_string_buffer(b"abc1abc1abc1abc1abc11111111111111111111");
   lib.getGimbal(m_str); #返回值为yaw,roll,pitch

   sinfo=m_str.value.decode('gbk');
   #ary=sinfo.split(',');
   #print('buf=', ary);  #time.sleep(2);
   return float(sinfo);
#---------------------------------------------------------------------------------
print('init...  ', getAngle());
#sys.exit();
#iYaw=2.5;
#print('init...  ', getAngle());
while (True):
    lib.setAngle(c_float(45));
    time.sleep(1);
    print('当前 pitch =', getAngle());
    time.sleep(2);
    
    lib.setAngle(c_float(0));
    time.sleep(1);
    print('当前 pitch =', getAngle());
    time.sleep(1);
        

#mySerial.setPitch(11.13);
#time.sleep(3)
#mySerial.setRoll(11.14);

