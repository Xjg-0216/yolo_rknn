sudo chmod 777 /dev/ttyUSB0

rm *.cxx -rf
rm *~ -rf
#I=/usr/include/python3.5

# swig -c++ -python siyia2.i
#g++ -c -fpic  siyia2.cpp  siyia2_wrap.cxx -I$I
#g++ -shared siyia2.o siyia2_wrap.o -o _siyia2.so
g++ -fPIC -shared xianfei.cpp tty.cpp -o xianfei.so
echo "恭喜，compile finish........"
