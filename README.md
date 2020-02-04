# gr-m2k

GNU Radio blocks for ADALM-2000

## How to build it?

git clone https://github.com/analogdevicesinc/gr-m2k.git  
cd gr-m2k  
mkdir build && cd build  
cmake ..  
make  
sudo make install  
cd ../..  
sudo ldconfig  

## Test gr-m2k

make test  

Make sure to have the following hardware configuration:  
1+ ---> W1  
2+ ---> W2
