# gr-m2k

GNU Radio blocks for ADALM-2000. More information available at https://wiki.analog.com/university/tools/m2k/libm2k/gr-m2k.

## Dependencies
 - gnuradio 3.8  
   For more information visit: https://wiki.gnuradio.org/index.php/InstallingGR
 - libm2k  
   For more information visit: https://github.com/analogdevicesinc/libm2k  
   When using ADI's chips with gr-m2k, make sure to enable tools option of libm2k

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
