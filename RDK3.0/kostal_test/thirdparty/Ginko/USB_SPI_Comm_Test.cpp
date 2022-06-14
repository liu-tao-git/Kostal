#ifndef OS_UNIX  
#include "stdafx.h"
#endif
#include "ControlSPI.h"
#include <stdio.h>
#include <iostream>
#include <unistd.h>
using namespace std;
int main(int argc, char* argv[])
{
    int ret;
	VSI_INIT_CONFIG SPI_Config;
    // Scan connected device 
    ret = VSI_ScanDevice(1);
    if (ret <= 0){
        printf("No device connect!\n");
        return ret;
    }
    // Open device
    ret = VSI_OpenDevice(VSI_USBSPI, 0, 0);
    if (ret != ERR_SUCCESS){
        printf("Open device error!\n");
        return ret;
    }
    // Initialize device(Slave Mode, Hardware SPI, Full-Duplex)
    SPI_Config.ControlMode = 0; 
    SPI_Config.MasterMode = 0;  // Slave Mode
    SPI_Config.CPHA = 1;        // Clock Polarity and Phase must be same as master
    SPI_Config.CPOL = 0;
    SPI_Config.LSBFirst = 0;
    SPI_Config.TranBits = 8;    // Support 8bit mode only
	SPI_Config.SelPolarity = 0;
    SPI_Config.ClockSpeed = 1395000;
    ret = VSI_InitSPI(VSI_USBSPI, 0, &SPI_Config);
    if (ret != ERR_SUCCESS){
        printf("Initialize device error!\n");
        return ret;
    }
    printf("Initialize device successfully!\n");
    while (true){
        uint8_t read_buffer[10240] = {0};
        int32_t read_data_num = 0;
        ret = VSI_SlaveReadBytes(VSI_USBSPI, 0, read_buffer, &read_data_num,100);
        if (ret != ERR_SUCCESS){
            printf("Read data error!\n");
            return ret;
        }else{
            if (read_data_num > 0){
                //printf("Read data num: %d\n",read_data_num);
                printf("Read data(Hex):\n");
                for (int i = 0; i < read_data_num; i++){
                    printf("%02X ",read_buffer[i]);
                }
            }else{
                //printf("No data!\n");
            }
        }
    }
    return 0;
}
