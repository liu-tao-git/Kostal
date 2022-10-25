/**
 * @example kstar.cpp
 * Select a plan from a list to execute using RDK's plan execution API.
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>
#include <string>
#include <iostream>
#include <cmath>
#include <mutex>
#include <thread>
#include "ControlSPI.h"
#include <stdio.h>
#include <unistd.h>
#include <stack>
#include <array>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <math.h>
#include "math.h"
#include <eigen3/Eigen/Eigen>
using namespace std;


namespace {
unsigned int g_printCounter = 0;
double outdata[20] ={0};
uint8_t SPIout[16] = {0};
int ret;
double M_Pi;
double rad = 180/M_Pi;
Eigen::Quaternion<double> q; // new a quaternion


//stack for data storage
std::stack<std::array<double, 20>> data_stack;
std::stack<std::array<uint8_t, 16>> SPI_stack;
std::stack<std::array<double, 20>> data_stack_rev;
std::stack<std::array<uint8_t, 16>> SPI_stack_rev;




struct PrintData
{
    std::vector<double> tcp_pose;
    std::vector<double> raw_sensor_data;
    std::vector<double> flangePose;

} g_printData;

struct Euler_out
{
    double Roll,Pitch,Yaw;
}g_euler_Out;



std::mutex g_printDataMutex;
std::mutex SPImutex;
}

//test function uint8 to binary 
void toBinary(uint8_t a)
{
    uint8_t i;

    for(i=0x80;i!=0;i>>=1)
        printf("%c",(a&i)?'1':'0'); 
}




//Quaternion to Euler

typedef struct Euler_out Struct;


Struct Quaternion_to_Euler (double w, double x, double y, double z)
{
    q.w() = w;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    auto euler = q.toRotationMatrix().eulerAngles(0,1,2);
    
    
    Struct Eout;
    {
    Eout.Roll = euler[0];
    Eout.Pitch = euler[1];
    Eout.Yaw = euler[2];
    }
    return Eout;
}

// read robot data and push robot data &SPI data to stack
void highPriorityPeriodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot)
{
    robot->getRobotStates(robotStates.get());
    {
        std::lock_guard<std::mutex> lock(g_printDataMutex);
        g_printData.tcp_pose = robotStates->m_tcpPose;
        g_printData.raw_sensor_data = robotStates->m_rawDataFromForceSensor;
        g_printData.flangePose = robotStates->m_flangePose;

    }
    outdata[0]=g_printData.tcp_pose[0];
    outdata[1]=g_printData.tcp_pose[1];
    outdata[2]=g_printData.tcp_pose[2];
    outdata[3]=g_printData.tcp_pose[3];
    outdata[4]=g_printData.tcp_pose[4];
    outdata[5]=g_printData.tcp_pose[5];
    outdata[6]=g_printData.tcp_pose[6];


    outdata[7]=g_printData.raw_sensor_data[0];
    outdata[8]=g_printData.raw_sensor_data[1];
    outdata[9]=g_printData.raw_sensor_data[2];
    outdata[10]=g_printData.raw_sensor_data[3];
    outdata[11]=g_printData.raw_sensor_data[4];
    outdata[12]=g_printData.raw_sensor_data[5];


    outdata[13]=g_printData.flangePose[0];
    outdata[14]=g_printData.flangePose[1];
    outdata[15]=g_printData.flangePose[2];
    outdata[16]=g_printData.flangePose[3];
    outdata[17]=g_printData.flangePose[4];
    outdata[18]=g_printData.flangePose[5];
    outdata[19]=g_printData.flangePose[6];


    data_stack.push({outdata[0],outdata[1],outdata[2],outdata[3],outdata[4],outdata[5],outdata[6],outdata[7],outdata[8],outdata[9],outdata[10],outdata[11],outdata[12],outdata[13],outdata[14],outdata[15],outdata[16],outdata[17],outdata[18],outdata[19]});
    {
    std::lock_guard<std::mutex> lock(SPImutex);
    SPI_stack.push({SPIout[0],SPIout[1],SPIout[2],SPIout[3],SPIout[4],SPIout[5],SPIout[6],SPIout[7],SPIout[8],SPIout[9],SPIout[10],SPIout[11],SPIout[12],SPIout[13],SPIout[14],SPIout[15]});
    }

    
}


void hptask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot)
{
robot->start(std::bind(highPriorityPeriodicTask, robotStates, robot));

}


//Read SPIdata

void SPIdata_collection()  
{
  while (true){
        uint8_t read_buffer[10240] = {0};
        int32_t read_data_num = 0;
        ret = VSI_SlaveReadBytes(VSI_USBSPI, 0, read_buffer, &read_data_num,100);
        if (ret != ERR_SUCCESS){
            printf("Read data error!\n");
            
        }else{
            if (read_data_num == 16){

                {
                std::lock_guard<std::mutex> lock(SPImutex);
                for (int i = 0; i < read_data_num; i++){
                    SPIout[i]=read_buffer[i];
                }
                }
            }else{
            }
             
    }   
}
}

int main(int argc, char* argv[])
{

    flexiv::Log log;
        // IP of the robot server
    std::string robotIP = "192.168.2.100";

    // IP of the workstation PC running this program
    std::string localIP = "192.168.2.104";

    // RDK Initialization
    //=============================================================================
    // instantiate robot interface
    auto robot = std::make_shared<flexiv::Robot>();

    // create data struct for storing robot states
    auto robotStates = std::make_shared<flexiv::RobotStates>();

    // initialize robot interface and connect to the robot server
    robot->init(robotIP, localIP);

    // wait for the connection to be established
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isConnected());

    // enable the robot, make sure the E-stop is released before enabling
    if (robot->enable()) {

        log.info("Enabling robot ...");
    }

    // wait for the robot to become operational
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isOperational());
    log.info("Robot is now operational");


    VSI_INIT_CONFIG SPI_Config;
    // Scan connected device
    ret = VSI_ScanDevice(1);
    if (ret <= 0) {
        printf("No device connect!\n");
        return ret;
    }
    // Open device
    ret = VSI_OpenDevice(VSI_USBSPI, 0, 0);
    if (ret != ERR_SUCCESS) {
        printf("Open device error!\n");
        return ret;
    }

    // Initialize device(Slave Mode, Hardware SPI, Full-Duplex)
    SPI_Config.ControlMode = 0;
    SPI_Config.MasterMode = 0; // Slave Mode
    SPI_Config.CPHA = 1; // Clock Polarity and Phase must be same as master
    SPI_Config.CPOL = 0;
    SPI_Config.LSBFirst = 0;
    SPI_Config.TranBits = 8; // Support 8bit mode only
    SPI_Config.SelPolarity = 0;
    SPI_Config.ClockSpeed = 1395000;
    ret = VSI_InitSPI(VSI_USBSPI, 0, &SPI_Config);
    if (ret != ERR_SUCCESS) {
        printf("Initialize device error!\n");
        return ret;
    }
    printf("SPI Initialize device successfully!\n");
    // initialize robot interface and connect to the robot server
    
  
    do {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_PLAN_EXECUTION);
         
    

    std::thread SPIthread(SPIdata_collection);
    std::thread robthread(std::bind(hptask,robotStates,robot));
    
    {
    std::lock_guard<std::mutex> lock(SPImutex);
    do {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (data_stack_rev.size()!=0);
    }


    robot->executePlanByName(argv[1]);               


    flexiv::SystemStatus systemStatus;
    robot->getSystemStatus(&systemStatus);

    do {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    robot->getSystemStatus(&systemStatus);

    } while (systemStatus.m_programRunning != true);

    do {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    robot->getSystemStatus(&systemStatus);

    } while (systemStatus.m_programRunning == true);

    
    while (!data_stack.empty())            
    {
    data_stack_rev.push({data_stack.top()[0],data_stack.top()[1],data_stack.top()[2],data_stack.top()[3],data_stack.top()[4],data_stack.top()[5],data_stack.top()[6],data_stack.top()[7],data_stack.top()[8],data_stack.top()[9],data_stack.top()[10],data_stack.top()[11],data_stack.top()[12]});
    SPI_stack_rev.push({SPI_stack.top()[0],SPI_stack.top()[1],SPI_stack.top()[2],SPI_stack.top()[3],SPI_stack.top()[4],SPI_stack.top()[5],SPI_stack.top()[6],SPI_stack.top()[7],SPI_stack.top()[8],SPI_stack.top()[9],SPI_stack.top()[10],SPI_stack.top()[11],SPI_stack.top()[12],SPI_stack.top()[13],SPI_stack.top()[14],SPI_stack.top()[15]});
    data_stack.pop();
    SPI_stack.pop();
    }

    //printf("go\n");
    
    std::time_t file_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string time_str = std::ctime(&file_time); 

    std::ofstream MyExcelFile;
    std::string filename="./data/"+std::string(argv[1])+time_str+".csv";
    MyExcelFile.open(filename);
    
    //add file headers  
    MyExcelFile << "TCP_x"<<","<<"TCP_y"<<"," << "TCP_z"<<"," ;
    MyExcelFile << "TCP_Rx"<<","<<"TCP_Ry"<<"," << "TCP_Rz"<<"," ;
    MyExcelFile << "FLANGE_x"<<","<<"FLANGE_y"<<"," << "FLANGE_z"<<"," ;
    MyExcelFile << "FLANGE_Rx"<<","<<"FLANGE_Ry"<<"," << "FLANGR_Rz"<<"," ;
    MyExcelFile << "RowDataSensor0"<<","<< "RowDataSensor1"<<","<< "RowDataSensor2"<<",";
    MyExcelFile << "RowDataSensor3"<<","<< "RowDataSensor4"<<","<< "RowDataSensor5"<<",";
    MyExcelFile << "SPI0-0"<<","<< "SPI0-1"<<","<< "SPI0-2"<<","<< "SPI0-3"<<","<< "SPI0-4"<<",";
    MyExcelFile << "SPI0-5"<<","<< "SPI0-6"<<","<< "SPI0-7"<<",";
    MyExcelFile << "SPI1-0"<<","<< "SPI1-1"<<","<< "SPI1-2"<<","<< "SPI1-3"<<","<< "SPI1-4"<<",";
    MyExcelFile << "SPI1-5"<<","<< "SPI1-6"<<","<< "SPI1-7"<<",";
    MyExcelFile << std::endl; 

    std::cout<< ' ' <<data_stack_rev.size() << std::endl;
    std::cout<< ' ' <<SPI_stack_rev.size() << std::endl;

    while (!data_stack_rev.empty())            
    {

    auto showdata = data_stack_rev.top();           
    uint8_t showspi = SPI_stack_rev.top()[6];           
   
    g_euler_Out = Quaternion_to_Euler(data_stack_rev.top()[3],data_stack_rev.top()[4],data_stack_rev.top()[5],data_stack_rev.top()[6]);
    
    //
    MyExcelFile << data_stack_rev.top()[0]<<",";
    MyExcelFile << data_stack_rev.top()[1]<<",";
    MyExcelFile << data_stack_rev.top()[2]<<",";

    MyExcelFile << g_euler_Out.Roll<<",";
    MyExcelFile << g_euler_Out.Pitch<<",";
    MyExcelFile << g_euler_Out.Yaw<<",";
    
    g_euler_Out = Quaternion_to_Euler(data_stack_rev.top()[16],data_stack_rev.top()[17],data_stack_rev.top()[18],data_stack_rev.top()[19]);

    MyExcelFile << data_stack_rev.top()[13]<<",";
    MyExcelFile << data_stack_rev.top()[14]<<",";
    MyExcelFile << data_stack_rev.top()[15]<<",";
    
    MyExcelFile << g_euler_Out.Roll<<",";
    MyExcelFile << g_euler_Out.Pitch<<",";
    MyExcelFile << g_euler_Out.Yaw<<",";
    //write sensor data
    for (auto i = 7; i < 13; i++){
    MyExcelFile << data_stack_rev.top()[i]<<",";
    }
    
    //Write SPIdata
    for (auto i = 0; i < 16; i++){
    MyExcelFile << std::setfill('0') << std::setw(2) << std::right<<std::hex ;
    MyExcelFile << + static_cast<uint8_t>(SPI_stack_rev.top()[i])<<",";
    }
    MyExcelFile << std::endl; 


    //check syncsats
    printf("%02X ",showspi);
    std::cout<< ' ' <<showdata[0]<< std::endl;
    data_stack_rev.pop();
    SPI_stack_rev.pop();


    }

    MyExcelFile.close();
    
    return 0;
}
