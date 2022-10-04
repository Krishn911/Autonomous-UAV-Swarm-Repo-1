#!/usr/bin/python
# -*- coding:utf-8 -*-
import time
import smbus
import numpy as np
ID = 0
TOF_I2C_ADDRESS  = 0x08 + ID     #7-bit slave address=ID+0x08  7位从机地址=ID+0x08

#ADDR_WRITE_SLAVE (uint8_t)((TOF_I2C_ADDRESS<<1)|0x00)   #8-bit slave write address     8位从机写地址
#ADDR_READ_SLAVE (uint8_t)((TOF_I2C_ADDRESS<<1)|0x01)    #8-bit slave read address      8位从机读地址


TOF_ADDR_MODE            = 0x0c   #Mode variable address    模式变量地址
TOF_SIZE_MODE            = 1      #The number of bytes occupied by the pattern variable     模式变量所占的字节数

TOF_ADDR_ID              = 0x0d   #ID variable address      ID变量地址
TOF_SIZE_ID              = 1      #Number of bytes occupied by the ID variable      ID变量所占的字节数

TOF_ADDR_UART_BAUDRATE   = 0x10   #UART baud rate variable address      UART波特率变量地址
TOF_SIZE_UART_BAUDRATE   = 4      #The number of bytes occupied by the UART baud rate variable      UART波特率变量所占的字节数

TOF_ADDR_SYSTEM_TIME     = 0x20   #System time variable address     系统时间变量地址
TOF_SIZE_SYSTEM_TIME     = 4      #The number of bytes occupied by the system time variable     系统时间变量所占的字节数

TOF_ADDR_DIS             = 0x24   #distance variable address        距离变量地址
TOF_SIZE_DIS             = 4      #The number of bytes occupied by the distance variable        距离变量所占的字节数

TOF_ADDR_DIS_STATUS      = 0x28   #Distance status indication variable address      距离状态指示变量地址
TOF_SIZE_DIS_STATUS      = 2      #The number of bytes occupied by the distance status indicator variable       距离状态指示变量所占的字节数

TOF_ADDR_SIGNAL_STRENGTH = 0x2a   #Signal Strength Variable Address     信号强度变量地址
TOF_SIZE_SIGNAL_STRENGTH = 2      #The number of bytes occupied by the signal strength variable     信号强度变量所占的字节数

TOF_ADDR_RANGE_PRECISION = 0x2c   #Ranging precision variable address       测距精度变量地址
TOF_SIZE_RANGE_PRECISION = 1      #The number of bytes occupied by the ranging precision variable       测距精度变量所占的字节数


TOF_IIC_TO_UART_DATA  = 0x40   #Change the communication mode to the byte data that UART needs to send   将通信模式改为UART需要发送的字节数据

class TOF_Sense_B(object):
    def __init__(self,address=TOF_I2C_ADDRESS):
        self._address = address
        self._bus = smbus.SMBus(1)

    def TOF_write_byte(self,address,cmd,val):
        self._bus.write_byte_data(address,cmd,val)

    def TOF_read_byte(self,address,cmd):
        return self._bus.read_byte_data(address,cmd)

    def TOF_read_block(self,address, reg, length=1):
        return self._bus.read_i2c_block_data(address, reg, length) 
    
    def TOF_write_block(self,address, reg, vals):
        return self._bus.write_i2c_block_data(address, reg, vals)
    
    def TOF_version(self):
        version = self.TOF_read_block(self._address,0x00,2)
        TOF_version = version[1] << 8 | version[0]
        return TOF_version

    def TOF_Read_ID(self):
        TOF_ID = self.TOF_read_byte(self._address,TOF_ADDR_ID)
        return TOF_ID

    def TOF_Read_Systime(self):
        Systime = self.TOF_read_block(self._address,TOF_ADDR_SYSTEM_TIME,TOF_SIZE_SYSTEM_TIME)
        TOF_Systime = Systime[3] << 24 | Systime[2] << 16 | Systime[1] << 8 | Systime[0]
        return TOF_Systime
    
    def TOF_Read_Mode(self):
        return self.TOF_read_byte(self._address,TOF_ADDR_MODE)
    
    def TOF_Read_Distance(self):
        Dis = self.TOF_read_block(self._address,TOF_ADDR_DIS,TOF_SIZE_DIS)
        TOF_Dis = Dis[3] << 24 | Dis[2] << 16 | Dis[1] << 8 | Dis[0]
        return TOF_Dis
    
    def TOF_Dis_status(self):
        status = self.TOF_read_block(self._address,TOF_ADDR_DIS_STATUS,TOF_SIZE_DIS_STATUS)
        TOF_status = status[1] << 8 | status[0]
        return TOF_status

    def TOF_Signal_strength(self):
        strength = self.TOF_read_block(self._address,TOF_ADDR_SIGNAL_STRENGTH,TOF_SIZE_SIGNAL_STRENGTH)
        TOF_strength = strength[1] << 8 | strength[0]
        return TOF_strength

    def TOF_Range_precision(self):
        return self.TOF_read_byte(self._address,TOF_ADDR_RANGE_PRECISION)

    def TOF_convert(self,int32_val): #Convert a 32-bit integer to 4 arrays of 8-bit integers     将32位整型转换成4个8位整数数组
        bin = np.binary_repr(int32_val, width = 32) 
        int8_arr = [int(bin[24:32],2),int(bin[16:24],2), 
                    int(bin[8:16],2),int(bin[0:8],2) ]
        return int8_arr 

    def TOF_Set_Uart_Baudrate(self,Baudrate):
#Uart_Baudrate:115200,230400,460800,921600,1000000,1200000, 1500000,2000000,3000000
#UART波特率:115200,230400,460800,921600,1000000,1200000, 1500000,2000000,3000000
        self.TOF_write_block(self._address,TOF_ADDR_UART_BAUDRATE,self.TOF_convert(Baudrate))
    
    def TOF_Read_Uart_Baudrate(self):
        Baudrate = self.TOF_read_block(self._address,TOF_ADDR_UART_BAUDRATE,TOF_SIZE_UART_BAUDRATE)
        TOF_Baudrate = Baudrate[3] << 24 | Baudrate[2] << 16 | Baudrate[1] << 8 | Baudrate[0]
        print(TOF_Baudrate)
    
    def TOF_IIC_TO_UART(self):
        self.TOF_Set_Uart_Baudrate(921600)
        self.TOF_write_byte(self._address,TOF_ADDR_MODE,TOF_IIC_TO_UART_DATA)
        print("Serial mode")
                            

if __name__ == "__main__":
    TOF = TOF_Sense_B()
    # TOF.TOF_IIC_TO_UART() #Switch to UART mode   切换到UART模式
    while True:
        try:      
            if TOF.TOF_version() == 0x06:
                print("TOF data is ok!")
                print("TOF id is: "+str(TOF.TOF_Read_ID()))
                print("TOF system time is: "+str(TOF.TOF_Read_Systime())+' ms      ')
                print("TOF distance is: "+str(TOF.TOF_Read_Distance())+'mm       ')
                print("TOF status is: "+str(TOF.TOF_Dis_status()))
                print("TOF signal is: "+str(TOF.TOF_Signal_strength()))
                print("TOF precision is: "+str(TOF.TOF_Range_precision())+'cm           ')
                print("\33[8A")
                time.sleep(0.1)
            else:
                print("TOF data is error!")
                break
        except(KeyboardInterrupt):
            print("\n\n\n\n\n\n") 
            break 
    
