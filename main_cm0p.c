/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "stdio.h"
#include <string.h>

double cnt = 0;
double oldCnt = 200;
double speed = 0;
uint32 cntPrev = 0;
char cnt_string[20];
static uint8 data[1] = {0};
uint8 oldData = 0;  //tracks what the last BLE write value was




#include "ssd1306.h"
#define DISPLAY_ADDRESS 0x3C

char buffer[100];
char buffer2[100];


    const unsigned char cylogo [] = {
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xF0, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xC0, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFE, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFC, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFC, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xF0, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xF0, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xE0, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xE0, 0x03, 0xE0, 0x00, 0x03, 0xFC, 0x1C, 0x38, 0x40, 0x78, 0x07, 0x80, 0x0E, 0x07, 0xE0, 0x63,
0xC0, 0x1F, 0xFE, 0x00, 0x01, 0xF0, 0x0C, 0x30, 0x40, 0x18, 0x01, 0x80, 0x0C, 0x03, 0xC0, 0x03,
0xC0, 0x7F, 0xFE, 0x00, 0x01, 0xE0, 0x04, 0x10, 0x40, 0x08, 0x01, 0x80, 0x08, 0x01, 0x80, 0x03,
0x80, 0x7F, 0xFF, 0xFF, 0xE1, 0xE0, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x80, 0x08, 0x00, 0x80, 0x07,
0x87, 0xFF, 0xFF, 0xFF, 0xFF, 0xE1, 0x82, 0x00, 0xC2, 0x08, 0x70, 0x83, 0xF8, 0x60, 0x86, 0x0F,
0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xE1, 0x83, 0x01, 0xC3, 0x08, 0x70, 0x83, 0xF8, 0x60, 0x86, 0x0F,
0x83, 0xFF, 0xFF, 0xFF, 0xF0, 0xE1, 0x83, 0x01, 0xC3, 0x08, 0x70, 0x83, 0xF8, 0x20, 0x82, 0x0F,
0x80, 0x7F, 0xFF, 0xFF, 0xF0, 0xE1, 0xC3, 0x01, 0xC3, 0x08, 0x70, 0x83, 0xF8, 0x1F, 0x81, 0xFF,
0x80, 0x1E, 0x07, 0xFF, 0x80, 0xE1, 0xFF, 0x03, 0xC2, 0x08, 0x00, 0x80, 0x18, 0x0F, 0x80, 0xFF,
0x80, 0x00, 0x07, 0xFE, 0x00, 0xE1, 0xFF, 0x83, 0xC0, 0x08, 0x00, 0x80, 0x1C, 0x07, 0xC0, 0x7F,
0x80, 0x00, 0x00, 0x0E, 0x1F, 0xE1, 0xFF, 0x83, 0xC0, 0x18, 0x01, 0x80, 0x1F, 0x03, 0xF0, 0x3F,
0x80, 0x00, 0x00, 0x00, 0x00, 0xE1, 0xFF, 0x83, 0xC0, 0x18, 0x01, 0x80, 0x1F, 0x01, 0xF0, 0x1F,
0x80, 0x00, 0x00, 0x00, 0x00, 0xE1, 0x83, 0x83, 0xC0, 0x78, 0x41, 0x83, 0xF8, 0x40, 0x84, 0x1F,
0x80, 0x00, 0x00, 0x20, 0x00, 0xE1, 0x83, 0x83, 0xC3, 0xF8, 0x41, 0x83, 0xF8, 0x60, 0x86, 0x0F,
0x80, 0x00, 0x00, 0xC0, 0x00, 0xE1, 0x83, 0x83, 0xC3, 0xF8, 0x61, 0x83, 0xF8, 0x60, 0x86, 0x0F,
0x80, 0x00, 0x01, 0x80, 0x00, 0xE1, 0x83, 0x83, 0xC3, 0xF8, 0x61, 0x83, 0xF8, 0x60, 0x86, 0x0F,
0x80, 0x00, 0x01, 0x00, 0x01, 0xE0, 0x03, 0x83, 0xC3, 0xF8, 0x60, 0x80, 0x08, 0x00, 0x80, 0x1F,
0xC0, 0x00, 0x02, 0x00, 0x01, 0xF0, 0x07, 0x83, 0xC3, 0xF8, 0x70, 0x80, 0x0C, 0x01, 0x80, 0x1F,
0xC0, 0x00, 0x04, 0x00, 0x01, 0xF0, 0x0F, 0x83, 0xC3, 0xF8, 0x70, 0x80, 0x0C, 0x03, 0xC0, 0x3F,
0xE0, 0x00, 0x08, 0x00, 0x03, 0xFC, 0x1F, 0x87, 0xC7, 0xF8, 0x70, 0x80, 0x0E, 0x07, 0xE0, 0x7F,
0xE0, 0x00, 0x08, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xF0, 0x00, 0x7F, 0xFF, 0xFF, 0xE0, 0x40, 0x08, 0x88, 0x0E, 0x16, 0x12, 0x48, 0x88, 0xC0, 0x01,
0xF0, 0x00, 0x00, 0x00, 0x1F, 0xE0, 0x00, 0x08, 0x00, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x01,
0xFC, 0x00, 0x00, 0x00, 0x1F, 0xE0, 0x00, 0x08, 0x00, 0x06, 0x07, 0x20, 0x00, 0x00, 0x00, 0x01,
0xFC, 0x00, 0x00, 0x00, 0x3F, 0xE0, 0x00, 0x08, 0x00, 0x06, 0x07, 0x20, 0x00, 0x00, 0x00, 0x1F,
0xFE, 0x00, 0x00, 0x00, 0x7F, 0xE0, 0x00, 0x08, 0x00, 0x06, 0x07, 0x20, 0x00, 0x00, 0x04, 0x3F,
0xFF, 0x80, 0x00, 0x00, 0xFF, 0xE0, 0x40, 0x08, 0x80, 0x06, 0x07, 0x20, 0x48, 0x80, 0x04, 0x3F,
0xFF, 0xE0, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xF0, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};





static cy_stc_scb_i2c_master_xfer_config_t register_setting;

static uint8 rbuff[2]; //Read Buffer
static uint8 wbuff[2]; //Write Buffer


static void WaitForOperation() // function that check to make sure either a write or a read function has completed
{
    while (0 != (SensorBus_MasterGetStatus() & CY_SCB_I2C_MASTER_BUSY)){}
    {
        CyDelayUs(1); //Keep Writing
    }
}

//to write, you want to write 2 values : (1) The register address that you want to write to
//                                       (2) The value you wish to write to the register

static void WriteRegister(uint8 reg_addr, uint8 data)
{
    wbuff[0] = reg_addr; //Assign the first element to be the register you want to write to (Parameter 1) "register address"
    wbuff[1] = data;// Assign the second element to be the value you wish to write to the register (Paramter 2)
    
    register_setting.buffer = wbuff;
    register_setting.bufferSize = 2;
    register_setting.xferPending = false;
    
    SensorBus_MasterWrite(&register_setting);
    WaitForOperation();
}

//to read you must  (1) Write the register address you want to read from to the slave address device 
//                  (2) use MasterReadBuf to store what is contained in the register into the read buffer

static uint8 ReadRegister(uint8 reg_addr)
{
    wbuff[0] = reg_addr;    //Buffer that will contain the register that will be read from
    
    register_setting.buffer = wbuff;    //Buffer that will contain the register that will be read from
    register_setting.bufferSize = 1;
    register_setting.xferPending = true;
    
    SensorBus_MasterWrite(&register_setting);
    WaitForOperation();
    
    register_setting.buffer = rbuff; //Buffer that will store the value read from the register
    register_setting.xferPending = false;
    
    SensorBus_MasterRead(&register_setting);
    WaitForOperation();
    
    return rbuff[0];    //return what was read and stored in the read buffer
    
}

void genericEventHandler(uint32_t event, void *eventParameter)
{
    switch(event)
    {
        case CY_BLE_EVT_STACK_ON:
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
        {
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            break;
        }
        
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {
            break; //could be turining on LED or run code
        }
        case CY_BLE_EVT_GATTS_WRITE_CMD_REQ:
        {
            cy_stc_ble_gatts_write_cmd_req_param_t *writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t *) eventParameter;
            
            if(CY_BLE_DEVICE_INTERFACE_DEVICE_INBOUND_CHAR_HANDLE == writeReqParameter->handleValPair.attrHandle)
            {
                data[0] = writeReqParameter->handleValPair.value.val[0];
                Cy_BLE_GATTS_WriteRsp(writeReqParameter->connHandle);
                
            }
            break;
        }
    }
}

void bleInterruptNotify()
{
    Cy_BLE_ProcessEvents(); //called any time bluetooth needs processer to do something
}

void SysTick_Handler(void)
{
        cnt = 2500 - QuadDec_GetCounter();
        QuadDec_SetCounter(2500);
        
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);
    

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    SensorBus_Start();
    register_setting.slaveAddress = 0x64;   //0x64 is default slave address motor driver


    I2COLED_Start();
    QuadDec_Start();
    UART_Start();
    setvbuf(stdin,NULL,_IONBF,0);
    
    

    
    
    //Starte BLE block, wait for bluetooth to start up, finally register callback function
    Cy_BLE_Start(genericEventHandler);
    
    while(Cy_BLE_GetState() != CY_BLE_STATE_ON)
    {
        Cy_BLE_ProcessEvents();
    }
    Cy_BLE_RegisterAppHostCallback(bleInterruptNotify);
    
    display_init(DISPLAY_ADDRESS);
    display_clear();
       
    gfx_drawBitmap(0,0,cylogo,128,64,WHITE,BLACK);
    display_update();
    CyDelay(2000);
    display_clear();
    display_update();
        
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_TIMER, 10000000); //0.1MHz Clock, Timer as source //Steps in 100 mys
    Cy_SysTick_SetCallback(0,SysTick_Handler);
    Cy_SysTick_Enable();
    QuadDec_SetCounter(2500);
    
    for(;;)
    {
        /* Place your application code here. */
        cy_stc_ble_gatt_handle_value_pair_t serviceHandle;
        cy_stc_ble_gatt_value_t serviceData;
        
        serviceData.val = (uint8*)data;
        serviceData.len = 1;
        
        serviceHandle.attrHandle = CY_BLE_DEVICE_INTERFACE_DEVICE_OUTBOUND_CHAR_HANDLE;
        serviceHandle.value = serviceData;
        
        Cy_BLE_GATTS_WriteAttributeValueLocal(&serviceHandle);
        
        /* Table for BLE inputs and motor outputs
        BLE input  |        Action          |   Register hex value
           0x00    |    Neutral/Stop        |       0x34
           0x01    |      F @ 1.04V         |       0x35
           0x02    |      F @ 2.01V         |       0x65
           0x03    |      F @ 2.97V         |       0x95
           0x04    |      F @ 4.02V         |       0xC9
           0x05    |      R @ 1.04V         |       0x36
           0x06    |      R @ 2.01V         |       0x66
           0x07    |      R @ 2.97V         |       0x96
           0x08    |      R @ 4.02V         |       0xCA
        
        */
        
        
        //if the user has entered a different input
        if (oldData != data[0]) {
            //now do case statements 
            switch(data[0])
            {
                //Stop 
                case 0x00:
                WriteRegister(0x00,0x34);
                
                oldData = data[0];
                break;
                
                //Forward @ 1.04V
                case 0x01:
                WriteRegister(0x00,0x35);

                oldData = data[0];
                break;
                
                //Forward @ 2.01V
                case 0x02:
                WriteRegister(0x00,0x65);
                
                oldData = data[0];
                break;
                
                //Forward @ 2.97V
                case 0x03:
                WriteRegister(0x00,0x95);
                
                oldData = data[0];
                break;
                
                //Forward @ 4.02V
                case 0x04:
                WriteRegister(0x00,0xC9);
                oldData = data[0];
                break;
                
                //Reverse @ 1.04V
                case 0x05:
                WriteRegister(0x00,0x36);
                
                oldData = data[0];
                break;
                
                //Reverse @ 2.01V
                case 0x06:
                WriteRegister(0x00,0x66);
               
                oldData = data[0];
                break;
                
                //Reverse @ 2.97V
                case 0x07:
                WriteRegister(0x00,0x96);
                oldData = data[0];
                break;
                
                //Reverse @ 4.02V
                case 0x08:
                WriteRegister(0x00,0xCA);
 
                oldData = data[0];
                break;
                

      
            }
        }
        
        if (oldCnt+5 < cnt || oldCnt-5 > cnt) {
            speed = (0.0036*cnt) + 0.2472;
            if(speed <= 0.2472)
            {
                speed = 0;
            }
            
            gcvt(speed, 5, cnt_string);
            display_clear();
            display_update();
            gfx_setTextSize(3);
            gfx_setTextColor(WHITE);
            gfx_setCursor(5,17); 
            sprintf(buffer, cnt_string);
            gfx_println(buffer);
            gfx_setCursor(5,40);
            sprintf(buffer2, "mph");
            gfx_println(buffer2);
            display_update();
            oldCnt = cnt;
        }
        
        
        
    }

   
}

/* [] END OF FILE */