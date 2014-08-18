///****************************************************************************
// *
// *   Copyright (C) 2013 Fortiss An-Institut TU Munchen All rights reserved.
// *   Author: Thomas Boehm <thomas.boehm@fortiss.org>
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// *
// * 1. Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// * 2. Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in
// *    the documentation and/or other materials provided with the
// *    distribution.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// * POSSIBILITY OF SUCH DAMAGE.
// *
// ****************************************************************************/

#include "i2c.h"
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include "led.h"
#include "i2c_frame.h"
#include "gyro.h"
#include "sonar.h"
#include "main.h"

static char offset = 0;
uint8_t dataRX=0;
uint8_t txData[2][I2C_FRAME_SIZE+I2C_INTEGRAL_FRAME_SIZE];
uint8_t publishedIndex = 0;
uint8_t readout_done_frame1 = 0;
uint8_t readout_done_frame2 = 0;
uint8_t stop_accumulation = 0;


void i2c_init()
{

    I2C_DeInit(I2C1 );       //Deinit and reset the I2C to avoid it locking up
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);

    GPIO_InitTypeDef gpio_init;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    gpio_init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    //Pull up resistor
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    //Open Drain
    gpio_init.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(GPIOB, &gpio_init);

    GPIO_InitTypeDef gpio_init2;

    gpio_init2.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio_init2.GPIO_Mode = GPIO_Mode_IN;
    gpio_init2.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init2.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio_init2);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1 ); // SCL
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1 ); // SDA

    NVIC_InitTypeDef NVIC_InitStructure, NVIC_InitStructure2;

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure2.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure2.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure2.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure2);

    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
    I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);

    I2C_InitTypeDef i2c_init;
    i2c_init.I2C_ClockSpeed = 100000;
    i2c_init.I2C_Mode = I2C_Mode_I2C;
    i2c_init.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c_init.I2C_OwnAddress1 = i2c_get_ownaddress1();
    i2c_init.I2C_Ack = I2C_Ack_Enable;
    i2c_init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c_init);

    I2C_StretchClockCmd(I2C1, ENABLE);
    I2C_Cmd(I2C1, ENABLE);
}

void I2C1_EV_IRQHandler(void)
{

    //uint8_t dataRX;
    static uint8_t txDataIndex = 0x00;
    static uint8_t rxDataIndex = 0x00;
    switch (I2C_GetLastEvent(I2C1 ))
    {

    case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED :
    {
        I2C1 ->SR1;
        I2C1 ->SR2;
        rxDataIndex = 0;
        break;
    }
    case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED :
    {
        I2C1 ->SR1;
        I2C1 ->SR2;
        break;
    }
    case I2C_EVENT_SLAVE_BYTE_RECEIVED :
    {
    	//receive address offset
        dataRX = I2C_ReceiveData(I2C1 );
        rxDataIndex++;
        //reset Index and indicate sending
        txDataIndex = 0;
        readout_done_frame1 = 0;
        readout_done_frame2 = 0;
        break;
    }
    case I2C_EVENT_SLAVE_BYTE_TRANSMITTING :
    case I2C_EVENT_SLAVE_BYTE_TRANSMITTED :
    {
    	//todo check overrun condition
        I2C_SendData(I2C1, txData[publishedIndex][dataRX+txDataIndex]);
        txDataIndex++;

        //check whether last byte is read what indicates readout of integral frame and force reset of accumulation
        if(dataRX+txDataIndex>=(I2C_FRAME_SIZE)){
        	readout_done_frame1=1;
        }

        if(dataRX+txDataIndex>=(I2C_FRAME_SIZE+I2C_INTEGRAL_FRAME_SIZE)){
        	readout_done_frame2=1;
        }

        break;
    }

    case I2C_EVENT_SLAVE_ACK_FAILURE :
    {
        I2C1 ->SR1 &= 0x00FF;
        break;
    }

    case I2C_EVENT_SLAVE_STOP_DETECTED :
    {
        I2C1 ->SR1;
        I2C1 ->CR1 |= 0x1;
        break;
    }
    }
}

void I2C1_ER_IRQHandler(void)
{

    /* Read SR1 register to get I2C error */
    if ((I2C_ReadRegister(I2C1, I2C_Register_SR1 ) & 0xFF00) != 0x00)
    {
        /* Clears error flags */
        I2C1 ->SR1 &= 0x00FF;
    }
}

void update_TX_buffer(float pixel_flow_x_sum, float pixel_flow_y_sum, float flow_comp_m_x, float flow_comp_m_y, uint16_t qual,
        float ground_distance, float gyro_x_rate, float gyro_y_rate, float gyro_z_rate)
{
    static uint16_t frame_count = 0;
    int i;
    union
    {
        i2c_frame f;
        char c[I2C_FRAME_SIZE];
    } u;

    u.f.frame_count = frame_count;
    u.f.pixel_flow_x_sum = pixel_flow_x_sum;
    u.f.pixel_flow_y_sum = pixel_flow_y_sum;
    u.f.flow_comp_m_x = flow_comp_m_x * 1000;
    u.f.flow_comp_m_y = flow_comp_m_y * 1000;
    u.f.qual = qual;
    u.f.ground_distance = ground_distance * 1000;

    u.f.gyro_x_rate = gyro_x_rate * getGyroScalingFactor();
    u.f.gyro_y_rate = gyro_y_rate * getGyroScalingFactor();
    u.f.gyro_z_rate = gyro_z_rate * getGyroScalingFactor();
    u.f.gyro_range = getGyroRange();


    uint32_t time_since_last_sonar_update;

    time_since_last_sonar_update = (get_boot_time_us() - get_sonar_measure_time());


    if (time_since_last_sonar_update < 255*1000){
    	u.f.sonar_timestamp = time_since_last_sonar_update/1000;//convert to ms
    }
    else{
    	u.f.sonar_timestamp = 255;
    }


    union
    {
        i2c_integral_frame f;
        char c_integral[I2C_INTEGRAL_FRAME_SIZE];
    } u_integral;

    static uint16_t accumulated_flow_x = 0;
    static uint16_t accumulated_flow_y = 0;
    static uint16_t accumulated_framecount = 0;
    static int16_t accumulated_gyro_x = 0;
    static int16_t accumulated_gyro_y = 0;
    static int16_t accumulated_gyro_z = 0;
    static uint32_t time_between_readings = 0;
    static uint32_t lasttime = 0;


    //accumulate flow and gyro values between sucessive I2C readings
    if(stop_accumulation==0 && readout_done_frame2==1){
    	 // reset if readout has been performed
         time_between_readings = time_between_readings/accumulated_framecount; //fill in minimal time between frames
    	 accumulated_flow_x = pixel_flow_x_sum;
    	 accumulated_flow_y = pixel_flow_y_sum;
    	 accumulated_framecount = 1;
    	 accumulated_gyro_x = gyro_x_rate * getGyroScalingFactor();
    	 accumulated_gyro_y = gyro_y_rate * getGyroScalingFactor();
    	 accumulated_gyro_z = gyro_z_rate * getGyroScalingFactor();
    }
    else{
    	 // accumulate if there is no readout
		 time_between_readings += (get_boot_time_us()-lasttime);
    	 accumulated_flow_x += pixel_flow_x_sum;
    	 accumulated_flow_y += pixel_flow_y_sum;
    	 accumulated_framecount++;
    	 accumulated_gyro_x += gyro_x_rate * getGyroScalingFactor();
    	 accumulated_gyro_y += gyro_y_rate * getGyroScalingFactor();
    	 accumulated_gyro_z += gyro_z_rate * getGyroScalingFactor();
    }

    lasttime = get_boot_time_us();
    stop_accumulation=readout_done_frame2;

    u_integral.f.frame_count_since_last_readout = accumulated_framecount;
    u_integral.f.gyro_x_rate_integral =accumulated_gyro_x;
    u_integral.f.gyro_y_rate_integral =accumulated_gyro_y;
    u_integral.f.gyro_z_rate_integral =accumulated_gyro_z;
    u_integral.f.gyro_range = getGyroRange();
    u_integral.f.pixel_flow_x_integral =accumulated_flow_x;
    u_integral.f.pixel_flow_y_integral =accumulated_flow_y;
    u_integral.f.time_since_last_readout = time_between_readings;
    u_integral.f.ground_distance = ground_distance * 1000;
    u_integral.f.sonar_timestamp = time_since_last_sonar_update;

    int notpublishedIndex = 1 - publishedIndex; // choose not the current published buffer

    // fill I2C transmitbuffer with values
    for (i = 0; i < I2C_FRAME_SIZE; i++){
        txData[notpublishedIndex][i] = u.c[i];
    }

    for (i = I2C_FRAME_SIZE; i < I2C_INTEGRAL_FRAME_SIZE+I2C_FRAME_SIZE; i++)
        txData[notpublishedIndex][i] = u_integral.c_integral[i-I2C_FRAME_SIZE];


    if(readout_done_frame1){ //checks frame1 only! todo implement both checks while maintaining compatibility
    	 publishedIndex = 1 -publishedIndex; //swap buffers if I2C bus is idle
    }

    frame_count++;

}

char readI2CAddressOffset()
{
	//read 3bit address offset of 7 bit address
    offset = 0x00;
    offset = GPIO_ReadInputData(GPIOC) >> 13;//bit 0
    offset = offset | ((GPIO_ReadInputData(GPIOC) >> 14)<<1);//bit 1
    offset = offset | ((GPIO_ReadInputData(GPIOC) >> 15)<<2);//bit 2
    offset = (~offset) & 0x07;
    return offset;
}

char i2c_get_ownaddress1()
{
    return (I2C1_OWNADDRESS_1_BASE + readI2CAddressOffset()) << 1;//add offset to base and shift 1 bit to generate valid 7 bit address
}
