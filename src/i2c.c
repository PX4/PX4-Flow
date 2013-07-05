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

void i2c_init()
{

    I2C_DeInit(I2C1 );       //Deinit and reset the I2C to avoid it locking up
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);

    GPIO_InitTypeDef gpio_init;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    gpio_init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    //Pull up resistor
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    //Open Drain
    gpio_init.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(GPIOB, &gpio_init);

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

    // enable I2C1
    I2C_StretchClockCmd(I2C1, DISABLE);

    I2C_StretchClockCmd(I2C1, ENABLE);
    // I2C_StretchClockCmd(I2C1, DISABLE);
    I2C_Cmd(I2C1, ENABLE);
//    while (1)
    //  {

    //}
}

uint8_t data[6];

void I2C1_EV_IRQHandler(void)
{

    uint8_t data2;
    static uint8_t txData = 0x00;
    switch (I2C_GetLastEvent(I2C1 ))
    {

    case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED :
    {
        volatile uint32_t temp;
        temp = I2C1 ->SR1;
        temp = I2C1 ->SR2;
        txData = 0;
        break;
    }
    case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED :
    {

        volatile uint32_t temp;
        temp = I2C1 ->SR1;
        temp = I2C1 ->SR2;
        txData = 0;
        break;
    }
    case I2C_EVENT_SLAVE_BYTE_RECEIVED :
    {
        data2 = I2C_ReceiveData(I2C1 );
        data[txData] = data2;
        txData++;
        break;
    }
    case I2C_EVENT_SLAVE_BYTE_TRANSMITTING :
    case I2C_EVENT_SLAVE_BYTE_TRANSMITTED :
    {
        I2C_SendData(I2C1, data[txData]);
        txData++;
        break;
    }

    case I2C_EVENT_SLAVE_ACK_FAILURE :
    {
        volatile uint32_t temp;
        I2C1 ->SR1 &= 0x00FF;
        break;
    }

    case I2C_EVENT_SLAVE_STOP_DETECTED :
    {

        volatile uint32_t temp;
        temp = I2C1 ->SR1;
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

char i2c_get_ownaddress1()
{
    return I2C1_OWNADDRESS_1;
}
