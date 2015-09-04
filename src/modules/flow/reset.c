/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file reset.c
 *
 * NOTE: The code here is stolen from NuttX, http://nuttx.org,
 * and BSD licensed.
 */

#include <stdint.h>
#include <stdbool.h>

#define ARMV7M_NVIC_BASE                0xe000e000
#define NVIC_AIRCR_OFFSET               0x0d0c /* Application interrupt/reset contol registr */
#define NVIC_AIRCR                      (ARMV7M_NVIC_BASE + NVIC_AIRCR_OFFSET)
#define NVIC_AIRCR_PRIGROUP_MASK        (7 << NVIC_AIRCR_PRIGROUP_SHIFT)
#define NVIC_AIRCR_VECTKEY_SHIFT        (16)      /* Bits 16-31: VECTKEY */
#define NVIC_AIRCR_SYSRESETREQ          (1 << 2)  /* Bit 2:  System reset */
#define NVIC_AIRCR_PRIGROUP_SHIFT       (8)       /* Bits 8-14: PRIGROUP */

#define STM32_PWR_BASE       0x40007000     /* 0x40007000-0x400073ff: Power control PWR */
#define STM32_PWR_CR_OFFSET    0x0000  /* Power control register */
#define PWR_CR_DBP             (1 << 8)  /* Bit 8: Disable Backup Domain write protection */

# define getreg32(a)          (*(volatile uint32_t *)(a))
# define putreg32(v,a)        (*(volatile uint32_t *)(a) = (v))


/* prototypes */
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);
void stm32_pwr_enablebkp(void);
void systemreset(bool to_bootloader);

void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits)
{
  uint32_t   regval;

  regval  = getreg32(addr);
  regval &= ~clearbits;
  regval |= setbits;
  putreg32(regval, addr);
}

void stm32_pwr_enablebkp(void)
{
  modifyreg32(STM32_PWR_BASE + (uint32_t)STM32_PWR_CR_OFFSET, (uint32_t)0, (uint32_t)PWR_CR_DBP);
}

void up_systemreset(void);

void up_systemreset(void)
{
  uint32_t regval;

  /* Set up for the system reset, retaining the priority group from the
   * the AIRCR register.
   */

  regval  = getreg32(NVIC_AIRCR) & NVIC_AIRCR_PRIGROUP_MASK;
  regval |= ((0x5fa << NVIC_AIRCR_VECTKEY_SHIFT) | NVIC_AIRCR_SYSRESETREQ);
  putreg32(regval, NVIC_AIRCR);

  /* Ensure completion of memory accesses */              

  __asm volatile ("dsb");

  /* Wait for the reset */

  for (;;);
}

void systemreset(bool to_bootloader)
{
    if (to_bootloader) {
        stm32_pwr_enablebkp();

        /* XXX wow, this is evil - write a magic number into backup register zero */
        *(uint32_t *)0x40002850 = 0xb007b007;
    }
    up_systemreset();

    /* lock up here */
    while(true);
}
