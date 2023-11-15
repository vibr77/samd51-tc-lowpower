/*
__   _____ ___ ___        Author: Vincent BESSON
 \ \ / /_ _| _ ) _ \      Release: 0.35
  \ V / | || _ \   /      Date: 20221227
   \_/ |___|___/_|_\      Description: Power Consumption / Direct CLock Counter on the SAMD51
                2022      Licence: Creative Commons
______________________

Description: 

The purpose of this small code is to count PIN Pulse (Pin 12 / PA22 on this SAMD51) without wakeing up the CPU.
The SAMD51 has several TC Counter and Generic Clock as well as a Ultra low Power Oscillator 32K Freq

TC0 Count the pulse Pin 22
TC5 Timer 32K ULP trigger after compare



NOTE:
-----
Warning: RTCZERO has been changed to work with SAMD51, do not use the standard lib
RTCZERO based on v1.60

WARNING: for stability Feather Mx SAMx1 without crystal for RTC zero are unstable, use M0/M4 Express for instance with 32k Chrystal
*/


#include <arduino.h>

// Adafruit Feather M4: Count input pulses with 32-bit timer TC0 on pin 12
#define P_PIN 12

void setupTimer();


void setup() {
  // Serial USB Port Output ///////////////////////////////////////////////////////////////////////////////
  
  Serial.begin(115200);           // Open the native USB port at 115200 baud
  while(!Serial);                 // Wait for the console to be opened
  delay(1000);
  Serial.print("Start P_PIN:");
  Serial.println(P_PIN);
  
  Serial.print("g_APinDescription(P_PIN).ulPort:");
  Serial.println(g_APinDescription[P_PIN].ulPin);

  setupTimer();
  // Configure PIN 12 // PA22 
  PORT->Group[g_APinDescription[P_PIN].ulPort].DIRCLR.reg=PORT_PA22;                                // DIRCLR => DIR=0 = INPUT
  PORT->Group[g_APinDescription[P_PIN].ulPort].PINCFG[g_APinDescription[P_PIN].ulPin].bit.PULLEN=1; // PAGE 889 
  PORT->Group[g_APinDescription[P_PIN].ulPort].PINCFG[g_APinDescription[P_PIN].ulPin].bit.INEN=1;   // PAGE 889 
  PORT->Group[g_APinDescription[P_PIN].ulPort].OUTSET.reg = PORT_PA22;                              // PAGE 889 out hads to be 1 for PullUp else Pulldown
  
  // Enable the port multiplexer on analog pin 12 // PA22
  PORT->Group[g_APinDescription[P_PIN].ulPort].PINCFG[g_APinDescription[P_PIN].ulPin].bit.PMUXEN = 1;
 
  // Set-up the pin as an EIC (interrupt) peripheral on analog pin  12 PA22 
  PORT->Group[g_APinDescription[P_PIN].ulPort].PMUX[g_APinDescription[P_PIN].ulPin >> 1].reg |= PORT_PMUX_PMUXE(0);
  

  // Manage the Ultra Low Power Clock Source on Gen clock 2 Add it to TC0
  GCLK->GENCTRL[2].bit.SRC            = 0x04; //  ADD VIB:Gen Clock 2 => p.166 GCLK Source: 0x04 is OSCULP32K oscillator output 
  GCLK->PCHCTRL[TC0_GCLK_ID].bit.GEN  = 2;    //   ADD VIB:p.168 Assign Peripheral Channel Control for TC2 to Generic Clock Generator 2 as above.
  GCLK->PCHCTRL[TC0_GCLK_ID].bit.CHEN = 1;    //   ADD VIB:p.167 "the peripheral channel is enabled"
  GCLK->GENCTRL[2].bit.GENEN          = 1;    //   ADD VIB:p.165 "generator is enabled"

  EIC->CTRLA.bit.ENABLE = 0;                        // Disable the EIC peripheral
  while (EIC->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization 
  EIC->CTRLA.bit.CKSEL=1;                           // ADD VIB: ULP CLK_ULP32K page 454

  //CASE 1
  EIC->CONFIG[0].reg = EIC_CONFIG_SENSE6_LOW;      // Set event on detecting a HIGH level
  EIC->CONFIG[0].bit.FILTEN7=1;                     // ADD VIB: add filter Majority Filter [1,1,0]=1 page 455

  //CASE 2
  //EIC->CONFIG[0].reg = EIC_CONFIG_SENSE6_FALL;      // Set event on detecting a FALLLING EVENT
 
  EIC->EVCTRL.reg = 1 << 6;                         // Enable event output on external interrupt 6 
  EIC->INTENCLR.reg = 1 << 6;                       // Clear interrupt on external interrupt 6
  EIC->ASYNCH.reg = 1 << 6;                         // Set-up interrupt as asynchronous input
  EIC->CTRLA.bit.ENABLE = 1;                        // Enable the EIC peripheral
  while (EIC->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization

  // TC0 Count Timer //////////////////////////////////////////////////////////////////////////////////

  //GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |         // Enable perhipheral channel for TC0
  //                                 GCLK_PCHCTRL_GEN_GCLK2;     // Connect generic clock 0 at 120MHz

  TC0->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32;              // Set-up TC0/TC1 timers in 32-bit mode
  
  // Event System ///////////////////////////////////////////////////////////////////////////////

  MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS;         // Switch on the event system peripheral
  
  // Select the event system user on channel 0 (USER number = channel number + 1)
  EVSYS->USER[EVSYS_ID_USER_TC0_EVU].reg = EVSYS_USER_CHANNEL(1);         // Set the event user (receiver) as timer TC0 

  // Select the event system generator on channel 0
  EVSYS->Channel[0].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |              // No event edge detection
                                  EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                 // Set event path as asynchronous
                                  EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_6);   // Set event generator (sender) as external interrupt 6    

  TC0->COUNT32.EVCTRL.reg = TC_EVCTRL_TCEI |              // Enable the TC event input                        
                            TC_EVCTRL_EVACT_COUNT;        // Set up the timer to count on event

  TC0->COUNT32.CTRLA.bit.RUNSTDBY=1;                      // When we go to sleep Zzz.. keep the counter running

  
  // Enable Timer  ///////////////////////////////////////////////////////////////////////////////
  
  TC0->COUNT32.CTRLA.bit.ENABLE = 1;                 // Enable timer TC0
  while (TC0->COUNT32.SYNCBUSY.bit.ENABLE);          // Wait for synchronization
}


void setupTimer(){
    // APB = Advanced Peripheral Bus, see whats connected to each bus on p.20
    MCLK->APBAMASK.reg |= MCLK_APBAMASK_EIC;  // Turn on External Interrupt Controller clock
    MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC2 | MCLK_APBBMASK_EVSYS; // Turn on Timer/Counter 2 and Event System clocks

    // Looks like we're using Generic Clock 7, I'm not sure why...
    GCLK->GENCTRL[7].bit.SRC            = 0x04; // p.166 GCLK Source: 0x03 is GCLK_GEN1: Generic Clock Generator 1 output
    GCLK->PCHCTRL[TC2_GCLK_ID].bit.GEN  = 7; // p.168 Assign Peripheral Channel Control for TC2 to Generic Clock Generator 7 as above.
    GCLK->PCHCTRL[TC2_GCLK_ID].bit.CHEN = 1; // p.167 "the peripheral channel is enabled"
    GCLK->GENCTRL[7].bit.GENEN          = 1; // p.165 "generator is enabled"

    // p.1754 Is "TC Register Summary - 16-bit Mode"
    TC2->COUNT32.CTRLA.bit.SWRST = 1;
    while(TC2->COUNT32.SYNCBUSY.bit.SWRST);

    // Connect TCC2 to Nested Vector Interrupt Controller (NVIC)


    // Note: on p.1755 there is COPENx which: selects the trigger source for capture operation, either events or I/O pin input.
    //       In other words, we can use a GPIO to trigger the TC directly.
    TC2->COUNT32.INTENSET.reg = TC_INTENSET_MC0;       //|  // p.1741 INTENSET
                               // TC_INTENSET_MC1       |  // Enable interrupts for Match/Capture 0, 1, and Overflow
                               // TC_INTENSET_OVF;

    //TC2->COUNT16.EVCTRL.reg   = TC_EVCTRL_EVACT_PPW   |  // p.1738 EVCTRL  "EVACT_PPW" is Event Action: Period captured in CC0, pulse width in CC1
    //                            TC_EVCTRL_TCEI;          //        TCEI: TC Event Enable: Incoming events are enabled
    TC2->COUNT32.WAVE.bit.WAVEGEN= TC_WAVE_WAVEGEN_MFRQ;
    TC2->COUNT32.CTRLA.reg    = TC_CTRLA_MODE_COUNT32 |  // p.1755 CTRLA  p.1757 "MODE" is 16-bit mode
                               // TC_CTRLA_CAPTEN0      |  // p.1756 CAPTENx: Enables capture on channel 0
                               // TC_CTRLA_CAPTEN1      |  //                 Enables capture on channel 1
                                TC_CTRLA_PRESCALER_DIV1024| // (32KHz / 1) is 32000 Hz or 1/freq is 31.125 uSec per clock tick and max 2.031sec for 0xFFFF (or 65,535 decimal for 16-bit timer)
                                TC_CTRLA_ENABLE;         // p.1757 ENABLE: The peripheral is enabled
    
    TC2->COUNT32.CC[0].reg = 200;                                 // Set the counter compare 0 (CC0) register for a PWM duty-cycle of 50%
    while(TC2->COUNT32.SYNCBUSY.bit.CC0);                          // Wait for synchronization

    NVIC_SetPriority(TC2_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC2 to 0 (highest) 
    NVIC_EnableIRQ(TC2_IRQn);
    
    TC2->COUNT32.CTRLA.bit.ENABLE = 1;                 // Enable timer TC0
    while (TC2->COUNT32.SYNCBUSY.bit.ENABLE);          // Wait for synchronization

}

void TC2_Handler() {
  
    //TC2->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;        // This completely reset the counter no start
    //while (TC2->COUNT32.CTRLA.bit.SWRST);

    TC2->COUNT32.INTFLAG.bit.MC0 = 1;       // Clear all the interrupt bits by writing 1's to them.
    TC2->COUNT32.COUNT.reg=0;               // Restart from 0; 
    while (TC0->COUNT32.SYNCBUSY.bit.COUNT);  

    Serial.printf("*");
}

void loop() {
  TC0->COUNT32.CTRLBSET.reg = TC_CTRLBCLR_CMD_READSYNC;     // Initiate read synchronization of COUNT register
  while (TC0->COUNT32.SYNCBUSY.bit.CTRLB);                  // Wait for CTRLBSET register write synchronization
  while (TC0->COUNT32.SYNCBUSY.bit.COUNT);                  // Wait for COUNT register read synchronization
  Serial.println(TC0->COUNT32.COUNT.reg);                   // Read and display the TC0 COUNT register 
  //TC0->COUNT32.CTRLBSET.reg = TC_CTRLBCLR_CMD_RETRIGGER;    // Initiate timer reset
  //while (TC0->COUNT32.SYNCBUSY.bit.CTRLB);                  // Wait for CTRLBSET register write synchronization
  delay(1000);                                              // Wait a second
}