/* #include "DSP28x_Project.h"
#include "f2802x_headers/include/F2802x_CpuTimers.h"
#include "f2802x_common/include/F2802x_globalprototypes.h" */
interrupt void cpu_timer1_isr(void);
void Adc_Config(void);
void adc_init(void);
void pwm_init(void);
void Gpio_init(void);
void Hall_Read(void);
void analogWrite(int module,Uint16 duty);
void speed(void);
void PID(void);
int analogRead(void);
void analogInit(void);
struct {
    int Hall1;
    int Hall2;
    int Hall3;
    int Hall3old;
}Hall;
struct {
    int Counter;
    float Speed;
    int SystemFlag;
}Speed;
struct {
    float Kp;
    float Ki;
    float Kd;
    float Error;
    float Out;
    float Integral;
    float Gas;
    float Duty;
}SpeedPID;
struct {
    float Speed;
    float Time;
    float a;
}Desired;



main(){
 Hall.Hall1=GpioDataRegs.GPADAT.bit.GPIO19;
 Hall.Hall2=GpioDataRegs.GPADAT.bit.GPIO29;
 Hall.Hall3=GpioDataRegs.GPADAT.bit.GPIO18;
 Hall.Hall3old=Hall.Hall3;
 Speed.Counter=0;
 Speed.Speed=0;
 SpeedPID.Kp=0.8;
 SpeedPID.Ki=1;
 Desired.Speed=0;
 SpeedPID.Out=0;
 SpeedPID.Integral=0;
 SpeedPID.Error=0;
 SpeedPID.Gas=0;
 Speed.SystemFlag=1;
 SpeedPID.Duty=0;


// PLL, WatchDog


    InitSysCtrl();
    #ifdef FLASH
    
    extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
    #endif

// GPIO:

    DINT;

// PIE Init


   IER = 0x0000;
   IFR = 0x0000;
   InitPieVectTable();
   EALLOW;
   PieVectTable.TINT1 = &cpu_timer1_isr;
   EDIS;
   InitCpuTimers();
   ConfigCpuTimer(&CpuTimer1, 60, 1000);
   CpuTimer1Regs.TCR.all = 0x4001;
   IER |= M_INT13;
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
   EINT;   // Enable Global interrupt INTM
   ERTM;


   Gpio_init();
   pwm_init();

  while(1){
      if(GpioDataRegs.GPADAT.bit.GPIO16==1)
      {
          SpeedPID.Gas=analogRead();

      }
      else
      {
          SpeedPID.Gas=0;
      }
      Hall_Read();
      //speed();
      //PID();
      if(Hall.Hall1==1 &&Hall.Hall2==0 &&Hall.Hall3==1)
        {
        analogWrite(0,0);

        GpioDataRegs.GPACLEAR.bit.GPIO1=1;
        analogWrite(2,SpeedPID.Gas);

        GpioDataRegs.GPACLEAR.bit.GPIO3=1;
        analogWrite(4,0);

        GpioDataRegs.GPASET.bit.GPIO5=1;
        }
        if(Hall.Hall1==0 &&Hall.Hall2==1 &&Hall.Hall3==0)
        {
        analogWrite(0,0);

        GpioDataRegs.GPACLEAR.bit.GPIO1=1;
        analogWrite(2,0);

        GpioDataRegs.GPASET.bit.GPIO3=1;
        analogWrite(4,SpeedPID.Gas);

        GpioDataRegs.GPACLEAR.bit.GPIO5=1;
        }
        if(Hall.Hall1==1 &&Hall.Hall2==1 &&Hall.Hall3==0)
        {
        analogWrite(0,0);

        GpioDataRegs.GPASET.bit.GPIO1=1;
        analogWrite(2,0);

        GpioDataRegs.GPACLEAR.bit.GPIO3=1;
        analogWrite(4,SpeedPID.Gas);

        GpioDataRegs.GPACLEAR.bit.GPIO5=1;
        }
        if(Hall.Hall1==0 &&Hall.Hall2==0 &&Hall.Hall3==1)
        {
        analogWrite(0,SpeedPID.Gas);

        GpioDataRegs.GPACLEAR.bit.GPIO1=1;
        analogWrite(2,0);

        GpioDataRegs.GPACLEAR.bit.GPIO3=1;
        analogWrite(4,0);

        GpioDataRegs.GPASET.bit.GPIO5=1;
        }
        if(Hall.Hall1==0 &&Hall.Hall2==1 &&Hall.Hall3==1)
        {
        analogWrite(0,SpeedPID.Gas);

        GpioDataRegs.GPACLEAR.bit.GPIO1=1;
        analogWrite(2,0);

        GpioDataRegs.GPASET.bit.GPIO3=1;
        analogWrite(4,0);

        GpioDataRegs.GPACLEAR.bit.GPIO5=1;
        }
        if(Hall.Hall1==1 &&Hall.Hall2==0 &&Hall.Hall3==0)
        {
        analogWrite(0,0);

        GpioDataRegs.GPASET.bit.GPIO1=1;
        analogWrite(2,SpeedPID.Gas);

        GpioDataRegs.GPACLEAR.bit.GPIO3=1;
        analogWrite(4,0);

        GpioDataRegs.GPACLEAR.bit.GPIO5=1;
        }
        if(Hall.Hall1==1 &&Hall.Hall2==1 &&Hall.Hall3==1)
        {
            analogWrite(0,0);

            GpioDataRegs.GPACLEAR.bit.GPIO1=1;
            analogWrite(2,0);

            GpioDataRegs.GPACLEAR.bit.GPIO3=1;
            analogWrite(4,0);

             GpioDataRegs.GPACLEAR.bit.GPIO5=1;
        }
        if(Hall.Hall1==0 &&Hall.Hall2==0 &&Hall.Hall3==0)
        {
            analogWrite(0,0);

            GpioDataRegs.GPACLEAR.bit.GPIO1=1;
            analogWrite(2,0);

            GpioDataRegs.GPACLEAR.bit.GPIO3=1;
            analogWrite(4,0);

            GpioDataRegs.GPACLEAR.bit.GPIO5=1;
        }



  }

}
void pwm_init(void){
    EALLOW;
      EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 
      EPwm1Regs.TBPRD = 6000;     
      EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; 
      EPwm1Regs.TBPHS.half.TBPHS = 0x0000;      
      EPwm1Regs.TBCTR = 0x0000;                 
      EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; 
      EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;

      EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
      EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
      EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
      EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

      EPwm1Regs.CMPA.all = 0;    
      EPwm1Regs.CMPB = 0;           

      EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            
      EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;        

      EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;            
      EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;         
     
      EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; p
      EPwm2Regs.TBPRD = 6000;      
      EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;    
      EPwm2Regs.TBPHS.half.TBPHS = 0x0000;      
      EPwm2Regs.TBCTR = 0x0000;                  
      EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   
      EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV2;
      EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
      EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
      EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
      EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
      EPwm2Regs.CMPA.all = 0;      
      EPwm2Regs.CMPB = 0;               
    
      EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;             
      EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;              
      EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;             
      EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;               
      // Setup TBCLK
      EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 
      EPwm3Regs.TBPRD = 6000;       // Set timer period
      EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;   
      EPwm3Regs.TBPHS.half.TBPHS = 0x0000;       
      EPwm3Regs.TBCTR = 0x0000;                 
      EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;  
      EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV2;
      
      EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
      EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
      EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
      EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
      EPwm3Regs.CMPA.all = 0; // Set compare A value
      EPwm3Regs.CMPB = 0;           // Set Compare B value
      
      EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;         
      EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;     
      EPwm3Regs.AQCTLB.bit.ZRO = AQ_TOGGLE;     
      EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;


          EDIS;


  return;
}
void Gpio_init(void){
  EALLOW;
  GpioCtrlRegs.GPAMUX1.bit.GPIO0=1;
  GpioCtrlRegs.GPAMUX1.bit.GPIO1=0;
  GpioCtrlRegs.GPAMUX1.bit.GPIO2=1;
  GpioCtrlRegs.GPAMUX1.bit.GPIO3=0;
  GpioCtrlRegs.GPAMUX1.bit.GPIO4=1;
  GpioCtrlRegs.GPAMUX1.bit.GPIO5=0;
  GpioCtrlRegs.GPAMUX2.bit.GPIO19=0;
  GpioCtrlRegs.GPAMUX2.bit.GPIO29=0;
  GpioCtrlRegs.GPAMUX2.bit.GPIO18=0;
  GpioCtrlRegs.GPAMUX2.bit.GPIO16=0;
  GpioCtrlRegs.GPADIR.bit.GPIO0=1;
  GpioCtrlRegs.GPADIR.bit.GPIO1=1;
  GpioCtrlRegs.GPADIR.bit.GPIO2=1;
  GpioCtrlRegs.GPADIR.bit.GPIO3=1;
  GpioCtrlRegs.GPADIR.bit.GPIO4=1;
  GpioCtrlRegs.GPADIR.bit.GPIO5=1;
  GpioCtrlRegs.GPADIR.bit.GPIO19=0;
  GpioCtrlRegs.GPADIR.bit.GPIO29=0;
  GpioCtrlRegs.GPADIR.bit.GPIO18=0;
  GpioCtrlRegs.GPADIR.bit.GPIO16=0;
  GpioCtrlRegs.GPAPUD.bit.GPIO19=1;
  GpioCtrlRegs.GPAPUD.bit.GPIO29=1;
  GpioCtrlRegs.GPAPUD.bit.GPIO18=1;
  GpioCtrlRegs.GPAPUD.bit.GPIO16=0;
  EDIS;
  return;
}
void Hall_Read(void){
    Hall.Hall3old=Hall.Hall3;
    Hall.Hall1=GpioDataRegs.GPADAT.bit.GPIO19;
    Hall.Hall2=GpioDataRegs.GPADAT.bit.GPIO29;
    Hall.Hall3=GpioDataRegs.GPADAT.bit.GPIO18;
  return;
}
void analogWrite(int module,Uint16 duty){
    EALLOW;
    switch (module){
    case 0:
        EPwm1Regs.CMPA.half.CMPA = duty;
        break;
    case 1:
        EPwm1Regs.CMPB=duty;
        break;
    case 2:
        EPwm2Regs.CMPA.half.CMPA=duty;
        break;
    case 3:
        EPwm2Regs.CMPB=duty;
        break;
    case 4:
        EPwm3Regs.CMPA.half.CMPA=duty;
        break;
    case 5:
        EPwm3Regs.CMPB=duty;
        break;
    }
    EDIS;
return;
}
interrupt void cpu_timer1_isr(void)
{
   /*CpuTimer1.InterruptCount++;
   if(GpioDataRegs.GPADAT.bit.GPIO16==0)
   {
       Desired.a=analogRead();
       Desired.Speed=Desired.a*0.25;
   }
   else if (GpioDataRegs.GPADAT.bit.GPIO16==1){
               Desired.Speed=0;
              Speed.Speed=0;
              Speed.Counter=0;
              Speed.SystemFlag=0;
              Hall.Hall1=GpioDataRegs.GPADAT.bit.GPIO19;
              Hall.Hall2=GpioDataRegs.GPADAT.bit.GPIO29;
              Hall.Hall3=GpioDataRegs.GPADAT.bit.GPIO18;
              Hall.Hall3old=0;
              SpeedPID.Out=0;
              SpeedPID.Integral=0;
              SpeedPID.Error=0;
              SpeedPID.Duty=0;
   }
   if (Speed.Counter>=11 )
   {
       Speed.Speed=60000/(CpuTimer1.InterruptCount);
       Speed.Counter=0;
       Desired.Time=CpuTimer1.InterruptCount*0.0001;
       CpuTimer1.InterruptCount=0;


   }
   else{
       Desired.Time=0.001;

   }
   if (CpuTimer1.InterruptCount>10000)
   {
       Speed.Speed=0;
       CpuTimer1.InterruptCount=0;
       Speed.Counter=0;
       Speed.SystemFlag=0;
       Hall.Hall1=GpioDataRegs.GPADAT.bit.GPIO19;
       Hall.Hall2=GpioDataRegs.GPADAT.bit.GPIO29;
       Hall.Hall3=GpioDataRegs.GPADAT.bit.GPIO18;
       Hall.Hall3old=0;
       Speed.Speed=0;
       SpeedPID.Out=0;
       SpeedPID.Integral=0;
       SpeedPID.Error=0;
       SpeedPID.Duty=0;
   }
   if (Desired.Speed>0){
       Speed.SystemFlag=1;
   }
   SpeedPID.Gas = (float)(SpeedPID.Duty);
   // The CPU acknowledges the interrupt.*/

}
void speed(void){
    if(Hall.Hall3^Hall.Hall3old==1){
        Speed.Counter++;
    }
    return;
}
void PID(void){
    if(Speed.SystemFlag==1)
    {
    SpeedPID.Error=Desired.Speed-Speed.Speed;
    SpeedPID.Integral=SpeedPID.Integral+(SpeedPID.Error*Desired.Time);
    SpeedPID.Out=(SpeedPID.Ki*SpeedPID.Integral)+(SpeedPID.Kp*SpeedPID.Error);
    SpeedPID.Duty=(float)SpeedPID.Out;
    if (SpeedPID.Duty>EPwm1Regs.TBPRD){
            SpeedPID.Duty=EPwm1Regs.TBPRD;
        }
        if (SpeedPID.Duty<0){
                SpeedPID.Duty=0;
            }
    }
    return;
}
int analogRead(void)
{

    analogInit();

    EALLOW;

            //Force SOC 0 and 1
            AdcRegs.ADCSOCFRC1.all = 3;
            EDIS;

            while(AdcRegs.ADCINTFLG.bit.ADCINT1 != 1)
            {
            
            }
            AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

            return (AdcResult.ADCRESULT1);
}
void analogInit(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
    (*Device_cal)();
    AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;      // Power ADC BG
       AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      // Power reference
       AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;      // Power ADC
       AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC
       AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      // Select internal BG
       AdcRegs.ADCSOC0CTL.bit.CHSEL= 0;    //A0
      //    AdcRegs.ADCSOC1CTL.bit.CHSEL= 1;  //A1
      //    AdcRegs.ADCSOC2CTL.bit.CHSEL= 2;  //A2
      //    AdcRegs.ADCSOC3CTL.bit.CHSEL= 3;  //A3
      //    AdcRegs.ADCSOC4CTL.bit.CHSEL= 4;  //A4
      //    AdcRegs.ADCSOC5CTL.bit.CHSEL= 6;  //A6
      //    AdcRegs.ADCSOC6CTL.bit.CHSEL= 7;  //A7
      //    AdcRegs.ADCSOC7CTL.bit.CHSEL= 9;  //B1
      //    AdcRegs.ADCSOC8CTL.bit.CHSEL= 10; //B2
      //    AdcRegs.ADCSOC9CTL.bit.CHSEL= 11; //B3
      //    AdcRegs.ADCSOC10CTL.bit.CHSEL= 12;    //B4
      //    AdcRegs.ADCSOC11CTL.bit.CHSEL= 14;    //B6
      //    AdcRegs.ADCSOC12CTL.bit.CHSEL= 15;    //B7
          AdcRegs.ADCSOC1CTL.bit.CHSEL= 4;    //A1
          AdcRegs.ADCSOC2CTL.bit.CHSEL= 4;    //A2
          AdcRegs.ADCSOC3CTL.bit.CHSEL= 4;    //A3
          AdcRegs.ADCSOC4CTL.bit.CHSEL= 0;    //A4
          AdcRegs.ADCSOC5CTL.bit.CHSEL= 0;    //A6
          AdcRegs.ADCSOC6CTL.bit.CHSEL= 0;    //A7
          AdcRegs.ADCSOC7CTL.bit.CHSEL= 0;    //B1
          AdcRegs.ADCSOC8CTL.bit.CHSEL= 0;    //B2
          AdcRegs.ADCSOC9CTL.bit.CHSEL= 0;    //B3
          AdcRegs.ADCSOC10CTL.bit.CHSEL= 0;   //B4
          AdcRegs.ADCSOC11CTL.bit.CHSEL= 0;   //B6
          AdcRegs.ADCSOC12CTL.bit.CHSEL= 0;   //B7
          AdcRegs.ADCSOC13CTL.bit.CHSEL= 0;
          AdcRegs.ADCSOC14CTL.bit.CHSEL= 0;
          AdcRegs.ADCSOC15CTL.bit.CHSEL= 0;

          AdcRegs.ADCSOC0CTL.bit.ACQPS  = 25;
          AdcRegs.ADCSOC1CTL.bit.ACQPS  = 25;
          AdcRegs.ADCSOC2CTL.bit.ACQPS  = 25;
          AdcRegs.ADCSOC3CTL.bit.ACQPS  = 25;
          AdcRegs.ADCSOC4CTL.bit.ACQPS  = 25;
          AdcRegs.ADCSOC5CTL.bit.ACQPS  = 25;
          AdcRegs.ADCSOC6CTL.bit.ACQPS  = 25;
          AdcRegs.ADCSOC7CTL.bit.ACQPS  = 25;
          AdcRegs.ADCSOC8CTL.bit.ACQPS  = 25;
          AdcRegs.ADCSOC9CTL.bit.ACQPS  = 25;
          AdcRegs.ADCSOC10CTL.bit.ACQPS = 25;
          AdcRegs.ADCSOC11CTL.bit.ACQPS = 25;
          AdcRegs.ADCSOC12CTL.bit.ACQPS = 25;
          AdcRegs.ADCSOC13CTL.bit.ACQPS = 25;
          AdcRegs.ADCSOC14CTL.bit.ACQPS = 25;
          AdcRegs.ADCSOC15CTL.bit.ACQPS = 25;

          AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;
          AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;
          AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;
          AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;
          AdcRegs.ADCSOC4CTL.bit.TRIGSEL  = 5;
          AdcRegs.ADCSOC5CTL.bit.TRIGSEL  = 5;
          AdcRegs.ADCSOC6CTL.bit.TRIGSEL  = 5;
          AdcRegs.ADCSOC7CTL.bit.TRIGSEL  = 5;
          AdcRegs.ADCSOC8CTL.bit.TRIGSEL  = 5;
          AdcRegs.ADCSOC9CTL.bit.TRIGSEL  = 5;
          AdcRegs.ADCSOC10CTL.bit.TRIGSEL = 5;
          AdcRegs.ADCSOC11CTL.bit.TRIGSEL = 5;
          AdcRegs.ADCSOC12CTL.bit.TRIGSEL = 5;
          AdcRegs.ADCSOC13CTL.bit.TRIGSEL = 5;
          AdcRegs.ADCSOC14CTL.bit.TRIGSEL = 5;
          AdcRegs.ADCSOC15CTL.bit.TRIGSEL = 5;

          //Assert interrupt for EOC1 and enable interrupt
          AdcRegs.ADCCTL1.bit.INTPULSEPOS   = 1;   
          AdcRegs.INTSEL1N2.bit.INT1SEL = 1;
          AdcRegs.INTSEL1N2.bit.INT1E = 1;
          PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
          AdcRegs.ADCCTL1.bit.INTPULSEPOS   = 1;    
              AdcRegs.INTSEL1N2.bit.INT1E     = 1;   
              AdcRegs.INTSEL1N2.bit.INT1SEL   = 2;   

              AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;   
              EPwm1Regs.ETSEL.bit.SOCAEN   = 1;       
              EPwm1Regs.ETSEL.bit.SOCASEL  = 4;      
              EPwm1Regs.ETPS.bit.SOCAPRD   = 1;      
          IER |= M_INT1;                       
          EINT;                              
          ERTM;
          EDIS;
}

