
// MSE 2202B Charlieplex test code
/*
to use Charieplexing
include both

#include <uSTimer2.h>
#include <CharliePlexM.h>

for 3 wire 6 LEDs (LEDs number 1, 2, 3, 10, 11, 12) use
  CharliePlexM::set(Dx1, Dx2, Dx3);
for 4 wire 12 LEDs  use
  CharliePlexM::set(Dx1, Dx2, Dx3, DX4);
for 3 wire 6 LEDs (LEDs number 1, 2, 3, 10, 11, 12) And Push Button use
  CharliePlexM::set(Dx1, Dx2, Dx3, PB);
for Encoder use
   CharliePlexM::setEncoders( En1, En2);
  
Dx = Arduino's Digital Port number,  to wire so LED1 is LED1 etc.  wire Dx1 to C1, DX2, to C2 ....
PB = Digital port Number that button is connected to must be same as one of Dx number already used 
En1, En2 =  Arduino's Digital Port number eth encoders are connected to and should NOT be the same as Dx or PB 

To change LED(s) brightness
CharliePlexM::Brightness(number form 1 to 10); //1 being dimmest

To turn on or off single LED
 CharliePlexM::Write(LED number,[1 for on/0 for off]);
 
 
to turn on matrix of LEDs
 CharliePlexM::WriteMatrix(LED Matrix Integer);
The matrix integer 
                      1963 1852 1741
                      2    1    0
uiLEDMatrixMSB = 0000 0000  
    uiLEDMatrixLSB =       0000 0000 
 
example for F  -  LED Matrix Integer = 0x0F55


  Led Layout                           
		             set( vc1,     vc2,     vc3)
		    set( vc1,     vc2,     vc3,     vc4) vc = charlieplex port variable 
	    	         DX1,     Dx2,     Dx3,     Dx4  DX = arduino's Digital Ports
			 C1 ,     C2 ,     C3 ,     C4   Chaplieplex pins on MSE-duino brd
			  |        |        |        |
			  |        |      LED1>      |
			  |        | <LED2  |        |
			  |        | LED3>  |        |
			  |      <LED4               |
			  |      LED5>               |
			  |     <LED6       |        |
			  |     LED7>       |        |
			  |  <LED8 |        |        |
			  |  LED9> |        |        |
			  |        |        | <LED10 |
			  |        |    <LED11       |
			  |        |        | LED12> |






The matrix command using in the serial monitor to enter numbers
                      1963 1852 1741
                      2    1    0
uiLEDMatrixMSB = 0000 0000  
    uiLEDMatrixLSB =       0000 0000 
 ie to print a letter A = uiLEDMatrixMSB = 15, uiLEDMatrixLSB = 95

*/




#include <uSTimer2.h>
#include <CharliePlexM.h>



const int ciDisplayTime = 100;

char  cTempDump;
char cLEDTable[13]= {'0','0','0','0','0','0','0','0','0','0','0','0','0'};

char cLEDCheck;
char cLEDCheckIndex;

char  cDisplayInput; 
char  cDisplayOutput;

unsigned int uiDisplayTime;
unsigned int uiLedtoDisplay;

unsigned int ui_LastBtnPress;


unsigned int uiLEDIndex;
unsigned int uiLedOnOff;  

unsigned int uiReadEncoders;  

unsigned int uiLEDMatrixMSB;
unsigned int uiLEDMatrixLSB;

unsigned int ui_RunServo;

unsigned int uiDoOnce;

long int liTime;
long int liLastTime;

/*
to use Charieplexing
include both

#include <uSTimer2.h>
#include <CharliePlexM.h>


//CharliePlexM::set(10,11,12, 13);     // Set up Charlieplexing for LEDs
  CharliePlexM::setBtn(10,11,12, 13, 13);     // Set up Charlieplexing for LEDs with button on pin 10
                             
		             set( vc1,     vc2,     vc3)
		    set( vc1,     vc2,     vc3,     vc4) vc = charlieplex port variable 
	    	         DX1,     Dx2,     Dx3,     Dx4  DX = arduino's Digital Ports
			 C1 ,     C2 ,     C3 ,     C4   Chaplieplex pins on MSE-duino brd
			  |        |        |        |
			  |        |      LED1>      |
			  |        | <LED2  |        |
			  |        | LED3>  |        |
			  |      <LED4               |
			  |      LED5>               |
			  |     <LED6       |        |
			  |     LED7>       |        |
			  |  <LED8 |        |        |
			  |  LED9> |        |        |
			  |        |        | <LED10 |
			  |        |    <LED11       |
			  |        |        | LED12> |




*/


#include <Servo.h> 
 
Servo myservo;



void setup()
{
  Serial.begin(9600);
 //CharliePlexM::set(8,9,10,11);     // Set up Charlieplexing for LEDs
 CharliePlexM::setBtn(4,7,12,13,7);     // Set up Charlieplexing for LEDs with button on pin 10
 //CharliePlexM::setEncoders(2,3);
 CharliePlexM::setEncoders(3,5,10,11);
 //CharliePlexM::set(10,11,12);     // Set up Charlieplexing for LEDs
 //CharliePlexM::setBtn(10,11,2,2);     // Set up Charlieplexing for LEDs with button on pin 10
  myservo.attach(8);
  cLEDCheck = 1;
 
  uiLedOnOff = 0;
  uiDisplayTime = ciDisplayTime;
  uiLedtoDisplay = 0;
  cDisplayOutput = 'h';
  cDisplayInput = 'H';
  liTime = 0;
  liLastTime = 0;
  uiLEDMatrixMSB = 0;
  uiLEDMatrixLSB = 0; 
  ui_LastBtnPress = 1;
  
  uiReadEncoders = 0;
  ui_RunServo = 0;
   
}

void loop() 
{
  while (Serial.available() > 0)
   {
    
    cDisplayInput = Serial.read();
     
    switch(cDisplayInput)
    {
      case 'b':
      case 'B':
      {
        uiLedtoDisplay = Serial.parseInt();
        
        CharliePlexM::Brightness(uiLedtoDisplay);
        
        cDisplayOutput = 'b';
        break;
      }
      case 'e':
      case 'E':
      {
        
        cDisplayOutput = 'e';
        break;
        
      }
      
      case 'f':
      case 'F':
      {
        
        cDisplayOutput = 'f';
        break;
        
      }
     
      case 'l':
      case 'L':
      {
        cDisplayOutput = 'l';
        uiLedtoDisplay = Serial.parseInt();
        uiLedOnOff = Serial.parseInt();
        if(uiLedOnOff == 0)
        {
          
         cLEDTable[uiLedtoDisplay] = '0';
        }
        else
        {
          cLEDTable[uiLedtoDisplay] = '1';
        
        }
        
        CharliePlexM::Write(uiLedtoDisplay,uiLedOnOff);
        break;
      }
      case 'M':
      case 'm':
      {
        cDisplayOutput = 'm';
        uiLEDMatrixMSB = Serial.parseInt();
        uiLEDMatrixLSB = Serial.parseInt();
       
       
        uiLEDMatrixMSB = uiLEDMatrixMSB << 8;
        uiLEDMatrixMSB = uiLEDMatrixMSB | uiLEDMatrixLSB;
        if((uiLEDMatrixMSB & 0x0001)  == 0)
        {
         cLEDTable[1] = '0';
        }
        else
        {
         cLEDTable[1] = '1';
        }
        if((uiLEDMatrixMSB & 0x0002)  == 0)
        {
         cLEDTable[4] = '0';
        }
        else
        {
         cLEDTable[4] = '1';
        }
        if((uiLEDMatrixMSB & 0x0004)  == 0)
        {
         cLEDTable[7] = '0';
        }
        else
        {
         cLEDTable[7] = '1';
        }
        if((uiLEDMatrixMSB & 0x0008)  == 0)
        {
         cLEDTable[10] = '0';
        }
        else
        {
         cLEDTable[10] = '1';
        }
        if((uiLEDMatrixMSB & 0x0010)  == 0)
        {
         cLEDTable[2] = '0';
        }
        else
        {
         cLEDTable[2] = '1';
        }
        if((uiLEDMatrixMSB & 0x0020)  == 0)
        {
         cLEDTable[5] = '0';
        }
        else
        {
         cLEDTable[5] = '1';
        }
        if((uiLEDMatrixMSB & 0x0040)  == 0)
        {
         cLEDTable[8] = '0';
        }
        else
        {
         cLEDTable[8] = '1';
        }
        if((uiLEDMatrixMSB & 0x0080)  == 0)
        {
         cLEDTable[11] = '0';
        }
        else
        {
         cLEDTable[11] = '1';
        }
        if((uiLEDMatrixMSB & 0x0100)  == 0)
        {
         cLEDTable[3] = '0';
        }
        else
        {
         cLEDTable[3] = '1';
        }
        if((uiLEDMatrixMSB & 0x0200)  == 0)
        {
         cLEDTable[6] = '0';
        }
        else
        {
         cLEDTable[6] = '1';
        }
        if((uiLEDMatrixMSB & 0x0400)  == 0)
        {
         cLEDTable[9] = '0';
        }
        else
        {
         cLEDTable[9] = '1';
        }
        if((uiLEDMatrixMSB & 0x0800)  == 0)
        {
         cLEDTable[12] = '0';
        }
        else
        {
         cLEDTable[12] = '1';
        }
        CharliePlexM::WriteMatrix(uiLEDMatrixMSB);
        break;
      }
      case 'H':
      case 'h':
      {
        cDisplayOutput = 'h';
        uiLedtoDisplay = Serial.parseInt();
        uiLedOnOff = Serial.parseInt();
        if((uiLedOnOff & 0x01) == 0)
        {
         cLEDTable[1] = '0';
        }
        else
        {
          cLEDTable[1] = '1';
        }
        break;
      }
      
     cDisplayInput = 0; 
    }
   }
  
  
  
  
  liTime = micros();
  if((liTime - liLastTime) >= 1000)
  {
    liLastTime = liTime;
   if(ui_RunServo)
   {
     
      myservo.write(120);
      ui_RunServo = ui_RunServo - 1;
      uiDoOnce = 1;
   }
   else
   {
     myservo.write(90);
     if(uiDoOnce)
     {
       uiDoOnce = 0;
     Serial.print("Encoder 1 =  ");
      Serial.println(CharliePlexM::ul_LeftEncoder_Count);
      Serial.print("Encoder 2 =  ");
      Serial.println(CharliePlexM::ul_RightEncoder_Count);
      Serial.print("Encoder 3 =  ");
      Serial.println(CharliePlexM::ul_Encoder3_Count);
      Serial.print("Encoder 4 =  ");
      Serial.println(CharliePlexM::ul_Encoder4_Count);
     }
   } 
   
    uiDisplayTime = uiDisplayTime - 1;
   
   
    if(uiDisplayTime == 0)
    {
     
         
     
       
      DisplayStuff();
      uiDisplayTime = ciDisplayTime;
      if(ui_LastBtnPress != CharliePlexM::ui_Btn)
      {
        ui_LastBtnPress = CharliePlexM::ui_Btn;
        if(CharliePlexM::ui_Btn)
        {
          Serial.println("Button Pressed");
          
        }
        else
        {
          Serial.println("Button Released ");
          CharliePlexM::ul_LeftEncoder_Count = 0;
          CharliePlexM::ul_RightEncoder_Count = 0;
          CharliePlexM::ul_Encoder3_Count = 0;
          CharliePlexM::ul_Encoder4_Count = 0;
        }
        
      }
         

    }
    
  }

}
void DisplayStuff(void)
{
  
  
  switch(cDisplayOutput)
  {
    case 'H':
    case 'h':
    {
      Serial.println("Mechatronics System Engineering 2202b"); 
      Serial.println("H - print this help screen"); 
      Serial.println("L,##,# - Turn on or off single LED, 1 - 12, 0 = off/ 1 = on");
      Serial.println("M,#,# - Matrix LED turn On, MSB, LSB. - ");
      Serial.println("B,# - Display Brightness 1- 100 ");
      Serial.println("E - Read and Display Encoder Count"); 
      Serial.println("F - Read and Display Encoder Count");
      cDisplayOutput = 'o';   
      break;
    }
    case 'L':
    case 'l':
    {
     Serial.println("Single LED---1 2 3 4 5 6 7 8 9 0 1 2"); 
     Serial.print("LEDs on/off -"); 
     for(uiLEDIndex = 1;uiLEDIndex < 12;uiLEDIndex++)
         {
           Serial.print(cLEDTable[uiLEDIndex]);
           Serial.print(" ");
         }
         Serial.println(cLEDTable[12]);
    
     cDisplayOutput = 'o';   
     
     break;
    }
    
    case 'M':
    case 'm':
    {
     Serial.println("Matrix LEDs--"); 
     for(uiLEDIndex = 3;uiLEDIndex > 0;uiLEDIndex--)
         {
           Serial.print(cLEDTable[uiLEDIndex]);
           Serial.print(" ");
         }
     Serial.println(" ");    
     for(uiLEDIndex = 6;uiLEDIndex > 3;uiLEDIndex--)
         {
           Serial.print(cLEDTable[uiLEDIndex]);
           Serial.print(" ");
         }
     Serial.println(" ");  
     for(uiLEDIndex = 9;uiLEDIndex > 6;uiLEDIndex--)
         {
           Serial.print(cLEDTable[uiLEDIndex]);
           Serial.print(" ");
         }
     Serial.println(" ");      
     for(uiLEDIndex = 12;uiLEDIndex > 10;uiLEDIndex--)
         {
           Serial.print(cLEDTable[uiLEDIndex]);
           Serial.print(" ");
         }
       
     Serial.println(cLEDTable[10]);
     cDisplayOutput = 'o';   
     break;  
    }
    case 'e':
    case 'E':
    {
      ui_RunServo = 1000;
      Serial.println(" ");
      Serial.print("Encoder 1 =  ");
      Serial.println(CharliePlexM::ul_LeftEncoder_Count);
      Serial.print("Encoder 2 =  ");
      Serial.println(CharliePlexM::ul_RightEncoder_Count);
      cDisplayOutput = 'e';   
      break;
    }
    case 'f':
    case 'F':
    {
      ui_RunServo = 1000;
       Serial.println(" ");
      Serial.print("Encoder 1 =  ");
      Serial.println(CharliePlexM::ul_LeftEncoder_Count);
      Serial.print("Encoder 2 =  ");
      Serial.println(CharliePlexM::ul_RightEncoder_Count);
      Serial.print("Encoder 3 =  ");
      Serial.println(CharliePlexM::ul_Encoder3_Count);
      Serial.print("Encoder 4 =  ");
      Serial.println(CharliePlexM::ul_Encoder4_Count);
      cDisplayOutput = 'f';   
      break;
    }
    case 'o':
    case 'O':
    {
    
      
      
      
      break;
    }
     
  }
  
  cDisplayOutput = 'o'; 
}

