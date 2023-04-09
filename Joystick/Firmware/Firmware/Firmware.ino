#include <Arduino.h>
#include <U8g2lib.h>
#include <string>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);    //Software I2C

float progress=0;
enum mode {Deweight, Mob, Assist, Connect, None};
mode Mode=None;
int button[2] = {0, 7};
int btns_last_state[2] = {1, 1};
int cnt = 0; //used for debounce

void setup(void) {
  u8g2.begin();
  drawScreen(Mode);

  pinMode(button[0], INPUT_PULLUP);
  pinMode(button[1], INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
}

void drawProgressBar(float p) {
  u8g2.setDrawColor(1);
  u8g2.drawBox(128-(int)(p*126)-1, 18, (int)(p*126), 22);
  u8g2.drawFrame(0, 17, 128, 24);
  u8g2.setDrawColor(0);
  u8g2.drawBox(1, 18, 126-(int)(p*126), 22);
  u8g2.setFont(u8g2_font_luBIS12_tf);
  String s = String((int)(p*100))+'%';
  char c[5];
  s.toCharArray(c, 5);
  u8g2.setFontMode(1);
  u8g2.setDrawColor(2);
  u8g2.drawStr(50,35,c);
  u8g2.updateDisplayArea(0, 2, 16, 3);//coordinates in tiles: 8x8
}
 
void drawScreen(mode_t m) {
  u8g2.clearBuffer();                   // clear the internal memory
  u8g2.setFont(u8g2_font_luBIS08_tf);   // choose a suitable font
  switch(m) {
    case Deweight:  
      u8g2.setDrawColor(1);
      u8g2.drawStr(20,10,"Deweighting");    // write something to the internal memory
      u8g2.setFont(u8g2_font_luBIS12_tf);
      u8g2.drawStr(0,60,"ADD");    // write something to the internal memory
      u8g2.drawStr(90,60,"DEL");    // write something to the internal memory
      break;
    case Mob:
      u8g2.setDrawColor(1);
      u8g2.drawStr(20,10,"Mobilisation");    // write something to the internal memory
      u8g2.setFont(u8g2_font_luBIS12_tf);
      u8g2.drawStr(74,62,"STOP");    // write something to the internal memory
      break;
    case Assist:
      u8g2.setDrawColor(1);
      u8g2.drawStr(20,10,"Assistance");    // write something to the internal memory
      drawProgressBar(0);
      u8g2.setDrawColor(1);
      u8g2.setFont(u8g2_font_luBIS12_tf);
      u8g2.drawStr(74,62,"STOP");    // write something to the internal memory
      break;
    case Connect:
      u8g2.setDrawColor(1);
      u8g2.setFont(u8g2_font_luBIS14_tf);
      u8g2.drawStr(40,40,"EMU");    // write something to the internal memory
      u8g2.drawStr(42,55,"<=>");    // write something to the internal memory
    default:
      u8g2.setDrawColor(1);
      u8g2.setFont(u8g2_font_luBIS14_tf);
      u8g2.drawStr(40,40,"EMU");    // write something to the internal memory
      //Nothing
  }
  u8g2.sendBuffer();                    // transfer internal memory to the display
}

void loop(void) {

  // Check serial command requesting change of display
  //S[D/A/M] to switch to Deweight, Assist or Mob mode
  //P[val] to update progress value (0-100) in Assist mode
  if (Serial.available() > 1) {
    char c = Serial.read();
    switch(c) {
      //Switch mode
      case 'S':
        switch(Serial.read()) {
          case 'D':
            Mode = Deweight;
            break;
          case 'A':
            Mode = Assist;
            break;
          case 'M':
            Mode = Mob;
            break;
          default:
            Mode = Connect;
        }
        drawScreen(Mode);
        break;
      
      //Update progress
      case 'P':
        progress=Serial.read()/100.;
        if(Mode==Assist)
          drawProgressBar(progress);
        break;

      //Who are you?
      case '?':
        Serial.println("PENDANT"); 
        break;

      default:
        //Nothing
        break;
    }
  }


  //Send button state if pressed
  int btn0=digitalRead(button[0]);
  int btn1=digitalRead(button[1]);
  if( ((!btn0 && btns_last_state[0]) || (!btn1 && btns_last_state[1])) && cnt>30 ) {
    btns_last_state[0] = btn0;
    btns_last_state[1] = btn1;
    Serial.print('B');
    Serial.print(!btn0);
    Serial.println(!btn1);
    cnt = 0;
  }
  else {
    btns_last_state[0] = btn0;
    btns_last_state[1] = btn1;
  }
  cnt++;
  delay(20);
}