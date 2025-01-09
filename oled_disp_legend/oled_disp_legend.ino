#define F_CPU 8000000UL

#include <GyverOLED.h>
#include "GyverOLEDMenu_mod.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include "util.h"
#include "icons.h"
#include <EEPROM.h>


#define TWI_SLAVEADDR	0x67  //I2C Led Display

#define KEY1 1 << PC3    // KEY1 up button, arduino pin 17
#define KEY2 1 << PC2    // KEY2 enter button, arduino pin 16
#define KEY3 1 << PC1    // KEY3 down button, arduino pin 15

#define LED1 1 << PD0    // LED1 left, arduino pin 0
#define LED2 1 << PD1    // LED2 right, arduino pin 1

#define UPDATE_DISPLAY 5
#define KEY_PUSH_DELAY 3
#define KEY_LONGPUSH_DELAY 25
#define BLINK_BUS_DELAY 12

#define WAIT_100HZ 0xB1  // Timer2 100Hz

#define PARAMETERS 25




//status byte 0
#define STATUS_BIT0_L_ON          0
#define STATUS_BIT0_R_ON          1
#define STATUS_BIT0_L_BUT         2
#define STATUS_BIT0_R_BUT         3
#define STATUS_BIT0_WATERMODE     4		 // Water Mode
#define STATUS_BIT0_SQUEEZE       5  
#define STATUS_BIT0_OVERTEMP      6
#define STATUS_BIT0_RESERVE       7

//status byte 1

#define STATUS_BIT1_L_TRIP_STAGE1 0   // L TRIP current protection
#define STATUS_BIT1_R_TRIP_STAGE1 1   // R TRIP current protection
#define STATUS_BIT1_L_TRIP_STAGE2 2  // L TRIP over current protection short circuit
#define STATUS_BIT1_R_TRIP_STAGE2 3  // R TRIP over current protection short circuit
#define STATUS_BIT1_DO_SHIFT      4
#define STATUS_BIT1_DO_ERROR      5
#define STATUS_BIT1_DI_MODE1      6
#define STATUS_BIT1_DI_MODE2      7

//status byte 2
#define STATUS_BIT2_GEAR_P        0
#define STATUS_BIT2_GEAR_R        1
#define STATUS_BIT2_GEAR_N        2
#define STATUS_BIT2_GEAR_D        3
#define STATUS_BIT2_CURR1_L       4
#define STATUS_BIT2_CURR2_L       5
#define STATUS_BIT2_CURR1_R       6
#define STATUS_BIT2_CURR2_R       7

#define KEY_PUSH_DELAY 3
#define KEY_LONGPUSH_DELAY 25

#define STR_LENGTH 15*2 // 2 bytes per symbol
#define I2C_REG_SIZE 14
#define MENUS_AT_PAGE 6

struct SwitchFilter{
	bool outstate;
	bool long_outstate;
	bool prev;
	unsigned char scanrate;
	unsigned char long_scanrate;
	volatile unsigned char *port;
	unsigned char pin;
	unsigned char curr_scan;
  unsigned char lock;
};

SwitchFilter Key1, Key2, Key3;


struct MainScreenSet{
  int16_t GP_Temp;
  uint16_t GP_Speed;
  bool Water_mode;
  bool Squeeze_mode;
  bool Left_button;
  bool Right_button;
  bool OverTemp;
  uint8_t Right_Strength;       // Empty, LOW, MID, HIGH
  uint8_t Right_Strength_level; //Level in persantage
  uint8_t Left_Strength;        // Empty, LOW, MID, HIGH
  uint8_t Left_Strength_level;  //Level in persantage
  uint8_t selector_mode;        // 1 - P, 2 - R, 3 - N, 4 - D
  uint8_t Left_fault;
  uint8_t Right_fault;
};


struct MenuRow{
  uint8_t id;
  char desc_ru[STR_LENGTH];
  char desc_en[STR_LENGTH];
  uint8_t def;
  uint8_t min;
  uint8_t max;
  uint8_t writable; // 1- writable, 0 - readonly
  int8_t dp; //decimal point position
  uint8_t symb; // 0 - null, 1 - c, 2 - %, 3- A, 4 - s, 5 - mc, 6 - ms, 7 - Celsius
};

const MenuRow menus[] PROGMEM = {
  {1,"01 Ток L LOW","01 I L LOW",30,0,100,1,0,2},
  {2,"02 Ток L MID","02 I L MID",60,0,100,1,0,2},
  {3,"03 Ток L HIGH","03 I L HIGH",100,0,100,1,0,2},
  {4,"04 Ток R LOW","04 I R LOW",30,0,100,1,0,2},
  {5,"05 Ток R MID","05 I R MID",60,0,100,1,0,2},
  {6,"06 Ток R HIGH","06 I R HIGH",100,0,100,1,0,2},
  {7,"07 Тотпуск. норм.","07 Trel. norm.",5,0,100,1,1,1},
  {8,"08 Tотпуск. вода","08 Trel. water",20,0,100,1,1,1},
  {9,"09 Акт. ток L","09 Act curr. L",0,0,0,0,1,3},
  {10,"10 Акт. ток R","10 Act curr. R",0,0,0,0,1,3},
  {11,"11 Тзад. отп L","11 Tr_del. L",0,0,100,1,1,1},
  {12,"12 Тзад. отп R","12 Tr_del. R",0,0,100,1,1,1},
  {13,"13 Ток парк. P","13 I parking P",15,0,100,1,0,2},
  {14,"14 Темп. авария","14 Temp fail",80,20,140,1,0,7},
  {15,"15 Откл темп.","15 Off on temp",1,0,1,1,0,0},
  {16,"16 Iкз. L","16 Iovercur. L",45,0,80,1,0,3},
  {17,"17 Iкз. R","17 Iovercur. R",45,0,80,1,0,3},
  {18,"18 CAN скор.","18 CAN speed",4,0,6,1,0,0},
  {19,"19 CAN адр.","19 CAN address",0x11,0x01,0xFE,1,0,0}, 
  {20,"20 ВХ Mode 1","20 DI mode 1",0,0,1,0,0,0}, 
  {21,"21 ВХ Mode 2","21 DI mode 2",0,0,1,0,0,0}, 
	{22,"22 Калибр I L","22 I Calibr L",40,10,200,1,0,0}, 
	{23,"23 Калибр I R","23 I Calibr R",40,10,200,1,0,0}, 
  {24,"24 Прошивка","24 Firmware", 10,0,100,0,1,0},
  {25,"25 Язык", "25 Language",1,0,1,1,0,0},
};

MainScreenSet main_oled;

struct i2c_data_{
  uint8_t statusbyte0;
  uint8_t statusbyte1;
  uint8_t statusbyte2;
  uint8_t GP_Speed;
  int8_t GP_temp;
  uint8_t current_page;
  uint8_t data[5];
  uint8_t paramfromdisp;
  uint8_t paramfromdisp_addr;
};

i2c_data_ i2c_data;
char i2c_buf[I2C_REG_SIZE];

GyverOLED<SSH1106_128x64, OLED_BUFFER, OLED_SPI, 10, 8, 9> oled;

OledMenu<6, GyverOLED<SSH1106_128x64, OLED_BUFFER, OLED_SPI, 10, 8, 9>> menu(&oled);

char param_text[MENUS_AT_PAGE][STR_LENGTH+2*4];
uint8_t params_buf[MENUS_AT_PAGE];
uint8_t params_min[MENUS_AT_PAGE];
uint8_t params_max[MENUS_AT_PAGE];
int8_t  params_dp[MENUS_AT_PAGE];
uint8_t params_symb[MENUS_AT_PAGE];

int tmp = 0;

bool next_state = false;
uint8_t timer2_count = 0;
bool menu_visible = false;
uint8_t old_param_value = 0;

int menu_index, selected_menu = 0;
int menu_page = 1;
int val = 0;

bool Setup_param = false;
uint8_t requested_menu = 1;
bool menu_received_ack = 0;
uint8_t lang_eeprom = 0;



void Init_i2c_data()
{
  i2c_data.statusbyte0 = 0;
  i2c_data.statusbyte1 = 0;
  i2c_data.statusbyte2 = 0;
  i2c_data.paramfromdisp = 0;
  i2c_data.paramfromdisp_addr = 0;
  i2c_data.current_page = 0;
  i2c_data.GP_Speed = 0;
  i2c_data.GP_temp = 0;
}

void InitMainScreenSet()
{
  main_oled.GP_Speed = 0xFFFF;
  main_oled.GP_Temp = 0;

  main_oled.Left_button = false;
  main_oled.Left_Strength_level = 0;
  main_oled.Left_Strength = 0;

  main_oled.Right_button = false;
  main_oled.Right_Strength_level = 0;
  main_oled.Right_Strength = 0;

  main_oled.Squeeze_mode = false;
  main_oled.Water_mode = false;
  main_oled.selector_mode = 0;
  main_oled.OverTemp = false;
}

void print_friction(uint8_t side, uint8_t strength_level)
{
  uint8_t x, y = 0;
 
  y = 26;

  if (side == 0) x = 55; else
  if (side == 1) x = 97;


  switch(strength_level)
  {
    case 0: break;
    case 1: oled.drawBitmap(x,y,friction_low,14,23);
            break;
    case 2: oled.drawBitmap(x,y,friction_mid,14,23);
            break;  
    case 3: oled.drawBitmap(x,y,friction_high,14,23);
            break;          
  }
}


bool _switch_filter(SwitchFilter * source)
{
	bool result;
	
	result = false;
	
	
	if (!((*source->port) & source->pin))
	{
		if (source->curr_scan >= source->scanrate){
			source->prev = true;
		} 
		
    	if (source->curr_scan >= source->long_scanrate){
			source->prev = false;	
			source->long_outstate = true;					
       } else
       source->curr_scan++;
		
		
		if  (source->scanrate == 0xFF) source->curr_scan = 0;
		
	} else
	{
		if (source->prev) result = true;
		
		source->prev = false;		
		source->long_outstate = false;
		source->curr_scan = 0;
	}
	
	
	return result;
}


ISR (TIMER2_OVF_vect) // 100 Hz
{	
  Key1.outstate = _switch_filter(&Key1);
  Key2.outstate = _switch_filter(&Key2);
  Key3.outstate = _switch_filter(&Key3);

  timer2_count  +=1;


	TCNT2 = WAIT_100HZ;

}

void drawIcon8x8(byte index) {
  size_t s = sizeof icons_8x8[index];  
  for(unsigned int i = 0; i < s; i++) {
    oled.drawByte(pgm_read_byte(&(icons_8x8[index][i])));
  }
}



void MainScreen(MainScreenSet * screen)
{
  oled.clear();
  oled.drawBitmap(0,0, honda_gear, 128, 64);

  oled.setCursorXY(50, 56); // left strength
  oled.setScale(1);
  
  
  switch(screen->Left_Strength)
  {
     case 0:  if (lang_eeprom == 0) oled.print(F("ОТКЛ")); else oled.print(F("OFF"));
              break;
     case 1:  if (lang_eeprom == 0) oled.print(F("НИЗ")); else oled.print(F("LOW"));
              break;
     case 2:  if (lang_eeprom == 0) oled.print(F("СР")); else oled.print(F("MID"));
              break;
     case 3:  if (lang_eeprom == 0) oled.print(F("ВЫС")); else oled.print(F("HIGH"));
              break;    
  }
  print_friction(0, screen->Left_Strength);


  oled.setCursorXY(95, 56); // right strength

  switch(screen->Right_Strength)
  {
     case 0:  if (lang_eeprom == 0) oled.print(F("ОТКЛ")); else oled.print(F("OFF"));
              break;
     case 1:  if (lang_eeprom == 0) oled.print(F("НИЗ")); else oled.print(F("LOW"));
              break;
     case 2:  if (lang_eeprom == 0) oled.print(F("СР")); else oled.print(F("MID"));
              break;
     case 3:  if (lang_eeprom == 0) oled.print(F("ВЫС")); else oled.print(F("HIGH"));
              break;    
  }
  print_friction(1, screen->Right_Strength);

  
  if (screen->Left_button)  // left letter (button)
  {
    oled.setCursorXY(60, 5); 
    oled.print(F("L"));
    oled.circle(61,8,8,OLED_STROKE);
  }

  if (screen->Right_button)
  {
    oled.setCursorXY(105, 5); // rigth letter (button)
    oled.print(F("R")); 
    oled.circle(106,8,8,OLED_STROKE);
  }

  oled.setCursorXY(0, 0);
  
  if (screen->OverTemp){
    if (lang_eeprom == 0) oled.print(F("ПЕРЕГРЕВ")); else oled.print(F("OVERHEAT"));
  } else
    oled.print(F("LEGEND"));
    

  oled.setCursorXY(0, 13);    //temperature
 
  if (screen->GP_Temp == -127)
    oled.print(F("--"));
  else
    oled.print(screen->GP_Temp);

  oled.setCursorXY(25, 13);
  oled.print(F("C"));

  oled.setCursorXY(15, 13);
  drawIcon8x8(0);       //degree symbol

  oled.setCursorXY(0, 26);  // Speed

  if (screen->GP_Speed == 0x7FFF)
    oled.print(F("--"));
  else
    oled.print(screen->GP_Speed);
  
  oled.setCursorXY(15, 26);
  if (lang_eeprom == 0) oled.print(F("км/ч")); else oled.print(F("km/h"));
  

  if (screen->Water_mode)  //Water mode
  {
    oled.setCursorXY(0, 41);
    if (lang_eeprom == 0) oled.print(F("ВОДА")); else oled.print(F("WATER"));
  }

  if (screen->Squeeze_mode)  //Squeeze mode
  {
    oled.setCursorXY(0, 56);
    if (lang_eeprom == 0) oled.print(F("ВЫЖИМ")); else oled.print(F("SQUEEZE"));
  }

   oled.setCursorXY(80, 30);
   oled.setScale(2);
  

  switch(screen->selector_mode)
  {
     case 0:  break;
     case 1:  oled.print(F("P"));
              break;
     case 2:  oled.print(F("R"));
              break;
     case 3:  oled.print(F("N"));
              break;    
     case 4:  oled.print(F("D"));
              break;  
  }

  oled.update();
}

void receiveEvent(int howMany) {
  
  uint8_t i = 0;


  while (0 < Wire.available()) { 
    i2c_buf[i] = Wire.read();
    if (i < (I2C_REG_SIZE-1)) i++;   
    
  }
    
  i2c_data.statusbyte0 = i2c_buf[1];
  i2c_data.statusbyte1 = i2c_buf[2];
  i2c_data.statusbyte2 = i2c_buf[3];
  i2c_data.GP_temp     = i2c_buf[4];
  i2c_data.GP_Speed    = i2c_buf[5];
  i2c_data.current_page = i2c_buf[6];
  
  for (i=0; i<5; i++)
  {
    i2c_data.data[i] = i2c_buf[7+i];
  }

}

void requestEvent() {

  if (menu_visible) 
  { 
    i2c_buf[0] = 0;
    i2c_buf[1] = menu_page; //requested_menu+(menu_page-1)*5;
    i2c_buf[2] = i2c_data.paramfromdisp;
    i2c_buf[3] = i2c_data.paramfromdisp_addr;
    requested_menu++;  
  } else
  {
    requested_menu = 0;
    i2c_buf[1] = 0;
    i2c_buf[2] = 0;
    i2c_buf[3] = 0;
  }
  Wire.write(&i2c_buf[0],I2C_REG_SIZE);

  if (requested_menu > 5) 
  {
    requested_menu = 1;
  } 

}

uint8_t BuffAddrFromDataAddr(uint8_t data_addr)
{ 
  if (data_addr == 0) return 0;

  int8_t result = data_addr % 5;
  if (result == 0) result = 5; 

  return result-1;
} 

void UpdateParams()
{
  int i,j = 0;

  //if ((requested_menu > 0) && (requested_menu < 6))
      //params_buf[BuffAddrFromDataAddr(i2c_data.paramtodisp_addr)] = i2c_data.paramtodisp; 




  for (i = 0; i < 5; i++)
  {
    if (menu_page > 0)
    j=i+((menu_page-1)*5); else
    j = 0;

    params_buf[i] = i2c_data.data[i];
    params_min[i] = pgm_read_byte(&menus[j].min);
    params_max[i] = pgm_read_byte(&menus[j].max);
    params_dp[i] =  (int8_t) pgm_read_byte(&menus[j].dp);
    params_symb[i] = pgm_read_byte(&menus[j].symb);
    
    if ((lang_eeprom == 1) && (params_symb[i] == 1)) // replace 'c' to 's' in english menu
      params_symb[i] = 4; 
    
    if ((lang_eeprom == 1) && (params_symb[i] == 5)) // replace 'c' to 's' in english menu
      params_symb[i] = 6;    

   // params_buf[i]=pgm_read_byte(&menus[j].def);
  }

  if (menu_page == 5) 
  {
     params_buf[(PARAMETERS-1)%5] = lang_eeprom;                     //25 parameter
  }
      
}



void UpdateTextBuf()
{
  uint8_t i, j, id = 0;

  for (i = 0; i < 5; i++)
  {
    if (menu_page > 0)
       j=i+((menu_page-1)*5); else
       j = 0;

    //id = pgm_read_byte(&menus[j].id); 

    if (j >= PARAMETERS) strcpy(&param_text[i][0], "");
    else
    {
      if (lang_eeprom == 0) strlcpy_P(&param_text[i][0], menus[j].desc_ru,STR_LENGTH);
      else
                            strlcpy_P(&param_text[i][0], menus[j].desc_en,STR_LENGTH);

    }

  }
}


void BuildMenu(void)
{ 
  int tmp,i,j = 0;
 
  UpdateTextBuf();
  UpdateParams();
  
  if (lang_eeprom == 0)  menu.addItem("<--  ВЫХОД");
  if (lang_eeprom == 1)  menu.addItem("<--  EXIT");

  for (i = 0; i < 5; i++)
  {
    //params_buf[i]=pgm_read_byte(&menus[i].def);
    menu.addItem(&param_text[i][0], 1, &params_buf[i],&params_min[i], &params_max[i], &params_dp[i], &params_symb[i]); 
  }

}

void Update_status()
{

  main_oled.Left_button = i2c_data.statusbyte0 & (1 << STATUS_BIT0_L_BUT);
  main_oled.Right_button = i2c_data.statusbyte0 & (1 << STATUS_BIT0_R_BUT);
  main_oled.Water_mode = i2c_data.statusbyte0 & (1 << STATUS_BIT0_WATERMODE);
  main_oled.OverTemp   = i2c_data.statusbyte0 & (1 << STATUS_BIT0_OVERTEMP);
 
  if (i2c_data.statusbyte0 & (1 << STATUS_BIT0_L_ON))
  PORTD |= LED1;
  else
  PORTD &= ~(LED1);
 

  if (i2c_data.statusbyte0 & (1 << STATUS_BIT0_R_ON))
  PORTD |= LED2;
  else
  PORTD &= ~(LED2); 

  main_oled.Left_fault = 0;
  if (i2c_data.statusbyte1 & (1 << STATUS_BIT1_L_TRIP_STAGE1)) main_oled.Left_fault+=1;
  if (i2c_data.statusbyte1 & (1 << STATUS_BIT1_L_TRIP_STAGE2)) main_oled.Left_fault+=2;  

  main_oled.Right_fault = 0;
  if (i2c_data.statusbyte1 & (1 << STATUS_BIT1_R_TRIP_STAGE1)) main_oled.Left_fault+=1;
  if (i2c_data.statusbyte1 & (1 << STATUS_BIT1_R_TRIP_STAGE2)) main_oled.Left_fault+=2;  
        
  main_oled.Squeeze_mode=i2c_data.statusbyte0 & (1 << STATUS_BIT0_SQUEEZE);  
  if (i2c_data.statusbyte2 & (1 << STATUS_BIT2_GEAR_P)) main_oled.selector_mode = 1;
  if (i2c_data.statusbyte2 & (1 << STATUS_BIT2_GEAR_R)) main_oled.selector_mode = 2;
  if (i2c_data.statusbyte2 & (1 << STATUS_BIT2_GEAR_N)) main_oled.selector_mode = 3;
  if (i2c_data.statusbyte2 & (1 << STATUS_BIT2_GEAR_D)) main_oled.selector_mode = 4;

  main_oled.Left_Strength = (bool(i2c_data.statusbyte2 & (1 << STATUS_BIT2_CURR1_L)) << 1) | (bool(i2c_data.statusbyte2 & (1 << STATUS_BIT2_CURR2_L)));
  
  main_oled.Right_Strength = (bool(i2c_data.statusbyte2 & (1 << STATUS_BIT2_CURR1_R)) << 1) | (bool(i2c_data.statusbyte2 & (1 << STATUS_BIT2_CURR2_R)));

  main_oled.GP_Temp = (int16_t) i2c_data.GP_temp;
  main_oled.GP_Speed = (uint16_t) i2c_data.GP_Speed;
}

void setup() {

  TCCR2A = 0; 
  TCCR2B = (1 << CS22)|(1 << CS21)|(1 << CS20); //1024 divider
		
	TCNT2 = WAIT_100HZ;
		
	TIFR2 = (1<<TOV2);
	TIMSK2 = (1<<TOIE2);

  DDRD = LED1 | LED2;
  
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = (1<<WDE) | (1<<WDP2) | (1<<WDP1);

  init_switches();
  sei();

  EEPROM.get(0, lang_eeprom);

  if (lang_eeprom > 1)
  {
    lang_eeprom = pgm_read_byte(&menus[PARAMETERS-1].def);;
    EEPROM.put(0, lang_eeprom);
  }

  Wire.setClock(100000);
  Wire.begin(TWI_SLAVEADDR); 
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);


  oled.init();
  oled.clear(); 
  oled.update();

  Init_i2c_data();

  InitMainScreenSet();

  MainScreen(&main_oled);
  
  menu.onChange(onItemChange, true);
  menu.onPrintOverride(onItemPrintOverride); 

  BuildMenu();
  
  menu_visible = false;
  menu.showMenu(menu_visible); 


}

void onItemChange(const int index, const void* val, const byte valType) {
  
  if (valType == VAL_ACTION) {
    if (index == 0) {
      menu_visible = false;
      menu.showMenu(menu_visible);
      MainScreen(&main_oled);
    }
  }


}

boolean onItemPrintOverride(const int index, const void* val, const byte valType) {

  if (index > 0)
  if (!((index >=6) && (menu_page == 5)))
      return false;
      

  return true;
} 

void init_switches(){
	
	Key1.outstate=false;
	Key1.long_outstate = false;
	Key1.curr_scan=0;
	Key1.scanrate = KEY_PUSH_DELAY;
	Key1.long_scanrate = KEY_LONGPUSH_DELAY;
	Key1.prev = false;
	Key1.port = &PINC;
	Key1.pin = KEY1;
  Key1.lock = 0;
	
	Key2.outstate=false;
	Key2.long_outstate = false;
	Key2.curr_scan=0;
	Key2.scanrate=KEY_PUSH_DELAY;
  Key2.long_scanrate = KEY_LONGPUSH_DELAY;
	Key2.prev = false;
	Key2.port = &PINC;
	Key2.pin = KEY2;
  Key2.lock = 0;

  Key3.outstate=false;
	Key3.long_outstate = false;
	Key3.curr_scan=0;
	Key3.scanrate=KEY_PUSH_DELAY;
  Key3.long_scanrate = KEY_LONGPUSH_DELAY;
	Key3.prev = false;
	Key3.port = &PINC;
	Key3.pin = KEY3;
  Key3.lock = 0;
}



void menu_page_select(int index)
{
  switch (index) {
    
    case 0 ... 5:   menu_page = 1;
                    break;
    case 6 ... 11:  menu_page = 2;
                    break;
    case 12 ... 17: menu_page = 3;
                    break;
    case 18 ... 23: menu_page = 4;
                    break;
    case 24 ... 29: menu_page = 5;
                    break;        
  }

}

void DownButton()
{
  if (menu_visible)
  {
  
        if (menu_index < PARAMETERS+4) //PARAMETERS+5-1. 5 EXIT titles
        {
          if (!Setup_param)
          {
            menu_index++;          
          }
        }
            
        else
        {
          if (!Setup_param)
               menu_index = 0;
        }

        menu_page_select(menu_index);
      
      
        UpdateTextBuf();
        //UpdateParams();
        menu.refresh();

       
        if (menu_index == 0)
        {
          if (!Setup_param)
               menu.gotoIndex_(0); 
          else
               menu.selectPrev(0);



        } else  
        if (!Setup_param)
            menu.selectNext(0);
        else 
            menu.selectPrev(0);
 
  }
}

void UpButton()
{
  if (menu_visible)
  {    
        if (menu_index == 0) 
        {
           menu_index = PARAMETERS+4;    
        }
        else
        if (menu_index > 0) 
        {
          if (!Setup_param)
          {
            menu_index--;
          }
        }

        menu_page_select(menu_index);    
        UpdateTextBuf();
        //UpdateParams();
        menu.refresh();

        if (menu_index == PARAMETERS+4)
        {
          if (!Setup_param)
          {
            menu.gotoIndex_(5); 
          } else
           menu.selectNext(0);


        } else  
        {
          if (!Setup_param)
            menu.selectPrev(0);
          else
            menu.selectNext(0);
        }

        
  }

}

uint8_t getSelectedMenu()
{
  uint8_t selected = 0;
  
  if (menu_page > 0)
  selected = menu.getSelectedItemIndex()+(menu_page-1)*5-1; 
  if ((selected < 0) || ( selected >= PARAMETERS)) selected = 0;
  
  return selected;
}
  

void loop() {

  int i,j = 0;
  int8_t addr = 0;


  asm("wdr"); 

    if (menu_index < 0) menu_index = 0;

		if (Key3.outstate && (!Key3.lock)) { // down button
			Key3.lock = 1;

      DownButton();      

    } else

    if (Key3.long_outstate) { 
      DownButton();
    }

    if (Key1.outstate && (!Key1.lock)) { // up button
			Key1.lock = 1;
      UpButton();

    } else
    
    if (Key1.long_outstate) {
       UpButton();
    }

    if (Key2.outstate && (!Key2.lock)) {   // enter button
			Key2.lock = 1;

      

      if (!menu_visible) {
          oled.setScale(1);
          menu_visible = true;
          //menu_index = 0;
          menu.showMenu(menu_visible);
      } else 
      {
        

       
        if (menu.getSelectedItemIndex() == 0) menu.toggleChangeSelected();
        else
        {
          selected_menu = getSelectedMenu();

          if (pgm_read_byte(&menus[selected_menu].writable))
          { 
              if (!menu.oledMenuItems[menu.getSelectedItemIndex()].isChange)
                  old_param_value = params_buf[menu.getSelectedItemIndex()-1];
              menu.toggleChangeSelected();
          }

        }

        Setup_param = menu.oledMenuItems[menu.getSelectedItemIndex()].isChange;

  
        
        if (!Setup_param)
        { 
          addr = menu.getSelectedItemIndex()-1;
          if (addr < 0 ) addr = 0;

          if (old_param_value != params_buf[addr]) {

            if ((menu_page == 5) && (addr == (PARAMETERS-1)%5)){ //23 parameter
              lang_eeprom = params_buf[addr];
              EEPROM.put(0, lang_eeprom);

            } else
            {
              i2c_data.paramfromdisp = params_buf[addr]; //oter parameters
              i2c_data.paramfromdisp_addr = getSelectedMenu()+1;
            }
          } 
          else
          {
          
            i2c_data.paramfromdisp = 0;
            i2c_data.paramfromdisp_addr = 0;            
          }
        }

        
      }
      
    } else
    
    if (Key2.long_outstate) {
       // do nothing
    }

    if (!Key1.outstate) Key1.lock = 0;
    if (!Key2.outstate) Key2.lock = 0; 
    if (!Key3.outstate) Key3.lock = 0;

      
    EVERY_N_MILLISECONDS(300){

      if (!menu_visible)  
      { 
        
        MainScreen(&main_oled);
      } else
      if (!Setup_param) {
        UpdateParams();
        menu.refresh();
      } else
      {

      }
    }
      
    EVERY_N_MILLISECONDS(200){
      Update_status();
    }

}
