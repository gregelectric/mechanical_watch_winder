// Department of Fisheries and Oceans Electro-Anaesthesia System
#include <Controllino.h>
#include "ModbusRtu.h"
#include <SimpleRotary.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include <ButtonDebounce.h>
#include "Fsm.h"
#include <Arduino.h>
#include "ST7036.h"
#include <Wire.h>
#include <pca9633.h>
#include <Adafruit_SleepyDog.h>

// Global Constants //////////////////////////////////////////////////////////////

// Uncomment MODBUS_MASTER to compile for SLAVE target
#define MODBUS_MASTER    

#define WATCHDOG_TIMER     4000 // 4 second timeout 

#define TANK_A             0 
#define TANK_B             1
#define POWER_STUN         0
#define POWER_EUTHANIZE    1

// Digital Output Defines
#define PWM_OUT             2   // CONTROLLINO_D0
#define INDICATOR_READY     3   // CONTROLLINO_D1
#define INDICATOR_A         4   // CONTROLLINO_D2
#define INDICATOR_B         5   // CONTROLLINO_D3
#define INDICATOR_STUN      6   // CONTROLLINO_D4
#define INDICATOR_EUTHANIZE 7   // CONTROLLINO_D5
#define INDICATOR_ACTIVE    8   // CONTROLLINO_D6
#define RELAY_A             9   // CONTROLLINO_D7
#define RELAY_B             10  // CONTROLLINO_D8
#define RELAY_CROWBAR       11  // CONTROLLINO_D9
#define BUZZER              12  // CONTROLLINO_D10
#define SPARE               13  // CONTROLLINO_D11

// Digital Input Defines
#define ENCODER_BUTTON      54  // CONTROLLINO_A0
#define ENCODER_B           55  // CONTROLLINO_A1
#define ENCODER_A           56  // CONTROLLINO_A2
#define BUTTON_ENABLE       57  // CONTROLLINO_A3
#define BUTTON_DISABLE      58  // CONTROLLINO_A4
#define SWITCH_A            59  // CONTROLLINO_A5
#define SWITCH_B            60  // CONTROLLINO_A6
#define BUTTON_START        61  // CONTROLLINO_A7
#define BUTTON_STOP         62  // CONTROLLINO_A8
#define BUTTON_E_STOP       63  // CONTROLLINO_A9
#define ALARM_1             18  // CONTROLLINO_IN0
#define ALARM_2             19  // CONTROLLINO_IN1
#define A10                 64  // CONTROLLINO_A10
#define A11                 65  // CONTROLLINO_A11
#define A12                 66  // CONTROLLINO_A12
#define A13                 67  // CONTROLLINO_A13

// Timer Defines
#define TIMER_LCD         250        // 250ms
#define TIMER_ONE_SECOND  1000       // 1 second
#define EEPROMCOUNTER     5          // 5 seconds

#define LCD_SPARE        4
#define LCD_RED          3
#define LCD_GREEN        2
#define LCD_BLUE         1

// For any Modbus slave devices are reserved addresses in the range from 1 to 247.
// Important note only address 0 is reserved for a Modbus master device!
#define MasterModbusAdd  0
#define SlaveModbusAdd  1
// For MAXI and MEGA RS485 is reserved UART Serial3.
#define RS485Serial     3
#define Modbus_timeout  1000
#define Modbus_wait     10
#define COMM_ERROR_LIMIT  2

#ifdef MODBUS_MASTER
Modbus ControllinoModbusMaster(MasterModbusAdd, RS485Serial, 0);
#else
Modbus ControllinoModbusSlave(SlaveModbusAdd, RS485Serial, 0);
#endif // MODBUS_MASTER
modbus_t ModbusQuery[2];
uint8_t myState; // machine state
uint8_t currentQuery; // pointer to message query
uint16_t comm_counter_last = 0;
uint16_t comm_counter_current = 0;
unsigned long WaitingTime;
uint8_t comm_error_count = COMM_ERROR_LIMIT;
#define MOD_BUFF  32  // Double NUM_COILS below, holds both TX and RX
#define NUM_COILS  16
#define SLAVE_ADD  16
// ModbusRegisters[modbus_slave_stun_voltage] - Slave Voltage to Master
// ModbusRegisters[modbus_slave_stun_current] - Slave Current to Master
// ModbusRegisters[modbus_slave_stun_time] - Slave Time to Master
// ModbusRegisters[modbus_slave_euth_voltage] - Slave Voltage to Master
// ModbusRegisters[modbus_slave_euth_current] - Slave Current to Master
// ModbusRegisters[modbus_slave_euth_time] - Slave Time to Master
// ModbusRegisters[modbus_slave_ramp_time] - Slave Voltage Ramp Time to Master
// ModbusRegisters[modbus_slave_variable] - Slave Variable to Master
// ModbusRegisters[modbus_slave_in_cnt] - Number of incoming messages.
// ModbusRegisters[modbus_slave_out_cnt] - Number of outgoing messages.
// ModbusRegisters[modbus_slave_buttons] - Slave buttons to Master
// ModbusRegisters[modbus_slave_spare] - Slave Spare
// ModbusRegisters[modbus_slave_spare2] - Slave Spare
// ModbusRegisters[modbus_slave_spare3] - Slave Spare
// ModbusRegisters[modbus_slave_spare4] - Slave Spare
// ModbusRegisters[modbus_slave_spare5] - Slave Spare

// ModbusRegisters[modbus_master_state] - Master State
// ModbusRegisters[modbus_master_active_timer] - Master Timer
// ModbusRegisters[modbus_master_voltage] - Master Voltage
// ModbusRegisters[modbus_master_current] - Master Current
// ModbusRegisters[modbus_master_watts] - Master Watts
// ModbusRegisters[modbus_master_stun_voltage] - Master Voltage
// ModbusRegisters[modbus_master_stun_current] - Master Current
// ModbusRegisters[modbus_master_stun_time] - Master Time
// ModbusRegisters[modbus_master_euth_voltage] - Master Voltage
// ModbusRegisters[modbus_master_euth_current] - Master Current
// ModbusRegisters[modbus_master_euth_time] - Master Time
// ModbusRegisters[modbus_master_ramp_time] - Master Votlage Ramp Time
// ModbusRegisters[modbus_master_tank] - Master Tank
// ModbusRegisters[modbus_master_power] - Master Power Setting
// ModbusRegisters[modbus_master_buttons] - Master Buttons Ack
// ModbusRegisters[modbus_master_leds] - Master Led

uint16_t ModbusRegisters[MOD_BUFF];
enum
{
  // Slave Variables
	modbus_slave_stun_voltage,
	modbus_slave_stun_current,
	modbus_slave_stun_time,
	modbus_slave_euth_voltage,
	modbus_slave_euth_current,
	modbus_slave_euth_time,
  modbus_slave_ramp_time,
	modbus_slave_variable,
	modbus_slave_in_cnt,
	modbus_slave_out_cnt,
	modbus_slave_buttons,
	modbus_slave_spare,
	modbus_slave_spare2,
	modbus_slave_spare3,
	modbus_slave_spare4,
  modbus_slave_spare5,
  //Master Variables
	modbus_master_state,
	modbus_master_active_timer,
	modbus_master_voltage,
	modbus_master_current,
  modbus_master_watts,
	modbus_master_stun_voltage,
	modbus_master_stun_current,
	modbus_master_stun_time,
	modbus_master_euth_voltage,
	modbus_master_euth_current,
	modbus_master_euth_time,
  modbus_master_ramp_time,
	modbus_master_tank,
	modbus_master_power,
	modbus_master_buttons,
	modbus_master_leds,
	modbus_max
};
enum
{
  modbus_button_min,
	modbus_button_start,
	modbus_button_stop,
	modbus_button_e_stop,
	modbus_button_enable,
	modbus_button_disable,
	modbus_button_tank,
	modbus_button_power,
	modbus_button_max
};

// Ethernet (Telnet)
EthernetClient client; 
byte clientMAC[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
IPAddress clientIp (192, 168, 0, 100);
IPAddress power_supply_ip (192, 168, 0, 101);

#define IN_BUF_SIZE 25
char in_buf[IN_BUF_SIZE] = {'\0'};
char out_buf[IN_BUF_SIZE] = {'\0'};
char volts[IN_BUF_SIZE] = {'\0'};
char volts_decimal[IN_BUF_SIZE] = {'\0'};
char amps[IN_BUF_SIZE] = {'\0'};
char amps_decimal[IN_BUF_SIZE] = {'\0'};
uint8_t in_buf_index = 0;

// Note that \r is required to correctly terminate a string to the power supply
const char *get_supply_protection = "PROT?\r";
const char *clr_prot = "PROT:CLE\r";
const char *get_supply_error = "ERROR?\r";
const char *set_current_cmd = "ISET ";
const char *set_volt_cmd = "VSET ";
const char *get_set_volt_cmd = "VSET?\r";
const char *get_set_cur_cmd = "ISET?\r";
const char *get_volt_on_cmd = "VOUT?\r";
const char *get_curr_on_cmd = "IOUT?\r";
const char *on_cmd = "OUT 1\r";
const char *off_cmd = "OUT 0\r"; 
const char *cls_cmd = "*CLS\r"; 
const char *rst_cmd = "*RST\r"; 
const char *idn_cmd = "*IDN?\r"; 
const char *carriage = "\r";
    
enum
{
   supply_query_volts,
   supply_read_volts,
   supply_query_amps,
   supply_read_amps,
   supply_vset,
   supply_iset,
   supply_send_control,
   supply_query_error,
   supply_query_protection,
   supply_read_error,
   supply_read_protection,
   supply_query_id,
   supply_read_id,   
   supply_max
};

enum
{
   var_stun_current,
   var_stun_voltage,
   var_stun_time,
   var_euth_current,
   var_euthanize_voltage,
   var_euthanize_time,
   var_ramp_time,
   var_max
};

enum
{
  PASS,
  FAIL
};

enum
{
  G_STATE_IDLE,
  G_STATE_ACTIVE,
  G_STATE_ESTOP,
  G_STATE_READY,
  G_STATE_ALARM_1,
  G_STATE_ALARM_2,
  G_STATE_POWER_ERROR,
  G_STATE_REMOTE_ERROR
};

// Eeprom Global Variables
uint8_t g_eepromCounter = 0;
uint8_t g_eepromCounterEnabled = 0;
int     g_eepromAddress = 0;

uint8_t g_active_timer = 0;
uint8_t g_lcd_timer = 0;
bool    g_pwm_toggle = 0;
uint16_t g_pwm_counter = 0;
int     g_second_count = 1;
int     g_milisecond_count = 0;
bool    g_power_up = 1;
uint8_t g_state = G_STATE_IDLE; 

float g_supply_volts = 0;
float g_supply_amps = 0;
float g_supply_watts = 0;
uint8_t g_power_supply_state = supply_query_volts;
uint16_t g_power_supply_timeout = 0;
uint8_t g_power_supply_timer = 0;
uint8_t g_power_supply_flag = 0;
uint8_t g_flag_remote_comm_failed = 0;
uint8_t g_flag_on_command_sent = 1; // Init to 1 to send a power off command upon reset
uint8_t g_remote_buttons = 0;

uint8_t g_stun_current = 0;
uint8_t g_euth_current = 0;
uint8_t g_stun_voltage = 0;
uint8_t g_stun_target_voltage = 0;
uint8_t g_stun_time = 0;
uint8_t g_euthanize_voltage = 0;
uint8_t g_euthanize_target_voltage = 0;
uint8_t g_euthanize_time = 0;
uint8_t g_ramp_time = 0;
uint8_t g_variable_to_set = var_max;

uint8_t g_test_pass_flag = 0;
uint8_t g_test_estop_fail_flag = 0;

uint8_t g_tank = 0;
uint8_t g_power = 0;
uint8_t g_leds = 0;

// LCD Display Instance
ST7036 lcd_display = ST7036 ( 2, 20, 0x78 );
// LCD Backlight Instance
PCA9633 rgbw; 
uint8_t g_colorR = 255;
uint8_t g_colorG = 255;
uint8_t g_colorB = 255;

// Rotary Encoder Global Variables
SimpleRotary rotary(ENCODER_A, ENCODER_B, ENCODER_BUTTON);
uint8_t g_ctr = 0;
int  g_lastDir = 0;

// FSM States
enum states
{
  ENCODER_BUTTON_PRESSED,
  ENABLE_BUTTON_PRESSED,  
  DISABLE_BUTTON_PRESSED, 
  STATE_SWITCH_A,        
  STATE_SWITCH_B,        
  START_BUTTON_PRESSED,   
  STOP_BUTTON_PRESSED,    
  ESTOP_BUTTON_PRESSED,
  ESTOP_BUTTON_RELEASED,   
  STATE_TEST_PASS,    
  STATE_TEST_FAIL,
  ALARM_1_ASSERTED,
  ALARM_1_DEASSERTED,
  ALARM_2_ASSERTED,
  ALARM_2_DEASSERTED,
  POWER_ERROR_ASSERTED,
  POWER_ERROR_DEASSERTED,
  REMOTE_ERROR_ASSERTED,
  REMOTE_ERROR_DEASSERTED,
  STATE_SYSTEM_RECOVER
};

// Global Functions //////////////////////////////////////////////////////////////

// Timer 1ms Period
void timer_one()
{
  rotary_encoder();
    
  // PWM out
  if(g_state == G_STATE_ACTIVE)
  {
    if(g_pwm_counter == 0)
    {
      digitalWrite(PWM_OUT, HIGH);  
    }    
    if(g_pwm_counter == 2)
    {
      digitalWrite(PWM_OUT, LOW);
    }
    if(g_pwm_counter == 20)
    {
      digitalWrite(PWM_OUT, HIGH);  
    }
    if(g_pwm_counter == 22)
    {
      digitalWrite(PWM_OUT, LOW);  
    }
    if(g_pwm_counter == 40)
    {
      digitalWrite(PWM_OUT, HIGH);
    }
    if(g_pwm_counter == 42)
    {
      digitalWrite(PWM_OUT, LOW);  
    }
    
    g_pwm_counter++;
    if(g_pwm_counter == 66)
    {
      // Start pulse train over again.
      g_pwm_counter = 0;  
    }
  }
  else
  {
    g_pwm_counter = 0;
    digitalWrite(PWM_OUT, LOW);
  }
  
  if(g_lcd_timer)
  {
    g_lcd_timer--;
  }    

  if(g_power_supply_timer)
  {
    g_power_supply_timer--;
  }  
  
  if(g_power_supply_timeout)
  {
    g_power_supply_timeout--;
  }
  
  // One Second Timer
  g_second_count ++;
  if(g_second_count == TIMER_ONE_SECOND)
  {    
    // Restart timer
    g_second_count = 0;

    if(g_active_timer > 0)
    {
      g_active_timer--;
    }    
    if(g_eepromCounter > 0)
    {
      g_eepromCounter--;
    } 
  }

  // Ramp the voltage 
  g_milisecond_count ++;
  if(g_milisecond_count >= g_ramp_time)
  { 
    // Restart timer
    g_milisecond_count = 0;

    // Ramp Voltage in Active State
    if(g_state == G_STATE_ACTIVE) 
    {
      if(g_euthanize_target_voltage < g_euthanize_voltage)
      {
        g_euthanize_target_voltage++;
      }
      if(g_stun_target_voltage < g_stun_voltage)
      {
        g_stun_target_voltage++;
      } 
    }  
  }

  // All other states Voltage is Immediate
  if(g_state == G_STATE_READY)
  {
    g_euthanize_target_voltage = 1;
    g_stun_target_voltage = 1;
  }  
  else if(g_state != G_STATE_ACTIVE) 
  {
    g_euthanize_target_voltage = g_euthanize_voltage;
    g_stun_target_voltage = g_stun_voltage;
  }
}


State state_idle(on_idle_enter, NULL, &on_idle_exit);

State state_ready(on_ready_enter, NULL, &on_ready_exit);

State state_active(on_active_enter, NULL, &on_active_exit);

State state_estop(on_estop_enter, NULL, &on_estop_exit);

State state_alarm_1(on_alarm_1_enter, NULL, &on_alarm_1_exit);

State state_alarm_2(on_alarm_2_enter, NULL, &on_alarm_2_exit);

State state_power_error(on_power_error_enter, NULL, &on_power_error_exit);

State state_remote_error(on_remote_error_enter, NULL, &on_remote_error_exit);

Fsm fsm(&state_idle);

// Transition callback functions
void on_trans_switchA()
{
    g_tank = !g_tank;
    Serial.println("Toggle Tank Selection");       
}
void on_trans_switchB()
{
    g_power = !g_power;
    Serial.println("Toggle Power Selection");
}
void on_trans_idle_ready()
{
    Serial.println("Transision State IDLE to READY");
}
void on_trans_ready_idle()
{
    Serial.println("Transision State READY to IDLE");
}
void on_trans_ready_active()
{
    Serial.println("Transision State READY to ACTIVE");
}
void on_trans_active_ready()
{
    Serial.println("Transision State ACTIVE to READY");
}
void on_trans_active_idle()
{
    Serial.println("Transision State ACTIVE to IDLE");
}
void on_trans_idle_estop()
{
    Serial.println("Transision State IDLE to ESTOP");
}
void on_trans_ready_estop()
{
    Serial.println("Transision State READY to ESTOP");
}
void on_trans_active_estop()
{
    Serial.println("Transision State ACTIVE to ESTOP");
}
void on_trans_estop_idle()
{
    Serial.println("Transision State ESTOP to IDLE");
}
void on_trans_alarm_1_idle()
{
    Serial.println("Transision State ALARM 1 to IDLE");
}
void on_trans_alarm_2_idle()
{
    Serial.println("Transision State ALARM 2 to IDLE");
}
void on_trans_power_error_idle()
{
    Serial.println("Transision State POWER ERROR to IDLE");
}
void on_trans_active_power_error()
{
    Serial.println("Transision State ACTIVE to POWER ERROR");
}
void on_trans_idle_power_error()
{
    Serial.println("Transision State IDLE to POWER ERROR");
}
void on_trans_ready_power_error()
{
    Serial.println("Transision State READY to POWER ERROR");
}
void on_trans_remote_error_idle()
{
    Serial.println("Transision State REMOTE ERROR to IDLE");
}
void on_trans_active_remote_error()
{
    Serial.println("Transision State ACTIVE to REMOTE ERROR");
}
void on_trans_idle_remote_error()
{
    Serial.println("Transision State IDLE to REMOTE ERROR");
}
void on_trans_ready_remote_error()
{
    Serial.println("Transision State READY to REMOTE ERROR");
}

// IDLE State
void on_idle_enter()
{
  Serial.println("Entering IDLE");
  g_state = G_STATE_IDLE;
}

void on_idle_exit()
{
  Serial.println("Exiting IDLE");
  //g_state = G_STATE_IDLE;
}

// READY State
void on_ready_enter()
{
  Serial.println("Entering READY");
  g_state = G_STATE_READY;
  // Start at Zero Volts in Ready State
  g_euthanize_target_voltage = 0;
  g_stun_target_voltage = 0; 
}

void on_ready_exit()
{
  Serial.println("Exiting READY");
}

// ACTIVE State
void on_active_enter()
{
  Serial.println("Entering ACTIVE");
  g_state = G_STATE_ACTIVE;

  if(g_power == POWER_STUN)
  {
    g_active_timer = g_stun_time;
  }
  else
  {
    g_active_timer = g_euthanize_time;
  }
}

void on_active_exit()
{
  g_variable_to_set = var_max;
  Serial.println("Exiting ACTIVE");
  g_active_timer = 0;
}

// E-Stop State
void on_estop_enter()
{
  Serial.println("Entering ESTOP");
  g_state = G_STATE_ESTOP;
  g_active_timer = 0;
}

void on_estop_exit()
{
  Serial.println("Exiting ESTOP");
}

// Alarm 1 State
void on_alarm_1_enter()
{
  Serial.println("Entering ALARM_1");
  g_state = G_STATE_ALARM_1;
  g_active_timer = 0;
}

void on_alarm_1_exit()
{
  Serial.println("Exiting ALARM_1");
  g_state = G_STATE_IDLE;
}

// Alarm 2 State
void on_alarm_2_enter()
{
  Serial.println("Entering ALARM_2");
  g_state = G_STATE_ALARM_2;
  g_active_timer = 0;
}

void on_alarm_2_exit()
{
  Serial.println("Exiting ALARM_2");
  g_state = G_STATE_IDLE;
}

// Power Supply Error State
void on_power_error_enter()
{
  Serial.println("Entering Power Supply Error");
  g_state = G_STATE_POWER_ERROR;
  g_active_timer = 0;
}

void on_power_error_exit()
{
  Serial.println("Exiting Power Supply Error");
  g_state = G_STATE_IDLE;
}

// Power Remote Control Error State
void on_remote_error_enter()
{
  Serial.println("Entering Remote Control Error");
  g_state = G_STATE_REMOTE_ERROR;
  g_active_timer = 0;
}

void on_remote_error_exit()
{
  Serial.println("Exiting Remote Control Error");
  g_state = G_STATE_IDLE;
}

// Button states and callbacks
//
//
ButtonDebounce encoder_button(ENCODER_BUTTON, 100);
void encoder_button_changed(int state)
{
  if(state)
  {
    g_variable_to_set++;
    if(g_variable_to_set > (var_max-1))
    {
      g_variable_to_set = 0;
    }
    Serial.println("ENCODER_BUTTON");
    Serial.println(g_variable_to_set);
    
    switch (g_variable_to_set)
    {
      case var_stun_current:
        g_ctr = g_stun_current/10;
        break;      
      case var_stun_voltage:
        g_ctr = g_stun_voltage;
        break;         
      case var_stun_time:
        g_ctr = g_stun_time;  
        break;  
      case var_euth_current:
        g_ctr = g_euth_current/10;
        break;              
      case var_euthanize_voltage:
        g_ctr = g_euthanize_voltage;  
        break;       
      case var_euthanize_time:
        g_ctr = g_euthanize_time;       
        break;  
      case var_ramp_time:
        g_ctr = g_ramp_time;       
        break;          
      case var_max:      
        break;         
      default:
        break;
    }
  }
}

ButtonDebounce button_enable(BUTTON_ENABLE, 100);
void button_enable_changed(int state)
{
  if(state)
  {
    fsm.trigger(ENABLE_BUTTON_PRESSED);
  }
}

ButtonDebounce button_disable(BUTTON_DISABLE, 100);
void button_disable_changed(int state)
{
  if(state)
  {
    fsm.trigger(DISABLE_BUTTON_PRESSED);
  }
}

ButtonDebounce switch_a(SWITCH_A, 100);
void switch_a_changed(int state)
{ 
  if(state)
  {	  
	  fsm.trigger(STATE_SWITCH_A);
  }
}

ButtonDebounce switch_b(SWITCH_B, 100);
void switch_b_changed(int state)
{
  if(state)
  {	  
	  fsm.trigger(STATE_SWITCH_B);
  }
}

ButtonDebounce button_start(BUTTON_START, 100);
void button_start_changed(int state)
{
  if(state)
  {
    fsm.trigger(START_BUTTON_PRESSED);
  }
}

ButtonDebounce button_stop(BUTTON_STOP, 100);
void button_stop_changed(int state)
{
  if(state)
  {
    fsm.trigger(STOP_BUTTON_PRESSED);
  }
}

ButtonDebounce button_estop(BUTTON_E_STOP, 100);
void button_estop_changed(int state)
{
  // The E-Stop button is normally closed and thus opposite to the other buttons
  if(state)
  {
    Serial.println("ESTOP_BUTTON_PRESSED");
    fsm.trigger(ESTOP_BUTTON_PRESSED);
  }
  else
  {
    Serial.println("ESTOP_BUTTON_RELEASED");
    fsm.trigger(ESTOP_BUTTON_RELEASED);
  }
}

ButtonDebounce alarm_1(ALARM_1, 100);
void alarm_1_changed(int state)
{
  if(state)
  {
    Serial.println("ALARM_1_ASSERTED");
    fsm.trigger(ALARM_1_ASSERTED);
  }
  else
  {
    Serial.println("ALARM_1_DEASSERTED");
    fsm.trigger(ALARM_1_DEASSERTED);
  }
}

ButtonDebounce alarm_2(ALARM_2, 100);
void alarm_2_changed(int state)
{
  if(state)
  {
    Serial.println("ALARM_2_ASSERTED");
    fsm.trigger(ALARM_2_ASSERTED);
  }
  else
  {
    Serial.println("ALARM_2_DEASSERTED");
    fsm.trigger(ALARM_2_DEASSERTED);
  }
}

void rotary_encoder()
{
	int rDir = rotary.rotate();
	int rBtn = rotary.push();
	int rLBtn = rotary.pushLong(1000);
	// Check direction
	if ( rDir == 2  )
	{
		// CW
		if(g_ctr < 255)
		{
			g_ctr++;
		}
		
		g_lastDir = rDir;
	}
	if ( rDir == 1 )
	{
		// CCW
		if(g_ctr > 0)
		{
			g_ctr--;
		}

		g_lastDir = rDir;
	}
	// Suppress power up button event
	if(g_power_up)
	{
		g_power_up = 0;
	}
	else if( rDir == 1 || rDir == 2 || rBtn == 1 || rLBtn == 1 )
	{
    // Trigger a write to EEPROM after a delay
		g_eepromCounter = EEPROMCOUNTER;
		g_eepromCounterEnabled = 1;
		// This lets us the know the Master is in control
		ModbusRegisters[modbus_slave_variable] = 0;       
		// Only allow voltage or time change while in IDLE or READY states 
		if(g_state == G_STATE_IDLE)
		{
			switch (g_variable_to_set)
			{
				case var_stun_current:
          if(g_ctr > 25)
          {
            g_ctr = 25;
          }
          else if(g_ctr == 0)
          {
            g_ctr = 1;
          }	        
          g_stun_current = g_ctr*10;
          break;
				case var_stun_voltage:
          if(g_ctr > 60)
          {
            g_ctr = 60;
          }
          else if(g_ctr == 0)
          {
            g_ctr = 1;
          }
          g_stun_voltage = g_ctr;
          break;
				case var_stun_time:
          g_stun_time = g_ctr;
          break;
				case var_euth_current:
          if(g_ctr > 25)
          {
            g_ctr = 25;
          }
          else if(g_ctr == 0)
          {
            g_ctr = 1;
          }	        
          g_euth_current = g_ctr*10;
          break;
				case var_euthanize_voltage:
          // Maximum voltage is 600V, minimum is 10V
          if(g_ctr > 60)
          {
            g_ctr = 60;
          }
          else if(g_ctr == 0)
          {
            g_ctr = 1;
          }
          g_euthanize_voltage = g_ctr;
				break;
				case var_euthanize_time:
          g_euthanize_time = g_ctr;
          break;
				case var_ramp_time:
          // Minimum Ramp Time 50ms
          if(g_ctr < 50)
          {
            g_ctr = 50;
          }        
          g_ramp_time = g_ctr;
          break;          
				case var_max:
          // Do nothing
          break;
				default:
				  break;
			}
		}
	}
}

void eeprom()
{
  // Preserve EEPROM by only writing 5 seconds after the last change
  if((g_eepromCounterEnabled && g_eepromCounter == 0))
  {   
    Serial.println("Saved Variables to EEPROM");
    g_eepromCounterEnabled = 0;
   
    EEPROM.write(var_stun_current, g_stun_current);
    EEPROM.write(var_stun_voltage, g_stun_voltage);
    EEPROM.write(var_stun_time, g_stun_time);
    EEPROM.write(var_euth_current, g_euth_current);
    EEPROM.write(var_euthanize_voltage, g_euthanize_voltage);
    EEPROM.write(var_euthanize_time, g_euthanize_time);
    EEPROM.write(var_ramp_time, g_ramp_time);  

    // After updating EEPROM display non-configurable screen
    g_variable_to_set = var_max;     
  }
}
void lcd_display_update()
{
  // Update LCD once per second
  if(g_lcd_timer == 0)
  {
    g_lcd_timer = TIMER_LCD;
    
    if(g_state == G_STATE_ESTOP)
    {
      lcd_display.setCursor(0,0); 
      //                "xxxxxxxxxxxxxxxxxxxx" 
      lcd_display.print("EMERGENCY STOP!     ");
      g_colorR = 0;
      g_colorG = 255;
      g_colorB = 0;
    }
    else if(g_state == G_STATE_ALARM_1)
    {
      lcd_display.setCursor(0,0); 
      //                "xxxxxxxxxxxxxxxxxxxx" 
      lcd_display.print("ISOLATION 1 FAILURE!");
      g_colorR = 0;
      g_colorG = 255;
      g_colorB = 0;
    }   
    else if(g_state == G_STATE_ALARM_2)
    {
      lcd_display.setCursor(0,0); 
      //                "xxxxxxxxxxxxxxxxxxxx" 
      lcd_display.print("ISOLATION 2 FAILURE!");
      g_colorR = 0;
      g_colorG = 255;
      g_colorB = 0;
    } 
    else if(g_state == G_STATE_POWER_ERROR)
    {
      lcd_display.setCursor(0,0); 
      //                "xxxxxxxxxxxxxxxxxxxx" 
      lcd_display.print("POWER SUPPLY ERROR! ");
      g_colorR = 0;
      g_colorG = 255;
      g_colorB = 0;
    } 
    else if(g_state == G_STATE_REMOTE_ERROR)
    {
      lcd_display.setCursor(0,0); 
      //                "xxxxxxxxxxxxxxxxxxxx" 
      lcd_display.print("REMOTE CONTROL ERROR");
      g_colorR = 0;
      g_colorG = 255;
      g_colorB = 0;
    }              
    else if(g_state == G_STATE_READY)
    {
      g_colorR = 0;
      g_colorG = 150;
      g_colorB = 255;
      lcd_display.setCursor(0,0);
      if(g_power)
      {
        lcd_display.print("Euth ");
        if(g_tank == TANK_A)
        lcd_display.print("Tank 1 ");
        else
        lcd_display.print("Tank 2 ");
        lcd_display.print(g_euthanize_time, DEC);
        lcd_display.print("s                 ");
      }
      else
      {
        lcd_display.print("Stun ");
        if(g_tank == TANK_A)
        lcd_display.print("Tank 1 ");
        else
        lcd_display.print("Tank 2 ");  
        lcd_display.print(g_stun_time, DEC);      
        lcd_display.print("s                 ");
      }        
    }  
    else if(g_state == G_STATE_ACTIVE)
    {
      g_colorR = 0;
      g_colorG = 255;
      g_colorB = 255;
      lcd_display.setCursor(0,0);
      if(g_power)
      {
        lcd_display.print("Euthanizing ");
        if(g_tank == TANK_A)
        lcd_display.print("1 ");
        else
        lcd_display.print("2 ");          
        lcd_display.print(g_active_timer);
        lcd_display.print("s                 ");
      }
      else
      {
        lcd_display.print("Stunning ");
        if(g_tank == TANK_A)
        lcd_display.print("1 ");
        else
        lcd_display.print("2 ");          
        lcd_display.print(g_active_timer);
        lcd_display.print("s                ");
      }             
    }
    else if(g_state == G_STATE_IDLE)
    {
      g_colorR = 255;
      g_colorG = 0;
      g_colorB = 255;
      lcd_display.setCursor(0,0);     
      switch (g_variable_to_set)
      { 
        case var_max:
          //                "xxxxxxxxxxxxxxxxxxxx"  
          lcd_display.print("Press to Configure->");      
          break;
        case var_stun_current:
          lcd_display.print("Stun Amps: ");
          lcd_display.print(g_stun_current/100, DEC);
          lcd_display.print('.');
          lcd_display.print(g_stun_current%100, DEC);
          lcd_display.print("                  ");
          break;             
        case var_stun_voltage:
          lcd_display.print("Stun Volts: ");
          lcd_display.print(g_stun_voltage*10, DEC);
          lcd_display.print("                  ");
          break;                      
        case var_stun_time: 
          lcd_display.print("Stun Seconds: ");
          lcd_display.print(g_stun_time, DEC); 
          lcd_display.print("                  ");
          break;   
        case var_euth_current:
          lcd_display.print("Euth Amps: ");
          lcd_display.print(g_euth_current/100, DEC);
          lcd_display.print('.');
          lcd_display.print(g_euth_current%100, DEC);
          lcd_display.print("                  ");
          break;              
        case var_euthanize_voltage:
          lcd_display.print("Euth Volts: ");
          lcd_display.print(g_euthanize_voltage*10, DEC);
          lcd_display.print("                  "); 
          break;         
        case var_euthanize_time: 
          lcd_display.print("Euth Seconds: ");
          lcd_display.print(g_euthanize_time, DEC); 
          lcd_display.print("                  ");      
          break;   
        case var_ramp_time: 
          lcd_display.print("Ramp mSeconds: ");
          lcd_display.print(g_ramp_time, DEC); 
          lcd_display.print("                  ");      
          break;            
        default:
          break;
      }
    }
    //rgbw.setrgbw(0, g_colorB, g_colorG, g_colorR);
    rgbw.setpwm(3,g_colorR); 
    rgbw.setpwm(2,g_colorG); 
    rgbw.setpwm(1,g_colorB); 
  
    // Display Real-time Power Supply Voltage/Current/Power on line 2 of the LCD
    lcd_display.setCursor(1,0);  
    lcd_display.print(g_supply_volts);
    lcd_display.print("V ");
    lcd_display.print(g_supply_amps);
    lcd_display.print("A ");
    g_supply_watts = g_supply_volts*g_supply_amps;
    lcd_display.print(g_supply_watts, 1);
    lcd_display.print("W   "); 
  } 
}

void buttons()
{
  encoder_button.update();
  button_enable.update();
  button_disable.update();
  switch_a.update();
  switch_b.update();
  button_start.update();
  button_stop.update();
  button_estop.update(); 
  alarm_1.update();
  alarm_2.update();
}

void lights()
{  
	// Update LED Status Indicators
	if((g_state == G_STATE_READY) || (g_state == G_STATE_ACTIVE))
	{
		digitalWrite(INDICATOR_READY, HIGH);
	}
	else
	{
		digitalWrite(INDICATOR_READY, LOW);
	}
	
	if(g_state == G_STATE_ACTIVE)
	{
		digitalWrite(INDICATOR_ACTIVE, HIGH);
	}
	else
	{
		digitalWrite(INDICATOR_ACTIVE, LOW);
	}

  // Relays turn on when ready and remain on till idle.
  // Avoid switching relays while voltage is applied.
  if((g_state == G_STATE_ACTIVE) || (g_state == G_STATE_READY))
  {
    if(g_tank)
    {
      digitalWrite(RELAY_A, LOW);
      digitalWrite(RELAY_B, HIGH);
    }
    else
    {
      digitalWrite(RELAY_A, HIGH);
      digitalWrite(RELAY_B, LOW);
    }
  }
  else
  {
    digitalWrite(RELAY_A, LOW);
    digitalWrite(RELAY_B, LOW);
  }
  // LED Tank Indicators
  if(g_tank)
  {
    digitalWrite(INDICATOR_A, LOW);
    digitalWrite(INDICATOR_B, HIGH);
  }
  else
  {
    digitalWrite(INDICATOR_A, HIGH);
    digitalWrite(INDICATOR_B, LOW);
  }
  // LED Power Output Indicator
  if(g_power)
  {
    digitalWrite(INDICATOR_EUTHANIZE, HIGH);
    digitalWrite(INDICATOR_STUN, LOW);
  }
  else
  {
    digitalWrite(INDICATOR_EUTHANIZE, LOW);
    digitalWrite(INDICATOR_STUN, HIGH);
  }
}

void state_machine()
{
  fsm.run_machine();

  // Turn Power Supply off when timer expires
  if((g_active_timer == 0) && (g_state == G_STATE_ACTIVE))
  {
    fsm.trigger(STOP_BUTTON_PRESSED);
  }
  
}

void power_supply()
{
  char c;
  
  if(!client.connected())
  {
    client.stop();
    if (client.connect(power_supply_ip, 5024)) 
    {
      Serial.println("Power Supply Re-connected");
      g_power_supply_state = supply_query_volts;
      fsm.trigger(POWER_ERROR_DEASSERTED);
    } 
    else 
    {
      // if you didn't get a connection to the server:
      Serial.println("Power Supply Re-connection Failed");
      fsm.trigger(POWER_ERROR_ASSERTED);
    } 
  }
  
  // Communicate with Power supply
  if (client.connected()) 
  {
    // Receive a character from power supply
    if (client.available()) 
    {
      c = client.read();
      //Serial.print(c);
      
      // Character buffer
      if(in_buf_index < IN_BUF_SIZE)
      {
          if((c == '>'))
          {
            // End of line
            in_buf[in_buf_index] = '\0';
            in_buf_index = 0;
            //Serial.print("in_buf: ");
            //Serial.println(in_buf);
          }
          else
          {  
            if(( c > '-' ) && ( c < ':' ))
            {     
              // Write char to buffer
              in_buf[in_buf_index++] = c;
            }
          }
      }
      else
      {
        // Buffer overflow, restart buffer index
        in_buf_index = 0;
        in_buf[in_buf_index] = '\0';
        Serial.println("Power Supply Buffer Overflow");
      }
    }

    // Send commands to power supply
    if(g_power_supply_timer == 0)
    {
      g_power_supply_timer = 100;            
      switch(g_power_supply_state)
      {
        case supply_query_volts:
            if(g_state == G_STATE_ACTIVE)
            {
              strcpy(out_buf,get_volt_on_cmd);
            }
            else
            {
              strcpy(out_buf,get_set_volt_cmd);
            }
            client.print(out_buf);
            g_power_supply_state = supply_read_volts;
            g_power_supply_timeout = 100;
        break;
        case supply_read_volts:
          if((c == '>'))
          {
            g_supply_volts = atof(in_buf);
            g_power_supply_state = supply_query_amps; 
            break;
          }   
          if(g_power_supply_timeout == 0)
          {
            Serial.println("Power Supply Read Volts Timeout");
            g_power_supply_state = supply_query_amps;
          }
        break;
        case supply_query_amps:
            if(g_state == G_STATE_ACTIVE)
            {
              strcpy(out_buf,get_curr_on_cmd);
            }
            else
            {
              strcpy(out_buf,get_set_cur_cmd);
            }        
            client.print(out_buf);
            g_power_supply_state = supply_read_amps;
            g_power_supply_timeout = 100;  
        break;
        case supply_read_amps:
          if((c == '>'))
          {
            g_supply_amps = atof(in_buf);           
            g_power_supply_state = supply_vset; 
            break;
          }             
          if(g_power_supply_timeout == 0)
          {
            Serial.println("Power Supply Read Amps Timeout");
            g_power_supply_state = supply_vset;
          }
        break; 
        case supply_vset:
          if(g_power == POWER_STUN)
          {
            itoa(g_stun_target_voltage*10, volts, 10);
          }
          else
          {
            itoa(g_euthanize_target_voltage*10, volts, 10);
          }
          strcpy(out_buf,set_volt_cmd);
          strcat(out_buf,volts);
          strcat(out_buf,carriage);
          client.print(out_buf);
          //Serial.println(out_buf);
            g_power_supply_state = supply_iset;           
        break; 
        case supply_iset:     
          if(g_power == POWER_STUN)
          {
            itoa(g_stun_current/100, amps, 10);
            itoa(g_stun_current%100, amps_decimal, 10);
          }
          else
          {
            itoa(g_euth_current/100, amps, 10);
            itoa(g_euth_current%100, amps_decimal, 10);
          }      
          strcat(amps, ".");
          strcat(amps, amps_decimal);
          strcpy(out_buf,set_current_cmd);
          strcat(out_buf,amps);
          strcat(out_buf,carriage);
          client.print(out_buf);
          //Serial.println(out_buf);
          g_power_supply_state = supply_send_control;  
        break;         
        case supply_send_control:
          if((g_state == G_STATE_ACTIVE) && (g_flag_on_command_sent == 0))
          {
            g_flag_on_command_sent = 1;
            strcpy(out_buf,on_cmd);
            client.println(out_buf);
            //Serial.println(out_buf);
          } 
          else if((g_state != G_STATE_ACTIVE) && (g_flag_on_command_sent == 1))
          {
            g_flag_on_command_sent = 0;
            strcpy(out_buf,off_cmd);
            client.println(out_buf);
            //Serial.println(out_buf);
          }
          g_power_supply_state = supply_query_volts;
          //g_power_supply_state = supply_query_id;      
        break;
        // case supply_query_id:
        //     strcpy(out_buf,idn_cmd);   
        //     client.print(out_buf);
        //     g_power_supply_state = supply_read_id;
        //     g_power_supply_timeout = 100;  
        // break;
        // case supply_read_id:
        //   if((c == '>'))
        //   {
        //     Serial.print("Power Supply ID: ");
        //     Serial.println(in_buf);           
        //     g_power_supply_state = supply_query_volts; 
        //     break;
        //   }             
        //   if(g_power_supply_timeout == 0)
        //   {
        //     Serial.println("Power Supply Read ID Timeout");
        //     g_power_supply_state = supply_query_volts;
        //     fsm.trigger(POWER_ERROR_ASSERTED);
        //   }
        // break;         
        // case supply_query_protection:
        //     strcpy(out_buf, get_supply_protection);   
        //     client.print(out_buf);
        //     g_power_supply_state = supply_read_protection;
        //     g_power_supply_timeout = 100;  
        // break;
        // case supply_read_protection:
        //   if((c == '>'))
        //   {
        //     //Serial.print("Power Supply Protection: ");
        //     //Serial.println(in_buf);
        //     if(!strcmp('0', in_buf))
        //     {
        //       // If power supply enters protected state reset it
        //       Serial.println("Reset Power Supply Protection State!");
        //       strcpy(out_buf, clr_prot);   
        //       client.print(out_buf);              
        //     }           
        //     g_power_supply_state = supply_query_volts; 
        //     break;
        //   }             
        //   if(g_power_supply_timeout == 0)
        //   {
        //     Serial.println("Power Supply Read State Timeout");
        //     g_power_supply_state = supply_query_volts;
        //   }
        // break;         
        // default:
        //   Serial.println("Power Supply Communication Flaw");
        //   g_power_supply_state = supply_query_volts;
        // break;
      }
    }
  }
}
void power_supply_terminal()
{
  itoa(g_stun_voltage*10, volts, 10);
  strcpy(out_buf,set_volt_cmd);
  strcat(out_buf,volts); 
  
  if (client.available()) {
    char c = client.read();
    Serial.print(c);
  }
       
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    if (client.connected()) {
      client.print(inChar);
    }
  }   
}

// Hardware Setup
void setup()
{
  Wire.begin();
  rgbw.setrgbw(0,0,0,0); // all LEDs on
  rgbw.begin(0x62, 0);
  // Initialize digital Outputs
  pinMode(PWM_OUT, OUTPUT);
  pinMode(INDICATOR_READY, OUTPUT);
  pinMode(INDICATOR_A, OUTPUT);
  pinMode(INDICATOR_B, OUTPUT);
  pinMode(INDICATOR_STUN, OUTPUT);
  pinMode(INDICATOR_EUTHANIZE, OUTPUT);
  pinMode(INDICATOR_ACTIVE, OUTPUT);
  pinMode(RELAY_A, OUTPUT);
  pinMode(RELAY_B, OUTPUT);
  pinMode(RELAY_CROWBAR, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(SPARE, OUTPUT);
  pinMode(LCD_RED, OUTPUT);
  pinMode(LCD_GREEN, OUTPUT);
  pinMode(LCD_BLUE, OUTPUT);
  
  // Initialize Digital Inputs
  pinMode(ENCODER_BUTTON, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(BUTTON_ENABLE, INPUT);
  pinMode(BUTTON_DISABLE, INPUT);
  pinMode(SWITCH_A, INPUT);
  pinMode(SWITCH_B, INPUT);
  pinMode(BUTTON_START, INPUT);
  pinMode(BUTTON_STOP, INPUT);
  pinMode(BUTTON_E_STOP, INPUT);
  pinMode(ALARM_1, INPUT);
  pinMode(ALARM_2, INPUT);
  pinMode(A10, INPUT_PULLUP);
    
  // Recover Eeprom variables
  g_stun_current = EEPROM.read(var_stun_current);  
  g_stun_voltage = EEPROM.read(var_stun_voltage);
  g_stun_time = EEPROM.read(var_stun_time);
  g_euth_current = EEPROM.read(var_euth_current); 
  g_euthanize_voltage = EEPROM.read(var_euthanize_voltage);
  g_euthanize_time = EEPROM.read(var_euthanize_time);
  g_ramp_time = EEPROM.read(var_ramp_time);

  // Initialize Serial Interface
  Serial.begin(115200);
  
  // Add Finite State Machine Transitions
  fsm.add_transition(&state_idle, &state_idle, STATE_SWITCH_A, &on_trans_switchA);
  fsm.add_transition(&state_idle, &state_idle, STATE_SWITCH_B, &on_trans_switchB);

  fsm.add_transition(&state_idle, &state_ready, ENABLE_BUTTON_PRESSED, &on_trans_idle_ready);
  fsm.add_transition(&state_ready, &state_idle, DISABLE_BUTTON_PRESSED, &on_trans_ready_idle);
  fsm.add_transition(&state_ready, &state_idle, STOP_BUTTON_PRESSED, &on_trans_ready_idle);
  fsm.add_transition(&state_ready, &state_active, START_BUTTON_PRESSED, &on_trans_ready_active);
  fsm.add_transition(&state_active, &state_idle, STOP_BUTTON_PRESSED, &on_trans_active_idle);
  fsm.add_transition(&state_active, &state_idle, DISABLE_BUTTON_PRESSED, &on_trans_active_idle);
  
  // E-stop
  fsm.add_transition(&state_idle, &state_estop, ESTOP_BUTTON_PRESSED, &on_trans_idle_estop);
  fsm.add_transition(&state_ready, &state_estop, ESTOP_BUTTON_PRESSED, &on_trans_ready_estop);
  fsm.add_transition(&state_active, &state_estop, ESTOP_BUTTON_PRESSED, &on_trans_active_estop);
  fsm.add_transition(&state_estop, &state_idle, ESTOP_BUTTON_RELEASED, &on_trans_estop_idle);  

   // Alarm 1
  fsm.add_transition(&state_idle, &state_alarm_1, ALARM_1_ASSERTED, &on_trans_idle_estop);
  fsm.add_transition(&state_ready, &state_alarm_1, ALARM_1_ASSERTED, &on_trans_ready_estop);
  fsm.add_transition(&state_active, &state_alarm_1, ALARM_1_ASSERTED, &on_trans_active_estop);
  fsm.add_transition(&state_alarm_1, &state_idle, ALARM_1_DEASSERTED, &on_trans_alarm_1_idle);

   // Alarm 2
  fsm.add_transition(&state_idle, &state_alarm_2, ALARM_2_ASSERTED, &on_trans_idle_estop);
  fsm.add_transition(&state_ready, &state_alarm_2, ALARM_2_ASSERTED, &on_trans_ready_estop);
  fsm.add_transition(&state_active, &state_alarm_2, ALARM_2_ASSERTED, &on_trans_active_estop);
  fsm.add_transition(&state_alarm_2, &state_idle, ALARM_2_DEASSERTED, &on_trans_alarm_2_idle);
 
  // Power Supply Error
  fsm.add_transition(&state_idle, &state_power_error, POWER_ERROR_ASSERTED, &on_trans_idle_power_error);
  fsm.add_transition(&state_ready, &state_power_error, POWER_ERROR_ASSERTED, &on_trans_ready_power_error);
  fsm.add_transition(&state_active, &state_power_error, POWER_ERROR_ASSERTED, &on_trans_active_power_error);
  fsm.add_transition(&state_power_error, &state_idle, POWER_ERROR_DEASSERTED, &on_trans_power_error_idle);

  // Remote Control Error
  fsm.add_transition(&state_idle, &state_remote_error, REMOTE_ERROR_ASSERTED, &on_trans_idle_remote_error);
  fsm.add_transition(&state_ready, &state_remote_error, REMOTE_ERROR_ASSERTED, &on_trans_ready_remote_error);
  fsm.add_transition(&state_active, &state_remote_error, REMOTE_ERROR_ASSERTED, &on_trans_active_remote_error);
  fsm.add_transition(&state_remote_error, &state_idle, REMOTE_ERROR_DEASSERTED, &on_trans_remote_error_idle);
  
  // Initialize buttons
  encoder_button.setCallback(encoder_button_changed);
  button_enable.setCallback(button_enable_changed);
  button_disable.setCallback(button_disable_changed);
  switch_a.setCallback(switch_a_changed);
  switch_b.setCallback(switch_b_changed);
  button_start.setCallback(button_start_changed);
  button_stop.setCallback(button_stop_changed);
  button_estop.setCallback(button_estop_changed);
  alarm_1.setCallback(alarm_1_changed);
  alarm_2.setCallback(alarm_2_changed);

  // Initialize LCD
  rgbw.setrgbw(0,0,0,0); // all LEDs on
  lcd_display.init ();
  lcd_display.setCursor(0,0);
  //                "xxxxxxxxxxxxxxxxxxxx" 
  lcd_display.print("ElectroAnesthesia II");
  lcd_display.setCursor(1,0);
  //                "xxxxxxxxxxxxxxxxxxxx" 
  lcd_display.print("HW 1.0 / FW 1.0");
  delay(5000);
  lcd_display.clear();
        
  // Initialize Rotary Encoder
  rotary.setTrigger(HIGH);
  rotary.setDebounceDelay(5);
  rotary.setErrorDelay(5);
  rotary_encoder();
  
  // Initialize Timer1
  Timer1.initialize(TIMER_ONE_SECOND); // interrupt every 1ms
  Timer1.attachInterrupt(timer_one);                

  // Instantiate MASTER or SLAVE
#ifdef MODBUS_MASTER
  Serial.println("Initialize Master Salmon Electro Anasthesia");
  // ModbusQuery 0: read registers
  ModbusQuery[0].u8id = SlaveModbusAdd; // slave address
  ModbusQuery[0].u8fct = MB_FC_READ_REGISTERS; // function code (this one is registers read)
  ModbusQuery[0].u16RegAdd = 0; // start address in slave
  ModbusQuery[0].u16CoilsNo = NUM_COILS; // number of elements (coils or registers) to read
  ModbusQuery[0].au16reg = ModbusRegisters; // pointer to a memory array in the CONTROLLINO

  // ModbusQuery 1: write a single register
  ModbusQuery[1].u8id = SlaveModbusAdd; // slave address
  ModbusQuery[1].u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS; // function code (this one is write a single register)
  ModbusQuery[1].u16RegAdd = SLAVE_ADD; // start address in slave
  ModbusQuery[1].u16CoilsNo = NUM_COILS; // number of elements (coils or registers) to write
  ModbusQuery[1].au16reg = ModbusRegisters+SLAVE_ADD; // pointer to a memory array in the CONTROLLINO
  
  ControllinoModbusMaster.begin( 19200 ); // baud-rate at 19200
  ControllinoModbusMaster.setTimeOut( 1000 ); // if there is no answer in 5000 ms, roll over
  
  WaitingTime = millis() + Modbus_wait;
  myState = 0;
  currentQuery = 0;
#else // SLAVE
// see slave
#endif // MODBUS_MASTER

#ifdef MODBUS_MASTER
  // Setup Ethernet
  Ethernet.begin (clientMAC, clientIp);
  Ethernet.init(10);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) 
  {
    Serial.println("Ethernet Interface Not Found");
    while (true) 
    {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  while (Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.println("Ethernet Cable Not Connected");
    delay(500);
  } 

  // give the Ethernet shield a second to initialize:
  delay(1000);
  Serial.println("Connecting to Power Supply");

  // if you get a connection, report back via serial:
  if (client.connect(power_supply_ip, 5024)) 
  {
    Serial.println("Connected to Power Supply");
    // Reset power supply
    strcpy(out_buf,rst_cmd);   
    client.print(out_buf);
    Serial.println("Reset Power Supply");
  } 
  else 
  {
    // if you didn't get a connection to the server:
    Serial.println("Connection to Power Supply Failed");
  }   
#endif // MODBUS_MASTER  

  // Start watchdog timer
  Watchdog.enable(WATCHDOG_TIMER);

state_machine();

  // Check if E-stop button is Pressed at startup
  if(digitalRead(BUTTON_E_STOP))
  {
    Serial.println("E-Stop engaded at startup!!!");
    fsm.trigger(ESTOP_BUTTON_PRESSED);
  }

}

void remote_control()
{
#ifdef MODBUS_MASTER
  uint8_t retval = PASS;

  switch(myState) 
  {
    case 0: 
      if (millis() > WaitingTime) 
      {
        myState++; // wait state
      }
      break;
    case 1:    
      // Serial.print("---- Sending query ");
      // Serial.print(currentQuery);
      // Serial.println(" -------------");      
      retval = ControllinoModbusMaster.query( ModbusQuery[currentQuery] ); // send query (only once)
      myState++;
      currentQuery++;
      if (currentQuery == 2) 
      {
        currentQuery = 0;
      }
      break;
    case 2:
      retval = ControllinoModbusMaster.poll(); // check incoming messages 
      if (ControllinoModbusMaster.getState() == COM_IDLE) 
      {             
        // response from the slave was received
        myState = 0;
        WaitingTime = millis() + Modbus_wait; 
        // debug printout
        if (currentQuery == 0)
        {
           //registers write was proceed
          //Serial.println("---------- WRITE RESPONSE RECEIVED ----");
          //Serial.println("");        
        }
        if (currentQuery == 1)
        {
          // Modbus counter stall indicates a communication timeout
          comm_counter_current = ModbusRegisters[modbus_slave_in_cnt];
          // Serial.print("comm_counter_last: ");
          // Serial.println(comm_counter_last, DEC);
          // Serial.print("comm_counter_current: ");
          // Serial.println(comm_counter_current, DEC);
          // Serial.println("");       
          if (comm_counter_current == comm_counter_last)
          {
            if(comm_error_count)
            {
              comm_error_count--;
            }
            if(comm_error_count == 0)
            {
              Serial.println("Remote Control Error!");
              fsm.trigger(REMOTE_ERROR_ASSERTED);
              retval = FAIL;
            }
          }
          else
          {
            comm_error_count = COMM_ERROR_LIMIT;
            fsm.trigger(REMOTE_ERROR_DEASSERTED);
          }
          comm_counter_last = comm_counter_current;
                    
          //registers read was proceed
          //Serial.println("---------- READ RESPONSE RECEIVED ----");
          ModbusRegisters[modbus_master_buttons] = 0;
          // Always recieve button inputs from the Slave
          g_remote_buttons = ModbusRegisters[modbus_slave_buttons];
          switch(g_remote_buttons)
          {
            case modbus_button_start:
            {
              Serial.println("Remote Start Button Pressed");
              ModbusRegisters[modbus_master_buttons] = 1;
              fsm.trigger(START_BUTTON_PRESSED);
              break;
            }
            case modbus_button_stop:
            {
              Serial.println("Remote Stop Button Pressed");
              ModbusRegisters[modbus_master_buttons] = 1;                
              fsm.trigger(STOP_BUTTON_PRESSED);
              break;
            }  
            case modbus_button_enable:
            {
              Serial.println("Remote Enable Button Pressed");     
              ModbusRegisters[modbus_master_buttons] = 1;          
              fsm.trigger(ENABLE_BUTTON_PRESSED);
              break;
            } 
            case modbus_button_disable:
            {
              Serial.println("Remote Disable Button Pressed");  
              ModbusRegisters[modbus_master_buttons] = 1;             
              fsm.trigger(DISABLE_BUTTON_PRESSED);
              break;
            } 
            case modbus_button_tank:
            {
              Serial.println("Remote Tank Select Button Pressed");
              ModbusRegisters[modbus_master_buttons] = 1;
              fsm.trigger(STATE_SWITCH_A);
              break;                
            }     
            case modbus_button_power:
            {
              Serial.println("Remote Power State Button Pressed");
              ModbusRegisters[modbus_master_buttons] = 1;
              fsm.trigger(STATE_SWITCH_B);
              break;                 
            }   
            default:
            {
              break;                 
            }             
          }       
          // If Slave is in control
          if(ModbusRegisters[modbus_slave_variable])
          {
              Serial.println("Remote Updated Configuration");
              ModbusRegisters[modbus_slave_variable] = 0;
              // Trigger a write to EEPROM after a delay
              g_eepromCounter = EEPROMCOUNTER;
              g_eepromCounterEnabled = 1;
              g_stun_current = ModbusRegisters[modbus_slave_stun_current];
              g_stun_voltage = ModbusRegisters[modbus_slave_stun_voltage];
              g_stun_time = ModbusRegisters[modbus_slave_stun_time];
              g_euth_current = ModbusRegisters[modbus_slave_euth_current];
              g_euthanize_voltage = ModbusRegisters[modbus_slave_euth_voltage];
              g_euthanize_time = ModbusRegisters[modbus_slave_euth_time];   
              g_ramp_time = ModbusRegisters[modbus_slave_ramp_time];                                                                    
          }
          else
          {
              ModbusRegisters[modbus_master_stun_current] = g_stun_current;
              ModbusRegisters[modbus_master_euth_current] = g_euth_current;
              ModbusRegisters[modbus_master_stun_voltage] = g_stun_voltage;
              ModbusRegisters[modbus_master_stun_time] = g_stun_time;
              ModbusRegisters[modbus_master_euth_voltage] = g_euthanize_voltage;
              ModbusRegisters[modbus_master_euth_time] = g_euthanize_time;  
              ModbusRegisters[modbus_master_ramp_time] = g_ramp_time;  
          }
          ModbusRegisters[modbus_master_state] = g_state;
          ModbusRegisters[modbus_master_active_timer] = g_active_timer;
          ModbusRegisters[modbus_master_voltage] = uint16_t(g_supply_volts*100);
          ModbusRegisters[modbus_master_current] = uint16_t(g_supply_amps*100);
          ModbusRegisters[modbus_master_watts] = uint16_t(g_supply_watts);
          ModbusRegisters[modbus_master_tank] = g_tank;
          ModbusRegisters[modbus_master_power] = g_power;  
          ModbusRegisters[modbus_master_leds] = g_leds;                             
        }
      }
      break;
    default:
      break;
  }
#else
// refer to slave
#endif // MODBUS_MASTER
}

// Main Program Loop
void loop() 
{ 
  Watchdog.reset();
  remote_control();
  lcd_display_update();
  buttons();
  lights();
  eeprom();  
  state_machine();
  
#ifdef MODBUS_MASTER 
  power_supply();
#endif // MODBUS_MASTER 
}                 
