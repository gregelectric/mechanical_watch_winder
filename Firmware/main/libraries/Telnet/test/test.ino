#include <CRC32.h>
#include <TelnetClient.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Controllino.h> 

EthernetClient client;                                   
telnetClient tc(client); 

byte clientMAC[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 

//put here your router's ip address
IPAddress power_supply_ip (192, 168, 0, 110);
   
#define PWM_OUT             CONTROLLINO_D4

void setup () 
{      
  pinMode(PWM_OUT, OUTPUT);                              
  Serial.begin (9600);                              
  if (!Ethernet.begin (clientMAC))
  {
       Serial.println("\r\nDHCP error");
       while(1);
  }
  
  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  while (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
    delay(500);
  }

  // give the Ethernet shield a second to initialize:
  delay(1000);
  Serial.println("connecting...");
  
  tc.setPromptChar("9258Telnet->");

  if(tc.login(power_supply_ip, "admin=admin", "admin"))
  {     
    Serial.println("connected"); 
    digitalWrite(PWM_OUT, HIGH);   // turn the LED on (HIGH is the voltage level)
    char power_on = "setpower=11110000\n";
    tc.sendCommand(power_on);
    delay(500);
    digitalWrite(PWM_OUT, LOW);    // turn the LED off by making the voltage LOW
    tc.sendCommand("setpower=00000000\n");
    delay(500);       
  }
  else
  {
    Serial.println("login failed");
    tc.disconnect();
    client.stop();
  } 

  delay(1000); 
}

void loop () 
{   
  Serial.println("running...");
  delay(1000); 
}
