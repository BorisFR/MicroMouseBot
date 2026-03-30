# TB6612FNG  
Driver pour 2 moteurs  

Alimentation:  
- partie logique: 2,7 à 5,5 Vcc (via le microcontrôleur)  
- partie moteur: 2,5 à 12 Vcc  
Sorties: 1,2 A par canal (2 A en pointe)  
Interfaces: PWM et digitales  
Dimensions: 20 x 19,5 mm  

                 ╭──────────────╮
 Alim Motors  VM │o     xx     o│ PWMA Motor A PWM control
  VCC Esp32  VCC │o   x    x   o│ AIN2 Motor A direction pin 2
             GND │o            o│ AIN1 Motor A direction pin 1
  - Motor 1  A01 │o    xxxx    o│ STBY Standby control pin
  + Motor 1  A02 │o    xxxx    o│ BIN1 Motor B direction pin 1
  + Motor 2  B02 │o    xxxx    o│ BIN2 Motor B direction pin 2
  - Motor 2  B01 │o            o│ PWMB Motor B PWM control
             GND │o            o│ GND
                 ╰──────────────╯

il faut un GND sur l'alimentation moteur et 1 GND sur l'ESP32. Les 3 GND sont commun.

