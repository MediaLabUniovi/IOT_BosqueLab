/* Proyecto Bosquelab de detección de incendios en bosques 
**
** MediaLab Uniovi
*/

/*-------------------DECLARACION DE LIBRERIAS Y VARIABLES--------------------*/
#include <Arduino.h>
#include <SDS011.h>                                             // SDS011
#include <MQUnifiedsensor.h>                                    // MQ135
#include <MQ7.h>   
#include <Adafruit_Sensor.h>                                    // BME
#include <Adafruit_BME280.h>
#include <Wire.h>                                               // BME
#include <Adafruit_SCD30.h>
#include <arduino_lmic.h>                                       // LoRaWAN_LMIC
#include <hal/hal.h>                                            // HAL to run LMIC 
#include <esp_sleep.h>                                          // Sleep

/*-----------------------SDS011------------------------*/
int sensorData;
float p10, p25;
int error;
SDS011 my_sds;
#define RX_PIN 15
#define TX_PIN 13
/*------------------------MQ135------------------------*/
//Definitions
#define         Board                   ("ESP-32") // Wemos ESP-32 or other board, whatever have ESP32 core.
#define         Pin                     (36)  //IO25 for your ESP32 WeMos Board, pinout here: https://i.pinimg.com/originals/66/9a/61/669a618d9435c702f4b67e12c40a11b8.jpg
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-135") //MQ3 or other MQ Sensor, if change this verify your a and b values.
#define         Voltage_Resolution      (5) // 
#define         ADC_Bit_Resolution      (12) // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         RatioMQ135CleanAir       (3.6) // Ratio of your sensor, for this example an MQ-3
/*****************************Globals***********************************************/
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
/*-------------------------MQ7------------------------*/
#define A_PIN 34
#define VOLTAGE 5
// init MQ7 device
MQ7 mq7(A_PIN, VOLTAGE);
/*-----------Transistor-------------*/
#define PowerPin   2
/*---------BATERIA------------------*/
#define BatteryPin 12
/*-------Librerias BME-------*/
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)
/*-----------SCD30-----------------*/
Adafruit_SCD30  scd30;
/*--------------------------LORA-------------------------*/
#define     TX_BUFFER_SIZE        23     //El paquete que se manda es de X bytes
static uint8_t txBuffer[TX_BUFFER_SIZE]; 
/*--------------------------SLEEP-------------------------*/
#define     WAKE_TIME             10000 
/*----------------------CREDENCIALES TTN--------------------------*/
//Se introducen las credenciales del  dispostivo registrado en The Things Network 
static const u1_t PROGMEM APPEUI[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
/*--------------------------LSB------------------------*/
static const u1_t PROGMEM DEVEUI[8]={0x59, 0x7C, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
/*-----------------------MSB*---------------------------*/
static const u1_t PROGMEM APPKEY[16] = {0xF0, 0x39, 0xDA, 0xB0, 0x2F, 0x42, 0x84, 0xF6, 0xC8, 0xA7, 0x55, 0x68, 0x92, 0x6C, 0xFD, 0xD0};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
/*--------------------PINES LORA Lilygo Lora 32------------------------*/
const lmic_pinmap lmic_pins = {           //Pines utilizamos para la el modulo LoRa
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32} 
};
// -------------------------- DECLARACION DE FUNCIONES ----------------------------
void doSensor(uint8_t txBuffer[]);
void sleep_millis(uint64_t ms);
float bmeTemp();
float bmePres();
float bmeAlt();
float bmeHum();
void onEvent (ev_t ev);
void printHex2(unsigned v);
void do_send();
float MQ135Value();
float COValue ();
void PMS ();
float SCD30();
float ReadBattery();
// ---------------------------------------------------------------------------------

int count = 0;
int count2 = 0;
int t1;
int t2;

/*--------------------------------SETUP----------------------------*/
void setup() {
    my_sds.begin(TX_PIN, RX_PIN);
    Serial.begin(9600);
    pinMode(PowerPin, OUTPUT);
    digitalWrite(PowerPin, HIGH);
    delay(10000);

    Serial.println("Calibrating MQ7");
    mq7.calibrate();		// calculates R0
    Serial.println("Calibration done!");

    Serial.begin(9600); //Init serial port

    //Set math model to calculate the PPM concentration and the value of constants
    MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
    MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to to calculate NH4 concentration

    /*
        Exponential regression:
    GAS      | a      | b
    CO       | 605.18 | -3.937  
    Alcohol  | 77.255 | -3.18 
    CO2      | 110.47 | -2.862
    Toluen  | 44.947 | -3.445
    NH4      | 102.2  | -2.473
    Aceton  | 34.668 | -3.369
    */
    
    MQ135.init(); 

    Serial.print("Calibrating please wait.");
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++){
        MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
        calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
        Serial.print(".");
    }
    MQ135.setR0(calcR0/10);
    Serial.println("  done!.");
    
    if(isinf(calcR0)){Serial.println("Warning: Conection issue, R0 MQ135 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
    if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
    /*****************************  MQ CAlibration ********************************************/ 
    MQ135.serialDebug(true);

 //inicializacion SCD30 
    if (!scd30.begin()) {
        Serial.println("Failed to find SCD30 chip");
        while (1) { delay(10); }
    }
    Serial.println("SCD30 Found!");

    Serial.print("Measurement Interval: "); 
    Serial.print(scd30.getMeasurementInterval()); 
    Serial.println(" seconds");

    if (!scd30.forceRecalibrationWithReference(400)){
        Serial.println("Failed to force recalibration with reference");
        while(1) { delay(10); }
    }
    Serial.print("Forced Recalibration reference: ");
    Serial.print(scd30.getForcedCalibrationReference());
    Serial.println(" ppm");

    //Inicializacion del sensor BME280   
    unsigned status;     
    status = bme.begin(0x76);

    os_init();          
    LMIC_reset();
    delay(1000);

    t1 = millis();  
    count2 = count;
}
/*--------------------------------LOOP-----------------------*/
void loop() {
    os_runloop_once();//Ejecucion del procesador del modulo LoRa

    int t2 = millis();//Se crea una variable del tiempo actual 
    
    if((t2-t1)>30000){ //Si ha pasado el tiempo de ejecucion de loop entramos en la funcion
        t1 = millis(); //Guardartel tiempo actual en T1
        doSensor(txBuffer); // Llama a la función doSensor con txBuffer
        Serial.println("Sending..."); // Agrega una nueva línea después de imprimir "Sending"
        do_send(); // Envía el paquete con la función do_send()
    }
}

//----------------------------------------------------------------------------------------------------
// Función para el tramitado de datos de sensores 
void doSensor(uint8_t txBuffer[]) {
    // Llenar el búfer con caracteres nulos para borrar el contenido anterior
    memset(txBuffer, 0, TX_BUFFER_SIZE); //Creamos un paquete del numero de bytes elegido en configuracion
    
    //Para crear los bytes pasamos todos los valores a enteros y usamos la cantidad de bytes necesarios con cada medida 
    float t = bmeTemp();
    int shiftTemp = int(t * 100);
    txBuffer[0] = byte(shiftTemp);
    txBuffer[1] = shiftTemp >> 8;

    float p = bmePres();
    int shiftpresion = int(p * 100);
    txBuffer[2] = byte(shiftpresion);
    txBuffer[3] = shiftpresion >> 8;
    txBuffer[4] = shiftpresion >> 16;
    txBuffer[5] = shiftpresion >> 32;

    float a = bmeAlt();
    int shiftAltura = int(a * 100);
    txBuffer[6] = byte(shiftAltura);
    txBuffer[7] = shiftAltura >> 8;

    float h = bmeHum();
    int shifthumedad = int(h * 100);
    txBuffer[8] = byte(shifthumedad);
    txBuffer[9] = shifthumedad >> 8;

    float MQ135 = MQ135Value();
    int shiftMQ135 = int(MQ135*100);
    txBuffer[10] = byte(shiftMQ135);
    txBuffer[11] = shiftMQ135 >> 8;
    
    float CO = COValue();
    int shiftCO = int(CO*100);
    txBuffer[12] = byte(shiftCO);
    txBuffer[13] = shiftCO >> 8;

    PMS();
    int PM25 = p25*100;
    txBuffer[14] = byte(PM25);
    txBuffer[15] = PM25 >> 8;

    int PM10 = p10*100;
    txBuffer[16] = byte(PM10);
    txBuffer[17] = PM10 >> 8;
    
    float bat = ReadBattery();
    int shiftbat = int(bat*100);
    txBuffer[18] = byte(shiftbat);
    txBuffer[19] = shiftbat >> 8;
    
    float CO2 = SCD30();
    int shiftCO2 = int (CO2*10);
    Serial.print(shiftCO2);
    
    txBuffer[20] = byte(shiftCO2);
    txBuffer[21] = shiftCO2 >> 8;

    return ;
}
// Función para dormir una cantidad de milisegundos
void sleep_millis(uint64_t ms) {
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_deep_sleep_start();
}
// Funcion para leer Temperatura con el BME280
float bmeTemp() {
  // Obtener y mostrar la temperatura
  float temp = bme.readTemperature();
  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" °C");
  return temp;
}
// Funcion para leer la presion con el BME280
float bmePres() {
  // Obtenemos y printeamos la presion en hpa
  float pressure = bme.readPressure();
  Serial.print("Pressure = ");
  Serial.print(pressure / 100.0F);
  Serial.println(" hPa");
  return pressure;
}
// Funcion para medir la altitud con el BME 280
float bmeAlt() {
  // Obtenemos y printeamos la altitud en metros
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float height = altitude + 196;
  Serial.print("Approx. Altitude = ");
  Serial.print(height);
  Serial.println(" m");
  return height;
}
// Funcion para leer la humedad con el BME 280
float bmeHum() {
  // Obtenemos y printeamos el porcentaje de humedad
  float hum = bme.readHumidity();
  Serial.print("Humidity = ");
  Serial.print(hum);
  Serial.println(" %");
  return hum;
}
// Tratamiento de eventos
void onEvent (ev_t ev) {
    switch(ev) {
        //Si no se encuentra el gateway
        case EV_SCAN_TIMEOUT:                         
            Serial.println(F("EV_SCAN_TIMEOUT"));     
            break;
        //Se encuentra el gateway
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));    
            break;
        //Se pierde el Gateway
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));    
            break;
        //Se rastrea el Gateway
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));   
            break;
        //Conexion con el gateway
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));          
            break;
        case EV_JOINED:
            //Conectado con el gateway
            Serial.println(F("EV_JOINED"));           
            {                     
              //Se Declaran variables para almacenar        
              // ID de la red           
              u4_t netid = 0;            
              //Direccion del dispositivo             
              devaddr_t devaddr = 0;    
              //Clave de sesion de Red              
              u1_t nwkKey[16];                  
              //Clave de la sesion de la aplicacion      
              u1_t artKey[16];                        
              //Obtenemos las claves desde el stack LoraWAN
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey); 
              //Se imprime el ID
              Serial.print("netid: ");                
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              //Se imprime la direccion del dispositivo
              Serial.println(devaddr, HEX);          
              //Se imprime la clave de sesion de la aplicacion AppKey 
              Serial.print("AppSKey: ");              
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
              //Hay que imprimir cada byte en formato hexadecimal
                printHex2(artKey[i]);                 
              }
              Serial.println(""); 
              //Se imprime la clave de sesion de Red NwkSKey
              Serial.print("NwkSKey: ");              
              for (size_t i=0; i<sizeof(nwkKey); ++i) { 
                      if (i != 0)
                              Serial.print("-");
                      //Cada byte en formato hexadecimal
                      printHex2(nwkKey[i]);           
              }
              Serial.println();
            }
          //Se establece el modo de verificacion (Desactivada)
            LMIC_setLinkCheckMode(0);
            break;
        //Si la union con el servidor LoRa falla
        case EV_JOIN_FAILED:                                             
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        //Si la reintegracion con el servidor LoRa falla
        case EV_REJOIN_FAILED:                        
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        //Transmision completada
        case EV_TXCOMPLETE:                           
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Recibido ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Recibido"));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes de payload"));
            }
            
            Serial.println("GO TO SLEEP");
            //Se manda a dormir el microcontrolador el tiempo especificado 
            sleep_millis(WAKE_TIME);                  
            break;

        //Si se pierde la sincronizacion de tiempo
        case EV_LOST_TSYNC:                           
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        //Evento de reseteo
        case EV_RESET:
            Serial.println(F("EV_RESET"));            
            break;
        //Si se completa la recepcion de datos
        case EV_RXCOMPLETE:                           
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        //Si se ha perdido la conexion
        case EV_LINK_DEAD:                            
            Serial.println(F("EV_LINK_DEAD"));
            break;
        //Si la conexion esta activa
        case EV_LINK_ALIVE:                           
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        //Comienza la transmision
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));          
            break;
        //Transimision cancelada
        case EV_TXCANCELED:                           
            Serial.println(F("EV_TXCANCELED"));
            break;
        // Comienza la recepcion de datos
        case EV_RXSTART:                              
            
            break;
        //Se completa la transimision de union sin conectarse
        case EV_JOIN_TXCOMPLETE:                      
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        //Event desconocido
        default:
            Serial.print(F("Unknown event: "));       
            Serial.println((unsigned) ev);
            break;
    }
}
// Imprimir el valor en formato hexadecimal
void printHex2(unsigned v) {
    //Limitamos el valor a un byte
    v &= 0xff;                             
    //Si el valor es menor que 16, imprimir un 0 para asegurar que no haya dos digitos en hexadecimal            
    if (v < 16)                                       
        Serial.print('0');
    Serial.print(v, HEX); 

}
// Transmisión de datos
void do_send(){
    //Verificar si no hay una transimision o recepcion en curso
    if (LMIC.opmode & OP_TXRXPEND) {                  
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        //Preparar la transmision de datos en proximo momento posible
        LMIC_setTxData2(1, txBuffer, sizeof(txBuffer)-1, 0); 
        Serial.println(F("Packet queued"));
    }
    // Siguiente transimision programada despues del evento TX_COMPLETE
}
// Medida CO2
float MQ135Value() {
  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
  MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  Serial.print("CO2 PPM = "); Serial.println(MQ135.getPPM());
	Serial.println(""); 	// blank new line

  return MQ135.getPPM();
}
// Medida CO (Monóxido de carbono)
float COValue (){
  Serial.print("CO PPM = "); Serial.println(mq7.readPpm());
	Serial.println(""); 	// blank new line
	delay(1000);
  return mq7.readPpm();
}
// Medida partículas (PM10 y PM25)
void PMS (){
  error = my_sds.read(&p25, &p10);
  if (!error) {
    Serial.println("P2.5: " + String(p25));
    Serial.println("P10:  " + String(p10));
  }
}
// Medida CO2
float SCD30(){ 
  float CO_2;

  if (scd30.dataReady()){
    Serial.println("Data available!");

    if (!scd30.read()){ Serial.println("Error reading sensor data"); return 0; }

    Serial.print("Temperature: ");
    Serial.print(scd30.temperature);
    Serial.println(" degrees C");
    
    Serial.print("Relative Humidity: ");
    Serial.print(scd30.relative_humidity);
    Serial.println(" %");
    
    CO_2 = scd30.CO2;
    Serial.print("CO2: ");
    Serial.print(scd30.CO2, 3);
    Serial.println(" ppm");
    Serial.println("");
  }
  return CO_2;
}
// Medida de estado de carga de batería
float ReadBattery() {

  int analogBat = analogRead(BatteryPin);//Se lee el valor analogico en el pin 12 
  Serial.print(analogBat);
  float digitalBat = (analogBat - 0) * (2.7 - 0) / (3000 - 0); //Como el voltaje esta dividido por un divisor solo leemos la mitad de los valores
  float battery = digitalBat * 1.56; //Multiplicamos por 2 para obtener el valor real
  Serial.print(" Bateria Real ");
  Serial.print(battery);

  return battery;
}

