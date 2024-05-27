/*-------------DECLARACION DE LIBRERIAS Y VARIABLES--------------------*/
/*---SDS011--------*/
#include "SDS011.h"
int sensorData;
float p10, p25;
int error;
SDS011 my_sds;
#define RX_PIN 13
#define TX_PIN 15
/*-----MQ135--------*/
#define MQ135Pin 12
int GasLevelMQ135;
/*-----MQ7----------*/
#define MQ7Pin 34
int CO;
/*----KY038---------*/
#define KY038Pin 4
float soundDB;
int soundAnalog;
int soundValue;
#define DIGITAL_PIN 2
/*-----------Transistor-------------*/
#define PowerPin   14
/*-------Librerias BME-------*/

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)
/*---------LORA-------------------*/
#include <lmic.h>
#include <hal/hal.h>
#define     TX_BUFFER_SIZE        20     //El paquete que se manda es de X bytes
static uint8_t txBuffer[TX_BUFFER_SIZE]; 
/*----------SLEEP--------------------*/
#include <esp_sleep.h>
#define     WAKE_TIME             30000 
/*------CREDENCIALES TTN--------------*/
//Se introducen las credenciales del  dispostivo registrado en The Things Network 
static const u1_t PROGMEM APPEUI[8]={0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]={0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
/*------------PINES LORA Lilygo Lora 32-----------------------*/
const lmic_pinmap lmic_pins = {           //Pines utilizamos para la el modulo LoRa
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32} 
};

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

  int MQ135 = MQ135Value();
  txBuffer[10] = byte(MQ135);
  txBuffer[11] = MQ135 >> 8;
  
  int CO = COValue();
  txBuffer[12] = byte(CO);
  txBuffer[13] = CO >> 8;

  PMS();
  int PM25 = p25*100;
  txBuffer[14] = byte(PM25);
  txBuffer[15] = PM25 >> 8;

  int PM10 = p10*100;
  txBuffer[16] = byte(PM10);
  txBuffer[17] = PM10 >> 8;
  

}

void sleep_millis(uint64_t ms) {
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_deep_sleep_start();
}
//Funcion para leer Temperatura
float bmeTemp() {
  // Obtener y mostrar la temperatura
  float temp = bme.readTemperature();
  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" °C");
  return temp;
}
//Funcion para leer la presion
float bmePres() {
  // Obtenemos y printeamos la presion en hpa
  float pressure = bme.readPressure();
  Serial.print("Pressure = ");
  Serial.print(pressure / 100.0F);
  Serial.println(" hPa");
  return pressure;
}
//Funcion para medir la altitud
float bmeAlt() {
  // Obtenemos y printeamos la altitud en metros
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float height = altitude + 196;
  Serial.print("Approx. Altitude = ");
  Serial.print(height);
  Serial.println(" m");
  return height;
}
//Funcion para leer la humedad
float bmeHum() {
  // Obtenemos y printeamos el porcentaje de humedad
  float hum = bme.readHumidity();
  Serial.print("Humidity = ");
  Serial.print(hum);
  Serial.println(" %");
  return hum;
}
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
void printHex2(unsigned v) {
    //Limitamos el valor a un byte
    v &= 0xff;                             
    //Si el valor es menor que 16, imprimir un 0 para asegurar que no haya dos digitos en hexadecimal            
    if (v < 16)                                       
        Serial.print('0');
        Serial.print(v, HEX); 
    //Imprimir el valor en formato hexadecimal
}
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
int MQ135Value() {
  GasLevelMQ135 = analogRead(MQ135Pin);       
  Serial.print("Air Quality:");
  Serial.print(GasLevelMQ135, DEC);               
  Serial.println("PPM");
  return GasLevelMQ135;
}
int COValue (){
  int CO = analogRead(MQ7Pin);
  Serial.print("CO value: ");
	Serial.println(CO);//prints the CO value
  return CO;
}
void PMS (){
  error = my_sds.read(&p25, &p10);
  if (!error) {
    Serial.println("P2.5: " + String(p25));
    Serial.println("P10:  " + String(p10));
  }
}
int count = 0;
int count2 = 0;
int t1;
int t2;

/*-----------SETUP----------------------*/
void setup() {
  my_sds.begin(TX_PIN, RX_PIN);
  Serial.begin(9600);
  pinMode(PowerPin, OUTPUT);
  digitalWrite(PowerPin, HIGH);
  delay(1000);

  //Inicializacion del sensor BME280   
  unsigned status;     
  status = bme.begin(0x76);

  os_init();          
  LMIC_reset();

  t1 = millis();  
  count2 = count;
}
/*-----------LOOP-----------------------*/
void loop() {

  os_runloop_once();//Ejecucion del procesador del modulo LoRa

  /*-------SONIDO (DE MOMENTO NO VA)-----------*/
  /*int sensorValue = analogRead(KY038Pin);
  Serial.println(sensorValue);
  soundValue = sensorValue;
  Serial.println(soundValue);
  float decibelios = map(soundValue, 0, 4095, 30, 90);
  Serial.print("KY038 SOUND: ");
  Serial.print(decibelios);
  Serial.println("DB");*/

  int t2 = millis();//Se crea una variable del tiempo actual 
    
  if((t2-t1)>30000){ //Si ha pasado el tiempo de ejecucion de loop entramos en la funcion

      t1 = millis(); //Guardartel tiempo actual en T1
      PMS();
      doSensor(txBuffer); // Llama a la función doSensor con txBuffer
      Serial.println("Sending"); // Agrega una nueva línea después de imprimir "Sending"
      do_send(); // Envía el paquete con la función do_send()

    }
  }
