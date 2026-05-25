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
#include "project_config.h"

#ifndef DEBUG_MODE_ENABLED
#define DEBUG_MODE_ENABLED     0
#endif

#ifndef DEBUG_TABLE_INTERVAL_MS
#define DEBUG_TABLE_INTERVAL_MS 12000UL
#endif

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
/*---------KY038--------------------*/
#define KY038_PIN 35
/*-------Librerias BME-------*/
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)
/*-----------SCD30-----------------*/
Adafruit_SCD30  scd30;
/*--------------------------LORA-------------------------*/
#define     TX_BUFFER_SIZE        23     //El paquete que se manda es de X bytes
static uint8_t txBuffer[TX_BUFFER_SIZE]; 
/*----------------------CREDENCIALES TTN--------------------------*/
//Se introducen las credenciales del  dispostivo registrado en The Things Network
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
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
int KY038Value();
void printDebugConfig();
void printDebugProbeResults();
void printSensorTable(bool periodicDebugView);
void printLoraIdentifiers();
void printPayloadHex(const uint8_t* data, size_t len);
void printLinkStatus(const char* tag);
void printStatusBlock(const char* title);
void printLastMeasurements();
// ---------------------------------------------------------------------------------

int count = 0;
int count2 = 0;
int t1;
int t2;
bool bmeReady = false;
bool scd30Ready = false;
bool mq7Ready = false;
bool mq135Ready = false;
bool sds011Ready = false;
bool ky038Ready = false;
bool loraJoined = false;
bool firstSendPending = false;
unsigned long txAttemptCounter = 0;
unsigned long loraTransmitStartTime = 0;
float lastTemp = 0.0f;
float lastPressure = 0.0f;
float lastHumidity = 0.0f;
float lastAltitude = 0.0f;
float lastGas = 0.0f;
float lastCO = 0.0f;
float lastPM25 = 0.0f;
float lastPM10 = 0.0f;
float lastBattery = 0.0f;
float lastCO2 = 0.0f;

static float sanitizeValue(float v) {
    if (isnan(v) || isinf(v)) {
        return 0.0f;
    }
    return v;
}

/*--------------------------------SETUP----------------------------*/
void setup() {
    #if SENSOR_SDS011_ENABLED
    my_sds.begin(TX_PIN, RX_PIN);
    sds011Ready = true;
    #else
    pinMode(TX_PIN, INPUT_PULLDOWN);
    pinMode(RX_PIN, INPUT_PULLDOWN);
    sds011Ready = false;
    #endif

    Serial.begin(115200);
    pinMode(PowerPin, OUTPUT);
    digitalWrite(PowerPin, HIGH);
    delay(10000);

    #if SENSOR_MQ7_ENABLED
    Serial.println("[SENSORES] Calibrando MQ7...");
    mq7.calibrate();		// calculates R0
    Serial.println("[SENSORES] MQ7 OK");
    mq7Ready = true;
    #else
    pinMode(A_PIN, INPUT_PULLDOWN);
    mq7Ready = false;
    #endif

    // Serial ya inicializado arriba

    //Set math model to calculate the PPM concentration and the value of constants
    #if SENSOR_MQ135_ENABLED
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

    Serial.print("[SENSORES] Calibrando MQ135");
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++){
        MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
        calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
        Serial.print(".");
    }
    MQ135.setR0(calcR0/10);
    Serial.println(" OK");
    
    if(isinf(calcR0)){
        Serial.println("Warning: Conection issue, R0 MQ135 is infinite (Open circuit detected) please check your wiring and supply");
        mq135Ready = false;
        #if !DEBUG_MODE_ENABLED
        while(1);
        #endif
    }
    if(calcR0 == 0){
        Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
        mq135Ready = false;
        #if !DEBUG_MODE_ENABLED
        while(1);
        #endif
    }
    if(!isinf(calcR0) && calcR0 != 0){
        mq135Ready = true;
    }
    /*****************************  MQ CAlibration ********************************************/ 
    MQ135.serialDebug(false);
    #else
    pinMode(Pin, INPUT_PULLDOWN);
    mq135Ready = false;
    #endif

 //inicializacion SCD30 
    #if SENSOR_SCD30_ENABLED
    // Use default I2C bus (same physical bus used previously by CCS811).
    Wire.begin();
    bool scd30BeginOk = scd30.begin(0x61, &Wire);
    if (!scd30BeginOk) {
        Serial.println("Failed to find SCD30 chip");
        scd30Ready = false;
        #if !DEBUG_MODE_ENABLED
        while (1) { delay(10); }
        #endif
    } else {
        scd30Ready = true;
        Serial.println("SCD30 Found!");
    }

    if (scd30Ready) {
        Serial.print("Measurement Interval: "); 
        Serial.print(scd30.getMeasurementInterval()); 
        Serial.println(" seconds");

        if (!scd30.forceRecalibrationWithReference(400)){
            Serial.println("Failed to force recalibration with reference");
            scd30Ready = false;
            #if !DEBUG_MODE_ENABLED
            while(1) { delay(10); }
            #endif
        }
        if (scd30Ready) {
            Serial.print("Forced Recalibration reference: ");
            Serial.print(scd30.getForcedCalibrationReference());
            Serial.println(" ppm");
        }
    }
    #else
    scd30Ready = false;
    #endif

    //Inicializacion del sensor BME280
    #if SENSOR_BME280_ENABLED
    bmeReady = bme.begin(0x76);
    #endif

    #if SENSOR_KY038_ENABLED
    pinMode(KY038_PIN, INPUT);
    ky038Ready = true;
    #else
    pinMode(KY038_PIN, INPUT_PULLDOWN);
    ky038Ready = false;
    #endif

    #if DEBUG_MODE_ENABLED
    printDebugConfig();
    printDebugProbeResults();
    printSensorTable(true);
    #else
    esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
    Serial.println();
    printStatusBlock("INICIO");
    Serial.print("[SISTEMA] Wakeup cause: ");
    Serial.println((int)wakeCause);
    Serial.println("[LORA] Inicializando...");
    os_init();          
    LMIC_reset();
    LMIC_setLinkCheckMode(0);
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    Serial.println("[LORA] Esperando JOIN...");
    firstSendPending = true;
    delay(1000);
    #endif

    t1 = millis();  
    count2 = count;
}
/*--------------------------------LOOP-----------------------*/
void loop() {
    #if DEBUG_MODE_ENABLED
    int now = millis();
    if((now - t1) > DEBUG_TABLE_INTERVAL_MS){
        t1 = millis();
        printSensorTable(true);
    }
    return;
    #endif

    os_runloop_once();//Ejecucion del procesador del modulo LoRa

    if (firstSendPending && !(LMIC.opmode & OP_TXRXPEND)) {
        firstSendPending = false;
        t1 = millis();
        Serial.println();
        Serial.println("=== PRIMER ENVIO TRAS RESET ===");
        printLinkStatus("Previo a lectura");
        doSensor(txBuffer);
        printSensorTable(false);
        if (!loraJoined) {
            Serial.println("[LORA] Sin JOIN aun: se encola uplink para disparar OTAA");
        } else {
            Serial.println("[LORA] Solicitando envio...");
        }
        do_send();
        return;
    }

    int t2 = millis();//Se crea una variable del tiempo actual 
    
    if((t2-t1)>30000){ //Si ha pasado el tiempo de ejecucion de loop entramos en la funcion
        t1 = millis(); //Guardartel tiempo actual en T1
        Serial.println();
        Serial.println("=== CICLO DE ENVIO ===");
        printLinkStatus("Previo a lectura");
        doSensor(txBuffer); // Llama a la función doSensor con txBuffer
        printSensorTable(false); // Muestra la misma tabla al medir antes de enviar
        if (!loraJoined) {
            Serial.println("[LORA] Sin JOIN aun: se encola uplink para disparar OTAA");
        } else {
            Serial.println("[LORA] Solicitando envio...");
        }
        do_send(); // En OTAA, el primer uplink dispara el proceso de join automáticamente
    }
}

//----------------------------------------------------------------------------------------------------
// Función para el tramitado de datos de sensores 
void doSensor(uint8_t txBuffer[]) {
    // Llenar el búfer con caracteres nulos para borrar el contenido anterior
    memset(txBuffer, 0, TX_BUFFER_SIZE); //Creamos un paquete del numero de bytes elegido en configuracion
    
    //Para crear los bytes pasamos todos los valores a enteros y usamos la cantidad de bytes necesarios con cada medida 
    float t = sanitizeValue(bmeTemp());
    lastTemp = t;
    int shiftTemp = int(t * 100);
    txBuffer[0] = byte(shiftTemp);
    txBuffer[1] = shiftTemp >> 8;

    float p = sanitizeValue(bmePres());
    lastPressure = p / 100.0f;
    int shiftpresion = int(p * 100);
    txBuffer[2] = byte(shiftpresion);
    txBuffer[3] = shiftpresion >> 8;
    txBuffer[4] = shiftpresion >> 16;
    txBuffer[5] = 0;

    float a = sanitizeValue(bmeAlt());
    lastAltitude = a;
    int shiftAltura = int(a * 100);
    txBuffer[6] = byte(shiftAltura);
    txBuffer[7] = shiftAltura >> 8;

    float h = sanitizeValue(bmeHum());
    lastHumidity = h;
    int shifthumedad = int(h * 100);
    txBuffer[8] = byte(shifthumedad);
    txBuffer[9] = shifthumedad >> 8;

    float MQ135 = sanitizeValue(MQ135Value());
    lastGas = MQ135;
    int shiftMQ135 = int(MQ135*100);
    txBuffer[10] = byte(shiftMQ135);
    txBuffer[11] = shiftMQ135 >> 8;
    
    float CO = sanitizeValue(COValue());
    lastCO = CO;
    int shiftCO = int(CO*100);
    txBuffer[12] = byte(shiftCO);
    txBuffer[13] = shiftCO >> 8;

    PMS();
    int PM25 = int(sanitizeValue(p25)*100);
    lastPM25 = sanitizeValue(p25);
    txBuffer[14] = byte(PM25);
    txBuffer[15] = PM25 >> 8;

    int PM10 = int(sanitizeValue(p10)*100);
    lastPM10 = sanitizeValue(p10);
    txBuffer[16] = byte(PM10);
    txBuffer[17] = PM10 >> 8;
    
    float bat = sanitizeValue(ReadBattery());
    lastBattery = bat;
    int shiftbat = int(bat*100);
    txBuffer[18] = byte(shiftbat);
    txBuffer[19] = shiftbat >> 8;
    
    float CO2 = sanitizeValue(SCD30());
    lastCO2 = CO2;
    int shiftCO2 = int (CO2*10);
    
    txBuffer[20] = byte(shiftCO2);
    txBuffer[21] = shiftCO2 >> 8;

    #if SENSOR_KY038_ENABLED
    int ky = KY038Value();
    Serial.print("KY038 = ");
    Serial.println(ky);
    #endif

    return ;
}
// Función para dormir una cantidad de milisegundos
void sleep_millis(uint64_t ms) {
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_deep_sleep_start();
}
// Funcion para leer Temperatura con el BME280
float bmeTemp() {
  #if SENSOR_BME280_ENABLED
  if (!bmeReady) {
    return 0.0f;
  }
  return bme.readTemperature();
  #else
  return 0.0f;
  #endif
}
// Funcion para leer la presion con el BME280
float bmePres() {
  #if SENSOR_BME280_ENABLED
  if (!bmeReady) {
    return 0.0f;
  }
  return bme.readPressure();
  #else
  return 0.0f;
  #endif
}
// Funcion para medir la altitud con el BME 280
float bmeAlt() {
  #if SENSOR_BME280_ENABLED
  if (!bmeReady) {
    return 0.0f;
  }
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  return altitude + 196;
  #else
  return 0.0f;
  #endif
}
// Funcion para leer la humedad con el BME 280
float bmeHum() {
  #if SENSOR_BME280_ENABLED
  if (!bmeReady) {
    return 0.0f;
  }
  return bme.readHumidity();
  #else
  return 0.0f;
  #endif
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
            printStatusBlock("LORA CONECTANDO");          
            break;
        case EV_JOINED:
            //Conectado con el gateway
            loraJoined = true;            printStatusBlock("LORA CONECTADO");
          //Se establece el modo de verificacion (Desactivada)
            LMIC_setLinkCheckMode(0);
            break;
        //Si la union con el servidor LoRa falla
        case EV_JOIN_FAILED:                                             
            loraJoined = false;
            printStatusBlock("LORA JOIN FALLIDO");
            break;
        //Si la reintegracion con el servidor LoRa falla
        case EV_REJOIN_FAILED:                        
            loraJoined = false;
            printStatusBlock("LORA REJOIN FALLIDO");
            break;
        //Transmision completada
        case EV_TXCOMPLETE:                           
            printStatusBlock("TRANSMITIDO");
            if (loraTransmitStartTime != 0) {
              unsigned long elapsed = millis() - loraTransmitStartTime;
              Serial.print(F("[LORA] Tiempo TX: "));
              Serial.print(elapsed);
              Serial.println(F(" ms"));
              loraTransmitStartTime = 0;
            }
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("[LORA] ACK: SI"));
            else
              Serial.println(F("[LORA] ACK: NO"));            printLastMeasurements();
            
            printStatusBlock("DURMIENDO");
            //Se manda a dormir el microcontrolador el tiempo especificado 
            #if !DEBUG_MODE_ENABLED
            delay(100);
            sleep_millis(WAKE_TIME_MS);
            #else
            Serial.println("[SISTEMA] Deep sleep deshabilitado por DEBUG");
            #endif
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
            printStatusBlock("TRANSMITIENDO");          
            break;
        //Transimision cancelada
        case EV_TXCANCELED:                           
            Serial.println(F("[LORA] EV_TXCANCELED"));
            break;
        // Comienza la recepcion de datos
        case EV_RXSTART:                              
            
            break;
        //Se completa la transimision de union sin conectarse
        case EV_JOIN_TXCOMPLETE:                      
            Serial.println(F("[LORA] JoinAccept no recibido aun"));
            break;
        default:
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
        Serial.println(F("[LORA] Ocupado, envio pendiente"));
    } else {
        //Preparar la transmision de datos en proximo momento posible
        txAttemptCounter++;
        Serial.print(F("[LORA] Envio #"));
        Serial.print(txAttemptCounter);
        Serial.println(F(" encolado"));
        LMIC_setTxData2(1, txBuffer, sizeof(txBuffer)-1, 0); 
        loraTransmitStartTime = millis();
        Serial.println(F("[LORA] Listo para transmitir"));
    }
    // Siguiente transimision programada despues del evento TX_COMPLETE
}

void printLoraIdentifiers() {
  u1_t appEui[8];
  u1_t devEui[8];
  os_getArtEui(appEui);
  os_getDevEui(devEui);

  Serial.print("[LORA] DevEUI: ");
  for (int i = 0; i < 8; i++) {
    if (i) Serial.print("-");
    printHex2(devEui[i]);
  }
  Serial.println();

  Serial.print("[LORA] AppEUI: ");
  for (int i = 0; i < 8; i++) {
    if (i) Serial.print("-");
    printHex2(appEui[i]);
  }
  Serial.println();
}

void printPayloadHex(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (i) Serial.print(" ");
    printHex2(data[i]);
  }
  Serial.println();
}

void printLinkStatus(const char* tag) {
  Serial.print("[LORA] Estado (");
  Serial.print(tag);
  Serial.print(") joined=");
  Serial.print(loraJoined ? "SI" : "NO");
  Serial.print(" txrxpend=");
  Serial.print((LMIC.opmode & OP_TXRXPEND) ? "SI" : "NO");
  Serial.print(" seqUp=");
  Serial.println(LMIC.seqnoUp);
}

void printStatusBlock(const char* title) {
  Serial.println();
  Serial.println("=================================");
  Serial.print("==== ");
  Serial.print(title);
  Serial.println(" ====");
  Serial.println("=================================");
}

void printLastMeasurements() {
  Serial.println("[DATOS] Envio:");
  Serial.print(" - Temperatura: "); Serial.print(lastTemp, 2); Serial.println(" C");
  Serial.print(" - Presion: "); Serial.print(lastPressure, 2); Serial.println(" hPa");
  Serial.print(" - Humedad: "); Serial.print(lastHumidity, 2); Serial.println(" %");
  Serial.print(" - Altitud: "); Serial.print(lastAltitude, 2); Serial.println(" m");
  Serial.print(" - Gas MQ135: "); Serial.print(lastGas, 2); Serial.println(" ppm");
  Serial.print(" - CO MQ7: "); Serial.print(lastCO, 2); Serial.println(" ppm");
  Serial.print(" - PM2.5: "); Serial.print(lastPM25, 2); Serial.println(" ug/m3");
  Serial.print(" - PM10: "); Serial.print(lastPM10, 2); Serial.println(" ug/m3");
  Serial.print(" - Bateria: "); Serial.print(lastBattery, 2); Serial.println(" %");
  Serial.print(" - CO2: "); Serial.print(lastCO2, 1); Serial.println(" ppm");
}
// Medida CO2
float MQ135Value() {
  #if SENSOR_MQ135_ENABLED
  if (!mq135Ready) {
    return 0.0f;
  }
  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
  MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  return MQ135.getPPM();
  #else
  return 0.0f;
  #endif
}
// Medida CO (Monóxido de carbono)
float COValue (){
  #if SENSOR_MQ7_ENABLED
  if (!mq7Ready) {
    return 0.0f;
  }
  delay(1000);
  return mq7.readPpm();
  #else
  return 0.0f;
  #endif
}
// Medida partículas (PM10 y PM25)
void PMS (){
  #if SENSOR_SDS011_ENABLED
  if (!sds011Ready) {
    p25 = 0.0f;
    p10 = 0.0f;
    return;
  }
  error = my_sds.read(&p25, &p10);
  if (error) {
    p25 = 0.0f;
    p10 = 0.0f;
  }
  #else
  p25 = 0.0f;
  p10 = 0.0f;
  #endif
}
// Medida CO2
float SCD30(){
  #if SENSOR_SCD30_ENABLED
  if (!scd30Ready) {
    return 0.0f;
  }
  float CO_2 = 0.0f;

  if (scd30.dataReady()){
    if (!scd30.read()){ Serial.println("Error reading sensor data"); return 0; }
    CO_2 = scd30.CO2;
  }
  return CO_2;
  #else
  return 0.0f;
  #endif
}
// Medida de estado de carga de batería
float ReadBattery() {

  int analogBat = analogRead(BatteryPin);//Se lee el valor analogico en el pin 12 
  float digitalBat = (analogBat - 0) * (2.7f - 0) / (3000 - 0); // Como el voltaje esta dividido por un divisor solo leemos la mitad de los valores
  float batteryVoltage = digitalBat * 1.56f; // Multiplicamos por el factor del divisor para obtener el valor real

  // 0% = 2.4V, 100% = 4.2V
  float batteryPercent = ((batteryVoltage - 2.4f) / (4.2f - 2.4f)) * 100.0f;
  if (batteryPercent < 0.0f) batteryPercent = 0.0f;
  if (batteryPercent > 100.0f) batteryPercent = 100.0f;

  return batteryPercent;
}

int KY038Value() {
  #if SENSOR_KY038_ENABLED
  return analogRead(KY038_PIN);
  #else
  return 0;
  #endif
}

static const char* tf(bool v) {
  return v ? "ON" : "OFF";
}

static void printSensorRow(const char* sensor, bool enabled, const String& data) {
  Serial.printf("| %-12s | %-6s | %s\r\n", sensor, tf(enabled), data.c_str());
}

void printDebugConfig() {
  Serial.println();
  Serial.println("=== DEBUG MODE: CONFIGURACION ===");
  Serial.println("Sensores disponibles (ON) y deshabilitados (OFF):");
  Serial.printf("- BME280: %s\r\n", tf(SENSOR_BME280_ENABLED));
  Serial.printf("- SCD30 : %s\r\n", tf(SENSOR_SCD30_ENABLED));
  Serial.printf("- MQ7   : %s\r\n", tf(SENSOR_MQ7_ENABLED));
  Serial.printf("- MQ135 : %s\r\n", tf(SENSOR_MQ135_ENABLED));
  Serial.printf("- SDS011: %s\r\n", tf(SENSOR_SDS011_ENABLED));
  Serial.printf("- KY038 : %s\r\n", tf(SENSOR_KY038_ENABLED));
}

void printDebugProbeResults() {
  Serial.println();
  Serial.println("=== DEBUG MODE: DETECCION Y ERRORES ===");
  Serial.printf("- BME280: %s\r\n", SENSOR_BME280_ENABLED ? (bmeReady ? "OK" : "ERROR") : "OFF");
  Serial.printf("- SCD30 : %s\r\n", SENSOR_SCD30_ENABLED ? (scd30Ready ? "OK" : "ERROR") : "OFF");
  Serial.printf("- MQ7   : %s\r\n", SENSOR_MQ7_ENABLED ? (mq7Ready ? "OK" : "ERROR") : "OFF");
  Serial.printf("- MQ135 : %s\r\n", SENSOR_MQ135_ENABLED ? (mq135Ready ? "OK" : "ERROR") : "OFF");
  Serial.printf("- SDS011: %s\r\n", SENSOR_SDS011_ENABLED ? (sds011Ready ? "OK" : "ERROR") : "OFF");
  Serial.printf("- KY038 : %s\r\n", SENSOR_KY038_ENABLED ? (ky038Ready ? "OK" : "ERROR") : "OFF");
}

void printSensorTable(bool periodicDebugView) {
  float t = sanitizeValue(bmeTemp());
  float p = sanitizeValue(bmePres()) / 100.0f;
  float h = sanitizeValue(bmeHum());
  float a = sanitizeValue(bmeAlt());
  float co2 = sanitizeValue(SCD30());
  float co = sanitizeValue(COValue());
  float mq135 = sanitizeValue(MQ135Value());
  PMS();
  float pm25 = sanitizeValue(p25);
  float pm10 = sanitizeValue(p10);
  float bat = sanitizeValue(ReadBattery());
  int ky = KY038Value();

  Serial.println();
  if (periodicDebugView) {
    Serial.println("=== DEBUG MODE: TABLA SENSORES (cada 0.2m) ===");
  } else {
    Serial.println("=== TABLA SENSORES: MEDICION PREVIA A ENVIO ===");
  }
  Serial.println("| Sensor       | ON/OFF | Datos");
  Serial.println("|--------------|--------|-----------------------------------------------");
  printSensorRow("BME280", SENSOR_BME280_ENABLED, bmeReady ? ("T=" + String(t, 2) + "C P=" + String(p, 2) + "hPa H=" + String(h, 2) + "% Alt=" + String(a, 2) + "m") : "0");
  printSensorRow("SCD30", SENSOR_SCD30_ENABLED, scd30Ready ? ("CO2=" + String(co2, 1) + "ppm") : "0");
  printSensorRow("MQ7", SENSOR_MQ7_ENABLED, mq7Ready ? ("CO=" + String(co, 2) + "ppm") : "0");
  printSensorRow("MQ135", SENSOR_MQ135_ENABLED, mq135Ready ? ("Gas=" + String(mq135, 2) + "ppm") : "0");
  printSensorRow("SDS011", SENSOR_SDS011_ENABLED, sds011Ready ? ("PM2.5=" + String(pm25, 2) + " PM10=" + String(pm10, 2)) : "0");
  printSensorRow("KY038", SENSOR_KY038_ENABLED, ky038Ready ? ("Raw=" + String(ky)) : "0");
  printSensorRow("Battery", true, String(bat, 2) + "%");
}





