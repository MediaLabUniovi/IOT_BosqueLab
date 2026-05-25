# BosqueLab LoRa Node (ESP32 + TTN)

Proyecto de monitorización ambiental para bosque con envío por LoRaWAN (TTN).

## Resumen rápido

- Hardware base: `ttgo-lora32-v21` (ESP32 + LoRa).
- Sensores soportados: `BME280`, `MQ7`, `MQ135`, `SDS011`, `SCD30`, `KY038`.
- Comunicación: LoRaWAN OTAA (LMIC) hacia TTN.
- Ciclo: medir -> enviar uplink -> deep sleep -> despertar -> repetir.

## Configuración principal

Toda la configuración importante está en:

- [project_config.h]

Ahí debes configurar:

1. Sensores ON/OFF:
- `#define SENSOR_..._ENABLED 1` para activar.
- `#define SENSOR_..._ENABLED 0` para desactivar.

2. Tiempo de deep sleep:
- `#define WAKE_TIME_MS 10000` (10 segundos).

3. Credenciales TTN OTAA:
- `APPEUI` (LSB)
- `DEVEUI` (LSB)
- `APPKEY` (MSB)

Si cambias de dispositivo en TTN, actualiza esas tres claves.

## Flujo de funcionamiento

1. Arranque/reset:
- Inicializa serie y sensores.
- Activa transistor de alimentación de sensores (`PowerPin=2`) y espera.
- Calibra `MQ7` y `MQ135`.
- Inicializa LMIC y queda a la espera de JOIN OTAA.

2. Primer envío tras reset:
- Se hace un primer envío inmediato (sin esperar 30 s).
- Si aún no hay JOIN, ese uplink dispara el proceso OTAA.

3. Ciclo normal:
- Cada ~30 s en activo toma mediciones.
- Construye payload binario compacto.
- Encola uplink LoRa.
- Al `EV_TXCOMPLETE` entra en deep sleep (`WAKE_TIME_MS`).

4. Tras despertar:
- Repite setup, calibración, medida y envío.

## Orden del payload uplink

Se envían `22 bytes` (`sizeof(txBuffer)-1`), en este orden:

1. `bytes 0-1`: Temperatura BME280 (`C * 100`, int16)
2. `bytes 2-5`: Presión BME280 (`Pa * 100`, usando 3 bytes + 1 reservado)
3. `bytes 6-7`: Altitud (`m * 100`, int16)
4. `bytes 8-9`: Humedad (`% * 100`, int16)
5. `bytes 10-11`: MQ135 (`ppm * 100`, int16)
6. `bytes 12-13`: MQ7 CO (`ppm * 100`, int16)
7. `bytes 14-15`: PM2.5 SDS011 (`*100`, int16)
8. `bytes 16-17`: PM10 SDS011 (`*100`, int16)
9. `bytes 18-19`: Batería (`% * 100`, int16)
10. `bytes 20-21`: CO2 SCD30 (`ppm * 10`, int16)

Nota:
- Si un sensor está desactivado o no listo, su valor se envía en `0`.

## Logs/estado por serial

El firmware está ajustado para mostrar estado legible, por ejemplo:

- `INICIO`
- `LORA CONECTANDO`
- `LORA CONECTADO`
- `TRANSMITIENDO`
- `TRANSMITIDO`
- `DURMIENDO`

Además, tras transmitir imprime los datos medidos en formato humano (no payload HEX).

## Cómo usarlo (PlatformIO)

1. Configura `src/project_config.h`:
- Sensores ON/OFF.
- Credenciales TTN OTAA.
- `WAKE_TIME_MS`.

2. Compila:
```powershell
platformio run
```

3. Flashea:
```powershell
platformio run --target upload
```

4. Monitor serie:
```powershell
platformio device monitor
```

## Archivo de entorno

- [platformio.ini]

Configura board, flags de compilación y velocidad de monitor.


## Nota importante sobre la PCB

El dise�o original de la placa tiene un error en la parte inferior (`BottomPCB.jpeg`).
Es imprescindible rascar/cortar la pista marcada en negro en esa imagen antes de usar la PCB, para evitar el problema del ruteo original.

## Créditos

Proyecto original de Jota y Miguel, reciclado y mejorado por José.
