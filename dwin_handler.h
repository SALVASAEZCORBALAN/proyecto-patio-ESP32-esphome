#ifndef DWIN_HANDLER_H
#define DWIN_HANDLER_H

#include "esphome.h"
#include "driver/twai.h" // Driver oficial de Espressif para el CAN Bus interno (TWAI)
#include <Adafruit_BME680.h>      // Librería nativa para sensor ambiental

// 🚀 DECLARACIÓN DEL PUNTERO GLOBAL PARA EL CONFIGURADOR YAML
class DWINHandler;
extern DWINHandler *my_custom_node;

// --- SUBRUTINAS DE CONVERSIÓN DE TIPOS DE DATOS ---
inline void FloatToHex(float f, uint8_t* hex) {
    uint8_t* f_byte = reinterpret_cast<uint8_t*>(&f);
    memcpy(hex, f_byte, 4);
}

inline void LargoToHex(unsigned long f, uint8_t* hex) {
    hex[0] = (f >> 8) & 0xFF;
    hex[1] = f & 0xFF;
}

// --- CLASE PRINCIPAL GESTORA DE HARDWARE (DWIN + CAN) ---
class DWINHandler : public esphome::Component {
    public:
        esphome::uart::UARTComponent *uart_bus = nullptr; // 🚀 Inicializado a un estado inicial seguro del Puntero al canal serie físico de la pantalla

// Variables globales del BME680 replicadas de tu Arduino
Adafruit_BME680 *bme;
unsigned long ultimaLecturaBME = 0;
const unsigned long intervaloBME = 60000; // Muestreo cada 60 segundos igual que tu Arduino

// Búferes y matrices de estado idénticos a los del firmware original de Arduino
uint8_t Bufer_1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Bufer_2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Bufer_H5A[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Estado canales piscina

// Matrices de tramas indexadas secuencialmente (8 bytes por trama para IDs 0x052 a 0x059)
uint8_t Bufer_H52[8] = {0, 0, 0, 0, 0, 0, 0, 0}; uint8_t Bufer_H53[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Bufer_H54[8] = {0, 0, 0, 0, 0, 0, 0, 0}; uint8_t Bufer_H55[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Bufer_H56[8] = {0, 0, 0, 0, 0, 0, 0, 0}; uint8_t Bufer_H57[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Bufer_H58[8] = {0, 0, 0, 0, 0, 0, 0, 0}; uint8_t Bufer_H59[8] = {0, 0, 0, 0, 0, 0, 0, 0};
// --- VARIABLES PARA EFECTOS RGB EN TIEMPO REAL (SIN DELAY) ---
uint8_t modo_efecto_rgb = 0; 
unsigned long ultimo_millis_efecto = 0;
int paso_efecto = 0;
bool estado_estrobo = false;
int direccion_pulso = 1;
int intensidad_pulso = 0;

// Mapeo físico de tus salidas a relés locales (Matriz Zona_1 de Arduino)
const uint8_t Zona_1[8] = {13, 14, 27, 26, 25, 33, 0, 22};

// Constructor: Recibe la UART de la pantalla
DWINHandler(esphome::uart::UARTComponent *uart_parent) : uart_bus(uart_parent) {}


// Inicialización del hardware nativo al arrancar el ESP32
void setup() override {
    my_custom_node = this; // 🚀 Enlaza el componente con la variable global del YAML

    // 1. Inicialización de relés locales
    for(int i=0; i<8; i++) {
        pinMode(Zona_1[i], OUTPUT);
        digitalWrite(Zona_1[i], LOW);
    }
    // 2. Inicialización del BME680 por SPI por software (Pin GPIO0 como MOSI)
    bme = new Adafruit_BME680(21, 0, 19, 18);
    if (bme->begin()) {
        ESP_LOGI("BME680", "Sensor BME680 inicializado correctamente por SPI por software");
        bme->setTemperatureOversampling(BME680_OS_8X);
        bme->setHumidityOversampling(BME680_OS_2X);
        bme->setPressureOversampling(BME680_OS_4X);
        bme->setIIRFilterSize(BME680_FILTER_SIZE_3);
    } else {
        ESP_LOGE("BME680", "¡Error crítico al inicializar el BME680!");
    }
    // 3. Configuración del Driver CAN Bus (TWAI) a 125 kbps (Pines GPIO5 y GPIO4)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)5, (gpio_num_t)4, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        ESP_LOGI("CAN_BUS", "Driver CAN Bus (TWAI) inicializado con éxito a 125kbps");
    } else {
        ESP_LOGE("CAN_BUS", "¡Error crítico al instalar el driver del CAN Bus!");
    }
}

// --- SUBRUTINA DE ENVÍO DIRECTO DE TRAMAS CAN ---
void enviarTramaCAN(uint32_t id, std::vector<uint8_t> data) {
    twai_message_t tx_msg;
    tx_msg.identifier = id;
    tx_msg.extd = 0; // Trama estándar de 11 bits
    tx_msg.rtr = 0;
    tx_msg.data_length_code = data.size() > 8 ? 8 : data.size();

    for (size_t i = 0; i < tx_msg.data_length_code; i++) {
        tx_msg.data[i] = data[i];
    }

    if (twai_transmit(&tx_msg, pdMS_TO_TICKS(10)) != ESP_OK) {
        ESP_LOGD("CAN_BUS", "Error al enviar trama por el bus CAN");
    }

}

void enviaValoresFlotantes(float floatValue, uint8_t VP_1, uint8_t VP_2) {
    // 🚀 PROTECCIÓN: Si el nodo global no apunta a este objeto o la UART no está lista, salimos
    if (my_custom_node != this || uart_bus == nullptr) return;

    uint8_t hex[4] = {0, 0, 0, 0}; // Array de 4 bytes para almacenar el float
    FloatToHex(floatValue, hex);
    uart_bus->write_array({0x5A, 0xA5, 0x07, 0x82, VP_1, VP_2, hex[0], hex[1], hex[2], hex[3]});
}

void enviaValoresLargos(unsigned long LargoValue, uint8_t VP_1, uint8_t VP_2) {
    // 🚀 PROTECCIÓN: Si el nodo global no apunta a este objeto o la UART no está lista, salimos
    if (my_custom_node != this || uart_bus == nullptr) return;

    uint8_t hex[2] = {0, 0};
    LargoToHex(LargoValue, hex);
    uart_bus->write_array({0x5A, 0xA5, 0x05, 0x82, VP_1, VP_2, hex[0], hex[1]});
}
void limpia_Bufer() { for (int i = 0; i < 8; i++) { Bufer_1[i] = 0; } }

void monta_CAN(int Puntero) {
    if (Puntero == 4) {
        Bufer_H53[Puntero] = Bufer_1[Puntero]; Bufer_H59[Puntero] = Bufer_1[Puntero];
    } else {
        Bufer_H52[Puntero] = Bufer_1[Puntero]; Bufer_H53[Puntero] = Bufer_1[Puntero];
        Bufer_H54[Puntero] = Bufer_1[Puntero]; Bufer_H55[Puntero] = Bufer_1[Puntero];
        Bufer_H56[Puntero] = Bufer_1[Puntero]; Bufer_H57[Puntero] = Bufer_1[Puntero];
        Bufer_H58[Puntero] = Bufer_1[Puntero]; Bufer_H59[Puntero] = Bufer_1[Puntero];
    }
}

void envia_Bufer() {
    for (int i = 0; i < 8; i++) {
        for (int a = 0; a < 8; a++) {
        if (i == 0) Bufer_2[a] = Bufer_H52[a];
        if (i == 1) Bufer_2[a] = Bufer_H53[a];
        if (i == 2) Bufer_2[a] = Bufer_H54[a];
        if (i == 3) Bufer_2[a] = Bufer_H55[a];
        if (i == 4) Bufer_2[a] = Bufer_H56[a];
        if (i == 5) Bufer_2[a] = Bufer_H57[a];
        if (i == 6) Bufer_2[a] = Bufer_H58[a];
        if (i == 7) Bufer_2[a] = Bufer_H59[a];

        }
    std::vector<uint8_t> can_data(Bufer_2, Bufer_2 + 8);
    enviarTramaCAN(0x052 + i, can_data);
    }
}

void monta_CAN2() {
    std::vector<uint8_t> can_data(Bufer_H5A, Bufer_H5A + 8);
    enviarTramaCAN(0x5A, can_data);
}

void resincronizarTodo(int pColor, int pRetardo, int pDura, int pPot, int pFiltra) {
        ESP_LOGI("STARTUP", "Sincronizando la memoria permanente con la DWIN...");
        
        // Asignamos cada valor a su posición de byte correspondiente dentro del array [8]
        Bufer_H5A[0] = (uint8_t)pColor;
        Bufer_H5A[1] = (uint8_t)pRetardo;
        Bufer_H5A[2] = (uint8_t)pDura;
        Bufer_H5A[3] = (uint8_t)pPot;
        Bufer_H5A[4] = (uint8_t)pFiltra; 

        // Enviamos el búfer empaquetado al CAN Bus y actualizamos la pantalla DWIN
        monta_CAN2();
        envia_Bufer();

        enviaValoresLargos(pColor, 0x36, 0x00);
        enviaValoresLargos(pRetardo, 0x37, 0x00);
        enviaValoresLargos(pDura, 0x37, 0x05);
        enviaValoresLargos(pPot, 0x37, 0x10);
        enviaValoresLargos(pFiltra, 0x20, 0x40);
    } 

// --- ESCUCHA PROACTIVA DE TRAMAS ENTRANTES CAN ---

void procesarCanEntrante(uint32_t can_id, std::vector<uint8_t> data) {
    if (can_id == 0x210 && data.size() >= 2) {
        uint16_t raw_presion = (data[0] << 8) | data[1];
        float presion_bar = (float)raw_presion / 100.0f;
        enviaValoresFlotantes(presion_bar, 0x40, 0x00);
    }

    if (can_id == 0x31E && data.size() >= 2) { enviaValoresLargos((data[0] << 8) | data[1], 0x50, 0x00); }
    if (can_id == 0x32E && data.size() >= 2) { enviaValoresLargos((data[0] << 8) | data[1], 0x50, 0x10); }
    if (can_id == 0x33E && data.size() >= 2) { enviaValoresLargos((data[0] << 8) | data[1], 0x50, 0x20); }
}


void procesarEventoHMI(int registro, int lastByte) {
    int Puntero = 0;
    switch (registro) {
    	case 3000: limpia_Bufer(); Puntero = 1; lee_DWIN(lastByte); monta_CAN(Puntero); envia_Bufer(); break;
    	case 3100: limpia_Bufer(); Puntero = 2; lee_DWIN(lastByte); monta_CAN(Puntero); envia_Bufer(); break;
    	case 3200: limpia_Bufer(); Puntero = 3; lee_DWIN(lastByte); monta_CAN(Puntero); envia_Bufer(); break;
    	case 3300: canal_33(lastByte); delay(100); Puntero = 4; canal_33CAN(lastByte); monta_CAN(Puntero); envia_Bufer(); break;
    	case 3400: canal_34(lastByte); break;
    	case 3500: Bufer_H5A[0] = lastByte; monta_CAN2(); break;
    	case 3600: Bufer_H5A[0] = lastByte; monta_CAN2(); break;
    	case 3700: Bufer_H5A[1] = lastByte; monta_CAN2(); break;
    	case 3705: Bufer_H5A[2] = lastByte; monta_CAN2(); break;
    	case 3710: Bufer_H5A[3] = lastByte; monta_CAN2(); break;
    	case 2040: Bufer_H5A[4] = lastByte; monta_CAN2(); break;
    	// 🚀 INTERCEPTAMOS EL CONTROL RGB DE LA PANTALLA
        case 6000: modo_efecto_rgb = lastByte; break;
    }
}

void escucharCanBus() {
    twai_message_t rx_msg;
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (!(rx_msg.rtr)) {
            uint32_t can_id = rx_msg.identifier;
            std::vector<uint8_t> data(rx_msg.data, rx_msg.data + rx_msg.data_length_code);
            procesarCanEntrante(can_id, data);
        }
    }
}


// --- BUCLE DE ESCUCHA CONTINUO SERIE (Implementado en C++ Nativo) ---
void loop() override {
    // 1. Lectura temporizada y arranque seguro del BME680 con protección de desconexión
        unsigned long actualMillis = millis();
        static bool sensor_bme_disponible = true; // Permite saltar la lectura si no está conectado

        if (sensor_bme_disponible && (actualMillis - ultimaLecturaBME >= intervaloBME)) {
            ultimaLecturaBME = actualMillis;
            
            // Si es la primera vez, intentamos instanciar el sensor de forma segura
            if (bme == nullptr) {
                bme = new Adafruit_BME680(21, 0, 19, 18);
                if (bme->begin()) {
                    bme->setTemperatureOversampling(BME680_OS_8X);
                    bme->setHumidityOversampling(BME680_OS_2X);
                    bme->setPressureOversampling(BME680_OS_4X);
                    bme->setIIRFilterSize(BME680_FILTER_SIZE_3);
                    ESP_LOGI("BME680", "Sensor BME680 detectado e inicializado con éxito");
                } else {
                    // 🚀 PROTECCIÓN: Si no está conectado, desactivamos futuras lecturas para no bloquear el ESP32
                    ESP_LOGW("BME680", "¡Sensor BME680 no detectado físicamente! Desactivando telemetría para evitar bloqueos.");
                    sensor_bme_disponible = false;
                    delete bme;
                    bme = nullptr;
                }
            }

            // Solo lee si la inicialización previa fue exitosa
            if (bme && bme->performReading()) {
                float temp = bme->temperature;
                enviaValoresFlotantes(temp, 0x20, 0x00);
                ESP_LOGD("BME680", "Lectura correcta - Temperatura: %.2f C", temp);
            }
        }

    // 2. Escucha e interpretación de la UART de la pantalla DWIN
        static uint8_t buffer_dwin[30]; // Definimos las dimensiones del array del buffer
        static size_t idx = 0;
        
        uint8_t b;
        while (uart_bus->available() && uart_bus->read_byte(&b)) {
            if (idx == 0 && b != 0x5A) continue;
            if (idx == 1 && b != 0xA5) { idx = 0; continue; }
            
            buffer_dwin[idx++] = b;
            
            if (idx >= 4) {
                uint8_t longitud = buffer_dwin[2];
                if (idx == (size_t)(longitud + 3)) {
                    uint8_t comando = buffer_dwin[3];
                    if (comando == 0x83) { // Respuesta de lectura / pulsación VP
                        int direccion_vp = (buffer_dwin[4] << 8) | buffer_dwin[5];
                        int ultimo_byte = buffer_dwin[idx - 1];
                        int valor_completo = (buffer_dwin[idx - 2] << 8) | buffer_dwin[idx - 1];

                        // Sincronización con las variables globales persistentes de ESPHome
                        if (direccion_vp == 3600) id(p_color) = ultimo_byte;
                        if (direccion_vp == 3700) id(p_retardo) = ultimo_byte;
                        if (direccion_vp == 3705) id(p_dura) = ultimo_byte;
                        if (direccion_vp == 3710) id(p_pot) = ultimo_byte;
                        if (direccion_vp == 2040) id(p_filtra) = ultimo_byte;
                        
                        // Actualización de entidades del Dashboard de HA desde la pantalla
                        if (direccion_vp == 3800) id(hz_variador_filtracion).publish_state((float)valor_completo / 10.0f);
                        if (direccion_vp == 3900) {
                            if (ultimo_byte == 1) id(bomba_cascada).turn_on();
                            else id(bomba_cascada).turn_off();
                        }
                        if (direccion_vp == 6000) {
                            auto call = id(rgb_cascada_virtual).make_call();
                            if (ultimo_byte == 0) {
                                call.set_state(false);
                            } else {
                                call.set_state(true);
                                if (ultimo_byte == 1) call.set_rgb(1.0f, 0.0f, 0.0f);
                                if (ultimo_byte == 2) call.set_rgb(0.0f, 1.0f, 0.0f);
                                if (ultimo_byte == 3) call.set_rgb(0.0f, 0.0f, 1.0f);
                                if (ultimo_byte == 4) call.set_rgb(1.0f, 1.0f, 1.0f);
                            }
                            call.perform();
                        }
                        procesarEventoHMI(direccion_vp, ultimo_byte);
                    }
                    idx = 0;
                }
            }
            if (idx >= sizeof(buffer_dwin)) { idx = 0; }
        }

    // 4. MOTOR DE EFECTOS RGB DINÁMICOS ENVIADOS AL ARDUINO NANO POR CAN BUS
        unsigned long millis_actuales = millis();

        if (modo_efecto_rgb >= 5) {
            std::vector<uint8_t> trama_rgb(8, 0);
            trama_rgb[0] = 1; // Indicador de encendido para el Nano

            // --- EFECTO 5: ARCOÍRIS DINÁMICO (FADE SUAVE) ---
            if (modo_efecto_rgb == 5 && (millis_actuales - ultimo_millis_efecto >= 30)) {
                ultimo_millis_efecto = millis_actuales;
                paso_efecto++;
                uint8_t pos = paso_efecto & 0xFF;
                if (pos < 85) {
                    trama_rgb[1] = 255 - pos * 3; trama_rgb[2] = pos * 3; trama_rgb[3] = 0;
                } else if (pos < 170) {
                    pos -= 85;
                    trama_rgb[1] = 0; trama_rgb[2] = 255 - pos * 3; trama_rgb[3] = pos * 3;
                } else {
                    pos -= 170;
                    trama_rgb[1] = pos * 3; trama_rgb[2] = 0; trama_rgb[3] = 255 - pos * 3;
                }
                enviarTramaCAN(0x350, trama_rgb);
            }

            // --- EFECTO 6: ESTROBO RÁPIDO (FLASH FLUIDO) ---
            if (modo_efecto_rgb == 6 && (millis_actuales - ultimo_millis_efecto >= 100)) {
                ultimo_millis_efecto = millis_actuales;
                estado_estrobo = !estado_estrobo;
                if (estado_estrobo) {
                    trama_rgb[1] = 255; trama_rgb[2] = 255; trama_rgb[3] = 255;
                } else {
                    trama_rgb[1] = 0; trama_rgb[2] = 0; trama_rgb[3] = 0;
                }
                enviarTramaCAN(0x350, trama_rgb);
            }

            // --- EFECTO 7: RESPIRACIÓN AZULADURA (AGUA CRISTALINA) ---
            if (modo_efecto_rgb == 7 && (millis_actuales - ultimo_millis_efecto >= 15)) {
                ultimo_millis_efecto = millis_actuales;
                intensidad_pulso += (direccion_pulso * 5);
                if (intensidad_pulso >= 255) { intensidad_pulso = 255; direccion_pulso = -1; }
                if (intensidad_pulso <= 10)  { intensidad_pulso = 10;  direccion_pulso = 1; }
                trama_rgb[1] = 0; 
                trama_rgb[2] = intensidad_pulso / 3; 
                trama_rgb[3] = intensidad_pulso; 
                enviarTramaCAN(0x350, trama_rgb);
            }
        }
// 3. Escucha activa de tramas en el transceptor CAN Bus
escucharCanBus(); 
}

private:
void lee_DWIN(int lastByte) { for (int i = 0; i < 8; i++) { Bufer_1[i] = ((lastByte >> i) & 1) ? 1 : 0; } }
void canal_33(int lastByte) { for (int i = 0; i < 2; i++) { digitalWrite(Zona_1[i], ((lastByte >> i) & 1) ? HIGH : LOW); } }
void canal_33CAN(int lastByte) { for (int i = 2; i < 4; i++) { Bufer_1[i] = ((lastByte >> i) & 1) ? 1 : 0; } }
void canal_34(int lastByte) { for (int i = 0; i < 2; i++) { digitalWrite(Zona_1[i + 2], ((lastByte >> i) & 1) ? HIGH : LOW); } }
};

DWINHandler *my_custom_node = nullptr;

#endif // DWIN_HANDLER_H
