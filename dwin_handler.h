#include "esphome.h"
#include "driver/twai.h" // Driver oficial de Espressif para el CAN Bus interno (TWAI) 

// --- SUBRUTINAS DE CONVERSIÓN DE TIPOS DE DATOS ---
void FloatToHex(float f, uint8_t* hex) {
uint8_t* f_byte = reinterpret_cast<uint8_t*>(&f);
memcpy(hex, f_byte, 4);
} 

void LargoToHex(unsigned long f, uint8_t* hex) {
uint8_t* f_byte = reinterpret_cast<uint8_t*>(&f);
memcpy(hex, f_byte, 2);
} 

// --- CLASE PRINCIPAL GESTORA DE HARDWARE (DWIN + CAN) ---
class DWINHandler : public esphome::Component {
public:
esphome::uart::UARTComponent *uart_bus; // Puntero al canal serie físico de la pantalla 

// Variables globales del BME680 replicadas de tu Arduino
Adafruit_BME680 *bme;
unsigned long ultimaLecturaBME = 0;
const unsigned long intervaloBME = 60000; // Muestreo cada 60 segundos igual que tu Arduino

// Búferes y matrices de estado idénticos a los del firmware original de Arduino
uint8_t Bufer_1 = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Bufer_2 = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Bufer_H5A = {0, 0, 0, 0, 0, 0, 0, 0}; // Estado canales piscina 

// Matrices de tramas indexadas secuencialmente (8 bytes por trama para IDs 0x052 a 0x059)
uint8_t Bufer_H52 = {0, 0, 0, 0, 0, 0, 0, 0}; uint8_t Bufer_H53 = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Bufer_H54 = {0, 0, 0, 0, 0, 0, 0, 0}; uint8_t Bufer_H55 = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Bufer_H56 = {0, 0, 0, 0, 0, 0, 0, 0}; uint8_t Bufer_H57 = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t Bufer_H58 = {0, 0, 0, 0, 0, 0, 0, 0}; uint8_t Bufer_H59 = {0, 0, 0, 0, 0, 0, 0, 0}; 

// Mapeo físico de tus salidas a relés locales (Matriz Zona_1 de Arduino)
const uint8_t Zona_1 = {13, 14, 27, 26, 25, 33, 0, 22}; 

// Constructor: Recibe la UART de la pantalla
DWINHandler(esphome::uart::UARTComponent *uart_parent) : uart_bus(uart_parent) {} 

// Inicialización del hardware nativo al arrancar el ESP32
void setup() override {
// 1. Inicialización de relés locales
for(int i=0; i<8; i++) { pinMode(Zona_1[i], OUTPUT); } 

// 2. Inicialización del BME680 por SPI por software (Absorbiendo tu parche del pin GPIO0 como MOSI)
// Parámetros: BME_CS=21, BME_MOSI=0, BME_MISO=19, BME_SCK=18
bme = new Adafruit_BME680(21, 0, 19, 18);
if (bme->begin()) {
esphome::ESP_LOGI("BME680", "Sensor BME680 inicializado correctamente por SPI por software");
bme->setTemperatureOversampling(BME680_OS_8X);
bme->setHumidityOversampling(BME680_OS_2X);
bme->setPressureOversampling(BME680_OS_4X);
bme->setIIRFilterSize(BME680_FILTER_SIZE_3);
} else {
esphome::ESP_LOGE("BME680", "¡Error crítico al inicializar el BME680!");
}
// 3. Configuración e Inicio del Driver CAN Bus interno a 125 kbps (Pines GPIO5 y GPIO4)
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)5, (gpio_num_t)4, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
twai_start();
esphome::ESP_LOGI("CAN_BUS", "Driver CAN Bus (TWAI) inicializado con éxito a 125kbps");
} else {
esphome::ESP_LOGE("CAN_BUS", "¡Error crítico al instalar el driver del CAN Bus!");
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
esphome::ESP_LOGD("CAN_BUS", "Error al enviar trama por el bus CAN");
}

} 

void enviaValoresFlotantes(float floatValue, uint8_t VP_1, uint8_t VP_2) {
uint8_t hex = {0};
FloatToHex(floatValue, hex);
uart_bus->write_array({0x5A, 0xA5, 0x07, 0x82, VP_1, VP_2, hex, hex, hex, hex});
} 

void enviaValoresLargos(unsigned long LargoValue, uint8_t VP_1, uint8_t VP_2) {
uint8_t hex = {0};
LargoToHex(LargoValue, hex);
uart_bus->write_array({0x5A, 0xA5, 0x05, 0x82, VP_1, VP_2, hex, hex});
} 

void resincronizarTodo(int pColor, int pRetardo, int pDura, int pPot, int pFiltra) {
esphome::ESP_LOGI("STARTUP", "Sincronizando la memoria permanente con la DWIN...");
Bufer_H5A = pColor;
Bufer_H5A = pRetardo;
Bufer_H5A = pDura;
Bufer_H5A = pPot;
Bufer_H5A = pFiltra; 

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
if (can_id == 0x210 && rx_msg.data_length_code >= 2) {
uint16_t raw_presion = (rx_msg.data << 8) | rx_msg.data;
float presion_bar = (float)raw_presion / 100.0;
enviaValoresFlotantes(presion_bar, 0x40, 0x00);
}
if (can_id == 0x31E && rx_msg.data_length_code >= 2) { enviaValoresLargos((rx_msg.data << 8) | rx_msg.data, 0x50, 0x00); }
if (can_id == 0x32E && rx_msg.data_length_code >= 2) { enviaValoresLargos((rx_msg.data << 8) | rx_msg.data, 0x50, 0x10); }
if (can_id == 0x33E && rx_msg.data_length_code >= 2) { enviaValoresLargos((rx_msg.data << 8) | rx_msg.data, 0x50, 0x20); }
}
} 

void limpia_Bufer() { for (int i = 0; i < 8; i++) { Bufer_1[i] = 0; } } 

void monta_CAN(int Puntero) {
if (Puntero == 4) {
Bufer_H53[Puntero] = Bufer_1; Bufer_H59[Puntero] = Bufer_1;
} else {
Bufer_H52[Puntero] = Bufer_1; Bufer_H53[Puntero] = Bufer_1;
Bufer_H54[Puntero] = Bufer_1; Bufer_H55[Puntero] = Bufer_1;
Bufer_H56[Puntero] = Bufer_1; Bufer_H57[Puntero] = Bufer_1;
Bufer_H58[Puntero] = Bufer_1; Bufer_H59[Puntero] = Bufer_1;
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

void procesarEventoHMI(int registro, int lastByte) {
int Puntero = 0;
switch (registro) {
case 3000: limpia_Bufer(); Puntero = 1; lee_DWIN(lastByte); monta_CAN(Puntero); envia_Bufer(); break;
case 3100: limpia_Bufer(); Puntero = 2; lee_DWIN(lastByte); monta_CAN(Puntero); envia_Bufer(); break;
case 3200: limpia_Bufer(); Puntero = 3; lee_DWIN(lastByte); monta_CAN(Puntero); envia_Bufer(); break;
case 3300: canal_33(lastByte); delay(100); Puntero = 4; canal_33CAN(lastByte); monta_CAN(Puntero); envia_Bufer(); break;
case 3400: canal_34(lastByte); break;
case 3500: Bufer_H5A = lastByte; monta_CAN2(); break;
case 3600: Bufer_H5A = lastByte; monta_CAN2(); break;
case 3700: Bufer_H5A = lastByte; monta_CAN2(); break;
case 3705: Bufer_H5A = lastByte; monta_CAN2(); break;
case 3710: Bufer_H5A = lastByte; monta_CAN2(); break;
case 2040: Bufer_H5A = lastByte; monta_CAN2(); break;
}
} 

void escucharCanBus() {
twai_message_t rx_msg;
while (twai_receive(&rx_msg, 0) == ESP_OK) {
uint32_t can_id = rx_msg.identifier;
std::vector<uint8_t> data(rx_msg.data, rx_msg.data + rx_msg.data_length_code);
procesarCanEntrante(can_id, data);
}
}


// --- BUCLE DE ESCUCHA CONTINUO SERIE (Implementado en C++ Nativo) ---
void loop() override {
// 1. Lectura temporizada del BME680 idéntica a tu loop() de Arduino
unsigned long actualMillis = millis();
if (actualMillis - ultimaLecturaBME >= intervaloBME) {
ultimaLecturaBME = actualMillis;
if (bme && bme->performReading()) {
float temp = bme->temperature;
enviaValoresFlotantes(temp, 0x20, 0x00); // Envía la temperatura real a la VP 2000
esphome::ESP_LOGD("BME680", "Lectura correcta - Temperatura: %.2f °C", temp);
}
}
// 2. Escucha de la UART de la pantalla DWIN
static uint8_t buffer;
static size_t idx = 0;
while (uart_bus->available()) {
uint8_t b = uart_bus->read();
if (idx == 0 && b != 0x5A) continue;
if (idx == 1 && b != 0xA5) { idx = 0; continue; }
buffer[idx++] = b;
if (idx >= 3 && idx == (buffer + 3)) {
if (buffer == 0x83) {
int direccion_vp = (buffer << 8) | buffer;
int ultimo_byte = buffer[idx - 1];
int valor_completo = (buffer[idx - 2] << 8) | buffer[idx - 1];
  if (direccion_vp == 3600) id(p_color) = ultimo_byte;
  if (direccion_vp == 3700) id(p_retardo) = ultimo_byte;
  if (direccion_vp == 3705) id(p_dura) = ultimo_byte;
  if (direccion_vp == 3710) id(p_pot) = ultimo_byte;
  if (direccion_vp == 2040) id(p_filtra) = ultimo_byte;
  
  if (direccion_vp == 3800) id(hz_variador_filtracion).publish_state((float)valor_completo / 10.0);
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
      if (ultimo_byte == 1) call.set_rgb(1.0, 0.0, 0.0);
      if (ultimo_byte == 2) call.set_rgb(0.0, 1.0, 0.0);
      if (ultimo_byte == 3) call.set_rgb(0.0, 0.0, 1.0);
      if (ultimo_byte == 4) call.set_rgb(1.0, 1.0, 1.0);
    }
    call.execute();
  }
  procesarEventoHMI(direccion_vp, ultimo_byte);
}
idx = 0;

}
}
escucharCanBus(); // Ejecuta la escucha proactiva del CAN Bus

} 

private:
void lee_DWIN(int lastByte) { for (int i = 0; i < 8; i++) { Bufer_1[i] = ((lastByte >> i) & 1) ? 1 : 0; } }
void canal_33(int lastByte) { for (int i = 0; i < 2; i++) { digitalWrite(Zona_1[i], ((lastByte >> i) & 1) ? HIGH : LOW); } }
void canal_33CAN(int lastByte) { for (int i = 2; i < 4; i++) { Bufer_1[i] = ((lastByte >> i) & 1) ? 1 : 0; } }
void canal_34(int lastByte) { for (int i = 0; i < 2; i++) { digitalWrite(Zona_1[i + 2], ((lastByte >> i) & 1) ? HIGH : LOW); } }
};