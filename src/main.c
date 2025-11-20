/* Ximena Cortés, Samantha Bravo y Dana Paola Valiente
El código configura dos botones y un sensor de temperatura, y expone ambos mediante Bluetooth Low Energy. 
Cada vez que se presionan los botones, se envía una notificación BLE con sus contadores. La temperatura se 
lee bajo demanda cuando una app solicita la característica. Todo el manejo se realiza mediante interrupciones 
y callbacks de BLE */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <stdio.h>

// Registro del módulo de logs
LOG_MODULE_REGISTER(BLE_Sensores, LOG_LEVEL_INF);

// Configuración de botones
#define BUTTON_A_NODE DT_ALIAS(sw0)
#define BUTTON_B_NODE DT_ALIAS(sw1)

static const struct gpio_dt_spec bA = GPIO_DT_SPEC_GET(BUTTON_A_NODE, gpios);
static const struct gpio_dt_spec bB = GPIO_DT_SPEC_GET(BUTTON_B_NODE, gpios);

// Callbacks usados por las interrupciones
static struct gpio_callback button_a_cb;
static struct gpio_callback button_b_cb;

// Sensor de temperatura
const struct device *temp_dev;
struct sensor_value temperature;

// Configuración BLE
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    // UUID del Servicio Principal (el mismo que tenías)
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                  0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12)
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_le_adv_param adv_params = {
    .options = BT_LE_ADV_OPT_CONN,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
};

// Variables compartidas para GATT
static struct bt_conn *current_conn;

// Contadores de botones -> enviados por notificación
static uint8_t button_counts[2] = {0, 0};

// Temperatura enviada por lectura GATT [entero, decimal]
static int8_t temp_value[2] = {0, 0};

// Prototipos para callbacks y GATT
static void button_a_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void button_b_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static ssize_t read_temperature_value(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset);
static void button_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

// Callbacks de lectura GATT
static ssize_t read_temperature_value(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset)
{
    LOG_INF("Solicitud de lectura de temperatura recibida.");

    // Toma nueva muestra del sensor
    if (sensor_sample_fetch(temp_dev)) {
        LOG_ERR("Fallo al leer el sensor de temperatura");
        // Enviar valores antiguos o cero si falla
    } else {
        // Obtiene el canal de temperatura interna del chip
        sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP, &temperature);

        // Convierte valores a formato entero + decimal
        temp_value[0] = (int8_t)temperature.val1;
        temp_value[1] = (int8_t)(temperature.val2 / 10000); // Tomar solo 2 decimales
        
        LOG_INF("Enviando Temperatura: %d.%02d C", temp_value[0], temp_value[1]);
    }

    // Respuesta a la lectura GATT
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &temp_value, sizeof(temp_value));
}

// Callback para notificaciones GATT
static void button_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("Notificaciones %s", (value == BT_GATT_CCC_NOTIFY) ? "habilitadas" : "deshabilitadas");
}

// Servicio GATT
BT_GATT_SERVICE_DEFINE(main_svc,
     // Servicio principal con UUID 128-bit
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12)),

    // Notificación de botones
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(
        0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12),
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, &button_counts), // Apunta a nuestro array de conteo
    
    // Descriptor CCC para habilitar notificaciones
    BT_GATT_CCC(button_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Lectura de temperatura
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(
        0xf2, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12),
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_temperature_value, NULL, &temp_value) // Llama al callback al leer
);

// Callbacks BLE
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Fallo en la conexion (err %u)", err);
    } else {
        current_conn = bt_conn_ref(conn);
        LOG_INF("Conectado");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Desconectado (razon %u)", reason);
    // Liberar referencia
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    // Reiniciar advertising después de desconectar
    int ret = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("No se pudo reiniciar el advertising (err %d)", ret);
    }
}

// Registrar callbacks BLE globales
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

// Interrupciones
static void notify_button_state(void)
{
    // Solo si existe un dispositivo conectado
    if (current_conn) {
        bt_gatt_notify(current_conn, &main_svc.attrs[2], &button_counts, sizeof(button_counts));
    }
}

// ISR para Botón A
static void button_a_pressed(const struct device *dev, struct gpio_callback *cb,
                             uint32_t pins)
{
    button_counts[0]++; // Incrementar contador
    LOG_INF("Boton A presionado. Total: %d", button_counts[0]);
    notify_button_state(); // Enviar notificación BLE
}

// ISR para Botón B
static void button_b_pressed(const struct device *dev, struct gpio_callback *cb,
                             uint32_t pins)
{
    button_counts[1]++;
    LOG_INF("Boton B presionado. Total: %d", button_counts[1]);
    notify_button_state();
}

int main(void)
{
    int ret;

    LOG_INF("...Inicializando app BLE de Botones y Temperatura...");

    // Configura Botón A
    if (!device_is_ready(bA.port)) {
        LOG_ERR("GPIO del boton A no esta listo");
        return 0;
    }
    ret = gpio_pin_configure_dt(&bA, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) return 0;
    gpio_init_callback(&button_a_cb, button_a_pressed, BIT(bA.pin));
    gpio_add_callback(bA.port, &button_a_cb);
    gpio_pin_interrupt_configure_dt(&bA, GPIO_INT_EDGE_TO_ACTIVE);
    LOG_INF("Boton A OK");

    // Configura Botón B
    if (!device_is_ready(bB.port)) {
        LOG_ERR("GPIO del boton B no esta listo");
        return 0;
    }
    ret = gpio_pin_configure_dt(&bB, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) return 0;
    gpio_init_callback(&button_b_cb, button_b_pressed, BIT(bB.pin));
    gpio_add_callback(bB.port, &button_b_cb);
    gpio_pin_interrupt_configure_dt(&bB, GPIO_INT_EDGE_TO_ACTIVE);
    LOG_INF("Boton B OK");

    // Inicializa sensor de temperatura
    temp_dev = DEVICE_DT_GET_ANY(nordic_nrf_temp);
    if (!device_is_ready(temp_dev)) {
        LOG_ERR("Sensor de temperatura no esta listo");
        return 0;
    }
    LOG_INF("Sensor de Temperatura OK");

    // Habilita el BLE
    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("No funciono el BLE (err %d)", ret);
        return 0;
    }
    LOG_INF("Bluetooth OK");

    // Inicia el advertising
    ret = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("El Advertising no fue posible (err %d)", ret);
        return 0;
    }
    LOG_INF("Advertising iniciado. Busca a: %s", CONFIG_BT_DEVICE_NAME);
    
    return 0;
}
