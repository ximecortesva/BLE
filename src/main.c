/* Ximena Cortés, Samantha Bravo y Dana Paola Valiente
*/

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

LOG_MODULE_REGISTER(BLE_Sensores, LOG_LEVEL_INF);

// --- Definiciones de Botones ---
#define BUTTON_A_NODE DT_ALIAS(sw0)
#define BUTTON_B_NODE DT_ALIAS(sw1)

static const struct gpio_dt_spec bA = GPIO_DT_SPEC_GET(BUTTON_A_NODE, gpios);
static const struct gpio_dt_spec bB = GPIO_DT_SPEC_GET(BUTTON_B_NODE, gpios);

static struct gpio_callback button_a_cb;
static struct gpio_callback button_b_cb;

// --- Definiciones de Sensor de Temperatura ---
const struct device *temp_dev;
struct sensor_value temperature;

// --- Definiciones de BLE ---
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

// --- Variables de Características GATT ---
static struct bt_conn *current_conn;

// button_counts[0] = Botón A
// button_counts[1] = Botón B
static uint8_t button_counts[2] = {0, 0};

// temp_value[0] = Parte entera (ej: 25)
// temp_value[1] = Parte decimal (ej: 34)
static int8_t temp_value[2] = {0, 0};

// --- Prototipos de Funciones ---
static void button_a_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void button_b_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static ssize_t read_temperature_value(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset);
static void button_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

// --- Callbacks de BLE GATT ---

/**
 * @brief Callback para la lectura de la característica de temperatura.
 * Se ejecuta CADA VEZ que la app nRF Connect solicita una lectura.
 */
static ssize_t read_temperature_value(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset)
{
    LOG_INF("Solicitud de lectura de temperatura recibida.");

    // 1. Tomar muestra del sensor
    if (sensor_sample_fetch(temp_dev)) {
        LOG_ERR("Fallo al leer el sensor de temperatura");
        // Enviar valores antiguos o cero si falla
    } else {
        // 2. Obtener el canal
        sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP, &temperature);

        // 3. Actualizar el valor que se enviará
        temp_value[0] = (int8_t)temperature.val1;
        temp_value[1] = (int8_t)(temperature.val2 / 10000); // Tomar solo 2 decimales
        
        LOG_INF("Enviando Temperatura: %d.%02d C", temp_value[0], temp_value[1]);
    }

    // 4. Enviar los datos al teléfono
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &temp_value, sizeof(temp_value));
}

/**
 * @brief Callback para la configuración de notificaciones (CCC).
 * Se ejecuta cuando la app se suscribe o desuscribe.
 */
static void button_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("Notificaciones %s", (value == BT_GATT_CCC_NOTIFY) ? "habilitadas" : "deshabilitadas");
}

// --- Definición del Servicio GATT ---
// UUID Servicio: ...123456789abcdef0
// UUID Char 1 (Botones): ...123456789abcdef1 (Notify)
// UUID Char 2 (Temp): ...123456789abcdef2 (Read)

BT_GATT_SERVICE_DEFINE(main_svc,
    // Servicio Primario
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12)),

    // Característica 1: Contador de Botones (Notify)
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(
        0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, // UUID ...f1
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12),
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_NONE,
        NULL, NULL, &button_counts), // Apunta a nuestro array de conteo
    
    // Descriptor CCC para habilitar las notificaciones
    BT_GATT_CCC(button_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Característica 2: Valor de Temperatura (Read)
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(
        0xf2, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, // UUID ...f2
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12),
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_temperature_value, NULL, &temp_value) // Llama al callback al leer
);

// --- Callbacks de Conexión BLE ---

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
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    // Reiniciar advertising
    int ret = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("No se pudo reiniciar el advertising (err %d)", ret);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

// --- ISRs (Interrupciones) de Botones ---

/**
 * @brief Notifica el estado de los botones al cliente BLE.
 */
static void notify_button_state(void)
{
    // Si hay un cliente conectado y suscrito...
    if (current_conn) {
        // ...enviar los datos.
        // OJO: El índice [2] es el atributo de valor de la Característica 1
        // Attrs: [0]Servicio, [1]Char1(decl), [2]Char1(valor), [3]Char1(CCC), [4]Char2(decl), [5]Char2(valor)
        bt_gatt_notify(current_conn, &main_svc.attrs[2], &button_counts, sizeof(button_counts));
    }
}

// ISR para Botón A
static void button_a_pressed(const struct device *dev, struct gpio_callback *cb,
                             uint32_t pins)
{
    button_counts[0]++;
    LOG_INF("Boton A presionado. Total: %d", button_counts[0]);
    notify_button_state();
}

// ISR para Botón B
static void button_b_pressed(const struct device *dev, struct gpio_callback *cb,
                             uint32_t pins)
{
    button_counts[1]++;
    LOG_INF("Boton B presionado. Total: %d", button_counts[1]);
    notify_button_state();
}

// --- Programa Principal ---

int main(void)
{
    int ret;

    LOG_INF("...Inicializando app BLE de Botones y Temperatura...");

    // 1. Inicializar Botón A
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

    // 2. Inicializar Botón B
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

    // 3. Inicializar Sensor de Temperatura
    temp_dev = DEVICE_DT_GET_ANY(nordic_nrf_temp);
    if (!device_is_ready(temp_dev)) {
        LOG_ERR("Sensor de temperatura no esta listo");
        return 0;
    }
    LOG_INF("Sensor de Temperatura OK");

    // 4. Habilitar el BLE
    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("No funciono el BLE (err %d)", ret);
        return 0;
    }
    LOG_INF("Bluetooth OK");

    // 5. Iniciar Advertising
    ret = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("El Advertising no fue posible (err %d)", ret);
        return 0;
    }
    LOG_INF("Advertising iniciado. Busca a: %s", CONFIG_BT_DEVICE_NAME);
    
    // El hilo principal puede terminar; las interrupciones y callbacks se encargan
    return 0;
}
