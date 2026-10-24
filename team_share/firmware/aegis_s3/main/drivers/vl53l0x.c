#include "vl53l0x.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "vl53l0x";

#define VL53_DEFAULT_ADDR 0x29
#define I2C_TIMEOUT_MS 10
#define INIT_TIMEOUT_US 100000

enum {
    SYSRANGE_START = 0x00,
    SYSTEM_SEQUENCE_CONFIG = 0x01,
    SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,
    SYSTEM_INTERRUPT_CLEAR = 0x0B,
    RESULT_INTERRUPT_STATUS = 0x13,
    RESULT_RANGE_STATUS = 0x14,
    MSRC_CONFIG_CONTROL = 0x60,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
    GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
    GPIO_HV_MUX_ACTIVE_HIGH = 0x84,
    VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV = 0x89,
    I2C_SLAVE_DEVICE_ADDRESS = 0x8A,
    IDENTIFICATION_MODEL_ID = 0xC0,
};

static esp_err_t wr8(vl53l0x_t *s, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return i2c_master_transmit(s->dev, b, 2, I2C_TIMEOUT_MS);
}

static esp_err_t wr16(vl53l0x_t *s, uint8_t reg, uint16_t val)
{
    uint8_t b[3] = {reg, (uint8_t)(val >> 8), (uint8_t)val};
    return i2c_master_transmit(s->dev, b, 3, I2C_TIMEOUT_MS);
}

static esp_err_t rd(vl53l0x_t *s, uint8_t reg, uint8_t *out, size_t n)
{
    return i2c_master_transmit_receive(s->dev, &reg, 1, out, n, I2C_TIMEOUT_MS);
}

static esp_err_t rd8(vl53l0x_t *s, uint8_t reg, uint8_t *out)
{
    return rd(s, reg, out, 1);
}

static esp_err_t wr_multi(vl53l0x_t *s, uint8_t reg, const uint8_t *src, size_t n)
{
    uint8_t b[8];
    b[0] = reg;
    for (size_t i = 0; i < n && i < sizeof(b) - 1; i++) b[i + 1] = src[i];
    return i2c_master_transmit(s->dev, b, n + 1, I2C_TIMEOUT_MS);
}

typedef struct {
    uint8_t reg, val;
} regval_t;

/* ST default tuning settings (DefaultTuningSettings in the ST API) */
static const regval_t TUNING[] = {
    {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00},
    {0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
    {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04}, {0x32, 0x03},
    {0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
    {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00}, {0x62, 0x00}, {0x64, 0x00},
    {0x65, 0x00}, {0x66, 0xA0}, {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
    {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21}, {0xFF, 0x01},
    {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
    {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44},
    {0xFF, 0x01}, {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
    {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28}, {0x67, 0x00}, {0x70, 0x04},
    {0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
    {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01},
    {0xFF, 0x00}, {0x80, 0x00},
};

static esp_err_t write_seq(vl53l0x_t *s, const regval_t *seq, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        ESP_RETURN_ON_ERROR(wr8(s, seq[i].reg, seq[i].val), TAG, "seq write 0x%02x", seq[i].reg);
    }
    return ESP_OK;
}

static esp_err_t wait_reg(vl53l0x_t *s, uint8_t reg, uint8_t mask, bool want_nonzero)
{
    int64_t t0 = esp_timer_get_time();
    uint8_t v = 0;
    do {
        ESP_RETURN_ON_ERROR(rd8(s, reg, &v), TAG, "poll 0x%02x", reg);
        if (((v & mask) != 0) == want_nonzero) return ESP_OK;
    } while (esp_timer_get_time() - t0 < INIT_TIMEOUT_US);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t get_spad_info(vl53l0x_t *s, uint8_t *count, bool *is_aperture)
{
    uint8_t tmp;
    static const regval_t pre[] = {{0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x06}};
    ESP_RETURN_ON_ERROR(write_seq(s, pre, 4), TAG, "spad pre");
    ESP_RETURN_ON_ERROR(rd8(s, 0x83, &tmp), TAG, "0x83");
    ESP_RETURN_ON_ERROR(wr8(s, 0x83, tmp | 0x04), TAG, "0x83");
    static const regval_t mid[] = {{0xFF, 0x07}, {0x81, 0x01}, {0x80, 0x01}, {0x94, 0x6B}, {0x83, 0x00}};
    ESP_RETURN_ON_ERROR(write_seq(s, mid, 5), TAG, "spad mid");
    ESP_RETURN_ON_ERROR(wait_reg(s, 0x83, 0xFF, true), TAG, "spad wait");
    ESP_RETURN_ON_ERROR(wr8(s, 0x83, 0x01), TAG, "0x83");
    ESP_RETURN_ON_ERROR(rd8(s, 0x92, &tmp), TAG, "0x92");
    *count = tmp & 0x7F;
    *is_aperture = (tmp >> 7) & 0x01;
    ESP_RETURN_ON_ERROR(wr8(s, 0x81, 0x00), TAG, "0x81");
    ESP_RETURN_ON_ERROR(wr8(s, 0xFF, 0x06), TAG, "0xff");
    ESP_RETURN_ON_ERROR(rd8(s, 0x83, &tmp), TAG, "0x83");
    ESP_RETURN_ON_ERROR(wr8(s, 0x83, tmp & ~0x04), TAG, "0x83");
    static const regval_t post[] = {{0xFF, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}};
    return write_seq(s, post, 4);
}

static esp_err_t single_ref_cal(vl53l0x_t *s, uint8_t vhv_init_byte)
{
    ESP_RETURN_ON_ERROR(wr8(s, SYSRANGE_START, 0x01 | vhv_init_byte), TAG, "cal start");
    ESP_RETURN_ON_ERROR(wait_reg(s, RESULT_INTERRUPT_STATUS, 0x07, true), TAG, "cal wait");
    ESP_RETURN_ON_ERROR(wr8(s, SYSTEM_INTERRUPT_CLEAR, 0x01), TAG, "cal clr");
    return wr8(s, SYSRANGE_START, 0x00);
}

static esp_err_t add_device(vl53l0x_t *s, uint8_t addr)
{
    if (s->dev) {
        i2c_master_bus_rm_device(s->dev);
        s->dev = NULL;
    }
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    s->addr = addr;
    return i2c_master_bus_add_device(s->bus, &cfg, &s->dev);
}

esp_err_t vl53l0x_init(vl53l0x_t *s, i2c_master_bus_handle_t bus, uint8_t new_addr)
{
    s->bus = bus;
    s->dev = NULL;
    s->ok = false;
    uint8_t id = 0;
    /* A sensor whose XSHUT is tied high keeps its new address across an MCU
     * reset (only a power cycle restores 0x29), so look there first. */
    ESP_RETURN_ON_ERROR(add_device(s, new_addr), TAG, "add dev");
    if (i2c_master_probe(bus, new_addr, I2C_TIMEOUT_MS) != ESP_OK) {
        ESP_RETURN_ON_ERROR(add_device(s, VL53_DEFAULT_ADDR), TAG, "add dev");
        ESP_RETURN_ON_ERROR(rd8(s, IDENTIFICATION_MODEL_ID, &id), TAG, "no sensor at 0x29");
        ESP_RETURN_ON_FALSE(id == 0xEE, ESP_ERR_NOT_FOUND, TAG, "model id 0x%02x, expected 0xEE", id);
        ESP_RETURN_ON_ERROR(wr8(s, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F), TAG, "set addr");
        ESP_RETURN_ON_ERROR(add_device(s, new_addr), TAG, "re-add dev");
    }
    ESP_RETURN_ON_ERROR(rd8(s, IDENTIFICATION_MODEL_ID, &id), TAG, "id at 0x%02x", new_addr);
    ESP_RETURN_ON_FALSE(id == 0xEE, ESP_ERR_NOT_FOUND, TAG, "model id 0x%02x, expected 0xEE", id);

    /* DataInit: 2V8 I/O mode, the ST API default for these breakouts */
    uint8_t v;
    ESP_RETURN_ON_ERROR(rd8(s, VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &v), TAG, "vhv");
    ESP_RETURN_ON_ERROR(wr8(s, VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, v | 0x01), TAG, "vhv");
    ESP_RETURN_ON_ERROR(wr8(s, 0x88, 0x00), TAG, "std i2c");
    static const regval_t a[] = {{0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}};
    ESP_RETURN_ON_ERROR(write_seq(s, a, 3), TAG, "stop var pre");
    ESP_RETURN_ON_ERROR(rd8(s, 0x91, &s->stop_variable), TAG, "stop var");
    static const regval_t b[] = {{0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}};
    ESP_RETURN_ON_ERROR(write_seq(s, b, 3), TAG, "stop var post");

    /* disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks */
    ESP_RETURN_ON_ERROR(rd8(s, MSRC_CONFIG_CONTROL, &v), TAG, "msrc");
    ESP_RETURN_ON_ERROR(wr8(s, MSRC_CONFIG_CONTROL, v | 0x12), TAG, "msrc");
    /* final range signal rate limit 0.25 MCPS (Q9.7) */
    ESP_RETURN_ON_ERROR(wr16(s, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(0.25f * 128)), TAG, "rate");
    ESP_RETURN_ON_ERROR(wr8(s, SYSTEM_SEQUENCE_CONFIG, 0xFF), TAG, "seq");

    /* StaticInit: reference SPAD map */
    uint8_t spad_count;
    bool spad_aperture;
    ESP_RETURN_ON_ERROR(get_spad_info(s, &spad_count, &spad_aperture), TAG, "spad info");
    uint8_t map[6];
    ESP_RETURN_ON_ERROR(rd(s, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map, 6), TAG, "spad map");
    static const regval_t c[] = {{0xFF, 0x01}, {DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00},
                                 {DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C}, {0xFF, 0x00},
                                 {GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4}};
    ESP_RETURN_ON_ERROR(write_seq(s, c, 5), TAG, "spad cfg");
    uint8_t first = spad_aperture ? 12 : 0, enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first || enabled == spad_count) {
            map[i / 8] &= ~(1 << (i % 8));
        } else if ((map[i / 8] >> (i % 8)) & 0x1) {
            enabled++;
        }
    }
    ESP_RETURN_ON_ERROR(wr_multi(s, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map, 6), TAG, "spad write");

    ESP_RETURN_ON_ERROR(write_seq(s, TUNING, sizeof(TUNING) / sizeof(TUNING[0])), TAG, "tuning");

    /* interrupt on new sample ready, active low */
    ESP_RETURN_ON_ERROR(wr8(s, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04), TAG, "gpio cfg");
    ESP_RETURN_ON_ERROR(rd8(s, GPIO_HV_MUX_ACTIVE_HIGH, &v), TAG, "hv mux");
    ESP_RETURN_ON_ERROR(wr8(s, GPIO_HV_MUX_ACTIVE_HIGH, v & ~0x10), TAG, "hv mux");
    ESP_RETURN_ON_ERROR(wr8(s, SYSTEM_INTERRUPT_CLEAR, 0x01), TAG, "int clr");

    /* default sequence: no MSRC, no TCC */
    ESP_RETURN_ON_ERROR(wr8(s, SYSTEM_SEQUENCE_CONFIG, 0xE8), TAG, "seq");

    /* PerformRefCalibration: VHV then phase */
    ESP_RETURN_ON_ERROR(wr8(s, SYSTEM_SEQUENCE_CONFIG, 0x01), TAG, "seq vhv");
    ESP_RETURN_ON_ERROR(single_ref_cal(s, 0x40), TAG, "vhv cal");
    ESP_RETURN_ON_ERROR(wr8(s, SYSTEM_SEQUENCE_CONFIG, 0x02), TAG, "seq phase");
    ESP_RETURN_ON_ERROR(single_ref_cal(s, 0x00), TAG, "phase cal");
    ESP_RETURN_ON_ERROR(wr8(s, SYSTEM_SEQUENCE_CONFIG, 0xE8), TAG, "seq restore");

    s->ok = true;
    ESP_LOGI(TAG, "sensor ready at 0x%02x (%u ref SPADs, %s)", new_addr, spad_count,
             spad_aperture ? "aperture" : "non-aperture");
    return ESP_OK;
}

esp_err_t vl53l0x_start_continuous(vl53l0x_t *s)
{
    static const regval_t a[] = {{0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}};
    ESP_RETURN_ON_ERROR(write_seq(s, a, 3), TAG, "cont pre");
    ESP_RETURN_ON_ERROR(wr8(s, 0x91, s->stop_variable), TAG, "stop var");
    static const regval_t b[] = {{0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}};
    ESP_RETURN_ON_ERROR(write_seq(s, b, 3), TAG, "cont post");
    return wr8(s, SYSRANGE_START, 0x02);   /* back-to-back mode */
}

esp_err_t vl53l0x_poll_mm(vl53l0x_t *s, uint16_t *mm)
{
    uint8_t st;
    ESP_RETURN_ON_ERROR(rd8(s, RESULT_INTERRUPT_STATUS, &st), TAG, "int status");
    if ((st & 0x07) == 0) return ESP_ERR_NOT_FINISHED;

    uint8_t buf[12];
    ESP_RETURN_ON_ERROR(rd(s, RESULT_RANGE_STATUS, buf, sizeof(buf)), TAG, "result");
    ESP_RETURN_ON_ERROR(wr8(s, SYSTEM_INTERRUPT_CLEAR, 0x01), TAG, "int clr");

    uint8_t range_status = (buf[0] & 0x78) >> 3;   /* 11 = range valid */
    uint16_t d = ((uint16_t)buf[10] << 8) | buf[11];
    if (d >= 8190) {
        /* no return signal: nothing within range, which is a valid "far" reading */
        d = VL53L0X_FAR_MM;
    } else if (range_status != 11) {
        return ESP_ERR_INVALID_RESPONSE;          /* sigma / wrap-around fail */
    }
    s->last_mm = d;
    *mm = d;
    return ESP_OK;
}
