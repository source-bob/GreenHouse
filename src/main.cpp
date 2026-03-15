#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <algorithm>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "i2c/PicoI2C.h"
#include "display/ssd1306os.h"

#include "uart/PicoOsUart.h"
#include "modbus/ModbusClient.h"
#include "modbus/ModbusRegister.h"

#include "hardware/timer.h"
#include "pico/cyw43_arch.h"
#include "ipstack/IPStack.h"

extern "C" bool run_tls_client_test(const uint8_t *cert, size_t cert_len,
                                    const char *server, const char *request, int timeout);

extern "C" bool run_tls_client_request(const uint8_t *cert, size_t cert_len,
                                       const char *server, const char *request, int timeout,
                                       char *response, size_t response_n);

extern "C" uint32_t read_runtime_ctr(void) {
    return timer_hw->timerawl;
}

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifndef WIFI_PHONE_SSID
#define WIFI_PHONE_SSID WIFI_SSID
#endif

#ifndef WIFI_PHONE_PASSWORD
#define WIFI_PHONE_PASSWORD WIFI_PASSWORD
#endif

#ifndef WIFI_SCHOOL_SSID
#define WIFI_SCHOOL_SSID "SCHOOL_WIFI_SSID"
#endif

#ifndef WIFI_SCHOOL_PASSWORD
#define WIFI_SCHOOL_PASSWORD "SCHOOL_WIFI_PASS"
#endif

struct WifiProfile {
    const char* name;
    const char* ssid;
    const char* password;
};

static constexpr WifiProfile WIFI_PROFILES[] = {
    { "Phone",  WIFI_PHONE_SSID,  WIFI_PHONE_PASSWORD  },
    { "School", WIFI_SCHOOL_SSID, WIFI_SCHOOL_PASSWORD }
};

static constexpr uint8_t WIFI_PROFILE_COUNT =
    (uint8_t)(sizeof(WIFI_PROFILES) / sizeof(WIFI_PROFILES[0]));

static uint8_t clamp_wifi_profile(uint8_t idx) {
    return (idx < WIFI_PROFILE_COUNT) ? idx : 0;
}

// ===== Modbus (spec) =====
static constexpr int  MODBUS_UART_NR  = 1;
static constexpr int  MODBUS_TX_PIN   = 4;
static constexpr int  MODBUS_RX_PIN   = 5;
static constexpr int  MODBUS_BAUD     = 9600;
static constexpr int  MODBUS_STOPBITS = 2;

// ===== UI input (board) =====
static constexpr uint ROT_A_PIN  = 10;
static constexpr uint ROT_B_PIN  = 11;
static constexpr uint ROT_SW_PIN = 9;

static constexpr uint CO2_RELAY_PIN = 27;
static constexpr uint LED_PIN = 22;

static constexpr uint8_t OLED_ADDR_1 = 0x3C;
static constexpr uint8_t OLED_ADDR_2 = 0x3D;
static constexpr uint8_t EEPROM_ADDR = 0x50;

static constexpr uint8_t PRESS_ADDR = 0x40;   // SDP610 on I2C1
static constexpr int     PRESS_SCALE = 60;    // SDP610-125Pa scale factor

// ===== thresholds =====
static constexpr uint16_t CO2_SETPOINT_MAX = 1500;
static constexpr uint16_t CO2_SAFETY_HIGH  = 2000;

static constexpr uint16_t CO2_HYST_LOW  = 20;
static constexpr uint16_t CO2_HYST_HIGH = 50;

static constexpr uint32_t VALVE_PULSE_MS     = 2000;
static constexpr uint32_t VALVE_STABILIZE_MS = 8000;

// ===== EEPROM layout =====
struct PersistedConfigV1 {
    uint32_t magic;
    uint16_t version;
    uint16_t setpoint_ppm;
    uint16_t crc;
} __attribute__((packed));

struct PersistedConfig {
    uint32_t magic;
    uint16_t version;
    uint16_t setpoint_ppm;
    uint8_t  wifi_profile;
    uint8_t  reserved;
    uint16_t crc;
} __attribute__((packed));

static constexpr uint32_t CFG_MAGIC   = 0x47484346u; // "GHCF"
static constexpr uint16_t CFG_VERSION = 2;
static constexpr uint16_t EEPROM_CFG_ADDR = 0x0000;

static constexpr uint32_t WIFI_RETRY_MS = 5000;

// ======= CLOUD ======

static constexpr char THINGSPEAK_CERT[] =
"-----BEGIN CERTIFICATE-----\n"
"MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh\n"
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
"MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\n"
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\n"
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI\n"
"2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx\n"
"1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ\n"
"q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz\n"
"tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ\n"
"vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP\n"
"BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV\n"
"5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY\n"
"1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4\n"
"NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG\n"
"Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91\n"
"8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe\n"
"pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl\n"
"MrY=\n"
"-----END CERTIFICATE-----\n";

static constexpr char THINGSPEAK_HOST[] = "api.thingspeak.com";
static constexpr char THINGSPEAK_PATH[] = "/update.json";


static constexpr char THINGSPEAK_WRITE_API_KEY[] = "MY_WRITE_API_KEY";

static constexpr char THINGSPEAK_TALKBACK_ID[] = "TALCKBACK_ID";
static constexpr char THINGSPEAK_TALKBACK_API_KEY[] = "MY_TALKBACK_API_KEY";
static constexpr char THINGSPEAK_TALKBACK_PATH_FMT[] = "/talkbacks/%s/commands/execute.json";

// free ThingSpeak: не чаще 1 update в 15 секунд
static constexpr uint32_t CLOUD_TELEMETRY_MS    = 15000;
static constexpr uint32_t CLOUD_REMOTE_POLL_MS  = 3000;
static constexpr uint32_t CLOUD_TCP_TIMEOUT_MS  = 4000;
static constexpr uint32_t CLOUD_CONNECT_WAIT_MS = 3000;

// ===== pages =====
enum class Page : uint8_t { Home, SensorDetails, SetTarget, WiFiSelect, Network, ErrorLog, _Count };

struct AppState {
    Page page = Page::Home;

    uint8_t enc_a = 0, enc_b = 0, enc_sw = 0;
    uint32_t input_beat = 0;

    uint32_t gpio_now = 0;
    uint32_t gpio_xor = 0;

    uint16_t co2_ppm = 0;
    int16_t  t_x10   = 0;   // temp * 10
    int16_t  rh_x10  = 0;   // rh * 10
    int16_t  dp_pa_x10 = 0;   // pressure in Pa * 10
    bool     pressure_ok = false;
    uint16_t pulses  = 0;

    bool modbus_ok = false;
    bool eeprom_ok = false;
    bool wifi_ok = false;
    bool cloud_ok = false;

    uint32_t telemetry_tx_count = 0;
    uint32_t telemetry_tx_fail  = 0;

    uint32_t remote_rx_count = 0;
    uint16_t remote_last_setpoint_ppm = 0;

    uint16_t setpoint_ppm = 800;
    uint16_t fan_cmd_0_1000 = 200;

    bool valve_open = false;
    TickType_t valve_until = 0;
    TickType_t stabilize_until = 0;

    char last_err[64]{};

    bool fan_stopped = false;

    bool target_edit = false;   

    bool save_cfg_req = false;     
    bool save_cfg_ok = true;       
    uint32_t save_cfg_seq = 0;     

    bool safety_active = false;

    uint8_t  wifi_profile = 0;
    bool     wifi_edit = false;
    bool wifi_reconnect_req = false;
};

static AppState g;
static SemaphoreHandle_t g_mu;
static SemaphoreHandle_t g_i2c1_mu;
static std::shared_ptr<PicoI2C> g_i2c1;

static uint16_t checksum16(const uint8_t* data, size_t n) {
    uint16_t s = 0;
    for (size_t i = 0; i < n; ++i) s = (uint16_t)(s + data[i]);
    return s;
}

static uint8_t detect_oled_addr(std::shared_ptr<PicoI2C> bus) {
    uint8_t b = 0x00;
    if (bus->write(OLED_ADDR_1, &b, 1) == 1) return OLED_ADDR_1;
    if (bus->write(OLED_ADDR_2, &b, 1) == 1) return OLED_ADDR_2;
    return OLED_ADDR_1;
}

static bool eeprom_read_cfg(std::shared_ptr<PicoI2C> bus0, PersistedConfig& out) {
    uint8_t buf[sizeof(PersistedConfig)]{};

    auto try_read = [&](int addr_len) -> bool {
        if (addr_len == 2) {
            uint8_t addr[2] = {
                (uint8_t)(EEPROM_CFG_ADDR >> 8),
                (uint8_t)(EEPROM_CFG_ADDR & 0xFF)
            };
            if (bus0->write(EEPROM_ADDR, addr, 2) != 2) return false;
        } else {
            uint8_t addr = (uint8_t)(EEPROM_CFG_ADDR & 0xFF);
            if (bus0->write(EEPROM_ADDR, &addr, 1) != 1) return false;
        }

        const uint got = bus0->read(EEPROM_ADDR, buf, (uint)sizeof(buf));
        return got == (uint)sizeof(buf);
    };

    if (!try_read(2) && !try_read(1)) return false;

    // v2
    PersistedConfig cfg2{};
    memcpy(&cfg2, buf, sizeof(cfg2));
    if (cfg2.magic == CFG_MAGIC && cfg2.version == 2) {
        const uint16_t crc_calc =
            checksum16((const uint8_t*)&cfg2, sizeof(cfg2) - sizeof(cfg2.crc));
        if (crc_calc == cfg2.crc) {
            out = cfg2;
            out.wifi_profile = clamp_wifi_profile(out.wifi_profile);
            return true;
        }
    }

    // backward compatibility: v1
    PersistedConfigV1 cfg1{};
    memcpy(&cfg1, buf, sizeof(cfg1));
    if (cfg1.magic == CFG_MAGIC && cfg1.version == 1) {
        const uint16_t crc_calc =
            checksum16((const uint8_t*)&cfg1, sizeof(cfg1) - sizeof(cfg1.crc));
        if (crc_calc == cfg1.crc) {
            out.magic = CFG_MAGIC;
            out.version = CFG_VERSION;
            out.setpoint_ppm = cfg1.setpoint_ppm;
            out.wifi_profile = 0;
            out.reserved = 0;
            out.crc = checksum16((const uint8_t*)&out, sizeof(out) - sizeof(out.crc));
            return true;
        }
    }

    return false;
}

static bool eeprom_write_cfg(std::shared_ptr<PicoI2C> bus0, const PersistedConfig& cfg) {
    uint8_t buf[2 + sizeof(PersistedConfig)]{};
    buf[0] = (uint8_t)(EEPROM_CFG_ADDR >> 8);
    buf[1] = (uint8_t)(EEPROM_CFG_ADDR & 0xFF);
    memcpy(buf + 2, &cfg, sizeof(cfg));

    const uint written = bus0->write(EEPROM_ADDR, buf, (uint)sizeof(buf));
    if (written != (uint)sizeof(buf)) return false;

    // EEPROM internal write cycle
    uint8_t poll_addr[2] = { (uint8_t)(EEPROM_CFG_ADDR >> 8), (uint8_t)(EEPROM_CFG_ADDR & 0xFF) };
    for (int i = 0; i < 20; ++i) {              // ~20 * 5ms = 100ms max
        if (bus0->write(EEPROM_ADDR, poll_addr, 2) == 2) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

static bool sdp610_start(std::shared_ptr<PicoI2C> bus1) {
    uint8_t cmd = 0xF1;   // start continuous measurement
    return bus1->write(PRESS_ADDR, &cmd, 1) == 1;
}

static bool sdp610_read_pa_x10(std::shared_ptr<PicoI2C> bus1, int16_t& pa_x10_out) {
    uint8_t buf[2]{};
    if (bus1->read(PRESS_ADDR, buf, 2) != 2) return false;

    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);

    // physical value in Pa * 10
    pa_x10_out = (int16_t)((raw * 10) / PRESS_SCALE);
    return true;
}

static void fmt_x10(char* out, size_t n, int16_t v10) {
    int v = v10;
    if (v < 0) {
        int a = -v;
        snprintf(out, n, "-%d.%d", a / 10, a % 10);
    } else {
        snprintf(out, n, "%d.%d", v / 10, v % 10);
    }
}

static void set_last_error(const char* msg) {
    xSemaphoreTake(g_mu, portMAX_DELAY);
    strncpy(g.last_err, msg, sizeof(g.last_err) - 1);
    g.last_err[sizeof(g.last_err) - 1] = '\0';
    xSemaphoreGive(g_mu);
}

static void apply_remote_setpoint(uint16_t sp) {
    sp = (uint16_t)std::clamp<int>(sp, 200, (int)CO2_SETPOINT_MAX);

    xSemaphoreTake(g_mu, portMAX_DELAY);

    
    if (!g.target_edit && g.setpoint_ppm != sp) {
        g.setpoint_ppm = sp;
        g.remote_last_setpoint_ppm = sp;
        g.remote_rx_count++;

        g.save_cfg_req = true;
        g.save_cfg_seq++;
    }

    xSemaphoreGive(g_mu);
}

static void build_telemetry_body(char* out, size_t n, const AppState& s) {
    const int rh_i = (s.rh_x10 >= 0) ? ((s.rh_x10 + 5) / 10) : ((s.rh_x10 - 5) / 10);
    const int t_i  = (s.t_x10  >= 0) ? ((s.t_x10  + 5) / 10) : ((s.t_x10  - 5) / 10);

    const unsigned fan_pct =
        (unsigned)std::clamp<int>(s.fan_cmd_0_1000 / 10, 0, 100);

    snprintf(out, n,
             "api_key=%s&field1=%u&field2=%d&field3=%d&field4=%u&field5=%u",
             THINGSPEAK_WRITE_API_KEY,
             (unsigned)s.co2_ppm,
             rh_i,
             t_i,
             fan_pct,
             (unsigned)s.setpoint_ppm);
}

static bool http_status_ok(const char* resp) {
    return (strstr(resp, "HTTP/1.1 200") != nullptr) ||
           (strstr(resp, "HTTP/1.0 200") != nullptr);
}

static const char* http_find_body(const char* resp) {
    const char* p = strstr(resp, "\r\n\r\n");
    return p ? (p + 4) : nullptr;
}

static bool parse_talkback_setpoint_body(const char* body, uint16_t& sp_out) {
    if (!body || !*body) return false;

    const char* p = strstr(body, "\"command_string\"");
    if (!p) return false;

    p = strchr(p, ':');
    if (!p) return false;
    ++p;

    while (*p == ' ' || *p == '\"') ++p;

    char cmd[32]{};
    size_t i = 0;
    while (*p && *p != '\"' && *p != '\r' && *p != '\n' && i < sizeof(cmd) - 1) {
        cmd[i++] = *p++;
    }
    cmd[i] = '\0';

    int sp = -1;
    if (sscanf(cmd, "SP=%d", &sp) != 1) {
        sp = atoi(cmd);
    }

    if (sp < 200 || sp > (int)CO2_SETPOINT_MAX) {
        return false;
    }

    sp_out = (uint16_t)sp;
    return true;
}

static bool cloud_get_remote_setpoint_thingspeak_once(uint16_t& sp_out) {
    char path[96];
    snprintf(path, sizeof(path),
             THINGSPEAK_TALKBACK_PATH_FMT,
             THINGSPEAK_TALKBACK_ID);

    char body[64];
    snprintf(body, sizeof(body),
             "api_key=%s",
             THINGSPEAK_TALKBACK_API_KEY);

    char req[384];
    snprintf(req, sizeof(req),
             "POST %s HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Content-Type: application/x-www-form-urlencoded\r\n"
             "Content-Length: %u\r\n"
             "Connection: close\r\n"
             "\r\n"
             "%s",
             path,
             THINGSPEAK_HOST,
             (unsigned)strlen(body),
             body);

    char resp[768];
    const uint8_t* cert = reinterpret_cast<const uint8_t*>(THINGSPEAK_CERT);

    if (!run_tls_client_request(cert, sizeof(THINGSPEAK_CERT),
                                THINGSPEAK_HOST, req, 15,
                                resp, sizeof(resp))) {
        set_last_error("ThingSpeak TalkBack TLS failed");
        return false;
    }

    if (!http_status_ok(resp)) {
        set_last_error("ThingSpeak TalkBack bad status");
        return false;
    }

    const char* body_ptr = http_find_body(resp);
    if (!body_ptr || !*body_ptr) {
        return false; 
    }

    return parse_talkback_setpoint_body(body_ptr, sp_out);
}

static bool cloud_post_telemetry_thingspeak_once() {
    AppState s;
    xSemaphoreTake(g_mu, portMAX_DELAY);
    s = g;
    xSemaphoreGive(g_mu);

    char body[192];
    build_telemetry_body(body, sizeof(body), s);

    char req[512];
    snprintf(req, sizeof(req),
             "POST %s HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Content-Type: application/x-www-form-urlencoded\r\n"
             "Content-Length: %u\r\n"
             "Connection: close\r\n"
             "\r\n"
             "%s",
             THINGSPEAK_PATH,
             THINGSPEAK_HOST,
             (unsigned)strlen(body),
             body);

    const uint8_t* cert = reinterpret_cast<const uint8_t*>(THINGSPEAK_CERT);
    bool ok = run_tls_client_test(cert, sizeof(THINGSPEAK_CERT), THINGSPEAK_HOST, req, 15);

    if (!ok) {
        set_last_error("ThingSpeak TLS POST failed");
    }

    return ok;
}

static void input_task(void*) {
    gpio_init(ROT_A_PIN);  gpio_set_dir(ROT_A_PIN, GPIO_IN);  gpio_set_pulls(ROT_A_PIN, true, false);
    gpio_init(ROT_B_PIN);  gpio_set_dir(ROT_B_PIN, GPIO_IN);  gpio_set_pulls(ROT_B_PIN, true, false);
    gpio_init(ROT_SW_PIN); gpio_set_dir(ROT_SW_PIN, GPIO_IN); gpio_set_pulls(ROT_SW_PIN, true, false);

    auto readAB = []() -> uint8_t {
        return (gpio_get(ROT_A_PIN) ? 1 : 0) | (gpio_get(ROT_B_PIN) ? 2 : 0);
    };

    // prev<<2 | cur -> delta
    static const int8_t tbl[16] = {
        0, -1, +1,  0,
        +1, 0,  0, -1,
        -1, 0,  0, +1,
        0, +1, -1,  0
    };

    uint8_t prev = readAB();
    int accum = 0;

    while (true) {
        uint8_t cur = readAB();
        if (cur != prev) {
            accum += tbl[(prev << 2) | cur];
            prev = cur;

            if (accum >= 4 || accum <= -4) {
                int dir = (accum > 0) ? +1 : -1;
                accum = 0;

                xSemaphoreTake(g_mu, portMAX_DELAY);

                if (g.page == Page::SetTarget && g.target_edit) {
                    int sp = (int)g.setpoint_ppm + dir * 10;
                    sp = std::clamp(sp, 200, (int)CO2_SETPOINT_MAX);
                    g.setpoint_ppm = (uint16_t)sp;
                } else if (g.page == Page::WiFiSelect && g.wifi_edit) {
                    int p = (int)g.wifi_profile + dir;
                    if (p < 0) p = (int)WIFI_PROFILE_COUNT - 1;
                    if (p >= (int)WIFI_PROFILE_COUNT) p = 0;
                    g.wifi_profile = (uint8_t)p;
                } else {
                    int p = (int)g.page + dir;
                    if (p < 0) p = (int)Page::_Count - 1;
                    if (p >= (int)Page::_Count) p = 0;
                    g.page = (Page)p;

                    if (g.page == Page::SetTarget) {
                        g.target_edit = false;
                    }
                    if (g.page == Page::WiFiSelect) {
                        g.wifi_edit = false;
                    }
                }

                xSemaphoreGive(g_mu);
            }
        }

        
        // button: at SetTarget -> toggle edit / save
        if (!gpio_get(ROT_SW_PIN)) {                 // pull-up => pressed = 0
            vTaskDelay(pdMS_TO_TICKS(30));           // debounce
            if (!gpio_get(ROT_SW_PIN)) {
                xSemaphoreTake(g_mu, portMAX_DELAY);

                if (g.page == Page::SetTarget) {
                    if (!g.target_edit) {
                        g.target_edit = true;
                    } else {
                        g.save_cfg_req = true;
                        g.save_cfg_seq++;
                        g.target_edit = false;
                    }
                } else if (g.page == Page::WiFiSelect) {
                    if (!g.wifi_edit) {
                        g.wifi_edit = true;
                    } else {
                        g.save_cfg_req = true;
                        g.save_cfg_seq++;
                        g.wifi_edit = false;
                        g.wifi_reconnect_req = true;
                        strncpy(g.last_err, "Switching WiFi...", sizeof(g.last_err) - 1);
                        g.last_err[sizeof(g.last_err) - 1] = '\0';
                    }
                }

                xSemaphoreGive(g_mu);

                while (!gpio_get(ROT_SW_PIN)) vTaskDelay(pdMS_TO_TICKS(10)); // wait release
            }
        }

        xSemaphoreTake(g_mu, portMAX_DELAY);
        g.enc_a = gpio_get(ROT_A_PIN);
        g.enc_b = gpio_get(ROT_B_PIN);
        g.enc_sw = gpio_get(ROT_SW_PIN);
        g.input_beat++;
        xSemaphoreGive(g_mu);

        static uint32_t prev_all = 0;
        uint32_t now_all = gpio_get_all();
        uint32_t x = now_all ^ prev_all;
        prev_all = now_all;

        xSemaphoreTake(g_mu, portMAX_DELAY);
        g.gpio_now = now_all;
        g.gpio_xor = x;
        xSemaphoreGive(g_mu);

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static void eeprom_task(void*) {
    std::shared_ptr<PicoI2C> bus0 = std::make_shared<PicoI2C>(0, 100000);

    PersistedConfig cfg{};
    bool ok = eeprom_read_cfg(bus0, cfg);

    xSemaphoreTake(g_mu, portMAX_DELAY);
    g.eeprom_ok = ok;
    if (ok) {
        g.setpoint_ppm = (uint16_t)std::clamp<int>(cfg.setpoint_ppm, 200, (int)CO2_SETPOINT_MAX);
        g.wifi_profile = clamp_wifi_profile(cfg.wifi_profile);
    }
    xSemaphoreGive(g_mu);

    while (true) {
        bool do_save = false;
        uint16_t sp = 0;

        xSemaphoreTake(g_mu, portMAX_DELAY);
        if (g.save_cfg_req) {
            g.save_cfg_req = false;
            do_save = true;
            sp = g.setpoint_ppm;
        }
        xSemaphoreGive(g_mu);

        if (do_save) {
            uint8_t wifi_profile = 0;

            xSemaphoreTake(g_mu, portMAX_DELAY);
            sp = g.setpoint_ppm;
            wifi_profile = clamp_wifi_profile(g.wifi_profile);
            xSemaphoreGive(g_mu);

            PersistedConfig w{};
            w.magic = CFG_MAGIC;
            w.version = CFG_VERSION;
            w.setpoint_ppm = sp;
            w.wifi_profile = wifi_profile;
            w.reserved = 0;
            w.crc = checksum16((const uint8_t*)&w, sizeof(w) - sizeof(w.crc));

            bool ok = eeprom_write_cfg(bus0, w);

            xSemaphoreTake(g_mu, portMAX_DELAY);
            g.eeprom_ok = ok;
            g.save_cfg_ok = ok;
            if (!ok) {
                strncpy(g.last_err, "EEPROM write failed", sizeof(g.last_err)-1);
                g.last_err[sizeof(g.last_err)-1] = '\0';
            }
            xSemaphoreGive(g_mu);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void pressure_task(void*) {
    auto bus1 = g_i2c1;

    bool started = false;
    xSemaphoreTake(g_i2c1_mu, portMAX_DELAY);
    started = sdp610_start(bus1);
    xSemaphoreGive(g_i2c1_mu);

    xSemaphoreTake(g_mu, portMAX_DELAY);
    g.pressure_ok = started;
    xSemaphoreGive(g_mu);

    vTaskDelay(pdMS_TO_TICKS(50));

    while (true) {
        int16_t pa_x10 = 0;
        bool ok = false;

        xSemaphoreTake(g_i2c1_mu, portMAX_DELAY);
        ok = sdp610_read_pa_x10(bus1, pa_x10);
        xSemaphoreGive(g_i2c1_mu);

        xSemaphoreTake(g_mu, portMAX_DELAY);
        g.pressure_ok = ok;
        if (ok) {
            g.dp_pa_x10 = pa_x10;
        } else {
            strncpy(g.last_err, "Pressure read failed", sizeof(g.last_err) - 1);
            g.last_err[sizeof(g.last_err) - 1] = '\0';
        }
        xSemaphoreGive(g_mu);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void wifi_task(void*) {
    xSemaphoreTake(g_mu, portMAX_DELAY);
    g.wifi_ok = false;
    g.cloud_ok = false;
    xSemaphoreGive(g_mu);

    if (cyw43_arch_init()) {
        xSemaphoreTake(g_mu, portMAX_DELAY);
        strncpy(g.last_err, "CYW43 init failed", sizeof(g.last_err) - 1);
        g.last_err[sizeof(g.last_err) - 1] = '\0';
        xSemaphoreGive(g_mu);

        vTaskDelete(nullptr);
        return;
    }

    cyw43_arch_enable_sta_mode();

    uint8_t connected_profile = 0xFF;

    while (true) {
        bool wifi_now = false;
        bool reconnect_req = false;
        uint8_t wifi_profile = 0;

        xSemaphoreTake(g_mu, portMAX_DELAY);
        wifi_now = g.wifi_ok;
        reconnect_req = g.wifi_reconnect_req;
        wifi_profile = clamp_wifi_profile(g.wifi_profile);
        xSemaphoreGive(g_mu);

        const char* ssid = WIFI_PROFILES[wifi_profile].ssid;
        const char* password = WIFI_PROFILES[wifi_profile].password;

        if (ssid[0] == '\0') {
            xSemaphoreTake(g_mu, portMAX_DELAY);
            g.wifi_ok = false;
            g.cloud_ok = false;
            strncpy(g.last_err, "Selected SSID empty", sizeof(g.last_err) - 1);
            g.last_err[sizeof(g.last_err) - 1] = '\0';
            xSemaphoreGive(g_mu);

            vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_MS));
            continue;
        }

        const bool profile_changed_while_connected =
            wifi_now && (connected_profile != wifi_profile);

        if (reconnect_req || profile_changed_while_connected) {
            xSemaphoreTake(g_mu, portMAX_DELAY);
            g.wifi_ok = false;
            g.cloud_ok = false;
            g.wifi_reconnect_req = false;
            strncpy(g.last_err, "Reconnecting WiFi...", sizeof(g.last_err) - 1);
            g.last_err[sizeof(g.last_err) - 1] = '\0';
            xSemaphoreGive(g_mu);

            cyw43_arch_disable_sta_mode();
            vTaskDelay(pdMS_TO_TICKS(300));
            cyw43_arch_enable_sta_mode();

            connected_profile = 0xFF;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (!wifi_now) {
            int rc = cyw43_arch_wifi_connect_timeout_ms(
                ssid,
                password,
                CYW43_AUTH_WPA2_AES_PSK,
                45000
            );

            xSemaphoreTake(g_mu, portMAX_DELAY);
            g.wifi_ok = (rc == 0);
            if (rc == 0) {
                connected_profile = wifi_profile;
                strncpy(g.last_err, "WiFi connected", sizeof(g.last_err) - 1);
                g.last_err[sizeof(g.last_err) - 1] = '\0';
            } else {
                g.cloud_ok = false;
                snprintf(g.last_err, sizeof(g.last_err), "WiFi rc=%d", rc);
            }
            xSemaphoreGive(g_mu);

            if (!g.wifi_ok) {
                vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_MS));
                continue;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void cloud_task(void*) {
    TickType_t last_tx = 0;
    TickType_t last_poll = 0;

    while (true) {
        bool wifi = false;

        xSemaphoreTake(g_mu, portMAX_DELAY);
        wifi = g.wifi_ok;
        if (!wifi) {
            g.cloud_ok = false;
        }
        xSemaphoreGive(g_mu);

        if (!wifi) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        TickType_t now = xTaskGetTickCount();

        if ((now - last_tx) >= pdMS_TO_TICKS(CLOUD_TELEMETRY_MS)) {
            const bool ok = cloud_post_telemetry_thingspeak_once();

            xSemaphoreTake(g_mu, portMAX_DELAY);
            if (ok) g.telemetry_tx_count++;
            else    g.telemetry_tx_fail++;
            g.cloud_ok = ok;
            xSemaphoreGive(g_mu);

            last_tx = now;
        }

        if ((now - last_poll) >= pdMS_TO_TICKS(CLOUD_REMOTE_POLL_MS)) {

            uint16_t remote_sp = 0;
            if (cloud_get_remote_setpoint_thingspeak_once(remote_sp)) {
                apply_remote_setpoint(remote_sp);

                xSemaphoreTake(g_mu, portMAX_DELAY);
                g.cloud_ok = true;
                xSemaphoreGive(g_mu);
            }

            last_poll = now;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void controller_task(void*) {
    gpio_init(CO2_RELAY_PIN);
    gpio_set_dir(CO2_RELAY_PIN, GPIO_OUT);
    gpio_put(CO2_RELAY_PIN, 0);

    static uint16_t last_cmd = 0;

    while (true) {
        xSemaphoreTake(g_mu, portMAX_DELAY);
        const uint16_t co2 = g.co2_ppm;
        const uint16_t sp  = g.setpoint_ppm;
        TickType_t now = xTaskGetTickCount();

        g.safety_active = (co2 >= CO2_SAFETY_HIGH);

        // ===== Safety mode =====
        if (co2 >= CO2_SAFETY_HIGH) {
            g.fan_cmd_0_1000 = 1000;
            g.valve_open = false;
            gpio_put(CO2_RELAY_PIN, 0);
            g.stabilize_until = now + pdMS_TO_TICKS(VALVE_STABILIZE_MS);
            xSemaphoreGive(g_mu);
            vTaskDelay(pdMS_TO_TICKS(100));
            last_cmd = 1000;
            continue;
        }

        // ===== Valve pulse + stabilize =====
        if (g.valve_open) {
            if (now >= g.valve_until) {
                g.valve_open = false;
                gpio_put(CO2_RELAY_PIN, 0);
                g.stabilize_until = now + pdMS_TO_TICKS(VALVE_STABILIZE_MS);
            }
        } else {
            if (now >= g.stabilize_until) {
                if (co2 + CO2_HYST_LOW < sp) {
                    g.valve_open = true;
                    gpio_put(CO2_RELAY_PIN, 1);
                    g.valve_until = now + pdMS_TO_TICKS(VALVE_PULSE_MS);
                }
            }
        }

        // ===== Compute desired fan cmd =====
        uint16_t desired = 0;
        if (co2 > sp + CO2_HYST_HIGH) {
            int diff = (int)co2 - (int)sp;
            int cmd = 200 + diff * 2;
            cmd = std::clamp(cmd, 200, 1000);
            desired = (uint16_t)cmd;
        }

        // ===== Kick-start (0 -> >0) =====
        if (last_cmd == 0 && desired > 0) {
            g.fan_cmd_0_1000 = 700;    // kick
            last_cmd = 700;
            xSemaphoreGive(g_mu);

            vTaskDelay(pdMS_TO_TICKS(1000)); // 1s kick time

            xSemaphoreTake(g_mu, portMAX_DELAY);
            g.fan_cmd_0_1000 = desired; // settle
        } else {
            g.fan_cmd_0_1000 = desired;
        }

        last_cmd = g.fan_cmd_0_1000;

        xSemaphoreGive(g_mu);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void modbus_task(void*) {
    auto uart   = std::make_shared<PicoOsUart>(MODBUS_UART_NR, MODBUS_TX_PIN, MODBUS_RX_PIN, MODBUS_BAUD, MODBUS_STOPBITS);
    auto client = std::make_shared<ModbusClient>(uart);

    ModbusRegister co2(client, 240, 256, true);
    ModbusRegister rh (client, 241, 256, true);
    ModbusRegister tc (client, 241, 257, true);

    ModbusRegister fan_cmd(client, 1, 0, true);  // 40001 -> 1
    ModbusRegister pulses (client, 1, 4, false); // 30005 -> 5

    while (true) {
        uint16_t co2v = co2.read();
        int16_t  rhv  = (int16_t)rh.read();
        int16_t  tcv  = (int16_t)tc.read();
        uint16_t pul  = pulses.read();

        static uint8_t zero_pulse_streak = 0;

        uint16_t cmd = 0;
        xSemaphoreTake(g_mu, portMAX_DELAY);
        cmd = g.fan_cmd_0_1000;
        xSemaphoreGive(g_mu);

        // write to Produal/MIO AO1
        fan_cmd.write(cmd);

        static uint16_t prev_cmd = 0;
        static TickType_t grace_until = 0;
        TickType_t now = xTaskGetTickCount();

        if (cmd > 0 && prev_cmd == 0) {
            grace_until = now + pdMS_TO_TICKS(1500); // 1.5s grace after start
        }
        bool in_grace = (now < grace_until);
        prev_cmd = cmd;

        if (cmd > 0 && !in_grace) {
            if (pul == 0) zero_pulse_streak++;
            else zero_pulse_streak = 0;
        } else {
            zero_pulse_streak = 0;
        }
        bool fan_stopped_now = (zero_pulse_streak >= 2);

        bool ok = !(co2v == 0 && rhv == 0 && tcv == 0);

        xSemaphoreTake(g_mu, portMAX_DELAY);
        g.modbus_ok = ok;
        g.fan_stopped = fan_stopped_now;

        g.co2_ppm = co2v;
        g.rh_x10  = rhv;
        g.t_x10   = tcv;
        g.pulses  = pul;

        if (fan_stopped_now) {
            strncpy(g.last_err, "FAN STOPPED (pulses=0)", sizeof(g.last_err)-1);
            g.last_err[sizeof(g.last_err)-1] = '\0';
        }

        xSemaphoreGive(g_mu);

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static void display_task(void*) {
    auto bus1 = g_i2c1;

    uint8_t oled_addr = 0;
    xSemaphoreTake(g_i2c1_mu, portMAX_DELAY);
    oled_addr = detect_oled_addr(bus1);
    ssd1306os oled(bus1, oled_addr);
    xSemaphoreGive(g_i2c1_mu);

    auto header = [&](const char* title) {
        oled.text(title, 0, 0);
        oled.text("----------------", 0, 8);
    };

    while (true) {
        AppState s;
        xSemaphoreTake(g_mu, portMAX_DELAY);
        s = g;
        xSemaphoreGive(g_mu);

        oled.fill(0);

        char tbuf[16], rhbuf[16], pbuf[16];
        fmt_x10(tbuf, sizeof(tbuf), s.t_x10);
        fmt_x10(rhbuf, sizeof(rhbuf), s.rh_x10);
        fmt_x10(pbuf, sizeof(pbuf), s.dp_pa_x10);

        TickType_t now = xTaskGetTickCount();
        uint32_t pulse_ms = (now < s.valve_until)
            ? (uint32_t)((s.valve_until - now) * portTICK_PERIOD_MS) : 0;
        uint32_t stab_ms = (now < s.stabilize_until)
            ? (uint32_t)((s.stabilize_until - now) * portTICK_PERIOD_MS) : 0;

        const char* fan_state = "OFF";
        if (s.fan_cmd_0_1000 == 0) {
            fan_state = "OFF";
        } else if (s.fan_stopped) {
            fan_state = "FAULT";
        } else {
            fan_state = "RUN";
        }

        switch (s.page) {
            case Page::Home: {
                header("Greenhouse CO2");

                char l1[32];
                snprintf(l1, sizeof(l1), "%u ppm -> %u",
                        (unsigned)s.co2_ppm, (unsigned)s.setpoint_ppm);
                oled.text(l1, 0, 18);

                char l2[32];
                snprintf(l2, sizeof(l2), "T:%sC RH:%s%%", tbuf, rhbuf);
                oled.text(l2, 0, 30);

                char l3[32];
                snprintf(l3, sizeof(l3), "Fan:%u%% %s",
                        (unsigned)(s.fan_cmd_0_1000 / 10), fan_state);
                oled.text(l3, 0, 42);

                char l4[32];
                snprintf(l4, sizeof(l4), "Valve:%u Safe:%u",
                        s.valve_open ? 1u : 0u,
                        s.safety_active ? 1u : 0u);
                oled.text(l4, 0, 54);
                break;
            }

            case Page::SensorDetails: {
                header("Sensor Details");

                char l1[32];
                snprintf(l1, sizeof(l1), "dP: %s Pa", pbuf);
                oled.text(l1, 0, 18);

                char l2[32];
                snprintf(l2, sizeof(l2), "Pulses: %u", (unsigned)s.pulses);
                oled.text(l2, 0, 30);

                char l3[32];
                snprintf(l3, sizeof(l3), "Pulse:%lus",
                        (unsigned long)(pulse_ms / 1000));
                oled.text(l3, 0, 42);

                char l4[32];
                snprintf(l4, sizeof(l4), "Stab:%lus",
                        (unsigned long)(stab_ms / 1000));
                oled.text(l4, 0, 54);

                break;
            }

            case Page::SetTarget: {
                header("Set CO2 Target");

                char l1[32];
                snprintf(l1, sizeof(l1), "Target: %u ppm",
                         (unsigned)s.setpoint_ppm);
                oled.text(l1, 0, 18);

                if (!s.target_edit) {
                    oled.text("Rotate: page", 0, 30);
                    oled.text("Press: EDIT", 0, 42);
                } else {
                    oled.text("Rotate: target", 0, 30);
                    oled.text("Press: SAVE", 0, 42);
                }

                oled.text(s.save_cfg_ok ? "Last save: OK" : "Last save: FAIL", 0, 54);
                break;
            }

            case Page::Network: {
                header("Network/System");

                const uint8_t wp = clamp_wifi_profile(s.wifi_profile);
                char l1[32];
                snprintf(l1, sizeof(l1), "WiFi: %s (%s)",
                        s.wifi_ok ? "OK" : "DOWN",
                        WIFI_PROFILES[wp].name);
                oled.text(l1, 0, 18);

                oled.text(s.cloud_ok ? "Cloud: OK" : "Cloud: DOWN", 0, 30);
                oled.text(s.modbus_ok ? "Modbus: OK" : "Modbus: ERROR", 0, 42);
                oled.text(s.eeprom_ok ? "EEPROM: OK" : "EEPROM: ERROR", 0, 54);
                break;
            }

            case Page::WiFiSelect: {
                header("WiFi Select");

                const uint8_t wp = clamp_wifi_profile(s.wifi_profile);
                const char* name = WIFI_PROFILES[wp].name;
                const char* ssid = WIFI_PROFILES[wp].ssid;

                char l1[32];
                snprintf(l1, sizeof(l1), "Profile: %s", name);
                oled.text(l1, 0, 18);

                char l2[32];
                snprintf(l2, sizeof(l2), "SSID: %.18s", ssid);
                oled.text(l2, 0, 30);

                if (!s.wifi_edit) {
                    oled.text("Rotate: page", 0, 42);
                    oled.text("Press: EDIT", 0, 54);
                } else {
                    oled.text("Rotate: profile", 0, 42);
                    oled.text("Press: SAVE", 0, 54);
                }
                break;
            }

            case Page::ErrorLog: {
                header("Error Log");

                if (s.last_err[0] == '\0') {
                    oled.text("(empty)", 0, 26);
                } else {
                    oled.text(s.last_err, 0, 26);
                }
                break;
            }

            default:
                break;
        }

        xSemaphoreTake(g_i2c1_mu, portMAX_DELAY);
        oled.show();
        xSemaphoreGive(g_i2c1_mu);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    g_mu = xSemaphoreCreateMutex();
    g_i2c1_mu = xSemaphoreCreateMutex();
    g_i2c1 = std::make_shared<PicoI2C>(1, 100000);

    xTaskCreate(input_task,      "input",  1024, nullptr, tskIDLE_PRIORITY + 2, nullptr);
    xTaskCreate(eeprom_task,     "eeprom", 1024, nullptr, tskIDLE_PRIORITY + 1, nullptr);
    xTaskCreate(controller_task, "ctrl",   1024, nullptr, tskIDLE_PRIORITY + 2, nullptr);
    xTaskCreate(modbus_task,     "mb",     2048, nullptr, tskIDLE_PRIORITY + 2, nullptr);
    xTaskCreate(display_task,    "oled",   2048, nullptr, tskIDLE_PRIORITY + 1, nullptr);
    xTaskCreate(pressure_task, "press", 1024, nullptr, tskIDLE_PRIORITY + 1, nullptr);
    xTaskCreate(wifi_task,       "wifi",  4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
    xTaskCreate(cloud_task,      "cloud", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);

    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
}