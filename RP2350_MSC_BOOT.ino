/*
 * RP2350_MSC_BOOT — USB MSC → TCP Streaming Bridge
 * =================================================
 * Ванильный Pico SDK USB (usbstack=nousb). Adafruit NeoPixel для LED.
 * Дескрипторы = RP2350 bootloader. FAT16 128MB виртуальный диск.
 * Sparse storage для записей хоста. TCP через CH9120 UART.
 *
 * Алгоритм:
 *   1. Boot → CH9120 в RESET (ток < 5mA), TinyUSB инит, ждём монтирования.
 *   2. USB смонтирован → CH9120 поднимается, настраивается как TCP Client.
 *   3. Хост пишет файл .txt на виртуальный диск (FAT16 128MB).
 *   4. После WRITE_IDLE_MS тишины: читаем новые строки, парсим, шлём по TCP.
 *   5. После STREAM_IDLE_RESET_MS тишины: сброс диска (media changed).
 *
 * FQBN: rp2040:rp2040:waveshare_rp2350_plus:usbstack=nousb
 * Зависимости: Adafruit NeoPixel
 */

#include <Arduino.h>
#include <hardware/flash.h>
#include <tusb.h>
#include <stdarg.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"

/* ═══════════════════════════════════════════════════════════════════
   FAT16 layout constants (определяем до объявления глобалов)
   ═══════════════════════════════════════════════════════════════════ */
#define SECTOR_SIZE          512u
#define VOLUME_SIZE          (128u * 1024u * 1024u)
#define SECTOR_COUNT         (VOLUME_SIZE / SECTOR_SIZE)    // 262144
#define VOLUME_SECTOR_COUNT  (SECTOR_COUNT - 1u)
#define CLUSTER_SIZE         4096u
#define CLUSTER_COUNT        (VOLUME_SIZE / CLUSTER_SIZE)   // 32768
#define FAT_COUNT            2u
#define MAX_ROOT_DIR_ENTRIES 512u
#define ROOT_DIR_SECTORS     (MAX_ROOT_DIR_ENTRIES * 32u / SECTOR_SIZE)  // 32
#define SECTORS_PER_FAT      ((2u * CLUSTER_COUNT + SECTOR_SIZE - 1u) / SECTOR_SIZE)  // 128
#define MEDIA_TYPE           0xF8u

// Абсолютные LBA от MBR (LBA 0 = MBR)
#define ABS_LBA_FAT1   2u
#define ABS_LBA_FAT2   (ABS_LBA_FAT1 + SECTORS_PER_FAT)                  // 130
#define ABS_LBA_ROOT   (ABS_LBA_FAT2 + SECTORS_PER_FAT)                  // 258
#define ABS_LBA_DATA   (ABS_LBA_ROOT + ROOT_DIR_SECTORS)                  // 290

/* ═══════════════════════════════════════════════════════════════════
   Глобальные переменные
   ═══════════════════════════════════════════════════════════════════ */

// LED
static Adafruit_NeoPixel pixel(1, WS2812_PIN, NEO_GRB + NEO_KHZ800);
static uint32_t led_timer    = 0;
static bool     led_state    = false;
static uint8_t  flash_count  = 0;
static uint8_t  flash_total  = 0;
static uint16_t flash_on_ms  = 500;
static uint16_t flash_off_ms = 500;
static bool     flash_looped = false;
static uint32_t flash_color  = 0x181818u;
static volatile uint32_t g_led_write_ms = 0;

// FAT16 disk
typedef struct __attribute__((packed)) {
    uint8_t  name[11]; uint8_t attr; uint8_t reserved; uint8_t ctime_frac;
    uint16_t ctime; uint16_t cdate; uint16_t adate; uint16_t cluster_hi;
    uint16_t mtime; uint16_t mdate; uint16_t cluster_lo; uint32_t size;
} DirEntry;
static uint32_t disk_serial;
static uint8_t  sec_buf[SECTOR_SIZE];

// Sparse sector storage
struct WrittenSector { uint32_t lba; uint8_t data[SECTOR_SIZE]; };
static WrittenSector written_sectors[MAX_WRITTEN_SECTORS];
static uint8_t written_count      = 0;  // всего секторов в пуле
static uint8_t written_data_count = 0;  // только data-секторы (LBA >= ABS_LBA_DATA)

// State machine
enum class State : uint8_t { IDLE, STREAMING };
static volatile State    state            = State::IDLE;
static volatile bool     g_write_pending  = false;
static volatile uint32_t g_last_write_ms  = 0;
static uint32_t          g_stream_cluster  = 0;
static uint32_t          g_processed_bytes = 0;

// USB
static volatile bool    usb_mounted = false;
static volatile uint8_t usb_events  = 0;
#define USB_EV_MOUNT  0x01u
#define USB_EV_UMOUNT 0x02u
static bool    msc_ready   = true;
static char    serial_str[17];

// Streaming
struct TxtFile { uint32_t start_cluster; uint32_t file_size; };
static char    s_line_buf[80];
static uint8_t s_line_len = 0;

// Debug/heartbeat
static bool     dbg_tcp_prev = false;
static uint32_t hb_last_ms   = 0;

/* ═══════════════════════════════════════════════════════════════════
   LED helpers
   ═══════════════════════════════════════════════════════════════════ */
#define LED_OFF    0x000000u
#define LED_RED    0x300000u  // IDLE, нет TCP
#define LED_BLUE   0x000030u  // IDLE, TCP OK
#define LED_GREEN  0x003000u  // STREAMING
#define LED_WHITE  0x181818u  // boot / success
#define LED_ORANGE 0x200800u  // ожидание TCP

static void led_set(uint32_t color) {
    pixel.setPixelColor(0, pixel.gamma32(color));
    pixel.show();
}

static void led_flash(uint8_t count, uint32_t color,
                      uint16_t on_ms = 500, uint16_t off_ms = 500,
                      bool loop = false) {
    flash_color  = color;
    flash_total  = count;
    flash_count  = 0;
    flash_on_ms  = on_ms;
    flash_off_ms = off_ms;
    flash_looped = loop;
    led_state    = false;
    led_timer    = 0;
    led_set(LED_OFF);
}

static void led_flash_sync(uint8_t count, uint32_t color,
                            uint16_t on_ms = 100, uint16_t off_ms = 100) {
    for (uint8_t i = 0; i < count; i++) {
        led_set(color); delay(on_ms);
        led_set(LED_OFF); delay(off_ms);
    }
}

// Градиент по заполнению data-секторов пула: белый(0%) → зелёный(33%) → жёлтый(66%) → красный(100%)
// Используем written_data_count (LBA >= ABS_LBA_DATA), FS-метаданные в процент не входят
static uint32_t write_fill_color() {
    // Ёмкость для данных = пул минус типичный overhead файловой системы (~20 секторов)
    static const uint8_t DATA_CAPACITY = MAX_WRITTEN_SECTORS - 20u;
    uint8_t pct = (uint8_t)((uint32_t)written_data_count * 100u / DATA_CAPACITY);
    uint8_t r, g, b;
    if (pct < 33u) {
        uint8_t t = pct * 255u / 33u;
        r = 0x18u - (uint8_t)((uint16_t)0x18u * t / 255u);
        g = 0x18u + (uint8_t)((uint16_t)0x18u * t / 255u);
        b = 0x18u - (uint8_t)((uint16_t)0x18u * t / 255u);
    } else if (pct < 66u) {
        uint8_t t = (pct - 33u) * 255u / 33u;
        r = (uint8_t)((uint16_t)0x30u * t / 255u); g = 0x30u; b = 0u;
    } else {
        uint8_t t = (uint8_t)min(255, (int)(pct - 66) * 255 / 34);
        r = 0x30u; g = 0x30u - (uint8_t)((uint16_t)0x30u * t / 255u); b = 0u;
    }
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

static void led_update() {
    // Запись активна: мигаем 5 Гц, цвет = градиент заполнения sparse pool
    if (millis() - g_led_write_ms < 500u) {
        led_set(((millis() / 100u) % 2u) ? write_fill_color() : LED_OFF);
        return;
    }
    if (flash_total == 0u) return;
    uint32_t now = millis();
    if (now - led_timer < (uint32_t)(led_state ? flash_on_ms : flash_off_ms)) return;
    led_timer = now;
    if (!led_state) {
        if (flash_count < flash_total) { led_state = true; led_set(flash_color); }
    } else {
        led_state = false; led_set(LED_OFF);
        if (++flash_count >= flash_total) {
            if (flash_looped) flash_count = 0; else flash_total = 0;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════
   USB-friendly delay — вызываем tud_task() во время ожидания
   ═══════════════════════════════════════════════════════════════════ */
static void tud_delay(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) { tud_task(); delay(1); }
}

/* ═══════════════════════════════════════════════════════════════════
   FAT16 128MB virtual disk — fill_sector (только чтение)
   ═══════════════════════════════════════════════════════════════════ */
#define RPI_TIME ((16u<<11u)|(20u<<5u)|(51u>>1u))
#define RPI_DATE ((28u<<9u)|(9u<<5u)|(5u))

static void fill_sector(uint32_t lba, uint8_t *buf) {
    memset(buf, 0, SECTOR_SIZE);

    if (lba == 0u) {  // MBR
        uint8_t *pt = buf + SECTOR_SIZE - 2u - 64u;
        pt[4] = 0x0Eu;
        pt[8] = 1u; pt[9] = 0u; pt[10] = 0u; pt[11] = 0u;
        uint32_t sc = SECTOR_COUNT - 1u; memcpy(pt + 12, &sc, 4);
        memcpy(buf + 0x1B8u, &disk_serial, 4);
        buf[SECTOR_SIZE-2u] = 0x55u; buf[SECTOR_SIZE-1u] = 0xAAu;
        return;
    }
    lba--;

    if (lba == 0u) {  // VBR
        buf[0]=0xEBu; buf[1]=0x3Cu; buf[2]=0x90u;
        memcpy(buf+3, "MSWIN4.1", 8);
        buf[11]=0x00u; buf[12]=0x02u;
        buf[13]=(uint8_t)(CLUSTER_SIZE/SECTOR_SIZE);
        buf[14]=1u; buf[15]=0u; buf[16]=FAT_COUNT;
        uint16_t rde=MAX_ROOT_DIR_ENTRIES; memcpy(buf+17, &rde, 2);
        buf[19]=0u; buf[20]=0u; buf[21]=MEDIA_TYPE;
        uint16_t spf=SECTORS_PER_FAT; memcpy(buf+22, &spf, 2);
        buf[24]=1u; buf[25]=0u; buf[26]=1u; buf[27]=0u;
        uint32_t hs=1u; memcpy(buf+28, &hs, 4);
        uint32_t vsc=VOLUME_SECTOR_COUNT; memcpy(buf+32, &vsc, 4);
        buf[36]=0x00u; buf[37]=0x00u; buf[38]=0x29u;
        memcpy(buf+39, &disk_serial, 4);
        memcpy(buf+43, "RPI-RP2350 ", 11);
        memcpy(buf+54, "FAT16   ", 8);
        buf[SECTOR_SIZE-2u]=0x55u; buf[SECTOR_SIZE-1u]=0xAAu;
        return;
    }
    lba--;

    if (lba < SECTORS_PER_FAT * FAT_COUNT) {  // FAT1 + FAT2 (пустые)
        if ((lba % SECTORS_PER_FAT) == 0u) {
            uint16_t *p = (uint16_t*)buf;
            p[0] = 0xFF00u | MEDIA_TYPE;  // FAT ID (cluster 0)
            p[1] = 0xFFFFu;               // cluster 1 reserved
        }
        return;
    }
    lba -= SECTORS_PER_FAT * FAT_COUNT;

    if (lba < ROOT_DIR_SECTORS) {  // Root dir: только метка тома
        if (lba == 0u) {
            DirEntry *e = (DirEntry*)buf;
            memcpy(e[0].name, "RPI-RP2350 ", 11);
            e[0].attr  = 0x08u | 0x20u;  // VOLUME_LABEL | ARCHIVE
            e[0].ctime = RPI_TIME; e[0].cdate = RPI_DATE;
            e[0].mtime = RPI_TIME; e[0].mdate = RPI_DATE;
        }
        return;
    }
    // Data area: нули
}

/* ═══════════════════════════════════════════════════════════════════
   Sparse sector storage
   ═══════════════════════════════════════════════════════════════════ */
static const uint8_t* get_written(uint32_t lba) {
    for (uint8_t i = 0; i < written_count; i++)
        if (written_sectors[i].lba == lba) return written_sectors[i].data;
    return nullptr;
}

static void store_written(uint32_t lba, const uint8_t* data,
                           uint32_t offset, uint32_t size) {
    for (uint8_t i = 0; i < written_count; i++) {
        if (written_sectors[i].lba == lba) {
            memcpy(written_sectors[i].data + offset, data, size);
            return;
        }
    }
    if (written_count >= MAX_WRITTEN_SECTORS) return;
    written_sectors[written_count].lba = lba;
    if (offset != 0u || size != SECTOR_SIZE)
        fill_sector(lba, written_sectors[written_count].data);
    else
        memset(written_sectors[written_count].data, 0, SECTOR_SIZE);
    memcpy(written_sectors[written_count].data + offset, data, size);
    written_count++;
    if (lba >= ABS_LBA_DATA) written_data_count++;
}

/* ═══════════════════════════════════════════════════════════════════
   FAT16 traversal + directory scan
   ═══════════════════════════════════════════════════════════════════ */
static uint16_t fat16_next(uint32_t cluster) {
    uint32_t sec_off  = cluster >> 8u;
    uint32_t byte_off = (cluster & 0xFFu) << 1u;
    const uint8_t* sec = get_written(ABS_LBA_FAT1 + sec_off);
    if (!sec) return 0xFFFFu;
    return sec[byte_off] | ((uint16_t)sec[byte_off + 1u] << 8u);
}

// Forward declaration for mutual recursion
static void fat16_scan_dir(uint32_t cluster, TxtFile& out, uint8_t depth);

static void fat16_scan_sector(const uint8_t* sec, TxtFile& out, uint8_t depth) {
    auto uc = [](uint8_t c) -> uint8_t {
        return (c >= 'a' && c <= 'z') ? (uint8_t)(c - 32u) : c;
    };
    for (int i = 0; i < (int)(SECTOR_SIZE / 32u); i++) {
        const uint8_t* de = sec + i * 32;
        if (de[0] == 0x00u) return;
        if (de[0] == 0xE5u || de[11] == 0x0Fu || (de[11] & 0x08u)) continue;
        uint16_t cl = de[26] | ((uint16_t)de[27] << 8u);
        if (de[11] & 0x10u) {
            if (de[0] != (uint8_t)'.' && cl >= 2u && depth > 0u)
                fat16_scan_dir(cl, out, depth - 1u);
        } else {
            if (uc(de[8])=='T' && uc(de[9])=='X' && uc(de[10])=='T') {
                uint32_t sz = de[28] | ((uint32_t)de[29]<<8u)
                            | ((uint32_t)de[30]<<16u) | ((uint32_t)de[31]<<24u);
                if (sz > 0u && cl >= 2u && cl > out.start_cluster) {
                    out.start_cluster = cl; out.file_size = sz;
                }
            }
        }
    }
}

static void fat16_scan_dir(uint32_t cluster, TxtFile& out, uint8_t depth) {
    uint8_t chain = 0;
    while (cluster >= 2u && cluster < 0xFFF8u && chain < 16u) {
        uint32_t lba0 = ABS_LBA_DATA + (cluster - 2u) * (CLUSTER_SIZE / SECTOR_SIZE);
        for (uint32_t s = 0u; s < CLUSTER_SIZE / SECTOR_SIZE; s++) {
            const uint8_t* sec = get_written(lba0 + s);
            if (sec) fat16_scan_sector(sec, out, depth);
        }
        cluster = fat16_next(cluster); chain++;
    }
}

static bool fat16_find_txt(TxtFile& out) {
    out.start_cluster = 0u; out.file_size = 0u;
    for (uint32_t lba = ABS_LBA_ROOT; lba < ABS_LBA_DATA; lba++) {
        const uint8_t* sec = get_written(lba);
        if (sec) fat16_scan_sector(sec, out, 2u);  // root → subdir → subsubdir
    }
    return out.start_cluster >= 2u;
}

/* ═══════════════════════════════════════════════════════════════════
   TinyUSB MSC callbacks
   ═══════════════════════════════════════════════════════════════════ */
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vid[8],
                         uint8_t pid[16], uint8_t rev[4]) {
    (void)lun;
    memcpy(vid, "RPI     ", 8);
    memcpy(pid, "RP2350          ", 16);
    memcpy(rev, "2   ",             4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void)lun; return msc_ready;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *bc, uint16_t *bs) {
    (void)lun; *bc = SECTOR_COUNT; *bs = SECTOR_SIZE;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t pwr, bool start, bool eject) {
    (void)lun; (void)pwr; (void)start; (void)eject; return true;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba,
                           uint32_t offset, void *buf, uint32_t bufsize) {
    (void)lun;
    const uint8_t* wr = get_written(lba);
    if (wr) memcpy(buf, wr + offset, bufsize);
    else { fill_sector(lba, sec_buf); memcpy(buf, sec_buf + offset, bufsize); }
    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba,
                            uint32_t offset, uint8_t *buf, uint32_t bufsize) {
    (void)lun;
    // FAT2 игнорируем (идентична FAT1, не тратим слоты на неё)
    if (lba >= ABS_LBA_FAT2 && lba < ABS_LBA_ROOT) {
        g_write_pending = true;
        g_last_write_ms = g_led_write_ms = millis();
        return (int32_t)bufsize;
    }
    store_written(lba, buf, offset, bufsize);
    g_write_pending = true;
    g_last_write_ms = g_led_write_ms = millis();
    if (state == State::IDLE) state = State::STREAMING;
    return (int32_t)bufsize;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const cmd[16],
                         void *buf, uint16_t bufsize) {
    (void)lun; (void)cmd; (void)buf; (void)bufsize;
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20u, 0x00u);
    return -1;
}

/* ═══════════════════════════════════════════════════════════════════
   TinyUSB descriptor callbacks — точная копия RP2350 bootloader
   ═══════════════════════════════════════════════════════════════════ */
static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,  // USB 2.0, без BOS — Windows ставит usbstor.sys
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x2E8A,  // Raspberry Pi
    .idProduct          = 0x000F,  // RP2350 Boot
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

const uint8_t *tud_descriptor_device_cb(void) {
    return (const uint8_t *)&desc_device;
}

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)
static const uint8_t desc_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0x80, 50),  // bus-powered, 50mA
    TUD_MSC_DESCRIPTOR(0, 0, 0x02, 0x81, 64)
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index; return desc_config;
}

static const uint16_t* make_str(const char* s, uint16_t* buf, uint8_t cap) {
    uint8_t len = strlen(s); if (len > cap-1) len = cap-1;
    buf[0] = (TUSB_DESC_STRING << 8) | (2 + 2*len);
    for (uint8_t i = 0; i < len; i++) buf[1+i] = s[i];
    return buf;
}

const uint16_t *tud_descriptor_string_cb(uint8_t idx, uint16_t langid) {
    (void)langid;
    static uint16_t buf[32];
    switch (idx) {
        case 0: buf[0]=(TUSB_DESC_STRING<<8)|4; buf[1]=0x0409; return buf;
        case 1: return make_str("Raspberry Pi", buf, 32);
        case 2: return make_str("RP2350 Boot",  buf, 32);
        case 3: return make_str(serial_str,     buf, 32);
        default: return NULL;
    }
}

/* ═══════════════════════════════════════════════════════════════════
   USB mount callbacks
   ═══════════════════════════════════════════════════════════════════ */
void tud_mount_cb(void)   { usb_mounted = true;  usb_events |= USB_EV_MOUNT;  }
void tud_umount_cb(void)  { usb_mounted = false; usb_events |= USB_EV_UMOUNT; }
void tud_suspend_cb(bool) {}
void tud_resume_cb(void)  {}

/* ═══════════════════════════════════════════════════════════════════
   CH9120 — конфигурация TCP Client
   ═══════════════════════════════════════════════════════════════════ */
static void ch9120_raw_cmd(uint8_t cmd, const uint8_t* data, uint8_t len) {
    const uint8_t hdr[2] = {0x57, 0xAB};
    CH9120_UART.write(hdr, 2);
    CH9120_UART.write(cmd);
    if (data && len > 0) CH9120_UART.write(data, len);
    CH9120_UART.flush();
    tud_delay(50);
}

static void ch9120_cfg_enter() {
    CH9120_UART.setTX(CH9120_TX_PIN);
    CH9120_UART.setRX(CH9120_RX_PIN);
    CH9120_UART.begin(CH9120_CFG_BAUD);
    while (CH9120_UART.available()) CH9120_UART.read();
    digitalWrite(CH9120_CFG_PIN, LOW);
    tud_delay(500);
}

static void ch9120_cfg_exit(bool save_eeprom) {
    const uint8_t hdr[2] = {0x57, 0xAB};
    if (save_eeprom) {
        CH9120_UART.write(hdr, 2); CH9120_UART.write((uint8_t)0x0D);
        CH9120_UART.flush(); tud_delay(200);
    }
    CH9120_UART.write(hdr, 2); CH9120_UART.write((uint8_t)0x0E);
    CH9120_UART.flush(); tud_delay(200);
    CH9120_UART.write(hdr, 2); CH9120_UART.write((uint8_t)0x5E);
    CH9120_UART.flush(); tud_delay(500);
    digitalWrite(CH9120_CFG_PIN, HIGH);
    CH9120_UART.setTX(CH9120_TX_PIN);
    CH9120_UART.setRX(CH9120_RX_PIN);
    CH9120_UART.begin(CH9120_DATA_BAUD);
}

static void ch9120_setup_tcp() {
    ch9120_cfg_enter();
    uint8_t mode = 0x01; ch9120_raw_cmd(0x10, &mode, 1);  // TCP Client
    uint8_t lip[]  = LOCAL_IP_BYTES;   ch9120_raw_cmd(0x11, lip,  4);
    uint8_t mask[] = LOCAL_MASK_BYTES; ch9120_raw_cmd(0x12, mask, 4);
    uint8_t gw[]   = LOCAL_GW_BYTES;   ch9120_raw_cmd(0x13, gw,   4);
    uint8_t lp[2]  = {(uint8_t)(LOCAL_PORT & 0xFF), (uint8_t)(LOCAL_PORT >> 8)};
    ch9120_raw_cmd(0x14, lp, 2);
    uint8_t dip[]  = SERVER_IP_BYTES;  ch9120_raw_cmd(0x15, dip,  4);
    uint8_t dp[2]  = {(uint8_t)(SERVER_PORT & 0xFF), (uint8_t)(SERVER_PORT >> 8)};
    ch9120_raw_cmd(0x16, dp, 2);
    uint32_t baud  = CH9120_DATA_BAUD;
    uint8_t bd[4]  = {(uint8_t)baud, (uint8_t)(baud>>8),
                      (uint8_t)(baud>>16), (uint8_t)(baud>>24)};
    ch9120_raw_cmd(0x21, bd, 4);
    ch9120_cfg_exit(true);
}

/* ═══════════════════════════════════════════════════════════════════
   TCP helpers
   ═══════════════════════════════════════════════════════════════════ */
static bool tcp_connected() {
    return digitalRead(CH9120_TCPCS_PIN) == LOW;
}

static bool tcp_wait_connected(uint32_t timeout_ms) {
    uint32_t t0 = millis();
    while (!tcp_connected() && millis() - t0 < timeout_ms) {
        tud_task(); delay(50);
    }
    return tcp_connected();
}

/* ═══════════════════════════════════════════════════════════════════
   Debug: фреймированный вывод [0x01][len][text] → TCP
   ═══════════════════════════════════════════════════════════════════ */
static void dbg_send(const char* msg) {
    uint8_t len = (uint8_t)min((size_t)255, strlen(msg));
    uint8_t hdr[2] = {0x01, len};
    CH9120_UART.write(hdr, 2);
    CH9120_UART.write((const uint8_t*)msg, len);
    CH9120_UART.flush();
}

static void dbg_sendf(const char* fmt, ...) {
    char buf[200];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    dbg_send(buf);
}

/* ═══════════════════════════════════════════════════════════════════
   Streaming: парсинг строк и отправка по TCP
   ═══════════════════════════════════════════════════════════════════ */

// Вход:  "891318   79.70    79.20"
// Выход: "891318:79.70;891319:79.20\r\n"
static void parse_and_send_line(const char* line, uint8_t len) {
    if (len == 0 || line[0] < '0' || line[0] > '9') return;
    uint8_t i = 0;
    uint32_t ev = 0;
    while (i < len && line[i] >= '0' && line[i] <= '9')
        ev = ev * 10u + (uint32_t)(line[i++] - '0');
    while (i < len && line[i] == ' ') i++;
    char w1[12]; uint8_t w1n = 0;
    while (i < len && w1n < 11 &&
           ((line[i] >= '0' && line[i] <= '9') || line[i] == '.'))
        w1[w1n++] = line[i++];
    w1[w1n] = '\0';
    while (i < len && line[i] == ' ') i++;
    char w2[12]; uint8_t w2n = 0;
    while (i < len && w2n < 11 &&
           ((line[i] >= '0' && line[i] <= '9') || line[i] == '.'))
        w2[w2n++] = line[i++];
    w2[w2n] = '\0';
    if (w1n == 0 || w2n == 0) return;
    char out[56];
    int n = snprintf(out, sizeof(out), "%lu:%s;%lu:%s\r\n",
                     (unsigned long)ev, w1, (unsigned long)(ev + 1u), w2);
    if (n > 0) { CH9120_UART.write((const uint8_t*)out, n); CH9120_UART.flush(); }
}

static void stream_process_new(uint32_t start_cluster,
                                uint32_t from_pos, uint32_t to_pos) {
    uint32_t cluster    = start_cluster;
    uint32_t clust_base = 0u;

    // Пропускаем кластеры до from_pos
    while (cluster >= 2u && cluster < 0xFFF8u &&
           clust_base + CLUSTER_SIZE <= from_pos) {
        clust_base += CLUSTER_SIZE;
        cluster = fat16_next(cluster);
    }

    while (cluster >= 2u && cluster < 0xFFF8u && clust_base < to_pos) {
        uint32_t lba0 = ABS_LBA_DATA + (cluster - 2u) * (CLUSTER_SIZE / SECTOR_SIZE);
        bool cluster_done = false;

        for (uint32_t s = 0u; s < CLUSTER_SIZE / SECTOR_SIZE; s++) {
            uint32_t sec_base = clust_base + s * SECTOR_SIZE;
            if (sec_base >= to_pos) { cluster_done = true; break; }

            const uint8_t* sec = get_written(lba0 + s);
            if (!sec) continue;

            uint32_t i0 = (from_pos > sec_base) ? (from_pos - sec_base) : 0u;
            uint32_t i1 = (to_pos < sec_base + SECTOR_SIZE)
                          ? (to_pos - sec_base) : (uint32_t)SECTOR_SIZE;

            for (uint32_t i = i0; i < i1; i++) {
                uint8_t c = sec[i];
                if (c == '\r') continue;
                if (c == '\n') {
                    s_line_buf[s_line_len] = '\0';
                    parse_and_send_line(s_line_buf, s_line_len);
                    s_line_len = 0;
                } else if (s_line_len < (uint8_t)(sizeof(s_line_buf) - 1u)) {
                    s_line_buf[s_line_len++] = (char)c;
                }
            }
        }
        if (cluster_done) break;
        clust_base += CLUSTER_SIZE;
        cluster = fat16_next(cluster);
    }
}

/* ═══════════════════════════════════════════════════════════════════
   Disk reset — очищаем sparse storage, сигнал хосту: media changed
   ═══════════════════════════════════════════════════════════════════ */
static void disk_reset() {
    dbg_sendf("DISK RESET: processed=%lu wr=%u",
              (unsigned long)g_processed_bytes, (unsigned)written_count);
    written_count      = 0;
    written_data_count = 0;
    g_stream_cluster  = 0;
    g_processed_bytes = 0;
    s_line_len        = 0;
    state             = State::IDLE;

    msc_ready = false;
    tud_delay(250);  // хост видит NOT READY → сбрасывает кэш ФС
    msc_ready = true;

    led_flash_sync(2, LED_WHITE, 80, 80);
    led_flash(255, tcp_connected() ? LED_BLUE : LED_RED, 500, 500, true);
}

/* ═══════════════════════════════════════════════════════════════════
   setup()
   ═══════════════════════════════════════════════════════════════════ */
void setup() {
    pixel.begin();
    pixel.setBrightness(255);
    led_set(LED_WHITE);

    // CH9120 в RESET — < 5mA, не превышаем лимит 100mA строгих хостов
    pinMode(CH9120_CFG_PIN,   OUTPUT);
    pinMode(CH9120_RST_PIN,   OUTPUT);
    pinMode(CH9120_TCPCS_PIN, INPUT_PULLUP);
    digitalWrite(CH9120_CFG_PIN, HIGH);
    digitalWrite(CH9120_RST_PIN, LOW);

    // Boot alive: 2 вспышки белым
    led_flash_sync(2, LED_WHITE, 80, 80);

    // Serial number из Chip ID
    uint8_t id[8]; flash_get_unique_id(id);
    snprintf(serial_str, sizeof(serial_str),
             "%02X%02X%02X%02X%02X%02X%02X%02X",
             id[0],id[1],id[2],id[3],id[4],id[5],id[6],id[7]);
    memcpy(&disk_serial, id, 4);

    // TinyUSB init
    tusb_rhport_init_t dev_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO };
    tusb_init(0, &dev_init);

    // Ждём монтирования USB (до USB_MOUNT_TIMEOUT_MS)
    // CH9120 всё ещё в RESET
    {
        uint32_t t0 = millis();
        while (!usb_mounted && millis() - t0 < USB_MOUNT_TIMEOUT_MS) {
            tud_task(); delay(1);
        }
    }

    // Поднимаем CH9120 и настраиваем TCP
    digitalWrite(CH9120_RST_PIN, HIGH);
    tud_delay(300);
    ch9120_setup_tcp();

    // Ждём TCP-соединения
    led_flash(255, LED_ORANGE, 100, 100, true);
    bool tcp_ok = tcp_wait_connected(TCP_CONNECT_TIMEOUT_MS);

    dbg_sendf("BOOT: USB=%s TCP=%s wr_pool=%u t=%lums",
              usb_mounted ? "MOUNTED" : "TIMEOUT",
              tcp_ok ? "OK" : "TIMEOUT",
              (unsigned)MAX_WRITTEN_SECTORS,
              (unsigned long)millis());
    dbg_send("READY");

    if (tcp_ok) led_flash_sync(3, LED_WHITE, 80, 80);
    dbg_tcp_prev = tcp_ok;
    led_flash(255, tcp_ok ? LED_BLUE : LED_RED, 500, 500, true);
}

/* ═══════════════════════════════════════════════════════════════════
   loop()
   ═══════════════════════════════════════════════════════════════════ */
void loop() {
    tud_task();
    led_update();

    // USB events
    if (usb_events) {
        uint8_t ev = usb_events; usb_events = 0;
        if (ev & USB_EV_MOUNT)  dbg_send("USB: MOUNTED");
        if (ev & USB_EV_UMOUNT) dbg_send("USB: UNMOUNTED");
    }

    // Heartbeat каждые 5 сек
    uint32_t now = millis();
    if (now - hb_last_ms >= 5000u) {
        hb_last_ms = now;
        dbg_sendf("HB: usb=%d tcp=%d wr=%u/%u st=%d t=%lu",
                  (int)usb_mounted, (int)tcp_connected(),
                  (unsigned)written_count, (unsigned)MAX_WRITTEN_SECTORS,
                  (int)state, (unsigned long)now);
    }

    // Детект смены TCP
    bool tcp_now = tcp_connected();
    if (tcp_now != dbg_tcp_prev) {
        dbg_sendf("TCP %s", tcp_now ? "CONNECTED" : "DISCONNECTED");
        dbg_tcp_prev = tcp_now;
    }

    switch (state) {
    case State::IDLE:
        if (!tcp_now && flash_color != LED_RED)
            led_flash(255, LED_RED,  500, 500, true);
        else if (tcp_now && flash_color != LED_BLUE)
            led_flash(255, LED_BLUE, 500, 500, true);
        break;

    case State::STREAMING:
        // Выключаем IDLE-мигание во время streaming
        if (flash_total != 0u && (flash_color == LED_RED || flash_color == LED_BLUE)) {
            flash_total = 0u; led_set(LED_OFF);
        }

        // Переполнение sparse pool → немедленный сброс
        if (written_count >= MAX_WRITTEN_SECTORS - 4u) {
            disk_reset(); break;
        }

        // Тишина STREAM_IDLE_RESET_MS → сброс диска
        if (!g_write_pending && now - g_last_write_ms > STREAM_IDLE_RESET_MS) {
            disk_reset(); break;
        }

        // Ждём накопления и паузы после последней записи
        if (!g_write_pending) break;
        if (now - g_last_write_ms < WRITE_IDLE_MS) break;

        g_write_pending = false;
        {
            uint8_t  _pct = (uint8_t)((uint32_t)written_count * 100u / MAX_WRITTEN_SECTORS);
            uint32_t _c   = write_fill_color();
            dbg_sendf("LED: wr=%u data=%u/%u pct=%u%% rgb=#%02X%02X%02X",
                      (unsigned)written_count,
                      (unsigned)written_data_count, (unsigned)(MAX_WRITTEN_SECTORS - 20u),
                      (unsigned)_pct,
                      (unsigned)((_c >> 16) & 0xFFu),
                      (unsigned)((_c >>  8) & 0xFFu),
                      (unsigned)( _c        & 0xFFu));
        }
        {
            TxtFile f;
            if (!fat16_find_txt(f)) {
                dbg_send("STREAM: no .txt -> IDLE");
                g_stream_cluster = 0; g_processed_bytes = 0; s_line_len = 0;
                state = State::IDLE;
                led_flash(255, tcp_now ? LED_BLUE : LED_RED, 500, 500, true);
                break;
            }
            if (g_stream_cluster != 0u && f.start_cluster != g_stream_cluster) {
                dbg_sendf("STREAM: new file (cl %lu->%lu)",
                          (unsigned long)g_stream_cluster,
                          (unsigned long)f.start_cluster);
                g_processed_bytes = 0; s_line_len = 0;
            }
            g_stream_cluster = f.start_cluster;
            if (f.file_size > g_processed_bytes) {
                stream_process_new(f.start_cluster, g_processed_bytes, f.file_size);
                g_processed_bytes = f.file_size;
            }
        }
        break;
    }
}
