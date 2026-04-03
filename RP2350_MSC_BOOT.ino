/*
 * RP2350_MSC_BOOT — USB MSC minimal, чистый Pico SDK
 * ===================================================
 * Нет внешних библиотек. Дескрипторы = точная копия RP2350 bootloader.
 * FAT16 128MB виртуальный диск. LED через WS2812B PIO.
 *
 * FQBN: rp2040:rp2040:waveshare_rp2350_plus:usbstack=nousb
 */

#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/flash.h>
#include <tusb.h>

/* ─── WS2812B через PIO ─────────────────────────────────────────── */
#define WS2812_PIN  21

// Программа ws2812 (T1=2 T2=5 T3=3 → 10 циклов @ 8MHz = 800kHz)
// Взято из pico-examples/pio/ws2812/ws2812.pio.h
static const uint16_t ws2812_instr[] = {
    0x6221, // out x, 1        side 0 [2]
    0x1223, // jmp !x, 3       side 1 [1]
    0x1200, // jmp 0           side 1 [4]
    0xa242, // nop             side 0 [4]
};
static const pio_program_t ws2812_prog = {
    .instructions = ws2812_instr, .length = 4, .origin = -1
};

static PIO  ws_pio;
static uint ws_sm;

static void led_init(void) {
    ws_pio = pio0;
    uint offset = pio_add_program(ws_pio, &ws2812_prog);
    ws_sm = pio_claim_unused_sm(ws_pio, true);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_sideset(&c, 1, false, false);
    // LEFT shift, autopull 24 бита
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    // Делитель: 8 МГц
    float div = (float)clock_get_hz(clk_sys) / 8000000.0f;
    sm_config_set_clkdiv(&c, div);
    sm_config_set_sideset_pins(&c, WS2812_PIN);
    sm_config_set_wrap(&c, offset, offset + 3);
    pio_gpio_init(ws_pio, WS2812_PIN);
    pio_sm_set_consecutive_pindirs(ws_pio, ws_sm, WS2812_PIN, 1, true);
    pio_sm_init(ws_pio, ws_sm, offset, &c);
    pio_sm_set_enabled(ws_pio, ws_sm, true);
}

// color: 0xRRGGBB
static void led_set(uint32_t color) {
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8)  & 0xFF;
    uint8_t b =  color        & 0xFF;
    // WS2812B ожидает GRB
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    pio_sm_put_blocking(ws_pio, ws_sm, grb << 8);
}

#define C_OFF    0x000000u
#define C_WHITE  0x181818u   // ожидание монтирования
#define C_GREEN  0x001800u   // смонтирован
#define C_BLUE   0x000018u   // идёт запись

/* ─── FAT16 128MB виртуальный диск ─────────────────────────────── */
#define SECTOR_SIZE          512u
#define VOLUME_SIZE          (128u * 1024u * 1024u)
#define SECTOR_COUNT         (VOLUME_SIZE / SECTOR_SIZE)   // 262144
#define VOLUME_SECTOR_COUNT  (SECTOR_COUNT - 1u)           // -1 для MBR

#define CLUSTER_SIZE         4096u
#define CLUSTER_SHIFT        3u      // log2(CLUSTER_SIZE/SECTOR_SIZE) = log2(8) = 3
#define CLUSTER_COUNT        (VOLUME_SIZE / CLUSTER_SIZE)  // 32768

#define FAT_COUNT            2u
#define MAX_ROOT_DIR_ENTRIES 512u
#define ROOT_DIR_SECTORS     (MAX_ROOT_DIR_ENTRIES * 32u / SECTOR_SIZE) // 32
#define SECTORS_PER_FAT      ((2u * CLUSTER_COUNT + SECTOR_SIZE - 1u) / SECTOR_SIZE) // 128

#define MEDIA_TYPE           0xF8u   // non-removable

// Метка и содержимое INFO_UF2.TXT (как у bootloader)
static const char INFO_TXT[] =
    "UF2 Bootloader Emulator\r\n"
    "Model: RP2350\r\n"
    "Board-ID: RPI-RP2350\r\n";
#define INFO_TXT_LEN (sizeof(INFO_TXT)-1u)

static uint32_t disk_serial;

// Структура FAT directory entry (32 байта)
typedef struct __attribute__((packed)) {
    uint8_t  name[11];
    uint8_t  attr;
    uint8_t  reserved;
    uint8_t  ctime_frac;
    uint16_t ctime;
    uint16_t cdate;
    uint16_t adate;
    uint16_t cluster_hi;
    uint16_t mtime;
    uint16_t mdate;
    uint16_t cluster_lo;
    uint32_t size;
} DirEntry;

// Raspberry Pi timestamp: 2008-09-05 16:20:51
#define RPI_TIME ((16u<<11u)|(20u<<5u)|(51u>>1u))
#define RPI_DATE ((28u<<9u) |(9u<<5u) |(5u))

static void fill_sector(uint32_t lba, uint8_t *buf) {
    memset(buf, 0, SECTOR_SIZE);

    /* LBA 0 = MBR */
    if (lba == 0) {
        uint8_t *pt = buf + SECTOR_SIZE - 2 - 64;  // partition entry 1
        pt[4]  = 0x0Eu;                             // FAT16 LBA
        pt[8]  = 1; pt[9] = 0; pt[10] = 0; pt[11] = 0; // start LBA = 1
        uint32_t sc = SECTOR_COUNT - 1u;
        memcpy(pt + 12, &sc, 4);                    // sector count
        memcpy(buf + 0x1B8, &disk_serial, 4);
        buf[SECTOR_SIZE - 2] = 0x55;
        buf[SECTOR_SIZE - 1] = 0xAA;
        return;
    }
    lba--;  // теперь работаем в пространстве раздела (0 = boot sector)

    /* Boot sector (VBR) */
    if (lba == 0) {
        buf[0] = 0xEB; buf[1] = 0x3C; buf[2] = 0x90;   // jump boot
        memcpy(buf + 3, "MSWIN4.1", 8);                  // OEM
        buf[11] = 0x00; buf[12] = 0x02;                  // bytes per sector = 512
        buf[13] = (uint8_t)(CLUSTER_SIZE / SECTOR_SIZE); // sectors per cluster = 8
        buf[14] = 1; buf[15] = 0;                         // reserved sectors = 1
        buf[16] = FAT_COUNT;                              // FAT count = 2
        uint16_t rde = MAX_ROOT_DIR_ENTRIES;
        memcpy(buf + 17, &rde, 2);                        // root entries = 512
        buf[19] = 0; buf[20] = 0;                         // sector count (0 = use 32-bit)
        buf[21] = MEDIA_TYPE;
        uint16_t spf = SECTORS_PER_FAT;
        memcpy(buf + 22, &spf, 2);                        // sectors per FAT = 128
        buf[24] = 1; buf[25] = 0;                         // sectors per track
        buf[26] = 1; buf[27] = 0;                         // number of heads
        uint32_t hs = 1;
        memcpy(buf + 28, &hs, 4);                         // hidden sectors = 1 (MBR)
        uint32_t vsc = VOLUME_SECTOR_COUNT;
        memcpy(buf + 32, &vsc, 4);                        // large sector count
        buf[36] = 0x00;                                   // drive number
        buf[37] = 0x00;
        buf[38] = 0x29;                                   // ext boot sig
        memcpy(buf + 39, &disk_serial, 4);
        memcpy(buf + 43, "RPI-RP2350 ", 11);              // volume label (11 chars)
        memcpy(buf + 54, "FAT16   ", 8);
        buf[SECTOR_SIZE - 2] = 0x55;
        buf[SECTOR_SIZE - 1] = 0xAA;
        return;
    }
    lba--;  // пропускаем VBR

    /* FAT1 и FAT2 */
    if (lba < SECTORS_PER_FAT * FAT_COUNT) {
        if ((lba % SECTORS_PER_FAT) == 0) {
            uint16_t *p = (uint16_t *)buf;
            p[0] = 0xFF00u | MEDIA_TYPE;  // FAT ID
            p[1] = 0xFFFFu;               // cluster 1 reserved
            p[2] = 0xFFFFu;               // cluster 2 = INFO_TXT (EOF)
        }
        return;
    }
    lba -= SECTORS_PER_FAT * FAT_COUNT;

    /* Root directory */
    if (lba < ROOT_DIR_SECTORS) {
        if (lba == 0) {
            DirEntry *e = (DirEntry *)buf;
            // Запись 0: Volume label
            memcpy(e[0].name, "RPI-RP2350 ", 11);
            e[0].attr  = 0x08u | 0x20u;   // VOLUME_LABEL | ARCHIVE
            e[0].ctime = RPI_TIME; e[0].cdate = RPI_DATE;
            e[0].mtime = RPI_TIME; e[0].mdate = RPI_DATE;
            // Запись 1: INFO_UF2.TXT
            memcpy(e[1].name, "INFO_UF2TXT", 11);
            e[1].attr       = 0x01u | 0x20u;  // READONLY | ARCHIVE
            e[1].ctime_frac = 100;
            e[1].ctime = RPI_TIME; e[1].cdate = RPI_DATE;
            e[1].mtime = RPI_TIME; e[1].mdate = RPI_DATE;
            e[1].cluster_lo = 2;
            e[1].size       = INFO_TXT_LEN;
        }
        return;
    }
    lba -= ROOT_DIR_SECTORS;

    /* Область данных */
    uint32_t cluster = lba >> CLUSTER_SHIFT;
    uint32_t offset  = lba & ((1u << CLUSTER_SHIFT) - 1u);
    if (cluster == 0 && offset == 0) {
        // Кластер 2 → INFO_UF2.TXT
        uint32_t n = INFO_TXT_LEN < SECTOR_SIZE ? (uint32_t)INFO_TXT_LEN : SECTOR_SIZE;
        memcpy(buf, INFO_TXT, n);
    }
}

/* ─── TinyUSB MSC callbacks ─────────────────────────────────────── */

static bool     msc_ready    = true;
static uint32_t last_write_ms = 0;

void tud_msc_inquiry_cb(uint8_t lun,
                        uint8_t vendor_id[8],
                        uint8_t product_id[16],
                        uint8_t product_rev[4]) {
    (void)lun;
    memcpy(vendor_id,   "RPI     ", 8);
    memcpy(product_id,  "RP2350          ", 16);
    memcpy(product_rev, "2   ",             4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void)lun;
    return msc_ready;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
    (void)lun;
    *block_count = SECTOR_COUNT;
    *block_size  = SECTOR_SIZE;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition,
                            bool start, bool load_eject) {
    (void)lun; (void)power_condition; (void)start; (void)load_eject;
    return true;
}

static uint8_t sec_buf[SECTOR_SIZE];

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba,
                           uint32_t offset, void *buffer, uint32_t bufsize) {
    (void)lun;
    fill_sector(lba, sec_buf);
    memcpy(buffer, sec_buf + offset, bufsize);
    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba,
                            uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    (void)lun; (void)lba; (void)offset; (void)buffer;
    last_write_ms = millis();
    return (int32_t)bufsize;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16],
                         void *buffer, uint16_t bufsize) {
    (void)lun; (void)scsi_cmd; (void)buffer; (void)bufsize;
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}

/* ─── TinyUSB Device descriptor callbacks ───────────────────────── */

// Device descriptor — точная копия RP2350 bootloader
static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0210,   // USB 2.1 (нужен для BOS)
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x2E8A,   // Raspberry Pi
    .idProduct          = 0x000F,   // RP2350 Boot
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

const uint8_t *tud_descriptor_device_cb(void) {
    return (const uint8_t *)&desc_device;
}

// Configuration descriptor
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

static const uint8_t desc_config[] = {
    // Config: 1 interface, bus-powered, 50mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0x80, 50),
    // MSC interface 0, no string, EP OUT=0x02, EP IN=0x81, ep size=64
    TUD_MSC_DESCRIPTOR(0, 0, 0x02, 0x81, 64)
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_config;
}

// BOS descriptor с MS OS 2.0 (как у bootloader, non-composite)
#define MS_OS_20_DESC_LEN  0x9Eu
#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

static const uint8_t desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, 1)
};

const uint8_t *tud_descriptor_bos_cb(void) {
    return desc_bos;
}

// MS OS 2.0 descriptor set (non-composite, size=0x9E)
static const uint8_t ms_os20_desc[] = {
    0x0A,0x00, 0x00,0x00, 0x00,0x00,0x03,0x06, MS_OS_20_DESC_LEN,0x00,  // header
    0x14,0x00, 0x03,0x00,                                                  // compatible ID
    'W','I','N','U','S','B',0x00,0x00,                                     // WINUSB
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,                               // sub-compat
    0x80,0x00, 0x04,0x00, 0x01,0x00, 0x28,0x00,                           // reg property
    // DeviceInterfaceGUID name (40 bytes)
    'D',0,'e',0,'v',0,'i',0,'c',0,'e',0,'I',0,'n',0,'t',0,'e',0,
    'r',0,'f',0,'a',0,'c',0,'e',0,'G',0,'U',0,'I',0,'D',0,0,0,
    0x4E,0x00,                                                             // data size 78
    // {ecceff35-146c-4ff3-acd9-8f992d09acdd}
    '{',0,'e',0,'c',0,'c',0,'e',0,'f',0,'f',0,'3',0,'5',0,'-',0,
    '1',0,'4',0,'6',0,'c',0,'-',0,'4',0,'f',0,'f',0,'3',0,'-',0,
    'a',0,'c',0,'d',0,'9',0,'-',0,'8',0,'f',0,'9',0,'9',0,'2',0,
    'd',0,'0',0,'9',0,'a',0,'c',0,'d',0,'d',0,'}',0,0,0
};

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                 tusb_control_request_t const *req) {
    if (stage != CONTROL_STAGE_SETUP) return true;
    if (req->bRequest == 1 && req->wIndex == 7) {
        return tud_control_xfer(rhport, req, (void*)ms_os20_desc, sizeof(ms_os20_desc));
    }
    return false;
}

// String descriptors
static char serial_str[17];

static const uint16_t *make_str(const char *s, uint16_t *buf, uint8_t cap) {
    uint8_t len = strlen(s);
    if (len > cap - 1) len = cap - 1;
    buf[0] = (TUSB_DESC_STRING << 8) | (2 + 2*len);
    for (uint8_t i = 0; i < len; i++) buf[1+i] = s[i];
    return buf;
}

const uint16_t *tud_descriptor_string_cb(uint8_t idx, uint16_t langid) {
    (void)langid;
    static uint16_t buf[32];
    switch (idx) {
        case 0: buf[0] = (TUSB_DESC_STRING<<8)|4; buf[1] = 0x0409; return buf;
        case 1: return make_str("Raspberry Pi", buf, 32);
        case 2: return make_str("RP2350 Boot",  buf, 32);
        case 3: return make_str(serial_str,      buf, 32);
        default: return NULL;
    }
}

/* ─── USB mount callbacks ────────────────────────────────────────── */
static bool usb_mounted = false;
void tud_mount_cb(void)   { usb_mounted = true;  }
void tud_umount_cb(void)  { usb_mounted = false; }
void tud_suspend_cb(bool) {}
void tud_resume_cb(void)  {}

/* ─── setup / loop ──────────────────────────────────────────────── */

void setup() {
    led_init();
    led_set(C_WHITE);

    // Serial number из Chip ID
    uint8_t id[8];
    flash_get_unique_id(id);
    snprintf(serial_str, sizeof(serial_str),
             "%02X%02X%02X%02X%02X%02X%02X%02X",
             id[0],id[1],id[2],id[3],id[4],id[5],id[6],id[7]);
    memcpy(&disk_serial, id, 4);

    // Инициализация TinyUSB
    tusb_rhport_init_t dev_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO };
    tusb_init(0, &dev_init);
}

void loop() {
    tud_task();  // обслуживаем USB

    uint32_t now = millis();
    if (usb_mounted) {
        if (now - last_write_ms < 200) led_set(C_BLUE);
        else                           led_set(C_GREEN);
    } else {
        // Мигаем белым
        led_set((now / 500) & 1 ? C_WHITE : C_OFF);
    }
}
