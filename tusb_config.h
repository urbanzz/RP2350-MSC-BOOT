/*
 * TinyUSB config — MSC only, no CDC.
 * Размещён в каталоге скетча чтобы перекрыть дефолтный из Adafruit.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_TUSB_MCU        OPT_MCU_RP2040
#define CFG_TUSB_OS         OPT_OS_PICO
#define CFG_TUSB_DEBUG      0

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN  TU_ATTR_ALIGNED(4)

// Device mode
#define CFG_TUD_ENABLED     1
#define CFG_TUD_ENDPOINT0_SIZE 64

// Только MSC
#define CFG_TUD_CDC     0
#define CFG_TUD_MSC     1
#define CFG_TUD_HID     0
#define CFG_TUD_MIDI    0
#define CFG_TUD_VENDOR  0
#define CFG_TUD_VIDEO   0
#define CFG_TUD_VIDEO_STREAMING 0

// MSC transfer buffer (должен быть >= SECTOR_SIZE)
#define CFG_TUD_MSC_EP_BUFSIZE 512

// Host — отключён
#define CFG_TUH_ENABLED 0

#ifdef __cplusplus
}
#endif
