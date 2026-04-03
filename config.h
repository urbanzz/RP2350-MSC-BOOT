#pragma once

// ── TCP сервер ───────────────────────────────────────────────────
#define SERVER_IP_BYTES   {10, 32, 232, 200}
#define SERVER_PORT       2000

// ── Сетевые настройки платы ──────────────────────────────────────
#define LOCAL_IP_BYTES    {10, 32, 232, 213}
#define LOCAL_MASK_BYTES  {255, 255, 254, 0}
#define LOCAL_GW_BYTES    {10, 32, 232, 1}
#define LOCAL_PORT        50000

// ── CH9120 UART/GPIO ─────────────────────────────────────────────
// GP20 = UART1 TX → CH9120 RXD
// GP21 = UART1 RX ← CH9120 TXD
// GP17 = TCPCS:  LOW = TCP установлен
// GP18 = CFG0:   LOW = режим конфигурации
// GP19 = RSTI:   LOW = аппаратный сброс
#define CH9120_UART         Serial2
#define CH9120_TX_PIN       20
#define CH9120_RX_PIN       21
#define CH9120_TCPCS_PIN    17
#define CH9120_CFG_PIN      18
#define CH9120_RST_PIN      19
#define CH9120_CFG_BAUD     9600UL
#define CH9120_DATA_BAUD    115200UL

// ── WS2812B RGB LED ──────────────────────────────────────────────
#define WS2812_PIN          25

// ── Таймауты ─────────────────────────────────────────────────────
// CH9120 держится в RESET до монтирования USB — не превышаем 100mA
#define USB_MOUNT_TIMEOUT_MS    30000
#define TCP_CONNECT_TIMEOUT_MS   5000

// ── Тайминги streaming ───────────────────────────────────────────
// Ждём после последней записи перед обработкой файла (мс)
#define WRITE_IDLE_MS           150
// Сброс диска после тишины в состоянии STREAMING (мс)
#define STREAM_IDLE_RESET_MS    10000UL

// ── Sparse sector pool ───────────────────────────────────────────
// Количество секторов от хоста, которые держим в RAM (каждый = 512 байт)
#define MAX_WRITTEN_SECTORS     128
