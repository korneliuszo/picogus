#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define SETTINGS_MAGIC 0x70677573  // "pgus" in ascii
#define SETTINGS_VERSION 2

// Settings struct has generous padding for future settings by aligning to 4 bytes
typedef struct Settings {
    uint32_t magic;  // should be "pgus" in ascii (0x70677573)
    uint8_t version;
    uint8_t startupMode;
    struct {
        uint8_t waveTableVolume;
    } Global;
    struct {
        uint16_t basePort;
    } Joy;
    struct {
        uint16_t basePort;
        uint8_t audioBuffer;
        uint8_t dmaInterval;
        bool force44k : 1;
    } GUS;
    struct {
        uint16_t basePort;
        uint16_t oplBasePort;
        bool oplSpeedSensitive : 1;
    } SB;
    struct {
        uint16_t basePort;
        bool delaySysex : 1;
        bool fakeAllNotesOff : 1;
    } MPU;
    struct {
        uint16_t basePort;
    } CMS;
    struct {
        uint16_t basePort;
    } Tandy;
    struct {
        uint16_t basePort;
        uint8_t protocol;
        uint8_t reportRate;
        int16_t sensitivity;
    } Mouse;
    struct {
        uint16_t basePort;
    } NE2K;
    struct {
        char ssid[33];
        char password[64];
    } WiFi;
} Settings;


void loadSettings(Settings* settings);
void saveSettings(const Settings* settings);
void getDefaultSettings(Settings* settings);

#ifdef __cplusplus
}
#endif
