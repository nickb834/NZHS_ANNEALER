/*---------------------------------------------------------------------------*/
/*! @brief      Brass Cartridge Case Annealer.
  @details      None.
  @author       Justin Spence, Mark Griffith. 2020
  @note         circuitworksnz@gmail.com
*//*-------------------------------------------------------------------------*/

//--Includes-------------------------------------------------------------------
#include <SPI.h>
#include <Wire.h>
#include "AnnealerPlatform.h"
#if NZHS_HAS_WIFI
#include "WebAssets.h"
#endif
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//-- macros---------------------------------------------------------------
//#define DEBUG //removes the splash screen and enables serial reset diagnostics at 115200 baud
#define SERVO
//                          Major Version
//                          | Minor Version
//                          | | LCD Type
//                          | | |
//                          | | |
//                          | | |
#if NZHS_HAS_WIFI
#define SOFTWARE_VERSION F("4.2.0")
#else
#define SOFTWARE_VERSION F("4.1.0")
#endif
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define PSU_OVERCURRENT 12300 //12.3A
#define CURRENT_SENSOR_SCALE 49  //Choose the current scaling factor to suit your sensor, default is for 20A device. ACS712-20A = 49, ACS712-30A = 74
#define TEMP_RESOLUTION 9 //ADC resolution on temp sensor
#define TEMP_LIMIT 55 //capacitor temperature limit degC
#define TEMP_CONVERSION_TIME 120 //measurement time for DS18B20 9,10,11,12 bit = 95ms, 190ms, 375ms, 750ms
#define TEMP_HYSTERESIS 15 //define how much temperature needs to drop to resume
#define TEMP_RAW_SCALE 128 //DallasTemperature raw readings are in 1/128 degC units
#define TEMP_SENSOR_MIN_RAW (-55 * TEMP_RAW_SCALE) //DS18B20 lower measurement limit
#define TEMP_SENSOR_MAX_RAW (125 * TEMP_RAW_SCALE) //DS18B20 upper measurement limit
#define DROP_TIME 500 //time to drop the case in ms
#define RELOAD_TIME 5000 //time for user to load a new case in free run mode (ms)
#define RELOAD_TIME_AUTO__FEED 2000 //time to feed case in auto feed mode (ms) - recommend leaving at 2000
#define MIN_ANNEAL_TIME 2000 //min anneal time in ms
#define MAX_ANNEAL_TIME 8000 //max anneal time in ms
#define LONG_PRESS_HOLD_TIME 15 //main-loop iterations before UP resets the selected time to 2.0 seconds
#define RAPID_TIME_PRESS_COUNT 5 //consecutive UP presses before changing in 0.5 second steps
#define RAPID_TIME_PRESS_INTERVAL 1000 //maximum milliseconds between rapid UP presses
#define RAPID_TIME_INCREMENT 500 //milliseconds added by rapid time adjustment
#define PROFILE_NAME_REPEAT_DELAY 1000 //hold UP for this long before profile-name characters begin repeating
#define PROFILE_NAME_REPEAT_PERIOD 150 //milliseconds between repeated profile-name characters
#define ANALYSIS_DURATION_MS 8000UL
#define ANALYSIS_SAMPLE_PERIOD_MS 25UL
#define ANALYSIS_GRAPH_COLUMNS 128
#define ANALYSIS_GRAPH_REFRESH_MS 100UL
#define ANALYSIS_GATE_OPEN_PERIOD_MS 5000UL
#define ANALYSIS_DUMP_STATUS_MS 1000UL
#define ANALYSIS_ABORT_HOLD_MS 300UL
#define ANALYSIS_SUPPLY_VOLTAGE_V 48UL
#define ANALYSIS_GRAPH_MAX_CURRENT_MA 12500UL
#define ANALYSIS_GRAPH_CURRENT_STEP_MA 50U
#define ANALYSIS_GRAPH_MAX_SAMPLE (ANALYSIS_GRAPH_MAX_CURRENT_MA / ANALYSIS_GRAPH_CURRENT_STEP_MA)
#define ANALYSIS_PEAK_CONFIRM_SAMPLES 3
#define ANALYSIS_DEFAULT_PEAK_DROP_PERCENT 10
#define ANALYSIS_MAX_ENERGY_J 9999
#define LOW_CURRENT_IGNORED_CYCLES 1 //first anneal cycle is ignored while the system settles
#define LOW_CURRENT_BASELINE_CYCLES 5 //accepted normal cycles retained in the moving baseline window
#define LOW_CURRENT_CONSECUTIVE_CYCLES 1 //low-current cycles that trigger a fault
#define LOW_CURRENT_RATIO_PERCENT 85 //a cycle below this percentage of the baseline is considered low current
#define CURRENT_SENSOR_DETECTION_MA 100 //minimum anneal-cycle average that verifies the fitted current sensor
#define RESET_DIAGNOSTIC_MAGIC 0x5A
#define INTERNAL_BANDGAP_MV 1100 //nominal ATmega328P band-gap voltage; calibrate if absolute accuracy is required
#if NZHS_HAS_WIFI
#define INFO_SCREEN_SCROLL_COUNT 5
#else
#define INFO_SCREEN_SCROLL_COUNT 3
#endif
#define SUPPLY_VOLTAGE_SAMPLE_PERIOD 1000 //refresh the Info-screen AVcc reading once per second
#define LOOP_TIME 120  //ms per main loop iteration
#define COOLDOWN_PERIOD 300000 //Cooling period in milliseconds
#define DISPLAY_ADDRESS 0x3C
#define SERVO_OPEN_POSITION 5  //timer load value for servo pulse. 128us per timer count. 7 => 0.89ms pulse
#define SERVO_CLOSE_POSITION 15 // 15 => 1.92ms pulse
#define STEPPER_SCALING_FACTOR 1 //1.28 //Used to compensate for BIGTREETECH controllers needing 256 steps per rev
#define STEPPER_STEPS_PER_TURN 200*STEPPER_SCALING_FACTOR*STEPPER_MICROSTEPS // stepper motor steps per revolution (e.g. 200 step motor) * microsteps.
#define STEPPER_MICROSTEPS 16 // number of microsteps. set to 1 if no microstepping
#define CASE_FEEDER_STEPS_DROP_TO_PRELOAD 185*STEPPER_MICROSTEPS*STEPPER_SCALING_FACTOR
#define CASE_FEEDER_STEPS_PRELOAD_TO_DROP (STEPPER_STEPS_PER_TURN - CASE_FEEDER_STEPS_DROP_TO_PRELOAD + 1)
#define CASE_FEEDER_HOPPER_START 70*STEPPER_MICROSTEPS*STEPPER_SCALING_FACTOR
#define CASE_FEEDER_HOPPER_END 130*STEPPER_MICROSTEPS*STEPPER_SCALING_FACTOR

#define SHOW_CASE_COUNT //This enables the display of the total number of cases annealed since powerup. Comment this out if you don't want to see the cases annealed counter.

// temp sensor pin asignment DS1820
#define ONE_WIRE_BUS 8

#define EEPROM_ADDRESS_ANNEAL_TIME 0
#define EEPROM_ADDRESS_AUTO_RESTART 1
#define EEPROM_ADDRESS_CONFIG_MAGIC 2
#define EEPROM_CONFIG_MAGIC 0xA4
#define EEPROM_ADDRESS_DUMP_BUTTON 3
#define EEPROM_ADDRESS_DUMP_BUTTON_MAGIC 4
#define EEPROM_DUMP_BUTTON_MAGIC 0x3C
#define EEPROM_ADDRESS_PROFILE_BASE 16
#define EEPROM_ADDRESS_PROFILE_RULE_BASE 160
#define EEPROM_ADDRESS_PROFILE_REFERENCE_BASE 192
#define PROFILE_COUNT 8
#define PROFILE_NAME_LENGTH 10
#define PROFILE_MAGIC 0xC7
#define PROFILE_FLAG_AUTO_RESTART 0x01
#define PROFILE_FLAG_DUMP_BUTTON 0x02
#define PROFILE_FLAG_STOP_TYPE_SHIFT 2
#define PROFILE_FLAG_STOP_TYPE_MASK 0x0C
#define PROFILE_REFERENCE_SAMPLE_COUNT 64
#define PROFILE_REFERENCE_MAGIC 0xD6
#define PROFILE_REFERENCE_SAMPLE_OFFSET 1
#define PROFILE_REFERENCE_PEAK_OFFSET 65
#define PROFILE_REFERENCE_ENERGY_OFFSET 67
#define PROFILE_REFERENCE_DURATION_OFFSET 69
#define PROFILE_REFERENCE_CHECKSUM_OFFSET 71
#define PROFILE_REFERENCE_RECORD_SIZE 72
#define ARDUINO_UNO_EEPROM_SIZE 1024
#if NZHS_HAS_WIFI
#define EEPROM_ADDRESS_WIFI_CONFIG 768
#define WIFI_CONFIG_MAGIC 0x57
#define WIFI_CONFIG_VERSION 1
#define WIFI_SSID_MAX_LENGTH 32
#define WIFI_PASSWORD_MAX_LENGTH 63
#define WIFI_CONNECT_TIMEOUT_MS 15000UL
#define WIFI_HTTP_REQUEST_MAX_LENGTH 1536
#define WIFI_HISTORY_RECORD_COUNT 16
#define WIFI_HISTORY_NO_PROFILE UINT8_MAX
#define WIFI_HISTORY_FLAG_ANALYSIS 0x01
#define WIFI_HISTORY_FLAG_MATCH 0x02
#endif
#define PROFILE_NOTICE_PERIOD 1000
#define RIGHT_PANEL_X 56
#define ANALYSIS_ENERGY_X 92

// Monochrome drawing modes retained from the Adafruit SSD1306 API so the
// existing screen code remains unchanged.
#define BLACK 0
#define WHITE 1
#define SSD1306_INVERSE 2

#ifndef FPSTR
#define FPSTR(address) (reinterpret_cast<const __FlashStringHelper *>(address))
#endif

// Reuse common labels rather than emitting a separate flash string at every
// display call site.
static const char TEXT_ANALYSE_ITEM[] PROGMEM = "ANALYSE >";
static const char TEXT_SETTINGS_ITEM[] PROGMEM = "SETTINGS >";
static const char TEXT_PROFILES[] PROGMEM = "PROFILES";
static const char TEXT_PROFILES_ITEM[] PROGMEM = "PROFILES >";
static const char TEXT_INFO_ITEM[] PROGMEM = "INFO >";
static const char TEXT_DIAGNOSTICS_ITEM[] PROGMEM = "DIAGNOSTICS>";
static const char TEXT_BACK_ITEM[] PROGMEM = "BACK >";
static const char TEXT_SAVE_ITEM[] PROGMEM = "SAVE >";
static const char TEXT_LOAD_ITEM[] PROGMEM = "LOAD >";
static const char TEXT_PERFORMANCE_ITEM[] PROGMEM = "PERFORMANCE >";
static const char TEXT_RENAME_ITEM[] PROGMEM = "RENAME >";
static const char TEXT_DELETE_ITEM[] PROGMEM = "DELETE >";
static const char TEXT_NEW_ITEM[] PROGMEM = "NEW >";
static const char TEXT_REVIEW_ITEM[] PROGMEM = "REVIEW >";
static const char TEXT_CONFIG_ITEM[] PROGMEM = "CONFIG >";
static const char TEXT_ON[] PROGMEM = "ON";
static const char TEXT_OFF[] PROGMEM = "OFF";
static const char TEXT_INPUT_ENERGY[] PROGMEM = ",input_energy_J=";
static const char * const PROFILE_ACTION_TEXT[] PROGMEM = {
  TEXT_LOAD_ITEM, TEXT_SAVE_ITEM, TEXT_RENAME_ITEM, TEXT_DELETE_ITEM,
  TEXT_PERFORMANCE_ITEM, TEXT_BACK_ITEM
};
static const char * const STOPPED_MENU_TEXT[] PROGMEM = {
  TEXT_SETTINGS_ITEM, TEXT_PROFILES_ITEM, TEXT_ANALYSE_ITEM, TEXT_INFO_ITEM,
  TEXT_DIAGNOSTICS_ITEM
};

// Exact glyph columns from the Adafruit GFX classic fixed-space font. The
// firmware only emits ASCII 32-95, lowercase 's', and its historical degree
// character (input byte 248 maps to CP437 glyph 249). Each glyph remains the
// original five columns by eight rows; the renderer supplies column six.
static const uint8_t annealerFont[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00,
  0x14, 0x7F, 0x14, 0x7F, 0x14, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62,
  0x36, 0x49, 0x56, 0x20, 0x50, 0x00, 0x08, 0x07, 0x03, 0x00, 0x00, 0x1C, 0x22, 0x41, 0x00,
  0x00, 0x41, 0x22, 0x1C, 0x00, 0x2A, 0x1C, 0x7F, 0x1C, 0x2A, 0x08, 0x08, 0x3E, 0x08, 0x08,
  0x00, 0x80, 0x70, 0x30, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x60, 0x60, 0x00,
  0x20, 0x10, 0x08, 0x04, 0x02, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00,
  0x72, 0x49, 0x49, 0x49, 0x46, 0x21, 0x41, 0x49, 0x4D, 0x33, 0x18, 0x14, 0x12, 0x7F, 0x10,
  0x27, 0x45, 0x45, 0x45, 0x39, 0x3C, 0x4A, 0x49, 0x49, 0x31, 0x41, 0x21, 0x11, 0x09, 0x07,
  0x36, 0x49, 0x49, 0x49, 0x36, 0x46, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00, 0x14, 0x00, 0x00,
  0x00, 0x40, 0x34, 0x00, 0x00, 0x00, 0x08, 0x14, 0x22, 0x41, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x00, 0x41, 0x22, 0x14, 0x08, 0x02, 0x01, 0x59, 0x09, 0x06, 0x3E, 0x41, 0x5D, 0x59, 0x4E,
  0x7C, 0x12, 0x11, 0x12, 0x7C, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E, 0x41, 0x41, 0x41, 0x22,
  0x7F, 0x41, 0x41, 0x41, 0x3E, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x7F, 0x09, 0x09, 0x09, 0x01,
  0x3E, 0x41, 0x41, 0x51, 0x73, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x41, 0x7F, 0x41, 0x00,
  0x20, 0x40, 0x41, 0x3F, 0x01, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x7F, 0x40, 0x40, 0x40, 0x40,
  0x7F, 0x02, 0x1C, 0x02, 0x7F, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x3E, 0x41, 0x41, 0x41, 0x3E,
  0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x7F, 0x09, 0x19, 0x29, 0x46,
  0x26, 0x49, 0x49, 0x49, 0x32, 0x03, 0x01, 0x7F, 0x01, 0x03, 0x3F, 0x40, 0x40, 0x40, 0x3F,
  0x1F, 0x20, 0x40, 0x20, 0x1F, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x63, 0x14, 0x08, 0x14, 0x63,
  0x03, 0x04, 0x78, 0x04, 0x03, 0x61, 0x59, 0x49, 0x4D, 0x43, 0x00, 0x7F, 0x41, 0x41, 0x41,
  0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x41, 0x41, 0x41, 0x7F, 0x04, 0x02, 0x01, 0x02, 0x04,
  0x40, 0x40, 0x40, 0x40, 0x40,
  // Lowercase 's'.
  0x48, 0x54, 0x54, 0x54, 0x24,
  // Degree symbol as rendered by Adafruit GFX for input byte 248.
  0x00, 0x00, 0x18, 0x18, 0x00,
};

// Minimal fixed 128x32 I2C display implementation. It deliberately preserves
// the subset of the Adafruit GFX/SSD1306 API used by this sketch while omitting
// unused panel sizes, SPI, rotation, scrolling, proportional fonts and generic
// drawing primitives.
class AnnealerDisplay : public Print
{
public:
  AnnealerDisplay(uint8_t width, uint8_t height, TwoWire *twi, int8_t resetPin,
                  uint32_t activeClock, uint32_t idleClock)
    : wire(twi), wireClk(activeClock), restoreClk(idleClock), cursor_x(0),
      cursor_y(0), textsize(1), textcolor(WHITE), textbgcolor(WHITE), wrap(true)
  {
    (void)width;
    (void)height;
    (void)resetPin;
  }

  bool beginFixed128x32I2C(void)
  {
    clearDisplay();
    wire->begin();
#ifdef WIRE_HAS_TIMEOUT
    wire->setWireTimeout(25000, true);
#endif
    wire->setClock(wireClk);
    static const uint8_t PROGMEM commands[] = {
      0xAE,             // display off
      0xD5, 0x80,       // clock divide
      0xA8, 0x1F,       // 32 multiplex rows
      0xD3, 0x00,       // display offset
      0x40,             // start line
      0x8D, 0x14,       // charge pump
      0x20, 0x00,       // horizontal memory mode
      0xA1,             // segment remap
      0xC8,             // COM scan direction
      0xDA, 0x02,       // COM pins for 128x32
      0x81, 0x8F,       // contrast
      0xD9, 0xF1,       // precharge
      0xDB, 0x40,       // VCOM detect
      0xA4,             // display follows RAM
      0xA6,             // normal display
      0x2E,             // deactivate scrolling
      0xAF              // display on
    };
    sendCommandList(commands, sizeof(commands));
    wire->setClock(restoreClk);
    return true;
  }

  void clearDisplay(void)
  {
    memset(framebuffer, 0, sizeof(framebuffer));
  }

  void display(void)
  {
    static const uint8_t PROGMEM addressCommands[] = {
      0x21, 0x00, 0x7F, // columns 0-127
      0x22, 0x00, 0x03  // pages 0-3
    };
    wire->setClock(wireClk);
    sendCommandList(addressCommands, sizeof(addressCommands));
    uint16_t offset = 0;
    while(offset < sizeof(framebuffer))
    {
      wire->beginTransmission(DISPLAY_ADDRESS);
      wire->write(0x40); // data stream
      uint8_t count = 0;
      // Wire's 32-byte AVR buffer leaves 31 bytes after the data-control byte.
      while(count < (BUFFER_LENGTH - 1) && offset < sizeof(framebuffer))
      {
        wire->write(framebuffer[offset++]);
        count++;
      }
      wire->endTransmission();
    }
    wire->setClock(restoreClk);
  }

  void invertDisplay(bool invert)
  {
    wire->setClock(wireClk);
    wire->beginTransmission(DISPLAY_ADDRESS);
    wire->write(0x00);
    wire->write(invert ? 0xA7 : 0xA6);
    wire->endTransmission();
    wire->setClock(restoreClk);
  }

  void drawPixel(int16_t x, int16_t y, uint16_t color)
  {
    if(x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT)
    {
      return;
    }
    uint8_t * const pixel = &framebuffer[x + (y >> 3) * SCREEN_WIDTH];
    uint8_t const mask = 1 << (y & 7);
    if(color == WHITE)
    {
      *pixel |= mask;
    }
    else if(color == BLACK)
    {
      *pixel &= ~mask;
    }
    else
    {
      *pixel ^= mask;
    }
  }

  void drawFastVLine(int16_t x, int16_t y, int16_t height, uint16_t color)
  {
    while(height-- > 0)
    {
      drawPixel(x, y++, color);
    }
  }

  void fillRect(int16_t x, int16_t y, int16_t width, int16_t height,
                uint16_t color)
  {
    for(int16_t column = 0; column < width; column++)
    {
      drawFastVLine(x + column, y, height, color);
    }
  }

  void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap,
                  int16_t width, int16_t height, uint16_t color)
  {
    uint8_t const byteWidth = (width + 7) / 8;
    for(int16_t row = 0; row < height; row++)
    {
      for(int16_t column = 0; column < width; column++)
      {
        uint8_t const bits = pgm_read_byte(bitmap + row * byteWidth + (column >> 3));
        if(bits & (0x80 >> (column & 7)))
        {
          drawPixel(x + column, y + row, color);
        }
      }
    }
  }

  void setCursor(int16_t x, int16_t y)
  {
    cursor_x = x;
    cursor_y = y;
  }

  void setTextSize(uint8_t size)
  {
    textsize = size ? size : 1;
  }

  void setTextColor(uint16_t color)
  {
    textcolor = color;
    textbgcolor = color;
  }

  void setTextColor(uint16_t color, uint16_t background)
  {
    textcolor = color;
    textbgcolor = background;
  }

  using Print::write;
  size_t write(uint8_t character) override
  {
    if(character == '\n')
    {
      cursor_x = 0;
      cursor_y += textsize * 8;
    }
    else if(character != '\r')
    {
      if(wrap && cursor_x + textsize * 6 > SCREEN_WIDTH)
      {
        cursor_x = 0;
        cursor_y += textsize * 8;
      }
      drawCharacter(cursor_x, cursor_y, character);
      cursor_x += textsize * 6;
    }
    return 1;
  }

private:
  void sendCommandList(const uint8_t *commands, uint8_t count)
  {
    wire->beginTransmission(DISPLAY_ADDRESS);
    wire->write(0x00);
    while(count--)
    {
      wire->write(pgm_read_byte(commands++));
    }
    wire->endTransmission();
  }

  uint16_t glyphOffset(uint8_t character) const
  {
    if(character >= 32 && character <= 95)
    {
      return (uint16_t)(character - 32) * 5;
    }
    if(character == 's')
    {
      return 320;
    }
    if(character == 248)
    {
      return 325;
    }
    return (uint16_t)('?' - 32) * 5;
  }

  void drawCharacter(int16_t x, int16_t y, uint8_t character)
  {
    if(x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT ||
       x + 6 * textsize - 1 < 0 || y + 8 * textsize - 1 < 0)
    {
      return;
    }

    uint16_t const offset = glyphOffset(character);
    for(uint8_t column = 0; column < 5; column++)
    {
      uint8_t bits = pgm_read_byte(&annealerFont[offset + column]);
      for(uint8_t row = 0; row < 8; row++, bits >>= 1)
      {
        uint16_t const color = bits & 1 ? textcolor : textbgcolor;
        if((bits & 1) || textbgcolor != textcolor)
        {
          if(textsize == 1)
          {
            drawPixel(x + column, y + row, color);
          }
          else
          {
            fillRect(x + column * textsize, y + row * textsize,
                     textsize, textsize, color);
          }
        }
      }
    }
    if(textbgcolor != textcolor)
    {
      fillRect(x + 5 * textsize, y, textsize, 8 * textsize, textbgcolor);
    }
  }

  TwoWire *wire;
  uint32_t wireClk;
  uint32_t restoreClk;
  int16_t cursor_x;
  int16_t cursor_y;
  uint8_t textsize;
  uint8_t textcolor;
  uint8_t textbgcolor;
  bool wrap;
  uint8_t framebuffer[SCREEN_WIDTH * ((SCREEN_HEIGHT + 7) / 8)];
};

AnnealerDisplay display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1, 200000, 200000);
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Global variables :(
DeviceAddress tempDeviceAddress;
// Cached at setup for the stopped-screen temperature display.
static uint8_t NumberDallasTempDevices = 0;
static bool CurrentSensorPresent = 0;
static uint16_t psuCurrentZeroOffset = 0;
static volatile uint16_t StepsToGo = 0;
static volatile uint16_t StepsFromHome = 0;
static volatile bool StepToggle = 0;
static volatile uint32_t SystemTimeTarget;

//--define state machine states-----------------------------------------------------------
typedef enum tStateMachineStates : uint8_t
{
  STATE_STOPPED = 0,
  STATE_PRELOAD,
  STATE_ANNEALING,
  STATE_DROPPING,
  STATE_RELOADING,
  STATE_COOLDOWN,
  STATE_JUST_BOOTED,
  STATE_SHOW_WARNING,
  STATE_SETTINGS,
  STATE_PROFILES,
  STATE_PROFILE_ACTIONS,
  STATE_PROFILE_PERFORMANCE,
  STATE_PROFILE_NAME_EDIT,
  STATE_PROFILE_DELETE_CONFIRM,
  STATE_PROFILE_NOTICE,
  STATE_ANALYSIS_LOAD,
  STATE_ANALYSING,
  STATE_ANALYSIS_GATE_OPEN,
  STATE_ANALYSIS_RESULT,
  STATE_DIAGNOSTICS,
  STATE_INFO,
  STATE_OVERCURRENT_WARNING,
  STATE_LOW_CURRENT_WARNING,
  STATE_TEMPERATURE_SENSOR_WARNING,
  STATE_ANALYSIS_MENU,
  STATE_ANALYSIS_CONFIG,
  STATE_TARGET_TIMEOUT_WARNING,
  STATE_CURRENT_SENSOR_REQUIRED,
  #if NZHS_HAS_WIFI
  STATE_WIFI_SETTINGS,
  STATE_WIFI_RESET_CONFIRM,
  #endif
  #if NZHS_PLATFORM_UNO_R4
  STATE_PLATFORM_WARNING,
  #endif
  STATE_UNKNOWN,
} tStateMachineStates;

typedef enum ModeList : uint8_t
{
  MODE_SINGLE_SHOT = 0,
  MODE_FREE_RUN,
  MODE_AUTOMATIC,
} ModeList;

typedef enum tStoppedScreenSelection : uint8_t
{
  STOPPED_SCREEN_TIME = 0,
  STOPPED_SCREEN_MODE,
  STOPPED_SCREEN_SETTINGS,
  STOPPED_SCREEN_PROFILES,
  STOPPED_SCREEN_ANALYSE,
  STOPPED_SCREEN_INFO,
  STOPPED_SCREEN_DIAGNOSTICS,
  STOPPED_SCREEN_SELECTION_COUNT,
} tStoppedScreenSelection;

typedef enum tSettingsScreenSelection : uint8_t
{
  SETTINGS_SCREEN_AUTO_RESTART = 0,
  SETTINGS_SCREEN_DUMP_BUTTON,
  #if NZHS_HAS_WIFI
  SETTINGS_SCREEN_WIFI,
  #endif
  SETTINGS_SCREEN_BACK,
  SETTINGS_SCREEN_SELECTION_COUNT,
} tSettingsScreenSelection;

#if NZHS_HAS_WIFI
typedef enum tWifiSettingsSelection : uint8_t
{
  WIFI_SETTINGS_MONITOR = 0,
  WIFI_SETTINGS_SETUP,
  WIFI_SETTINGS_RESET,
  WIFI_SETTINGS_BACK,
  WIFI_SETTINGS_SELECTION_COUNT,
} tWifiSettingsSelection;

typedef enum tWifiResetSelection : uint8_t
{
  WIFI_RESET_CONFIRM = 0,
  WIFI_RESET_BACK,
  WIFI_RESET_SELECTION_COUNT,
} tWifiResetSelection;

typedef enum tR4WifiMode : uint8_t
{
  R4_WIFI_OFF = 0,
  R4_WIFI_STATION_CONNECTING,
  R4_WIFI_STATION_MONITOR,
  R4_WIFI_DIRECT_MONITOR,
  R4_WIFI_SETUP_AP,
  R4_WIFI_ERROR,
} tR4WifiMode;

typedef enum tWifiHistoryReason : uint8_t
{
  WIFI_HISTORY_ANALYSE = 0,
  WIFI_HISTORY_USER_ABORT,
  WIFI_HISTORY_TIME,
  WIFI_HISTORY_ENERGY,
  WIFI_HISTORY_PEAK_DROP,
  WIFI_HISTORY_TIMEOUT,
  WIFI_HISTORY_OVERCURRENT,
  WIFI_HISTORY_LOW_CURRENT,
  WIFI_HISTORY_TEMP_ERROR,
} tWifiHistoryReason;
#endif

typedef enum tAnalysisMenuSelection : uint8_t
{
  ANALYSIS_MENU_NEW = 0,
  ANALYSIS_MENU_REVIEW,
  ANALYSIS_MENU_CONFIG,
  ANALYSIS_MENU_BACK,
  ANALYSIS_MENU_SELECTION_COUNT,
} tAnalysisMenuSelection;

typedef enum tProfileStopType : uint8_t
{
  PROFILE_STOP_TIME = 0,
  PROFILE_STOP_ENERGY,
  PROFILE_STOP_PEAK_DROP,
  PROFILE_STOP_TYPE_COUNT,
} tProfileStopType;

typedef enum tAnalysisConfigSelection : uint8_t
{
  ANALYSIS_CONFIG_TYPE = 0,
  ANALYSIS_CONFIG_TARGET,
  ANALYSIS_CONFIG_ENERGY_DIGIT_2,
  ANALYSIS_CONFIG_ENERGY_DIGIT_3,
  ANALYSIS_CONFIG_ENERGY_DIGIT_4,
  ANALYSIS_CONFIG_MAX_TIME,
  ANALYSIS_CONFIG_PROFILE,
  ANALYSIS_CONFIG_SAVE,
  ANALYSIS_CONFIG_BACK,
  ANALYSIS_CONFIG_SELECTION_COUNT,
} tAnalysisConfigSelection;

typedef enum tProfileActionSelection : uint8_t
{
  PROFILE_ACTION_LOAD = 0,
  PROFILE_ACTION_SAVE,
  PROFILE_ACTION_RENAME,
  PROFILE_ACTION_DELETE,
  PROFILE_ACTION_PERFORMANCE,
  PROFILE_ACTION_BACK,
  PROFILE_ACTION_SELECTION_COUNT,
} tProfileActionSelection;

typedef enum tProfileNotice : uint8_t
{
  PROFILE_NOTICE_SAVED = 0,
  PROFILE_NOTICE_LOADED,
  PROFILE_NOTICE_DELETED,
  PROFILE_NOTICE_EMPTY,
  PROFILE_NOTICE_NO_DATA,
} tProfileNotice;

typedef enum tPerformanceFooter : uint8_t
{
  PERFORMANCE_FOOTER_LIVE = 0,
  PERFORMANCE_FOOTER_REVIEW,
  PERFORMANCE_FOOTER_DROP,
  PERFORMANCE_FOOTER_NEXT,
} tPerformanceFooter;

typedef struct tUserSettings
{
  uint16_t annealTime_ms;
  bool autoRestartAfterCooldown;
  bool dumpButtonEnabled;
  tStoppedScreenSelection stoppedScreenSelection;
  tSettingsScreenSelection settingsScreenSelection;
  tAnalysisMenuSelection analysisMenuSelection;
  uint8_t profileSlot;
  tProfileActionSelection profileActionSelection;
  uint8_t profileNameCursor;
  bool profileDeleteConfirmed;
  tProfileStopType stopType;
  uint16_t targetEnergy_J;
  uint8_t peakDropPercent;
} tUserSettings;

typedef struct __attribute__((packed)) tCartridgeProfile
{
  uint8_t magic;
  char name[PROFILE_NAME_LENGTH];
  uint16_t annealTime_ms;
  uint8_t mode;
  uint8_t flags;
  uint8_t checksum;
} tCartridgeProfile;

#if NZHS_HAS_WIFI
typedef struct __attribute__((packed)) tR4WifiConfig
{
  uint8_t magic;
  uint8_t version;
  uint8_t monitorEnabled;
  char ssid[WIFI_SSID_MAX_LENGTH + 1];
  char password[WIFI_PASSWORD_MAX_LENGTH + 1];
  uint8_t checksum;
} tR4WifiConfig;

typedef struct tWifiHistoryRecord
{
  uint32_t completedAt_ms;
  uint32_t inputEnergy_mJ;
  uint16_t id;
  uint16_t elapsedTime_ms;
  uint16_t peakCurrent_ma;
  uint16_t energyPercent;
  uint8_t matchPercent;
  uint8_t reason;
  uint8_t profileSlot;
  uint8_t flags;
  uint8_t graphCount;
  uint8_t graphSamples[ANALYSIS_GRAPH_COLUMNS];
} tWifiHistoryRecord;
#endif

static_assert(EEPROM_ADDRESS_PROFILE_BASE + (PROFILE_COUNT * sizeof(tCartridgeProfile)) <=
              EEPROM_ADDRESS_PROFILE_RULE_BASE,
              "Profile stop rules overlap cartridge profiles");
static_assert(EEPROM_ADDRESS_PROFILE_RULE_BASE + (PROFILE_COUNT * 3) <=
              EEPROM_ADDRESS_PROFILE_REFERENCE_BASE,
              "Profile references overlap profile stop rules");
static_assert(EEPROM_ADDRESS_PROFILE_REFERENCE_BASE +
              (PROFILE_COUNT * PROFILE_REFERENCE_RECORD_SIZE) <= ARDUINO_UNO_EEPROM_SIZE,
              "Profile references exceed Arduino Uno EEPROM");
#if NZHS_HAS_WIFI
static_assert(EEPROM_ADDRESS_WIFI_CONFIG >=
              EEPROM_ADDRESS_PROFILE_REFERENCE_BASE +
              (PROFILE_COUNT * PROFILE_REFERENCE_RECORD_SIZE),
              "WiFi configuration overlaps profile references");
static_assert(EEPROM_ADDRESS_WIFI_CONFIG + sizeof(tR4WifiConfig) <= ARDUINO_UNO_EEPROM_SIZE,
              "WiFi configuration exceeds EEPROM capacity");
#endif

typedef struct tRunSafetyState
{
  bool cooldownRestartPending;
  bool cooldownLockActive;
  uint32_t annealingCurrentTotal_ma;
  uint16_t annealingCurrentSamples;
  uint32_t restartCurrentTotal_ma;
  uint16_t restartCurrentSamples;
  uint8_t completedAnnealCycles;
  uint16_t baselineCurrent_ma;
  uint16_t baselineCurrentWindow_ma[LOW_CURRENT_BASELINE_CYCLES];
  uint8_t ignoredCurrentCycles;
  uint8_t baselineCurrentCycles;
  uint8_t baselineCurrentWindowIndex;
  uint8_t lowCurrentConsecutiveCycles;
} tRunSafetyState;

typedef struct tAnalysisState
{
  uint32_t startTime;
  uint32_t nextSampleTime;
  uint32_t lastSampleTime;
  uint32_t lastGraphDrawTime;
  uint32_t inputEnergy_mJ;
  uint16_t peakCurrent_ma;
  uint16_t elapsedTime_ms;
  uint8_t graphColumn;
  uint16_t graphCurrentTotal_ma;
  uint16_t graphCurrent_ma;
  uint8_t graphCurrentSamples;
  uint8_t graphSamples[ANALYSIS_GRAPH_COLUMNS];
  bool graphValid;
  bool graphIsAnalysis;
} tAnalysisState;

typedef struct tAnalysisConfigState
{
  tProfileStopType stopType;
  uint16_t stopTime_ms;
  uint16_t maxTime_ms;
  uint8_t energyDigits[4];
  uint8_t peakDropPercent;
  uint8_t selection;
  uint8_t profileSlot;
  bool profileSaveInProgress;
} tAnalysisConfigState;

typedef struct tAdaptiveAnnealState
{
  uint32_t energyCurrentSumTarget;
  uint16_t peakCurrent_ma;
  uint8_t belowPeakSamples;
} tAdaptiveAnnealState;

typedef struct tCasePerformanceState
{
  uint32_t errorTotal;
  uint32_t referenceTotal;
  uint32_t referenceEnergy_mJ;
  uint16_t referencePeakCurrent_ma;
  uint16_t energyPercent;
  uint16_t peakPercent;
  uint8_t referenceSlot;
  uint8_t resultSlot;
  uint8_t matchPercent;
  bool referenceValid;
  bool currentCycleCompared;
} tCasePerformanceState;

typedef struct tResetDiagnostics
{
  uint8_t magic;
  uint8_t resetFlags;
  uint8_t lastSystemState;
  uint8_t previousSystemState;
} tResetDiagnostics;

//--global constant declarations-----------------------------------------
static const uint8_t g_StartStopButtonPin   = 2;
static const uint8_t g_ModeButtonPin        = 3;
static const uint8_t g_AnnealerPin          = 6;
static const uint8_t g_DropServoPin         = 9;
static const uint8_t g_FeederStepPin       = 12;
static const uint8_t g_StartStopLedPin      = 4;
static const uint8_t g_CoolingFanPin        = 7;
static const uint8_t g_ModeLedPin           = 11;
#if NZHS_PLATFORM_UNO_R3
static const uint8_t g_PsuCurrentAdcPin     = 0;
#else
static const uint8_t g_PsuCurrentAdcPin     = A0;
#endif
static const uint8_t g_DropSolenoidPin      = 10;
static const uint8_t g_TimeSetButtonPin     = 16;
static const uint8_t g_FeederDirPin         = 13;
static const uint8_t g_FeederStepperEnPin   = 5;
static tAnalysisState g_Analysis;


// Custom startup image cropped to its non-blank 120x28 region. It is drawn at
// (0,2), reconstructing the original 128x32 frame without storing blank rows.
#if 0
const unsigned char anneallogo [] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00,
0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9E, 0x00, 0x00,
0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x3C,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x70, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00, 0x00, 0xE0, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xC3, 0x3F, 0x30,
0xC7, 0x80, 0x00, 0x00, 0x00, 0xDE, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xE3, 0x3F, 0x30, 0xCF,
0xC0, 0x00, 0x00, 0x00, 0xDE, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xE3, 0x03, 0x30, 0xCC, 0x40,
0x00, 0x00, 0x00, 0xDE, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xB3, 0x06, 0x30, 0xCC, 0x00, 0x00,
0x00, 0x00, 0xDE, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xB3, 0x0C, 0x3F, 0xCF, 0x80, 0x00, 0x00,
0x00, 0xDE, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x9B, 0x1C, 0x3F, 0xC3, 0xC0, 0x00, 0x00, 0x00,
0xDE, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x8B, 0x18, 0x30, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0xDE,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x8F, 0x30, 0x30, 0xC8, 0xC0, 0x00, 0x00, 0x00, 0xDE, 0x01,
0x80, 0x00, 0x00, 0x00, 0x01, 0x87, 0x3F, 0x30, 0xCF, 0xC0, 0x00, 0x00, 0x00, 0xDE, 0x01, 0x80,
0x00, 0x00, 0x00, 0x01, 0x87, 0x3F, 0x30, 0xCF, 0x80, 0x00, 0x00, 0x00, 0xDE, 0x01, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x01, 0xFF, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00, 0xFF, 0xC0, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0x9E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x0E
};
#endif

// The same logo split into its occupied regions. The straight horizontal
// edges are drawn as rectangles, saving flash without changing any pixels.
static const uint8_t anneallogoLeft[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x3C,
  0x00, 0x00, 0x70, 0x00, 0x00, 0xE0, 0x00, 0xFF, 0xC0, 0x01, 0xFF, 0x00,
  0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00,
  0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00,
  0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00,
  0x01, 0xFF, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x70,
  0x00, 0x00, 0x3C, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
};

static const uint8_t anneallogoText[] PROGMEM = {
  0x01, 0xC3, 0x3F, 0x30, 0xC7, 0x80, 0x01, 0xE3, 0x3F, 0x30, 0xCF, 0xC0,
  0x01, 0xE3, 0x03, 0x30, 0xCC, 0x40, 0x01, 0xB3, 0x06, 0x30, 0xCC, 0x00,
  0x01, 0xB3, 0x0C, 0x3F, 0xCF, 0x80, 0x01, 0x9B, 0x1C, 0x3F, 0xC3, 0xC0,
  0x01, 0x8B, 0x18, 0x30, 0xC0, 0xC0, 0x01, 0x8F, 0x30, 0x30, 0xC8, 0xC0,
  0x01, 0x87, 0x3F, 0x30, 0xCF, 0xC0, 0x01, 0x87, 0x3F, 0x30, 0xCF, 0x80,
};

static const uint8_t anneallogoRight[] PROGMEM = {
  0x0E, 0x9E, 0xFE, 0xFE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE,
  0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE,
  0xFE, 0xFE, 0x9E, 0x0E,
};

// The second logo frame is reproduced from anneallogo plus anneallogoDelta.
// Retained here only as source artwork while the delta form is validated.
#if 0
const unsigned char anneallogo2 [] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00,
0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9E, 0x00,
0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00,
0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x60, 0xE1, 0x9C, 0x33, 0xF0, 0x60, 0xC1, 0xF9, 0xF8, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0xF0, 0xF1, 0x9E, 0x33, 0xF0, 0xF0, 0xC1, 0xF9, 0xFC, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0xF0, 0xF1, 0x9E, 0x33, 0x00, 0xF0, 0xC1, 0x81, 0x8C, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x01, 0x98, 0xD9, 0x9B, 0x33, 0x01, 0x98, 0xC1, 0x81, 0x8C, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x01, 0x98, 0xD9, 0x9B, 0x33, 0xE1, 0x98, 0xC1, 0xF1, 0xF8, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x03, 0x0C, 0xCD, 0x99, 0xB3, 0xE3, 0x0C, 0xC1, 0xF1, 0xF0, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x03, 0xFC, 0xCD, 0x99, 0xB3, 0x03, 0xFC, 0xC1, 0x81, 0x98, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x03, 0xFC, 0xC7, 0x98, 0xF3, 0x03, 0xFC, 0xC1, 0x81, 0x98, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x06, 0x06, 0xC7, 0x98, 0xF3, 0xF6, 0x06, 0xFD, 0xF9, 0x8C, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x06, 0x06, 0xC3, 0x98, 0x73, 0xF6, 0x06, 0xFD, 0xF9, 0x8C, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00,
0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9E, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

 #endif

// Projectile image cropped to its non-blank 88x19 region. It is drawn at
// (16,6), reconstructing the original 128x32 frame without storing blank data.
const unsigned char projectile [] PROGMEM = {
0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x03, 0xFE, 0x00,
0x07, 0xC0, 0x01, 0xF0, 0x7F, 0x80, 0x00, 0x00, 0x3C, 0x1F, 0x00, 0x03, 0xF0, 0x00, 0xFC, 0x00,
0x78, 0x00, 0x01, 0xC0, 0x0F, 0xC0, 0x01, 0xFC, 0x00, 0x7F, 0x80, 0x04, 0x00, 0x06, 0x00, 0x03,
0xF0, 0x00, 0x7F, 0x00, 0x0F, 0xC0, 0x04, 0x00, 0x18, 0x00, 0x00, 0xFC, 0x00, 0x1F, 0x80, 0x03,
0xE0, 0x04, 0x00, 0xE0, 0x00, 0x00, 0x7F, 0x00, 0x07, 0xE0, 0x00, 0xC0, 0x04, 0x03, 0x00, 0x00,
0x00, 0x1F, 0xC0, 0x01, 0xF8, 0x00, 0x00, 0x04, 0x0C, 0x00, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x7E,
0x00, 0x00, 0x04, 0x30, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x00, 0x1F, 0x80, 0x00, 0x04, 0x0C, 0x00,
0x00, 0x00, 0x00, 0x7E, 0x00, 0x07, 0xE0, 0x00, 0x04, 0x03, 0x00, 0x00, 0x30, 0x00, 0x1F, 0x80,
0x03, 0xF8, 0x00, 0x04, 0x00, 0xE0, 0x00, 0x7C, 0x00, 0x0F, 0xE0, 0x00, 0xFE, 0x00, 0x04, 0x00,
0x18, 0x00, 0x3E, 0x00, 0x03, 0xF0, 0x00, 0x3F, 0x80, 0x04, 0x00, 0x06, 0x00, 0x1F, 0x80, 0x00,
0xFC, 0x00, 0x0F, 0xC0, 0x04, 0x00, 0x01, 0xC0, 0x07, 0xE0, 0x00, 0x3F, 0x00, 0x03, 0xE0, 0x04,
0x00, 0x00, 0x3C, 0x01, 0xF0, 0x00, 0x0F, 0x80, 0x00, 0xE0, 0x78, 0x00, 0x00, 0x03, 0xC0, 0xF8,
0x00, 0x07, 0xC0, 0x00, 0x7F, 0x80, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80,
0x00
};

// The second projectile frame is reproduced from projectile plus projectileDelta.
// Retained here only as source artwork while the delta form is validated.
#if 0
const unsigned char projectile2 [] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x0F, 0x80, 0x01, 0xF0, 0x00, 0x7F, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x07, 0xC0, 0x00, 0xFC, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x03, 0xF0, 0x00, 0x7F, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x1F, 0x80, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x07, 0xE0, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xE0, 0x00, 0x18, 0x00, 0x1F, 0xC0, 0x01, 0xF8, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x03, 0x00, 0x00, 0x3E, 0x00, 0x07, 0xF0, 0x00, 0x7E, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x0C, 0x00, 0x00, 0x1F, 0x80, 0x01, 0xF8, 0x00, 0x3F, 0x80, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x30, 0x00, 0x00, 0x0F, 0xE0, 0x00, 0x7E, 0x00, 0x0F, 0xC0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x0C, 0x00, 0x00, 0x03, 0xF0, 0x00, 0x1F, 0x80, 0x03, 0xE0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x07, 0xE0, 0x00, 0xE0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x3F, 0x00, 0x03, 0xF8, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x0F, 0xC0, 0x00, 0xFC, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x07, 0xF0, 0x00, 0x3F, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x01, 0xFC, 0x00, 0x0F, 0xC0, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x7E, 0x00, 0x03, 0xE0, 0x00, 0x78, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x1F, 0x00, 0x01, 0xF0, 0x7F, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


#endif

// XOR tiles which change the 100 ms alternate splash frames. Drawing one over
// its base frame with SSD1306_INVERSE recreates the original pixels exactly.
static const uint8_t anneallogoDelta[] PROGMEM = {
  0x00, 0x60, 0xE0, 0x5F, 0x0C, 0xC0, 0xA7, 0x41, 0xF9, 0xF8,
  0x00, 0xF0, 0xF0, 0x7D, 0x0C, 0xC0, 0x3F, 0x01, 0xF9, 0xFC,
  0x00, 0xF0, 0xF0, 0x7D, 0x30, 0x30, 0x3C, 0x81, 0x81, 0x8C,
  0x01, 0x98, 0xD8, 0x28, 0x35, 0x31, 0x54, 0xC1, 0x81, 0x8C,
  0x01, 0x98, 0xD8, 0x28, 0x3F, 0xDE, 0x57, 0x41, 0xF1, 0xF8,
  0x03, 0x0C, 0xCC, 0x02, 0xAF, 0xDC, 0xCF, 0x01, 0xF1, 0xF0,
  0x03, 0xFC, 0xCC, 0x12, 0xAB, 0x33, 0x3C, 0x01, 0x81, 0x98,
  0x03, 0xFC, 0xC6, 0x17, 0xC3, 0x33, 0x34, 0x01, 0x81, 0x98,
  0x06, 0x06, 0xC6, 0x1F, 0xCC, 0xC6, 0xC9, 0x3D, 0xF9, 0x8C,
  0x06, 0x06, 0xC2, 0x1F, 0x4C, 0xC6, 0xC9, 0x7D, 0xF9, 0x8C
};

static const uint8_t projectileDelta[] PROGMEM = {
  0x3E, 0x0F, 0x87, 0xC1, 0xF1, 0xF0, 0x00,
  0x1F, 0x07, 0xC3, 0xF0, 0xFC, 0xFC, 0x00,
  0x0F, 0xC3, 0xF1, 0xFC, 0x7F, 0x7F, 0x80,
  0x03, 0xF0, 0xFC, 0x7F, 0x1F, 0x8F, 0xC0,
  0x00, 0xFC, 0x3F, 0x1F, 0x87, 0xE3, 0xE0,
  0x18, 0x7F, 0x1F, 0xC7, 0xE1, 0xF8, 0xC0,
  0x3E, 0x1F, 0xC7, 0xF1, 0xF8, 0x7E, 0x00,
  0x1F, 0x87, 0xE1, 0xF8, 0x7E, 0x3F, 0x80,
  0x0F, 0xE1, 0xF8, 0x7E, 0x1F, 0x8F, 0xC0,
  0x03, 0xF0, 0x7E, 0x1F, 0x87, 0xE3, 0xE0,
  0x30, 0xFC, 0x1F, 0x87, 0xE3, 0xF8, 0xE0,
  0x7C, 0x3F, 0x0F, 0xE3, 0xF8, 0xFE, 0x00,
  0x3E, 0x0F, 0xC3, 0xF0, 0xFC, 0x3F, 0x80,
  0x1F, 0x87, 0xF0, 0xFC, 0x3F, 0x0F, 0xC0,
  0x07, 0xE1, 0xFC, 0x3F, 0x0F, 0xC3, 0xE0,
  0x01, 0xF0, 0x7E, 0x0F, 0x83, 0xE0, 0xE0,
  0x00, 0xF8, 0x1F, 0x07, 0xC1, 0xF0, 0x00
};

//-- global variables declarations----------------------------------------
static volatile tStateMachineStates g_SystemState = STATE_JUST_BOOTED;
static tStateMachineStates g_SystemStatePrev = STATE_UNKNOWN;

static ModeList CurrentMode = MODE_SINGLE_SHOT;
static tUserSettings g_UserSettings;
static tRunSafetyState g_RunSafety;
static tCartridgeProfile g_ProfileEditor;
static tProfileNotice g_ProfileNotice = PROFILE_NOTICE_SAVED;
static tAnalysisConfigState g_AnalysisConfig;
static tAdaptiveAnnealState g_AdaptiveAnneal;
static tCasePerformanceState g_CasePerformance;
static uint8_t g_InfoScreenScroll = 0;
static uint16_t g_SupplyVoltage_mv = 0;
static uint32_t g_SupplyVoltageSampleTime = 0;
// Deliberately not initialized by the C runtime, so a watchdog reset can
// preserve the last state reached by the application.
static tResetDiagnostics g_ResetDiagnostics __attribute__((section(".noinit")));

#if NZHS_PLATFORM_UNO_R4
static FspTimer g_R4FeederTimer;
static Servo g_R4DropServo;
static wdt_instance_ctrl_t g_R4WatchdogControl;
static uint32_t g_R4FeederBasePeriod = 0;
static volatile uint8_t g_R4FeederCompare = 0;
static uint8_t g_R4AppliedFeederCompare = UINT8_MAX;
static bool g_R4FeederTimerReady = false;
static bool g_R4DropServoReady = false;
static bool g_R4WatchdogReady = false;
#endif

#if NZHS_HAS_LED_MATRIX
static ArduinoLEDMatrix g_R4LedMatrix;
static bool g_R4MatrixDebugActive = false;
static bool g_R4MatrixReady = false;
static uint8_t g_R4MatrixPage = 0;
static uint8_t g_R4MatrixLastPage = UINT8_MAX;
static uint8_t g_R4MatrixHeartbeat = 0;
static uint8_t g_R4MatrixLastButtonMask = UINT8_MAX;
static bool g_R4ButtonTraceActive = false;
static bool g_R4MatrixButtonChanged = false;
static uint32_t g_R4MatrixPageTime = 0;
static uint32_t g_R4MatrixHeartbeatTime = 0;
#endif

#if NZHS_HAS_WIFI
static WiFiServer g_R4WifiServer(80);
static WiFiClient g_R4WifiClient;
static tR4WifiConfig g_R4WifiConfig;
static tR4WifiMode g_R4WifiMode = R4_WIFI_OFF;
static tWifiSettingsSelection g_R4WifiSettingsSelection = WIFI_SETTINGS_MONITOR;
static tWifiResetSelection g_R4WifiResetSelection = WIFI_RESET_BACK;
static bool g_R4WifiMonitorActive = false;
static bool g_R4WifiConfigValid = false;
static char g_R4WifiRequest[WIFI_HTTP_REQUEST_MAX_LENGTH];
static uint16_t g_R4WifiRequestLength = 0;
static uint16_t g_R4WifiHeaderLength = 0;
static uint16_t g_R4WifiContentLength = 0;
static uint32_t g_R4WifiClientDeadline = 0;
static uint32_t g_R4WifiConnectDeadline = 0;
static uint32_t g_R4WifiRestartTime = 0;
static uint32_t g_R4WifiStatusCheckTime = 0;
static tWifiHistoryRecord g_R4WifiHistory[WIFI_HISTORY_RECORD_COUNT];
static uint16_t g_R4WifiHistoryNextId = 1;
static uint8_t g_R4WifiHistoryHead = 0;
static uint8_t g_R4WifiHistoryCount = 0;
static bool g_R4WifiHistoryCaptureActive = false;
#endif

#if NZHS_PLATFORM_UNO_R3
  #define FEEDER_TIMER_COMPARE OCR2A
#else
  #define FEEDER_TIMER_COMPARE g_R4FeederCompare
#endif

// Run before normal C/C++ initialization. Capture the AVR reset flags and
// disable a watchdog that may still be active after a watchdog reset. The Uno
// bootloader can clear MCUSR first, so the recorded cause is best-effort.
#if NZHS_PLATFORM_UNO_R3
void captureResetDiagnostics(void) __attribute__((naked, used, section(".init3")));
void captureResetDiagnostics(void)
{
  g_ResetDiagnostics.resetFlags = MCUSR;
  MCUSR = 0;
  wdt_disable();
}
#else
static void captureResetDiagnostics(void)
{
  uint8_t resetFlags = 0;
  if(R_SYSTEM->RSTSR1_b.WDTRF || R_SYSTEM->RSTSR1_b.IWDTRF)
  {
    resetFlags |= _BV(WDRF);
  }
  if(R_SYSTEM->RSTSR0_b.LVD0RF || R_SYSTEM->RSTSR0_b.LVD1RF ||
     R_SYSTEM->RSTSR0_b.LVD2RF)
  {
    resetFlags |= _BV(BORF);
  }
  if(R_SYSTEM->RSTSR0 & R_SYSTEM_RSTSR0_PORF_Msk)
  {
    resetFlags |= _BV(PORF);
  }
  if(resetFlags == 0)
  {
    resetFlags = _BV(EXTRF);
  }
  g_ResetDiagnostics.resetFlags = resetFlags;
}
#endif

//-- function declarations------------------------------------------------
static void updateSystemState(tStateMachineStates const state);
static bool hasSystemStateChanged(void);
static inline bool hasTimeElapsed(uint32_t const timeTarget, uint32_t const currentTime);
static void setSystemTimeTarget(uint32_t const timeTarget);
static bool readStartButton(void);
static bool readModeButton(void);
static bool readUpButton(void);
static void turnAnnealerOn(void);
static void turnAnnealerOff(void);
static void openDropGate(void);
static void closeDropGate(void);
static void turnStartStopLedOn(void);
static void turnStartStopLedOff(void);
static void turnModeLedOn(void);
static void turnModeLedOff(void);
static void turnCoolingFanOn(void);
static void turnCoolingFanOff(void);
static uint16_t readSupplyVoltage_mv(void);
static void refreshSupplyVoltage(void);
static uint16_t readPsuCurrent_ma(void);
static uint16_t readAnnealingTime_ms(void);
static void preloadCase(void);
static void loadCase(void);
static void returnCaseFeederHome(void);
static bool caseFeederStillMoving(void);
static int16_t readTemperature(void);
static bool isTemperatureReadingValid(int16_t const temperature);
static void addStepsToGo(uint16_t const steps);
static void setStepsToGo(uint16_t const steps);
static uint16_t getStepsToGo(void);
static uint16_t getStepsFromHome(void);
static void loadUserSettings(void);
static void resetRunSafetyState(void);
static void enterCooldown(bool const allowAutomaticRestart, bool const cycleStopRequested);
static void advanceStoppedScreenSelection(void);
static void advanceSettingsScreenSelection(void);
static void advanceAnalysisMenuSelection(void);
static void advanceAnalysisConfigSelection(void);
static void advanceProfileSlot(void);
static void advanceProfileActionSelection(void);
static void returnToStoppedScreen(void);
static void enterAnalysis(void);
static void beginAnalysis(void);
static void sampleAnalysisCurrent(uint32_t const currentTime);
static void finishAnalysis(bool const aborted);
static void openAnalysisDropGate(void);
static void showSavedAnalysis(void);
static void beginAnalysisConfig(void);
static inline void updateAnalysisConfig(bool const rapidTimeAdjust) __attribute__((always_inline));
static void saveAnalysisConfigToProfile(uint8_t const slot);
static void resetGraphCapture(uint32_t const currentTime, bool const isAnalysis);
static uint16_t recordGraphCurrent(uint16_t const current_ma, uint32_t const currentTime);
static void beginCasePerformance(uint32_t const currentTime);
static void recordCasePerformance(uint16_t const current_ma, uint32_t const currentTime);
static void finishCasePerformance(void);
static void drawAnalysisMenuScreen(void);
static void drawAnalysisConfigScreen(void);
static void setTextSelected(bool const selected) __attribute__((noinline));
static void beginFullWidthScreen(void) __attribute__((noinline));
static void drawAnalysisLoadScreen(void);
static void drawAnalysisGraph(void);
static void drawAnalysisStatus(bool const dumping);
static void drawCasePerformanceGraph(tPerformanceFooter const footer,
                                     uint16_t const remainingTime_ms);
static void printAnalysisEnergy_J(uint32_t const energy_mJ);
static void setFreeRunMode(void);
static void setDumpButtonEnabled(bool const enabled);
static void cycleCurrentMode(void);
static uint16_t incrementTimeSetting(uint16_t const time_ms, bool const rapidTimeAdjust) __attribute__((noinline));
static void updateStoppedScreenSetting(bool const rapidTimeAdjust);
static void updateSettingsScreenSetting(void);
static uint16_t getProfileAddress(uint8_t const slot);
static uint8_t calculateProfileChecksum(tCartridgeProfile const * const profile);
static bool loadProfile(uint8_t const slot, tCartridgeProfile * const profile);
static void saveProfile(uint8_t const slot, tCartridgeProfile const * const profile);
static void loadProfileStopRule(uint8_t const slot);
static void saveProfileStopRule(uint8_t const slot, uint16_t const targetEnergy_J,
                                uint8_t const peakDropPercent);
static uint16_t getProfileReferenceAddress(uint8_t const slot);
static uint8_t readProfileReferenceSample(uint8_t const slot, uint8_t const sample);
static bool isProfileReferenceValid(uint8_t const slot);
static void activateProfileReference(uint8_t const slot);
static void saveProfileReference(uint8_t const slot);
static void clearProfileReference(uint8_t const slot);
static void clearProfile(uint8_t const slot);
static void makeDefaultProfile(uint8_t const slot, tCartridgeProfile * const profile);
static void applyProfile(tCartridgeProfile const * const profile);
static void saveCurrentSettingsToProfile(uint8_t const slot);
static void beginProfileNameEdit(uint8_t const slot);
static void advanceProfileNameCharacter(void);
static void saveEditedProfileName(void);
static void advanceInfoScreenScroll(void);
static bool lowCurrentGuardFault(uint16_t const cycleAverageCurrent_ma);
static void drawCurrentMode(uint8_t const y, bool const selected);
static void printTemperatureTenths(int16_t const temperature);
static void drawTemperature(uint8_t const y, int16_t const temperature);
static void drawCaseCount(uint8_t const y, uint16_t const casesAnnealed);
static void drawTimePanel(bool const selected);
static void drawStoppedScreen(bool const fanIsOn, int16_t const temperature, uint16_t const casesAnnealed);
static void drawSettingsScreen(void);
#if NZHS_HAS_WIFI
static void drawWifiSettingsScreen(void);
static void drawWifiResetScreen(void);
#endif
static void drawProfilesScreen(void);
static void drawProfileActionsScreen(void);
static void drawProfilePerformanceScreen(void);
static void drawProfileReferenceScreen(void);
static void drawProfileNameEditScreen(void);
static void drawProfileDeleteConfirmScreen(void);
static void drawProfileNoticeScreen(void);
static void drawDiagnosticsScreen(void);
static void drawInfoScreen(void);
static void drawResetDiagnostics(void);
static void printResetDiagnostics(void);
static void drawStartupLogo(void) __attribute__((noinline));
static void drawFaultScreen(__FlashStringHelper const * const line1,
                            __FlashStringHelper const * const line2) __attribute__((noinline));
#if NZHS_PLATFORM_UNO_R4
static void r4ConfigureFeederTimer(void);
static void r4StartFeederTimer(void);
static void r4ApplyFeederTimerCompare(void);
static void r4FeederTimerCallback(timer_callback_args_t *args);
static void r4BeginWatchdog(void);
static void r4ResetWatchdog(void);
static bool r4PlatformReady(void);
#endif
#if NZHS_HAS_LED_MATRIX
static void r4HandleBenchSerial(void);
static void r4UpdateButtonTrace(void);
static void r4BeginMatrixDebug(void);
static void r4UpdateMatrixDebug(int16_t const temperature);
static void r4DrawMatrixWord(uint8_t pixels[96], char const * const word);
static void r4RenderMatrixPage(int16_t const temperature);
static void r4PrintMatrixDebugStatus(void);
#endif
#if NZHS_HAS_WIFI
static void r4BeginWifiMonitor(void);
static void r4LoadWifiConfig(void);
static void r4SaveWifiConfig(void);
static void r4ClearWifiConfig(void);
static void r4SetWifiMonitorEnabled(bool const enabled);
static void r4StartConfiguredWifi(void);
static void r4StartWifiSetupAp(bool const fallback);
static void r4StopWifi(void);
static void r4UpdateWifiConnection(void);
static bool r4WifiHasIpAddress(void);
static void r4PrintWifiStatus(void);
static void r4UpdateWifiMonitor(int16_t const temperature,
                                uint16_t const current_ma,
                                uint16_t const casesAnnealed,
                                bool const fanIsOn);
static void r4SendWifiMonitorPage(WiFiClient &client);
static void r4SendWifiSetupPage(WiFiClient &client, char const * const message = NULL);
static void r4SendWifiStatus(WiFiClient &client, int16_t const temperature,
                             uint16_t const current_ma,
                             uint16_t const casesAnnealed,
                             bool const fanIsOn);
static void r4SendWifiCurve(WiFiClient &client);
static void r4SendWifiHistory(WiFiClient &client);
static void r4SendWifiHistoryCurve(WiFiClient &client, uint16_t const id);
static void r4SendWifiHistoryCsv(WiFiClient &client, uint16_t const id);
static char const * r4WifiHistoryReasonName(uint8_t const reason);
static tWifiHistoryRecord const * r4FindWifiHistory(uint16_t const id);
static void r4FinalizeGraphCapture(void);
static void r4StoreWifiHistory(tWifiHistoryReason const reason,
                               bool const analysis,
                               bool const matchValid);
static void r4FinishAnnealHistory(tWifiHistoryReason const reason);
static void r4HandleWifiRequest(WiFiClient &client,
                                int16_t const temperature,
                                uint16_t const current_ma,
                                uint16_t const casesAnnealed,
                                bool const fanIsOn);
#endif

/*---------------------------------------------------------------------------*/
/*! @brief      Initialize the Case Annealer.
  @details      None.
  @param        None.
  @return       None.
*//*-------------------------------------------------------------------------*/
void setup()
{
  #if NZHS_PLATFORM_UNO_R4
  captureResetDiagnostics();
  #endif
  if(g_ResetDiagnostics.magic != RESET_DIAGNOSTIC_MAGIC)
  {
    g_ResetDiagnostics.lastSystemState = STATE_UNKNOWN;
  }
  g_ResetDiagnostics.previousSystemState = g_ResetDiagnostics.lastSystemState;
  g_ResetDiagnostics.magic = RESET_DIAGNOSTIC_MAGIC;

  //TCCR0B = TCCR0B & B11111000 | B00000101; //PWM on D5 & D6 set to 61.04Hz Timer 0 -- Timer Used for system ms tick
 // TCCR2B = TCCR2B & B11111000 | B00000110; //PWM on D3 & D11 set to 122.55Hz Timer 2  <--- tiner 2
  #if NZHS_PLATFORM_UNO_R3
  TCCR1B = TCCR1B & B11111000 | B00000101; //PWM on D9 & D10 of 30.64 Hz Timer 1   <---- USE IO9 PWM for drop gate Servo
  #endif

  if(EEPROM.read(EEPROM_ADDRESS_CONFIG_MAGIC) != EEPROM_CONFIG_MAGIC)
  {
    EEPROM.update(EEPROM_ADDRESS_AUTO_RESTART, 0);
    EEPROM.update(EEPROM_ADDRESS_CONFIG_MAGIC, EEPROM_CONFIG_MAGIC);
  }
  if(EEPROM.read(EEPROM_ADDRESS_DUMP_BUTTON_MAGIC) != EEPROM_DUMP_BUTTON_MAGIC)
  {
    EEPROM.update(EEPROM_ADDRESS_DUMP_BUTTON, 0);
    EEPROM.update(EEPROM_ADDRESS_DUMP_BUTTON_MAGIC, EEPROM_DUMP_BUTTON_MAGIC);
  }

//set timer2 interrupt
  #if NZHS_PLATFORM_UNO_R3
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register - divide by microsteps to shorten step period
  FEEDER_TIMER_COMPARE = 170 / STEPPER_MICROSTEPS;
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS20-22 bit for prescaler
  TCCR2B |= (1 << CS22);
  TCCR2B |= (1 << CS21);

  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  #else
  FEEDER_TIMER_COMPARE = 170 / STEPPER_MICROSTEPS;
  r4ConfigureFeederTimer();
  #endif

  // Setup IO.
  pinMode(g_StartStopButtonPin, INPUT_PULLUP);
  pinMode(g_ModeButtonPin, INPUT_PULLUP);
  pinMode(g_TimeSetButtonPin, INPUT_PULLUP);
  pinMode(g_AnnealerPin, OUTPUT);
  pinMode(g_StartStopLedPin, OUTPUT);
  pinMode(g_ModeLedPin, OUTPUT);
  pinMode(g_CoolingFanPin, OUTPUT);
  pinMode(g_DropSolenoidPin, OUTPUT);
  pinMode(g_DropServoPin,OUTPUT);
  pinMode(g_FeederStepPin,OUTPUT);
  pinMode(g_FeederDirPin,OUTPUT);
  pinMode(g_FeederStepperEnPin,OUTPUT);
  #if NZHS_PLATFORM_UNO_R4
  g_R4DropServoReady = g_R4DropServo.attach(g_DropServoPin, 500, 2500) != 0;
  #endif
  digitalWrite(g_FeederDirPin,HIGH);
  digitalWrite(g_FeederStepperEnPin,LOW); //disable stepper driver
  closeDropGate();
  turnStartStopLedOff();
  turnModeLedOff();
  turnAnnealerOff();
  turnCoolingFanOff();
  closeDropGate();
  #if NZHS_PLATFORM_UNO_R4
  analogReadResolution(10);
  r4StartFeederTimer();
  #endif
  Serial.begin(115200);
  #ifdef DEBUG
  delay(20);
  Serial.println(F("Debug active."));
  printResetDiagnostics();
  #endif

  delay(200);

  display.beginFixed128x32I2C();

  display.clearDisplay();
  // Setup text and draw splash screen
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  #ifndef DEBUG //dont do the splash startup in debug
    drawStartupLogo();
    display.display();
    delay(2000);
    display.clearDisplay();
    drawStartupLogo();
    display.drawBitmap(24, 11, anneallogoDelta, 80, 10, SSD1306_INVERSE);
    display.display();
    delay(2000);

    for(uint8_t i = 0; i <= 20; i++)
    {
      display.clearDisplay();
      display.drawBitmap(16, 6, projectile, 88, 19, 1);
      display.display();
      delay(100);
      display.clearDisplay();
      display.drawBitmap(16, 6, projectile, 88, 19, 1);
      display.drawBitmap(40, 7, projectileDelta, 56, 17, SSD1306_INVERSE);
      display.display();
      delay(100);
    }
  #else
    delay(2000);
  #endif
  display.clearDisplay();
  //Setup temp sensor and read 1-wire address. initiate first temp reading
  sensors.begin();

  NumberDallasTempDevices = sensors.getDeviceCount(); //see how many temp sensors are on the 1-wire
  for(uint8_t i=0; i<16; i++)
  {
    psuCurrentZeroOffset += analogRead(g_PsuCurrentAdcPin);
    delay(10);
  }
  psuCurrentZeroOffset = psuCurrentZeroOffset >> 4; //divide by 16
  if(psuCurrentZeroOffset > 200) //see if there is a sensor on the ADC pin. should be mid-rail with no current
  {
    CurrentSensorPresent = true;
  }
  loadUserSettings();
  #if NZHS_HAS_WIFI
  r4LoadWifiConfig();
  #endif

  #ifdef DEBUG
  Serial.print(F("Software Version : "));
  Serial.println(SOFTWARE_VERSION);
  Serial.print(F("Number of Dallas temp sensors found : "));
  Serial.println(sensors.getDeviceCount());
  Serial.print(F("Current sensors found : "));
  Serial.println(CurrentSensorPresent);
  Serial.print(F("Drop gate control : "));
  #ifdef SERVO
    Serial.println(F("Servo"));
  #else
    Serial.println(F("Solenoid"));
  #endif
  Serial.print(F("PSU Current zero offset : "));
  Serial.println(psuCurrentZeroOffset);
  Serial.print(F("\n\n\n"));
  #endif

  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, TEMP_RESOLUTION);
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false);
  delay(TEMP_CONVERSION_TIME); // let the first temp read happen
  //setup the watchdog timer. it needs a boot every 500ms.
  #if NZHS_PLATFORM_UNO_R3
  wdt_enable(WDTO_500MS);
  #else
  r4BeginWatchdog();
  if(!g_R4FeederTimerReady) Serial.println(F("R4 INIT ERROR: FEED TIMER"));
  if(!g_R4DropServoReady) Serial.println(F("R4 INIT ERROR: DROP SERVO"));
  if(!g_R4WatchdogReady) Serial.println(F("R4 INIT ERROR: WATCHDOG"));
  #if NZHS_HAS_LED_MATRIX
  Serial.println(F("R4 bench: M=matrix, B=buttons, W=direct, S=setup, I=status, O=WiFi off, X=clear."));
  #endif
  #endif
  digitalWrite(g_FeederStepperEnPin,HIGH); //disable stepper driver
  #if NZHS_HAS_WIFI
  if(g_R4WifiConfig.monitorEnabled)
  {
    r4StartConfiguredWifi();
  }
  #endif
}
/*---------------------------------------------------------------------------*/
/*! @brief      Timer2 ISR
  @details      None.
  @param        None.
  @return       Never.
*//*-------------------------------------------------------------------------*/

#if NZHS_PLATFORM_UNO_R3
ISR(TIMER2_COMPA_vect){//timer2 interrupt
#else
static void r4FeederTimerCallback(timer_callback_args_t *args){
  (void)args;
#endif
  if(StepsToGo)
	  {
	  if (StepToggle)
	  {
	    digitalWrite(g_FeederStepPin,HIGH);
	    StepToggle = 0;
	    if(StepsFromHome + 1 >= STEPPER_STEPS_PER_TURN)
		  {
		  	StepsFromHome = 0;
		  }
		  else
		  {
		  	StepsFromHome = StepsFromHome + 1;
		  }
      if(StepsFromHome < CASE_FEEDER_HOPPER_START) //move feed wheel quickly to pick the next case
      {
          // set compare match register - divide by microsteps to shorten step period
          #if STEPPER_MICROSTEPS >= 4 //check we arent going to overflow the 8 bit timer register
            FEEDER_TIMER_COMPARE = 120 / STEPPER_MICROSTEPS;
          #else
            FEEDER_TIMER_COMPARE = 170;
          #endif

      }
      else if(StepsFromHome < CASE_FEEDER_HOPPER_END) //slow down the feed wheel while picking the case for more reliable pickups
      {
          // set compare match register - divide by microsteps to shorten step period
          #if STEPPER_MICROSTEPS >= 4 //check we arent going to overflow the 8 bit timer register
            FEEDER_TIMER_COMPARE = 800 / STEPPER_MICROSTEPS;
          #else
            FEEDER_TIMER_COMPARE = 254;
          #endif
      }
      else //speed up again once new case is picked
      {
        // set compare match register - divide by microsteps to shorten step period
          FEEDER_TIMER_COMPARE = 170 / STEPPER_MICROSTEPS;
      }
		StepsToGo = StepsToGo - 1;
	  }
	  else{
	    digitalWrite(g_FeederStepPin,LOW);
	    StepToggle = 1;
	  }
  }
  else
  {
  	digitalWrite(g_FeederStepPin,LOW);
  	StepToggle = 1;
  }

  if ((g_SystemState == STATE_ANNEALING || g_SystemState == STATE_ANALYSING) && hasTimeElapsed(SystemTimeTarget, millis()))
  {
    turnAnnealerOff();
  }

  #if NZHS_PLATFORM_UNO_R4
  r4ApplyFeederTimerCompare();
  #endif

}

#if NZHS_PLATFORM_UNO_R4
/*---------------------------------------------------------------------------*/
/*! @brief      Reserve a Renesas periodic timer for feeder step pulses.
*//*-------------------------------------------------------------------------*/
static void r4ConfigureFeederTimer(void)
{
  uint8_t timerType = GPT_TIMER;
  int8_t const channel = FspTimer::get_available_timer(timerType);
  if(channel < 0)
  {
    return;
  }
  float const initialFrequency = 16000000.0f /
    (256.0f * (FEEDER_TIMER_COMPARE + 1));
  g_R4FeederTimerReady = g_R4FeederTimer.begin(
    TIMER_MODE_PERIODIC, timerType, channel, initialFrequency, 0.0f,
    r4FeederTimerCallback);
  if(g_R4FeederTimerReady)
  {
    g_R4FeederTimer.set_period_buffer(false);
    g_R4FeederTimerReady = g_R4FeederTimer.setup_overflow_irq(12) &&
                           g_R4FeederTimer.open();
  }
  if(g_R4FeederTimerReady)
  {
    g_R4FeederBasePeriod = g_R4FeederTimer.get_period_raw();
    g_R4AppliedFeederCompare = FEEDER_TIMER_COMPARE;
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Start feeder callbacks after all shield pins are configured.
*//*-------------------------------------------------------------------------*/
static void r4StartFeederTimer(void)
{
  if(g_R4FeederTimerReady)
  {
    g_R4FeederTimerReady = g_R4FeederTimer.start();
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Match the AVR Timer2 compare periods on the Renesas timer.
*//*-------------------------------------------------------------------------*/
static void r4ApplyFeederTimerCompare(void)
{
  uint8_t const timerCompare = FEEDER_TIMER_COMPARE;
  if(!g_R4FeederTimerReady || timerCompare == g_R4AppliedFeederCompare)
  {
    return;
  }
  uint16_t const initialCounts = (170 / STEPPER_MICROSTEPS) + 1;
  uint32_t const period =
    ((uint64_t)g_R4FeederBasePeriod * (timerCompare + 1) +
     (initialCounts / 2)) / initialCounts;
  g_R4FeederTimer.set_period(period);
  g_R4AppliedFeederCompare = timerCompare;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Start the RA4M1 watchdog after lengthy setup work is complete.
*//*-------------------------------------------------------------------------*/
static void r4BeginWatchdog(void)
{
  wdt_cfg_t config = {};
  config.timeout = WDT_TIMEOUT_16384;
  config.clock_division = WDT_CLOCK_DIVISION_2048;
  config.window_start = WDT_WINDOW_START_100;
  config.window_end = WDT_WINDOW_END_0;
  config.reset_control = WDT_RESET_CONTROL_RESET;
  config.stop_control = WDT_STOP_CONTROL_ENABLE;
  g_R4WatchdogReady = R_WDT_Open(&g_R4WatchdogControl, &config) == FSP_SUCCESS;
  if(g_R4WatchdogReady)
  {
    R_WDT_Refresh(&g_R4WatchdogControl);
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Refresh the RA4M1 watchdog when its backend opened correctly.
*//*-------------------------------------------------------------------------*/
static void r4ResetWatchdog(void)
{
  if(g_R4WatchdogReady)
  {
    R_WDT_Refresh(&g_R4WatchdogControl);
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Require all timing and gate safety backends before a run.
*//*-------------------------------------------------------------------------*/
static bool r4PlatformReady(void)
{
  return g_R4FeederTimerReady && g_R4DropServoReady && g_R4WatchdogReady;
}
#endif

#if NZHS_HAS_LED_MATRIX
static const uint8_t MATRIX_FONT_3X5[26][5] PROGMEM = {
  {0b010, 0b101, 0b111, 0b101, 0b101}, // A
  {0b110, 0b101, 0b110, 0b101, 0b110}, // B
  {0b011, 0b100, 0b100, 0b100, 0b011}, // C
  {0b110, 0b101, 0b101, 0b101, 0b110}, // D
  {0b111, 0b100, 0b110, 0b100, 0b111}, // E
  {0b111, 0b100, 0b110, 0b100, 0b100}, // F
  {0b011, 0b100, 0b101, 0b101, 0b011}, // G
  {0b101, 0b101, 0b111, 0b101, 0b101}, // H
  {0b111, 0b010, 0b010, 0b010, 0b111}, // I
  {0b001, 0b001, 0b001, 0b101, 0b010}, // J
  {0b101, 0b101, 0b110, 0b101, 0b101}, // K
  {0b100, 0b100, 0b100, 0b100, 0b111}, // L
  {0b101, 0b111, 0b111, 0b101, 0b101}, // M
  {0b101, 0b111, 0b111, 0b111, 0b101}, // N
  {0b010, 0b101, 0b101, 0b101, 0b010}, // O
  {0b110, 0b101, 0b110, 0b100, 0b100}, // P
  {0b010, 0b101, 0b101, 0b111, 0b011}, // Q
  {0b110, 0b101, 0b110, 0b101, 0b101}, // R
  {0b011, 0b100, 0b010, 0b001, 0b110}, // S
  {0b111, 0b010, 0b010, 0b010, 0b010}, // T
  {0b101, 0b101, 0b101, 0b101, 0b111}, // U
  {0b101, 0b101, 0b101, 0b101, 0b010}, // V
  {0b101, 0b101, 0b111, 0b111, 0b101}, // W
  {0b101, 0b101, 0b010, 0b101, 0b101}, // X
  {0b101, 0b101, 0b010, 0b010, 0b010}, // Y
  {0b111, 0b001, 0b010, 0b100, 0b111}  // Z
};

static const uint8_t MATRIX_DIGITS_3X5[10][5] PROGMEM = {
  {0b111, 0b101, 0b101, 0b101, 0b111}, // 0
  {0b010, 0b110, 0b010, 0b010, 0b111}, // 1
  {0b110, 0b001, 0b010, 0b100, 0b111}, // 2
  {0b110, 0b001, 0b010, 0b001, 0b110}, // 3
  {0b101, 0b101, 0b111, 0b001, 0b001}, // 4
  {0b111, 0b100, 0b110, 0b001, 0b110}, // 5
  {0b011, 0b100, 0b110, 0b101, 0b010}, // 6
  {0b111, 0b001, 0b010, 0b010, 0b010}, // 7
  {0b111, 0b101, 0b111, 0b101, 0b111}, // 8
  {0b010, 0b101, 0b011, 0b001, 0b110}  // 9
};

static const uint8_t MATRIX_MINUS_3X5[5] PROGMEM = {
  0b000, 0b000, 0b111, 0b000, 0b000
};

/*---------------------------------------------------------------------------*/
/*! @brief      Draw a readable three-character 3x5 word into a 12x8 frame.
*//*-------------------------------------------------------------------------*/
static void r4DrawMatrixWord(uint8_t pixels[96], char const * const word)
{
  for(uint8_t character = 0; character < 3 && word[character]; character++)
  {
    char const letter = word[character];
    for(uint8_t row = 0; row < 5; row++)
    {
      uint8_t bits = 0;
      if(letter >= 'A' && letter <= 'Z')
      {
        bits = pgm_read_byte(&MATRIX_FONT_3X5[letter - 'A'][row]);
      }
      else if(letter >= '0' && letter <= '9')
      {
        bits = pgm_read_byte(&MATRIX_DIGITS_3X5[letter - '0'][row]);
      }
      else if(letter == '-')
      {
        bits = pgm_read_byte(&MATRIX_MINUS_3X5[row]);
      }
      for(uint8_t column = 0; column < 3; column++)
      {
        if(bits & (1 << (2 - column)))
        {
          pixels[((row + 1) * 12) + (character * 4) + column] = 1;
        }
      }
    }
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Print the detailed legend backing the matrix status pages.
*//*-------------------------------------------------------------------------*/
static void r4PrintMatrixDebugStatus(void)
{
  Serial.println(F("MATRIX DEBUG ACTIVE - RESET TO EXIT"));
  Serial.println(F("START IS BLOCKED; ANNEALER OUTPUT FORCED OFF"));
  Serial.println(F("Pages: RDY/ERR, reset cause, OUT, SNS, TMP/value, HBT"));
  Serial.println(F("Send N or M for the next page."));
}

/*---------------------------------------------------------------------------*/
/*! @brief      Render the selected bench-diagnostic page on the matrix.
*//*-------------------------------------------------------------------------*/
static void r4RenderMatrixPage(int16_t const temperature)
{
  uint8_t pixels[96] = {};
  bool const pageChanged = g_R4MatrixLastPage != g_R4MatrixPage;
  if(g_R4MatrixPage == 0)
  {
    bool const ready = r4PlatformReady() &&
                       EEPROM.length() >= ARDUINO_UNO_EEPROM_SIZE;
    if(ready || (g_R4MatrixHeartbeat & 1))
    {
      r4DrawMatrixWord(pixels, ready ? "RDY" : "ERR");
    }
    pixels[(7 * 12) + 1] = g_R4FeederTimerReady;
    pixels[(7 * 12) + 4] = g_R4DropServoReady;
    pixels[(7 * 12) + 7] = g_R4WatchdogReady;
    pixels[(7 * 12) + 10] = EEPROM.length() >= ARDUINO_UNO_EEPROM_SIZE;
    if(pageChanged)
    {
      Serial.print(F("PAGE "));
      Serial.print(ready ? F("RDY") : F("ERR"));
      Serial.print(F(": feeder="));
      Serial.print(g_R4FeederTimerReady ? F("Y") : F("N"));
      Serial.print(F(" servo="));
      Serial.print(g_R4DropServoReady ? F("Y") : F("N"));
      Serial.print(F(" watchdog="));
      Serial.print(g_R4WatchdogReady ? F("Y") : F("N"));
      Serial.print(F(" eeprom="));
      Serial.println(EEPROM.length() >= ARDUINO_UNO_EEPROM_SIZE ? F("Y") : F("N"));
    }
  }
  else if(g_R4MatrixPage == 1)
  {
    char const * resetWord = "UNK";
    char const * resetName = "unknown";
    if(g_ResetDiagnostics.resetFlags & _BV(WDRF))
    {
      resetWord = "WDT";
      resetName = "watchdog";
    }
    else if(g_ResetDiagnostics.resetFlags & _BV(BORF))
    {
      resetWord = "BRN";
      resetName = "brown-out";
    }
    else if(g_ResetDiagnostics.resetFlags & _BV(EXTRF))
    {
      resetWord = "EXT";
      resetName = "external/other";
    }
    else if(g_ResetDiagnostics.resetFlags & _BV(PORF))
    {
      resetWord = "PWR";
      resetName = "power-on";
    }
    r4DrawMatrixWord(pixels, resetWord);
    if(pageChanged)
    {
      Serial.print(F("PAGE "));
      Serial.print(resetWord);
      Serial.print(F(": reset="));
      Serial.println(resetName);
    }
  }
  else if(g_R4MatrixPage == 2)
  {
    r4DrawMatrixWord(pixels, "OUT");
    pixels[(7 * 12) + 0] = digitalRead(g_AnnealerPin);
    pixels[(7 * 12) + 2] = digitalRead(g_CoolingFanPin);
    pixels[(7 * 12) + 4] = digitalRead(g_DropSolenoidPin);
    pixels[(7 * 12) + 6] = !digitalRead(g_FeederStepperEnPin);
    pixels[(7 * 12) + 8] = digitalRead(g_FeederDirPin);
    pixels[(7 * 12) + 10] = StepsToGo != 0;
    if(pageChanged)
    {
      Serial.print(F("PAGE OUT: annealer="));
      Serial.print(digitalRead(g_AnnealerPin));
      Serial.print(F(" fan="));
      Serial.print(digitalRead(g_CoolingFanPin));
      Serial.print(F(" gate="));
      Serial.print(digitalRead(g_DropSolenoidPin));
      Serial.print(F(" feeder_enabled="));
      Serial.print(!digitalRead(g_FeederStepperEnPin));
      Serial.print(F(" direction="));
      Serial.print(digitalRead(g_FeederDirPin));
      Serial.print(F(" stepping="));
      Serial.println(StepsToGo != 0);
    }
  }
  else if(g_R4MatrixPage == 3)
  {
    r4DrawMatrixWord(pixels, "SNS");
    pixels[(7 * 12) + 1] = NumberDallasTempDevices != 0;
    pixels[(7 * 12) + 4] = isTemperatureReadingValid(temperature);
    pixels[(7 * 12) + 7] = CurrentSensorPresent;
    pixels[(7 * 12) + 10] = g_CasePerformance.referenceValid;
    if(pageChanged)
    {
      Serial.print(F("PAGE SNS: temp_device="));
      Serial.print(NumberDallasTempDevices ? F("Y") : F("N"));
      Serial.print(F(" temp_valid="));
      Serial.print(isTemperatureReadingValid(temperature) ? F("Y") : F("N"));
      Serial.print(F(" current="));
      Serial.print(CurrentSensorPresent ? F("Y") : F("N"));
      Serial.print(F(" reference="));
      Serial.println(g_CasePerformance.referenceValid ? F("Y") : F("N"));
    }
  }
  else if(g_R4MatrixPage == 4)
  {
    char temperatureWord[4] = "ERR";
    bool const validTemperature = isTemperatureReadingValid(temperature);
    if(validTemperature)
    {
      int16_t roundedTemperature =
        ((int32_t)temperature + (temperature < 0 ? -(TEMP_RAW_SCALE / 2) :
                                                   (TEMP_RAW_SCALE / 2))) /
        TEMP_RAW_SCALE;
      if(roundedTemperature >= 100)
      {
        temperatureWord[0] = '0' + ((roundedTemperature / 100) % 10);
        temperatureWord[1] = '0' + ((roundedTemperature / 10) % 10);
        temperatureWord[2] = '0' + (roundedTemperature % 10);
      }
      else if(roundedTemperature >= 10)
      {
        temperatureWord[0] = '0' + (roundedTemperature / 10);
        temperatureWord[1] = '0' + (roundedTemperature % 10);
        temperatureWord[2] = 'C';
      }
      else if(roundedTemperature >= 0)
      {
        temperatureWord[0] = ' ';
        temperatureWord[1] = '0' + roundedTemperature;
        temperatureWord[2] = 'C';
      }
      else
      {
        uint8_t const magnitude = -roundedTemperature;
        temperatureWord[0] = '-';
        if(magnitude < 10)
        {
          temperatureWord[1] = '0' + magnitude;
          temperatureWord[2] = 'C';
        }
        else
        {
          temperatureWord[1] = '0' + ((magnitude / 10) % 10);
          temperatureWord[2] = '0' + (magnitude % 10);
        }
      }
    }
    r4DrawMatrixWord(pixels,
      (g_R4MatrixHeartbeat & 1) ? temperatureWord : "TMP");
    if(pageChanged)
    {
      Serial.print(F("PAGE TMP: temperature="));
      if(validTemperature)
      {
        int16_t const tenths = ((int32_t)temperature * 10 +
          (temperature < 0 ? -(TEMP_RAW_SCALE / 2) : (TEMP_RAW_SCALE / 2))) /
          TEMP_RAW_SCALE;
        if(tenths < 0 && tenths > -10) Serial.write('-');
        Serial.print(tenths / 10);
        Serial.write('.');
        Serial.print(tenths < 0 ? (uint8_t)(-tenths % 10) :
                                  (uint8_t)(tenths % 10));
        Serial.println(F("C"));
      }
      else
      {
        Serial.println(F("ERR"));
      }
    }
  }
  else
  {
    r4DrawMatrixWord(pixels, "HBT");
    pixels[(7 * 12) + (g_R4MatrixHeartbeat % 11)] = 1;
    if(pageChanged)
    {
      Serial.println(F("PAGE HBT: main-loop heartbeat"));
    }
  }
  if(g_R4MatrixLastButtonMask & 0x01)
  {
    pixels[(0 * 12) + 11] = 1;
    pixels[(1 * 12) + 11] = 1;
  }
  if(g_R4MatrixLastButtonMask & 0x02)
  {
    pixels[(3 * 12) + 11] = 1;
    pixels[(4 * 12) + 11] = 1;
  }
  if(g_R4MatrixLastButtonMask & 0x04)
  {
    pixels[(6 * 12) + 11] = 1;
    pixels[(7 * 12) + 11] = 1;
  }
  g_R4LedMatrix.loadPixels(pixels, sizeof(pixels));
  g_R4MatrixLastPage = g_R4MatrixPage;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Start non-persistent matrix diagnostics while safely stopped.
*//*-------------------------------------------------------------------------*/
static void r4BeginMatrixDebug(void)
{
  turnAnnealerOff();
  turnStartStopLedOff();
  closeDropGate();
  digitalWrite(g_FeederStepperEnPin, HIGH);
  r4ResetWatchdog();
  g_R4MatrixDebugActive = true;
  g_R4MatrixReady = g_R4LedMatrix.begin() != 0;
  g_R4MatrixPage = 0;
  g_R4MatrixLastPage = UINT8_MAX;
  g_R4MatrixPageTime = millis();
  g_R4MatrixHeartbeatTime = g_R4MatrixPageTime;
  g_R4MatrixLastButtonMask = UINT8_MAX;
  g_R4MatrixButtonChanged = true;
  r4ResetWatchdog();
  if(!g_R4MatrixReady)
  {
    Serial.println(F("MATRIX DEBUG ERROR: TIMER/INITIALISATION FAILED"));
    Serial.println(F("RESET REQUIRED BEFORE A RUN"));
    return;
  }
  r4PrintMatrixDebugStatus();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Dispatch non-persistent R4 WiFi bench commands.
*//*-------------------------------------------------------------------------*/
static void r4HandleBenchSerial(void)
{
  while(Serial.available())
  {
    char const command = Serial.read();
    if(command == 'B' || command == 'b')
    {
      g_R4ButtonTraceActive = !g_R4ButtonTraceActive;
      g_R4MatrixLastButtonMask = UINT8_MAX;
      Serial.print(F("BUTTON TRACE "));
      Serial.print(g_R4ButtonTraceActive ? F("ON") : F("OFF"));
      if(!g_R4ButtonTraceActive && g_R4MatrixDebugActive)
      {
        Serial.print(F(" (MATRIX TRACE REMAINS ACTIVE)"));
      }
      Serial.println();
      continue;
    }
    if(command == 'O' || command == 'o')
    {
      #if NZHS_HAS_WIFI
      if(g_SystemState == STATE_STOPPED)
      {
        r4StopWifi();
        Serial.println(F("WIFI RUNTIME STOPPED; SAVED CONFIG RETAINED"));
      }
      else
      {
        Serial.println(F("WIFI STOP REFUSED: STOP THE ANNEALER FIRST"));
      }
      #endif
      continue;
    }
    if(command == 'X' || command == 'x')
    {
      #if NZHS_HAS_WIFI
      if(g_SystemState == STATE_STOPPED)
      {
        r4ClearWifiConfig();
      }
      else
      {
        Serial.println(F("WIFI CLEAR REFUSED: STOP THE ANNEALER FIRST"));
      }
      #endif
      continue;
    }
    if(command == 'I' || command == 'i')
    {
      #if NZHS_HAS_WIFI
      r4PrintWifiStatus();
      #endif
      continue;
    }
    if(command == 'S' || command == 's')
    {
      #if NZHS_HAS_WIFI
      if(g_SystemState == STATE_STOPPED)
      {
        r4StartWifiSetupAp(false);
      }
      else
      {
        Serial.println(F("WIFI SETUP REFUSED: STOP THE ANNEALER FIRST"));
      }
      #endif
      continue;
    }
    if(command == 'W' || command == 'w')
    {
      #if NZHS_HAS_WIFI
      if(g_R4WifiMode != R4_WIFI_OFF && g_R4WifiMode != R4_WIFI_ERROR)
      {
        Serial.println(F("WIFI MONITOR ALREADY ACTIVE"));
      }
      else if(g_SystemState == STATE_STOPPED)
      {
        r4BeginWifiMonitor();
      }
      else
      {
        Serial.println(F("WIFI MONITOR REFUSED: STOP THE ANNEALER FIRST"));
      }
      #endif
      continue;
    }
    if(command != 'M' && command != 'm' && command != 'N' && command != 'n')
    {
      continue;
    }
    if(!g_R4MatrixDebugActive)
    {
      if(command == 'N' || command == 'n')
      {
        continue;
      }
      if(g_SystemState == STATE_STOPPED)
      {
        r4BeginMatrixDebug();
      }
      else
      {
        Serial.println(F("MATRIX DEBUG REFUSED: STOP THE ANNEALER FIRST"));
      }
      continue;
    }
    if(g_R4MatrixReady)
    {
      g_R4MatrixPage = (g_R4MatrixPage + 1) % 6;
      g_R4MatrixLastPage = UINT8_MAX;
      g_R4MatrixPageTime = millis();
      g_R4MatrixHeartbeatTime = g_R4MatrixPageTime;
      g_R4MatrixHeartbeat = 0;
    }
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Report raw active-low button transitions when requested.
*//*-------------------------------------------------------------------------*/
static void r4UpdateButtonTrace(void)
{
  if(!g_R4ButtonTraceActive && !g_R4MatrixDebugActive)
  {
    return;
  }
  uint8_t const buttonMask = (readStartButton() ? 0x01 : 0) |
                             (readModeButton() ? 0x02 : 0) |
                             (readUpButton() ? 0x04 : 0);
  if(buttonMask == g_R4MatrixLastButtonMask)
  {
    return;
  }
  bool const startPressed = (buttonMask & 0x01) &&
                            !(g_R4MatrixLastButtonMask & 0x01);
  Serial.print(F("BUTTONS START="));
  Serial.print(buttonMask & 0x01 ? F("ON") : F("OFF"));
  Serial.print(F(" MODE="));
  Serial.print(buttonMask & 0x02 ? F("ON") : F("OFF"));
  Serial.print(F(" UP="));
  Serial.println(buttonMask & 0x04 ? F("ON") : F("OFF"));
  g_R4MatrixLastButtonMask = buttonMask;
  if(g_R4MatrixDebugActive)
  {
    g_R4MatrixButtonChanged = true;
    if(startPressed)
    {
      Serial.println(F("START BLOCKED: RESET TO EXIT MATRIX DEBUG"));
    }
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Keep the diagnostic display alive and all heating disabled.
*//*-------------------------------------------------------------------------*/
static void r4UpdateMatrixDebug(int16_t const temperature)
{
  if(!g_R4MatrixDebugActive)
  {
    return;
  }
  turnAnnealerOff();
  turnStartStopLedOff();
  digitalWrite(g_FeederStepperEnPin, HIGH);
  bool const buttonChanged = g_R4MatrixButtonChanged;
  g_R4MatrixButtonChanged = false;
  if(!g_R4MatrixReady)
  {
    return;
  }
  uint32_t const currentTime = millis();
  if(hasTimeElapsed(g_R4MatrixPageTime + 4000UL, currentTime))
  {
    g_R4MatrixPage = (g_R4MatrixPage + 1) % 6;
    g_R4MatrixLastPage = UINT8_MAX;
    g_R4MatrixPageTime = currentTime;
    g_R4MatrixHeartbeatTime = currentTime;
    g_R4MatrixHeartbeat = 0;
  }
  bool const animatePage = g_R4MatrixPage == 4 || g_R4MatrixPage == 5 ||
    (g_R4MatrixPage == 0 &&
     (!r4PlatformReady() || EEPROM.length() < ARDUINO_UNO_EEPROM_SIZE));
  uint16_t const animationPeriod_ms = g_R4MatrixPage == 5 ? 250 :
                                      g_R4MatrixPage == 4 ? 2000 : 500;
  if(animatePage &&
     hasTimeElapsed(g_R4MatrixHeartbeatTime + animationPeriod_ms, currentTime))
  {
    g_R4MatrixHeartbeat++;
    g_R4MatrixHeartbeatTime = currentTime;
    r4RenderMatrixPage(temperature);
  }
  else if(buttonChanged || g_R4MatrixLastPage != g_R4MatrixPage)
  {
    r4RenderMatrixPage(temperature);
  }
}
#endif

#if NZHS_HAS_WIFI
static const char R4_WIFI_MONITOR_HTML[] = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NZHS Annealer Monitor</title><link rel="icon" href="/favicon.ico"><link rel="apple-touch-icon" sizes="180x180" href="/apple-touch-icon.png"><link rel="manifest" href="/manifest.webmanifest"><meta name="theme-color" content="#080b10"><meta name="apple-mobile-web-app-capable" content="yes"><meta name="apple-mobile-web-app-status-bar-style" content="black-translucent"><meta name="apple-mobile-web-app-title" content="Annealer"><style>
:root{color-scheme:dark;font-family:system-ui,sans-serif}body{margin:0;background:#080b10;color:#edf6ff}main{max-width:900px;margin:auto;padding:18px}.head{display:flex;justify-content:space-between;align-items:baseline;gap:12px}h1{font-size:1.35rem;margin:0}h2{font-size:1.05rem;margin:18px 0 8px}.tag{color:#7fc8ff}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(135px,1fr));gap:10px;margin:16px 0}.card{background:#111a24;border:1px solid #24374a;border-radius:8px;padding:10px}.label{color:#8da4b8;font-size:.72rem;text-transform:uppercase}.value{font-size:1.35rem;margin-top:3px}canvas{width:100%;height:auto;background:#05080c;border:1px solid #24374a;border-radius:8px}.foot{color:#8da4b8;font-size:.8rem;margin-top:10px}.bad{color:#ff847c}.ok{color:#8ce99a}.history{overflow-x:auto;border:1px solid #24374a;border-radius:8px}table{width:100%;border-collapse:collapse;font-size:.82rem}th,td{padding:8px;text-align:left;border-bottom:1px solid #24374a;white-space:nowrap}th{color:#8da4b8}.action{display:inline-block;background:#19344b;color:#edf6ff;border:1px solid #385b78;border-radius:5px;padding:5px 8px;text-decoration:none;font:inherit}
</style></head><body><main><div class="head"><h1>NZHS Annealer</h1><span class="tag">read-only monitor</span></div>
<div class="grid"><div class="card"><div class="label">State</div><div class="value" id="state">-</div></div><div class="card"><div class="label">Mode</div><div class="value" id="mode">-</div></div><div class="card"><div class="label">Current</div><div class="value" id="current">-</div></div><div class="card"><div class="label">Temperature</div><div class="value" id="temp">-</div></div><div class="card"><div class="label">Energy</div><div class="value" id="energy">-</div></div><div class="card"><div class="label">Peak</div><div class="value" id="peak">-</div></div><div class="card"><div class="label">Cases</div><div class="value" id="cases">-</div></div><div class="card"><div class="label">Remaining</div><div class="value" id="remaining">-</div></div></div>
<canvas id="curve" width="800" height="320"></canvas><div class="foot" id="graphmode">LIVE</div><div class="foot" id="detail">Connecting...</div><div class="head"><h2>Session history</h2><button class="action" onclick="live()">LIVE</button></div><div class="history"><table><thead><tr><th>#</th><th>Result</th><th>Time</th><th>Peak</th><th>Energy</th><th>Match</th><th>CSV</th></tr></thead><tbody id="history"><tr><td colspan="7">No retained results</td></tr></tbody></table></div></main><script>
const $=id=>document.getElementById(id),cv=$('curve'),cx=cv.getContext('2d');let curve={actual:[],reference:[],max_ma:12500,duration_ms:8000},reviewId=null;
function value(id,v,s=''){ $(id).textContent=v==null?'-':v+s }
function draw(){let w=cv.width,h=cv.height,l=48,r=12,t=12,b=30;cx.clearRect(0,0,w,h);cx.strokeStyle='#24374a';cx.fillStyle='#8da4b8';cx.font='12px system-ui';for(let i=0;i<3;i++){let y=t+(h-t-b)*i/2;cx.beginPath();cx.moveTo(l,y);cx.lineTo(w-r,y);cx.stroke();cx.fillText((curve.max_ma*(2-i)/2000).toFixed(i?2:1)+'A',3,y+4)}for(let i=0;i<3;i++){let x=l+(w-l-r)*i/2;cx.fillText((curve.duration_ms*i/2000).toFixed(0)+'s',x-8,h-8)}function line(a,color,n){if(!a.length)return;cx.strokeStyle=color;cx.lineWidth=2;cx.beginPath();a.forEach((v,i)=>{let x=l+(w-l-r)*i/(n-1),y=t+(h-t-b)*(1-v/250);i?cx.lineTo(x,y):cx.moveTo(x,y)});cx.stroke()}line(curve.reference,'#718096',64);line(curve.actual,'#55b9ff',128)}
async function status(){try{let s=await fetch('/api/status',{cache:'no-store'}).then(r=>r.json());value('state',s.state);value('mode',s.mode);value('current',s.current_a,' A');value('temp',s.temperature_c,' C');value('energy',s.energy_j,' J');value('peak',s.peak_a,' A');value('cases',s.cases);value('remaining',(s.remaining_ms/1000).toFixed(1),' s');$('detail').className='foot '+(s.fault?'bad':'ok');$('detail').textContent=(s.fault?'FAULT | ':'')+'Profile '+s.profile+' | match '+(s.match_pct==null?'-':s.match_pct+'%')+' | energy '+(s.energy_pct==null?'-':s.energy_pct+'%')+' | '+(s.cooldown_lock?'cooldown lock active':'monitoring')}catch(e){$('detail').className='foot bad';$('detail').textContent='Monitor unavailable'}}
async function graph(){try{let u=reviewId==null?'/api/curve':'/api/history/'+reviewId;curve=await fetch(u,{cache:'no-store'}).then(r=>r.json());draw();$('graphmode').textContent=reviewId==null?'LIVE':'HISTORY #'+reviewId}catch(e){}}
function live(){reviewId=null;graph()}function review(id){reviewId=id;graph()}
async function refreshHistory(){try{let h=await fetch('/api/history',{cache:'no-store'}).then(r=>r.json());$('history').innerHTML=h.records.length?h.records.map(r=>`<tr><td><button class="action" onclick="review(${r.id})">${r.id}</button></td><td>${r.reason}</td><td>${(r.elapsed_ms/1000).toFixed(2)}s</td><td>${(r.peak_ma/1000).toFixed(1)}A</td><td>${(r.energy_mj/1000).toFixed(1)}J</td><td>${r.match_pct==null?'-':r.match_pct+'%'}</td><td><a class="action" href="/history/${r.id}.csv">CSV</a></td></tr>`).join(''):'<tr><td colspan="7">No retained results</td></tr>'}catch(e){}}
status();graph();refreshHistory();setInterval(status,500);setInterval(graph,1000);setInterval(refreshHistory,2000);
</script></body></html>)HTML";

static const char R4_WIFI_SETUP_HTML_START[] = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NZHS Annealer WiFi Setup</title><link rel="icon" href="/favicon.ico"><link rel="apple-touch-icon" sizes="180x180" href="/apple-touch-icon.png"><meta name="theme-color" content="#080b10"><style>:root{color-scheme:dark;font-family:system-ui,sans-serif}body{margin:0;background:#080b10;color:#edf6ff}main{max-width:420px;margin:auto;padding:24px}label{display:block;margin-top:16px;color:#9fb3c8}input{box-sizing:border-box;width:100%;padding:12px;margin-top:5px;background:#111a24;color:#fff;border:1px solid #38526c;border-radius:6px}button{width:100%;padding:12px;margin-top:22px;background:#1976b9;color:#fff;border:0;border-radius:6px;font-size:1rem}.note{color:#9fb3c8;font-size:.85rem}.msg{color:#8ce99a}</style></head><body><main><h1>NZHS Annealer</h1><h2>WiFi setup</h2>)HTML";

static const char R4_WIFI_SETUP_HTML_FORM[] = R"HTML(<form method="post" action="/setup/save"><label for="ssid">Network name</label><input id="ssid" name="ssid" maxlength="32" required><label for="password">Password</label><input id="password" name="password" type="password" maxlength="63"><button type="submit">Save and connect</button></form><p class="note">Credentials are stored unencrypted in the R4 EEPROM-backed storage. The monitor remains read-only.</p></main></body></html>)HTML";

static const char R4_WIFI_MANIFEST[] = R"JSON({"name":"NZHS Annealer Monitor","short_name":"Annealer","start_url":"/","display":"standalone","background_color":"#080b10","theme_color":"#080b10","icons":[{"src":"/icon-192.png","sizes":"192x192","type":"image/png","purpose":"any"},{"src":"/icon-512.png","sizes":"512x512","type":"image/png","purpose":"any maskable"}]})JSON";

/*---------------------------------------------------------------------------*/
/*! @brief      Return a compact human-readable name for web telemetry.
*//*-------------------------------------------------------------------------*/
static char const * r4WifiStateName(tStateMachineStates const state)
{
  switch(state)
  {
    case STATE_STOPPED: return "STOPPED";
    case STATE_PRELOAD: return "PRELOAD";
    case STATE_ANNEALING: return "ANNEALING";
    case STATE_DROPPING: return "DROPPING";
    case STATE_RELOADING: return "RELOADING";
    case STATE_COOLDOWN: return "COOLDOWN";
    case STATE_JUST_BOOTED: return "BOOTING";
    case STATE_SHOW_WARNING: return "WARNING";
    case STATE_SETTINGS: return "SETTINGS";
    case STATE_PROFILES: return "PROFILES";
    case STATE_PROFILE_ACTIONS: return "PROFILE";
    case STATE_PROFILE_PERFORMANCE: return "PERFORMANCE";
    case STATE_PROFILE_NAME_EDIT: return "RENAME";
    case STATE_PROFILE_DELETE_CONFIRM: return "DELETE";
    case STATE_PROFILE_NOTICE: return "PROFILE NOTICE";
    case STATE_ANALYSIS_LOAD: return "ANALYSE READY";
    case STATE_ANALYSING: return "ANALYSING";
    case STATE_ANALYSIS_GATE_OPEN: return "ANALYSE DUMP";
    case STATE_ANALYSIS_RESULT: return "ANALYSE RESULT";
    case STATE_DIAGNOSTICS: return "DIAGNOSTICS";
    case STATE_INFO: return "INFO";
    case STATE_OVERCURRENT_WARNING: return "OVERCURRENT";
    case STATE_LOW_CURRENT_WARNING: return "LOW CURRENT";
    case STATE_TEMPERATURE_SENSOR_WARNING: return "TEMP ERROR";
    case STATE_ANALYSIS_MENU: return "ANALYSE";
    case STATE_ANALYSIS_CONFIG: return "ANALYSE CONFIG";
    case STATE_TARGET_TIMEOUT_WARNING: return "TARGET TIMEOUT";
    case STATE_CURRENT_SENSOR_REQUIRED: return "CURRENT REQUIRED";
    case STATE_WIFI_SETTINGS: return "WIFI SETTINGS";
    case STATE_WIFI_RESET_CONFIRM: return "WIFI RESET";
    case STATE_PLATFORM_WARNING: return "R4 HW ERROR";
    default: return "UNKNOWN";
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Write common HTTP headers for a short-lived response.
*//*-------------------------------------------------------------------------*/
static void r4SendWifiHeaders(WiFiClient &client, char const * const contentType,
                              uint16_t const status = 200)
{
  client.print(F("HTTP/1.1 "));
  client.print(status);
  if(status == 200) client.println(F(" OK"));
  else if(status == 400) client.println(F(" Bad Request"));
  else client.println(F(" Not Found"));
  client.print(F("Content-Type: "));
  client.println(contentType);
  client.println(F("Cache-Control: no-store"));
  client.println(F("Connection: close"));
  client.println();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Serve the embedded read-only browser dashboard.
*//*-------------------------------------------------------------------------*/
static void r4SendWifiMonitorPage(WiFiClient &client)
{
  r4SendWifiHeaders(client, "text/html; charset=utf-8");
  client.print(R4_WIFI_MONITOR_HTML);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Serve the station-credential setup form.
*//*-------------------------------------------------------------------------*/
static void r4SendWifiSetupPage(WiFiClient &client, char const * const message)
{
  r4SendWifiHeaders(client, "text/html; charset=utf-8");
  client.print(R4_WIFI_SETUP_HTML_START);
  if(message)
  {
    client.print(F("<p class=\"msg\">"));
    client.print(message);
    client.println(F("</p>"));
  }
  client.print(R4_WIFI_SETUP_HTML_FORM);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Serve one embedded icon in watchdog-safe chunks.
*//*-------------------------------------------------------------------------*/
static void r4SendWifiAsset(WiFiClient &client, char const * const contentType,
                            uint8_t const * const data, size_t const length)
{
  client.println(F("HTTP/1.1 200 OK"));
  client.print(F("Content-Type: "));
  client.println(contentType);
  client.print(F("Content-Length: "));
  client.println(length);
  client.println(F("Cache-Control: public, max-age=86400"));
  client.println(F("Connection: close"));
  client.println();
  for(size_t offset = 0; offset < length; offset += 512)
  {
    size_t const remaining = length - offset;
    client.write(data + offset, remaining > 512 ? 512 : remaining);
    r4ResetWatchdog();
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Serve the current operating state as JSON.
*//*-------------------------------------------------------------------------*/
static void r4SendWifiStatus(WiFiClient &client, int16_t const temperature,
                             uint16_t const current_ma,
                             uint16_t const casesAnnealed,
                             bool const fanIsOn)
{
  tStateMachineStates const state = g_SystemState;
  uint32_t remaining_ms = 0;
  uint32_t const currentTime = millis();
  if((state == STATE_ANNEALING || state == STATE_DROPPING ||
      state == STATE_RELOADING || state == STATE_ANALYSING ||
      state == STATE_ANALYSIS_GATE_OPEN) &&
     !hasTimeElapsed(SystemTimeTarget, currentTime))
  {
    remaining_ms = SystemTimeTarget - currentTime;
  }
  uint16_t liveCurrent_ma = current_ma;
  bool const graphLive = state == STATE_ANALYSING ||
    (state == STATE_ANNEALING &&
     (g_UserSettings.stopType != PROFILE_STOP_TIME ||
      (g_CasePerformance.referenceValid && CurrentSensorPresent) ||
      g_R4WifiHistoryCaptureActive));
  bool const graphAvailable = graphLive ||
    (state != STATE_ANNEALING && g_Analysis.graphValid);
  if(graphLive)
  {
    liveCurrent_ma = g_Analysis.graphCurrent_ma;
  }
  else if(state != STATE_ANNEALING)
  {
    liveCurrent_ma = 0;
  }
  bool const fault = state == STATE_OVERCURRENT_WARNING ||
                     state == STATE_LOW_CURRENT_WARNING ||
                     state == STATE_TEMPERATURE_SENSOR_WARNING ||
                     state == STATE_TARGET_TIMEOUT_WARNING ||
                     state == STATE_CURRENT_SENSOR_REQUIRED ||
                     state == STATE_PLATFORM_WARNING;
  char const * mode = CurrentMode == MODE_AUTOMATIC ? "AUTO FEED" :
                      CurrentMode == MODE_FREE_RUN ? "FREE RUN" : "SINGLE";

  r4SendWifiHeaders(client, "application/json");
  client.print(F("{\"state\":\""));
  client.print(r4WifiStateName(state));
  client.print(F("\",\"mode\":\""));
  client.print(mode);
  client.print(F("\",\"wifi\":\""));
  if(g_R4WifiMode == R4_WIFI_STATION_MONITOR) client.print(F("LAN"));
  else if(g_R4WifiMode == R4_WIFI_DIRECT_MONITOR) client.print(F("DIRECT AP"));
  else if(g_R4WifiMode == R4_WIFI_SETUP_AP) client.print(F("SETUP AP"));
  else if(g_R4WifiMode == R4_WIFI_STATION_CONNECTING) client.print(F("CONNECTING"));
  else if(g_R4WifiMode == R4_WIFI_ERROR) client.print(F("ERROR"));
  else client.print(F("OFF"));
  client.print(F("\",\"ip\":\""));
  if(g_R4WifiMonitorActive) client.print(WiFi.localIP());
  client.print(F("\",\"temperature_c\":"));
  if(NumberDallasTempDevices && isTemperatureReadingValid(temperature))
  {
    int16_t const tenths = ((int32_t)temperature * 10 +
      (temperature < 0 ? -(TEMP_RAW_SCALE / 2) : (TEMP_RAW_SCALE / 2))) /
      TEMP_RAW_SCALE;
    if(tenths < 0 && tenths > -10) client.write('-');
    client.print(tenths / 10);
    client.write('.');
    client.print(tenths < 0 ? (uint8_t)(-tenths % 10) :
                              (uint8_t)(tenths % 10));
  }
  else
  {
    client.print(F("null"));
  }
  client.print(F(",\"current_a\":"));
  if(CurrentSensorPresent)
  {
    client.print(liveCurrent_ma / 1000);
    client.write('.');
    client.print((liveCurrent_ma % 1000) / 100);
  }
  else
  {
    client.print(F("null"));
  }
  client.print(F(",\"energy_j\":"));
  if(graphAvailable)
  {
    client.print(g_Analysis.inputEnergy_mJ / 1000UL);
    client.write('.');
    client.print((g_Analysis.inputEnergy_mJ % 1000UL) / 100UL);
  }
  else client.print(F("null"));
  client.print(F(",\"peak_a\":"));
  if(graphAvailable)
  {
    client.print(g_Analysis.peakCurrent_ma / 1000);
    client.write('.');
    client.print((g_Analysis.peakCurrent_ma % 1000) / 100);
  }
  else client.print(F("null"));
  client.print(F(",\"cases\":"));
  client.print(casesAnnealed);
  client.print(F(",\"remaining_ms\":"));
  client.print(remaining_ms);
  client.print(F(",\"profile\":"));
  client.print(g_UserSettings.profileSlot + 1);
  client.print(F(",\"match_pct\":"));
  if(g_CasePerformance.currentCycleCompared) client.print(g_CasePerformance.matchPercent);
  else client.print(F("null"));
  client.print(F(",\"energy_pct\":"));
  if(g_CasePerformance.currentCycleCompared) client.print(g_CasePerformance.energyPercent);
  else client.print(F("null"));
  client.print(F(",\"fan\":"));
  client.print(fanIsOn ? F("true") : F("false"));
  client.print(F(",\"cooldown_lock\":"));
  client.print(g_RunSafety.cooldownLockActive ? F("true") : F("false"));
  client.print(F(",\"fault\":"));
  client.print(fault ? F("true") : F("false"));
  client.println('}');
}

/*---------------------------------------------------------------------------*/
/*! @brief      Serve compact 0-250 graph samples and the active reference.
*//*-------------------------------------------------------------------------*/
static void r4SendWifiCurve(WiFiClient &client)
{
  uint8_t const actualCount = g_Analysis.graphColumn == UINT8_MAX ? 0 :
    g_Analysis.graphColumn + 1;
  r4SendWifiHeaders(client, "application/json");
  client.print(F("{\"duration_ms\":"));
  client.print(ANALYSIS_DURATION_MS);
  client.print(F(",\"max_ma\":"));
  client.print(ANALYSIS_GRAPH_MAX_CURRENT_MA);
  client.print(F(",\"actual\":["));
  for(uint8_t sample = 0; sample < actualCount; sample++)
  {
    if(sample) client.write(',');
    client.print(g_Analysis.graphSamples[sample]);
  }
  client.print(F("],\"reference\":["));
  if(g_CasePerformance.referenceValid)
  {
    for(uint8_t sample = 0; sample < PROFILE_REFERENCE_SAMPLE_COUNT; sample++)
    {
      if(sample) client.write(',');
      client.print(readProfileReferenceSample(g_CasePerformance.referenceSlot, sample));
    }
  }
  client.println(F("]}"));
}

/*---------------------------------------------------------------------------*/
/*! @brief      Return the stable browser/CSV name for a history stop reason.
*//*-------------------------------------------------------------------------*/
static char const * r4WifiHistoryReasonName(uint8_t const reason)
{
  switch(reason)
  {
    case WIFI_HISTORY_ANALYSE: return "ANALYSE";
    case WIFI_HISTORY_USER_ABORT: return "USER ABORT";
    case WIFI_HISTORY_TIME: return "TIME";
    case WIFI_HISTORY_ENERGY: return "ENERGY";
    case WIFI_HISTORY_PEAK_DROP: return "PEAK DROP";
    case WIFI_HISTORY_TIMEOUT: return "TIMEOUT";
    case WIFI_HISTORY_OVERCURRENT: return "OVERCURRENT";
    case WIFI_HISTORY_LOW_CURRENT: return "LOW CURRENT";
    case WIFI_HISTORY_TEMP_ERROR: return "TEMP ERROR";
    default: return "UNKNOWN";
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Find a retained session record by its rollover-safe ID.
*//*-------------------------------------------------------------------------*/
static tWifiHistoryRecord const * r4FindWifiHistory(uint16_t const id)
{
  for(uint8_t offset = 0; offset < g_R4WifiHistoryCount; offset++)
  {
    uint8_t const index =
      (g_R4WifiHistoryHead + WIFI_HISTORY_RECORD_COUNT - 1 - offset) %
      WIFI_HISTORY_RECORD_COUNT;
    if(g_R4WifiHistory[index].id == id)
    {
      return &g_R4WifiHistory[index];
    }
  }
  return NULL;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Finalise aggregate fields for any shared graph capture.
*//*-------------------------------------------------------------------------*/
static void r4FinalizeGraphCapture(void)
{
  g_Analysis.elapsedTime_ms = millis() - g_Analysis.startTime;
  if(g_Analysis.elapsedTime_ms > ANALYSIS_DURATION_MS)
  {
    g_Analysis.elapsedTime_ms = ANALYSIS_DURATION_MS;
  }
  g_Analysis.graphValid = g_Analysis.graphColumn != UINT8_MAX;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Copy the current graph and aggregates into the RAM ring.
*//*-------------------------------------------------------------------------*/
static void r4StoreWifiHistory(tWifiHistoryReason const reason,
                               bool const analysis,
                               bool const matchValid)
{
  if(!g_Analysis.graphValid)
  {
    return;
  }
  tWifiHistoryRecord * const record = &g_R4WifiHistory[g_R4WifiHistoryHead];
  memset(record, 0, sizeof(*record));
  record->completedAt_ms = millis();
  record->inputEnergy_mJ = g_Analysis.inputEnergy_mJ;
  record->id = g_R4WifiHistoryNextId++;
  if(g_R4WifiHistoryNextId == 0) g_R4WifiHistoryNextId = 1;
  record->elapsedTime_ms = g_Analysis.elapsedTime_ms;
  record->peakCurrent_ma = g_Analysis.peakCurrent_ma;
  record->reason = reason;
  record->profileSlot = matchValid ? g_CasePerformance.resultSlot :
                                     WIFI_HISTORY_NO_PROFILE;
  record->flags = (analysis ? WIFI_HISTORY_FLAG_ANALYSIS : 0) |
                  (matchValid ? WIFI_HISTORY_FLAG_MATCH : 0);
  if(matchValid)
  {
    record->matchPercent = g_CasePerformance.matchPercent;
    record->energyPercent = g_CasePerformance.energyPercent;
  }
  record->graphCount = g_Analysis.graphColumn == UINT8_MAX ? 0 :
                       g_Analysis.graphColumn + 1;
  memcpy(record->graphSamples, g_Analysis.graphSamples, record->graphCount);

  g_R4WifiHistoryHead = (g_R4WifiHistoryHead + 1) % WIFI_HISTORY_RECORD_COUNT;
  if(g_R4WifiHistoryCount < WIFI_HISTORY_RECORD_COUNT)
  {
    g_R4WifiHistoryCount++;
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Finalise a normal anneal capture and retain it once.
*//*-------------------------------------------------------------------------*/
static void r4FinishAnnealHistory(tWifiHistoryReason const reason)
{
  bool const hasCapture =
    (g_CasePerformance.referenceValid && CurrentSensorPresent) ||
    g_R4WifiHistoryCaptureActive;
  if(!hasCapture)
  {
    return;
  }
  if(g_CasePerformance.referenceValid && CurrentSensorPresent)
  {
    if(!g_CasePerformance.currentCycleCompared)
    {
      finishCasePerformance();
    }
  }
  else if(g_R4WifiHistoryCaptureActive)
  {
    r4FinalizeGraphCapture();
    g_Analysis.graphIsAnalysis = false;
  }
  r4StoreWifiHistory(reason, false,
                     g_CasePerformance.currentCycleCompared);
  g_R4WifiHistoryCaptureActive = false;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Serve retained session summaries, newest first.
*//*-------------------------------------------------------------------------*/
static void r4SendWifiHistory(WiFiClient &client)
{
  r4SendWifiHeaders(client, "application/json");
  client.print(F("{\"count\":"));
  client.print(g_R4WifiHistoryCount);
  client.print(F(",\"capacity\":"));
  client.print(WIFI_HISTORY_RECORD_COUNT);
  client.print(F(",\"records\":["));
  for(uint8_t offset = 0; offset < g_R4WifiHistoryCount; offset++)
  {
    uint8_t const index =
      (g_R4WifiHistoryHead + WIFI_HISTORY_RECORD_COUNT - 1 - offset) %
      WIFI_HISTORY_RECORD_COUNT;
    tWifiHistoryRecord const * const record = &g_R4WifiHistory[index];
    if(offset) client.write(',');
    client.print(F("{\"id\":"));
    client.print(record->id);
    client.print(F(",\"reason\":\""));
    client.print(r4WifiHistoryReasonName(record->reason));
    client.print(F("\",\"elapsed_ms\":"));
    client.print(record->elapsedTime_ms);
    client.print(F(",\"energy_mj\":"));
    client.print(record->inputEnergy_mJ);
    client.print(F(",\"peak_ma\":"));
    client.print(record->peakCurrent_ma);
    client.print(F(",\"analysis\":"));
    client.print(record->flags & WIFI_HISTORY_FLAG_ANALYSIS ? F("true") : F("false"));
    client.print(F(",\"profile\":"));
    if(record->profileSlot == WIFI_HISTORY_NO_PROFILE) client.print(F("null"));
    else client.print(record->profileSlot + 1);
    client.print(F(",\"match_pct\":"));
    if(record->flags & WIFI_HISTORY_FLAG_MATCH) client.print(record->matchPercent);
    else client.print(F("null"));
    client.print(F(",\"energy_pct\":"));
    if(record->flags & WIFI_HISTORY_FLAG_MATCH) client.print(record->energyPercent);
    else client.print(F("null"));
    client.write('}');
  }
  client.println(F("]}"));
}

/*---------------------------------------------------------------------------*/
/*! @brief      Serve one retained graph using the live-graph JSON schema.
*//*-------------------------------------------------------------------------*/
static void r4SendWifiHistoryCurve(WiFiClient &client, uint16_t const id)
{
  tWifiHistoryRecord const * const record = r4FindWifiHistory(id);
  if(!record)
  {
    r4SendWifiHeaders(client, "text/plain", 404);
    client.println(F("History record not found"));
    return;
  }
  r4SendWifiHeaders(client, "application/json");
  client.print(F("{\"duration_ms\":"));
  client.print(ANALYSIS_DURATION_MS);
  client.print(F(",\"max_ma\":"));
  client.print(ANALYSIS_GRAPH_MAX_CURRENT_MA);
  client.print(F(",\"actual\":["));
  for(uint8_t sample = 0; sample < record->graphCount; sample++)
  {
    if(sample) client.write(',');
    client.print(record->graphSamples[sample]);
  }
  client.print(F("],\"reference\":["));
  if(record->profileSlot != WIFI_HISTORY_NO_PROFILE &&
     isProfileReferenceValid(record->profileSlot))
  {
    for(uint8_t sample = 0; sample < PROFILE_REFERENCE_SAMPLE_COUNT; sample++)
    {
      if(sample) client.write(',');
      client.print(readProfileReferenceSample(record->profileSlot, sample));
    }
  }
  client.println(F("]}"));
}

/*---------------------------------------------------------------------------*/
/*! @brief      Download one retained current trace as spreadsheet-ready CSV.
*//*-------------------------------------------------------------------------*/
static void r4SendWifiHistoryCsv(WiFiClient &client, uint16_t const id)
{
  tWifiHistoryRecord const * const record = r4FindWifiHistory(id);
  if(!record)
  {
    r4SendWifiHeaders(client, "text/plain", 404);
    client.println(F("History record not found"));
    return;
  }
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/csv"));
  client.print(F("Content-Disposition: attachment; filename=annealer-"));
  client.print(record->id);
  client.println(F(".csv"));
  client.println(F("Cache-Control: no-store"));
  client.println(F("Connection: close"));
  client.println();
  client.println(F("id,reason,profile,elapsed_ms,energy_mJ,peak_mA,match_pct,sample_t_ms,current_mA"));
  for(uint8_t sample = 0; sample < record->graphCount; sample++)
  {
    client.print(record->id);
    client.write(',');
    client.print(r4WifiHistoryReasonName(record->reason));
    client.write(',');
    if(record->profileSlot != WIFI_HISTORY_NO_PROFILE) client.print(record->profileSlot + 1);
    client.write(',');
    client.print(record->elapsedTime_ms);
    client.write(',');
    client.print(record->inputEnergy_mJ);
    client.write(',');
    client.print(record->peakCurrent_ma);
    client.write(',');
    if(record->flags & WIFI_HISTORY_FLAG_MATCH) client.print(record->matchPercent);
    client.write(',');
    client.print(((uint32_t)sample * ANALYSIS_DURATION_MS) /
                 ANALYSIS_GRAPH_COLUMNS);
    client.write(',');
    client.println((uint16_t)record->graphSamples[sample] *
                   ANALYSIS_GRAPH_CURRENT_STEP_MA);
    if((sample & 0x0F) == 0) r4ResetWatchdog();
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Compute the compact persistent WiFi-record checksum.
*//*-------------------------------------------------------------------------*/
static uint8_t r4WifiConfigChecksum(tR4WifiConfig const * const config)
{
  uint8_t checksum = 0;
  uint8_t const * bytes = (uint8_t const *)config;
  for(uint16_t offset = 1; offset < sizeof(tR4WifiConfig) - 1; offset++)
  {
    checksum ^= bytes[offset];
  }
  return checksum;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Load credentials and the persistent monitor-enabled flag.
*//*-------------------------------------------------------------------------*/
static void r4LoadWifiConfig(void)
{
  EEPROM.get(EEPROM_ADDRESS_WIFI_CONFIG, g_R4WifiConfig);
  g_R4WifiConfig.ssid[WIFI_SSID_MAX_LENGTH] = 0;
  g_R4WifiConfig.password[WIFI_PASSWORD_MAX_LENGTH] = 0;
  g_R4WifiConfigValid = g_R4WifiConfig.magic == WIFI_CONFIG_MAGIC &&
    g_R4WifiConfig.version == WIFI_CONFIG_VERSION &&
    g_R4WifiConfig.monitorEnabled <= 1 &&
    g_R4WifiConfig.checksum == r4WifiConfigChecksum(&g_R4WifiConfig);
  if(!g_R4WifiConfigValid)
  {
    memset(&g_R4WifiConfig, 0, sizeof(g_R4WifiConfig));
    g_R4WifiConfig.magic = WIFI_CONFIG_MAGIC;
    g_R4WifiConfig.version = WIFI_CONFIG_VERSION;
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Persist credentials and monitor state in the unused EEPROM tail.
*//*-------------------------------------------------------------------------*/
static void r4SaveWifiConfig(void)
{
  g_R4WifiConfig.magic = WIFI_CONFIG_MAGIC;
  g_R4WifiConfig.version = WIFI_CONFIG_VERSION;
  g_R4WifiConfig.ssid[WIFI_SSID_MAX_LENGTH] = 0;
  g_R4WifiConfig.password[WIFI_PASSWORD_MAX_LENGTH] = 0;
  g_R4WifiConfig.checksum = r4WifiConfigChecksum(&g_R4WifiConfig);
  EEPROM.put(EEPROM_ADDRESS_WIFI_CONFIG, g_R4WifiConfig);
  g_R4WifiConfigValid = true;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Erase credentials, disable monitoring and stop the radio.
*//*-------------------------------------------------------------------------*/
static void r4ClearWifiConfig(void)
{
  r4StopWifi();
  memset(&g_R4WifiConfig, 0, sizeof(g_R4WifiConfig));
  g_R4WifiConfig.magic = WIFI_CONFIG_MAGIC;
  g_R4WifiConfig.version = WIFI_CONFIG_VERSION;
  r4SaveWifiConfig();
  Serial.println(F("WIFI CONFIG CLEARED"));
  r4PrintWifiStatus();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Stop all network runtime state without erasing credentials.
*//*-------------------------------------------------------------------------*/
static void r4StopWifi(void)
{
  g_R4WifiClient.stop();
  g_R4WifiServer.end();
  WiFi.end();
  g_R4WifiMonitorActive = false;
  g_R4WifiMode = R4_WIFI_OFF;
  g_R4WifiRequestLength = 0;
  g_R4WifiHeaderLength = 0;
  g_R4WifiContentLength = 0;
  g_R4WifiRestartTime = 0;
  g_R4WifiStatusCheckTime = 0;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Start the configured station connection without waiting.
*//*-------------------------------------------------------------------------*/
static void r4StartConfiguredWifi(void)
{
  if(!g_R4WifiConfig.monitorEnabled)
  {
    r4StopWifi();
    return;
  }
  if(!g_R4WifiConfigValid || g_R4WifiConfig.ssid[0] == 0)
  {
    r4StartWifiSetupAp(false);
    return;
  }
  r4StopWifi();
  if(WiFi.status() == WL_NO_MODULE)
  {
    g_R4WifiMode = R4_WIFI_ERROR;
    Serial.println(F("WIFI ERROR: MODULE NOT FOUND"));
    return;
  }
  Serial.print(F("WIFI: CONNECTING TO "));
  Serial.println(g_R4WifiConfig.ssid);
  if(g_R4WifiConfig.password[0])
  {
    WiFi.begin(g_R4WifiConfig.ssid, g_R4WifiConfig.password);
  }
  else
  {
    WiFi.begin(g_R4WifiConfig.ssid);
  }
  g_R4WifiMode = R4_WIFI_STATION_CONNECTING;
  g_R4WifiConnectDeadline = millis() + WIFI_CONNECT_TIMEOUT_MS;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Start the temporary credential setup/fallback access point.
*//*-------------------------------------------------------------------------*/
static void r4StartWifiSetupAp(bool const fallback)
{
  r4StopWifi();
  if(WiFi.status() == WL_NO_MODULE)
  {
    g_R4WifiMode = R4_WIFI_ERROR;
    Serial.println(F("WIFI SETUP ERROR: MODULE NOT FOUND"));
    return;
  }
  uint8_t const status = WiFi.beginAP("NZHS-Annealer-Setup");
  if(status != WL_AP_LISTENING && status != WL_AP_CONNECTED)
  {
    g_R4WifiMode = R4_WIFI_ERROR;
    Serial.print(F("WIFI SETUP ERROR: AP STATUS "));
    Serial.println(status);
    WiFi.end();
    return;
  }
  g_R4WifiServer.begin();
  g_R4WifiMonitorActive = true;
  g_R4WifiMode = R4_WIFI_SETUP_AP;
  Serial.println(fallback ? F("WIFI: SAVED NETWORK UNAVAILABLE; SETUP AP ACTIVE") :
                              F("WIFI SETUP AP ACTIVE"));
  Serial.println(F("SSID: NZHS-Annealer-Setup (open)"));
  Serial.print(F("Open http://"));
  Serial.println(WiFi.localIP());
}

/*---------------------------------------------------------------------------*/
/*! @brief      Preserve the original on-demand direct read-only monitor AP.
*//*-------------------------------------------------------------------------*/
static void r4BeginWifiMonitor(void)
{
  r4StopWifi();
  if(WiFi.status() == WL_NO_MODULE)
  {
    g_R4WifiMode = R4_WIFI_ERROR;
    Serial.println(F("WIFI MONITOR ERROR: WIFI MODULE NOT FOUND"));
    return;
  }
  Serial.println(F("WIFI MONITOR: STARTING OPEN READ-ONLY ACCESS POINT"));
  uint8_t const status = WiFi.beginAP("NZHS-Annealer");
  if(status != WL_AP_LISTENING && status != WL_AP_CONNECTED)
  {
    g_R4WifiMode = R4_WIFI_ERROR;
    Serial.print(F("WIFI MONITOR ERROR: AP STATUS "));
    Serial.println(status);
    WiFi.end();
    return;
  }
  g_R4WifiServer.begin();
  g_R4WifiMonitorActive = true;
  g_R4WifiMode = R4_WIFI_DIRECT_MONITOR;
  Serial.println(F("WIFI MONITOR ACTIVE - RESET TO EXIT"));
  Serial.println(F("SSID: NZHS-Annealer (open, read-only)"));
  Serial.print(F("Open http://"));
  Serial.println(WiFi.localIP());
}

/*---------------------------------------------------------------------------*/
/*! @brief      Persist and apply the visible monitor ON/OFF setting.
*//*-------------------------------------------------------------------------*/
static void r4SetWifiMonitorEnabled(bool const enabled)
{
  g_R4WifiConfig.monitorEnabled = enabled ? 1 : 0;
  r4SaveWifiConfig();
  if(enabled) r4StartConfiguredWifi();
  else r4StopWifi();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Confirm DHCP has supplied a usable non-zero local address.
*//*-------------------------------------------------------------------------*/
static bool r4WifiHasIpAddress(void)
{
  IPAddress const address = WiFi.localIP();
  return address[0] || address[1] || address[2] || address[3];
}

/*---------------------------------------------------------------------------*/
/*! @brief      Print current network mode and usable address over USB.
*//*-------------------------------------------------------------------------*/
static void r4PrintWifiStatus(void)
{
  Serial.print(F("WIFI STATUS: "));
  if(g_R4WifiMode == R4_WIFI_OFF) Serial.print(F("OFF"));
  else if(g_R4WifiMode == R4_WIFI_STATION_CONNECTING) Serial.print(F("CONNECTING"));
  else if(g_R4WifiMode == R4_WIFI_STATION_MONITOR) Serial.print(F("LAN"));
  else if(g_R4WifiMode == R4_WIFI_SETUP_AP) Serial.print(F("SETUP AP"));
  else if(g_R4WifiMode == R4_WIFI_DIRECT_MONITOR) Serial.print(F("DIRECT AP"));
  else Serial.print(F("ERROR"));
  Serial.print(F(" IP="));
  if(r4WifiHasIpAddress()) Serial.println(WiFi.localIP());
  else Serial.println(F("--"));
}

/*---------------------------------------------------------------------------*/
/*! @brief      Advance station connection and fallback without blocking.
*//*-------------------------------------------------------------------------*/
static void r4UpdateWifiConnection(void)
{
  uint32_t const currentTime = millis();
  bool const canReconfigure = g_SystemState != STATE_PRELOAD &&
    g_SystemState != STATE_ANNEALING && g_SystemState != STATE_DROPPING &&
    g_SystemState != STATE_RELOADING && g_SystemState != STATE_ANALYSING &&
    g_SystemState != STATE_ANALYSIS_GATE_OPEN;
  if(g_R4WifiRestartTime && hasTimeElapsed(g_R4WifiRestartTime, currentTime) &&
     canReconfigure)
  {
    g_R4WifiRestartTime = 0;
    r4StartConfiguredWifi();
    return;
  }
  if(!canReconfigure ||
     !hasTimeElapsed(g_R4WifiStatusCheckTime, currentTime))
  {
    return;
  }
  g_R4WifiStatusCheckTime = currentTime +
    (g_R4WifiMode == R4_WIFI_STATION_CONNECTING ? 250UL : 1000UL);
  if(g_R4WifiMode == R4_WIFI_STATION_CONNECTING)
  {
    if(WiFi.status() == WL_CONNECTED && r4WifiHasIpAddress())
    {
      g_R4WifiServer.begin();
      g_R4WifiMonitorActive = true;
      g_R4WifiMode = R4_WIFI_STATION_MONITOR;
      Serial.print(F("WIFI MONITOR LAN: http://"));
      Serial.println(WiFi.localIP());
    }
    else if(hasTimeElapsed(g_R4WifiConnectDeadline, currentTime) && canReconfigure)
    {
      r4StartWifiSetupAp(true);
    }
  }
  else if(g_R4WifiMode == R4_WIFI_STATION_MONITOR &&
          WiFi.status() != WL_CONNECTED && canReconfigure)
  {
    Serial.println(F("WIFI: CONNECTION LOST; RECONNECTING"));
    r4StartConfiguredWifi();
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Decode one hexadecimal form-escape digit.
*//*-------------------------------------------------------------------------*/
static int8_t r4WifiHexNibble(char const value)
{
  if(value >= '0' && value <= '9') return value - '0';
  char const upper = value & 0xDF;
  if(upper >= 'A' && upper <= 'F') return upper - 'A' + 10;
  return -1;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Decode one application/x-www-form-urlencoded field.
*//*-------------------------------------------------------------------------*/
static bool r4DecodeWifiFormField(char const * const body,
                                  char const * const key,
                                  char * const output,
                                  uint8_t const outputLength)
{
  size_t const keyLength = strlen(key);
  char const * value = body;
  while(value && *value)
  {
    if((value == body || value[-1] == '&') &&
       strncmp(value, key, keyLength) == 0 && value[keyLength] == '=')
    {
      value += keyLength + 1;
      uint8_t written = 0;
      while(*value && *value != '&' && written + 1 < outputLength)
      {
        char decoded = *value++;
        if(decoded == '+') decoded = ' ';
        else if(decoded == '%')
        {
          if(!value[0] || !value[1]) return false;
          char const high = *value++;
          char const low = *value++;
          int8_t const highValue = r4WifiHexNibble(high);
          int8_t const lowValue = r4WifiHexNibble(low);
          if(highValue < 0 || lowValue < 0) return false;
          decoded = (highValue << 4) | lowValue;
        }
        output[written++] = decoded;
      }
      output[written] = 0;
      return true;
    }
    value = strchr(value, '&');
    if(value) value++;
  }
  output[0] = 0;
  return false;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Route a complete fixed-buffer HTTP request.
*//*-------------------------------------------------------------------------*/
static void r4HandleWifiRequest(WiFiClient &client,
                                int16_t const temperature,
                                uint16_t const current_ma,
                                uint16_t const casesAnnealed,
                                bool const fanIsOn)
{
  g_R4WifiRequest[g_R4WifiRequestLength] = 0;
  char * method = g_R4WifiRequest;
  char * requestTarget = strchr(method, ' ');
  if(!requestTarget)
  {
    r4SendWifiHeaders(client, "text/plain", 400);
    client.println(F("Bad request"));
    return;
  }
  *requestTarget++ = 0;
  char * requestTargetEnd = strchr(requestTarget, ' ');
  if(requestTargetEnd) *requestTargetEnd = 0;
  requestTargetEnd = strchr(requestTarget, '?');
  if(requestTargetEnd) *requestTargetEnd = 0;
  uint8_t const requestTargetLength = strlen(requestTarget);
  if(requestTargetLength > 1 && requestTarget[requestTargetLength - 1] == '/')
  {
    requestTarget[requestTargetLength - 1] = 0;
  }

  if(strcmp(method, "POST") == 0 && strcmp(requestTarget, "/setup/save") == 0 &&
     g_R4WifiMode == R4_WIFI_SETUP_AP)
  {
    char * const body = g_R4WifiRequest + g_R4WifiHeaderLength;
    char ssid[WIFI_SSID_MAX_LENGTH + 1] = {};
    char password[WIFI_PASSWORD_MAX_LENGTH + 1] = {};
    bool const ssidPresent = r4DecodeWifiFormField(body, "ssid", ssid, sizeof(ssid));
    bool const passwordPresent = r4DecodeWifiFormField(body, "password", password,
                                                       sizeof(password));
    if(!ssidPresent || !passwordPresent || ssid[0] == 0)
    {
      r4SendWifiSetupPage(client, "Enter a valid network name.");
      return;
    }
    strncpy(g_R4WifiConfig.ssid, ssid, sizeof(g_R4WifiConfig.ssid));
    strncpy(g_R4WifiConfig.password, password, sizeof(g_R4WifiConfig.password));
    g_R4WifiConfig.ssid[WIFI_SSID_MAX_LENGTH] = 0;
    g_R4WifiConfig.password[WIFI_PASSWORD_MAX_LENGTH] = 0;
    g_R4WifiConfig.monitorEnabled = 1;
    r4SaveWifiConfig();
    memset(password, 0, sizeof(password));
    memset(body, 0, g_R4WifiContentLength);
    r4SendWifiSetupPage(client, "Saved. Reconnect to your normal WiFi network.");
    g_R4WifiRestartTime = millis() + 1500UL;
    Serial.print(F("WIFI: SAVED NETWORK "));
    Serial.println(g_R4WifiConfig.ssid);
  }
  else if(strcmp(method, "GET") == 0 && strcmp(requestTarget, "/api/status") == 0)
  {
    r4SendWifiStatus(client, temperature, current_ma, casesAnnealed, fanIsOn);
  }
  else if(strcmp(method, "GET") == 0 && strcmp(requestTarget, "/api/curve") == 0)
  {
    r4SendWifiCurve(client);
  }
  else if(strcmp(method, "GET") == 0 && strcmp(requestTarget, "/api/history") == 0)
  {
    r4SendWifiHistory(client);
  }
  else if(strcmp(method, "GET") == 0 &&
          strncmp(requestTarget, "/api/history/", 13) == 0)
  {
    char * end = NULL;
    unsigned long const id = strtoul(requestTarget + 13, &end, 10);
    if(id > 0 && id <= UINT16_MAX && end && *end == 0)
    {
      r4SendWifiHistoryCurve(client, id);
    }
    else
    {
      r4SendWifiHeaders(client, "text/plain", 404);
      client.println(F("History record not found"));
    }
  }
  else if(strcmp(method, "GET") == 0 &&
          strncmp(requestTarget, "/history/", 9) == 0)
  {
    char * end = NULL;
    unsigned long const id = strtoul(requestTarget + 9, &end, 10);
    if(id > 0 && id <= UINT16_MAX && end && strcmp(end, ".csv") == 0)
    {
      r4SendWifiHistoryCsv(client, id);
    }
    else
    {
      r4SendWifiHeaders(client, "text/plain", 404);
      client.println(F("History record not found"));
    }
  }
  else if(strcmp(method, "GET") == 0 &&
          (strcmp(requestTarget, "/") == 0 || strcmp(requestTarget, "/setup") == 0))
  {
    if(g_R4WifiMode == R4_WIFI_SETUP_AP) r4SendWifiSetupPage(client);
    else r4SendWifiMonitorPage(client);
  }
  else if(strcmp(method, "GET") == 0 && strcmp(requestTarget, "/favicon.ico") == 0)
  {
    r4SendWifiAsset(client, "image/x-icon", R4_FAVICON_ICO,
                    R4_FAVICON_ICO_LENGTH);
  }
  else if(strcmp(method, "GET") == 0 &&
          (strcmp(requestTarget, "/apple-touch-icon.png") == 0 ||
           strcmp(requestTarget, "/apple-touch-icon-precomposed.png") == 0))
  {
    r4SendWifiAsset(client, "image/png", R4_APPLE_TOUCH_ICON_PNG,
                    R4_APPLE_TOUCH_ICON_PNG_LENGTH);
  }
  else if(strcmp(method, "GET") == 0 && strcmp(requestTarget, "/icon-192.png") == 0)
  {
    r4SendWifiAsset(client, "image/png", R4_ICON_192_PNG,
                    R4_ICON_192_PNG_LENGTH);
  }
  else if(strcmp(method, "GET") == 0 && strcmp(requestTarget, "/icon-512.png") == 0)
  {
    r4SendWifiAsset(client, "image/png", R4_ICON_512_PNG,
                    R4_ICON_512_PNG_LENGTH);
  }
  else if(strcmp(method, "GET") == 0 &&
          strcmp(requestTarget, "/manifest.webmanifest") == 0)
  {
    client.println(F("HTTP/1.1 200 OK"));
    client.println(F("Content-Type: application/manifest+json"));
    client.print(F("Content-Length: "));
    client.println(strlen(R4_WIFI_MANIFEST));
    client.println(F("Cache-Control: public, max-age=300"));
    client.println(F("Connection: close"));
    client.println();
    client.print(R4_WIFI_MANIFEST);
  }
  else
  {
    Serial.print(F("WIFI HTTP 404: "));
    Serial.println(requestTarget);
    r4SendWifiHeaders(client, "text/plain", 404);
    client.println(F("Not found"));
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Service one bounded HTTP request without waiting for a client.
*//*-------------------------------------------------------------------------*/
static void r4UpdateWifiMonitor(int16_t const temperature,
                                uint16_t const current_ma,
                                uint16_t const casesAnnealed,
                                bool const fanIsOn)
{
  r4UpdateWifiConnection();
  if(!g_R4WifiMonitorActive)
  {
    return;
  }
  if(!g_R4WifiClient || !g_R4WifiClient.connected())
  {
    g_R4WifiClient.stop();
    g_R4WifiClient = g_R4WifiServer.available();
    if(!g_R4WifiClient)
    {
      return;
    }
    g_R4WifiRequestLength = 0;
    g_R4WifiHeaderLength = 0;
    g_R4WifiContentLength = 0;
    g_R4WifiClientDeadline = millis() + 4000UL;
  }

  uint8_t bytesRead = 0;
  while(g_R4WifiClient.available() && bytesRead < 512)
  {
    char const incoming = g_R4WifiClient.read();
    bytesRead++;
    if(g_R4WifiRequestLength + 1 >= sizeof(g_R4WifiRequest))
    {
      r4SendWifiHeaders(g_R4WifiClient, "text/plain", 400);
      g_R4WifiClient.println(F("Request too large"));
      g_R4WifiClient.stop();
      g_R4WifiRequestLength = 0;
      return;
    }
    g_R4WifiRequest[g_R4WifiRequestLength++] = incoming;
    g_R4WifiRequest[g_R4WifiRequestLength] = 0;

    if(g_R4WifiHeaderLength == 0 && g_R4WifiRequestLength >= 4 &&
       memcmp(g_R4WifiRequest + g_R4WifiRequestLength - 4, "\r\n\r\n", 4) == 0)
    {
      g_R4WifiHeaderLength = g_R4WifiRequestLength;
      char const * contentLength = strstr(g_R4WifiRequest, "Content-Length:");
      if(contentLength)
      {
        g_R4WifiContentLength = atoi(contentLength + 15);
      }
      if(g_R4WifiHeaderLength + g_R4WifiContentLength >= sizeof(g_R4WifiRequest))
      {
        r4SendWifiHeaders(g_R4WifiClient, "text/plain", 400);
        g_R4WifiClient.println(F("Request too large"));
        g_R4WifiClient.stop();
        g_R4WifiRequestLength = 0;
        return;
      }
    }

    if(g_R4WifiHeaderLength &&
       g_R4WifiRequestLength >= g_R4WifiHeaderLength + g_R4WifiContentLength)
    {
      r4HandleWifiRequest(g_R4WifiClient, temperature, current_ma,
                          casesAnnealed, fanIsOn);
      g_R4WifiClient.stop();
      g_R4WifiRequestLength = 0;
      return;
    }
  }
  if(bytesRead)
  {
    g_R4WifiClientDeadline = millis() + 4000UL;
  }
  if(hasTimeElapsed(g_R4WifiClientDeadline, millis()))
  {
    g_R4WifiClient.stop();
    g_R4WifiRequestLength = 0;
  }
}
#endif

/*---------------------------------------------------------------------------*/
/*! @brief      Main Loop.
  @details      None.
  @param        None.
  @return       Never.
*//*-------------------------------------------------------------------------*/
void loop()
{
  // Retained upstream serial-interface placeholder; no serial UI is active.
  static bool isSerialInterface = false;
  static bool start;
  static bool startPrev;

  static bool modeKey;
  static bool modeKeyPrev;
  static bool upKey=0;
  static bool upKeyPrev=0;
  static uint8_t upKeyDuration = 0;
  static uint8_t rapidTimePresses = 0;
  static uint32_t lastTimePress = 0;
  static uint32_t profileNameRepeatStart = 0;
  static uint32_t profileNameNextRepeat = 0;
  static bool FanIsOn = false;
  static bool annealTimeChanged = false;
  // Retained upstream measurement placeholder; voltage is not displayed yet.
  static uint16_t psuVoltage_mv;
  static uint16_t psuCurrent_ma;
  static bool manualDumpInProgress = false;
  static uint32_t analysisModeKeyPressTime = 0;
  static uint32_t cooling_timer = 0;
  static uint32_t LoopStartTime;
  static int16_t temperature = 0;
  static bool Just_Booted = 1;
  static bool Next_Cycle_Is_STOPPED = 0;
  static bool targetTimeoutPending = false;
  static uint16_t CasesAnnealed = 0;

  //boot the watchdog
  #if NZHS_PLATFORM_UNO_R3
  wdt_reset();
  #else
  r4ResetWatchdog();
  #endif
  g_ResetDiagnostics.lastSystemState = g_SystemState;
  //read keys
  LoopStartTime = millis(); // capture time when loop starts
  start = readStartButton();
  modeKey = readModeButton();
  upKey = readUpButton();
  #if NZHS_HAS_LED_MATRIX
  r4HandleBenchSerial();
  r4UpdateButtonTrace();
  if(g_R4MatrixDebugActive)
  {
    // Matrix diagnostics own the physical buttons. Their raw state remains
    // visible over Serial and on the right-edge LED pairs, but they must not
    // navigate menus or reach any run state.
    start = false;
    modeKey = false;
    upKey = false;
  }
  #endif

  // DS18B20 conversion takes longer than the 25 ms analysis sampler.
  if(g_SystemState != STATE_ANALYSING &&
     !(g_SystemState == STATE_ANNEALING &&
       (g_UserSettings.stopType != PROFILE_STOP_TIME ||
        (g_CasePerformance.referenceValid && CurrentSensorPresent)
        #if NZHS_HAS_WIFI
        || ((g_R4WifiConfig.monitorEnabled ||
             g_R4WifiMode == R4_WIFI_DIRECT_MONITOR) &&
            CurrentSensorPresent)
        #endif
        )))
  {
    temperature = readTemperature();
  }

  // Leaving the cooldown screen must not make a new run possible until the
  // original hysteresis threshold has been reached.
  if(NumberDallasTempDevices != 0 &&
     isTemperatureReadingValid(temperature) &&
     g_RunSafety.cooldownLockActive &&
     temperature < ((TEMP_LIMIT - TEMP_HYSTERESIS) * TEMP_RAW_SCALE))
  {
    g_RunSafety.cooldownLockActive = false;
  }

  if(NumberDallasTempDevices != 0 &&
     !isTemperatureReadingValid(temperature) &&
     g_SystemState != STATE_TEMPERATURE_SENSOR_WARNING)
  {
    // An invalid temperature means the capacitor temperature is unknown.
    // Stop safely and require a valid sensor reading before another run.
    turnAnnealerOff();
    closeDropGate();
    turnStartStopLedOff();
    g_RunSafety.cooldownRestartPending = false;
    cooling_timer = COOLDOWN_PERIOD + millis();
    updateSystemState(STATE_TEMPERATURE_SENSOR_WARNING);
  }

  /*temperature = sensors.getTempCByIndex(0);
  sensors.requestTemperatures(); // this takes quite some time to complete ~90ms or longer. read it on the next loop*/

  if (start && !startPrev) //Start key pressed?
  {
    if (g_SystemState == STATE_STOPPED)
    {
      #if NZHS_HAS_LED_MATRIX
      if(g_R4MatrixDebugActive)
      {
        turnAnnealerOff();
        turnStartStopLedOff();
        Serial.println(F("START BLOCKED: RESET TO EXIT MATRIX DEBUG"));
      }
      else
      #endif
      #if NZHS_PLATFORM_UNO_R4
      if(!r4PlatformReady())
      {
        updateSystemState(STATE_PLATFORM_WARNING);
      }
      else
      #endif
      if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_ANALYSE)
      {
        enterAnalysis();
      }
      else if(g_RunSafety.cooldownLockActive)
      {
        // The user may browse menus while cooling, but cannot start a run yet.
      }
      else if(NumberDallasTempDevices != 0 && temperature > (TEMP_LIMIT * TEMP_RAW_SCALE))
      {
        enterCooldown(false, false);
      }
      else if(g_UserSettings.stopType != PROFILE_STOP_TIME && !CurrentSensorPresent)
      {
        updateSystemState(STATE_CURRENT_SENSOR_REQUIRED);
      }
      else
      {
        resetRunSafetyState();
        if(Just_Booted) //Show the warning screen to set the right case heigth and time 1st time
        {
          updateSystemState(STATE_SHOW_WARNING);
          Just_Booted = 0;
        }
        else if (CurrentMode == MODE_AUTOMATIC)
        {
          updateSystemState(STATE_ANNEALING); //STATE_PRELOAD
        }
        else
        {
          updateSystemState(STATE_ANNEALING);
        }
      }
    }
    else if(g_SystemState == STATE_SHOW_WARNING)
    {
      updateSystemState(STATE_STOPPED);
    }
    else if(g_SystemState == STATE_SETTINGS ||
            g_SystemState == STATE_PROFILES ||
            g_SystemState == STATE_PROFILE_ACTIONS ||
            g_SystemState == STATE_PROFILE_PERFORMANCE ||
            g_SystemState == STATE_PROFILE_NAME_EDIT ||
            g_SystemState == STATE_PROFILE_DELETE_CONFIRM ||
            g_SystemState == STATE_PROFILE_NOTICE ||
            g_SystemState == STATE_ANALYSIS_MENU ||
            g_SystemState == STATE_ANALYSIS_CONFIG ||
            g_SystemState == STATE_DIAGNOSTICS ||
            g_SystemState == STATE_INFO
            #if NZHS_HAS_WIFI
            || g_SystemState == STATE_WIFI_SETTINGS ||
               g_SystemState == STATE_WIFI_RESET_CONFIRM
            #endif
            )
    {
      // Menus always leave through their visible BACK > item and UP.
    }
    else if(g_SystemState == STATE_ANALYSIS_LOAD)
    {
      if(CurrentSensorPresent &&
         !g_RunSafety.cooldownLockActive &&
         (NumberDallasTempDevices == 0 || temperature <= (TEMP_LIMIT * TEMP_RAW_SCALE)))
      {
        beginAnalysis();
      }
    }
    else if(g_SystemState == STATE_ANALYSIS_RESULT)
    {
      // Leave through the visible BACK > item with UP, like the other menus.
    }
    else if(g_SystemState == STATE_COOLDOWN)
    {
      g_RunSafety.cooldownRestartPending = false;
      updateSystemState(STATE_STOPPED);
    }
    else if (g_SystemState != STATE_COOLDOWN) //confirm it's not in cooldown mode
    {
      Next_Cycle_Is_STOPPED = 1;
      /*closeDropGate();
      if(CurrentMode == MODE_AUTOMATIC)
      {
      	returnCaseFeederHome();
      }
      updateSystemState(STATE_STOPPED);
      */
    }
  }

  if(modeKey == 0)
  {
    if(modeKeyPrev && manualDumpInProgress)
    {
      closeDropGate();
      manualDumpInProgress = false;
    }
    analysisModeKeyPressTime = 0;
  }
  else
  {
    if (!modeKeyPrev) //mode key just pressed?
    {
      #if NZHS_PLATFORM_UNO_R4
      if(g_SystemState == STATE_PLATFORM_WARNING)
      {
        updateSystemState(STATE_STOPPED);
      }
      else
      #endif
      if (g_SystemState == STATE_OVERCURRENT_WARNING ||
          g_SystemState == STATE_LOW_CURRENT_WARNING ||
          g_SystemState == STATE_TEMPERATURE_SENSOR_WARNING ||
          g_SystemState == STATE_TARGET_TIMEOUT_WARNING ||
          g_SystemState == STATE_CURRENT_SENSOR_REQUIRED)
      {
        updateSystemState(STATE_STOPPED);
      }
      else if (g_SystemState == STATE_STOPPED || g_SystemState == STATE_JUST_BOOTED)
      {
        advanceStoppedScreenSelection();
      }
      else if(g_SystemState == STATE_SETTINGS)
      {
        advanceSettingsScreenSelection();
      }
      #if NZHS_HAS_WIFI
      else if(g_SystemState == STATE_WIFI_SETTINGS)
      {
        g_R4WifiSettingsSelection =
          (tWifiSettingsSelection)((g_R4WifiSettingsSelection + 1) %
                                   WIFI_SETTINGS_SELECTION_COUNT);
      }
      else if(g_SystemState == STATE_WIFI_RESET_CONFIRM)
      {
        g_R4WifiResetSelection =
          (tWifiResetSelection)((g_R4WifiResetSelection + 1) %
                                WIFI_RESET_SELECTION_COUNT);
      }
      #endif
      else if(g_SystemState == STATE_ANALYSIS_MENU)
      {
        advanceAnalysisMenuSelection();
      }
      else if(g_SystemState == STATE_ANALYSIS_CONFIG)
      {
        advanceAnalysisConfigSelection();
      }
      else if(g_SystemState == STATE_PROFILES)
      {
        advanceProfileSlot();
      }
      else if(g_SystemState == STATE_PROFILE_ACTIONS)
      {
        advanceProfileActionSelection();
      }
      else if(g_SystemState == STATE_PROFILE_PERFORMANCE)
      {
        // The result screen has one visible BACK > action operated by UP.
      }
      else if(g_SystemState == STATE_PROFILE_NAME_EDIT)
      {
        g_UserSettings.profileNameCursor = (g_UserSettings.profileNameCursor + 1) % (PROFILE_NAME_LENGTH + 2);
      }
      else if(g_SystemState == STATE_PROFILE_DELETE_CONFIRM)
      {
        g_UserSettings.profileDeleteConfirmed = !g_UserSettings.profileDeleteConfirmed;
      }
      else if(g_SystemState == STATE_PROFILE_NOTICE)
      {
        // Ignore MODE while the brief acknowledgement is visible.
      }
      else if(g_SystemState == STATE_INFO)
      {
        advanceInfoScreenScroll();
      }
      else if(g_SystemState == STATE_ANALYSIS_GATE_OPEN ||
              g_SystemState == STATE_ANALYSIS_RESULT)
      {
        // The analysed case is already being, or has already been, dumped.
      }
      else if(g_SystemState == STATE_ANALYSIS_LOAD)
      {
        // MODE is reserved for navigating the Analyse menu or aborting a run.
      }
      else if(g_SystemState == STATE_ANALYSING)
      {
        analysisModeKeyPressTime = millis();
      }
      else if(g_UserSettings.dumpButtonEnabled && CurrentMode == MODE_FREE_RUN)
      {
        openDropGate();
        manualDumpInProgress = true;
      }
    }
    else if(g_SystemState == STATE_ANALYSING &&
            analysisModeKeyPressTime != 0 &&
            hasTimeElapsed(analysisModeKeyPressTime + ANALYSIS_ABORT_HOLD_MS, millis()))
    {
      finishAnalysis(true);
      openAnalysisDropGate();
      analysisModeKeyPressTime = 0;
    }
  }

  // Give every time editor the same UP-button behaviour as the home screen.
  bool rapidTimeAdjust = false;
  bool persistTimeSetting = false;
  uint16_t * selectedTime_ms = NULL;
  if(g_SystemState == STATE_STOPPED &&
     g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_TIME)
  {
    selectedTime_ms = &g_UserSettings.annealTime_ms;
    persistTimeSetting = true;
  }
  else if(g_SystemState == STATE_ANALYSIS_CONFIG)
  {
    if(g_AnalysisConfig.stopType == PROFILE_STOP_TIME &&
       g_AnalysisConfig.selection == ANALYSIS_CONFIG_TARGET)
    {
      selectedTime_ms = &g_AnalysisConfig.stopTime_ms;
    }
    else if(g_AnalysisConfig.stopType != PROFILE_STOP_TIME &&
            g_AnalysisConfig.selection == ANALYSIS_CONFIG_MAX_TIME)
    {
      selectedTime_ms = &g_AnalysisConfig.maxTime_ms;
    }
  }

  if(selectedTime_ms != NULL)
  {
    upKeyDuration = upKey ? upKeyDuration + 1 : 0;
    if(upKeyDuration >= LONG_PRESS_HOLD_TIME)
    {
      *selectedTime_ms = MIN_ANNEAL_TIME;
      if(persistTimeSetting) annealTimeChanged = true;
      upKeyDuration = 0;
      rapidTimePresses = 0;
      lastTimePress = 0;
    }
    if(upKey && !upKeyPrev)
    {
      uint32_t const currentTime = millis();
      if(lastTimePress == 0 || hasTimeElapsed(lastTimePress + RAPID_TIME_PRESS_INTERVAL, currentTime))
      {
        rapidTimePresses = 1;
      }
      else if(rapidTimePresses < RAPID_TIME_PRESS_COUNT)
      {
        rapidTimePresses++;
      }
      lastTimePress = currentTime;
      rapidTimeAdjust = rapidTimePresses >= RAPID_TIME_PRESS_COUNT;
      if(persistTimeSetting) annealTimeChanged = true;
    }
  }
  else
  {
    upKeyDuration = 0;
    rapidTimePresses = 0;
    lastTimePress = 0;
  }


  switch (g_SystemState) //State machine.
  {
    case STATE_STOPPED:
    {
      updateSystemState(g_SystemState);
      if (upKey && !upKeyPrev) //up key pressed?
      {
        updateStoppedScreenSetting(rapidTimeAdjust);
        if(g_SystemState != STATE_STOPPED)
        {
          break;
        }
      }

      drawStoppedScreen(FanIsOn, temperature, CasesAnnealed);
      turnStartStopLedOff();
      turnAnnealerOff();
      psuCurrent_ma = readPsuCurrent_ma(); //--------- added this
      updateSystemState(STATE_STOPPED);
    }
    break;

    case STATE_SETTINGS:
    {
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        updateSettingsScreenSetting();
        if(g_SystemState != STATE_SETTINGS)
        {
          break;
        }
      }
      drawSettingsScreen();
    }
    break;

    #if NZHS_HAS_WIFI
    case STATE_WIFI_SETTINGS:
    {
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        if(g_R4WifiSettingsSelection == WIFI_SETTINGS_MONITOR)
        {
          r4SetWifiMonitorEnabled(!g_R4WifiConfig.monitorEnabled);
        }
        else if(g_R4WifiSettingsSelection == WIFI_SETTINGS_SETUP)
        {
          r4StartWifiSetupAp(false);
        }
        else if(g_R4WifiSettingsSelection == WIFI_SETTINGS_RESET)
        {
          g_R4WifiResetSelection = WIFI_RESET_BACK;
          updateSystemState(STATE_WIFI_RESET_CONFIRM);
          break;
        }
        else
        {
          updateSystemState(STATE_SETTINGS);
          break;
        }
      }
      drawWifiSettingsScreen();
    }
    break;

    case STATE_WIFI_RESET_CONFIRM:
    {
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        if(g_R4WifiResetSelection == WIFI_RESET_CONFIRM)
        {
          r4ClearWifiConfig();
          g_R4WifiSettingsSelection = WIFI_SETTINGS_MONITOR;
        }
        updateSystemState(STATE_WIFI_SETTINGS);
        break;
      }
      drawWifiResetScreen();
    }
    break;
    #endif

    case STATE_PROFILES:
    {
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        if(g_UserSettings.profileSlot >= PROFILE_COUNT)
        {
          returnToStoppedScreen();
          break;
        }
        // Every profile opens on LOAD, whether or not the slot is populated.
        g_UserSettings.profileActionSelection = PROFILE_ACTION_LOAD;
        updateSystemState(STATE_PROFILE_ACTIONS);
        break;
      }
      drawProfilesScreen();
    }
    break;

    case STATE_PROFILE_ACTIONS:
    {
      tCartridgeProfile profile;
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        if(g_UserSettings.profileActionSelection == PROFILE_ACTION_LOAD)
        {
          if(loadProfile(g_UserSettings.profileSlot, &profile))
          {
            applyProfile(&profile);
            g_ProfileNotice = PROFILE_NOTICE_LOADED;
          }
          else
          {
            g_ProfileNotice = PROFILE_NOTICE_EMPTY;
          }
          updateSystemState(STATE_PROFILE_NOTICE);
        }
        else if(g_UserSettings.profileActionSelection == PROFILE_ACTION_PERFORMANCE)
        {
          if(isProfileReferenceValid(g_UserSettings.profileSlot))
          {
            updateSystemState(STATE_PROFILE_PERFORMANCE);
          }
          else
          {
            g_ProfileNotice = PROFILE_NOTICE_NO_DATA;
            updateSystemState(STATE_PROFILE_NOTICE);
          }
        }
        else if(g_UserSettings.profileActionSelection == PROFILE_ACTION_SAVE)
        {
          bool const wasSaved = loadProfile(g_UserSettings.profileSlot, &profile);
          saveCurrentSettingsToProfile(g_UserSettings.profileSlot);
          if(!wasSaved)
          {
            beginProfileNameEdit(g_UserSettings.profileSlot);
          }
          else
          {
            g_ProfileNotice = PROFILE_NOTICE_SAVED;
            updateSystemState(STATE_PROFILE_NOTICE);
          }
        }
        else if(g_UserSettings.profileActionSelection == PROFILE_ACTION_RENAME)
        {
          beginProfileNameEdit(g_UserSettings.profileSlot);
        }
        else if(g_UserSettings.profileActionSelection == PROFILE_ACTION_DELETE)
        {
          if(loadProfile(g_UserSettings.profileSlot, &profile))
          {
            g_UserSettings.profileDeleteConfirmed = false;
            updateSystemState(STATE_PROFILE_DELETE_CONFIRM);
          }
        }
        else
        {
          updateSystemState(STATE_PROFILES);
        }
        break;
      }
      drawProfileActionsScreen();
    }
    break;

    case STATE_PROFILE_PERFORMANCE:
    {
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        updateSystemState(STATE_PROFILE_ACTIONS);
        break;
      }
      drawProfilePerformanceScreen();
    }
    break;

    case STATE_PROFILE_NAME_EDIT:
    {
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        if(g_UserSettings.profileNameCursor < PROFILE_NAME_LENGTH)
        {
          advanceProfileNameCharacter();
          profileNameRepeatStart = millis();
          profileNameNextRepeat = profileNameRepeatStart + PROFILE_NAME_REPEAT_DELAY;
        }
        else if(g_UserSettings.profileNameCursor == PROFILE_NAME_LENGTH)
        {
          saveEditedProfileName();
          g_ProfileNotice = PROFILE_NOTICE_SAVED;
          updateSystemState(STATE_PROFILE_NOTICE);
          break;
        }
        else
        {
          updateSystemState(g_AnalysisConfig.profileSaveInProgress ? STATE_ANALYSIS_CONFIG : STATE_PROFILE_ACTIONS);
          break;
        }
      }
      else if(upKey &&
              g_UserSettings.profileNameCursor < PROFILE_NAME_LENGTH &&
              profileNameRepeatStart != 0 &&
              hasTimeElapsed(profileNameNextRepeat, millis()))
      {
        // Keep name entry quick without making the character sequence unreadable.
        advanceProfileNameCharacter();
        profileNameNextRepeat = millis() + PROFILE_NAME_REPEAT_PERIOD;
      }
      if(!upKey)
      {
        profileNameRepeatStart = 0;
        profileNameNextRepeat = 0;
      }
      drawProfileNameEditScreen();
    }
    break;

    case STATE_PROFILE_DELETE_CONFIRM:
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        if(g_UserSettings.profileDeleteConfirmed)
        {
          clearProfile(g_UserSettings.profileSlot);
          g_ProfileNotice = PROFILE_NOTICE_DELETED;
          updateSystemState(STATE_PROFILE_NOTICE);
        }
        else
        {
          updateSystemState(STATE_PROFILE_ACTIONS);
        }
        break;
      }
      drawProfileDeleteConfirmScreen();
    break;

    case STATE_PROFILE_NOTICE:
    {
      if(hasSystemStateChanged())
      {
        setSystemTimeTarget(millis() + PROFILE_NOTICE_PERIOD);
      }
      updateSystemState(g_SystemState);
      if(hasTimeElapsed(SystemTimeTarget, millis()))
      {
        if(g_AnalysisConfig.profileSaveInProgress)
        {
          g_AnalysisConfig.profileSaveInProgress = false;
          g_UserSettings.analysisMenuSelection = ANALYSIS_MENU_NEW;
          g_UserSettings.profileActionSelection = PROFILE_ACTION_PERFORMANCE;
          updateSystemState(STATE_PROFILE_ACTIONS);
        }
        else if(g_ProfileNotice == PROFILE_NOTICE_LOADED)
        {
          // Loading is the one profile action that returns ready to run.
          g_UserSettings.stoppedScreenSelection = STOPPED_SCREEN_TIME;
          returnToStoppedScreen();
        }
        else if(g_ProfileNotice == PROFILE_NOTICE_NO_DATA)
        {
          updateSystemState(STATE_PROFILE_ACTIONS);
        }
        else
        {
          updateSystemState(STATE_PROFILES);
        }
        break;
      }
      drawProfileNoticeScreen();
    }
    break;

    case STATE_ANALYSIS_MENU:
    {
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        if(g_UserSettings.analysisMenuSelection == ANALYSIS_MENU_NEW)
        {
          updateSystemState(STATE_ANALYSIS_LOAD);
        }
        else if(g_UserSettings.analysisMenuSelection == ANALYSIS_MENU_REVIEW)
        {
          showSavedAnalysis();
        }
        else if(g_UserSettings.analysisMenuSelection == ANALYSIS_MENU_CONFIG)
        {
          beginAnalysisConfig();
        }
        else
        {
          returnToStoppedScreen();
        }
        break;
      }
      drawAnalysisMenuScreen();
    }
    break;

    case STATE_ANALYSIS_CONFIG:
    {
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        updateAnalysisConfig(rapidTimeAdjust);
        if(g_SystemState != STATE_ANALYSIS_CONFIG)
        {
          break;
        }
      }
      drawAnalysisConfigScreen();
    }
    break;

    case STATE_ANALYSIS_LOAD:
    {
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        updateSystemState(g_Analysis.graphValid && g_Analysis.graphIsAnalysis ?
                          STATE_ANALYSIS_MENU : STATE_STOPPED);
        break;
      }
      drawAnalysisLoadScreen();
    }
    break;

    case STATE_ANALYSING:
    {
      uint32_t const currentTime = millis();
      updateSystemState(g_SystemState);
      sampleAnalysisCurrent(currentTime);
      if(g_SystemState != STATE_ANALYSING)
      {
        break;
      }
      if(hasTimeElapsed(g_Analysis.startTime + ANALYSIS_DURATION_MS, currentTime))
      {
        finishAnalysis(false);
        openAnalysisDropGate();
        break;
      }
      if(hasTimeElapsed(g_Analysis.lastGraphDrawTime + ANALYSIS_GRAPH_REFRESH_MS, currentTime))
      {
        drawAnalysisGraph();
        g_Analysis.lastGraphDrawTime = currentTime;
      }
    }
    break;

    case STATE_ANALYSIS_GATE_OPEN:
    {
      uint32_t const currentTime = millis();
      uint32_t const gateCloseTime = SystemTimeTarget;
      updateSystemState(g_SystemState);
      if(hasTimeElapsed(gateCloseTime, currentTime))
      {
        closeDropGate();
        updateSystemState(STATE_ANALYSIS_RESULT);
        break;
      }
      if(!hasTimeElapsed(gateCloseTime - (ANALYSIS_GATE_OPEN_PERIOD_MS - ANALYSIS_DUMP_STATUS_MS), currentTime))
      {
        drawAnalysisStatus(true);
      }
      else
      {
        drawAnalysisGraph();
      }
    }
    break;

    case STATE_ANALYSIS_RESULT:
    {
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        g_UserSettings.analysisMenuSelection = ANALYSIS_MENU_REVIEW;
        updateSystemState(STATE_ANALYSIS_MENU);
        break;
      }
      drawAnalysisStatus(false);
    }
    break;

    case STATE_ANNEALING:
    {
      uint32_t const currentTime = millis();
      bool stopConditionReached = false;
      if (hasSystemStateChanged())
      {
        setSystemTimeTarget(millis() + g_UserSettings.annealTime_ms);
        g_RunSafety.annealingCurrentTotal_ma = 0;
        g_RunSafety.annealingCurrentSamples = 0;
        g_RunSafety.restartCurrentTotal_ma = 0;
        g_RunSafety.restartCurrentSamples = 0;
        g_AdaptiveAnneal.energyCurrentSumTarget =
          ((uint32_t)g_UserSettings.targetEnergy_J * 2500UL) / 3UL;
        g_AdaptiveAnneal.peakCurrent_ma = 0;
        g_AdaptiveAnneal.belowPeakSamples = 0;
        g_CasePerformance.currentCycleCompared = false;
        #if NZHS_HAS_WIFI
        g_R4WifiHistoryCaptureActive =
          (g_R4WifiConfig.monitorEnabled ||
           g_R4WifiMode == R4_WIFI_DIRECT_MONITOR) &&
          CurrentSensorPresent &&
          !g_CasePerformance.referenceValid;
        if(g_R4WifiHistoryCaptureActive)
        {
          resetGraphCapture(currentTime, false);
        }
        #endif
        if(g_CasePerformance.referenceValid && CurrentSensorPresent)
        {
          beginCasePerformance(currentTime);
        }
        targetTimeoutPending = false;
        turnStartStopLedOn();
        turnAnnealerOn();
        cooling_timer = COOLDOWN_PERIOD + millis(); // 5 minute cooldown after last anneal
          if(CurrentMode == MODE_AUTOMATIC)
          {
            preloadCase();
          }
      }
      updateSystemState(g_SystemState);

      if(CurrentSensorPresent)
      {
        psuCurrent_ma = readPsuCurrent_ma();
        g_RunSafety.restartCurrentTotal_ma += psuCurrent_ma;
        g_RunSafety.restartCurrentSamples++;
        if(psuCurrent_ma >= PSU_OVERCURRENT) //overloaded the PSU - may damage the ZVS converter
        {
          turnAnnealerOff();
          turnStartStopLedOff();
          #if NZHS_HAS_WIFI
          if(g_CasePerformance.referenceValid)
            recordCasePerformance(psuCurrent_ma, currentTime);
          else if(g_R4WifiHistoryCaptureActive)
            recordGraphCurrent(psuCurrent_ma, currentTime);
          r4FinishAnnealHistory(WIFI_HISTORY_OVERCURRENT);
          #endif
          updateSystemState(STATE_OVERCURRENT_WARNING);
          break;
        }
        g_RunSafety.annealingCurrentTotal_ma += psuCurrent_ma;
        g_RunSafety.annealingCurrentSamples++;

        if(g_CasePerformance.referenceValid)
        {
          recordCasePerformance(psuCurrent_ma, currentTime);
        }
        #if NZHS_HAS_WIFI
        else if(g_R4WifiHistoryCaptureActive)
        {
          recordGraphCurrent(psuCurrent_ma, currentTime);
        }
        #endif

        if(g_UserSettings.stopType != PROFILE_STOP_TIME)
        {
          if(psuCurrent_ma > g_AdaptiveAnneal.peakCurrent_ma)
          {
            g_AdaptiveAnneal.peakCurrent_ma = psuCurrent_ma;
          }

          if(g_UserSettings.stopType == PROFILE_STOP_ENERGY)
          {
            stopConditionReached = g_RunSafety.restartCurrentTotal_ma >=
                                   g_AdaptiveAnneal.energyCurrentSumTarget;
          }
          else if(hasTimeElapsed(SystemTimeTarget - g_UserSettings.annealTime_ms + MIN_ANNEAL_TIME, currentTime) &&
                  g_AdaptiveAnneal.peakCurrent_ma > CURRENT_SENSOR_DETECTION_MA)
          {
            if((uint32_t)psuCurrent_ma * 100UL <=
               (uint32_t)g_AdaptiveAnneal.peakCurrent_ma * (100 - g_UserSettings.peakDropPercent))
            {
              if(g_AdaptiveAnneal.belowPeakSamples < ANALYSIS_PEAK_CONFIRM_SAMPLES)
              {
                g_AdaptiveAnneal.belowPeakSamples++;
              }
            }
            else
            {
              g_AdaptiveAnneal.belowPeakSamples = 0;
            }
            stopConditionReached = g_AdaptiveAnneal.belowPeakSamples >= ANALYSIS_PEAK_CONFIRM_SAMPLES;
          }
        }
      }

      uint32_t systemTimeTarget = SystemTimeTarget;
      bool const timeExpired = hasTimeElapsed(systemTimeTarget, currentTime);
      if(g_UserSettings.stopType == PROFILE_STOP_TIME)
      {
        stopConditionReached = timeExpired;
      }
      if(stopConditionReached || timeExpired)
      {
        turnAnnealerOff();
        if(g_UserSettings.stopType != PROFILE_STOP_TIME && !stopConditionReached)
        {
          targetTimeoutPending = true;
          Next_Cycle_Is_STOPPED = 1;
          g_RunSafety.cooldownRestartPending = false;
        }
        if(CurrentSensorPresent && g_RunSafety.restartCurrentSamples)
        {
          uint16_t restartCycleAverageCurrent_ma = g_RunSafety.restartCurrentTotal_ma / g_RunSafety.restartCurrentSamples;
          if(g_RunSafety.completedAnnealCycles >= LOW_CURRENT_IGNORED_CYCLES &&
             restartCycleAverageCurrent_ma <= CURRENT_SENSOR_DETECTION_MA)
          {
            // The first case is a deliberate leap of faith while the system
            // settles. After it, a cycle below 0.1 A means no usable annealing
            // current: stop before opening the gate and prevent a future
            // unattended cooldown restart. This check is only meaningful
            // when setup detected a sensor at A0.
            g_UserSettings.autoRestartAfterCooldown = false;
            EEPROM.update(EEPROM_ADDRESS_AUTO_RESTART, 0);
            g_RunSafety.cooldownRestartPending = false;
            turnStartStopLedOff();
            #if NZHS_HAS_WIFI
            r4FinishAnnealHistory(WIFI_HISTORY_LOW_CURRENT);
            #endif
            updateSystemState(STATE_LOW_CURRENT_WARNING);
            break;
          }
        }

        if(CurrentSensorPresent && g_RunSafety.annealingCurrentSamples)
        {
          uint16_t cycleAverageCurrent_ma = g_RunSafety.annealingCurrentTotal_ma / g_RunSafety.annealingCurrentSamples;
          if(lowCurrentGuardFault(cycleAverageCurrent_ma))
          {
            turnStartStopLedOff();
            #if NZHS_HAS_WIFI
            r4FinishAnnealHistory(WIFI_HISTORY_LOW_CURRENT);
            #endif
            updateSystemState(STATE_LOW_CURRENT_WARNING);
            break;
          }
        }
        #if NZHS_HAS_WIFI
        tWifiHistoryReason historyReason = WIFI_HISTORY_TIME;
        if(targetTimeoutPending) historyReason = WIFI_HISTORY_TIMEOUT;
        else if(g_UserSettings.stopType == PROFILE_STOP_ENERGY)
          historyReason = WIFI_HISTORY_ENERGY;
        else if(g_UserSettings.stopType == PROFILE_STOP_PEAK_DROP)
          historyReason = WIFI_HISTORY_PEAK_DROP;
        r4FinishAnnealHistory(historyReason);
        #else
          if(g_CasePerformance.referenceValid && CurrentSensorPresent)
          {
            finishCasePerformance();
          }
        #endif
        g_RunSafety.completedAnnealCycles++;
        openDropGate();
        updateSystemState(STATE_DROPPING);
        // Do not render the annealing countdown after expiry.
        break;
      }

      if(!hasTimeElapsed(systemTimeTarget, currentTime))
      {
        if(g_CasePerformance.referenceValid && CurrentSensorPresent)
        {
          if(hasTimeElapsed(g_Analysis.lastGraphDrawTime +
                            ANALYSIS_GRAPH_REFRESH_MS, currentTime))
          {
            drawCasePerformanceGraph(PERFORMANCE_FOOTER_LIVE, 0);
            g_Analysis.lastGraphDrawTime = currentTime;
          }
          break;
        }
        uint32_t remainingTime = systemTimeTarget - currentTime;
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.println(F("ANNEALING"));
        if(CurrentSensorPresent)
        {
          display.print(psuCurrent_ma/1000,DEC);
          display.write('.');
          display.print((psuCurrent_ma%1000)/100, DEC);
          display.print(F("A  "));
        }
        display.print(remainingTime/1000, DEC);
        display.write('.');
        display.print((remainingTime%1000)/100, DEC);
        display.write('s');
        display.display();
      }


    }
    break;

    case STATE_DROPPING:
    {
      if (hasSystemStateChanged())
      {
        if(caseFeederStillMoving()) //case feeder is still moving so wait until it's finished moving before starting the drop sequence
        {
          break;
        }
        setSystemTimeTarget(millis() + DROP_TIME);
      }
      updateSystemState(g_SystemState);

      if (!hasTimeElapsed(SystemTimeTarget, millis())) // wait time is not up, break.
      {
        if(g_CasePerformance.currentCycleCompared)
        {
          drawCasePerformanceGraph(PERFORMANCE_FOOTER_DROP, 0);
          break;
        }
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.println(F("DROPPING"));
        if(NumberDallasTempDevices != 0)
        {
          display.setCursor(0, 16);
          printTemperatureTenths(temperature);
          display.print((char PROGMEM)248);
          display.write('C');
        }
        display.display();
        break;
      }
      closeDropGate();
      CasesAnnealed++;

      if(targetTimeoutPending)
      {
        targetTimeoutPending = false;
        Next_Cycle_Is_STOPPED = 0;
        if(NumberDallasTempDevices != 0 && temperature > (TEMP_LIMIT * TEMP_RAW_SCALE))
        {
          g_RunSafety.cooldownLockActive = true;
        }
        if(CurrentMode == MODE_AUTOMATIC)
        {
          returnCaseFeederHome();
        }
        updateSystemState(STATE_TARGET_TIMEOUT_WARNING);
      }
      else if(NumberDallasTempDevices != 0 && temperature > (TEMP_LIMIT * TEMP_RAW_SCALE))
      {
        enterCooldown(true, Next_Cycle_Is_STOPPED);
        if(CurrentMode == MODE_AUTOMATIC)
        {
          returnCaseFeederHome();
        }
      }
      else if(Next_Cycle_Is_STOPPED)
      {
        updateSystemState(STATE_STOPPED);
        Next_Cycle_Is_STOPPED = 0;
        if(CurrentMode == MODE_AUTOMATIC)
        {
          returnCaseFeederHome();
        }
      }
      else if(CurrentMode == MODE_SINGLE_SHOT) //modestate bit will determine if we free run or go to stopped state
      {
        updateSystemState(STATE_STOPPED);
      }
      else
      {
        updateSystemState(STATE_RELOADING);
      }
    }
    break;

    case STATE_PRELOAD:
    {
      if (hasSystemStateChanged())
      {
        preloadCase(); //pick up first case after start button is pressed
      }
      updateSystemState(g_SystemState);
      if(caseFeederStillMoving()) //case feeder is still moving so wait until it's finished moving before starting the drop sequence
        {
          break;
        }
      updateSystemState(STATE_RELOADING);
    }
    break;

    case STATE_RELOADING:
    {
      if (hasSystemStateChanged())
      {
        setSystemTimeTarget(millis() + RELOAD_TIME); //load time to fit new case
        if(CurrentMode == MODE_AUTOMATIC)
        	{
        		loadCase();
            setSystemTimeTarget(millis() + RELOAD_TIME_AUTO__FEED); //load time when in auto feed mode
        	}
      }
      updateSystemState(g_SystemState);


      uint32_t systemTimeTarget = SystemTimeTarget;
      uint32_t currentTime = millis();
      if (!hasTimeElapsed(systemTimeTarget, currentTime))
      {
        uint32_t remainingTime = systemTimeTarget - currentTime;
        if(CurrentMode == MODE_AUTOMATIC &&
           g_CasePerformance.currentCycleCompared)
        {
          drawCasePerformanceGraph(PERFORMANCE_FOOTER_NEXT,
                                   remainingTime > UINT16_MAX ? UINT16_MAX : remainingTime);
          break;
        }
        display.clearDisplay();
        display.setCursor(0, 0);
        #ifdef SHOW_CASE_COUNT
          display.println(F("LOAD"));
        #else
          display.println(F("LOADING"));
        #endif
        display.print(remainingTime/1000, DEC);
        display.write('.');
        display.print((remainingTime%1000)/100, DEC);
        display.write('s');

        #ifdef SHOW_CASE_COUNT
          display.setCursor(65, 0);
          display.print(F("CASES"));
          display.setCursor(65, 16);
          display.print(CasesAnnealed, 1);
          display.drawFastVLine(57,0,32,WHITE);
        #endif

        display.display();
        break;
      }
      updateSystemState(STATE_ANNEALING);
    }
    break;

    case STATE_SHOW_WARNING:
    {
      updateSystemState(g_SystemState);

      display.clearDisplay();
      display.setCursor(0, 0);
      display.print(F("TIME"));
      display.setTextSize(1);
      display.print(F(" & "));
      display.setTextSize(2);
      display.println(F("CASE"));
      display.println(F("HEIGHT OK?"));
      display.display();

    }
    break;

    case STATE_DIAGNOSTICS:
    {
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        returnToStoppedScreen();
        break;
      }
      drawDiagnosticsScreen();
    }
    break;

    case STATE_INFO:
    {
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        returnToStoppedScreen();
        break;
      }
      drawInfoScreen();
    }
    break;

    case STATE_OVERCURRENT_WARNING:
    {
      updateSystemState(g_SystemState);
      drawFaultScreen(F("! FAULT !"), F("CHECK COIL"));
    }
    break;

    case STATE_LOW_CURRENT_WARNING:
    {
      updateSystemState(g_SystemState);
      drawFaultScreen(F("CHECK CASE"), F("CURRENT LO!"));
    }
    break;

    case STATE_TEMPERATURE_SENSOR_WARNING:
    {
      updateSystemState(g_SystemState);
      drawFaultScreen(F("TEMP ERROR"), F("CHECK TEMP"));
    }
    break;

    case STATE_TARGET_TIMEOUT_WARNING:
    {
      updateSystemState(g_SystemState);
      drawFaultScreen(F("TARGET"), F("TIMEOUT"));
    }
    break;

    case STATE_CURRENT_SENSOR_REQUIRED:
    {
      updateSystemState(g_SystemState);
      drawFaultScreen(F("CUR SENSOR"), F("REQUIRED"));
    }
    break;

    case STATE_COOLDOWN:
    {
      if (hasSystemStateChanged())
      {
        setSystemTimeTarget(millis() + TEMP_CONVERSION_TIME); //time for temp conversion
        turnStartStopLedOff();
      }
      updateSystemState(g_SystemState);

      cooling_timer = COOLDOWN_PERIOD + millis(); //keep resetting fan timer while in cooldown mode
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println(F("COOLDOWN"));
      display.setCursor(0, 16);
      printTemperatureTenths(temperature);
      display.print((char PROGMEM)248);
      display.write('C');
      display.setTextSize(1);
      if(g_RunSafety.cooldownRestartPending)
      {
        display.setCursor(86, 24);
        display.print(F("AUTO ON"));
      }
      else
      {
        display.setCursor(80, 24);
        display.print(F("AUTO OFF"));
      }
      display.display();
      if(NumberDallasTempDevices != 0 &&
         isTemperatureReadingValid(temperature) &&
         temperature < ((TEMP_LIMIT - TEMP_HYSTERESIS) * TEMP_RAW_SCALE)) //has it cooled enough to resume?
      {
        if(g_RunSafety.cooldownRestartPending)
        {
          g_RunSafety.cooldownRestartPending = false;
          if(CurrentMode == MODE_AUTOMATIC)
          {
            updateSystemState(STATE_ANNEALING);
          }
          else
          {
            updateSystemState(STATE_RELOADING);
          }
        }
        else
        {
          Next_Cycle_Is_STOPPED = 0;
          updateSystemState(STATE_STOPPED);
        }
      }

    }
    break;
    case STATE_JUST_BOOTED:
    {
      //temperature = sensors.getTempCByIndex(0);
      if(NumberDallasTempDevices != 0 && temperature > (TEMP_LIMIT * TEMP_RAW_SCALE))
      {
        enterCooldown(false, false);
      }
      else
      {
        updateSystemState(STATE_STOPPED);
        setSystemTimeTarget(millis() + TEMP_CONVERSION_TIME); //time for temp conversion
      }
    }
    break;
    #if NZHS_PLATFORM_UNO_R4
    case STATE_PLATFORM_WARNING:
    {
      updateSystemState(g_SystemState);
      turnAnnealerOff();
      turnStartStopLedOff();
      drawFaultScreen(F("R4 HW"), F("INIT ERR"));
    }
    break;
    #endif
    default:
    {
      updateSystemState(g_SystemState);
      updateSystemState(STATE_STOPPED);
    }
  }
  startPrev = start;
  modeKeyPrev = modeKey;
  upKeyPrev = upKey;

  if(g_SystemState == STATE_TEMPERATURE_SENSOR_WARNING)
  {
    // Keep cooling until a valid temperature reading is acknowledged.
    turnCoolingFanOn();
    FanIsOn=true;
  }
  else if(!hasTimeElapsed(cooling_timer, millis()))
  {
    turnCoolingFanOn();
    FanIsOn=true;
  }
  else
  {
    turnCoolingFanOff();
    FanIsOn=false;
  }

  #if NZHS_HAS_LED_MATRIX
  r4UpdateMatrixDebug(temperature);
  #endif
  #if NZHS_HAS_WIFI
  r4UpdateWifiMonitor(temperature, psuCurrent_ma, CasesAnnealed, FanIsOn);
  #endif

  #ifdef DEBUG

  Serial.print(F("Annealer current;"));
  Serial.print(psuCurrent_ma/1000,DEC);
  Serial.write('.');
  Serial.print((psuCurrent_ma%1000)/100, DEC);
  Serial.print(F(";A;"));

  Serial.print(F("Anneal Time;"));
  Serial.print(g_UserSettings.annealTime_ms);
  Serial.print(F(";ms;"));

  Serial.print("Step count;");
  Serial.print(getStepsToGo());
  Serial.write(';');

  Serial.print("Steps from home;");
  Serial.print(getStepsFromHome());
  Serial.write(';');

  Serial.print(F("State;"));
  Serial.print(g_SystemState);
  Serial.write(';');

  Serial.print(F("Loop Time Remaining;"));
  Serial.print(LoopStartTime + LOOP_TIME - millis());
  Serial.println(F(";ms;"));


  #endif


  bool const fastCurrentSampling = g_SystemState == STATE_ANALYSING ||
                                   (g_SystemState == STATE_ANNEALING &&
                                    (g_UserSettings.stopType != PROFILE_STOP_TIME ||
                                     (g_CasePerformance.referenceValid &&
                                      CurrentSensorPresent)
                                     #if NZHS_HAS_WIFI
                                     || ((g_R4WifiConfig.monitorEnabled ||
                                          g_R4WifiMode == R4_WIFI_DIRECT_MONITOR) &&
                                         CurrentSensorPresent)
                                     #endif
                                     ));
  uint32_t const loopPeriod = fastCurrentSampling ? ANALYSIS_SAMPLE_PERIOD_MS : LOOP_TIME;
  while(!hasTimeElapsed(LoopStartTime + loopPeriod, millis())) // wait for the loop time to expire
  {

      if(((millis() & 0x00003FFF) == 0x00003FFF) && (annealTimeChanged == true)) // write to EEPROM every ~16 seconds only if anneal time has changed
      {
        EEPROM.update(EEPROM_ADDRESS_ANNEAL_TIME,g_UserSettings.annealTime_ms/100);
        annealTimeChanged = false;
      }
  }

}

/*---------------------------------------------------------------------------*/
/*! @brief      Load EEPROM-backed settings after hardware detection.
*//*-------------------------------------------------------------------------*/
static void loadUserSettings(void)
{
  uint16_t savedAnnealTime_ms = EEPROM.read(EEPROM_ADDRESS_ANNEAL_TIME) * 100;
  if(savedAnnealTime_ms < MIN_ANNEAL_TIME || savedAnnealTime_ms > MAX_ANNEAL_TIME)
  {
    g_UserSettings.annealTime_ms = MIN_ANNEAL_TIME;
    EEPROM.update(EEPROM_ADDRESS_ANNEAL_TIME, MIN_ANNEAL_TIME / 100);
  }
  else
  {
    g_UserSettings.annealTime_ms = savedAnnealTime_ms;
  }
  g_UserSettings.autoRestartAfterCooldown = EEPROM.read(EEPROM_ADDRESS_AUTO_RESTART) == 1;
  g_UserSettings.dumpButtonEnabled = EEPROM.read(EEPROM_ADDRESS_DUMP_BUTTON) == 1;
  g_UserSettings.stoppedScreenSelection = STOPPED_SCREEN_TIME;
  g_UserSettings.settingsScreenSelection = SETTINGS_SCREEN_AUTO_RESTART;
  g_UserSettings.analysisMenuSelection = ANALYSIS_MENU_NEW;
  g_UserSettings.profileSlot = 0;
  g_UserSettings.profileActionSelection = PROFILE_ACTION_LOAD;
  g_UserSettings.profileNameCursor = 0;
  g_UserSettings.profileDeleteConfirmed = false;
  g_UserSettings.stopType = PROFILE_STOP_TIME;
  g_UserSettings.targetEnergy_J = 0;
  g_UserSettings.peakDropPercent = ANALYSIS_DEFAULT_PEAK_DROP_PERCENT;
  g_AnalysisConfig.profileSaveInProgress = false;
  if(g_UserSettings.dumpButtonEnabled)
  {
    // Manual case handling must not resume unattended after cooldown.
    setDumpButtonEnabled(true);
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Reset learned current and automatic-restart state for a new run.
*//*-------------------------------------------------------------------------*/
static void resetRunSafetyState(void)
{
  g_RunSafety.cooldownRestartPending = false;
  g_RunSafety.cooldownLockActive = false;
  g_RunSafety.annealingCurrentTotal_ma = 0;
  g_RunSafety.annealingCurrentSamples = 0;
  g_RunSafety.restartCurrentTotal_ma = 0;
  g_RunSafety.restartCurrentSamples = 0;
  g_RunSafety.completedAnnealCycles = 0;
  g_RunSafety.baselineCurrent_ma = 0;
  g_RunSafety.ignoredCurrentCycles = 0;
  g_RunSafety.baselineCurrentCycles = 0;
  g_RunSafety.baselineCurrentWindowIndex = 0;
  g_RunSafety.lowCurrentConsecutiveCycles = 0;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Enter cooldown and decide whether this cooldown may restart.
*//*-------------------------------------------------------------------------*/
static void enterCooldown(bool const allowAutomaticRestart, bool const cycleStopRequested)
{
  g_RunSafety.cooldownLockActive = true;
  g_RunSafety.cooldownRestartPending = allowAutomaticRestart &&
                                       g_UserSettings.autoRestartAfterCooldown &&
                                       CurrentMode != MODE_SINGLE_SHOT &&
                                       !cycleStopRequested;
  updateSystemState(STATE_COOLDOWN);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Return the EEPROM address of a fixed-size cartridge profile.
*//*-------------------------------------------------------------------------*/
static uint16_t getProfileAddress(uint8_t const slot)
{
  return EEPROM_ADDRESS_PROFILE_BASE + ((uint16_t)slot * sizeof(tCartridgeProfile));
}

/*---------------------------------------------------------------------------*/
/*! @brief      Calculate the lightweight integrity check for one profile.
*//*-------------------------------------------------------------------------*/
static uint8_t calculateProfileChecksum(tCartridgeProfile const * const profile)
{
  uint8_t checksum = 0;
  uint8_t index;
  uint8_t const * const bytes = (uint8_t const *)profile;
  for(index = 1; index < sizeof(tCartridgeProfile) - 1; index++)
  {
    checksum ^= bytes[index];
  }
  return checksum;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Load a valid profile; blank or interrupted records are rejected.
*//*-------------------------------------------------------------------------*/
static bool loadProfile(uint8_t const slot, tCartridgeProfile * const profile)
{
  if(slot >= PROFILE_COUNT)
  {
    return false;
  }
  EEPROM.get(getProfileAddress(slot), *profile);
  return profile->magic == PROFILE_MAGIC &&
         profile->checksum == calculateProfileChecksum(profile) &&
         profile->annealTime_ms >= MIN_ANNEAL_TIME &&
         profile->annealTime_ms <= MAX_ANNEAL_TIME &&
         profile->mode <= MODE_AUTOMATIC;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Persist a profile with its valid marker written last.
*//*-------------------------------------------------------------------------*/
static void saveProfile(uint8_t const slot, tCartridgeProfile const * const profile)
{
  #if NZHS_PLATFORM_UNO_R4
  tCartridgeProfile savedProfile = *profile;
  savedProfile.magic = PROFILE_MAGIC;
  savedProfile.checksum = calculateProfileChecksum(&savedProfile);
  // Renesas EEPROM is virtual data flash; put() batches this record instead
  // of committing one flash transaction per changed byte.
  EEPROM.put(getProfileAddress(slot), savedProfile);
  #else
  uint8_t index;
  uint16_t address = getProfileAddress(slot);
  tCartridgeProfile savedProfile = *profile;
  uint8_t const * bytes;

  savedProfile.magic = PROFILE_MAGIC;
  savedProfile.checksum = calculateProfileChecksum(&savedProfile);
  EEPROM.update(address, 0);
  bytes = (uint8_t const *)&savedProfile;
  for(index = 1; index < sizeof(tCartridgeProfile); index++)
  {
    EEPROM.update(address + index, bytes[index]);
  }
  EEPROM.update(address, PROFILE_MAGIC);
  #endif
}

/*---------------------------------------------------------------------------*/
/*! @brief      Load the optional three-byte analysis rule for a profile slot.
*//*-------------------------------------------------------------------------*/
static void loadProfileStopRule(uint8_t const slot)
{
  uint16_t const address = EEPROM_ADDRESS_PROFILE_RULE_BASE + ((uint16_t)slot * 3);
  g_UserSettings.targetEnergy_J = EEPROM.read(address) | ((uint16_t)EEPROM.read(address + 1) << 8);
  g_UserSettings.peakDropPercent = EEPROM.read(address + 2);
  if((g_UserSettings.stopType == PROFILE_STOP_ENERGY &&
      (g_UserSettings.targetEnergy_J == 0 || g_UserSettings.targetEnergy_J > ANALYSIS_MAX_ENERGY_J)) ||
     (g_UserSettings.stopType == PROFILE_STOP_PEAK_DROP &&
      (g_UserSettings.peakDropPercent == 0 || g_UserSettings.peakDropPercent >= 100)))
  {
    g_UserSettings.stopType = PROFILE_STOP_TIME;
    g_UserSettings.targetEnergy_J = 0;
    g_UserSettings.peakDropPercent = ANALYSIS_DEFAULT_PEAK_DROP_PERCENT;
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Persist a compact stop rule alongside an unchanged profile.
*//*-------------------------------------------------------------------------*/
static void saveProfileStopRule(uint8_t const slot, uint16_t const targetEnergy_J,
                                uint8_t const peakDropPercent)
{
  uint16_t const address = EEPROM_ADDRESS_PROFILE_RULE_BASE + ((uint16_t)slot * 3);
  #if NZHS_PLATFORM_UNO_R4
  uint8_t const rule[3] = {
    (uint8_t)(targetEnergy_J & 0xFF),
    (uint8_t)(targetEnergy_J >> 8),
    peakDropPercent
  };
  EEPROM.put(address, rule);
  #else
  EEPROM.update(address, targetEnergy_J & 0xFF);
  EEPROM.update(address + 1, targetEnergy_J >> 8);
  EEPROM.update(address + 2, peakDropPercent);
  #endif
}

/*---------------------------------------------------------------------------*/
/*! @brief      Return the EEPROM address of a profile's compact reference.
*//*-------------------------------------------------------------------------*/
static uint16_t getProfileReferenceAddress(uint8_t const slot)
{
  return EEPROM_ADDRESS_PROFILE_REFERENCE_BASE +
         ((uint16_t)slot * PROFILE_REFERENCE_RECORD_SIZE);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Read one 125 ms reference-current sample in 50 mA units.
*//*-------------------------------------------------------------------------*/
static uint8_t readProfileReferenceSample(uint8_t const slot, uint8_t const sample)
{
  return EEPROM.read(getProfileReferenceAddress(slot) +
                     PROFILE_REFERENCE_SAMPLE_OFFSET + sample);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Validate a reference marker, checksum and aggregate fields.
*//*-------------------------------------------------------------------------*/
static bool isProfileReferenceValid(uint8_t const slot)
{
  uint8_t checksum = 0;
  uint16_t const address = getProfileReferenceAddress(slot);
  if(slot >= PROFILE_COUNT || EEPROM.read(address) != PROFILE_REFERENCE_MAGIC)
  {
    return false;
  }
  for(uint8_t offset = 1; offset < PROFILE_REFERENCE_CHECKSUM_OFFSET; offset++)
  {
    uint8_t const value = EEPROM.read(address + offset);
    if(offset < PROFILE_REFERENCE_PEAK_OFFSET && value > ANALYSIS_GRAPH_MAX_SAMPLE)
    {
      return false;
    }
    checksum ^= value;
  }
  if(checksum != EEPROM.read(address + PROFILE_REFERENCE_CHECKSUM_OFFSET))
  {
    return false;
  }
  uint16_t const peakCurrent_ma = EEPROM.read(address + PROFILE_REFERENCE_PEAK_OFFSET) |
    ((uint16_t)EEPROM.read(address + PROFILE_REFERENCE_PEAK_OFFSET + 1) << 8);
  uint16_t const energy_J = EEPROM.read(address + PROFILE_REFERENCE_ENERGY_OFFSET) |
    ((uint16_t)EEPROM.read(address + PROFILE_REFERENCE_ENERGY_OFFSET + 1) << 8);
  uint16_t const duration_ms = EEPROM.read(address + PROFILE_REFERENCE_DURATION_OFFSET) |
    ((uint16_t)EEPROM.read(address + PROFILE_REFERENCE_DURATION_OFFSET + 1) << 8);
  return peakCurrent_ma <= ANALYSIS_GRAPH_MAX_CURRENT_MA &&
         energy_J <= ANALYSIS_MAX_ENERGY_J &&
         duration_ms > 0 && duration_ms <= ANALYSIS_DURATION_MS;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Select the EEPROM reference belonging to a loaded profile.
*//*-------------------------------------------------------------------------*/
static void activateProfileReference(uint8_t const slot)
{
  g_CasePerformance.referenceValid = isProfileReferenceValid(slot);
  if(g_CasePerformance.referenceValid)
  {
    g_CasePerformance.referenceSlot = slot;
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Save the retained Analyse curve as 64 current samples.
*//*-------------------------------------------------------------------------*/
static void saveProfileReference(uint8_t const slot)
{
  #if NZHS_PLATFORM_UNO_R4
  if(slot >= PROFILE_COUNT || !g_Analysis.graphValid || !g_Analysis.graphIsAnalysis)
  {
    return;
  }
  uint8_t record[PROFILE_REFERENCE_RECORD_SIZE] = {};
  for(uint8_t sample = 0; sample < PROFILE_REFERENCE_SAMPLE_COUNT; sample++)
  {
    uint8_t const firstColumn = sample * 2;
    uint16_t sampleTotal = 0;
    uint8_t sampleCount = 0;
    if(firstColumn <= g_Analysis.graphColumn)
    {
      sampleTotal = g_Analysis.graphSamples[firstColumn];
      sampleCount = 1;
    }
    if(firstColumn + 1 <= g_Analysis.graphColumn)
    {
      sampleTotal += g_Analysis.graphSamples[firstColumn + 1];
      sampleCount++;
    }
    record[PROFILE_REFERENCE_SAMPLE_OFFSET + sample] = sampleCount ?
      (sampleTotal + (sampleCount / 2)) / sampleCount : 0;
  }
  uint16_t const energy_J = (g_Analysis.inputEnergy_mJ + 500UL) / 1000UL;
  uint16_t const values[] = {
    g_Analysis.peakCurrent_ma,
    energy_J,
    g_Analysis.elapsedTime_ms
  };
  uint8_t const offsets[] = {
    PROFILE_REFERENCE_PEAK_OFFSET,
    PROFILE_REFERENCE_ENERGY_OFFSET,
    PROFILE_REFERENCE_DURATION_OFFSET
  };
  for(uint8_t value = 0; value < 3; value++)
  {
    record[offsets[value]] = values[value] & 0xFF;
    record[offsets[value] + 1] = values[value] >> 8;
  }
  uint8_t checksum = 0;
  for(uint8_t offset = 1; offset < PROFILE_REFERENCE_CHECKSUM_OFFSET; offset++)
  {
    checksum ^= record[offset];
  }
  record[0] = PROFILE_REFERENCE_MAGIC;
  record[PROFILE_REFERENCE_CHECKSUM_OFFSET] = checksum;
  EEPROM.put(getProfileReferenceAddress(slot), record);
  #else
  uint8_t checksum = 0;
  uint16_t const address = getProfileReferenceAddress(slot);
  if(slot >= PROFILE_COUNT || !g_Analysis.graphValid || !g_Analysis.graphIsAnalysis)
  {
    return;
  }

  // Invalidate first so interrupted writes cannot create a usable reference.
  EEPROM.update(address, 0);
  for(uint8_t sample = 0; sample < PROFILE_REFERENCE_SAMPLE_COUNT; sample++)
  {
    uint8_t const firstColumn = sample * 2;
    uint16_t sampleTotal = 0;
    uint8_t sampleCount = 0;
    if(firstColumn <= g_Analysis.graphColumn)
    {
      sampleTotal = g_Analysis.graphSamples[firstColumn];
      sampleCount = 1;
    }
    if(firstColumn + 1 <= g_Analysis.graphColumn)
    {
      sampleTotal += g_Analysis.graphSamples[firstColumn + 1];
      sampleCount++;
    }
    uint8_t const value = sampleCount ? (sampleTotal + (sampleCount / 2)) / sampleCount : 0;
    EEPROM.update(address + PROFILE_REFERENCE_SAMPLE_OFFSET + sample, value);
    checksum ^= value;
  }

  uint16_t const energy_J = (g_Analysis.inputEnergy_mJ + 500UL) / 1000UL;
  uint16_t const values[] = {
    g_Analysis.peakCurrent_ma,
    energy_J,
    g_Analysis.elapsedTime_ms
  };
  uint8_t const offsets[] = {
    PROFILE_REFERENCE_PEAK_OFFSET,
    PROFILE_REFERENCE_ENERGY_OFFSET,
    PROFILE_REFERENCE_DURATION_OFFSET
  };
  for(uint8_t value = 0; value < 3; value++)
  {
    uint8_t const lowByte = values[value] & 0xFF;
    uint8_t const highByte = values[value] >> 8;
    EEPROM.update(address + offsets[value], lowByte);
    EEPROM.update(address + offsets[value] + 1, highByte);
    checksum ^= lowByte;
    checksum ^= highByte;
  }
  EEPROM.update(address + PROFILE_REFERENCE_CHECKSUM_OFFSET, checksum);
  EEPROM.update(address, PROFILE_REFERENCE_MAGIC);
  #endif
}

/*---------------------------------------------------------------------------*/
/*! @brief      Invalidate one saved reference without wearing its data bytes.
*//*-------------------------------------------------------------------------*/
static void clearProfileReference(uint8_t const slot)
{
  EEPROM.update(getProfileReferenceAddress(slot), 0);
  if(g_CasePerformance.referenceValid && g_CasePerformance.referenceSlot == slot)
  {
    g_CasePerformance.referenceValid = false;
  }
  if(g_Analysis.graphValid && !g_Analysis.graphIsAnalysis &&
     g_CasePerformance.resultSlot == slot)
  {
    g_Analysis.graphValid = false;
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Mark a profile slot blank without touching neighbouring slots.
*//*-------------------------------------------------------------------------*/
static void clearProfile(uint8_t const slot)
{
  EEPROM.update(getProfileAddress(slot), 0);
  EEPROM.update(EEPROM_ADDRESS_PROFILE_RULE_BASE + ((uint16_t)slot * 3), 0xFF);
  clearProfileReference(slot);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Create a named snapshot of the active settings.
*//*-------------------------------------------------------------------------*/
static void makeDefaultProfile(uint8_t const slot, tCartridgeProfile * const profile)
{
  uint8_t index;
  char const defaultName[] = "PROFILE ";
  profile->magic = PROFILE_MAGIC;
  for(index = 0; index < PROFILE_NAME_LENGTH; index++)
  {
    profile->name[index] = ' ';
  }
  for(index = 0; index < sizeof(defaultName) - 1; index++)
  {
    profile->name[index] = defaultName[index];
  }
  profile->name[PROFILE_NAME_LENGTH - 1] = '1' + slot;
  profile->annealTime_ms = g_UserSettings.annealTime_ms;
  profile->mode = CurrentMode;
  profile->flags = (g_UserSettings.autoRestartAfterCooldown ? PROFILE_FLAG_AUTO_RESTART : 0) |
                   (g_UserSettings.dumpButtonEnabled ? PROFILE_FLAG_DUMP_BUTTON : 0) |
                   (g_UserSettings.stopType << PROFILE_FLAG_STOP_TYPE_SHIFT);
  profile->checksum = calculateProfileChecksum(profile);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Apply a profile and mirror its values into the legacy settings.
*//*-------------------------------------------------------------------------*/
static void applyProfile(tCartridgeProfile const * const profile)
{
  g_UserSettings.annealTime_ms = profile->annealTime_ms;
  g_UserSettings.dumpButtonEnabled = (profile->flags & PROFILE_FLAG_DUMP_BUTTON) != 0;
  g_UserSettings.autoRestartAfterCooldown =
    ((profile->flags & PROFILE_FLAG_AUTO_RESTART) != 0) && !g_UserSettings.dumpButtonEnabled;
  g_UserSettings.stopType = (tProfileStopType)((profile->flags & PROFILE_FLAG_STOP_TYPE_MASK) >> PROFILE_FLAG_STOP_TYPE_SHIFT);
  if(g_UserSettings.stopType >= PROFILE_STOP_TYPE_COUNT)
  {
    g_UserSettings.stopType = PROFILE_STOP_TIME;
  }
  loadProfileStopRule(g_UserSettings.profileSlot);
  activateProfileReference(g_UserSettings.profileSlot);
  CurrentMode = (ModeList)profile->mode;
  EEPROM.update(EEPROM_ADDRESS_ANNEAL_TIME, g_UserSettings.annealTime_ms / 100);
  EEPROM.update(EEPROM_ADDRESS_AUTO_RESTART, g_UserSettings.autoRestartAfterCooldown ? 1 : 0);
  EEPROM.update(EEPROM_ADDRESS_DUMP_BUTTON, g_UserSettings.dumpButtonEnabled ? 1 : 0);
  if(g_UserSettings.dumpButtonEnabled)
  {
    setFreeRunMode();
  }
  else if(CurrentMode == MODE_AUTOMATIC)
  {
    digitalWrite(g_FeederStepperEnPin, LOW);
    turnModeLedOn();
  }
  else if(CurrentMode == MODE_SINGLE_SHOT)
  {
    digitalWrite(g_FeederStepperEnPin, HIGH);
    turnModeLedOff();
  }
  else
  {
    setFreeRunMode();
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Save the active settings into the selected profile slot.
*//*-------------------------------------------------------------------------*/
static void saveCurrentSettingsToProfile(uint8_t const slot)
{
  tCartridgeProfile profile;
  if(!loadProfile(slot, &profile))
  {
    makeDefaultProfile(slot, &profile);
  }
  profile.annealTime_ms = g_UserSettings.annealTime_ms;
  profile.mode = CurrentMode;
  profile.flags = (g_UserSettings.autoRestartAfterCooldown ? PROFILE_FLAG_AUTO_RESTART : 0) |
                  (g_UserSettings.dumpButtonEnabled ? PROFILE_FLAG_DUMP_BUTTON : 0) |
                  (g_UserSettings.stopType << PROFILE_FLAG_STOP_TYPE_SHIFT);
  saveProfile(slot, &profile);
  saveProfileStopRule(slot, g_UserSettings.targetEnergy_J, g_UserSettings.peakDropPercent);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Open the fixed-width A-Z/0-9 name editor for a profile slot.
*//*-------------------------------------------------------------------------*/
static void beginProfileNameEdit(uint8_t const slot)
{
  if(!loadProfile(slot, &g_ProfileEditor))
  {
    makeDefaultProfile(slot, &g_ProfileEditor);
  }
  g_UserSettings.profileNameCursor = 0;
  updateSystemState(STATE_PROFILE_NAME_EDIT);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Cycle the selected profile-name character through a safe set.
*//*-------------------------------------------------------------------------*/
static void advanceProfileNameCharacter(void)
{
  char * character = &g_ProfileEditor.name[g_UserSettings.profileNameCursor];
  if(*character == ' ')
  {
    *character = 'A';
  }
  else if(*character >= 'A' && *character < 'Z')
  {
    (*character)++;
  }
  else if(*character == 'Z')
  {
    *character = '0';
  }
  else if(*character >= '0' && *character < '9')
  {
    (*character)++;
  }
  else if(*character == '9')
  {
    *character = '-';
  }
  else if(*character == '-')
  {
    *character = '.';
  }
  else
  {
    *character = ' ';
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Persist the edited name and current settings in one profile.
*//*-------------------------------------------------------------------------*/
static void saveEditedProfileName(void)
{
  if(!g_AnalysisConfig.profileSaveInProgress)
  {
    g_ProfileEditor.annealTime_ms = g_UserSettings.annealTime_ms;
    g_ProfileEditor.mode = CurrentMode;
    g_ProfileEditor.flags = (g_UserSettings.autoRestartAfterCooldown ? PROFILE_FLAG_AUTO_RESTART : 0) |
                           (g_UserSettings.dumpButtonEnabled ? PROFILE_FLAG_DUMP_BUTTON : 0) |
                           (g_UserSettings.stopType << PROFILE_FLAG_STOP_TYPE_SHIFT);
  }
  saveProfile(g_UserSettings.profileSlot, &g_ProfileEditor);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Move to the next stopped-screen item that is available.
*//*-------------------------------------------------------------------------*/
static void advanceStoppedScreenSelection(void)
{
  g_UserSettings.stoppedScreenSelection = (tStoppedScreenSelection)((g_UserSettings.stoppedScreenSelection + 1) % STOPPED_SCREEN_SELECTION_COUNT);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Move to the next Settings screen item.
*//*-------------------------------------------------------------------------*/
static void advanceSettingsScreenSelection(void)
{
  g_UserSettings.settingsScreenSelection = (tSettingsScreenSelection)((g_UserSettings.settingsScreenSelection + 1) % SETTINGS_SCREEN_SELECTION_COUNT);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Move to the next Analyse menu item.
*//*-------------------------------------------------------------------------*/
static void advanceAnalysisMenuSelection(void)
{
  g_UserSettings.analysisMenuSelection = (tAnalysisMenuSelection)((g_UserSettings.analysisMenuSelection + 1) % ANALYSIS_MENU_SELECTION_COUNT);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Move down the single scrolling analysis-config menu.
*//*-------------------------------------------------------------------------*/
static void advanceAnalysisConfigSelection(void)
{
  uint8_t selection = g_AnalysisConfig.selection + 1;
  if(g_AnalysisConfig.stopType != PROFILE_STOP_ENERGY &&
     selection >= ANALYSIS_CONFIG_ENERGY_DIGIT_2 &&
     selection <= ANALYSIS_CONFIG_ENERGY_DIGIT_4)
  {
    selection = ANALYSIS_CONFIG_MAX_TIME;
  }
  if(g_AnalysisConfig.stopType == PROFILE_STOP_TIME && selection == ANALYSIS_CONFIG_MAX_TIME)
  {
    selection = ANALYSIS_CONFIG_PROFILE;
  }
  if(selection >= ANALYSIS_CONFIG_SELECTION_COUNT)
  {
    selection = ANALYSIS_CONFIG_TYPE;
  }
  g_AnalysisConfig.selection = selection;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Select the next stored cartridge-profile slot.
*//*-------------------------------------------------------------------------*/
static void advanceProfileSlot(void)
{
  g_UserSettings.profileSlot = (g_UserSettings.profileSlot + 1) % (PROFILE_COUNT + 1);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Select the next visible action for the selected profile.
*//*-------------------------------------------------------------------------*/
static void advanceProfileActionSelection(void)
{
  g_UserSettings.profileActionSelection = (tProfileActionSelection)((g_UserSettings.profileActionSelection + 1) % PROFILE_ACTION_SELECTION_COUNT);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Leave a menu while preserving its originating home selection.
*//*-------------------------------------------------------------------------*/
static void returnToStoppedScreen(void)
{
  updateSystemState(STATE_STOPPED);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Enter Analyse directly until a completed graph is available.
*//*-------------------------------------------------------------------------*/
static void enterAnalysis(void)
{
  if(g_Analysis.graphValid && g_Analysis.graphIsAnalysis)
  {
    g_UserSettings.analysisMenuSelection = ANALYSIS_MENU_NEW;
    updateSystemState(STATE_ANALYSIS_MENU);
  }
  else
  {
    updateSystemState(STATE_ANALYSIS_LOAD);
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Start an editor seeded from the retained analysis and settings.
*//*-------------------------------------------------------------------------*/
static void beginAnalysisConfig(void)
{
  g_AnalysisConfig.stopType = PROFILE_STOP_TIME;
  g_AnalysisConfig.stopTime_ms = g_UserSettings.annealTime_ms;
  g_AnalysisConfig.maxTime_ms = MAX_ANNEAL_TIME;
  memset(g_AnalysisConfig.energyDigits, 0, sizeof(g_AnalysisConfig.energyDigits));
  g_AnalysisConfig.peakDropPercent = ANALYSIS_DEFAULT_PEAK_DROP_PERCENT;
  g_AnalysisConfig.selection = ANALYSIS_CONFIG_TYPE;
  g_AnalysisConfig.profileSlot = 0;
  g_AnalysisConfig.profileSaveInProgress = false;
  updateSystemState(STATE_ANALYSIS_CONFIG);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Apply UP to the selected analysis-config value or action.
*//*-------------------------------------------------------------------------*/
static inline void updateAnalysisConfig(bool const rapidTimeAdjust)
{
  uint8_t const selection = g_AnalysisConfig.selection;
  if(selection == ANALYSIS_CONFIG_TYPE)
  {
    g_AnalysisConfig.stopType = (tProfileStopType)((g_AnalysisConfig.stopType + 1) % PROFILE_STOP_TYPE_COUNT);
    return;
  }
  if(selection >= ANALYSIS_CONFIG_TARGET && selection <= ANALYSIS_CONFIG_ENERGY_DIGIT_4)
  {
    if(g_AnalysisConfig.stopType == PROFILE_STOP_ENERGY)
    {
      uint8_t * const digit = &g_AnalysisConfig.energyDigits[selection - ANALYSIS_CONFIG_TARGET];
      *digit = *digit >= 9 ? 0 : *digit + 1;
    }
    else if(g_AnalysisConfig.stopType == PROFILE_STOP_TIME)
    {
      g_AnalysisConfig.stopTime_ms = incrementTimeSetting(g_AnalysisConfig.stopTime_ms,
                                                         rapidTimeAdjust);
    }
    else
    {
      g_AnalysisConfig.peakDropPercent = g_AnalysisConfig.peakDropPercent >= 99 ?
        1 : g_AnalysisConfig.peakDropPercent + 1;
    }
    return;
  }
  if(selection == ANALYSIS_CONFIG_MAX_TIME)
  {
    g_AnalysisConfig.maxTime_ms = incrementTimeSetting(g_AnalysisConfig.maxTime_ms,
                                                       rapidTimeAdjust);
    return;
  }
  if(selection == ANALYSIS_CONFIG_PROFILE)
  {
    g_AnalysisConfig.profileSlot = (g_AnalysisConfig.profileSlot + 1) % PROFILE_COUNT;
    return;
  }
  if(selection == ANALYSIS_CONFIG_SAVE)
  {
    if(g_AnalysisConfig.stopType == PROFILE_STOP_ENERGY &&
       g_AnalysisConfig.energyDigits[0] == 0 && g_AnalysisConfig.energyDigits[1] == 0 &&
       g_AnalysisConfig.energyDigits[2] == 0 && g_AnalysisConfig.energyDigits[3] == 0)
    {
      g_AnalysisConfig.energyDigits[3] = 1;
    }
    saveAnalysisConfigToProfile(g_AnalysisConfig.profileSlot);
    return;
  }
  g_UserSettings.analysisMenuSelection = ANALYSIS_MENU_CONFIG;
  updateSystemState(STATE_ANALYSIS_MENU);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Store the stop rule and retained graph in a profile.
*//*-------------------------------------------------------------------------*/
static void saveAnalysisConfigToProfile(uint8_t const slot)
{
  tCartridgeProfile profile;
  uint16_t const targetEnergy_J = (g_AnalysisConfig.energyDigits[0] * 1000U) +
                                  (g_AnalysisConfig.energyDigits[1] * 100U) +
                                  (g_AnalysisConfig.energyDigits[2] * 10U) +
                                  g_AnalysisConfig.energyDigits[3];
  if(!loadProfile(slot, &profile))
  {
    makeDefaultProfile(slot, &profile);
  }
  profile.annealTime_ms = g_AnalysisConfig.stopType == PROFILE_STOP_TIME ?
    g_AnalysisConfig.stopTime_ms : g_AnalysisConfig.maxTime_ms;
  profile.flags = (profile.flags & ~PROFILE_FLAG_STOP_TYPE_MASK) |
                  (g_AnalysisConfig.stopType << PROFILE_FLAG_STOP_TYPE_SHIFT);
  saveProfile(slot, &profile);
  saveProfileStopRule(slot, targetEnergy_J, g_AnalysisConfig.peakDropPercent);
  saveProfileReference(slot);
  // Once committed, the reference belongs to the profile. Analyse no longer
  // presents a duplicate session copy through REVIEW or CONFIG.
  g_Analysis.graphValid = false;
  g_Analysis.graphIsAnalysis = false;
  g_UserSettings.profileSlot = slot;
  g_AnalysisConfig.profileSaveInProgress = true;
  // Analysis configuration uses the default PROFILE n name for an empty
  // slot. Renaming remains an explicit action in the Profiles menu.
  g_ProfileNotice = PROFILE_NOTICE_SAVED;
  updateSystemState(STATE_PROFILE_NOTICE);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Convert milliamps to the byte stored in a graph or reference.
*//*-------------------------------------------------------------------------*/
static uint8_t graphSampleFromCurrent(uint16_t const current_ma)
{
  return current_ma >= ANALYSIS_GRAPH_MAX_CURRENT_MA ?
    ANALYSIS_GRAPH_MAX_SAMPLE : current_ma / ANALYSIS_GRAPH_CURRENT_STEP_MA;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Convert a stored current sample to its 32-pixel graph height.
*//*-------------------------------------------------------------------------*/
static uint8_t graphHeightFromSample(uint8_t const sample)
{
  return ((uint16_t)sample * 31U) / ANALYSIS_GRAPH_MAX_SAMPLE;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Reset the shared graph buffer for Analyse or profile matching.
*//*-------------------------------------------------------------------------*/
static void resetGraphCapture(uint32_t const currentTime, bool const isAnalysis)
{
  g_Analysis.startTime = currentTime;
  g_Analysis.nextSampleTime = currentTime;
  g_Analysis.lastSampleTime = currentTime;
  g_Analysis.lastGraphDrawTime = currentTime;
  g_Analysis.inputEnergy_mJ = 0;
  g_Analysis.peakCurrent_ma = 0;
  g_Analysis.elapsedTime_ms = 0;
  g_Analysis.graphColumn = UINT8_MAX;
  g_Analysis.graphCurrentTotal_ma = 0;
  g_Analysis.graphCurrent_ma = 0;
  g_Analysis.graphCurrentSamples = 0;
  memset(g_Analysis.graphSamples, 0, sizeof(g_Analysis.graphSamples));
  g_Analysis.graphValid = false;
  g_Analysis.graphIsAnalysis = isAnalysis;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Add one current reading to the shared eight-second graph.
    @return     Milliseconds represented by this reading.
*//*-------------------------------------------------------------------------*/
static uint16_t recordGraphCurrent(uint16_t const current_ma, uint32_t const currentTime)
{
  uint16_t elapsed_ms = currentTime - g_Analysis.startTime;
  uint16_t const samplePeriod_ms = currentTime - g_Analysis.lastSampleTime;
  g_Analysis.lastSampleTime = currentTime;
  if(elapsed_ms >= ANALYSIS_DURATION_MS)
  {
    elapsed_ms = ANALYSIS_DURATION_MS - 1;
  }
  g_Analysis.inputEnergy_mJ +=
    (uint32_t)current_ma * samplePeriod_ms * ANALYSIS_SUPPLY_VOLTAGE_V / 1000UL;
  if(current_ma > g_Analysis.peakCurrent_ma)
  {
    g_Analysis.peakCurrent_ma = current_ma;
  }

  uint8_t column = ((uint32_t)elapsed_ms * ANALYSIS_GRAPH_COLUMNS) /
                   ANALYSIS_DURATION_MS;
  if(column >= ANALYSIS_GRAPH_COLUMNS)
  {
    column = ANALYSIS_GRAPH_COLUMNS - 1;
  }
  if(column != g_Analysis.graphColumn)
  {
    g_Analysis.graphColumn = column;
    g_Analysis.graphCurrentTotal_ma = current_ma;
    g_Analysis.graphCurrentSamples = 1;
  }
  else
  {
    g_Analysis.graphCurrentTotal_ma += current_ma;
    g_Analysis.graphCurrentSamples++;
  }
  g_Analysis.graphCurrent_ma =
    g_Analysis.graphCurrentTotal_ma / g_Analysis.graphCurrentSamples;
  g_Analysis.graphSamples[column] = graphSampleFromCurrent(g_Analysis.graphCurrent_ma);
  return samplePeriod_ms;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Initialise a manual, current-trace analysis run.
*//*-------------------------------------------------------------------------*/
static void beginAnalysis(void)
{
  uint32_t const currentTime = millis();

  display.clearDisplay();
  resetGraphCapture(currentTime, true);
  drawAnalysisGraph();
  setSystemTimeTarget(currentTime + ANALYSIS_DURATION_MS);
  digitalWrite(g_FeederStepperEnPin,HIGH);
  closeDropGate();
  turnStartStopLedOn();
  turnAnnealerOn();
  Serial.println(F("ANALYSE,START"));
  updateSystemState(STATE_ANALYSING);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Sample and stream the analysis trace at a fixed target rate.
*//*-------------------------------------------------------------------------*/
static void sampleAnalysisCurrent(uint32_t const currentTime)
{
  uint16_t current_ma;
  uint16_t elapsed_ms;

  if(!hasTimeElapsed(g_Analysis.nextSampleTime, currentTime))
  {
    return;
  }
  g_Analysis.nextSampleTime = currentTime + ANALYSIS_SAMPLE_PERIOD_MS;
  elapsed_ms = currentTime - g_Analysis.startTime;
  if(elapsed_ms >= ANALYSIS_DURATION_MS)
  {
    elapsed_ms = ANALYSIS_DURATION_MS - 1;
  }
  current_ma = readPsuCurrent_ma();
  recordGraphCurrent(current_ma, currentTime);

  Serial.print(F("ANALYSE,SAMPLE,t_ms="));
  Serial.print(elapsed_ms);
  Serial.print(F(",current_ma="));
  Serial.print(current_ma);
  Serial.print(FPSTR(TEXT_INPUT_ENERGY));
  printAnalysisEnergy_J(g_Analysis.inputEnergy_mJ);
  Serial.println();

  if(current_ma >= PSU_OVERCURRENT)
  {
    turnAnnealerOff();
    turnStartStopLedOff();
    #if NZHS_HAS_WIFI
    r4FinalizeGraphCapture();
    r4StoreWifiHistory(WIFI_HISTORY_OVERCURRENT, true, false);
    #endif
    updateSystemState(STATE_OVERCURRENT_WARNING);
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Start comparing a normal case with the loaded profile curve.
*//*-------------------------------------------------------------------------*/
static void beginCasePerformance(uint32_t const currentTime)
{
  resetGraphCapture(currentTime, false);
  g_CasePerformance.currentCycleCompared = false;
  g_CasePerformance.errorTotal = 0;
  g_CasePerformance.referenceTotal = 0;
  g_CasePerformance.referenceEnergy_mJ = 0;
  g_CasePerformance.referencePeakCurrent_ma = 0;
  g_CasePerformance.energyPercent = 0;
  g_CasePerformance.peakPercent = 0;
  g_CasePerformance.matchPercent = 0;
  g_CasePerformance.resultSlot = g_CasePerformance.referenceSlot;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Compare one normal annealing sample with the saved reference.
*//*-------------------------------------------------------------------------*/
static void recordCasePerformance(uint16_t const current_ma,
                                  uint32_t const currentTime)
{
  uint16_t const samplePeriod_ms = recordGraphCurrent(current_ma, currentTime);
  uint16_t elapsed_ms = currentTime - g_Analysis.startTime;
  if(elapsed_ms >= ANALYSIS_DURATION_MS)
  {
    elapsed_ms = ANALYSIS_DURATION_MS - 1;
  }
  uint8_t referenceSample = ((uint32_t)elapsed_ms *
                             PROFILE_REFERENCE_SAMPLE_COUNT) /
                            ANALYSIS_DURATION_MS;
  if(referenceSample >= PROFILE_REFERENCE_SAMPLE_COUNT)
  {
    referenceSample = PROFILE_REFERENCE_SAMPLE_COUNT - 1;
  }
  uint8_t const referenceValue =
    readProfileReferenceSample(g_CasePerformance.referenceSlot, referenceSample);
  uint8_t const actualValue = graphSampleFromCurrent(g_Analysis.graphCurrent_ma);
  uint8_t const difference = actualValue > referenceValue ?
    actualValue - referenceValue : referenceValue - actualValue;
  uint16_t const referenceCurrent_ma =
    (uint16_t)referenceValue * ANALYSIS_GRAPH_CURRENT_STEP_MA;

  g_CasePerformance.errorTotal += (uint32_t)difference * samplePeriod_ms;
  g_CasePerformance.referenceTotal +=
    (uint32_t)referenceValue * samplePeriod_ms;
  g_CasePerformance.referenceEnergy_mJ +=
    (uint32_t)referenceCurrent_ma * samplePeriod_ms *
    ANALYSIS_SUPPLY_VOLTAGE_V / 1000UL;
  if(referenceCurrent_ma > g_CasePerformance.referencePeakCurrent_ma)
  {
    g_CasePerformance.referencePeakCurrent_ma = referenceCurrent_ma;
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Finalise the comparison values retained for RESULT >.
*//*-------------------------------------------------------------------------*/
static void finishCasePerformance(void)
{
  g_Analysis.elapsedTime_ms = millis() - g_Analysis.startTime;
  if(g_Analysis.elapsedTime_ms > ANALYSIS_DURATION_MS)
  {
    g_Analysis.elapsedTime_ms = ANALYSIS_DURATION_MS;
  }
  g_Analysis.graphValid = g_Analysis.graphColumn != UINT8_MAX;
  g_Analysis.graphIsAnalysis = false;
  g_CasePerformance.currentCycleCompared = g_Analysis.graphValid;

  uint32_t errorPercent = g_CasePerformance.referenceTotal ?
    (g_CasePerformance.errorTotal * 100UL) /
    g_CasePerformance.referenceTotal : 100;
  if(errorPercent > 100)
  {
    errorPercent = 100;
  }
  g_CasePerformance.matchPercent = 100 - errorPercent;

  uint32_t ratio = g_CasePerformance.referenceEnergy_mJ ?
    (g_Analysis.inputEnergy_mJ * 100UL) /
    g_CasePerformance.referenceEnergy_mJ : 0;
  g_CasePerformance.energyPercent = ratio > 999 ? 999 : ratio;
  ratio = g_CasePerformance.referencePeakCurrent_ma ?
    ((uint32_t)g_Analysis.peakCurrent_ma * 100UL) /
    g_CasePerformance.referencePeakCurrent_ma : 0;
  g_CasePerformance.peakPercent = ratio > 999 ? 999 : ratio;

  Serial.print(F("PROFILE,RESULT,slot="));
  Serial.print(g_CasePerformance.resultSlot + 1);
  Serial.print(F(",match_pct="));
  Serial.print(g_CasePerformance.matchPercent);
  Serial.print(F(",energy_pct="));
  Serial.print(g_CasePerformance.energyPercent);
  Serial.print(F(",peak_pct="));
  Serial.print(g_CasePerformance.peakPercent);
  Serial.print(FPSTR(TEXT_INPUT_ENERGY));
  printAnalysisEnergy_J(g_Analysis.inputEnergy_mJ);
  Serial.print(F(",peak_ma="));
  Serial.println(g_Analysis.peakCurrent_ma);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Finish analysis and publish its aggregate values to serial.
*//*-------------------------------------------------------------------------*/
static void finishAnalysis(bool const aborted)
{
  turnAnnealerOff();
  turnStartStopLedOff();
  g_Analysis.elapsedTime_ms = millis() - g_Analysis.startTime;
  if(g_Analysis.elapsedTime_ms > ANALYSIS_DURATION_MS)
  {
    g_Analysis.elapsedTime_ms = ANALYSIS_DURATION_MS;
  }
  g_Analysis.graphValid = g_Analysis.graphColumn != UINT8_MAX;
  #if NZHS_HAS_WIFI
  r4StoreWifiHistory(aborted ? WIFI_HISTORY_USER_ABORT : WIFI_HISTORY_ANALYSE,
                     true, false);
  #endif
  if(aborted)
  {
    Serial.print(F("ANALYSE,ABORT,reason=USER ABORT,t_ms="));
  }
  else
  {
    Serial.print(F("ANALYSE,END,t_ms="));
  }
  Serial.print(g_Analysis.elapsedTime_ms);
  Serial.print(FPSTR(TEXT_INPUT_ENERGY));
  printAnalysisEnergy_J(g_Analysis.inputEnergy_mJ);
  Serial.print(F(",peak_ma="));
  Serial.println(g_Analysis.peakCurrent_ma);
  updateSystemState(STATE_ANALYSIS_RESULT);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Drop the analysed case for a fixed, operator-visible period.
*//*-------------------------------------------------------------------------*/
static void openAnalysisDropGate(void)
{
  turnAnnealerOff();
  turnStartStopLedOff();
  drawAnalysisGraph();
  openDropGate();
  setSystemTimeTarget(millis() + ANALYSIS_GATE_OPEN_PERIOD_MS);
  Serial.println(F("ANALYSE,GATE_OPEN"));
  updateSystemState(STATE_ANALYSIS_GATE_OPEN);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Reconstruct the retained analysis graph in the OLED buffer.
*//*-------------------------------------------------------------------------*/
static void showSavedAnalysis(void)
{
  if(!g_Analysis.graphValid || !g_Analysis.graphIsAnalysis)
  {
    updateSystemState(STATE_ANALYSIS_LOAD);
    return;
  }

  drawAnalysisGraph();
  updateSystemState(STATE_ANALYSIS_RESULT);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Select free-run mode and disable the stepper feeder.
*//*-------------------------------------------------------------------------*/
static void setFreeRunMode(void)
{
  CurrentMode = MODE_FREE_RUN;
  digitalWrite(g_FeederStepperEnPin,HIGH);
  turnModeLedOn();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Enable or disable the manual dump button.
*//*-------------------------------------------------------------------------*/
static void setDumpButtonEnabled(bool const enabled)
{
  g_UserSettings.dumpButtonEnabled = enabled;
  if(enabled)
  {
    setFreeRunMode();
    // Dump mode requires the operator to handle every case, so do not allow
    // an unattended restart after a temperature cooldown.
    g_UserSettings.autoRestartAfterCooldown = false;
    EEPROM.update(EEPROM_ADDRESS_AUTO_RESTART, 0);
  }
  EEPROM.update(EEPROM_ADDRESS_DUMP_BUTTON, enabled ? 1 : 0);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Cycle the operating mode and associated feeder state.
*//*-------------------------------------------------------------------------*/
static void cycleCurrentMode(void)
{
  if(g_UserSettings.dumpButtonEnabled)
  {
    setDumpButtonEnabled(false);
  }

  if(CurrentMode == MODE_SINGLE_SHOT)
  {
    setFreeRunMode();
  }
  else if(CurrentMode == MODE_FREE_RUN)
  {
    CurrentMode = MODE_AUTOMATIC;
    digitalWrite(g_FeederStepperEnPin,LOW);
    turnModeLedOn();
  }
  else
  {
    CurrentMode = MODE_SINGLE_SHOT;
    digitalWrite(g_FeederStepperEnPin,HIGH);
    turnModeLedOff();
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Change the value selected on the stopped screen.
*//*-------------------------------------------------------------------------*/
static void updateStoppedScreenSetting(bool const rapidTimeAdjust)
{
  if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_TIME)
  {
    g_UserSettings.annealTime_ms = incrementTimeSetting(g_UserSettings.annealTime_ms,
                                                        rapidTimeAdjust);
  }
  else if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_MODE)
  {
    cycleCurrentMode();
  }
  else if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_ANALYSE)
  {
    enterAnalysis();
  }
  else if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_SETTINGS)
  {
    g_UserSettings.settingsScreenSelection = SETTINGS_SCREEN_AUTO_RESTART;
    updateSystemState(STATE_SETTINGS);
  }
  else if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_PROFILES)
  {
    g_UserSettings.profileSlot = 0;
    updateSystemState(STATE_PROFILES);
  }
  else if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_INFO)
  {
    g_InfoScreenScroll = 0;
    updateSystemState(STATE_INFO);
  }
  else if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_DIAGNOSTICS)
  {
    updateSystemState(STATE_DIAGNOSTICS);
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Advance a time value using the common normal/rapid increment.
*//*-------------------------------------------------------------------------*/
static uint16_t incrementTimeSetting(uint16_t const time_ms, bool const rapidTimeAdjust)
{
  uint16_t const increment = rapidTimeAdjust ? RAPID_TIME_INCREMENT : 100;
  return time_ms > (MAX_ANNEAL_TIME - increment) ? MIN_ANNEAL_TIME : time_ms + increment;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Change the selected Settings screen item.
*//*-------------------------------------------------------------------------*/
static void updateSettingsScreenSetting(void)
{
  if(g_UserSettings.settingsScreenSelection == SETTINGS_SCREEN_AUTO_RESTART)
  {
    g_UserSettings.autoRestartAfterCooldown = !g_UserSettings.autoRestartAfterCooldown;
    if(g_UserSettings.autoRestartAfterCooldown)
    {
      // Automatic restart cannot coexist with manual dump handling.
      g_UserSettings.dumpButtonEnabled = false;
      EEPROM.update(EEPROM_ADDRESS_DUMP_BUTTON, 0);
    }
    EEPROM.update(EEPROM_ADDRESS_AUTO_RESTART, g_UserSettings.autoRestartAfterCooldown ? 1 : 0);
  }
  else if(g_UserSettings.settingsScreenSelection == SETTINGS_SCREEN_DUMP_BUTTON)
  {
    setDumpButtonEnabled(!g_UserSettings.dumpButtonEnabled);
  }
  #if NZHS_HAS_WIFI
  else if(g_UserSettings.settingsScreenSelection == SETTINGS_SCREEN_WIFI)
  {
    g_R4WifiSettingsSelection = WIFI_SETTINGS_MONITOR;
    updateSystemState(STATE_WIFI_SETTINGS);
  }
  #endif
  else
  {
    returnToStoppedScreen();
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Scroll the Info rows while keeping the Back row visible.
*//*-------------------------------------------------------------------------*/
static void advanceInfoScreenScroll(void)
{
  g_InfoScreenScroll = (g_InfoScreenScroll + 1) % INFO_SCREEN_SCROLL_COUNT;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Update the learned-current guard.
  @return       true when the current fault threshold has been reached.
*//*-------------------------------------------------------------------------*/
static bool lowCurrentGuardFault(uint16_t const cycleAverageCurrent_ma)
{
  uint32_t baselineCurrentTotal_ma = 0;
  uint8_t cycle;

  if(g_RunSafety.ignoredCurrentCycles < LOW_CURRENT_IGNORED_CYCLES)
  {
    g_RunSafety.ignoredCurrentCycles++;
    g_RunSafety.lowCurrentConsecutiveCycles = 0;
    return false;
  }

  if(g_RunSafety.baselineCurrentCycles < LOW_CURRENT_BASELINE_CYCLES)
  {
    g_RunSafety.baselineCurrentWindow_ma[g_RunSafety.baselineCurrentCycles] = cycleAverageCurrent_ma;
    g_RunSafety.baselineCurrentCycles++;
    g_RunSafety.baselineCurrentWindowIndex = g_RunSafety.baselineCurrentCycles % LOW_CURRENT_BASELINE_CYCLES;
  }
  else if((uint32_t)cycleAverageCurrent_ma * 100 < (uint32_t)g_RunSafety.baselineCurrent_ma * LOW_CURRENT_RATIO_PERCENT)
  {
    g_RunSafety.lowCurrentConsecutiveCycles++;
    if(g_RunSafety.lowCurrentConsecutiveCycles >= LOW_CURRENT_CONSECUTIVE_CYCLES)
    {
      g_RunSafety.cooldownRestartPending = false;
      return true;
    }
    return false;
  }
  else
  {
    g_RunSafety.lowCurrentConsecutiveCycles = 0;
    g_RunSafety.baselineCurrentWindow_ma[g_RunSafety.baselineCurrentWindowIndex] = cycleAverageCurrent_ma;
    g_RunSafety.baselineCurrentWindowIndex = (g_RunSafety.baselineCurrentWindowIndex + 1) % LOW_CURRENT_BASELINE_CYCLES;
  }

  for(cycle = 0; cycle < g_RunSafety.baselineCurrentCycles; cycle++)
  {
    baselineCurrentTotal_ma += g_RunSafety.baselineCurrentWindow_ma[cycle];
  }
  g_RunSafety.baselineCurrent_ma = baselineCurrentTotal_ma / g_RunSafety.baselineCurrentCycles;
  return false;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the current operating mode on the right-hand panel.
*//*-------------------------------------------------------------------------*/
static void drawCurrentMode(uint8_t const y, bool const selected)
{
  display.setCursor(RIGHT_PANEL_X, y);
  setTextSelected(selected);
  if(CurrentMode == MODE_FREE_RUN)
  {
    display.println(F("FREE RUN"));
  }
  else if(CurrentMode == MODE_AUTOMATIC)
  {
    display.println(F("AUTO FEED"));
  }
  else
  {
    display.println(F("ONE SHOT"));
  }
  display.setTextColor(WHITE);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Print a temperature rounded to one decimal place without
                linking Print::printFloat().
*//*-------------------------------------------------------------------------*/
static void printTemperatureTenths(int16_t const temperature)
{
  int16_t tenths = ((int32_t)temperature * 10 + (temperature < 0 ? -(TEMP_RAW_SCALE / 2) : (TEMP_RAW_SCALE / 2))) / TEMP_RAW_SCALE;
  uint8_t const fractionalTenths = tenths < 0 ? (uint8_t)(-tenths % 10) : (uint8_t)(tenths % 10);

  // C/C++ integer division truncates toward zero, so preserve the sign for
  // valid readings between -1.0 C and 0.0 C (for example, -0.5 C).
  if(tenths < 0 && tenths > -10)
  {
    display.write('-');
  }
  display.print(tenths / 10);
  display.write('.');
  display.print(fractionalTenths);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw temperature when a sensor is fitted.
*//*-------------------------------------------------------------------------*/
static void drawTemperature(uint8_t const y, int16_t const temperature)
{
  if(NumberDallasTempDevices != 0)
  {
    display.setCursor(RIGHT_PANEL_X, y);
    printTemperatureTenths(temperature);
    display.print((char PROGMEM)248);
    display.write('C');
    if(g_RunSafety.cooldownLockActive)
    {
      display.print(F(" COOL!"));
    }
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the optional case counter.
*//*-------------------------------------------------------------------------*/
static void drawCaseCount(uint8_t const y, uint16_t const casesAnnealed)
{
  #ifdef SHOW_CASE_COUNT
    display.setCursor(RIGHT_PANEL_X, y);
    display.print(F("CASES: "));
    display.print(casesAnnealed, 1);
  #endif
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the common left-side anneal-time panel.
*//*-------------------------------------------------------------------------*/
static void drawTimePanel(bool const selected)
{
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(g_UserSettings.stopType == PROFILE_STOP_TIME ? F("TIME") : F("MAX"));
  if(selected) display.write('>');
  display.setTextSize(2);
  display.setCursor(0,16);
  display.print(g_UserSettings.annealTime_ms/1000, DEC);
  display.write('.');
  display.print((g_UserSettings.annealTime_ms%1000)/100, DEC);
  display.write('s');
  display.setTextSize(1);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the Analyse actions available after a retained run.
*//*-------------------------------------------------------------------------*/
static void drawAnalysisMenuScreen(void)
{
  uint8_t const selectedRow = g_UserSettings.analysisMenuSelection + 1;
  uint8_t const firstRow = selectedRow > 3 ? selectedRow - 3 : 0;
  beginFullWidthScreen();
  for(uint8_t row = 0; row < 4; row++)
  {
    uint8_t const visibleRow = firstRow + row;
    display.setCursor(0, row * 8);
    if(visibleRow == 0)
    {
      display.print(F("ANALYSE"));
    }
    else
    {
      uint8_t const action = visibleRow - 1;
      setTextSelected(action == g_UserSettings.analysisMenuSelection);
      if(action == ANALYSIS_MENU_NEW) display.print(FPSTR(TEXT_NEW_ITEM));
      else if(action == ANALYSIS_MENU_REVIEW) display.print(FPSTR(TEXT_REVIEW_ITEM));
      else if(action == ANALYSIS_MENU_CONFIG) display.print(FPSTR(TEXT_CONFIG_ITEM));
      else display.print(FPSTR(TEXT_BACK_ITEM));
      display.setTextColor(WHITE);
    }
  }
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the single scrolling analysis stop-rule menu.
*//*-------------------------------------------------------------------------*/
static void drawAnalysisConfigScreen(void)
{
  bool const hasMaxTime = g_AnalysisConfig.stopType != PROFILE_STOP_TIME;
  uint8_t selectedRow;
  if(g_AnalysisConfig.selection == ANALYSIS_CONFIG_TYPE)
  {
    selectedRow = 0;
  }
  else if(g_AnalysisConfig.selection <= ANALYSIS_CONFIG_ENERGY_DIGIT_4)
  {
    selectedRow = 1;
  }
  else
  {
    selectedRow = g_AnalysisConfig.selection - (hasMaxTime ? 3 : 4);
  }
  uint8_t const firstRow = selectedRow > 3 ? selectedRow - 3 : 0;

  beginFullWidthScreen();
  for(uint8_t row = 0; row < 4; row++)
  {
    uint8_t const visibleRow = firstRow + row;
    uint8_t const item = visibleRow == 0 ? ANALYSIS_CONFIG_TYPE :
                         visibleRow == 1 ? ANALYSIS_CONFIG_TARGET :
                         (hasMaxTime ? ANALYSIS_CONFIG_MAX_TIME : ANALYSIS_CONFIG_PROFILE) + visibleRow - 2;
    display.setCursor(0, row * 8);
    if(item == ANALYSIS_CONFIG_TYPE)
    {
      setTextSelected(g_AnalysisConfig.selection == item);
      display.print(F("TYPE: "));
      if(g_AnalysisConfig.stopType == PROFILE_STOP_TIME) display.print(F("TIME"));
      else if(g_AnalysisConfig.stopType == PROFILE_STOP_ENERGY) display.print(F("ENERGY"));
      else display.print(F("PEAK DROP"));
    }
    else if(item == ANALYSIS_CONFIG_TARGET && g_AnalysisConfig.stopType == PROFILE_STOP_ENERGY)
    {
      display.print(F("ENERGY: "));
      for(uint8_t digit = 0; digit < 4; digit++)
      {
        setTextSelected(g_AnalysisConfig.selection == ANALYSIS_CONFIG_TARGET + digit);
        display.write('0' + g_AnalysisConfig.energyDigits[digit]);
      }
      display.setTextColor(WHITE);
      display.write('J');
    }
    else if(item == ANALYSIS_CONFIG_TARGET)
    {
      setTextSelected(g_AnalysisConfig.selection == item);
      if(g_AnalysisConfig.stopType == PROFILE_STOP_TIME)
      {
        display.print(F("TIME: "));
        display.print(g_AnalysisConfig.stopTime_ms / 1000);
        display.write('.');
        display.print((g_AnalysisConfig.stopTime_ms % 1000) / 100);
        display.write('s');
      }
      else
      {
        display.print(F("DROP: "));
        display.print(g_AnalysisConfig.peakDropPercent);
        display.write('%');
      }
    }
    else if(item == ANALYSIS_CONFIG_MAX_TIME)
    {
      setTextSelected(g_AnalysisConfig.selection == item);
      display.print(F("MAX TIME: "));
      display.print(g_AnalysisConfig.maxTime_ms / 1000);
      display.write('.');
      display.print((g_AnalysisConfig.maxTime_ms % 1000) / 100);
      display.write('s');
    }
    else if(item == ANALYSIS_CONFIG_PROFILE)
    {
      tCartridgeProfile profile;
      setTextSelected(g_AnalysisConfig.selection == item);
      display.print(F("PROFILE: "));
      if(loadProfile(g_AnalysisConfig.profileSlot, &profile))
      {
        display.write((uint8_t *)profile.name, PROFILE_NAME_LENGTH);
      }
      else
      {
        display.print(F("PROFILE "));
        display.print(g_AnalysisConfig.profileSlot + 1);
      }
    }
    else
    {
      setTextSelected(g_AnalysisConfig.selection == item);
      display.print(item == ANALYSIS_CONFIG_SAVE ? FPSTR(TEXT_SAVE_ITEM) : FPSTR(TEXT_BACK_ITEM));
    }
    display.setTextColor(WHITE);
  }
  display.display();
}

static void beginFullWidthScreen(void)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
}

static void setTextSelected(bool const selected)
{
  if(selected) display.setTextColor(BLACK, WHITE);
  else display.setTextColor(WHITE);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the analysis prompt or a current-sensor requirement.
*//*-------------------------------------------------------------------------*/
static void drawAnalysisLoadScreen(void)
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println(F("ANALYSE"));
  display.setTextSize(1);
  display.setCursor(0,17);
  if(CurrentSensorPresent)
  {
    display.println(F("LOAD CASE"));
    display.setCursor(0,25);
    display.print(F("PRESS START"));
  }
  else
  {
    display.println(F("CUR SENSOR"));
    display.setCursor(0,25);
    display.print(F("REQUIRED"));
  }
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw captured points, optionally over a dotted profile curve.
*//*-------------------------------------------------------------------------*/
static void drawCapturedGraph(bool const withReference)
{
  display.clearDisplay();
  if(withReference)
  {
    for(uint8_t sample = 0; sample < PROFILE_REFERENCE_SAMPLE_COUNT; sample++)
    {
      uint8_t const height = graphHeightFromSample(
        readProfileReferenceSample(g_CasePerformance.resultSlot, sample));
      display.drawPixel(sample * 2, 31 - height, WHITE);
    }
  }
  if(g_Analysis.graphColumn != UINT8_MAX)
  {
    uint8_t previousY = 31 - graphHeightFromSample(g_Analysis.graphSamples[0]);
    for(uint8_t column = 0; column <= g_Analysis.graphColumn; column++)
    {
      uint8_t const y = 31 - graphHeightFromSample(g_Analysis.graphSamples[column]);
      display.drawPixel(column, y, WHITE);
      if(withReference && column > 0)
      {
        uint8_t const top = y < previousY ? y : previousY;
        uint8_t const height = y > previousY ? y - previousY + 1 : previousY - y + 1;
        display.drawFastVLine(column, top, height, WHITE);
      }
      previousY = y;
    }
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the live current and accumulated energy graph labels.
*//*-------------------------------------------------------------------------*/
static void drawGraphMeasurements(void)
{
  uint16_t const energy_J = (g_Analysis.inputEnergy_mJ + 500UL) / 1000UL;

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.fillRect(42, 24, 42, 8, BLACK);
  display.setCursor(42, 24);
  display.print(F("A:"));
  display.print(g_Analysis.graphCurrent_ma / 1000);
  display.write('.');
  display.print((g_Analysis.graphCurrent_ma % 1000) / 100);
  display.write('A');
  display.fillRect(ANALYSIS_ENERGY_X, 24, SCREEN_WIDTH - ANALYSIS_ENERGY_X, 8, BLACK);
  display.setCursor(ANALYSIS_ENERGY_X, 24);
  display.print(F("J:"));
  display.print(energy_J);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the full-width, fixed-scale eight-second current trace.
*//*-------------------------------------------------------------------------*/
static void drawAnalysisGraph(void)
{
  drawCapturedGraph(false);
  drawGraphMeasurements();
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Compare the latest case line with its dotted reference line.
*//*-------------------------------------------------------------------------*/
static void drawCasePerformanceGraph(tPerformanceFooter const footer,
                                     uint16_t const remainingTime_ms)
{
  drawCapturedGraph(true);
  if(footer == PERFORMANCE_FOOTER_LIVE)
  {
    drawGraphMeasurements();
    display.display();
    return;
  }

  display.fillRect(0, 24, SCREEN_WIDTH, 8, BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  if(footer == PERFORMANCE_FOOTER_REVIEW)
  {
    display.setCursor(0, 24);
    display.setTextColor(BLACK, WHITE);
    display.print(FPSTR(TEXT_BACK_ITEM));
    display.setTextColor(WHITE);
    display.setCursor(42, 24);
    display.write('M');
    display.print(g_CasePerformance.matchPercent);
    display.write('%');
    display.setCursor(78, 24);
    display.write('E');
    display.print(g_CasePerformance.energyPercent);
    display.write('%');
  }
  else
  {
    display.setCursor(0, 24);
    display.write('M');
    display.print(g_CasePerformance.matchPercent);
    display.print(F("% E"));
    display.print(g_CasePerformance.energyPercent);
    if(footer == PERFORMANCE_FOOTER_DROP)
    {
      display.print(F("% DROP"));
    }
    else
    {
      display.print(F("% NEXT "));
      display.print(remainingTime_ms / 1000);
      display.write('.');
      display.print((remainingTime_ms % 1000) / 100);
      display.write('s');
    }
  }
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Overlay analysis completion or dumping status on the graph.
*//*-------------------------------------------------------------------------*/
static void drawAnalysisStatus(bool const dumping)
{
  display.setTextSize(1);
  if(dumping)
  {
    display.fillRect(0, 24, SCREEN_WIDTH, 8, BLACK);
    display.setCursor(0, 24);
    display.setTextColor(WHITE);
    display.print(F("DUMPING..."));
  }
  else
  {
    display.fillRect(0, 24, SCREEN_WIDTH, 8, BLACK);
    display.setCursor(0, 24);
    display.setTextColor(BLACK, WHITE);
    display.print(FPSTR(TEXT_BACK_ITEM));
    display.setTextColor(WHITE);
    display.print(F(" A:"));
    display.print(g_Analysis.peakCurrent_ma / 1000);
    display.write('.');
    display.print((g_Analysis.peakCurrent_ma % 1000) / 100);
    display.write('A');
    display.setCursor(ANALYSIS_ENERGY_X, 24);
    display.print(F("J:"));
    display.print((g_Analysis.inputEnergy_mJ + 500UL) / 1000UL);
  }
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Print integer millijoules as a three-decimal joule value.
*//*-------------------------------------------------------------------------*/
static void printAnalysisEnergy_J(uint32_t const energy_mJ)
{
  uint16_t const fractional_mJ = energy_mJ % 1000UL;

  Serial.print(energy_mJ / 1000UL);
  Serial.write('.');
  if(fractional_mJ < 100)
  {
    Serial.write('0');
  }
  if(fractional_mJ < 10)
  {
    Serial.write('0');
  }
  Serial.print(fractional_mJ);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the stopped screen and its selected right-panel item.
*//*-------------------------------------------------------------------------*/
static void drawStoppedScreen(bool const fanIsOn, int16_t const temperature, uint16_t const casesAnnealed)
{
  display.clearDisplay();
  drawTimePanel(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_TIME);

  if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_TIME ||
     g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_MODE)
  {
    display.setCursor(RIGHT_PANEL_X,0);
    display.println(fanIsOn ? F("FAN ON") : F("FAN OFF"));
    drawCurrentMode(8, g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_MODE);
    drawCaseCount(16, casesAnnealed);
    drawTemperature(24, temperature);
  }
  else
  {
    uint8_t item = g_UserSettings.stoppedScreenSelection - 1;
    for(uint8_t row = 0; row < 4; row++, item++)
    {
      uint8_t const y = row * 8;
      if(item == 1) drawCurrentMode(y, false);
      else if(item == 2) drawCaseCount(y, casesAnnealed);
      else if(item == 3) drawTemperature(y, temperature);
      else
      {
        display.setCursor(RIGHT_PANEL_X, y);
        setTextSelected(row == 3);
        display.print(FPSTR(pgm_read_ptr(&STOPPED_MENU_TEXT[item - 4])));
      }
    }
    display.setTextColor(WHITE);
  }

  display.drawFastVLine(54,0,32,WHITE);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the Settings screen.
*//*-------------------------------------------------------------------------*/
static void drawSettingsScreen(void)
{
  #if NZHS_HAS_WIFI
  uint8_t const selectedRow = g_UserSettings.settingsScreenSelection + 1;
  uint8_t const firstRow = selectedRow > 3 ? selectedRow - 3 : 0;
  beginFullWidthScreen();
  for(uint8_t row = 0; row < 4; row++)
  {
    uint8_t const item = firstRow + row;
    display.setCursor(0, row * 8);
    if(item == 0)
    {
      display.print(F("SETTINGS"));
      continue;
    }
    tSettingsScreenSelection const selection =
      (tSettingsScreenSelection)(item - 1);
    setTextSelected(selection == g_UserSettings.settingsScreenSelection);
    if(selection == SETTINGS_SCREEN_AUTO_RESTART)
    {
      display.print(F("RESTART: "));
      display.print(g_UserSettings.autoRestartAfterCooldown ? FPSTR(TEXT_ON) : FPSTR(TEXT_OFF));
    }
    else if(selection == SETTINGS_SCREEN_DUMP_BUTTON)
    {
      display.print(F("DUMP: "));
      display.print(g_UserSettings.dumpButtonEnabled ? FPSTR(TEXT_ON) : FPSTR(TEXT_OFF));
    }
    else if(selection == SETTINGS_SCREEN_WIFI)
    {
      display.print(F("WIFI >"));
    }
    else
    {
      display.print(FPSTR(TEXT_BACK_ITEM));
    }
    display.setTextColor(WHITE);
  }
  display.display();
  #else
  beginFullWidthScreen();
  display.setCursor(0,0);
  display.print(F("SETTINGS"));
  display.setCursor(0,8);
  setTextSelected(g_UserSettings.settingsScreenSelection == SETTINGS_SCREEN_AUTO_RESTART);
  display.print(F("RESTART: "));
  display.print(g_UserSettings.autoRestartAfterCooldown ? FPSTR(TEXT_ON) : FPSTR(TEXT_OFF));
  display.setTextColor(WHITE);
  display.setCursor(0,16);
  setTextSelected(g_UserSettings.settingsScreenSelection == SETTINGS_SCREEN_DUMP_BUTTON);
  display.print(F("DUMP: "));
  display.print(g_UserSettings.dumpButtonEnabled ? FPSTR(TEXT_ON) : FPSTR(TEXT_OFF));
  display.setTextColor(WHITE);
  display.setCursor(0,24);
  setTextSelected(g_UserSettings.settingsScreenSelection == SETTINGS_SCREEN_BACK);
  display.print(FPSTR(TEXT_BACK_ITEM));
  display.setTextColor(WHITE);
  display.display();
  #endif
}

#if NZHS_HAS_WIFI
/*---------------------------------------------------------------------------*/
/*! @brief      Draw persistent WiFi monitor and setup actions.
*//*-------------------------------------------------------------------------*/
static void drawWifiSettingsScreen(void)
{
  uint8_t const selectedRow = g_R4WifiSettingsSelection + 1;
  uint8_t const firstRow = selectedRow > 3 ? selectedRow - 3 : 0;
  beginFullWidthScreen();
  for(uint8_t row = 0; row < 4; row++)
  {
    uint8_t const item = firstRow + row;
    display.setCursor(0, row * 8);
    if(item == 0)
    {
      display.print(F("WIFI"));
      continue;
    }
    tWifiSettingsSelection const selection =
      (tWifiSettingsSelection)(item - 1);
    setTextSelected(selection == g_R4WifiSettingsSelection);
    if(selection == WIFI_SETTINGS_MONITOR)
    {
      display.print(F("MONITOR: "));
      display.print(g_R4WifiConfig.monitorEnabled ? FPSTR(TEXT_ON) : FPSTR(TEXT_OFF));
    }
    else if(selection == WIFI_SETTINGS_SETUP)
    {
      display.print(g_R4WifiMode == R4_WIFI_SETUP_AP ? F("SETUP: ACTIVE") : F("SETUP >"));
    }
    else if(selection == WIFI_SETTINGS_RESET)
    {
      display.print(F("RESET >"));
    }
    else
    {
      display.print(FPSTR(TEXT_BACK_ITEM));
    }
    display.setTextColor(WHITE);
  }
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Require an explicit second-screen selection before erasing.
*//*-------------------------------------------------------------------------*/
static void drawWifiResetScreen(void)
{
  beginFullWidthScreen();
  display.setCursor(0,0);
  display.print(F("RESET WIFI?"));
  display.setCursor(0,8);
  display.print(F("ERASE SSID/PASS"));
  display.setCursor(0,16);
  setTextSelected(g_R4WifiResetSelection == WIFI_RESET_CONFIRM);
  display.print(F("CONFIRM >"));
  display.setTextColor(WHITE);
  display.setCursor(0,24);
  setTextSelected(g_R4WifiResetSelection == WIFI_RESET_BACK);
  display.print(FPSTR(TEXT_BACK_ITEM));
  display.setTextColor(WHITE);
  display.display();
}
#endif

/*---------------------------------------------------------------------------*/
/*! @brief      Draw a profile-slot selector, including a visible Back option.
*//*-------------------------------------------------------------------------*/
static void drawProfilesScreen(void)
{
  tCartridgeProfile profile;
  beginFullWidthScreen();
  display.setCursor(0, 0);
  if(g_UserSettings.profileSlot >= PROFILE_COUNT)
  {
    display.print(FPSTR(TEXT_PROFILES));
    display.setCursor(0, 24);
    display.setTextColor(BLACK, WHITE);
    display.print(FPSTR(TEXT_BACK_ITEM));
    display.setTextColor(WHITE);
  }
  else
  {
    // A dedicated name row shows all ten stored characters without clipping.
    display.print(FPSTR(TEXT_PROFILES));
    display.setCursor(0, 8);
    if(loadProfile(g_UserSettings.profileSlot, &profile))
    {
      display.write((uint8_t *)profile.name, PROFILE_NAME_LENGTH);
    }
    else
    {
      display.print(F("PROFILE "));
      display.print(g_UserSettings.profileSlot + 1);
    }
    display.setCursor(0, 24);
    display.setTextColor(BLACK, WHITE);
    display.print(F("OPEN >"));
    display.setTextColor(WHITE);
  }
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the scrolling action list for one profile.
*//*-------------------------------------------------------------------------*/
static void drawProfileActionsScreen(void)
{
  uint8_t const selectedRow = g_UserSettings.profileActionSelection + 1;
  uint8_t const firstRow = selectedRow > 3 ? selectedRow - 3 : 0;
  beginFullWidthScreen();
  for(uint8_t row = 0; row < 4; row++)
  {
    uint8_t const visibleRow = firstRow + row;
    display.setCursor(0, row * 8);
    if(visibleRow == 0)
    {
      display.write('P');
      display.print(g_UserSettings.profileSlot + 1);
      display.print(F(" ACTIONS"));
    }
    else
    {
      uint8_t const action = visibleRow - 1;
      setTextSelected(action == g_UserSettings.profileActionSelection);
      display.print(FPSTR(pgm_read_ptr(&PROFILE_ACTION_TEXT[action])));
      display.setTextColor(WHITE);
    }
  }
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Review a profile reference or its latest comparison case.
*//*-------------------------------------------------------------------------*/
static void drawProfilePerformanceScreen(void)
{
  if(g_Analysis.graphValid && !g_Analysis.graphIsAnalysis &&
     g_CasePerformance.resultSlot == g_UserSettings.profileSlot)
  {
    drawCasePerformanceGraph(PERFORMANCE_FOOTER_REVIEW, 0);
  }
  else
  {
    drawProfileReferenceScreen();
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Review a saved reference before a comparison case exists.
*//*-------------------------------------------------------------------------*/
static void drawProfileReferenceScreen(void)
{
  uint16_t const address = getProfileReferenceAddress(g_UserSettings.profileSlot);
  uint16_t const peakCurrent_ma =
    EEPROM.read(address + PROFILE_REFERENCE_PEAK_OFFSET) |
    ((uint16_t)EEPROM.read(address + PROFILE_REFERENCE_PEAK_OFFSET + 1) << 8);
  uint16_t const energy_J =
    EEPROM.read(address + PROFILE_REFERENCE_ENERGY_OFFSET) |
    ((uint16_t)EEPROM.read(address + PROFILE_REFERENCE_ENERGY_OFFSET + 1) << 8);

  display.clearDisplay();
  uint8_t previousY = 31 - graphHeightFromSample(
    readProfileReferenceSample(g_UserSettings.profileSlot, 0));
  for(uint8_t sample = 0; sample < PROFILE_REFERENCE_SAMPLE_COUNT; sample++)
  {
    uint8_t const x = sample * 2;
    uint8_t const y = 31 - graphHeightFromSample(
      readProfileReferenceSample(g_UserSettings.profileSlot, sample));
    uint8_t const top = y < previousY ? y : previousY;
    uint8_t const height = y > previousY ? y - previousY + 1 : previousY - y + 1;
    display.drawFastVLine(x, top, height, WHITE);
    display.drawPixel(x + 1, y, WHITE);
    previousY = y;
  }

  display.fillRect(0, 24, SCREEN_WIDTH, 8, BLACK);
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.setTextColor(BLACK, WHITE);
  display.print(FPSTR(TEXT_BACK_ITEM));
  display.setTextColor(WHITE);
  display.setCursor(42, 24);
  display.print(F("A:"));
  display.print(peakCurrent_ma / 1000);
  display.write('.');
  display.print((peakCurrent_ma % 1000) / 100);
  display.write('A');
  display.setCursor(84, 24);
  display.print(F("J:"));
  display.print(energy_J);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the fixed-width arcade-style profile-name editor.
*//*-------------------------------------------------------------------------*/
static void drawProfileNameEditScreen(void)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("NAME P"));
  display.print(g_UserSettings.profileSlot + 1);
  display.setCursor(0, 10);
  display.write((uint8_t *)g_ProfileEditor.name, PROFILE_NAME_LENGTH);
  display.setCursor(g_UserSettings.profileNameCursor * 6, 18);
  if(g_UserSettings.profileNameCursor < PROFILE_NAME_LENGTH)
  {
    display.write('^');
  }
  display.setCursor(0, 24);
  setTextSelected(g_UserSettings.profileNameCursor == PROFILE_NAME_LENGTH);
  if(g_UserSettings.profileNameCursor == PROFILE_NAME_LENGTH)
  {
    display.print(FPSTR(TEXT_SAVE_ITEM));
  }
  else if(g_UserSettings.profileNameCursor == PROFILE_NAME_LENGTH + 1)
  {
    display.setTextColor(BLACK, WHITE);
    display.print(FPSTR(TEXT_BACK_ITEM));
  }
  else
  {
    display.print(FPSTR(TEXT_SAVE_ITEM));
  }
  display.setTextColor(WHITE);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Ask for explicit UP confirmation before deleting a profile.
*//*-------------------------------------------------------------------------*/
static void drawProfileDeleteConfirmScreen(void)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("DELETE P"));
  display.print(g_UserSettings.profileSlot + 1);
  display.setCursor(0, 12);
  setTextSelected(g_UserSettings.profileDeleteConfirmed);
  display.print(F("DELETE >"));
  display.setTextColor(WHITE);
  display.setCursor(0, 24);
  setTextSelected(!g_UserSettings.profileDeleteConfirmed);
  display.print(FPSTR(TEXT_BACK_ITEM));
  display.setTextColor(WHITE);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Acknowledge a profile operation before returning from it.
*//*-------------------------------------------------------------------------*/
static void drawProfileNoticeScreen(void)
{
  beginFullWidthScreen();
  display.setCursor(0, 0);
  display.print(FPSTR(TEXT_PROFILES));
  display.setTextSize(2);
  display.setCursor(0, 12);
  switch(g_ProfileNotice)
  {
    case PROFILE_NOTICE_LOADED:
      display.print(F("LOADED"));
      break;
    case PROFILE_NOTICE_DELETED:
      display.print(F("DELETED"));
      break;
    case PROFILE_NOTICE_EMPTY:
      display.print(F("EMPTY"));
      break;
    case PROFILE_NOTICE_NO_DATA:
      display.print(F("NO DATA"));
      break;
    default:
      display.print(F("SAVED"));
      break;
  }
  display.setTextSize(1);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the original startup logo from losslessly split regions.
*//*-------------------------------------------------------------------------*/
static void drawStartupLogo(void)
{
  display.drawBitmap(0, 2, anneallogoLeft, 24, 28, WHITE);
  display.drawBitmap(40, 11, anneallogoText, 48, 10, WHITE);
  display.drawBitmap(112, 2, anneallogoRight, 8, 28, WHITE);
  display.fillRect(24, 3, 88, 2, WHITE);
  display.fillRect(24, 27, 88, 2, WHITE);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw a common full-width, two-line fault message.
*//*-------------------------------------------------------------------------*/
static void drawFaultScreen(__FlashStringHelper const * const line1,
                            __FlashStringHelper const * const line2)
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(line1);
  display.setCursor(0, 16);
  display.println(line2);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the full diagnostics screen.
*//*-------------------------------------------------------------------------*/
static void drawDiagnosticsScreen(void)
{
  beginFullWidthScreen();
  display.setCursor(0,0);
  display.print(F("DIAGNOSTICS"));
  display.setCursor(0,8);
  display.print(F("TEMP:"));
  display.print(sensors.getDeviceCount());
  display.print(F(" CUR:"));
  display.write(CurrentSensorPresent ? 'Y' : 'N');
  display.setCursor(0,16);
  drawResetDiagnostics();
  display.setCursor(0,24);
  display.setTextColor(BLACK, WHITE);
  display.print(FPSTR(TEXT_BACK_ITEM));
  display.setTextColor(WHITE);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw scrolling low-current, supply-voltage, and firmware information.
*//*-------------------------------------------------------------------------*/
static void drawInfoScreen(void)
{
  #if NZHS_HAS_WIFI
  beginFullWidthScreen();
  for(uint8_t row = 0; row < 3; row++)
  {
    uint8_t const item = g_InfoScreenScroll + row;
    display.setCursor(0, row * 8);
    if(item == 0)
    {
      display.print(F("INFO"));
    }
    else if(item == 1)
    {
      display.print(F("LOW: "));
      display.print(LOW_CURRENT_RATIO_PERCENT);
      display.print(F("% N"));
      display.print(LOW_CURRENT_BASELINE_CYCLES);
    }
    else if(item == 2)
    {
      display.print(F("BASE: "));
      if(g_RunSafety.baselineCurrentCycles >= LOW_CURRENT_BASELINE_CYCLES)
      {
        display.print(g_RunSafety.baselineCurrent_ma/1000, DEC);
        display.write('.');
        display.print((g_RunSafety.baselineCurrent_ma%1000)/100, DEC);
        display.write('A');
      }
      else
      {
        display.print(F("--"));
      }
    }
    else if(item == 3)
    {
      refreshSupplyVoltage();
      display.print(F("5V: "));
      display.print(g_SupplyVoltage_mv/1000, DEC);
      display.write('.');
      display.print((g_SupplyVoltage_mv%1000)/100, DEC);
      display.write('V');
    }
    else if(item == 4)
    {
      display.print(F("FW: "));
      display.print(SOFTWARE_VERSION);
    }
    else if(item == 5)
    {
      display.print(F("WIFI: "));
      if(g_R4WifiMode == R4_WIFI_OFF) display.print(F("OFF"));
      else if(g_R4WifiMode == R4_WIFI_STATION_CONNECTING) display.print(F("CONNECT"));
      else if(g_R4WifiMode == R4_WIFI_STATION_MONITOR) display.print(F("LAN"));
      else if(g_R4WifiMode == R4_WIFI_SETUP_AP) display.print(F("SETUP AP"));
      else if(g_R4WifiMode == R4_WIFI_DIRECT_MONITOR) display.print(F("DIRECT AP"));
      else display.print(F("ERROR"));
    }
    else
    {
      display.print(F("IP: "));
      if(g_R4WifiMonitorActive) display.print(WiFi.localIP());
      else display.print(F("--"));
    }
  }
  display.setCursor(0,24);
  display.setTextColor(BLACK, WHITE);
  display.print(FPSTR(TEXT_BACK_ITEM));
  display.setTextColor(WHITE);
  display.display();
  #else
  beginFullWidthScreen();
  if(g_InfoScreenScroll == 0)
  {
    display.setCursor(0,0);
    display.print(F("INFO"));
  }

  if(g_InfoScreenScroll < 2)
  {
    display.setCursor(0, 8 - (g_InfoScreenScroll * 8));
    display.print(F("LOW: "));
    display.print(LOW_CURRENT_RATIO_PERCENT);
    display.print(F("% N"));
    display.print(LOW_CURRENT_BASELINE_CYCLES);
  }

  display.setCursor(0, 16 - (g_InfoScreenScroll * 8));
  display.print(F("BASE: "));
  if(g_RunSafety.baselineCurrentCycles >= LOW_CURRENT_BASELINE_CYCLES)
  {
    display.print(g_RunSafety.baselineCurrent_ma/1000, DEC);
    display.write('.');
    display.print((g_RunSafety.baselineCurrent_ma%1000)/100, DEC);
    display.write('A');
  }
  else
  {
    display.print(F("--"));
  }

  if(g_InfoScreenScroll > 0)
  {
    refreshSupplyVoltage();
    display.setCursor(0, 24 - (g_InfoScreenScroll * 8));
    display.print(F("5V: "));
    display.print(g_SupplyVoltage_mv/1000, DEC);
    display.write('.');
    display.print((g_SupplyVoltage_mv%1000)/100, DEC);
    display.write('V');
  }

  if(g_InfoScreenScroll > 1)
  {
    display.setCursor(0,16);
    display.print(F("FW: "));
    display.print(SOFTWARE_VERSION);
  }
  display.setCursor(0,24);
  display.setTextColor(BLACK, WHITE);
  display.print(FPSTR(TEXT_BACK_ITEM));
  display.setTextColor(WHITE);
  display.display();
  #endif
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw compact reset diagnostics: cause/state.
  @details      Cause codes: W=watchdog, B=brown-out, E=external, P=power-on.
                The cause is best-effort because the bootloader can clear it.
*//*-------------------------------------------------------------------------*/
static void drawResetDiagnostics(void)
{
  display.print(F("RST:"));
  if(g_ResetDiagnostics.resetFlags & _BV(WDRF))
  {
    display.write('W');
  }
  else if(g_ResetDiagnostics.resetFlags & _BV(BORF))
  {
    display.write('B');
  }
  else if(g_ResetDiagnostics.resetFlags & _BV(EXTRF))
  {
    display.write('E');
  }
  else if(g_ResetDiagnostics.resetFlags & _BV(PORF))
  {
    display.write('P');
  }
  else
  {
    display.write('-');
  }

  display.write('|');
  if(g_ResetDiagnostics.previousSystemState == STATE_ANNEALING)
  {
    display.write('A');
  }
  else if(g_ResetDiagnostics.previousSystemState == STATE_DROPPING)
  {
    display.write('D');
  }
  else if(g_ResetDiagnostics.previousSystemState == STATE_RELOADING)
  {
    display.write('R');
  }
  else if(g_ResetDiagnostics.previousSystemState == STATE_COOLDOWN)
  {
    display.write('C');
  }
  else if(g_ResetDiagnostics.previousSystemState == STATE_STOPPED)
  {
    display.write('S');
  }
  else
  {
    display.write('?');
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Print reset diagnostics to the DEBUG serial console.
*//*-------------------------------------------------------------------------*/
static void printResetDiagnostics(void)
{
  Serial.print(F("Reset flags: "));
  if(g_ResetDiagnostics.resetFlags & _BV(WDRF)) Serial.print(F("WDRF "));
  if(g_ResetDiagnostics.resetFlags & _BV(BORF))  Serial.print(F("BORF "));
  if(g_ResetDiagnostics.resetFlags & _BV(EXTRF)) Serial.print(F("EXTRF "));
  if(g_ResetDiagnostics.resetFlags & _BV(PORF))  Serial.print(F("PORF "));
  if(g_ResetDiagnostics.resetFlags == 0) Serial.print(F("none"));
  Serial.print(F("; previous state: "));
  Serial.println(g_ResetDiagnostics.previousSystemState);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Set system state.
  @param        state: System state.
*//*-------------------------------------------------------------------------*/
static void updateSystemState(tStateMachineStates const state)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    g_SystemStatePrev = g_SystemState;
    g_SystemState = state;
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Has system state changed.
  @return       false - system state has not changed. Else true.
*//*-------------------------------------------------------------------------*/
static bool hasSystemStateChanged(void)
{
  if (g_SystemStatePrev == g_SystemState)
  {
    return false;
  }

  return true;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Return true after a millisecond deadline, including rollover.
*//*-------------------------------------------------------------------------*/
static inline bool hasTimeElapsed(uint32_t const timeTarget, uint32_t const currentTime)
{
  return (int32_t)(currentTime - timeTarget) >= 0;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Atomically update the deadline used by the timer ISR.
*//*-------------------------------------------------------------------------*/
static void setSystemTimeTarget(uint32_t const timeTarget)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    SystemTimeTarget = timeTarget;
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Read the start button state.
  @return       Start button state. 0 = low, else non-zero.
*//*-------------------------------------------------------------------------*/
static bool readStartButton(void)
{
  return !digitalRead(g_StartStopButtonPin);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Read the mode button state.
  @return       Start button state. 0 = low, else non-zero.
*//*-------------------------------------------------------------------------*/
static bool readModeButton(void)
{
  return !digitalRead(g_ModeButtonPin);
}
/*---------------------------------------------------------------------------*/
/*! @brief      Read the up button state.
  @return       Start button state. 0 = low, else non-zero.
*//*-------------------------------------------------------------------------*/
static bool readUpButton(void)
{
  return !digitalRead(A2);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Turn the annealer on.
*//*-------------------------------------------------------------------------*/
static void turnAnnealerOn(void)
{
  digitalWrite(g_AnnealerPin, HIGH);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Turn the annealer off.
*//*-------------------------------------------------------------------------*/
static void turnAnnealerOff(void)
{
  digitalWrite(g_AnnealerPin, LOW);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Open the drop gate.
*//*-------------------------------------------------------------------------*/
static void openDropGate(void)
{

    #if NZHS_PLATFORM_UNO_R3
    analogWrite(g_DropServoPin,SERVO_OPEN_POSITION);
    #else
    if(g_R4DropServoReady)
    {
      g_R4DropServo.writeMicroseconds(SERVO_OPEN_POSITION * 128U);
    }
    #endif
    digitalWrite(g_DropSolenoidPin, HIGH);

}

/*---------------------------------------------------------------------------*/
/*! @brief      Close the drop gate.
*//*-------------------------------------------------------------------------*/
static void closeDropGate(void)
{

    #if NZHS_PLATFORM_UNO_R3
    analogWrite(g_DropServoPin,SERVO_CLOSE_POSITION); //IO9 PWM output
    #else
    if(g_R4DropServoReady)
    {
      g_R4DropServo.writeMicroseconds(SERVO_CLOSE_POSITION * 128U);
    }
    #endif
    digitalWrite(g_DropSolenoidPin, LOW);

}
/*---------------------------------------------------------------------------*/
/*! @brief      Turn the start/stop LED on.
*//*-------------------------------------------------------------------------*/
static void turnStartStopLedOn(void)
{
  digitalWrite(g_StartStopLedPin, HIGH);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Turn the start/stop LED off.
*//*-------------------------------------------------------------------------*/
static void turnStartStopLedOff(void)
{
  digitalWrite(g_StartStopLedPin, LOW);
}
/*---------------------------------------------------------------------------*/
/*! @brief      Turn the start/stop LED on.
*//*-------------------------------------------------------------------------*/
static void turnModeLedOn(void)
{
  digitalWrite(g_ModeLedPin, HIGH);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Turn the start/stop LED off.
*//*-------------------------------------------------------------------------*/
static void turnModeLedOff(void)
{
  digitalWrite(g_ModeLedPin, LOW);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Turn the cooling fan on.
*//*-------------------------------------------------------------------------*/
static void turnCoolingFanOn(void)
{
  digitalWrite(g_CoolingFanPin, HIGH);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Turn the cooling fan off.
*//*-------------------------------------------------------------------------*/
static void turnCoolingFanOff(void)
{
  digitalWrite(g_CoolingFanPin, LOW);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Read the PSU current.
  @return       PSU current in millamps (ma).
*//*-------------------------------------------------------------------------*/
static uint16_t readPsuCurrent_ma(void)
{
  uint16_t adc = 0;

  adc = analogRead(g_PsuCurrentAdcPin);
  adc = abs(adc - psuCurrentZeroOffset)*CURRENT_SENSOR_SCALE; //2.5V ADC offset , Scaling factor to suit sensor chosen.
  if(adc > 25000) // error from abs function can return large numbers if ADC measurement goes

  {
    adc = 0;
  }
  return adc;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Estimate the regulated AVcc/5 V rail using the internal band-gap.
  @details      The internal band-gap has device tolerance; calibrate
                INTERNAL_BANDGAP_MV against a meter if absolute accuracy matters.
  @return       Estimated AVcc in millivolts.
*//*-------------------------------------------------------------------------*/
static uint16_t readSupplyVoltage_mv(void)
{
  #if NZHS_PLATFORM_UNO_R3
  uint8_t previousAdmux = ADMUX;

  // AVcc is the ADC reference; channel 14 is the internal 1.1 V band-gap.
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); //allow the band-gap and ADC sample capacitor to settle
  ADCSRA |= _BV(ADSC);
  while(ADCSRA & _BV(ADSC))
  {
  }

  uint16_t adc = ADC;
  ADMUX = previousAdmux;
  ADCSRA |= _BV(ADSC); //discard one conversion after switching back from band-gap
  while(ADCSRA & _BV(ADSC))
  {
  }
  if(adc == 0)
  {
    return 0;
  }
  return ((uint32_t)INTERNAL_BANDGAP_MV * 1023UL + (adc / 2)) / adc;
  #else
  float const supplyVoltage = analogReference();
  if(!(supplyVoltage > 0.0f) || supplyVoltage > 6.0f)
  {
    return 0;
  }
  return (uint16_t)(supplyVoltage * 1000.0f + 0.5f);
  #endif
}

/*---------------------------------------------------------------------------*/
/*! @brief      Refresh the cached AVcc estimate at a display-friendly rate.
*//*-------------------------------------------------------------------------*/
static void refreshSupplyVoltage(void)
{
  uint32_t currentTime = millis();
  if(g_SupplyVoltageSampleTime == 0 ||
     hasTimeElapsed(g_SupplyVoltageSampleTime + SUPPLY_VOLTAGE_SAMPLE_PERIOD, currentTime))
  {
    g_SupplyVoltage_mv = readSupplyVoltage_mv();
    g_SupplyVoltageSampleTime = currentTime;
  }
}


/*---------------------------------------------------------------------------*/
/*! @brief      rotate case loader to preload position from home
*//*-------------------------------------------------------------------------*/
static void preloadCase(void)
{
	addStepsToGo(CASE_FEEDER_STEPS_DROP_TO_PRELOAD);
	//enableStepperPulses(1);
}

/*---------------------------------------------------------------------------*/
/*! @brief      rotate case loader to drop position
*//*-------------------------------------------------------------------------*/
static void loadCase(void)
{
	//StepsToGo = StepsToGo + CASE_FEEDER_STEPS_PRELOAD_TO_DROP; //multiply by 2 for the 2 half cycles counted by the timer interrupt
  returnCaseFeederHome();
	//enableStepperPulses(1);
}

/*---------------------------------------------------------------------------*/
/*! @brief      rotate case loader to home/park position from anywhere
*//*-------------------------------------------------------------------------*/
static void returnCaseFeederHome(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(StepsFromHome)
    {
      StepsToGo = STEPPER_STEPS_PER_TURN - StepsFromHome;
    }
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      is case feeder still moving?
*//*-------------------------------------------------------------------------*/
static bool caseFeederStillMoving(void)
{
  if(getStepsToGo())
    {
      return true;
    }
    else
    {
      return false;
    }

}

/*---------------------------------------------------------------------------*/
/*! @brief      Atomically add feeder steps while the timer ISR is active.
*//*-------------------------------------------------------------------------*/
static void addStepsToGo(uint16_t const steps)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    StepsToGo += steps;
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Atomically set the remaining feeder steps.
*//*-------------------------------------------------------------------------*/
static void setStepsToGo(uint16_t const steps)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    StepsToGo = steps;
  }
}

/*---------------------------------------------------------------------------*/
/*! @brief      Atomically read the remaining feeder steps.
*//*-------------------------------------------------------------------------*/
static uint16_t getStepsToGo(void)
{
  #if NZHS_PLATFORM_UNO_R4
  uint16_t steps = 0;
  #else
  uint16_t steps;
  #endif
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    steps = StepsToGo;
  }
  return steps;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Atomically read the feeder position.
*//*-------------------------------------------------------------------------*/
static uint16_t getStepsFromHome(void)
{
  uint16_t steps;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    steps = StepsFromHome;
  }
  return steps;
}

/*---------------------------------------------------------------------------*/
/*! @brief      is case feeder still moving?
*//*-------------------------------------------------------------------------*/
static int16_t readTemperature(void)
{
  int16_t t = NumberDallasTempDevices ? sensors.getTemp(tempDeviceAddress) : DEVICE_DISCONNECTED_RAW;
  sensors.requestTemperatures(); // this takes quite some time to complete ~90ms or longer. read it on the next loop
  return t;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Check whether a DS18B20 temperature reading is valid.
*//*-------------------------------------------------------------------------*/
static bool isTemperatureReadingValid(int16_t const temperature)
{
  // DEVICE_DISCONNECTED_RAW is the same numeric value as -55 C, and the
  // library's Celsius conversion also treats it as invalid.
  return temperature > TEMP_SENSOR_MIN_RAW && temperature <= TEMP_SENSOR_MAX_RAW;
}
