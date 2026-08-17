/*---------------------------------------------------------------------------*/
/*! @brief      Brass Cartridge Case Annealer.
  @details      None.
  @author       Justin Spence, Mark Griffith. 2020
  @note         circuitworksnz@gmail.com
*//*-------------------------------------------------------------------------*/

//--Includes-------------------------------------------------------------------
#include <SPI.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/atomic.h>
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
#define SOFTWARE_VERSION F("4.0.0")
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
#define INFO_SCREEN_SCROLL_COUNT 3
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
#define PROFILE_COUNT 8
#define PROFILE_NAME_LENGTH 10
#define PROFILE_MAGIC 0xC7
#define PROFILE_FLAG_AUTO_RESTART 0x01
#define PROFILE_FLAG_DUMP_BUTTON 0x02
#define PROFILE_FLAG_STOP_TYPE_SHIFT 2
#define PROFILE_FLAG_STOP_TYPE_MASK 0x0C
#define PROFILE_SAVE_NOTICE_PERIOD 1000
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
static const char TEXT_RENAME_ITEM[] PROGMEM = "RENAME >";
static const char TEXT_DELETE_ITEM[] PROGMEM = "DELETE >";
static const char TEXT_NEW_ITEM[] PROGMEM = "NEW >";
static const char TEXT_REVIEW_ITEM[] PROGMEM = "REVIEW >";
static const char TEXT_CONFIG_ITEM[] PROGMEM = "CONFIG >";
static const char TEXT_ON[] PROGMEM = "ON";
static const char TEXT_OFF[] PROGMEM = "OFF";
static const char TEXT_INPUT_ENERGY[] PROGMEM = ",input_energy_J=";
static const char * const PROFILE_ACTION_TEXT[] PROGMEM = {
  TEXT_LOAD_ITEM, TEXT_SAVE_ITEM, TEXT_RENAME_ITEM, TEXT_DELETE_ITEM, TEXT_BACK_ITEM
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
  STATE_PROFILE_NAME_EDIT,
  STATE_PROFILE_DELETE_CONFIRM,
  STATE_PROFILE_SAVED,
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
  SETTINGS_SCREEN_BACK,
  SETTINGS_SCREEN_SELECTION_COUNT,
} tSettingsScreenSelection;

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
  PROFILE_ACTION_BACK,
  PROFILE_ACTION_SELECTION_COUNT,
} tProfileActionSelection;

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

static_assert(EEPROM_ADDRESS_PROFILE_BASE + (PROFILE_COUNT * sizeof(tCartridgeProfile)) <=
              EEPROM_ADDRESS_PROFILE_RULE_BASE,
              "Profile stop rules overlap cartridge profiles");

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
  uint8_t graphHeight;
  uint8_t graphHeights[ANALYSIS_GRAPH_COLUMNS];
  bool graphValid;
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
static const uint8_t g_PsuCurrentAdcPin     = 0;
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
static tAnalysisConfigState g_AnalysisConfig;
static tAdaptiveAnnealState g_AdaptiveAnneal;
static uint8_t g_InfoScreenScroll = 0;
static uint16_t g_SupplyVoltage_mv = 0;
static uint32_t g_SupplyVoltageSampleTime = 0;
// Deliberately not initialized by the C runtime, so a watchdog reset can
// preserve the last state reached by the application.
static tResetDiagnostics g_ResetDiagnostics __attribute__((section(".noinit")));

// Run before normal C/C++ initialization. Capture the AVR reset flags and
// disable a watchdog that may still be active after a watchdog reset. The Uno
// bootloader can clear MCUSR first, so the recorded cause is best-effort.
void captureResetDiagnostics(void) __attribute__((naked, used, section(".init3")));
void captureResetDiagnostics(void)
{
  g_ResetDiagnostics.resetFlags = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

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
static void drawAnalysisMenuScreen(void);
static void drawAnalysisConfigScreen(void);
static void setTextSelected(bool const selected) __attribute__((noinline));
static void beginFullWidthScreen(void) __attribute__((noinline));
static void drawAnalysisLoadScreen(void);
static void drawAnalysisGraph(void);
static void drawAnalysisStatus(bool const dumping);
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
static void drawProfilesScreen(void);
static void drawProfileActionsScreen(void);
static void drawProfileNameEditScreen(void);
static void drawProfileDeleteConfirmScreen(void);
static void drawProfileSavedScreen(void);
static void drawDiagnosticsScreen(void);
static void drawInfoScreen(void);
static void drawResetDiagnostics(void);
static void printResetDiagnostics(void);
static void drawStartupLogo(void) __attribute__((noinline));
static void drawFaultScreen(__FlashStringHelper const * const line1,
                            __FlashStringHelper const * const line2) __attribute__((noinline));

/*---------------------------------------------------------------------------*/
/*! @brief      Initialize the Case Annealer.
  @details      None.
  @param        None.
  @return       None.
*//*-------------------------------------------------------------------------*/
void setup()
{
  if(g_ResetDiagnostics.magic != RESET_DIAGNOSTIC_MAGIC)
  {
    g_ResetDiagnostics.lastSystemState = STATE_UNKNOWN;
  }
  g_ResetDiagnostics.previousSystemState = g_ResetDiagnostics.lastSystemState;
  g_ResetDiagnostics.magic = RESET_DIAGNOSTIC_MAGIC;

  //TCCR0B = TCCR0B & B11111000 | B00000101; //PWM on D5 & D6 set to 61.04Hz Timer 0 -- Timer Used for system ms tick
 // TCCR2B = TCCR2B & B11111000 | B00000110; //PWM on D3 & D11 set to 122.55Hz Timer 2  <--- tiner 2
  TCCR1B = TCCR1B & B11111000 | B00000101; //PWM on D9 & D10 of 30.64 Hz Timer 1   <---- USE IO9 PWM for drop gate Servo

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
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register - divide by microsteps to shorten step period
  OCR2A = 170 / STEPPER_MICROSTEPS;
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS20-22 bit for prescaler
  TCCR2B |= (1 << CS22);
  TCCR2B |= (1 << CS21);

  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

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
  digitalWrite(g_FeederDirPin,HIGH);
  digitalWrite(g_FeederStepperEnPin,LOW); //disable stepper driver
  closeDropGate();
  turnStartStopLedOff();
  turnModeLedOff();
  turnAnnealerOff();
  turnCoolingFanOff();
  closeDropGate();
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
  wdt_enable(WDTO_500MS);
  digitalWrite(g_FeederStepperEnPin,HIGH); //disable stepper driver
}
/*---------------------------------------------------------------------------*/
/*! @brief      Timer2 ISR
  @details      None.
  @param        None.
  @return       Never.
*//*-------------------------------------------------------------------------*/

ISR(TIMER2_COMPA_vect){//timer2 interrupt
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
            OCR2A = 120 / STEPPER_MICROSTEPS;
          #else
            OCR2A = 170;
          #endif

      }
      else if(StepsFromHome < CASE_FEEDER_HOPPER_END) //slow down the feed wheel while picking the case for more reliable pickups
      {
          // set compare match register - divide by microsteps to shorten step period
          #if STEPPER_MICROSTEPS >= 4 //check we arent going to overflow the 8 bit timer register
            OCR2A = 800 / STEPPER_MICROSTEPS;
          #else
            OCR2A = 254;
          #endif
      }
      else //speed up again once new case is picked
      {
        // set compare match register - divide by microsteps to shorten step period
          OCR2A = 170 / STEPPER_MICROSTEPS;
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

}

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
  wdt_reset();
  g_ResetDiagnostics.lastSystemState = g_SystemState;
  //read keys
  LoopStartTime = millis(); // capture time when loop starts
  start = readStartButton();
  modeKey = readModeButton();
  upKey = readUpButton();

  // DS18B20 conversion takes longer than the 25 ms analysis sampler.
  if(g_SystemState != STATE_ANALYSING &&
     !(g_SystemState == STATE_ANNEALING && g_UserSettings.stopType != PROFILE_STOP_TIME))
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
            g_SystemState == STATE_PROFILE_NAME_EDIT ||
            g_SystemState == STATE_PROFILE_DELETE_CONFIRM ||
            g_SystemState == STATE_PROFILE_SAVED ||
            g_SystemState == STATE_ANALYSIS_MENU ||
            g_SystemState == STATE_ANALYSIS_CONFIG ||
            g_SystemState == STATE_DIAGNOSTICS ||
            g_SystemState == STATE_INFO)
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
      else if(g_SystemState == STATE_PROFILE_NAME_EDIT)
      {
        g_UserSettings.profileNameCursor = (g_UserSettings.profileNameCursor + 1) % (PROFILE_NAME_LENGTH + 2);
      }
      else if(g_SystemState == STATE_PROFILE_DELETE_CONFIRM)
      {
        g_UserSettings.profileDeleteConfirmed = !g_UserSettings.profileDeleteConfirmed;
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

    case STATE_PROFILES:
    {
      tCartridgeProfile profile;
      updateSystemState(g_SystemState);
      if(upKey && !upKeyPrev)
      {
        if(g_UserSettings.profileSlot >= PROFILE_COUNT)
        {
          returnToStoppedScreen();
          break;
        }
        g_UserSettings.profileActionSelection = loadProfile(g_UserSettings.profileSlot, &profile) ? PROFILE_ACTION_LOAD : PROFILE_ACTION_SAVE;
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
            // Loading is the one menu action that returns ready to run.
            g_UserSettings.stoppedScreenSelection = STOPPED_SCREEN_TIME;
            returnToStoppedScreen();
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
            updateSystemState(STATE_PROFILE_SAVED);
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
          updateSystemState(STATE_PROFILE_SAVED);
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
          updateSystemState(STATE_PROFILES);
        }
        else
        {
          updateSystemState(STATE_PROFILE_ACTIONS);
        }
        break;
      }
      drawProfileDeleteConfirmScreen();
    break;

    case STATE_PROFILE_SAVED:
    {
      if(hasSystemStateChanged())
      {
        setSystemTimeTarget(millis() + PROFILE_SAVE_NOTICE_PERIOD);
      }
      updateSystemState(g_SystemState);
      if(hasTimeElapsed(SystemTimeTarget, millis()))
      {
        if(g_AnalysisConfig.profileSaveInProgress)
        {
          g_AnalysisConfig.profileSaveInProgress = false;
          g_UserSettings.analysisMenuSelection = ANALYSIS_MENU_CONFIG;
          updateSystemState(STATE_ANALYSIS_MENU);
        }
        else
        {
          updateSystemState(STATE_PROFILES);
        }
        break;
      }
      drawProfileSavedScreen();
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
        updateSystemState(g_Analysis.graphValid ? STATE_ANALYSIS_MENU : STATE_STOPPED);
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
          updateSystemState(STATE_OVERCURRENT_WARNING);
          break;
        }
        g_RunSafety.annealingCurrentTotal_ma += psuCurrent_ma;
        g_RunSafety.annealingCurrentSamples++;

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
            updateSystemState(STATE_LOW_CURRENT_WARNING);
            break;
          }
        }
        g_RunSafety.completedAnnealCycles++;
        openDropGate();
        updateSystemState(STATE_DROPPING);
        // Do not render the annealing countdown after expiry.
        break;
      }

      if(!hasTimeElapsed(systemTimeTarget, currentTime))
      {
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
                                    g_UserSettings.stopType != PROFILE_STOP_TIME);
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
  EEPROM.update(address, targetEnergy_J & 0xFF);
  EEPROM.update(address + 1, targetEnergy_J >> 8);
  EEPROM.update(address + 2, peakDropPercent);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Mark a profile slot blank without touching neighbouring slots.
*//*-------------------------------------------------------------------------*/
static void clearProfile(uint8_t const slot)
{
  EEPROM.update(getProfileAddress(slot), 0);
  EEPROM.update(EEPROM_ADDRESS_PROFILE_RULE_BASE + ((uint16_t)slot * 3), 0xFF);
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
  if(g_Analysis.graphValid)
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
/*! @brief      Store the configured stop rule without changing other settings.
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
  g_UserSettings.profileSlot = slot;
  g_AnalysisConfig.profileSaveInProgress = true;
  // Analysis configuration uses the default PROFILE n name for an empty
  // slot. Renaming remains an explicit action in the Profiles menu.
  updateSystemState(STATE_PROFILE_SAVED);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Initialise a manual, current-trace analysis run.
*//*-------------------------------------------------------------------------*/
static void beginAnalysis(void)
{
  uint32_t const currentTime = millis();

  // Keep one height per OLED column so the completed trace can be reconstructed
  // after other screens have replaced the display framebuffer.
  display.clearDisplay();
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
  g_Analysis.graphHeight = 0;
  memset(g_Analysis.graphHeights, 0, sizeof(g_Analysis.graphHeights));
  g_Analysis.graphValid = false;
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
  uint16_t averageCurrent_ma;
  uint16_t elapsed_ms;
  uint8_t column;
  uint8_t graphHeight;
  uint32_t samplePeriod_ms;

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
  samplePeriod_ms = currentTime - g_Analysis.lastSampleTime;
  g_Analysis.lastSampleTime = currentTime;
  g_Analysis.inputEnergy_mJ += (uint32_t)current_ma * samplePeriod_ms * ANALYSIS_SUPPLY_VOLTAGE_V / 1000UL;
  if(current_ma > g_Analysis.peakCurrent_ma)
  {
    g_Analysis.peakCurrent_ma = current_ma;
  }
  column = ((uint32_t)elapsed_ms * ANALYSIS_GRAPH_COLUMNS) / ANALYSIS_DURATION_MS;
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
  averageCurrent_ma = g_Analysis.graphCurrentTotal_ma / g_Analysis.graphCurrentSamples;
  g_Analysis.graphCurrent_ma = averageCurrent_ma;
  graphHeight = averageCurrent_ma >= ANALYSIS_GRAPH_MAX_CURRENT_MA ?
    31 : ((uint32_t)averageCurrent_ma * 31UL) / ANALYSIS_GRAPH_MAX_CURRENT_MA;
  g_Analysis.graphHeight = graphHeight;
  g_Analysis.graphHeights[column] = graphHeight;

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
    updateSystemState(STATE_OVERCURRENT_WARNING);
  }
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
  if(!g_Analysis.graphValid)
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
/*! @brief      Draw the full-width, fixed-scale eight-second current trace.
*//*-------------------------------------------------------------------------*/
static void drawAnalysisGraph(void)
{
  uint8_t column;
  uint16_t const energy_J = (g_Analysis.inputEnergy_mJ + 500UL) / 1000UL;

  display.clearDisplay();
  if(g_Analysis.graphColumn != UINT8_MAX)
  {
    for(column = 0; column <= g_Analysis.graphColumn; column++)
    {
      display.drawPixel(column, 31 - g_Analysis.graphHeights[column], WHITE);
    }
  }

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
        display.print(FPSTR(pgm_read_word(&STOPPED_MENU_TEXT[item - 4])));
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
}

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
      display.print(FPSTR(pgm_read_word(&PROFILE_ACTION_TEXT[action])));
      display.setTextColor(WHITE);
    }
  }
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
/*! @brief      Acknowledge that a profile was written before returning to it.
*//*-------------------------------------------------------------------------*/
static void drawProfileSavedScreen(void)
{
  beginFullWidthScreen();
  display.setCursor(0, 0);
  display.print(FPSTR(TEXT_PROFILES));
  display.setTextSize(2);
  display.setCursor(0, 12);
  display.print(F("SAVED"));
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

    analogWrite(g_DropServoPin,SERVO_OPEN_POSITION);
    digitalWrite(g_DropSolenoidPin, HIGH);

}

/*---------------------------------------------------------------------------*/
/*! @brief      Close the drop gate.
*//*-------------------------------------------------------------------------*/
static void closeDropGate(void)
{

    analogWrite(g_DropServoPin,SERVO_CLOSE_POSITION); //IO9 PWM output
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
  uint16_t steps;
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
