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
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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
#define TEMP_SENSOR_MIN_C -55.0f //DS18B20 lower measurement limit
#define TEMP_SENSOR_MAX_C 125.0f //DS18B20 upper measurement limit
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
#define PROFILE_COUNT 8
#define PROFILE_NAME_LENGTH 10
#define PROFILE_MAGIC 0xC7
#define PROFILE_FLAG_AUTO_RESTART 0x01
#define PROFILE_FLAG_DUMP_BUTTON 0x02
#define PROFILE_SAVE_NOTICE_PERIOD 1000
#define RIGHT_PANEL_X 56

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1, 200000, 200000);
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
typedef enum tStateMachineStates
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
  STATE_DIAGNOSTICS,
  STATE_INFO,
  STATE_OVERCURRENT_WARNING,
  STATE_LOW_CURRENT_WARNING,
  STATE_TEMPERATURE_SENSOR_WARNING,
  STATE_UNKNOWN,
} tStateMachineStates;

typedef enum ModeList
{
  MODE_SINGLE_SHOT = 0,
  MODE_FREE_RUN,
  MODE_AUTOMATIC,
} ModeList;

typedef enum tStoppedScreenSelection
{
  STOPPED_SCREEN_TIME = 0,
  STOPPED_SCREEN_MODE,
  STOPPED_SCREEN_SETTINGS,
  STOPPED_SCREEN_PROFILES,
  STOPPED_SCREEN_INFO,
  STOPPED_SCREEN_DIAGNOSTICS,
  STOPPED_SCREEN_SELECTION_COUNT,
} tStoppedScreenSelection;

typedef enum tSettingsScreenSelection
{
  SETTINGS_SCREEN_AUTO_RESTART = 0,
  SETTINGS_SCREEN_DUMP_BUTTON,
  SETTINGS_SCREEN_BACK,
  SETTINGS_SCREEN_SELECTION_COUNT,
} tSettingsScreenSelection;

typedef enum tProfileActionSelection
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
  uint8_t profileSlot;
  tProfileActionSelection profileActionSelection;
  uint8_t profileNameCursor;
  bool profileDeleteConfirmed;
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

typedef struct tRunSafetyState
{
  bool cooldownRestartPending;
  bool cooldownLockActive;
  uint32_t annealingCurrentTotal_ma;
  uint16_t annealingCurrentSamples;
  uint32_t restartCurrentTotal_ma;
  uint16_t restartCurrentSamples;
  uint16_t baselineCurrent_ma;
  uint16_t baselineCurrentWindow_ma[LOW_CURRENT_BASELINE_CYCLES];
  uint8_t ignoredCurrentCycles;
  uint8_t baselineCurrentCycles;
  uint8_t baselineCurrentWindowIndex;
  uint8_t lowCurrentConsecutiveCycles;
} tRunSafetyState;

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


 // custom startup image, 128x32px
const unsigned char anneallogo [] PROGMEM = {
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
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xC3, 0x3F, 0x30, 0xC7, 0x80, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xE3, 0x3F, 0x30, 0xCF, 0xC0, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xE3, 0x03, 0x30, 0xCC, 0x40, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xB3, 0x06, 0x30, 0xCC, 0x00, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0xB3, 0x0C, 0x3F, 0xCF, 0x80, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x9B, 0x1C, 0x3F, 0xC3, 0xC0, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x8B, 0x18, 0x30, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x8F, 0x30, 0x30, 0xC8, 0xC0, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x87, 0x3F, 0x30, 0xCF, 0xC0, 0x00, 0x00, 0x00, 0xDE, 0x00,
0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x87, 0x3F, 0x30, 0xCF, 0x80, 0x00, 0x00, 0x00, 0xDE, 0x00,
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

const unsigned char projectile [] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0xFE, 0x00, 0x07, 0xC0, 0x01, 0xF0, 0x7F, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x3C, 0x1F, 0x00, 0x03, 0xF0, 0x00, 0xFC, 0x00, 0x78, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x0F, 0xC0, 0x01, 0xFC, 0x00, 0x7F, 0x80, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x06, 0x00, 0x03, 0xF0, 0x00, 0x7F, 0x00, 0x0F, 0xC0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0xFC, 0x00, 0x1F, 0x80, 0x03, 0xE0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x7F, 0x00, 0x07, 0xE0, 0x00, 0xC0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x1F, 0xC0, 0x01, 0xF8, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x7E, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x00, 0x1F, 0x80, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x07, 0xE0, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x03, 0x00, 0x00, 0x30, 0x00, 0x1F, 0x80, 0x03, 0xF8, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xE0, 0x00, 0x7C, 0x00, 0x0F, 0xE0, 0x00, 0xFE, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x18, 0x00, 0x3E, 0x00, 0x03, 0xF0, 0x00, 0x3F, 0x80, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x06, 0x00, 0x1F, 0x80, 0x00, 0xFC, 0x00, 0x0F, 0xC0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x07, 0xE0, 0x00, 0x3F, 0x00, 0x03, 0xE0, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x3C, 0x01, 0xF0, 0x00, 0x0F, 0x80, 0x00, 0xE0, 0x78, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0xF8, 0x00, 0x07, 0xC0, 0x00, 0x7F, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

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


//-- global variables declarations----------------------------------------
static volatile tStateMachineStates g_SystemState = STATE_JUST_BOOTED;
static tStateMachineStates g_SystemStatePrev = STATE_UNKNOWN;

static ModeList CurrentMode = MODE_SINGLE_SHOT;
static tUserSettings g_UserSettings;
static tRunSafetyState g_RunSafety;
static tCartridgeProfile g_ProfileEditor;
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
static float readTemperature(uint8_t);
static bool isTemperatureReadingValid(float const temperature);
static void addStepsToGo(uint16_t const steps);
static void setStepsToGo(uint16_t const steps);
static uint16_t getStepsToGo(void);
static uint16_t getStepsFromHome(void);
static void loadUserSettings(void);
static void resetRunSafetyState(void);
static void enterCooldown(bool const allowAutomaticRestart, bool const cycleStopRequested);
static void advanceStoppedScreenSelection(void);
static void advanceSettingsScreenSelection(void);
static void advanceProfileSlot(void);
static void advanceProfileActionSelection(void);
static void returnToStoppedScreen(void);
static void setFreeRunMode(void);
static void setDumpButtonEnabled(bool const enabled);
static void cycleCurrentMode(void);
static void updateStoppedScreenSetting(bool const rapidTimeAdjust);
static void updateSettingsScreenSetting(void);
static uint16_t getProfileAddress(uint8_t const slot);
static uint8_t calculateProfileChecksum(tCartridgeProfile const * const profile);
static bool loadProfile(uint8_t const slot, tCartridgeProfile * const profile);
static void saveProfile(uint8_t const slot, tCartridgeProfile const * const profile);
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
static void drawTemperature(uint8_t const y, float const temperature);
static void drawCaseCount(uint8_t const y, uint16_t const casesAnnealed);
static void drawTimePanel(bool const selected);
static void drawStoppedScreen(bool const fanIsOn, float const temperature, uint16_t const casesAnnealed);
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
  #ifdef DEBUG
  Serial.begin(115200);
  delay(20);
  Serial.println(F("Debug active."));
  printResetDiagnostics();
  #endif

  delay(200);

  display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS);

  display.clearDisplay();
  // Setup text and draw splash screen
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  #ifndef DEBUG //dont do the splash startup in debug
    display.drawBitmap(0, 0,  anneallogo, 128, 32, 1);
    display.display();
    delay(2000);
    display.clearDisplay();
    display.drawBitmap(0, 0,  anneallogo2, 128, 32, 1);
    display.display();
    delay(2000);

    for(uint8_t i = 0; i <= 20; i++)
    {
      display.clearDisplay();
      display.drawBitmap(0, 0,  projectile, 128, 32, 1);
      display.display();
      delay(100);
      display.clearDisplay();
      display.drawBitmap(0, 0,  projectile2, 128, 32, 1);
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

  if ((g_SystemState == STATE_ANNEALING) && hasTimeElapsed(SystemTimeTarget, millis()))
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
  static uint32_t cooling_timer = 0;
  static uint32_t LoopStartTime;
  static float temperature = 0;
  static bool Just_Booted = 1;
  static bool Next_Cycle_Is_STOPPED = 0;
  static uint16_t CasesAnnealed = 0;

  //boot the watchdog
  wdt_reset();
  g_ResetDiagnostics.lastSystemState = g_SystemState;
  //read keys
  LoopStartTime = millis(); // capture time when loop starts
  start = readStartButton();
  modeKey = readModeButton();
  upKey = readUpButton();

  temperature = readTemperature(0);

  // Leaving the cooldown screen must not make a new run possible until the
  // original hysteresis threshold has been reached.
  if(g_RunSafety.cooldownLockActive && temperature < (TEMP_LIMIT - TEMP_HYSTERESIS))
  {
    g_RunSafety.cooldownLockActive = false;
  }

  if(!isTemperatureReadingValid(temperature) &&
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
      if(g_RunSafety.cooldownLockActive)
      {
        // The user may browse menus while cooling, but cannot start a run yet.
      }
      else if(temperature > TEMP_LIMIT)
      {
        enterCooldown(false, false);
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
            g_SystemState == STATE_DIAGNOSTICS ||
            g_SystemState == STATE_INFO)
    {
      // Menus always leave through their visible BACK > item and UP.
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
  }
  else
  {
    if (!modeKeyPrev) //mode key just pressed?
    {
      if (g_SystemState == STATE_OVERCURRENT_WARNING ||
          g_SystemState == STATE_LOW_CURRENT_WARNING ||
          g_SystemState == STATE_TEMPERATURE_SENSOR_WARNING)
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
      else if(g_UserSettings.dumpButtonEnabled && CurrentMode == MODE_FREE_RUN)
      {
        openDropGate();
        manualDumpInProgress = true;
      }
    }
  }


  switch (g_SystemState) //State machine.
  {
    case STATE_STOPPED:
    {
      updateSystemState(g_SystemState);
      if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_TIME)
      {
        upKeyDuration = upKey ? upKeyDuration + 1 : 0;
        if(upKeyDuration >= LONG_PRESS_HOLD_TIME)
        {
          g_UserSettings.annealTime_ms = MIN_ANNEAL_TIME;
          annealTimeChanged = true;
          upKeyDuration = 0;
          rapidTimePresses = 0;
          lastTimePress = 0;
        }
      }
      else
      {
        upKeyDuration = 0;
        rapidTimePresses = 0;
        lastTimePress = 0;
      }
      if (upKey && !upKeyPrev) //up key pressed?
      {
        bool rapidTimeAdjust = false;
        if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_TIME)
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
          annealTimeChanged = true;
        }
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
          updateSystemState(STATE_PROFILE_ACTIONS);
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
        updateSystemState(STATE_PROFILES);
        break;
      }
      drawProfileSavedScreen();
    }
    break;

    case STATE_ANNEALING:
    {
      if (hasSystemStateChanged())
      {
        setSystemTimeTarget(millis() + g_UserSettings.annealTime_ms);
        g_RunSafety.annealingCurrentTotal_ma = 0;
        g_RunSafety.annealingCurrentSamples = 0;
        g_RunSafety.restartCurrentTotal_ma = 0;
        g_RunSafety.restartCurrentSamples = 0;
        turnStartStopLedOn();
        turnAnnealerOn();
        cooling_timer = COOLDOWN_PERIOD + millis(); // 5 minute cooldown after last anneal
          if(CurrentMode == MODE_AUTOMATIC)
          {
            preloadCase();
          }
      }
      updateSystemState(g_SystemState);

      psuCurrent_ma = readPsuCurrent_ma();
      g_RunSafety.restartCurrentTotal_ma += psuCurrent_ma;
      g_RunSafety.restartCurrentSamples++;
      if(CurrentSensorPresent)
      {
        if(psuCurrent_ma >= PSU_OVERCURRENT) //overloaded the PSU - may damage the ZVS converter
        {
          turnAnnealerOff();
          turnStartStopLedOff();
          updateSystemState(STATE_OVERCURRENT_WARNING);
          break;
        }
        g_RunSafety.annealingCurrentTotal_ma += psuCurrent_ma;
        g_RunSafety.annealingCurrentSamples++;
      }

      uint32_t systemTimeTarget = SystemTimeTarget;
      uint32_t currentTime = millis();
      if (hasTimeElapsed(systemTimeTarget, currentTime))
      {
        turnAnnealerOff();
        if(g_RunSafety.restartCurrentSamples)
        {
          uint16_t restartCycleAverageCurrent_ma = g_RunSafety.restartCurrentTotal_ma / g_RunSafety.restartCurrentSamples;
          if(restartCycleAverageCurrent_ma <= CURRENT_SENSOR_DETECTION_MA)
          {
            // This check applies only to automatic restart. The original A0
            // sensor-presence detection remains in use for all other features.
            g_UserSettings.autoRestartAfterCooldown = false;
            EEPROM.update(EEPROM_ADDRESS_AUTO_RESTART, 0);
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
          display.print(F("."));
          display.print((psuCurrent_ma%1000)/100, DEC);
          display.print(F("A  "));
        }
        display.print(remainingTime/1000, DEC);
        display.print(F("."));
        display.print((remainingTime%1000)/100, DEC);
        display.print(F("s"));
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
        display.setCursor(0, 16);
        display.print(temperature, 1);
        display.print((char PROGMEM)248);
        display.print(F("C"));
        display.display();
        break;
      }
      closeDropGate();
      CasesAnnealed++;

      if(temperature > TEMP_LIMIT)
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
        display.print(".");
        display.print((remainingTime%1000)/100, DEC);
        display.print("s");

        #ifdef SHOW_CASE_COUNT
          display.setCursor(65, 0);
          display.print(F("CASES"));
          display.setCursor(65, 16);
          display.print(CasesAnnealed, 1);
          display.drawLine(57,0,57,32,WHITE);
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

      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println(F("! FAULT !"));
      display.println(F("CHECK COIL"));
      display.display();

    }
    break;

    case STATE_LOW_CURRENT_WARNING:
    {
      updateSystemState(g_SystemState);

      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println(F("CHECK CASE"));
      display.setCursor(0, 16);
      display.println(F("CURRENT LO!"));
      display.display();

    }
    break;

    case STATE_TEMPERATURE_SENSOR_WARNING:
    {
      updateSystemState(g_SystemState);
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println(F("TEMP ERROR"));
      display.setCursor(0, 16);
      display.println(F("CHECK TEMP"));
      display.display();
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
      display.print(temperature, 1);
      display.print((char PROGMEM)248);
      display.print(F("C"));
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
      if(temperature < (TEMP_LIMIT - TEMP_HYSTERESIS)) //has it cooled enough to resume?
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
      if(temperature > TEMP_LIMIT)
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
  Serial.print(F("."));
  Serial.print((psuCurrent_ma%1000)/100, DEC);
  Serial.print(F(";A;"));

  Serial.print(F("Anneal Time;"));
  Serial.print(g_UserSettings.annealTime_ms);
  Serial.print(F(";ms;"));

  Serial.print("Step count;");
  Serial.print(getStepsToGo());
  Serial.print(F(";"));

  Serial.print("Steps from home;");
  Serial.print(getStepsFromHome());
  Serial.print(F(";"));

  Serial.print(F("State;"));
  Serial.print(g_SystemState);
  Serial.print(F(";"));

  Serial.print(F("Loop Time Remaining;"));
  Serial.print(LoopStartTime + LOOP_TIME - millis());
  Serial.println(F(";ms;"));


  #endif


  while(!hasTimeElapsed(LoopStartTime + LOOP_TIME, millis())) // wait for the loop time to expire
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
  g_UserSettings.profileSlot = 0;
  g_UserSettings.profileActionSelection = PROFILE_ACTION_LOAD;
  g_UserSettings.profileNameCursor = 0;
  g_UserSettings.profileDeleteConfirmed = false;
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
/*! @brief      Mark a profile slot blank without touching neighbouring slots.
*//*-------------------------------------------------------------------------*/
static void clearProfile(uint8_t const slot)
{
  EEPROM.update(getProfileAddress(slot), 0);
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
                   (g_UserSettings.dumpButtonEnabled ? PROFILE_FLAG_DUMP_BUTTON : 0);
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
                  (g_UserSettings.dumpButtonEnabled ? PROFILE_FLAG_DUMP_BUTTON : 0);
  saveProfile(slot, &profile);
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
  g_ProfileEditor.annealTime_ms = g_UserSettings.annealTime_ms;
  g_ProfileEditor.mode = CurrentMode;
  g_ProfileEditor.flags = (g_UserSettings.autoRestartAfterCooldown ? PROFILE_FLAG_AUTO_RESTART : 0) |
                         (g_UserSettings.dumpButtonEnabled ? PROFILE_FLAG_DUMP_BUTTON : 0);
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
    uint16_t const increment = rapidTimeAdjust ? RAPID_TIME_INCREMENT : 100;
    g_UserSettings.annealTime_ms = g_UserSettings.annealTime_ms > (MAX_ANNEAL_TIME - increment) ?
      MIN_ANNEAL_TIME : g_UserSettings.annealTime_ms + increment;
  }
  else if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_MODE)
  {
    cycleCurrentMode();
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
  if(selected) display.setTextColor(BLACK, WHITE);
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
/*! @brief      Draw temperature when a sensor is fitted.
*//*-------------------------------------------------------------------------*/
static void drawTemperature(uint8_t const y, float const temperature)
{
  if(NumberDallasTempDevices != 0)
  {
    display.setCursor(RIGHT_PANEL_X, y);
    display.print(temperature, 1);
    display.print((char PROGMEM)248);
    display.print(F("C"));
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
  display.print(selected ? F("TIME>") : F("TIME"));
  display.setTextSize(2);
  display.setCursor(0,16);
  display.print(g_UserSettings.annealTime_ms/1000, DEC);
  display.print(F("."));
  display.print((g_UserSettings.annealTime_ms%1000)/100, DEC);
  display.print(F("s"));
  display.setTextSize(1);
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the stopped screen and its selected right-panel item.
*//*-------------------------------------------------------------------------*/
static void drawStoppedScreen(bool const fanIsOn, float const temperature, uint16_t const casesAnnealed)
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
  else if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_SETTINGS)
  {
    drawCurrentMode(0, false);
    drawCaseCount(8, casesAnnealed);
    drawTemperature(16, temperature);
    display.setCursor(RIGHT_PANEL_X,24);
    display.setTextColor(BLACK, WHITE);
    display.print(F("SETTINGS >"));
    display.setTextColor(WHITE);
  }
  else if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_PROFILES)
  {
    drawCaseCount(0, casesAnnealed);
    drawTemperature(8, temperature);
    display.setCursor(RIGHT_PANEL_X,16);
    display.print(F("SETTINGS >"));
    display.setCursor(RIGHT_PANEL_X,24);
    display.setTextColor(BLACK, WHITE);
    display.print(F("PROFILES >"));
    display.setTextColor(WHITE);
  }
  else if(g_UserSettings.stoppedScreenSelection == STOPPED_SCREEN_INFO)
  {
    drawTemperature(0, temperature);
    display.setCursor(RIGHT_PANEL_X,8);
    display.print(F("SETTINGS >"));
    display.setCursor(RIGHT_PANEL_X,16);
    display.print(F("PROFILES >"));
    display.setCursor(RIGHT_PANEL_X,24);
    display.setTextColor(BLACK, WHITE);
    display.print(F("INFO >"));
    display.setTextColor(WHITE);
  }
  else
  {
    display.setCursor(RIGHT_PANEL_X,0);
    display.print(F("SETTINGS >"));
    display.setCursor(RIGHT_PANEL_X,8);
    display.print(F("PROFILES >"));
    display.setCursor(RIGHT_PANEL_X,16);
    display.print(F("INFO >"));
    display.setCursor(RIGHT_PANEL_X,24);
    display.setTextColor(BLACK, WHITE);
    display.print(F("DIAGNOSTICS>"));
    display.setTextColor(WHITE);
  }

  display.drawLine(54,0,54,32,WHITE);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the Settings screen.
*//*-------------------------------------------------------------------------*/
static void drawSettingsScreen(void)
{
  display.clearDisplay();
  drawTimePanel(false);
  display.setCursor(RIGHT_PANEL_X,0);
  display.print(F("SETTINGS"));
  display.setCursor(RIGHT_PANEL_X,8);
  if(g_UserSettings.settingsScreenSelection == SETTINGS_SCREEN_AUTO_RESTART) display.setTextColor(BLACK, WHITE);
  display.print(F("RESTART: "));
  display.print(g_UserSettings.autoRestartAfterCooldown ? F("ON") : F("OFF"));
  display.setTextColor(WHITE);
  display.setCursor(RIGHT_PANEL_X,16);
  if(g_UserSettings.settingsScreenSelection == SETTINGS_SCREEN_DUMP_BUTTON) display.setTextColor(BLACK, WHITE);
  display.print(F("DUMP: "));
  display.print(g_UserSettings.dumpButtonEnabled ? F("ON") : F("OFF"));
  display.setTextColor(WHITE);
  display.setCursor(RIGHT_PANEL_X,24);
  if(g_UserSettings.settingsScreenSelection == SETTINGS_SCREEN_BACK) display.setTextColor(BLACK, WHITE);
  display.print(F("BACK >"));
  display.setTextColor(WHITE);
  display.drawLine(54,0,54,32,WHITE);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw a profile-slot selector, including a visible Back option.
*//*-------------------------------------------------------------------------*/
static void drawProfilesScreen(void)
{
  tCartridgeProfile profile;
  display.clearDisplay();
  drawTimePanel(false);
  display.setCursor(RIGHT_PANEL_X, 0);
  if(g_UserSettings.profileSlot >= PROFILE_COUNT)
  {
    display.print(F("PROFILES"));
    display.setCursor(RIGHT_PANEL_X, 24);
    display.setTextColor(BLACK, WHITE);
    display.print(F("BACK >"));
    display.setTextColor(WHITE);
  }
  else
  {
    // A dedicated name row shows all ten stored characters without clipping.
    display.print(F("PROFILES"));
    display.setCursor(RIGHT_PANEL_X, 8);
    if(loadProfile(g_UserSettings.profileSlot, &profile))
    {
      display.write((uint8_t *)profile.name, PROFILE_NAME_LENGTH);
    }
    else
    {
      display.print(F("PROFILE "));
      display.print(g_UserSettings.profileSlot + 1);
    }
    display.setCursor(RIGHT_PANEL_X, 24);
    display.setTextColor(BLACK, WHITE);
    display.print(F("OPEN >"));
    display.setTextColor(WHITE);
  }
  display.drawLine(54,0,54,32,WHITE);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the scrolling action list for one profile.
*//*-------------------------------------------------------------------------*/
static void drawProfileActionsScreen(void)
{
  static char const * const actions[] = { "LOAD >", "SAVE >", "RENAME >", "DELETE >", "BACK >" };
  uint8_t row;
  uint8_t action;
  display.clearDisplay();
  drawTimePanel(false);
  display.setCursor(RIGHT_PANEL_X, 0);
  display.print(F("P"));
  display.print(g_UserSettings.profileSlot + 1);
  display.print(F(" ACTIONS"));
  for(row = 0; row < 3; row++)
  {
    action = g_UserSettings.profileActionSelection + row;
    if(action >= PROFILE_ACTION_SELECTION_COUNT)
    {
      break;
    }
    display.setCursor(RIGHT_PANEL_X, 8 + (row * 8));
    if(action == g_UserSettings.profileActionSelection) display.setTextColor(BLACK, WHITE);
    display.print(actions[action]);
    display.setTextColor(WHITE);
  }
  display.drawLine(54,0,54,32,WHITE);
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
    display.print(F("^"));
  }
  display.setCursor(0, 24);
  if(g_UserSettings.profileNameCursor == PROFILE_NAME_LENGTH) display.setTextColor(BLACK, WHITE);
  if(g_UserSettings.profileNameCursor == PROFILE_NAME_LENGTH)
  {
    display.print(F("SAVE >"));
  }
  else if(g_UserSettings.profileNameCursor == PROFILE_NAME_LENGTH + 1)
  {
    display.setTextColor(BLACK, WHITE);
    display.print(F("BACK >"));
  }
  else
  {
    display.print(F("SAVE >"));
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
  if(g_UserSettings.profileDeleteConfirmed) display.setTextColor(BLACK, WHITE);
  display.print(F("DELETE >"));
  display.setTextColor(WHITE);
  display.setCursor(0, 24);
  if(!g_UserSettings.profileDeleteConfirmed) display.setTextColor(BLACK, WHITE);
  display.print(F("BACK >"));
  display.setTextColor(WHITE);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Acknowledge that a profile was written before returning to it.
*//*-------------------------------------------------------------------------*/
static void drawProfileSavedScreen(void)
{
  display.clearDisplay();
  drawTimePanel(false);
  display.setCursor(RIGHT_PANEL_X, 0);
  display.print(F("PROFILES"));
  display.setTextSize(2);
  display.setCursor(RIGHT_PANEL_X, 12);
  display.print(F("SAVED"));
  display.setTextSize(1);
  display.drawLine(54,0,54,32,WHITE);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw the full diagnostics screen.
*//*-------------------------------------------------------------------------*/
static void drawDiagnosticsScreen(void)
{
  display.clearDisplay();
  drawTimePanel(false);
  display.setCursor(RIGHT_PANEL_X,0);
  display.print(F("DIAGNOSTICS"));
  display.setCursor(RIGHT_PANEL_X,8);
  display.print(F("TEMP:"));
  display.print(sensors.getDeviceCount());
  display.print(F(" CUR:"));
  display.print(CurrentSensorPresent ? F("Y") : F("N"));
  display.setCursor(RIGHT_PANEL_X,16);
  drawResetDiagnostics();
  display.setCursor(RIGHT_PANEL_X,24);
  display.setTextColor(BLACK, WHITE);
  display.print(F("BACK >"));
  display.setTextColor(WHITE);
  display.drawLine(54,0,54,32,WHITE);
  display.display();
}

/*---------------------------------------------------------------------------*/
/*! @brief      Draw scrolling low-current, supply-voltage, and firmware information.
*//*-------------------------------------------------------------------------*/
static void drawInfoScreen(void)
{
  display.clearDisplay();
  drawTimePanel(false);
  if(g_InfoScreenScroll == 0)
  {
    display.setCursor(RIGHT_PANEL_X,0);
    display.print(F("INFO"));
  }

  if(g_InfoScreenScroll < 2)
  {
    display.setCursor(RIGHT_PANEL_X, 8 - (g_InfoScreenScroll * 8));
    display.print(F("LOW: "));
    display.print(LOW_CURRENT_RATIO_PERCENT);
    display.print(F("% N"));
    display.print(LOW_CURRENT_BASELINE_CYCLES);
  }

  display.setCursor(RIGHT_PANEL_X, 16 - (g_InfoScreenScroll * 8));
  display.print(F("BASE: "));
  if(g_RunSafety.baselineCurrentCycles >= LOW_CURRENT_BASELINE_CYCLES)
  {
    display.print(g_RunSafety.baselineCurrent_ma/1000, DEC);
    display.print(F("."));
    display.print((g_RunSafety.baselineCurrent_ma%1000)/100, DEC);
    display.print(F("A"));
  }
  else
  {
    display.print(F("--"));
  }

  if(g_InfoScreenScroll > 0)
  {
    refreshSupplyVoltage();
    display.setCursor(RIGHT_PANEL_X, 24 - (g_InfoScreenScroll * 8));
    display.print(F("5V: "));
    display.print(g_SupplyVoltage_mv/1000, DEC);
    display.print(F("."));
    display.print((g_SupplyVoltage_mv%1000)/100, DEC);
    display.print(F("V"));
  }

  if(g_InfoScreenScroll > 1)
  {
    display.setCursor(RIGHT_PANEL_X,16);
    display.print(F("FW: "));
    display.print(SOFTWARE_VERSION);
  }
  display.setCursor(RIGHT_PANEL_X,24);
  display.setTextColor(BLACK, WHITE);
  display.print(F("BACK >"));
  display.setTextColor(WHITE);
  display.drawLine(54,0,54,32,WHITE);
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
    display.print(F("W"));
  }
  else if(g_ResetDiagnostics.resetFlags & _BV(BORF))
  {
    display.print(F("B"));
  }
  else if(g_ResetDiagnostics.resetFlags & _BV(EXTRF))
  {
    display.print(F("E"));
  }
  else if(g_ResetDiagnostics.resetFlags & _BV(PORF))
  {
    display.print(F("P"));
  }
  else
  {
    display.print(F("-"));
  }

  display.print(F("|"));
  if(g_ResetDiagnostics.previousSystemState == STATE_ANNEALING)
  {
    display.print(F("A"));
  }
  else if(g_ResetDiagnostics.previousSystemState == STATE_DROPPING)
  {
    display.print(F("D"));
  }
  else if(g_ResetDiagnostics.previousSystemState == STATE_RELOADING)
  {
    display.print(F("R"));
  }
  else if(g_ResetDiagnostics.previousSystemState == STATE_COOLDOWN)
  {
    display.print(F("C"));
  }
  else if(g_ResetDiagnostics.previousSystemState == STATE_STOPPED)
  {
    display.print(F("S"));
  }
  else
  {
    display.print(F("?"));
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
static float readTemperature(uint8_t index)
{
  float t = sensors.getTempCByIndex(index);
  sensors.requestTemperatures(); // this takes quite some time to complete ~90ms or longer. read it on the next loop
  return t;
}

/*---------------------------------------------------------------------------*/
/*! @brief      Check whether a DS18B20 temperature reading is valid.
*//*-------------------------------------------------------------------------*/
static bool isTemperatureReadingValid(float const temperature)
{
  return temperature >= TEMP_SENSOR_MIN_C && temperature <= TEMP_SENSOR_MAX_C;
}
