#pragma once

// Compile one production sketch for either the ATmega328P Uno family or the
// Renesas RA4M1 Uno R4 family. Board-specific code is selected at compile time
// so the unused backend contributes no flash or SRAM to the other target.
#if defined(__AVR_ATmega328P__)
  #define NZHS_PLATFORM_UNO_R3 1
  #define NZHS_PLATFORM_UNO_R4 0
  #define NZHS_HAS_LED_MATRIX 0
  #define NZHS_HAS_WIFI 0
  #include <avr/io.h>
  #include <avr/wdt.h>
  #include <util/atomic.h>
#elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  #define NZHS_PLATFORM_UNO_R3 0
  #define NZHS_PLATFORM_UNO_R4 1
  #include <FspTimer.h>
  #include <Servo.h>
  #include "r_wdt.h"

  #if defined(ARDUINO_UNOR4_WIFI)
    #define NZHS_HAS_LED_MATRIX 1
    #define NZHS_HAS_WIFI 1
    #include <Arduino_LED_Matrix.h>
    #include <WiFiS3.h>
  #else
    #define NZHS_HAS_LED_MATRIX 0
    #define NZHS_HAS_WIFI 0
  #endif

  #ifndef BUFFER_LENGTH
    #define BUFFER_LENGTH I2C_BUFFER_LENGTH
  #endif

  // Preserve the interrupt-enable state around data shared with the feeder
  // timer callback, matching ATOMIC_RESTORESTATE on AVR.
  class AnnealerAtomicGuard
  {
  public:
    AnnealerAtomicGuard() : interruptState(__get_PRIMASK()), active(true)
    {
      __disable_irq();
    }

    ~AnnealerAtomicGuard()
    {
      finish();
    }

    operator bool() const
    {
      return active;
    }

    void finish()
    {
      if(active)
      {
        if((interruptState & 1U) == 0)
        {
          __enable_irq();
        }
        active = false;
      }
    }

  private:
    uint32_t interruptState;
    bool active;
  };

  #define ATOMIC_RESTORESTATE 0
  #define ATOMIC_BLOCK(type) \
    for(AnnealerAtomicGuard annealerAtomicGuard; annealerAtomicGuard; \
        annealerAtomicGuard.finish())

  // Reset diagnostics use the existing compact AVR-shaped flag byte so the
  // common diagnostics renderer remains unchanged.
  #ifndef _BV
    #define _BV(bit) (1U << (bit))
  #endif
  #define PORF 0
  #define EXTRF 1
  #define BORF 2
  #define WDRF 3
#else
  #error "NZHS Annealer supports ATmega328P Uno boards and Arduino Uno R4 boards"
#endif
