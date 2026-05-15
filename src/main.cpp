#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Ticker.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#define SPEED_PIN 5
#define WIFI_CHANNEL 1

// Sampling frequency
#define SAMPLE_INTERVAL_MS 100

// Speed sensor tuning
#define FILTER_WEIGHT 0.40f // Parameterized smoothing
// Precomputed: 10,000,000 / 1.139 (PULSE_FREQ_TO_MPH). Yields speed in 0.1 MPH units
// when used as: (acc_pulses * K_SPEED_X10) / acc_period_us
static const uint32_t K_SPEED_X10 = 8779631UL;
#define SPEED_DEADZONE_US 2000UL
#define SNAP_TO_ZERO_US 500000UL
#define OUTPUT_KPH 0 // Set to 1 to output KPH instead of MPH

static uint8_t receiver_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct __attribute__((packed)) SpeedPacket
{
  uint8_t unit;   // 'M' or 'K'
  uint16_t speed; // speed in 0.1 unit increments (e.g. 657 = 65.7 MPH)
};

// ---------------------------------------------------------------------------
// Logic Globals
// ---------------------------------------------------------------------------

static volatile uint32_t s_acc_period_us = 0;
static volatile uint32_t s_acc_pulses = 0;
static volatile uint32_t s_last_pulse_us = 0;

static float s_smoothed_mph = 0.0f;
static float s_last_valid_mph = 0.0f;
static Ticker s_sample_ticker;

static uint32_t s_last_update_us = 0;

static volatile bool s_tick_flag = false;
static uint32_t s_last_send_ok_ms = 0;
static const uint32_t RADIO_WATCHDOG_MS = 10000;

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

static void on_send(uint8_t *mac, uint8_t status)
{
  if (status == 0)
    s_last_send_ok_ms = millis();
}

static void set_tick_flag() { s_tick_flag = true; }

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------

void ICACHE_RAM_ATTR speed_isr()
{
  uint32_t now = micros();
  if ((now - s_last_pulse_us) > SPEED_DEADZONE_US)
  {
    if (s_last_pulse_us > 0)
    {
      s_acc_period_us += (now - s_last_pulse_us);
      s_acc_pulses++;
    }
    s_last_pulse_us = now;
  }
}

// ---------------------------------------------------------------------------
// Test Mode Logic (Dynamically scaled by SAMPLE_INTERVAL_MS)
// ---------------------------------------------------------------------------

struct TestCycle
{
  int duration_s;
  int max_speed;
};
static const TestCycle TEST_CYCLES[] = {
    {30, 120}, // 30s ramp
    {20, 120}, // 20s ramp
    {15, 60},  // 15s ramp
};
static const int NUM_TEST_CYCLES = 3;

static volatile bool s_test_mode = false;
static int s_test_tick = 0;
static float s_test_speed = 0.0f;

static void test_mode_tick()
{
  s_test_tick++;

  // Calculate total ticks needed based on SAMPLE_INTERVAL_MS
  int rem = s_test_tick;
  int cycle;
  for (cycle = 0; cycle < NUM_TEST_CYCLES; cycle++)
  {
    int cycle_ticks = (TEST_CYCLES[cycle].duration_s * 1000) / SAMPLE_INTERVAL_MS;
    if (rem <= cycle_ticks)
      break;
    rem -= cycle_ticks;
  }

  if (cycle >= NUM_TEST_CYCLES)
  {
    s_test_tick = 0;
    return;
  }

  int total_cycle_ticks = (TEST_CYCLES[cycle].duration_s * 1000) / SAMPLE_INTERVAL_MS;
  int half = total_cycle_ticks / 2;
  int max = TEST_CYCLES[cycle].max_speed;

  s_test_speed = (float)((rem <= half)
                             ? ((float)rem * max) / half
                             : ((float)(total_cycle_ticks - rem) * max) / half);
}

// ---------------------------------------------------------------------------
// Sampling & Delivery
// ---------------------------------------------------------------------------

static void sample_and_send()
{
  float current_mph;
  uint32_t now = micros();

  if (s_test_mode)
  {
    test_mode_tick();
    current_mph = s_test_speed;
    s_last_valid_mph = current_mph;

    // Keep accumulators clear while in test mode to prevent a flood of stale data when test mode ends
    noInterrupts();
    s_acc_period_us = 0;
    s_acc_pulses = 0;
    s_last_pulse_us = 0;
    interrupts();

    s_last_update_us = now;
  }
  else
  {
    // 1. Safely grab accumulators
    noInterrupts();
    uint32_t acc_period = s_acc_period_us;
    uint32_t acc_pulses = s_acc_pulses;
    uint32_t last_pulse = s_last_pulse_us;

    if (acc_pulses > 0)
    {
      s_acc_period_us = 0;
      s_acc_pulses = 0;
    }

    uint32_t time_since_last = (last_pulse > 0) ? (now - last_pulse) : 0;

    // SAFE TIMEOUT RESET: Must be inside noInterrupts to prevent ISR race conditions
    if (time_since_last > SNAP_TO_ZERO_US)
    {
      s_last_pulse_us = 0;
      last_pulse = 0;
    }
    interrupts();

    // 2. Calculate Raw Speed
    if (acc_pulses > 0)
    {
      current_mph = (float)((uint64_t)acc_pulses * K_SPEED_X10 / acc_period) * 0.1f;

      // --- SIMULATOR GLITCH FILTER ---
      // Catch skipped/elongated pulses by enforcing real-world physics caps.
      if (s_last_update_us > 0)
      {
        float dt = (float)(now - s_last_update_us) / 1000000.0f;
        if (dt > 0.01f && dt < 0.2f)
        {
          float max_drop = 35.0f * dt; // Max braking threshold (~1.7 MPH per 50ms)
          float max_jump = 25.0f * dt; // Max acceleration threshold (~1.2 MPH per 50ms)

          if (current_mph < (s_last_valid_mph - max_drop))
          {
            current_mph = s_last_valid_mph - max_drop; // Ignore the dropped pulse
          }
          else if (current_mph > (s_last_valid_mph + max_jump))
          {
            current_mph = s_last_valid_mph + max_jump; // Ignore the false double-pulse
          }
        }
      }
      s_last_valid_mph = current_mph;
    }
    else if (last_pulse == 0)
    {
      // Complete stop
      current_mph = 0.0f;
      s_last_valid_mph = 0.0f;
    }
    else
    {
      // No pulses this window: Decay check with 50% buffer
      if (s_last_valid_mph > 0.5f)
      {
        uint32_t expected_period_us = K_SPEED_X10 / (uint32_t)(s_last_valid_mph * 10.0f);

        if (time_since_last > (expected_period_us + (expected_period_us >> 1)))
        {
          current_mph = (float)(K_SPEED_X10 / time_since_last) * 0.1f;
        }
        else
        {
          current_mph = s_last_valid_mph; // Hold steady
        }
      }
      else
      {
        current_mph = 0.0f;
      }
    }

    s_last_update_us = now;
  }

  // 3. Apply parametric smoothing
  s_smoothed_mph = (current_mph * FILTER_WEIGHT) + (s_smoothed_mph * (1.0f - FILTER_WEIGHT));

  // Hard snap to zero to prevent the exponential moving average floating point from decaying infinitely above 0.0
  if (current_mph < 0.5f && s_smoothed_mph < 0.5f)
  {
    s_smoothed_mph = 0.0f;
  }

  // 4. Dispatch
  SpeedPacket pkt;
#if OUTPUT_KPH
  pkt.unit = 'K';
  pkt.speed = (uint16_t)(s_smoothed_mph * 16.09344f + 0.5f);
#else
  pkt.unit = 'M';
  pkt.speed = (uint16_t)(s_smoothed_mph * 10.0f + 0.5f);
#endif
  esp_now_send(receiver_mac, (uint8_t *)&pkt, sizeof(pkt));
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != 0)
  {
    ESP.restart();
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(on_send);
  esp_now_add_peer(receiver_mac, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);

  pinMode(SPEED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_PIN), speed_isr, FALLING);

  s_sample_ticker.attach_ms(SAMPLE_INTERVAL_MS, set_tick_flag);
  s_last_send_ok_ms = millis();

  Serial.println("VSS System Initialized at 100ms intervals.");
}

void loop()
{
  if (s_tick_flag)
  {
    s_tick_flag = false;
    sample_and_send();
  }

  if (Serial.available())
  {
    char c = Serial.read();
    if (c == 't' || c == 'T')
    {
      if (s_test_mode)
      {
        s_test_mode = false;
        Serial.println("Test mode OFF");
      }
      else
      {
        s_test_tick = 0;
        s_test_mode = true;
        Serial.println("Test mode ON");
      }
    }
  }

  if (millis() - s_last_send_ok_ms > RADIO_WATCHDOG_MS)
  {
    Serial.println("Watchdog: radio hang detected, rebooting.");
    delay(100);
    ESP.restart();
  }

  yield();
}