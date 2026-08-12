#ifndef CONFIG_H
#define CONFIG_H
#include "Arduino.h"

/*
  The console is the Serial instance in main.cpp (USART1 on PB6/PB7, the
  debug header on this PCB). The variant's own `Serial` is an unused LPUART1 on
  PA2/PA3, so alias it here and every existing Serial.print lands on the header.
  USART1 supports only this orientation: PB6 is in PinMap_UART_TX, PB7 in
  PinMap_UART_RX. Swapping them leaves the line undriven.
*/

static constexpr uint8_t BUOY_MODE {0};
static constexpr uint8_t BST_MODE {1};
static constexpr uint8_t MOORED_MODE {2};

static constexpr uint8_t WIO_MODE {BST_MODE}; 

// Base station parameters
static constexpr uint8_t base_station_ID            {102};
static constexpr int max_message_length             {255};

// SD card parameters
const int8_t DISABLE_CS_PIN = -1;

/*
  Bus wiring. Same PCB as the drifter, so these must match its config.h.

  SPI1 is shared by the SD card and the LSM6DSVTR IMU, so both chip selects
  must be driven high before either slave is addressed. RF_SWITCH pins are the
  RAK3172 module's antenna switch, per the core's variant_RAK3172_MODULE.h.
*/
static constexpr uint32_t SPI_MOSI_PIN                   {PA10};
static constexpr uint32_t SPI_MISO_PIN                   {PB14};
static constexpr uint32_t SPI_SCK_PIN                    {PB13};
static constexpr uint32_t SPI_CS_SD_PIN                  {PB9};
static constexpr uint32_t I2C_SDA_PIN                    {PIN_WIRE_SDA};
static constexpr uint32_t I2C_SCL_PIN                    {PIN_WIRE_SCL};

// Radio parameters
static constexpr float LoRa_freq_send               {863};


/*
  LoRa_freq_receive is the target frequency to be used to receive
  messages from a buoy. If set to negative, it will be computed pseudorandomly
  otherwise, the fixed value will be used. 
*/
static constexpr float LoRa_freq_receive_min        {864.00};
static           float LoRa_freq_receive            {-1};
static constexpr float LoRa_freq_receive_max        {867.00};

static constexpr uint8_t num_LoRa_channels          {20};
static constexpr float LoRa_bw                      {125.0};
static constexpr int LoRa_sf                        {8};
static constexpr int LoRa_cr                        {6};
static constexpr int LoRa_power                     {20};
static constexpr uint32_t max_radio_fix_look_time   {60000};
static constexpr uint32_t max_radio_wait_time       {40000};
static constexpr uint32_t listen_time               {30000};

// Enable or disable parameters
static constexpr int debug_serial                   {1};
static constexpr int debug_SD                       {1};
static constexpr bool enable_handshake              {true};

/*
  Whether a received 'P' message is dumped ONE LINE PER BIN.

  Off here, because that dump is not free the way the other debug prints are. Every
  debugSerialPrint does a debugFile.sync(), and the bin loop makes five calls per bin
  - about 510 syncs for a 102-bin spectrum, which blocks the receive loop for seconds.

  That matters now that a wave measurement arrives as TWO packets. listenByteArray
  clears operationDone on entry, so an RxDone that fires while the console is busy is
  discarded and the packet is never read out of the radio. With a drifter moving
  straight on to the next queued result after its 'P', a spectrum dump is long enough
  to swallow the 'W' that follows it.

  The summary line above the loop - bin count, f_min, f_max, df, max_value - is
  printed either way, so a spectrum that arrived and parsed still says so. The bins
  reach the cloud regardless; the uplink forwards raw bytes and never prints.

  The drifter's bench fixture keeps this ON (see drifter/src/config.h): there the dump
  IS the test, and nothing is racing it.
*/
static constexpr bool debug_print_psd_bins          {false};

// Debugging parameters
static constexpr int serial_baud                    {115200};

// Drifters parameters
static constexpr int max_number_of_thermometres          {8};
static constexpr uint32_t scale_factor              {100000};

// Latitude/longitude keep the receiver's native 1e-7 deg and travel signed,
// see the drifter config for why
static constexpr int32_t  gps_coord_scale           {10000000};

// Watchdog parameters
static constexpr bool enable_watchdog               {true};
static constexpr uint32_t watchdog_wait_time        {32000};


// Notecard parameters
static constexpr int output_sync_frequency          {30}; // 30 minutes
static constexpr int input_sync_frequency           {30}; // 30 minutes
static constexpr int health_frequency               {30 * 60 * 1000}; // 30 minutes
static constexpr int reset_frequency                {120 * 60 * 1000}; // 2 h

// Adaptive frequency parameters
static constexpr bool get_frequency_from_notehub            {false}; 
static           uint32_t base_measurement_period           {10*60*1000}; // 10 minutes
static           bool default_update_frequency              {false};
static           bool enable_motion_detection               {false};
static           uint32_t target_reading_distance             {100};
static           uint32_t motion_treshold                   {2000}; // m/s, divided by 10^4

// Buoy rescue parameters
static constexpr bool enable_rescue_from_notehub            {true}; 
static constexpr float LoRa_freq_beacon                     {868.00};

/*
  Capacity of wave_spectrum_Reading::wave_spectrum, i.e. the most bins this station
  can DECODE out of a 'P' message. It is not a wire contract - the sender states its
  own num_bins and the parser clamps to this - but a spectrum longer than this is
  printed truncated, so it must be at least what the drifters actually send.

  102 is the fork drifter's welch_bins. The format ceiling is 114: a 'P' message is
  26 + 2*num_bins bytes and the SX126x payload-length field is one byte, so 255 is
  hard. Raising this beyond 114 trips the static_assert in message_parser.h.

  Forwarding is unaffected either way - the uplink copies numBytes raw and never
  looks at the spectrum - so a station running short here still delivers a full
  spectrum to the cloud and only shortchanges its own console.
*/
static constexpr size_t welch_bins {102};

#endif