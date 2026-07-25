#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include "config.h"
#include "parser_utils.h"
#include "RadioLib.h"
#include "IWatchdog.h"
#include "messages.h"
#include "sd_writer.h"
#include "readings.h"
#include "message_parser.h"

//This function is difficult to weave into a class structure
void setFlag(void);

// Thin subclass exposing SX126x's protected diagnostics: the device-error
// register (why a modem operation failed) and the chip status byte (which mode
// the chip is actually in). Used to debug a transmit that never reaches TxDone.
class STM32WLx_Diag : public STM32WLx {
  public:
    STM32WLx_Diag(STM32WLx_Module* mod) : STM32WLx(mod) {}
    uint16_t deviceErrors() { return getDeviceErrors(); }
    int16_t  clearErrors()  { return clearDeviceErrors(); }
    uint8_t  chipStatus()   { return getStatus(); }  // bits 6:4 = mode (0x6 = TX)
};

static volatile bool operationDone = false;

 
static constexpr uint8_t end_message_size {15};

class LoRa_Transceiver{
  public:
    // Important constants
    uint32_t WiO_ID;
    uint32_t listenTime = 0;
    uint8_t baseStationID = 0;
    uint32_t startup_timestamp = 0;
    bool available = false;
    int16_t state;
    int16_t packet_count;
    uint32_t lastTransmission;
    uint16_t msgCounter = 0;
    
    // This variable is a LORA variable as we might give
    // instructions to the buoy in terms of measurement period
    uint32_t measurement_period = base_measurement_period;

    // Wave analysis is gated by its own runtime enable + period, kept on the LoRa
    // object so the base station can re-tune them like measurement_period. Defaults
    // come from common_config.h (shared by both targets).
    bool     enable_wave_analysis            = base_enable_wave_analysis;
    uint32_t measurement_period_wave_analysis = base_measurement_period_wave_analysis;

    // Setup functions
    void getWiOID(void);

    void computeReceptionChannel(uint8_t num_LoRa_channels, double fmin, double fmax);
    void beginRadio(double freq, double bw, int sf, int cr, int power);
    void setDefaultSendFrequency (double freq);
    void setBaseStationID(uint8_t id);

    // Sending
    void transmit(String msg);
    void transmit(const char * msg);
    void transmitB(byte * msg, uint8_t msgSize);
    Message_Data sendData(byte * message, uint8_t msgSize, uint32_t message_send_time);
    
    // Utility
    void waitUntilReady();
    void create_update_message(FrequencyMessage frequency_message);
    void changeFrequency(double);

    // Reception
    float getRSSI();
    void listen(uint32_t max_wait_time);
    void listenByteArray(uint32_t max_wait_time);

    // Handshake 
    void findBaseStation(uint32_t max_wait_time);
    bool handshake(uint32_t maxListenTime);
    bool connectToBaseStation(uint32_t find_timeout, uint32_t handshake_timeout);
    void transmitFinished(uint8_t packetsLeft);
    
    // Transmission wrap up
    void receiveInstructions(void);
    void receiveDesiredMeasurementIDs(void);
    void updateMeasurementFrequency(uint32_t velocity, uint32_t max_period, uint32_t min_period);
    void sendFinalMessage(FrequencyMessage frequency_message);
    
    // Power management
    void sleep(void);
    void wakeUp(void);

    // Recovery
    void transmitBeaconMessage(byte * beaconMsg, uint8_t msgSize);

    // Message containers
    StringMessage string_msg;
    ByteMessage byte_msg;
    byte end_message[end_message_size];

    //BST utils
    buoyInfo initEmptyBuoy();
    buoyInfo findBuoy(uint32_t);    
  private:
    STM32WLx_Diag radio = new STM32WLx_Module();
    const uint32_t rfswitch_pins[5] = {PB8, PC13, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};
    const Module::RfSwitchMode_t rfswitch_table[4] = {
      {STM32WLx::MODE_IDLE,  {LOW,  LOW}},
      {STM32WLx::MODE_RX,    {HIGH, LOW}},
      {STM32WLx::MODE_TX_HP, {LOW, HIGH}},
      END_OF_MODE_TABLE,
    };
    int16_t transmissionState = RADIOLIB_ERR_NONE;

    ByteMessage receivedMessage;
    bool listening = false;
    bool sleeping = false;
    float send_frequency;
    buoyInfo buoy;
};

extern LoRa_Transceiver LORA;
#endif