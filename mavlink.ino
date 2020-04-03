// Include MAVLink header
// Remember that it's necessary to init and update git submodule
#include "c_library_v2/common/mavlink.h"

void setup() {
  // MAVLink serial port
  Serial.begin(57600); // default baudrate for QGC

  // Enable LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Send heartbeat via serial port
  send_heartbeat();

  // Check reception buffer
  decode_messages();

}

void send_heartbeat() {
   static uint32_t last_time = millis();
   const uint32_t current_time = millis();
   constexpr const uint32_t heartbeat_interval = 1000; // 1 second
   if (current_time - last_time > heartbeat_interval) {
      last_time = current_time; // Update for the next loop
      
      static mavlink_message_t mavlink_message;
      static uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
      static uint16_t mavlink_message_length = 0;
      
      if (mavlink_message_length == 0) { // Create message if not
        const int system_id = 1;
        const int component_id = 1;
        const int mavlink_type = MAV_TYPE_GENERIC;
        const int autopilot_type = MAV_AUTOPILOT_INVALID;
        const int system_mode = MAV_MODE_PREFLIGHT;
        const int custom_mode = 0x0000; // No flag
        const int mavlink_state = MAV_STATE_ACTIVE;
        mavlink_msg_heartbeat_pack(
          system_id, component_id, &mavlink_message, mavlink_type, autopilot_type, system_mode, custom_mode, mavlink_state
        );
        mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &mavlink_message);
      }
      Serial.write(mavlink_message_buffer, mavlink_message_length);
   }
}

void decode_messages() {
  static mavlink_message_t message;
  static mavlink_status_t status;

  // Read all data available in serial port and parse the mavlink message
  while(Serial.available() > 0) {
    uint8_t serial_byte = Serial.read();

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, serial_byte, &message, &status)) {
      
      // Handle message
      switch(message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            // Blink when a HEARTBEAT is received
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            break;
        // Add new messages here
        default:
            break;        
      }
    }
  }
}
