#include <Arduino.h>
#include <driver/twai.h>

twai_general_config_t twai_general_cfg = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_38, GPIO_NUM_35, TWAI_MODE_NORMAL);
twai_timing_config_t twai_speed_cfg = TWAI_TIMING_CONFIG_25KBITS();

// We want to accept 7DF and 7E0.
// 7DF: 0x0111 0x1101 0x1111
// 7E0: 0x0111 0x1110 0x0000
// Longest common prefix: 0x0111 0x1100 0x0000
// Acceptance code: 0x7C0 << 21
// Acceptance mask: ~(0x7C0 << 21)
twai_filter_config_t twai_filters_cfg = {
        .acceptance_code = ((uint32_t) 0x7C0 << 21),
        .acceptance_mask = ~((uint32_t) 0x7C0 << 21),
        .single_filter = true
};

// Flow control queue that stores multi-frame message packets.
uint8_t can_flow_queue[5][8];

// These values are not dynamically updated, for now.
static const float rpm = 1200;
static const float speed = 60;
static const char vehicle_vin[18] = "ESP32OBD2EMULATOR";

void setup() {
    Serial.begin(115200);

    if (twai_driver_install(&twai_general_cfg, &twai_speed_cfg, &twai_filters_cfg) == ESP_OK) {
        Serial.println("Driver installed");
    } else {
        Serial.println("Failed to install driver");
    }

    if (twai_start() == ESP_OK) {
        Serial.println("Driver started");
    } else {
        Serial.println("Failed to start driver");
    }
}

void reply(twai_message_t &tx_msg) {
    if (twai_transmit(&tx_msg, portMAX_DELAY) == ESP_OK) {
        Serial.println("Message transmitted.");
    } else {
        Serial.println("Failed to transmit message.");
    }
}

void reply(uint8_t mode, uint8_t pid, size_t bytes, uint8_t A, uint8_t B, uint8_t C, uint8_t D) {
    twai_message_t tx_msg;
    tx_msg.identifier = 0x7E8;
    tx_msg.data_length_code = 8;
    tx_msg.rtr = 0;
    tx_msg.extd = 0;
    tx_msg.data[0] = bytes;
    tx_msg.data[1] = mode | 0x40;
    tx_msg.data[2] = pid;
    tx_msg.data[3] = A;
    tx_msg.data[4] = B;
    tx_msg.data[5] = C;
    tx_msg.data[6] = D;

    reply(tx_msg);
}

void loop() {
    Serial.println("Waiting for message...");

    twai_message_t rx_msg;
    twai_receive(&rx_msg, portMAX_DELAY);

    if (rx_msg.identifier == 0x7df) {
        // Diagnostic request message.

        uint8_t mode = rx_msg.data[1];
        uint8_t pid = rx_msg.data[2];

        Serial.printf("Mode: %02X, PID: %02X\n", mode, pid);

        switch (mode) {
            case 0x01: // show current data
                switch (pid) {
                    case 0x00: // supported pids(0x01-0x20)
                        reply(mode, pid, 4, 0b00000000, 0b00011000, 0b00000000, 0b00000000);
                        break;

                    case 0x0C: // engine rpm
                        reply(mode, pid, 2, (unsigned int) ((long) (rpm * 4) / 256),
                              (unsigned int) ((long) (rpm * 4) % 256), 0, 0);
                        break;

                    case 0x0D: // vehicle speed
                        reply(mode, pid, 1, (uint8_t) speed, 0, 0, 0);
                        break;

                    case 0x20:
                    case 0x40:
                    case 0x60:
                    case 0x80:
                    case 0xA0:
                    case 0xC0:
                    case 0xE0:
                        reply(mode, pid, 4, 0, 0, 0, 0);
                        break;
                }
                break;

            case 0x09: // vehicle information
                switch (pid) {
                    case 0x00: // supported pids(0x01-0x20)
                        reply(mode, pid, 4, 0b01000000, 0b00000000, 0b00000000, 0b00000000);
                        break;
                    case 0x02: // vehicle identification number
                        // Initiate multi-frame message packet
                        twai_message_t tx_msg;
                        tx_msg.identifier = 0x7E8;
                        tx_msg.data_length_code = 8;
                        tx_msg.rtr = 0;
                        tx_msg.extd = 0;
                        tx_msg.data[0] = 0x10; // FF (First Frame, ISO_15765-2)
                        tx_msg.data[1] = 0x14; // Length (20 bytes)
                        tx_msg.data[2] = 0x49; // Mode (+ 0x40)
                        tx_msg.data[3] = 0x02; // PID
                        tx_msg.data[4] = 0x01; // Data byte 1
                        tx_msg.data[5] = vehicle_vin[0]; // Data byte 2
                        tx_msg.data[6] = vehicle_vin[1]; // Data byte 3
                        tx_msg.data[7] = vehicle_vin[2]; // Data byte 4
                        reply(tx_msg);

                        // Fill flow control queue
                        // Part 1
                        can_flow_queue[0][0] = 0x21; // CF (Consecutive Frame, ISO_15765-2), sequence number
                        can_flow_queue[0][1] = vehicle_vin[3]; // Data byte 1
                        can_flow_queue[0][2] = vehicle_vin[4]; // Data byte 2
                        can_flow_queue[0][3] = vehicle_vin[5]; // Data byte 3
                        can_flow_queue[0][4] = vehicle_vin[6]; // Data byte 4
                        can_flow_queue[0][5] = vehicle_vin[7]; // Data byte 5
                        can_flow_queue[0][6] = vehicle_vin[8]; // Data byte 6
                        can_flow_queue[0][7] = vehicle_vin[9]; // Data byte 7
                        // Part 2
                        can_flow_queue[1][0] = 0x22; // CF
                        can_flow_queue[1][1] = vehicle_vin[10]; // Data byte 1
                        can_flow_queue[1][2] = vehicle_vin[11]; // Data byte 2
                        can_flow_queue[1][3] = vehicle_vin[12]; // Data byte 3
                        can_flow_queue[1][4] = vehicle_vin[13]; // Data byte 4
                        can_flow_queue[1][5] = vehicle_vin[14]; // Data byte 5
                        can_flow_queue[1][6] = vehicle_vin[15]; // Data byte 6
                        can_flow_queue[1][7] = vehicle_vin[16]; // Data byte 7
                        break;
                }
                break;
        }
    } else if (rx_msg.identifier == 0x7e0) {
        // ECU targeted message.

        if (rx_msg.data[0] == 0x30) {
            // Flow control message.

            twai_message_t tx_msg;
            tx_msg.identifier = 0x7E8;
            tx_msg.data_length_code = 8;
            tx_msg.rtr = 0;
            tx_msg.extd = 0;

            // Send messages in the flow control queue.
            for (auto &data: can_flow_queue) {
                if (data[0] == 0) {
                    continue;
                }

                for (int j = 0; j < 8; j++) {
                    tx_msg.data[j] = data[j];
                }

                reply(tx_msg);
            }

            // Clear the queue.
            memset(can_flow_queue, 0, 40);
        }
    }
}
