#include "telemetry.h"
#include "telemetry_packet.h"
#include <string.h>

static UART_HandleTypeDef *telemetry_uart = NULL;
static uint8_t tx_buf[27];

// RX DMA Circular Buffer & State
#define RX_BUFFER_SIZE 128
static uint8_t rx_dma_buffer[RX_BUFFER_SIZE];

static FlightControl_t vehicle_cmd = {
    .roll = 0,
    .pitch = 0,
    .throttle = 1000, // Safe idle floor
    .yaw = 0,
    .armed = false,
    .flight_mode = 0
};

void Telemetry_Init(UART_HandleTypeDef *uart_handle) {
    telemetry_uart = uart_handle;
}

void Telemetry_Send_Attitude(Attitude_Data_t *attitude) {
	if (telemetry_uart == NULL) return;

	// Static buffer guarantees stable memory address for DMA


	// 1. Set Sync Headers
	tx_buf[0] = 0xAA;
	tx_buf[1] = 0x55;

	// 2. Copy Floats directly into the byte array starting at index 2
	memcpy(&tx_buf[2],  &attitude->roll,       4);
	memcpy(&tx_buf[6],  &attitude->pitch,      4);
	memcpy(&tx_buf[10], &attitude->yaw,        4);
	memcpy(&tx_buf[14], &attitude->rollspeed,  4);
	memcpy(&tx_buf[18], &attitude->pitchspeed, 4);
	memcpy(&tx_buf[22], &attitude->yawspeed,   4);

	// 3. Calculate XOR checksum over the payload bytes (indices 2 through 25)
	uint8_t sum = 0;
	for (int i = 2; i < 26; i++) {
		sum ^= tx_buf[i];
	}
	tx_buf[26] = sum;

	// 4. Fire DMA
	if (telemetry_uart->gState == HAL_UART_STATE_READY) {
		HAL_UART_Transmit_DMA(telemetry_uart, tx_buf, 27);
	}
}

void Telemetry_Start_Receive(void) {
    if (telemetry_uart == NULL) return;
    // Start circular DMA receiver with idle line detection
    HAL_UARTEx_ReceiveToIdle_DMA(telemetry_uart, rx_dma_buffer, RX_BUFFER_SIZE);
    // Disable half-transfer interrupt to eliminate unnecessary overhead
    __HAL_DMA_DISABLE_IT(telemetry_uart->hdmarx, DMA_IT_HT);
}

const FlightControl_t* Telemetry_Get_Control(void) {
    return &vehicle_cmd;
}

static void parse_uart_packet(uint8_t *buf, uint16_t len) {
    static enum {
        RX_WAIT_H1 = 0,
        RX_WAIT_H2,
        RX_WAIT_TYPE,
        RX_READ_PAYLOAD,
        RX_WAIT_CKSUM
    } state = RX_WAIT_H1;

    static uint8_t packet_buffer[16];
    static uint8_t payload_len = 0;
    static uint8_t rx_index = 0;
    static uint8_t pkt_type = 0;
    static uint8_t calc_cksum = 0;

    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = buf[i];

        switch (state) {
            case RX_WAIT_H1:
                if (byte == PKT_HEADER_1) {
                    calc_cksum = 0;
                    state = RX_WAIT_H2;
                }
                break;
            case RX_WAIT_H2:
                if (byte == PKT_HEADER_2) {
                    state = RX_WAIT_TYPE;
                } else {
                    state = (byte == PKT_HEADER_1) ? RX_WAIT_H2 : RX_WAIT_H1;
                }
                break;
            case RX_WAIT_TYPE:
                pkt_type = byte;
                calc_cksum ^= byte;
                rx_index = 0;

                if (pkt_type == PKT_TYPE_RC) {
                    payload_len = 8;
                } else if (pkt_type == PKT_TYPE_ARM || pkt_type == PKT_TYPE_MODE || pkt_type == PKT_TYPE_CALIB) {
                    payload_len = 1;
                } else {
                    state = RX_WAIT_H1;
                    break;
                }
                state = RX_READ_PAYLOAD;
                break;
            case RX_READ_PAYLOAD:
                packet_buffer[rx_index++] = byte;
                calc_cksum ^= byte;
                if (rx_index >= payload_len) {
                    state = RX_WAIT_CKSUM;
                }
                break;
            case RX_WAIT_CKSUM:
                if (byte == calc_cksum) {
                    if (pkt_type == PKT_TYPE_RC) {
                        memcpy(&vehicle_cmd.roll,     &packet_buffer[0], 2);
                        memcpy(&vehicle_cmd.pitch,    &packet_buffer[2], 2);
                        memcpy(&vehicle_cmd.throttle, &packet_buffer[4], 2);
                        memcpy(&vehicle_cmd.yaw,      &packet_buffer[6], 2);
                    }
                    else if (pkt_type == PKT_TYPE_ARM) {
                        uint8_t val = packet_buffer[0];
                        if (val == 2) {
                            vehicle_cmd.armed = false;
                        } else {
                            vehicle_cmd.armed = (val == 1);
                        }
                    }
                    else if (pkt_type == PKT_TYPE_MODE) {
                        vehicle_cmd.flight_mode = packet_buffer[0];
                    }
                }
                state = RX_WAIT_H1;
                break;
        }
    }
}

// STM32 HAL Idle Line DMA Callback override
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        parse_uart_packet(rx_dma_buffer, Size);
        // Restart DMA receiver for next burst
        HAL_UARTEx_ReceiveToIdle_DMA(telemetry_uart, rx_dma_buffer, RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(telemetry_uart->hdmarx, DMA_IT_HT);
    }
}
