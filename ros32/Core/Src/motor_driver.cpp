#include "motor_driver.h"


void MotorDriver::init(UART_HandleTypeDef* left, UART_HandleTypeDef* right){
    leftPort = left;
    rightPort = right;
    uint16_t length = LEN_X_BAUD;
    uint8_t data [1] ={TARGET_BAUD};

    // set torque
    length = LEN_X_TORQUE_ENABLE;

    left_packet = (uint8_t *)malloc(length + 12 + (length / 3));
    right_packet = (uint8_t *)malloc(length + 12 + (length / 3));
    data[0] = TORQUE_ENABLE;

    makeData(left_packet, LEFT_ID, INST_WRITE, data, length, ADDR_X_TORQUE_ENABLE);
    makeData(right_packet, RIGHT_ID, INST_WRITE, data, length, ADDR_X_TORQUE_ENABLE);

    txPacket(left_packet, right_packet);

    free(left_packet);
    free(right_packet);


    // Instruction for reading encoder
    length = 2;

    readPacketL[PKT_ID] = LEFT_ID;
    readPacketL[PKT_LENGTH_L] = DXL_LOBYTE(2 + 5);
    readPacketL[PKT_LENGTH_H] = DXL_HIBYTE(2 + 5);
    readPacketL[PKT_INSTRUCTION] = INST_READ;
    readPacketL[PKT_PARAMETER0 + 0] = (uint8_t)DXL_LOBYTE(ADDR_X_PRESENT_POSITION);
    readPacketL[PKT_PARAMETER0 + 1] = (uint8_t)DXL_HIBYTE(ADDR_X_PRESENT_POSITION);
    readPacketL[PKT_PARAMETER0 + 2] = (uint8_t)DXL_LOBYTE(LEN_X_PRESENT_POSITION);
    readPacketL[PKT_PARAMETER0 + 3] = (uint8_t)DXL_HIBYTE(LEN_X_PRESENT_POSITION);

    addStuffing(readPacketL);

    read_packet_length_L = DXL_MAKEWORD(readPacketL[PKT_LENGTH_L],
    readPacketL[PKT_LENGTH_H]) + 7;
    readPacketL[PKT_HEADER0] = 0xFF;
    readPacketL[PKT_HEADER1] = 0xFF;
    readPacketL[PKT_HEADER2] = 0xFD;
    readPacketL[PKT_RESERVED] = 0x00;

    readPacketR[PKT_ID] = RIGHT_ID;
    readPacketR[PKT_LENGTH_L] = DXL_LOBYTE(2 + 5);
    readPacketR[PKT_LENGTH_H] = DXL_HIBYTE(2 + 5);
    readPacketR[PKT_INSTRUCTION] = INST_READ;
    readPacketR[PKT_PARAMETER0 + 0] = (uint8_t)DXL_LOBYTE(ADDR_X_PRESENT_POSITION);
    readPacketR[PKT_PARAMETER0 + 1] = (uint8_t)DXL_HIBYTE(ADDR_X_PRESENT_POSITION);
    readPacketR[PKT_PARAMETER0 + 2] = (uint8_t)DXL_LOBYTE(LEN_X_PRESENT_POSITION);
    readPacketR[PKT_PARAMETER0 + 3] = (uint8_t)DXL_HIBYTE(LEN_X_PRESENT_POSITION);

    addStuffing(readPacketR);

    read_packet_length_R = DXL_MAKEWORD(readPacketR[PKT_LENGTH_L],
    readPacketR[PKT_LENGTH_H]) + 7;
    readPacketR[PKT_HEADER0] = 0xFF;
    readPacketR[PKT_HEADER1] = 0xFF;
    readPacketR[PKT_HEADER2] = 0xFD;
    readPacketR[PKT_RESERVED] = 0x00;

    uint16_t crcL = updateCRC(0, readPacketL, read_packet_length_L - 2); // 2: CRC16
    readPacketL[read_packet_length_L - 2] = DXL_LOBYTE(crcL);
    readPacketL[read_packet_length_L - 1] = DXL_HIBYTE(crcL);

    uint16_t crcR = updateCRC(0, readPacketR, read_packet_length_R - 2); // 2: CRC16
    readPacketR[read_packet_length_R - 2] = DXL_LOBYTE(crcR);
    readPacketR[read_packet_length_R - 1] = DXL_HIBYTE(crcR);
}


void MotorDriver::makeData(uint8_t* packet, uint8_t id, uint8_t op, uint8_t* data, uint16_t dataLength, uint16_t address){
    packet[PKT_ID] = id;
    packet[PKT_LENGTH_L] = DXL_LOBYTE(dataLength + 5);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(dataLength + 5);
    packet[PKT_INSTRUCTION] = op;
    packet[PKT_PARAMETER0 + 0] = (uint8_t)DXL_LOBYTE(address);
    packet[PKT_PARAMETER0 + 1] = (uint8_t)DXL_HIBYTE(address);

    for (uint16_t s = 0; s < dataLength; s++)
        packet[PKT_PARAMETER0 + 2 + s] = data[s];
}

bool MotorDriver::readEncoder(int32_t &left_value, int32_t &right_value){
    uint8_t rxpacketL[100], rxpacketR[100];
    // Read Left Motor
    if (HAL_UART_Transmit(leftPort, readPacketL, read_packet_length_L, 10) == HAL_OK){
        HAL_HalfDuplex_EnableReceiver(leftPort);
        rxPacket(leftPort, rxpacketL);
        HAL_HalfDuplex_EnableTransmitter(leftPort);
        left_value = DXL_MAKEDWORD(DXL_MAKEWORD(rxpacketL[9 + 0], rxpacketL[9 +1]), DXL_MAKEWORD(rxpacketL[9 + 2], rxpacketL[9 + 3]));
    }
    // Read Right Motor
    if (HAL_UART_Transmit(rightPort, readPacketR, read_packet_length_R, 10) == HAL_OK){
        HAL_HalfDuplex_EnableReceiver(rightPort);
        rxPacket(rightPort, rxpacketR);
        HAL_HalfDuplex_EnableTransmitter(rightPort);
        right_value = DXL_MAKEDWORD(DXL_MAKEWORD(rxpacketR[9 + 0], rxpacketR[9 + 1]),
        DXL_MAKEWORD(rxpacketR[9 + 2], rxpacketR[9 + 3]));
    }
    return true;
}

bool MotorDriver::writeVelocity(int64_t left_value, int64_t right_value){
    uint16_t length = LEN_X_GOAL_VELOCITY;
    uint8_t left_data_byte[4] = {0};
    uint8_t right_data_byte[4] = {0};
    uint8_t* left_vel_packet;
    uint8_t* right_vel_packet;

    left_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(left_value));
    left_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(left_value));
    left_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(left_value));
    left_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(left_value));

    right_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(right_value));
    right_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(right_value));
    right_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(right_value));
    right_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(right_value));

    // Send Data
    left_vel_packet = (uint8_t *)malloc(length + 12 + (length / 3));
    right_vel_packet = (uint8_t *)malloc(length + 12 + (length / 3));
    makeData(left_vel_packet, LEFT_ID, INST_WRITE, left_data_byte, LEN_X_GOAL_VELOCITY, ADDR_X_GOAL_VELOCITY);
    makeData(right_vel_packet, RIGHT_ID, INST_WRITE, right_data_byte, LEN_X_GOAL_VELOCITY, ADDR_X_GOAL_VELOCITY);

    txPacket(left_vel_packet, right_vel_packet);

    free(left_vel_packet);
    free(right_vel_packet);

    return true;
}


bool MotorDriver::controlMotor(const float wheel_radius, const float wheel_separation, float* value){
    float wheel_velocity_cmd[2];
    float lin_vel = value[LINEAR];
    float ang_vel = value[ANGULAR];

    wheel_velocity_cmd[LEFT] = lin_vel - (ang_vel * wheel_separation / 2);
    wheel_velocity_cmd[RIGHT] = lin_vel + (ang_vel * wheel_separation / 2);

    // convert cmd_vel value
    wheel_velocity_cmd[LEFT] = constrain(wheel_velocity_cmd[LEFT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -DXL_LIMIT_MAX_VELOCITY, DXL_LIMIT_MAX_VELOCITY);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius, -DXL_LIMIT_MAX_VELOCITY, DXL_LIMIT_MAX_VELOCITY);

    if (writeVelocity((int64_t)wheel_velocity_cmd[LEFT], (int64_t)wheel_velocity_cmd[RIGHT]))
        return true;
    else
        return false;
}


unsigned short MotorDriver::updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    uint16_t i;
    static const uint16_t crc_table[256] = {0x0000,
    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
    0x820D, 0x8207, 0x0202 
    };
    for (uint16_t j = 0; j < data_blk_size; j++){
        i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}


void MotorDriver::addStuffing(uint8_t *packet){
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;

    if (packet_length_in < 8) // INSTRUCTION, ADDR_L, ADDR_H, CRC16_L, CRC16_H + FF FF FD
        return;

    uint8_t *packet_ptr;
    uint16_t packet_length_before_crc = packet_length_in - 2;

    for (uint16_t i = 3; i < packet_length_before_crc; i++) {
        packet_ptr = &packet[i+PKT_INSTRUCTION-2];
        if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD)
            packet_length_out++;
    }

    if (packet_length_in == packet_length_out) // no stuffing required
        return;

    uint16_t out_index = packet_length_out + 6 - 2; // last index before crc
    uint16_t in_index = packet_length_in + 6 - 2; // last index before crc

    while (out_index != in_index){
        if (packet[in_index] == 0xFD && packet[in_index-1] == 0xFF && packet[in_index-2] == 0xFF){
            packet[out_index--] = 0xFD; // byte stuffing
            if (out_index != in_index){
                packet[out_index--] = packet[in_index--]; // FD
                packet[out_index--] = packet[in_index--]; // FF
                packet[out_index--] = packet[in_index--]; // FF
            }
        }
        else{
        packet[out_index--] = packet[in_index--];
        }
    }
    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
    return;
}


void MotorDriver::removeStuffing(uint8_t *packet){
    int i = 0, index = 0;
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;
    index = PKT_INSTRUCTION;

    for (i = 0; i < packet_length_in - 2; i++){
        if (packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION+1] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF){ // FF FF FD FD
            packet_length_out--;
            i++;
        }
        packet[index++] = packet[i+PKT_INSTRUCTION];
    }

    packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
    packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];
    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}


int MotorDriver::txPacket(uint8_t *txpacketL, uint8_t *txpacketR){
    // byte stuffing for header
    addStuffing(txpacketL);
    addStuffing(txpacketR);
    // check max packet length
    uint16_t total_packet_length_left = DXL_MAKEWORD(txpacketL[PKT_LENGTH_L], txpacketL[PKT_LENGTH_H]) + 7;
    uint16_t total_packet_length_right = DXL_MAKEWORD(txpacketR[PKT_LENGTH_L], txpacketR[PKT_LENGTH_H]) + 7;

    // 7: HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H
    // make packet header for left motor
    txpacketL[PKT_HEADER0] = 0xFF;
    txpacketL[PKT_HEADER1] = 0xFF;
    txpacketL[PKT_HEADER2] = 0xFD;
    txpacketL[PKT_RESERVED] = 0x00;

    // make packet header for right motor
    txpacketR[PKT_HEADER0] = 0xFF;
    txpacketR[PKT_HEADER1] = 0xFF;
    txpacketR[PKT_HEADER2] = 0xFD;
    txpacketR[PKT_RESERVED] = 0x00;

    // add CRC16
    uint16_t crc_left = updateCRC(0, txpacketL, total_packet_length_left - 2); // 2: CRC16
    uint16_t crc_right = updateCRC(0, txpacketR, total_packet_length_right - 2); // 2: CRC16
    txpacketL[total_packet_length_left - 2] = DXL_LOBYTE(crc_left);
    txpacketL[total_packet_length_left - 1] = DXL_HIBYTE(crc_left);
    txpacketR[total_packet_length_right - 2] = DXL_LOBYTE(crc_right);
    txpacketR[total_packet_length_right - 1] = DXL_HIBYTE(crc_right);

    // tx packet
    HAL_UART_Transmit(leftPort, txpacketL, total_packet_length_left, 10);
    HAL_UART_Transmit(rightPort, txpacketR, total_packet_length_right, 10);

    return COMM_SUCCESS;
}


int MotorDriver::rxPacket(UART_HandleTypeDef* port, uint8_t *rxpacket){
    int result = COMM_TX_FAIL;
    uint16_t rx_length = 0;
    uint16_t wait_length = 11; // minimum length (HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H)
    uint8_t err = 0;

    while(true){
        err = 0;
        __HAL_UART_CLEAR_FLAG(port, UART_FLAG_RXNE);
        if (HAL_UART_Receive(port, &rxpacket[rx_length], 1, 10) != HAL_OK)
            err = 1;
        //break;

        if (err)
            continue;
        rx_length ++;

        if (rx_length >= wait_length){
            uint16_t idx = 0;
            // find packet header
            for (idx = 0; idx < (rx_length - 3); idx++){
                if ((rxpacket[idx] == 0xFF) && (rxpacket[idx+1] == 0xFF) && (rxpacket[idx+2] == 0xFD) && (rxpacket[idx+3] != 0xFD))
                break;
            }

            if (idx == 0){
                if (rxpacket[PKT_RESERVED] != 0x00 ||rxpacket[PKT_ID] > 0xFC || DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) > RXPACKET_MAX_LEN ||rxpacket[PKT_INSTRUCTION] != 0x55){
                    // remove the first byte in the packet
                    for (uint16_t s = 0; s < rx_length - 1; s++)
                        rxpacket[s] = rxpacket[1 + s];
                    
                    rx_length -= 1;
                    continue;
                }

                // re-calculate the length of the rx packet
                if (wait_length != DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1){
                    wait_length = DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1;
                    continue;
                }

                // verify CRC
                uint16_t crc = DXL_MAKEWORD(rxpacket[wait_length-2], rxpacket[wait_length-1]);
                if (updateCRC(0, rxpacket, wait_length - 2) == crc){
                    result = COMM_SUCCESS;
                }
                else {
                    result = COMM_RX_CORRUPT;
                }
                break;
            }
            else{
                // remove unnecessary packets
                for (uint16_t s = 0; s < rx_length - idx; s++)
                    rxpacket[s] = rxpacket[idx + s];
                rx_length -= idx;
            }
        }
    }
    if (result == COMM_SUCCESS)
        removeStuffing(rxpacket);
    return result;
}

// int MotorDriver::txRxPacket(uint8_t *txpacketL, uint8_t *txpacketR, uint8_t *rxpacketL, uint8_t *rxpacketR){
//     int result = COMM_TX_FAIL;
//     // tx packet
//     result = txPacket(txpacketL, txpacketR);
//     if (result != COMM_SUCCESS)
//         return result;
//     HAL_HalfDuplex_EnableReceiver(leftPort);
//     HAL_HalfDuplex_EnableReceiver(rightPort);

//     result = rxPacket(rxpacketL, rxpacketR);

//     HAL_HalfDuplex_EnableTransmitter(leftPort);
//     HAL_HalfDuplex_EnableTransmitter(rightPort);
//         return result;
//     }
