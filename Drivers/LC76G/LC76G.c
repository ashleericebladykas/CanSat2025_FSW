#include "LC76G.h"
#include <string.h>
#include <math.h>

// Initialize global fields
uint8_t gps_dma_buffer[GPS_DMA_BUFFER_SIZE] = {0};
LC76G_gps_data gps_data;

void LC76G_init()
{
    // Disable all other types of NEMA sentences
    HAL_UART_Transmit(&huart5, LC76_DISABLE_GGL, strlen(LC76_DISABLE_GGL), TIMEOUT);
    HAL_UART_Receive(&huart5, NULL, 32, TIMEOUT);

    HAL_UART_Transmit(&huart5, LC76_DISABLE_GSA, strlen(LC76_DISABLE_GSA), TIMEOUT);
    HAL_UART_Receive(&huart5, NULL, 32, TIMEOUT);

    HAL_UART_Transmit(&huart5, LC76_DISABLE_GSV, strlen(LC76_DISABLE_GSV), TIMEOUT);
    HAL_UART_Receive(&huart5, NULL, 32, TIMEOUT);

    HAL_UART_Transmit(&huart5, LC76_DISABLE_RMC, strlen(LC76_DISABLE_RMC), TIMEOUT);
    HAL_UART_Receive(&huart5, NULL, 32, TIMEOUT);

    HAL_UART_Transmit(&huart5, LC76_DISABLE_VTG8, strlen(LC76_DISABLE_VTG8), TIMEOUT);
    HAL_UART_Receive(&huart5, NULL, 32, TIMEOUT);

    // Enable GGA sentences
    HAL_UART_Transmit(&huart5, LC76_ENABLE_GGA, strlen(LC76_ENABLE_GGA), TIMEOUT);
    HAL_UART_Receive(&huart5, NULL, 32, TIMEOUT);

    // Clear UART idle flag if needed
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    }

    // Enable DMA reception
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, gps_dma_buffer, GPS_DMA_BUFFER_SIZE);
}

//void LC76_receive_data() {
//    static size_t old_pos;
//    size_t pos;
//
//    /* Calculate current position in buffer and check for new data available */
//    pos = ARRAY_LEN(gps_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);
//    if (pos != old_pos) {                       /* Check change in received data */
//        if (pos > old_pos) {                    /* Current position is over previous one */
//            /*
//             * Processing is done in "linear" mode.
//             *
//             * Application processing is fast with single data block,
//             * length is simply calculated by subtracting pointers
//             *
//             * [   0   ]
//             * [   1   ] <- old_pos |------------------------------------|
//             * [   2   ]            |                                    |
//             * [   3   ]            | Single block (len = pos - old_pos) |
//             * [   4   ]            |                                    |
//             * [   5   ]            |------------------------------------|
//             * [   6   ] <- pos
//             * [   7   ]
//             * [ N - 1 ]
//             */
//            usart_process_data(&gps_dma_buffer[old_pos], pos - old_pos);
//        } else {
//            /*
//             * Processing is done in "overflow" mode..
//             *
//             * Application must process data twice,
//             * since there are 2 linear memory blocks to handle
//             *
//             * [   0   ]            |---------------------------------|
//             * [   1   ]            | Second block (len = pos)        |
//             * [   2   ]            |---------------------------------|
//             * [   3   ] <- pos
//             * [   4   ] <- old_pos |---------------------------------|
//             * [   5   ]            |                                 |
//             * [   6   ]            | First block (len = N - old_pos) |
//             * [   7   ]            |                                 |
//             * [ N - 1 ]            |---------------------------------|
//             */
//            usart_process_data(&gps_dma_buffer[old_pos], ARRAY_LEN(gps_dma_buffer) - old_pos);
//            if (pos > 0) {
//                usart_process_data(&gps_dma_buffer[0], pos);
//            }
//        }
//        old_pos = pos;                          /* Save current position as old for next transfers */
//    }
//}

void usart_process_data(const void* data, size_t len) {
    const uint8_t* d = data;
    
    /*
     * This function is called on DMA TC or HT events, and on UART IDLE (if enabled) event.
     * 
     * For the sake of this example, function does a loop-back data over UART in polling mode.
     * Check ringbuff RX-based example for implementation with TX & RX DMA transfer.
     */
    
    LC76G_parse_data();
}

// Used by the DMA event callback
void LC76G_parse_data() {
    // Find the start of the GGA sentence
    char *gga_start = strchr((char *)gps_dma_buffer, '$');
    if (!gga_start)
        return;

    // Find the end of the sentence
    char *gga_end = strchr(gga_start, '\n');
    if (!gga_end)
        return;

    // Copy the sentence to a buffer
    char GGA_sentence[GPS_DMA_BUFFER_SIZE] = {0};
    size_t len = gga_end - gga_start + 1;
    if (len > sizeof(GGA_sentence) - 1) // Safe truncation for null-termination
        len = sizeof(GGA_sentence) - 1;
    strncpy(GGA_sentence, gga_start, len);

    // Iterate through the comma-delimited string
    char *token = strtok(GGA_sentence, ",");
    uint8_t tokenNum = 1;
    while (token != NULL && strchr(token, '$') == NULL) {
        switch(tokenNum) {
            case 1: // <UTC>
                char temp1[3] = {0};
                strncpy(temp1, token, 2);        // Hours
                gps_data.time_H = strtol(temp1, NULL, 10);

                memset(temp1, 0, 3 * sizeof(char));
                strncpy(temp1, token + 2, 2);    // Minutes
                gps_data.time_M = strtol(temp1, NULL, 10);

                memset(temp1, 0, 3 * sizeof(char));
                strncpy(temp1, token + 4, 2);    // Seconds
                gps_data.time_M = strtol(temp1, NULL, 10);
                break;
            case 2: // <Lat>
                char temp2[8] = {0};
                strncpy(temp2, token, 2);        // Degrees
                gps_data.lat = (double)strtol(temp2, NULL, 10);

                memset(temp2, 0, 8 * sizeof(char));
                strncpy(temp2, token + 2, 2);    // Minutes
                gps_data.lat += (double)strtol(temp2, NULL, 10) / 60;

                memset(temp2, 0, 8 * sizeof(char));
                strncpy(temp2, token + 5, 6);    // Decimal minutes
                gps_data.lat += ((double)strtol(temp2, NULL) / pow(10, strlen(temp2))) / 60;
                break;
            case 3: // <N/S>
                // Flip sign of latitude if in southern hemisphere
                if (token[0] == 'S')
                    gps_data.lat = -gps_data.lat;
                break;
            case 4: // <Lon>
                char temp3[8] = {0};
                strncpy(temp3, token, 3);        // Degrees
                gps_data.lon = (double)strtol(temp3, NULL, 10);

                memset(temp3, 0, 8 * sizeof(char));
                strncpy(temp3, token + 1, 2);    // Minutes
                gps_data.lon += (double)strtol(temp3, NULL, 10) / 60;

                memset(temp3, 0, 8 * sizeof(char));
                strncpy(temp3, token + 6, 6);    // Decimal minutes
                gps_data.lon += ((double)strtol(temp3, NULL) / pow(10, strlen(temp3))) / 60;
                break;
            case 5: // <E/W>
                // Flip sign of latitude if in western hemisphere
                if (token[0] == 'W')
                    gps_data.lon = -gps_data.lon;
                break;
            case 7: // <NumSatUsed>
                gps_data.num_sat_used = strtol(token, NULL, 10);
                break;
            case 9: // <Alt>
                gps_data.altitude = strtol(token, NULL, 10);
                break;
            default:
                break;
        }   // End switch statement

        token = strtok(NULL, ",");
        tokenNum++;
    }   // End string iteration
}


LC76G_gps_data LC76G_read_data()
{
    /*
    // Zero-initalized buffer
    unsigned char GGA_sentence[128] = {0};

    // Receive GGA sentence from GPS module
    // HAL_UART_Receive(&huart5, GGA_sentence, 128, TIMEOUT);
    //HAL_UART_Transmit(&huart3, GGA_sentence, strlen(buf), TIMEOUT);
    //printf("Raw data from GPS module: %s", GGA_sentence);
    //HAL_UART_Transmit(&huart3, GGA_sentence, strlen(GGA_sentence), TIMEOUT);


    // GGA format:
    // $<TalkerID>GGA,<UTC>,<Lat>,<N/S>,<Lon>,<E/W>,<Quality>,<NumSatUsed>,
    // <HDOP>,<Alt>,M,<Sep>,M,<DiffAge>,<DiffStation>*<Checksum><CR><LF>

    // Iterate through the comma-delimited string
    char *token = strtok(GGA_sentence, ",");
    uint8_t tokenNum = 1;
  
    while (token != NULL && strchr(token, '$') == NULL) {
        switch(tokenNum) {
            case 1: // <UTC>
                char temp1[3] = {0};
                strncpy(temp1, token, 2);        // Hours
                gps_data.time_H = strtol(temp1, NULL, 10);

                memset(temp1, 0, 3 * sizeof(char));
                strncpy(temp1, token + 2, 2);    // Minutes
                gps_data.time_M = strtol(temp1, NULL, 10);

                memset(temp1, 0, 3 * sizeof(char));
                strncpy(temp1, token + 4, 2);    // Seconds
                gps_data.time_M = strtol(temp1, NULL, 10);
                break;
            case 2: // <Lat>
                char temp2[8] = {0};
                strncpy(temp2, token, 2);        // Degrees
                gps_data.lat = (double)strtol(temp2, NULL, 10);

                memset(temp2, 0, 8 * sizeof(char));
                strncpy(temp2, token + 2, 2);    // Minutes
                gps_data.lat += (double)strtol(temp2, NULL, 10) / 60;

                memset(temp2, 0, 8 * sizeof(char));
                strncpy(temp2, token + 5, 6);    // Decimal minutes
                gps_data.lat += ((double)strtol(temp2, NULL) / pow(10, strlen(temp2))) / 60;
                break;
            case 3: // <N/S>
                // Flip sign of latitude if in southern hemisphere
                if (token[0] == 'S')
                    gps_data.lat = -gps_data.lat;
                break;
            case 4: // <Lon>
                char temp3[8] = {0};
                strncpy(temp3, token, 3);        // Degrees
                gps_data.lon = (double)strtol(temp3, NULL, 10);

                memset(temp3, 0, 8 * sizeof(char));
                strncpy(temp3, token + 1, 2);    // Minutes
                gps_data.lon += (double)strtol(temp3, NULL, 10) / 60;

                memset(temp3, 0, 8 * sizeof(char));
                strncpy(temp3, token + 6, 6);    // Decimal minutes
                gps_data.lon += ((double)strtol(temp3, NULL) / pow(10, strlen(temp3))) / 60;
                break;
            case 5: // <E/W>
                // Flip sign of latitude if in western hemisphere
                if (token[0] == 'W')
                    gps_data.lon = -gps_data.lon;
                break;
            case 7: // <NumSatUsed>
                gps_data.num_sat_used = strtol(token, NULL, 10);
                break;
            case 9: // <Alt>
                gps_data.altitude = strtol(token, NULL, 10);
                break;
            default:
                break;
        }   // End switch statement
      
        token = strtok(NULL, ",");
        tokenNum++;
    }   // End string iteration
    */
    return gps_data;
}

/*void LC76G_Send_Command(char *data)
{
    HAL_UART_Transmit(&huart3, &data, strlen(data), TIMEOUT);
    HAL_Delay(10);
}
*/
