/*
 * charger_uart.c
 *
 *  Created on: 20 May 2026
 *      Author: amrlxyz
 */


#define uartHandle huart2

#define UART_RX_BUF_SZ 1024   // UART circular DMA RX buffer size
#define UART_TX_BUF_SZ 10000

#include "charger_uart.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdbool.h"
#include "main.h"
#include "usart.h"


typedef enum {
    STATE_WAITING_FOR_START,
    STATE_RECEIVING_DATA
} RxState;


static const uint8_t BYTE_START = '\\';
static const uint8_t BYTE_STOP  = '\n';

volatile uint32_t rxframe_tail = 0;
volatile uint32_t rxframe_len = 0;

uint8_t rxbuffer[UART_RX_BUF_SZ];
uint8_t rxframebuf[UART_RX_BUF_SZ];

RxState rxState = STATE_WAITING_FOR_START;


void uart_init(void){
    HAL_UARTEx_ReceiveToIdle_DMA(&uartHandle, rxbuffer, UART_RX_BUF_SZ);
}


volatile uint32_t uart_rx_hits;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

    if (huart == &uartHandle)
    {
        uart_rx_hits++;

        while (Size != rxframe_tail)
        {
            uint8_t b = rxbuffer[rxframe_tail++];

            switch (rxState) {

            case STATE_WAITING_FOR_START:
                if (b == BYTE_START) {
                    rxState = STATE_RECEIVING_DATA;
                    rxframe_len = 0;
                }
                // If it's not a start byte, we just drop it (clears corrupt data)
                break;

            case STATE_RECEIVING_DATA:
                // Prevent buffer overflow if the BYTE_STOP was corrupted/lost
                if (rxframe_len >= UART_RX_BUF_SZ) {
                    rxState = STATE_WAITING_FOR_START;

                    // Edge case: Maybe THIS byte is actually a new start byte?
                    if (b == BYTE_START) {
                        rxState = STATE_RECEIVING_DATA;
                        rxframe_len = 0;
                    }
                    break;
                }


                // Store the incoming byte
                rxframebuf[rxframe_len++] = b;

                // Check if the frame is complete
                if (b == BYTE_STOP) {
                    // We have a complete, bounded frame!
                    uart_parseRxFrame(rxframebuf, rxframe_len);

                    // Reset to wait for the next packet
                    rxState = STATE_WAITING_FOR_START;
                }
                break;

            default:
                break;
            }

            if (rxframe_tail > UART_RX_BUF_SZ){
                rxframe_tail = 0;
            }
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &uartHandle){
        // Restart UART after error
        HAL_UART_AbortReceive(&uartHandle);
        HAL_UARTEx_ReceiveToIdle_DMA(&uartHandle, rxbuffer, UART_RX_BUF_SZ);
        rxframe_tail = 0;
        rxframe_len = 0;
        printfDma("UART ERROR Detected\n");
    }
}

// -------------- //

char txbuffer[UART_TX_BUF_SZ] = {0};
volatile int head = 0;
volatile int tail = 0;

static uint32_t bufferFreeSize = 0;

bool isFull = false;
bool isWrapped = false;

int tailDma = 0; // Stores the tail index of the buffer being sent


int getDataLen(void)
{
    int tempTail = tail;

    if (head >= tempTail)
    {
        return head - tempTail; // Data is in a single contiguous block
    }
    else
    {
        return UART_TX_BUF_SZ - tempTail + head; // Data wraps around the buffer
    }
}


void startUartDmaTx(void)
{
    // Get the data len up to the buffer size only
    int dataLen = isWrapped ? UART_TX_BUF_SZ - tail : head - tail;

    // get pointer to the tail data
    uint8_t* dataPtr = (uint8_t*)(txbuffer + tail);

    tailDma = tail + dataLen;

    HAL_UART_Transmit_DMA(&uartHandle, dataPtr, dataLen);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart)
{
    if (huart == &uartHandle)
    {
        tail = tailDma;
        if (tail == UART_TX_BUF_SZ)
        {
            tail = 0;
            isWrapped = false;
        }
        if (head > tail)
        {
            startUartDmaTx();
        }
    }
}


// Function to append formatted data to the ring buffer
int printfDma(const char *format, ...)
{
    bufferFreeSize = UART_TX_BUF_SZ - getDataLen();

    const int TEMP_BUFF_SIZE = 256;

    char temp_buffer[TEMP_BUFF_SIZE];
    va_list args;
    va_start(args, format);
    int written = vsnprintf(temp_buffer, TEMP_BUFF_SIZE, format, args);
    va_end(args);

    if (written < 0)
    {
        return -1; // Error in formatting
    }
    else if (written > TEMP_BUFF_SIZE)
    {
        // Over limit of temp buffer
        Error_Handler();
    }
    else if (getDataLen() + written > UART_TX_BUF_SZ)
    {
        // Buffer full
//        Error_Handler();
    }

    for (int i = 0; i < written; i++)
    {
        txbuffer[head++] = temp_buffer[i];

        if (head == UART_TX_BUF_SZ)
        {
            isWrapped = true;
            head = 0;
        }

        if (head == tail)
        {
            printfDma("\n\n UART BUFFER FULL DATA OVERWRITTEN \n\n");
//            Error_Handler();
        }
    }

    uint32_t uart_state = HAL_UART_GetState(&uartHandle);

    // Check if not busy with TX
    if ((uart_state == HAL_UART_STATE_BUSY_RX) ||
        (uart_state == HAL_UART_STATE_READY)) {
        startUartDmaTx();
    }

    return written;
}



void printfFlushBuffer(void)
{
    if (HAL_UART_GetState(&uartHandle) == HAL_UART_STATE_READY)
    {
        startUartDmaTx();
    }
}



















