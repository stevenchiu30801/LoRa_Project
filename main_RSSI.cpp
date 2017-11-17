/*
 *  This code refers to Mbed example - SX1276PingPong (https://os.mbed.com/teams/Semtech/code/SX1276PingPong/)
 *  The program uses LoRa to measure RSSI values.
 */
#include "mbed.h"
#include "main.h"
#include "sx1276-hal.h"
#include "debug.h"

/* Set this flag to '1' to display debug messages on the console */
#define DEBUG_MESSAGE   1

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    915000000 // Hz
#define TX_OUTPUT_POWER                                 14        // 14 dBm

#if USE_MODEM_LORA == 1

    #define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                                  //  1: 250 kHz,
                                                                  //  2: 500 kHz,
                                                                  //  3: Reserved]
    #define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
    #define LORA_CODINGRATE                             1         // [1: 4/5,
                                                                  //  2: 4/6,
                                                                  //  3: 4/7,
                                                                  //  4: 4/8]
    #define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
    #define LORA_SYMBOL_TIMEOUT                         5         // Symbols
    #define LORA_FIX_LENGTH_PAYLOAD_ON                  false
    #define LORA_FHSS_ENABLED                           false  
    #define LORA_NB_SYMB_HOP                            4     
    #define LORA_IQ_INVERSION_ON                        false
    #define LORA_CRC_ENABLED                            true

#elif USE_MODEM_FSK == 1

    #define FSK_FDEV                                    25000     // Hz
    #define FSK_DATARATE                                19200     // bps
    #define FSK_BANDWIDTH                               50000     // Hz
    #define FSK_AFC_BANDWIDTH                           83333     // Hz
    #define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
    #define FSK_FIX_LENGTH_PAYLOAD_ON                   false
    #define FSK_CRC_ENABLED                             true

#else
    #error "Please define a modem in the compiler options."
#endif

#define RX_TIMEOUT_VALUE                                3500      // in ms
#define BUFFER_SIZE                                     32        // Define the payload size here

#if( defined ( TARGET_KL25Z ) || defined ( TARGET_LPC11U6X ) )
DigitalOut led( LED2 );
#else
DigitalOut led( LED1 );
#endif

#define FILTER_FRAME_SIZE 10

/*
 *  Global variables declarations
 */
typedef enum
{
    LOWPOWER = 0,
    IDLE,

    RX,
    RX_TIMEOUT,
    RX_ERROR,

    TX,
    TX_TIMEOUT,

    CAD,
    CAD_DONE
}AppStates_t;

volatile AppStates_t State = LOWPOWER;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*
 *  Global variables declarations
 */
Serial pc(SERIAL_TX, SERIAL_RX);

SX1276MB1xAS Radio( NULL );

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;

int8_t frame_idx = 0;
int16_t rssi_frame[FILTER_FRAME_SIZE] = {0};

int16_t average_filter(){
    int32_t sum_rssi = 0;
    for(int i = 0; i < FILTER_FRAME_SIZE; i++)
        sum_rssi += rssi_frame[i];

    return sum_rssi / FILTER_FRAME_SIZE;
}

int16_t median_filter(){
    int16_t tmp, rssi_cpy[FILTER_FRAME_SIZE];
    int8_t i, j;
    for(i = 0; i < FILTER_FRAME_SIZE; i++)
        rssi_cpy[i] = rssi_frame[i];

    // Bubble sort
    for(i = 0; i < FILTER_FRAME_SIZE - 1; i++)
        for(j = i; j < FILTER_FRAME_SIZE; j++)
            if(rssi_cpy[i] > rssi_cpy[j]){
                tmp = rssi_cpy[i];
                rssi_cpy[i] = rssi_cpy[j];
                rssi_cpy[j] = tmp;
            }

    return rssi_cpy[FILTER_FRAME_SIZE / 2];
}

int main( void ) 
{
    debug( "\n\n\r     SX1276 RSSI Measurement \n\n\r" );

    // Initialize Radio driver
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init( &RadioEvents );

    // verify the connection with the board
    while( Radio.Read( REG_VERSION ) == 0x00  )
    {
        debug( "Radio could not be detected!\n\r", NULL );
        wait( 1 );
    }

    debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1276MB1LAS ) ), "\n\r > Board Type: SX1276MB1LAS < \n\r" );
    debug_if( ( DEBUG_MESSAGE & ( Radio.DetectBoardType( ) == SX1276MB1MAS ) ), "\n\r > Board Type: SX1276MB1MAS < \n\r" );

    Radio.SetChannel( RF_FREQUENCY ); 

#if USE_MODEM_LORA == 1

    debug_if( LORA_FHSS_ENABLED, "\n\n\r             > LORA FHSS Mode < \n\n\r" );
    debug_if( !LORA_FHSS_ENABLED, "\n\n\r             > LORA Mode < \n\n\r" );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                         LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                         LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                         LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                         LORA_IQ_INVERSION_ON, 2000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                         LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                         LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                         LORA_CRC_ENABLED, LORA_FHSS_ENABLED, LORA_NB_SYMB_HOP,
                         LORA_IQ_INVERSION_ON, true );

#elif USE_MODEM_FSK == 1

    debug("\n\n\r              > FSK Mode < \n\n\r" );
    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                         FSK_DATARATE, 0,
                         FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                         FSK_CRC_ENABLED, 0, 0, 0, 2000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                         0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                         0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, FSK_CRC_ENABLED,
                         0, 0, false, true );

#else

#error "Please define a modem in the compiler options."

#endif

    debug_if( DEBUG_MESSAGE, "Starting RSSI measuring loop\r\n" );

    led = 0;

    pc.format(8, Serial::None, 1);
    pc.baud(9600);

    int16_t filter_rssi;
    Radio.Rx(0);

    while(1)
    {
        switch(State)
        {
        case RX:
            if(BufferSize > 0){
                BufferSize = 0;
                // rssi = Radio.GetRssi(MODEM_LORA);
                // printf("instant rssi: %d\n", RssiValue);
                rssi_frame[frame_idx] = RssiValue;
                frame_idx = (++frame_idx) % FILTER_FRAME_SIZE;
                if(frame_idx == 0){
                    filter_rssi = average_filter();
                    printf("%d\n", filter_rssi);
                }
                wait_ms(100);
            }
            Radio.Rx(0);
            State = LOWPOWER;
            break;
        case RX_ERROR:
            Radio.Rx(0);
            State = LOWPOWER;
            break;
        case LOWPOWER:
            break;
        default:
            State = LOWPOWER;
            break;
        }
    }
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
    debug_if( DEBUG_MESSAGE, "> OnTxDone\n\r" );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    // Radio.Sleep();
    BufferSize = size;
    // memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    State = RX;
    // debug_if( DEBUG_MESSAGE, "> OnRxDone\n\r" );
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
    debug_if( DEBUG_MESSAGE, "> OnTxTimeout\n\r" );
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    Buffer[BufferSize] = 0;
    State = RX_TIMEOUT;
    debug_if( DEBUG_MESSAGE, "> OnRxTimeout\n\r" );
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    debug_if( DEBUG_MESSAGE, "> OnRxError\n\r" );
}
