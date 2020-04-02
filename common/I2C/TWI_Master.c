
#ifdef  __4MHZ__
#define F_CPU 4000000UL
#else
#define F_CPU 8000000UL
#endif
#include <avr/io.h>
#include "TWI_Master.h"
#include <util/delay.h>

#if defined(__AVR_AT90Mega169__) | defined(__AVR_ATmega169PA__) | \
defined(__AVR_AT90Mega165__) | defined(__AVR_ATmega165__) | \
defined(__AVR_ATmega325__) | defined(__AVR_ATmega3250__) | \
defined(__AVR_ATmega645__) | defined(__AVR_ATmega6450__) | \
defined(__AVR_ATmega329__) | defined(__AVR_ATmega3290__) | \
defined(__AVR_ATmega649__) | defined(__AVR_ATmega6490__) |\
defined(__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined(__AVR_ATtiny85__) | \
defined(__AVR_AT90Tiny26__) | defined(__AVR_ATtiny26__) |\
defined(__AVR_AT90Tiny2313__) | defined(__AVR_ATtiny2313__) |\
defined(__AVR_ATtiny84__) | defined(__AVR_ATtiny84A__)

unsigned char USI_TWI_Master_Transfer( unsigned char );
unsigned char USI_TWI_Master_Stop( void );

union  USI_TWI_state
{
  unsigned char errorState;         // Can reuse the TWI_state for error states due to that it will not be need if there exists an error.
  struct
  {
    unsigned char addressMode         : 1;
    unsigned char masterWriteDataMode : 1;
    unsigned char unused              : 6;
  }; 
}   USI_TWI_state;

/*---------------------------------------------------------------
 USI TWI single master initialization function
---------------------------------------------------------------*/
void TWI_Master_Initialise( void )
{
  PORT_USI |= (1<<PIN_USI_SDA);           // Enable pullup on SDA, to set high as released state.
  PORT_USI |= (1<<PIN_USI_SCL);           // Enable pullup on SCL, to set high as released state.
  
  DDR_USI  |= (1<<PIN_USI_SCL);           // Enable SCL as output.
  DDR_USI  |= (1<<PIN_USI_SDA);           // Enable SDA as output.
  
  USIDR    =  0xFF;                       // Preload dataregister with "released level" data.
  USICR    =  (0<<USISIE)|(0<<USIOIE)|                            // Disable Interrupts.
              (1<<USIWM1)|(0<<USIWM0)|                            // Set USI in Two-wire mode.
              (1<<USICS1)|(0<<USICS0)|(1<<USICLK)|                // Software stobe as counter clock source
              (0<<USITC);
  USISR   =   (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Clear flags,
              (0x0<<USICNT0);                                     // and reset counter.
}

/*---------------------------------------------------------------
Use this function to get hold of the error message from the last transmission
---------------------------------------------------------------*/
unsigned char USI_TWI_Get_State_Info( void )
{
  return ( USI_TWI_state.errorState );                            // Return error state.
}

/*---------------------------------------------------------------
 USI Transmit and receive function. LSB of first byte in data 
 indicates if a read or write cycles is performed. If set a read
 operation is performed.

 Function generates (Repeated) Start Condition, sends address and
 R/W, Reads/Writes Data, and verifies/sends ACK.
 
 Success or error code is returned. Error codes are defined in 
 USI_TWI_Master.h
---------------------------------------------------------------*/
unsigned char USI_TWI_Start_Transceiver_With_Data( unsigned char *msg, unsigned char msgSize)
{
  unsigned char tempUSISR_8bit = (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Prepare register value to: Clear flags, and
                                 (0x0<<USICNT0);                                     // set USI to shift 8 bits i.e. count 16 clock edges.
  unsigned char tempUSISR_1bit = (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Prepare register value to: Clear flags, and
                                 (0xE<<USICNT0);                                     // set USI to shift 1 bit i.e. count 2 clock edges.

  USI_TWI_state.errorState = 0;
  USI_TWI_state.addressMode = TRUE;

#ifdef PARAM_VERIFICATION
  if(msg > (unsigned char*)RAMEND)                 // Test if address is outside SRAM space
  {
    USI_TWI_state.errorState = USI_TWI_DATA_OUT_OF_BOUND;
    return (FALSE);
  }
  if(msgSize <= 1)                                 // Test if the transmission buffer is empty
  {
    USI_TWI_state.errorState = USI_TWI_NO_DATA;
    return (FALSE);
  }
#endif

#ifdef NOISE_TESTING                                // Test if any unexpected conditions have arrived prior to this execution.
  if( USISR & (1<<USISIF) )
  {
    USI_TWI_state.errorState = USI_TWI_UE_START_CON;
    return (FALSE);
  }
  if( USISR & (1<<USIPF) )
  {
    USI_TWI_state.errorState = USI_TWI_UE_STOP_CON;
    return (FALSE);
  }
  if( USISR & (1<<USIDC) )
  {
    USI_TWI_state.errorState = USI_TWI_UE_DATA_COL;
    return (FALSE);
  }
#endif

  if ( !(*msg & (1<<TWI_READ_BIT)) )                // The LSB in the address byte determines if is a masterRead or masterWrite operation.
  {
    USI_TWI_state.masterWriteDataMode = TRUE;
  }

/* Release SCL to ensure that (repeated) Start can be performed */
  PORT_USI |= (1<<PIN_USI_SCL);                     // Release SCL.
  while( !(PIN_USI & (1<<PIN_USI_SCL)) );          // Verify that SCL becomes high.
#ifdef TWI_FAST_MODE
  _delay_us( T4_TWI/4 );                         // Delay for T4TWI if TWI_FAST_MODE
#else
  _delay_us( T2_TWI/4 );                         // Delay for T2TWI if TWI_STANDARD_MODE
#endif

/* Generate Start Condition */
  PORT_USI &= ~(1<<PIN_USI_SDA);                    // Force SDA LOW.
  _delay_us( T4_TWI/4 );                         
  PORT_USI &= ~(1<<PIN_USI_SCL);                    // Pull SCL LOW.
  PORT_USI |= (1<<PIN_USI_SDA);                     // Release SDA.

#ifdef SIGNAL_VERIFY
  if( !(USISR & (1<<USISIF)) )
  {
    USI_TWI_state.errorState = USI_TWI_MISSING_START_CON;  
    return (FALSE);
  }
#endif

/*Write address and Read/Write data */
  do
  {
    /* If masterWrite cycle (or inital address tranmission)*/
    if (USI_TWI_state.addressMode || USI_TWI_state.masterWriteDataMode)
    {
      /* Write a byte */
      PORT_USI &= ~(1<<PIN_USI_SCL);                // Pull SCL LOW.
      USIDR     = *(msg++);                        // Setup data.
      USI_TWI_Master_Transfer( tempUSISR_8bit );    // Send 8 bits on bus.
      
      /* Clock and verify (N)ACK from slave */
      DDR_USI  &= ~(1<<PIN_USI_SDA);                // Enable SDA as input.
      if( USI_TWI_Master_Transfer( tempUSISR_1bit ) & (1<<TWI_NACK_BIT) ) 
      {
        if ( USI_TWI_state.addressMode )
          USI_TWI_state.errorState = USI_TWI_NO_ACK_ON_ADDRESS;
        else
          USI_TWI_state.errorState = USI_TWI_NO_ACK_ON_DATA;
        return (FALSE);
      }
      USI_TWI_state.addressMode = FALSE;            // Only perform address transmission once.
    }
    /* Else masterRead cycle*/
    else
    {
      /* Read a data byte */
      DDR_USI   &= ~(1<<PIN_USI_SDA);               // Enable SDA as input.
      *(msg++)  = USI_TWI_Master_Transfer( tempUSISR_8bit );

      /* Prepare to generate ACK (or NACK in case of End Of Transmission) */
      if( msgSize == 1)                            // If transmission of last byte was performed.
      {
        USIDR = 0xFF;                              // Load NACK to confirm End Of Transmission.
      }
      else
      {
        USIDR = 0x00;                              // Load ACK. Set data register bit 7 (output for SDA) low.
      }
      USI_TWI_Master_Transfer( tempUSISR_1bit );   // Generate ACK/NACK.
    }
  }while( --msgSize) ;                             // Until all data sent/received.
  
  USI_TWI_Master_Stop();                           // Send a STOP condition on the TWI bus.

/* Transmission successfully completed*/
  return (TRUE);
}

/*---------------------------------------------------------------
 Core function for shifting data in and out from the USI.
 Data to be sent has to be placed into the USIDR prior to calling
 this function. Data read, will be return'ed from the function.
---------------------------------------------------------------*/
unsigned char USI_TWI_Master_Transfer( unsigned char temp )
{
  USISR = temp;                                     // Set USISR according to temp.
                                                    // Prepare clocking.
  temp  =  (0<<USISIE)|(0<<USIOIE)|                 // Interrupts disabled
           (1<<USIWM1)|(0<<USIWM0)|                 // Set USI in Two-wire mode.
           (1<<USICS1)|(0<<USICS0)|(1<<USICLK)|     // Software clock strobe as source.
           (1<<USITC);                              // Toggle Clock Port.
  do
  {
    _delay_us( T2_TWI/4 );              
    USICR = temp;                          // Generate positve SCL edge.
    while( !(PIN_USI & (1<<PIN_USI_SCL)) );// Wait for SCL to go high.
    _delay_us( T4_TWI/4 );              
    USICR = temp;                          // Generate negative SCL edge.
  }while( !(USISR & (1<<USIOIF)) );        // Check for transfer complete.
  
  _delay_us( T2_TWI/4 );                
  temp  = USIDR;                           // Read out data.
  USIDR = 0xFF;                            // Release SDA.
  DDR_USI |= (1<<PIN_USI_SDA);             // Enable SDA as output.

  return temp;                             // Return the data from the USIDR
}

/*---------------------------------------------------------------
 Function for generating a TWI Stop Condition. Used to release 
 the TWI bus.
---------------------------------------------------------------*/
unsigned char USI_TWI_Master_Stop( void )
{
  PORT_USI &= ~(1<<PIN_USI_SDA);           // Pull SDA low.
  PORT_USI |= (1<<PIN_USI_SCL);            // Release SCL.
  while( !(PIN_USI & (1<<PIN_USI_SCL)) );  // Wait for SCL to go high.
  _delay_us( T4_TWI/4 );               
  PORT_USI |= (1<<PIN_USI_SDA);            // Release SDA.
  _delay_us( T2_TWI/4 );                
  
#ifdef SIGNAL_VERIFY
  if( !(USISR & (1<<USIPF)) )
  {
    USI_TWI_state.errorState = USI_TWI_MISSING_STOP_CON;    
    return (FALSE);
  }
#endif

  return (TRUE);
}



unsigned char I2c_WriteByte(unsigned char msg) {
  unsigned char tempUSISR_8bit = (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Prepare register value to: Clear flags, and
  (0x0<<USICNT0);                                     // set USI to shift 8 bits i.e. count 16 clock edges.
  unsigned char tempUSISR_1bit = (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Prepare register value to: Clear flags, and
  (0xE<<USICNT0);                                     // set USI to shift 1 bit i.e. count 2 clock edges.

	/* Write a byte */
	PORT_USI &= ~(1<<PIN_USI_SCL);                // Pull SCL LOW.
	USIDR     = msg;                        // Setup data.
	USI_TWI_Master_Transfer( tempUSISR_8bit );    // Send 8 bits on bus.
	/* Clock and verify (N)ACK from slave */
	DDR_USI  &= ~(1<<PIN_USI_SDA);                // Enable SDA as input.
	if( USI_TWI_Master_Transfer( tempUSISR_1bit ) & (1<<TWI_NACK_BIT) ){
		if ( USI_TWI_state.addressMode )
			USI_TWI_state.errorState = USI_TWI_NO_ACK_ON_ADDRESS;
		else
			USI_TWI_state.errorState = USI_TWI_NO_ACK_ON_DATA;
		return 2;
	}
	return 0;
}
unsigned char I2c_ReadByte(unsigned char ack_mode) {
  unsigned char tempUSISR_8bit = (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Prepare register value to: Clear flags, and
  (0x0<<USICNT0);                                     // set USI to shift 8 bits i.e. count 16 clock edges.
  unsigned char tempUSISR_1bit = (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Prepare register value to: Clear flags, and
  (0xE<<USICNT0);                                     // set USI to shift 1 bit i.e. count 2 clock edges.
	
	/* Read a data byte */
	DDR_USI   &= ~(1<<PIN_USI_SDA);               // Enable SDA as input.
    unsigned char msg = USI_TWI_Master_Transfer( tempUSISR_8bit );

    /* Prepare to generate ACK (or NACK in case of End Of Transmission) */
    if( ack_mode == NO_ACK) {                           // If transmission of last byte was performed.
		USIDR = 0xFF;                              // Load NACK to confirm End Of Transmission.
    } else   {
		USIDR = 0x00;                              // Load ACK. Set data register bit 7 (output for SDA) low.
    }
    USI_TWI_Master_Transfer( tempUSISR_1bit );   // Generate ACK/NACK.
	return msg;
}

void I2c_StartCondition(void) {
/* Release SCL to ensure that (repeated) Start can be performed */
PORT_USI |= (1<<PIN_USI_SCL);                     // Release SCL.
while( !(PIN_USI & (1<<PIN_USI_SCL)) );          // Verify that SCL becomes high.
#ifdef TWI_FAST_MODE
_delay_us( T4_TWI/4 );                         // Delay for T4TWI if TWI_FAST_MODE
#else
_delay_us( T2_TWI/4 );                         // Delay for T2TWI if TWI_STANDARD_MODE
#endif

/* Generate Start Condition */
PORT_USI &= ~(1<<PIN_USI_SDA);                    // Force SDA LOW.
_delay_us( T4_TWI/4 );
PORT_USI &= ~(1<<PIN_USI_SCL);                    // Pull SCL LOW.
PORT_USI |= (1<<PIN_USI_SDA);                     // Release SDA.
	
	
}
void I2c_StopCondition(void) {
	USI_TWI_Master_Stop();
}

#endif




#if defined(__AVR_ATmega328PB__)| defined(__AVR_ATmega328P__)


#include <util/twi.h>

void TWI_Wait_Busy() {
	_delay_us(100);
	while (!( TWCR & (1<<TWINT) )); 
}


void TWI_Master_Initialise( void ) {
	TWBR = TWI_TWBR;                                  // Set bit rate register (Baud rate). Defined in header file.Driver presumes prescaler to be 00.	TWSR=0;	TWDR = 0xFF;                                      // Default content = SDA released.	TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins.		(0<<TWIE)|(0<<TWINT)|                      // Disable Interrupt.		(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests.		(0<<TWWC);
		
}

unsigned char I2c_WriteByte(unsigned char msg) {
	uint8_t   twst;	TWDR = msg;	TWCR = (1<<TWINT) | (1<<TWEN);	TWI_Wait_Busy();	 // check value of TWI Status Register. Mask prescaler bits	 twst = TW_STATUS & 0xF8;	 if( twst != TW_MT_DATA_ACK) return 1;	return 0;
}
unsigned char I2c_ReadByte(unsigned char ack_mode) {
	TWCR = (1<<TWINT) | (1<<TWEN) |ack_mode;	TWI_Wait_Busy();    return TWDR;

}
void I2c_StartCondition(void) {
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	TWI_Wait_Busy();
	uint8_t twst = TW_STATUS & 0xF8;	if ( (twst != TW_START) && (twst != TW_REP_START)) return ;// return 1;}
/*
// send device address
TWDR = address;
TWCR = (1<<TWINT) | (1<<TWEN);

// wail until transmission completed and ACK/NACK has been received
while(!(TWCR & (1<<TWINT)));

// check value of TWI Status Register. Mask prescaler bits.
twst = TW_STATUS & 0xF8;
if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

return 0;

}
*/
void I2c_StopCondition(void) {

   TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
   while(TWCR & (1<<TWSTO));

}

#endif