/*
    Driver for 0.96in 128 * 64 px OLED with ssd1306 controller via I2C. 
    This file contains only the example function calls implemented in the 
    file ssd1306_driver.c
*/

/*
* Temporary Notes :
* R = 0x02
* B = 0x04
* G = 0x08
* Y = 0x0A
*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "ssd1306_driver.h"

int main( void )
{
    // Initializations:
    I2C_enable() ;
    I2C3_setup() ;
    PORTD_init() ;
    oled_init() ;
    oledClear() ;

    while(1) {
        oledPrintStr("Rishabh") ;
    }
}
