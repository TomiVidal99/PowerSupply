#ifndef DEBUG_GUARD_H
#define DEBUG_GUARD_H

/* - - - - - - - - - -  ADC - - - - - - - - - -
 The ADC it's debugged through the PIN:

    PB1 (DEBUG_UI_PIN)

It shows how much it takes to make all the convertions.
It triggers when the conversions start, and ends when
the convertions end.
*/

/* - - - - - - - - - -  UI - - - - - - - - - -
The UI it's debugged through the PIN:

    PB2 (DEBUG_ADC_PIN)
    
It shows the time the UI it's updating.
HIGH when starts updating.
LOW when it ends.
*/

#include <avr/io.h>

#define DEBUG_ADC_PIN (PB1)
#define DEBUG_UI_PIN (PB2)

#endif // DEBUG_GUARD_H