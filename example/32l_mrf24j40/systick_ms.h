/* 
 * File:   systick_ms.h
 * Author: karlp
 *
 * Created on December 9, 2011, 11:04 PM
 */

#ifndef SYSTICK_MS_H
#define	SYSTICK_MS_H

#ifdef	__cplusplus
extern "C" {
#endif

    int64_t millis(void);
    void delay_ms(int ms);
    void systick_ms_init(void);
    
#ifdef	__cplusplus
}
#endif

#endif	/* SYSTICK_MS_H */

