/*
 * usb_device.h
 *
 *  Created on: May 17, 2021
 *      Author: atmat
 */

#ifndef APP_USB_DEVICE_H_
#define APP_USB_DEVICE_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"
#include "stm32f3xx_hal.h"
#include "usbd_def.h"

/** USB Device initialization function. */
void MX_USB_DEVICE_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_USB_DEVICE_H_ */
