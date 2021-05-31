/*
 * usbd_desc.h
 *
 *  Created on: May 17, 2021
 *      Author: atmat
 */

#ifndef APP_USBD_DESC_H_
#define APP_USBD_DESC_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

#define         DEVICE_ID1          (UID_BASE)
#define         DEVICE_ID2          (UID_BASE + 0x4)
#define         DEVICE_ID3          (UID_BASE + 0x8)

#define  USB_SIZ_STRING_SERIAL       0x1A

extern USBD_DescriptorsTypeDef FS_Desc;

#ifdef __cplusplus
}
#endif

#endif /* APP_USBD_DESC_H_ */
