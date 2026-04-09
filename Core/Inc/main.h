/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define SPI3_NSS_Pin GPIO_PIN_4
#define SPI3_NSS_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_14
#define SPI1_CS_GPIO_Port GPIOD
#define USB_OverCurrent_Pin GPIO_PIN_5
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define STLK_RX_Pin GPIO_PIN_7
#define STLK_RX_GPIO_Port GPIOG
#define STLK_TX_Pin GPIO_PIN_8
#define STLK_TX_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi1 // USER DISK IO SPI HANDLE

// ========== Flash Configuration ==========
//#define FLASH_PAGE_SIZE	2048
#define FLASH_CONFIG_PAGE	255
#define FLASH_CONFIG_BANK	FLASH_BANK_2
#define FLASH_CONFIG_BASE	0x080FF800

#define FLASH_ADDR_CIRCULAR_SIZE          (FLASH_CONFIG_BASE + 0x00)
#define FLASH_ADDR_CIRCULAR_INTERVAL      (FLASH_CONFIG_BASE + 0x08)
#define FLASH_ADDR_INTEGRITY_INTERVAL     (FLASH_CONFIG_BASE + 0x10)

// ========== Global Config Variables ==========
extern uint64_t CIRCULAR_SIZE;
extern uint64_t CIRCULAR_INTERVAL;
extern uint64_t INTEGRITY_CHECK_INTERVAL;


// ========== Protocol Constants ==========
#define MSG_REQ  0
#define MSG_RESP 1

#define MOD_SD      0
#define MOD_FLASH   1
#define MOD_PAYLOAD 2

#define CMD_INIT_SD             0
#define CMD_DEINIT_SD           1

// ---------- TEST folder ----------
#define CMD_WRITE_SD_TEST       21
#define CMD_READ_SD_TEST        31
#define CMD_LIST_FILE_TEST      41
#define CMD_DEL_FILE_TEST       51
#define CMD_EDIT_FILE_TEST      81

// ---------- OBC folder ----------
#define CMD_WRITE_SD_OBC        22
#define CMD_READ_SD_OBC         32
#define CMD_LIST_FILE_OBC       42
#define CMD_DEL_FILE_OBC        52
#define CMD_EDIT_FILE_OBC       82

// ---------- PAYLOAD folder ----------
#define CMD_WRITE_SD_PAYLOAD    23
#define CMD_READ_SD_PAYLOAD     33
#define CMD_LIST_FILE_PAYLOAD   43
#define CMD_DEL_FILE_PAYLOAD    53
#define CMD_EDIT_FILE_PAYLOAD   83

#define CMD_READ_CONFIG         6
#define CMD_WRITE_CONFIG        7

#define RESP_OK         9
#define RESP_ERR        1
#define RESP_CONTINUE   2

// ========== Structures ==========
typedef struct {
    uint8_t type;
    uint8_t module_id;
    uint8_t cmd_id;
    uint16_t seq_id;
    uint16_t payload_len;
} __attribute__((packed)) msg_header_t;

// ========== Request Bodies ==========
typedef struct {
    char filename[32];
} __attribute__((packed)) cmd_read_sd_t;

typedef struct {
    char filename[32];
    uint8_t data[256];
    uint32_t data_len;
} __attribute__((packed)) cmd_write_sd_t;

typedef struct {
    char filename[32];
} __attribute__((packed)) cmd_delete_file_t;

typedef struct {
    uint64_t circular_size;
    uint64_t circular_interval;
    uint64_t integrity_check_interval;
} __attribute__((packed)) cmd_write_config_t;

typedef struct {
    char filename[32];
    uint32_t offset;
    uint8_t data[256];
    uint32_t data_len;
} __attribute__((packed)) cmd_edit_file_t;

// ========== Response Bodies ==========
typedef struct {
    uint8_t status;
} __attribute__((packed)) resp_common_t;

typedef struct {
    char filename[32];
} __attribute__((packed)) resp_list_file_t;

typedef struct {
    uint8_t data[256];
    uint32_t data_len;
    uint32_t crc32;
} __attribute__((packed)) resp_read_sd_t;

typedef struct {
    uint32_t crc32;
} __attribute__((packed)) resp_write_sd_t;

typedef struct {
    uint64_t circular_size;
    uint64_t circular_interval;
    uint64_t integrity_check_interval;
} __attribute__((packed)) resp_read_config_t;

// ========== Request Message ==========
typedef struct {
    msg_header_t header;
    union {
        cmd_read_sd_t read_sd;
        cmd_write_sd_t write_sd;
        cmd_delete_file_t delete_file;
        cmd_write_config_t write_config;
        cmd_edit_file_t edit_file;
    } body;
} __attribute__((packed)) msg_req_t;

// ========== Response Message ==========
typedef struct {
    msg_header_t header;
    union {
        resp_common_t common_resp;
        resp_list_file_t list_file_resp;
        resp_read_sd_t read_sd_resp;
        resp_write_sd_t write_sd_resp;
        resp_read_config_t read_config_resp;

    } body;
} __attribute__((packed)) msg_resp_t;

//typedef union {
//    msg_req_t req;
//    msg_resp_t resp;
//} __attribute__((packed)) msg_t;


typedef struct __attribute__((packed)) {
    int16_t temp_centi;   // 2534 = 25.34 C
} payload_msg_t;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
