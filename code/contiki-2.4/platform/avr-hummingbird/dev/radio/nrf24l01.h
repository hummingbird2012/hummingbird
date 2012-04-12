/*   Copyright (c) 2011, M2M SIG
 *  All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of the copyright holders nor the names of
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *    \addtogroup nrf24l01
 *   @{
 */
/**
 *  \file
 *  \brief This file contains radio driver code.
 *
 *   $Id: nrf24l01.h,v 1.1 2011/08/20 16:14:44 nick Exp $
 */

#ifndef _NRF24L01_H_
#define _NRF24L01_H_
/*============================ INCLUDE =======================================*/
#include <stdint.h>
#include <stdbool.h>
#include <util/crc16.h>

/*============================ MACROS ========================================*/
#define SUPPORTED_PART_NUMBER                   ( 2 )
#define RF230_REVA                              ( 1 )
#define RF230_REVB                              ( 2 )
#define SUPPORTED_MANUFACTURER_ID               ( 31 )
#define RF230_SUPPORTED_INTERRUPT_MASK          ( 0x0C )

#define RF230_MIN_CHANNEL                       ( 11 )
#define RF230_MAX_CHANNEL                       ( 26 )
#define RF230_MIN_ED_THRESHOLD                  ( 0 )
#define RF230_MAX_ED_THRESHOLD                  ( 15 )
#define NRF24L01_MAX_TX_FRAME_LENGTH               ( 96 ) /**< 127 Byte PSDU. */

#define TX_PWR_0DBM                             ( 0x3 )
#define TX_PWR_6DBM                          ( 0x2 )
#define TX_PWR_12DBM                          ( 0x1 )
#define TX_PWR_18DBM                          ( 0x0 )

#define BATTERY_MONITOR_HIGHEST_VOLTAGE         ( 15 )
#define BATTERY_MONITOR_VOLTAGE_UNDER_THRESHOLD ( 0 )
#define BATTERY_MONITOR_HIGH_VOLTAGE            ( 1 )
#define BATTERY_MONITOR_LOW_VOLTAGE             ( 0 )

#define FTN_CALIBRATION_DONE                    ( 0 )
#define PLL_DCU_CALIBRATION_DONE                ( 0 )
#define PLL_CF_CALIBRATION_DONE                 ( 0 )

//#define RC_OSC_REFERENCE_COUNT_MAX  (1.005*F_CPU*31250UL/8000000UL)
//#define RC_OSC_REFERENCE_COUNT_MIN  (0.995*F_CPU*31250UL/8000000UL)

#ifndef RF_CHANNEL
#define RF_CHANNEL              26
#endif
/*============================ TYPEDEFS ======================================*/

/** \brief  This macro defines the start value for the RADIO_* status constants.
 *
 *          It was chosen to have this macro so that the user can define where
 *          the status returned from the TAT starts. This can be useful in a
 *          system where numerous drivers are used, and some range of status codes
 *          are occupied.
 *
 *  \see radio_status_t
 */
#define RADIO_STATUS_START_VALUE                  ( 0x40 )

/** \brief  This enumeration defines the possible return values for the TAT API
 *          functions.
 *
 *          These values are defined so that they should not collide with the
 *          return/status codes defined in the IEEE 802.15.4 standard.
 *
 */
typedef enum{
    RADIO_SUCCESS = RADIO_STATUS_START_VALUE,  /**< The requested service was performed successfully. */
    RADIO_UNSUPPORTED_DEVICE,         /**< The connected device is not an Atmel AT86RF230. */
    RADIO_INVALID_ARGUMENT,           /**< One or more of the supplied function arguments are invalid. */
    RADIO_TIMED_OUT,                  /**< The requested service timed out. */
    RADIO_WRONG_STATE,                /**< The end-user tried to do an invalid state transition. */
    RADIO_BUSY_STATE,                 /**< The radio transceiver is busy receiving or transmitting. */
    RADIO_STATE_TRANSITION_FAILED,    /**< The requested state transition could not be completed. */
    RADIO_CCA_IDLE,                   /**< Channel is clear, available to transmit a new frame. */
    RADIO_CCA_BUSY,                   /**< Channel busy. */
    RADIO_TRX_BUSY,                   /**< Transceiver is busy receiving or transmitting data. */
    RADIO_BAT_LOW,                    /**< Measured battery voltage is lower than voltage threshold. */
    RADIO_BAT_OK,                     /**< Measured battery voltage is above the voltage threshold. */
    RADIO_CRC_FAILED,                 /**< The CRC failed for the actual frame. */
    RADIO_CHANNEL_ACCESS_FAILURE,     /**< The channel access failed during the auto mode. */
    RADIO_NO_ACK,                     /**< No acknowledge frame was received. */
}radio_status_t;

typedef enum radio_cmd{
    IOCTL_SET_MODE = 0,
    IOCTL_GET_MODE = 1,
}radio_cmd_t;

/**
 * \name Transaction status codes
 * \{
 */
#define TRAC_SUCCESS                0
#define TRAC_SUCCESS_DATA_PENDING   1
#define TRAC_SUCCESS_WAIT_FOR_ACK   2
#define TRAC_CHANNEL_ACCESS_FAILURE 3
#define TRAC_NO_ACK                 5
#define TRAC_INVALID                7
/** \} */


/** \brief  This enumeration defines the possible modes available for the
 *          Clear Channel Assessment algorithm.
 *
 *          These constants are extracted from the datasheet.
 *
 */
typedef enum{
    CCA_ED                    = 0,    /**< Use energy detection above threshold mode. */
    CCA_CARRIER_SENSE         = 1,    /**< Use carrier sense mode. */
    CCA_CARRIER_SENSE_WITH_ED = 2     /**< Use a combination of both energy detection and carrier sense. */
}radio_cca_mode_t;


/** \brief  This enumeration defines the possible CLKM speeds.
 *
 *          These constants are extracted from the RF230 datasheet.
 *
 */
typedef enum{
    CLKM_DISABLED      = 0,
    CLKM_1MHZ          = 1,
    CLKM_2MHZ          = 2,
    CLKM_4MHZ          = 3,
    CLKM_8MHZ          = 4,
    CLKM_16MHZ         = 5
}radio_clkm_speed_t;

typedef void (*radio_rx_callback) (uint16_t data);
extern uint8_t rxMode;

//*********************************************NRF24L01*************************
    #define RF_TX_MODE	0
    #define RF_RX_MODE	1

    #define ADDR_WIDTH    5    // 5 uints address width
    #define PLOAD_WIDTH  0x20   // 20 uints TX payload    

    #define TX_EMPTY  (4)
    #define RX_EMPTY  (0)

    //******************NRF24L01寄存器指令**************************************
    #define READ_REG        0x00   // 读寄存器指令
    #define WRITE_REG       0x20  // 写寄存器指令
    #define RD_RX_PLOAD     0x61   // 读取接收数据指令
    #define WR_TX_PLOAD     0xA0   // 写待发数据指令
    #define FLUSH_TX        0xE1  // 冲洗发送 FIFO指令
    #define FLUSH_RX        0xE2   // 冲洗接收 FIFO指令
    #define REUSE_TX_PL     0xE3   // 定义重复装载数据指令
    #define RF_NOP             0xFF   // 保留

    //***************************SPI(nRF24L01)寄存器地址*****************
    #define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
    #define EN_AA           0x01  // 自动应答功能设置
    #define EN_RXADDR       0x02  // 可用信道设置
    #define SETUP_AW        0x03  // 收发地址宽度设置
    #define SETUP_RETR      0x04  // 自动重发功能设置
    #define RF_CH           0x05  // 工作频率设置
    #define RF_SETUP        0x06  // 发射速率、功耗功能设置
    #define STATUS          0x07  // 状态寄存器
    #define OBSERVE_TX      0x08  // 发送监测功能
    //#define CD              0x09  // 地址检测           
    //#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
    //#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
    //#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
    //#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
    //#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
    //#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
    #define TX_ADDR         0x10  // 发送地址寄存器
    #define RX_PW_P0        0x11  // 接收频道0接收数据长度
    #define RX_PW_P1        0x12  // 接收频道0接收数据长度
    #define RX_PW_P2        0x13  // 接收频道0接收数据长度
    #define RX_PW_P3        0x14  // 接收频道0接收数据长度
    #define RX_PW_P4        0x15  // 接收频道0接收数据长度
    #define RX_PW_P5        0x16  // 接收频道0接收数据长度
    #define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
    #define DYNPD               0x1C   //enable dynamic payload
    #define FEATURE            0x1D   //Feature register

    #define RF_SETUP_POWER        RF_SETUP, 0x06, 0x01  // 发射速率、功耗功能设置
    #define FIFO_STATUS_RX_EMPTY        FIFO_STATUS, 0x01, 0x00  // FIFO status, RX_EMPTY
   //**************************************************************************************

/* Number of receive buffers in RAM. */
#ifndef RF_CONF_RX_BUFFERS
#define RF_CONF_RX_BUFFERS 1
#endif

/*============================ PROTOTYPES ====================================*/
extern const struct radio_driver nrf24l01_driver;
void nrf24l01_init(uint8_t mode);
int nrf24l01_get_channel(void);
void nrf24l01_set_channel(int c);
void nrf24l01_set_pan_addr(uint16_t pan,uint16_t addr,uint8_t *ieee_addr);
void nrf24l01_set_txpower(uint8_t power);
int nrf24l01_get_txpower(void);

//#define delay_us( us )   ( _delay_loop_2( ( F_CPU / 4000000UL ) * ( us ) ) )

#endif   //_NRF24L01_H_
/** @} */
/*EOF*/