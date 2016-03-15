/**
 ******************************************************************************
 * @addtogroup TauLabsTargets Tau Labs Targets
 * @{
 * @addtogroup MyFirstFC MyFirstFC support files
 * @{
 *
 * @file       pios_board.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2014
 * @author     dRonin, http://dronin.org Copyright (C) 2015
 * @brief      The board specific initialization routines
 * @see        The GNU Public License (GPL) Version 3
 * 
 *****************************************************************************/
/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* Pull in the board-specific static HW definitions.
 * Including .c files is a bit ugly but this allows all of
 * the HW definitions to be const and static to limit their
 * scope.  
 *
 * NOTE: THIS IS THE ONLY PLACE THAT SHOULD EVER INCLUDE THIS FILE
 */

#include "board_hw_defs.c"

#include <pios.h>
#include <pios_hal.h>
#include <openpilot.h>
#include <uavobjectsinit.h>
#include "hwmyfirstfc.h"
#include "manualcontrolsettings.h"
#include "modulesettings.h"

uintptr_t pios_com_openlog_logging_id;
//uintptr_t pios_com_spiflash_logging_id;
uintptr_t pios_uavo_settings_fs_id;
uintptr_t pios_waypoints_settings_fs_id;
uintptr_t pios_internal_adc_id;
uintptr_t streamfs_id;

uintptr_t external_i2c_adapter_id = 0;


/**
 * Configuration for the MPU6050 chip
 */
#if defined(PIOS_INCLUDE_MPU6050)
#include "pios_mpu6050.h"
static const struct pios_exti_cfg pios_exti_mpu6050_cfg __exti_config = {
	.vector = PIOS_MPU6050_IRQHandler,
	.line = EXTI_Line6,
	.pin = {
		.gpio = GPIOA,
		.init = {
			.GPIO_Pin = GPIO_Pin_6,
			.GPIO_Speed = GPIO_Speed_50MHz,
			.GPIO_Mode = GPIO_Mode_IN,
			.GPIO_OType = GPIO_OType_OD,
			.GPIO_PuPd = GPIO_PuPd_NOPULL,
		},
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel = EXTI9_5_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority = 0,
			.NVIC_IRQChannelCmd = ENABLE,
		},
	},
	.exti = {
		.init = {
			.EXTI_Line = EXTI_Line6, // matches above GPIO pin
			.EXTI_Mode = EXTI_Mode_Interrupt,
			.EXTI_Trigger = EXTI_Trigger_Rising,
			.EXTI_LineCmd = ENABLE,
		},
	},
};

static const struct pios_mpu60x0_cfg pios_mpu6050_cfg = {
	.exti_cfg = &pios_exti_mpu6050_cfg,
	.default_samplerate = 500,
	.interrupt_cfg = PIOS_MPU60X0_INT_CLR_ANYRD,
	.interrupt_en = PIOS_MPU60X0_INTEN_DATA_RDY,
	.User_ctl = 0,
	.Pwr_mgmt_clk = PIOS_MPU60X0_PWRMGMT_PLL_Z_CLK,
	.default_filter = PIOS_MPU60X0_LOWPASS_188_HZ,
	.orientation = PIOS_MPU60X0_TOP_180DEG
};
#endif /* PIOS_INCLUDE_MPU6050 */

/**
 * PIOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from System/openpilot.c
 */

#include <pios_board_info.h>

void PIOS_Board_Init(void) {

	/* Delay system */
	PIOS_DELAY_Init();
	
	const struct pios_board_info * bdinfo = &pios_board_info_blob;

#if defined(PIOS_INCLUDE_LED)
	const struct pios_led_cfg * led_cfg = PIOS_BOARD_HW_DEFS_GetLedCfg(bdinfo->board_rev);
	PIOS_Assert(led_cfg);
	PIOS_LED_Init(led_cfg);
#endif	/* PIOS_INCLUDE_LED */

#if defined(PIOS_INCLUDE_I2C)
	if (PIOS_I2C_Init(&pios_i2c_internal_adapter_id, &pios_i2c_internal_adapter_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_I2C_CheckClear(pios_i2c_internal_adapter_id) != 0)
		PIOS_HAL_Panic(PIOS_LED_ALARM, PIOS_HAL_PANIC_I2C_INT);
#endif


#if defined(PIOS_INCLUDE_FLASH)
    /* Initialize all flash drivers */
    if (PIOS_Flash_Internal_Init(&pios_internal_flash_id, &flash_internal_cfg) != 0)
        PIOS_HAL_Panic(PIOS_LED_ALARM, PIOS_HAL_PANIC_FLASH);

    /* Register the partition table */
    const struct pios_flash_partition * flash_partition_table;
    uint32_t num_partitions;
    flash_partition_table = PIOS_BOARD_HW_DEFS_GetPartitionTable(bdinfo->board_rev, &num_partitions);
    PIOS_FLASH_register_partition_table(flash_partition_table, num_partitions);

    /* Mount all filesystems */
    if (PIOS_FLASHFS_Logfs_Init(&pios_uavo_settings_fs_id, &flashfs_settings_cfg, FLASH_PARTITION_LABEL_SETTINGS) != 0)
        PIOS_HAL_Panic(PIOS_LED_ALARM, PIOS_HAL_PANIC_FILESYS);
    if (PIOS_FLASHFS_Logfs_Init(&pios_waypoints_settings_fs_id, &flashfs_waypoints_cfg, FLASH_PARTITION_LABEL_WAYPOINTS) != 0)
        PIOS_HAL_Panic(PIOS_LED_ALARM, PIOS_HAL_PANIC_FILESYS);
#endif    /* PIOS_INCLUDE_FLASH */

	/* Initialize the task monitor library */
	TaskMonitorInitialize();

	/* Initialize UAVObject libraries */
	UAVObjInitialize();

	/* Initialize the alarms library. Reads RCC reset flags */
	AlarmsInitialize();
	PIOS_RESET_Clear(); // Clear the RCC reset flags after use.

	/* Initialize the hardware UAVOs */
	HwMyFirstFCInitialize();
	ModuleSettingsInitialize();

#if defined(PIOS_INCLUDE_RTC)
	/* Initialize the real-time clock and its associated tick */
	PIOS_RTC_Init(&pios_rtc_main_cfg);
#endif

	/* Initialize watchdog as early as possible to catch faults during init
	 * but do it only if there is no debugger connected
	 */
	if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) == 0) {
		PIOS_WDG_Init();
	}

	/* Set up pulse timers */
	//Timers used for inputs (1, 2, 5, 8)
	PIOS_TIM_InitClock(&tim_1_cfg);
	PIOS_TIM_InitClock(&tim_2_cfg);
	PIOS_TIM_InitClock(&tim_5_cfg);
	PIOS_TIM_InitClock(&tim_8_cfg);
	// Timers used for outputs (3, 10, 11, 12)
	PIOS_TIM_InitClock(&tim_3_cfg);
	PIOS_TIM_InitClock(&tim_10_cfg);
	PIOS_TIM_InitClock(&tim_11_cfg);
	PIOS_TIM_InitClock(&tim_12_cfg);

	/* IAP System Setup */
	PIOS_IAP_Init();
	uint16_t boot_count = PIOS_IAP_ReadBootCount();
	if (boot_count < 3) {
		PIOS_IAP_WriteBootCount(++boot_count);
		AlarmsClear(SYSTEMALARMS_ALARM_BOOTFAULT);
	} else {
		/* Too many failed boot attempts, force hw config to defaults */
		HwMyFirstFCSetDefaults(HwMyFirstFCHandle(), 0);
		ModuleSettingsSetDefaults(ModuleSettingsHandle(),0);
		AlarmsSet(SYSTEMALARMS_ALARM_BOOTFAULT, SYSTEMALARMS_ALARM_CRITICAL);
	}

#if defined(PIOS_INCLUDE_USB)
	/* Initialize board specific USB data */
	PIOS_USB_BOARD_DATA_Init();

	/* Flags to determine if various USB interfaces are advertised */
	bool usb_hid_present = false;
	bool usb_cdc_present = false;

#if defined(PIOS_INCLUDE_USB_CDC)
	if (PIOS_USB_DESC_HID_CDC_Init()) {
		PIOS_Assert(0);
	}
	usb_hid_present = true;
	usb_cdc_present = true;
#else
	if (PIOS_USB_DESC_HID_ONLY_Init()) {
		PIOS_Assert(0);
	}
	usb_hid_present = true;
#endif

	uintptr_t pios_usb_id;
	PIOS_USB_Init(&pios_usb_id, PIOS_BOARD_HW_DEFS_GetUsbCfg(bdinfo->board_rev));

#if defined(PIOS_INCLUDE_USB_CDC)

	uint8_t hw_usb_vcpport;
	/* Configure the USB VCP port */
	HwMyFirstFCUSB_VCPPortGet(&hw_usb_vcpport);

	if (!usb_cdc_present) {
		/* Force VCP port function to disabled if we haven't advertised VCP in our USB descriptor */
		hw_usb_vcpport = HWMYFIRSTFC_USB_VCPPORT_DISABLED;
	}

	PIOS_HAL_ConfigureCDC(hw_usb_vcpport, pios_usb_id, &pios_usb_cdc_cfg);
	
#endif	/* PIOS_INCLUDE_USB_CDC */

#if defined(PIOS_INCLUDE_USB_HID)
	/* Configure the usb HID port */
	uint8_t hw_usb_hidport;
	HwMyFirstFCUSB_HIDPortGet(&hw_usb_hidport);

	if (!usb_hid_present) {
		/* Force HID port function to disabled if we haven't advertised HID in our USB descriptor */
		hw_usb_hidport = HWMYFIRSTFC_USB_HIDPORT_DISABLED;
	}

	PIOS_HAL_ConfigureHID(hw_usb_hidport, pios_usb_id, &pios_usb_hid_cfg);

#endif	/* PIOS_INCLUDE_USB_HID */

	if (usb_hid_present || usb_cdc_present) {
		PIOS_USBHOOK_Activate();
	}
#endif	/* PIOS_INCLUDE_USB */

	/* Configure the IO ports */
	HwMyFirstFCDSMxModeOptions hw_DSMxMode;
	HwMyFirstFCDSMxModeGet(&hw_DSMxMode);

	/* UART1 Port */
/*
	uint8_t hw_uart1;
	HwMyFirstFCUart1Get(&hw_uart1);

	PIOS_HAL_ConfigurePort(hw_uart1,             // port type protocol
			&pios_usart1_cfg,                    // usart_port_cfg
			&pios_usart_com_driver,              // com_driver
			&pios_i2c_usart1_adapter_id,         // i2c_id
			&pios_i2c_usart1_adapter_cfg,        // i2c_cfg
			NULL,                                // ppm_cfg
			NULL,                                // pwm_cfg
			PIOS_LED_ALARM,                      // led_id
			&pios_usart1_dsm_aux_cfg,            // dsm_cfg
			hw_DSMxMode,                         // dsm_mode
			NULL);                               // sbus_cfg
*/

	/* UART2 Port */
	uint8_t hw_uart2;
	HwMyFirstFCUart2Get(&hw_uart2);

	PIOS_HAL_ConfigurePort(hw_uart2,             // port type protocol
			&pios_usart2_cfg,                    // usart_port_cfg
			&pios_usart_com_driver,              // com_driver
			NULL,                                // i2c_id
			NULL,                                // i2c_cfg
			NULL,                                // ppm_cfg
			NULL,                                // pwm_cfg
			PIOS_LED_ALARM,                      // led_id
			&pios_usart2_dsm_aux_cfg,            // dsm_cfg
			hw_DSMxMode,                         // dsm_mode
			&pios_usart2_sbus_aux_cfg);          // sbus_cfg

	/* UART3 Port */
/*
	uint8_t hw_uart3;
	HwMyFirstFCUart3Get(&hw_uart3);

	PIOS_HAL_ConfigurePort(hw_uart3,             // port type protocol
			&pios_usart3_cfg,                    // usart_port_cfg
			&pios_usart_com_driver,              // com_driver
			&pios_i2c_usart3_adapter_id,         // i2c_id
			&pios_i2c_usart3_adapter_cfg,        // i2c_cfg
			NULL,                                // ppm_cfg
			NULL,                                // pwm_cfg
			PIOS_LED_ALARM,                      // led_id
			&pios_usart3_dsm_aux_cfg,            // dsm_cfg
			hw_DSMxMode,                         // dsm_mode
			NULL);                               // sbus_cfg

*/
	/* UART4 Port */
	uint8_t hw_uart4;
	HwMyFirstFCUart4Get(&hw_uart4);

	PIOS_HAL_ConfigurePort(hw_uart4,             // port type protocol
			&pios_usart4_cfg,                    // usart_port_cfg
			&pios_usart_com_driver,              // com_driver
			NULL,                                // i2c_id
			NULL,                                // i2c_cfg
			NULL,                                // ppm_cfg
			NULL,                                // pwm_cfg
			PIOS_LED_ALARM,                      // led_id
			&pios_usart4_dsm_aux_cfg,            // dsm_cfg
			hw_DSMxMode,                         // dsm_mode
			NULL);                               // sbus_cfg

	/* UART5 Port */
	uint8_t hw_uart5;
	HwMyFirstFCUart5Get(&hw_uart5);

	PIOS_HAL_ConfigurePort(hw_uart5,             // port type protocol
			&pios_usart5_cfg,                    // usart_port_cfg
			&pios_usart_com_driver,              // com_driver
			NULL,                                // i2c_id
			NULL,                                // i2c_cfg
			NULL,                                // ppm_cfg
			NULL,                                // pwm_cfg
			PIOS_LED_ALARM,                      // led_id
			&pios_usart5_dsm_aux_cfg,            // dsm_cfg
			hw_DSMxMode,                         // dsm_mode
			NULL);                               // sbus_cfg

	/* Configure the rcvr port */
	uint8_t hw_rcvrport;
	HwMyFirstFCRcvrPortGet(&hw_rcvrport);

	switch (hw_rcvrport) {
	case HWMYFIRSTFC_RCVRPORT_DISABLED:
		break;
	
	case HWMYFIRSTFC_RCVRPORT_PWM:
		PIOS_HAL_ConfigurePort(HWSHARED_PORTTYPES_PWM,  // port type protocol
				NULL,                                   // usart_port_cfg
				NULL,                                   // com_driver
				NULL,                                   // i2c_id
				NULL,                                   // i2c_cfg
				NULL,                                   // ppm_cfg
				&pios_pwm_cfg,                          // pwm_cfg
				PIOS_LED_ALARM,                         // led_id
				NULL,                                   // dsm_cfg
				0,                                      // dsm_mode
				NULL);                                  // sbus_cfg
		break;

	case HWMYFIRSTFC_RCVRPORT_PWMADC:
		PIOS_HAL_ConfigurePort(HWSHARED_PORTTYPES_PWM,  // port type protocol
				NULL,                                   // usart_port_cfg
				NULL,                                   // com_driver
				NULL,                                   // i2c_id
				NULL,                                   // i2c_cfg
				NULL,                                   // ppm_cfg
				&pios_pwm_with_adc_cfg,                 // pwm_cfg
				PIOS_LED_ALARM,                         // led_id
				NULL,                                   // dsm_cfg
				0,                                      // dsm_mode
				NULL);                                  // sbus_cfg
		break;

	case HWMYFIRSTFC_RCVRPORT_PPM:
	case HWMYFIRSTFC_RCVRPORT_PPMADC:
	case HWMYFIRSTFC_RCVRPORT_PPMOUTPUTS:
	case HWMYFIRSTFC_RCVRPORT_PPMOUTPUTSADC:
		PIOS_HAL_ConfigurePort(HWSHARED_PORTTYPES_PPM,  // port type protocol
				NULL,                                   // usart_port_cfg
				NULL,                                   // com_driver
				NULL,                                   // i2c_id
				NULL,                                   // i2c_cfg
				&pios_ppm_cfg,                          // ppm_cfg
				NULL,                                   // pwm_cfg
				PIOS_LED_ALARM,                         // led_id
				NULL,                                   // dsm_cfg
				0,                                      // dsm_mode
				NULL);                                  // sbus_cfg
		break;

	case HWMYFIRSTFC_RCVRPORT_PPMPWM:
		PIOS_HAL_ConfigurePort(HWSHARED_PORTTYPES_PPM,  // port type protocol
				NULL,                                   // usart_port_cfg
				NULL,                                   // com_driver
				NULL,                                   // i2c_id
				NULL,                                   // i2c_cfg
				&pios_ppm_cfg,                          // ppm_cfg
				NULL,                                   // pwm_cfg
				PIOS_LED_ALARM,                         // led_id
				NULL,                                   // dsm_cfg
				0,                                      // dsm_mode
				NULL);                                  // sbus_cfg

		PIOS_HAL_ConfigurePort(HWSHARED_PORTTYPES_PWM,  // port type protocol
				NULL,                                   // usart_port_cfg
				NULL,                                   // com_driver
				NULL,                                   // i2c_id
				NULL,                                   // i2c_cfg
				NULL,                                   // ppm_cfg
				&pios_pwm_with_ppm_cfg,                 // pwm_cfg
				PIOS_LED_ALARM,                         // led_id
				NULL,                                   // dsm_cfg
				0,                                      // dsm_mode
				NULL);                                  // sbus_cfg
		break;

	case HWMYFIRSTFC_RCVRPORT_PPMPWMADC:
		PIOS_HAL_ConfigurePort(HWSHARED_PORTTYPES_PPM,  // port type protocol
				NULL,                                   // usart_port_cfg
				NULL,                                   // com_driver
				NULL,                                   // i2c_id
				NULL,                                   // i2c_cfg
				&pios_ppm_cfg,                          // ppm_cfg
				NULL,                                   // pwm_cfg
				PIOS_LED_ALARM,                         // led_id
				NULL,                                   // dsm_cfg
				0,                                      // dsm_mode
				NULL);                                  // sbus_cfg

		PIOS_HAL_ConfigurePort(HWSHARED_PORTTYPES_PWM,  // port type protocol
				NULL,                                   // usart_port_cfg
				NULL,                                   // com_driver
				NULL,                                   // i2c_id
				NULL,                                   // i2c_cfg
				NULL,                                   // ppm_cfg
				&pios_pwm_with_ppm_with_adc_cfg,        // pwm_cfg
				PIOS_LED_ALARM,                         // led_id
				NULL,                                   // dsm_cfg
				0,                                      // dsm_mode
				NULL);                                  // sbus_cfg
		break;
	}

#if defined(PIOS_INCLUDE_GCSRCVR)
	GCSReceiverInitialize();
	uintptr_t pios_gcsrcvr_id;
	PIOS_GCSRCVR_Init(&pios_gcsrcvr_id);
	uintptr_t pios_gcsrcvr_rcvr_id;
	if (PIOS_RCVR_Init(&pios_gcsrcvr_rcvr_id, &pios_gcsrcvr_rcvr_driver, pios_gcsrcvr_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_GCS] = pios_gcsrcvr_rcvr_id;
#endif	/* PIOS_INCLUDE_GCSRCVR */

#ifndef PIOS_DEBUG_ENABLE_DEBUG_PINS
	switch (hw_rcvrport) {
		case HWMYFIRSTFC_RCVRPORT_DISABLED:
		case HWMYFIRSTFC_RCVRPORT_PWM:
		case HWMYFIRSTFC_RCVRPORT_PWMADC:
		case HWMYFIRSTFC_RCVRPORT_PPM:
		case HWMYFIRSTFC_RCVRPORT_PPMADC:
		case HWMYFIRSTFC_RCVRPORT_PPMPWM:
		case HWMYFIRSTFC_RCVRPORT_PPMPWMADC:
			/* Set up the servo outputs */
#ifdef PIOS_INCLUDE_SERVO
			PIOS_Servo_Init(&pios_servo_cfg);
#endif
			break;
		case HWMYFIRSTFC_RCVRPORT_PPMOUTPUTS:
		case HWMYFIRSTFC_RCVRPORT_OUTPUTS:
#ifdef PIOS_INCLUDE_SERVO
			PIOS_Servo_Init(&pios_servo_with_rcvr_cfg);
#endif
			break;
		case HWMYFIRSTFC_RCVRPORT_PPMOUTPUTSADC:
		case HWMYFIRSTFC_RCVRPORT_OUTPUTSADC:
#ifdef PIOS_INCLUDE_SERVO
			PIOS_Servo_Init(&pios_servo_with_rcvr_with_adc_cfg);
#endif
			break;
	}
#else
	PIOS_DEBUG_Init(&pios_tim_servo_all_channels, NELEMENTS(pios_tim_servo_all_channels));
#endif

/* init sensor queue registration */
	PIOS_SENSORS_Init();

	PIOS_WDG_Clear();
	PIOS_DELAY_WaitmS(200);
	PIOS_WDG_Clear();


#if defined(PIOS_INCLUDE_I2C)

#if defined(PIOS_INCLUDE_MPU6050)
	{
		if (PIOS_MPU6050_Init(pios_i2c_internal_adapter_id, PIOS_MPU6050_I2C_ADD_A0_LOW, &pios_mpu6050_cfg) != 0)
			PIOS_HAL_Panic(PIOS_LED_ALARM, PIOS_HAL_PANIC_IMU);
		if (PIOS_MPU6050_Test() != 0)
			PIOS_HAL_Panic(PIOS_LED_ALARM, PIOS_HAL_PANIC_IMU);

		// To be safe map from UAVO enum to driver enum
		uint8_t hw_gyro_range;
		HwMyFirstFCGyroRangeGet(&hw_gyro_range);
		switch (hw_gyro_range) {
		case HWMYFIRSTFC_GYRORANGE_250:
			PIOS_MPU6050_SetGyroRange(PIOS_MPU60X0_SCALE_250_DEG);
			break;
		case HWMYFIRSTFC_GYRORANGE_500:
			PIOS_MPU6050_SetGyroRange(PIOS_MPU60X0_SCALE_500_DEG);
			break;
		case HWMYFIRSTFC_GYRORANGE_1000:
			PIOS_MPU6050_SetGyroRange(PIOS_MPU60X0_SCALE_1000_DEG);
			break;
		case HWMYFIRSTFC_GYRORANGE_2000:
			PIOS_MPU6050_SetGyroRange(PIOS_MPU60X0_SCALE_2000_DEG);
			break;
		}

		uint8_t hw_accel_range;
		HwMyFirstFCAccelRangeGet(&hw_accel_range);
		switch (hw_accel_range) {
		case HWMYFIRSTFC_ACCELRANGE_2G:
			PIOS_MPU6050_SetAccelRange(PIOS_MPU60X0_ACCEL_2G);
			break;
		case HWMYFIRSTFC_ACCELRANGE_4G:
			PIOS_MPU6050_SetAccelRange(PIOS_MPU60X0_ACCEL_4G);
			break;
		case HWMYFIRSTFC_ACCELRANGE_8G:
			PIOS_MPU6050_SetAccelRange(PIOS_MPU60X0_ACCEL_8G);
			break;
		case HWMYFIRSTFC_ACCELRANGE_16G:
			PIOS_MPU6050_SetAccelRange(PIOS_MPU60X0_ACCEL_16G);
			break;
		}
	}

#endif /* PIOS_INCLUDE_MPU6050 */

	//I2C is slow, sensor init as well, reset watchdog to prevent reset here
	PIOS_WDG_Clear();

#endif	/* PIOS_INCLUDE_I2C */


#if defined(PIOS_INCLUDE_GPIO)
	PIOS_GPIO_Init();
#endif

#if defined(PIOS_INCLUDE_ADC)
	if (hw_rcvrport == HWMYFIRSTFC_RCVRPORT_PWMADC ||
			hw_rcvrport == HWMYFIRSTFC_RCVRPORT_PPMADC ||
			hw_rcvrport == HWMYFIRSTFC_RCVRPORT_PPMPWMADC ||
			hw_rcvrport == HWMYFIRSTFC_RCVRPORT_OUTPUTSADC ||
			hw_rcvrport == HWMYFIRSTFC_RCVRPORT_PPMOUTPUTSADC) {
		uint32_t internal_adc_id;
		PIOS_INTERNAL_ADC_Init(&internal_adc_id, &pios_adc_cfg);
		PIOS_ADC_Init(&pios_internal_adc_id, &pios_internal_adc_driver, internal_adc_id);
	}
#endif


	//Set battery input pin to output, because of the voltage divider usage as input is not useful
	//Take care of the voltage divider connected to this pin
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_15);

	//Set buzzer output to low as long as it is unused
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);

	/* Make sure we have at least one telemetry link configured or else fail initialization */
	PIOS_Assert(pios_com_telem_serial_id || pios_com_telem_usb_id);
}

/**
 * @}
 */
