/**
  ******************************************************************************
  * @file    LSM6DSV16BXTask.h
  * @author  SRA - MCD
  * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  ******************************************************************************
  */
#ifndef LSM6DSV16BXTASK_H_
#define LSM6DSV16BXTASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "services/AManagedTaskEx.h"
#include "services/AManagedTaskEx_vtbl.h"
#include "ABusIF.h"
#include "SensorManager.h"
#include "events/DataEventSrc.h"
#include "events/DataEventSrc_vtbl.h"
#include "ISensorMems.h"
#include "ISensorMems_vtbl.h"
#include "ISensorLL.h"
#include "ISensorLL_vtbl.h"
#include "mx.h"

#define LSM6DSV16BX_MAX_DRDY_PERIOD           (1.0)    /* seconds */

#ifndef LSM6DSV16BX_MAX_WTM_LEVEL
#define LSM6DSV16BX_MAX_WTM_LEVEL             (128)    /* samples */
#endif

#define LSM6DSV16BX_MAX_SAMPLES_PER_IT        (LSM6DSV16BX_MAX_WTM_LEVEL)

#ifndef LSM6DSV16BX_TDM_ENABLED
#define LSM6DSV16BX_TDM_ENABLED                0
#endif

#ifndef LSM6DSV16BX_TDM_DMA_SAMPLES
#define LSM6DSV16BX_TDM_DMA_SAMPLES            16000u*3u*2u /* 16000 samples * 3 axes * 2 (double buffer) */
#endif


#define LSM6DSV16BX_CFG_MAX_LISTENERS         2

/**
  * Create  type name for _LSM6DSV16BXTask.
  */
typedef struct _LSM6DSV16BXTask LSM6DSV16BXTask;

/**
  *  LSM6DSV16BXTask internal structure.
  */
struct _LSM6DSV16BXTask
{
  /**
    * Base class object.
    */
  AManagedTaskEx super;

  // Task variables should be added here.

  /**
    * IRQ GPIO configuration parameters.
    */
  const MX_GPIOParams_t *pIRQConfig;

  /**
    * MLC GPIO configuration parameters.
    */
  const MX_GPIOParams_t *pMLCConfig;

  /**
    * SPI CS GPIO configuration parameters.
    */
  const MX_GPIOParams_t *pCSConfig;

#if LSM6DSV16BX_TDM_ENABLED
  /**
    * SAI configuration parameters used by TDM stream.
    */
  const MX_SAIParams_t *pSAIConfig;
#endif

  /**
    * I3C flag
    */
  boolean_t i3c_flag;

  /**
    * Bus IF object used to connect the sensor task to the specific bus.
    */
  ABusIF *p_sensor_bus_if;

  /**
    * Implements the accelerometer ISensorMems interface.
    */
  ISensorMems_t acc_sensor_if;

#if LSM6DSV16BX_TDM_ENABLED
  /**
    * Implements the TDM accelerometer ISensorMems interface.
    */
  ISensorMems_t tdm_acc_sensor_if;
#endif

  /**
    * Implements the gyroscope ISensorMems interface.
    */
  ISensorMems_t gyro_sensor_if;

  /**
    * Implements the mlc ISensorMems interface.
    */
  ISensorMems_t mlc_sensor_if;

  /**
    * Implements the ISensorMemsLL interface - Sensor Low-level.
    */
  ISensorLL_t sensor_ll_if;

  /**
    * Specifies accelerometer sensor capabilities.
    */
  const SensorDescriptor_t *acc_sensor_descriptor;

  /**
    * Specifies accelerometer sensor configuration.
    */
  SensorStatus_t acc_sensor_status;

  /**
    * Specifies acc output data
    */
  EMData_t data_acc;

#if LSM6DSV16BX_TDM_ENABLED
  /**
    * Specifies TDM accelerometer sensor capabilities.
    */
  const SensorDescriptor_t *tdm_acc_sensor_descriptor;

  /**
    * Specifies TDM accelerometer sensor configuration.
    */
  SensorStatus_t tdm_acc_sensor_status;

  /**
    * Specifies TDM acc output data.
    */
  EMData_t data_tdm_acc;
#endif
  /**
    * Specifies gyroscope sensor capabilities.
    */
  const SensorDescriptor_t *gyro_sensor_descriptor;

  /**
    * Specifies gyroscope sensor configuration.
    */
  SensorStatus_t gyro_sensor_status;

  /**
    * Specifies gyro output data
    */
  EMData_t data_gyro;
  /**
    * Specifies mlc sensor capabilities.
    */
  const SensorDescriptor_t *mlc_sensor_descriptor;

  /**
    * Specifies mlc sensor configuration.
    */
  SensorStatus_t mlc_sensor_status;

  /**
    * Specifies mlc output data
    */
  EMData_t data;

  /**
    * Specifies the sensor ID for the accelerometer subsensor.
    */
  uint8_t acc_id;

#if LSM6DSV16BX_TDM_ENABLED
  /**
    * Specifies the sensor ID for the TDM accelerometer subsensor.
    */
  uint8_t tdm_acc_id;
#endif

  /**
    * Specifies the sensor ID for the gyroscope subsensor.
    */
  uint8_t gyro_id;

  /**
    * Specifies the sensor ID for the mlc subsensor.
    */
  uint8_t mlc_id;

  /**
    * Specifies mlc status.
    */
  boolean_t mlc_enable;

  /**
    * Synchronization object used to send command to the task.
    */
  TX_QUEUE in_queue;

#if LSM6DSV16BX_FIFO_ENABLED
  /**
    * Buffer to store the data read from the sensor FIFO.
    * It is reused also to save data from the faster subsensor
    */
  uint8_t p_fast_sensor_data_buff[LSM6DSV16BX_MAX_SAMPLES_PER_IT * 7];

  /**
    * Buffer to store the data from the slower subsensor
    */
  uint8_t p_slow_sensor_data_buff[LSM6DSV16BX_MAX_SAMPLES_PER_IT / 2 * 6];
#else
  /**
    * Buffer to store the data read from the sensor FIFO.
    * It is reused also to save data from the faster subsensor
    */
  uint8_t p_acc_sample[6];

  /**
    * Buffer to store the data from the slower subsensor
   */
  uint8_t p_gyro_sample[6];

  /**
    * Save acc data ready status
   */
  uint8_t acc_drdy;

  /**
    * Save gyro data ready status
    */
  uint8_t gyro_drdy;
#endif

  /**
    * Buffer to store the data from mlc
    */
  uint8_t p_mlc_sensor_data_buff[5];

  /**
    * Specifies the FIFO level
    */
  uint16_t fifo_level;

  /**
    * Specifies the FIFO watermark level (it depends from odr)
    */
  uint16_t samples_per_it;

  /**
    * If both subsensors are active, specifies the amount of ACC samples in the FIFO
    */
  uint16_t acc_samples_count;

  /**
    * If both subsensors are active, specifies the amount of GYRO samples in the FIFO
    */
  uint16_t gyro_samples_count;

  /**
    * ::IEventSrc interface implementation for this class.
    */
  IEventSrc *p_acc_event_src;

#if LSM6DSV16BX_TDM_ENABLED
  /**
    * ::IEventSrc interface implementation for TDM accelerometer stream.
    */
  IEventSrc *p_tdm_acc_event_src;
#endif

  /**
    * ::IEventSrc interface implementation for this class.
    */
  IEventSrc *p_gyro_event_src;

  /**
    * ::IEventSrc interface implementation for this class.
    */
  IEventSrc *p_mlc_event_src;

  /**
    * Software timer used to generate the read command
    */
  TX_TIMER read_timer;

  /**
    * Timer period used to schedule the read command
    */
  ULONG lsm6dsv16bx_task_cfg_timer_period_ms;

  /**
    * Software timer used to generate the mlc read command
    */
  TX_TIMER mlc_timer;

  /**
    * Used to update the instantaneous odr.
    */
  double_t prev_timestamp;

  /**
    * Internal model (FW) is in sync with the component (HW registers)
    */
  bool sync;

  /*
    * First data ready flag: fist data must be discarded, see sensor AN
    * */
  uint8_t first_data_ready;
  uint8_t first_data_ready_threshold;

#if LSM6DSV16BX_TDM_ENABLED
  /**
    * Circular DMA buffer used by SAI to acquire TDM accelerometer samples.
    */
  int16_t p_tdm_dma_buff[LSM6DSV16BX_TDM_DMA_SAMPLES];

  /**
    * Pointer to the currently active half-buffer produced by SAI DMA callbacks.
    */
  int16_t *p_tdm_data_ptr;

  /**
    * Number of XYZ samples available in p_tdm_data_ptr.
    */
  uint16_t tdm_samples_count;

  /**
    * Used to update the instantaneous odr.
    */
  double_t tdm_prev_timestamp;

  /**
    * Last half-buffer index produced by SAI callback (1=first half, 2=second half).
    */
  uint8_t tdm_half;

  /**
    * True when TDM capture via SAI DMA is running.
    */
  boolean_t tdm_running;
#endif
};

// Public API declaration
//***********************

/**
  * Get the ISourceObservable interface for the accelerometer.
  * @param _this [IN] specifies a pointer to a task object.
  * @return a pointer to the generic object ::ISourceObservable if success,
  * or NULL if out of memory error occurs.
  */
ISourceObservable *LSM6DSV16BXTaskGetAccSensorIF(LSM6DSV16BXTask *_this);

#if LSM6DSV16BX_TDM_ENABLED
/**
  * Get the ISourceObservable interface for the TDM accelerometer.
  * @param _this [IN] specifies a pointer to a task object.
  * @return a pointer to the generic object ::ISourceObservable if success,
  * or NULL if out of memory error occurs.
  */
ISourceObservable *LSM6DSV16BXTaskGetTdmAccSensorIF(LSM6DSV16BXTask *_this);
#endif

/**
  * Get the ISourceObservable interface for the gyroscope.
  * @param _this [IN] specifies a pointer to a task object.
  * @return a pointer to the generic object ::ISourceObservable if success,
  * or NULL if out of memory error occurs.
  */
ISourceObservable *LSM6DSV16BXTaskGetGyroSensorIF(LSM6DSV16BXTask *_this);

/**
  * Get the ISourceObservable interface for mlc.
  * @param _this [IN] specifies a pointer to a task object.
  * @return a pointer to the generic object ::ISourceObservable if success,
  * or NULL if out of memory error occurs.
  */
ISourceObservable *LSM6DSV16BXTaskGetMlcSensorIF(LSM6DSV16BXTask *_this);

/**
  * Get the ::ISensorMemsLL_t interface the sensor.
  * @param _this [IN] specifies a pointer to a task object.
  * @return a pointer to the generic object ::ISensorMemsLL_t
  */
ISensorLL_t *LSM6DSV16BXTaskGetSensorLLIF(LSM6DSV16BXTask *_this);

/**
  * Allocate an instance of LSM6DSV16BXTask in the system heap.
  *
  * @param pIRQConfig [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file.
  *        It must be a GPIO connected to the LSM6DSV16BX sensor and configured in EXTI mode.
  *        If it is NULL then the sensor is configured in polling mode.
  * @param pMLCConfig [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file.
  *        It must be a GPIO connected to the ISM33DHCX MLC and configured in EXTI mode.
  * @param pCSConfig [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file.
  *        It must be a GPIO identifying the SPI CS Pin.
  * @return a pointer to the generic object ::AManagedTaskEx if success,
  * or NULL if out of memory error occurs.
  */
AManagedTaskEx *LSM6DSV16BXTaskAlloc(const void *pIRQConfig, const void *pMLCConfig, const void *pCSConfig,
#if LSM6DSV16BX_TDM_ENABLED
                                     const void *pSAIConfig,
#endif
                                     boolean_t i3c_flag);

/**
  * Call the default ::LSM6DSV16BXTaskAlloc and then it overwrite sensor name
  *
  * @param pIRQConfig [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file.
  *        It must be a GPIO connected to the LSM6DSV16BX sensor and configured in EXTI mode.
  *        If it is NULL then the sensor is configured in polling mode.
  * @param pMLCConfig [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file.
  *        It must be a GPIO connected to the ISM33DHCX MLC and configured in EXTI mode.
  * @param pCSConfig [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file.
  *        It must be a GPIO identifying the SPI CS Pin.
  * @return a pointer to the generic object ::AManagedTaskEx if success,
  * or NULL if out of memory error occurs.
  */
AManagedTaskEx *LSM6DSV16BXTaskAllocSetName(const void *pIRQConfig, const void *pMLCConfig, const void *pCSConfig,
#if LSM6DSV16BX_TDM_ENABLED
                                            const void *pSAIConfig,
#endif
                                            boolean_t i3c_flag, const char *p_name);

/**
  * Allocate an instance of ::LSM6DSV16BXTask in a memory block specified by the application.
  * The size of the memory block must be greater or equal to `sizeof(LSM6DSV16BXTask)`.
  * This allocator allows the application to avoid the dynamic allocation.
  *
  * \code
  * LSM6DSV16BXTask sensor_task;
  * LSM6DSV16BXTaskStaticAlloc(&sensor_task);
  * \endcode
  *
  * @param p_mem_block [IN] specify a memory block allocated by the application.
  *        The size of the memory block must be greater or equal to `sizeof(LSM6DSV16BXTask)`.
  * @param pIRQConfig [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file.
  *        It must be a GPIO connected to the LSM6DSV16BX sensor and configured in EXTI mode.
  *        If it is NULL then the sensor is configured in polling mode.
  * @param pMLCConfig [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file.
  *        It must be a GPIO connected to the ISM33DHCX MLC and configured in EXTI mode.
  * @param pCSConfig [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file.
  *        It must be a GPIO identifying the SPI CS Pin.
  * @return a pointer to the generic object ::AManagedTaskEx_t if success,
  * or NULL if out of memory error occurs.
  */
AManagedTaskEx *LSM6DSV16BXTaskStaticAlloc(void *p_mem_block, const void *pIRQConfig, const void *pMLCConfig,
                                           const void *pCSConfig,
#if LSM6DSV16BX_TDM_ENABLED
                                           const void *pSAIConfig,
#endif
                                           boolean_t i3c_flag);

/**
  * Call the default ::LSM6DSV16BXTaskAlloc and then it overwrite sensor name
  *
  * \code
  * LSM6DSV16BXTask sensor_task;
  * LSM6DSV16BXTaskStaticAlloc(&sensor_task);
  * \endcode
  *
  * @param p_mem_block [IN] specify a memory block allocated by the application.
  *        The size of the memory block must be greater or equal to `sizeof(LSM6DSV16BXTask)`.
  * @param pIRQConfig [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file.
  *        It must be a GPIO connected to the LSM6DSV16BX sensor and configured in EXTI mode.
  *        If it is NULL then the sensor is configured in polling mode.
  * @param pMLCConfig [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file.
  *        It must be a GPIO connected to the ISM33DHCX MLC and configured in EXTI mode.
  * @param pCSConfig [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file.
  *        It must be a GPIO identifying the SPI CS Pin.
  * @return a pointer to the generic object ::AManagedTaskEx_t if success,
  * or NULL if out of memory error occurs.
  */
AManagedTaskEx *LSM6DSV16BXTaskStaticAllocSetName(void *p_mem_block, const void *pIRQConfig, const void *pMLCConfig,
                                                  const void *pCSConfig,
#if LSM6DSV16BX_TDM_ENABLED
                                                  const void *pSAIConfig,
#endif
                                                  boolean_t i3c_flag,
                                                  const char *p_name);

/**
  * Get the Bus interface for the sensor task.
  *
  * @param _this [IN] specifies a pointer to a task object.
  * @return a pointer to the Bus interface of the sensor.
  */
ABusIF *LSM6DSV16BXTaskGetSensorIF(LSM6DSV16BXTask *_this);

/**
  * Get the ::IEventSrc interface for the sensor task.
  * @param _this [IN] specifies a pointer to a task object.
  * @return a pointer to the ::IEventSrc interface of the sensor.
  */
IEventSrc *LSM6DSV16BXTaskGetAccEventSrcIF(LSM6DSV16BXTask *_this);

#if LSM6DSV16BX_TDM_ENABLED
/**
  * Get the ::IEventSrc interface for the TDM accelerometer stream.
  * @param _this [IN] specifies a pointer to a task object.
  * @return a pointer to the ::IEventSrc interface of the sensor.
  */
IEventSrc *LSM6DSV16BXTaskGetTdmAccEventSrcIF(LSM6DSV16BXTask *_this);
#endif

/**
  * Get the ::IEventSrc interface for the sensor task.
  * @param _this [IN] specifies a pointer to a task object.
  * @return a pointer to the ::IEventSrc interface of the sensor.
  */
IEventSrc *LSM6DSV16BXTaskGetGyroEventSrcIF(LSM6DSV16BXTask *_this);

/**
  * Get the ::IEventSrc interface for the sensor task.
  * @param _this [IN] specifies a pointer to a task object.
  * @return a pointer to the ::IEventSrc interface of the sensor.
  */
IEventSrc *LSM6DSV16BXTaskGetMlcEventSrcIF(LSM6DSV16BXTask *_this);

/**
  * IRQ callback
  */
void LSM6DSV16BXTask_EXTI_Callback(uint16_t Pin);
void INT2_DSV16BX_EXTI_Callback(uint16_t Pin);

// Inline functions definition
// ***************************

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSV16BXTASK_H_ */
