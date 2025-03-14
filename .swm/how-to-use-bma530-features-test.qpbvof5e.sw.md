---
title: How to use BMA530 Features - Test
---
# Introduction

This document will walk you through the implementation of the <SwmToken path="/bma530_features.h" pos="843:8:8" line-data=" * \defgroup bma530ApiInit BMA530 Initialization">`BMA530`</SwmToken> features, focusing on the communication with the COINES platform, the feature definitions, and an example of temperature measurement.

We will cover:

1. Functional documentation of the communication file.
2. Summary and list of features in the <SwmToken path="/bma530_features.h" pos="843:8:8" line-data=" * \defgroup bma530ApiInit BMA530 Initialization">`BMA530`</SwmToken> features definition file.
3. Functional overview of the temperature example.

# Common.c file (Communication file with COINES)

## Functional documentation

<SwmSnippet path="/examples/common/common.c" line="42">

---

The communication with the COINES platform is established through <SwmToken path="/examples/common/common.c" pos="46:6:6" line-data=" * @brief I2C read function map to COINES platform">`I2C`</SwmToken> and SPI interfaces. The functions <SwmToken path="/examples/common/common.c" pos="48:2:2" line-data="BMA5_INTF_RET_TYPE bma5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)">`bma5_i2c_read`</SwmToken> and <SwmToken path="/examples/common/common.h" pos="125:2:2" line-data="BMA5_INTF_RET_TYPE bma5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);">`bma5_i2c_write`</SwmToken> handle <SwmToken path="/examples/common/common.c" pos="46:6:6" line-data=" * @brief I2C read function map to COINES platform">`I2C`</SwmToken> communication by mapping to the COINES platform functions.

```
/* Variable to store the device address */
static uint8_t dev_addr;

/*!
 * @brief I2C read function map to COINES platform
 */
BMA5_INTF_RET_TYPE bma5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
```

---

</SwmSnippet>

<SwmSnippet path="/examples/common/common.c" line="65">

---

Similarly, <SwmToken path="/examples/common/common.c" pos="68:2:2" line-data="BMA5_INTF_RET_TYPE bma5_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)">`bma5_spi_read`</SwmToken> and <SwmToken path="/examples/common/common.h" pos="108:2:2" line-data="BMA5_INTF_RET_TYPE bma5_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);">`bma5_spi_write`</SwmToken> manage SPI communication.

```
/*!
 * @brief SPI read function map to COINES platform
 */
BMA5_INTF_RET_TYPE bma5_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t*)intf_ptr;

    return coines_read_spi(COINES_SPI_BUS_0, dev_addr, reg_addr, reg_data, (uint16_t)len);
}
```

---

</SwmSnippet>

<SwmSnippet path="/examples/common/common.c" line="85">

---

The delay function <SwmToken path="/examples/common/common.c" pos="88:2:2" line-data="void bma5_delay_us(uint32_t period, void *intf_ptr)">`bma5_delay_us`</SwmToken> is mapped to the COINES platform to introduce necessary delays in microseconds.

```
/*!
 * @brief Delay function map to COINES platform
 */
void bma5_delay_us(uint32_t period, void *intf_ptr)
{
    coines_delay_usec(period);
}
```

---

</SwmSnippet>

<SwmSnippet path="/examples/common/common.c" line="93">

---

Error handling is performed using <SwmToken path="/examples/common/common.c" pos="93:2:2" line-data="void bma5_check_rslt(const char api_name[], int8_t rslt)">`bma5_check_rslt`</SwmToken>, which checks the result of API calls and prints error messages based on the error code.

```
void bma5_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMA5_OK:

            /* Do nothing */
            break;
        case BMA5_E_NULL_PTR:
            printf("API name %s\t", api_name);
            printf("Error  [%d] : Null pointer\r\n", rslt);
            break;
        case BMA5_E_COM_FAIL:
            printf("API name %s\t", api_name);
            printf("Error  [%d] : Communication failure\r\n", rslt);
            break;
        case BMA5_E_DEV_NOT_FOUND:
            printf("API name %s\t", api_name);
            printf("Error  [%d] : Device not found\r\n", rslt);
            break;
        default:
            printf("API name %s\t", api_name);
            printf("Error  [%d] : Unknown error code\r\n", rslt);
            break;
    }
}
```

---

</SwmSnippet>

<SwmSnippet path="/examples/common/common.c" line="120">

---

The <SwmToken path="/examples/common/common.c" pos="120:2:2" line-data="int8_t bma5_interface_init(struct bma5_dev *bma5, uint8_t intf, enum bma5_context context)">`bma5_interface_init`</SwmToken> function initializes the communication interface, setting up either <SwmToken path="/examples/common/common.c" pos="46:6:6" line-data=" * @brief I2C read function map to COINES platform">`I2C`</SwmToken> or SPI based on the provided interface parameter.

```
int8_t bma5_interface_init(struct bma5_dev *bma5, uint8_t intf, enum bma5_context context)
{
    int8_t rslt = BMA5_OK;

    if (bma5 != NULL)
    {
        int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);
```

---

</SwmSnippet>

<SwmSnippet path="/examples/common/common.c" line="187">

---

The deinitialization of the COINES interface is handled by <SwmToken path="/examples/common/common.c" pos="187:2:2" line-data="void bma5_coines_deinit(void)">`bma5_coines_deinit`</SwmToken>, ensuring proper closure of communication.

```
void bma5_coines_deinit(void)
{
    fflush(stdout);

    coines_set_shuttleboard_vdd_vddio_config(0, 0);

    coines_delay_msec(2000);
```

---

</SwmSnippet>

## Sequence diagram

A sequence diagram would illustrate the initialization of the communication interface, followed by read/write operations, and finally the deinitialization process.

# BMA530_features.h file (File with definitions of the BMA530_features)

## Summary

This header file contains macros and structures that define the features of the <SwmToken path="/bma530_features.h" pos="843:8:8" line-data=" * \defgroup bma530ApiInit BMA530 Initialization">`BMA530`</SwmToken> sensor. It provides configurations for various sensor functionalities such as step counting, motion detection, and orientation.

## List of features

<SwmSnippet path="/bma530_features.h" line="85">

---

The file defines several macros for feature configurations, such as step counter settings, motion detection thresholds, and orientation parameters.

```
#define BMA530_SC_DEFAULT_WATERMARK_LEVEL        UINT16_C(0x0)
#define BMA530_SC_DEFAULT_RESET_COUNTER          UINT8_C(0x0)
#define BMA530_SC_DEFAULT_SD_EN                  UINT8_C(0x1)
#define BMA530_SC_DEFAULT_SC_EN                  UINT8_C(0x1)
#define BMA530_SC_DEFAULT_FILTER_COEFF_B_2       UINT16_C(0x55F)
#define BMA530_SC_DEFAULT_FILTER_COEFF_B_1       UINT16_C(0xABE)
#define BMA530_SC_DEFAULT_FILTER_COEFF_B_0       UINT16_C(0x55F)
#define BMA530_SC_DEFAULT_FILTER_COEFF_A_2       UINT16_C(0xE897)
#define BMA530_SC_DEFAULT_FILTER_COEFF_A_1       UINT16_C(0x41EF)
#define BMA530_SC_DEFAULT_FILTER_COEFF_SCALE_A   UINT8_C(0xE)
#define BMA530_SC_DEFAULT_FILTER_COEFF_SCALE_B   UINT8_C(0xE)
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="614">

---

It also includes structures to store configurations for features like step counting, significant motion, tilt, and orientation.

```
/*!
 *  @brief Structure to store step counter config
 */
struct bma530_step_cntr
{
    /*! An interrupt will be triggered every time the difference in number of
       * steps counted from last event is equal to (set value * 20). If 0, the interrupt is disabled */
    uint16_t watermark_level;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="975">

---

The API functions declared in this file allow setting and getting configurations for these features, enabling flexible sensor management.

```
/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_set_step_counter_config bma530_set_step_counter_config
 * \code
 * int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev);
 * \endcode
 * @details This API sets step counter configuration
 *
 * @param[in] step_cntr      : Structure instance of bma530_step_cntr.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev);
```

---

</SwmSnippet>

# Temperature.c file (Example File)

## Functional overview

The temperature example demonstrates how to initialize the <SwmToken path="/bma530_features.h" pos="843:8:8" line-data=" * \defgroup bma530ApiInit BMA530 Initialization">`BMA530`</SwmToken> sensor, configure it for temperature measurement, and read temperature data.

<SwmSnippet path="/examples/temperature/temperature.c" line="35">

---

The main function initializes the device and sets up the SPI interface for communication.

```
#include <stdio.h>
#include "common.h"
#include "bma530_features.h"

/******************************************************************************/
int main(void)
{
    struct bma5_dev dev;
    int8_t rslt;
    uint8_t loop = 0;
    struct bma5_temp_conf config;
    struct bma5_sensor_status status;
    uint8_t temperature = 0;
    int8_t temp_celsius = 0;
    uint8_t iteration = 50;
```

---

</SwmSnippet>

<SwmSnippet path="/examples/temperature/temperature.c" line="63">

---

The device is initialized using <SwmToken path="/examples/temperature/temperature.c" pos="64:5:5" line-data="    rslt = bma530_init(&amp;dev);">`bma530_init`</SwmToken>, which reads the chip ID to verify the sensor.

```
    /* Initialize the device */
    rslt = bma530_init(&dev);
    bma5_check_rslt("bma530_init", rslt);
    printf("Chip ID:0x%x\n\n", dev.chip_id);
```

---

</SwmSnippet>

<SwmSnippet path="/examples/temperature/temperature.c" line="68">

---

Temperature configuration is retrieved and set using <SwmToken path="/examples/temperature/temperature.c" pos="69:5:5" line-data="    rslt = bma5_get_temp_conf(&amp;config, &amp;dev);">`bma5_get_temp_conf`</SwmToken> and <SwmToken path="/bma5.c" pos="522:2:2" line-data="int8_t bma5_set_temp_conf(const struct bma5_temp_conf *config, struct bma5_dev *dev)">`bma5_set_temp_conf`</SwmToken>.

```
    /* Get temperature config */
    rslt = bma5_get_temp_conf(&config, &dev);
    bma5_check_rslt("bma5_get_temp_conf", rslt);
```

---

</SwmSnippet>

<SwmSnippet path="/examples/temperature/temperature.c" line="86">

---

The example continuously checks the sensor status to determine if temperature data is ready, retrieves the data, and prints it in a loop.

```
    printf("\nCount, Temparature data\n");

    while (loop < iteration)
    {
        /* Get temperature data ready status */
        rslt = bma5_get_sensor_status(&status, &dev);
        bma5_check_rslt("bma5_get_sensor_status", rslt);
```

---

</SwmSnippet>

## Sequence diagram

A sequence diagram would show the initialization of the sensor, configuration of temperature settings, continuous data retrieval, and final deinitialization.

<SwmMeta version="3.0.0" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No" repo-name="BMA530_SensorAPI"><sup>Powered by [Swimm](https://app.swimm.io/)</sup></SwmMeta>
