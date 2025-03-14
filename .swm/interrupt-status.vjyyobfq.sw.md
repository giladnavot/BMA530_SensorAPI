---
title: Interrupt Status
---
# Introduction

This document will walk you through the "Interrupt Status" functionality implemented in the <SwmPath>[bma530.c](/bma530.c)</SwmPath> file. The purpose of this function is to retrieve and parse the interrupt status from the BMA530 sensor.

We will cover:

1. Parameters: What inputs does the function require?
2. Control Flow: How does the function process the inputs?
3. Result: What does the function return?
4. Dependencies: What external functions or macros does it rely on?

# Parameters

The function <SwmToken path="/bma530.c" pos="72:2:2" line-data="int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)">`bma530_get_int_status`</SwmToken> takes three parameters:

- <SwmToken path="/bma530.c" pos="72:9:9" line-data="int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)">`config`</SwmToken>: A pointer to a structure array where the interrupt status will be stored.
- <SwmToken path="/bma530.c" pos="72:14:14" line-data="int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)">`n_status`</SwmToken>: The number of interrupt status types to be processed.
- <SwmToken path="/bma530.c" pos="72:22:22" line-data="int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)">`dev`</SwmToken>: A pointer to the device structure for communication with the sensor.

<SwmSnippet path="/bma530.c" line="72">

---

These parameters are crucial for defining the scope of the interrupt status retrieval and ensuring the function has the necessary context to operate.

```
int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Temporary variable to carry the register value */
    uint8_t reg_value[2] = { 0 };

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_status; loop++)
        {
            switch (config[loop].int_src)
            {
                case BMA530_INT_STATUS_INT1:
                    result = bma5_get_regs(BMA530_REG_INT_STATUS_INT1_0, reg_value, 2, dev);
                    break;

                case BMA530_INT_STATUS_INT2:
                    result = bma5_get_regs(BMA530_REG_INT_STATUS_INT2_0, reg_value, 2, dev);
                    break;

                case BMA530_INT_STATUS_I3C:
                    result = bma5_get_regs(BMA530_REG_INT_STATUS_I3C_0, reg_value, 2, dev);
                    break;

                default:
                    result = BMA5_E_INVALID_INT_STATUS;
                    break;
            }

            if (BMA5_OK != result)
            {
                break;
            }

            /* Parse needed details from received serial data */
            config[loop].int_status.acc_drdy_int_status = BMA5_GET_BITS_POS_0(reg_value[0], BMA530_ACC_DRDY_INT_STATUS);
            config[loop].int_status.fifo_wm_int_status = BMA5_GET_BITS(reg_value[0], BMA530_FIFO_WM_INT_STATUS);
            config[loop].int_status.fifo_full_int_status = BMA5_GET_BITS(reg_value[0], BMA530_FIFO_FULL_INT_STATUS);
            config[loop].int_status.gen_int1_int_status = BMA5_GET_BITS(reg_value[0], BMA530_GEN_INT1_INT_STATUS);
            config[loop].int_status.gen_int2_int_status = BMA5_GET_BITS(reg_value[0], BMA530_GEN_INT2_INT_STATUS);
            config[loop].int_status.gen_int3_int_status = BMA5_GET_BITS(reg_value[0], BMA530_GEN_INT3_INT_STATUS);
            config[loop].int_status.step_det_int_status = BMA5_GET_BITS(reg_value[0], BMA530_STEP_DET_INT_STATUS);
            config[loop].int_status.step_cnt_int_status = BMA5_GET_BITS(reg_value[0], BMA530_STEP_CNT_INT_STATUS);

            config[loop].int_status.sig_mo_int_status = BMA5_GET_BITS_POS_0(reg_value[1], BMA530_SIG_MO_INT_STATUS);
            config[loop].int_status.tilt_int_status = BMA5_GET_BITS(reg_value[1], BMA530_TILT_INT_STATUS);
            config[loop].int_status.orient_int_status = BMA5_GET_BITS(reg_value[1], BMA530_ORIENT_INT_STATUS);
            config[loop].int_status.acc_foc_int_status = BMA5_GET_BITS(reg_value[1], BMA530_ACC_FOC_INT_STATUS);
            config[loop].int_status.feat_eng_err_int_status =
                BMA5_GET_BITS(reg_value[1], BMA530_FEAT_ENG_ERR_INT_STATUS);
        }
    }

    return result;
}
```

---

</SwmSnippet>

# Control flow

The function begins by checking if the <SwmToken path="/bma530.c" pos="72:9:9" line-data="int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)">`config`</SwmToken> pointer is <SwmToken path="/bma530.c" pos="81:4:4" line-data="    if (NULL == config)">`NULL`</SwmToken>. If it is, the function returns an error code <SwmToken path="/bma530.c" pos="83:5:5" line-data="        result = BMA5_E_NULL_PTR;">`BMA5_E_NULL_PTR`</SwmToken>. This is a safeguard to prevent dereferencing a null pointer.

The main logic involves iterating over the <SwmToken path="/bma530.c" pos="72:14:14" line-data="int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)">`n_status`</SwmToken> elements in the <SwmToken path="/bma530.c" pos="72:9:9" line-data="int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)">`config`</SwmToken> array. For each element, it determines the interrupt source and retrieves the corresponding register values using <SwmToken path="/bma530.c" pos="92:5:5" line-data="                    result = bma5_get_regs(BMA530_REG_INT_STATUS_INT1_0, reg_value, 2, dev);">`bma5_get_regs`</SwmToken>. The switch-case structure handles different interrupt sources, ensuring the correct registers are accessed.

If any call to <SwmToken path="/bma530.c" pos="92:5:5" line-data="                    result = bma5_get_regs(BMA530_REG_INT_STATUS_INT1_0, reg_value, 2, dev);">`bma5_get_regs`</SwmToken> fails, the loop breaks, and the function returns the error code. This ensures that the function does not proceed with invalid data.

# Result

The function returns an <SwmToken path="/bma530.c" pos="72:0:0" line-data="int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)">`int8_t`</SwmToken> result, which indicates the success or failure of the operation. A return value of <SwmToken path="/bma530.c" pos="75:7:7" line-data="    int8_t result = BMA5_OK;">`BMA5_OK`</SwmToken> signifies successful execution, while other values indicate specific errors encountered during the process.

# Dependencies

The function relies on several external functions and macros:

- <SwmToken path="/bma530.c" pos="92:5:5" line-data="                    result = bma5_get_regs(BMA530_REG_INT_STATUS_INT1_0, reg_value, 2, dev);">`bma5_get_regs`</SwmToken>: Used to read register values from the sensor.
- Macros like <SwmToken path="/bma530.c" pos="115:12:12" line-data="            config[loop].int_status.fifo_wm_int_status = BMA5_GET_BITS(reg_value[0], BMA530_FIFO_WM_INT_STATUS);">`BMA5_GET_BITS`</SwmToken> and <SwmToken path="/bma530.c" pos="114:12:12" line-data="            config[loop].int_status.acc_drdy_int_status = BMA5_GET_BITS_POS_0(reg_value[0], BMA530_ACC_DRDY_INT_STATUS);">`BMA5_GET_BITS_POS_0`</SwmToken>: Used to parse specific bits from the register values.

These dependencies are essential for interacting with the sensor hardware and extracting the necessary interrupt status information.

<SwmMeta version="3.0.0" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No" repo-name="BMA530_SensorAPI"><sup>Powered by [Swimm](https://app.swimm.io/)</sup></SwmMeta>
