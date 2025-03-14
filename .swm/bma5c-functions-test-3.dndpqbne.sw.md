---
title: BMA5.c Functions - Test 3
---
# CHIP_ID

<SwmSnippet path="/bma530.c" line="47">

---

Here we retrieve the chip ID from the `BMA530` sensor by reading the `BMA530_REG_CHIP_ID` register and extracting the relevant bits.

```c
int8_t bma530_get_chip_id(uint8_t *chip_id, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == chip_id)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA530_REG_CHIP_ID, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *chip_id = BMA5_GET_BITS_POS_0(reg_value, BMA530_CHIP_ID);
        }
    }

    return result;
}
```

---

</SwmSnippet>

# Interrupt Status

<SwmSnippet path="/bma530.c" line="72">

---

Here we retrieve interrupt status information from the `bma530` device by reading specific registers based on the provided `config` and populating the corresponding `int_status` fields.

```c
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

<SwmSnippet path="/bma530.c" line="72">

---

Parameters :&nbsp;\
Here we retrieve interrupt status information from specific registers based on the `int_src` values in `config`, and populate the `int_status` structure with parsed details from the read data.

&nbsp;

Parametrs:

config - the configuration for the interrupt status

n_status - the number of interrupt statuses to retrieve

dev - the device used for retrieving interrupt status information

```c
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

<SwmSnippet path="/bma530.c" line="72">

---

Here we retrieve the interrupt status using `bma530_get_int_status`, updating the `config` structure with the current interrupt information.

```c
int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)
```

---

</SwmSnippet>

<SwmSnippet path="/bma530.c" line="75">

---

Here we initialize `result` to `BMA5_OK`, declare a `loop` variable, and create a temporary array `reg_value` to store register values.

```c
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Temporary variable to carry the register value */
    uint8_t reg_value[2] = { 0 };
```

---

</SwmSnippet>

<SwmMeta version="3.0.0" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No" repo-name="BMA530_SensorAPI"><sup>Powered by [Swimm](https://app.swimm.io/)</sup></SwmMeta>
