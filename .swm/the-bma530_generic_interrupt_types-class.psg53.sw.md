---
title: The bma530_generic_interrupt_types class
---
This document will cover the class <SwmToken path="bma530_features.c" pos="1724:8:8" line-data="int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,">`bma530_generic_interrupt_types`</SwmToken> in the file <SwmPath>[bma530_features.c](bma530_features.c)</SwmPath>. We will cover:

1. What is <SwmToken path="bma530_features.c" pos="1724:8:8" line-data="int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,">`bma530_generic_interrupt_types`</SwmToken>
2. Variables and functions defined in <SwmToken path="bma530_features.c" pos="1724:8:8" line-data="int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,">`bma530_generic_interrupt_types`</SwmToken>

# What is <SwmToken path="bma530_features.c" pos="1724:8:8" line-data="int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,">`bma530_generic_interrupt_types`</SwmToken>

<SwmToken path="bma530_features.c" pos="1724:8:8" line-data="int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,">`bma530_generic_interrupt_types`</SwmToken> is a structure defined in <SwmPath>[bma530_features.c](bma530_features.c)</SwmPath>. It is used to specify and configure generic interrupts for the <SwmToken path="bma530_features.h" pos="843:8:8" line-data=" * \defgroup bma530ApiInit BMA530 Initialization">`BMA530`</SwmToken> sensor. This structure allows the user to define settings for up to three generic interrupts, which can be used to detect various events based on the sensor's data.

<SwmSnippet path="/bma530_features.h" line="577">

---

The variable <SwmToken path="bma530_features.h" pos="577:3:3" line-data="    uint8_t generic_interrupt;">`generic_interrupt`</SwmToken> is used to specify which generic interrupt (1, 2, or 3) is being configured.

```c
    uint8_t generic_interrupt;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="580">

---

The variable <SwmToken path="bma530_features.h" pos="580:5:5" line-data="    struct bma530_generic_interrupt gen_int;">`gen_int`</SwmToken> is a structure of type <SwmToken path="bma530_features.h" pos="580:3:3" line-data="    struct bma530_generic_interrupt gen_int;">`bma530_generic_interrupt`</SwmToken> that holds the configuration settings for the specified generic interrupt.

```c
    struct bma530_generic_interrupt gen_int;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.c" line="1724">

---

The function <SwmToken path="bma530_features.c" pos="1724:2:2" line-data="int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,">`bma530_set_generic_int_config`</SwmToken> is used to set the configuration for the generic interrupts. It takes an array of <SwmToken path="bma530_features.c" pos="1724:8:8" line-data="int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,">`bma530_generic_interrupt_types`</SwmToken> structures, the number of interrupts to configure, and a device handle.

```c
int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,
                                     uint8_t n_ints,
                                     struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Variable to store base address of generic interrupt 1 */
    uint8_t data;

    /* Array to store generic interrupt 1 data */
    uint8_t int_1_data[16] = { 0 };

    uint16_t slope_thres_1, slope_thres_2, comb_sel, axis_sel, hysteresis_1, hysteresis_2, criterion_sel;
    uint16_t acc_ref_up, duration_1, duration_2, wait_time, quiet_time_1, quiet_time_2;
    uint16_t ref_acc_x_1, ref_acc_x_2, ref_acc_y_1, ref_acc_y_2, ref_acc_z_1, ref_acc_z_2;

    if (gen_int == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_ints; loop++)
        {
            switch (gen_int[loop].generic_interrupt)
            {
                case BMA530_GEN_INT_1:
                    data = BMA530_BASE_ADDR_GENERIC_INT1;
                    break;

                case BMA530_GEN_INT_2:
                    data = BMA530_BASE_ADDR_GENERIC_INT2;
                    break;

                case BMA530_GEN_INT_3:
                    data = BMA530_BASE_ADDR_GENERIC_INT3;
                    break;

                default:
                    result = BMA5_E_INVALID_GEN_INT;

                    return result;
            }

            if (result == BMA5_OK)
            {
                /* Set the generic interrupt base address to feature engine transmission address to start DMA
                 * transaction */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);
            }

            if (result == BMA5_OK)
            {
                /* Get the configuration from the feature engine register */
                result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, int_1_data, 16, dev);

                if (result == BMA5_OK)
                {
                    /* Settings 1 */
                    slope_thres_1 =
                        (BMA5_SET_BITS_POS_0(int_1_data[2], BMA530_GEN_INT_SLOPE_THRES,
                                             gen_int[loop].gen_int.slope_thres) & BMA530_GEN_INT_SLOPE_THRES_MSK);

                    slope_thres_2 = (uint16_t)(int_1_data[3] << 8);

                    slope_thres_2 =
                        (BMA5_SET_BITS_POS_0(slope_thres_2, BMA530_GEN_INT_SLOPE_THRES,
                                             gen_int[loop].gen_int.slope_thres) & BMA530_GEN_INT_SLOPE_THRES_MSK);

                    comb_sel =
                        (BMA5_SET_BITS(int_1_data[3], BMA530_GEN_INT_COMB_SEL,
                                       gen_int[loop].gen_int.comb_sel) & BMA530_GEN_INT_COMB_SEL_MSK);

                    axis_sel =
                        (BMA5_SET_BITS(int_1_data[3], BMA530_GEN_INT_AXIS_SEL,
                                       gen_int[loop].gen_int.axis_sel) & BMA530_GEN_INT_AXIS_SEL_MSK);

                    /* Settings 2 */
                    hysteresis_1 =
                        (BMA5_SET_BITS_POS_0(int_1_data[4], BMA530_GEN_INT_HYST,
                                             gen_int[loop].gen_int.hysteresis) & BMA530_GEN_INT_HYST_MSK);

                    hysteresis_2 = (uint16_t)(int_1_data[5] << 8);

                    hysteresis_2 =
                        (BMA5_SET_BITS_POS_0(hysteresis_2, BMA530_GEN_INT_HYST,
                                             gen_int[loop].gen_int.hysteresis) & BMA530_GEN_INT_HYST_MSK);

                    criterion_sel =
                        (BMA5_SET_BITS(int_1_data[5], BMA530_GEN_INT_CRIT_SEL,
                                       gen_int[loop].gen_int.criterion_sel) & BMA530_GEN_INT_CRIT_SEL_MSK);

                    acc_ref_up =
                        (BMA5_SET_BITS(int_1_data[5], BMA530_GEN_INT_ACC_REF_UP,
                                       gen_int[loop].gen_int.acc_ref_up) & BMA530_GEN_INT_ACC_REF_UP_MSK);

                    /* Settings 3 */
                    duration_1 =
                        (BMA5_SET_BITS_POS_0(int_1_data[6], BMA530_GEN_INT_DURATION,
                                             gen_int[loop].gen_int.duration) & BMA530_GEN_INT_DURATION_MSK);

                    duration_2 = (uint16_t)(int_1_data[7] << 8);

                    duration_2 =
                        (BMA5_SET_BITS_POS_0(duration_2, BMA530_GEN_INT_DURATION,
                                             gen_int[loop].gen_int.duration) & BMA530_GEN_INT_DURATION_MSK);

                    wait_time =
                        (BMA5_SET_BITS(int_1_data[7], BMA530_GEN_INT_WAIT_TIME,
                                       gen_int[loop].gen_int.wait_time) & BMA530_GEN_INT_WAIT_TIME_MSK);

                    /* Settings 4 */
                    quiet_time_1 =
                        (BMA5_SET_BITS_POS_0(int_1_data[8], BMA530_GEN_INT_QUIET_TIME,
                                             gen_int[loop].gen_int.quiet_time) & BMA530_GEN_INT_QUIET_TIME_MSK);

                    quiet_time_2 = (uint16_t)(int_1_data[9] << 8);

                    quiet_time_2 =
                        (BMA5_SET_BITS_POS_0(quiet_time_2, BMA530_GEN_INT_QUIET_TIME,
                                             gen_int[loop].gen_int.quiet_time) & BMA530_GEN_INT_QUIET_TIME_MSK);

                    /* Settings 5 */
                    ref_acc_x_1 =
                        BMA5_SET_BITS_POS_0(int_1_data[10], BMA530_GEN_INT_REF_ACC_X,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_x & BMA530_GEN_INT_REF_ACC_X_MSK));

                    ref_acc_x_2 = (uint16_t)(int_1_data[11] << 8);

                    ref_acc_x_2 =
                        BMA5_SET_BITS_POS_0(ref_acc_x_2, BMA530_GEN_INT_REF_ACC_X,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_x & BMA530_GEN_INT_REF_ACC_X_MSK));

                    /* Settings 6 */
                    ref_acc_y_1 =
                        BMA5_SET_BITS_POS_0(int_1_data[12], BMA530_GEN_INT_REF_ACC_Y,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_y & BMA530_GEN_INT_REF_ACC_Y_MSK));

                    ref_acc_y_2 = (uint16_t)(int_1_data[13] << 8);

                    ref_acc_y_2 =
                        BMA5_SET_BITS_POS_0(ref_acc_y_2, BMA530_GEN_INT_REF_ACC_Y,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_y & BMA530_GEN_INT_REF_ACC_Y_MSK));

                    /* Settings 7 */
                    ref_acc_z_1 =
                        BMA5_SET_BITS_POS_0(int_1_data[14], BMA530_GEN_INT_REF_ACC_Z,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_z & BMA530_GEN_INT_REF_ACC_Z_MSK));

                    ref_acc_z_2 = (uint16_t)(int_1_data[15] << 8);

                    ref_acc_z_2 =
                        BMA5_SET_BITS_POS_0(ref_acc_z_2, BMA530_GEN_INT_REF_ACC_Z,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_z & BMA530_GEN_INT_REF_ACC_Z_MSK));

                    int_1_data[0] = (uint8_t)slope_thres_1;
                    int_1_data[1] = (uint8_t)((slope_thres_2 | comb_sel | axis_sel) >> 8);
                    int_1_data[2] = (uint8_t)hysteresis_1;
                    int_1_data[3] = (uint8_t)((hysteresis_2 | criterion_sel | acc_ref_up) >> 8);
                    int_1_data[4] = (uint8_t)duration_1;
                    int_1_data[5] = (uint8_t)((duration_2 | wait_time) >> 8);
                    int_1_data[6] = (uint8_t)quiet_time_1;
                    int_1_data[7] = (uint8_t)(quiet_time_2 >> 8);
                    int_1_data[8] = (uint8_t)ref_acc_x_1;
                    int_1_data[9] = (uint8_t)(ref_acc_x_2 >> 8);
                    int_1_data[10] = (uint8_t)ref_acc_y_1;
                    int_1_data[11] = (uint8_t)(ref_acc_y_2 >> 8);
                    int_1_data[12] = (uint8_t)ref_acc_z_1;
                    int_1_data[13] = (uint8_t)(ref_acc_z_2 >> 8);

                    /* Set the configuration from the feature engine register */
                    result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, int_1_data, 14, dev);
                }
            }
        }
    }

    return result;
}
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.c" line="2017">

---

The function <SwmToken path="bma530_features.c" pos="2017:2:2" line-data="int8_t bma530_get_generic_int_config(struct bma530_generic_interrupt_types *gen_int,">`bma530_get_generic_int_config`</SwmToken> is used to get the current configuration of the generic interrupts. It takes an array of <SwmToken path="bma530_features.c" pos="2017:6:6" line-data="int8_t bma530_get_generic_int_config(struct bma530_generic_interrupt_types *gen_int,">`bma530_generic_interrupt_types`</SwmToken> structures, the number of interrupts to retrieve, and a device handle.

```c
int8_t bma530_get_generic_int_config(struct bma530_generic_interrupt_types *gen_int,
                                     uint8_t n_ints,
                                     struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Variable to store base address of generic interrupt 1 */
    uint8_t data;

    /* Array to store generic interrupt 1 data */
    uint8_t int_1_data[16] = { 0 };

    /* Variable to define array offset */
    uint8_t idx = 0;

    /* Variable to define LSB */
    uint16_t lsb;

    /* Variable to define MSB */
    uint16_t msb;

    /* Variable to define a word */
    uint16_t lsb_msb;

    if (gen_int == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_ints; loop++)
        {
            switch (gen_int[loop].generic_interrupt)
            {
                case BMA530_GEN_INT_1:
                    data = BMA530_BASE_ADDR_GENERIC_INT1;
                    break;

                case BMA530_GEN_INT_2:
                    data = BMA530_BASE_ADDR_GENERIC_INT2;
                    break;

                case BMA530_GEN_INT_3:
                    data = BMA530_BASE_ADDR_GENERIC_INT3;
                    break;

                default:
                    result = BMA5_E_INVALID_GEN_INT;

                    return result;
            }

            if (result == BMA5_OK)
            {
                /* Set the generic interrupt base address to feature engine transmission address to start DMA
                 * transaction */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);
            }

            if (result == BMA5_OK)
            {
                /* Get the configuration from the feature engine register */
                result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, int_1_data, 16, dev);

                if (result == BMA5_OK)
                {
                    /* First two bytes are dummy bytes */
                    idx = 2;

                    /* Settings 1 */
                    /* Get word to calculate slope threshold, comb_sel and axis select from same word */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.slope_thres = lsb_msb & BMA530_GEN_INT_SLOPE_THRES_MSK;

                    gen_int[loop].gen_int.comb_sel = (lsb_msb & BMA530_GEN_INT_COMB_SEL_MSK) >>
                                                     BMA530_GEN_INT_COMB_SEL_POS;

                    gen_int[loop].gen_int.axis_sel = (lsb_msb & BMA530_GEN_INT_AXIS_SEL_MSK) >>
                                                     BMA530_GEN_INT_AXIS_SEL_POS;

                    /* Settings 2 */
                    /* Get word to calculate hysteresis, criterion_sel and acc_ref_up from same word */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.hysteresis = lsb_msb & BMA530_GEN_INT_HYST_MSK;

                    gen_int[loop].gen_int.criterion_sel = (lsb_msb & BMA530_GEN_INT_CRIT_SEL_MSK) >>
                                                          BMA530_GEN_INT_CRIT_SEL_POS;

                    gen_int[loop].gen_int.acc_ref_up = (lsb_msb & BMA530_GEN_INT_ACC_REF_UP_MSK) >>
                                                       BMA530_GEN_INT_ACC_REF_UP_POS;

                    /* Settings 3 */
                    /* Get word to calculate duration and wait time from same word */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.duration = lsb_msb & BMA530_GEN_INT_DURATION_MSK;

                    gen_int[loop].gen_int.wait_time = (lsb_msb & BMA530_GEN_INT_WAIT_TIME_MSK) >>
                                                      BMA530_GEN_INT_WAIT_TIME_POS;

                    /* Settings 4 */
                    /* Get word to calculate quiet time */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.quiet_time = lsb_msb & BMA530_GEN_INT_QUIET_TIME_MSK;

                    /* Settings 5 */
                    /* Get word to calculate ref_acc_x */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.ref_acc_x = (int16_t)(lsb_msb & BMA530_GEN_INT_REF_ACC_X_MSK);

                    /* Settings 6 */
                    /* Get word to calculate ref_acc_y */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.ref_acc_y = (int16_t)(lsb_msb & BMA530_GEN_INT_REF_ACC_Y_MSK);

                    /* Settings 7 */
                    /* Get word to calculate ref_acc_z */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.ref_acc_z = (int16_t)(lsb_msb & BMA530_GEN_INT_REF_ACC_Z_MSK);
                }
            }
        }
    }

    return result;
}
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.c" line="1909">

---

The function <SwmToken path="bma530_features.c" pos="1909:2:2" line-data="int8_t bma530_get_default_generic_int_config(struct bma530_generic_interrupt_types *gen_int,">`bma530_get_default_generic_int_config`</SwmToken> is used to get the default configuration for the generic interrupts. It takes an array of <SwmToken path="bma530_features.c" pos="1909:6:6" line-data="int8_t bma530_get_default_generic_int_config(struct bma530_generic_interrupt_types *gen_int,">`bma530_generic_interrupt_types`</SwmToken> structures, the number of interrupts to retrieve, and a device handle.

```c
int8_t bma530_get_default_generic_int_config(struct bma530_generic_interrupt_types *gen_int,
                                             uint8_t n_ints,
                                             struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Variable to store base address of generic interrupt 1 */
    uint8_t data;

    if (gen_int == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_ints; loop++)
        {
            switch (gen_int[loop].generic_interrupt)
            {
                case BMA530_GEN_INT_1:
                    data = BMA530_BASE_ADDR_GENERIC_INT1;
                    break;

                case BMA530_GEN_INT_2:
                    data = BMA530_BASE_ADDR_GENERIC_INT2;
                    break;

                default:
                    result = BMA5_E_INVALID_GEN_INT;

                    return result;
            }

            if (result == BMA5_OK)
            {
                /* Set the generic interrupt base address to feature engine transmission address to start DMA
                 * transaction */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);
            }

            if (result == BMA5_OK)
            {
                gen_int[loop].gen_int.comb_sel = BMA530_GEN_INT_COMB_SEL_LOGICAL_OR;

                gen_int[loop].gen_int.axis_sel = BMA530_AXIS_SEL_DEFAULT;

                gen_int[loop].gen_int.criterion_sel = BMA530_GEN_INT_CRI_SEL_ACT;

                gen_int[loop].gen_int.acc_ref_up = BMA530_GEN_INT_ACC_REF_UP_ON_ALWAYS;

                gen_int[loop].gen_int.duration = BMA530_DURATION_DEFAULT;

                gen_int[loop].gen_int.wait_time = BMA530_WAIT_TIME_DEFAULT;

                gen_int[loop].gen_int.quiet_time = BMA530_QUIET_TIME_DEFAULT;

                gen_int[loop].gen_int.ref_acc_x = BMA530_ACC_REF_X_DEFAULT;

                gen_int[loop].gen_int.ref_acc_y = BMA530_ACC_REF_Y_DEFAULT;

                gen_int[loop].gen_int.ref_acc_z = BMA530_ACC_REF_Z_DEFAULT;

                if (dev->context == BMA5_HEARABLE && gen_int[loop].generic_interrupt == BMA530_GEN_INT_1)
                {
                    gen_int[loop].gen_int.slope_thres = BMA530_GENERIC_INTERRUPT1_1_GI1_SLOPE_THRES_H;
                    gen_int[loop].gen_int.hysteresis = BMA530_GENERIC_INTERRUPT1_2_GI1_HYSTERESIS_H;
                }
                else if (dev->context == BMA5_WEARABLE && gen_int[loop].generic_interrupt == BMA530_GEN_INT_1)
                {
                    gen_int[loop].gen_int.slope_thres = BMA530_GENERIC_INTERRUPT1_1_GI1_SLOPE_THRES_W;
                    gen_int[loop].gen_int.hysteresis = BMA530_GENERIC_INTERRUPT1_2_GI1_HYSTERESIS_W;
                }
                else if (dev->context == BMA5_SMARTPHONE && gen_int[loop].generic_interrupt == BMA530_GEN_INT_1)
                {
                    gen_int[loop].gen_int.slope_thres = BMA530_GENERIC_INTERRUPT1_1_GI1_SLOPE_THRES_S;
                    gen_int[loop].gen_int.hysteresis = BMA530_GENERIC_INTERRUPT1_2_GI1_HYSTERESIS_S;
                }
                else if (dev->context == BMA5_HEARABLE && gen_int[loop].generic_interrupt == BMA530_GEN_INT_2)
                {
                    gen_int[loop].gen_int.slope_thres = BMA530_GENERIC_INTERRUPT2_1_GI2_SLOPE_THRES_H;
                    gen_int[loop].gen_int.hysteresis = BMA530_GENERIC_INTERRUPT2_2_GI2_HYSTERESIS_H;
                }
                else if (dev->context == BMA5_WEARABLE && gen_int[loop].generic_interrupt == BMA530_GEN_INT_2)
                {
                    gen_int[loop].gen_int.slope_thres = BMA530_GENERIC_INTERRUPT2_1_GI2_SLOPE_THRES_W;
                    gen_int[loop].gen_int.hysteresis = BMA530_GENERIC_INTERRUPT2_2_GI2_HYSTERESIS_W;
                }
                else if (dev->context == BMA5_SMARTPHONE && gen_int[loop].generic_interrupt == BMA530_GEN_INT_2)
                {
                    gen_int[loop].gen_int.slope_thres = BMA530_GENERIC_INTERRUPT2_1_GI2_SLOPE_THRES_S;
                    gen_int[loop].gen_int.hysteresis = BMA530_GENERIC_INTERRUPT2_2_GI2_HYSTERESIS_S;
                }
                else
                {
                    result = BMA5_E_INVALID_CONTEXT_PARAM;
                }
            }
        }
    }

    return result;
}
```

---

</SwmSnippet>

# Usage

<SwmSnippet path="/bma530_features.c" line="1724">

---

The <SwmToken path="bma530_features.c" pos="1724:2:2" line-data="int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,">`bma530_set_generic_int_config`</SwmToken> function sets generic interrupt configurations using the <SwmToken path="bma530_features.c" pos="1724:8:8" line-data="int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,">`bma530_generic_interrupt_types`</SwmToken> class. This function takes a pointer to a <SwmToken path="bma530_features.c" pos="1724:8:8" line-data="int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,">`bma530_generic_interrupt_types`</SwmToken> structure, the number of interrupts, and a device structure as parameters.

```c
int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,
                                     uint8_t n_ints,
                                     struct bma5_dev *dev)
{
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.c" line="1909">

---

The <SwmToken path="bma530_features.c" pos="1909:2:2" line-data="int8_t bma530_get_default_generic_int_config(struct bma530_generic_interrupt_types *gen_int,">`bma530_get_default_generic_int_config`</SwmToken> function retrieves the default generic interrupt 1 configurations. It uses a pointer to a <SwmToken path="bma530_features.c" pos="1909:6:6" line-data="int8_t bma530_get_default_generic_int_config(struct bma530_generic_interrupt_types *gen_int,">`bma530_generic_interrupt_types`</SwmToken> structure, the number of interrupts, and a device structure as parameters.

```c
int8_t bma530_get_default_generic_int_config(struct bma530_generic_interrupt_types *gen_int,
                                             uint8_t n_ints,
                                             struct bma5_dev *dev)
{
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.c" line="2017">

---

The <SwmToken path="bma530_features.c" pos="2017:2:2" line-data="int8_t bma530_get_generic_int_config(struct bma530_generic_interrupt_types *gen_int,">`bma530_get_generic_int_config`</SwmToken> function gets the generic interrupt 1 configurations. Similar to the other functions, it uses a pointer to a <SwmToken path="bma530_features.c" pos="2017:6:6" line-data="int8_t bma530_get_generic_int_config(struct bma530_generic_interrupt_types *gen_int,">`bma530_generic_interrupt_types`</SwmToken> structure, the number of interrupts, and a device structure as parameters.

```c
int8_t bma530_get_generic_int_config(struct bma530_generic_interrupt_types *gen_int,
                                     uint8_t n_ints,
                                     struct bma5_dev *dev)
{
```

---

</SwmSnippet>

&nbsp;

*This is an auto-generated document by Swimm 🌊 and has not yet been verified by a human*

<SwmMeta version="3.0.0" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No" repo-name="BMA530_SensorAPI"><sup>Powered by [Swimm](/)</sup></SwmMeta>
