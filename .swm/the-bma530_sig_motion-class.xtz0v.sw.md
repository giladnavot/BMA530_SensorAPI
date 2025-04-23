---
title: The bma530_sig_motion class
---
This document will cover the class <SwmToken path="bma530_features.c" pos="1181:8:8" line-data="int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_sig_motion`</SwmToken> in the file <SwmPath>[bma530_features.c](bma530_features.c)</SwmPath>. We will cover:

1. What <SwmToken path="bma530_features.c" pos="1181:8:8" line-data="int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_sig_motion`</SwmToken> is and what it is used for.
2. Variables and functions defined in <SwmToken path="bma530_features.c" pos="1181:8:8" line-data="int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_sig_motion`</SwmToken>.

# What is <SwmToken path="bma530_features.c" pos="1181:8:8" line-data="int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_sig_motion`</SwmToken>

The <SwmToken path="bma530_features.c" pos="1181:8:8" line-data="int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_sig_motion`</SwmToken> class in <SwmPath>[bma530_features.c](bma530_features.c)</SwmPath> is used to handle the significant motion detection feature of the <SwmToken path="bma530_features.h" pos="843:8:8" line-data=" * \defgroup bma530ApiInit BMA530 Initialization">`BMA530`</SwmToken> sensor. This feature is used to detect significant movements of the device, which can be useful in various applications such as activity tracking and motion-based user interfaces.

<SwmSnippet path="/bma530_features.h" line="720">

---

The variable <SwmToken path="bma530_features.h" pos="720:3:3" line-data="    uint16_t block_size;">`block_size`</SwmToken> is used to store the size of the segment for detecting significant motion of the device.

```c
    uint16_t block_size;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="723">

---

The variable <SwmToken path="bma530_features.h" pos="723:3:3" line-data="    uint16_t p2p_min;">`p2p_min`</SwmToken> is used to store the minimum value of the peak-to-peak acceleration magnitude.

```c
    uint16_t p2p_min;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="726">

---

The variable <SwmToken path="bma530_features.h" pos="726:3:3" line-data="    uint8_t mcr_min;">`mcr_min`</SwmToken> is used to store the minimum number of mean crossings per second in acceleration magnitude.

```c
    uint8_t mcr_min;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="729">

---

The variable <SwmToken path="bma530_features.h" pos="729:3:3" line-data="    uint16_t p2p_max;">`p2p_max`</SwmToken> is used to store the maximum value of the peak-to-peak acceleration magnitude.

```c
    uint16_t p2p_max;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="732">

---

The variable <SwmToken path="bma530_features.h" pos="732:3:3" line-data="    uint8_t mcr_max;">`mcr_max`</SwmToken> is used to store the maximum number of mean crossings per second in acceleration magnitude.

```c
    uint8_t mcr_max;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.c" line="1269">

---

The function <SwmToken path="bma530_features.c" pos="1269:2:2" line-data="int8_t bma530_get_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_get_sig_motion_config`</SwmToken> is used to get the significant motion configuration from the sensor.

```c
int8_t bma530_get_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of sig-motion */
    uint8_t data = BMA530_BASE_ADDR_SIG_MOTION;

    /* Array to store sig-motion data */
    uint8_t sig_data[8] = { 0 };

    /* Variable to define array offset */
    uint8_t idx;

    /* Variable to define LSB */
    uint16_t lsb;

    /* Variable to define MSB */
    uint16_t msb;

    /* Variable to define a word */
    uint16_t lsb_msb;

    if (sig_mot == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the sig-motion base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, sig_data, 8, dev);

            if (result == BMA5_OK)
            {
                /* First two bytes are dummy bytes */
                idx = 2;

                /* Settings 1 */
                /* Get word to calculate block_size */
                lsb = (uint16_t) sig_data[idx++];
                msb = ((uint16_t) sig_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                sig_mot->block_size = lsb_msb & BMA530_SIG_MOT_BLOCK_SIZE_MSK;

                /* Settings 2 */
                /* Get word to calculate p2p_min and mcr_min from same word */
                lsb = (uint16_t) sig_data[idx++];
                msb = ((uint16_t) sig_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                sig_mot->p2p_min = lsb_msb & BMA530_SIG_MOT_P2P_MIN_MSK;

                sig_mot->mcr_min = (lsb_msb & BMA530_SIG_MOT_MCR_MIN_MSK) >> BMA530_SIG_MOT_MCR_MIN_POS;

                /* Settings 3 */
                /* Get word to calculate p2p_max and mcr_max from same word */
                lsb = (uint16_t) sig_data[idx++];
                msb = ((uint16_t) sig_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                sig_mot->p2p_max = lsb_msb & BMA530_SIG_MOT_P2P_MAX_MSK;

                sig_mot->mcr_max = (lsb_msb & BMA530_SIG_MOT_MCR_MAX_MSK) >> BMA530_SIG_MOT_MCR_MAX_POS;
            }
        }
    }

    return result;
}
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.c" line="1181">

---

The function <SwmToken path="bma530_features.c" pos="1181:2:2" line-data="int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_set_sig_motion_config`</SwmToken> is used to set the significant motion configuration to the sensor.

```c
int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of sig-motion */
    uint8_t data = BMA530_BASE_ADDR_SIG_MOTION;

    /* Array to store sig-motion data */
    uint8_t sig_data[8] = { 0 };

    /* Variables to store the data to be written in register */
    uint16_t block_size_1, block_size_2, p2p_min_1, p2p_min_2, mcr_min, p2p_max_1, p2p_max_2, mcr_max;

    if (sig_mot == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the sig-motion base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, sig_data, 8, dev);

            if (result == BMA5_OK)
            {
                /* Settings 1 */
                block_size_1 =
                    (BMA5_SET_BITS_POS_0(sig_data[2], BMA530_SIG_MOT_BLOCK_SIZE,
                                         sig_mot->block_size) & BMA530_SIG_MOT_BLOCK_SIZE_MSK);

                block_size_2 = (uint16_t)(sig_data[3] << 8);

                block_size_2 =
                    (BMA5_SET_BITS_POS_0(block_size_2, BMA530_SIG_MOT_BLOCK_SIZE,
                                         sig_mot->block_size) & BMA530_SIG_MOT_BLOCK_SIZE_MSK);

                /* Settings 2 */
                p2p_min_1 =
                    (BMA5_SET_BITS_POS_0(sig_data[4], BMA530_SIG_MOT_P2P_MIN,
                                         sig_mot->p2p_min) & BMA530_SIG_MOT_P2P_MIN_MSK);

                p2p_min_2 = (uint16_t)(sig_data[5] << 8);

                p2p_min_2 =
                    (BMA5_SET_BITS_POS_0(p2p_min_2, BMA530_SIG_MOT_P2P_MIN,
                                         sig_mot->p2p_min) & BMA530_SIG_MOT_P2P_MIN_MSK);

                mcr_min =
                    (BMA5_SET_BITS(sig_data[5], BMA530_SIG_MOT_MCR_MIN, sig_mot->mcr_min) & BMA530_SIG_MOT_MCR_MIN_MSK);

                /* Settings 3 */
                p2p_max_1 =
                    (BMA5_SET_BITS_POS_0(sig_data[6], BMA530_SIG_MOT_P2P_MAX,
                                         sig_mot->p2p_max) & BMA530_SIG_MOT_P2P_MAX_MSK);

                p2p_max_2 = (uint16_t)(sig_data[7] << 8);

                p2p_max_2 =
                    (BMA5_SET_BITS_POS_0(p2p_max_2, BMA530_SIG_MOT_P2P_MAX,
                                         sig_mot->p2p_max) & BMA530_SIG_MOT_P2P_MAX_MSK);

                mcr_max =
                    (BMA5_SET_BITS(sig_data[7], BMA530_SIG_MOT_MCR_MAX, sig_mot->mcr_max) & BMA530_SIG_MOT_MCR_MAX_MSK);

                sig_data[0] = (uint8_t)block_size_1;
                sig_data[1] = (uint8_t)(block_size_2 >> 8);
                sig_data[2] = (uint8_t)p2p_min_1;
                sig_data[3] = (uint8_t)((p2p_min_2 | mcr_min) >> 8);
                sig_data[4] = (uint8_t)p2p_max_1;
                sig_data[5] = (uint8_t)((p2p_max_2 | mcr_max) >> 8);

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, sig_data, 6, dev);
            }
        }
    }

    return result;
}
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.c" line="1348">

---

The function <SwmToken path="bma530_features.c" pos="1348:2:2" line-data="int8_t bma530_get_default_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_get_default_sig_motion_config`</SwmToken> is used to get the default significant motion configuration.

```c
int8_t bma530_get_default_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of sig-motion */
    uint8_t data = BMA530_BASE_ADDR_SIG_MOTION;

    if (sig_mot == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the sig-motion base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            if (dev->context == BMA5_HEARABLE)
            {
                /* Settings 1 */

                sig_mot->block_size = BMA530_SIG_MOTION_1_BLOCK_SIZE_H;

                /* Settings 2 */

                sig_mot->p2p_min = BMA530_SIG_MOTION_2_P2P_MIN_H;

                sig_mot->mcr_min = BMA530_SIG_MOTION_2_MCR_MIN_H;

                /* Settings 3 */

                sig_mot->p2p_max = BMA530_SIG_MOTION_3_P2P_MAX_H;

                sig_mot->mcr_max = BMA530_SIG_MOTION_3_MCR_MAX_H;
            }
            else if (dev->context == BMA5_WEARABLE)
            {
                /* Settings 1 */

                sig_mot->block_size = BMA530_SIG_MOTION_1_BLOCK_SIZE_W;

                /* Settings 2 */

                sig_mot->p2p_min = BMA530_SIG_MOTION_2_P2P_MIN_W;

                sig_mot->mcr_min = BMA530_SIG_MOTION_2_MCR_MIN_W;

                /* Settings 3 */

                sig_mot->p2p_max = BMA530_SIG_MOTION_3_P2P_MAX_W;

                sig_mot->mcr_max = BMA530_SIG_MOTION_3_MCR_MAX_W;
            }
            else if (dev->context == BMA5_SMARTPHONE)
            {
                /* Settings 1 */

                sig_mot->block_size = BMA530_SIG_MOTION_1_BLOCK_SIZE_S;

                /* Settings 2 */

                sig_mot->p2p_min = BMA530_SIG_MOTION_2_P2P_MIN_S;

                sig_mot->mcr_min = BMA530_SIG_MOTION_2_MCR_MIN_S;

                /* Settings 3 */

                sig_mot->p2p_max = BMA530_SIG_MOTION_3_P2P_MAX_S;

                sig_mot->mcr_max = BMA530_SIG_MOTION_3_MCR_MAX_S;
            }
            else
            {
                result = BMA5_E_INVALID_CONTEXT_PARAM;
            }
        }
    }

    return result;
}
```

---

</SwmSnippet>

# Usage

<SwmSnippet path="/bma530_features.c" line="1178">

---

The <SwmToken path="bma530_features.c" pos="1181:2:2" line-data="int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_set_sig_motion_config`</SwmToken> function sets the significant motion configuration using the <SwmToken path="bma530_features.c" pos="1181:8:8" line-data="int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_sig_motion`</SwmToken> class. This function takes a pointer to a <SwmToken path="bma530_features.c" pos="1181:8:8" line-data="int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_sig_motion`</SwmToken> structure and a <SwmToken path="bma530_features.c" pos="1181:16:16" line-data="int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma5_dev`</SwmToken> structure as parameters. It initializes a variable to define the error and returns an integer indicating the result of the operation.

```c
/*!
 * @brief This API sets sig-motion configuration
 */
int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.c" line="1266">

---

The <SwmToken path="bma530_features.c" pos="1269:2:2" line-data="int8_t bma530_get_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_get_sig_motion_config`</SwmToken> function retrieves the significant motion configuration using the <SwmToken path="bma530_features.c" pos="1269:6:6" line-data="int8_t bma530_get_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_sig_motion`</SwmToken> class. Similar to the set function, it takes a pointer to a <SwmToken path="bma530_features.c" pos="1269:6:6" line-data="int8_t bma530_get_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_sig_motion`</SwmToken> structure and a <SwmToken path="bma530_features.c" pos="1269:14:14" line-data="int8_t bma530_get_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma5_dev`</SwmToken> structure as parameters. It also initializes a variable to define the error and returns an integer indicating the result of the operation.

```c
/*!
 * @brief This API gets sig-motion configuration
 */
int8_t bma530_get_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.c" line="1345">

---

The <SwmToken path="bma530_features.c" pos="1348:2:2" line-data="int8_t bma530_get_default_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_get_default_sig_motion_config`</SwmToken> function retrieves the default significant motion configuration using the <SwmToken path="bma530_features.c" pos="1348:6:6" line-data="int8_t bma530_get_default_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_sig_motion`</SwmToken> class. This function also takes a pointer to a <SwmToken path="bma530_features.c" pos="1348:6:6" line-data="int8_t bma530_get_default_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma530_sig_motion`</SwmToken> structure and a <SwmToken path="bma530_features.c" pos="1348:14:14" line-data="int8_t bma530_get_default_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)">`bma5_dev`</SwmToken> structure as parameters. It initializes a variable to define the error and returns an integer indicating the result of the operation.

```c
/*!
 * @brief This API gets default sig-motion configuration
 */
int8_t bma530_get_default_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
```

---

</SwmSnippet>

&nbsp;

*This is an auto-generated document by Swimm 🌊 and has not yet been verified by a human*

<SwmMeta version="3.0.0" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No" repo-name="BMA530_SensorAPI"><sup>Powered by [Swimm](/)</sup></SwmMeta>
