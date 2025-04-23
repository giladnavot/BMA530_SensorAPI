---
title: Configuring Feature Engineering Group 0
---
This document describes the process of configuring feature engineering parameters for group 0. The flow involves checking the configuration pointer, reading current register values, setting new values based on the configuration, and writing these values back to the register. For example, if the configuration enables features like step detection and orientation, the register values will be updated to reflect these settings.

```mermaid
sequenceDiagram
  participant System
  participant Sensor
  System->>Sensor: Check configuration pointer
  Sensor->>System: Read current register values
  System->>Sensor: Set new register values
  Sensor->>System: Write updated values
```

# Where is this flow used?

This flow is used multiple times in the codebase as represented in the following diagram:

(Note - these are only some of the entry points of this flow)

```mermaid
graph TD;
      subgraph bma530c["bma530.c"]
48131c6a37ed2530f3ba7afc870fab40b771246af87f173d2488dab9bebc2787(main) --> 2b54eec76512cc0f3ef72e904f8ed5142391fd60ef8372a53f72afb287f7046b(bma530_set_feat_eng_gpr_0):::mainFlowStyle
end

classDef mainFlowStyle color:#000000,fill:#7CB9F4
classDef rootsStyle color:#000000,fill:#00FFF4
classDef Style1 color:#000000,fill:#00FFAA
classDef Style2 color:#000000,fill:#FFFF00
classDef Style3 color:#000000,fill:#AA7CB9
```

# Initializing Feature Engineering Group 0

```mermaid
flowchart TD
    node1{Check configuration pointer} -->|Not null| node2[Reading Register Values]
    node2 --> node3[Set new register values based on configuration]

subgraph node2 [bma5_get_regs]
  sgmain_1_node1[Start reading register data] --> sgmain_1_node2{Verify device handle}
  sgmain_1_node2 -->|Valid| sgmain_1_node3[Check data pointer and configure interface]
  sgmain_1_node3 -->|Data pointer valid and read successful| loop1[Copy data from buffer]
  subgraph loop1[For each byte to copy]
  sgmain_1_node3 --> sgmain_1_node3
  end
 
end
```

<SwmSnippet path="/bma530.c" line="396" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

We start the flow by checking if the configuration pointer is null, returning an error if so. Next, we call bma5_get_regs to read the current register value, which is necessary for modifying specific bits based on the configuration.

```c
int8_t bma530_set_feat_eng_gpr_0(const struct bma530_feat_eng_gpr_0 *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    uint8_t gen_int1_en, gen_int2_en, gen_int3_en, step_en, sig_mo_en, tilt_en, orient_en, acc_foc_en;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA530_REG_FEAT_ENG_GPR_0, &reg_value, sizeof(reg_value), dev);

```

---

</SwmSnippet>

## Reading Register Values

```mermaid
flowchart TD
    node1[Start reading register data] --> node2{Verify device handle}
    node2 -->|Valid| node3[Check data pointer and configure interface]
    node3 -->|Data pointer valid and read successful| loop1[Copy data from buffer]

    subgraph loop1[For each byte to copy]
        node3 --> node3
    end
    
    loop1 --> node4[Return result]

subgraph node2 [verify_handle]
  sgmain_1_node1[Check if device and its operations are valid] --> sgmain_1_node2{Is device pointer not null?}
  sgmain_1_node2 -->|Yes| sgmain_1_node3{Are device operations valid?}
  sgmain_1_node3 -->|Yes| sgmain_1_node4[Return valid handle]
  sgmain_1_node3 -->|No| sgmain_1_node5[Return error]
  sgmain_1_node2 -->|No| sgmain_1_node5
end
```

<SwmSnippet path="/bma5.c" line="1333" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

Next, we read data from the specified register, initializing a temporary buffer for storage. We call verify_handle to ensure the device handle is valid before proceeding.

```c
 * @brief This API reads the data from the given register address of bma5
 *        sensor.
 */
int8_t bma5_get_regs(uint8_t addr, uint8_t *data, uint32_t len, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary buffer to receive the serial data from sensor */
    uint8_t temp_buf[BMA5_MAX_BUFFER_SIZE] = { 0 };

    /* Number of bytes to be copied from temp_buf to data */
    uint32_t bytes_to_copy = len;

    result = verify_handle(dev);
```

---

</SwmSnippet>

### Validating Device Handle

```mermaid
flowchart TD
    node1[Check if device and its operations are valid] --> node2{Is device pointer not null?}
    node2 -->|Yes| node3{Are device operations valid?}
    node3 -->|Yes| node4[Return valid handle]
    node3 -->|No| node5[Return error]
    node2 -->|No| node5
```

<SwmSnippet path="/bma5.c" line="1698" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

This is the implementation of verify_handle, returning a success code if the device and its function pointers are valid, otherwise returning an error for a null pointer.

```c
/*********************** Static function definitions **************************/
/******************************************************************************/
static int8_t verify_handle(const struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result = BMA5_E_NULL_PTR;

    if (NULL != dev)
    {
        if ((NULL != dev->bus_read) && (NULL != dev->bus_write) && (NULL != dev->delay_us))
        {
            result = BMA5_OK;
        }
    }

    return result;
}
```

---

</SwmSnippet>

### Processing Register Read

```mermaid
flowchart TD
    node1[Start reading sensor registers] -->|Result OK and data not NULL| node2{SPI Interface?}
    node2 -->|Yes| node3[Configure register address]
    node2 -->|No| node4[Read data from registers]
    node3 --> node4
    node4 -->|Communication success| node5[Copy data to output]

    subgraph loop1[Copy each byte from temp_buf to data]
        node5 --> node5
    end

    node5 --> node6[Finish reading process]
```

<SwmSnippet path="/bma5.c" line="1348" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

Back in bma5_get_regs, after verify_handle, we check the data pointer and configure the register address for SPI. We perform the read operation and ensure communication success before copying data.

```c
    if ((BMA5_OK == result) && (NULL == data))
    {
        result = BMA5_E_NULL_PTR;
    }

    if (BMA5_OK == result)
    {
        /* Configuring register address for SPI Interface */
        if (BMA5_SPI_INTF == dev->intf)
        {
            addr = addr | BMA5_SPI_RD_MSK;
        }

        dev->intf_rslt = dev->bus_read(addr, temp_buf, (len + dev->dummy_byte), dev->intf_ptr);

        if (BMA5_INTF_RET_SUCCESS != dev->intf_rslt)
        {
            result = BMA5_E_COM_FAIL;
        }
    }

    if (BMA5_OK == result)
    {
        while (bytes_to_copy--)
        {
            data[bytes_to_copy] = temp_buf[bytes_to_copy + dev->dummy_byte];
        }
```

---

</SwmSnippet>

## Configuring Feature Engineering Parameters

<SwmSnippet path="/bma530.c" line="414" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

Back in bma530_set_feat_eng_gpr_0, after bma5_get_regs, we set up the register value based on configuration. We call bma5_set_regs to write the new value, applying the changes.

```c
        if (BMA5_OK == result)
        {
            /* Bring up the register value to be set, as per the input details */
            gen_int1_en =
                (BMA5_SET_BITS_POS_0(reg_value, BMA530_GEN_INT1_EN, config->gen_int1_en) & BMA530_GEN_INT1_EN_MSK);
            gen_int2_en = (BMA5_SET_BITS(reg_value, BMA530_GEN_INT2_EN, config->gen_int2_en) & BMA530_GEN_INT2_EN_MSK);
            gen_int3_en = (BMA5_SET_BITS(reg_value, BMA530_GEN_INT3_EN, config->gen_int3_en) & BMA530_GEN_INT3_EN_MSK);
            step_en = (BMA5_SET_BITS(reg_value, BMA530_STEP_EN, config->step_en) & BMA530_STEP_EN_MSK);
            sig_mo_en = (BMA5_SET_BITS(reg_value, BMA530_SIG_MO_EN, config->sig_mo_en) & BMA530_SIG_MO_EN_MSK);
            tilt_en = (BMA5_SET_BITS(reg_value, BMA530_TILT_EN, config->tilt_en) & BMA530_TILT_EN_MSK);
            orient_en = (BMA5_SET_BITS(reg_value, BMA530_ORIENT_EN, config->orient_en) & BMA530_ORIENT_EN_MSK);
            acc_foc_en = (BMA5_SET_BITS(reg_value, BMA530_ACC_FOC_EN, config->acc_foc_en) & BMA530_ACC_FOC_EN_MSK);

            reg_value =
                (uint8_t)(gen_int1_en | gen_int2_en | gen_int3_en | step_en | sig_mo_en | tilt_en | orient_en |
                          acc_foc_en);

            result = bma5_set_regs(BMA530_REG_FEAT_ENG_GPR_0, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
        }
    }

    return result;
}
```

---

</SwmSnippet>

# Writing Register Values

```mermaid
flowchart TD
    node1[Start: Write data to BMA5 sensor register]
    node1 -->|Verify device handle| node2{Is handle valid?}
    node2 -->|No| node3[Return error: Invalid handle]
    node2 -->|Yes| node4{Is data pointer null?}
    node4 -->|Yes| node5[Return error: Null data pointer]
    node4 -->|No| node6{Is interface SPI?}
    node6 -->|Yes| node7[Adjust address for SPI]
    node6 -->|No| node8[Proceed with bus write]
    node7 --> node8
    node8 -->|Bus write success?| node9{Success?}
    node9 -->|No| node10[Return error: Communication failure]
    node9 -->|Yes| node11[Return success]
```

<SwmSnippet path="/bma5.c" line="1381" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

First, we write data to the specified register, calling verify_handle to ensure the device handle is valid before proceeding.

```c
 * @brief This API writes data to the given register address of bma5 sensor.
 */
int8_t bma5_set_regs(uint8_t addr, const uint8_t *data, uint32_t len, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    result = verify_handle(dev);
```

---

</SwmSnippet>

<SwmSnippet path="/bma5.c" line="1389" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

Back in bma5_set_regs, after verify_handle, we check the data pointer and configure the register address for SPI. We perform the write operation and ensure communication success before returning the result.

```c
    if ((BMA5_OK == result) && (NULL == data))
    {
        result = BMA5_E_NULL_PTR;
    }

    if (BMA5_OK == result)
    {
        /* Configuring register address for SPI Interface */
        if (BMA5_SPI_INTF == dev->intf)
        {
            addr = (addr & BMA5_SPI_WR_MSK);
        }

        dev->intf_rslt = dev->bus_write(addr, data, len, dev->intf_ptr);

        if (BMA5_INTF_RET_SUCCESS != dev->intf_rslt)
        {
            result = BMA5_E_COM_FAIL;
        }
    }

    return result;
}
```

---

</SwmSnippet>

&nbsp;

*This is an auto-generated document by Swimm 🌊 and has not yet been verified by a human*

<SwmMeta version="3.0.0"><sup>Powered by [Swimm](https://app.swimm.io/)</sup></SwmMeta>
