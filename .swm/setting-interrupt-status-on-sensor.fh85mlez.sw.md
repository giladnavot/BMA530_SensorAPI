---
title: Setting Interrupt Status on Sensor
---
This document describes the process of setting interrupt statuses on the sensor. The flow involves validating the input configurations, processing each interrupt status, determining the interrupt source, and setting the appropriate registers. For example, if the configuration specifies an interrupt for data ready and FIFO full, the flow will set these statuses in the sensor's registers and return a success result.

The main steps are:

- Validate the configuration input.
- Process each interrupt status configuration.
- Determine the interrupt source.
- Set the appropriate register based on the interrupt source.
- Write the register values to the sensor.
- Return the result of the operation.

```mermaid
sequenceDiagram
  participant Config
  participant Sensor
  Config->>Sensor: Validate configuration
  loop Process each status
    Config->>Sensor: Determine interrupt source
    Sensor->>Sensor: Set register
  end
  Sensor->>Sensor: Write to registers
  Sensor->>Config: Return result
```

# Where is this flow used?

This flow is used multiple times in the codebase as represented in the following diagram:

(Note - these are only some of the entry points of this flow)

```mermaid
graph TD;
     









subgraph bma530c["bma530.c"]
90397682c75b2db97937f55d6f54abbdc5345b77bd0a0a2a6301e9156f912c66(get_fifo_full_8_bit_data) --> 92d5fea71247f5432aaa9ab86b9f38d5801e1588be3d0affc50542be1a4ce884(bma530_set_int_status):::mainFlowStyle
end

995b7fc0fd14f2c4b3f5c5598f034967f07410e6b23421882a261c2a19d2ab69(main) --> 90397682c75b2db97937f55d6f54abbdc5345b77bd0a0a2a6301e9156f912c66(get_fifo_full_8_bit_data)



subgraph bma530c["bma530.c"]
cc403b1810ce0a74a5616c13d964bb3e231d19a414192427644d3362d100f159(get_fifo_full_16_bit_data) --> 92d5fea71247f5432aaa9ab86b9f38d5801e1588be3d0affc50542be1a4ce884(bma530_set_int_status):::mainFlowStyle
end

9165887b073c46519c72eb2348bd335ecaa957754fefcab085e18349d280721c(main) --> cc403b1810ce0a74a5616c13d964bb3e231d19a414192427644d3362d100f159(get_fifo_full_16_bit_data)


classDef mainFlowStyle color:#000000,fill:#7CB9F4
classDef rootsStyle color:#000000,fill:#00FFF4
classDef Style1 color:#000000,fill:#00FFAA
classDef Style2 color:#000000,fill:#FFFF00
classDef Style3 color:#000000,fill:#AA7CB9
```

# Setting Interrupt Status

```mermaid
flowchart TD
    node1[Start setting interrupt status] --> node2{Is config null?}
    node2 -->|No| node3[Process each interrupt status]

    subgraph loop1[For each interrupt status configuration]
        node3 --> node4{Determine interrupt source}
        node4 -->|INT1| node5[Set INT1 register]
        node4 -->|INT2| node6[Set INT2 register]
        node4 -->|I3C| node7[Set I3C register]
        node4 -->|Invalid| node8[Handle invalid status]
        node5 --> node3
        node6 --> node3
        node7 --> node3
        node8 --> node3
    end

    node3 --> node9[Return result]

subgraph node2 [bma5_set_regs]
  sgmain_1_node1[Validating Device Handle] --> sgmain_1_node2{Data null check}
  sgmain_1_node2 -->|Not Null| sgmain_1_node3[Write data to register]
  sgmain_1_node3 --> sgmain_1_node4[Return write result]
end
```

<SwmSnippet path="/bma530.c" line="135" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

We start the flow by preparing the register values based on the provided configurations. Next, we call bma5_set_regs to write these values to the sensor's registers, which is necessary to update the interrupt status on the sensor.

```c
int8_t bma530_set_int_status(const struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value[2] = { 0 };

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_status; loop++)
        {
            /* Bring up the register value to be set, as per the input details */
            reg_value[0] = BMA5_SET_BITS_POS_0(reg_value[0],
                                               BMA530_ACC_DRDY_INT_STATUS,
                                               config[loop].int_status.acc_drdy_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_FIFO_WM_INT_STATUS,
                                         config[loop].int_status.fifo_wm_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_FIFO_FULL_INT_STATUS,
                                         config[loop].int_status.fifo_full_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_GEN_INT1_INT_STATUS,
                                         config[loop].int_status.gen_int1_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_GEN_INT2_INT_STATUS,
                                         config[loop].int_status.gen_int2_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_GEN_INT3_INT_STATUS,
                                         config[loop].int_status.gen_int3_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_STEP_DET_INT_STATUS,
                                         config[loop].int_status.step_det_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_STEP_CNT_INT_STATUS,
                                         config[loop].int_status.step_cnt_int_status);

            /* Bring up the register value to be set, as per the input details */
            reg_value[1] = BMA5_SET_BITS_POS_0(reg_value[1],
                                               BMA530_SIG_MO_INT_STATUS,
                                               config[loop].int_status.sig_mo_int_status);
            reg_value[1] = BMA5_SET_BITS(reg_value[1], BMA530_TILT_INT_STATUS, config[loop].int_status.tilt_int_status);
            reg_value[1] = BMA5_SET_BITS(reg_value[1],
                                         BMA530_ORIENT_INT_STATUS,
                                         config[loop].int_status.orient_int_status);
            reg_value[1] = BMA5_SET_BITS(reg_value[1],
                                         BMA530_ACC_FOC_INT_STATUS,
                                         config[loop].int_status.acc_foc_int_status);
            reg_value[1] = BMA5_SET_BITS(reg_value[1],
                                         BMA530_FEAT_ENG_ERR_INT_STATUS,
                                         config[loop].int_status.feat_eng_err_int_status);

            switch (config[loop].int_src)
            {
                case BMA530_INT_STATUS_INT1:
                    result = bma5_set_regs(BMA530_REG_INT_STATUS_INT1_0, reg_value, 2, dev);
                    break;

```

---

</SwmSnippet>

## Writing to Sensor Registers

```mermaid
flowchart TD
    node1[Validating Device Handle] --> node2{Data null check}
    node2 -->|Not Null| node3[Write data to register]
    node3 --> node4[Return write result]

subgraph node1 [verify_handle]
  sgmain_1_node1[Validating Device Handle]
end
```

<SwmSnippet path="/bma5.c" line="1381" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

Next, we write data to the specified register address. We call verify_handle to ensure the device structure is valid, which is necessary to prevent errors during the writing process.

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

### Validating Device Handle

<SwmSnippet path="/bma5.c" line="1698" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

Returns BMA5_OK if the device structure is valid, otherwise returns BMA5_E_NULL_PTR.

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

### Handling Register Write Logic

```mermaid
flowchart TD
    node1[Start setting registers] --> node2{Is result OK and data not NULL?}
    node2 -->|No| node3[Return NULL pointer error]
    node2 -->|Yes| node4{Is SPI interface used?}
    node4 -->|Yes| node5[Configure register address for SPI]
    node4 -->|No| node6[Write data to bus]
    node5 --> node6
    node6 --> node7{Was write successful?}
    node7 -->|No| node8[Return communication failure error]
    node7 -->|Yes| node9[Return success]
```

<SwmSnippet path="/bma5.c" line="1389" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

Back in bma5_set_regs, we check if the data pointer is NULL and configure the register address for SPI. We perform the bus write operation and return the result.

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

## Processing Interrupt Source INT2

<SwmSnippet path="/bma530.c" line="199" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

Back in the function, we process interrupt source INT2 and call bma5_set_regs to update the interrupt status.

```c
                case BMA530_INT_STATUS_INT2:
                    result = bma5_set_regs(BMA530_REG_INT_STATUS_INT2_0, reg_value, 2, dev);
                    break;

```

---

</SwmSnippet>

## Processing Interrupt Source I3C

<SwmSnippet path="/bma530.c" line="203" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

Back in the function, we process interrupt source I3C and call bma5_set_regs to update the interrupt status.

```c
                case BMA530_INT_STATUS_I3C:
                    result = bma5_set_regs(BMA530_REG_INT_STATUS_I3C_0, reg_value, 2, dev);
                    break;
```

---

</SwmSnippet>

## Finalizing Interrupt Status Update

<SwmSnippet path="/bma530.c" line="206" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No">

---

Back in the function, we check the result and break the loop on error, then return the result to complete the process.

```c
                default:
                    result = BMA5_E_INVALID_INT_STATUS;
            }

            if (BMA5_OK != result)
            {
                break;
            }
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
