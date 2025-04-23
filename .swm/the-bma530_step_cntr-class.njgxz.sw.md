---
title: The bma530_step_cntr class
---
This document will cover the class <SwmToken path="bma530_features.c" pos="374:8:8" line-data="int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)">`bma530_step_cntr`</SwmToken> in the file <SwmPath>[bma530_features.c](bma530_features.c)</SwmPath>. We will cover:

1. What <SwmToken path="bma530_features.c" pos="374:8:8" line-data="int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)">`bma530_step_cntr`</SwmToken> is and what it is used for.
2. The variables and functions defined in <SwmToken path="bma530_features.c" pos="374:8:8" line-data="int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)">`bma530_step_cntr`</SwmToken>.

# What is <SwmToken path="bma530_features.c" pos="374:8:8" line-data="int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)">`bma530_step_cntr`</SwmToken>

The <SwmToken path="bma530_features.c" pos="374:8:8" line-data="int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)">`bma530_step_cntr`</SwmToken> is a structure defined in <SwmPath>[bma530_features.c](bma530_features.c)</SwmPath> that is used to manage the step counter configuration for the <SwmToken path="bma530_features.h" pos="843:8:8" line-data=" * \defgroup bma530ApiInit BMA530 Initialization">`BMA530`</SwmToken> sensor. It includes various parameters that control how steps are detected and counted by the sensor.

<SwmSnippet path="/bma530_features.h" line="621">

---

The variable <SwmToken path="bma530_features.h" pos="621:3:3" line-data="    uint16_t watermark_level;">`watermark_level`</SwmToken> is used to set the threshold for triggering an interrupt based on the number of steps counted. If the difference in the number of steps counted from the last event is equal to <SwmToken path="bma530_features.h" pos="98:19:27" line-data=" * steps counted from last event is equal to (set value * 20). If 0, the interrupt is disabled */">`(set value * 20)`</SwmToken>, an interrupt is triggered. If set to 0, the interrupt is disabled.

```c
    uint16_t watermark_level;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="624">

---

The variable <SwmToken path="bma530_features.h" pos="624:3:3" line-data="    uint8_t reset_counter;">`reset_counter`</SwmToken> is used to reset the accumulated step count value.

```c
    uint8_t reset_counter;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="627">

---

The variable <SwmToken path="bma530_features.h" pos="627:3:3" line-data="    uint8_t sd_en;">`sd_en`</SwmToken> is used to enable the step detector.

```c
    uint8_t sd_en;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="630">

---

The variable <SwmToken path="bma530_features.h" pos="630:3:3" line-data="    uint8_t sc_en;">`sc_en`</SwmToken> is used to enable the step counter.

```c
    uint8_t sc_en;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="633">

---

The variable <SwmToken path="bma530_features.h" pos="633:3:3" line-data="    uint16_t envelope_up_thres;">`envelope_up_thres`</SwmToken> sets the threshold for the upper peak of acceleration magnitude for step detection.

```c
    uint16_t envelope_up_thres;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="636">

---

The variable <SwmToken path="bma530_features.h" pos="636:3:3" line-data="    uint16_t envelope_up_decay_coeff;">`envelope_up_decay_coeff`</SwmToken> is an adaptive upper peak threshold decay coefficient.

```c
    uint16_t envelope_up_decay_coeff;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="639">

---

The variable <SwmToken path="bma530_features.h" pos="639:3:3" line-data="    uint16_t envelope_down_thres;">`envelope_down_thres`</SwmToken> sets the threshold for the lower peak of acceleration magnitude for step detection.

```c
    uint16_t envelope_down_thres;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="642">

---

The variable <SwmToken path="bma530_features.h" pos="642:3:3" line-data="    uint16_t envelope_down_decay_coeff;">`envelope_down_decay_coeff`</SwmToken> is an adaptive lower peak threshold decay coefficient.

```c
    uint16_t envelope_down_decay_coeff;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="645">

---

The variable <SwmToken path="bma530_features.h" pos="645:3:3" line-data="    uint16_t acc_mean_decay_coeff;">`acc_mean_decay_coeff`</SwmToken> is an exponential smoothing filter coefficient for computing the mean of acceleration magnitude.

```c
    uint16_t acc_mean_decay_coeff;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="648">

---

The variable <SwmToken path="bma530_features.h" pos="648:3:3" line-data="    uint16_t step_dur_mean_decay_coeff;">`step_dur_mean_decay_coeff`</SwmToken> is an exponential smoothing filter coefficient for computing the mean duration between steps.

```c
    uint16_t step_dur_mean_decay_coeff;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="651">

---

The variable <SwmToken path="bma530_features.h" pos="651:3:3" line-data="    uint8_t step_buffer_size;">`step_buffer_size`</SwmToken> sets the minimum number of consecutive steps to be detected for updating the step count.

```c
    uint8_t step_buffer_size;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="654">

---

The variable <SwmToken path="bma530_features.h" pos="654:3:3" line-data="    uint8_t filter_cascade_enabled;">`filter_cascade_enabled`</SwmToken> enables or disables the cascading of filters.

```c
    uint8_t filter_cascade_enabled;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="657">

---

The variable <SwmToken path="bma530_features.h" pos="657:3:3" line-data="    uint16_t step_counter_increment;">`step_counter_increment`</SwmToken> is a scale factor for the step count to handle overcounting or undercounting.

```c
    uint16_t step_counter_increment;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="660">

---

The variable <SwmToken path="bma530_features.h" pos="660:3:3" line-data="    uint8_t en_half_step;">`en_half_step`</SwmToken> enables or disables the detection of half steps.

```c
    uint8_t en_half_step;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="663">

---

The variable <SwmToken path="bma530_features.h" pos="663:3:3" line-data="    uint8_t peak_duration_min_walking;">`peak_duration_min_walking`</SwmToken> sets the minimum duration between two consecutive steps while walking.

```c
    uint8_t peak_duration_min_walking;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="666">

---

The variable <SwmToken path="bma530_features.h" pos="666:3:3" line-data="    uint8_t peak_duration_min_running;">`peak_duration_min_running`</SwmToken> sets the minimum duration between two consecutive steps while running.

```c
    uint8_t peak_duration_min_running;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="669">

---

The variable <SwmToken path="bma530_features.h" pos="669:3:3" line-data="    uint8_t activity_detection_factor;">`activity_detection_factor`</SwmToken> is the ratio of acceleration magnitude variance during running to walking.

```c
    uint8_t activity_detection_factor;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="672">

---

The variable <SwmToken path="bma530_features.h" pos="672:3:3" line-data="    uint16_t activity_detection_thres;">`activity_detection_thres`</SwmToken> is the acceleration magnitude variance threshold for activity classification.

```c
    uint16_t activity_detection_thres;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="675">

---

The variable <SwmToken path="bma530_features.h" pos="675:3:3" line-data="    uint8_t step_duration_max;">`step_duration_max`</SwmToken> sets the maximum duration between two consecutive step occurrences.

```c
    uint8_t step_duration_max;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="678">

---

The variable <SwmToken path="bma530_features.h" pos="678:3:3" line-data="    uint8_t step_duration_window;">`step_duration_window`</SwmToken> sets the maximum duration since the last step where the next step shall be detected to add a missed step, if any.

```c
    uint8_t step_duration_window;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="681">

---

The variable <SwmToken path="bma530_features.h" pos="681:3:3" line-data="    uint8_t en_step_dur_pp;">`en_step_dur_pp`</SwmToken> enables or disables <SwmToken path="bma530_features.h" pos="178:8:10" line-data="/*! Enable or disable post-processing for duration between steps */">`post-processing`</SwmToken> for the duration between steps.

```c
    uint8_t en_step_dur_pp;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="684">

---

The variable <SwmToken path="bma530_features.h" pos="684:3:3" line-data="    uint8_t step_dur_thres;">`step_dur_thres`</SwmToken> is a scale factor for the mean step duration for step processing.

```c
    uint8_t step_dur_thres;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="687">

---

The variable <SwmToken path="bma530_features.h" pos="687:3:3" line-data="    uint8_t en_mcr_pp;">`en_mcr_pp`</SwmToken> enables or disables <SwmToken path="bma530_features.h" pos="178:8:10" line-data="/*! Enable or disable post-processing for duration between steps */">`post-processing`</SwmToken> of steps based on mean crossing.

```c
    uint8_t en_mcr_pp;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="690">

---

The variable <SwmToken path="bma530_features.h" pos="690:3:3" line-data="    uint16_t mcr_thres;">`mcr_thres`</SwmToken> sets the threshold for the number of mean crossings between two consecutive steps.

```c
    uint16_t mcr_thres;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="693">

---

The variable <SwmToken path="bma530_features.h" pos="693:3:3" line-data="    uint16_t filter_coeff_b_2;">`filter_coeff_b_2`</SwmToken> is the filter coefficient <SwmToken path="bma530_features.h" pos="194:6:6" line-data="/*! Filter coefficient B2 of 2nd order IIR filter */">`B2`</SwmToken> of the 2nd order IIR filter.

```c
    uint16_t filter_coeff_b_2;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="696">

---

The variable <SwmToken path="bma530_features.h" pos="696:3:3" line-data="    uint16_t filter_coeff_b_1;">`filter_coeff_b_1`</SwmToken> is the filter coefficient <SwmToken path="bma530_features.h" pos="198:6:6" line-data="/*! Filter coefficient B1 of 2nd order IIR filter */">`B1`</SwmToken> of the 2nd order IIR filter.

```c
    uint16_t filter_coeff_b_1;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="699">

---

The variable <SwmToken path="bma530_features.h" pos="699:3:3" line-data="    uint16_t filter_coeff_b_0;">`filter_coeff_b_0`</SwmToken> is the filter coefficient <SwmToken path="bma530_features.h" pos="202:6:6" line-data="/*! Filter coefficient B0 of 2nd order IIR filter */">`B0`</SwmToken> of the 2nd order IIR filter.

```c
    uint16_t filter_coeff_b_0;
```

---

</SwmSnippet>

<SwmSnippet path="/bma530_features.h" line="702">

---

The variable <SwmToken path="bma530_features.h" pos="702:3:3" line-data="    uint16_t filter_coeff_a_2;">`filter_coeff_a_2`</SwmToken> is the filter coefficient <SwmToken path="bma530_features.h" pos="206:6:6" line-data="/*! Filter coefficient A2 of 2nd order IIR filter */">`A2`</SwmToken> of the 2nd order IIR filter.

```c
    uint16_t filter_coeff_a_2;
```

---

</SwmSnippet>

# Usage

<SwmSnippet path="/bma530_features.c" line="371">

---

The <SwmToken path="bma530_features.c" pos="374:8:8" line-data="int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)">`bma530_step_cntr`</SwmToken> class is used in the <SwmToken path="bma530_features.c" pos="374:2:2" line-data="int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)">`bma530_set_step_counter_config`</SwmToken> function to set the step counter configuration. This function takes a pointer to a <SwmToken path="bma530_features.c" pos="374:8:8" line-data="int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)">`bma530_step_cntr`</SwmToken> structure and a pointer to a <SwmToken path="bma530_features.c" pos="374:16:16" line-data="int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)">`bma5_dev`</SwmToken> structure as parameters and returns an error code indicating the success or failure of the operation.

```c
/*!
 * @brief This API sets step counter configuration
 */
int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
```

---

</SwmSnippet>

&nbsp;

*This is an auto-generated document by Swimm 🌊 and has not yet been verified by a human*

<SwmMeta version="3.0.0" repo-id="Z2l0aHViJTNBJTNBQk1BNTMwX1NlbnNvckFQSSUzQSUzQVNoYW50YW51TWFuZHBlLUJvc2No" repo-name="BMA530_SensorAPI"><sup>Powered by [Swimm](/)</sup></SwmMeta>
