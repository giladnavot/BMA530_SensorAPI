/**
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include "common.h"
#include "bma530_features.h"

/******************************************************************************/
int main(void)
{
    int8_t rslt;
    uint8_t gpr_ctrl_host = BMA5_ENABLE;
    uint8_t n_ints = 1;
    uint8_t n_status = 1;

    /*Structue to hold the configurations */
    struct bma5_dev dev;
    struct bma530_int_map int_map = { 0 };
    struct bma5_int_conf_types int_config = { 0 };
    struct bma530_sig_motion conf = { 0 };
    struct bma530_feat_eng_gpr_0 gpr_0 = { 0 };
    struct bma530_int_status_types int_status = { 0 };

    /* Mapping to hardware interrupt pin on sensor */
    int_config.int_src = BMA5_INT_1;

    /* Variable to hold configurations related to interrupt pin */
    int_status.int_src = BMA530_INT_STATUS_INT1;

    /* Assign context parameter selection */
    enum bma5_context context;
    context = BMA5_SMARTPHONE;

    /* Interface reference is given as a parameter
     *         For I2C : BMA5_I2C_INTF
     *         For SPI : BMA5_SPI_INTF
     */
    rslt = bma5_interface_init(&dev, BMA5_SPI_INTF, context);
    bma5_check_rslt("bma5_interface_init", rslt);

    /* Initialize the device BMA530 */
    rslt = bma530_init(&dev);
    bma5_check_rslt("bma530_init", rslt);
    printf("Chip ID:0x%x\n\n", dev.chip_id);

    printf("\nDefault configurations\n\n");
    rslt = bma530_get_default_sig_motion_config(&conf, &dev);
    bma5_check_rslt("bma530_get_default_sig_motion_config", rslt);

    printf("block_size :: 0x%x\n", conf.block_size);
    printf("mcr_max :: 0x%x\n", conf.mcr_max);
    printf("mcr_min :: 0x%x\n", conf.mcr_min);
    printf("p2p_max :: 0x%x\n", conf.p2p_max);
    printf("p2p_min :: 0x%x\n", conf.p2p_min);

    /* Set the configurations */
    conf.block_size = 0xFA;
    conf.mcr_max = 0x11;
    conf.mcr_min = 0x11;
    conf.p2p_max = 0x253;
    conf.p2p_min = 0x26;

    rslt = bma530_set_sig_motion_config(&conf, &dev);
    bma5_check_rslt("bma530_set_sig_motion_config", rslt);

    printf("\nConfigurations set\n\n");
    printf("block_size :: 0x%x\n", conf.block_size);
    printf("mcr_max :: 0x%x\n", conf.mcr_max);
    printf("mcr_min :: 0x%x\n", conf.mcr_min);
    printf("p2p_max :: 0x%x\n", conf.p2p_max);
    printf("p2p_min :: 0x%x\n", conf.p2p_min);

    rslt = bma530_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma530_get_feat_eng_gpr_0", rslt);

    /* Enable significant motion */
    gpr_0.sig_mo_en = BMA5_ENABLE;

    rslt = bma530_set_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma530_set_feat_eng_gpr_0", rslt);

    if (rslt == BMA5_OK)
    {
        printf("\nSignificant motion enabled\n");
    }

    rslt = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    /* Get the interrupt mapping */
    rslt = bma530_get_int_map(&int_map, &dev);
    bma5_check_rslt("bma530_get_int_map", rslt);

    /* Map sig motion */
    int_map.sig_mo_int_map = BMA530_SIG_MO_INT_MAP_INT1;
    rslt = bma530_set_int_map(&int_map, &dev);
    bma5_check_rslt("bma530_set_int_map", rslt);

    /* Map hardware interrupt pin configurations */
    rslt = bma5_get_int_conf(&int_config, n_ints, &dev);
    bma5_check_rslt("bma5_get_int_conf", rslt);

    /* Set the hardware interrupt pin configuration */
    int_config.int_conf.int_mode = BMA5_INT1_MODE_PULSED_SHORT;
    int_config.int_conf.int_od = BMA5_INT1_OD_PUSH_PULL;
    int_config.int_conf.int_lvl = BMA5_INT1_LVL_ACTIVE_LOW;

    rslt = bma5_set_int_conf(&int_config, n_ints, &dev);
    bma5_check_rslt("bma5_set_int_conf", rslt);

    printf("\nInterrupt configurations\n");
    printf("\nINT1 Mode = %s\t\n", enum_to_string(BMA5_INT1_MODE_PULSED_SHORT));
    printf("INT1 Output type = %s\t\n", enum_to_string(BMA5_INT1_OD_PUSH_PULL));
    printf("INT1 Active level = %s\t\n", enum_to_string(BMA5_INT1_LVL_ACTIVE_HIGH));

    printf("\nMove the board to get significant motion interrupt\n");

    for (;;)
    {
        /* Get the interrupt status */
        rslt = bma530_get_int_status(&int_status, n_status, &dev);
        bma5_check_rslt("bma530_get_int_status", rslt);

        /* Check if significant motion interrupt occurred */
        if (int_status.int_status.sig_mo_int_status & BMA5_ENABLE)
        {
            rslt = bma530_set_int_status(&int_status, n_status, &dev);
            bma5_check_rslt("bma530_set_int_status", rslt);

            printf("Significant motion interrupt occurred\n");

            break;
        }
    }

    bma5_coines_deinit();

    return rslt;
}
