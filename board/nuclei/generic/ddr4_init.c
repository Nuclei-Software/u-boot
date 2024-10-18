#include <linux/types.h>
#include "ddr_init.h"

#define  DDR_DBG_EN                         0

#define  REG32(addr)                        (*((volatile unsigned int*)(addr)))
#define  DDR_TOP0_BASE                      (0xf8b400000ULL)
#define  DDR_TOP0_CFG_PHY_BASE              (0xf8b500000ULL)

#define  DDR_TOP0_CLK_MUX_SEL_OSC_CLK_25M    (0UL)
#define  DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL    (1UL)
#define  DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL_BK (2UL)


enum clock_pll_e{
    SYS_CLK_PLL,
    SYS_CLK_PLL_BK,
    XDC_CLK_PLL,
    XDC_CLK_PLL_BK,
    XEC_CLK_PLL,
    XUC_CLK_PLL,
    SYSCTL_PLL_MAX
};

static void ddr4_mrs(uint32_t rank, uint32_t mr_addr, uint32_t mr_data)
{
    while (1)
    {
        //delay_1ms(1);
        if ((REG32(DDR_TOP0_BASE+XDC_MRL3_OFFSET) & XDC_MRL3_MR_WR_BUSY) == 0)
            break;
    }

    // Configure MR OP
    REG32(DDR_TOP0_BASE+XDC_MRL0_OFFSET) =
                                        XDC_MRL0_MR_RANK(rank & 0x3) |
                                        XDC_MRL0_MR_ADDR(mr_addr & 0xf);
    REG32(DDR_TOP0_BASE+XDC_MRL1_OFFSET) =
                                        XDC_MRL1_MR_DATA(mr_data);
    // Trigger MR OP                                    
    REG32(DDR_TOP0_BASE+XDC_MRL0_OFFSET) =
                                        XDC_MRL0_MR_RANK(rank & 0x3) |
                                        XDC_MRL0_MR_ADDR(mr_addr & 0xf) |
                                        XDC_MRL0_MR_WR_VAL(1);

    // Polling MR OP busy
    while (1)
    {
        //delay_1ms(1);
        if ((REG32(DDR_TOP0_BASE+XDC_MRL3_OFFSET) & XDC_MRL3_MR_WR_BUSY) == 0)
            break;
    }
}

static uint32_t check_train_status(uint32_t cs_map)
{
    uint32_t status;
    uint32_t train_err = 0;

#if DDR_DBG_EN == 1
    printf("\n/*****trained delay registers:*****/\n");
    read_dbyte_trained_dly(0x0, 0x0);
    read_dbyte_trained_dly(0x1, 0x0);
    if (cs_map & 0x2)
    {
        read_dbyte_trained_dly(0x0, 0x1);
        read_dbyte_trained_dly(0x1, 0x1); 
    }

    printf("\n/*****trained obs dly:*****/\n");
    // read_wrlvl_obs_dly(0x0);
    // read_gtlvl_obs_dly(0x0);
    read_rdlvl_obs_dly(0x0);
    read_wdqlvl_obs_dly(0x0);

    // read_wrlvl_obs_dly(0x1);
    // read_gtlvl_obs_dly(0x1);
    read_rdlvl_obs_dly(0x1);
    read_wdqlvl_obs_dly(0x1);

    printf("\n/*****training status:*****/\n");
    parse_mstr_obs(0);
#endif

    status = REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_CBT_STATUS_OFFSET);
    train_err = train_err | status;
#if DDR_DBG_EN == 1
    printf("cbt_status=%x\n", status);
#endif

    for (uint32_t i=0; i<2; i++)
    {
        status = REG32(DDR_TOP0_CFG_PHY_BASE+0x2000*i+0x40000+DBYTE_AF_DB_LVL_STATUS_OFFSET);
        train_err = train_err | status;
#if DDR_DBG_EN == 1
        printf("db%d_lvl_status=%x (r0_wrlvl_err=%1x, r0_gtlvl_err=%1x, r0_rdlvl_err=%1x, r0_wdqlvl_err=%1x)\n",
                i, status,
                (status & 0x1),
                (status & 0x2) >> 1,
                (status & 0x30) >> 4,
                (status & 0x40) >> 6);
        if (cs_map & 0x2)
        {
            printf("db%d_lvl_status=%x (r1_wrlvl_err=%1x, r1_gtlvl_err=%1x, r1_rdlvl_err=%1x, r1_wdqlvl_err=%1x)\n",
                i, status,
                (status & 0x100) >> 8,
                (status & 0x200) >> 9,
                (status & 0x3000) >> 12,
                (status & 0x4000) >> 14);
        }
#endif
    }
    
    status = REG32(DDR_TOP0_CFG_PHY_BASE+0x60034); // IDT_LVL_STATUS (check training status, must be 0x0)
    train_err = train_err | status;
#if DDR_DBG_EN == 1
    printf("idt_lvl_status:%x (r0_wrlvl_err=%1x, r0_gtlvl_err=%1x, r0_rdlvl_err=%1x, r0_wdqlvl_err=%1x)\n",
                status,
                (status & 0x1),
                (status & 0x2) >> 1,
                (status & 0x4) >> 2,
                (status & 0x8) >> 3);
    if (cs_map & 0x2)
    {
        printf("idt_lvl_status:%x (r1_wrlvl_err=%1x, r1_gtlvl_err=%1x, r1_rdlvl_err=%1x, r1_wdqlvl_err=%1x)\n",
                status,
                (status & 0x100) >> 8,
                (status & 0x200) >> 9,
                (status & 0x400) >> 10,
                (status & 0x800) >> 11);
    }
#endif

    return train_err;
					   
		
					 
}

int32_t ddr_init(void)
{
    uint32_t ddr_pll_freq_val = 1600;
    uint32_t cs_map = 0x3;
    uint32_t train_dbg_mode = 0;
    uint32_t wrlvl_en = 1;
    uint32_t gtlvl_en = 1;
    uint32_t rdlvl_en = 1;
    uint32_t wdqlvl_en = 1;

    uint32_t dev_srf_en = 1;
    uint32_t dev_pwd_en = 0;
    uint32_t dll_on = 1;
    uint32_t dev_drv = 0;
    uint32_t rtt_nom = 0;
    uint32_t rtt_wr = ddr_pll_freq_val > 800 ? 0x4 : 0x0;
    uint32_t rtt_park = 0;
    uint32_t dq_vref_init_val = 0x8;//rtt_wr == 0 ? 0x50 : 0x10;
    uint32_t dq_vref_st = 0x40;//rtt_wr == 0 ? 0x40 : 0x0;
    uint32_t dq_vref_ed = 0x72;//rtt_wr == 0 ? 0x72 : 0x32;
    uint32_t dq_vref_step= 0x1;
    uint32_t slewp = 0x4;
    uint32_t slewn = 0x4;
    uint32_t ac_vref_en = 0;
    uint32_t ac_vrefsel = 0x50;
    uint32_t ac_enslicep = 0xf;
    uint32_t ac_enslicen = 0xf;
    uint32_t wr_enslicep = 0xf;
    uint32_t wr_enslicen = 0xf;
    uint32_t rd_enslicep = 0x1;
    uint32_t rd_enslicen = 0x0;
    uint32_t rd_tsel_en = ddr_pll_freq_val > 800 ? 1 : 0;
    uint32_t vrefsel = rd_tsel_en ? 0x6060 : 0x4040;
    uint32_t vref_stb_time = (uint32_t)(4000.0 / (1000.0 / ((float)ddr_pll_freq_val / 4.0))); // 4us
    uint32_t t_refi_val_x32 = (uint32_t)(7800.0 / (1000.0 / ((float)ddr_pll_freq_val / 4.0)) / 32.0); // 7.8us

    uint32_t wrlvl_all_dq_en = 1;
    uint32_t wrlvl_capt_cnt = 2;
    uint32_t wrlvl_dly_step = 0x4;
    uint32_t wrlvl_eval = 0x000;
    uint32_t wrdqs_dly = 0x10;

    uint32_t gtlvl_start = 0x0;
    uint32_t gtlvl_dly = 0x1803;
    uint32_t gtlvl_capt_cnt = 2;
    uint32_t gtlvl_dly_step = 0x4;
    uint32_t gtlvl_back_step = 0xe8;

    uint32_t rddq_dly_enc = 0x0;
    uint32_t rddqs_r_dly = 0x80;
    uint32_t rddqs_f_dly = 0x80;
    uint32_t rdlvl_capt_cnt = 2;
    uint32_t rdlvl_dly_step = 0x4;
    uint32_t rdlvl_bit_mask = 0x100;
    uint32_t rdlvl_dq_deskew_en = 1;
    uint32_t rdlvl_vref_train_en = 1;
    uint32_t rdlvl_vref_train_step = 0x1;

    uint32_t db0_wrdq_dly = 0x380;
    uint32_t db1_wrdq_dly = 0x380;
    uint32_t wdqlvl_win_siz_thr = 0x40;
    uint32_t wdqlvl_fdly_step = 0x4;
    uint32_t wdqlvl_cdly_step = 0x8;
    uint32_t wdqlvl_bit_mask = 0x100;
    uint32_t wdqlvl_slv_dly_start = 0x200;
    uint32_t wdqlvl_left_abrt_thr = 0x500;
    uint32_t wdqlvl_te_jump_step = 0x40;
    uint32_t wdqlvl_bst_len = 3;

    uint32_t t_phy_wrlat = 0xb;
    uint32_t mr2_cwl_val = 2; // 0->9, 2->11

    uint32_t ie_dly = 2;//ddr_pll_freq_val >= 800 ? 2 : 0;
    uint32_t tsel_dly = 1;//ddr_pll_freq_val >= 800 ? 1 : 0;
    uint32_t rd_fifo_dly = 7;
    uint32_t wr_preamble = 0;
    uint32_t wr_postamble = 0;
    uint32_t rd_preamble = 0;
    uint32_t rd_postamble = 0;

    uint32_t dqs_oe_tmg = 0x61;
    uint32_t dq_oe_tmg = 0x62;
    uint32_t ie_tmg = 0xc0;
    uint32_t rd_tsel_tmg = 0x72;

    uint32_t dis_srx_zqcl = 0;
    uint32_t dis_auto_zq = 0;
    uint32_t dis_auto_ctrlupd_srx = 0;
    uint32_t dis_auto_ctrlupd = 0;
    uint32_t phyupd_en = 0;
    uint32_t phymstr_en = 0;
    uint32_t phymstr_intrvl = 0x12; // PHY intrvl cnt has bug, cannot be larger than 0x12
    uint32_t fc_retrain_en = 0;

    uint32_t init_pad_cal_en = 1;
    uint32_t intrvl_pad_cal_en = 0;
    uint32_t pvt_cal_intrvl = 0x12; // PHY intrvl cnt has bug, cannot be larger than 0x12
    uint32_t pad_cal_wait_cyc = 0x80;
    uint32_t pvt_code = 0x2020;

//#if 1 use this line for ctest simulation!!!
#if DDR_DBG_EN == 1
    uint32_t ddr_pll_freq;
    if (ddr_pll_freq_val > 1590)
    {
        ddr_pll_freq_val = 1600;
        ddr_pll_freq = PLL_BACKUP_MUL_1600MHZ;
    }
    else if (ddr_pll_freq_val > 1450)
    {
        ddr_pll_freq_val = 1504;
        ddr_pll_freq = PLL_BACKUP_MUL_1504MHZ;
    }
    else if (ddr_pll_freq_val > 1200)
    {
        ddr_pll_freq_val = 1408;
        ddr_pll_freq = PLL_BACKUP_MUL_1408MHZ;
    }
    else if (ddr_pll_freq_val > 1000)
    {
        ddr_pll_freq_val = 1200;
        ddr_pll_freq = PLL_BACKUP_MUL_1200MHZ;
    }
    else if (ddr_pll_freq_val > 800)
    {
        ddr_pll_freq_val = 1000;
        ddr_pll_freq = PLL_BACKUP_MUL_1000MHZ;
    }
    else if (ddr_pll_freq_val > 500)
    {
        ddr_pll_freq_val = 800;
        ddr_pll_freq = PLL_BACKUP_MUL_800MHZ;
    }
    else
    {
        ddr_pll_freq_val = 400;
        ddr_pll_freq = PLL_BACKUP_MUL_400MHZ;
    }

    uint32_t ddr_pll_sel = XDC_CLK_PLL_BK;

    //#if DDR_CPU_STREAM_TEST_EN == 0
    DisableDCache();
    //#endif

    ddr4_clk_init(ddr_pll_sel, ddr_pll_freq);
#endif

    ddr_top0_clk_mux_sel(DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL_BK);
    ddr_top0_clk_en(ENABLE);
    ddr_top0_set_rst(DISABLE);
    ddr_top0_set_rst(ENABLE);
    
    // Configure DDR_CTRL registers
    REG32(DDR_TOP0_BASE+XDC_MODE_OFFSET) =
                        XDC_MODE_XIF_BURST_LEN(0x4) |
                        XDC_MODE_RANK_NUM(cs_map) |
                        XDC_MODE_DEV_CFG(0x2) | // x16 device
                        XDC_MODE_BURST_LEN(0x4) |
                        XDC_MODE_GRD_EN_VAL(0) |
                        XDC_MODE_DLL_OFF_VAL(0) |
                        XDC_MODE_BC_EN_VAL(0) |
                        XDC_MODE_WORK_MODE(0x1);
    REG32(DDR_TOP0_BASE+XDC_INIT_TMG0_OFFSET) = XDC_INIT_TMG0_INIT_STATE(0x3);
    REG32(DDR_TOP0_BASE+XDC_ZQCAL_CTRL_OFFSET) =
                            XDC_ZQCAL_CTRL_DIS_SRX_ZQCL_VAL(dis_srx_zqcl) |
                            XDC_ZQCAL_CTRL_DIS_AUTO_ZQ_VAL(dis_auto_zq);
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL0_OFFSET) = 0x1c80;
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL2_OFFSET) = 0x3c05 |
                            XDC_DFI_CTRL2_DIS_AUTO_CTRLUPD_VAL(dis_auto_ctrlupd) |
                            XDC_DFI_CTRL2_DIS_AUTO_CTRLUPD_SRX_VAL(dis_auto_ctrlupd_srx);
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL3_OFFSET) =
                            XDC_DFI_CTRL3_DFI_T_CTRLUPD_INT_MIN(0x1) |  // 0x5
                            XDC_DFI_CTRL3_DFI_T_CTRLUPD_INT_MAX(0x4);   // 0x75
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL4_OFFSET) =
                            XDC_DFI_CTRL4_DFI_PHYMSTR_EN_VAL(phymstr_en) |
                            XDC_DFI_CTRL4_DFI_PHYUPD_EN_VAL(phyupd_en);

    REG32(DDR_TOP0_BASE+XDC_ADDRMAP0_OFFSET) = 0x339; //regmodel.xdc_top.ADDRMAP0
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP1_OFFSET) = 0x7d4a; //regmodel.xdc_top.ADDRMAP1
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP2_OFFSET) = 0x101; //regmodel.xdc_top.ADDRMAP2
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP3_OFFSET) = 0x1000; //regmodel.xdc_top.ADDRMAP3
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP4_OFFSET) = 0x1111; //regmodel.xdc_top.ADDRMAP4
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP5_OFFSET) = 0xff11; //regmodel.xdc_top.ADDRMAP5
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP6_OFFSET) = 0x8888; //regmodel.xdc_top.ADDRMAP6
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP7_OFFSET) = 0x8888; //regmodel.xdc_top.ADDRMAP7
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP8_OFFSET) = 0x8888; //regmodel.xdc_top.ADDRMAP8
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP9_OFFSET) = 0x8888; //regmodel.xdc_top.ADDRMAP9
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP10_OFFSET) = 0xf8; //regmodel.xdc_top.ADDRMAP10
    REG32(DDR_TOP0_BASE+XDC_REF_TMG0_FFC0_OFFSET) =
                            XDC_REF_TMG0_FFC0_REF_MARGIN_FFC0(0x2) |
                            XDC_REF_TMG0_FFC0_REF_GAP_FFC0(0x10) |
                            XDC_REF_TMG0_FFC0_REF_BURST_FFC0(4);
    // REG32(DDR_TOP0_BASE+XDC_REF_TMG1_FFC0_OFFSET) =
    //                         XDC_REF_TMG1_FFC0_REF_TIMER1_VAL_FFC0(0) |
    //                         XDC_REF_TMG1_FFC0_REF_TIMER0_VAL_FFC0(0);
    // if (ddr_pll_freq_val > 1333)
    // {
        // use the same timing paramerters as 1600M, except the t_refi_val_x32
        REG32(DDR_TOP0_BASE+XDC_REF_TMG3_FFC0_OFFSET) =
                        XDC_REF_TMG3_FFC0_T_REFI_X1_SEL_FFC0_VAL(1) |
                        XDC_REF_TMG3_FFC0_T_PBR2PBR_FFC0(0x80) |
                        XDC_REF_TMG3_FFC0_T_RFC_MIN_FFC0(0xdc) |
                        XDC_REF_TMG3_FFC0_T_REFI_VAL_FFC0(t_refi_val_x32);
                        // (0x600184dc & ~XDC_REF_TMG3_FFC0_T_REFI_VAL_FFC0(0xfff)) |
                        // XDC_REF_TMG3_FFC0_T_REFI_VAL_FFC0(t_refi_val_x32);
        REG32(DDR_TOP0_BASE+XDC_RANK_TMG0_FFC0_OFFSET) = 0x60062; //regmodel.xdc_top.RANK_TMG0_FFC0
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG0_FFC0_OFFSET) = 0x7146cd; //regmodel.xdc_top.DRAM_TMG0_FFC0
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG1_FFC0_OFFSET) = 0x6193; //regmodel.xdc_top.DRAM_TMG1_FFC0
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG2_FFC0_OFFSET) = 0x18618b; //regmodel.xdc_top.DRAM_TMG2_FFC0
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG4_FFC0_OFFSET) = 0x18c47; //regmodel.xdc_top.DRAM_TMG4_FFC0
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG5_FFC0_OFFSET) = 0x102062; //regmodel.xdc_top.DRAM_TMG5_FFC0
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG6_FFC0_OFFSET) = 0x810807; //regmodel.xdc_top.DRAM_TMG6_FFC0
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG7_FFC0_OFFSET) = 0x20c; //regmodel.xdc_top.DRAM_TMG7_FFC0
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG8_FFC0_OFFSET) = 0x889; //regmodel.xdc_top.DRAM_TMG8_FFC0
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG9_FFC0_OFFSET) = 0x1cc5; //regmodel.xdc_top.DRAM_TMG9_FFC0
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG10_FFC0_OFFSET) = 0x1732e; //regmodel.xdc_top.DRAM_TMG10_FFC0
        REG32(DDR_TOP0_BASE+XDC_ZQCAL_TMG0_FFC0_OFFSET) = 0x40040; //regmodel.xdc_top.ZQCAL_TMG0_FFC0
        REG32(DDR_TOP0_BASE+XDC_ZQCAL_TMG1_FFC0_OFFSET) = 0x80000; //regmodel.xdc_top.ZQCAL_TMG1_FFC0
        REG32(DDR_TOP0_BASE+XDC_DFI_TMG0_FFC0_OFFSET) = 0x30289040 | t_phy_wrlat; //regmodel.xdc_top.DFI_TMG0_FFC0
        REG32(DDR_TOP0_BASE+XDC_DFI_TMG1_FFC0_OFFSET) = 0x3484; //regmodel.xdc_top.DFI_TMG1_FFC0
        REG32(DDR_TOP0_BASE+XDC_DFI_TMG2_FFC0_OFFSET) = 0x1c5; //regmodel.xdc_top.DFI_TMG2_FFC0
        REG32(DDR_TOP0_BASE+XDC_DFI_TMG3_FFC0_OFFSET) = 0x1e; //regmodel.xdc_top.DFI_TMG3_FFC0
        REG32(DDR_TOP0_BASE+XDC_ODT_TMG_FFC0_OFFSET) = 0x180c1; //regmodel.xdc_top.ODT_TMG_FFC0
    // }
    // else
    // {
    // }
    REG32(DDR_TOP0_BASE+XDC_LP_CTRL_OFFSET) =
                        XDC_LP_CTRL_SPD_EN_VAL(0) |
                        XDC_LP_CTRL_SRF_GAP(0x40) |
                        XDC_LP_CTRL_PWD_GAP(0x10) |
                        XDC_LP_CTRL_SRF_TRIGGER_VAL(1) |
                        XDC_LP_CTRL_DRAM_CLK_DIS_VAL(0) |
                        XDC_LP_CTRL_PWD_EN_VAL(dev_pwd_en) |
                        XDC_LP_CTRL_SRF_EN_VAL(dev_srf_en);
    REG32(DDR_TOP0_BASE+XDC_REF_CTRL_OFFSET) =
                        XDC_REF_CTRL_PB_REF_EN_VAL(0) |
                        XDC_REF_CTRL_DERATE_EN_VAL(0) |
                        XDC_REF_CTRL_SRX_REF_NUM(0x1);
    REG32(DDR_TOP0_BASE+XDC_DBI_CTRL_OFFSET) =
                        XDC_DBI_CTRL_RD_DBI_EN_VAL(0) |
                        XDC_DBI_CTRL_WR_DBI_EN_VAL(0) |
                        XDC_DBI_CTRL_DM_EN_VAL(1);
    REG32(DDR_TOP0_BASE+XDC_SCH_CTRL_OFFSET) =
                        XDC_SCH_CTRL_PGC_TIME(0x20) |
                        XDC_SCH_CTRL_RDWR_GAP(0x10) |
                        XDC_SCH_CTRL_OPT_ACT_VAL(0) |
                        XDC_SCH_CTRL_PGC_EN_VAL(0) |
                        XDC_SCH_CTRL_PRF_WR_VAL(0);
    REG32(DDR_TOP0_BASE+XDC_RDCAM_OFFSET) =
                        XDC_RDCAM_RD_RUN_LEN(0x20) |
                        XDC_RDCAM_RD_MAX_STR(0x40);
    REG32(DDR_TOP0_BASE+XDC_WRCAM_OFFSET) =
                        XDC_WRCAM_WR_RUN_LEN(0x20) |
                        XDC_WRCAM_WR_MAX_STR(0x40);
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL5_OFFSET) =
                        XDC_DFI_CTRL5_DFI_FREQUENCY(0x0) |
                        XDC_DFI_CTRL5_DFI_INIT_START_VAL(0) |
                        XDC_DFI_CTRL5_DFI_DATA_CS_POLARITY_VAL(0) |
                        XDC_DFI_CTRL5_DFI_INIT_COMPLETE_EN_VAL(0);
    REG32(DDR_TOP0_BASE+XDC_DEBUG0_OFFSET) =
                        XDC_DEBUG0_RSV0_VAL(1) |
                        XDC_DEBUG0_REG_INIT_DONE_VAL(1);

    // configure DDR_PHY TOP and ACS4 registers
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_DDR_TYPE_OFFSET) =
                        TOP_GNRL_DDR_TYPE_DDR4_VAL(1);
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_HM_DIS_CFG_OFFSET) =
                        TOP_GNRL_HM_DIS_CFG_AC_DISABLE(0x0) |
                        TOP_GNRL_HM_DIS_CFG_DB_DISABLE(0x0);
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_MISC_CFG_OFFSET) =
                        TOP_GNRL_MISC_CFG_DS_LS_THR(8) |
                        TOP_GNRL_MISC_CFG_DATA_CS_POLRTY_INV_VAL(1);
    // power down unused acs4 PADs
    // REG32(DDR_TOP0_CFG_PHY_BASE+TOP_PAD_FUNC_AC_PD_CFG0_OFFSET) = 0xffc4c0c0;

    // TOP dq swap
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_PAD_SWAP_DB0_SWAP_CFG0_OFFSET) =
                        TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ0(4) |
                        TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ1(3) |
                        TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ2(2) |
                        TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ3(1) |
                        TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ4(6) |
                        TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ5(5) |
                        TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ6(0) |
                        TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ7(7) ;
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_PAD_SWAP_DB1_SWAP_CFG0_OFFSET) =
                        TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ0(7) |
                        TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ1(2) |
                        TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ2(3) |
                        TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ3(5) |
                        TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ4(4) |
                        TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ5(6) |
                        TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ6(1) |
                        TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ7(0) ;

    // DBYTE dq swap
    REG32(DDR_TOP0_CFG_PHY_BASE+0x40000+DBYTE_AF_DB_SWAP_CFG0_OFFSET) =
                        DBYTE_AF_DB_SWAP_CFG0_DQ0_SWAP(4) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ1_SWAP(3) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ2_SWAP(2) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ3_SWAP(1) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ4_SWAP(6) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ5_SWAP(5) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ6_SWAP(0) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ7_SWAP(7) ;
    REG32(DDR_TOP0_CFG_PHY_BASE+0x42000+DBYTE_AF_DB_SWAP_CFG0_OFFSET) =
                        DBYTE_AF_DB_SWAP_CFG0_DQ0_SWAP(7) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ1_SWAP(2) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ2_SWAP(3) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ3_SWAP(5) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ4_SWAP(4) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ5_SWAP(6) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ6_SWAP(1) |
                        DBYTE_AF_DB_SWAP_CFG0_DQ7_SWAP(0) ;

    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_PAD_CAL_CFG_OFFSET) =
                        TOP_GNRL_PAD_CAL_CFG_WAIT_CYC(pad_cal_wait_cyc) |
                        TOP_GNRL_PAD_CAL_CFG_INTRVL_CAL_EN_VAL(intrvl_pad_cal_en) |
                        TOP_GNRL_PAD_CAL_CFG_INIT_CAL_EN_VAL(init_pad_cal_en);
    if (init_pad_cal_en == 0)
    {
        REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_NXT_IO_PVT_CODE_OFFSET) = pvt_code;
        REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_CUR_IO_PVT_CODE_OFFSET) = pvt_code;
    }

    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_PAD_FUNC_SLEW_CFG_OFFSET) =
                        TOP_PAD_FUNC_SLEW_CFG_CLK_PAD_SLEW_N(slewn) |
                        TOP_PAD_FUNC_SLEW_CFG_CLK_PAD_SLEW_P(slewp) |
                        TOP_PAD_FUNC_SLEW_CFG_DB_PAD_SLEW_N(slewn) |
                        TOP_PAD_FUNC_SLEW_CFG_DB_PAD_SLEW_P(slewp) |
                        TOP_PAD_FUNC_SLEW_CFG_AC_PAD_SLEW_N(slewn) |
                        TOP_PAD_FUNC_SLEW_CFG_AC_PAD_SLEW_P(slewp);
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_PAD_FUNC_TSEL_CFG_OFFSET) =
                        TOP_PAD_FUNC_TSEL_CFG_CLK_TSEL_N(ac_enslicen) |
                        TOP_PAD_FUNC_TSEL_CFG_CLK_TSEL_P(ac_enslicep);
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_PAD_FUNC_MISC_PAD_CFG_OFFSET) =
                        (0x3f << TOP_PAD_FUNC_MISC_PAD_CFG_F0_DQS_GTLVL_BIAS_FS_EN_OFS) |  // BIAS_FS_EN
                        TOP_PAD_FUNC_MISC_PAD_CFG_AC_VREFSEL(ac_vrefsel) |
                        TOP_PAD_FUNC_MISC_PAD_CFG_DB_VREF_GEN_EN_VAL(1) |
                        TOP_PAD_FUNC_MISC_PAD_CFG_AC_VREF_GEN_EN_VAL(ac_vref_en) |
                        TOP_PAD_FUNC_MISC_PAD_CFG_PAD_DDR4_EN_VAL(1);
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_TOP_F0_TMG_OFFSET) =
                        TOP_GNRL_TOP_F0_TMG_PER_RANK_DIS_VAL(0) |
                        TOP_GNRL_TOP_F0_TMG_CAL_INTRVL(pvt_cal_intrvl) |
                        TOP_GNRL_TOP_F0_TMG_CAL_CLK_DIV_SEL(0x10) |
                        TOP_GNRL_TOP_F0_TMG_T_CBT_PAT_CHG(0xa) |
                        TOP_GNRL_TOP_F0_TMG_T_CKSRE(0x4);

    // acs4 configuration
    // acs4 bit_mask                                        
    REG32(DDR_TOP0_CFG_PHY_BASE+0x2a000) = 0x8; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[5].AC_GNRL_CFG

    for (uint32_t i=0; i<8; i++)
    {   
        // acs4 TSEL
        REG32(DDR_TOP0_CFG_PHY_BASE+0x2000*i+0x20010) = (ac_enslicen << 8) | (ac_enslicep << 4);
    }

    // dbyte configuration
    uint32_t db_idx;
    uint32_t db_addr_ofst;
    for (db_idx=0; db_idx<2; db_idx++) {
        db_addr_ofst = 0x40000+db_idx*0x2000;
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_AF_DB_GNRL_CFG_OFFSET) =
                        DBYTE_AF_DB_GNRL_CFG_PER_RANK_RDDQ_VAL(1) |
                        DBYTE_AF_DB_GNRL_CFG_PER_RANK_RDDQS_VAL(1) |
                        DBYTE_AF_DB_GNRL_CFG_PER_RANK_RDDQS_GT_VAL(1) |
                        DBYTE_AF_DB_GNRL_CFG_PER_RANK_WRDQ_VAL(1) |
                        DBYTE_AF_DB_GNRL_CFG_PER_RANK_WRDQS_VAL(1) |
                        DBYTE_AF_DB_GNRL_CFG_CS_MAP(cs_map) |
                        DBYTE_AF_DB_GNRL_CFG_DDR4_VAL(1) |
                        DBYTE_AF_DB_GNRL_CFG_BIT_MASK(0x0);
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_AF_DB_LVL_COM_CFG_OFFSET) =
                        DBYTE_AF_DB_LVL_COM_CFG_DBG_MODE_VAL(train_dbg_mode) |
                        DBYTE_AF_DB_LVL_COM_CFG_UPDT_WAIT_CYC(4);
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_AF_DB_WRLVL_CFG_OFFSET) =
                        DBYTE_AF_DB_WRLVL_CFG_ALL_DQ_VAL(wrlvl_all_dq_en) |
                        DBYTE_AF_DB_WRLVL_CFG_DLY_STEP(wrlvl_dly_step) |
                        DBYTE_AF_DB_WRLVL_CFG_CAPT_CNT(wrlvl_capt_cnt);
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_AF_DB_GTLVL_CFG_OFFSET) =
                        DBYTE_AF_DB_GTLVL_CFG_RESP_WAIT_CYC(0xf) |
                        DBYTE_AF_DB_GTLVL_CFG_BACK_STEP(gtlvl_back_step) |
                        DBYTE_AF_DB_GTLVL_CFG_DLY_STEP(gtlvl_dly_step) |
                        DBYTE_AF_DB_GTLVL_CFG_CAPT_CNT(gtlvl_capt_cnt); 
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_AF_DB_RDLVL_CFG0_OFFSET) =
                        DBYTE_AF_DB_RDLVL_CFG0_DQ_DESKEW_EN_VAL(rdlvl_dq_deskew_en) |
                        DBYTE_AF_DB_RDLVL_CFG0_RDLVL_BIT_MASK(rdlvl_bit_mask) |
                        DBYTE_AF_DB_RDLVL_CFG0_DLY_STEP(rdlvl_dly_step) |
                        DBYTE_AF_DB_RDLVL_CFG0_CAPT_CNT(rdlvl_capt_cnt);
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_AF_DB_RDLVL_VREF_CFG_OFFSET) =
                        DBYTE_AF_DB_RDLVL_VREF_CFG_MAX(0x9f) |
                        DBYTE_AF_DB_RDLVL_VREF_CFG_MIN(0x0) |
                        DBYTE_AF_DB_RDLVL_VREF_CFG_STEP(rdlvl_vref_train_step);
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_AF_DB_WDQLVL_CFG0_OFFSET) =
                        DBYTE_AF_DB_WDQLVL_CFG0_WIN_SIZ_THR(wdqlvl_win_siz_thr) |
                        DBYTE_AF_DB_WDQLVL_CFG0_FDLY_STEP(wdqlvl_fdly_step) |
                        DBYTE_AF_DB_WDQLVL_CFG0_CDLY_STEP(wdqlvl_cdly_step) |
                        DBYTE_AF_DB_WDQLVL_CFG0_WDQLVL_BIT_MASK(wdqlvl_bit_mask) |
                        DBYTE_AF_DB_WDQLVL_CFG0_WRDQ_TRAIN_EN_VAL(1);
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_AF_DB_WDQLVL_CFG1_OFFSET) =
                        DBYTE_AF_DB_WDQLVL_CFG1_TE_JUMP_STEP(wdqlvl_te_jump_step) |
                        DBYTE_AF_DB_WDQLVL_CFG1_LEFT_ABRT_THR(wdqlvl_left_abrt_thr) |
                        DBYTE_AF_DB_WDQLVL_CFG1_SLV_DLY_START(wdqlvl_slv_dly_start);
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_AF_DB_TSEL_CFG_OFFSET) =
                        DBYTE_AF_DB_TSEL_CFG_IDLE_TSEL_VAL((wr_enslicen << 4) | wr_enslicep) |
                        DBYTE_AF_DB_TSEL_CFG_RD_TSEL_VAL((rd_enslicen << 4) | rd_enslicep) |
                        DBYTE_AF_DB_TSEL_CFG_RD_TSEL_EN_VAL(rd_tsel_en);
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_DB_F0_TMG_OFFSET) =
                        DBYTE_FX_DB_F0_TMG_WRLVL_RESP_WAIT_CYC(0x20) |
                        DBYTE_FX_DB_F0_TMG_RD_POSTAMBLE_VAL(rd_postamble) |
                        DBYTE_FX_DB_F0_TMG_RD_PREAMBLE_VAL(rd_preamble) |
                        DBYTE_FX_DB_F0_TMG_WR_POSTAMBLE_VAL(wr_postamble) |
                        DBYTE_FX_DB_F0_TMG_WR_PREAMBLE_VAL(wr_preamble) |
                        DBYTE_FX_DB_F0_TMG_RD_FIFO_DLY(rd_fifo_dly) |
                        DBYTE_FX_DB_F0_TMG_TSEL_DLY(tsel_dly) |
                        DBYTE_FX_DB_F0_TMG_IE_DLY(ie_dly);
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_DB_F0_TMG1_OFFSET) =
                    DBYTE_FX_DB_F0_TMG1_RD_TSEL_TMG(rd_tsel_tmg) |
                    DBYTE_FX_DB_F0_TMG1_IE_TMG(ie_tmg) | 
                    DBYTE_FX_DB_F0_TMG1_OE_TMG(dq_oe_tmg);
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_BOOT_DB_BOOT_TMG1_OFFSET) =
                    DBYTE_BOOT_DB_BOOT_TMG1_RD_TSEL_TMG(0x50) |
                    DBYTE_BOOT_DB_BOOT_TMG1_IE_TMG(0xc0) | 
                    DBYTE_BOOT_DB_BOOT_TMG1_OE_TMG(dqs_oe_tmg); // ECO for tc_smic40
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_DB_F0_VREF_CFG_OFFSET) =
                    DBYTE_FX_DB_F0_VREF_CFG_STABLE_TIME(vref_stb_time) |
                    DBYTE_FX_DB_F0_VREF_CFG_EN_VAL(rdlvl_vref_train_en);
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_DB_F0_VREFSEL_CFG_OFFSET) = vrefsel;

        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_RX_DB_F0_R0_WRDQS_INIT_CFG_OFFSET) = wrlvl_eval;
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_START_CFG_OFFSET) = gtlvl_start;
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_CFG_OFFSET) = gtlvl_dly;

        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_RX_DB_F0_R1_WRDQS_INIT_CFG_OFFSET) = wrlvl_eval;
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_START_CFG_OFFSET) = gtlvl_start;
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_CFG_OFFSET) = gtlvl_dly;

        // for (uint32_t i=0; i<9; i++)
        // {
        //     REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_OFFSET+i*4) =
        //             DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_DQS_F_DLY(rddqs_f_dly) |
        //             DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_DQS_R_DLY(rddqs_r_dly) |
        //             DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_DQ_DLY_ENC(rddq_dly_enc);
        // }

        // for (uint32_t i=0; i<9; i++)
        // {
        //     if (db_idx == 0)
        //         REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_RX_DB_F0_R0_WRDQ0_CFG_OFFSET+i*4) = db0_wrdq_dly;
        //     else
        //         REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_RX_DB_F0_R0_WRDQ0_CFG_OFFSET+i*4) = db1_wrdq_dly;
        // }
    }

    // if (wrlvl_en == 0)
    // {
    //     if (ddr_pll_freq_val == 1600)
    //     {
    //         REG32(DDR_TOP0_CFG_PHY_BASE+0x40000+0*0x2000+DBYTE_FX_RX_DB_F0_R0_WRDQS_CFG_OFFSET) = 0x430;
    //         REG32(DDR_TOP0_CFG_PHY_BASE+0x40000+1*0x2000+DBYTE_FX_RX_DB_F0_R0_WRDQS_CFG_OFFSET) = 0x610;
    //     }
    // }

    // IDT configuration
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60000) = 0x1810100 | cs_map; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60004) = 0x1 | // poi_en
                                            (1 << 21) | // buf_mode_en
                                            (1 << 12) | // wrlvl_strb_len
                                            (wdqlvl_bst_len << 16) |
                                            (wdqlvl_en << 4) |
                                            (rdlvl_en << 3) |
                                            (gtlvl_en << 2) |
                                            (wrlvl_en << 1); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.TRAIN_CFG
    
    // config dq vref weight
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60010) = (60 << 26) | (4 << 20) | (dq_vref_step << 16);
    // Important!!!, phymstr_en must be configured to 0 before dfi init
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60030) = 0; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.RETRAIN_CFG
    // REG32(DDR_TOP0_CFG_PHY_BASE+0x60138) = phymstr_intrvl; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_RETRAIN_TMG
            
    // if (ddr_pll_freq_val > 1333)
    // {
		REG32(DDR_TOP0_CFG_PHY_BASE+0x60200) =
                                            0x314 |    // MR0
                                            (rtt_nom << 24) |
                                            (dev_drv << 17) |
                                            (dll_on << 16); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_MR_CFG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60204) =          // MR3 = 0
                                            0x0 | (mr2_cwl_val << 3) | //0x0010 |
                                            (rtt_wr << 9); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_MR_CFG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60208) =          // MR4 = 0
                                            0x0 |
                                            (rtt_park << 22); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_MR_CFG2
        REG32(DDR_TOP0_CFG_PHY_BASE+0x6020c) =
                                            0x0400 | // tCCD_L
                                            dq_vref_init_val; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_MR_CFG3
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60244) = (rtt_park << 8) | (rtt_wr << 4) | rtt_nom; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_D34_R1_MR_CFG

        REG32(DDR_TOP0_CFG_PHY_BASE+0x600e0) = 0x3010004f; //regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.AF_DDR_TMG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x600e4) = 0x100c4; //regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.AF_DDR_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60020) = (dq_vref_ed << 8) | (dq_vref_st); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_VREF_RANGE_CFG
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60100) = 0x6020c04; //regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60104) = 0x3020703; //regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60108) = 0x2000e00a; //regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG2
        REG32(DDR_TOP0_CFG_PHY_BASE+0x6010c) = 0x3c0f03c; //regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG3
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60114) = 0x5037061; //regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG5
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60120) = 0x00d4a; //regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60124) = 0x0105a; //regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60128) = 0x0305a; //regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG2
        REG32(DDR_TOP0_CFG_PHY_BASE+0x6012c) = 0x20090100 | t_phy_wrlat; //regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG3
    // }
    // else
    // {
    // }

    // DFI timing for wrlvl
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60120) = 0xf5070;//0xf084a; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG0
    // DFI timing for rdlvl
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60124) = 0xf5070;//0xf324a; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG1
    // DFI timing for wdqlvl
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60128) = 0xf5070; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG2

    // Start DFI INIT (including device init and training)
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL5_OFFSET) =
                        XDC_DFI_CTRL5_DFI_FREQUENCY(0x0) |
                        XDC_DFI_CTRL5_DFI_INIT_START_VAL(1) |
                        XDC_DFI_CTRL5_DFI_DATA_CS_POLARITY_VAL(0) |
                        XDC_DFI_CTRL5_DFI_INIT_COMPLETE_EN_VAL(0);

    // Polling regmodel.xdc_top.DFI_STATUS until bit 0 is 1.
    uint32_t phy_init_done= 0;
    uint32_t status;
    uint32_t train_timout_cnt = 0;

    while(phy_init_done == 0)
    {
#if DDR_DBG_EN == 1
        train_timout_cnt++;
        if (train_timout_cnt == 10)
        {
            printf("================================\n");
            printf("DDR PHY training timeout!!!\n");
            printf("================================\n");
            return;
        }
        
        delay_1ms(100);  
#endif
#if 0     
        parse_rdlvl_ctrl_obs();
        parse_mstr_obs(0x0);
        if (wrlvl_en)
        {
            parse_wrlvl_obs(0x0);
            parse_wrlvl_obs(0x1);
        }

        parse_gtlvl_obs(0x0);
        parse_gtlvl_obs(0x1);

        printf("db0_wrdqs_dly:%x\n", REG32(DDR_TOP0_CFG_PHY_BASE+0x40204));
        printf("db1_wrdqs_dly:%x\n", REG32(DDR_TOP0_CFG_PHY_BASE+0x42204));
        printf("db0_gt_dly:%x\n", REG32(DDR_TOP0_CFG_PHY_BASE+0x4020c));
        printf("db1_gt_dly:%x\n", REG32(DDR_TOP0_CFG_PHY_BASE+0x4220c));
        
        status = REG32(DDR_TOP0_CFG_PHY_BASE+0x60034); // IDT_LVL_STATUS (check training status, must be 0x0)
        printf("idt_lvl_status:%x (r0_wrlvl_err=%1x, r0_gtlvl_err=%1x, r0_rdlvl_err=%1x, r0_wdqlvl_err=%1x)\n",
                    status,
                    (status & 0x1),
                    (status & 0x2) >> 1,
                    (status & 0x4) >> 2,
                    (status & 0x8) >> 3);
        if (cs_map & 0x2)
        {
            printf("idt_lvl_status:%x (r1_wrlvl_err=%1x, r1_gtlvl_err=%1x, r1_rdlvl_err=%1x, r1_wdqlvl_err=%1x)\n",
                    status,
                    (status & 0x100) >> 8,
                    (status & 0x200) >> 9,
                    (status & 0x400) >> 10,
                    (status & 0x800) >> 11);
        }
        if (status !=0)
        {
            break;
        }
#endif
        phy_init_done = REG32(DDR_TOP0_BASE+0x88) & XDC_DFI_STATUS_DFI_INIT_COMPLETE;
    };


    if (check_train_status(cs_map) == 0)
    {
													   
        printf("================================\n");
        printf("DDR PHY training is succeeded!!!\n");
        printf("PLL freq:%dMHZ, active rank:%x\n", ddr_pll_freq_val, cs_map);
        printf("================================\n");
    }
    else
    {
													   
        printf("================================\n");
        printf("DDR PHY training is failed!!!!!!\n");
        printf("PLL freq:%dMHZ, active rank:%x\n", ddr_pll_freq_val, cs_map);
        printf("================================\n");
        return -1;
    }

#if DDR_DBG_EN == 1 // vref result
    printf("phy_init_vref=0x%x, rd_tsel_en=%x, pvt_code=0x%x\n", vrefsel, rd_tsel_en, REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_CUR_IO_PVT_CODE_OFFSET));
    if (rdlvl_en == 1 && rdlvl_vref_train_en == 1)
    {
        printf("trained db0_vref=0x%x, trained db1_vref=0x%x\n",
            REG32(DDR_TOP0_CFG_PHY_BASE+0x40000+DBYTE_FX_DB_F0_VREFSEL_CFG_OFFSET),
            REG32(DDR_TOP0_CFG_PHY_BASE+0x42000+DBYTE_FX_DB_F0_VREFSEL_CFG_OFFSET));
    }
    printf("rtt_wr=0x%x, dq_vref(read phy reg)=0x%x\n", rtt_wr, REG32(DDR_TOP0_CFG_PHY_BASE+0x6020c) & 0xff);
#endif
    
    // Configure DDRC to enter mission mode
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL5_OFFSET) =
                        XDC_DFI_CTRL5_DFI_FREQUENCY(0x0) |
                        XDC_DFI_CTRL5_DFI_INIT_START_VAL(0) |
                        XDC_DFI_CTRL5_DFI_DATA_CS_POLARITY_VAL(0) |
                        XDC_DFI_CTRL5_DFI_INIT_COMPLETE_EN_VAL(0);
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL5_OFFSET) =
                        XDC_DFI_CTRL5_DFI_FREQUENCY(0x0) |
                        XDC_DFI_CTRL5_DFI_INIT_START_VAL(0) |
                        XDC_DFI_CTRL5_DFI_DATA_CS_POLARITY_VAL(0) |
                        XDC_DFI_CTRL5_DFI_INIT_COMPLETE_EN_VAL(1);

    // exit software-triggered self-refresh
    REG32(DDR_TOP0_BASE+XDC_LP_CTRL_OFFSET) =
                        XDC_LP_CTRL_SPD_EN_VAL(0) |
                        XDC_LP_CTRL_SRF_GAP(0x40) |
                        XDC_LP_CTRL_PWD_GAP(0x10) |
                        XDC_LP_CTRL_SRF_TRIGGER_VAL(0) |
                        XDC_LP_CTRL_DRAM_CLK_DIS_VAL(0) |
                        XDC_LP_CTRL_PWD_EN_VAL(dev_pwd_en) |
                        XDC_LP_CTRL_SRF_EN_VAL(dev_srf_en);

    // Polling regmodel.xdc_top.STA until bit [2:0] is not 0x0.
    status = 0x0;
    while(status == 0){
        status = REG32(DDR_TOP0_BASE+XDC_STA_OFFSET) & XDC_STA_GFSM_MODE;
    };

    REG32(DDR_TOP0_BASE+XDC_DEBUG0_OFFSET) =
                        XDC_DEBUG0_RSV0_VAL(1) |
                        XDC_DEBUG0_REG_INIT_DONE_VAL(1) |
                        XDC_DEBUG0_DIS_DQ_VAL(1);
    ddr4_mrs(0x3, 0x5, 0x400); // Enable Data Mask
    REG32(DDR_TOP0_BASE+XDC_DEBUG0_OFFSET) =
                        XDC_DEBUG0_RSV0_VAL(1) |
                        XDC_DEBUG0_REG_INIT_DONE_VAL(1);

    return 0;
}