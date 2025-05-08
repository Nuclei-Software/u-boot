#include <linux/types.h>
#include "ddr_init.h"

#define  DDR_DBG_EN                         0
#define  DDR_PRINTF_EN                      0
#if DDR_PRINTF_EN == 1
#define  DDR_DBG_INFO(format, ...)          printf(format, ##__VA_ARGS__)
#define  DDR_DLY_1MS(format)                delay_1ms(format)
#else
#define  DDR_DBG_INFO(format, ...)
#define  DDR_DLY_1MS(format)
#endif

#define  REG32(addr)                        (*((volatile unsigned int*)(addr)))
#define  DDR_TOP0_BASE                      (0xf8b400000ULL)
#define  DDR_TOP0_CFG_PHY_BASE              (0xf8b500000ULL)

#define  DDR_TOP0_CLK_MUX_SEL_OSC_CLK_25M    (0UL)
#define  DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL    (1UL)
#define  DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL_BK (2UL)

extern uint32_t ddr_top0_ddr_dfs_req(void);
extern void ddr_top0_clk_mux_sel(uint32_t src_sel);
extern void ddr_top0_set_rst(ControlStatus Status);

enum clock_pll_e{
    SYS_CLK_PLL,
    SYS_CLK_PLL_BK,
    XDC_CLK_PLL,
    XDC_CLK_PLL_BK,
    XEC_CLK_PLL,
    XUC_CLK_PLL,
    SYSCTL_PLL_MAX
};

static uint32_t check_train_status(uint32_t cs_map)
{
    uint32_t status;
    uint32_t train_err = 0;

    status = REG32(DDR_TOP0_CFG_PHY_BASE+0x00074); // Read regmodel.ddr_phy_ral_top.ddr_phy_top.CBT_STATUS (check training status, must be 0x0)
    train_err = train_err | status;

    for (uint32_t i=0; i<2; i++)
    {
        status = REG32(DDR_TOP0_CFG_PHY_BASE+0x2000*i+0x40050); // Read regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_LVL_STATUS (check training status, must be 0x0)
        train_err = train_err | status;
    }
    
    status = REG32(DDR_TOP0_CFG_PHY_BASE+0x60034); // Read regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.IDT_LVL_STATUS (check training status, must be 0x0)
    train_err = train_err | status;

    return train_err;
}

int32_t ddr_init(void)
{
    uint32_t ddr_pll_freq_val = 1200;
    uint32_t cs_map = 0x3;
    uint32_t cbt_en = 1;
    uint32_t wrlvl_en = 1;
    uint32_t gtlvl_en = 1;
    uint32_t rdlvl_en = 1;
    uint32_t wdqlvl_en = 1;
    uint32_t chg_cs_dr_lvl = 1; // chg_cs_dr_lvl "0" is only for lpddr4

    uint32_t dev_srf_en = 1;
    uint32_t dev_pwd_en = 1;
    uint32_t dev_spd_en = 1;

    uint32_t ca_odt = ddr_pll_freq_val > 1400 ? 0x6 : 0x0; // no influence
    // uint32_t ca_vref_st = ca_odt !=0 ? 0x20 : 0x32;
    // uint32_t ca_vref_ed = ca_odt !=0 ? 0x20 : 0x32;
    // If not enable ca vref train and ca odt is 0, ca vref should be set to 0x72(max value).
    uint32_t ca_vref_st = ca_odt !=0 ? 0x00 : 0x40;
    uint32_t ca_vref_ed = ca_odt !=0 ? 0x32 : 0x72;
    uint32_t ca_vref_win_wght = 1;
    uint32_t ca_vref_best_wght = 63;
    uint32_t ca_vref_step= 0x4;
    uint32_t dq_odt = 0x0;  // 0 is better than non-0
    uint32_t dq_vref_st = dq_odt != 0 ? 0x00 : 0x40;
    uint32_t dq_vref_ed = dq_odt != 0 ? 0x32 : 0x72;
    uint32_t dq_vref_win_wght = 1;
    uint32_t dq_vref_best_wght = 63;
    uint32_t dq_vref_step= 0x4;
    uint32_t slewp = 0x0; // no influence
    uint32_t slewn = 0x7; // no influence
    uint32_t ac_vref_en = 0;
    uint32_t ac_vrefsel = 0x40;
    uint32_t ac_enslicep = 0xc;
    uint32_t ac_enslicen = 0xf;
    uint32_t wr_enslicep = 0xc; // 0xc is the best value
    uint32_t wr_enslicen = 0xf;
    uint32_t rd_enslicep = 0x0;
    uint32_t rd_enslicen[2];
    if (ddr_pll_freq_val > 1400)
    {
        rd_enslicen[0] = 0x1;//0x2;
        rd_enslicen[1] = 0x1;//0xc;
    }
    else
    {
        rd_enslicen[0] = 0x1;
        rd_enslicen[1] = 0x1;
    }
    uint32_t rd_tsel_en = 1;//ddr_pll_freq_val > 800 ? 1 : 0;
    uint32_t vrefsel = rd_tsel_en ? 0x2020 : 0x3030;
    uint32_t vref_stb_time = (uint32_t)(4000.0 / (1000.0 / ((float)ddr_pll_freq_val / 4.0))); // 4us
    uint32_t dev_pu_cal = 1; // 0->VDDQ/2.5, 1(default value)->VDDQ/3, no influence to 1600M stability???
    uint32_t dev_pdds = 0x6; // default value
    uint32_t soc_odt = 0; // 0(default value) is best value??? the trained dbyte vref is highest when soc_odt is set to 0.
    uint32_t dq_cal_pat = 0xa55a;
    uint32_t dq_cal_pat_inv = 0x55;
    uint32_t t_refi_val_x32 = (uint32_t)(3904.0 / (1000.0 / ((float)ddr_pll_freq_val / 4.0)) / 32.0); // 3.904us

    uint32_t cbt_pat_cnt = 4;
    uint32_t cbt_capt_cnt = 2;
    uint32_t cbt_fdly_step = 0x4;
    uint32_t cbt_cdly_step = 0xf;
    uint32_t cbt_margin = 0x10;
    uint32_t cbt_win_siz_thr = 0x10;

    uint32_t wrlvl_all_dq_en = 0;
    uint32_t wrlvl_capt_cnt = 2;
    uint32_t wrlvl_dly_step = 0x4;
    uint32_t wrlvl_eval = 0x000;

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
    uint32_t rdlvl_bit_mask = 0x000;
    uint32_t rdlvl_dq_deskew_en = 0; // 0 is better than 1
    uint32_t rdlvl_vref_train_en = 1;//ddr_pll_freq_val > 1400 ? 1 : 0; // 0 is better for 1400
    uint32_t rdlvl_vref_train_step = 0x1;
    
    uint32_t db0_wrdq_dly = 0x3b0;
    uint32_t db1_wrdq_dly = 0x3b0;
    uint32_t wdqlvl_win_siz_thr = 0x40;
    uint32_t wdqlvl_fdly_step = 0x4;
    uint32_t wdqlvl_cdly_step = 0xc;
    uint32_t wdqlvl_bit_mask = 0x000;
    uint32_t wdqlvl_slv_dly_start = 0x200;
    uint32_t wdqlvl_left_abrt_thr = 0x500;
    uint32_t wdqlvl_te_jump_step = 0x40;
    uint32_t wdqlvl_bst_len = 3;

    uint32_t ie_dly = ddr_pll_freq_val > 1000 ? 1 : 0;
    uint32_t tsel_dly = ddr_pll_freq_val > 1000 ? 1 : 0;
    uint32_t rd_fifo_dly = 7;
    uint32_t wr_postamble = ddr_pll_freq_val > 1400 ? 1 : 0; // no influence
    uint32_t rd_preamble_tgl = ddr_pll_freq_val > 1400 ? 1 : 0;
    uint32_t rd_postamble = ddr_pll_freq_val > 1400 ? 1 : 0;

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
    uint32_t intrvl_pad_cal_en = 1;
    uint32_t pvt_cal_intrvl = 0x12; // PHY intrvl cnt has bug, cannot be larger than 0x12
    uint32_t pad_cal_wait_cyc = 0x80;
    uint32_t pvt_code = 0x2020; // best pvt code


    ddr_top0_clk_en(ENABLE);
    ddr_top0_set_rst(DISABLE);
    ddr_top0_set_rst(ENABLE);


    // Configure DDR_CTRL registers
    REG32(DDR_TOP0_BASE+XDC_MODE_OFFSET) =
                        XDC_MODE_XIF_BURST_LEN(0x8) |
                        XDC_MODE_RANK_NUM(cs_map) |
                        XDC_MODE_DEV_CFG(0x2) | // x16 device
                        XDC_MODE_BURST_LEN(0x8) |
                        XDC_MODE_WORK_MODE(0x3);
    REG32(DDR_TOP0_BASE+XDC_INIT_TMG0_OFFSET) = XDC_INIT_TMG0_INIT_STATE(0x3);
    REG32(DDR_TOP0_BASE+XDC_ZQCAL_CTRL_OFFSET) =
                            XDC_ZQCAL_CTRL_DIS_SRX_ZQCL_VAL(dis_srx_zqcl) |
                            XDC_ZQCAL_CTRL_DIS_AUTO_ZQ_VAL(dis_auto_zq);
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL0_OFFSET) = 0x1c9e;
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL2_OFFSET) = 0x3c05 |
                            XDC_DFI_CTRL2_DIS_AUTO_CTRLUPD_VAL(dis_auto_ctrlupd) |
                            XDC_DFI_CTRL2_DIS_AUTO_CTRLUPD_SRX_VAL(dis_auto_ctrlupd_srx);
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL3_OFFSET) =
                            XDC_DFI_CTRL3_DFI_T_CTRLUPD_INT_MIN(0x1) |  // 0x5
                            XDC_DFI_CTRL3_DFI_T_CTRLUPD_INT_MAX(0x4);   // 0x75
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL4_OFFSET) =
                            XDC_DFI_CTRL4_DFI_PHYMSTR_EN_VAL(phymstr_en) |
                            XDC_DFI_CTRL4_DFI_PHYUPD_EN_VAL(phyupd_en);
#if 0   // No rotation
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP0_OFFSET) = 0x02f7;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP1_OFFSET) = 0x6318;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP2_OFFSET) = 0x03ff;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP5_OFFSET) = 0xff00;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP6_OFFSET) = 0x4444;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP7_OFFSET) = 0x4444;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP8_OFFSET) = 0x4444;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP9_OFFSET) = 0x4444;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP10_OFFSET) = 0xff;
#else   // Bank[1:0] rotation
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP0_OFFSET) = 0x02f7;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP1_OFFSET) = 0x2042;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP2_OFFSET) = 0x03ff;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP4_OFFSET) = 0x2222;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP5_OFFSET) = 0xff22;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP6_OFFSET) = 0x7777;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP7_OFFSET) = 0x7777;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP8_OFFSET) = 0x7777;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP9_OFFSET) = 0x7777;
    REG32(DDR_TOP0_BASE+XDC_ADDRMAP10_OFFSET) = 0xff;
#endif
    REG32(DDR_TOP0_BASE+XDC_REF_TMG0_FFC0_OFFSET) =
                            XDC_REF_TMG0_FFC0_REF_MARGIN_FFC0(0x2) |
                            XDC_REF_TMG0_FFC0_REF_GAP_FFC0(0x10) |
                            XDC_REF_TMG0_FFC0_REF_BURST_FFC0(4);
    REG32(DDR_TOP0_BASE+XDC_REF_TMG1_FFC0_OFFSET) =
                            XDC_REF_TMG1_FFC0_REF_TIMER1_VAL_FFC0(0) |
                            XDC_REF_TMG1_FFC0_REF_TIMER0_VAL_FFC0(0);

    if (ddr_pll_freq_val > 1066)
    {
        // use the same timing paramerters as 1600M, except the t_refi_val_x32
        REG32(DDR_TOP0_BASE+XDC_REF_TMG3_FFC0_OFFSET) = 0x20000070 |
                        XDC_REF_TMG3_FFC0_T_REFI_VAL_FFC0(t_refi_val_x32);
        REG32(DDR_TOP0_BASE+XDC_RANK_TMG0_FFC0_OFFSET) = 0x50082;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG0_FFC0_OFFSET) = 0x8206d1;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG1_FFC0_OFFSET) = 0x6218;
        if (dq_odt != 0)
            REG32(DDR_TOP0_BASE+XDC_DRAM_TMG2_FFC0_OFFSET) = 0x1c43cd;
        else
            REG32(DDR_TOP0_BASE+XDC_DRAM_TMG2_FFC0_OFFSET) = 0x1c430d;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG3_FFC0_OFFSET) = 0x1806;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG4_FFC0_OFFSET) = 0x2108a;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG5_FFC0_OFFSET) = 0x18c3;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG11_FFC0_OFFSET) = 0x20040012;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG12_FFC0_OFFSET) = 0x2850;
        REG32(DDR_TOP0_BASE+XDC_DERATE_TMG_FFC0_OFFSET) = 0xc350021;
        REG32(DDR_TOP0_BASE+XDC_ZQCAL_TMG0_FFC0_OFFSET) =
                        XDC_ZQCAL_TMG0_FFC0_T_ZQOPER_FFC0(0x190) |
                        XDC_ZQCAL_TMG0_FFC0_T_ZQCS_FFC0(0xc);
        REG32(DDR_TOP0_BASE+XDC_ZQCAL_TMG1_FFC0_OFFSET) = 0x80000;
        REG32(DDR_TOP0_BASE+XDC_DFI_TMG0_FFC0_OFFSET) = 0x3028a048;
        REG32(DDR_TOP0_BASE+XDC_DFI_TMG2_FFC0_OFFSET) = 0x202;
        REG32(DDR_TOP0_BASE+XDC_DFI_TMG3_FFC0_OFFSET) = 0x1e;
    }
    else
    {
        // use the same timing paramerters as 1000M, except the t_refi_val_x32
        REG32(DDR_TOP0_BASE+XDC_REF_TMG3_FFC0_OFFSET) = 0x20000046 |
                        XDC_REF_TMG3_FFC0_T_REFI_VAL_FFC0(t_refi_val_x32);
        REG32(DDR_TOP0_BASE+XDC_RANK_TMG0_FFC0_OFFSET) = 0x40082;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG0_FFC0_OFFSET) = 0x61444b;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG1_FFC0_OFFSET) = 0x620f;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG2_FFC0_OFFSET) = 0x14328c;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG3_FFC0_OFFSET) = 0x1004;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG4_FFC0_OFFSET) = 0x15067;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG5_FFC0_OFFSET) = 0x1882;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG11_FFC0_OFFSET) = 0x2004000e;
        REG32(DDR_TOP0_BASE+XDC_DRAM_TMG12_FFC0_OFFSET) = 0x1932;
        REG32(DDR_TOP0_BASE+XDC_DERATE_TMG_FFC0_OFFSET) = 0xc350011;
        REG32(DDR_TOP0_BASE+XDC_ZQCAL_TMG0_FFC0_OFFSET) = 0x3e808;
        REG32(DDR_TOP0_BASE+XDC_ZQCAL_TMG1_FFC0_OFFSET) = 0x80000;
        REG32(DDR_TOP0_BASE+XDC_DFI_TMG0_FFC0_OFFSET) = 0x30288046; // TODO: t_rddata_en does not match ie_dly
        REG32(DDR_TOP0_BASE+XDC_DFI_TMG2_FFC0_OFFSET) = 0x100;
        REG32(DDR_TOP0_BASE+XDC_DFI_TMG3_FFC0_OFFSET) = 0x1e;
    }
    REG32(DDR_TOP0_BASE+XDC_LP_CTRL_OFFSET) =
                        XDC_LP_CTRL_SRF_GAP(0x40) |
                        XDC_LP_CTRL_PWD_GAP(0x10) |
                        XDC_LP_CTRL_SRF_TRIGGER_VAL(1) |
                        XDC_LP_CTRL_DRAM_CLK_DIS_VAL(0) |
                        XDC_LP_CTRL_SPD_EN_VAL(0) | // SPD_EN must be 0 befor issuing LPDDR4 MRR
                        XDC_LP_CTRL_PWD_EN_VAL(dev_pwd_en) |
                        XDC_LP_CTRL_SRF_EN_VAL(dev_srf_en);
    REG32(DDR_TOP0_BASE+XDC_REF_CTRL_OFFSET) =
                        XDC_REF_CTRL_PB_REF_EN_VAL(0) |
                        XDC_REF_CTRL_DERATE_EN_VAL(0) |
                        XDC_REF_CTRL_SRX_REF_NUM(0x2);
    REG32(DDR_TOP0_BASE+XDC_DBI_CTRL_OFFSET) =
                        XDC_DBI_CTRL_RD_DBI_EN_VAL(0) |
                        XDC_DBI_CTRL_WR_DBI_EN_VAL(0) |
                        XDC_DBI_CTRL_DM_EN_VAL(1);
    REG32(DDR_TOP0_BASE+XDC_SCH_CTRL_OFFSET) =
                        XDC_SCH_CTRL_PGC_TIME(0x20) |
                        XDC_SCH_CTRL_RDWR_GAP(0x10) |
                        XDC_SCH_CTRL_OPT_ACT_VAL(1) |
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
                        XDC_DFI_CTRL5_DFI_DATA_CS_POLARITY_VAL(1) |
                        XDC_DFI_CTRL5_DFI_INIT_COMPLETE_EN_VAL(0);
    REG32(DDR_TOP0_BASE+XDC_DEBUG0_OFFSET) =
                        XDC_DEBUG0_RSV0_VAL(1) |
                        XDC_DEBUG0_REG_INIT_DONE_VAL(1);

    // configure DDR_PHY registers
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_DDR_TYPE_OFFSET) =
                        TOP_GNRL_DDR_TYPE_LP4_VAL(1);
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_HM_DIS_CFG_OFFSET) =
                        TOP_GNRL_HM_DIS_CFG_AC_DISABLE(0xc0) |
                        TOP_GNRL_HM_DIS_CFG_DB_DISABLE(0x0);
    // power down unused acs4 PADs
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_PAD_FUNC_AC_PD_CFG0_OFFSET) = 0xffc4c0c0;

    // acs4 swap
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_OFFSET) = 0x090a0c08;
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_PAD_SWAP_LP34_AC_SWAP_CFG3_OFFSET) = 0x0d0b;
    // cbt cfg
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_CBT_CFG0_OFFSET) =
                                    TOP_GNRL_CBT_CFG0_UPDT_WAIT_CYC(0x4) |
                                    TOP_GNRL_CBT_CFG0_FDLY_STEP(cbt_fdly_step) |
                                    TOP_GNRL_CBT_CFG0_CDLY_STEP(cbt_cdly_step) |
                                    TOP_GNRL_CBT_CFG0_CAPT_CNT(cbt_capt_cnt) |
                                    TOP_GNRL_CBT_CFG0_PAT_CNT(cbt_pat_cnt);
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_CBT_CFG3_OFFSET) =
                                    TOP_GNRL_CBT_CFG3_CA_WIN_SIZ_THR(cbt_win_siz_thr) |
                                    TOP_GNRL_CBT_CFG3_CS_WIN_SIZ_THR(cbt_win_siz_thr) |
                                    TOP_GNRL_CBT_CFG3_CA_MARGIN(cbt_margin) |
                                    TOP_GNRL_CBT_CFG3_CS_MARGIN(cbt_margin);


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
                                    TOP_PAD_FUNC_MISC_PAD_CFG_PAD_LPDDR4_EN_VAL(1);
    REG32(DDR_TOP0_CFG_PHY_BASE+TOP_GNRL_TOP_F0_TMG_OFFSET) =
                                    TOP_GNRL_TOP_F0_TMG_PER_RANK_DIS_VAL(0) |
                                    TOP_GNRL_TOP_F0_TMG_CAL_INTRVL(pvt_cal_intrvl) |
                                    TOP_GNRL_TOP_F0_TMG_CAL_CLK_DIV_SEL(0x1) |
                                    TOP_GNRL_TOP_F0_TMG_T_CBT_PAT_CHG(0xa) |
                                    TOP_GNRL_TOP_F0_TMG_T_CKSRE(0x4);

    // acs4 configuration
    // acs4 bit_mask
    REG32(DDR_TOP0_CFG_PHY_BASE+0x22000) = 0xc; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[1].AC_GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x26000) = 0xc; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[3].AC_GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x28000) = 0x4; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[4].AC_GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x2a000) = 0xc; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[5].AC_GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x2c000) = 0xf; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[6].AC_GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x2e000) = 0xf; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[7].AC_GNRL_CFG

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
                        DBYTE_AF_DB_GNRL_CFG_LP4_VAL(1) |
                        DBYTE_AF_DB_GNRL_CFG_BIT_MASK(0x0);
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
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_AF_DB_RDLVL_CFG1_OFFSET) =
                        DBYTE_AF_DB_RDLVL_CFG1_PAT(dq_cal_pat) |
                        DBYTE_AF_DB_RDLVL_CFG1_PAT_INV(dq_cal_pat_inv);
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
                        DBYTE_AF_DB_TSEL_CFG_RD_TSEL_VAL((rd_enslicen[db_idx] << 4) | rd_enslicep) |
                        DBYTE_AF_DB_TSEL_CFG_RD_TSEL_EN_VAL(rd_tsel_en);

        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+DBYTE_FX_DB_F0_TMG_OFFSET) =
                        DBYTE_FX_DB_F0_TMG_WRLVL_RESP_WAIT_CYC(0x20) |
                        DBYTE_FX_DB_F0_TMG_RD_POSTAMBLE_VAL(rd_postamble) |
                        DBYTE_FX_DB_F0_TMG_RD_PREAMBLE_VAL(1) |
                        DBYTE_FX_DB_F0_TMG_WR_POSTAMBLE_VAL(wr_postamble) |
                        DBYTE_FX_DB_F0_TMG_WR_PREAMBLE_VAL(1) |
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

    // IDT configuration
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60000) = 0x00810100 |
                                            (chg_cs_dr_lvl << 24) |
                                             cs_map; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60004) = 0x11201001 |
                                            (wdqlvl_bst_len << 16) |
                                            (cbt_en << 5) |
                                            (rdlvl_en << 3) |
                                            (wdqlvl_en << 4) |
                                            (gtlvl_en << 2) |
                                            (wrlvl_en << 1); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.TRAIN_CFG
    // config dq vref step and weight
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60010) = (dq_vref_win_wght << 26) | (dq_vref_best_wght << 20) | (dq_vref_step << 16);
    // config ca vref step and weight
    REG32(DDR_TOP0_CFG_PHY_BASE+0x6001c) = (ca_vref_win_wght << 26) | (ca_vref_best_wght << 20) | (ca_vref_step << 16);
    // Important!!!, phymstr_en must be configured to 0 before dfi init
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60030) = 0xc | (fc_retrain_en << 1); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.RETRAIN_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60138) = phymstr_intrvl; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_RETRAIN_TMG
    
    // device odt
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60210) = (soc_odt << 24) | (ca_odt << 4) | dq_odt; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_LP34_R0_MR_CFG0
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60220) = (soc_odt << 24) | (ca_odt << 4) | dq_odt; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_LP34_R1_MR_CFG0
            
    if (ddr_pll_freq_val > 1066)
    {
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60200) = 0x240000 |
                                               (rd_preamble_tgl << 19) |
                                               (rd_postamble << 23); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_MR_CFG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60204) = 0x12 | 
                                               (dev_pu_cal << 16) |
                                               (wr_postamble << 17) |
                                               (dev_pdds << 19); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_MR_CFG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603a4) = (dq_cal_pat << 16) | (dq_cal_pat_inv << 8) | dq_cal_pat_inv; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.AF_MR_CFG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603c0) = 0x4050505; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_DDR_TMG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603c4) = 0x4040203; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_DDR_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603c8) = 0x4; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_DDR_TMG2
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603e0) = 0x84; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_MR_CFG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603e4) = 0x10; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_MR_CFG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x600e0) = 0x4010003; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.AF_DDR_TMG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x600e4) = 0x10019; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.AF_DDR_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60020) = (ca_vref_ed << 24) | (ca_vref_st << 16) | (dq_vref_ed << 8) | (dq_vref_st); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_VREF_RANGE_CFG
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60100) = 0x8040604; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60104) = 0x4040904; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60108) = 0x20007304; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG2
        REG32(DDR_TOP0_CFG_PHY_BASE+0x6010c) = 0x640b455; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG3
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60110) = 0x9100d; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG4
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60114) = 0x501c030; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG5
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60118) = 0x6419064; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG6
        REG32(DDR_TOP0_CFG_PHY_BASE+0x6011c) = 0x10064; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG7
        REG32(DDR_TOP0_CFG_PHY_BASE+0x6012c) = 0x200a0108; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG3
    }
    else
    {
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60200) = 0x140000 |
                                               (rd_preamble_tgl << 19) |
                                               (rd_postamble << 23); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_MR_CFG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60204) = 0x9 |
                                               (dev_pu_cal << 16) |
                                               (wr_postamble << 17) |
                                               (dev_pdds << 19); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_MR_CFG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603a4) = (dq_cal_pat << 16) | (dq_cal_pat_inv << 8) | dq_cal_pat_inv; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.AF_MR_CFG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603c0) = 0x4050505; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_DDR_TMG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603c4) = 0x4040203; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_DDR_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603c8) = 0x4; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_DDR_TMG2
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603e0) = 0x84; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_MR_CFG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603e4) = 0x10; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_MR_CFG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x600e0) = 0x4010003; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.AF_DDR_TMG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x600e4) = 0x10019; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.AF_DDR_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60020) = (ca_vref_ed << 24) | (ca_vref_st << 16) | (dq_vref_ed << 8) | (dq_vref_st); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_VREF_RANGE_CFG
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60100) = 0x5030403; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60104) = 0x4040604; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60108) = 0x20004803; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG2
        REG32(DDR_TOP0_CFG_PHY_BASE+0x6010c) = 0x3f07837; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG3
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60110) = 0x70c0c; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG4
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60114) = 0x501181e; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG5
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60118) = 0x3f0fc3f; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG6
        REG32(DDR_TOP0_CFG_PHY_BASE+0x6011c) = 0x1003f; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG7
        // TODO: t_rddata_en does not match ie_dly
        REG32(DDR_TOP0_CFG_PHY_BASE+0x6012c) = 0x20080106; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG3
    }

    // DFI timing for wrlvl
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60120) = 0xf5070;//0xf084a; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG0
    // DFI timing for rdlvl
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60124) = 0xf5070;//0xf324a; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG1
    // DFI timing for wdqlvl
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60128) = 0xf5070; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG2
    // DFI timing for calvl
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60130) = 0x71030; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG4
    // DFI timing for calvl
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60134) = 0xf2070; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DFI_TMG5

    // Start DFI INIT (including device init and training)
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL5_OFFSET) =
                        XDC_DFI_CTRL5_DFI_FREQUENCY(0x0) |
                        XDC_DFI_CTRL5_DFI_INIT_START_VAL(1) |
                        XDC_DFI_CTRL5_DFI_DATA_CS_POLARITY_VAL(1) |
                        XDC_DFI_CTRL5_DFI_INIT_COMPLETE_EN_VAL(0);

    // Polling regmodel.xdc_top.DFI_STATUS until bit 0 is 1.
    uint32_t phy_init_done= 0;
    uint32_t ddr_dfs_req = 0;
    uint32_t ddr_dfs_freq = 0;
    uint32_t train_timout_cnt = 0;
    while(phy_init_done == 0)
    {
        ddr_dfs_req = ddr_top0_ddr_dfs_req();
        ddr_dfs_freq = ddr_top0_ddr_dfs_freq();
        // DDR_DBG_INFO("dfs_req=%x, dfs_freq=%x\n", ddr_dfs_req, ddr_dfs_freq);
        if (ddr_dfs_req != 0) 
        {
            // LPDDR4 frequency change handshake
            if (ddr_dfs_freq == 0)
            {
                DDR_DBG_INFO("change ddr clk to OSC\n");
                ddr_top0_clk_mux_sel(DDR_TOP0_CLK_MUX_SEL_OSC_CLK_25M);
                ddr_top0_ddr_dfs_ack_pulse();
            }
            else if (ddr_dfs_freq == 1)
            {
                DDR_DBG_INFO("change ddr clk to XDC_CLK_PLL_BK\n");
                ddr_top0_clk_mux_sel(DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL_BK);
                ddr_top0_ddr_dfs_ack_pulse();
            }
            else if (ddr_dfs_freq == 2)
            {
                DDR_DBG_INFO("change ddr clk to XDC_CLK_PLL\n");
                ddr_top0_clk_mux_sel(DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL);
                ddr_top0_ddr_dfs_ack_pulse();               
            }
        }

        train_timout_cnt++;
        if (train_timout_cnt == 100000)
        {
            printf("================================\n");
            printf("DDR PHY training timeout!!!\n");
            printf("================================\n");
            return 1;
        }

        //DDR_DLY_1MS(100);
        for(int i=0; i<100000; i++);

        phy_init_done = REG32(DDR_TOP0_BASE+XDC_DFI_STATUS_OFFSET) & XDC_DFI_STATUS_DFI_INIT_COMPLETE;
    };

    if (check_train_status(cs_map) == 0)
    {
        printf("============================\n");
        printf("PHY training is succeeded!!!\n");
        printf("============================\n");
    }
    else
    {
        printf("=========================\n");
        printf("PHY training is failed!!!\n");
        printf("=========================\n");
        return 2;
    }

    // Configure DDRC to enter mission mode
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL5_OFFSET) =
                        XDC_DFI_CTRL5_DFI_FREQUENCY(0x0) |
                        XDC_DFI_CTRL5_DFI_INIT_START_VAL(0) |
                        XDC_DFI_CTRL5_DFI_DATA_CS_POLARITY_VAL(1) |
                        XDC_DFI_CTRL5_DFI_INIT_COMPLETE_EN_VAL(0);
    REG32(DDR_TOP0_BASE+XDC_DFI_CTRL5_OFFSET) =
                        XDC_DFI_CTRL5_DFI_FREQUENCY(0x0) |
                        XDC_DFI_CTRL5_DFI_INIT_START_VAL(0) |
                        XDC_DFI_CTRL5_DFI_DATA_CS_POLARITY_VAL(1) |
                        XDC_DFI_CTRL5_DFI_INIT_COMPLETE_EN_VAL(1);
    // exit software-triggered self-refresh
    REG32(DDR_TOP0_BASE+XDC_LP_CTRL_OFFSET) =
                        XDC_LP_CTRL_SRF_GAP(0x40) |
                        XDC_LP_CTRL_PWD_GAP(0x10) |
                        XDC_LP_CTRL_SRF_TRIGGER_VAL(0) |
                        XDC_LP_CTRL_DRAM_CLK_DIS_VAL(0) |
                        XDC_LP_CTRL_SPD_EN_VAL(dev_spd_en) |
                        XDC_LP_CTRL_PWD_EN_VAL(dev_pwd_en) |
                        XDC_LP_CTRL_SRF_EN_VAL(dev_srf_en);
    // Important!!! phymstr_en must be configured after de-asserting dfi_init_start 
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60030) = 0xc | (fc_retrain_en << 1) | phymstr_en; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.RETRAIN_CFG

    // Polling regmodel.xdc_top.STA until bit [2:0] is not 0x0.
    uint32_t gfsm_mode = 0x0;
    while(gfsm_mode == 0){
        gfsm_mode = REG32(DDR_TOP0_BASE+XDC_STA_OFFSET) & XDC_STA_GFSM_MODE;
    };

    return 0;
}
