#include <linux/types.h>

#define  DDR_DBG_EN                         0

#define  REG32(addr)                        (*((volatile unsigned int*)(addr)))
#define  DDR_TOP0_BASE                      (0xf8b400000ULL)
#define  DDR_TOP0_CFG_PHY_BASE              (0xf8b500000ULL)

#define  DDR_TOP0_CLK_MUX_SEL_OSC_CLK_25M    (0UL)
#define  DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL    (1UL)
#define  DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL_BK (2UL)
#define  XDC_DFI_STATUS_DFI_INIT_COMPLETE    (1UL << 0)
#define  XDC_STA_GFSM_MODE                    7  

typedef enum {
    DISABLE = 0,
    ENABLE = !DISABLE
} ControlStatus;

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
    uint32_t ddr_pll_freq;
    uint32_t ddr_pll_freq_val = 1200;

    uint32_t lpbk_test = 0;
    uint32_t ac_vref_en = 0;
    uint32_t ac_vrefsel = 0x40;
    uint32_t cs_map = 0x3;
    uint32_t init_pad_cal_en = 1;
    uint32_t pvt_code = 0x2020;
    uint32_t cbt_en = 1;
    uint32_t gtlvl_en = 1;
    uint32_t rdlvl_en = 1;
    uint32_t wdqlvl_en = 1;

    uint32_t ca_odt = 0x0;
    uint32_t ca_vref_st = 0x32;
    uint32_t ca_vref_ed = 0x32;
    uint32_t dq_odt = 0x0;  // 0 is better than non-0
    uint32_t dq_vref_st = dq_odt != 0 ? 0x0 : 0x40;
    uint32_t dq_vref_ed = dq_odt != 0 ? 0x32 : 0x72;
    uint32_t dq_vref_step= 0x4;
    uint32_t slewp = 0x4; // no influence
    uint32_t slewn = 0x4; // no influence
    uint32_t ac_enslicep = 0xc;
    uint32_t ac_enslicen = 0xf;
    uint32_t wr_enslicep = 0xc; // 0xb or 0xc are the best value
    uint32_t wr_enslicen = 0xf;
    uint32_t rd_enslicep = 0x0;
    uint32_t rd_enslicen = 0x1;
    uint32_t rd_tsel_en = 1;
    uint32_t vrefsel = rd_tsel_en ? 0x2020 : 0x3030;
    uint32_t dev_pu_cal = 0;
    uint32_t dev_pdds = 0x6;
    uint32_t soc_odt = 0x2; // if set to 1, 1200M will be failed. 2 is best for 1504M.

    uint32_t cbt_pat_cnt = 1;
    uint32_t cbt_capt_cnt = 1;
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
    uint32_t rdlvl_capt_cnt = 4;
    uint32_t rdlvl_dly_step = 0x8;
    uint32_t rdlvl_bit_mask = 0x000;
    uint32_t rdlvl_dq_deskew_en = 0; // 0 is better than 1
    uint32_t rdlvl_vref_train_en = 0;
    uint32_t rdlvl_vref_train_step = 0x4;
    uint32_t vref_stb_time = 4000 / (1000 / (ddr_pll_freq_val / 4));
    
    uint32_t db0_wrdq_dly = 0x3b0;
    uint32_t db1_wrdq_dly = 0x3b0;
    uint32_t wdqlvl_win_siz_thr = 0x40;
    uint32_t wdqlvl_fdly_step = 0x8;
    uint32_t wdqlvl_cdly_step = 0xf;
    uint32_t wdqlvl_bit_mask = 0x000;
    uint32_t wdqlvl_slv_dly_start = 0x400;
    uint32_t wdqlvl_left_abrt_thr = 0x500;
    uint32_t wdqlvl_te_jump_step = 0x40;
    uint32_t wdqlvl_bst_len = 3;

    uint32_t ie_dly = 2;
    uint32_t tsel_dly = 1;
    uint32_t rd_fifo_dly = 7;
    uint32_t wr_postamble = 0;
    uint32_t rd_preamble_tgl = 1; // no influence to 1504M???

    uint32_t ie_tmg = 0xf0;
    uint32_t dq_oe_tmg = 0xf0;
    uint32_t dqs_oe_tmg = 0xf0;
    uint32_t rd_tsel_tmg = 0xf0;
    if (ddr_pll_freq_val >= 1200)
    {
        dqs_oe_tmg = 0x61;
        dq_oe_tmg = 0x62;
        ie_tmg = 0xc0;
        rd_tsel_tmg = 0x72;
    }
    uint32_t phymstr_en = 1;
    uint32_t phymstr_intrvl = 0x12; // PHY cnt has bug, cannot be larger than 0x12
    uint32_t fc_retrain_en = 0;
    uint32_t phyupd_en = 0;
    uint32_t pvt_cal_intrvl = 0x10;
    uint32_t dis_auto_ctrlupd_srx = 0;
    uint32_t dis_auto_ctrlupd = 0;

    ddr_top0_clk_en(ENABLE);
    ddr_top0_set_rst(DISABLE);
    ddr_top0_set_rst(ENABLE);

    uint32_t ddr_pll_sel;
    // if (ddr_pll_freq == PLL_BACKUP_MUL_1600MHZ)
         ddr_pll_sel = XDC_CLK_PLL_BK;
    // else
    //    ddr_pll_sel = XDC_CLK_PLL;

    if (lpbk_test == 1)
    {
        ac_vref_en = 1;
        cbt_en = 0;
        gtlvl_en = 0;
        rdlvl_en = 0;
        wdqlvl_en = 0;
    }

    // Configure DDR_CTRL registers
    REG32(DDR_TOP0_BASE+0x00004) = 0x800123 | (cs_map << 13); // regmodel.xdc_top.MODE
    REG32(DDR_TOP0_BASE+0x00018) = 0x20410; // regmodel.xdc_top.LP_CTRL
    REG32(DDR_TOP0_BASE+0x00040) = 0xc02002; // regmodel.xdc_top.INIT_TMG0
    REG32(DDR_TOP0_BASE+0x00070) = 0x1c9e; // regmodel.xdc_top.DFI_CTRL0
    REG32(DDR_TOP0_BASE+0x00078) = 0x3c05 | (dis_auto_ctrlupd << 22) | (dis_auto_ctrlupd_srx << 21); // regmodel.xdc_top.DFI_CTRL2
    //REG32(DDR_TOP0_BASE+0x0007c) = 0x575; // regmodel.xdc_top.DFI_CTRL3
    REG32(DDR_TOP0_BASE+0x0007c) = 0x104; // regmodel.xdc_top.DFI_CTRL3
    REG32(DDR_TOP0_BASE+0x00080) = (phymstr_en << 1) | phyupd_en; // regmodel.xdc_top.DFI_CTRL4
    REG32(DDR_TOP0_BASE+0x00084) = 0x0; // regmodel.xdc_top.DFI_CTRL5
    REG32(DDR_TOP0_BASE+0x00100) = 0x2f7; // regmodel.xdc_top.ADDRMAP0
    REG32(DDR_TOP0_BASE+0x00104) = 0x6318; // regmodel.xdc_top.ADDRMAP1
    REG32(DDR_TOP0_BASE+0x00108) = 0x3ff; // regmodel.xdc_top.ADDRMAP2
    REG32(DDR_TOP0_BASE+0x00114) = 0xff00; // regmodel.xdc_top.ADDRMAP5
    REG32(DDR_TOP0_BASE+0x00118) = 0x4444; // regmodel.xdc_top.ADDRMAP6
    REG32(DDR_TOP0_BASE+0x0011c) = 0x4444; // regmodel.xdc_top.ADDRMAP7
    REG32(DDR_TOP0_BASE+0x00120) = 0x4444; // regmodel.xdc_top.ADDRMAP8
    REG32(DDR_TOP0_BASE+0x00124) = 0x4444; // regmodel.xdc_top.ADDRMAP9
    REG32(DDR_TOP0_BASE+0x00128) = 0xff; // regmodel.xdc_top.ADDRMAP10
    if (ddr_pll_freq_val > 1066)
    {
        REG32(DDR_TOP0_BASE+0x00200) = 0x1404; // regmodel.xdc_top.REF_TMG0_FFC0
        // set t_refi to 0x20 for 1200MT/s
        REG32(DDR_TOP0_BASE+0x0020c) = 0x20008070;//0x2000c070; // regmodel.xdc_top.REF_TMG3_FFC0
        REG32(DDR_TOP0_BASE+0x00220) = 0x50082; // regmodel.xdc_top.RANK_TMG0_FFC0
        REG32(DDR_TOP0_BASE+0x00230) = 0x8206d1; // regmodel.xdc_top.DRAM_TMG0_FFC0
        REG32(DDR_TOP0_BASE+0x00234) = 0x6218; // regmodel.xdc_top.DRAM_TMG1_FFC0
        REG32(DDR_TOP0_BASE+0x00238) = 0x1c430d; // regmodel.xdc_top.DRAM_TMG2_FFC0
        REG32(DDR_TOP0_BASE+0x0023c) = 0x1806; // regmodel.xdc_top.DRAM_TMG3_FFC0
        REG32(DDR_TOP0_BASE+0x00240) = 0x2108a; // regmodel.xdc_top.DRAM_TMG4_FFC0
        REG32(DDR_TOP0_BASE+0x00244) = 0x18c3; // regmodel.xdc_top.DRAM_TMG5_FFC0
        REG32(DDR_TOP0_BASE+0x0025c) = 0x20040012; // regmodel.xdc_top.DRAM_TMG11_FFC0
        REG32(DDR_TOP0_BASE+0x00260) = 0x2850; // regmodel.xdc_top.DRAM_TMG12_FFC0
        REG32(DDR_TOP0_BASE+0x00264) = 0xc350021; // regmodel.xdc_top.DERATE_TMG_FFC0
        REG32(DDR_TOP0_BASE+0x00270) = 0x6400c; // regmodel.xdc_top.ZQCAL_TMG0_FFC0
        REG32(DDR_TOP0_BASE+0x00274) = 0x1d; // regmodel.xdc_top.ZQCAL_TMG1_FFC0
        REG32(DDR_TOP0_BASE+0x00280) = 0x3028a048; // regmodel.xdc_top.DFI_TMG0_FFC0
        REG32(DDR_TOP0_BASE+0x00288) = 0x202; // regmodel.xdc_top.DFI_TMG2_FFC0
        REG32(DDR_TOP0_BASE+0x0028c) = 0x1e; // regmodel.xdc_top.DFI_TMG3_FFC0
    }
    else
    {
        REG32(DDR_TOP0_BASE+0x00200) = 0x1404; // regmodel.xdc_top.REF_TMG0_FFC0
        REG32(DDR_TOP0_BASE+0x0020c) = 0x2000482b; // regmodel.xdc_top.REF_TMG3_FFC0
        REG32(DDR_TOP0_BASE+0x00220) = 0x40082; // regmodel.xdc_top.RANK_TMG0_FFC0
        REG32(DDR_TOP0_BASE+0x00230) = 0x50e287; // regmodel.xdc_top.DRAM_TMG0_FFC0
        REG32(DDR_TOP0_BASE+0x00234) = 0x620a; // regmodel.xdc_top.DRAM_TMG1_FFC0
        REG32(DDR_TOP0_BASE+0x00238) = 0x14328c; // regmodel.xdc_top.DRAM_TMG2_FFC0
        REG32(DDR_TOP0_BASE+0x0023c) = 0xc03; // regmodel.xdc_top.DRAM_TMG3_FFC0
        REG32(DDR_TOP0_BASE+0x00240) = 0xd045; // regmodel.xdc_top.DRAM_TMG4_FFC0
        REG32(DDR_TOP0_BASE+0x00244) = 0x1862; // regmodel.xdc_top.DRAM_TMG5_FFC0
        REG32(DDR_TOP0_BASE+0x0025c) = 0x2004000a; // regmodel.xdc_top.DRAM_TMG11_FFC0
        REG32(DDR_TOP0_BASE+0x00260) = 0x101f; // regmodel.xdc_top.DRAM_TMG12_FFC0
        REG32(DDR_TOP0_BASE+0x00264) = 0xc350011; // regmodel.xdc_top.DERATE_TMG_FFC0
        REG32(DDR_TOP0_BASE+0x00270) = 0x25c05; // regmodel.xdc_top.ZQCAL_TMG0_FFC0
        REG32(DDR_TOP0_BASE+0x00274) = 0x1d; // regmodel.xdc_top.ZQCAL_TMG1_FFC0
        REG32(DDR_TOP0_BASE+0x00280) = 0x30288046; // regmodel.xdc_top.DFI_TMG0_FFC0
        REG32(DDR_TOP0_BASE+0x00288) = 0x100; // regmodel.xdc_top.DFI_TMG2_FFC0
        REG32(DDR_TOP0_BASE+0x0028c) = 0x1e; // regmodel.xdc_top.DFI_TMG3_FFC0
    }
    REG32(DDR_TOP0_BASE+0x00018) = 0x14151; // regmodel.xdc_top.LP_CTRL
    REG32(DDR_TOP0_BASE+0x0001c) = 0x2000000; // regmodel.xdc_top.REF_CTRL
    REG32(DDR_TOP0_BASE+0x0008c) = 0x1; // regmodel.xdc_top.DBI_CTRL
    REG32(DDR_TOP0_BASE+0x00144) = 0x2a0c05; // regmodel.xdc_top.SCH_CTRL
    REG32(DDR_TOP0_BASE+0x00148) = 0xf0070; // regmodel.xdc_top.RDCAM
    REG32(DDR_TOP0_BASE+0x0014c) = 0x9007d; // regmodel.xdc_top.WRCAM
    REG32(DDR_TOP0_BASE+0x00084) = 0x2; // regmodel.xdc_top.DFI_CTRL5
    REG32(DDR_TOP0_BASE+0x00160) = 0x30; // regmodel.xdc_top.DEBUG0


    // Configure DDR_PHY registers
    REG32(DDR_TOP0_CFG_PHY_BASE+0x00004) = 0x8; // regmodel.ddr_phy_ral_top.ddr_phy_top.DDR_TYPE
    REG32(DDR_TOP0_CFG_PHY_BASE+0x00008) = 0x0; // regmodel.ddr_phy_ral_top.ddr_phy_top.HM_DIS_CFG
    // ac swap
    REG32(DDR_TOP0_CFG_PHY_BASE+0x00148) = 0x090a0c08;
    REG32(DDR_TOP0_CFG_PHY_BASE+0x0014C) = 0x0d0b;
    // cbt cfg
    REG32(DDR_TOP0_CFG_PHY_BASE+0x0050) = (0x4 << 16) | (cbt_fdly_step << 12) | (cbt_cdly_step << 8) |
                                            (cbt_capt_cnt << 4) | (cbt_pat_cnt);
    REG32(DDR_TOP0_CFG_PHY_BASE+0x005C) = (cbt_win_siz_thr << 24) | (cbt_win_siz_thr << 16) |
                                            (cbt_margin << 8) | (cbt_margin << 0);

    REG32(DDR_TOP0_CFG_PHY_BASE+0x40000) = 0x1f008000 | (cs_map << 16); // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x42000) = 0x1f008000 | (cs_map << 16); // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[1].DB_GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x22000) = 0xc; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[1].AC_GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x26000) = 0xc; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[3].AC_GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x28000) = 0x4; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[4].AC_GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x2a000) = 0xc; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[5].AC_GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x2c000) = 0x8; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[6].AC_GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x2e000) = 0x8; // regmodel.ddr_phy_ral_top.ddr_phy_acs4[7].AC_GNRL_CFG

    for (uint32_t i=0; i<8; i++)
    {   
        // config acs4 TSEL
        REG32(DDR_TOP0_CFG_PHY_BASE+0x2000*i+0x20010) = (ac_enslicen << 8) | (ac_enslicep << 4);
    }

    REG32(DDR_TOP0_CFG_PHY_BASE+0x00010) = 0x8000 | init_pad_cal_en;
    if (init_pad_cal_en == 0)
    {
        REG32(DDR_TOP0_CFG_PHY_BASE+0x00018) = pvt_code;
        REG32(DDR_TOP0_CFG_PHY_BASE+0x0001C) = pvt_code;
    }

    REG32(DDR_TOP0_CFG_PHY_BASE+0x00200) = (slewn << 20 | slewp << 16) |
                                           (slewn << 12 | slewp << 8 ) |
                                           (slewn << 4  | slewp << 0 ) ; // regmodel.ddr_phy_ral_top.ddr_phy_top.SLEW_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x00208) = (ac_enslicen << 8) | (ac_enslicep << 4); // regmodel.ddr_phy_ral_top.ddr_phy_top.TSEL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x0020c) = 0x3f00104 | (ac_vrefsel << 12) | (ac_vref_en << 7); // regmodel.ddr_phy_ral_top.ddr_phy_top.MISC_PAD_CFG
    
    // t_cksre and t_cbt_pat_chg
    if (ddr_pll_freq_val > 1066)
    {
        REG32(DDR_TOP0_CFG_PHY_BASE+0x00090) = 0x00010a04 | pvt_cal_intrvl; // regmodel.ddr_phy_ral_top.ddr_phy_top.TOP_F0_TMG
    }
    else
    {
        REG32(DDR_TOP0_CFG_PHY_BASE+0x00090) = 0x00010a03 | pvt_cal_intrvl; // regmodel.ddr_phy_ral_top.ddr_phy_top.TOP_F0_TMG
    }

    uint32_t db_idx;
    uint32_t db_addr_ofst;
    for (db_idx=0; db_idx<2; db_idx++) {
        db_addr_ofst = db_idx*0x2000;
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40024) = (wrlvl_all_dq_en << 16) | (wrlvl_dly_step << 8) | (wrlvl_capt_cnt);
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40028) = 0x000000f | (gtlvl_back_step << 16) | (gtlvl_dly_step << 12) | (gtlvl_capt_cnt << 8); 
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x4002c) = (rdlvl_dq_deskew_en << 24) | (rdlvl_bit_mask << 8) |
                                                            (rdlvl_dly_step << 4) | (rdlvl_capt_cnt); // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_RDLVL_CFG0
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40038) = 0x9f0000 | rdlvl_vref_train_step; // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_RDLVL_VREF_CFG
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40040) = 0x0000001 |
                                                            (wdqlvl_win_siz_thr << 20) |
                                                            (wdqlvl_fdly_step << 16) |
                                                            (wdqlvl_cdly_step << 12) |
                                                            (wdqlvl_bit_mask << 1); // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_WDQLVL_CFG0
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40044) = (wdqlvl_te_jump_step << 24) |
                                                            (wdqlvl_left_abrt_thr << 12) |
                                                            (wdqlvl_slv_dly_start); // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_WDQLVL_CFG1
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40084) = (wr_enslicen << 28) | (wr_enslicep << 24) |
                                                            (rd_enslicen << 20) | (rd_enslicep << 16) |
                                                            (rd_tsel_en << 1); // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_TSEL_CFG
        
        if (ddr_pll_freq_val > 1066)
        {
            REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40110) =
                    0x20d0000 |
                    (wr_postamble << 17) |
                    (rd_fifo_dly << 8) |
                    (tsel_dly << 4) |
                    (ie_dly); // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_F0_TMG
                    //(0x1 << 4) |
                    //(0x2); // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_F0_TMG
        }
        else
        {
            REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40110) =
                    0x08d0000 |
                    (wr_postamble << 17) |
                    (rd_fifo_dly << 8) |
                    (tsel_dly << 4) |
                    (ie_dly); // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_F0_TMG
        }

        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40120) = (rd_tsel_tmg << 24) | (dqs_oe_tmg << 16) |
                                                            (ie_tmg << 8) | (dq_oe_tmg); // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_F0_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40130) = (vref_stb_time << 8) | rdlvl_vref_train_en; // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_F0_VREF_CFG
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40140) = vrefsel; // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_F0_VREFSEL_CFG

        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40200) = wrlvl_eval;
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40208) = gtlvl_start;
        REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x4020c) = gtlvl_dly; // regmodel.ddr_phy_ral_top.ddr_phy_dbyte[0].DB_F0_R0_RDDQS_GT_CFG
        
        for (uint32_t i=0; i<9; i++)
        {
            REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40220+i*4) = (rddqs_f_dly << 20) | (rddqs_r_dly << 8) | rddq_dly_enc;
        }

        for (uint32_t i=0; i<9; i++)
        {
            if (db_idx == 0)
                REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40244+i*4) = db0_wrdq_dly;
            else
                REG32(DDR_TOP0_CFG_PHY_BASE+db_addr_ofst+0x40244+i*4) = db1_wrdq_dly;
        }
    }

    REG32(DDR_TOP0_CFG_PHY_BASE+0x60000) = 0x1810100 | cs_map; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.GNRL_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60004) = 0x11201003 | (wdqlvl_bst_len << 16 | cbt_en << 5 | rdlvl_en << 3 | wdqlvl_en << 4 | gtlvl_en << 2) ; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.TRAIN_CFG
    // config dq vref weight
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60010) = (60 << 26) | (4 << 20) | (dq_vref_step << 16);
    // Important!!!, phymstr_en must be configured to 0 before dfi init
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60030) = 0xc | (fc_retrain_en << 1); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.RETRAIN_CFG
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60138) = phymstr_intrvl; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_RETRAIN_TMG
    
    // TODO: two rank ca odt
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60210) = (soc_odt << 24) | (ca_odt << 4) | dq_odt; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_LP34_R0_MR_CFG0
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60220) = (soc_odt << 24) | (ca_odt << 4) | dq_odt; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_LP34_R1_MR_CFG0
            
    if (ddr_pll_freq_val > 1066)
    {
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60200) = 0xa40000 | (rd_preamble_tgl << 19); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_MR_CFG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60204) = 0x12 | 
                                               (dev_pu_cal << 8) |
                                               (wr_postamble << 9) |
                                               (dev_pdds << 11); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_MR_CFG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603a4) = 0x3c5a5555; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.AF_MR_CFG1
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
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60200) = 0x940000 | (rd_preamble_tgl << 19); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_MR_CFG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60204) = 0x9 |
                                               (0x1 << 8) |
                                               (wr_postamble << 9) |
                                               (0x6 << 11); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_MR_CFG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603a4) = 0x3c5a5555; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.AF_MR_CFG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603c0) = 0x4050505; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_DDR_TMG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603c4) = 0x4040203; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_DDR_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603c8) = 0x4; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_DDR_TMG2
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603e0) = 0x84; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_MR_CFG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x603e4) = 0x10; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.BOOT_MR_CFG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x600e0) = 0x4010003; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.AF_DDR_TMG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x600e4) = 0x10019; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.AF_DDR_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60020) = (ca_vref_ed << 24) | (ca_vref_st << 16) | (dq_vref_ed << 8) | (dq_vref_st); // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_VREF_RANGE_CFG
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60100) = 0x3020302; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG0
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60104) = 0x4040404; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG1
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60108) = 0x20002c03; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG2
        REG32(DDR_TOP0_CFG_PHY_BASE+0x6010c) = 0x2605424; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG3
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60110) = 0x70b0c; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG4
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60114) = 0x500ac12; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG5
        REG32(DDR_TOP0_CFG_PHY_BASE+0x60118) = 0x2609826; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG6
        REG32(DDR_TOP0_CFG_PHY_BASE+0x6011c) = 0x10026; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.F0_DDR_TMG7
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
    REG32(DDR_TOP0_BASE+0x00084) = 0x6; // regmodel.xdc_top.DFI_CTRL5

    // LPDDR4 frequency change handshake
    while (ddr_top0_ddr_dfs_req()==0);
#if DDR_DBG_EN == 1
    printf("ddr_top0_ddr_dfs_req == 1, switch to PLL start\n");
#endif
    if (ddr_pll_sel == XDC_CLK_PLL_BK)
        ddr_top0_clk_mux_sel(DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL_BK);
    else
        ddr_top0_clk_mux_sel(DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL);
    REG32(0xf8b300000+0xc88) |=(1<<4);
    while (ddr_top0_ddr_dfs_req()==1);
    REG32(0xf8b300000+0xc88) &=~(1<<4);
#if DDR_DBG_EN == 1
    printf("ddr_top0_ddr_dfs_req == 0, switch to PLL done\n");
#endif

    uint32_t cbt_dfs_pair_num;
    if (cbt_en)
        if (cs_map & 0x2)
            cbt_dfs_pair_num = 2;
        else
            cbt_dfs_pair_num = 1;
    else
        cbt_dfs_pair_num = 0;

    for (uint32_t i=0; i<cbt_dfs_pair_num; i++)
    {
        while (ddr_top0_ddr_dfs_req()==0);
#if DDR_DBG_EN == 1
        printf("ddr_top0_ddr_dfs_req == 1, switch to 25M start\n");
#endif
        ddr_top0_clk_mux_sel(DDR_TOP0_CLK_MUX_SEL_OSC_CLK_25M);
        REG32(0xf8b300000+0xc88) |=(1<<4);
        while (ddr_top0_ddr_dfs_req()==1);
        REG32(0xf8b300000+0xc88) &=~(1<<4);
#if DDR_DBG_EN == 1
        printf("ddr_top0_ddr_dfs_req == 0, switch to 25M done\n");
#endif

        while (ddr_top0_ddr_dfs_req()==0);
#if DDR_DBG_EN == 1
        printf("ddr_top0_ddr_dfs_req == 1, switch to PLL start\n");
#endif
        if (ddr_pll_sel == XDC_CLK_PLL_BK)
            ddr_top0_clk_mux_sel(DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL_BK);
        else
            ddr_top0_clk_mux_sel(DDR_TOP0_CLK_MUX_SEL_XDC_CLK_PLL);
        REG32(0xf8b300000+0xc88) |=(1<<4);
        while (ddr_top0_ddr_dfs_req()==1);
        REG32(0xf8b300000+0xc88) &=~(1<<4);
#if DDR_DBG_EN == 1
        printf("ddr_top0_ddr_dfs_req == 0, switch to PLL done\n");
#endif
    }

    // Polling regmodel.xdc_top.DFI_STATUS until bit 0 is 1.
    uint32_t phy_init_done= 0;
    while(phy_init_done==0){
#if DDR_DBG_EN == 1
        delay_1ms(10000);       
        parse_cbt_obs();
        parse_rdlvl_ctrl_obs();
        parse_mstr_obs(0x0);
        parse_wrlvl_obs(0x0);
        parse_gtlvl_obs(0x0);
        printf("db0_wrdqs_dly:%x\n", REG32(DDR_TOP0_CFG_PHY_BASE+0x40204)); 
        printf("db0_gt_dly:%x\n", REG32(DDR_TOP0_CFG_PHY_BASE+0x4020c));
        parse_mstr_obs(0x1);
        parse_wrlvl_obs(0x1);
        parse_gtlvl_obs(0x1);
        printf("db1_wrdqs_dly:%x\n", REG32(DDR_TOP0_CFG_PHY_BASE+0x42204)); 
        printf("db1_gt_dly:%x\n", REG32(DDR_TOP0_CFG_PHY_BASE+0x4220c));
#endif
        phy_init_done = REG32(DDR_TOP0_BASE+0x88) & XDC_DFI_STATUS_DFI_INIT_COMPLETE;
    };

#if 1//DDR_DBG_EN == 1
    //printf("dfi init done\n");                                                                
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
    }
#endif

    // Configure DDRC to enter mission mode
    REG32(DDR_TOP0_BASE+0x00084) = 0x2;     // regmodel.xdc_top.DFI_CTRL5
    REG32(DDR_TOP0_BASE+0x00084) = 0x3;     // regmodel.xdc_top.DFI_CTRL5
    REG32(DDR_TOP0_BASE+0x00018) = 0x14141; // regmodel.xdc_top.LP_CTRL

    // Important!!! phymstr_en must be configured after de-asserting dfi_init_start 
    REG32(DDR_TOP0_CFG_PHY_BASE+0x60030) = 0xc | (fc_retrain_en << 1) | phymstr_en; // regmodel.ddr_phy_ral_top.ddr_phy_indpdt_train.RETRAIN_CFG

    // Polling regmodel.xdc_top.STA until bit [2:0] is not 0x0.
    uint32_t gfsm_mode = 0x0;
    while(gfsm_mode==0){
        gfsm_mode = REG32(DDR_TOP0_BASE+0x00150) & XDC_STA_GFSM_MODE;
    };
	
	return 0;
}
