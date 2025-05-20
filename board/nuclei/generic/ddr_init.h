#ifndef _NUCLEI_DDR_INIT_H_
#define _NUCLEI_DDR_INIT_H_

#include <linux/bitops.h>

#define BITS(l,h) 	GENMASK(h, l)

typedef enum {
    DISABLE = 0,
    ENABLE = !DISABLE
} ControlStatus;

extern uint32_t ddr_top0_ddr_dfs_req(void);
extern void ddr_top0_clk_mux_sel(uint32_t src_sel);
extern void ddr_top0_set_rst(ControlStatus Status);
extern void ddr_top0_ddr_dfs_ack_pulse(void);
extern uint32_t ddr_top0_ddr_dfs_freq(void);
extern void ddr_top0_clk_en(ControlStatus Status);

#define XDC_VER_OFFSET                                                0x0 /*!< VER */
#define XDC_MODE_OFFSET                                               0x4 /*!< MODE */
#define XDC_MRL0_OFFSET                                               0x8 /*!< MRL0 */
#define XDC_MRL1_OFFSET                                               0xc /*!< MRL1 */
#define XDC_MRL2_OFFSET                                               0x10 /*!< MRL2 */
#define XDC_MRL3_OFFSET                                               0x14 /*!< MRL3 */
#define XDC_LP_CTRL_OFFSET                                            0x18 /*!< LP_CTRL */
#define XDC_REF_CTRL_OFFSET                                           0x1c /*!< REF_CTRL */
#define XDC_CRC_CTRL_OFFSET                                           0x20 /*!< CRC_CTRL */
#define XDC_PARITY_CTRL_OFFSET                                        0x24 /*!< PARITY_CTRL */
#define XDC_ALERT_CTRL_OFFSET                                         0x28 /*!< ALERT_CTRL */
#define XDC_ALERT_STATUS_OFFSET                                       0x2c /*!< ALERT_STATUS */
#define XDC_DERATE_CTRL_OFFSET                                        0x30 /*!< DERATE_CTRL */
#define XDC_DERATE_STATUS_OFFSET                                      0x34 /*!< DERATE_STATUS */
#define XDC_INIT_TMG0_OFFSET                                          0x40 /*!< INIT_TMG0 */
#define XDC_INIT_TMG1_OFFSET                                          0x44 /*!< INIT_TMG1 */
#define XDC_INIT_TMG2_OFFSET                                          0x48 /*!< INIT_TMG2 */
#define XDC_DIMM_CTRL_OFFSET                                          0x60 /*!< DIMM_CTRL */
#define XDC_ZQCAL_CTRL_OFFSET                                         0x64 /*!< ZQCAL_CTRL */
#define XDC_DFI_CTRL0_OFFSET                                          0x70 /*!< DFI_CTRL0 */
#define XDC_DFI_CTRL1_OFFSET                                          0x74 /*!< DFI_CTRL1 */
#define XDC_DFI_CTRL2_OFFSET                                          0x78 /*!< DFI_CTRL2 */
#define XDC_DFI_CTRL3_OFFSET                                          0x7c /*!< DFI_CTRL3 */
#define XDC_DFI_CTRL4_OFFSET                                          0x80 /*!< DFI_CTRL4 */
#define XDC_DFI_CTRL5_OFFSET                                          0x84 /*!< DFI_CTRL5 */
#define XDC_DFI_STATUS_OFFSET                                         0x88 /*!< DFI_STATUS */
#define XDC_DBI_CTRL_OFFSET                                           0x8c /*!< DBI_CTRL */
#define XDC_ADDRMAP0_OFFSET                                           0x100 /*!< ADDRMAP0 */
#define XDC_ADDRMAP1_OFFSET                                           0x104 /*!< ADDRMAP1 */
#define XDC_ADDRMAP2_OFFSET                                           0x108 /*!< ADDRMAP2 */
#define XDC_ADDRMAP3_OFFSET                                           0x10c /*!< ADDRMAP3 */
#define XDC_ADDRMAP4_OFFSET                                           0x110 /*!< ADDRMAP4 */
#define XDC_ADDRMAP5_OFFSET                                           0x114 /*!< ADDRMAP5 */
#define XDC_ADDRMAP6_OFFSET                                           0x118 /*!< ADDRMAP6 */
#define XDC_ADDRMAP7_OFFSET                                           0x11c /*!< ADDRMAP7 */
#define XDC_ADDRMAP8_OFFSET                                           0x120 /*!< ADDRMAP8 */
#define XDC_ADDRMAP9_OFFSET                                           0x124 /*!< ADDRMAP9 */
#define XDC_ADDRMAP10_OFFSET                                          0x128 /*!< ADDRMAP10 */
#define XDC_ODT_CTRL_OFFSET                                           0x140 /*!< ODT_CTRL */
#define XDC_SCH_CTRL_OFFSET                                           0x144 /*!< SCH_CTRL */
#define XDC_RDCAM_OFFSET                                              0x148 /*!< RDCAM */
#define XDC_WRCAM_OFFSET                                              0x14c /*!< WRCAM */
#define XDC_STA_OFFSET                                                0x150 /*!< STA */
#define XDC_DEBUG0_OFFSET                                             0x160 /*!< DEBUG0 */
#define XDC_DEBUG1_OFFSET                                             0x164 /*!< DEBUG1 */
#define XDC_DEBUG2_OFFSET                                             0x168 /*!< DEBUG2 */
#define XDC_DEBUG3_OFFSET                                             0x16c /*!< DEBUG3 */
#define XDC_DEBUG6_OFFSET                                             0x178 /*!< DEBUG6 */
#define XDC_DEBUG7_OFFSET                                             0x17c /*!< DEBUG7 */
#define XDC_DEBUG8_OFFSET                                             0x180 /*!< DEBUG8 */
#define XDC_DEBUG9_OFFSET                                             0x184 /*!< DEBUG9 */
#define XDC_DEBUG10_OFFSET                                            0x188 /*!< DEBUG10 */
#define XDC_REF_TMG0_FFC0_OFFSET                                      0x200 /*!< REF_TMG0_FFC0 */
#define XDC_REF_TMG1_FFC0_OFFSET                                      0x204 /*!< REF_TMG1_FFC0 */
#define XDC_REF_TMG3_FFC0_OFFSET                                      0x20c /*!< REF_TMG3_FFC0 */
#define XDC_MR_VAL0_FFC0_OFFSET                                       0x210 /*!< MR_VAL0_FFC0 */
#define XDC_MR_VAL1_FFC0_OFFSET                                       0x214 /*!< MR_VAL1_FFC0 */
#define XDC_MR_VAL2_FFC0_OFFSET                                       0x218 /*!< MR_VAL2_FFC0 */
#define XDC_MR_VAL3_FFC0_OFFSET                                       0x21c /*!< MR_VAL3_FFC0 */
#define XDC_RANK_TMG0_FFC0_OFFSET                                     0x220 /*!< RANK_TMG0_FFC0 */
#define XDC_DRAM_TMG0_FFC0_OFFSET                                     0x230 /*!< DRAM_TMG0_FFC0 */
#define XDC_DRAM_TMG1_FFC0_OFFSET                                     0x234 /*!< DRAM_TMG1_FFC0 */
#define XDC_DRAM_TMG2_FFC0_OFFSET                                     0x238 /*!< DRAM_TMG2_FFC0 */
#define XDC_DRAM_TMG3_FFC0_OFFSET                                     0x23c /*!< DRAM_TMG3_FFC0 */
#define XDC_DRAM_TMG4_FFC0_OFFSET                                     0x240 /*!< DRAM_TMG4_FFC0 */
#define XDC_DRAM_TMG5_FFC0_OFFSET                                     0x244 /*!< DRAM_TMG5_FFC0 */
#define XDC_DRAM_TMG6_FFC0_OFFSET                                     0x248 /*!< DRAM_TMG6_FFC0 */
#define XDC_DRAM_TMG7_FFC0_OFFSET                                     0x24c /*!< DRAM_TMG7_FFC0 */
#define XDC_DRAM_TMG8_FFC0_OFFSET                                     0x250 /*!< DRAM_TMG8_FFC0 */
#define XDC_DRAM_TMG9_FFC0_OFFSET                                     0x254 /*!< DRAM_TMG9_FFC0 */
#define XDC_DRAM_TMG10_FFC0_OFFSET                                    0x258 /*!< DRAM_TMG10_FFC0 */
#define XDC_DRAM_TMG11_FFC0_OFFSET                                    0x25c /*!< DRAM_TMG11_FFC0 */
#define XDC_DRAM_TMG12_FFC0_OFFSET                                    0x260 /*!< DRAM_TMG12_FFC0 */
#define XDC_DERATE_TMG_FFC0_OFFSET                                    0x264 /*!< DRAM_TMG11_FFC0 */
#define XDC_ZQCAL_TMG0_FFC0_OFFSET                                    0x270 /*!< ZQCAL_TMG0_FFC0 */
#define XDC_ZQCAL_TMG1_FFC0_OFFSET                                    0x274 /*!< ZQCAL_TMG1_FFC0 */
#define XDC_DFI_TMG0_FFC0_OFFSET                                      0x280 /*!< DFI_TMG0_FFC0 */
#define XDC_DFI_TMG1_FFC0_OFFSET                                      0x284 /*!< DFI_TMG1_FFC0 */
#define XDC_DFI_TMG2_FFC0_OFFSET                                      0x288 /*!< DFI_TMG2_FFC0 */
#define XDC_DFI_TMG3_FFC0_OFFSET                                      0x28c /*!< DFI_TMG3_FFC0 */
#define XDC_ODT_TMG_FFC0_OFFSET                                       0x290 /*!< ODT_TMG_FFC0 */
#define XDC_REF_TMG0_FFC1_OFFSET                                      0x300 /*!< REF_TMG0_FFC1 */
#define XDC_REF_TMG1_FFC1_OFFSET                                      0x304 /*!< REF_TMG1_FFC1 */
#define XDC_REF_TMG3_FFC1_OFFSET                                      0x30c /*!< REF_TMG3_FFC1 */
#define XDC_MR_VAL0_FFC1_OFFSET                                       0x310 /*!< MR_VAL0_FFC1 */
#define XDC_MR_VAL1_FFC1_OFFSET                                       0x314 /*!< MR_VAL1_FFC1 */
#define XDC_MR_VAL2_FFC1_OFFSET                                       0x318 /*!< MR_VAL2_FFC1 */
#define XDC_MR_VAL3_FFC1_OFFSET                                       0x31c /*!< MR_VAL3_FFC1 */
#define XDC_RANK_TMG0_FFC1_OFFSET                                     0x320 /*!< RANK_TMG0_FFC1 */
#define XDC_DRAM_TMG0_FFC1_OFFSET                                     0x330 /*!< DRAM_TMG0_FFC1 */
#define XDC_DRAM_TMG1_FFC1_OFFSET                                     0x334 /*!< DRAM_TMG1_FFC1 */
#define XDC_DRAM_TMG2_FFC1_OFFSET                                     0x338 /*!< DRAM_TMG2_FFC1 */
#define XDC_DRAM_TMG3_FFC1_OFFSET                                     0x33c /*!< DRAM_TMG3_FFC1 */
#define XDC_DRAM_TMG4_FFC1_OFFSET                                     0x340 /*!< DRAM_TMG4_FFC1 */
#define XDC_DRAM_TMG5_FFC1_OFFSET                                     0x344 /*!< DRAM_TMG5_FFC1 */
#define XDC_DRAM_TMG6_FFC1_OFFSET                                     0x348 /*!< DRAM_TMG6_FFC1 */
#define XDC_DRAM_TMG7_FFC1_OFFSET                                     0x34c /*!< DRAM_TMG7_FFC1 */
#define XDC_DRAM_TMG8_FFC1_OFFSET                                     0x350 /*!< DRAM_TMG8_FFC1 */
#define XDC_DRAM_TMG9_FFC1_OFFSET                                     0x354 /*!< DRAM_TMG9_FFC1 */
#define XDC_DRAM_TMG10_FFC1_OFFSET                                    0x358 /*!< DRAM_TMG10_FFC1 */
#define XDC_DRAM_TMG11_FFC1_OFFSET                                    0x35c /*!< DRAM_TMG11_FFC1 */
#define XDC_DRAM_TMG12_FFC1_OFFSET                                    0x360 /*!< DRAM_TMG12_FFC1 */
#define XDC_DERATE_TMG_FFC1_OFFSET                                    0x364 /*!< DRAM_TMG11_FFC1 */
#define XDC_ZQCAL_TMG0_FFC1_OFFSET                                    0x370 /*!< ZQCAL_TMG0_FFC1 */
#define XDC_ZQCAL_TMG1_FFC1_OFFSET                                    0x374 /*!< ZQCAL_TMG1_FFC1 */
#define XDC_DFI_TMG0_FFC1_OFFSET                                      0x380 /*!< DFI_TMG0_FFC1 */
#define XDC_DFI_TMG1_FFC1_OFFSET                                      0x384 /*!< DFI_TMG1_FFC1 */
#define XDC_DFI_TMG2_FFC1_OFFSET                                      0x388 /*!< DFI_TMG2_FFC1 */
#define XDC_DFI_TMG3_FFC1_OFFSET                                      0x38c /*!< DFI_TMG3_FFC1 */
#define XDC_ODT_TMG_FFC1_OFFSET                                       0x390 /*!< ODT_TMG_FFC1 */
#define XDC_PWCFG0_OFFSET                                             0x600 /*!< PWCFG0 */
#define XDC_PRCFG0_OFFSET                                             0x604 /*!< PRCFG0 */

 /* ===== XDC VER Register definition ===== */
#define XDC_VER_IDVAL                        BITS(0,23)                
 
 /* ===== XDC MODE Register definition ===== */
#define XDC_MODE_WORK_MODE_MASK               BITS(0,1)                                   /*!< XDC MODE: WORK_MODE Bit Mask */  
#define XDC_MODE_WORK_MODE_OFS                0U                                          /*!< XDC MODE: WORK_MODE Bit Offset */
#define XDC_MODE_WORK_MODE(regval)            (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< XDC MODE: WORK_MODE Bit Value */  
#define XDC_MODE_BURST_LEN_MASK               BITS(2,5)                                   /*!< XDC MODE: BURST_LEN Bit Mask */  
#define XDC_MODE_BURST_LEN_OFS                2U                                          /*!< XDC MODE: BURST_LEN Bit Offset */
#define XDC_MODE_BURST_LEN(regval)            (BITS(2,5) & ((uint32_t)(regval) << 2))        /*!< XDC MODE: BURST_LEN Bit Value */  
#define XDC_MODE_BC_EN                        BIT(6)                                      /*!< 0 - Disable burst chop 1 - Enable burst chop */
#define XDC_MODE_BC_EN_OFS                    6U                                          /*!< XDC MODE: BC_EN Bit Offset */
#define XDC_MODE_BC_EN_VAL(regval)                (BIT(6) & ((uint32_t)(regval) << 6))        /*!< XDC MODE: BC_EN Bit Value */  
#define XDC_MODE_DLL_OFF                      BIT(7)                                      /*!< 0 - Disable dll off mode 1 - Enable dll off mode */
#define XDC_MODE_DLL_OFF_OFS                  7U                                          /*!< XDC MODE: DLL_OFF Bit Offset */
#define XDC_MODE_DLL_OFF_VAL(regval)              (BIT(7) & ((uint32_t)(regval) << 7))        /*!< XDC MODE: DLL_OFF Bit Value */  
#define XDC_MODE_DEV_CFG_MASK                 BITS(8,9)                                   /*!< XDC MODE: DEV_CFG Bit Mask */  
#define XDC_MODE_DEV_CFG_OFS                  8U                                          /*!< XDC MODE: DEV_CFG Bit Offset */
#define XDC_MODE_DEV_CFG(regval)              (BITS(8,9) & ((uint32_t)(regval) << 8))        /*!< XDC MODE: DEV_CFG Bit Value */  
#define XDC_MODE_GRD_EN                       BIT(10)                                      /*!< 0 - Disable geardown mode 1 - Enable geardown mode */
#define XDC_MODE_GRD_EN_OFS                   10U                                          /*!< XDC MODE: GRD_EN Bit Offset */
#define XDC_MODE_GRD_EN_VAL(regval)               (BIT(10) & ((uint32_t)(regval) << 10))        /*!< XDC MODE: GRD_EN Bit Value */  
#define XDC_MODE_BUS_WIDTH_MASK               BITS(11,12)                                   /*!< XDC MODE: BUS_WIDTH Bit Mask */  
#define XDC_MODE_BUS_WIDTH_OFS                11U                                          /*!< XDC MODE: BUS_WIDTH Bit Offset */
#define XDC_MODE_BUS_WIDTH(regval)            (BITS(11,12) & ((uint32_t)(regval) << 11))        /*!< XDC MODE: BUS_WIDTH Bit Value */  
#define XDC_MODE_RANK_NUM_MASK                BITS(13,16)                                   /*!< XDC MODE: RANK_NUM Bit Mask */  
#define XDC_MODE_RANK_NUM_OFS                 13U                                          /*!< XDC MODE: RANK_NUM Bit Offset */
#define XDC_MODE_RANK_NUM(regval)             (BITS(13,16) & ((uint32_t)(regval) << 13))        /*!< XDC MODE: RANK_NUM Bit Value */  
#define XDC_MODE_FFC_PTR_MASK                 BITS(17,18)                                   /*!< XDC MODE: FFC_PTR Bit Mask */  
#define XDC_MODE_FFC_PTR_OFS                  17U                                          /*!< XDC MODE: FFC_PTR Bit Offset */
#define XDC_MODE_FFC_PTR(regval)              (BITS(17,18) & ((uint32_t)(regval) << 17))        /*!< XDC MODE: FFC_PTR Bit Value */  
#define XDC_MODE_FFC_SET                      BIT(19)                                      /*!< Fast frequency change set. Note: The hardware automatically clears this bit after it has been synchronized to XDC clock domain. */
#define XDC_MODE_FFC_SET_OFS                  19U                                          /*!< XDC MODE: FFC_SET Bit Offset */
#define XDC_MODE_FFC_SET_VAL(regval)              (BIT(19) & ((uint32_t)(regval) << 19))        /*!< XDC MODE: FFC_SET Bit Value */  
#define XDC_MODE_XIF_BURST_LEN_MASK           BITS(20,23)                                   /*!< XDC MODE: XIF_BURST_LEN Bit Mask */  
#define XDC_MODE_XIF_BURST_LEN_OFS            20U                                          /*!< XDC MODE: XIF_BURST_LEN Bit Offset */
#define XDC_MODE_XIF_BURST_LEN(regval)        (BITS(20,23) & ((uint32_t)(regval) << 20))        /*!< XDC MODE: XIF_BURST_LEN Bit Value */  
 
 /* ===== XDC MRL0 Register definition ===== */
#define XDC_MRL0_MR_TYPE                      BIT(0)                                      /*!< Mode register operation is read or write 0 - Write 1 - Read */
#define XDC_MRL0_MR_TYPE_OFS                  0U                                          /*!< XDC MRL0: MR_TYPE Bit Offset */
#define XDC_MRL0_MR_TYPE_VAL(regval)              (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC MRL0: MR_TYPE Bit Value */  
#define XDC_MRL0_MPR_EN                       BIT(1)                                      /*!< Mode register operation is MRS or RD/WR for MPR 0 - MRS 1 - RD/WR MPR */
#define XDC_MRL0_MPR_EN_OFS                   1U                                          /*!< XDC MRL0: MPR_EN Bit Offset */
#define XDC_MRL0_MPR_EN_VAL(regval)               (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC MRL0: MPR_EN Bit Value */  
#define XDC_MRL0_PDA_EN                       BIT(2)                                      /*!< Mode register operation is MRS in normal mode or in pda mode 0 - MRS in normal mode 1 - MRS in PDA mode */
#define XDC_MRL0_PDA_EN_OFS                   2U                                          /*!< XDC MRL0: PDA_EN Bit Offset */
#define XDC_MRL0_PDA_EN_VAL(regval)               (BIT(2) & ((uint32_t)(regval) << 2))        /*!< XDC MRL0: PDA_EN Bit Value */  
#define XDC_MRL0_MR_ADDR_MASK                 BITS(3,6)                                   /*!< XDC MRL0: MR_ADDR Bit Mask */  
#define XDC_MRL0_MR_ADDR_OFS                  3U                                          /*!< XDC MRL0: MR_ADDR Bit Offset */
#define XDC_MRL0_MR_ADDR(regval)              (BITS(3,6) & ((uint32_t)(regval) << 3))        /*!< XDC MRL0: MR_ADDR Bit Value */  
#define XDC_MRL0_MR_WR                        BIT(7)                                      /*!< Set this bit to trigger a mode register write operation. Note: The hardware automatically clears this bit after it has been synchronized to XDC clock domain. */
#define XDC_MRL0_MR_WR_OFS                    7U                                          /*!< XDC MRL0: MR_WR Bit Offset */
#define XDC_MRL0_MR_WR_VAL(regval)                (BIT(7) & ((uint32_t)(regval) << 7))        /*!< XDC MRL0: MR_WR Bit Value */  
#define XDC_MRL0_MR_RANK_MASK                 BITS(8,11)                                   /*!< XDC MRL0: MR_RANK Bit Mask */  
#define XDC_MRL0_MR_RANK_OFS                  8U                                          /*!< XDC MRL0: MR_RANK Bit Offset */
#define XDC_MRL0_MR_RANK(regval)              (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< XDC MRL0: MR_RANK Bit Value */  
 
 /* ===== XDC MRL1 Register definition ===== */
#define XDC_MRL1_MR_DATA_MASK                 BITS(0,17)                                   /*!< XDC MRL1: MR_DATA Bit Mask */  
#define XDC_MRL1_MR_DATA_OFS                  0U                                          /*!< XDC MRL1: MR_DATA Bit Offset */
#define XDC_MRL1_MR_DATA(regval)              (BITS(0,17) & ((uint32_t)(regval) << 0))        /*!< XDC MRL1: MR_DATA Bit Value */  
 
 /* ===== XDC MRL2 Register definition ===== */
#define XDC_MRL2_DEV_SEL_MASK                 BITS(0,15)                                   /*!< XDC MRL2: DEV_SEL Bit Mask */  
#define XDC_MRL2_DEV_SEL_OFS                  0U                                          /*!< XDC MRL2: DEV_SEL Bit Offset */
#define XDC_MRL2_DEV_SEL(regval)              (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< XDC MRL2: DEV_SEL Bit Value */  
 
 /* ===== XDC MRL3 Register definition ===== */
#define XDC_MRL3_MR_WR_BUSY                   BIT(0)                                      /*!< Indicate that mrl request has been received and has not done when it assert. */
#define XDC_MRL3_PDA_DONE                     BIT(1)                                      /*!< Active high, it assert when MRS in PDA mode has completed. */
 
 /* ===== XDC LP_CTRL Register definition ===== */
#define XDC_LP_CTRL_SRF_EN                       BIT(0)                                      /*!< XDC automatically put sdram into self-refresh state when XDC stay in idle state beyond a programmable clock cycles. */
#define XDC_LP_CTRL_SRF_EN_OFS                   0U                                          /*!< XDC LP CTRL: SRF_EN Bit Offset */
#define XDC_LP_CTRL_SRF_EN_VAL(regval)               (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC LP CTRL: SRF_EN Bit Value */  
#define XDC_LP_CTRL_PWD_EN                       BIT(1)                                      /*!< XDC automatically put sdram into power-down state when XDC stay in idle state beyond a programmable clock cycles. */
#define XDC_LP_CTRL_PWD_EN_OFS                   1U                                          /*!< XDC LP CTRL: PWD_EN Bit Offset */
#define XDC_LP_CTRL_PWD_EN_VAL(regval)               (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC LP CTRL: PWD_EN Bit Value */  
#define XDC_LP_CTRL_MPSM_EN                      BIT(2)                                      /*!< Active high, set it to put sdram into MPSM mode. */
#define XDC_LP_CTRL_MPSM_EN_OFS                  2U                                          /*!< XDC LP CTRL: MPSM_EN Bit Offset */
#define XDC_LP_CTRL_MPSM_EN_VAL(regval)              (BIT(2) & ((uint32_t)(regval) << 2))        /*!< XDC LP CTRL: MPSM_EN Bit Value */  
#define XDC_LP_CTRL_DRAM_CLK_DIS                 BIT(3)                                      /*!< Enable to assert dfi_dram_clk_disable when XDC is in low power state. */
#define XDC_LP_CTRL_DRAM_CLK_DIS_OFS             3U                                          /*!< XDC LP CTRL: DRAM_CLK_DIS Bit Offset */
#define XDC_LP_CTRL_DRAM_CLK_DIS_VAL(regval)         (BIT(3) & ((uint32_t)(regval) << 3))        /*!< XDC LP CTRL: DRAM_CLK_DIS Bit Value */  
#define XDC_LP_CTRL_SRF_TRIGGER                  BIT(4)                                      /*!< 0 - Trigger Exit to self-refresh 1 - Trigger Entry to self-refresh */
#define XDC_LP_CTRL_SRF_TRIGGER_OFS              4U                                          /*!< XDC LP CTRL: SRF_TRIGGER Bit Offset */
#define XDC_LP_CTRL_SRF_TRIGGER_VAL(regval)          (BIT(4) & ((uint32_t)(regval) << 4))        /*!< XDC LP CTRL: SRF_TRIGGER Bit Value */  
#define XDC_LP_CTRL_SRE_CAM_NOT_EMPTY            BIT(5)                                      /*!< 0 - CAM must be empty before entering self-refresh 1 - Ignore CAM state before entering self-refresh */
#define XDC_LP_CTRL_SRE_CAM_NOT_EMPTY_OFS        5U                                          /*!< XDC LP CTRL: SRE_CAM_NOT_EMPTY Bit Offset */
#define XDC_LP_CTRL_SRE_CAM_NOT_EMPTY_VAL(regval)    (BIT(5) & ((uint32_t)(regval) << 5))        /*!< XDC LP CTRL: SRE_CAM_NOT_EMPTY Bit Value */  
#define XDC_LP_CTRL_PWD_GAP_MASK                 BITS(6,10)                                   /*!< XDC LP CTRL: PWD_GAP Bit Mask */  
#define XDC_LP_CTRL_PWD_GAP_OFS                  6U                                          /*!< XDC LP CTRL: PWD_GAP Bit Offset */
#define XDC_LP_CTRL_PWD_GAP(regval)              (BITS(6,10) & ((uint32_t)(regval) << 6))        /*!< XDC LP CTRL: PWD_GAP Bit Value */  
#define XDC_LP_CTRL_SRF_GAP_MASK                 BITS(11,18)                                   /*!< XDC LP CTRL: SRF_GAP Bit Mask */  
#define XDC_LP_CTRL_SRF_GAP_OFS                  11U                                          /*!< XDC LP CTRL: SRF_GAP Bit Offset */
#define XDC_LP_CTRL_SRF_GAP(regval)              (BITS(11,18) & ((uint32_t)(regval) << 11))        /*!< XDC LP CTRL: SRF_GAP Bit Value */  
#define XDC_LP_CTRL_SPD_EN                       BIT(19)                                      /*!< XDC automatically put LPDDR4 sdram into self-refresh power down state. LP_CTRL.srf_en must also be set to 1 to enable this feature. */
#define XDC_LP_CTRL_SPD_EN_OFS                   19U                                          /*!< XDC LP CTRL: SPD_EN Bit Offset */
#define XDC_LP_CTRL_SPD_EN_VAL(regval)               (BIT(19) & ((uint32_t)(regval) << 19))        /*!< XDC LP CTRL: SPD_EN Bit Value */  
#define XDC_LP_CTRL_STAY_IN_SRF                  BIT(20)                                      /*!< Reserved */
#define XDC_LP_CTRL_STAY_IN_SRF_OFS              20U                                          /*!< XDC LP CTRL: STAY_IN_SRF Bit Offset */
#define XDC_LP_CTRL_STAY_IN_SRF_VAL(regval)          (BIT(20) & ((uint32_t)(regval) << 20))        /*!< XDC LP CTRL: STAY_IN_SRF Bit Value */  
 
 /* ===== XDC REF_CTRL Register definition ===== */
#define XDC_REF_CTRL_REF_UPD                      BIT(0)                                      /*!< Set this bit to update all refresh related configurations. Note: The hardware automatically clears this bit after it has been synchronized to XDC clock domain. */
#define XDC_REF_CTRL_REF_UPD_OFS                  0U                                          /*!< XDC REF CTRL: REF_UPD Bit Offset */
#define XDC_REF_CTRL_REF_UPD_VAL(regval)              (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC REF CTRL: REF_UPD Bit Value */  
#define XDC_REF_CTRL_DIS_AUTO_REF                 BIT(1)                                      /*!< 0 - Enable auto-refresh 1 - Disable auto-refresh */
#define XDC_REF_CTRL_DIS_AUTO_REF_OFS             1U                                          /*!< XDC REF CTRL: DIS_AUTO_REF Bit Offset */
#define XDC_REF_CTRL_DIS_AUTO_REF_VAL(regval)         (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC REF CTRL: DIS_AUTO_REF Bit Value */  
#define XDC_REF_CTRL_DIS_SPEC_REF                 BIT(2)                                      /*!< 0 - Enable speculative-refresh 1 - Disable speculative-refresh */
#define XDC_REF_CTRL_DIS_SPEC_REF_OFS             2U                                          /*!< XDC REF CTRL: DIS_SPEC_REF Bit Offset */
#define XDC_REF_CTRL_DIS_SPEC_REF_VAL(regval)         (BIT(2) & ((uint32_t)(regval) << 2))        /*!< XDC REF CTRL: DIS_SPEC_REF Bit Value */  
#define XDC_REF_CTRL_PB_REF_EN                    BIT(3)                                      /*!< 0 - Enable LPDDR4 per bank refresh 1 - Disable LPDDR4 per bank refresh */
#define XDC_REF_CTRL_PB_REF_EN_OFS                3U                                          /*!< XDC REF CTRL: PB_REF_EN Bit Offset */
#define XDC_REF_CTRL_PB_REF_EN_VAL(regval)            (BIT(3) & ((uint32_t)(regval) << 3))        /*!< XDC REF CTRL: PB_REF_EN Bit Value */  
#define XDC_REF_CTRL_DERATE_EN                    BIT(4)                                      /*!< If set this field to 1, XDC will derate the refresh rate and certain timing parameters for different temperature. For LPDDR4, XDC will automatically read the MR4. For DDR4, XDC will automatically read the MPR page2 (location 01). */
#define XDC_REF_CTRL_DERATE_EN_OFS                4U                                          /*!< XDC REF CTRL: DERATE_EN Bit Offset */
#define XDC_REF_CTRL_DERATE_EN_VAL(regval)            (BIT(4) & ((uint32_t)(regval) << 4))        /*!< XDC REF CTRL: DERATE_EN Bit Value */  
#define XDC_REF_CTRL_TS_AUTO_RD_PAUSE             BIT(5)                                      /*!< If set this field to 1, the autonomous temperature sensor read will be paused. This field must be set to 1 before changing frequency. */
#define XDC_REF_CTRL_TS_AUTO_RD_PAUSE_OFS         5U                                          /*!< XDC REF CTRL: TS_AUTO_RD_PAUSE Bit Offset */
#define XDC_REF_CTRL_TS_AUTO_RD_PAUSE_VAL(regval)     (BIT(5) & ((uint32_t)(regval) << 5))        /*!< XDC REF CTRL: TS_AUTO_RD_PAUSE Bit Value */  
#define XDC_REF_CTRL_DERATE_DOM_BYTE_MASK         BITS(8,11)                                   /*!< XDC REF CTRL: DERATE_DOM_BYTE Bit Mask */  
#define XDC_REF_CTRL_DERATE_DOM_BYTE_OFS          8U                                          /*!< XDC REF CTRL: DERATE_DOM_BYTE Bit Offset */
#define XDC_REF_CTRL_DERATE_DOM_BYTE(regval)      (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< XDC REF CTRL: DERATE_DOM_BYTE Bit Value */  
#define XDC_REF_CTRL_DERATE_REFI_SW_MODE          BIT(12)                                      /*!< Reserved */
#define XDC_REF_CTRL_DERATE_REFI_SW_MODE_OFS      12U                                          /*!< XDC REF CTRL: DERATE_REFI_SW_MODE Bit Offset */
#define XDC_REF_CTRL_DERATE_REFI_SW_MODE_VAL(regval)  (BIT(12) & ((uint32_t)(regval) << 12))        /*!< XDC REF CTRL: DERATE_REFI_SW_MODE Bit Value */  
#define XDC_REF_CTRL_DERATE_REFI_VAL_MASK         BITS(13,15)                                   /*!< XDC REF CTRL: DERATE_REFI_VAL Bit Mask */  
#define XDC_REF_CTRL_DERATE_REFI_VAL_OFS          13U                                          /*!< XDC REF CTRL: DERATE_REFI_VAL Bit Offset */
#define XDC_REF_CTRL_DERATE_REFI_VAL(regval)      (BITS(13,15) & ((uint32_t)(regval) << 13))        /*!< XDC REF CTRL: DERATE_REFI_VAL Bit Value */  
#define XDC_REF_CTRL_SRX_REF_NUM_MASK             BITS(24,25)                                   /*!< XDC REF CTRL: SRX_REF_NUM Bit Mask */  
#define XDC_REF_CTRL_SRX_REF_NUM_OFS              24U                                          /*!< XDC REF CTRL: SRX_REF_NUM Bit Offset */
#define XDC_REF_CTRL_SRX_REF_NUM(regval)          (BITS(24,25) & ((uint32_t)(regval) << 24))        /*!< XDC REF CTRL: SRX_REF_NUM Bit Value */  
 
 /* ===== XDC CRC_CTRL Register definition ===== */
#define XDC_CRC_CTRL_CRC_EN                       BIT(0)                                      /*!< 0 - disable crc calculation 1 - enable crc calculation */
#define XDC_CRC_CTRL_CRC_EN_OFS                   0U                                          /*!< XDC CRC CTRL: CRC_EN Bit Offset */
#define XDC_CRC_CTRL_CRC_EN_VAL(regval)               (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC CRC CTRL: CRC_EN Bit Value */  
#define XDC_CRC_CTRL_CRC_INC_DM                   BIT(1)                                      /*!< 0 - crc calculation exclude dm signals 1 - crc calculation include dm signals */
#define XDC_CRC_CTRL_CRC_INC_DM_OFS               1U                                          /*!< XDC CRC CTRL: CRC_INC_DM Bit Offset */
#define XDC_CRC_CTRL_CRC_INC_DM_VAL(regval)           (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC CRC CTRL: CRC_INC_DM Bit Value */  
 
 /* ===== XDC PARITY_CTRL Register definition ===== */
#define XDC_PARITY_CTRL_PAR_DIS_BF_SRE               BIT(0)                                      /*!< 1 - CA parity is disabled before srf entry 0 - CA parity is not disabled before srf entry */
#define XDC_PARITY_CTRL_PAR_DIS_BF_SRE_OFS           0U                                          /*!< XDC PARITY CTRL: PAR_DIS_BF_SRE Bit Offset */
#define XDC_PARITY_CTRL_PAR_DIS_BF_SRE_VAL(regval)       (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC PARITY CTRL: PAR_DIS_BF_SRE Bit Value */  
 
 /* ===== XDC ALERT_CTRL Register definition ===== */
#define XDC_ALERT_CTRL_ALERT_ERR_INT_EN             BIT(0)                                      /*!< Enable bit for DFI alert error interrupt, set this bit to enable error interrupt on ALERT_STA.alert_err_int. */
#define XDC_ALERT_CTRL_ALERT_ERR_INT_EN_OFS         0U                                          /*!< XDC ALERT CTRL: ALERT_ERR_INT_EN Bit Offset */
#define XDC_ALERT_CTRL_ALERT_ERR_INT_EN_VAL(regval)     (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC ALERT CTRL: ALERT_ERR_INT_EN Bit Value */  
#define XDC_ALERT_CTRL_ALERT_ERR_INT_CLR            BIT(1)                                      /*!< Clear bit for DFI alert error interrupt, set this bit to clear error interrupt on ALERT_STA.alert_err_int. Note: The hardware automatically clears this bit after it has been synchronized to XDC clock domain. */
#define XDC_ALERT_CTRL_ALERT_ERR_INT_CLR_OFS        1U                                          /*!< XDC ALERT CTRL: ALERT_ERR_INT_CLR Bit Offset */
#define XDC_ALERT_CTRL_ALERT_ERR_INT_CLR_VAL(regval)    (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC ALERT CTRL: ALERT_ERR_INT_CLR Bit Value */  
#define XDC_ALERT_CTRL_ALERT_ERR_CNT_CLR            BIT(2)                                      /*!< Clear bit for DFI alert error counter, set this bit to clear error counter on ALERT_STA.alert_err_cnt. Note: The hardware automatically clears this bit after it has been synchronized to XDC clock domain. */
#define XDC_ALERT_CTRL_ALERT_ERR_CNT_CLR_OFS        2U                                          /*!< XDC ALERT CTRL: ALERT_ERR_CNT_CLR Bit Offset */
#define XDC_ALERT_CTRL_ALERT_ERR_CNT_CLR_VAL(regval)    (BIT(2) & ((uint32_t)(regval) << 2))        /*!< XDC ALERT CTRL: ALERT_ERR_CNT_CLR Bit Value */  
 
 /* ===== XDC ALERT_STATUS Register definition ===== */
#define XDC_ALERT_STATUS_ALERT_ERR_INT                BIT(0)                                      /*!< If a parity/CRC error is detected on dfi_alert_n and ALERT_CTRL.alert_err_int_en is set, then this interrupt bit is set. It can be cleared by setting ALERT_CTRL.alert_err_int_clear. */
#define XDC_ALERT_STATUS_ALERT_ERR_CNT                BITS(1,16)                
 
 /* ===== XDC DERATE_CTRL Register definition ===== */
#define XDC_DERATE_CTRL_DERATE_TEMP_LIM_INT_EN         BIT(0)                                      /*!< Enable bit for derate_temp_lim_int interrupt. */
#define XDC_DERATE_CTRL_DERATE_TEMP_LIM_INT_EN_OFS     0U                                          /*!< XDC DERATE CTRL: DERATE_TEMP_LIM_INT_EN Bit Offset */
#define XDC_DERATE_CTRL_DERATE_TEMP_LIM_INT_EN_VAL(regval) (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC DERATE CTRL: DERATE_TEMP_LIM_INT_EN Bit Value */  
#define XDC_DERATE_CTRL_DERATE_TEMP_LIM_INT_CLR         BIT(1)                                      /*!< Clear bit for derate_temp_lim_int interrupt. Note: The hardware automatically clears this bit after it has been synchronized to XDC clock domain. */
#define XDC_DERATE_CTRL_DERATE_TEMP_LIM_INT_CLR_OFS     1U                                          /*!< XDC DERATE CTRL: DERATE_TEMP_LIM_INT_CLR Bit Offset */
#define XDC_DERATE_CTRL_DERATE_TEMP_LIM_INT_CLR_VAL(regval) (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC DERATE CTRL: DERATE_TEMP_LIM_INT_CLR Bit Value */  
 
 /* ===== XDC DERATE_STATUS Register definition ===== */
#define XDC_DERATE_STATUS_DERATE_TEMP_LIM_INT          BIT(0)                                      /*!< If a parity/CRC error is detected on dfi_alert_n and ALERT_CTRL.alert_err_int_en is set, then this interrupt bit is set. It can be cleared by setting ALERT_CTRL.alert_err_int_clear. */
 
 /* ===== XDC INIT_TMG0 Register definition ===== */
#define XDC_INIT_TMG0_PRE_CKE_MASK                 BITS(0,11)                                   /*!< XDC INIT TMG0: PRE_CKE Bit Mask */  
#define XDC_INIT_TMG0_PRE_CKE_OFS                  0U                                          /*!< XDC INIT TMG0: PRE_CKE Bit Offset */
#define XDC_INIT_TMG0_PRE_CKE(regval)              (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< XDC INIT TMG0: PRE_CKE Bit Value */  
#define XDC_INIT_TMG0_POST_CKE_MASK                BITS(12,21)                                   /*!< XDC INIT TMG0: POST_CKE Bit Mask */  
#define XDC_INIT_TMG0_POST_CKE_OFS                 12U                                          /*!< XDC INIT TMG0: POST_CKE Bit Offset */
#define XDC_INIT_TMG0_POST_CKE(regval)             (BITS(12,21) & ((uint32_t)(regval) << 12))        /*!< XDC INIT TMG0: POST_CKE Bit Value */  
#define XDC_INIT_TMG0_INIT_STATE_MASK              BITS(22,23)                                   /*!< XDC INIT TMG0: INIT_STATE Bit Mask */  
#define XDC_INIT_TMG0_INIT_STATE_OFS               22U                                          /*!< XDC INIT TMG0: INIT_STATE Bit Offset */
#define XDC_INIT_TMG0_INIT_STATE(regval)           (BITS(22,23) & ((uint32_t)(regval) << 22))        /*!< XDC INIT TMG0: INIT_STATE Bit Value */  
 
 /* ===== XDC INIT_TMG1 Register definition ===== */
#define XDC_INIT_TMG1_DRAM_RSTN_MASK               BITS(0,8)                                   /*!< XDC INIT TMG1: DRAM_RSTN Bit Mask */  
#define XDC_INIT_TMG1_DRAM_RSTN_OFS                0U                                          /*!< XDC INIT TMG1: DRAM_RSTN Bit Offset */
#define XDC_INIT_TMG1_DRAM_RSTN(regval)            (BITS(0,8) & ((uint32_t)(regval) << 0))        /*!< XDC INIT TMG1: DRAM_RSTN Bit Value */  
 
 /* ===== XDC INIT_TMG2 Register definition ===== */
#define XDC_INIT_TMG2_ZQINIT_MASK                  BITS(0,7)                                   /*!< XDC INIT TMG2: ZQINIT Bit Mask */  
#define XDC_INIT_TMG2_ZQINIT_OFS                   0U                                          /*!< XDC INIT TMG2: ZQINIT Bit Offset */
#define XDC_INIT_TMG2_ZQINIT(regval)               (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< XDC INIT TMG2: ZQINIT Bit Value */  
 
 /* ===== XDC DIMM_CTRL Register definition ===== */
#define XDC_DIMM_CTRL_ADDR_MIRR_EN                 BIT(0)                                      /*!< 0 - disable address mirror 1 - enable address mirror */
#define XDC_DIMM_CTRL_ADDR_MIRR_EN_OFS             0U                                          /*!< XDC DIMM CTRL: ADDR_MIRR_EN Bit Offset */
#define XDC_DIMM_CTRL_ADDR_MIRR_EN_VAL(regval)         (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC DIMM CTRL: ADDR_MIRR_EN Bit Value */  
#define XDC_DIMM_CTRL_DIS_BG_MIRR                  BIT(1)                                      /*!< 0 - enable bank groups bit mirror 1 - disable bank groups bit mirror */
#define XDC_DIMM_CTRL_DIS_BG_MIRR_OFS              1U                                          /*!< XDC DIMM CTRL: DIS_BG_MIRR Bit Offset */
#define XDC_DIMM_CTRL_DIS_BG_MIRR_VAL(regval)          (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC DIMM CTRL: DIS_BG_MIRR Bit Value */  
 
 /* ===== XDC ZQCAL_CTRL Register definition ===== */
#define XDC_ZQCAL_CTRL_DIS_SRX_ZQCL                 BIT(21)                                      /*!< 0 - Enable send ZQCL when exiting self-refresh 1 - Disable send ZQCL when exiting self-refresh */
#define XDC_ZQCAL_CTRL_DIS_SRX_ZQCL_OFS             21U                                          /*!< XDC ZQCAL CTRL: DIS_SRX_ZQCL Bit Offset */
#define XDC_ZQCAL_CTRL_DIS_SRX_ZQCL_VAL(regval)         (BIT(21) & ((uint32_t)(regval) << 21))        /*!< XDC ZQCAL CTRL: DIS_SRX_ZQCL Bit Value */  
#define XDC_ZQCAL_CTRL_DIS_AUTO_ZQ                  BIT(22)                                      /*!< 0 - Enable automatically ZQ calibration 1 - Disable automatically ZQ calibration */
#define XDC_ZQCAL_CTRL_DIS_AUTO_ZQ_OFS              22U                                          /*!< XDC ZQCAL CTRL: DIS_AUTO_ZQ Bit Offset */
#define XDC_ZQCAL_CTRL_DIS_AUTO_ZQ_VAL(regval)          (BIT(22) & ((uint32_t)(regval) << 22))        /*!< XDC ZQCAL CTRL: DIS_AUTO_ZQ Bit Value */  
#define XDC_ZQCAL_CTRL_DIS_MPX_ZQCL                 BIT(23)                                      /*!< 0 - enable sending ZQCL command at MPSM mode exit. 1 - disable sending ZQCL command at MPSM mode exit. */
#define XDC_ZQCAL_CTRL_DIS_MPX_ZQCL_OFS             23U                                          /*!< XDC ZQCAL CTRL: DIS_MPX_ZQCL Bit Offset */
#define XDC_ZQCAL_CTRL_DIS_MPX_ZQCL_VAL(regval)         (BIT(23) & ((uint32_t)(regval) << 23))        /*!< XDC ZQCAL CTRL: DIS_MPX_ZQCL Bit Value */  
#define XDC_ZQCAL_CTRL_SHR_ZQ_RES                   BIT(24)                                      /*!< 0 - ZQ resistor is not shared between ranks. 1 - ZQ resistor is shared between ranks. */
#define XDC_ZQCAL_CTRL_SHR_ZQ_RES_OFS               24U                                          /*!< XDC ZQCAL CTRL: SHR_ZQ_RES Bit Offset */
#define XDC_ZQCAL_CTRL_SHR_ZQ_RES_VAL(regval)           (BIT(24) & ((uint32_t)(regval) << 24))        /*!< XDC ZQCAL CTRL: SHR_ZQ_RES Bit Value */  
 
 /* ===== XDC DFI_CTRL0 Register definition ===== */
#define XDC_DFI_CTRL0_DFI_LP_EN_PWD                BIT(0)                                      /*!< Enables DFI Low Power interface handshaking during power-down Entry/Exit. */
#define XDC_DFI_CTRL0_DFI_LP_EN_PWD_OFS            0U                                          /*!< XDC DFI CTRL0: DFI_LP_EN_PWD Bit Offset */
#define XDC_DFI_CTRL0_DFI_LP_EN_PWD_VAL(regval)        (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC DFI CTRL0: DFI_LP_EN_PWD Bit Value */  
#define XDC_DFI_CTRL0_DFI_LP_WAKEUP_PWD_MASK       BITS(1,4)                                   /*!< XDC DFI CTRL0: DFI_LP_WAKEUP_PWD Bit Mask */  
#define XDC_DFI_CTRL0_DFI_LP_WAKEUP_PWD_OFS        1U                                          /*!< XDC DFI CTRL0: DFI_LP_WAKEUP_PWD Bit Offset */
#define XDC_DFI_CTRL0_DFI_LP_WAKEUP_PWD(regval)    (BITS(1,4) & ((uint32_t)(regval) << 1))        /*!< XDC DFI CTRL0: DFI_LP_WAKEUP_PWD Bit Value */  
#define XDC_DFI_CTRL0_DFI_LP_EN_SRF                BIT(5)                                      /*!< Enables DFI Low Power interface handshaking during self-refresh Entry/Exit. */
#define XDC_DFI_CTRL0_DFI_LP_EN_SRF_OFS            5U                                          /*!< XDC DFI CTRL0: DFI_LP_EN_SRF Bit Offset */
#define XDC_DFI_CTRL0_DFI_LP_EN_SRF_VAL(regval)        (BIT(5) & ((uint32_t)(regval) << 5))        /*!< XDC DFI CTRL0: DFI_LP_EN_SRF Bit Value */  
#define XDC_DFI_CTRL0_DFI_LP_WAKEUP_SRF_MASK       BITS(6,9)                                   /*!< XDC DFI CTRL0: DFI_LP_WAKEUP_SRF Bit Mask */  
#define XDC_DFI_CTRL0_DFI_LP_WAKEUP_SRF_OFS        6U                                          /*!< XDC DFI CTRL0: DFI_LP_WAKEUP_SRF Bit Offset */
#define XDC_DFI_CTRL0_DFI_LP_WAKEUP_SRF(regval)    (BITS(6,9) & ((uint32_t)(regval) << 6))        /*!< XDC DFI CTRL0: DFI_LP_WAKEUP_SRF Bit Value */  
#define XDC_DFI_CTRL0_DFI_T_LP_RESP_MASK           BITS(10,14)                                   /*!< XDC DFI CTRL0: DFI_T_LP_RESP Bit Mask */  
#define XDC_DFI_CTRL0_DFI_T_LP_RESP_OFS            10U                                          /*!< XDC DFI CTRL0: DFI_T_LP_RESP Bit Offset */
#define XDC_DFI_CTRL0_DFI_T_LP_RESP(regval)        (BITS(10,14) & ((uint32_t)(regval) << 10))        /*!< XDC DFI CTRL0: DFI_T_LP_RESP Bit Value */  
 
 /* ===== XDC DFI_CTRL1 Register definition ===== */
#define XDC_DFI_CTRL1_DFI_LP_EN_MPSM               BIT(0)                                      /*!< Enables DFI Low Power interface handshaking during MPSM Entry/Exit. */
#define XDC_DFI_CTRL1_DFI_LP_EN_MPSM_OFS           0U                                          /*!< XDC DFI CTRL1: DFI_LP_EN_MPSM Bit Offset */
#define XDC_DFI_CTRL1_DFI_LP_EN_MPSM_VAL(regval)       (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC DFI CTRL1: DFI_LP_EN_MPSM Bit Value */  
#define XDC_DFI_CTRL1_DFI_LP_WAKEUP_MPSM_MASK      BITS(1,4)                                   /*!< XDC DFI CTRL1: DFI_LP_WAKEUP_MPSM Bit Mask */  
#define XDC_DFI_CTRL1_DFI_LP_WAKEUP_MPSM_OFS       1U                                          /*!< XDC DFI CTRL1: DFI_LP_WAKEUP_MPSM Bit Offset */
#define XDC_DFI_CTRL1_DFI_LP_WAKEUP_MPSM(regval)   (BITS(1,4) & ((uint32_t)(regval) << 1))        /*!< XDC DFI CTRL1: DFI_LP_WAKEUP_MPSM Bit Value */  
 
 /* ===== XDC DFI_CTRL2 Register definition ===== */
#define XDC_DFI_CTRL2_DFI_T_CTRLUP_MIN_MASK        BITS(0,9)                                   /*!< XDC DFI CTRL2: DFI_T_CTRLUP_MIN Bit Mask */  
#define XDC_DFI_CTRL2_DFI_T_CTRLUP_MIN_OFS         0U                                          /*!< XDC DFI CTRL2: DFI_T_CTRLUP_MIN Bit Offset */
#define XDC_DFI_CTRL2_DFI_T_CTRLUP_MIN(regval)     (BITS(0,9) & ((uint32_t)(regval) << 0))        /*!< XDC DFI CTRL2: DFI_T_CTRLUP_MIN Bit Value */  
#define XDC_DFI_CTRL2_DFI_T_CTRLUP_MAX_MASK        BITS(10,19)                                   /*!< XDC DFI CTRL2: DFI_T_CTRLUP_MAX Bit Mask */  
#define XDC_DFI_CTRL2_DFI_T_CTRLUP_MAX_OFS         10U                                          /*!< XDC DFI CTRL2: DFI_T_CTRLUP_MAX Bit Offset */
#define XDC_DFI_CTRL2_DFI_T_CTRLUP_MAX(regval)     (BITS(10,19) & ((uint32_t)(regval) << 10))        /*!< XDC DFI CTRL2: DFI_T_CTRLUP_MAX Bit Value */  
#define XDC_DFI_CTRL2_DIS_AUTO_CTRLUPD_SRX         BIT(21)                                      /*!< 0 - Enable automatically controller-initialized update when exiting self-refresh 1 - Disable automatically controller-initialized update when exiting self-refresh */
#define XDC_DFI_CTRL2_DIS_AUTO_CTRLUPD_SRX_OFS     21U                                          /*!< XDC DFI CTRL2: DIS_AUTO_CTRLUPD_SRX Bit Offset */
#define XDC_DFI_CTRL2_DIS_AUTO_CTRLUPD_SRX_VAL(regval) (BIT(21) & ((uint32_t)(regval) << 21))        /*!< XDC DFI CTRL2: DIS_AUTO_CTRLUPD_SRX Bit Value */  
#define XDC_DFI_CTRL2_DIS_AUTO_CTRLUPD             BIT(22)                                      /*!< 0 - Enable periodically controller-initialized update 1 - Disable periodically controller-initialized update */
#define XDC_DFI_CTRL2_DIS_AUTO_CTRLUPD_OFS         22U                                          /*!< XDC DFI CTRL2: DIS_AUTO_CTRLUPD Bit Offset */
#define XDC_DFI_CTRL2_DIS_AUTO_CTRLUPD_VAL(regval)     (BIT(22) & ((uint32_t)(regval) << 22))        /*!< XDC DFI CTRL2: DIS_AUTO_CTRLUPD Bit Value */  
 
 /* ===== XDC DFI_CTRL3 Register definition ===== */
#define XDC_DFI_CTRL3_DFI_T_CTRLUPD_INT_MAX_MASK    BITS(0,7)                                   /*!< XDC DFI CTRL3: DFI_T_CTRLUPD_INT_MAX Bit Mask */  
#define XDC_DFI_CTRL3_DFI_T_CTRLUPD_INT_MAX_OFS     0U                                          /*!< XDC DFI CTRL3: DFI_T_CTRLUPD_INT_MAX Bit Offset */
#define XDC_DFI_CTRL3_DFI_T_CTRLUPD_INT_MAX(regval) (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< XDC DFI CTRL3: DFI_T_CTRLUPD_INT_MAX Bit Value */  
#define XDC_DFI_CTRL3_DFI_T_CTRLUPD_INT_MIN_MASK    BITS(8,15)                                   /*!< XDC DFI CTRL3: DFI_T_CTRLUPD_INT_MIN Bit Mask */  
#define XDC_DFI_CTRL3_DFI_T_CTRLUPD_INT_MIN_OFS     8U                                          /*!< XDC DFI CTRL3: DFI_T_CTRLUPD_INT_MIN Bit Offset */
#define XDC_DFI_CTRL3_DFI_T_CTRLUPD_INT_MIN(regval) (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< XDC DFI CTRL3: DFI_T_CTRLUPD_INT_MIN Bit Value */  
 
 /* ===== XDC DFI_CTRL4 Register definition ===== */
#define XDC_DFI_CTRL4_DFI_PHYUPD_EN                BIT(0)                                      /*!< 0 - Disable phy-initialized update 1 - Enable phy-initialized update */
#define XDC_DFI_CTRL4_DFI_PHYUPD_EN_OFS            0U                                          /*!< XDC DFI CTRL4: DFI_PHYUPD_EN Bit Offset */
#define XDC_DFI_CTRL4_DFI_PHYUPD_EN_VAL(regval)        (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC DFI CTRL4: DFI_PHYUPD_EN Bit Value */  
#define XDC_DFI_CTRL4_DFI_PHYMSTR_EN               BIT(1)                                      /*!< 0 - Disable the PHY Master Interface 1 - Enable the PHY Master Interface */
#define XDC_DFI_CTRL4_DFI_PHYMSTR_EN_OFS           1U                                          /*!< XDC DFI CTRL4: DFI_PHYMSTR_EN Bit Offset */
#define XDC_DFI_CTRL4_DFI_PHYMSTR_EN_VAL(regval)       (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC DFI CTRL4: DFI_PHYMSTR_EN Bit Value */  
 
 /* ===== XDC DFI_CTRL5 Register definition ===== */
#define XDC_DFI_CTRL5_DFI_INIT_COMPLETE_EN         BIT(0)                                      /*!< use dfi_init_complete to trigger SDRAM initialization. */
#define XDC_DFI_CTRL5_DFI_INIT_COMPLETE_EN_OFS     0U                                          /*!< XDC DFI CTRL5: DFI_INIT_COMPLETE_EN Bit Offset */
#define XDC_DFI_CTRL5_DFI_INIT_COMPLETE_EN_VAL(regval) (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC DFI CTRL5: DFI_INIT_COMPLETE_EN Bit Value */  
#define XDC_DFI_CTRL5_DFI_DATA_CS_POLARITY         BIT(1)                                      /*!< set the polarity of dfi_wrdata_cs and dfi_rddata_cs: 0 - active low 1 - active high */
#define XDC_DFI_CTRL5_DFI_DATA_CS_POLARITY_OFS     1U                                          /*!< XDC DFI CTRL5: DFI_DATA_CS_POLARITY Bit Offset */
#define XDC_DFI_CTRL5_DFI_DATA_CS_POLARITY_VAL(regval) (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC DFI CTRL5: DFI_DATA_CS_POLARITY Bit Value */  
#define XDC_DFI_CTRL5_DFI_INIT_START               BIT(2)                                      /*!< Set it to trigger the phy init start request. */
#define XDC_DFI_CTRL5_DFI_INIT_START_OFS           2U                                          /*!< XDC DFI CTRL5: DFI_INIT_START Bit Offset */
#define XDC_DFI_CTRL5_DFI_INIT_START_VAL(regval)       (BIT(2) & ((uint32_t)(regval) << 2))        /*!< XDC DFI CTRL5: DFI_INIT_START Bit Value */  
#define XDC_DFI_CTRL5_DFI_FREQUENCY_MASK           BITS(3,7)                                   /*!< XDC DFI CTRL5: DFI_FREQUENCY Bit Mask */  
#define XDC_DFI_CTRL5_DFI_FREQUENCY_OFS            3U                                          /*!< XDC DFI CTRL5: DFI_FREQUENCY Bit Offset */
#define XDC_DFI_CTRL5_DFI_FREQUENCY(regval)        (BITS(3,7) & ((uint32_t)(regval) << 3))        /*!< XDC DFI CTRL5: DFI_FREQUENCY Bit Value */  
 
 /* ===== XDC DFI_STATUS Register definition ===== */
#define XDC_DFI_STATUS_DFI_INIT_COMPLETE            BIT(0)                                      /*!< Indicate the returned value of the dfi_init_complete on DFI interface. */
#define XDC_DFI_STATUS_DFI_LP_ACK                   BIT(1)                                      /*!< Indicate the returned value of the dfi_lp_ack on DFI interface. */
 
 /* ===== XDC DBI_CTRL Register definition ===== */
#define XDC_DBI_CTRL_DM_EN                        BIT(0)                                      /*!< 0 - disable DM. 1 - enable DM. */
#define XDC_DBI_CTRL_DM_EN_OFS                    0U                                          /*!< XDC DBI CTRL: DM_EN Bit Offset */
#define XDC_DBI_CTRL_DM_EN_VAL(regval)                (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC DBI CTRL: DM_EN Bit Value */  
#define XDC_DBI_CTRL_DBI_MODE                     BIT(1)                                      /*!< 0 - ddr controller implements DBI. 1 - ddr phy implements DBI. */
#define XDC_DBI_CTRL_DBI_MODE_OFS                 1U                                          /*!< XDC DBI CTRL: DBI_MODE Bit Offset */
#define XDC_DBI_CTRL_DBI_MODE_VAL(regval)             (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC DBI CTRL: DBI_MODE Bit Value */  
#define XDC_DBI_CTRL_WR_DBI_EN                    BIT(2)                                      /*!< 0 - disable write DBI. 1 - enable write DBI. */
#define XDC_DBI_CTRL_WR_DBI_EN_OFS                2U                                          /*!< XDC DBI CTRL: WR_DBI_EN Bit Offset */
#define XDC_DBI_CTRL_WR_DBI_EN_VAL(regval)            (BIT(2) & ((uint32_t)(regval) << 2))        /*!< XDC DBI CTRL: WR_DBI_EN Bit Value */  
#define XDC_DBI_CTRL_RD_DBI_EN                    BIT(3)                                      /*!< 0 - disable read DBI. 1 - enable read DBI. */
#define XDC_DBI_CTRL_RD_DBI_EN_OFS                3U                                          /*!< XDC DBI CTRL: RD_DBI_EN Bit Offset */
#define XDC_DBI_CTRL_RD_DBI_EN_VAL(regval)            (BIT(3) & ((uint32_t)(regval) << 3))        /*!< XDC DBI CTRL: RD_DBI_EN Bit Value */  
 
 /* ===== XDC ADDRMAP0 Register definition ===== */
#define XDC_ADDRMAP0_RANK_OFFSET_B0_MASK          BITS(0,4)                                   /*!< XDC ADDRMAP0: RANK_OFFSET_B0 Bit Mask */  
#define XDC_ADDRMAP0_RANK_OFFSET_B0_OFS           0U                                          /*!< XDC ADDRMAP0: RANK_OFFSET_B0 Bit Offset */
#define XDC_ADDRMAP0_RANK_OFFSET_B0(regval)       (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC ADDRMAP0: RANK_OFFSET_B0 Bit Value */  
#define XDC_ADDRMAP0_RANK_OFFSET_B1_MASK          BITS(5,9)                                   /*!< XDC ADDRMAP0: RANK_OFFSET_B1 Bit Mask */  
#define XDC_ADDRMAP0_RANK_OFFSET_B1_OFS           5U                                          /*!< XDC ADDRMAP0: RANK_OFFSET_B1 Bit Offset */
#define XDC_ADDRMAP0_RANK_OFFSET_B1(regval)       (BITS(5,9) & ((uint32_t)(regval) << 5))        /*!< XDC ADDRMAP0: RANK_OFFSET_B1 Bit Value */  
 
 /* ===== XDC ADDRMAP1 Register definition ===== */
#define XDC_ADDRMAP1_BANK_OFFSET_B0_MASK          BITS(0,4)                                   /*!< XDC ADDRMAP1: BANK_OFFSET_B0 Bit Mask */  
#define XDC_ADDRMAP1_BANK_OFFSET_B0_OFS           0U                                          /*!< XDC ADDRMAP1: BANK_OFFSET_B0 Bit Offset */
#define XDC_ADDRMAP1_BANK_OFFSET_B0(regval)       (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC ADDRMAP1: BANK_OFFSET_B0 Bit Value */  
#define XDC_ADDRMAP1_BANK_OFFSET_B1_MASK          BITS(5,9)                                   /*!< XDC ADDRMAP1: BANK_OFFSET_B1 Bit Mask */  
#define XDC_ADDRMAP1_BANK_OFFSET_B1_OFS           5U                                          /*!< XDC ADDRMAP1: BANK_OFFSET_B1 Bit Offset */
#define XDC_ADDRMAP1_BANK_OFFSET_B1(regval)       (BITS(5,9) & ((uint32_t)(regval) << 5))        /*!< XDC ADDRMAP1: BANK_OFFSET_B1 Bit Value */  
#define XDC_ADDRMAP1_BANK_OFFSET_B2_MASK          BITS(10,14)                                   /*!< XDC ADDRMAP1: BANK_OFFSET_B2 Bit Mask */  
#define XDC_ADDRMAP1_BANK_OFFSET_B2_OFS           10U                                          /*!< XDC ADDRMAP1: BANK_OFFSET_B2 Bit Offset */
#define XDC_ADDRMAP1_BANK_OFFSET_B2(regval)       (BITS(10,14) & ((uint32_t)(regval) << 10))        /*!< XDC ADDRMAP1: BANK_OFFSET_B2 Bit Value */  
 
 /* ===== XDC ADDRMAP2 Register definition ===== */
#define XDC_ADDRMAP2_BG_OFFSET_B0_MASK            BITS(0,4)                                   /*!< XDC ADDRMAP2: BG_OFFSET_B0 Bit Mask */  
#define XDC_ADDRMAP2_BG_OFFSET_B0_OFS             0U                                          /*!< XDC ADDRMAP2: BG_OFFSET_B0 Bit Offset */
#define XDC_ADDRMAP2_BG_OFFSET_B0(regval)         (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC ADDRMAP2: BG_OFFSET_B0 Bit Value */  
#define XDC_ADDRMAP2_BG_OFFSET_B1_MASK            BITS(5,9)                                   /*!< XDC ADDRMAP2: BG_OFFSET_B1 Bit Mask */  
#define XDC_ADDRMAP2_BG_OFFSET_B1_OFS             5U                                          /*!< XDC ADDRMAP2: BG_OFFSET_B1 Bit Offset */
#define XDC_ADDRMAP2_BG_OFFSET_B1(regval)         (BITS(5,9) & ((uint32_t)(regval) << 5))        /*!< XDC ADDRMAP2: BG_OFFSET_B1 Bit Value */  
 
 /* ===== XDC ADDRMAP3 Register definition ===== */
#define XDC_ADDRMAP3_COL_OFFSET_B0_MASK           BITS(0,3)                                   /*!< XDC ADDRMAP3: COL_OFFSET_B0 Bit Mask */  
#define XDC_ADDRMAP3_COL_OFFSET_B0_OFS            0U                                          /*!< XDC ADDRMAP3: COL_OFFSET_B0 Bit Offset */
#define XDC_ADDRMAP3_COL_OFFSET_B0(regval)        (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< XDC ADDRMAP3: COL_OFFSET_B0 Bit Value */  
#define XDC_ADDRMAP3_COL_OFFSET_B1_MASK           BITS(4,7)                                   /*!< XDC ADDRMAP3: COL_OFFSET_B1 Bit Mask */  
#define XDC_ADDRMAP3_COL_OFFSET_B1_OFS            4U                                          /*!< XDC ADDRMAP3: COL_OFFSET_B1 Bit Offset */
#define XDC_ADDRMAP3_COL_OFFSET_B1(regval)        (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< XDC ADDRMAP3: COL_OFFSET_B1 Bit Value */  
#define XDC_ADDRMAP3_COL_OFFSET_B2_MASK           BITS(8,11)                                   /*!< XDC ADDRMAP3: COL_OFFSET_B2 Bit Mask */  
#define XDC_ADDRMAP3_COL_OFFSET_B2_OFS            8U                                          /*!< XDC ADDRMAP3: COL_OFFSET_B2 Bit Offset */
#define XDC_ADDRMAP3_COL_OFFSET_B2(regval)        (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< XDC ADDRMAP3: COL_OFFSET_B2 Bit Value */  
#define XDC_ADDRMAP3_COL_OFFSET_B3_MASK           BITS(12,15)                                   /*!< XDC ADDRMAP3: COL_OFFSET_B3 Bit Mask */  
#define XDC_ADDRMAP3_COL_OFFSET_B3_OFS            12U                                          /*!< XDC ADDRMAP3: COL_OFFSET_B3 Bit Offset */
#define XDC_ADDRMAP3_COL_OFFSET_B3(regval)        (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< XDC ADDRMAP3: COL_OFFSET_B3 Bit Value */  
 
 /* ===== XDC ADDRMAP4 Register definition ===== */
#define XDC_ADDRMAP4_COL_OFFSET_B4_MASK           BITS(0,3)                                   /*!< XDC ADDRMAP4: COL_OFFSET_B4 Bit Mask */  
#define XDC_ADDRMAP4_COL_OFFSET_B4_OFS            0U                                          /*!< XDC ADDRMAP4: COL_OFFSET_B4 Bit Offset */
#define XDC_ADDRMAP4_COL_OFFSET_B4(regval)        (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< XDC ADDRMAP4: COL_OFFSET_B4 Bit Value */  
#define XDC_ADDRMAP4_COL_OFFSET_B5_MASK           BITS(4,7)                                   /*!< XDC ADDRMAP4: COL_OFFSET_B5 Bit Mask */  
#define XDC_ADDRMAP4_COL_OFFSET_B5_OFS            4U                                          /*!< XDC ADDRMAP4: COL_OFFSET_B5 Bit Offset */
#define XDC_ADDRMAP4_COL_OFFSET_B5(regval)        (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< XDC ADDRMAP4: COL_OFFSET_B5 Bit Value */  
#define XDC_ADDRMAP4_COL_OFFSET_B6_MASK           BITS(8,11)                                   /*!< XDC ADDRMAP4: COL_OFFSET_B6 Bit Mask */  
#define XDC_ADDRMAP4_COL_OFFSET_B6_OFS            8U                                          /*!< XDC ADDRMAP4: COL_OFFSET_B6 Bit Offset */
#define XDC_ADDRMAP4_COL_OFFSET_B6(regval)        (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< XDC ADDRMAP4: COL_OFFSET_B6 Bit Value */  
#define XDC_ADDRMAP4_COL_OFFSET_B7_MASK           BITS(12,15)                                   /*!< XDC ADDRMAP4: COL_OFFSET_B7 Bit Mask */  
#define XDC_ADDRMAP4_COL_OFFSET_B7_OFS            12U                                          /*!< XDC ADDRMAP4: COL_OFFSET_B7 Bit Offset */
#define XDC_ADDRMAP4_COL_OFFSET_B7(regval)        (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< XDC ADDRMAP4: COL_OFFSET_B7 Bit Value */  
 
 /* ===== XDC ADDRMAP5 Register definition ===== */
#define XDC_ADDRMAP5_COL_OFFSET_B8_MASK           BITS(0,3)                                   /*!< XDC ADDRMAP5: COL_OFFSET_B8 Bit Mask */  
#define XDC_ADDRMAP5_COL_OFFSET_B8_OFS            0U                                          /*!< XDC ADDRMAP5: COL_OFFSET_B8 Bit Offset */
#define XDC_ADDRMAP5_COL_OFFSET_B8(regval)        (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< XDC ADDRMAP5: COL_OFFSET_B8 Bit Value */  
#define XDC_ADDRMAP5_COL_OFFSET_B9_MASK           BITS(4,7)                                   /*!< XDC ADDRMAP5: COL_OFFSET_B9 Bit Mask */  
#define XDC_ADDRMAP5_COL_OFFSET_B9_OFS            4U                                          /*!< XDC ADDRMAP5: COL_OFFSET_B9 Bit Offset */
#define XDC_ADDRMAP5_COL_OFFSET_B9(regval)        (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< XDC ADDRMAP5: COL_OFFSET_B9 Bit Value */  
#define XDC_ADDRMAP5_COL_OFFSET_B10_MASK          BITS(8,11)                                   /*!< XDC ADDRMAP5: COL_OFFSET_B10 Bit Mask */  
#define XDC_ADDRMAP5_COL_OFFSET_B10_OFS           8U                                          /*!< XDC ADDRMAP5: COL_OFFSET_B10 Bit Offset */
#define XDC_ADDRMAP5_COL_OFFSET_B10(regval)       (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< XDC ADDRMAP5: COL_OFFSET_B10 Bit Value */  
#define XDC_ADDRMAP5_COL_OFFSET_B11_MASK          BITS(12,15)                                   /*!< XDC ADDRMAP5: COL_OFFSET_B11 Bit Mask */  
#define XDC_ADDRMAP5_COL_OFFSET_B11_OFS           12U                                          /*!< XDC ADDRMAP5: COL_OFFSET_B11 Bit Offset */
#define XDC_ADDRMAP5_COL_OFFSET_B11(regval)       (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< XDC ADDRMAP5: COL_OFFSET_B11 Bit Value */  
 
 /* ===== XDC ADDRMAP6 Register definition ===== */
#define XDC_ADDRMAP6_ROW_OFFSET_B0_MASK           BITS(0,3)                                   /*!< XDC ADDRMAP6: ROW_OFFSET_B0 Bit Mask */  
#define XDC_ADDRMAP6_ROW_OFFSET_B0_OFS            0U                                          /*!< XDC ADDRMAP6: ROW_OFFSET_B0 Bit Offset */
#define XDC_ADDRMAP6_ROW_OFFSET_B0(regval)        (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< XDC ADDRMAP6: ROW_OFFSET_B0 Bit Value */  
#define XDC_ADDRMAP6_ROW_OFFSET_B1_MASK           BITS(4,7)                                   /*!< XDC ADDRMAP6: ROW_OFFSET_B1 Bit Mask */  
#define XDC_ADDRMAP6_ROW_OFFSET_B1_OFS            4U                                          /*!< XDC ADDRMAP6: ROW_OFFSET_B1 Bit Offset */
#define XDC_ADDRMAP6_ROW_OFFSET_B1(regval)        (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< XDC ADDRMAP6: ROW_OFFSET_B1 Bit Value */  
#define XDC_ADDRMAP6_ROW_OFFSET_B2_MASK           BITS(8,11)                                   /*!< XDC ADDRMAP6: ROW_OFFSET_B2 Bit Mask */  
#define XDC_ADDRMAP6_ROW_OFFSET_B2_OFS            8U                                          /*!< XDC ADDRMAP6: ROW_OFFSET_B2 Bit Offset */
#define XDC_ADDRMAP6_ROW_OFFSET_B2(regval)        (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< XDC ADDRMAP6: ROW_OFFSET_B2 Bit Value */  
#define XDC_ADDRMAP6_ROW_OFFSET_B3_MASK           BITS(12,15)                                   /*!< XDC ADDRMAP6: ROW_OFFSET_B3 Bit Mask */  
#define XDC_ADDRMAP6_ROW_OFFSET_B3_OFS            12U                                          /*!< XDC ADDRMAP6: ROW_OFFSET_B3 Bit Offset */
#define XDC_ADDRMAP6_ROW_OFFSET_B3(regval)        (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< XDC ADDRMAP6: ROW_OFFSET_B3 Bit Value */  
 
 /* ===== XDC ADDRMAP7 Register definition ===== */
#define XDC_ADDRMAP7_ROW_OFFSET_B4_MASK           BITS(0,3)                                   /*!< XDC ADDRMAP7: ROW_OFFSET_B4 Bit Mask */  
#define XDC_ADDRMAP7_ROW_OFFSET_B4_OFS            0U                                          /*!< XDC ADDRMAP7: ROW_OFFSET_B4 Bit Offset */
#define XDC_ADDRMAP7_ROW_OFFSET_B4(regval)        (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< XDC ADDRMAP7: ROW_OFFSET_B4 Bit Value */  
#define XDC_ADDRMAP7_ROW_OFFSET_B5_MASK           BITS(4,7)                                   /*!< XDC ADDRMAP7: ROW_OFFSET_B5 Bit Mask */  
#define XDC_ADDRMAP7_ROW_OFFSET_B5_OFS            4U                                          /*!< XDC ADDRMAP7: ROW_OFFSET_B5 Bit Offset */
#define XDC_ADDRMAP7_ROW_OFFSET_B5(regval)        (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< XDC ADDRMAP7: ROW_OFFSET_B5 Bit Value */  
#define XDC_ADDRMAP7_ROW_OFFSET_B6_MASK           BITS(8,11)                                   /*!< XDC ADDRMAP7: ROW_OFFSET_B6 Bit Mask */  
#define XDC_ADDRMAP7_ROW_OFFSET_B6_OFS            8U                                          /*!< XDC ADDRMAP7: ROW_OFFSET_B6 Bit Offset */
#define XDC_ADDRMAP7_ROW_OFFSET_B6(regval)        (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< XDC ADDRMAP7: ROW_OFFSET_B6 Bit Value */  
#define XDC_ADDRMAP7_ROW_OFFSET_B7_MASK           BITS(12,15)                                   /*!< XDC ADDRMAP7: ROW_OFFSET_B7 Bit Mask */  
#define XDC_ADDRMAP7_ROW_OFFSET_B7_OFS            12U                                          /*!< XDC ADDRMAP7: ROW_OFFSET_B7 Bit Offset */
#define XDC_ADDRMAP7_ROW_OFFSET_B7(regval)        (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< XDC ADDRMAP7: ROW_OFFSET_B7 Bit Value */  
 
 /* ===== XDC ADDRMAP8 Register definition ===== */
#define XDC_ADDRMAP8_ROW_OFFSET_B8_MASK           BITS(0,3)                                   /*!< XDC ADDRMAP8: ROW_OFFSET_B8 Bit Mask */  
#define XDC_ADDRMAP8_ROW_OFFSET_B8_OFS            0U                                          /*!< XDC ADDRMAP8: ROW_OFFSET_B8 Bit Offset */
#define XDC_ADDRMAP8_ROW_OFFSET_B8(regval)        (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< XDC ADDRMAP8: ROW_OFFSET_B8 Bit Value */  
#define XDC_ADDRMAP8_ROW_OFFSET_B9_MASK           BITS(4,7)                                   /*!< XDC ADDRMAP8: ROW_OFFSET_B9 Bit Mask */  
#define XDC_ADDRMAP8_ROW_OFFSET_B9_OFS            4U                                          /*!< XDC ADDRMAP8: ROW_OFFSET_B9 Bit Offset */
#define XDC_ADDRMAP8_ROW_OFFSET_B9(regval)        (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< XDC ADDRMAP8: ROW_OFFSET_B9 Bit Value */  
#define XDC_ADDRMAP8_ROW_OFFSET_B10_MASK          BITS(8,11)                                   /*!< XDC ADDRMAP8: ROW_OFFSET_B10 Bit Mask */  
#define XDC_ADDRMAP8_ROW_OFFSET_B10_OFS           8U                                          /*!< XDC ADDRMAP8: ROW_OFFSET_B10 Bit Offset */
#define XDC_ADDRMAP8_ROW_OFFSET_B10(regval)       (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< XDC ADDRMAP8: ROW_OFFSET_B10 Bit Value */  
#define XDC_ADDRMAP8_ROW_OFFSET_B11_MASK          BITS(12,15)                                   /*!< XDC ADDRMAP8: ROW_OFFSET_B11 Bit Mask */  
#define XDC_ADDRMAP8_ROW_OFFSET_B11_OFS           12U                                          /*!< XDC ADDRMAP8: ROW_OFFSET_B11 Bit Offset */
#define XDC_ADDRMAP8_ROW_OFFSET_B11(regval)       (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< XDC ADDRMAP8: ROW_OFFSET_B11 Bit Value */  
 
 /* ===== XDC ADDRMAP9 Register definition ===== */
#define XDC_ADDRMAP9_ROW_OFFSET_B12_MASK          BITS(0,3)                                   /*!< XDC ADDRMAP9: ROW_OFFSET_B12 Bit Mask */  
#define XDC_ADDRMAP9_ROW_OFFSET_B12_OFS           0U                                          /*!< XDC ADDRMAP9: ROW_OFFSET_B12 Bit Offset */
#define XDC_ADDRMAP9_ROW_OFFSET_B12(regval)       (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< XDC ADDRMAP9: ROW_OFFSET_B12 Bit Value */  
#define XDC_ADDRMAP9_ROW_OFFSET_B13_MASK          BITS(4,7)                                   /*!< XDC ADDRMAP9: ROW_OFFSET_B13 Bit Mask */  
#define XDC_ADDRMAP9_ROW_OFFSET_B13_OFS           4U                                          /*!< XDC ADDRMAP9: ROW_OFFSET_B13 Bit Offset */
#define XDC_ADDRMAP9_ROW_OFFSET_B13(regval)       (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< XDC ADDRMAP9: ROW_OFFSET_B13 Bit Value */  
#define XDC_ADDRMAP9_ROW_OFFSET_B14_MASK          BITS(8,11)                                   /*!< XDC ADDRMAP9: ROW_OFFSET_B14 Bit Mask */  
#define XDC_ADDRMAP9_ROW_OFFSET_B14_OFS           8U                                          /*!< XDC ADDRMAP9: ROW_OFFSET_B14 Bit Offset */
#define XDC_ADDRMAP9_ROW_OFFSET_B14(regval)       (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< XDC ADDRMAP9: ROW_OFFSET_B14 Bit Value */  
#define XDC_ADDRMAP9_ROW_OFFSET_B15_MASK          BITS(12,15)                                   /*!< XDC ADDRMAP9: ROW_OFFSET_B15 Bit Mask */  
#define XDC_ADDRMAP9_ROW_OFFSET_B15_OFS           12U                                          /*!< XDC ADDRMAP9: ROW_OFFSET_B15 Bit Offset */
#define XDC_ADDRMAP9_ROW_OFFSET_B15(regval)       (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< XDC ADDRMAP9: ROW_OFFSET_B15 Bit Value */  
 
 /* ===== XDC ADDRMAP10 Register definition ===== */
#define XDC_ADDRMAP10_ROW_OFFSET_B16_MASK          BITS(0,3)                                   /*!< XDC ADDRMAP10: ROW_OFFSET_B16 Bit Mask */  
#define XDC_ADDRMAP10_ROW_OFFSET_B16_OFS           0U                                          /*!< XDC ADDRMAP10: ROW_OFFSET_B16 Bit Offset */
#define XDC_ADDRMAP10_ROW_OFFSET_B16(regval)       (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< XDC ADDRMAP10: ROW_OFFSET_B16 Bit Value */  
#define XDC_ADDRMAP10_ROW_OFFSET_B17_MASK          BITS(4,7)                                   /*!< XDC ADDRMAP10: ROW_OFFSET_B17 Bit Mask */  
#define XDC_ADDRMAP10_ROW_OFFSET_B17_OFS           4U                                          /*!< XDC ADDRMAP10: ROW_OFFSET_B17 Bit Offset */
#define XDC_ADDRMAP10_ROW_OFFSET_B17(regval)       (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< XDC ADDRMAP10: ROW_OFFSET_B17 Bit Value */  
 
 /* ===== XDC ODT_CTRL Register definition ===== */
#define XDC_ODT_CTRL_RANK0_WR_ODT_MASK            BITS(0,1)                                   /*!< XDC ODT CTRL: RANK0_WR_ODT Bit Mask */  
#define XDC_ODT_CTRL_RANK0_WR_ODT_OFS             0U                                          /*!< XDC ODT CTRL: RANK0_WR_ODT Bit Offset */
#define XDC_ODT_CTRL_RANK0_WR_ODT(regval)         (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< XDC ODT CTRL: RANK0_WR_ODT Bit Value */  
#define XDC_ODT_CTRL_RANK0_RD_ODT_MASK            BITS(2,3)                                   /*!< XDC ODT CTRL: RANK0_RD_ODT Bit Mask */  
#define XDC_ODT_CTRL_RANK0_RD_ODT_OFS             2U                                          /*!< XDC ODT CTRL: RANK0_RD_ODT Bit Offset */
#define XDC_ODT_CTRL_RANK0_RD_ODT(regval)         (BITS(2,3) & ((uint32_t)(regval) << 2))        /*!< XDC ODT CTRL: RANK0_RD_ODT Bit Value */  
#define XDC_ODT_CTRL_RANK1_WR_ODT_MASK            BITS(4,5)                                   /*!< XDC ODT CTRL: RANK1_WR_ODT Bit Mask */  
#define XDC_ODT_CTRL_RANK1_WR_ODT_OFS             4U                                          /*!< XDC ODT CTRL: RANK1_WR_ODT Bit Offset */
#define XDC_ODT_CTRL_RANK1_WR_ODT(regval)         (BITS(4,5) & ((uint32_t)(regval) << 4))        /*!< XDC ODT CTRL: RANK1_WR_ODT Bit Value */  
#define XDC_ODT_CTRL_RANK1_RD_ODT_MASK            BITS(6,7)                                   /*!< XDC ODT CTRL: RANK1_RD_ODT Bit Mask */  
#define XDC_ODT_CTRL_RANK1_RD_ODT_OFS             6U                                          /*!< XDC ODT CTRL: RANK1_RD_ODT Bit Offset */
#define XDC_ODT_CTRL_RANK1_RD_ODT(regval)         (BITS(6,7) & ((uint32_t)(regval) << 6))        /*!< XDC ODT CTRL: RANK1_RD_ODT Bit Value */  
 
 /* ===== XDC SCH_CTRL Register definition ===== */
#define XDC_SCH_CTRL_PRF_WR                       BIT(0)                                      /*!< Set it to make write operation has higher priority than read operation. */
#define XDC_SCH_CTRL_PRF_WR_OFS                   0U                                          /*!< XDC SCH CTRL: PRF_WR Bit Offset */
#define XDC_SCH_CTRL_PRF_WR_VAL(regval)               (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC SCH CTRL: PRF_WR Bit Value */  
#define XDC_SCH_CTRL_PGC_EN                       BIT(1)                                      /*!< If set this field to 1, bank is kept open only while there are page hit wirte/read command available in the CAM to that bank. This field is reserved for future use. */
#define XDC_SCH_CTRL_PGC_EN_OFS                   1U                                          /*!< XDC SCH CTRL: PGC_EN Bit Offset */
#define XDC_SCH_CTRL_PGC_EN_VAL(regval)               (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC SCH CTRL: PGC_EN Bit Value */  
#define XDC_SCH_CTRL_OPT_ACT                      BIT(2)                                      /*!< Optimize ACT scheduling for LPDDR4. This field is reserved for future use. */
#define XDC_SCH_CTRL_OPT_ACT_OFS                  2U                                          /*!< XDC SCH CTRL: OPT_ACT Bit Offset */
#define XDC_SCH_CTRL_OPT_ACT_VAL(regval)              (BIT(2) & ((uint32_t)(regval) << 2))        /*!< XDC SCH CTRL: OPT_ACT Bit Value */  
#define XDC_SCH_CTRL_RDWR_GAP_MASK                BITS(8,14)                                   /*!< XDC SCH CTRL: RDWR_GAP Bit Mask */  
#define XDC_SCH_CTRL_RDWR_GAP_OFS                 8U                                          /*!< XDC SCH CTRL: RDWR_GAP Bit Offset */
#define XDC_SCH_CTRL_RDWR_GAP(regval)             (BITS(8,14) & ((uint32_t)(regval) << 8))        /*!< XDC SCH CTRL: RDWR_GAP Bit Value */  
#define XDC_SCH_CTRL_PGC_TIME_MASK                BITS(16,23)                                   /*!< XDC SCH CTRL: PGC_TIME Bit Mask */  
#define XDC_SCH_CTRL_PGC_TIME_OFS                 16U                                          /*!< XDC SCH CTRL: PGC_TIME Bit Offset */
#define XDC_SCH_CTRL_PGC_TIME(regval)             (BITS(16,23) & ((uint32_t)(regval) << 16))        /*!< XDC SCH CTRL: PGC_TIME Bit Value */  
#define XDC_SCH_CTRL_MAX_RANK_WR_MASK             BITS(24,27)                                   /*!< XDC SCH CTRL: MAX_RANK_WR Bit Mask */  
#define XDC_SCH_CTRL_MAX_RANK_WR_OFS              24U                                          /*!< XDC SCH CTRL: MAX_RANK_WR Bit Offset */
#define XDC_SCH_CTRL_MAX_RANK_WR(regval)          (BITS(24,27) & ((uint32_t)(regval) << 24))        /*!< XDC SCH CTRL: MAX_RANK_WR Bit Value */  
#define XDC_SCH_CTRL_MAX_RANK_RD_MASK             BITS(28,31)                                   /*!< XDC SCH CTRL: MAX_RANK_RD Bit Mask */  
#define XDC_SCH_CTRL_MAX_RANK_RD_OFS              28U                                          /*!< XDC SCH CTRL: MAX_RANK_RD Bit Offset */
#define XDC_SCH_CTRL_MAX_RANK_RD(regval)          (BITS(28,31) & ((uint32_t)(regval) << 28))        /*!< XDC SCH CTRL: MAX_RANK_RD Bit Value */  
 
 /* ===== XDC RDCAM Register definition ===== */
#define XDC_RDCAM_RD_MAX_STR_MASK              BITS(0,15)                                   /*!< XDC RDCAM: RD_MAX_STR Bit Mask */  
#define XDC_RDCAM_RD_MAX_STR_OFS               0U                                          /*!< XDC RDCAM: RD_MAX_STR Bit Offset */
#define XDC_RDCAM_RD_MAX_STR(regval)           (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< XDC RDCAM: RD_MAX_STR Bit Value */  
#define XDC_RDCAM_RD_RUN_LEN_MASK              BITS(16,23)                                   /*!< XDC RDCAM: RD_RUN_LEN Bit Mask */  
#define XDC_RDCAM_RD_RUN_LEN_OFS               16U                                          /*!< XDC RDCAM: RD_RUN_LEN Bit Offset */
#define XDC_RDCAM_RD_RUN_LEN(regval)           (BITS(16,23) & ((uint32_t)(regval) << 16))        /*!< XDC RDCAM: RD_RUN_LEN Bit Value */  
 
 /* ===== XDC WRCAM Register definition ===== */
#define XDC_WRCAM_WR_MAX_STR_MASK              BITS(0,15)                                   /*!< XDC WRCAM: WR_MAX_STR Bit Mask */  
#define XDC_WRCAM_WR_MAX_STR_OFS               0U                                          /*!< XDC WRCAM: WR_MAX_STR Bit Offset */
#define XDC_WRCAM_WR_MAX_STR(regval)           (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< XDC WRCAM: WR_MAX_STR Bit Value */  
#define XDC_WRCAM_WR_RUN_LEN_MASK              BITS(16,23)                                   /*!< XDC WRCAM: WR_RUN_LEN Bit Mask */  
#define XDC_WRCAM_WR_RUN_LEN_OFS               16U                                          /*!< XDC WRCAM: WR_RUN_LEN Bit Offset */
#define XDC_WRCAM_WR_RUN_LEN(regval)           (BITS(16,23) & ((uint32_t)(regval) << 16))        /*!< XDC WRCAM: WR_RUN_LEN Bit Value */  
 
 /* ===== XDC STA Register definition ===== */
#define XDC_STA_GFSM_MODE                    BITS(0,2)                
#define XDC_STA_SRE_STATE                    BIT(3)                                      /*!< Reserved */
#define XDC_STA_SRF_STATE                    BITS(4,5)                
#define XDC_STA_SRF_TYPE                     BITS(6,7)                
 
 /* ===== XDC DEBUG0 Register definition ===== */
#define XDC_DEBUG0_DIS_MAX_RANK_RD              BIT(0)                                      /*!< If set, then disable optimized max_rank_rd feature. */
#define XDC_DEBUG0_DIS_MAX_RANK_RD_OFS          0U                                          /*!< XDC DEBUG0: DIS_MAX_RANK_RD Bit Offset */
#define XDC_DEBUG0_DIS_MAX_RANK_RD_VAL(regval)      (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC DEBUG0: DIS_MAX_RANK_RD Bit Value */  
#define XDC_DEBUG0_DIS_MAX_RANK_WR              BIT(1)                                      /*!< If set, then disable optimized max_rank_wr feature. */
#define XDC_DEBUG0_DIS_MAX_RANK_WR_OFS          1U                                          /*!< XDC DEBUG0: DIS_MAX_RANK_WR Bit Offset */
#define XDC_DEBUG0_DIS_MAX_RANK_WR_VAL(regval)      (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC DEBUG0: DIS_MAX_RANK_WR Bit Value */  
#define XDC_DEBUG0_DIS_DQ                       BIT(2)                                      /*!< If set, then there is no data will be read from CAM. */
#define XDC_DEBUG0_DIS_DQ_OFS                   2U                                          /*!< XDC DEBUG0: DIS_DQ Bit Offset */
#define XDC_DEBUG0_DIS_DQ_VAL(regval)               (BIT(2) & ((uint32_t)(regval) << 2))        /*!< XDC DEBUG0: DIS_DQ Bit Value */  
#define XDC_DEBUG0_DIS_XIF                      BIT(3)                                      /*!< If set, then incoming data will not be received by core module. */
#define XDC_DEBUG0_DIS_XIF_OFS                  3U                                          /*!< XDC DEBUG0: DIS_XIF Bit Offset */
#define XDC_DEBUG0_DIS_XIF_VAL(regval)              (BIT(3) & ((uint32_t)(regval) << 3))        /*!< XDC DEBUG0: DIS_XIF Bit Value */  
#define XDC_DEBUG0_REG_INIT_DONE                BIT(4)                                      /*!< Reserved */
#define XDC_DEBUG0_REG_INIT_DONE_OFS            4U                                          /*!< XDC DEBUG0: REG_INIT_DONE Bit Offset */
#define XDC_DEBUG0_REG_INIT_DONE_VAL(regval)        (BIT(4) & ((uint32_t)(regval) << 4))        /*!< XDC DEBUG0: REG_INIT_DONE Bit Value */  
#define XDC_DEBUG0_RSV0                         BIT(5)                                      /*!< Reserved */
#define XDC_DEBUG0_RSV0_OFS                     5U                                          /*!< XDC DEBUG0: RSV0 Bit Offset */
#define XDC_DEBUG0_RSV0_VAL(regval)                 (BIT(5) & ((uint32_t)(regval) << 5))        /*!< XDC DEBUG0: RSV0 Bit Value */  
#define XDC_DEBUG0_PS_DBG_SEL_MASK              BITS(8,11)                                   /*!< XDC DEBUG0: PS_DBG_SEL Bit Mask */  
#define XDC_DEBUG0_PS_DBG_SEL_OFS               8U                                          /*!< XDC DEBUG0: PS_DBG_SEL Bit Offset */
#define XDC_DEBUG0_PS_DBG_SEL(regval)           (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< XDC DEBUG0: PS_DBG_SEL Bit Value */  
#define XDC_DEBUG0_BM_DBG_SEL_MASK              BITS(16,20)                                   /*!< XDC DEBUG0: BM_DBG_SEL Bit Mask */  
#define XDC_DEBUG0_BM_DBG_SEL_OFS               16U                                          /*!< XDC DEBUG0: BM_DBG_SEL Bit Offset */
#define XDC_DEBUG0_BM_DBG_SEL(regval)           (BITS(16,20) & ((uint32_t)(regval) << 16))        /*!< XDC DEBUG0: BM_DBG_SEL Bit Value */  
 
 /* ===== XDC DEBUG1 Register definition ===== */
#define XDC_DEBUG1_ZQCS                         BIT(0)                                      /*!< Set this bit to request XDC to issue zqcs to DDR4 or ZQCAL/ZQLAT to LPDDR4. Note: The hardware automatically clears this bit after it has been synchronized to XDC clock domain. */
#define XDC_DEBUG1_ZQCS_OFS                     0U                                          /*!< XDC DEBUG1: ZQCS Bit Offset */
#define XDC_DEBUG1_ZQCS_VAL(regval)                 (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC DEBUG1: ZQCS Bit Value */  
#define XDC_DEBUG1_CTRLUPD                      BIT(1)                                      /*!< Set this bit to request XDC to issue ctrlupd to PHY. Note: The hardware automatically clears this bit after it has been synchronized to XDC clock domain. */
#define XDC_DEBUG1_CTRLUPD_OFS                  1U                                          /*!< XDC DEBUG1: CTRLUPD Bit Offset */
#define XDC_DEBUG1_CTRLUPD_VAL(regval)              (BIT(1) & ((uint32_t)(regval) << 1))        /*!< XDC DEBUG1: CTRLUPD Bit Value */  
#define XDC_DEBUG1_RANK0_REF                    BIT(4)                                      /*!< Set this bit to request XDC to issue refresh to rank0. This operation can be performed only when REF_CTRL.dis_auto_ref = 1. Note: The hardware automatically clears this bit after it has been synchronized to XDC clock domain. */
#define XDC_DEBUG1_RANK0_REF_OFS                4U                                          /*!< XDC DEBUG1: RANK0_REF Bit Offset */
#define XDC_DEBUG1_RANK0_REF_VAL(regval)            (BIT(4) & ((uint32_t)(regval) << 4))        /*!< XDC DEBUG1: RANK0_REF Bit Value */  
#define XDC_DEBUG1_RANK1_REF                    BIT(5)                                      /*!< Set this bit to request XDC to issue refresh to rank1. This operation can be performed only when REF_CTRL.dis_auto_ref = 1. Note: The hardware automatically clears this bit after it has been synchronized to XDC clock domain. */
#define XDC_DEBUG1_RANK1_REF_OFS                5U                                          /*!< XDC DEBUG1: RANK1_REF Bit Offset */
#define XDC_DEBUG1_RANK1_REF_VAL(regval)            (BIT(5) & ((uint32_t)(regval) << 5))        /*!< XDC DEBUG1: RANK1_REF Bit Value */  
 
 /* ===== XDC DEBUG2 Register definition ===== */
#define XDC_DEBUG2_ZQCS_BUSY                    BIT(0)                                      /*!< Indicate that zqcs request has been received and has not done when it assert. */
#define XDC_DEBUG2_CTRLUPD_BUSY                 BIT(1)                                      /*!< Indicate that ctrlupd request has been received and has not done when it assert. */
#define XDC_DEBUG2_WR_QUE_EMPTY                 BIT(2)                                      /*!< Indicate the write cam and write buffer are both empty. */
#define XDC_DEBUG2_RD_QUE_EMPTY                 BIT(3)                                      /*!< Indicate the read cam and read buffer are both empty. */
#define XDC_DEBUG2_WR_BUF_EMPTY                 BIT(4)                                      /*!< Indicate the write buffer is empty. */
#define XDC_DEBUG2_RD_BUF_EMPTY                 BIT(5)                                      /*!< Indicate the read buffer is empty. */
#define XDC_DEBUG2_WR_STALL                     BIT(6)                                      /*!< Indicate that write channel has been stalled, used for debug. */
#define XDC_DEBUG2_RD_STALL                     BIT(7)                                      /*!< Indicate that read channel has been stalled, used for debug. */
#define XDC_DEBUG2_RANK0_REF_BUSY               BIT(8)                                      /*!< Indicate that refresh to rank0 has been received and has not done when it assert. */
#define XDC_DEBUG2_RANK1_REF_BUSY               BIT(9)                                      /*!< Indicate that refresh to rank1 has been received and has not done when it assert. */
#define XDC_DEBUG2_MRR_DATA_RDY                 BIT(12)                                      /*!< Indicate that MRR/MPR data is ready. */
 
 /* ===== XDC DEBUG3 Register definition ===== */
#define XDC_DEBUG3_MRR_DATA_BYTE0               BITS(0,7)                
#define XDC_DEBUG3_MRR_DATA_BYTE1               BITS(8,15)                
 
 /* ===== XDC DEBUG6 Register definition ===== */
#define XDC_DEBUG6_IP_DBG                       BITS(0,31)                
 
 /* ===== XDC DEBUG7 Register definition ===== */
#define XDC_DEBUG7_FS_WR_DBG                    BITS(0,31)                
 
 /* ===== XDC DEBUG8 Register definition ===== */
#define XDC_DEBUG8_FS_RD_DBG                    BITS(0,31)                
 
 /* ===== XDC DEBUG9 Register definition ===== */
#define XDC_DEBUG9_BM_DBG                       BITS(0,31)                
 
 /* ===== XDC DEBUG10 Register definition ===== */
#define XDC_DEBUG10_PS_DBG                       BITS(0,31)                
 
 /* ===== XDC REF_TMG0_FFC0 Register definition ===== */
#define XDC_REF_TMG0_FFC0_REF_BURST_FFC0_MASK          BITS(0,5)                                   /*!< XDC REF TMG0 FFC0: REF_BURST_FFC0 Bit Mask */  
#define XDC_REF_TMG0_FFC0_REF_BURST_FFC0_OFS           0U                                          /*!< XDC REF TMG0 FFC0: REF_BURST_FFC0 Bit Offset */
#define XDC_REF_TMG0_FFC0_REF_BURST_FFC0(regval)       (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC REF TMG0 FFC0: REF_BURST_FFC0 Bit Value */  
#define XDC_REF_TMG0_FFC0_REF_GAP_FFC0_MASK            BITS(6,10)                                   /*!< XDC REF TMG0 FFC0: REF_GAP_FFC0 Bit Mask */  
#define XDC_REF_TMG0_FFC0_REF_GAP_FFC0_OFS             6U                                          /*!< XDC REF TMG0 FFC0: REF_GAP_FFC0 Bit Offset */
#define XDC_REF_TMG0_FFC0_REF_GAP_FFC0(regval)         (BITS(6,10) & ((uint32_t)(regval) << 6))        /*!< XDC REF TMG0 FFC0: REF_GAP_FFC0 Bit Value */  
#define XDC_REF_TMG0_FFC0_REF_MARGIN_FFC0_MASK         BITS(11,14)                                   /*!< XDC REF TMG0 FFC0: REF_MARGIN_FFC0 Bit Mask */  
#define XDC_REF_TMG0_FFC0_REF_MARGIN_FFC0_OFS          11U                                          /*!< XDC REF TMG0 FFC0: REF_MARGIN_FFC0 Bit Offset */
#define XDC_REF_TMG0_FFC0_REF_MARGIN_FFC0(regval)      (BITS(11,14) & ((uint32_t)(regval) << 11))        /*!< XDC REF TMG0 FFC0: REF_MARGIN_FFC0 Bit Value */  
#define XDC_REF_TMG0_FFC0_REF_MODE_FFC0_MASK           BITS(15,17)                                   /*!< XDC REF TMG0 FFC0: REF_MODE_FFC0 Bit Mask */  
#define XDC_REF_TMG0_FFC0_REF_MODE_FFC0_OFS            15U                                          /*!< XDC REF TMG0 FFC0: REF_MODE_FFC0 Bit Offset */
#define XDC_REF_TMG0_FFC0_REF_MODE_FFC0(regval)        (BITS(15,17) & ((uint32_t)(regval) << 15))        /*!< XDC REF TMG0 FFC0: REF_MODE_FFC0 Bit Value */  
 
 /* ===== XDC REF_TMG1_FFC0 Register definition ===== */
#define XDC_REF_TMG1_FFC0_REF_TIMER0_VAL_FFC0_MASK     BITS(0,11)                                   /*!< XDC REF TMG1 FFC0: REF_TIMER0_VAL_FFC0 Bit Mask */  
#define XDC_REF_TMG1_FFC0_REF_TIMER0_VAL_FFC0_OFS      0U                                          /*!< XDC REF TMG1 FFC0: REF_TIMER0_VAL_FFC0 Bit Offset */
#define XDC_REF_TMG1_FFC0_REF_TIMER0_VAL_FFC0(regval)  (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< XDC REF TMG1 FFC0: REF_TIMER0_VAL_FFC0 Bit Value */  
#define XDC_REF_TMG1_FFC0_REF_TIMER1_VAL_FFC0_MASK     BITS(12,23)                                   /*!< XDC REF TMG1 FFC0: REF_TIMER1_VAL_FFC0 Bit Mask */  
#define XDC_REF_TMG1_FFC0_REF_TIMER1_VAL_FFC0_OFS      12U                                          /*!< XDC REF TMG1 FFC0: REF_TIMER1_VAL_FFC0 Bit Offset */
#define XDC_REF_TMG1_FFC0_REF_TIMER1_VAL_FFC0(regval)  (BITS(12,23) & ((uint32_t)(regval) << 12))        /*!< XDC REF TMG1 FFC0: REF_TIMER1_VAL_FFC0 Bit Value */  
 
 /* ===== XDC REF_TMG3_FFC0 Register definition ===== */
#define XDC_REF_TMG3_FFC0_T_RFC_MIN_FFC0_MASK          BITS(0,9)                                   /*!< XDC REF TMG3 FFC0: T_RFC_MIN_FFC0 Bit Mask */  
#define XDC_REF_TMG3_FFC0_T_RFC_MIN_FFC0_OFS           0U                                          /*!< XDC REF TMG3 FFC0: T_RFC_MIN_FFC0 Bit Offset */
#define XDC_REF_TMG3_FFC0_T_RFC_MIN_FFC0(regval)       (BITS(0,9) & ((uint32_t)(regval) << 0))        /*!< XDC REF TMG3 FFC0: T_RFC_MIN_FFC0 Bit Value */  
#define XDC_REF_TMG3_FFC0_T_REFI_VAL_FFC0_MASK         BITS(10,21)                                   /*!< XDC REF TMG3 FFC0: T_REFI_VAL_FFC0 Bit Mask */  
#define XDC_REF_TMG3_FFC0_T_REFI_VAL_FFC0_OFS          10U                                          /*!< XDC REF TMG3 FFC0: T_REFI_VAL_FFC0 Bit Offset */
#define XDC_REF_TMG3_FFC0_T_REFI_VAL_FFC0(regval)      (BITS(10,21) & ((uint32_t)(regval) << 10))        /*!< XDC REF TMG3 FFC0: T_REFI_VAL_FFC0 Bit Value */  
#define XDC_REF_TMG3_FFC0_T_PBR2PBR_FFC0_MASK          BITS(22,29)                                   /*!< XDC REF TMG3 FFC0: T_PBR2PBR_FFC0 Bit Mask */  
#define XDC_REF_TMG3_FFC0_T_PBR2PBR_FFC0_OFS           22U                                          /*!< XDC REF TMG3 FFC0: T_PBR2PBR_FFC0 Bit Offset */
#define XDC_REF_TMG3_FFC0_T_PBR2PBR_FFC0(regval)       (BITS(22,29) & ((uint32_t)(regval) << 22))        /*!< XDC REF TMG3 FFC0: T_PBR2PBR_FFC0 Bit Value */  
#define XDC_REF_TMG3_FFC0_T_REFI_X1_SEL_FFC0           BIT(30)                                      /*!< Indicates the time unit of t_refi_val. 0 - use x1 XDC clock cycle as time unit 1 - use x32 XDC clock cycle as time unit */
#define XDC_REF_TMG3_FFC0_T_REFI_X1_SEL_FFC0_OFS       30U                                          /*!< XDC REF TMG3 FFC0: T_REFI_X1_SEL_FFC0 Bit Offset */
#define XDC_REF_TMG3_FFC0_T_REFI_X1_SEL_FFC0_VAL(regval)   (BIT(30) & ((uint32_t)(regval) << 30))        /*!< XDC REF TMG3 FFC0: T_REFI_X1_SEL_FFC0 Bit Value */  
 
 /* ===== XDC MR_VAL0_FFC0 Register definition ===== */
#define XDC_MR_VAL0_FFC0_MR0_FFC0_MASK                BITS(0,15)                                   /*!< XDC MR VAL0 FFC0: MR0_FFC0 Bit Mask */  
#define XDC_MR_VAL0_FFC0_MR0_FFC0_OFS                 0U                                          /*!< XDC MR VAL0 FFC0: MR0_FFC0 Bit Offset */
#define XDC_MR_VAL0_FFC0_MR0_FFC0(regval)             (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< XDC MR VAL0 FFC0: MR0_FFC0 Bit Value */  
#define XDC_MR_VAL0_FFC0_MR1_FFC0_MASK                BITS(16,31)                                   /*!< XDC MR VAL0 FFC0: MR1_FFC0 Bit Mask */  
#define XDC_MR_VAL0_FFC0_MR1_FFC0_OFS                 16U                                          /*!< XDC MR VAL0 FFC0: MR1_FFC0 Bit Offset */
#define XDC_MR_VAL0_FFC0_MR1_FFC0(regval)             (BITS(16,31) & ((uint32_t)(regval) << 16))        /*!< XDC MR VAL0 FFC0: MR1_FFC0 Bit Value */  
 
 /* ===== XDC MR_VAL1_FFC0 Register definition ===== */
#define XDC_MR_VAL1_FFC0_MR2_FFC0_MASK                BITS(0,15)                                   /*!< XDC MR VAL1 FFC0: MR2_FFC0 Bit Mask */  
#define XDC_MR_VAL1_FFC0_MR2_FFC0_OFS                 0U                                          /*!< XDC MR VAL1 FFC0: MR2_FFC0 Bit Offset */
#define XDC_MR_VAL1_FFC0_MR2_FFC0(regval)             (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< XDC MR VAL1 FFC0: MR2_FFC0 Bit Value */  
#define XDC_MR_VAL1_FFC0_MR3_FFC0_MASK                BITS(16,31)                                   /*!< XDC MR VAL1 FFC0: MR3_FFC0 Bit Mask */  
#define XDC_MR_VAL1_FFC0_MR3_FFC0_OFS                 16U                                          /*!< XDC MR VAL1 FFC0: MR3_FFC0 Bit Offset */
#define XDC_MR_VAL1_FFC0_MR3_FFC0(regval)             (BITS(16,31) & ((uint32_t)(regval) << 16))        /*!< XDC MR VAL1 FFC0: MR3_FFC0 Bit Value */  
 
 /* ===== XDC MR_VAL2_FFC0 Register definition ===== */
#define XDC_MR_VAL2_FFC0_MR4_FFC0_MASK                BITS(0,15)                                   /*!< XDC MR VAL2 FFC0: MR4_FFC0 Bit Mask */  
#define XDC_MR_VAL2_FFC0_MR4_FFC0_OFS                 0U                                          /*!< XDC MR VAL2 FFC0: MR4_FFC0 Bit Offset */
#define XDC_MR_VAL2_FFC0_MR4_FFC0(regval)             (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< XDC MR VAL2 FFC0: MR4_FFC0 Bit Value */  
#define XDC_MR_VAL2_FFC0_MR5_FFC0_MASK                BITS(16,31)                                   /*!< XDC MR VAL2 FFC0: MR5_FFC0 Bit Mask */  
#define XDC_MR_VAL2_FFC0_MR5_FFC0_OFS                 16U                                          /*!< XDC MR VAL2 FFC0: MR5_FFC0 Bit Offset */
#define XDC_MR_VAL2_FFC0_MR5_FFC0(regval)             (BITS(16,31) & ((uint32_t)(regval) << 16))        /*!< XDC MR VAL2 FFC0: MR5_FFC0 Bit Value */  
 
 /* ===== XDC MR_VAL3_FFC0 Register definition ===== */
#define XDC_MR_VAL3_FFC0_MR6_FFC0_MASK                BITS(0,15)                                   /*!< XDC MR VAL3 FFC0: MR6_FFC0 Bit Mask */  
#define XDC_MR_VAL3_FFC0_MR6_FFC0_OFS                 0U                                          /*!< XDC MR VAL3 FFC0: MR6_FFC0 Bit Offset */
#define XDC_MR_VAL3_FFC0_MR6_FFC0(regval)             (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< XDC MR VAL3 FFC0: MR6_FFC0 Bit Value */  
 
 /* ===== XDC RANK_TMG0_FFC0 Register definition ===== */
#define XDC_RANK_TMG0_FFC0_DIFF_RANK_RD_GAP_FFC0_MASK    BITS(0,4)                                   /*!< XDC RANK TMG0 FFC0: DIFF_RANK_RD_GAP_FFC0 Bit Mask */  
#define XDC_RANK_TMG0_FFC0_DIFF_RANK_RD_GAP_FFC0_OFS     0U                                          /*!< XDC RANK TMG0 FFC0: DIFF_RANK_RD_GAP_FFC0 Bit Offset */
#define XDC_RANK_TMG0_FFC0_DIFF_RANK_RD_GAP_FFC0(regval) (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC RANK TMG0 FFC0: DIFF_RANK_RD_GAP_FFC0 Bit Value */  
#define XDC_RANK_TMG0_FFC0_DIFF_RANK_WR_GAP_FFC0_MASK    BITS(5,9)                                   /*!< XDC RANK TMG0 FFC0: DIFF_RANK_WR_GAP_FFC0 Bit Mask */  
#define XDC_RANK_TMG0_FFC0_DIFF_RANK_WR_GAP_FFC0_OFS     5U                                          /*!< XDC RANK TMG0 FFC0: DIFF_RANK_WR_GAP_FFC0 Bit Offset */
#define XDC_RANK_TMG0_FFC0_DIFF_RANK_WR_GAP_FFC0(regval) (BITS(5,9) & ((uint32_t)(regval) << 5))        /*!< XDC RANK TMG0 FFC0: DIFF_RANK_WR_GAP_FFC0 Bit Value */  
#define XDC_RANK_TMG0_FFC0_DIFF_RANK_T_WR2RD_FFC0_MASK    BITS(16,21)                                   /*!< XDC RANK TMG0 FFC0: DIFF_RANK_T_WR2RD_FFC0 Bit Mask */  
#define XDC_RANK_TMG0_FFC0_DIFF_RANK_T_WR2RD_FFC0_OFS     16U                                          /*!< XDC RANK TMG0 FFC0: DIFF_RANK_T_WR2RD_FFC0 Bit Offset */
#define XDC_RANK_TMG0_FFC0_DIFF_RANK_T_WR2RD_FFC0(regval) (BITS(16,21) & ((uint32_t)(regval) << 16))        /*!< XDC RANK TMG0 FFC0: DIFF_RANK_T_WR2RD_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG0_FFC0 Register definition ===== */
#define XDC_DRAM_TMG0_FFC0_T_RAS_MIN_FFC0_MASK          BITS(0,5)                                   /*!< XDC DRAM TMG0 FFC0: T_RAS_MIN_FFC0 Bit Mask */  
#define XDC_DRAM_TMG0_FFC0_T_RAS_MIN_FFC0_OFS           0U                                          /*!< XDC DRAM TMG0 FFC0: T_RAS_MIN_FFC0 Bit Offset */
#define XDC_DRAM_TMG0_FFC0_T_RAS_MIN_FFC0(regval)       (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG0 FFC0: T_RAS_MIN_FFC0 Bit Value */  
#define XDC_DRAM_TMG0_FFC0_T_RAS_MAX_FFC0_MASK          BITS(6,12)                                   /*!< XDC DRAM TMG0 FFC0: T_RAS_MAX_FFC0 Bit Mask */  
#define XDC_DRAM_TMG0_FFC0_T_RAS_MAX_FFC0_OFS           6U                                          /*!< XDC DRAM TMG0 FFC0: T_RAS_MAX_FFC0 Bit Offset */
#define XDC_DRAM_TMG0_FFC0_T_RAS_MAX_FFC0(regval)       (BITS(6,12) & ((uint32_t)(regval) << 6))        /*!< XDC DRAM TMG0 FFC0: T_RAS_MAX_FFC0 Bit Value */  
#define XDC_DRAM_TMG0_FFC0_T_FAW_FFC0_MASK              BITS(13,18)                                   /*!< XDC DRAM TMG0 FFC0: T_FAW_FFC0 Bit Mask */  
#define XDC_DRAM_TMG0_FFC0_T_FAW_FFC0_OFS               13U                                          /*!< XDC DRAM TMG0 FFC0: T_FAW_FFC0 Bit Offset */
#define XDC_DRAM_TMG0_FFC0_T_FAW_FFC0(regval)           (BITS(13,18) & ((uint32_t)(regval) << 13))        /*!< XDC DRAM TMG0 FFC0: T_FAW_FFC0 Bit Value */  
#define XDC_DRAM_TMG0_FFC0_T_WR2PRE_FFC0_MASK           BITS(19,25)                                   /*!< XDC DRAM TMG0 FFC0: T_WR2PRE_FFC0 Bit Mask */  
#define XDC_DRAM_TMG0_FFC0_T_WR2PRE_FFC0_OFS            19U                                          /*!< XDC DRAM TMG0 FFC0: T_WR2PRE_FFC0 Bit Offset */
#define XDC_DRAM_TMG0_FFC0_T_WR2PRE_FFC0(regval)        (BITS(19,25) & ((uint32_t)(regval) << 19))        /*!< XDC DRAM TMG0 FFC0: T_WR2PRE_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG1_FFC0 Register definition ===== */
#define XDC_DRAM_TMG1_FFC0_T_RC_FFC0_MASK               BITS(0,6)                                   /*!< XDC DRAM TMG1 FFC0: T_RC_FFC0 Bit Mask */  
#define XDC_DRAM_TMG1_FFC0_T_RC_FFC0_OFS                0U                                          /*!< XDC DRAM TMG1 FFC0: T_RC_FFC0 Bit Offset */
#define XDC_DRAM_TMG1_FFC0_T_RC_FFC0(regval)            (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG1 FFC0: T_RC_FFC0 Bit Value */  
#define XDC_DRAM_TMG1_FFC0_T_RD2PRE_FFC0_MASK           BITS(7,12)                                   /*!< XDC DRAM TMG1 FFC0: T_RD2PRE_FFC0 Bit Mask */  
#define XDC_DRAM_TMG1_FFC0_T_RD2PRE_FFC0_OFS            7U                                          /*!< XDC DRAM TMG1 FFC0: T_RD2PRE_FFC0 Bit Offset */
#define XDC_DRAM_TMG1_FFC0_T_RD2PRE_FFC0(regval)        (BITS(7,12) & ((uint32_t)(regval) << 7))        /*!< XDC DRAM TMG1 FFC0: T_RD2PRE_FFC0 Bit Value */  
#define XDC_DRAM_TMG1_FFC0_T_XP_FFC0_MASK               BITS(13,17)                                   /*!< XDC DRAM TMG1 FFC0: T_XP_FFC0 Bit Mask */  
#define XDC_DRAM_TMG1_FFC0_T_XP_FFC0_OFS                13U                                          /*!< XDC DRAM TMG1 FFC0: T_XP_FFC0 Bit Offset */
#define XDC_DRAM_TMG1_FFC0_T_XP_FFC0(regval)            (BITS(13,17) & ((uint32_t)(regval) << 13))        /*!< XDC DRAM TMG1 FFC0: T_XP_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG2_FFC0 Register definition ===== */
#define XDC_DRAM_TMG2_FFC0_T_WR2RD_FFC0_MASK            BITS(0,5)                                   /*!< XDC DRAM TMG2 FFC0: T_WR2RD_FFC0 Bit Mask */  
#define XDC_DRAM_TMG2_FFC0_T_WR2RD_FFC0_OFS             0U                                          /*!< XDC DRAM TMG2 FFC0: T_WR2RD_FFC0 Bit Offset */
#define XDC_DRAM_TMG2_FFC0_T_WR2RD_FFC0(regval)         (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG2 FFC0: T_WR2RD_FFC0 Bit Value */  
#define XDC_DRAM_TMG2_FFC0_T_RD2WR_FFC0_MASK            BITS(6,11)                                   /*!< XDC DRAM TMG2 FFC0: T_RD2WR_FFC0 Bit Mask */  
#define XDC_DRAM_TMG2_FFC0_T_RD2WR_FFC0_OFS             6U                                          /*!< XDC DRAM TMG2 FFC0: T_RD2WR_FFC0 Bit Offset */
#define XDC_DRAM_TMG2_FFC0_T_RD2WR_FFC0(regval)         (BITS(6,11) & ((uint32_t)(regval) << 6))        /*!< XDC DRAM TMG2 FFC0: T_RD2WR_FFC0 Bit Value */  
#define XDC_DRAM_TMG2_FFC0_T_WR_LAT_FFC0_MASK           BITS(12,17)                                   /*!< XDC DRAM TMG2 FFC0: T_WR_LAT_FFC0 Bit Mask */  
#define XDC_DRAM_TMG2_FFC0_T_WR_LAT_FFC0_OFS            12U                                          /*!< XDC DRAM TMG2 FFC0: T_WR_LAT_FFC0 Bit Offset */
#define XDC_DRAM_TMG2_FFC0_T_WR_LAT_FFC0(regval)        (BITS(12,17) & ((uint32_t)(regval) << 12))        /*!< XDC DRAM TMG2 FFC0: T_WR_LAT_FFC0 Bit Value */  
#define XDC_DRAM_TMG2_FFC0_T_RD_LAT_FFC0_MASK           BITS(18,23)                                   /*!< XDC DRAM TMG2 FFC0: T_RD_LAT_FFC0 Bit Mask */  
#define XDC_DRAM_TMG2_FFC0_T_RD_LAT_FFC0_OFS            18U                                          /*!< XDC DRAM TMG2 FFC0: T_RD_LAT_FFC0 Bit Offset */
#define XDC_DRAM_TMG2_FFC0_T_RD_LAT_FFC0(regval)        (BITS(18,23) & ((uint32_t)(regval) << 18))        /*!< XDC DRAM TMG2 FFC0: T_RD_LAT_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG3_FFC0 Register definition ===== */
#define XDC_DRAM_TMG3_FFC0_T_MOD_FFC0_MASK              BITS(0,9)                                   /*!< XDC DRAM TMG3 FFC0: T_MOD_FFC0 Bit Mask */  
#define XDC_DRAM_TMG3_FFC0_T_MOD_FFC0_OFS               0U                                          /*!< XDC DRAM TMG3 FFC0: T_MOD_FFC0 Bit Offset */
#define XDC_DRAM_TMG3_FFC0_T_MOD_FFC0(regval)           (BITS(0,9) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG3 FFC0: T_MOD_FFC0 Bit Value */  
#define XDC_DRAM_TMG3_FFC0_T_MRD_FFC0_MASK              BITS(10,19)                                   /*!< XDC DRAM TMG3 FFC0: T_MRD_FFC0 Bit Mask */  
#define XDC_DRAM_TMG3_FFC0_T_MRD_FFC0_OFS               10U                                          /*!< XDC DRAM TMG3 FFC0: T_MRD_FFC0 Bit Offset */
#define XDC_DRAM_TMG3_FFC0_T_MRD_FFC0(regval)           (BITS(10,19) & ((uint32_t)(regval) << 10))        /*!< XDC DRAM TMG3 FFC0: T_MRD_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG4_FFC0 Register definition ===== */
#define XDC_DRAM_TMG4_FFC0_T_RP_FFC0_MASK               BITS(0,4)                                   /*!< XDC DRAM TMG4 FFC0: T_RP_FFC0 Bit Mask */  
#define XDC_DRAM_TMG4_FFC0_T_RP_FFC0_OFS                0U                                          /*!< XDC DRAM TMG4 FFC0: T_RP_FFC0 Bit Offset */
#define XDC_DRAM_TMG4_FFC0_T_RP_FFC0(regval)            (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG4 FFC0: T_RP_FFC0 Bit Value */  
#define XDC_DRAM_TMG4_FFC0_T_RRD_FFC0_MASK              BITS(5,9)                                   /*!< XDC DRAM TMG4 FFC0: T_RRD_FFC0 Bit Mask */  
#define XDC_DRAM_TMG4_FFC0_T_RRD_FFC0_OFS               5U                                          /*!< XDC DRAM TMG4 FFC0: T_RRD_FFC0 Bit Offset */
#define XDC_DRAM_TMG4_FFC0_T_RRD_FFC0(regval)           (BITS(5,9) & ((uint32_t)(regval) << 5))        /*!< XDC DRAM TMG4 FFC0: T_RRD_FFC0 Bit Value */  
#define XDC_DRAM_TMG4_FFC0_T_CCD_FFC0_MASK              BITS(10,13)                                   /*!< XDC DRAM TMG4 FFC0: T_CCD_FFC0 Bit Mask */  
#define XDC_DRAM_TMG4_FFC0_T_CCD_FFC0_OFS               10U                                          /*!< XDC DRAM TMG4 FFC0: T_CCD_FFC0 Bit Offset */
#define XDC_DRAM_TMG4_FFC0_T_CCD_FFC0(regval)           (BITS(10,13) & ((uint32_t)(regval) << 10))        /*!< XDC DRAM TMG4 FFC0: T_CCD_FFC0 Bit Value */  
#define XDC_DRAM_TMG4_FFC0_T_RCD_FFC0_MASK              BITS(14,18)                                   /*!< XDC DRAM TMG4 FFC0: T_RCD_FFC0 Bit Mask */  
#define XDC_DRAM_TMG4_FFC0_T_RCD_FFC0_OFS               14U                                          /*!< XDC DRAM TMG4 FFC0: T_RCD_FFC0 Bit Offset */
#define XDC_DRAM_TMG4_FFC0_T_RCD_FFC0(regval)           (BITS(14,18) & ((uint32_t)(regval) << 14))        /*!< XDC DRAM TMG4 FFC0: T_RCD_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG5_FFC0 Register definition ===== */
#define XDC_DRAM_TMG5_FFC0_T_CKE_FFC0_MASK              BITS(0,4)                                   /*!< XDC DRAM TMG5 FFC0: T_CKE_FFC0 Bit Mask */  
#define XDC_DRAM_TMG5_FFC0_T_CKE_FFC0_OFS               0U                                          /*!< XDC DRAM TMG5 FFC0: T_CKE_FFC0 Bit Offset */
#define XDC_DRAM_TMG5_FFC0_T_CKE_FFC0(regval)           (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG5 FFC0: T_CKE_FFC0 Bit Value */  
#define XDC_DRAM_TMG5_FFC0_T_CKESR_FFC0_MASK            BITS(5,10)                                   /*!< XDC DRAM TMG5 FFC0: T_CKESR_FFC0 Bit Mask */  
#define XDC_DRAM_TMG5_FFC0_T_CKESR_FFC0_OFS             5U                                          /*!< XDC DRAM TMG5 FFC0: T_CKESR_FFC0 Bit Offset */
#define XDC_DRAM_TMG5_FFC0_T_CKESR_FFC0(regval)         (BITS(5,10) & ((uint32_t)(regval) << 5))        /*!< XDC DRAM TMG5 FFC0: T_CKESR_FFC0 Bit Value */  
#define XDC_DRAM_TMG5_FFC0_T_CKSRE_FFC0_MASK            BITS(11,17)                                   /*!< XDC DRAM TMG5 FFC0: T_CKSRE_FFC0 Bit Mask */  
#define XDC_DRAM_TMG5_FFC0_T_CKSRE_FFC0_OFS             11U                                          /*!< XDC DRAM TMG5 FFC0: T_CKSRE_FFC0 Bit Offset */
#define XDC_DRAM_TMG5_FFC0_T_CKSRE_FFC0(regval)         (BITS(11,17) & ((uint32_t)(regval) << 11))        /*!< XDC DRAM TMG5 FFC0: T_CKSRE_FFC0 Bit Value */  
#define XDC_DRAM_TMG5_FFC0_T_CKSRX_FFC0_MASK            BITS(18,21)                                   /*!< XDC DRAM TMG5 FFC0: T_CKSRX_FFC0 Bit Mask */  
#define XDC_DRAM_TMG5_FFC0_T_CKSRX_FFC0_OFS             18U                                          /*!< XDC DRAM TMG5 FFC0: T_CKSRX_FFC0 Bit Offset */
#define XDC_DRAM_TMG5_FFC0_T_CKSRX_FFC0(regval)         (BITS(18,21) & ((uint32_t)(regval) << 18))        /*!< XDC DRAM TMG5 FFC0: T_CKSRX_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG6_FFC0 Register definition ===== */
#define XDC_DRAM_TMG6_FFC0_T_XS_FFC0_MASK               BITS(0,6)                                   /*!< XDC DRAM TMG6 FFC0: T_XS_FFC0 Bit Mask */  
#define XDC_DRAM_TMG6_FFC0_T_XS_FFC0_OFS                0U                                          /*!< XDC DRAM TMG6 FFC0: T_XS_FFC0 Bit Offset */
#define XDC_DRAM_TMG6_FFC0_T_XS_FFC0(regval)            (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG6 FFC0: T_XS_FFC0 Bit Value */  
#define XDC_DRAM_TMG6_FFC0_T_XS_DLL_FFC0_MASK           BITS(7,13)                                   /*!< XDC DRAM TMG6 FFC0: T_XS_DLL_FFC0 Bit Mask */  
#define XDC_DRAM_TMG6_FFC0_T_XS_DLL_FFC0_OFS            7U                                          /*!< XDC DRAM TMG6 FFC0: T_XS_DLL_FFC0 Bit Offset */
#define XDC_DRAM_TMG6_FFC0_T_XS_DLL_FFC0(regval)        (BITS(7,13) & ((uint32_t)(regval) << 7))        /*!< XDC DRAM TMG6 FFC0: T_XS_DLL_FFC0 Bit Value */  
#define XDC_DRAM_TMG6_FFC0_T_XS_ABORT_FFC0_MASK         BITS(14,20)                                   /*!< XDC DRAM TMG6 FFC0: T_XS_ABORT_FFC0 Bit Mask */  
#define XDC_DRAM_TMG6_FFC0_T_XS_ABORT_FFC0_OFS          14U                                          /*!< XDC DRAM TMG6 FFC0: T_XS_ABORT_FFC0 Bit Offset */
#define XDC_DRAM_TMG6_FFC0_T_XS_ABORT_FFC0(regval)      (BITS(14,20) & ((uint32_t)(regval) << 14))        /*!< XDC DRAM TMG6 FFC0: T_XS_ABORT_FFC0 Bit Value */  
#define XDC_DRAM_TMG6_FFC0_T_XS_FAST_FFC0_MASK          BITS(21,27)                                   /*!< XDC DRAM TMG6 FFC0: T_XS_FAST_FFC0 Bit Mask */  
#define XDC_DRAM_TMG6_FFC0_T_XS_FAST_FFC0_OFS           21U                                          /*!< XDC DRAM TMG6 FFC0: T_XS_FAST_FFC0 Bit Offset */
#define XDC_DRAM_TMG6_FFC0_T_XS_FAST_FFC0(regval)       (BITS(21,27) & ((uint32_t)(regval) << 21))        /*!< XDC DRAM TMG6 FFC0: T_XS_FAST_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG7_FFC0 Register definition ===== */
#define XDC_DRAM_TMG7_FFC0_T_WR_MPR_FFC0_MASK           BITS(0,5)                                   /*!< XDC DRAM TMG7 FFC0: T_WR_MPR_FFC0 Bit Mask */  
#define XDC_DRAM_TMG7_FFC0_T_WR_MPR_FFC0_OFS            0U                                          /*!< XDC DRAM TMG7 FFC0: T_WR_MPR_FFC0 Bit Offset */
#define XDC_DRAM_TMG7_FFC0_T_WR_MPR_FFC0(regval)        (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG7 FFC0: T_WR_MPR_FFC0 Bit Value */  
#define XDC_DRAM_TMG7_FFC0_T_MRD_PDA_FFC0_MASK          BITS(6,10)                                   /*!< XDC DRAM TMG7 FFC0: T_MRD_PDA_FFC0 Bit Mask */  
#define XDC_DRAM_TMG7_FFC0_T_MRD_PDA_FFC0_OFS           6U                                          /*!< XDC DRAM TMG7 FFC0: T_MRD_PDA_FFC0 Bit Offset */
#define XDC_DRAM_TMG7_FFC0_T_MRD_PDA_FFC0(regval)       (BITS(6,10) & ((uint32_t)(regval) << 6))        /*!< XDC DRAM TMG7 FFC0: T_MRD_PDA_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG8_FFC0 Register definition ===== */
#define XDC_DRAM_TMG8_FFC0_T_WR2RD_S_FFC0_MASK          BITS(0,5)                                   /*!< XDC DRAM TMG8 FFC0: T_WR2RD_S_FFC0 Bit Mask */  
#define XDC_DRAM_TMG8_FFC0_T_WR2RD_S_FFC0_OFS           0U                                          /*!< XDC DRAM TMG8 FFC0: T_WR2RD_S_FFC0 Bit Offset */
#define XDC_DRAM_TMG8_FFC0_T_WR2RD_S_FFC0(regval)       (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG8 FFC0: T_WR2RD_S_FFC0 Bit Value */  
#define XDC_DRAM_TMG8_FFC0_T_RRD_S_FFC0_MASK            BITS(6,9)                                   /*!< XDC DRAM TMG8 FFC0: T_RRD_S_FFC0 Bit Mask */  
#define XDC_DRAM_TMG8_FFC0_T_RRD_S_FFC0_OFS             6U                                          /*!< XDC DRAM TMG8 FFC0: T_RRD_S_FFC0 Bit Offset */
#define XDC_DRAM_TMG8_FFC0_T_RRD_S_FFC0(regval)         (BITS(6,9) & ((uint32_t)(regval) << 6))        /*!< XDC DRAM TMG8 FFC0: T_RRD_S_FFC0 Bit Value */  
#define XDC_DRAM_TMG8_FFC0_T_CCD_S_FFC0_MASK            BITS(10,12)                                   /*!< XDC DRAM TMG8 FFC0: T_CCD_S_FFC0 Bit Mask */  
#define XDC_DRAM_TMG8_FFC0_T_CCD_S_FFC0_OFS             10U                                          /*!< XDC DRAM TMG8 FFC0: T_CCD_S_FFC0 Bit Offset */
#define XDC_DRAM_TMG8_FFC0_T_CCD_S_FFC0(regval)         (BITS(10,12) & ((uint32_t)(regval) << 10))        /*!< XDC DRAM TMG8 FFC0: T_CCD_S_FFC0 Bit Value */  
#define XDC_DRAM_TMG8_FFC0_WR_PRE_FFC0                  BIT(13)                                      /*!< DDR4 write preamble setting: 0 - 1tCK preamble 1 - 2tCK preamble */
#define XDC_DRAM_TMG8_FFC0_WR_PRE_FFC0_OFS              13U                                          /*!< XDC DRAM TMG8 FFC0: WR_PRE_FFC0 Bit Offset */
#define XDC_DRAM_TMG8_FFC0_WR_PRE_FFC0_VAL(regval)          (BIT(13) & ((uint32_t)(regval) << 13))        /*!< XDC DRAM TMG8 FFC0: WR_PRE_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG9_FFC0 Register definition ===== */
#define XDC_DRAM_TMG9_FFC0_T_GEAR_HOLD_FFC0_MASK        BITS(0,1)                                   /*!< XDC DRAM TMG9 FFC0: T_GEAR_HOLD_FFC0 Bit Mask */  
#define XDC_DRAM_TMG9_FFC0_T_GEAR_HOLD_FFC0_OFS         0U                                          /*!< XDC DRAM TMG9 FFC0: T_GEAR_HOLD_FFC0 Bit Offset */
#define XDC_DRAM_TMG9_FFC0_T_GEAR_HOLD_FFC0(regval)     (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG9 FFC0: T_GEAR_HOLD_FFC0 Bit Value */  
#define XDC_DRAM_TMG9_FFC0_T_GEAR_SETUP_FFC0_MASK       BITS(2,3)                                   /*!< XDC DRAM TMG9 FFC0: T_GEAR_SETUP_FFC0 Bit Mask */  
#define XDC_DRAM_TMG9_FFC0_T_GEAR_SETUP_FFC0_OFS        2U                                          /*!< XDC DRAM TMG9 FFC0: T_GEAR_SETUP_FFC0 Bit Offset */
#define XDC_DRAM_TMG9_FFC0_T_GEAR_SETUP_FFC0(regval)    (BITS(2,3) & ((uint32_t)(regval) << 2))        /*!< XDC DRAM TMG9 FFC0: T_GEAR_SETUP_FFC0 Bit Value */  
#define XDC_DRAM_TMG9_FFC0_T_CMD_GEAR_FFC0_MASK         BITS(4,8)                                   /*!< XDC DRAM TMG9 FFC0: T_CMD_GEAR_FFC0 Bit Mask */  
#define XDC_DRAM_TMG9_FFC0_T_CMD_GEAR_FFC0_OFS          4U                                          /*!< XDC DRAM TMG9 FFC0: T_CMD_GEAR_FFC0 Bit Offset */
#define XDC_DRAM_TMG9_FFC0_T_CMD_GEAR_FFC0(regval)      (BITS(4,8) & ((uint32_t)(regval) << 4))        /*!< XDC DRAM TMG9 FFC0: T_CMD_GEAR_FFC0 Bit Value */  
#define XDC_DRAM_TMG9_FFC0_T_SYNC_GEAR_FFC0_MASK        BITS(9,13)                                   /*!< XDC DRAM TMG9 FFC0: T_SYNC_GEAR_FFC0 Bit Mask */  
#define XDC_DRAM_TMG9_FFC0_T_SYNC_GEAR_FFC0_OFS         9U                                          /*!< XDC DRAM TMG9 FFC0: T_SYNC_GEAR_FFC0 Bit Offset */
#define XDC_DRAM_TMG9_FFC0_T_SYNC_GEAR_FFC0(regval)     (BITS(9,13) & ((uint32_t)(regval) << 9))        /*!< XDC DRAM TMG9 FFC0: T_SYNC_GEAR_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG10_FFC0 Register definition ===== */
#define XDC_DRAM_TMG10_FFC0_T_CKMPE_FFC0_MASK            BITS(0,4)                                   /*!< XDC DRAM TMG10 FFC0: T_CKMPE_FFC0 Bit Mask */  
#define XDC_DRAM_TMG10_FFC0_T_CKMPE_FFC0_OFS             0U                                          /*!< XDC DRAM TMG10 FFC0: T_CKMPE_FFC0 Bit Offset */
#define XDC_DRAM_TMG10_FFC0_T_CKMPE_FFC0(regval)         (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG10 FFC0: T_CKMPE_FFC0 Bit Value */  
#define XDC_DRAM_TMG10_FFC0_T_MPX_S_FFC0_MASK            BITS(5,6)                                   /*!< XDC DRAM TMG10 FFC0: T_MPX_S_FFC0 Bit Mask */  
#define XDC_DRAM_TMG10_FFC0_T_MPX_S_FFC0_OFS             5U                                          /*!< XDC DRAM TMG10 FFC0: T_MPX_S_FFC0 Bit Offset */
#define XDC_DRAM_TMG10_FFC0_T_MPX_S_FFC0(regval)         (BITS(5,6) & ((uint32_t)(regval) << 5))        /*!< XDC DRAM TMG10 FFC0: T_MPX_S_FFC0 Bit Value */  
#define XDC_DRAM_TMG10_FFC0_T_MPX_LH_FFC0_MASK           BITS(7,11)                                   /*!< XDC DRAM TMG10 FFC0: T_MPX_LH_FFC0 Bit Mask */  
#define XDC_DRAM_TMG10_FFC0_T_MPX_LH_FFC0_OFS            7U                                          /*!< XDC DRAM TMG10 FFC0: T_MPX_LH_FFC0 Bit Offset */
#define XDC_DRAM_TMG10_FFC0_T_MPX_LH_FFC0(regval)        (BITS(7,11) & ((uint32_t)(regval) << 7))        /*!< XDC DRAM TMG10 FFC0: T_MPX_LH_FFC0 Bit Value */  
#define XDC_DRAM_TMG10_FFC0_T_XMP_DLL_FFC0_MASK          BITS(12,18)                                   /*!< XDC DRAM TMG10 FFC0: T_XMP_DLL_FFC0 Bit Mask */  
#define XDC_DRAM_TMG10_FFC0_T_XMP_DLL_FFC0_OFS           12U                                          /*!< XDC DRAM TMG10 FFC0: T_XMP_DLL_FFC0 Bit Offset */
#define XDC_DRAM_TMG10_FFC0_T_XMP_DLL_FFC0(regval)       (BITS(12,18) & ((uint32_t)(regval) << 12))        /*!< XDC DRAM TMG10 FFC0: T_XMP_DLL_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG11_FFC0 Register definition ===== */
#define XDC_DRAM_TMG11_FFC0_T_CMDCKE_FFC0_MASK           BITS(0,1)                                   /*!< XDC DRAM TMG11 FFC0: T_CMDCKE_FFC0 Bit Mask */  
#define XDC_DRAM_TMG11_FFC0_T_CMDCKE_FFC0_OFS            0U                                          /*!< XDC DRAM TMG11 FFC0: T_CMDCKE_FFC0 Bit Offset */
#define XDC_DRAM_TMG11_FFC0_T_CMDCKE_FFC0(regval)        (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG11 FFC0: T_CMDCKE_FFC0 Bit Value */  
#define XDC_DRAM_TMG11_FFC0_T_XSR_FFC0_MASK              BITS(2,13)                                   /*!< XDC DRAM TMG11 FFC0: T_XSR_FFC0 Bit Mask */  
#define XDC_DRAM_TMG11_FFC0_T_XSR_FFC0_OFS               2U                                          /*!< XDC DRAM TMG11 FFC0: T_XSR_FFC0 Bit Offset */
#define XDC_DRAM_TMG11_FFC0_T_XSR_FFC0(regval)           (BITS(2,13) & ((uint32_t)(regval) << 2))        /*!< XDC DRAM TMG11 FFC0: T_XSR_FFC0 Bit Value */  
#define XDC_DRAM_TMG11_FFC0_T_CCD_MW_FFC0_MASK           BITS(14,19)                                   /*!< XDC DRAM TMG11 FFC0: T_CCD_MW_FFC0 Bit Mask */  
#define XDC_DRAM_TMG11_FFC0_T_CCD_MW_FFC0_OFS            14U                                          /*!< XDC DRAM TMG11 FFC0: T_CCD_MW_FFC0 Bit Offset */
#define XDC_DRAM_TMG11_FFC0_T_CCD_MW_FFC0(regval)        (BITS(14,19) & ((uint32_t)(regval) << 14))        /*!< XDC DRAM TMG11 FFC0: T_CCD_MW_FFC0 Bit Value */  
#define XDC_DRAM_TMG11_FFC0_T_ODTL_OFF_FFC0_MASK         BITS(20,26)                                   /*!< XDC DRAM TMG11 FFC0: T_ODTL_OFF_FFC0 Bit Mask */  
#define XDC_DRAM_TMG11_FFC0_T_ODTL_OFF_FFC0_OFS          20U                                          /*!< XDC DRAM TMG11 FFC0: T_ODTL_OFF_FFC0 Bit Offset */
#define XDC_DRAM_TMG11_FFC0_T_ODTL_OFF_FFC0(regval)      (BITS(20,26) & ((uint32_t)(regval) << 20))        /*!< XDC DRAM TMG11 FFC0: T_ODTL_OFF_FFC0 Bit Value */  
#define XDC_DRAM_TMG11_FFC0_T_PPD_FFC0_MASK              BITS(28,30)                                   /*!< XDC DRAM TMG11 FFC0: T_PPD_FFC0 Bit Mask */  
#define XDC_DRAM_TMG11_FFC0_T_PPD_FFC0_OFS               28U                                          /*!< XDC DRAM TMG11 FFC0: T_PPD_FFC0 Bit Offset */
#define XDC_DRAM_TMG11_FFC0_T_PPD_FFC0(regval)           (BITS(28,30) & ((uint32_t)(regval) << 28))        /*!< XDC DRAM TMG11 FFC0: T_PPD_FFC0 Bit Value */  
 
 /* ===== XDC DRAM_TMG12_FFC0 Register definition ===== */
#define XDC_DRAM_TMG12_FFC0_T_VRCG_EN_FFC0_MASK          BITS(0,7)                                   /*!< XDC DRAM TMG12 FFC0: T_VRCG_EN_FFC0 Bit Mask */  
#define XDC_DRAM_TMG12_FFC0_T_VRCG_EN_FFC0_OFS           0U                                          /*!< XDC DRAM TMG12 FFC0: T_VRCG_EN_FFC0 Bit Offset */
#define XDC_DRAM_TMG12_FFC0_T_VRCG_EN_FFC0(regval)       (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG12 FFC0: T_VRCG_EN_FFC0 Bit Value */  
#define XDC_DRAM_TMG12_FFC0_T_VRCG_DIS_FFC0_MASK         BITS(8,15)                                   /*!< XDC DRAM TMG12 FFC0: T_VRCG_DIS_FFC0 Bit Mask */  
#define XDC_DRAM_TMG12_FFC0_T_VRCG_DIS_FFC0_OFS          8U                                          /*!< XDC DRAM TMG12 FFC0: T_VRCG_DIS_FFC0 Bit Offset */
#define XDC_DRAM_TMG12_FFC0_T_VRCG_DIS_FFC0(regval)      (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< XDC DRAM TMG12 FFC0: T_VRCG_DIS_FFC0 Bit Value */  
 
 /* ===== XDC DERATE_TMG_FFC0 Register definition ===== */
#define XDC_DERATE_TMG_FFC0_DERATE_VAL_FFC0_MASK         BITS(0,1)                                   /*!< XDC DERATE TMG FFC0: DERATE_VAL_FFC0 Bit Mask */  
#define XDC_DERATE_TMG_FFC0_DERATE_VAL_FFC0_OFS          0U                                          /*!< XDC DERATE TMG FFC0: DERATE_VAL_FFC0 Bit Offset */
#define XDC_DERATE_TMG_FFC0_DERATE_VAL_FFC0(regval)      (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< XDC DERATE TMG FFC0: DERATE_VAL_FFC0 Bit Value */  
#define XDC_DERATE_TMG_FFC0_RC_DERATE_VAL_FFC0_MASK      BITS(4,6)                                   /*!< XDC DERATE TMG FFC0: RC_DERATE_VAL_FFC0 Bit Mask */  
#define XDC_DERATE_TMG_FFC0_RC_DERATE_VAL_FFC0_OFS       4U                                          /*!< XDC DERATE TMG FFC0: RC_DERATE_VAL_FFC0 Bit Offset */
#define XDC_DERATE_TMG_FFC0_RC_DERATE_VAL_FFC0(regval)   (BITS(4,6) & ((uint32_t)(regval) << 4))        /*!< XDC DERATE TMG FFC0: RC_DERATE_VAL_FFC0 Bit Value */  
#define XDC_DERATE_TMG_FFC0_TS_RD_INTRVL_FFC0_MASK       BITS(8,31)                                   /*!< XDC DERATE TMG FFC0: TS_RD_INTRVL_FFC0 Bit Mask */  
#define XDC_DERATE_TMG_FFC0_TS_RD_INTRVL_FFC0_OFS        8U                                          /*!< XDC DERATE TMG FFC0: TS_RD_INTRVL_FFC0 Bit Offset */
#define XDC_DERATE_TMG_FFC0_TS_RD_INTRVL_FFC0(regval)    (BITS(8,31) & ((uint32_t)(regval) << 8))        /*!< XDC DERATE TMG FFC0: TS_RD_INTRVL_FFC0 Bit Value */  
 
 /* ===== XDC ZQCAL_TMG0_FFC0 Register definition ===== */
#define XDC_ZQCAL_TMG0_FFC0_T_ZQCS_FFC0_MASK             BITS(0,9)                                   /*!< XDC ZQCAL TMG0 FFC0: T_ZQCS_FFC0 Bit Mask */  
#define XDC_ZQCAL_TMG0_FFC0_T_ZQCS_FFC0_OFS              0U                                          /*!< XDC ZQCAL TMG0 FFC0: T_ZQCS_FFC0 Bit Offset */
#define XDC_ZQCAL_TMG0_FFC0_T_ZQCS_FFC0(regval)          (BITS(0,9) & ((uint32_t)(regval) << 0))        /*!< XDC ZQCAL TMG0 FFC0: T_ZQCS_FFC0 Bit Value */  
#define XDC_ZQCAL_TMG0_FFC0_T_ZQOPER_FFC0_MASK           BITS(10,20)                                   /*!< XDC ZQCAL TMG0 FFC0: T_ZQOPER_FFC0 Bit Mask */  
#define XDC_ZQCAL_TMG0_FFC0_T_ZQOPER_FFC0_OFS            10U                                          /*!< XDC ZQCAL TMG0 FFC0: T_ZQOPER_FFC0 Bit Offset */
#define XDC_ZQCAL_TMG0_FFC0_T_ZQOPER_FFC0(regval)        (BITS(10,20) & ((uint32_t)(regval) << 10))        /*!< XDC ZQCAL TMG0 FFC0: T_ZQOPER_FFC0 Bit Value */  
 
 /* ===== XDC ZQCAL_TMG1_FFC0 Register definition ===== */
#define XDC_ZQCAL_TMG1_FFC0_T_ZQCS_INT_FFC0_MASK         BITS(0,19)                                   /*!< XDC ZQCAL TMG1 FFC0: T_ZQCS_INT_FFC0 Bit Mask */  
#define XDC_ZQCAL_TMG1_FFC0_T_ZQCS_INT_FFC0_OFS          0U                                          /*!< XDC ZQCAL TMG1 FFC0: T_ZQCS_INT_FFC0 Bit Offset */
#define XDC_ZQCAL_TMG1_FFC0_T_ZQCS_INT_FFC0(regval)      (BITS(0,19) & ((uint32_t)(regval) << 0))        /*!< XDC ZQCAL TMG1 FFC0: T_ZQCS_INT_FFC0 Bit Value */  
 
 /* ===== XDC DFI_TMG0_FFC0 Register definition ===== */
#define XDC_DFI_TMG0_FFC0_DFI_TPHY_WRLAT_FFC0_MASK     BITS(0,5)                                   /*!< XDC DFI TMG0 FFC0: DFI_TPHY_WRLAT_FFC0 Bit Mask */  
#define XDC_DFI_TMG0_FFC0_DFI_TPHY_WRLAT_FFC0_OFS      0U                                          /*!< XDC DFI TMG0 FFC0: DFI_TPHY_WRLAT_FFC0 Bit Offset */
#define XDC_DFI_TMG0_FFC0_DFI_TPHY_WRLAT_FFC0(regval)  (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC DFI TMG0 FFC0: DFI_TPHY_WRLAT_FFC0 Bit Value */  
#define XDC_DFI_TMG0_FFC0_DFI_TPHY_WRDATA_FFC0_MASK    BITS(6,11)                                   /*!< XDC DFI TMG0 FFC0: DFI_TPHY_WRDATA_FFC0 Bit Mask */  
#define XDC_DFI_TMG0_FFC0_DFI_TPHY_WRDATA_FFC0_OFS     6U                                          /*!< XDC DFI TMG0 FFC0: DFI_TPHY_WRDATA_FFC0 Bit Offset */
#define XDC_DFI_TMG0_FFC0_DFI_TPHY_WRDATA_FFC0(regval) (BITS(6,11) & ((uint32_t)(regval) << 6))        /*!< XDC DFI TMG0 FFC0: DFI_TPHY_WRDATA_FFC0 Bit Value */  
#define XDC_DFI_TMG0_FFC0_DFI_T_RDDATA_EN_FFC0_MASK    BITS(12,18)                                   /*!< XDC DFI TMG0 FFC0: DFI_T_RDDATA_EN_FFC0 Bit Mask */  
#define XDC_DFI_TMG0_FFC0_DFI_T_RDDATA_EN_FFC0_OFS     12U                                          /*!< XDC DFI TMG0 FFC0: DFI_T_RDDATA_EN_FFC0 Bit Offset */
#define XDC_DFI_TMG0_FFC0_DFI_T_RDDATA_EN_FFC0(regval) (BITS(12,18) & ((uint32_t)(regval) << 12))        /*!< XDC DFI TMG0 FFC0: DFI_T_RDDATA_EN_FFC0 Bit Value */  
#define XDC_DFI_TMG0_FFC0_DFI_T_CTRL_DELAY_FFC0_MASK    BITS(19,23)                                   /*!< XDC DFI TMG0 FFC0: DFI_T_CTRL_DELAY_FFC0 Bit Mask */  
#define XDC_DFI_TMG0_FFC0_DFI_T_CTRL_DELAY_FFC0_OFS     19U                                          /*!< XDC DFI TMG0 FFC0: DFI_T_CTRL_DELAY_FFC0 Bit Offset */
#define XDC_DFI_TMG0_FFC0_DFI_T_CTRL_DELAY_FFC0(regval) (BITS(19,23) & ((uint32_t)(regval) << 19))        /*!< XDC DFI TMG0 FFC0: DFI_T_CTRL_DELAY_FFC0 Bit Value */  
#define XDC_DFI_TMG0_FFC0_DFI_T_CMD_LAT_FFC0_MASK      BITS(24,27)                                   /*!< XDC DFI TMG0 FFC0: DFI_T_CMD_LAT_FFC0 Bit Mask */  
#define XDC_DFI_TMG0_FFC0_DFI_T_CMD_LAT_FFC0_OFS       24U                                          /*!< XDC DFI TMG0 FFC0: DFI_T_CMD_LAT_FFC0 Bit Offset */
#define XDC_DFI_TMG0_FFC0_DFI_T_CMD_LAT_FFC0(regval)   (BITS(24,27) & ((uint32_t)(regval) << 24))        /*!< XDC DFI TMG0 FFC0: DFI_T_CMD_LAT_FFC0 Bit Value */  
#define XDC_DFI_TMG0_FFC0_DFI_WRDATA_SDR_FFC0          BIT(28)                                      /*!< Set the time unit used for timing parameters related to dfi write data path: 0 - use x1 XDC clock cycle as time unit. 1 - use x1 MEM clock cycle as time unit. */
#define XDC_DFI_TMG0_FFC0_DFI_WRDATA_SDR_FFC0_OFS      28U                                          /*!< XDC DFI TMG0 FFC0: DFI_WRDATA_SDR_FFC0 Bit Offset */
#define XDC_DFI_TMG0_FFC0_DFI_WRDATA_SDR_FFC0_VAL(regval)  (BIT(28) & ((uint32_t)(regval) << 28))        /*!< XDC DFI TMG0 FFC0: DFI_WRDATA_SDR_FFC0 Bit Value */  
#define XDC_DFI_TMG0_FFC0_DFI_RDDATA_SDR_FFC0          BIT(29)                                      /*!< Set the time unit used for timing parameters related to dfi read data path: 0 - use x1 XDC clock cycle as time unit. 1 - use x1 MEM clock cycle as time unit. */
#define XDC_DFI_TMG0_FFC0_DFI_RDDATA_SDR_FFC0_OFS      29U                                          /*!< XDC DFI TMG0 FFC0: DFI_RDDATA_SDR_FFC0 Bit Offset */
#define XDC_DFI_TMG0_FFC0_DFI_RDDATA_SDR_FFC0_VAL(regval)  (BIT(29) & ((uint32_t)(regval) << 29))        /*!< XDC DFI TMG0 FFC0: DFI_RDDATA_SDR_FFC0 Bit Value */  
 
 /* ===== XDC DFI_TMG1_FFC0 Register definition ===== */
#define XDC_DFI_TMG1_FFC0_DFI_T_DRAM_CLK_ENABLE_FFC0_MASK    BITS(0,4)                                   /*!< XDC DFI TMG1 FFC0: DFI_T_DRAM_CLK_ENABLE_FFC0 Bit Mask */  
#define XDC_DFI_TMG1_FFC0_DFI_T_DRAM_CLK_ENABLE_FFC0_OFS     0U                                          /*!< XDC DFI TMG1 FFC0: DFI_T_DRAM_CLK_ENABLE_FFC0 Bit Offset */
#define XDC_DFI_TMG1_FFC0_DFI_T_DRAM_CLK_ENABLE_FFC0(regval) (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC DFI TMG1 FFC0: DFI_T_DRAM_CLK_ENABLE_FFC0 Bit Value */  
#define XDC_DFI_TMG1_FFC0_DFI_T_DRAM_CLK_DISABLE_FFC0_MASK    BITS(5,9)                                   /*!< XDC DFI TMG1 FFC0: DFI_T_DRAM_CLK_DISABLE_FFC0 Bit Mask */  
#define XDC_DFI_TMG1_FFC0_DFI_T_DRAM_CLK_DISABLE_FFC0_OFS     5U                                          /*!< XDC DFI TMG1 FFC0: DFI_T_DRAM_CLK_DISABLE_FFC0 Bit Offset */
#define XDC_DFI_TMG1_FFC0_DFI_T_DRAM_CLK_DISABLE_FFC0(regval) (BITS(5,9) & ((uint32_t)(regval) << 5))        /*!< XDC DFI TMG1 FFC0: DFI_T_DRAM_CLK_DISABLE_FFC0 Bit Value */  
#define XDC_DFI_TMG1_FFC0_DFI_T_WRDATA_DELAY_FFC0_MASK    BITS(10,15)                                   /*!< XDC DFI TMG1 FFC0: DFI_T_WRDATA_DELAY_FFC0 Bit Mask */  
#define XDC_DFI_TMG1_FFC0_DFI_T_WRDATA_DELAY_FFC0_OFS     10U                                          /*!< XDC DFI TMG1 FFC0: DFI_T_WRDATA_DELAY_FFC0 Bit Offset */
#define XDC_DFI_TMG1_FFC0_DFI_T_WRDATA_DELAY_FFC0(regval) (BITS(10,15) & ((uint32_t)(regval) << 10))        /*!< XDC DFI TMG1 FFC0: DFI_T_WRDATA_DELAY_FFC0 Bit Value */  
#define XDC_DFI_TMG1_FFC0_DFI_T_PARIN_LAT_FFC0_MASK    BITS(16,17)                                   /*!< XDC DFI TMG1 FFC0: DFI_T_PARIN_LAT_FFC0 Bit Mask */  
#define XDC_DFI_TMG1_FFC0_DFI_T_PARIN_LAT_FFC0_OFS     16U                                          /*!< XDC DFI TMG1 FFC0: DFI_T_PARIN_LAT_FFC0 Bit Offset */
#define XDC_DFI_TMG1_FFC0_DFI_T_PARIN_LAT_FFC0(regval) (BITS(16,17) & ((uint32_t)(regval) << 16))        /*!< XDC DFI TMG1 FFC0: DFI_T_PARIN_LAT_FFC0 Bit Value */  
 
 /* ===== XDC DFI_TMG2_FFC0 Register definition ===== */
#define XDC_DFI_TMG2_FFC0_DFI_TPHY_WRCSLAT_FFC0_MASK    BITS(0,5)                                   /*!< XDC DFI TMG2 FFC0: DFI_TPHY_WRCSLAT_FFC0 Bit Mask */  
#define XDC_DFI_TMG2_FFC0_DFI_TPHY_WRCSLAT_FFC0_OFS     0U                                          /*!< XDC DFI TMG2 FFC0: DFI_TPHY_WRCSLAT_FFC0 Bit Offset */
#define XDC_DFI_TMG2_FFC0_DFI_TPHY_WRCSLAT_FFC0(regval) (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC DFI TMG2 FFC0: DFI_TPHY_WRCSLAT_FFC0 Bit Value */  
#define XDC_DFI_TMG2_FFC0_DFI_TPHY_RDCSLAT_FFC0_MASK    BITS(6,12)                                   /*!< XDC DFI TMG2 FFC0: DFI_TPHY_RDCSLAT_FFC0 Bit Mask */  
#define XDC_DFI_TMG2_FFC0_DFI_TPHY_RDCSLAT_FFC0_OFS     6U                                          /*!< XDC DFI TMG2 FFC0: DFI_TPHY_RDCSLAT_FFC0 Bit Offset */
#define XDC_DFI_TMG2_FFC0_DFI_TPHY_RDCSLAT_FFC0(regval) (BITS(6,12) & ((uint32_t)(regval) << 6))        /*!< XDC DFI TMG2 FFC0: DFI_TPHY_RDCSLAT_FFC0 Bit Value */  
 
 /* ===== XDC DFI_TMG3_FFC0 Register definition ===== */
#define XDC_DFI_TMG3_FFC0_DFI_T_GEARDOWN_DELAY_FFC0_MASK    BITS(0,4)                                   /*!< XDC DFI TMG3 FFC0: DFI_T_GEARDOWN_DELAY_FFC0 Bit Mask */  
#define XDC_DFI_TMG3_FFC0_DFI_T_GEARDOWN_DELAY_FFC0_OFS     0U                                          /*!< XDC DFI TMG3 FFC0: DFI_T_GEARDOWN_DELAY_FFC0 Bit Offset */
#define XDC_DFI_TMG3_FFC0_DFI_T_GEARDOWN_DELAY_FFC0(regval) (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC DFI TMG3 FFC0: DFI_T_GEARDOWN_DELAY_FFC0 Bit Value */  
 
 /* ===== XDC ODT_TMG_FFC0 Register definition ===== */
#define XDC_ODT_TMG_FFC0_RD_ODT_DELAY_FFC0_MASK       BITS(0,4)                                   /*!< XDC ODT TMG FFC0: RD_ODT_DELAY_FFC0 Bit Mask */  
#define XDC_ODT_TMG_FFC0_RD_ODT_DELAY_FFC0_OFS        0U                                          /*!< XDC ODT TMG FFC0: RD_ODT_DELAY_FFC0 Bit Offset */
#define XDC_ODT_TMG_FFC0_RD_ODT_DELAY_FFC0(regval)    (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC ODT TMG FFC0: RD_ODT_DELAY_FFC0 Bit Value */  
#define XDC_ODT_TMG_FFC0_RD_ODT_HOLD_FFC0_MASK        BITS(5,8)                                   /*!< XDC ODT TMG FFC0: RD_ODT_HOLD_FFC0 Bit Mask */  
#define XDC_ODT_TMG_FFC0_RD_ODT_HOLD_FFC0_OFS         5U                                          /*!< XDC ODT TMG FFC0: RD_ODT_HOLD_FFC0 Bit Offset */
#define XDC_ODT_TMG_FFC0_RD_ODT_HOLD_FFC0(regval)     (BITS(5,8) & ((uint32_t)(regval) << 5))        /*!< XDC ODT TMG FFC0: RD_ODT_HOLD_FFC0 Bit Value */  
#define XDC_ODT_TMG_FFC0_WR_ODT_DELAY_FFC0_MASK       BITS(9,13)                                   /*!< XDC ODT TMG FFC0: WR_ODT_DELAY_FFC0 Bit Mask */  
#define XDC_ODT_TMG_FFC0_WR_ODT_DELAY_FFC0_OFS        9U                                          /*!< XDC ODT TMG FFC0: WR_ODT_DELAY_FFC0 Bit Offset */
#define XDC_ODT_TMG_FFC0_WR_ODT_DELAY_FFC0(regval)    (BITS(9,13) & ((uint32_t)(regval) << 9))        /*!< XDC ODT TMG FFC0: WR_ODT_DELAY_FFC0 Bit Value */  
#define XDC_ODT_TMG_FFC0_WR_ODT_HOLD_FFC0_MASK        BITS(14,17)                                   /*!< XDC ODT TMG FFC0: WR_ODT_HOLD_FFC0 Bit Mask */  
#define XDC_ODT_TMG_FFC0_WR_ODT_HOLD_FFC0_OFS         14U                                          /*!< XDC ODT TMG FFC0: WR_ODT_HOLD_FFC0 Bit Offset */
#define XDC_ODT_TMG_FFC0_WR_ODT_HOLD_FFC0(regval)     (BITS(14,17) & ((uint32_t)(regval) << 14))        /*!< XDC ODT TMG FFC0: WR_ODT_HOLD_FFC0 Bit Value */  
 
 /* ===== XDC REF_TMG0_FFC1 Register definition ===== */
#define XDC_REF_TMG0_FFC1_REF_BURST_FFC1_MASK          BITS(0,5)                                   /*!< XDC REF TMG0 FFC1: REF_BURST_FFC1 Bit Mask */  
#define XDC_REF_TMG0_FFC1_REF_BURST_FFC1_OFS           0U                                          /*!< XDC REF TMG0 FFC1: REF_BURST_FFC1 Bit Offset */
#define XDC_REF_TMG0_FFC1_REF_BURST_FFC1(regval)       (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC REF TMG0 FFC1: REF_BURST_FFC1 Bit Value */  
#define XDC_REF_TMG0_FFC1_REF_GAP_FFC1_MASK            BITS(6,10)                                   /*!< XDC REF TMG0 FFC1: REF_GAP_FFC1 Bit Mask */  
#define XDC_REF_TMG0_FFC1_REF_GAP_FFC1_OFS             6U                                          /*!< XDC REF TMG0 FFC1: REF_GAP_FFC1 Bit Offset */
#define XDC_REF_TMG0_FFC1_REF_GAP_FFC1(regval)         (BITS(6,10) & ((uint32_t)(regval) << 6))        /*!< XDC REF TMG0 FFC1: REF_GAP_FFC1 Bit Value */  
#define XDC_REF_TMG0_FFC1_REF_MARGIN_FFC1_MASK         BITS(11,14)                                   /*!< XDC REF TMG0 FFC1: REF_MARGIN_FFC1 Bit Mask */  
#define XDC_REF_TMG0_FFC1_REF_MARGIN_FFC1_OFS          11U                                          /*!< XDC REF TMG0 FFC1: REF_MARGIN_FFC1 Bit Offset */
#define XDC_REF_TMG0_FFC1_REF_MARGIN_FFC1(regval)      (BITS(11,14) & ((uint32_t)(regval) << 11))        /*!< XDC REF TMG0 FFC1: REF_MARGIN_FFC1 Bit Value */  
#define XDC_REF_TMG0_FFC1_REF_MODE_FFC1_MASK           BITS(15,17)                                   /*!< XDC REF TMG0 FFC1: REF_MODE_FFC1 Bit Mask */  
#define XDC_REF_TMG0_FFC1_REF_MODE_FFC1_OFS            15U                                          /*!< XDC REF TMG0 FFC1: REF_MODE_FFC1 Bit Offset */
#define XDC_REF_TMG0_FFC1_REF_MODE_FFC1(regval)        (BITS(15,17) & ((uint32_t)(regval) << 15))        /*!< XDC REF TMG0 FFC1: REF_MODE_FFC1 Bit Value */  
 
 /* ===== XDC REF_TMG1_FFC1 Register definition ===== */
#define XDC_REF_TMG1_FFC1_REF_TIMER0_VAL_FFC1_MASK     BITS(0,11)                                   /*!< XDC REF TMG1 FFC1: REF_TIMER0_VAL_FFC1 Bit Mask */  
#define XDC_REF_TMG1_FFC1_REF_TIMER0_VAL_FFC1_OFS      0U                                          /*!< XDC REF TMG1 FFC1: REF_TIMER0_VAL_FFC1 Bit Offset */
#define XDC_REF_TMG1_FFC1_REF_TIMER0_VAL_FFC1(regval)  (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< XDC REF TMG1 FFC1: REF_TIMER0_VAL_FFC1 Bit Value */  
#define XDC_REF_TMG1_FFC1_REF_TIMER1_VAL_FFC1_MASK     BITS(12,23)                                   /*!< XDC REF TMG1 FFC1: REF_TIMER1_VAL_FFC1 Bit Mask */  
#define XDC_REF_TMG1_FFC1_REF_TIMER1_VAL_FFC1_OFS      12U                                          /*!< XDC REF TMG1 FFC1: REF_TIMER1_VAL_FFC1 Bit Offset */
#define XDC_REF_TMG1_FFC1_REF_TIMER1_VAL_FFC1(regval)  (BITS(12,23) & ((uint32_t)(regval) << 12))        /*!< XDC REF TMG1 FFC1: REF_TIMER1_VAL_FFC1 Bit Value */  
 
 /* ===== XDC REF_TMG3_FFC1 Register definition ===== */
#define XDC_REF_TMG3_FFC1_T_RFC_MIN_FFC1_MASK          BITS(0,9)                                   /*!< XDC REF TMG3 FFC1: T_RFC_MIN_FFC1 Bit Mask */  
#define XDC_REF_TMG3_FFC1_T_RFC_MIN_FFC1_OFS           0U                                          /*!< XDC REF TMG3 FFC1: T_RFC_MIN_FFC1 Bit Offset */
#define XDC_REF_TMG3_FFC1_T_RFC_MIN_FFC1(regval)       (BITS(0,9) & ((uint32_t)(regval) << 0))        /*!< XDC REF TMG3 FFC1: T_RFC_MIN_FFC1 Bit Value */  
#define XDC_REF_TMG3_FFC1_T_REFI_VAL_FFC1_MASK         BITS(10,21)                                   /*!< XDC REF TMG3 FFC1: T_REFI_VAL_FFC1 Bit Mask */  
#define XDC_REF_TMG3_FFC1_T_REFI_VAL_FFC1_OFS          10U                                          /*!< XDC REF TMG3 FFC1: T_REFI_VAL_FFC1 Bit Offset */
#define XDC_REF_TMG3_FFC1_T_REFI_VAL_FFC1(regval)      (BITS(10,21) & ((uint32_t)(regval) << 10))        /*!< XDC REF TMG3 FFC1: T_REFI_VAL_FFC1 Bit Value */  
#define XDC_REF_TMG3_FFC1_T_PBR2PBR_FFC1_MASK          BITS(22,29)                                   /*!< XDC REF TMG3 FFC1: T_PBR2PBR_FFC1 Bit Mask */  
#define XDC_REF_TMG3_FFC1_T_PBR2PBR_FFC1_OFS           22U                                          /*!< XDC REF TMG3 FFC1: T_PBR2PBR_FFC1 Bit Offset */
#define XDC_REF_TMG3_FFC1_T_PBR2PBR_FFC1(regval)       (BITS(22,29) & ((uint32_t)(regval) << 22))        /*!< XDC REF TMG3 FFC1: T_PBR2PBR_FFC1 Bit Value */  
#define XDC_REF_TMG3_FFC1_T_REFI_X1_SEL_FFC1           BIT(30)                                      /*!< Indicates the time unit of t_refi_val. 0 - use x1 XDC clock cycle as time unit 1 - use x32 XDC clock cycle as time unit */
#define XDC_REF_TMG3_FFC1_T_REFI_X1_SEL_FFC1_OFS       30U                                          /*!< XDC REF TMG3 FFC1: T_REFI_X1_SEL_FFC1 Bit Offset */
#define XDC_REF_TMG3_FFC1_T_REFI_X1_SEL_FFC1_VAL(regval)   (BIT(30) & ((uint32_t)(regval) << 30))        /*!< XDC REF TMG3 FFC1: T_REFI_X1_SEL_FFC1 Bit Value */  
 
 /* ===== XDC MR_VAL0_FFC1 Register definition ===== */
#define XDC_MR_VAL0_FFC1_MR0_FFC1_MASK                BITS(0,15)                                   /*!< XDC MR VAL0 FFC1: MR0_FFC1 Bit Mask */  
#define XDC_MR_VAL0_FFC1_MR0_FFC1_OFS                 0U                                          /*!< XDC MR VAL0 FFC1: MR0_FFC1 Bit Offset */
#define XDC_MR_VAL0_FFC1_MR0_FFC1(regval)             (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< XDC MR VAL0 FFC1: MR0_FFC1 Bit Value */  
#define XDC_MR_VAL0_FFC1_MR1_FFC1_MASK                BITS(16,31)                                   /*!< XDC MR VAL0 FFC1: MR1_FFC1 Bit Mask */  
#define XDC_MR_VAL0_FFC1_MR1_FFC1_OFS                 16U                                          /*!< XDC MR VAL0 FFC1: MR1_FFC1 Bit Offset */
#define XDC_MR_VAL0_FFC1_MR1_FFC1(regval)             (BITS(16,31) & ((uint32_t)(regval) << 16))        /*!< XDC MR VAL0 FFC1: MR1_FFC1 Bit Value */  
 
 /* ===== XDC MR_VAL1_FFC1 Register definition ===== */
#define XDC_MR_VAL1_FFC1_MR2_FFC1_MASK                BITS(0,15)                                   /*!< XDC MR VAL1 FFC1: MR2_FFC1 Bit Mask */  
#define XDC_MR_VAL1_FFC1_MR2_FFC1_OFS                 0U                                          /*!< XDC MR VAL1 FFC1: MR2_FFC1 Bit Offset */
#define XDC_MR_VAL1_FFC1_MR2_FFC1(regval)             (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< XDC MR VAL1 FFC1: MR2_FFC1 Bit Value */  
#define XDC_MR_VAL1_FFC1_MR3_FFC1_MASK                BITS(16,31)                                   /*!< XDC MR VAL1 FFC1: MR3_FFC1 Bit Mask */  
#define XDC_MR_VAL1_FFC1_MR3_FFC1_OFS                 16U                                          /*!< XDC MR VAL1 FFC1: MR3_FFC1 Bit Offset */
#define XDC_MR_VAL1_FFC1_MR3_FFC1(regval)             (BITS(16,31) & ((uint32_t)(regval) << 16))        /*!< XDC MR VAL1 FFC1: MR3_FFC1 Bit Value */  
 
 /* ===== XDC MR_VAL2_FFC1 Register definition ===== */
#define XDC_MR_VAL2_FFC1_MR4_FFC1_MASK                BITS(0,15)                                   /*!< XDC MR VAL2 FFC1: MR4_FFC1 Bit Mask */  
#define XDC_MR_VAL2_FFC1_MR4_FFC1_OFS                 0U                                          /*!< XDC MR VAL2 FFC1: MR4_FFC1 Bit Offset */
#define XDC_MR_VAL2_FFC1_MR4_FFC1(regval)             (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< XDC MR VAL2 FFC1: MR4_FFC1 Bit Value */  
#define XDC_MR_VAL2_FFC1_MR5_FFC1_MASK                BITS(16,31)                                   /*!< XDC MR VAL2 FFC1: MR5_FFC1 Bit Mask */  
#define XDC_MR_VAL2_FFC1_MR5_FFC1_OFS                 16U                                          /*!< XDC MR VAL2 FFC1: MR5_FFC1 Bit Offset */
#define XDC_MR_VAL2_FFC1_MR5_FFC1(regval)             (BITS(16,31) & ((uint32_t)(regval) << 16))        /*!< XDC MR VAL2 FFC1: MR5_FFC1 Bit Value */  
 
 /* ===== XDC MR_VAL3_FFC1 Register definition ===== */
#define XDC_MR_VAL3_FFC1_MR6_FFC1_MASK                BITS(0,15)                                   /*!< XDC MR VAL3 FFC1: MR6_FFC1 Bit Mask */  
#define XDC_MR_VAL3_FFC1_MR6_FFC1_OFS                 0U                                          /*!< XDC MR VAL3 FFC1: MR6_FFC1 Bit Offset */
#define XDC_MR_VAL3_FFC1_MR6_FFC1(regval)             (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< XDC MR VAL3 FFC1: MR6_FFC1 Bit Value */  
 
 /* ===== XDC RANK_TMG0_FFC1 Register definition ===== */
#define XDC_RANK_TMG0_FFC1_DIFF_RANK_RD_GAP_FFC1_MASK    BITS(0,4)                                   /*!< XDC RANK TMG0 FFC1: DIFF_RANK_RD_GAP_FFC1 Bit Mask */  
#define XDC_RANK_TMG0_FFC1_DIFF_RANK_RD_GAP_FFC1_OFS     0U                                          /*!< XDC RANK TMG0 FFC1: DIFF_RANK_RD_GAP_FFC1 Bit Offset */
#define XDC_RANK_TMG0_FFC1_DIFF_RANK_RD_GAP_FFC1(regval) (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC RANK TMG0 FFC1: DIFF_RANK_RD_GAP_FFC1 Bit Value */  
#define XDC_RANK_TMG0_FFC1_DIFF_RANK_WR_GAP_FFC1_MASK    BITS(5,9)                                   /*!< XDC RANK TMG0 FFC1: DIFF_RANK_WR_GAP_FFC1 Bit Mask */  
#define XDC_RANK_TMG0_FFC1_DIFF_RANK_WR_GAP_FFC1_OFS     5U                                          /*!< XDC RANK TMG0 FFC1: DIFF_RANK_WR_GAP_FFC1 Bit Offset */
#define XDC_RANK_TMG0_FFC1_DIFF_RANK_WR_GAP_FFC1(regval) (BITS(5,9) & ((uint32_t)(regval) << 5))        /*!< XDC RANK TMG0 FFC1: DIFF_RANK_WR_GAP_FFC1 Bit Value */  
#define XDC_RANK_TMG0_FFC1_DIFF_RANK_T_WR2RD_FFC1_MASK    BITS(16,21)                                   /*!< XDC RANK TMG0 FFC1: DIFF_RANK_T_WR2RD_FFC1 Bit Mask */  
#define XDC_RANK_TMG0_FFC1_DIFF_RANK_T_WR2RD_FFC1_OFS     16U                                          /*!< XDC RANK TMG0 FFC1: DIFF_RANK_T_WR2RD_FFC1 Bit Offset */
#define XDC_RANK_TMG0_FFC1_DIFF_RANK_T_WR2RD_FFC1(regval) (BITS(16,21) & ((uint32_t)(regval) << 16))        /*!< XDC RANK TMG0 FFC1: DIFF_RANK_T_WR2RD_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG0_FFC1 Register definition ===== */
#define XDC_DRAM_TMG0_FFC1_T_RAS_MIN_FFC1_MASK          BITS(0,5)                                   /*!< XDC DRAM TMG0 FFC1: T_RAS_MIN_FFC1 Bit Mask */  
#define XDC_DRAM_TMG0_FFC1_T_RAS_MIN_FFC1_OFS           0U                                          /*!< XDC DRAM TMG0 FFC1: T_RAS_MIN_FFC1 Bit Offset */
#define XDC_DRAM_TMG0_FFC1_T_RAS_MIN_FFC1(regval)       (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG0 FFC1: T_RAS_MIN_FFC1 Bit Value */  
#define XDC_DRAM_TMG0_FFC1_T_RAS_MAX_FFC1_MASK          BITS(6,12)                                   /*!< XDC DRAM TMG0 FFC1: T_RAS_MAX_FFC1 Bit Mask */  
#define XDC_DRAM_TMG0_FFC1_T_RAS_MAX_FFC1_OFS           6U                                          /*!< XDC DRAM TMG0 FFC1: T_RAS_MAX_FFC1 Bit Offset */
#define XDC_DRAM_TMG0_FFC1_T_RAS_MAX_FFC1(regval)       (BITS(6,12) & ((uint32_t)(regval) << 6))        /*!< XDC DRAM TMG0 FFC1: T_RAS_MAX_FFC1 Bit Value */  
#define XDC_DRAM_TMG0_FFC1_T_FAW_FFC1_MASK              BITS(13,18)                                   /*!< XDC DRAM TMG0 FFC1: T_FAW_FFC1 Bit Mask */  
#define XDC_DRAM_TMG0_FFC1_T_FAW_FFC1_OFS               13U                                          /*!< XDC DRAM TMG0 FFC1: T_FAW_FFC1 Bit Offset */
#define XDC_DRAM_TMG0_FFC1_T_FAW_FFC1(regval)           (BITS(13,18) & ((uint32_t)(regval) << 13))        /*!< XDC DRAM TMG0 FFC1: T_FAW_FFC1 Bit Value */  
#define XDC_DRAM_TMG0_FFC1_T_WR2PRE_FFC1_MASK           BITS(19,25)                                   /*!< XDC DRAM TMG0 FFC1: T_WR2PRE_FFC1 Bit Mask */  
#define XDC_DRAM_TMG0_FFC1_T_WR2PRE_FFC1_OFS            19U                                          /*!< XDC DRAM TMG0 FFC1: T_WR2PRE_FFC1 Bit Offset */
#define XDC_DRAM_TMG0_FFC1_T_WR2PRE_FFC1(regval)        (BITS(19,25) & ((uint32_t)(regval) << 19))        /*!< XDC DRAM TMG0 FFC1: T_WR2PRE_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG1_FFC1 Register definition ===== */
#define XDC_DRAM_TMG1_FFC1_T_RC_FFC1_MASK               BITS(0,6)                                   /*!< XDC DRAM TMG1 FFC1: T_RC_FFC1 Bit Mask */  
#define XDC_DRAM_TMG1_FFC1_T_RC_FFC1_OFS                0U                                          /*!< XDC DRAM TMG1 FFC1: T_RC_FFC1 Bit Offset */
#define XDC_DRAM_TMG1_FFC1_T_RC_FFC1(regval)            (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG1 FFC1: T_RC_FFC1 Bit Value */  
#define XDC_DRAM_TMG1_FFC1_T_RD2PRE_FFC1_MASK           BITS(7,12)                                   /*!< XDC DRAM TMG1 FFC1: T_RD2PRE_FFC1 Bit Mask */  
#define XDC_DRAM_TMG1_FFC1_T_RD2PRE_FFC1_OFS            7U                                          /*!< XDC DRAM TMG1 FFC1: T_RD2PRE_FFC1 Bit Offset */
#define XDC_DRAM_TMG1_FFC1_T_RD2PRE_FFC1(regval)        (BITS(7,12) & ((uint32_t)(regval) << 7))        /*!< XDC DRAM TMG1 FFC1: T_RD2PRE_FFC1 Bit Value */  
#define XDC_DRAM_TMG1_FFC1_T_XP_FFC1_MASK               BITS(13,17)                                   /*!< XDC DRAM TMG1 FFC1: T_XP_FFC1 Bit Mask */  
#define XDC_DRAM_TMG1_FFC1_T_XP_FFC1_OFS                13U                                          /*!< XDC DRAM TMG1 FFC1: T_XP_FFC1 Bit Offset */
#define XDC_DRAM_TMG1_FFC1_T_XP_FFC1(regval)            (BITS(13,17) & ((uint32_t)(regval) << 13))        /*!< XDC DRAM TMG1 FFC1: T_XP_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG2_FFC1 Register definition ===== */
#define XDC_DRAM_TMG2_FFC1_T_WR2RD_FFC1_MASK            BITS(0,5)                                   /*!< XDC DRAM TMG2 FFC1: T_WR2RD_FFC1 Bit Mask */  
#define XDC_DRAM_TMG2_FFC1_T_WR2RD_FFC1_OFS             0U                                          /*!< XDC DRAM TMG2 FFC1: T_WR2RD_FFC1 Bit Offset */
#define XDC_DRAM_TMG2_FFC1_T_WR2RD_FFC1(regval)         (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG2 FFC1: T_WR2RD_FFC1 Bit Value */  
#define XDC_DRAM_TMG2_FFC1_T_RD2WR_FFC1_MASK            BITS(6,11)                                   /*!< XDC DRAM TMG2 FFC1: T_RD2WR_FFC1 Bit Mask */  
#define XDC_DRAM_TMG2_FFC1_T_RD2WR_FFC1_OFS             6U                                          /*!< XDC DRAM TMG2 FFC1: T_RD2WR_FFC1 Bit Offset */
#define XDC_DRAM_TMG2_FFC1_T_RD2WR_FFC1(regval)         (BITS(6,11) & ((uint32_t)(regval) << 6))        /*!< XDC DRAM TMG2 FFC1: T_RD2WR_FFC1 Bit Value */  
#define XDC_DRAM_TMG2_FFC1_T_WR_LAT_FFC1_MASK           BITS(12,17)                                   /*!< XDC DRAM TMG2 FFC1: T_WR_LAT_FFC1 Bit Mask */  
#define XDC_DRAM_TMG2_FFC1_T_WR_LAT_FFC1_OFS            12U                                          /*!< XDC DRAM TMG2 FFC1: T_WR_LAT_FFC1 Bit Offset */
#define XDC_DRAM_TMG2_FFC1_T_WR_LAT_FFC1(regval)        (BITS(12,17) & ((uint32_t)(regval) << 12))        /*!< XDC DRAM TMG2 FFC1: T_WR_LAT_FFC1 Bit Value */  
#define XDC_DRAM_TMG2_FFC1_T_RD_LAT_FFC1_MASK           BITS(18,23)                                   /*!< XDC DRAM TMG2 FFC1: T_RD_LAT_FFC1 Bit Mask */  
#define XDC_DRAM_TMG2_FFC1_T_RD_LAT_FFC1_OFS            18U                                          /*!< XDC DRAM TMG2 FFC1: T_RD_LAT_FFC1 Bit Offset */
#define XDC_DRAM_TMG2_FFC1_T_RD_LAT_FFC1(regval)        (BITS(18,23) & ((uint32_t)(regval) << 18))        /*!< XDC DRAM TMG2 FFC1: T_RD_LAT_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG3_FFC1 Register definition ===== */
#define XDC_DRAM_TMG3_FFC1_T_MOD_FFC1_MASK              BITS(0,9)                                   /*!< XDC DRAM TMG3 FFC1: T_MOD_FFC1 Bit Mask */  
#define XDC_DRAM_TMG3_FFC1_T_MOD_FFC1_OFS               0U                                          /*!< XDC DRAM TMG3 FFC1: T_MOD_FFC1 Bit Offset */
#define XDC_DRAM_TMG3_FFC1_T_MOD_FFC1(regval)           (BITS(0,9) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG3 FFC1: T_MOD_FFC1 Bit Value */  
#define XDC_DRAM_TMG3_FFC1_T_MRD_FFC1_MASK              BITS(10,19)                                   /*!< XDC DRAM TMG3 FFC1: T_MRD_FFC1 Bit Mask */  
#define XDC_DRAM_TMG3_FFC1_T_MRD_FFC1_OFS               10U                                          /*!< XDC DRAM TMG3 FFC1: T_MRD_FFC1 Bit Offset */
#define XDC_DRAM_TMG3_FFC1_T_MRD_FFC1(regval)           (BITS(10,19) & ((uint32_t)(regval) << 10))        /*!< XDC DRAM TMG3 FFC1: T_MRD_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG4_FFC1 Register definition ===== */
#define XDC_DRAM_TMG4_FFC1_T_RP_FFC1_MASK               BITS(0,4)                                   /*!< XDC DRAM TMG4 FFC1: T_RP_FFC1 Bit Mask */  
#define XDC_DRAM_TMG4_FFC1_T_RP_FFC1_OFS                0U                                          /*!< XDC DRAM TMG4 FFC1: T_RP_FFC1 Bit Offset */
#define XDC_DRAM_TMG4_FFC1_T_RP_FFC1(regval)            (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG4 FFC1: T_RP_FFC1 Bit Value */  
#define XDC_DRAM_TMG4_FFC1_T_RRD_FFC1_MASK              BITS(5,9)                                   /*!< XDC DRAM TMG4 FFC1: T_RRD_FFC1 Bit Mask */  
#define XDC_DRAM_TMG4_FFC1_T_RRD_FFC1_OFS               5U                                          /*!< XDC DRAM TMG4 FFC1: T_RRD_FFC1 Bit Offset */
#define XDC_DRAM_TMG4_FFC1_T_RRD_FFC1(regval)           (BITS(5,9) & ((uint32_t)(regval) << 5))        /*!< XDC DRAM TMG4 FFC1: T_RRD_FFC1 Bit Value */  
#define XDC_DRAM_TMG4_FFC1_T_CCD_FFC1_MASK              BITS(10,13)                                   /*!< XDC DRAM TMG4 FFC1: T_CCD_FFC1 Bit Mask */  
#define XDC_DRAM_TMG4_FFC1_T_CCD_FFC1_OFS               10U                                          /*!< XDC DRAM TMG4 FFC1: T_CCD_FFC1 Bit Offset */
#define XDC_DRAM_TMG4_FFC1_T_CCD_FFC1(regval)           (BITS(10,13) & ((uint32_t)(regval) << 10))        /*!< XDC DRAM TMG4 FFC1: T_CCD_FFC1 Bit Value */  
#define XDC_DRAM_TMG4_FFC1_T_RCD_FFC1_MASK              BITS(14,18)                                   /*!< XDC DRAM TMG4 FFC1: T_RCD_FFC1 Bit Mask */  
#define XDC_DRAM_TMG4_FFC1_T_RCD_FFC1_OFS               14U                                          /*!< XDC DRAM TMG4 FFC1: T_RCD_FFC1 Bit Offset */
#define XDC_DRAM_TMG4_FFC1_T_RCD_FFC1(regval)           (BITS(14,18) & ((uint32_t)(regval) << 14))        /*!< XDC DRAM TMG4 FFC1: T_RCD_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG5_FFC1 Register definition ===== */
#define XDC_DRAM_TMG5_FFC1_T_CKE_FFC1_MASK              BITS(0,4)                                   /*!< XDC DRAM TMG5 FFC1: T_CKE_FFC1 Bit Mask */  
#define XDC_DRAM_TMG5_FFC1_T_CKE_FFC1_OFS               0U                                          /*!< XDC DRAM TMG5 FFC1: T_CKE_FFC1 Bit Offset */
#define XDC_DRAM_TMG5_FFC1_T_CKE_FFC1(regval)           (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG5 FFC1: T_CKE_FFC1 Bit Value */  
#define XDC_DRAM_TMG5_FFC1_T_CKESR_FFC1_MASK            BITS(5,10)                                   /*!< XDC DRAM TMG5 FFC1: T_CKESR_FFC1 Bit Mask */  
#define XDC_DRAM_TMG5_FFC1_T_CKESR_FFC1_OFS             5U                                          /*!< XDC DRAM TMG5 FFC1: T_CKESR_FFC1 Bit Offset */
#define XDC_DRAM_TMG5_FFC1_T_CKESR_FFC1(regval)         (BITS(5,10) & ((uint32_t)(regval) << 5))        /*!< XDC DRAM TMG5 FFC1: T_CKESR_FFC1 Bit Value */  
#define XDC_DRAM_TMG5_FFC1_T_CKSRE_FFC1_MASK            BITS(11,17)                                   /*!< XDC DRAM TMG5 FFC1: T_CKSRE_FFC1 Bit Mask */  
#define XDC_DRAM_TMG5_FFC1_T_CKSRE_FFC1_OFS             11U                                          /*!< XDC DRAM TMG5 FFC1: T_CKSRE_FFC1 Bit Offset */
#define XDC_DRAM_TMG5_FFC1_T_CKSRE_FFC1(regval)         (BITS(11,17) & ((uint32_t)(regval) << 11))        /*!< XDC DRAM TMG5 FFC1: T_CKSRE_FFC1 Bit Value */  
#define XDC_DRAM_TMG5_FFC1_T_CKSRX_FFC1_MASK            BITS(18,21)                                   /*!< XDC DRAM TMG5 FFC1: T_CKSRX_FFC1 Bit Mask */  
#define XDC_DRAM_TMG5_FFC1_T_CKSRX_FFC1_OFS             18U                                          /*!< XDC DRAM TMG5 FFC1: T_CKSRX_FFC1 Bit Offset */
#define XDC_DRAM_TMG5_FFC1_T_CKSRX_FFC1(regval)         (BITS(18,21) & ((uint32_t)(regval) << 18))        /*!< XDC DRAM TMG5 FFC1: T_CKSRX_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG6_FFC1 Register definition ===== */
#define XDC_DRAM_TMG6_FFC1_T_XS_FFC1_MASK               BITS(0,6)                                   /*!< XDC DRAM TMG6 FFC1: T_XS_FFC1 Bit Mask */  
#define XDC_DRAM_TMG6_FFC1_T_XS_FFC1_OFS                0U                                          /*!< XDC DRAM TMG6 FFC1: T_XS_FFC1 Bit Offset */
#define XDC_DRAM_TMG6_FFC1_T_XS_FFC1(regval)            (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG6 FFC1: T_XS_FFC1 Bit Value */  
#define XDC_DRAM_TMG6_FFC1_T_XS_DLL_FFC1_MASK           BITS(7,13)                                   /*!< XDC DRAM TMG6 FFC1: T_XS_DLL_FFC1 Bit Mask */  
#define XDC_DRAM_TMG6_FFC1_T_XS_DLL_FFC1_OFS            7U                                          /*!< XDC DRAM TMG6 FFC1: T_XS_DLL_FFC1 Bit Offset */
#define XDC_DRAM_TMG6_FFC1_T_XS_DLL_FFC1(regval)        (BITS(7,13) & ((uint32_t)(regval) << 7))        /*!< XDC DRAM TMG6 FFC1: T_XS_DLL_FFC1 Bit Value */  
#define XDC_DRAM_TMG6_FFC1_T_XS_ABORT_FFC1_MASK         BITS(14,20)                                   /*!< XDC DRAM TMG6 FFC1: T_XS_ABORT_FFC1 Bit Mask */  
#define XDC_DRAM_TMG6_FFC1_T_XS_ABORT_FFC1_OFS          14U                                          /*!< XDC DRAM TMG6 FFC1: T_XS_ABORT_FFC1 Bit Offset */
#define XDC_DRAM_TMG6_FFC1_T_XS_ABORT_FFC1(regval)      (BITS(14,20) & ((uint32_t)(regval) << 14))        /*!< XDC DRAM TMG6 FFC1: T_XS_ABORT_FFC1 Bit Value */  
#define XDC_DRAM_TMG6_FFC1_T_XS_FAST_FFC1_MASK          BITS(21,27)                                   /*!< XDC DRAM TMG6 FFC1: T_XS_FAST_FFC1 Bit Mask */  
#define XDC_DRAM_TMG6_FFC1_T_XS_FAST_FFC1_OFS           21U                                          /*!< XDC DRAM TMG6 FFC1: T_XS_FAST_FFC1 Bit Offset */
#define XDC_DRAM_TMG6_FFC1_T_XS_FAST_FFC1(regval)       (BITS(21,27) & ((uint32_t)(regval) << 21))        /*!< XDC DRAM TMG6 FFC1: T_XS_FAST_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG7_FFC1 Register definition ===== */
#define XDC_DRAM_TMG7_FFC1_T_WR_MPR_FFC1_MASK           BITS(0,5)                                   /*!< XDC DRAM TMG7 FFC1: T_WR_MPR_FFC1 Bit Mask */  
#define XDC_DRAM_TMG7_FFC1_T_WR_MPR_FFC1_OFS            0U                                          /*!< XDC DRAM TMG7 FFC1: T_WR_MPR_FFC1 Bit Offset */
#define XDC_DRAM_TMG7_FFC1_T_WR_MPR_FFC1(regval)        (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG7 FFC1: T_WR_MPR_FFC1 Bit Value */  
#define XDC_DRAM_TMG7_FFC1_T_MRD_PDA_FFC1_MASK          BITS(6,10)                                   /*!< XDC DRAM TMG7 FFC1: T_MRD_PDA_FFC1 Bit Mask */  
#define XDC_DRAM_TMG7_FFC1_T_MRD_PDA_FFC1_OFS           6U                                          /*!< XDC DRAM TMG7 FFC1: T_MRD_PDA_FFC1 Bit Offset */
#define XDC_DRAM_TMG7_FFC1_T_MRD_PDA_FFC1(regval)       (BITS(6,10) & ((uint32_t)(regval) << 6))        /*!< XDC DRAM TMG7 FFC1: T_MRD_PDA_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG8_FFC1 Register definition ===== */
#define XDC_DRAM_TMG8_FFC1_T_WR2RD_S_FFC1_MASK          BITS(0,5)                                   /*!< XDC DRAM TMG8 FFC1: T_WR2RD_S_FFC1 Bit Mask */  
#define XDC_DRAM_TMG8_FFC1_T_WR2RD_S_FFC1_OFS           0U                                          /*!< XDC DRAM TMG8 FFC1: T_WR2RD_S_FFC1 Bit Offset */
#define XDC_DRAM_TMG8_FFC1_T_WR2RD_S_FFC1(regval)       (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG8 FFC1: T_WR2RD_S_FFC1 Bit Value */  
#define XDC_DRAM_TMG8_FFC1_T_RRD_S_FFC1_MASK            BITS(6,9)                                   /*!< XDC DRAM TMG8 FFC1: T_RRD_S_FFC1 Bit Mask */  
#define XDC_DRAM_TMG8_FFC1_T_RRD_S_FFC1_OFS             6U                                          /*!< XDC DRAM TMG8 FFC1: T_RRD_S_FFC1 Bit Offset */
#define XDC_DRAM_TMG8_FFC1_T_RRD_S_FFC1(regval)         (BITS(6,9) & ((uint32_t)(regval) << 6))        /*!< XDC DRAM TMG8 FFC1: T_RRD_S_FFC1 Bit Value */  
#define XDC_DRAM_TMG8_FFC1_T_CCD_S_FFC1_MASK            BITS(10,12)                                   /*!< XDC DRAM TMG8 FFC1: T_CCD_S_FFC1 Bit Mask */  
#define XDC_DRAM_TMG8_FFC1_T_CCD_S_FFC1_OFS             10U                                          /*!< XDC DRAM TMG8 FFC1: T_CCD_S_FFC1 Bit Offset */
#define XDC_DRAM_TMG8_FFC1_T_CCD_S_FFC1(regval)         (BITS(10,12) & ((uint32_t)(regval) << 10))        /*!< XDC DRAM TMG8 FFC1: T_CCD_S_FFC1 Bit Value */  
#define XDC_DRAM_TMG8_FFC1_WR_PRE_FFC1                  BIT(13)                                      /*!< DDR4 write preamble setting: 0 - 1tCK preamble 1 - 2tCK preamble */
#define XDC_DRAM_TMG8_FFC1_WR_PRE_FFC1_OFS              13U                                          /*!< XDC DRAM TMG8 FFC1: WR_PRE_FFC1 Bit Offset */
#define XDC_DRAM_TMG8_FFC1_WR_PRE_FFC1_VAL(regval)          (BIT(13) & ((uint32_t)(regval) << 13))        /*!< XDC DRAM TMG8 FFC1: WR_PRE_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG9_FFC1 Register definition ===== */
#define XDC_DRAM_TMG9_FFC1_T_GEAR_HOLD_FFC1_MASK        BITS(0,1)                                   /*!< XDC DRAM TMG9 FFC1: T_GEAR_HOLD_FFC1 Bit Mask */  
#define XDC_DRAM_TMG9_FFC1_T_GEAR_HOLD_FFC1_OFS         0U                                          /*!< XDC DRAM TMG9 FFC1: T_GEAR_HOLD_FFC1 Bit Offset */
#define XDC_DRAM_TMG9_FFC1_T_GEAR_HOLD_FFC1(regval)     (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG9 FFC1: T_GEAR_HOLD_FFC1 Bit Value */  
#define XDC_DRAM_TMG9_FFC1_T_GEAR_SETUP_FFC1_MASK       BITS(2,3)                                   /*!< XDC DRAM TMG9 FFC1: T_GEAR_SETUP_FFC1 Bit Mask */  
#define XDC_DRAM_TMG9_FFC1_T_GEAR_SETUP_FFC1_OFS        2U                                          /*!< XDC DRAM TMG9 FFC1: T_GEAR_SETUP_FFC1 Bit Offset */
#define XDC_DRAM_TMG9_FFC1_T_GEAR_SETUP_FFC1(regval)    (BITS(2,3) & ((uint32_t)(regval) << 2))        /*!< XDC DRAM TMG9 FFC1: T_GEAR_SETUP_FFC1 Bit Value */  
#define XDC_DRAM_TMG9_FFC1_T_CMD_GEAR_FFC1_MASK         BITS(4,8)                                   /*!< XDC DRAM TMG9 FFC1: T_CMD_GEAR_FFC1 Bit Mask */  
#define XDC_DRAM_TMG9_FFC1_T_CMD_GEAR_FFC1_OFS          4U                                          /*!< XDC DRAM TMG9 FFC1: T_CMD_GEAR_FFC1 Bit Offset */
#define XDC_DRAM_TMG9_FFC1_T_CMD_GEAR_FFC1(regval)      (BITS(4,8) & ((uint32_t)(regval) << 4))        /*!< XDC DRAM TMG9 FFC1: T_CMD_GEAR_FFC1 Bit Value */  
#define XDC_DRAM_TMG9_FFC1_T_SYNC_GEAR_FFC1_MASK        BITS(9,13)                                   /*!< XDC DRAM TMG9 FFC1: T_SYNC_GEAR_FFC1 Bit Mask */  
#define XDC_DRAM_TMG9_FFC1_T_SYNC_GEAR_FFC1_OFS         9U                                          /*!< XDC DRAM TMG9 FFC1: T_SYNC_GEAR_FFC1 Bit Offset */
#define XDC_DRAM_TMG9_FFC1_T_SYNC_GEAR_FFC1(regval)     (BITS(9,13) & ((uint32_t)(regval) << 9))        /*!< XDC DRAM TMG9 FFC1: T_SYNC_GEAR_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG10_FFC1 Register definition ===== */
#define XDC_DRAM_TMG10_FFC1_T_CKMPE_FFC1_MASK            BITS(0,4)                                   /*!< XDC DRAM TMG10 FFC1: T_CKMPE_FFC1 Bit Mask */  
#define XDC_DRAM_TMG10_FFC1_T_CKMPE_FFC1_OFS             0U                                          /*!< XDC DRAM TMG10 FFC1: T_CKMPE_FFC1 Bit Offset */
#define XDC_DRAM_TMG10_FFC1_T_CKMPE_FFC1(regval)         (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG10 FFC1: T_CKMPE_FFC1 Bit Value */  
#define XDC_DRAM_TMG10_FFC1_T_MPX_S_FFC1_MASK            BITS(5,6)                                   /*!< XDC DRAM TMG10 FFC1: T_MPX_S_FFC1 Bit Mask */  
#define XDC_DRAM_TMG10_FFC1_T_MPX_S_FFC1_OFS             5U                                          /*!< XDC DRAM TMG10 FFC1: T_MPX_S_FFC1 Bit Offset */
#define XDC_DRAM_TMG10_FFC1_T_MPX_S_FFC1(regval)         (BITS(5,6) & ((uint32_t)(regval) << 5))        /*!< XDC DRAM TMG10 FFC1: T_MPX_S_FFC1 Bit Value */  
#define XDC_DRAM_TMG10_FFC1_T_MPX_LH_FFC1_MASK           BITS(7,11)                                   /*!< XDC DRAM TMG10 FFC1: T_MPX_LH_FFC1 Bit Mask */  
#define XDC_DRAM_TMG10_FFC1_T_MPX_LH_FFC1_OFS            7U                                          /*!< XDC DRAM TMG10 FFC1: T_MPX_LH_FFC1 Bit Offset */
#define XDC_DRAM_TMG10_FFC1_T_MPX_LH_FFC1(regval)        (BITS(7,11) & ((uint32_t)(regval) << 7))        /*!< XDC DRAM TMG10 FFC1: T_MPX_LH_FFC1 Bit Value */  
#define XDC_DRAM_TMG10_FFC1_T_XMP_DLL_FFC1_MASK          BITS(12,18)                                   /*!< XDC DRAM TMG10 FFC1: T_XMP_DLL_FFC1 Bit Mask */  
#define XDC_DRAM_TMG10_FFC1_T_XMP_DLL_FFC1_OFS           12U                                          /*!< XDC DRAM TMG10 FFC1: T_XMP_DLL_FFC1 Bit Offset */
#define XDC_DRAM_TMG10_FFC1_T_XMP_DLL_FFC1(regval)       (BITS(12,18) & ((uint32_t)(regval) << 12))        /*!< XDC DRAM TMG10 FFC1: T_XMP_DLL_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG11_FFC1 Register definition ===== */
#define XDC_DRAM_TMG11_FFC1_T_CMDCKE_FFC1_MASK           BITS(0,1)                                   /*!< XDC DRAM TMG11 FFC1: T_CMDCKE_FFC1 Bit Mask */  
#define XDC_DRAM_TMG11_FFC1_T_CMDCKE_FFC1_OFS            0U                                          /*!< XDC DRAM TMG11 FFC1: T_CMDCKE_FFC1 Bit Offset */
#define XDC_DRAM_TMG11_FFC1_T_CMDCKE_FFC1(regval)        (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG11 FFC1: T_CMDCKE_FFC1 Bit Value */  
#define XDC_DRAM_TMG11_FFC1_T_XSR_FFC1_MASK              BITS(2,13)                                   /*!< XDC DRAM TMG11 FFC1: T_XSR_FFC1 Bit Mask */  
#define XDC_DRAM_TMG11_FFC1_T_XSR_FFC1_OFS               2U                                          /*!< XDC DRAM TMG11 FFC1: T_XSR_FFC1 Bit Offset */
#define XDC_DRAM_TMG11_FFC1_T_XSR_FFC1(regval)           (BITS(2,13) & ((uint32_t)(regval) << 2))        /*!< XDC DRAM TMG11 FFC1: T_XSR_FFC1 Bit Value */  
#define XDC_DRAM_TMG11_FFC1_T_CCD_MW_FFC1_MASK           BITS(14,19)                                   /*!< XDC DRAM TMG11 FFC1: T_CCD_MW_FFC1 Bit Mask */  
#define XDC_DRAM_TMG11_FFC1_T_CCD_MW_FFC1_OFS            14U                                          /*!< XDC DRAM TMG11 FFC1: T_CCD_MW_FFC1 Bit Offset */
#define XDC_DRAM_TMG11_FFC1_T_CCD_MW_FFC1(regval)        (BITS(14,19) & ((uint32_t)(regval) << 14))        /*!< XDC DRAM TMG11 FFC1: T_CCD_MW_FFC1 Bit Value */  
#define XDC_DRAM_TMG11_FFC1_T_ODTL_OFF_FFC1_MASK         BITS(20,26)                                   /*!< XDC DRAM TMG11 FFC1: T_ODTL_OFF_FFC1 Bit Mask */  
#define XDC_DRAM_TMG11_FFC1_T_ODTL_OFF_FFC1_OFS          20U                                          /*!< XDC DRAM TMG11 FFC1: T_ODTL_OFF_FFC1 Bit Offset */
#define XDC_DRAM_TMG11_FFC1_T_ODTL_OFF_FFC1(regval)      (BITS(20,26) & ((uint32_t)(regval) << 20))        /*!< XDC DRAM TMG11 FFC1: T_ODTL_OFF_FFC1 Bit Value */  
#define XDC_DRAM_TMG11_FFC1_T_PPD_FFC1_MASK              BITS(28,30)                                   /*!< XDC DRAM TMG11 FFC1: T_PPD_FFC1 Bit Mask */  
#define XDC_DRAM_TMG11_FFC1_T_PPD_FFC1_OFS               28U                                          /*!< XDC DRAM TMG11 FFC1: T_PPD_FFC1 Bit Offset */
#define XDC_DRAM_TMG11_FFC1_T_PPD_FFC1(regval)           (BITS(28,30) & ((uint32_t)(regval) << 28))        /*!< XDC DRAM TMG11 FFC1: T_PPD_FFC1 Bit Value */  
 
 /* ===== XDC DRAM_TMG12_FFC1 Register definition ===== */
#define XDC_DRAM_TMG12_FFC1_T_VRCG_EN_FFC1_MASK          BITS(0,7)                                   /*!< XDC DRAM TMG12 FFC1: T_VRCG_EN_FFC1 Bit Mask */  
#define XDC_DRAM_TMG12_FFC1_T_VRCG_EN_FFC1_OFS           0U                                          /*!< XDC DRAM TMG12 FFC1: T_VRCG_EN_FFC1 Bit Offset */
#define XDC_DRAM_TMG12_FFC1_T_VRCG_EN_FFC1(regval)       (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< XDC DRAM TMG12 FFC1: T_VRCG_EN_FFC1 Bit Value */  
#define XDC_DRAM_TMG12_FFC1_T_VRCG_DIS_FFC1_MASK         BITS(8,15)                                   /*!< XDC DRAM TMG12 FFC1: T_VRCG_DIS_FFC1 Bit Mask */  
#define XDC_DRAM_TMG12_FFC1_T_VRCG_DIS_FFC1_OFS          8U                                          /*!< XDC DRAM TMG12 FFC1: T_VRCG_DIS_FFC1 Bit Offset */
#define XDC_DRAM_TMG12_FFC1_T_VRCG_DIS_FFC1(regval)      (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< XDC DRAM TMG12 FFC1: T_VRCG_DIS_FFC1 Bit Value */  
 
 /* ===== XDC DERATE_TMG_FFC1 Register definition ===== */
#define XDC_DERATE_TMG_FFC1_DERATE_VAL_FFC1_MASK         BITS(0,1)                                   /*!< XDC DERATE TMG FFC1: DERATE_VAL_FFC1 Bit Mask */  
#define XDC_DERATE_TMG_FFC1_DERATE_VAL_FFC1_OFS          0U                                          /*!< XDC DERATE TMG FFC1: DERATE_VAL_FFC1 Bit Offset */
#define XDC_DERATE_TMG_FFC1_DERATE_VAL_FFC1(regval)      (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< XDC DERATE TMG FFC1: DERATE_VAL_FFC1 Bit Value */  
#define XDC_DERATE_TMG_FFC1_RC_DERATE_VAL_FFC1_MASK      BITS(4,6)                                   /*!< XDC DERATE TMG FFC1: RC_DERATE_VAL_FFC1 Bit Mask */  
#define XDC_DERATE_TMG_FFC1_RC_DERATE_VAL_FFC1_OFS       4U                                          /*!< XDC DERATE TMG FFC1: RC_DERATE_VAL_FFC1 Bit Offset */
#define XDC_DERATE_TMG_FFC1_RC_DERATE_VAL_FFC1(regval)   (BITS(4,6) & ((uint32_t)(regval) << 4))        /*!< XDC DERATE TMG FFC1: RC_DERATE_VAL_FFC1 Bit Value */  
#define XDC_DERATE_TMG_FFC1_TS_RD_INTRVL_FFC1_MASK       BITS(8,31)                                   /*!< XDC DERATE TMG FFC1: TS_RD_INTRVL_FFC1 Bit Mask */  
#define XDC_DERATE_TMG_FFC1_TS_RD_INTRVL_FFC1_OFS        8U                                          /*!< XDC DERATE TMG FFC1: TS_RD_INTRVL_FFC1 Bit Offset */
#define XDC_DERATE_TMG_FFC1_TS_RD_INTRVL_FFC1(regval)    (BITS(8,31) & ((uint32_t)(regval) << 8))        /*!< XDC DERATE TMG FFC1: TS_RD_INTRVL_FFC1 Bit Value */  
 
 /* ===== XDC ZQCAL_TMG0_FFC1 Register definition ===== */
#define XDC_ZQCAL_TMG0_FFC1_T_ZQCS_FFC1_MASK             BITS(0,9)                                   /*!< XDC ZQCAL TMG0 FFC1: T_ZQCS_FFC1 Bit Mask */  
#define XDC_ZQCAL_TMG0_FFC1_T_ZQCS_FFC1_OFS              0U                                          /*!< XDC ZQCAL TMG0 FFC1: T_ZQCS_FFC1 Bit Offset */
#define XDC_ZQCAL_TMG0_FFC1_T_ZQCS_FFC1(regval)          (BITS(0,9) & ((uint32_t)(regval) << 0))        /*!< XDC ZQCAL TMG0 FFC1: T_ZQCS_FFC1 Bit Value */  
#define XDC_ZQCAL_TMG0_FFC1_T_ZQOPER_FFC1_MASK           BITS(10,20)                                   /*!< XDC ZQCAL TMG0 FFC1: T_ZQOPER_FFC1 Bit Mask */  
#define XDC_ZQCAL_TMG0_FFC1_T_ZQOPER_FFC1_OFS            10U                                          /*!< XDC ZQCAL TMG0 FFC1: T_ZQOPER_FFC1 Bit Offset */
#define XDC_ZQCAL_TMG0_FFC1_T_ZQOPER_FFC1(regval)        (BITS(10,20) & ((uint32_t)(regval) << 10))        /*!< XDC ZQCAL TMG0 FFC1: T_ZQOPER_FFC1 Bit Value */  
 
 /* ===== XDC ZQCAL_TMG1_FFC1 Register definition ===== */
#define XDC_ZQCAL_TMG1_FFC1_T_ZQCS_INT_FFC1_MASK         BITS(0,19)                                   /*!< XDC ZQCAL TMG1 FFC1: T_ZQCS_INT_FFC1 Bit Mask */  
#define XDC_ZQCAL_TMG1_FFC1_T_ZQCS_INT_FFC1_OFS          0U                                          /*!< XDC ZQCAL TMG1 FFC1: T_ZQCS_INT_FFC1 Bit Offset */
#define XDC_ZQCAL_TMG1_FFC1_T_ZQCS_INT_FFC1(regval)      (BITS(0,19) & ((uint32_t)(regval) << 0))        /*!< XDC ZQCAL TMG1 FFC1: T_ZQCS_INT_FFC1 Bit Value */  
 
 /* ===== XDC DFI_TMG0_FFC1 Register definition ===== */
#define XDC_DFI_TMG0_FFC1_DFI_TPHY_WRLAT_FFC1_MASK     BITS(0,5)                                   /*!< XDC DFI TMG0 FFC1: DFI_TPHY_WRLAT_FFC1 Bit Mask */  
#define XDC_DFI_TMG0_FFC1_DFI_TPHY_WRLAT_FFC1_OFS      0U                                          /*!< XDC DFI TMG0 FFC1: DFI_TPHY_WRLAT_FFC1 Bit Offset */
#define XDC_DFI_TMG0_FFC1_DFI_TPHY_WRLAT_FFC1(regval)  (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC DFI TMG0 FFC1: DFI_TPHY_WRLAT_FFC1 Bit Value */  
#define XDC_DFI_TMG0_FFC1_DFI_TPHY_WRDATA_FFC1_MASK    BITS(6,11)                                   /*!< XDC DFI TMG0 FFC1: DFI_TPHY_WRDATA_FFC1 Bit Mask */  
#define XDC_DFI_TMG0_FFC1_DFI_TPHY_WRDATA_FFC1_OFS     6U                                          /*!< XDC DFI TMG0 FFC1: DFI_TPHY_WRDATA_FFC1 Bit Offset */
#define XDC_DFI_TMG0_FFC1_DFI_TPHY_WRDATA_FFC1(regval) (BITS(6,11) & ((uint32_t)(regval) << 6))        /*!< XDC DFI TMG0 FFC1: DFI_TPHY_WRDATA_FFC1 Bit Value */  
#define XDC_DFI_TMG0_FFC1_DFI_T_RDDATA_EN_FFC1_MASK    BITS(12,18)                                   /*!< XDC DFI TMG0 FFC1: DFI_T_RDDATA_EN_FFC1 Bit Mask */  
#define XDC_DFI_TMG0_FFC1_DFI_T_RDDATA_EN_FFC1_OFS     12U                                          /*!< XDC DFI TMG0 FFC1: DFI_T_RDDATA_EN_FFC1 Bit Offset */
#define XDC_DFI_TMG0_FFC1_DFI_T_RDDATA_EN_FFC1(regval) (BITS(12,18) & ((uint32_t)(regval) << 12))        /*!< XDC DFI TMG0 FFC1: DFI_T_RDDATA_EN_FFC1 Bit Value */  
#define XDC_DFI_TMG0_FFC1_DFI_T_CTRL_DELAY_FFC1_MASK    BITS(19,23)                                   /*!< XDC DFI TMG0 FFC1: DFI_T_CTRL_DELAY_FFC1 Bit Mask */  
#define XDC_DFI_TMG0_FFC1_DFI_T_CTRL_DELAY_FFC1_OFS     19U                                          /*!< XDC DFI TMG0 FFC1: DFI_T_CTRL_DELAY_FFC1 Bit Offset */
#define XDC_DFI_TMG0_FFC1_DFI_T_CTRL_DELAY_FFC1(regval) (BITS(19,23) & ((uint32_t)(regval) << 19))        /*!< XDC DFI TMG0 FFC1: DFI_T_CTRL_DELAY_FFC1 Bit Value */  
#define XDC_DFI_TMG0_FFC1_DFI_T_CMD_LAT_FFC1_MASK      BITS(24,27)                                   /*!< XDC DFI TMG0 FFC1: DFI_T_CMD_LAT_FFC1 Bit Mask */  
#define XDC_DFI_TMG0_FFC1_DFI_T_CMD_LAT_FFC1_OFS       24U                                          /*!< XDC DFI TMG0 FFC1: DFI_T_CMD_LAT_FFC1 Bit Offset */
#define XDC_DFI_TMG0_FFC1_DFI_T_CMD_LAT_FFC1(regval)   (BITS(24,27) & ((uint32_t)(regval) << 24))        /*!< XDC DFI TMG0 FFC1: DFI_T_CMD_LAT_FFC1 Bit Value */  
#define XDC_DFI_TMG0_FFC1_DFI_WRDATA_SDR_FFC1          BIT(28)                                      /*!< Set the time unit used for timing parameters related to dfi write data path: 0 - use x1 XDC clock cycle as time unit. 1 - use x1 MEM clock cycle as time unit. */
#define XDC_DFI_TMG0_FFC1_DFI_WRDATA_SDR_FFC1_OFS      28U                                          /*!< XDC DFI TMG0 FFC1: DFI_WRDATA_SDR_FFC1 Bit Offset */
#define XDC_DFI_TMG0_FFC1_DFI_WRDATA_SDR_FFC1_VAL(regval)  (BIT(28) & ((uint32_t)(regval) << 28))        /*!< XDC DFI TMG0 FFC1: DFI_WRDATA_SDR_FFC1 Bit Value */  
#define XDC_DFI_TMG0_FFC1_DFI_RDDATA_SDR_FFC1          BIT(29)                                      /*!< Set the time unit used for timing parameters related to dfi read data path: 0 - use x1 XDC clock cycle as time unit. 1 - use x1 MEM clock cycle as time unit. */
#define XDC_DFI_TMG0_FFC1_DFI_RDDATA_SDR_FFC1_OFS      29U                                          /*!< XDC DFI TMG0 FFC1: DFI_RDDATA_SDR_FFC1 Bit Offset */
#define XDC_DFI_TMG0_FFC1_DFI_RDDATA_SDR_FFC1_VAL(regval)  (BIT(29) & ((uint32_t)(regval) << 29))        /*!< XDC DFI TMG0 FFC1: DFI_RDDATA_SDR_FFC1 Bit Value */  
 
 /* ===== XDC DFI_TMG1_FFC1 Register definition ===== */
#define XDC_DFI_TMG1_FFC1_DFI_T_DRAM_CLK_ENABLE_FFC1_MASK    BITS(0,4)                                   /*!< XDC DFI TMG1 FFC1: DFI_T_DRAM_CLK_ENABLE_FFC1 Bit Mask */  
#define XDC_DFI_TMG1_FFC1_DFI_T_DRAM_CLK_ENABLE_FFC1_OFS     0U                                          /*!< XDC DFI TMG1 FFC1: DFI_T_DRAM_CLK_ENABLE_FFC1 Bit Offset */
#define XDC_DFI_TMG1_FFC1_DFI_T_DRAM_CLK_ENABLE_FFC1(regval) (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC DFI TMG1 FFC1: DFI_T_DRAM_CLK_ENABLE_FFC1 Bit Value */  
#define XDC_DFI_TMG1_FFC1_DFI_T_DRAM_CLK_DISABLE_FFC1_MASK    BITS(5,9)                                   /*!< XDC DFI TMG1 FFC1: DFI_T_DRAM_CLK_DISABLE_FFC1 Bit Mask */  
#define XDC_DFI_TMG1_FFC1_DFI_T_DRAM_CLK_DISABLE_FFC1_OFS     5U                                          /*!< XDC DFI TMG1 FFC1: DFI_T_DRAM_CLK_DISABLE_FFC1 Bit Offset */
#define XDC_DFI_TMG1_FFC1_DFI_T_DRAM_CLK_DISABLE_FFC1(regval) (BITS(5,9) & ((uint32_t)(regval) << 5))        /*!< XDC DFI TMG1 FFC1: DFI_T_DRAM_CLK_DISABLE_FFC1 Bit Value */  
#define XDC_DFI_TMG1_FFC1_DFI_T_WRDATA_DELAY_FFC1_MASK    BITS(10,15)                                   /*!< XDC DFI TMG1 FFC1: DFI_T_WRDATA_DELAY_FFC1 Bit Mask */  
#define XDC_DFI_TMG1_FFC1_DFI_T_WRDATA_DELAY_FFC1_OFS     10U                                          /*!< XDC DFI TMG1 FFC1: DFI_T_WRDATA_DELAY_FFC1 Bit Offset */
#define XDC_DFI_TMG1_FFC1_DFI_T_WRDATA_DELAY_FFC1(regval) (BITS(10,15) & ((uint32_t)(regval) << 10))        /*!< XDC DFI TMG1 FFC1: DFI_T_WRDATA_DELAY_FFC1 Bit Value */  
#define XDC_DFI_TMG1_FFC1_DFI_T_PARIN_LAT_FFC1_MASK    BITS(16,17)                                   /*!< XDC DFI TMG1 FFC1: DFI_T_PARIN_LAT_FFC1 Bit Mask */  
#define XDC_DFI_TMG1_FFC1_DFI_T_PARIN_LAT_FFC1_OFS     16U                                          /*!< XDC DFI TMG1 FFC1: DFI_T_PARIN_LAT_FFC1 Bit Offset */
#define XDC_DFI_TMG1_FFC1_DFI_T_PARIN_LAT_FFC1(regval) (BITS(16,17) & ((uint32_t)(regval) << 16))        /*!< XDC DFI TMG1 FFC1: DFI_T_PARIN_LAT_FFC1 Bit Value */  
 
 /* ===== XDC DFI_TMG2_FFC1 Register definition ===== */
#define XDC_DFI_TMG2_FFC1_DFI_TPHY_WRCSLAT_FFC1_MASK    BITS(0,5)                                   /*!< XDC DFI TMG2 FFC1: DFI_TPHY_WRCSLAT_FFC1 Bit Mask */  
#define XDC_DFI_TMG2_FFC1_DFI_TPHY_WRCSLAT_FFC1_OFS     0U                                          /*!< XDC DFI TMG2 FFC1: DFI_TPHY_WRCSLAT_FFC1 Bit Offset */
#define XDC_DFI_TMG2_FFC1_DFI_TPHY_WRCSLAT_FFC1(regval) (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< XDC DFI TMG2 FFC1: DFI_TPHY_WRCSLAT_FFC1 Bit Value */  
#define XDC_DFI_TMG2_FFC1_DFI_TPHY_RDCSLAT_FFC1_MASK    BITS(6,12)                                   /*!< XDC DFI TMG2 FFC1: DFI_TPHY_RDCSLAT_FFC1 Bit Mask */  
#define XDC_DFI_TMG2_FFC1_DFI_TPHY_RDCSLAT_FFC1_OFS     6U                                          /*!< XDC DFI TMG2 FFC1: DFI_TPHY_RDCSLAT_FFC1 Bit Offset */
#define XDC_DFI_TMG2_FFC1_DFI_TPHY_RDCSLAT_FFC1(regval) (BITS(6,12) & ((uint32_t)(regval) << 6))        /*!< XDC DFI TMG2 FFC1: DFI_TPHY_RDCSLAT_FFC1 Bit Value */  
 
 /* ===== XDC DFI_TMG3_FFC1 Register definition ===== */
#define XDC_DFI_TMG3_FFC1_DFI_T_GEARDOWN_DELAY_FFC1_MASK    BITS(0,4)                                   /*!< XDC DFI TMG3 FFC1: DFI_T_GEARDOWN_DELAY_FFC1 Bit Mask */  
#define XDC_DFI_TMG3_FFC1_DFI_T_GEARDOWN_DELAY_FFC1_OFS     0U                                          /*!< XDC DFI TMG3 FFC1: DFI_T_GEARDOWN_DELAY_FFC1 Bit Offset */
#define XDC_DFI_TMG3_FFC1_DFI_T_GEARDOWN_DELAY_FFC1(regval) (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC DFI TMG3 FFC1: DFI_T_GEARDOWN_DELAY_FFC1 Bit Value */  
 
 /* ===== XDC ODT_TMG_FFC1 Register definition ===== */
#define XDC_ODT_TMG_FFC1_RD_ODT_DELAY_FFC1_MASK       BITS(0,4)                                   /*!< XDC ODT TMG FFC1: RD_ODT_DELAY_FFC1 Bit Mask */  
#define XDC_ODT_TMG_FFC1_RD_ODT_DELAY_FFC1_OFS        0U                                          /*!< XDC ODT TMG FFC1: RD_ODT_DELAY_FFC1 Bit Offset */
#define XDC_ODT_TMG_FFC1_RD_ODT_DELAY_FFC1(regval)    (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< XDC ODT TMG FFC1: RD_ODT_DELAY_FFC1 Bit Value */  
#define XDC_ODT_TMG_FFC1_RD_ODT_HOLD_FFC1_MASK        BITS(5,8)                                   /*!< XDC ODT TMG FFC1: RD_ODT_HOLD_FFC1 Bit Mask */  
#define XDC_ODT_TMG_FFC1_RD_ODT_HOLD_FFC1_OFS         5U                                          /*!< XDC ODT TMG FFC1: RD_ODT_HOLD_FFC1 Bit Offset */
#define XDC_ODT_TMG_FFC1_RD_ODT_HOLD_FFC1(regval)     (BITS(5,8) & ((uint32_t)(regval) << 5))        /*!< XDC ODT TMG FFC1: RD_ODT_HOLD_FFC1 Bit Value */  
#define XDC_ODT_TMG_FFC1_WR_ODT_DELAY_FFC1_MASK       BITS(9,13)                                   /*!< XDC ODT TMG FFC1: WR_ODT_DELAY_FFC1 Bit Mask */  
#define XDC_ODT_TMG_FFC1_WR_ODT_DELAY_FFC1_OFS        9U                                          /*!< XDC ODT TMG FFC1: WR_ODT_DELAY_FFC1 Bit Offset */
#define XDC_ODT_TMG_FFC1_WR_ODT_DELAY_FFC1(regval)    (BITS(9,13) & ((uint32_t)(regval) << 9))        /*!< XDC ODT TMG FFC1: WR_ODT_DELAY_FFC1 Bit Value */  
#define XDC_ODT_TMG_FFC1_WR_ODT_HOLD_FFC1_MASK        BITS(14,17)                                   /*!< XDC ODT TMG FFC1: WR_ODT_HOLD_FFC1 Bit Mask */  
#define XDC_ODT_TMG_FFC1_WR_ODT_HOLD_FFC1_OFS         14U                                          /*!< XDC ODT TMG FFC1: WR_ODT_HOLD_FFC1 Bit Offset */
#define XDC_ODT_TMG_FFC1_WR_ODT_HOLD_FFC1(regval)     (BITS(14,17) & ((uint32_t)(regval) << 14))        /*!< XDC ODT TMG FFC1: WR_ODT_HOLD_FFC1 Bit Value */  
 
 /* ===== XDC PWCFG0 Register definition ===== */
#define XDC_PWCFG0_WPORT_AGE_EN_P0              BIT(0)                                      /*!< Set the number of clocks of gap in data responses when performing consecutive reads to different ranks. */
#define XDC_PWCFG0_WPORT_AGE_EN_P0_OFS          0U                                          /*!< XDC PWCFG0: WPORT_AGE_EN_P0 Bit Offset */
#define XDC_PWCFG0_WPORT_AGE_EN_P0_VAL(regval)      (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC PWCFG0: WPORT_AGE_EN_P0 Bit Value */  
#define XDC_PWCFG0_WPORT_PRIO_P0_MASK           BITS(1,10)                                   /*!< XDC PWCFG0: WPORT_PRIO_P0 Bit Mask */  
#define XDC_PWCFG0_WPORT_PRIO_P0_OFS            1U                                          /*!< XDC PWCFG0: WPORT_PRIO_P0 Bit Offset */
#define XDC_PWCFG0_WPORT_PRIO_P0(regval)        (BITS(1,10) & ((uint32_t)(regval) << 1))        /*!< XDC PWCFG0: WPORT_PRIO_P0 Bit Value */  
 
 /* ===== XDC PRCFG0 Register definition ===== */
#define XDC_PRCFG0_RPORT_AGE_EN_P0              BIT(0)                                      /*!< Set the number of clocks of gap in data responses when performing consecutive reads to different ranks. */
#define XDC_PRCFG0_RPORT_AGE_EN_P0_OFS          0U                                          /*!< XDC PRCFG0: RPORT_AGE_EN_P0 Bit Offset */
#define XDC_PRCFG0_RPORT_AGE_EN_P0_VAL(regval)      (BIT(0) & ((uint32_t)(regval) << 0))        /*!< XDC PRCFG0: RPORT_AGE_EN_P0 Bit Value */  
#define XDC_PRCFG0_RPORT_PRIO_P0_MASK           BITS(1,10)                                   /*!< XDC PRCFG0: RPORT_PRIO_P0 Bit Mask */  
#define XDC_PRCFG0_RPORT_PRIO_P0_OFS            1U                                          /*!< XDC PRCFG0: RPORT_PRIO_P0 Bit Offset */
#define XDC_PRCFG0_RPORT_PRIO_P0(regval)        (BITS(1,10) & ((uint32_t)(regval) << 1))        /*!< XDC PRCFG0: RPORT_PRIO_P0 Bit Value */  

#define TOP_GNRL_PHY_VER_OFFSET                                            0x0 /*!< DDR PHY version */
#define TOP_GNRL_DDR_TYPE_OFFSET                                           0x4 /*!< DDR protocol, these fields must be one hot. */
#define TOP_GNRL_HM_DIS_CFG_OFFSET                                         0x8 /*!< Hard macro disable configuration */
#define TOP_GNRL_PAD_CAL_CFG_OFFSET                                        0x10 /*!< PAD calibration configuration */
#define TOP_GNRL_PAD_CAL_CODE_OFFSET                                       0x14 /*!< PAD calibration code */
#define TOP_GNRL_NXT_IO_PVT_CODE_OFFSET                                    0x18 /*!< Next PVT code for normal IO. These values will be updated to CUR_IO_PVT_CODE when dfi ctrlupd or dfi phyupd handshake has been completed. */
#define TOP_GNRL_CUR_IO_PVT_CODE_OFFSET                                    0x1c /*!< Current PVT code for normal IO */
#define TOP_GNRL_MCB_LPBK_CFG_OFFSET                                       0x28 /*!< Loop back configuration for mem clks */
#define TOP_GNRL_MCB0_LPBK_STATUS_OFFSET                                   0x30 /*!< Loop back configuration for MCB u0 */
#define TOP_GNRL_CBT_CFG0_OFFSET                                           0x50 /*!< Command bust training configuration */
#define TOP_GNRL_CBT_CFG1_OFFSET                                           0x54 /*!< Command bust training configuration */
#define TOP_GNRL_CBT_CFG2_OFFSET                                           0x58 /*!< Command bust training configuration */
#define TOP_GNRL_CBT_CFG3_OFFSET                                           0x5c /*!< Command bust training configuration */
#define TOP_GNRL_CBT_CFG4_OFFSET                                           0x60 /*!< Command bust training configuration */
#define TOP_GNRL_CBT_CFG5_OFFSET                                           0x64 /*!< Command bust training configuration */
#define TOP_GNRL_CBT_CFG6_OFFSET                                           0x68 /*!< Command bust training configuration */
#define TOP_GNRL_CBT_CFG7_OFFSET                                           0x6c /*!< Command bust training configuration */
#define TOP_GNRL_CBT_CFG8_OFFSET                                           0x70 /*!< Command bust training configuration */
#define TOP_GNRL_CBT_STATUS_OFFSET                                         0x74 /*!< Command bust training status */
#define TOP_GNRL_MISC_CFG_OFFSET                                           0x80 /*!< PHY top miscellaneous configuration */
#define TOP_GNRL_ACS4_MSTR_CFG_OFFSET                                      0x84 /*!< N/A */
#define TOP_GNRL_TOP_F0_TMG_OFFSET                                         0x90 /*!< PHY_TOP timing configuration for frequency 0 */
#define TOP_GNRL_TOP_F1_TMG_OFFSET                                         0x94 /*!< PHY_TOP timing configuration for frequency 1 */
#define TOP_OBS_CFG_OFFSET                                                0xc0 /*!< Reserved for debug */
#define TOP_OBS_MISC_CTRL_OBS_OFFSET                                      0xc4 /*!< Reserved for debug */
#define TOP_OBS_LP_OBS_OFFSET                                             0xc8 /*!< Reserved for debug */
#define TOP_OBS_PAD_CAL_OBS_OFFSET                                        0xcc /*!< Reserved for debug */
#define TOP_OBS_CBT_MISC_OBS_OFFSET                                       0xdc /*!< Reserved for debug */
#define TOP_OBS_CBT_CA_GRP0_LE_OBS_OFFSET                                 0xe0 /*!< Reserved for debug */
#define TOP_OBS_CBT_CA_GRP0_TE_OBS_OFFSET                                 0xe4 /*!< Reserved for debug */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_OFFSET                                   0x100 /*!< DDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_OFFSET                                   0x104 /*!< DDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_OFFSET                                   0x108 /*!< DDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_OFFSET                                   0x10c /*!< DDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_OFFSET                                   0x110 /*!< DDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_OFFSET                                   0x114 /*!< DDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_OFFSET                                   0x118 /*!< DDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG7_OFFSET                                   0x11c /*!< DDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_OFFSET                                  0x140 /*!< LPDDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG1_OFFSET                                  0x144 /*!< LPDDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_OFFSET                                  0x148 /*!< LPDDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG3_OFFSET                                  0x14c /*!< LPDDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG4_OFFSET                                  0x150 /*!< LPDDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG5_OFFSET                                  0x154 /*!< LPDDR34 DFI to ACS4 PAD swap configuration */
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_OFFSET                                      0x180 /*!< Swap configuration for DBYTE */
#define TOP_PAD_SWAP_DB0_SWAP_CFG1_OFFSET                                      0x184 /*!< Swap configuration for DBYTE */
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_OFFSET                                      0x188 /*!< Swap configuration for DBYTE */
#define TOP_PAD_SWAP_DB1_SWAP_CFG1_OFFSET                                      0x18c /*!< Swap configuration for DBYTE */
#define TOP_PAD_FUNC_SLEW_CFG_OFFSET                                           0x200 /*!< PAD slew_p/slew_n configuration register */
#define TOP_PAD_FUNC_RX_CAL_CFG_OFFSET                                         0x204 /*!< RX offset calibration code for the address/command/CLK PADs */
#define TOP_PAD_FUNC_TSEL_CFG_OFFSET                                           0x208 /*!< Impedance configuration for the CLK/alert PADs */
#define TOP_PAD_FUNC_MISC_PAD_CFG_OFFSET                                       0x20c /*!< Miscellaneous PAD configuration */
#define TOP_PAD_FUNC_AC_PD_CFG0_OFFSET                                         0x210 /*!< PAD power down configuration */
#define TOP_PAD_FUNC_MISC_PD_CFG_OFFSET                                        0x218 /*!< PAD power down configuration */
#define TOP_PAD_FUNC_DB_PD_CFG_OFFSET                                          0x21c /*!< PAD power down configuration */
#define TOP_PAD_ATB_AC_ATB_CFG0_OFFSET                                        0x300 /*!< PAD power down configuration */
#define TOP_PAD_ATB_AC_ATB_CFG1_OFFSET                                        0x304 /*!< PAD power down configuration */
#define TOP_PAD_ATB_MISC_ATB_CFG_OFFSET                                       0x30c /*!< Reserved */
#define TOP_PAD_ATB_DB_ATB_CFG0_OFFSET                                        0x310 /*!< Reserved */
#define TOP_PAD_ATB_DB_ATB_CFG1_OFFSET                                        0x314 /*!< Reserved */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_OFFSET                                    0x340 /*!< Reserved */
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_OFFSET                                    0x344 /*!< Reserved */

 /* ===== TOP GNRL PHY_VER Register definition ===== */
#define TOP_GNRL_PHY_VER_VERSION                      BITS(0,31)                
 
 /* ===== TOP GNRL DDR_TYPE Register definition ===== */
#define TOP_GNRL_DDR_TYPE_DDR4                         BIT(1)                                      /*!< Selects DDR4 SDRAM 1 - DDR4 SDRAM device in use. 0 - non-DDR4 SDRAM device in use. */
#define TOP_GNRL_DDR_TYPE_DDR4_OFS                     1U                                          /*!< TOP GNRL DDR TYPE: DDR4 Bit Offset */
#define TOP_GNRL_DDR_TYPE_DDR4_VAL(regval)                 (BIT(1) & ((uint32_t)(regval) << 1))        /*!< TOP GNRL DDR TYPE: DDR4 Bit Value */  
#define TOP_GNRL_DDR_TYPE_LP4                          BIT(3)                                      /*!< Selects LPDDR4 SDRAM 1 - LPDDR4 SDRAM device in use. 0 - non-LPDDR4 SDRAM device in use. */
#define TOP_GNRL_DDR_TYPE_LP4_OFS                      3U                                          /*!< TOP GNRL DDR TYPE: LP4 Bit Offset */
#define TOP_GNRL_DDR_TYPE_LP4_VAL(regval)                  (BIT(3) & ((uint32_t)(regval) << 3))        /*!< TOP GNRL DDR TYPE: LP4 Bit Value */  
 
 /* ===== TOP GNRL HM_DIS_CFG Register definition ===== */
#define TOP_GNRL_HM_DIS_CFG_DB_DISABLE_MASK              BITS(0,1)                                   /*!< TOP GNRL HM DIS CFG: DB_DISABLE Bit Mask */  
#define TOP_GNRL_HM_DIS_CFG_DB_DISABLE_OFS               0U                                          /*!< TOP GNRL HM DIS CFG: DB_DISABLE Bit Offset */
#define TOP_GNRL_HM_DIS_CFG_DB_DISABLE(regval)           (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL HM DIS CFG: DB_DISABLE Bit Value */  
#define TOP_GNRL_HM_DIS_CFG_AC_DISABLE_MASK              BITS(12,19)                                   /*!< TOP GNRL HM DIS CFG: AC_DISABLE Bit Mask */  
#define TOP_GNRL_HM_DIS_CFG_AC_DISABLE_OFS               12U                                          /*!< TOP GNRL HM DIS CFG: AC_DISABLE Bit Offset */
#define TOP_GNRL_HM_DIS_CFG_AC_DISABLE(regval)           (BITS(12,19) & ((uint32_t)(regval) << 12))        /*!< TOP GNRL HM DIS CFG: AC_DISABLE Bit Value */  
 
 /* ===== TOP GNRL PAD_CAL_CFG Register definition ===== */
#define TOP_GNRL_PAD_CAL_CFG_INIT_CAL_EN                  BIT(0)                                      /*!< 1 - Enable pad calibration during first DFI init 0 - Disable pad calibration during first DFI init */
#define TOP_GNRL_PAD_CAL_CFG_INIT_CAL_EN_OFS              0U                                          /*!< TOP GNRL PAD CAL CFG: INIT_CAL_EN Bit Offset */
#define TOP_GNRL_PAD_CAL_CFG_INIT_CAL_EN_VAL(regval)          (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL PAD CAL CFG: INIT_CAL_EN Bit Value */  
#define TOP_GNRL_PAD_CAL_CFG_INTRVL_CAL_EN                BIT(1)                                      /*!< 1 - Enable periodically pad calibration in mission mode 0 - Disable periodically pad calibration in mission mode */
#define TOP_GNRL_PAD_CAL_CFG_INTRVL_CAL_EN_OFS            1U                                          /*!< TOP GNRL PAD CAL CFG: INTRVL_CAL_EN Bit Offset */
#define TOP_GNRL_PAD_CAL_CFG_INTRVL_CAL_EN_VAL(regval)        (BIT(1) & ((uint32_t)(regval) << 1))        /*!< TOP GNRL PAD CAL CFG: INTRVL_CAL_EN Bit Value */  
#define TOP_GNRL_PAD_CAL_CFG_SW_TRG                       BIT(2)                                      /*!< Setting this field to 1 to trigger pad calibration. Note: The hardware automatically clears this bit at the next clock cycle. */
#define TOP_GNRL_PAD_CAL_CFG_SW_TRG_OFS                   2U                                          /*!< TOP GNRL PAD CAL CFG: SW_TRG Bit Offset */
#define TOP_GNRL_PAD_CAL_CFG_SW_TRG_VAL(regval)               (BIT(2) & ((uint32_t)(regval) << 2))        /*!< TOP GNRL PAD CAL CFG: SW_TRG Bit Value */  
#define TOP_GNRL_PAD_CAL_CFG_WAIT_CYC_MASK                BITS(8,15)                                   /*!< TOP GNRL PAD CAL CFG: WAIT_CYC Bit Mask */  
#define TOP_GNRL_PAD_CAL_CFG_WAIT_CYC_OFS                 8U                                          /*!< TOP GNRL PAD CAL CFG: WAIT_CYC Bit Offset */
#define TOP_GNRL_PAD_CAL_CFG_WAIT_CYC(regval)             (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< TOP GNRL PAD CAL CFG: WAIT_CYC Bit Value */  
 
 /* ===== TOP GNRL PAD_CAL_CODE Register definition ===== */
#define TOP_GNRL_PAD_CAL_CODE_PVTP_MASK                    BITS(0,5)                                   /*!< TOP GNRL PAD CAL CODE: PVTP Bit Mask */  
#define TOP_GNRL_PAD_CAL_CODE_PVTP_OFS                     0U                                          /*!< TOP GNRL PAD CAL CODE: PVTP Bit Offset */
#define TOP_GNRL_PAD_CAL_CODE_PVTP(regval)                 (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL PAD CAL CODE: PVTP Bit Value */  
#define TOP_GNRL_PAD_CAL_CODE_PVTN_MASK                    BITS(8,13)                                   /*!< TOP GNRL PAD CAL CODE: PVTN Bit Mask */  
#define TOP_GNRL_PAD_CAL_CODE_PVTN_OFS                     8U                                          /*!< TOP GNRL PAD CAL CODE: PVTN Bit Offset */
#define TOP_GNRL_PAD_CAL_CODE_PVTN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< TOP GNRL PAD CAL CODE: PVTN Bit Value */  
 
 /* ===== TOP GNRL NXT_IO_PVT_CODE Register definition ===== */
#define TOP_GNRL_NXT_IO_PVT_CODE_PVTP_MASK                    BITS(0,5)                                   /*!< TOP GNRL NXT IO PVT CODE: PVTP Bit Mask */  
#define TOP_GNRL_NXT_IO_PVT_CODE_PVTP_OFS                     0U                                          /*!< TOP GNRL NXT IO PVT CODE: PVTP Bit Offset */
#define TOP_GNRL_NXT_IO_PVT_CODE_PVTP(regval)                 (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL NXT IO PVT CODE: PVTP Bit Value */  
#define TOP_GNRL_NXT_IO_PVT_CODE_PVTN_MASK                    BITS(8,13)                                   /*!< TOP GNRL NXT IO PVT CODE: PVTN Bit Mask */  
#define TOP_GNRL_NXT_IO_PVT_CODE_PVTN_OFS                     8U                                          /*!< TOP GNRL NXT IO PVT CODE: PVTN Bit Offset */
#define TOP_GNRL_NXT_IO_PVT_CODE_PVTN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< TOP GNRL NXT IO PVT CODE: PVTN Bit Value */  
 
 /* ===== TOP GNRL CUR_IO_PVT_CODE Register definition ===== */
#define TOP_GNRL_CUR_IO_PVT_CODE_PVTP_MASK                    BITS(0,5)                                   /*!< TOP GNRL CUR IO PVT CODE: PVTP Bit Mask */  
#define TOP_GNRL_CUR_IO_PVT_CODE_PVTP_OFS                     0U                                          /*!< TOP GNRL CUR IO PVT CODE: PVTP Bit Offset */
#define TOP_GNRL_CUR_IO_PVT_CODE_PVTP(regval)                 (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL CUR IO PVT CODE: PVTP Bit Value */  
#define TOP_GNRL_CUR_IO_PVT_CODE_PVTN_MASK                    BITS(8,13)                                   /*!< TOP GNRL CUR IO PVT CODE: PVTN Bit Mask */  
#define TOP_GNRL_CUR_IO_PVT_CODE_PVTN_OFS                     8U                                          /*!< TOP GNRL CUR IO PVT CODE: PVTN Bit Offset */
#define TOP_GNRL_CUR_IO_PVT_CODE_PVTN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< TOP GNRL CUR IO PVT CODE: PVTN Bit Value */  
#define TOP_GNRL_CUR_IO_PVT_CODE_PVTR_MASK                    BITS(16,20)                                   /*!< TOP GNRL CUR IO PVT CODE: PVTR Bit Mask */  
#define TOP_GNRL_CUR_IO_PVT_CODE_PVTR_OFS                     16U                                          /*!< TOP GNRL CUR IO PVT CODE: PVTR Bit Offset */
#define TOP_GNRL_CUR_IO_PVT_CODE_PVTR(regval)                 (BITS(16,20) & ((uint32_t)(regval) << 16))        /*!< TOP GNRL CUR IO PVT CODE: PVTR Bit Value */  
 
 /* ===== TOP GNRL MCB_LPBK_CFG Register definition ===== */
#define TOP_GNRL_MCB_LPBK_CFG_EN                           BIT(0)                                      /*!< 1 - Enable loop back mode for CLK PADs. 0 - Disable loop back mode for CLK PADs. */
#define TOP_GNRL_MCB_LPBK_CFG_EN_OFS                       0U                                          /*!< TOP GNRL MCB LPBK CFG: EN Bit Offset */
#define TOP_GNRL_MCB_LPBK_CFG_EN_VAL(regval)                   (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL MCB LPBK CFG: EN Bit Value */  
#define TOP_GNRL_MCB_LPBK_CFG_GO                           BIT(1)                                      /*!< 1 - Start loop back test 0 - Stop loop back test */
#define TOP_GNRL_MCB_LPBK_CFG_GO_OFS                       1U                                          /*!< TOP GNRL MCB LPBK CFG: GO Bit Offset */
#define TOP_GNRL_MCB_LPBK_CFG_GO_VAL(regval)                   (BIT(1) & ((uint32_t)(regval) << 1))        /*!< TOP GNRL MCB LPBK CFG: GO Bit Value */  
#define TOP_GNRL_MCB_LPBK_CFG_EXT                          BIT(3)                                      /*!< 1 - External loop back 0 - Internal loop back */
#define TOP_GNRL_MCB_LPBK_CFG_EXT_OFS                      3U                                          /*!< TOP GNRL MCB LPBK CFG: EXT Bit Offset */
#define TOP_GNRL_MCB_LPBK_CFG_EXT_VAL(regval)                  (BIT(3) & ((uint32_t)(regval) << 3))        /*!< TOP GNRL MCB LPBK CFG: EXT Bit Value */  
#define TOP_GNRL_MCB_LPBK_CFG_SAMP_CNT_MASK                BITS(4,5)                                   /*!< TOP GNRL MCB LPBK CFG: SAMP_CNT Bit Mask */  
#define TOP_GNRL_MCB_LPBK_CFG_SAMP_CNT_OFS                 4U                                          /*!< TOP GNRL MCB LPBK CFG: SAMP_CNT Bit Offset */
#define TOP_GNRL_MCB_LPBK_CFG_SAMP_CNT(regval)             (BITS(4,5) & ((uint32_t)(regval) << 4))        /*!< TOP GNRL MCB LPBK CFG: SAMP_CNT Bit Value */  
#define TOP_GNRL_MCB_LPBK_CFG_CLR                          BIT(8)                                      /*!< Setting this field to 1 causes the loop back status to be cleared. Note: The hardware automatically clears this bit at the next clock cycle. */
#define TOP_GNRL_MCB_LPBK_CFG_CLR_OFS                      8U                                          /*!< TOP GNRL MCB LPBK CFG: CLR Bit Offset */
#define TOP_GNRL_MCB_LPBK_CFG_CLR_VAL(regval)                  (BIT(8) & ((uint32_t)(regval) << 8))        /*!< TOP GNRL MCB LPBK CFG: CLR Bit Value */  
 
 /* ===== TOP GNRL MCB0_LPBK_STATUS Register definition ===== */
#define TOP_GNRL_MCB0_LPBK_STATUS_DONE                         BIT(0)                                      /*!< Loopback done status */
#define TOP_GNRL_MCB0_LPBK_STATUS_ERR                          BIT(1)                                      /*!< Loopback error status */
#define TOP_GNRL_MCB0_LPBK_STATUS_CNT                          BITS(16,31)                
 
 /* ===== TOP GNRL CBT_CFG0 Register definition ===== */
#define TOP_GNRL_CBT_CFG0_PAT_CNT_MASK                 BITS(0,2)                                   /*!< TOP GNRL CBT CFG0: PAT_CNT Bit Mask */  
#define TOP_GNRL_CBT_CFG0_PAT_CNT_OFS                  0U                                          /*!< TOP GNRL CBT CFG0: PAT_CNT Bit Offset */
#define TOP_GNRL_CBT_CFG0_PAT_CNT(regval)              (BITS(0,2) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL CBT CFG0: PAT_CNT Bit Value */  
#define TOP_GNRL_CBT_CFG0_CAPT_CNT_MASK                BITS(4,6)                                   /*!< TOP GNRL CBT CFG0: CAPT_CNT Bit Mask */  
#define TOP_GNRL_CBT_CFG0_CAPT_CNT_OFS                 4U                                          /*!< TOP GNRL CBT CFG0: CAPT_CNT Bit Offset */
#define TOP_GNRL_CBT_CFG0_CAPT_CNT(regval)             (BITS(4,6) & ((uint32_t)(regval) << 4))        /*!< TOP GNRL CBT CFG0: CAPT_CNT Bit Value */  
#define TOP_GNRL_CBT_CFG0_CDLY_STEP_MASK               BITS(8,11)                                   /*!< TOP GNRL CBT CFG0: CDLY_STEP Bit Mask */  
#define TOP_GNRL_CBT_CFG0_CDLY_STEP_OFS                8U                                          /*!< TOP GNRL CBT CFG0: CDLY_STEP Bit Offset */
#define TOP_GNRL_CBT_CFG0_CDLY_STEP(regval)            (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< TOP GNRL CBT CFG0: CDLY_STEP Bit Value */  
#define TOP_GNRL_CBT_CFG0_FDLY_STEP_MASK               BITS(12,15)                                   /*!< TOP GNRL CBT CFG0: FDLY_STEP Bit Mask */  
#define TOP_GNRL_CBT_CFG0_FDLY_STEP_OFS                12U                                          /*!< TOP GNRL CBT CFG0: FDLY_STEP Bit Offset */
#define TOP_GNRL_CBT_CFG0_FDLY_STEP(regval)            (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< TOP GNRL CBT CFG0: FDLY_STEP Bit Value */  
#define TOP_GNRL_CBT_CFG0_UPDT_WAIT_CYC_MASK           BITS(16,18)                                   /*!< TOP GNRL CBT CFG0: UPDT_WAIT_CYC Bit Mask */  
#define TOP_GNRL_CBT_CFG0_UPDT_WAIT_CYC_OFS            16U                                          /*!< TOP GNRL CBT CFG0: UPDT_WAIT_CYC Bit Offset */
#define TOP_GNRL_CBT_CFG0_UPDT_WAIT_CYC(regval)        (BITS(16,18) & ((uint32_t)(regval) << 16))        /*!< TOP GNRL CBT CFG0: UPDT_WAIT_CYC Bit Value */  
#define TOP_GNRL_CBT_CFG0_BYTE_MODE_MASK               BITS(20,21)                                   /*!< TOP GNRL CBT CFG0: BYTE_MODE Bit Mask */  
#define TOP_GNRL_CBT_CFG0_BYTE_MODE_OFS                20U                                          /*!< TOP GNRL CBT CFG0: BYTE_MODE Bit Offset */
#define TOP_GNRL_CBT_CFG0_BYTE_MODE(regval)            (BITS(20,21) & ((uint32_t)(regval) << 20))        /*!< TOP GNRL CBT CFG0: BYTE_MODE Bit Value */  
#define TOP_GNRL_CBT_CFG0_DBG_MODE                     BIT(28)                                      /*!< 1 - Run training in debug mode 0 - Run training in normal mode */
#define TOP_GNRL_CBT_CFG0_DBG_MODE_OFS                 28U                                          /*!< TOP GNRL CBT CFG0: DBG_MODE Bit Offset */
#define TOP_GNRL_CBT_CFG0_DBG_MODE_VAL(regval)             (BIT(28) & ((uint32_t)(regval) << 28))        /*!< TOP GNRL CBT CFG0: DBG_MODE Bit Value */  
#define TOP_GNRL_CBT_CFG0_DBG_CONT                     BIT(29)                                      /*!< 1 - Continue to run training 0 - Pause FSM at analyze state Note: The hardware automatically clears this bit at the next clock cycle. */
#define TOP_GNRL_CBT_CFG0_DBG_CONT_OFS                 29U                                          /*!< TOP GNRL CBT CFG0: DBG_CONT Bit Offset */
#define TOP_GNRL_CBT_CFG0_DBG_CONT_VAL(regval)             (BIT(29) & ((uint32_t)(regval) << 29))        /*!< TOP GNRL CBT CFG0: DBG_CONT Bit Value */  
#define TOP_GNRL_CBT_CFG0_CLR                          BIT(30)                                      /*!< Setting this field to 1 causes the training status to be cleared. Note: The hardware automatically clears this bit at the next clock cycle. */
#define TOP_GNRL_CBT_CFG0_CLR_OFS                      30U                                          /*!< TOP GNRL CBT CFG0: CLR Bit Offset */
#define TOP_GNRL_CBT_CFG0_CLR_VAL(regval)                  (BIT(30) & ((uint32_t)(regval) << 30))        /*!< TOP GNRL CBT CFG0: CLR Bit Value */  
 
 /* ===== TOP GNRL CBT_CFG1 Register definition ===== */
#define TOP_GNRL_CBT_CFG1_CS_START_MASK                BITS(0,10)                                   /*!< TOP GNRL CBT CFG1: CS_START Bit Mask */  
#define TOP_GNRL_CBT_CFG1_CS_START_OFS                 0U                                          /*!< TOP GNRL CBT CFG1: CS_START Bit Offset */
#define TOP_GNRL_CBT_CFG1_CS_START(regval)             (BITS(0,10) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL CBT CFG1: CS_START Bit Value */  
#define TOP_GNRL_CBT_CFG1_CS_END_MASK                  BITS(16,26)                                   /*!< TOP GNRL CBT CFG1: CS_END Bit Mask */  
#define TOP_GNRL_CBT_CFG1_CS_END_OFS                   16U                                          /*!< TOP GNRL CBT CFG1: CS_END Bit Offset */
#define TOP_GNRL_CBT_CFG1_CS_END(regval)               (BITS(16,26) & ((uint32_t)(regval) << 16))        /*!< TOP GNRL CBT CFG1: CS_END Bit Value */  
 
 /* ===== TOP GNRL CBT_CFG2 Register definition ===== */
#define TOP_GNRL_CBT_CFG2_CA_START_MASK                BITS(0,10)                                   /*!< TOP GNRL CBT CFG2: CA_START Bit Mask */  
#define TOP_GNRL_CBT_CFG2_CA_START_OFS                 0U                                          /*!< TOP GNRL CBT CFG2: CA_START Bit Offset */
#define TOP_GNRL_CBT_CFG2_CA_START(regval)             (BITS(0,10) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL CBT CFG2: CA_START Bit Value */  
#define TOP_GNRL_CBT_CFG2_CA_END_MASK                  BITS(16,26)                                   /*!< TOP GNRL CBT CFG2: CA_END Bit Mask */  
#define TOP_GNRL_CBT_CFG2_CA_END_OFS                   16U                                          /*!< TOP GNRL CBT CFG2: CA_END Bit Offset */
#define TOP_GNRL_CBT_CFG2_CA_END(regval)               (BITS(16,26) & ((uint32_t)(regval) << 16))        /*!< TOP GNRL CBT CFG2: CA_END Bit Value */  
 
 /* ===== TOP GNRL CBT_CFG3 Register definition ===== */
#define TOP_GNRL_CBT_CFG3_CS_MARGIN_MASK               BITS(0,7)                                   /*!< TOP GNRL CBT CFG3: CS_MARGIN Bit Mask */  
#define TOP_GNRL_CBT_CFG3_CS_MARGIN_OFS                0U                                          /*!< TOP GNRL CBT CFG3: CS_MARGIN Bit Offset */
#define TOP_GNRL_CBT_CFG3_CS_MARGIN(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL CBT CFG3: CS_MARGIN Bit Value */  
#define TOP_GNRL_CBT_CFG3_CA_MARGIN_MASK               BITS(8,15)                                   /*!< TOP GNRL CBT CFG3: CA_MARGIN Bit Mask */  
#define TOP_GNRL_CBT_CFG3_CA_MARGIN_OFS                8U                                          /*!< TOP GNRL CBT CFG3: CA_MARGIN Bit Offset */
#define TOP_GNRL_CBT_CFG3_CA_MARGIN(regval)            (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< TOP GNRL CBT CFG3: CA_MARGIN Bit Value */  
#define TOP_GNRL_CBT_CFG3_CS_WIN_SIZ_THR_MASK          BITS(16,23)                                   /*!< TOP GNRL CBT CFG3: CS_WIN_SIZ_THR Bit Mask */  
#define TOP_GNRL_CBT_CFG3_CS_WIN_SIZ_THR_OFS           16U                                          /*!< TOP GNRL CBT CFG3: CS_WIN_SIZ_THR Bit Offset */
#define TOP_GNRL_CBT_CFG3_CS_WIN_SIZ_THR(regval)       (BITS(16,23) & ((uint32_t)(regval) << 16))        /*!< TOP GNRL CBT CFG3: CS_WIN_SIZ_THR Bit Value */  
#define TOP_GNRL_CBT_CFG3_CA_WIN_SIZ_THR_MASK          BITS(24,31)                                   /*!< TOP GNRL CBT CFG3: CA_WIN_SIZ_THR Bit Mask */  
#define TOP_GNRL_CBT_CFG3_CA_WIN_SIZ_THR_OFS           24U                                          /*!< TOP GNRL CBT CFG3: CA_WIN_SIZ_THR Bit Offset */
#define TOP_GNRL_CBT_CFG3_CA_WIN_SIZ_THR(regval)       (BITS(24,31) & ((uint32_t)(regval) << 24))        /*!< TOP GNRL CBT CFG3: CA_WIN_SIZ_THR Bit Value */  
 
 /* ===== TOP GNRL CBT_CFG4 Register definition ===== */
#define TOP_GNRL_CBT_CFG4_CA_GRP_DISABLE               BIT(0)                                      /*!< One bit per CA group. For 1 CA group, LPDDR4 legal value is 1, LPDDR3 legal value is 1. For 2 CA group, LPDDR4 legal values are 1/3, LPDDR3 legal values is 1. For 4 CA group, LPDDR4 legal values are 1/3/7, LPDDR3 legal values are 1/3. */
#define TOP_GNRL_CBT_CFG4_CA_GRP_DISABLE_OFS           0U                                          /*!< TOP GNRL CBT CFG4: CA_GRP_DISABLE Bit Offset */
#define TOP_GNRL_CBT_CFG4_CA_GRP_DISABLE_VAL(regval)       (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL CBT CFG4: CA_GRP_DISABLE Bit Value */  
#define TOP_GNRL_CBT_CFG4_CS_GRP_DISABLE               BIT(4)                                      /*!< One bit per CS group. All ranks share one cs_grp_disable. For 1 CS group, LPDDR4 legal value is 1, LPDDR3 legal value is 1. For 2 CS group, LPDDR4 legal values are 1/3, LPDDR3 legal values is 1. For 4 CS group, LPDDR4 legal values are 1/3/7, LPDDR3 legal values are 1/3. */
#define TOP_GNRL_CBT_CFG4_CS_GRP_DISABLE_OFS           4U                                          /*!< TOP GNRL CBT CFG4: CS_GRP_DISABLE Bit Offset */
#define TOP_GNRL_CBT_CFG4_CS_GRP_DISABLE_VAL(regval)       (BIT(4) & ((uint32_t)(regval) << 4))        /*!< TOP GNRL CBT CFG4: CS_GRP_DISABLE Bit Value */  
 
 /* ===== TOP GNRL CBT_CFG5 Register definition ===== */
#define TOP_GNRL_CBT_CFG5_PAT0_MASK                    BITS(0,19)                                   /*!< TOP GNRL CBT CFG5: PAT0 Bit Mask */  
#define TOP_GNRL_CBT_CFG5_PAT0_OFS                     0U                                          /*!< TOP GNRL CBT CFG5: PAT0 Bit Offset */
#define TOP_GNRL_CBT_CFG5_PAT0(regval)                 (BITS(0,19) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL CBT CFG5: PAT0 Bit Value */  
 
 /* ===== TOP GNRL CBT_CFG6 Register definition ===== */
#define TOP_GNRL_CBT_CFG6_PAT1_MASK                    BITS(0,19)                                   /*!< TOP GNRL CBT CFG6: PAT1 Bit Mask */  
#define TOP_GNRL_CBT_CFG6_PAT1_OFS                     0U                                          /*!< TOP GNRL CBT CFG6: PAT1 Bit Offset */
#define TOP_GNRL_CBT_CFG6_PAT1(regval)                 (BITS(0,19) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL CBT CFG6: PAT1 Bit Value */  
 
 /* ===== TOP GNRL CBT_CFG7 Register definition ===== */
#define TOP_GNRL_CBT_CFG7_PAT2_MASK                    BITS(0,19)                                   /*!< TOP GNRL CBT CFG7: PAT2 Bit Mask */  
#define TOP_GNRL_CBT_CFG7_PAT2_OFS                     0U                                          /*!< TOP GNRL CBT CFG7: PAT2 Bit Offset */
#define TOP_GNRL_CBT_CFG7_PAT2(regval)                 (BITS(0,19) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL CBT CFG7: PAT2 Bit Value */  
 
 /* ===== TOP GNRL CBT_CFG8 Register definition ===== */
#define TOP_GNRL_CBT_CFG8_PAT3_MASK                    BITS(0,19)                                   /*!< TOP GNRL CBT CFG8: PAT3 Bit Mask */  
#define TOP_GNRL_CBT_CFG8_PAT3_OFS                     0U                                          /*!< TOP GNRL CBT CFG8: PAT3 Bit Offset */
#define TOP_GNRL_CBT_CFG8_PAT3(regval)                 (BITS(0,19) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL CBT CFG8: PAT3 Bit Value */  
 
 /* ===== TOP GNRL CBT_STATUS Register definition ===== */
#define TOP_GNRL_CBT_STATUS_R0_GRP0_CSLVL_ERR            BIT(0)                                      /*!< cslvl_err status for rank 0, ca group 0 */
#define TOP_GNRL_CBT_STATUS_R0_GRP0_CALVL_ERR            BIT(1)                                      /*!< calvl_err status for rank 0, ca group 0 */
#define TOP_GNRL_CBT_STATUS_R1_GRP0_CSLVL_ERR            BIT(2)                                      /*!< cslvl_err status for rank 1, ca group 0 */
#define TOP_GNRL_CBT_STATUS_R1_GRP0_CALVL_ERR            BIT(3)                                      /*!< calvl_err status for rank 1, ca group 0 */
 
 /* ===== TOP GNRL MISC_CFG Register definition ===== */
#define TOP_GNRL_MISC_CFG_CS_POLRTY_INV                BIT(0)                                      /*!< If the polarity of dfi_cs_p0/p1 matches the polarity of SDRAM chip select, set this bit to 0. If the polarity of dfi_cs_p0/p1 does not match the polarity of SDRAM chip select, set this bit to 1. */
#define TOP_GNRL_MISC_CFG_CS_POLRTY_INV_OFS            0U                                          /*!< TOP GNRL MISC CFG: CS_POLRTY_INV Bit Offset */
#define TOP_GNRL_MISC_CFG_CS_POLRTY_INV_VAL(regval)        (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL MISC CFG: CS_POLRTY_INV Bit Value */  
#define TOP_GNRL_MISC_CFG_DM_POLRTY_INV                BIT(1)                                      /*!< If the polarity of dfi_wrdata_mask_p0/p1 matches the polarity of SDRAM DM/DMI pin, set this bit to 0. If the polarity of dfi_wrdata_mask_p0/p1 does not match the polarity of SDRAM DM/DMI pin, set this bit to 1. */
#define TOP_GNRL_MISC_CFG_DM_POLRTY_INV_OFS            1U                                          /*!< TOP GNRL MISC CFG: DM_POLRTY_INV Bit Offset */
#define TOP_GNRL_MISC_CFG_DM_POLRTY_INV_VAL(regval)        (BIT(1) & ((uint32_t)(regval) << 1))        /*!< TOP GNRL MISC CFG: DM_POLRTY_INV Bit Value */  
#define TOP_GNRL_MISC_CFG_DATA_CS_POLRTY_INV           BIT(2)                                      /*!< If dfi_wrdata_cs_p0/p1 and dfi_rddata_cs_0/1 are active high, set this bit to 0. If dfi_wrdata_cs_p0/p1 and dfi_rddata_cs_0/1 are active low, set this bit to 1. */
#define TOP_GNRL_MISC_CFG_DATA_CS_POLRTY_INV_OFS       2U                                          /*!< TOP GNRL MISC CFG: DATA_CS_POLRTY_INV Bit Offset */
#define TOP_GNRL_MISC_CFG_DATA_CS_POLRTY_INV_VAL(regval)   (BIT(2) & ((uint32_t)(regval) << 2))        /*!< TOP GNRL MISC CFG: DATA_CS_POLRTY_INV Bit Value */  
#define TOP_GNRL_MISC_CFG_DS_LS_THR_MASK               BITS(4,7)                                   /*!< TOP GNRL MISC CFG: DS_LS_THR Bit Mask */  
#define TOP_GNRL_MISC_CFG_DS_LS_THR_OFS                4U                                          /*!< TOP GNRL MISC CFG: DS_LS_THR Bit Offset */
#define TOP_GNRL_MISC_CFG_DS_LS_THR(regval)            (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< TOP GNRL MISC CFG: DS_LS_THR Bit Value */  
 
 /* ===== TOP GNRL ACS4_MSTR_CFG Register definition ===== */
#define TOP_GNRL_ACS4_MSTR_CFG_AC0_MSTR_SEL                 BIT(0)                                      /*!< Select which DBYTE’s master delay line calculation value */
#define TOP_GNRL_ACS4_MSTR_CFG_AC0_MSTR_SEL_OFS             0U                                          /*!< TOP GNRL ACS4 MSTR CFG: AC0_MSTR_SEL Bit Offset */
#define TOP_GNRL_ACS4_MSTR_CFG_AC0_MSTR_SEL_VAL(regval)         (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL ACS4 MSTR CFG: AC0_MSTR_SEL Bit Value */  
#define TOP_GNRL_ACS4_MSTR_CFG_AC1_MSTR_SEL                 BIT(1)                                      /*!< Select which DBYTE’s master delay line calculation value */
#define TOP_GNRL_ACS4_MSTR_CFG_AC1_MSTR_SEL_OFS             1U                                          /*!< TOP GNRL ACS4 MSTR CFG: AC1_MSTR_SEL Bit Offset */
#define TOP_GNRL_ACS4_MSTR_CFG_AC1_MSTR_SEL_VAL(regval)         (BIT(1) & ((uint32_t)(regval) << 1))        /*!< TOP GNRL ACS4 MSTR CFG: AC1_MSTR_SEL Bit Value */  
#define TOP_GNRL_ACS4_MSTR_CFG_AC2_MSTR_SEL                 BIT(2)                                      /*!< Select which DBYTE’s master delay line calculation value */
#define TOP_GNRL_ACS4_MSTR_CFG_AC2_MSTR_SEL_OFS             2U                                          /*!< TOP GNRL ACS4 MSTR CFG: AC2_MSTR_SEL Bit Offset */
#define TOP_GNRL_ACS4_MSTR_CFG_AC2_MSTR_SEL_VAL(regval)         (BIT(2) & ((uint32_t)(regval) << 2))        /*!< TOP GNRL ACS4 MSTR CFG: AC2_MSTR_SEL Bit Value */  
#define TOP_GNRL_ACS4_MSTR_CFG_AC3_MSTR_SEL                 BIT(3)                                      /*!< Select which DBYTE’s master delay line calculation value */
#define TOP_GNRL_ACS4_MSTR_CFG_AC3_MSTR_SEL_OFS             3U                                          /*!< TOP GNRL ACS4 MSTR CFG: AC3_MSTR_SEL Bit Offset */
#define TOP_GNRL_ACS4_MSTR_CFG_AC3_MSTR_SEL_VAL(regval)         (BIT(3) & ((uint32_t)(regval) << 3))        /*!< TOP GNRL ACS4 MSTR CFG: AC3_MSTR_SEL Bit Value */  
#define TOP_GNRL_ACS4_MSTR_CFG_AC4_MSTR_SEL                 BIT(4)                                      /*!< Select which DBYTE’s master delay line calculation value */
#define TOP_GNRL_ACS4_MSTR_CFG_AC4_MSTR_SEL_OFS             4U                                          /*!< TOP GNRL ACS4 MSTR CFG: AC4_MSTR_SEL Bit Offset */
#define TOP_GNRL_ACS4_MSTR_CFG_AC4_MSTR_SEL_VAL(regval)         (BIT(4) & ((uint32_t)(regval) << 4))        /*!< TOP GNRL ACS4 MSTR CFG: AC4_MSTR_SEL Bit Value */  
#define TOP_GNRL_ACS4_MSTR_CFG_AC5_MSTR_SEL                 BIT(5)                                      /*!< Select which DBYTE’s master delay line calculation value */
#define TOP_GNRL_ACS4_MSTR_CFG_AC5_MSTR_SEL_OFS             5U                                          /*!< TOP GNRL ACS4 MSTR CFG: AC5_MSTR_SEL Bit Offset */
#define TOP_GNRL_ACS4_MSTR_CFG_AC5_MSTR_SEL_VAL(regval)         (BIT(5) & ((uint32_t)(regval) << 5))        /*!< TOP GNRL ACS4 MSTR CFG: AC5_MSTR_SEL Bit Value */  
#define TOP_GNRL_ACS4_MSTR_CFG_AC6_MSTR_SEL                 BIT(6)                                      /*!< Select which DBYTE’s master delay line calculation value */
#define TOP_GNRL_ACS4_MSTR_CFG_AC6_MSTR_SEL_OFS             6U                                          /*!< TOP GNRL ACS4 MSTR CFG: AC6_MSTR_SEL Bit Offset */
#define TOP_GNRL_ACS4_MSTR_CFG_AC6_MSTR_SEL_VAL(regval)         (BIT(6) & ((uint32_t)(regval) << 6))        /*!< TOP GNRL ACS4 MSTR CFG: AC6_MSTR_SEL Bit Value */  
#define TOP_GNRL_ACS4_MSTR_CFG_AC7_MSTR_SEL                 BIT(7)                                      /*!< Select which DBYTE’s master delay line calculation value */
#define TOP_GNRL_ACS4_MSTR_CFG_AC7_MSTR_SEL_OFS             7U                                          /*!< TOP GNRL ACS4 MSTR CFG: AC7_MSTR_SEL Bit Offset */
#define TOP_GNRL_ACS4_MSTR_CFG_AC7_MSTR_SEL_VAL(regval)         (BIT(7) & ((uint32_t)(regval) << 7))        /*!< TOP GNRL ACS4 MSTR CFG: AC7_MSTR_SEL Bit Value */  
 
 /* ===== TOP GNRL TOP_F0_TMG Register definition ===== */
#define TOP_GNRL_TOP_F0_TMG_T_CKSRE_MASK                 BITS(0,5)                                   /*!< TOP GNRL TOP F0 TMG: T_CKSRE Bit Mask */  
#define TOP_GNRL_TOP_F0_TMG_T_CKSRE_OFS                  0U                                          /*!< TOP GNRL TOP F0 TMG: T_CKSRE Bit Offset */
#define TOP_GNRL_TOP_F0_TMG_T_CKSRE(regval)              (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL TOP F0 TMG: T_CKSRE Bit Value */  
#define TOP_GNRL_TOP_F0_TMG_T_CBT_PAT_CHG_MASK           BITS(8,12)                                   /*!< TOP GNRL TOP F0 TMG: T_CBT_PAT_CHG Bit Mask */  
#define TOP_GNRL_TOP_F0_TMG_T_CBT_PAT_CHG_OFS            8U                                          /*!< TOP GNRL TOP F0 TMG: T_CBT_PAT_CHG Bit Offset */
#define TOP_GNRL_TOP_F0_TMG_T_CBT_PAT_CHG(regval)        (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP GNRL TOP F0 TMG: T_CBT_PAT_CHG Bit Value */  
#define TOP_GNRL_TOP_F0_TMG_CAL_CLK_DIV_SEL_MASK         BITS(16,23)                                   /*!< TOP GNRL TOP F0 TMG: CAL_CLK_DIV_SEL Bit Mask */  
#define TOP_GNRL_TOP_F0_TMG_CAL_CLK_DIV_SEL_OFS          16U                                          /*!< TOP GNRL TOP F0 TMG: CAL_CLK_DIV_SEL Bit Offset */
#define TOP_GNRL_TOP_F0_TMG_CAL_CLK_DIV_SEL(regval)      (BITS(16,23) & ((uint32_t)(regval) << 16))        /*!< TOP GNRL TOP F0 TMG: CAL_CLK_DIV_SEL Bit Value */  
#define TOP_GNRL_TOP_F0_TMG_CAL_INTRVL_MASK              BITS(24,28)                                   /*!< TOP GNRL TOP F0 TMG: CAL_INTRVL Bit Mask */  
#define TOP_GNRL_TOP_F0_TMG_CAL_INTRVL_OFS               24U                                          /*!< TOP GNRL TOP F0 TMG: CAL_INTRVL Bit Offset */
#define TOP_GNRL_TOP_F0_TMG_CAL_INTRVL(regval)           (BITS(24,28) & ((uint32_t)(regval) << 24))        /*!< TOP GNRL TOP F0 TMG: CAL_INTRVL Bit Value */  
#define TOP_GNRL_TOP_F0_TMG_PER_RANK_DIS                 BIT(31)                                      /*!< 1 - Always use the timing configuration of rank 0. 0 - Use dfi_wrdata_cs_p0/p1 and dfi_rddata_cs_p0/p1 to select the timing configuration.  */
#define TOP_GNRL_TOP_F0_TMG_PER_RANK_DIS_OFS             31U                                          /*!< TOP GNRL TOP F0 TMG: PER_RANK_DIS Bit Offset */
#define TOP_GNRL_TOP_F0_TMG_PER_RANK_DIS_VAL(regval)         (BIT(31) & ((uint32_t)(regval) << 31))        /*!< TOP GNRL TOP F0 TMG: PER_RANK_DIS Bit Value */  
 
 /* ===== TOP GNRL TOP_F1_TMG Register definition ===== */
#define TOP_GNRL_TOP_F1_TMG_T_CKSRE_MASK                 BITS(0,5)                                   /*!< TOP GNRL TOP F1 TMG: T_CKSRE Bit Mask */  
#define TOP_GNRL_TOP_F1_TMG_T_CKSRE_OFS                  0U                                          /*!< TOP GNRL TOP F1 TMG: T_CKSRE Bit Offset */
#define TOP_GNRL_TOP_F1_TMG_T_CKSRE(regval)              (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< TOP GNRL TOP F1 TMG: T_CKSRE Bit Value */  
#define TOP_GNRL_TOP_F1_TMG_T_CBT_PAT_CHG_MASK           BITS(8,12)                                   /*!< TOP GNRL TOP F1 TMG: T_CBT_PAT_CHG Bit Mask */  
#define TOP_GNRL_TOP_F1_TMG_T_CBT_PAT_CHG_OFS            8U                                          /*!< TOP GNRL TOP F1 TMG: T_CBT_PAT_CHG Bit Offset */
#define TOP_GNRL_TOP_F1_TMG_T_CBT_PAT_CHG(regval)        (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP GNRL TOP F1 TMG: T_CBT_PAT_CHG Bit Value */  
#define TOP_GNRL_TOP_F1_TMG_CAL_CLK_DIV_SEL_MASK         BITS(16,23)                                   /*!< TOP GNRL TOP F1 TMG: CAL_CLK_DIV_SEL Bit Mask */  
#define TOP_GNRL_TOP_F1_TMG_CAL_CLK_DIV_SEL_OFS          16U                                          /*!< TOP GNRL TOP F1 TMG: CAL_CLK_DIV_SEL Bit Offset */
#define TOP_GNRL_TOP_F1_TMG_CAL_CLK_DIV_SEL(regval)      (BITS(16,23) & ((uint32_t)(regval) << 16))        /*!< TOP GNRL TOP F1 TMG: CAL_CLK_DIV_SEL Bit Value */  
#define TOP_GNRL_TOP_F1_TMG_CAL_INTRVL_MASK              BITS(24,28)                                   /*!< TOP GNRL TOP F1 TMG: CAL_INTRVL Bit Mask */  
#define TOP_GNRL_TOP_F1_TMG_CAL_INTRVL_OFS               24U                                          /*!< TOP GNRL TOP F1 TMG: CAL_INTRVL Bit Offset */
#define TOP_GNRL_TOP_F1_TMG_CAL_INTRVL(regval)           (BITS(24,28) & ((uint32_t)(regval) << 24))        /*!< TOP GNRL TOP F1 TMG: CAL_INTRVL Bit Value */  
#define TOP_GNRL_TOP_F1_TMG_PER_RANK_DIS                 BIT(31)                                      /*!< 1 - Always use the timing configuration of rank 0. 0 - Use dfi_wrdata_cs_p0/p1 and dfi_rddata_cs_p0/p1 to select the timing configuration.  */
#define TOP_GNRL_TOP_F1_TMG_PER_RANK_DIS_OFS             31U                                          /*!< TOP GNRL TOP F1 TMG: PER_RANK_DIS Bit Offset */
#define TOP_GNRL_TOP_F1_TMG_PER_RANK_DIS_VAL(regval)         (BIT(31) & ((uint32_t)(regval) << 31))        /*!< TOP GNRL TOP F1 TMG: PER_RANK_DIS Bit Value */  
 
 /* ===== TOP OBS CFG Register definition ===== */
#define TOP_OBS_CFG_MISC_CTRL_OBS_SEL            BIT(0)                                      /*!< N/A */
#define TOP_OBS_CFG_MISC_CTRL_OBS_SEL_OFS        0U                                          /*!< TOP OBS CFG: MISC_CTRL_OBS_SEL Bit Offset */
#define TOP_OBS_CFG_MISC_CTRL_OBS_SEL_VAL(regval)    (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP OBS CFG: MISC_CTRL_OBS_SEL Bit Value */  
#define TOP_OBS_CFG_PAD_CAL_OBS_SEL_MASK         BITS(4,5)                                   /*!< TOP OBS CFG: PAD_CAL_OBS_SEL Bit Mask */  
#define TOP_OBS_CFG_PAD_CAL_OBS_SEL_OFS          4U                                          /*!< TOP OBS CFG: PAD_CAL_OBS_SEL Bit Offset */
#define TOP_OBS_CFG_PAD_CAL_OBS_SEL(regval)      (BITS(4,5) & ((uint32_t)(regval) << 4))        /*!< TOP OBS CFG: PAD_CAL_OBS_SEL Bit Value */  
#define TOP_OBS_CFG_CBT_CA_BIT_OBS_SEL_MASK      BITS(8,11)                                   /*!< TOP OBS CFG: CBT_CA_BIT_OBS_SEL Bit Mask */  
#define TOP_OBS_CFG_CBT_CA_BIT_OBS_SEL_OFS       8U                                          /*!< TOP OBS CFG: CBT_CA_BIT_OBS_SEL Bit Offset */
#define TOP_OBS_CFG_CBT_CA_BIT_OBS_SEL(regval)   (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< TOP OBS CFG: CBT_CA_BIT_OBS_SEL Bit Value */  
#define TOP_OBS_CFG_CBT_MISC_OBS_SEL_MASK        BITS(12,13)                                   /*!< TOP OBS CFG: CBT_MISC_OBS_SEL Bit Mask */  
#define TOP_OBS_CFG_CBT_MISC_OBS_SEL_OFS         12U                                          /*!< TOP OBS CFG: CBT_MISC_OBS_SEL Bit Offset */
#define TOP_OBS_CFG_CBT_MISC_OBS_SEL(regval)     (BITS(12,13) & ((uint32_t)(regval) << 12))        /*!< TOP OBS CFG: CBT_MISC_OBS_SEL Bit Value */  
 
 /* ===== TOP OBS MISC_CTRL_OBS Register definition ===== */
#define TOP_OBS_MISC_CTRL_OBS_OBS                          BITS(0,31)                
 
 /* ===== TOP OBS LP_OBS Register definition ===== */
#define TOP_OBS_LP_OBS_OBS                          BITS(0,31)                
 
 /* ===== TOP OBS PAD_CAL_OBS Register definition ===== */
#define TOP_OBS_PAD_CAL_OBS_OBS                          BITS(0,31)                
 
 /* ===== TOP OBS CBT_MISC_OBS Register definition ===== */
#define TOP_OBS_CBT_MISC_OBS_OBS                          BITS(0,31)                
 
 /* ===== TOP OBS CBT_CA_GRP0_LE_OBS Register definition ===== */
#define TOP_OBS_CBT_CA_GRP0_LE_OBS_LE_DLY                       BITS(0,10)                
#define TOP_OBS_CBT_CA_GRP0_LE_OBS_LE_FND                       BIT(11)                                      /*!< N/A */
#define TOP_OBS_CBT_CA_GRP0_LE_OBS_COM_LE_DLY                   BITS(16,26)                
 
 /* ===== TOP OBS CBT_CA_GRP0_TE_OBS Register definition ===== */
#define TOP_OBS_CBT_CA_GRP0_TE_OBS_TE_DLY                       BITS(0,10)                
#define TOP_OBS_CBT_CA_GRP0_TE_OBS_TE_FND                       BIT(11)                                      /*!< N/A */
#define TOP_OBS_CBT_CA_GRP0_TE_OBS_COM_TE_DLY                   BITS(16,26)                
 
 /* ===== TOP PAD SWAP D34_AC_SWAP_CFG0 Register definition ===== */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_DFI_ADDR0_MASK               BITS(0,4)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG0: DFI_ADDR0 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_DFI_ADDR0_OFS                0U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG0: DFI_ADDR0 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_DFI_ADDR0(regval)            (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP D34 AC SWAP CFG0: DFI_ADDR0 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_DFI_ADDR1_MASK               BITS(8,12)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG0: DFI_ADDR1 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_DFI_ADDR1_OFS                8U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG0: DFI_ADDR1 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_DFI_ADDR1(regval)            (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP D34 AC SWAP CFG0: DFI_ADDR1 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_DFI_ADDR2_MASK               BITS(16,20)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG0: DFI_ADDR2 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_DFI_ADDR2_OFS                16U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG0: DFI_ADDR2 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_DFI_ADDR2(regval)            (BITS(16,20) & ((uint32_t)(regval) << 16))        /*!< TOP PAD SWAP D34 AC SWAP CFG0: DFI_ADDR2 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_DFI_ADDR3_MASK               BITS(24,28)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG0: DFI_ADDR3 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_DFI_ADDR3_OFS                24U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG0: DFI_ADDR3 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG0_DFI_ADDR3(regval)            (BITS(24,28) & ((uint32_t)(regval) << 24))        /*!< TOP PAD SWAP D34 AC SWAP CFG0: DFI_ADDR3 Bit Value */  
 
 /* ===== TOP PAD SWAP D34_AC_SWAP_CFG1 Register definition ===== */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_DFI_ADDR4_MASK               BITS(0,4)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG1: DFI_ADDR4 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_DFI_ADDR4_OFS                0U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG1: DFI_ADDR4 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_DFI_ADDR4(regval)            (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP D34 AC SWAP CFG1: DFI_ADDR4 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_DFI_ADDR5_MASK               BITS(8,12)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG1: DFI_ADDR5 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_DFI_ADDR5_OFS                8U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG1: DFI_ADDR5 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_DFI_ADDR5(regval)            (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP D34 AC SWAP CFG1: DFI_ADDR5 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_DFI_ADDR6_MASK               BITS(16,20)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG1: DFI_ADDR6 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_DFI_ADDR6_OFS                16U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG1: DFI_ADDR6 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_DFI_ADDR6(regval)            (BITS(16,20) & ((uint32_t)(regval) << 16))        /*!< TOP PAD SWAP D34 AC SWAP CFG1: DFI_ADDR6 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_DFI_ADDR7_MASK               BITS(24,28)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG1: DFI_ADDR7 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_DFI_ADDR7_OFS                24U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG1: DFI_ADDR7 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG1_DFI_ADDR7(regval)            (BITS(24,28) & ((uint32_t)(regval) << 24))        /*!< TOP PAD SWAP D34 AC SWAP CFG1: DFI_ADDR7 Bit Value */  
 
 /* ===== TOP PAD SWAP D34_AC_SWAP_CFG2 Register definition ===== */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_DFI_ADDR8_MASK               BITS(0,4)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG2: DFI_ADDR8 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_DFI_ADDR8_OFS                0U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG2: DFI_ADDR8 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_DFI_ADDR8(regval)            (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP D34 AC SWAP CFG2: DFI_ADDR8 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_DFI_ADDR9_MASK               BITS(8,12)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG2: DFI_ADDR9 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_DFI_ADDR9_OFS                8U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG2: DFI_ADDR9 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_DFI_ADDR9(regval)            (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP D34 AC SWAP CFG2: DFI_ADDR9 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_DFI_ADDR10_MASK              BITS(16,20)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG2: DFI_ADDR10 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_DFI_ADDR10_OFS               16U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG2: DFI_ADDR10 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_DFI_ADDR10(regval)           (BITS(16,20) & ((uint32_t)(regval) << 16))        /*!< TOP PAD SWAP D34 AC SWAP CFG2: DFI_ADDR10 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_DFI_ADDR11_MASK              BITS(24,28)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG2: DFI_ADDR11 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_DFI_ADDR11_OFS               24U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG2: DFI_ADDR11 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG2_DFI_ADDR11(regval)           (BITS(24,28) & ((uint32_t)(regval) << 24))        /*!< TOP PAD SWAP D34 AC SWAP CFG2: DFI_ADDR11 Bit Value */  
 
 /* ===== TOP PAD SWAP D34_AC_SWAP_CFG3 Register definition ===== */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_DFI_ADDR12_MASK              BITS(0,4)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG3: DFI_ADDR12 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_DFI_ADDR12_OFS               0U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG3: DFI_ADDR12 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_DFI_ADDR12(regval)           (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP D34 AC SWAP CFG3: DFI_ADDR12 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_DFI_ADDR13_MASK              BITS(8,12)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG3: DFI_ADDR13 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_DFI_ADDR13_OFS               8U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG3: DFI_ADDR13 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_DFI_ADDR13(regval)           (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP D34 AC SWAP CFG3: DFI_ADDR13 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_DFI_ADDR14_BG1_MASK          BITS(16,20)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG3: DFI_ADDR14_BG1 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_DFI_ADDR14_BG1_OFS           16U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG3: DFI_ADDR14_BG1 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_DFI_ADDR14_BG1(regval)       (BITS(16,20) & ((uint32_t)(regval) << 16))        /*!< TOP PAD SWAP D34 AC SWAP CFG3: DFI_ADDR14_BG1 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_DFI_ADDR15_17_MASK           BITS(24,28)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG3: DFI_ADDR15_17 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_DFI_ADDR15_17_OFS            24U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG3: DFI_ADDR15_17 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG3_DFI_ADDR15_17(regval)        (BITS(24,28) & ((uint32_t)(regval) << 24))        /*!< TOP PAD SWAP D34 AC SWAP CFG3: DFI_ADDR15_17 Bit Value */  
 
 /* ===== TOP PAD SWAP D34_AC_SWAP_CFG4 Register definition ===== */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_DFI_BA0_MASK                 BITS(0,4)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG4: DFI_BA0 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_DFI_BA0_OFS                  0U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG4: DFI_BA0 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_DFI_BA0(regval)              (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP D34 AC SWAP CFG4: DFI_BA0 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_DFI_BA1_MASK                 BITS(8,12)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG4: DFI_BA1 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_DFI_BA1_OFS                  8U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG4: DFI_BA1 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_DFI_BA1(regval)              (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP D34 AC SWAP CFG4: DFI_BA1 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_DFI_BA2_BG0_MASK             BITS(16,20)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG4: DFI_BA2_BG0 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_DFI_BA2_BG0_OFS              16U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG4: DFI_BA2_BG0 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_DFI_BA2_BG0(regval)          (BITS(16,20) & ((uint32_t)(regval) << 16))        /*!< TOP PAD SWAP D34 AC SWAP CFG4: DFI_BA2_BG0 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_DFI_PAR_MASK                 BITS(24,28)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG4: DFI_PAR Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_DFI_PAR_OFS                  24U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG4: DFI_PAR Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG4_DFI_PAR(regval)              (BITS(24,28) & ((uint32_t)(regval) << 24))        /*!< TOP PAD SWAP D34 AC SWAP CFG4: DFI_PAR Bit Value */  
 
 /* ===== TOP PAD SWAP D34_AC_SWAP_CFG5 Register definition ===== */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_DFI_RAS_N_MASK               BITS(0,4)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG5: DFI_RAS_N Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_DFI_RAS_N_OFS                0U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG5: DFI_RAS_N Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_DFI_RAS_N(regval)            (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP D34 AC SWAP CFG5: DFI_RAS_N Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_DFI_CAS_N_MASK               BITS(8,12)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG5: DFI_CAS_N Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_DFI_CAS_N_OFS                8U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG5: DFI_CAS_N Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_DFI_CAS_N(regval)            (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP D34 AC SWAP CFG5: DFI_CAS_N Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_DFI_WE_N_MASK                BITS(16,20)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG5: DFI_WE_N Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_DFI_WE_N_OFS                 16U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG5: DFI_WE_N Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_DFI_WE_N(regval)             (BITS(16,20) & ((uint32_t)(regval) << 16))        /*!< TOP PAD SWAP D34 AC SWAP CFG5: DFI_WE_N Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_DFI_ACT_N_MASK               BITS(24,28)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG5: DFI_ACT_N Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_DFI_ACT_N_OFS                24U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG5: DFI_ACT_N Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG5_DFI_ACT_N(regval)            (BITS(24,28) & ((uint32_t)(regval) << 24))        /*!< TOP PAD SWAP D34 AC SWAP CFG5: DFI_ACT_N Bit Value */  
 
 /* ===== TOP PAD SWAP D34_AC_SWAP_CFG6 Register definition ===== */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_DFI_CS0_MASK                 BITS(0,4)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG6: DFI_CS0 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_DFI_CS0_OFS                  0U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG6: DFI_CS0 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_DFI_CS0(regval)              (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP D34 AC SWAP CFG6: DFI_CS0 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_DFI_CKE0_MASK                BITS(8,12)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG6: DFI_CKE0 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_DFI_CKE0_OFS                 8U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG6: DFI_CKE0 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_DFI_CKE0(regval)             (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP D34 AC SWAP CFG6: DFI_CKE0 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_DFI_ODT0_MASK                BITS(16,20)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG6: DFI_ODT0 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_DFI_ODT0_OFS                 16U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG6: DFI_ODT0 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_DFI_ODT0(regval)             (BITS(16,20) & ((uint32_t)(regval) << 16))        /*!< TOP PAD SWAP D34 AC SWAP CFG6: DFI_ODT0 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_DFI_RESET_N_MASK             BITS(24,28)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG6: DFI_RESET_N Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_DFI_RESET_N_OFS              24U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG6: DFI_RESET_N Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG6_DFI_RESET_N(regval)          (BITS(24,28) & ((uint32_t)(regval) << 24))        /*!< TOP PAD SWAP D34 AC SWAP CFG6: DFI_RESET_N Bit Value */  
 
 /* ===== TOP PAD SWAP D34_AC_SWAP_CFG7 Register definition ===== */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG7_DFI_CS1_MASK                 BITS(0,4)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG7: DFI_CS1 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG7_DFI_CS1_OFS                  0U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG7: DFI_CS1 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG7_DFI_CS1(regval)              (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP D34 AC SWAP CFG7: DFI_CS1 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG7_DFI_CKE1_MASK                BITS(8,12)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG7: DFI_CKE1 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG7_DFI_CKE1_OFS                 8U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG7: DFI_CKE1 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG7_DFI_CKE1(regval)             (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP D34 AC SWAP CFG7: DFI_CKE1 Bit Value */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG7_DFI_ODT1_MASK                BITS(16,20)                                   /*!< TOP PAD SWAP D34 AC SWAP CFG7: DFI_ODT1 Bit Mask */  
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG7_DFI_ODT1_OFS                 16U                                          /*!< TOP PAD SWAP D34 AC SWAP CFG7: DFI_ODT1 Bit Offset */
#define TOP_PAD_SWAP_D34_AC_SWAP_CFG7_DFI_ODT1(regval)             (BITS(16,20) & ((uint32_t)(regval) << 16))        /*!< TOP PAD SWAP D34 AC SWAP CFG7: DFI_ODT1 Bit Value */  
 
 /* ===== TOP PAD SWAP LP34_AC_SWAP_CFG0 Register definition ===== */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_R0_DFI_ADDR0_MASK            BITS(0,4)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG0: R0_DFI_ADDR0 Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_R0_DFI_ADDR0_OFS             0U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG0: R0_DFI_ADDR0 Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_R0_DFI_ADDR0(regval)         (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP LP34 AC SWAP CFG0: R0_DFI_ADDR0 Bit Value */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_R0_DFI_ADDR1_MASK            BITS(8,12)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG0: R0_DFI_ADDR1 Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_R0_DFI_ADDR1_OFS             8U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG0: R0_DFI_ADDR1 Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_R0_DFI_ADDR1(regval)         (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP LP34 AC SWAP CFG0: R0_DFI_ADDR1 Bit Value */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_R0_DFI_ADDR2_MASK            BITS(16,20)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG0: R0_DFI_ADDR2 Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_R0_DFI_ADDR2_OFS             16U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG0: R0_DFI_ADDR2 Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_R0_DFI_ADDR2(regval)         (BITS(16,20) & ((uint32_t)(regval) << 16))        /*!< TOP PAD SWAP LP34 AC SWAP CFG0: R0_DFI_ADDR2 Bit Value */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_R0_DFI_ADDR3_MASK            BITS(24,28)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG0: R0_DFI_ADDR3 Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_R0_DFI_ADDR3_OFS             24U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG0: R0_DFI_ADDR3 Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG0_R0_DFI_ADDR3(regval)         (BITS(24,28) & ((uint32_t)(regval) << 24))        /*!< TOP PAD SWAP LP34 AC SWAP CFG0: R0_DFI_ADDR3 Bit Value */  
 
 /* ===== TOP PAD SWAP LP34_AC_SWAP_CFG1 Register definition ===== */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG1_R0_DFI_ADDR4_MASK            BITS(0,4)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG1: R0_DFI_ADDR4 Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG1_R0_DFI_ADDR4_OFS             0U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG1: R0_DFI_ADDR4 Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG1_R0_DFI_ADDR4(regval)         (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP LP34 AC SWAP CFG1: R0_DFI_ADDR4 Bit Value */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG1_R0_DFI_ADDR5_MASK            BITS(8,12)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG1: R0_DFI_ADDR5 Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG1_R0_DFI_ADDR5_OFS             8U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG1: R0_DFI_ADDR5 Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG1_R0_DFI_ADDR5(regval)         (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP LP34 AC SWAP CFG1: R0_DFI_ADDR5 Bit Value */  
 
 /* ===== TOP PAD SWAP LP34_AC_SWAP_CFG2 Register definition ===== */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_R1_DFI_ADDR0_MASK            BITS(0,4)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG2: R1_DFI_ADDR0 Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_R1_DFI_ADDR0_OFS             0U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG2: R1_DFI_ADDR0 Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_R1_DFI_ADDR0(regval)         (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP LP34 AC SWAP CFG2: R1_DFI_ADDR0 Bit Value */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_R1_DFI_ADDR1_MASK            BITS(8,12)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG2: R1_DFI_ADDR1 Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_R1_DFI_ADDR1_OFS             8U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG2: R1_DFI_ADDR1 Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_R1_DFI_ADDR1(regval)         (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP LP34 AC SWAP CFG2: R1_DFI_ADDR1 Bit Value */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_R1_DFI_ADDR2_MASK            BITS(16,20)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG2: R1_DFI_ADDR2 Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_R1_DFI_ADDR2_OFS             16U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG2: R1_DFI_ADDR2 Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_R1_DFI_ADDR2(regval)         (BITS(16,20) & ((uint32_t)(regval) << 16))        /*!< TOP PAD SWAP LP34 AC SWAP CFG2: R1_DFI_ADDR2 Bit Value */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_R1_DFI_ADDR3_MASK            BITS(24,28)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG2: R1_DFI_ADDR3 Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_R1_DFI_ADDR3_OFS             24U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG2: R1_DFI_ADDR3 Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG2_R1_DFI_ADDR3(regval)         (BITS(24,28) & ((uint32_t)(regval) << 24))        /*!< TOP PAD SWAP LP34 AC SWAP CFG2: R1_DFI_ADDR3 Bit Value */  
 
 /* ===== TOP PAD SWAP LP34_AC_SWAP_CFG3 Register definition ===== */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG3_R1_DFI_ADDR4_MASK            BITS(0,4)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG3: R1_DFI_ADDR4 Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG3_R1_DFI_ADDR4_OFS             0U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG3: R1_DFI_ADDR4 Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG3_R1_DFI_ADDR4(regval)         (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP LP34 AC SWAP CFG3: R1_DFI_ADDR4 Bit Value */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG3_R1_DFI_ADDR5_MASK            BITS(8,12)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG3: R1_DFI_ADDR5 Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG3_R1_DFI_ADDR5_OFS             8U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG3: R1_DFI_ADDR5 Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG3_R1_DFI_ADDR5(regval)         (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP LP34 AC SWAP CFG3: R1_DFI_ADDR5 Bit Value */  
 
 /* ===== TOP PAD SWAP LP34_AC_SWAP_CFG4 Register definition ===== */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG4_R0_DFI_CS_MASK               BITS(0,4)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG4: R0_DFI_CS Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG4_R0_DFI_CS_OFS                0U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG4: R0_DFI_CS Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG4_R0_DFI_CS(regval)            (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP LP34 AC SWAP CFG4: R0_DFI_CS Bit Value */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG4_R0_DFI_CKE_MASK              BITS(8,12)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG4: R0_DFI_CKE Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG4_R0_DFI_CKE_OFS               8U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG4: R0_DFI_CKE Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG4_R0_DFI_CKE(regval)           (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP LP34 AC SWAP CFG4: R0_DFI_CKE Bit Value */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG4_DFI_RESET_N_MASK             BITS(24,28)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG4: DFI_RESET_N Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG4_DFI_RESET_N_OFS              24U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG4: DFI_RESET_N Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG4_DFI_RESET_N(regval)          (BITS(24,28) & ((uint32_t)(regval) << 24))        /*!< TOP PAD SWAP LP34 AC SWAP CFG4: DFI_RESET_N Bit Value */  
 
 /* ===== TOP PAD SWAP LP34_AC_SWAP_CFG5 Register definition ===== */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG5_R1_DFI_CS_MASK               BITS(0,4)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG5: R1_DFI_CS Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG5_R1_DFI_CS_OFS                0U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG5: R1_DFI_CS Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG5_R1_DFI_CS(regval)            (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP LP34 AC SWAP CFG5: R1_DFI_CS Bit Value */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG5_R1_DFI_CKE_MASK              BITS(8,12)                                   /*!< TOP PAD SWAP LP34 AC SWAP CFG5: R1_DFI_CKE Bit Mask */  
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG5_R1_DFI_CKE_OFS               8U                                          /*!< TOP PAD SWAP LP34 AC SWAP CFG5: R1_DFI_CKE Bit Offset */
#define TOP_PAD_SWAP_LP34_AC_SWAP_CFG5_R1_DFI_CKE(regval)           (BITS(8,12) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP LP34 AC SWAP CFG5: R1_DFI_CKE Bit Value */  
 
 /* ===== TOP PAD SWAP DB0_SWAP_CFG0 Register definition ===== */
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ0_MASK                     BITS(0,3)                                   /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ0 Bit Mask */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ0_OFS                      0U                                          /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ0 Bit Offset */
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ0(regval)                  (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ0 Bit Value */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ1_MASK                     BITS(4,7)                                   /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ1 Bit Mask */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ1_OFS                      4U                                          /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ1 Bit Offset */
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ1(regval)                  (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ1 Bit Value */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ2_MASK                     BITS(8,11)                                   /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ2 Bit Mask */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ2_OFS                      8U                                          /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ2 Bit Offset */
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ2(regval)                  (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ2 Bit Value */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ3_MASK                     BITS(12,15)                                   /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ3 Bit Mask */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ3_OFS                      12U                                          /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ3 Bit Offset */
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ3(regval)                  (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ3 Bit Value */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ4_MASK                     BITS(16,19)                                   /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ4 Bit Mask */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ4_OFS                      16U                                          /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ4 Bit Offset */
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ4(regval)                  (BITS(16,19) & ((uint32_t)(regval) << 16))        /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ4 Bit Value */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ5_MASK                     BITS(20,23)                                   /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ5 Bit Mask */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ5_OFS                      20U                                          /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ5 Bit Offset */
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ5(regval)                  (BITS(20,23) & ((uint32_t)(regval) << 20))        /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ5 Bit Value */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ6_MASK                     BITS(24,27)                                   /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ6 Bit Mask */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ6_OFS                      24U                                          /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ6 Bit Offset */
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ6(regval)                  (BITS(24,27) & ((uint32_t)(regval) << 24))        /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ6 Bit Value */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ7_MASK                     BITS(28,31)                                   /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ7 Bit Mask */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ7_OFS                      28U                                          /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ7 Bit Offset */
#define TOP_PAD_SWAP_DB0_SWAP_CFG0_DQ7(regval)                  (BITS(28,31) & ((uint32_t)(regval) << 28))        /*!< TOP PAD SWAP DB0 SWAP CFG0: DQ7 Bit Value */  
 
 /* ===== TOP PAD SWAP DB0_SWAP_CFG1 Register definition ===== */
#define TOP_PAD_SWAP_DB0_SWAP_CFG1_DM_DBI_MASK                  BITS(0,3)                                   /*!< TOP PAD SWAP DB0 SWAP CFG1: DM_DBI Bit Mask */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG1_DM_DBI_OFS                   0U                                          /*!< TOP PAD SWAP DB0 SWAP CFG1: DM_DBI Bit Offset */
#define TOP_PAD_SWAP_DB0_SWAP_CFG1_DM_DBI(regval)               (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP DB0 SWAP CFG1: DM_DBI Bit Value */  
#define TOP_PAD_SWAP_DB0_SWAP_CFG1_DBYTE                        BIT(4)                                      /*!< Indicates the target DBYTE that the given DFI data bus are mapped to. Legal range: 0~1. The DFI data bus includes below signals: dfi_wrdata_p0/p1[0+:8], dfi_wrdata_p0/p1[16+:8], dfi_wrdata_mask_p0/p1, dfi_rddata_w0/w1[0+:8], dfi_rddata_w0/w1[16+:8], dfi_rddata_dbi_w0/w1 Note: All these DB*_SWAP_CFG1.dbyte must be unique value */
#define TOP_PAD_SWAP_DB0_SWAP_CFG1_DBYTE_OFS                    4U                                          /*!< TOP PAD SWAP DB0 SWAP CFG1: DBYTE Bit Offset */
#define TOP_PAD_SWAP_DB0_SWAP_CFG1_DBYTE_VAL(regval)                (BIT(4) & ((uint32_t)(regval) << 4))        /*!< TOP PAD SWAP DB0 SWAP CFG1: DBYTE Bit Value */  
 
 /* ===== TOP PAD SWAP DB1_SWAP_CFG0 Register definition ===== */
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ0_MASK                     BITS(0,3)                                   /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ0 Bit Mask */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ0_OFS                      0U                                          /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ0 Bit Offset */
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ0(regval)                  (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ0 Bit Value */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ1_MASK                     BITS(4,7)                                   /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ1 Bit Mask */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ1_OFS                      4U                                          /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ1 Bit Offset */
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ1(regval)                  (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ1 Bit Value */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ2_MASK                     BITS(8,11)                                   /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ2 Bit Mask */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ2_OFS                      8U                                          /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ2 Bit Offset */
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ2(regval)                  (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ2 Bit Value */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ3_MASK                     BITS(12,15)                                   /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ3 Bit Mask */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ3_OFS                      12U                                          /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ3 Bit Offset */
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ3(regval)                  (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ3 Bit Value */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ4_MASK                     BITS(16,19)                                   /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ4 Bit Mask */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ4_OFS                      16U                                          /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ4 Bit Offset */
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ4(regval)                  (BITS(16,19) & ((uint32_t)(regval) << 16))        /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ4 Bit Value */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ5_MASK                     BITS(20,23)                                   /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ5 Bit Mask */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ5_OFS                      20U                                          /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ5 Bit Offset */
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ5(regval)                  (BITS(20,23) & ((uint32_t)(regval) << 20))        /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ5 Bit Value */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ6_MASK                     BITS(24,27)                                   /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ6 Bit Mask */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ6_OFS                      24U                                          /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ6 Bit Offset */
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ6(regval)                  (BITS(24,27) & ((uint32_t)(regval) << 24))        /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ6 Bit Value */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ7_MASK                     BITS(28,31)                                   /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ7 Bit Mask */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ7_OFS                      28U                                          /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ7 Bit Offset */
#define TOP_PAD_SWAP_DB1_SWAP_CFG0_DQ7(regval)                  (BITS(28,31) & ((uint32_t)(regval) << 28))        /*!< TOP PAD SWAP DB1 SWAP CFG0: DQ7 Bit Value */  
 
 /* ===== TOP PAD SWAP DB1_SWAP_CFG1 Register definition ===== */
#define TOP_PAD_SWAP_DB1_SWAP_CFG1_DM_DBI_MASK                  BITS(0,3)                                   /*!< TOP PAD SWAP DB1 SWAP CFG1: DM_DBI Bit Mask */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG1_DM_DBI_OFS                   0U                                          /*!< TOP PAD SWAP DB1 SWAP CFG1: DM_DBI Bit Offset */
#define TOP_PAD_SWAP_DB1_SWAP_CFG1_DM_DBI(regval)               (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< TOP PAD SWAP DB1 SWAP CFG1: DM_DBI Bit Value */  
#define TOP_PAD_SWAP_DB1_SWAP_CFG1_DBYTE                        BIT(4)                                      /*!< Indicates the target DBYTE that the given DFI data bus are mapped to. Legal range: 0~1. The DFI data bus includes below signals: dfi_wrdata_p0/p1[8+:8], dfi_wrdata_p0/p1[24+:8], dfi_wrdata_mask_p0/p1, dfi_rddata_w0/w1[8+:8], dfi_rddata_w0/w1[24+:8], dfi_rddata_dbi_w0/w1 Note: All these DB*_SWAP_CFG1.dbyte must be unique value */
#define TOP_PAD_SWAP_DB1_SWAP_CFG1_DBYTE_OFS                    4U                                          /*!< TOP PAD SWAP DB1 SWAP CFG1: DBYTE Bit Offset */
#define TOP_PAD_SWAP_DB1_SWAP_CFG1_DBYTE_VAL(regval)                (BIT(4) & ((uint32_t)(regval) << 4))        /*!< TOP PAD SWAP DB1 SWAP CFG1: DBYTE Bit Value */  
 
 /* ===== TOP PAD FUNC SLEW_CFG Register definition ===== */
#define TOP_PAD_FUNC_SLEW_CFG_AC_PAD_SLEW_P_MASK           BITS(0,2)                                   /*!< TOP PAD FUNC SLEW CFG: AC_PAD_SLEW_P Bit Mask */  
#define TOP_PAD_FUNC_SLEW_CFG_AC_PAD_SLEW_P_OFS            0U                                          /*!< TOP PAD FUNC SLEW CFG: AC_PAD_SLEW_P Bit Offset */
#define TOP_PAD_FUNC_SLEW_CFG_AC_PAD_SLEW_P(regval)        (BITS(0,2) & ((uint32_t)(regval) << 0))        /*!< TOP PAD FUNC SLEW CFG: AC_PAD_SLEW_P Bit Value */  
#define TOP_PAD_FUNC_SLEW_CFG_AC_PAD_SLEW_N_MASK           BITS(4,6)                                   /*!< TOP PAD FUNC SLEW CFG: AC_PAD_SLEW_N Bit Mask */  
#define TOP_PAD_FUNC_SLEW_CFG_AC_PAD_SLEW_N_OFS            4U                                          /*!< TOP PAD FUNC SLEW CFG: AC_PAD_SLEW_N Bit Offset */
#define TOP_PAD_FUNC_SLEW_CFG_AC_PAD_SLEW_N(regval)        (BITS(4,6) & ((uint32_t)(regval) << 4))        /*!< TOP PAD FUNC SLEW CFG: AC_PAD_SLEW_N Bit Value */  
#define TOP_PAD_FUNC_SLEW_CFG_DB_PAD_SLEW_P_MASK           BITS(8,10)                                   /*!< TOP PAD FUNC SLEW CFG: DB_PAD_SLEW_P Bit Mask */  
#define TOP_PAD_FUNC_SLEW_CFG_DB_PAD_SLEW_P_OFS            8U                                          /*!< TOP PAD FUNC SLEW CFG: DB_PAD_SLEW_P Bit Offset */
#define TOP_PAD_FUNC_SLEW_CFG_DB_PAD_SLEW_P(regval)        (BITS(8,10) & ((uint32_t)(regval) << 8))        /*!< TOP PAD FUNC SLEW CFG: DB_PAD_SLEW_P Bit Value */  
#define TOP_PAD_FUNC_SLEW_CFG_DB_PAD_SLEW_N_MASK           BITS(12,14)                                   /*!< TOP PAD FUNC SLEW CFG: DB_PAD_SLEW_N Bit Mask */  
#define TOP_PAD_FUNC_SLEW_CFG_DB_PAD_SLEW_N_OFS            12U                                          /*!< TOP PAD FUNC SLEW CFG: DB_PAD_SLEW_N Bit Offset */
#define TOP_PAD_FUNC_SLEW_CFG_DB_PAD_SLEW_N(regval)        (BITS(12,14) & ((uint32_t)(regval) << 12))        /*!< TOP PAD FUNC SLEW CFG: DB_PAD_SLEW_N Bit Value */  
#define TOP_PAD_FUNC_SLEW_CFG_CLK_PAD_SLEW_P_MASK          BITS(16,18)                                   /*!< TOP PAD FUNC SLEW CFG: CLK_PAD_SLEW_P Bit Mask */  
#define TOP_PAD_FUNC_SLEW_CFG_CLK_PAD_SLEW_P_OFS           16U                                          /*!< TOP PAD FUNC SLEW CFG: CLK_PAD_SLEW_P Bit Offset */
#define TOP_PAD_FUNC_SLEW_CFG_CLK_PAD_SLEW_P(regval)       (BITS(16,18) & ((uint32_t)(regval) << 16))        /*!< TOP PAD FUNC SLEW CFG: CLK_PAD_SLEW_P Bit Value */  
#define TOP_PAD_FUNC_SLEW_CFG_CLK_PAD_SLEW_N_MASK          BITS(20,22)                                   /*!< TOP PAD FUNC SLEW CFG: CLK_PAD_SLEW_N Bit Mask */  
#define TOP_PAD_FUNC_SLEW_CFG_CLK_PAD_SLEW_N_OFS           20U                                          /*!< TOP PAD FUNC SLEW CFG: CLK_PAD_SLEW_N Bit Offset */
#define TOP_PAD_FUNC_SLEW_CFG_CLK_PAD_SLEW_N(regval)       (BITS(20,22) & ((uint32_t)(regval) << 20))        /*!< TOP PAD FUNC SLEW CFG: CLK_PAD_SLEW_N Bit Value */  
 
 /* ===== TOP PAD FUNC RX_CAL_CFG Register definition ===== */
#define TOP_PAD_FUNC_RX_CAL_CFG_AC_CODE_UP_MASK              BITS(0,5)                                   /*!< TOP PAD FUNC RX CAL CFG: AC_CODE_UP Bit Mask */  
#define TOP_PAD_FUNC_RX_CAL_CFG_AC_CODE_UP_OFS               0U                                          /*!< TOP PAD FUNC RX CAL CFG: AC_CODE_UP Bit Offset */
#define TOP_PAD_FUNC_RX_CAL_CFG_AC_CODE_UP(regval)           (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< TOP PAD FUNC RX CAL CFG: AC_CODE_UP Bit Value */  
#define TOP_PAD_FUNC_RX_CAL_CFG_AC_CODE_DOWN_MASK            BITS(8,13)                                   /*!< TOP PAD FUNC RX CAL CFG: AC_CODE_DOWN Bit Mask */  
#define TOP_PAD_FUNC_RX_CAL_CFG_AC_CODE_DOWN_OFS             8U                                          /*!< TOP PAD FUNC RX CAL CFG: AC_CODE_DOWN Bit Offset */
#define TOP_PAD_FUNC_RX_CAL_CFG_AC_CODE_DOWN(regval)         (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< TOP PAD FUNC RX CAL CFG: AC_CODE_DOWN Bit Value */  
#define TOP_PAD_FUNC_RX_CAL_CFG_CLK_CODE_UP_MASK             BITS(16,21)                                   /*!< TOP PAD FUNC RX CAL CFG: CLK_CODE_UP Bit Mask */  
#define TOP_PAD_FUNC_RX_CAL_CFG_CLK_CODE_UP_OFS              16U                                          /*!< TOP PAD FUNC RX CAL CFG: CLK_CODE_UP Bit Offset */
#define TOP_PAD_FUNC_RX_CAL_CFG_CLK_CODE_UP(regval)          (BITS(16,21) & ((uint32_t)(regval) << 16))        /*!< TOP PAD FUNC RX CAL CFG: CLK_CODE_UP Bit Value */  
#define TOP_PAD_FUNC_RX_CAL_CFG_CLK_CODE_DOWN_MASK           BITS(24,29)                                   /*!< TOP PAD FUNC RX CAL CFG: CLK_CODE_DOWN Bit Mask */  
#define TOP_PAD_FUNC_RX_CAL_CFG_CLK_CODE_DOWN_OFS            24U                                          /*!< TOP PAD FUNC RX CAL CFG: CLK_CODE_DOWN Bit Offset */
#define TOP_PAD_FUNC_RX_CAL_CFG_CLK_CODE_DOWN(regval)        (BITS(24,29) & ((uint32_t)(regval) << 24))        /*!< TOP PAD FUNC RX CAL CFG: CLK_CODE_DOWN Bit Value */  
 
 /* ===== TOP PAD FUNC TSEL_CFG Register definition ===== */
#define TOP_PAD_FUNC_TSEL_CFG_CLK_TSEL                     BIT(0)                                      /*!< TSEL for the CLK PADs. */
#define TOP_PAD_FUNC_TSEL_CFG_CLK_TSEL_OFS                 0U                                          /*!< TOP PAD FUNC TSEL CFG: CLK_TSEL Bit Offset */
#define TOP_PAD_FUNC_TSEL_CFG_CLK_TSEL_VAL(regval)             (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP PAD FUNC TSEL CFG: CLK_TSEL Bit Value */  
#define TOP_PAD_FUNC_TSEL_CFG_CLK_TSEL_P_MASK              BITS(4,7)                                   /*!< TOP PAD FUNC TSEL CFG: CLK_TSEL_P Bit Mask */  
#define TOP_PAD_FUNC_TSEL_CFG_CLK_TSEL_P_OFS               4U                                          /*!< TOP PAD FUNC TSEL CFG: CLK_TSEL_P Bit Offset */
#define TOP_PAD_FUNC_TSEL_CFG_CLK_TSEL_P(regval)           (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< TOP PAD FUNC TSEL CFG: CLK_TSEL_P Bit Value */  
#define TOP_PAD_FUNC_TSEL_CFG_CLK_TSEL_N_MASK              BITS(8,11)                                   /*!< TOP PAD FUNC TSEL CFG: CLK_TSEL_N Bit Mask */  
#define TOP_PAD_FUNC_TSEL_CFG_CLK_TSEL_N_OFS               8U                                          /*!< TOP PAD FUNC TSEL CFG: CLK_TSEL_N Bit Offset */
#define TOP_PAD_FUNC_TSEL_CFG_CLK_TSEL_N(regval)           (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< TOP PAD FUNC TSEL CFG: CLK_TSEL_N Bit Value */  
 
 /* ===== TOP PAD FUNC MISC_PAD_CFG Register definition ===== */
#define TOP_PAD_FUNC_MISC_PAD_CFG_PAD_DDR3_EN                  BIT(0)                                      /*!< Set this field to 1 if PHY works in DDR3 or LPDDR3 mode. */
#define TOP_PAD_FUNC_MISC_PAD_CFG_PAD_DDR3_EN_OFS              0U                                          /*!< TOP PAD FUNC MISC PAD CFG: PAD_DDR3_EN Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_PAD_DDR3_EN_VAL(regval)          (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP PAD FUNC MISC PAD CFG: PAD_DDR3_EN Bit Value */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_PAD_DDR4_EN                  BIT(1)                                      /*!< Set this field to 1 if PHY works in DDR4 mode. */
#define TOP_PAD_FUNC_MISC_PAD_CFG_PAD_DDR4_EN_OFS              1U                                          /*!< TOP PAD FUNC MISC PAD CFG: PAD_DDR4_EN Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_PAD_DDR4_EN_VAL(regval)          (BIT(1) & ((uint32_t)(regval) << 1))        /*!< TOP PAD FUNC MISC PAD CFG: PAD_DDR4_EN Bit Value */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_PAD_LPDDR4_EN                BIT(2)                                      /*!< Set this field to 1 if PHY works in LPDDR4 mode. */
#define TOP_PAD_FUNC_MISC_PAD_CFG_PAD_LPDDR4_EN_OFS            2U                                          /*!< TOP PAD FUNC MISC PAD CFG: PAD_LPDDR4_EN Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_PAD_LPDDR4_EN_VAL(regval)        (BIT(2) & ((uint32_t)(regval) << 2))        /*!< TOP PAD FUNC MISC PAD CFG: PAD_LPDDR4_EN Bit Value */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_AC_VREF_GEN_EN               BIT(7)                                      /*!< EN_LV for all the vref_gen for all the ACS4 PADs and CLK PADs. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_MISC_PAD_CFG_AC_VREF_GEN_EN_OFS           7U                                          /*!< TOP PAD FUNC MISC PAD CFG: AC_VREF_GEN_EN Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_AC_VREF_GEN_EN_VAL(regval)       (BIT(7) & ((uint32_t)(regval) << 7))        /*!< TOP PAD FUNC MISC PAD CFG: AC_VREF_GEN_EN Bit Value */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_DB_VREF_GEN_EN               BIT(8)                                      /*!< EN_LV for all the vref_gen for all the DBYTE PADs. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_MISC_PAD_CFG_DB_VREF_GEN_EN_OFS           8U                                          /*!< TOP PAD FUNC MISC PAD CFG: DB_VREF_GEN_EN Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_DB_VREF_GEN_EN_VAL(regval)       (BIT(8) & ((uint32_t)(regval) << 8))        /*!< TOP PAD FUNC MISC PAD CFG: DB_VREF_GEN_EN Bit Value */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_ATB_EN                       BIT(9)                                      /*!< EN for the DDR_ANA_TEST PAD. */
#define TOP_PAD_FUNC_MISC_PAD_CFG_ATB_EN_OFS                   9U                                          /*!< TOP PAD FUNC MISC PAD CFG: ATB_EN Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_ATB_EN_VAL(regval)               (BIT(9) & ((uint32_t)(regval) << 9))        /*!< TOP PAD FUNC MISC PAD CFG: ATB_EN Bit Value */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_AC_VREFSEL_MASK              BITS(12,19)                                   /*!< TOP PAD FUNC MISC PAD CFG: AC_VREFSEL Bit Mask */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_AC_VREFSEL_OFS               12U                                          /*!< TOP PAD FUNC MISC PAD CFG: AC_VREFSEL Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_AC_VREFSEL(regval)           (BITS(12,19) & ((uint32_t)(regval) << 12))        /*!< TOP PAD FUNC MISC PAD CFG: AC_VREFSEL Bit Value */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_F0_DQS_GTLVL_BIAS_FS_EN         BIT(20)                                      /*!< Reserved */
#define TOP_PAD_FUNC_MISC_PAD_CFG_F0_DQS_GTLVL_BIAS_FS_EN_OFS     20U                                          /*!< TOP PAD FUNC MISC PAD CFG: F0_DQS_GTLVL_BIAS_FS_EN Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_F0_DQS_GTLVL_BIAS_FS_EN_VAL(regval) (BIT(20) & ((uint32_t)(regval) << 20))        /*!< TOP PAD FUNC MISC PAD CFG: F0_DQS_GTLVL_BIAS_FS_EN Bit Value */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_F0_DQS_ALL_BIAS_FS_EN         BIT(21)                                      /*!< Reserved */
#define TOP_PAD_FUNC_MISC_PAD_CFG_F0_DQS_ALL_BIAS_FS_EN_OFS     21U                                          /*!< TOP PAD FUNC MISC PAD CFG: F0_DQS_ALL_BIAS_FS_EN Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_F0_DQS_ALL_BIAS_FS_EN_VAL(regval) (BIT(21) & ((uint32_t)(regval) << 21))        /*!< TOP PAD FUNC MISC PAD CFG: F0_DQS_ALL_BIAS_FS_EN Bit Value */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_F0_DQ_ALL_BIAS_FS_EN         BIT(22)                                      /*!< Reserved */
#define TOP_PAD_FUNC_MISC_PAD_CFG_F0_DQ_ALL_BIAS_FS_EN_OFS     22U                                          /*!< TOP PAD FUNC MISC PAD CFG: F0_DQ_ALL_BIAS_FS_EN Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_F0_DQ_ALL_BIAS_FS_EN_VAL(regval) (BIT(22) & ((uint32_t)(regval) << 22))        /*!< TOP PAD FUNC MISC PAD CFG: F0_DQ_ALL_BIAS_FS_EN Bit Value */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_F1_DQS_GTLVL_BIAS_FS_EN         BIT(23)                                      /*!< Reserved */
#define TOP_PAD_FUNC_MISC_PAD_CFG_F1_DQS_GTLVL_BIAS_FS_EN_OFS     23U                                          /*!< TOP PAD FUNC MISC PAD CFG: F1_DQS_GTLVL_BIAS_FS_EN Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_F1_DQS_GTLVL_BIAS_FS_EN_VAL(regval) (BIT(23) & ((uint32_t)(regval) << 23))        /*!< TOP PAD FUNC MISC PAD CFG: F1_DQS_GTLVL_BIAS_FS_EN Bit Value */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_F1_DQS_ALL_BIAS_FS_EN         BIT(24)                                      /*!< Reserved */
#define TOP_PAD_FUNC_MISC_PAD_CFG_F1_DQS_ALL_BIAS_FS_EN_OFS     24U                                          /*!< TOP PAD FUNC MISC PAD CFG: F1_DQS_ALL_BIAS_FS_EN Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_F1_DQS_ALL_BIAS_FS_EN_VAL(regval) (BIT(24) & ((uint32_t)(regval) << 24))        /*!< TOP PAD FUNC MISC PAD CFG: F1_DQS_ALL_BIAS_FS_EN Bit Value */  
#define TOP_PAD_FUNC_MISC_PAD_CFG_F1_DQ_ALL_BIAS_FS_EN         BIT(25)                                      /*!< Reserved */
#define TOP_PAD_FUNC_MISC_PAD_CFG_F1_DQ_ALL_BIAS_FS_EN_OFS     25U                                          /*!< TOP PAD FUNC MISC PAD CFG: F1_DQ_ALL_BIAS_FS_EN Bit Offset */
#define TOP_PAD_FUNC_MISC_PAD_CFG_F1_DQ_ALL_BIAS_FS_EN_VAL(regval) (BIT(25) & ((uint32_t)(regval) << 25))        /*!< TOP PAD FUNC MISC PAD CFG: F1_DQ_ALL_BIAS_FS_EN Bit Value */  
 
 /* ===== TOP PAD FUNC AC_PD_CFG0 Register definition ===== */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA0_PD                       BIT(0)                                      /*!< PD for the ca0_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA0_PD_OFS                   0U                                          /*!< TOP PAD FUNC AC PD CFG0: CA0_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA0_PD_VAL(regval)               (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP PAD FUNC AC PD CFG0: CA0_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA1_PD                       BIT(1)                                      /*!< PD for the ca1_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA1_PD_OFS                   1U                                          /*!< TOP PAD FUNC AC PD CFG0: CA1_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA1_PD_VAL(regval)               (BIT(1) & ((uint32_t)(regval) << 1))        /*!< TOP PAD FUNC AC PD CFG0: CA1_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA2_PD                       BIT(2)                                      /*!< PD for the ca2_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA2_PD_OFS                   2U                                          /*!< TOP PAD FUNC AC PD CFG0: CA2_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA2_PD_VAL(regval)               (BIT(2) & ((uint32_t)(regval) << 2))        /*!< TOP PAD FUNC AC PD CFG0: CA2_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA3_PD                       BIT(3)                                      /*!< PD for the ca3_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA3_PD_OFS                   3U                                          /*!< TOP PAD FUNC AC PD CFG0: CA3_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA3_PD_VAL(regval)               (BIT(3) & ((uint32_t)(regval) << 3))        /*!< TOP PAD FUNC AC PD CFG0: CA3_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA4_PD                       BIT(4)                                      /*!< PD for the ca4_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA4_PD_OFS                   4U                                          /*!< TOP PAD FUNC AC PD CFG0: CA4_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA4_PD_VAL(regval)               (BIT(4) & ((uint32_t)(regval) << 4))        /*!< TOP PAD FUNC AC PD CFG0: CA4_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA5_PD                       BIT(5)                                      /*!< PD for the ca5_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA5_PD_OFS                   5U                                          /*!< TOP PAD FUNC AC PD CFG0: CA5_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA5_PD_VAL(regval)               (BIT(5) & ((uint32_t)(regval) << 5))        /*!< TOP PAD FUNC AC PD CFG0: CA5_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA6_PD                       BIT(6)                                      /*!< PD for the ca6_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA6_PD_OFS                   6U                                          /*!< TOP PAD FUNC AC PD CFG0: CA6_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA6_PD_VAL(regval)               (BIT(6) & ((uint32_t)(regval) << 6))        /*!< TOP PAD FUNC AC PD CFG0: CA6_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA7_PD                       BIT(7)                                      /*!< PD for the ca7_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA7_PD_OFS                   7U                                          /*!< TOP PAD FUNC AC PD CFG0: CA7_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA7_PD_VAL(regval)               (BIT(7) & ((uint32_t)(regval) << 7))        /*!< TOP PAD FUNC AC PD CFG0: CA7_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA8_PD                       BIT(8)                                      /*!< PD for the ca8_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA8_PD_OFS                   8U                                          /*!< TOP PAD FUNC AC PD CFG0: CA8_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA8_PD_VAL(regval)               (BIT(8) & ((uint32_t)(regval) << 8))        /*!< TOP PAD FUNC AC PD CFG0: CA8_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA9_PD                       BIT(9)                                      /*!< PD for the ca9_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA9_PD_OFS                   9U                                          /*!< TOP PAD FUNC AC PD CFG0: CA9_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA9_PD_VAL(regval)               (BIT(9) & ((uint32_t)(regval) << 9))        /*!< TOP PAD FUNC AC PD CFG0: CA9_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA10_PD                      BIT(10)                                      /*!< PD for the ca10_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA10_PD_OFS                  10U                                          /*!< TOP PAD FUNC AC PD CFG0: CA10_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA10_PD_VAL(regval)              (BIT(10) & ((uint32_t)(regval) << 10))        /*!< TOP PAD FUNC AC PD CFG0: CA10_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA11_PD                      BIT(11)                                      /*!< PD for the ca11_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA11_PD_OFS                  11U                                          /*!< TOP PAD FUNC AC PD CFG0: CA11_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA11_PD_VAL(regval)              (BIT(11) & ((uint32_t)(regval) << 11))        /*!< TOP PAD FUNC AC PD CFG0: CA11_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA12_PD                      BIT(12)                                      /*!< PD for the ca12_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA12_PD_OFS                  12U                                          /*!< TOP PAD FUNC AC PD CFG0: CA12_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA12_PD_VAL(regval)              (BIT(12) & ((uint32_t)(regval) << 12))        /*!< TOP PAD FUNC AC PD CFG0: CA12_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA13_PD                      BIT(13)                                      /*!< PD for the ca13_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA13_PD_OFS                  13U                                          /*!< TOP PAD FUNC AC PD CFG0: CA13_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA13_PD_VAL(regval)              (BIT(13) & ((uint32_t)(regval) << 13))        /*!< TOP PAD FUNC AC PD CFG0: CA13_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA14_PD                      BIT(14)                                      /*!< PD for the ca14_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA14_PD_OFS                  14U                                          /*!< TOP PAD FUNC AC PD CFG0: CA14_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA14_PD_VAL(regval)              (BIT(14) & ((uint32_t)(regval) << 14))        /*!< TOP PAD FUNC AC PD CFG0: CA14_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA15_PD                      BIT(15)                                      /*!< PD for the ca15_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA15_PD_OFS                  15U                                          /*!< TOP PAD FUNC AC PD CFG0: CA15_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA15_PD_VAL(regval)              (BIT(15) & ((uint32_t)(regval) << 15))        /*!< TOP PAD FUNC AC PD CFG0: CA15_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA16_PD                      BIT(16)                                      /*!< PD for the ca16_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA16_PD_OFS                  16U                                          /*!< TOP PAD FUNC AC PD CFG0: CA16_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA16_PD_VAL(regval)              (BIT(16) & ((uint32_t)(regval) << 16))        /*!< TOP PAD FUNC AC PD CFG0: CA16_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA17_PD                      BIT(17)                                      /*!< PD for the ca17_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA17_PD_OFS                  17U                                          /*!< TOP PAD FUNC AC PD CFG0: CA17_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA17_PD_VAL(regval)              (BIT(17) & ((uint32_t)(regval) << 17))        /*!< TOP PAD FUNC AC PD CFG0: CA17_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA18_PD                      BIT(18)                                      /*!< PD for the ca18_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA18_PD_OFS                  18U                                          /*!< TOP PAD FUNC AC PD CFG0: CA18_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA18_PD_VAL(regval)              (BIT(18) & ((uint32_t)(regval) << 18))        /*!< TOP PAD FUNC AC PD CFG0: CA18_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA19_PD                      BIT(19)                                      /*!< PD for the ca19_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA19_PD_OFS                  19U                                          /*!< TOP PAD FUNC AC PD CFG0: CA19_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA19_PD_VAL(regval)              (BIT(19) & ((uint32_t)(regval) << 19))        /*!< TOP PAD FUNC AC PD CFG0: CA19_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA20_PD                      BIT(20)                                      /*!< PD for the ca20_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA20_PD_OFS                  20U                                          /*!< TOP PAD FUNC AC PD CFG0: CA20_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA20_PD_VAL(regval)              (BIT(20) & ((uint32_t)(regval) << 20))        /*!< TOP PAD FUNC AC PD CFG0: CA20_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA21_PD                      BIT(21)                                      /*!< PD for the ca21_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA21_PD_OFS                  21U                                          /*!< TOP PAD FUNC AC PD CFG0: CA21_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA21_PD_VAL(regval)              (BIT(21) & ((uint32_t)(regval) << 21))        /*!< TOP PAD FUNC AC PD CFG0: CA21_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA22_PD                      BIT(22)                                      /*!< PD for the ca22_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA22_PD_OFS                  22U                                          /*!< TOP PAD FUNC AC PD CFG0: CA22_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA22_PD_VAL(regval)              (BIT(22) & ((uint32_t)(regval) << 22))        /*!< TOP PAD FUNC AC PD CFG0: CA22_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA23_PD                      BIT(23)                                      /*!< PD for the ca23_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA23_PD_OFS                  23U                                          /*!< TOP PAD FUNC AC PD CFG0: CA23_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA23_PD_VAL(regval)              (BIT(23) & ((uint32_t)(regval) << 23))        /*!< TOP PAD FUNC AC PD CFG0: CA23_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA24_PD                      BIT(24)                                      /*!< PD for the ca24_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA24_PD_OFS                  24U                                          /*!< TOP PAD FUNC AC PD CFG0: CA24_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA24_PD_VAL(regval)              (BIT(24) & ((uint32_t)(regval) << 24))        /*!< TOP PAD FUNC AC PD CFG0: CA24_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA25_PD                      BIT(25)                                      /*!< PD for the ca25_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA25_PD_OFS                  25U                                          /*!< TOP PAD FUNC AC PD CFG0: CA25_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA25_PD_VAL(regval)              (BIT(25) & ((uint32_t)(regval) << 25))        /*!< TOP PAD FUNC AC PD CFG0: CA25_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA26_PD                      BIT(26)                                      /*!< PD for the ca26_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA26_PD_OFS                  26U                                          /*!< TOP PAD FUNC AC PD CFG0: CA26_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA26_PD_VAL(regval)              (BIT(26) & ((uint32_t)(regval) << 26))        /*!< TOP PAD FUNC AC PD CFG0: CA26_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA27_PD                      BIT(27)                                      /*!< PD for the ca27_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA27_PD_OFS                  27U                                          /*!< TOP PAD FUNC AC PD CFG0: CA27_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA27_PD_VAL(regval)              (BIT(27) & ((uint32_t)(regval) << 27))        /*!< TOP PAD FUNC AC PD CFG0: CA27_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA28_PD                      BIT(28)                                      /*!< PD for the ca28_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA28_PD_OFS                  28U                                          /*!< TOP PAD FUNC AC PD CFG0: CA28_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA28_PD_VAL(regval)              (BIT(28) & ((uint32_t)(regval) << 28))        /*!< TOP PAD FUNC AC PD CFG0: CA28_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA29_PD                      BIT(29)                                      /*!< PD for the ca29_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA29_PD_OFS                  29U                                          /*!< TOP PAD FUNC AC PD CFG0: CA29_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA29_PD_VAL(regval)              (BIT(29) & ((uint32_t)(regval) << 29))        /*!< TOP PAD FUNC AC PD CFG0: CA29_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA30_PD                      BIT(30)                                      /*!< PD for the ca30_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA30_PD_OFS                  30U                                          /*!< TOP PAD FUNC AC PD CFG0: CA30_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA30_PD_VAL(regval)              (BIT(30) & ((uint32_t)(regval) << 30))        /*!< TOP PAD FUNC AC PD CFG0: CA30_PD Bit Value */  
#define TOP_PAD_FUNC_AC_PD_CFG0_CA31_PD                      BIT(31)                                      /*!< PD for the ca31_pad. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA31_PD_OFS                  31U                                          /*!< TOP PAD FUNC AC PD CFG0: CA31_PD Bit Offset */
#define TOP_PAD_FUNC_AC_PD_CFG0_CA31_PD_VAL(regval)              (BIT(31) & ((uint32_t)(regval) << 31))        /*!< TOP PAD FUNC AC PD CFG0: CA31_PD Bit Value */  
 
 /* ===== TOP PAD FUNC MISC_PD_CFG Register definition ===== */
#define TOP_PAD_FUNC_MISC_PD_CFG_CLK0_PD                      BIT(0)                                      /*!< PD for the CLK PADs. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_MISC_PD_CFG_CLK0_PD_OFS                  0U                                          /*!< TOP PAD FUNC MISC PD CFG: CLK0_PD Bit Offset */
#define TOP_PAD_FUNC_MISC_PD_CFG_CLK0_PD_VAL(regval)              (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP PAD FUNC MISC PD CFG: CLK0_PD Bit Value */  
 
 /* ===== TOP PAD FUNC DB_PD_CFG Register definition ===== */
#define TOP_PAD_FUNC_DB_PD_CFG_DB0_LNIB_PD                  BIT(0)                                      /*!< PD for the dqs/dqs_dm/dq PADs. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_DB_PD_CFG_DB0_LNIB_PD_OFS              0U                                          /*!< TOP PAD FUNC DB PD CFG: DB0_LNIB_PD Bit Offset */
#define TOP_PAD_FUNC_DB_PD_CFG_DB0_LNIB_PD_VAL(regval)          (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP PAD FUNC DB PD CFG: DB0_LNIB_PD Bit Value */  
#define TOP_PAD_FUNC_DB_PD_CFG_DB0_HNIB_PD                  BIT(1)                                      /*!< PD for the dqs/dqs_dm/dq PADs. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_DB_PD_CFG_DB0_HNIB_PD_OFS              1U                                          /*!< TOP PAD FUNC DB PD CFG: DB0_HNIB_PD Bit Offset */
#define TOP_PAD_FUNC_DB_PD_CFG_DB0_HNIB_PD_VAL(regval)          (BIT(1) & ((uint32_t)(regval) << 1))        /*!< TOP PAD FUNC DB PD CFG: DB0_HNIB_PD Bit Value */  
#define TOP_PAD_FUNC_DB_PD_CFG_DB1_LNIB_PD                  BIT(2)                                      /*!< PD for the dqs/dqs_dm/dq PADs. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_DB_PD_CFG_DB1_LNIB_PD_OFS              2U                                          /*!< TOP PAD FUNC DB PD CFG: DB1_LNIB_PD Bit Offset */
#define TOP_PAD_FUNC_DB_PD_CFG_DB1_LNIB_PD_VAL(regval)          (BIT(2) & ((uint32_t)(regval) << 2))        /*!< TOP PAD FUNC DB PD CFG: DB1_LNIB_PD Bit Value */  
#define TOP_PAD_FUNC_DB_PD_CFG_DB1_HNIB_PD                  BIT(3)                                      /*!< PD for the dqs/dqs_dm/dq PADs. Note: See details in the IO spec. */
#define TOP_PAD_FUNC_DB_PD_CFG_DB1_HNIB_PD_OFS              3U                                          /*!< TOP PAD FUNC DB PD CFG: DB1_HNIB_PD Bit Offset */
#define TOP_PAD_FUNC_DB_PD_CFG_DB1_HNIB_PD_VAL(regval)          (BIT(3) & ((uint32_t)(regval) << 3))        /*!< TOP PAD FUNC DB PD CFG: DB1_HNIB_PD Bit Value */  
 
 /* ===== TOP PAD ATB AC_ATB_CFG0 Register definition ===== */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA0_ATB_SEL_MASK             BITS(0,1)                                   /*!< TOP PAD ATB AC ATB CFG0: CA0_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA0_ATB_SEL_OFS              0U                                          /*!< TOP PAD ATB AC ATB CFG0: CA0_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA0_ATB_SEL(regval)          (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< TOP PAD ATB AC ATB CFG0: CA0_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA1_ATB_SEL_MASK             BITS(2,3)                                   /*!< TOP PAD ATB AC ATB CFG0: CA1_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA1_ATB_SEL_OFS              2U                                          /*!< TOP PAD ATB AC ATB CFG0: CA1_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA1_ATB_SEL(regval)          (BITS(2,3) & ((uint32_t)(regval) << 2))        /*!< TOP PAD ATB AC ATB CFG0: CA1_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA2_ATB_SEL_MASK             BITS(4,5)                                   /*!< TOP PAD ATB AC ATB CFG0: CA2_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA2_ATB_SEL_OFS              4U                                          /*!< TOP PAD ATB AC ATB CFG0: CA2_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA2_ATB_SEL(regval)          (BITS(4,5) & ((uint32_t)(regval) << 4))        /*!< TOP PAD ATB AC ATB CFG0: CA2_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA3_ATB_SEL_MASK             BITS(6,7)                                   /*!< TOP PAD ATB AC ATB CFG0: CA3_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA3_ATB_SEL_OFS              6U                                          /*!< TOP PAD ATB AC ATB CFG0: CA3_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA3_ATB_SEL(regval)          (BITS(6,7) & ((uint32_t)(regval) << 6))        /*!< TOP PAD ATB AC ATB CFG0: CA3_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA4_ATB_SEL_MASK             BITS(8,9)                                   /*!< TOP PAD ATB AC ATB CFG0: CA4_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA4_ATB_SEL_OFS              8U                                          /*!< TOP PAD ATB AC ATB CFG0: CA4_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA4_ATB_SEL(regval)          (BITS(8,9) & ((uint32_t)(regval) << 8))        /*!< TOP PAD ATB AC ATB CFG0: CA4_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA5_ATB_SEL_MASK             BITS(10,11)                                   /*!< TOP PAD ATB AC ATB CFG0: CA5_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA5_ATB_SEL_OFS              10U                                          /*!< TOP PAD ATB AC ATB CFG0: CA5_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA5_ATB_SEL(regval)          (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< TOP PAD ATB AC ATB CFG0: CA5_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA6_ATB_SEL_MASK             BITS(12,13)                                   /*!< TOP PAD ATB AC ATB CFG0: CA6_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA6_ATB_SEL_OFS              12U                                          /*!< TOP PAD ATB AC ATB CFG0: CA6_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA6_ATB_SEL(regval)          (BITS(12,13) & ((uint32_t)(regval) << 12))        /*!< TOP PAD ATB AC ATB CFG0: CA6_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA7_ATB_SEL_MASK             BITS(14,15)                                   /*!< TOP PAD ATB AC ATB CFG0: CA7_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA7_ATB_SEL_OFS              14U                                          /*!< TOP PAD ATB AC ATB CFG0: CA7_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA7_ATB_SEL(regval)          (BITS(14,15) & ((uint32_t)(regval) << 14))        /*!< TOP PAD ATB AC ATB CFG0: CA7_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA8_ATB_SEL_MASK             BITS(16,17)                                   /*!< TOP PAD ATB AC ATB CFG0: CA8_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA8_ATB_SEL_OFS              16U                                          /*!< TOP PAD ATB AC ATB CFG0: CA8_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA8_ATB_SEL(regval)          (BITS(16,17) & ((uint32_t)(regval) << 16))        /*!< TOP PAD ATB AC ATB CFG0: CA8_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA9_ATB_SEL_MASK             BITS(18,19)                                   /*!< TOP PAD ATB AC ATB CFG0: CA9_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA9_ATB_SEL_OFS              18U                                          /*!< TOP PAD ATB AC ATB CFG0: CA9_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA9_ATB_SEL(regval)          (BITS(18,19) & ((uint32_t)(regval) << 18))        /*!< TOP PAD ATB AC ATB CFG0: CA9_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA10_ATB_SEL_MASK            BITS(20,21)                                   /*!< TOP PAD ATB AC ATB CFG0: CA10_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA10_ATB_SEL_OFS             20U                                          /*!< TOP PAD ATB AC ATB CFG0: CA10_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA10_ATB_SEL(regval)         (BITS(20,21) & ((uint32_t)(regval) << 20))        /*!< TOP PAD ATB AC ATB CFG0: CA10_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA11_ATB_SEL_MASK            BITS(22,23)                                   /*!< TOP PAD ATB AC ATB CFG0: CA11_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA11_ATB_SEL_OFS             22U                                          /*!< TOP PAD ATB AC ATB CFG0: CA11_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA11_ATB_SEL(regval)         (BITS(22,23) & ((uint32_t)(regval) << 22))        /*!< TOP PAD ATB AC ATB CFG0: CA11_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA12_ATB_SEL_MASK            BITS(24,25)                                   /*!< TOP PAD ATB AC ATB CFG0: CA12_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA12_ATB_SEL_OFS             24U                                          /*!< TOP PAD ATB AC ATB CFG0: CA12_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA12_ATB_SEL(regval)         (BITS(24,25) & ((uint32_t)(regval) << 24))        /*!< TOP PAD ATB AC ATB CFG0: CA12_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA13_ATB_SEL_MASK            BITS(26,27)                                   /*!< TOP PAD ATB AC ATB CFG0: CA13_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA13_ATB_SEL_OFS             26U                                          /*!< TOP PAD ATB AC ATB CFG0: CA13_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA13_ATB_SEL(regval)         (BITS(26,27) & ((uint32_t)(regval) << 26))        /*!< TOP PAD ATB AC ATB CFG0: CA13_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA14_ATB_SEL_MASK            BITS(28,29)                                   /*!< TOP PAD ATB AC ATB CFG0: CA14_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA14_ATB_SEL_OFS             28U                                          /*!< TOP PAD ATB AC ATB CFG0: CA14_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA14_ATB_SEL(regval)         (BITS(28,29) & ((uint32_t)(regval) << 28))        /*!< TOP PAD ATB AC ATB CFG0: CA14_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA15_ATB_SEL_MASK            BITS(30,31)                                   /*!< TOP PAD ATB AC ATB CFG0: CA15_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG0_CA15_ATB_SEL_OFS             30U                                          /*!< TOP PAD ATB AC ATB CFG0: CA15_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG0_CA15_ATB_SEL(regval)         (BITS(30,31) & ((uint32_t)(regval) << 30))        /*!< TOP PAD ATB AC ATB CFG0: CA15_ATB_SEL Bit Value */  
 
 /* ===== TOP PAD ATB AC_ATB_CFG1 Register definition ===== */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA16_ATB_SEL_MASK            BITS(0,1)                                   /*!< TOP PAD ATB AC ATB CFG1: CA16_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA16_ATB_SEL_OFS             0U                                          /*!< TOP PAD ATB AC ATB CFG1: CA16_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA16_ATB_SEL(regval)         (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< TOP PAD ATB AC ATB CFG1: CA16_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA17_ATB_SEL_MASK            BITS(2,3)                                   /*!< TOP PAD ATB AC ATB CFG1: CA17_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA17_ATB_SEL_OFS             2U                                          /*!< TOP PAD ATB AC ATB CFG1: CA17_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA17_ATB_SEL(regval)         (BITS(2,3) & ((uint32_t)(regval) << 2))        /*!< TOP PAD ATB AC ATB CFG1: CA17_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA18_ATB_SEL_MASK            BITS(4,5)                                   /*!< TOP PAD ATB AC ATB CFG1: CA18_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA18_ATB_SEL_OFS             4U                                          /*!< TOP PAD ATB AC ATB CFG1: CA18_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA18_ATB_SEL(regval)         (BITS(4,5) & ((uint32_t)(regval) << 4))        /*!< TOP PAD ATB AC ATB CFG1: CA18_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA19_ATB_SEL_MASK            BITS(6,7)                                   /*!< TOP PAD ATB AC ATB CFG1: CA19_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA19_ATB_SEL_OFS             6U                                          /*!< TOP PAD ATB AC ATB CFG1: CA19_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA19_ATB_SEL(regval)         (BITS(6,7) & ((uint32_t)(regval) << 6))        /*!< TOP PAD ATB AC ATB CFG1: CA19_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA20_ATB_SEL_MASK            BITS(8,9)                                   /*!< TOP PAD ATB AC ATB CFG1: CA20_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA20_ATB_SEL_OFS             8U                                          /*!< TOP PAD ATB AC ATB CFG1: CA20_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA20_ATB_SEL(regval)         (BITS(8,9) & ((uint32_t)(regval) << 8))        /*!< TOP PAD ATB AC ATB CFG1: CA20_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA21_ATB_SEL_MASK            BITS(10,11)                                   /*!< TOP PAD ATB AC ATB CFG1: CA21_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA21_ATB_SEL_OFS             10U                                          /*!< TOP PAD ATB AC ATB CFG1: CA21_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA21_ATB_SEL(regval)         (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< TOP PAD ATB AC ATB CFG1: CA21_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA22_ATB_SEL_MASK            BITS(12,13)                                   /*!< TOP PAD ATB AC ATB CFG1: CA22_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA22_ATB_SEL_OFS             12U                                          /*!< TOP PAD ATB AC ATB CFG1: CA22_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA22_ATB_SEL(regval)         (BITS(12,13) & ((uint32_t)(regval) << 12))        /*!< TOP PAD ATB AC ATB CFG1: CA22_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA23_ATB_SEL_MASK            BITS(14,15)                                   /*!< TOP PAD ATB AC ATB CFG1: CA23_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA23_ATB_SEL_OFS             14U                                          /*!< TOP PAD ATB AC ATB CFG1: CA23_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA23_ATB_SEL(regval)         (BITS(14,15) & ((uint32_t)(regval) << 14))        /*!< TOP PAD ATB AC ATB CFG1: CA23_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA24_ATB_SEL_MASK            BITS(16,17)                                   /*!< TOP PAD ATB AC ATB CFG1: CA24_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA24_ATB_SEL_OFS             16U                                          /*!< TOP PAD ATB AC ATB CFG1: CA24_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA24_ATB_SEL(regval)         (BITS(16,17) & ((uint32_t)(regval) << 16))        /*!< TOP PAD ATB AC ATB CFG1: CA24_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA25_ATB_SEL_MASK            BITS(18,19)                                   /*!< TOP PAD ATB AC ATB CFG1: CA25_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA25_ATB_SEL_OFS             18U                                          /*!< TOP PAD ATB AC ATB CFG1: CA25_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA25_ATB_SEL(regval)         (BITS(18,19) & ((uint32_t)(regval) << 18))        /*!< TOP PAD ATB AC ATB CFG1: CA25_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA26_ATB_SEL_MASK            BITS(20,21)                                   /*!< TOP PAD ATB AC ATB CFG1: CA26_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA26_ATB_SEL_OFS             20U                                          /*!< TOP PAD ATB AC ATB CFG1: CA26_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA26_ATB_SEL(regval)         (BITS(20,21) & ((uint32_t)(regval) << 20))        /*!< TOP PAD ATB AC ATB CFG1: CA26_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA27_ATB_SEL_MASK            BITS(22,23)                                   /*!< TOP PAD ATB AC ATB CFG1: CA27_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA27_ATB_SEL_OFS             22U                                          /*!< TOP PAD ATB AC ATB CFG1: CA27_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA27_ATB_SEL(regval)         (BITS(22,23) & ((uint32_t)(regval) << 22))        /*!< TOP PAD ATB AC ATB CFG1: CA27_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA28_ATB_SEL_MASK            BITS(24,25)                                   /*!< TOP PAD ATB AC ATB CFG1: CA28_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA28_ATB_SEL_OFS             24U                                          /*!< TOP PAD ATB AC ATB CFG1: CA28_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA28_ATB_SEL(regval)         (BITS(24,25) & ((uint32_t)(regval) << 24))        /*!< TOP PAD ATB AC ATB CFG1: CA28_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA29_ATB_SEL_MASK            BITS(26,27)                                   /*!< TOP PAD ATB AC ATB CFG1: CA29_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA29_ATB_SEL_OFS             26U                                          /*!< TOP PAD ATB AC ATB CFG1: CA29_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA29_ATB_SEL(regval)         (BITS(26,27) & ((uint32_t)(regval) << 26))        /*!< TOP PAD ATB AC ATB CFG1: CA29_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA30_ATB_SEL_MASK            BITS(28,29)                                   /*!< TOP PAD ATB AC ATB CFG1: CA30_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA30_ATB_SEL_OFS             28U                                          /*!< TOP PAD ATB AC ATB CFG1: CA30_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA30_ATB_SEL(regval)         (BITS(28,29) & ((uint32_t)(regval) << 28))        /*!< TOP PAD ATB AC ATB CFG1: CA30_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA31_ATB_SEL_MASK            BITS(30,31)                                   /*!< TOP PAD ATB AC ATB CFG1: CA31_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_AC_ATB_CFG1_CA31_ATB_SEL_OFS             30U                                          /*!< TOP PAD ATB AC ATB CFG1: CA31_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_ATB_CFG1_CA31_ATB_SEL(regval)         (BITS(30,31) & ((uint32_t)(regval) << 30))        /*!< TOP PAD ATB AC ATB CFG1: CA31_ATB_SEL Bit Value */  
 
 /* ===== TOP PAD ATB MISC_ATB_CFG Register definition ===== */
#define TOP_PAD_ATB_MISC_ATB_CFG_CLK0_ATB_SEL_MASK            BITS(0,1)                                   /*!< TOP PAD ATB MISC ATB CFG: CLK0_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_MISC_ATB_CFG_CLK0_ATB_SEL_OFS             0U                                          /*!< TOP PAD ATB MISC ATB CFG: CLK0_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_MISC_ATB_CFG_CLK0_ATB_SEL(regval)         (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< TOP PAD ATB MISC ATB CFG: CLK0_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_MISC_ATB_CFG_PAD_CAL_ATB_SEL_MASK         BITS(4,8)                                   /*!< TOP PAD ATB MISC ATB CFG: PAD_CAL_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_MISC_ATB_CFG_PAD_CAL_ATB_SEL_OFS          4U                                          /*!< TOP PAD ATB MISC ATB CFG: PAD_CAL_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_MISC_ATB_CFG_PAD_CAL_ATB_SEL(regval)      (BITS(4,8) & ((uint32_t)(regval) << 4))        /*!< TOP PAD ATB MISC ATB CFG: PAD_CAL_ATB_SEL Bit Value */  
 
 /* ===== TOP PAD ATB DB_ATB_CFG0 Register definition ===== */
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ0_ATB_SEL_MASK             BITS(0,1)                                   /*!< TOP PAD ATB DB ATB CFG0: DQ0_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ0_ATB_SEL_OFS              0U                                          /*!< TOP PAD ATB DB ATB CFG0: DQ0_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ0_ATB_SEL(regval)          (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< TOP PAD ATB DB ATB CFG0: DQ0_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ1_ATB_SEL_MASK             BITS(2,3)                                   /*!< TOP PAD ATB DB ATB CFG0: DQ1_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ1_ATB_SEL_OFS              2U                                          /*!< TOP PAD ATB DB ATB CFG0: DQ1_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ1_ATB_SEL(regval)          (BITS(2,3) & ((uint32_t)(regval) << 2))        /*!< TOP PAD ATB DB ATB CFG0: DQ1_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ2_ATB_SEL_MASK             BITS(4,5)                                   /*!< TOP PAD ATB DB ATB CFG0: DQ2_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ2_ATB_SEL_OFS              4U                                          /*!< TOP PAD ATB DB ATB CFG0: DQ2_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ2_ATB_SEL(regval)          (BITS(4,5) & ((uint32_t)(regval) << 4))        /*!< TOP PAD ATB DB ATB CFG0: DQ2_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ3_ATB_SEL_MASK             BITS(6,7)                                   /*!< TOP PAD ATB DB ATB CFG0: DQ3_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ3_ATB_SEL_OFS              6U                                          /*!< TOP PAD ATB DB ATB CFG0: DQ3_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ3_ATB_SEL(regval)          (BITS(6,7) & ((uint32_t)(regval) << 6))        /*!< TOP PAD ATB DB ATB CFG0: DQ3_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ4_ATB_SEL_MASK             BITS(8,9)                                   /*!< TOP PAD ATB DB ATB CFG0: DQ4_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ4_ATB_SEL_OFS              8U                                          /*!< TOP PAD ATB DB ATB CFG0: DQ4_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ4_ATB_SEL(regval)          (BITS(8,9) & ((uint32_t)(regval) << 8))        /*!< TOP PAD ATB DB ATB CFG0: DQ4_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ5_ATB_SEL_MASK             BITS(10,11)                                   /*!< TOP PAD ATB DB ATB CFG0: DQ5_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ5_ATB_SEL_OFS              10U                                          /*!< TOP PAD ATB DB ATB CFG0: DQ5_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ5_ATB_SEL(regval)          (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< TOP PAD ATB DB ATB CFG0: DQ5_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ6_ATB_SEL_MASK             BITS(12,13)                                   /*!< TOP PAD ATB DB ATB CFG0: DQ6_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ6_ATB_SEL_OFS              12U                                          /*!< TOP PAD ATB DB ATB CFG0: DQ6_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ6_ATB_SEL(regval)          (BITS(12,13) & ((uint32_t)(regval) << 12))        /*!< TOP PAD ATB DB ATB CFG0: DQ6_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ7_ATB_SEL_MASK             BITS(14,15)                                   /*!< TOP PAD ATB DB ATB CFG0: DQ7_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ7_ATB_SEL_OFS              14U                                          /*!< TOP PAD ATB DB ATB CFG0: DQ7_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ7_ATB_SEL(regval)          (BITS(14,15) & ((uint32_t)(regval) << 14))        /*!< TOP PAD ATB DB ATB CFG0: DQ7_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ8_ATB_SEL_MASK             BITS(16,17)                                   /*!< TOP PAD ATB DB ATB CFG0: DQ8_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ8_ATB_SEL_OFS              16U                                          /*!< TOP PAD ATB DB ATB CFG0: DQ8_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG0_DQ8_ATB_SEL(regval)          (BITS(16,17) & ((uint32_t)(regval) << 16))        /*!< TOP PAD ATB DB ATB CFG0: DQ8_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQS_ATB_SEL_MASK             BITS(18,19)                                   /*!< TOP PAD ATB DB ATB CFG0: DQS_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG0_DQS_ATB_SEL_OFS              18U                                          /*!< TOP PAD ATB DB ATB CFG0: DQS_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG0_DQS_ATB_SEL(regval)          (BITS(18,19) & ((uint32_t)(regval) << 18))        /*!< TOP PAD ATB DB ATB CFG0: DQS_ATB_SEL Bit Value */  
 
 /* ===== TOP PAD ATB DB_ATB_CFG1 Register definition ===== */
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ0_ATB_SEL_MASK             BITS(0,1)                                   /*!< TOP PAD ATB DB ATB CFG1: DQ0_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ0_ATB_SEL_OFS              0U                                          /*!< TOP PAD ATB DB ATB CFG1: DQ0_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ0_ATB_SEL(regval)          (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< TOP PAD ATB DB ATB CFG1: DQ0_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ1_ATB_SEL_MASK             BITS(2,3)                                   /*!< TOP PAD ATB DB ATB CFG1: DQ1_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ1_ATB_SEL_OFS              2U                                          /*!< TOP PAD ATB DB ATB CFG1: DQ1_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ1_ATB_SEL(regval)          (BITS(2,3) & ((uint32_t)(regval) << 2))        /*!< TOP PAD ATB DB ATB CFG1: DQ1_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ2_ATB_SEL_MASK             BITS(4,5)                                   /*!< TOP PAD ATB DB ATB CFG1: DQ2_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ2_ATB_SEL_OFS              4U                                          /*!< TOP PAD ATB DB ATB CFG1: DQ2_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ2_ATB_SEL(regval)          (BITS(4,5) & ((uint32_t)(regval) << 4))        /*!< TOP PAD ATB DB ATB CFG1: DQ2_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ3_ATB_SEL_MASK             BITS(6,7)                                   /*!< TOP PAD ATB DB ATB CFG1: DQ3_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ3_ATB_SEL_OFS              6U                                          /*!< TOP PAD ATB DB ATB CFG1: DQ3_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ3_ATB_SEL(regval)          (BITS(6,7) & ((uint32_t)(regval) << 6))        /*!< TOP PAD ATB DB ATB CFG1: DQ3_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ4_ATB_SEL_MASK             BITS(8,9)                                   /*!< TOP PAD ATB DB ATB CFG1: DQ4_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ4_ATB_SEL_OFS              8U                                          /*!< TOP PAD ATB DB ATB CFG1: DQ4_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ4_ATB_SEL(regval)          (BITS(8,9) & ((uint32_t)(regval) << 8))        /*!< TOP PAD ATB DB ATB CFG1: DQ4_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ5_ATB_SEL_MASK             BITS(10,11)                                   /*!< TOP PAD ATB DB ATB CFG1: DQ5_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ5_ATB_SEL_OFS              10U                                          /*!< TOP PAD ATB DB ATB CFG1: DQ5_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ5_ATB_SEL(regval)          (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< TOP PAD ATB DB ATB CFG1: DQ5_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ6_ATB_SEL_MASK             BITS(12,13)                                   /*!< TOP PAD ATB DB ATB CFG1: DQ6_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ6_ATB_SEL_OFS              12U                                          /*!< TOP PAD ATB DB ATB CFG1: DQ6_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ6_ATB_SEL(regval)          (BITS(12,13) & ((uint32_t)(regval) << 12))        /*!< TOP PAD ATB DB ATB CFG1: DQ6_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ7_ATB_SEL_MASK             BITS(14,15)                                   /*!< TOP PAD ATB DB ATB CFG1: DQ7_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ7_ATB_SEL_OFS              14U                                          /*!< TOP PAD ATB DB ATB CFG1: DQ7_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ7_ATB_SEL(regval)          (BITS(14,15) & ((uint32_t)(regval) << 14))        /*!< TOP PAD ATB DB ATB CFG1: DQ7_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ8_ATB_SEL_MASK             BITS(16,17)                                   /*!< TOP PAD ATB DB ATB CFG1: DQ8_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ8_ATB_SEL_OFS              16U                                          /*!< TOP PAD ATB DB ATB CFG1: DQ8_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG1_DQ8_ATB_SEL(regval)          (BITS(16,17) & ((uint32_t)(regval) << 16))        /*!< TOP PAD ATB DB ATB CFG1: DQ8_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQS_ATB_SEL_MASK             BITS(18,19)                                   /*!< TOP PAD ATB DB ATB CFG1: DQS_ATB_SEL Bit Mask */  
#define TOP_PAD_ATB_DB_ATB_CFG1_DQS_ATB_SEL_OFS              18U                                          /*!< TOP PAD ATB DB ATB CFG1: DQS_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_ATB_CFG1_DQS_ATB_SEL(regval)          (BITS(18,19) & ((uint32_t)(regval) << 18))        /*!< TOP PAD ATB DB ATB CFG1: DQS_ATB_SEL Bit Value */  
 
 /* ===== TOP PAD ATB AC_VREF_ATB_CFG Register definition ===== */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF0_ATB_SEL                BIT(0)                                      /*!< ATB_SEL for the u_vref_gen_0. Note: See details in the IO spec. */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF0_ATB_SEL_OFS            0U                                          /*!< TOP PAD ATB AC VREF ATB CFG: VREF0_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF0_ATB_SEL_VAL(regval)        (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP PAD ATB AC VREF ATB CFG: VREF0_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF1_ATB_SEL                BIT(1)                                      /*!< ATB_SEL for the u_vref_gen_1. Note: See details in the IO spec. */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF1_ATB_SEL_OFS            1U                                          /*!< TOP PAD ATB AC VREF ATB CFG: VREF1_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF1_ATB_SEL_VAL(regval)        (BIT(1) & ((uint32_t)(regval) << 1))        /*!< TOP PAD ATB AC VREF ATB CFG: VREF1_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF2_ATB_SEL                BIT(2)                                      /*!< ATB_SEL for the u_vref_gen_2. Note: See details in the IO spec. */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF2_ATB_SEL_OFS            2U                                          /*!< TOP PAD ATB AC VREF ATB CFG: VREF2_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF2_ATB_SEL_VAL(regval)        (BIT(2) & ((uint32_t)(regval) << 2))        /*!< TOP PAD ATB AC VREF ATB CFG: VREF2_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF3_ATB_SEL                BIT(3)                                      /*!< ATB_SEL for the u_vref_gen_3. Note: See details in the IO spec. */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF3_ATB_SEL_OFS            3U                                          /*!< TOP PAD ATB AC VREF ATB CFG: VREF3_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF3_ATB_SEL_VAL(regval)        (BIT(3) & ((uint32_t)(regval) << 3))        /*!< TOP PAD ATB AC VREF ATB CFG: VREF3_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF4_ATB_SEL                BIT(4)                                      /*!< ATB_SEL for the u_vref_gen_4. Note: See details in the IO spec. */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF4_ATB_SEL_OFS            4U                                          /*!< TOP PAD ATB AC VREF ATB CFG: VREF4_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF4_ATB_SEL_VAL(regval)        (BIT(4) & ((uint32_t)(regval) << 4))        /*!< TOP PAD ATB AC VREF ATB CFG: VREF4_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF5_ATB_SEL                BIT(5)                                      /*!< ATB_SEL for the u_vref_gen_5. Note: See details in the IO spec. */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF5_ATB_SEL_OFS            5U                                          /*!< TOP PAD ATB AC VREF ATB CFG: VREF5_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_AC_VREF_ATB_CFG_VREF5_ATB_SEL_VAL(regval)        (BIT(5) & ((uint32_t)(regval) << 5))        /*!< TOP PAD ATB AC VREF ATB CFG: VREF5_ATB_SEL Bit Value */  
 
 /* ===== TOP PAD ATB DB_VREF_ATB_CFG Register definition ===== */
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_LNIB_VREF0_ATB_SEL           BIT(0)                                      /*!< ATB_SEL for the vref_gen of DBYTE[0] low nibble. Note: See details in the IO spec. */
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_LNIB_VREF0_ATB_SEL_OFS       0U                                          /*!< TOP PAD ATB DB VREF ATB CFG: LNIB_VREF0_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_LNIB_VREF0_ATB_SEL_VAL(regval)   (BIT(0) & ((uint32_t)(regval) << 0))        /*!< TOP PAD ATB DB VREF ATB CFG: LNIB_VREF0_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_HNIB_VREF0_ATB_SEL           BIT(1)                                      /*!< ATB_SEL for the vref_gen of DBYTE[0] high nibble. Note: See details in the IO spec. */
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_HNIB_VREF0_ATB_SEL_OFS       1U                                          /*!< TOP PAD ATB DB VREF ATB CFG: HNIB_VREF0_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_HNIB_VREF0_ATB_SEL_VAL(regval)   (BIT(1) & ((uint32_t)(regval) << 1))        /*!< TOP PAD ATB DB VREF ATB CFG: HNIB_VREF0_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_LNIB_VREF1_ATB_SEL           BIT(2)                                      /*!< ATB_SEL for the vref_gen of DBYTE[1] low nibble. Note: See details in the IO spec. */
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_LNIB_VREF1_ATB_SEL_OFS       2U                                          /*!< TOP PAD ATB DB VREF ATB CFG: LNIB_VREF1_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_LNIB_VREF1_ATB_SEL_VAL(regval)   (BIT(2) & ((uint32_t)(regval) << 2))        /*!< TOP PAD ATB DB VREF ATB CFG: LNIB_VREF1_ATB_SEL Bit Value */  
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_HNIB_VREF1_ATB_SEL           BIT(3)                                      /*!< ATB_SEL for the vref_gen of DBYTE[1] high nibble. Note: See details in the IO spec. */
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_HNIB_VREF1_ATB_SEL_OFS       3U                                          /*!< TOP PAD ATB DB VREF ATB CFG: HNIB_VREF1_ATB_SEL Bit Offset */
#define TOP_PAD_ATB_DB_VREF_ATB_CFG_HNIB_VREF1_ATB_SEL_VAL(regval)   (BIT(3) & ((uint32_t)(regval) << 3))        /*!< TOP PAD ATB DB VREF ATB CFG: HNIB_VREF1_ATB_SEL Bit Value */  

#define DBYTE_AF_DB_GNRL_CFG_OFFSET                                        0x0 /*!< General configuration */
#define DBYTE_AF_DB_LPBK_CFG_OFFSET                                        0x4 /*!< DBYTE loop back configuration */
#define DBYTE_AF_DB_LPBK_STATUS_OFFSET                                     0x8 /*!< DBYTE loop back status */
#define DBYTE_AF_DB_MSTR_CFG_OFFSET                                        0x10 /*!< DBYTE master delay line configuration for all frequencies */
#define DBYTE_AF_DB_SW_CTRL_CFG_OFFSET                                     0x14 /*!< DBYTE software control configruration */
#define DBYTE_AF_DB_SWAP_CFG0_OFFSET                                       0x18 /*!< DBYTE swap configuration */
#define DBYTE_AF_DB_SWAP_CFG1_OFFSET                                       0x1c /*!< DBYTE swap configuration */
#define DBYTE_AF_DB_LVL_COM_CFG_OFFSET                                     0x20 /*!< DBYTE common configuration for training */
#define DBYTE_AF_DB_WRLVL_CFG_OFFSET                                       0x24 /*!< DBYTE write leveling configuration */
#define DBYTE_AF_DB_GTLVL_CFG_OFFSET                                       0x28 /*!< DBYTE read DQS gate training configuration */
#define DBYTE_AF_DB_RDLVL_CFG0_OFFSET                                      0x2c /*!< DBYTE read data eye training configuration */
#define DBYTE_AF_DB_RDLVL_CFG1_OFFSET                                      0x30 /*!< DBYTE read data eye training configuration */
#define DBYTE_AF_DB_RDLVL_VREF_CFG_OFFSET                                  0x38 /*!< DBYTE vref configuration for read data eye training */
#define DBYTE_AF_DB_WDQLVL_CFG0_OFFSET                                     0x40 /*!< DBYTE write DQ training configuration */
#define DBYTE_AF_DB_WDQLVL_CFG1_OFFSET                                     0x44 /*!< DBYTE write DQ training configuration */
#define DBYTE_AF_DB_WDQLVL_CFG2_OFFSET                                     0x48 /*!< DBYTE write DQ training configuration */
#define DBYTE_AF_DB_WDQLVL_CFG3_OFFSET                                     0x4c /*!< DBYTE write DQ training configuration */
#define DBYTE_AF_DB_LVL_STATUS_OFFSET                                      0x50 /*!< DBYTE training error status */
#define DBYTE_AF_DB_RX_CAL_CFG_OFFSET                                      0x54 /*!< DBYTE RX offset calibration configuration */
#define DBYTE_AF_DB_DQ0_RX_CAL_CODE_OFFSET                                 0x58 /*!< DBYTE RX offset calibration code for each PAD */
#define DBYTE_AF_DB_DQ1_RX_CAL_CODE_OFFSET                                 0x5c /*!< DBYTE RX offset calibration code for each PAD */
#define DBYTE_AF_DB_DQ2_RX_CAL_CODE_OFFSET                                 0x60 /*!< DBYTE RX offset calibration code for each PAD */
#define DBYTE_AF_DB_DQ3_RX_CAL_CODE_OFFSET                                 0x64 /*!< DBYTE RX offset calibration code for each PAD */
#define DBYTE_AF_DB_DQ4_RX_CAL_CODE_OFFSET                                 0x68 /*!< DBYTE RX offset calibration code for each PAD */
#define DBYTE_AF_DB_DQ5_RX_CAL_CODE_OFFSET                                 0x6c /*!< DBYTE RX offset calibration code for each PAD */
#define DBYTE_AF_DB_DQ6_RX_CAL_CODE_OFFSET                                 0x70 /*!< DBYTE RX offset calibration code for each PAD */
#define DBYTE_AF_DB_DQ7_RX_CAL_CODE_OFFSET                                 0x74 /*!< DBYTE RX offset calibration code for each PAD */
#define DBYTE_AF_DB_DQ8_RX_CAL_CODE_OFFSET                                 0x78 /*!< DBYTE RX offset calibration code for each PAD */
#define DBYTE_AF_DB_DQS_RX_CAL_CODE_OFFSET                                 0x7c /*!< DBYTE RX offset calibration code for each PAD */
#define DBYTE_AF_DB_TSEL_CFG_OFFSET                                        0x84 /*!< DBYTE PAD TSEL configuration */
#define DBYTE_OBS_DB_OBS_CFG_OFFSET                                         0xa0 /*!< Reserved for debug */
#define DBYTE_OBS_DB_MSTR_OBS_OFFSET                                        0xa4 /*!< Reserved for debug */
#define DBYTE_OBS_DB_SLV_WRDQS_OBS_OFFSET                                   0xa8 /*!< Reserved for debug */
#define DBYTE_OBS_DB_SLV_WRDQ_OBS_OFFSET                                    0xac /*!< Reserved for debug */
#define DBYTE_OBS_DB_SLV_RDDQS_OBS_OFFSET                                   0xb0 /*!< Reserved for debug */
#define DBYTE_OBS_DB_SLV_RDDQS_GT_OBS_OFFSET                                0xb4 /*!< Reserved for debug */
#define DBYTE_OBS_DB_SLV_MISC_OBS_OFFSET                                    0xb8 /*!< Reserved for debug */
#define DBYTE_OBS_DB_WRLVL_OBS_OFFSET                                       0xbc /*!< Reserved for debug */
#define DBYTE_OBS_DB_WRLVL_MISC_OBS_OFFSET                                  0xc4 /*!< Reserved for debug */
#define DBYTE_OBS_DB_GTLVL_OBS_OFFSET                                       0xc8 /*!< Reserved for debug */
#define DBYTE_OBS_DB_GTLVL_MISC_OBS_OFFSET                                  0xd0 /*!< Reserved for debug */
#define DBYTE_OBS_DB_RDLVL_DQS_R_OBS_OFFSET                                 0xd4 /*!< Reserved for debug */
#define DBYTE_OBS_DB_RDLVL_DQS_F_OBS_OFFSET                                 0xd8 /*!< Reserved for debug */
#define DBYTE_OBS_DB_RDLVL_MISC_OBS_OFFSET                                  0xdc /*!< Reserved for debug */
#define DBYTE_OBS_DB_WDQLVL_OBS_OFFSET                                      0xe0 /*!< Reserved for debug */
#define DBYTE_OBS_DB_WDQLVL_MISC_OBS_OFFSET                                 0xe4 /*!< Reserved for debug */
#define DBYTE_OBS_DB_RX_CAL_OBS_OFFSET                                      0xe8 /*!< Reserved for debug */
#define DBYTE_OBS_DB_RD_FIFO_OBS_OFFSET                                     0xec /*!< Reserved for debug */
#define DBYTE_OBS_DB_LPBK_OBS_OFFSET                                        0xf0 /*!< Reserved for debug */
#define DBYTE_FX_DB_F0_MSTR_CFG_OFFSET                                     0x100 /*!< DBYTE master delay line configuration for frequency 0 */
#define DBYTE_FX_DB_F0_TMG_OFFSET                                          0x110 /*!< DBYTE timing configuration for frequency 0 */
#define DBYTE_FX_DB_F0_TMG1_OFFSET                                         0x120 /*!< DBYTE timing configuration for frequency 0 */
#define DBYTE_FX_DB_F0_VREF_CFG_OFFSET                                     0x130 /*!< DBYTE read vref configuration for frequency 0 */
#define DBYTE_FX_DB_F0_VREFSEL_CFG_OFFSET                                  0x140 /*!< DBYTE read vrefsel for frequency 0 */
#define DBYTE_FX_DB_F1_MSTR_CFG_OFFSET                                     0x104 /*!< DBYTE master delay line configuration for frequency 1 */
#define DBYTE_FX_DB_F1_TMG_OFFSET                                          0x114 /*!< DBYTE timing configuration for frequency 1 */
#define DBYTE_FX_DB_F1_TMG1_OFFSET                                         0x124 /*!< DBYTE timing configuration for frequency 1 */
#define DBYTE_FX_DB_F1_VREF_CFG_OFFSET                                     0x134 /*!< DBYTE read vref configuration for frequency 1 */
#define DBYTE_FX_DB_F1_VREFSEL_CFG_OFFSET                                  0x144 /*!< DBYTE read vrefsel for frequency 1 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_INIT_CFG_OFFSET                            0x200 /*!< Evaluated skew between clk and write DQS for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_CFG_OFFSET                                 0x204 /*!< Trained write DQS delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_START_CFG_OFFSET                        0x208 /*!< Evaluated start point of read gate training for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_CFG_OFFSET                              0x20c /*!< Trained read gate delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_OFFSET                             0x220 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_CFG_OFFSET                             0x224 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_CFG_OFFSET                             0x228 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_CFG_OFFSET                             0x22c /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_CFG_OFFSET                             0x230 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_CFG_OFFSET                             0x234 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_CFG_OFFSET                             0x238 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_CFG_OFFSET                             0x23c /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_CFG_OFFSET                             0x240 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_CFG_OFFSET                                 0x244 /*!< Trained write DQ delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_CFG_OFFSET                                 0x248 /*!< Trained write DQ delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_CFG_OFFSET                                 0x24c /*!< Trained write DQ delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_CFG_OFFSET                                 0x250 /*!< Trained write DQ delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_CFG_OFFSET                                 0x254 /*!< Trained write DQ delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_CFG_OFFSET                                 0x258 /*!< Trained write DQ delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_CFG_OFFSET                                 0x25c /*!< Trained write DQ delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_CFG_OFFSET                                 0x260 /*!< Trained write DQ delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_CFG_OFFSET                                 0x264 /*!< Trained write DQ delay for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_OFFSET                              0x268 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ_SW_CFG_OFFSET                               0x270 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_OFFSET                              0x274 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_OFFSET                              0x278 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_OFFSET                              0x27c /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_OFFSET                              0x280 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_OFFSET                              0x284 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_OFFSET                              0x288 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_OFFSET                              0x28c /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_OFFSET                              0x290 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_OFFSET                              0x294 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_SW_CFG_OFFSET                              0x298 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_SW_CFG_OFFSET                          0x29c /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_SW_CFG_OFFSET                          0x2a0 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_SW_CFG_OFFSET                          0x2a4 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_SW_CFG_OFFSET                          0x2a8 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_SW_CFG_OFFSET                          0x2ac /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_SW_CFG_OFFSET                          0x2b0 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_SW_CFG_OFFSET                          0x2b4 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_SW_CFG_OFFSET                          0x2b8 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_SW_CFG_OFFSET                          0x2bc /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_SW_CFG_OFFSET                           0x2c0 /*!< DBYTE software mode configuration for frequency 0, rank 0 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_INIT_CFG_OFFSET                            0x300 /*!< Evaluated skew between clk and write DQS for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_CFG_OFFSET                                 0x304 /*!< Trained write DQS delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_START_CFG_OFFSET                        0x308 /*!< Evaluated start point of read gate training for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_CFG_OFFSET                              0x30c /*!< Trained read gate delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_CFG_OFFSET                             0x320 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_CFG_OFFSET                             0x324 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_CFG_OFFSET                             0x328 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_CFG_OFFSET                             0x32c /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_CFG_OFFSET                             0x330 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_CFG_OFFSET                             0x334 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_CFG_OFFSET                             0x338 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_CFG_OFFSET                             0x33c /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_CFG_OFFSET                             0x340 /*!< Trained read DQ delay enc and read DQS delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_CFG_OFFSET                                 0x344 /*!< Trained write DQ delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_CFG_OFFSET                                 0x348 /*!< Trained write DQ delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_CFG_OFFSET                                 0x34c /*!< Trained write DQ delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_CFG_OFFSET                                 0x350 /*!< Trained write DQ delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_CFG_OFFSET                                 0x354 /*!< Trained write DQ delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_CFG_OFFSET                                 0x358 /*!< Trained write DQ delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_CFG_OFFSET                                 0x35c /*!< Trained write DQ delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_CFG_OFFSET                                 0x360 /*!< Trained write DQ delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_CFG_OFFSET                                 0x364 /*!< Trained write DQ delay for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_OFFSET                              0x368 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ_SW_CFG_OFFSET                               0x370 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_OFFSET                              0x374 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_OFFSET                              0x378 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_OFFSET                              0x37c /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_OFFSET                              0x380 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_OFFSET                              0x384 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_OFFSET                              0x388 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_OFFSET                              0x38c /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_OFFSET                              0x390 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_OFFSET                              0x394 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_SW_CFG_OFFSET                              0x398 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_SW_CFG_OFFSET                          0x39c /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_SW_CFG_OFFSET                          0x3a0 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_SW_CFG_OFFSET                          0x3a4 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_SW_CFG_OFFSET                          0x3a8 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_SW_CFG_OFFSET                          0x3ac /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_SW_CFG_OFFSET                          0x3b0 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_SW_CFG_OFFSET                          0x3b4 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_SW_CFG_OFFSET                          0x3b8 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_SW_CFG_OFFSET                          0x3bc /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_SW_CFG_OFFSET                           0x3c0 /*!< DBYTE software mode configuration for frequency 0, rank 1 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_INIT_CFG_OFFSET                            0x400 /*!< Evaluated skew between clk and write DQS for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_CFG_OFFSET                                 0x404 /*!< Trained write DQS delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_START_CFG_OFFSET                        0x408 /*!< Evaluated start point of read gate training for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_CFG_OFFSET                              0x40c /*!< Trained read gate delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_CFG_OFFSET                             0x420 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_CFG_OFFSET                             0x424 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_CFG_OFFSET                             0x428 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_CFG_OFFSET                             0x42c /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_CFG_OFFSET                             0x430 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_CFG_OFFSET                             0x434 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_CFG_OFFSET                             0x438 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_CFG_OFFSET                             0x43c /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_CFG_OFFSET                             0x440 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_CFG_OFFSET                                 0x444 /*!< Trained write DQ delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_CFG_OFFSET                                 0x448 /*!< Trained write DQ delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_CFG_OFFSET                                 0x44c /*!< Trained write DQ delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_CFG_OFFSET                                 0x450 /*!< Trained write DQ delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_CFG_OFFSET                                 0x454 /*!< Trained write DQ delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_CFG_OFFSET                                 0x458 /*!< Trained write DQ delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_CFG_OFFSET                                 0x45c /*!< Trained write DQ delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_CFG_OFFSET                                 0x460 /*!< Trained write DQ delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_CFG_OFFSET                                 0x464 /*!< Trained write DQ delay for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_OFFSET                              0x468 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ_SW_CFG_OFFSET                               0x470 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_OFFSET                              0x474 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_OFFSET                              0x478 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_OFFSET                              0x47c /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_OFFSET                              0x480 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_OFFSET                              0x484 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_OFFSET                              0x488 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_OFFSET                              0x48c /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_OFFSET                              0x490 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_OFFSET                              0x494 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_SW_CFG_OFFSET                              0x498 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_SW_CFG_OFFSET                          0x49c /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_SW_CFG_OFFSET                          0x4a0 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_SW_CFG_OFFSET                          0x4a4 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_SW_CFG_OFFSET                          0x4a8 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_SW_CFG_OFFSET                          0x4ac /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_SW_CFG_OFFSET                          0x4b0 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_SW_CFG_OFFSET                          0x4b4 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_SW_CFG_OFFSET                          0x4b8 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_SW_CFG_OFFSET                          0x4bc /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_SW_CFG_OFFSET                           0x4c0 /*!< DBYTE software mode configuration for frequency 1, rank 0 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_INIT_CFG_OFFSET                            0x500 /*!< Evaluated skew between clk and write DQS for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_CFG_OFFSET                                 0x504 /*!< Trained write DQS delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_START_CFG_OFFSET                        0x508 /*!< Evaluated start point of read gate training for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_CFG_OFFSET                              0x50c /*!< Trained read gate delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_CFG_OFFSET                             0x520 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_CFG_OFFSET                             0x524 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_CFG_OFFSET                             0x528 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_CFG_OFFSET                             0x52c /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_CFG_OFFSET                             0x530 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_CFG_OFFSET                             0x534 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_CFG_OFFSET                             0x538 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_CFG_OFFSET                             0x53c /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_CFG_OFFSET                             0x540 /*!< Trained read DQ delay enc and read DQS delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_CFG_OFFSET                                 0x544 /*!< Trained write DQ delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_CFG_OFFSET                                 0x548 /*!< Trained write DQ delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_CFG_OFFSET                                 0x54c /*!< Trained write DQ delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_CFG_OFFSET                                 0x550 /*!< Trained write DQ delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_CFG_OFFSET                                 0x554 /*!< Trained write DQ delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_CFG_OFFSET                                 0x558 /*!< Trained write DQ delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_CFG_OFFSET                                 0x55c /*!< Trained write DQ delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_CFG_OFFSET                                 0x560 /*!< Trained write DQ delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_CFG_OFFSET                                 0x564 /*!< Trained write DQ delay for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_OFFSET                              0x568 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ_SW_CFG_OFFSET                               0x570 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_OFFSET                              0x574 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_OFFSET                              0x578 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_OFFSET                              0x57c /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_OFFSET                              0x580 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_OFFSET                              0x584 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_OFFSET                              0x588 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_OFFSET                              0x58c /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_OFFSET                              0x590 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_OFFSET                              0x594 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_SW_CFG_OFFSET                              0x598 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_SW_CFG_OFFSET                          0x59c /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_SW_CFG_OFFSET                          0x5a0 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_SW_CFG_OFFSET                          0x5a4 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_SW_CFG_OFFSET                          0x5a8 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_SW_CFG_OFFSET                          0x5ac /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_SW_CFG_OFFSET                          0x5b0 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_SW_CFG_OFFSET                          0x5b4 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_SW_CFG_OFFSET                          0x5b8 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_SW_CFG_OFFSET                          0x5bc /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_SW_CFG_OFFSET                           0x5c0 /*!< DBYTE software mode configuration for frequency 1, rank 1 */
#define DBYTE_BOOT_DB_BOOT_TMG_OFFSET                                        0x1200 /*!< DBYTE timing configuration for LPDDR4 boot frequency */
#define DBYTE_BOOT_DB_BOOT_TMG1_OFFSET                                       0x1204 /*!< DBYTE timing configuration for LPDDR4 boot frequency */
#define DBYTE_BOOT_DB_BOOT_WRDQS_CFG_OFFSET                                  0x1210 /*!< DBYTE write DQS delay for LPDDR4 boot frequency */
#define DBYTE_BOOT_DB_BOOT_RDDQS_GT_CFG_OFFSET                               0x1214 /*!< DBYTE read gate delay for LPDDR4 boot frequency */
#define DBYTE_BOOT_DB_BOOT_RDDQ_CFG_OFFSET                                   0x1218 /*!< DBYTE read DQ and read DQS delay for LPDDR4 boot frequency */
#define DBYTE_BOOT_DB_BOOT_WRDQ_CFG_OFFSET                                   0x121c /*!< DBYTE write DQ delay for LPDDR4 boot frequency */

 /* ===== DBYTE AF DB_GNRL_CFG Register definition ===== */
#define DBYTE_AF_DB_GNRL_CFG_BIT_MASK_MASK                BITS(0,8)                                   /*!< DBYTE AF DB GNRL CFG: BIT_MASK Bit Mask */  
#define DBYTE_AF_DB_GNRL_CFG_BIT_MASK_OFS                 0U                                          /*!< DBYTE AF DB GNRL CFG: BIT_MASK Bit Offset */
#define DBYTE_AF_DB_GNRL_CFG_BIT_MASK(regval)             (BITS(0,8) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB GNRL CFG: BIT_MASK Bit Value */  
#define DBYTE_AF_DB_GNRL_CFG_DDR3                         BIT(12)                                      /*!< Selects DDR3 SDRAM 1 - DDR3 SDRAM device in use. 0 - non-DDR3 SDRAM device in use. */
#define DBYTE_AF_DB_GNRL_CFG_DDR3_OFS                     12U                                          /*!< DBYTE AF DB GNRL CFG: DDR3 Bit Offset */
#define DBYTE_AF_DB_GNRL_CFG_DDR3_VAL(regval)                 (BIT(12) & ((uint32_t)(regval) << 12))        /*!< DBYTE AF DB GNRL CFG: DDR3 Bit Value */  
#define DBYTE_AF_DB_GNRL_CFG_DDR4                         BIT(13)                                      /*!< Selects DDR4 SDRAM 1 - DDR4 SDRAM device in use. 0 - non-DDR4 SDRAM device in use. */
#define DBYTE_AF_DB_GNRL_CFG_DDR4_OFS                     13U                                          /*!< DBYTE AF DB GNRL CFG: DDR4 Bit Offset */
#define DBYTE_AF_DB_GNRL_CFG_DDR4_VAL(regval)                 (BIT(13) & ((uint32_t)(regval) << 13))        /*!< DBYTE AF DB GNRL CFG: DDR4 Bit Value */  
#define DBYTE_AF_DB_GNRL_CFG_LP3                          BIT(14)                                      /*!< Selects LPDDR3 SDRAM 1 - LPDDR3 SDRAM device in use. 0 - non-LPDDR3 SDRAM device in use. */
#define DBYTE_AF_DB_GNRL_CFG_LP3_OFS                      14U                                          /*!< DBYTE AF DB GNRL CFG: LP3 Bit Offset */
#define DBYTE_AF_DB_GNRL_CFG_LP3_VAL(regval)                  (BIT(14) & ((uint32_t)(regval) << 14))        /*!< DBYTE AF DB GNRL CFG: LP3 Bit Value */  
#define DBYTE_AF_DB_GNRL_CFG_LP4                          BIT(15)                                      /*!< Selects LPDDR4 SDRAM 1 - LPDDR4 SDRAM device in use. 0 - non-LPDDR4 SDRAM device in use. */
#define DBYTE_AF_DB_GNRL_CFG_LP4_OFS                      15U                                          /*!< DBYTE AF DB GNRL CFG: LP4 Bit Offset */
#define DBYTE_AF_DB_GNRL_CFG_LP4_VAL(regval)                  (BIT(15) & ((uint32_t)(regval) << 15))        /*!< DBYTE AF DB GNRL CFG: LP4 Bit Value */  
#define DBYTE_AF_DB_GNRL_CFG_CS_MAP_MASK                  BITS(16,17)                                   /*!< DBYTE AF DB GNRL CFG: CS_MAP Bit Mask */  
#define DBYTE_AF_DB_GNRL_CFG_CS_MAP_OFS                   16U                                          /*!< DBYTE AF DB GNRL CFG: CS_MAP Bit Offset */
#define DBYTE_AF_DB_GNRL_CFG_CS_MAP(regval)               (BITS(16,17) & ((uint32_t)(regval) << 16))        /*!< DBYTE AF DB GNRL CFG: CS_MAP Bit Value */  
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_WRDQS               BIT(24)                                      /*!< 1 - Different rank use different wrdqs dly registers 0 - All ranks use the same wrdqs dly registers of rank 0 */
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_WRDQS_OFS           24U                                          /*!< DBYTE AF DB GNRL CFG: PER_RANK_WRDQS Bit Offset */
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_WRDQS_VAL(regval)       (BIT(24) & ((uint32_t)(regval) << 24))        /*!< DBYTE AF DB GNRL CFG: PER_RANK_WRDQS Bit Value */  
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_WRDQ                BIT(25)                                      /*!< 1 - Different rank use different wrdq dly registers 0 - All ranks use the same wrdq dly registers of rank 0 */
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_WRDQ_OFS            25U                                          /*!< DBYTE AF DB GNRL CFG: PER_RANK_WRDQ Bit Offset */
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_WRDQ_VAL(regval)        (BIT(25) & ((uint32_t)(regval) << 25))        /*!< DBYTE AF DB GNRL CFG: PER_RANK_WRDQ Bit Value */  
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_RDDQS_GT            BIT(26)                                      /*!< 1 - Different rank use different rddqs_gt dly registers 0 - All ranks use the same rddqs_gt registers of rank 0 */
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_RDDQS_GT_OFS        26U                                          /*!< DBYTE AF DB GNRL CFG: PER_RANK_RDDQS_GT Bit Offset */
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_RDDQS_GT_VAL(regval)    (BIT(26) & ((uint32_t)(regval) << 26))        /*!< DBYTE AF DB GNRL CFG: PER_RANK_RDDQS_GT Bit Value */  
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_RDDQS               BIT(27)                                      /*!< 1 - Different rank use different rddqs registers 0 - All ranks use the same rddqs registers of rank 0 */
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_RDDQS_OFS           27U                                          /*!< DBYTE AF DB GNRL CFG: PER_RANK_RDDQS Bit Offset */
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_RDDQS_VAL(regval)       (BIT(27) & ((uint32_t)(regval) << 27))        /*!< DBYTE AF DB GNRL CFG: PER_RANK_RDDQS Bit Value */  
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_RDDQ                BIT(28)                                      /*!< 1 - Different rank use different rddq registers 0 - All ranks use the same rddq registers of rank 0 */
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_RDDQ_OFS            28U                                          /*!< DBYTE AF DB GNRL CFG: PER_RANK_RDDQ Bit Offset */
#define DBYTE_AF_DB_GNRL_CFG_PER_RANK_RDDQ_VAL(regval)        (BIT(28) & ((uint32_t)(regval) << 28))        /*!< DBYTE AF DB GNRL CFG: PER_RANK_RDDQ Bit Value */  
 
 /* ===== DBYTE AF DB_LPBK_CFG Register definition ===== */
#define DBYTE_AF_DB_LPBK_CFG_EN                           BIT(0)                                      /*!< 1 - Enable loop back mode 0 - Disable loop back mode */
#define DBYTE_AF_DB_LPBK_CFG_EN_OFS                       0U                                          /*!< DBYTE AF DB LPBK CFG: EN Bit Offset */
#define DBYTE_AF_DB_LPBK_CFG_EN_VAL(regval)                   (BIT(0) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB LPBK CFG: EN Bit Value */  
#define DBYTE_AF_DB_LPBK_CFG_GO                           BIT(1)                                      /*!< 1 - Start loop back test 0 - Stop loop back test */
#define DBYTE_AF_DB_LPBK_CFG_GO_OFS                       1U                                          /*!< DBYTE AF DB LPBK CFG: GO Bit Offset */
#define DBYTE_AF_DB_LPBK_CFG_GO_VAL(regval)                   (BIT(1) & ((uint32_t)(regval) << 1))        /*!< DBYTE AF DB LPBK CFG: GO Bit Value */  
#define DBYTE_AF_DB_LPBK_CFG_PAT                          BIT(2)                                      /*!< 1 - Use PRBS pattern 0 - Use clock pattern(010101...) */
#define DBYTE_AF_DB_LPBK_CFG_PAT_OFS                      2U                                          /*!< DBYTE AF DB LPBK CFG: PAT Bit Offset */
#define DBYTE_AF_DB_LPBK_CFG_PAT_VAL(regval)                  (BIT(2) & ((uint32_t)(regval) << 2))        /*!< DBYTE AF DB LPBK CFG: PAT Bit Value */  
#define DBYTE_AF_DB_LPBK_CFG_EXT                          BIT(3)                                      /*!< 1 - External loop back 0 - Internal loop back */
#define DBYTE_AF_DB_LPBK_CFG_EXT_OFS                      3U                                          /*!< DBYTE AF DB LPBK CFG: EXT Bit Offset */
#define DBYTE_AF_DB_LPBK_CFG_EXT_VAL(regval)                  (BIT(3) & ((uint32_t)(regval) << 3))        /*!< DBYTE AF DB LPBK CFG: EXT Bit Value */  
#define DBYTE_AF_DB_LPBK_CFG_SAMP_CNT_MASK                BITS(4,5)                                   /*!< DBYTE AF DB LPBK CFG: SAMP_CNT Bit Mask */  
#define DBYTE_AF_DB_LPBK_CFG_SAMP_CNT_OFS                 4U                                          /*!< DBYTE AF DB LPBK CFG: SAMP_CNT Bit Offset */
#define DBYTE_AF_DB_LPBK_CFG_SAMP_CNT(regval)             (BITS(4,5) & ((uint32_t)(regval) << 4))        /*!< DBYTE AF DB LPBK CFG: SAMP_CNT Bit Value */  
#define DBYTE_AF_DB_LPBK_CFG_CLR                          BIT(8)                                      /*!< Setting this field to 1 causes the loop back status to be cleared. Note: The hardware automatically clears this bit at the next clock cycle. */
#define DBYTE_AF_DB_LPBK_CFG_CLR_OFS                      8U                                          /*!< DBYTE AF DB LPBK CFG: CLR Bit Offset */
#define DBYTE_AF_DB_LPBK_CFG_CLR_VAL(regval)                  (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB LPBK CFG: CLR Bit Value */  
 
 /* ===== DBYTE AF DB_LPBK_STATUS Register definition ===== */
#define DBYTE_AF_DB_LPBK_STATUS_DONE                         BIT(0)                                      /*!< Loopback done status */
#define DBYTE_AF_DB_LPBK_STATUS_ERR                          BIT(1)                                      /*!< Loopback error status */
#define DBYTE_AF_DB_LPBK_STATUS_ERR_CNT                      BITS(4,7)                
#define DBYTE_AF_DB_LPBK_STATUS_ERR_IDX_1ST                  BITS(16,25)                
 
 /* ===== DBYTE AF DB_MSTR_CFG Register definition ===== */
#define DBYTE_AF_DB_MSTR_CFG_CAPT_CNT_MASK                BITS(0,2)                                   /*!< DBYTE AF DB MSTR CFG: CAPT_CNT Bit Mask */  
#define DBYTE_AF_DB_MSTR_CFG_CAPT_CNT_OFS                 0U                                          /*!< DBYTE AF DB MSTR CFG: CAPT_CNT Bit Offset */
#define DBYTE_AF_DB_MSTR_CFG_CAPT_CNT(regval)             (BITS(0,2) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB MSTR CFG: CAPT_CNT Bit Value */  
#define DBYTE_AF_DB_MSTR_CFG_WAIT_CYC_MASK                BITS(4,8)                                   /*!< DBYTE AF DB MSTR CFG: WAIT_CYC Bit Mask */  
#define DBYTE_AF_DB_MSTR_CFG_WAIT_CYC_OFS                 4U                                          /*!< DBYTE AF DB MSTR CFG: WAIT_CYC Bit Offset */
#define DBYTE_AF_DB_MSTR_CFG_WAIT_CYC(regval)             (BITS(4,8) & ((uint32_t)(regval) << 4))        /*!< DBYTE AF DB MSTR CFG: WAIT_CYC Bit Value */  
#define DBYTE_AF_DB_MSTR_CFG_START_DLY_ENC_MASK           BITS(12,21)                                   /*!< DBYTE AF DB MSTR CFG: START_DLY_ENC Bit Mask */  
#define DBYTE_AF_DB_MSTR_CFG_START_DLY_ENC_OFS            12U                                          /*!< DBYTE AF DB MSTR CFG: START_DLY_ENC Bit Offset */
#define DBYTE_AF_DB_MSTR_CFG_START_DLY_ENC(regval)        (BITS(12,21) & ((uint32_t)(regval) << 12))        /*!< DBYTE AF DB MSTR CFG: START_DLY_ENC Bit Value */  
#define DBYTE_AF_DB_MSTR_CFG_STEP_MASK                    BITS(24,27)                                   /*!< DBYTE AF DB MSTR CFG: STEP Bit Mask */  
#define DBYTE_AF_DB_MSTR_CFG_STEP_OFS                     24U                                          /*!< DBYTE AF DB MSTR CFG: STEP Bit Offset */
#define DBYTE_AF_DB_MSTR_CFG_STEP(regval)                 (BITS(24,27) & ((uint32_t)(regval) << 24))        /*!< DBYTE AF DB MSTR CFG: STEP Bit Value */  
 
 /* ===== DBYTE AF DB_SW_CTRL_CFG Register definition ===== */
#define DBYTE_AF_DB_SW_CTRL_CFG_MSTR_SW_BYP_MODE             BIT(0)                                      /*!< 1 - Enable bypass mode for master delay line 0 - Disable bypass mode for master delay line Note: This field should be stable when PHY is in mission mode. */
#define DBYTE_AF_DB_SW_CTRL_CFG_MSTR_SW_BYP_MODE_OFS         0U                                          /*!< DBYTE AF DB SW CTRL CFG: MSTR_SW_BYP_MODE Bit Offset */
#define DBYTE_AF_DB_SW_CTRL_CFG_MSTR_SW_BYP_MODE_VAL(regval)     (BIT(0) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB SW CTRL CFG: MSTR_SW_BYP_MODE Bit Value */  
#define DBYTE_AF_DB_SW_CTRL_CFG_MSTR_SW_HALF_CLK_MODE         BIT(1)                                      /*!< 1 - Enable half clock mode for master delay line 0 - Disable half clock mode for master delay line Note: This field should be stable when PHY is in mission mode. */
#define DBYTE_AF_DB_SW_CTRL_CFG_MSTR_SW_HALF_CLK_MODE_OFS     1U                                          /*!< DBYTE AF DB SW CTRL CFG: MSTR_SW_HALF_CLK_MODE Bit Offset */
#define DBYTE_AF_DB_SW_CTRL_CFG_MSTR_SW_HALF_CLK_MODE_VAL(regval) (BIT(1) & ((uint32_t)(regval) << 1))        /*!< DBYTE AF DB SW CTRL CFG: MSTR_SW_HALF_CLK_MODE Bit Value */  
#define DBYTE_AF_DB_SW_CTRL_CFG_SLV_SW_MODE                  BIT(2)                                      /*!< 1 - Enable software mode for slave delay 0 - Disable software mode for slave delay Note: In software mode, the delay encode values are driven by software registers directly. In hardware mode, the delay encode values are calculated by hardware. */
#define DBYTE_AF_DB_SW_CTRL_CFG_SLV_SW_MODE_OFS              2U                                          /*!< DBYTE AF DB SW CTRL CFG: SLV_SW_MODE Bit Offset */
#define DBYTE_AF_DB_SW_CTRL_CFG_SLV_SW_MODE_VAL(regval)          (BIT(2) & ((uint32_t)(regval) << 2))        /*!< DBYTE AF DB SW CTRL CFG: SLV_SW_MODE Bit Value */  
#define DBYTE_AF_DB_SW_CTRL_CFG_FIFO_SW_PTR_RESET            BIT(3)                                      /*!< Setting this field to 1 causes the read fifo pointer to be cleared. Note: The hardware automatically clears this bit at the next clock cycle. */
#define DBYTE_AF_DB_SW_CTRL_CFG_FIFO_SW_PTR_RESET_OFS        3U                                          /*!< DBYTE AF DB SW CTRL CFG: FIFO_SW_PTR_RESET Bit Offset */
#define DBYTE_AF_DB_SW_CTRL_CFG_FIFO_SW_PTR_RESET_VAL(regval)    (BIT(3) & ((uint32_t)(regval) << 3))        /*!< DBYTE AF DB SW CTRL CFG: FIFO_SW_PTR_RESET Bit Value */  
 
 /* ===== DBYTE AF DB_SWAP_CFG0 Register definition ===== */
#define DBYTE_AF_DB_SWAP_CFG0_DQ0_SWAP_MASK                BITS(0,3)                                   /*!< DBYTE AF DB SWAP CFG0: DQ0_SWAP Bit Mask */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ0_SWAP_OFS                 0U                                          /*!< DBYTE AF DB SWAP CFG0: DQ0_SWAP Bit Offset */
#define DBYTE_AF_DB_SWAP_CFG0_DQ0_SWAP(regval)             (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB SWAP CFG0: DQ0_SWAP Bit Value */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ1_SWAP_MASK                BITS(4,7)                                   /*!< DBYTE AF DB SWAP CFG0: DQ1_SWAP Bit Mask */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ1_SWAP_OFS                 4U                                          /*!< DBYTE AF DB SWAP CFG0: DQ1_SWAP Bit Offset */
#define DBYTE_AF_DB_SWAP_CFG0_DQ1_SWAP(regval)             (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< DBYTE AF DB SWAP CFG0: DQ1_SWAP Bit Value */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ2_SWAP_MASK                BITS(8,11)                                   /*!< DBYTE AF DB SWAP CFG0: DQ2_SWAP Bit Mask */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ2_SWAP_OFS                 8U                                          /*!< DBYTE AF DB SWAP CFG0: DQ2_SWAP Bit Offset */
#define DBYTE_AF_DB_SWAP_CFG0_DQ2_SWAP(regval)             (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB SWAP CFG0: DQ2_SWAP Bit Value */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ3_SWAP_MASK                BITS(12,15)                                   /*!< DBYTE AF DB SWAP CFG0: DQ3_SWAP Bit Mask */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ3_SWAP_OFS                 12U                                          /*!< DBYTE AF DB SWAP CFG0: DQ3_SWAP Bit Offset */
#define DBYTE_AF_DB_SWAP_CFG0_DQ3_SWAP(regval)             (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< DBYTE AF DB SWAP CFG0: DQ3_SWAP Bit Value */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ4_SWAP_MASK                BITS(16,19)                                   /*!< DBYTE AF DB SWAP CFG0: DQ4_SWAP Bit Mask */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ4_SWAP_OFS                 16U                                          /*!< DBYTE AF DB SWAP CFG0: DQ4_SWAP Bit Offset */
#define DBYTE_AF_DB_SWAP_CFG0_DQ4_SWAP(regval)             (BITS(16,19) & ((uint32_t)(regval) << 16))        /*!< DBYTE AF DB SWAP CFG0: DQ4_SWAP Bit Value */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ5_SWAP_MASK                BITS(20,23)                                   /*!< DBYTE AF DB SWAP CFG0: DQ5_SWAP Bit Mask */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ5_SWAP_OFS                 20U                                          /*!< DBYTE AF DB SWAP CFG0: DQ5_SWAP Bit Offset */
#define DBYTE_AF_DB_SWAP_CFG0_DQ5_SWAP(regval)             (BITS(20,23) & ((uint32_t)(regval) << 20))        /*!< DBYTE AF DB SWAP CFG0: DQ5_SWAP Bit Value */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ6_SWAP_MASK                BITS(24,27)                                   /*!< DBYTE AF DB SWAP CFG0: DQ6_SWAP Bit Mask */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ6_SWAP_OFS                 24U                                          /*!< DBYTE AF DB SWAP CFG0: DQ6_SWAP Bit Offset */
#define DBYTE_AF_DB_SWAP_CFG0_DQ6_SWAP(regval)             (BITS(24,27) & ((uint32_t)(regval) << 24))        /*!< DBYTE AF DB SWAP CFG0: DQ6_SWAP Bit Value */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ7_SWAP_MASK                BITS(28,31)                                   /*!< DBYTE AF DB SWAP CFG0: DQ7_SWAP Bit Mask */  
#define DBYTE_AF_DB_SWAP_CFG0_DQ7_SWAP_OFS                 28U                                          /*!< DBYTE AF DB SWAP CFG0: DQ7_SWAP Bit Offset */
#define DBYTE_AF_DB_SWAP_CFG0_DQ7_SWAP(regval)             (BITS(28,31) & ((uint32_t)(regval) << 28))        /*!< DBYTE AF DB SWAP CFG0: DQ7_SWAP Bit Value */  
 
 /* ===== DBYTE AF DB_SWAP_CFG1 Register definition ===== */
#define DBYTE_AF_DB_SWAP_CFG1_DM_DBI_SWAP_MASK             BITS(0,3)                                   /*!< DBYTE AF DB SWAP CFG1: DM_DBI_SWAP Bit Mask */  
#define DBYTE_AF_DB_SWAP_CFG1_DM_DBI_SWAP_OFS              0U                                          /*!< DBYTE AF DB SWAP CFG1: DM_DBI_SWAP Bit Offset */
#define DBYTE_AF_DB_SWAP_CFG1_DM_DBI_SWAP(regval)          (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB SWAP CFG1: DM_DBI_SWAP Bit Value */  
 
 /* ===== DBYTE AF DB_LVL_COM_CFG Register definition ===== */
#define DBYTE_AF_DB_LVL_COM_CFG_UPDT_WAIT_CYC_MASK           BITS(8,10)                                   /*!< DBYTE AF DB LVL COM CFG: UPDT_WAIT_CYC Bit Mask */  
#define DBYTE_AF_DB_LVL_COM_CFG_UPDT_WAIT_CYC_OFS            8U                                          /*!< DBYTE AF DB LVL COM CFG: UPDT_WAIT_CYC Bit Offset */
#define DBYTE_AF_DB_LVL_COM_CFG_UPDT_WAIT_CYC(regval)        (BITS(8,10) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB LVL COM CFG: UPDT_WAIT_CYC Bit Value */  
#define DBYTE_AF_DB_LVL_COM_CFG_DBG_MODE                     BIT(28)                                      /*!< 1 - Run training in debug mode 0 - Run training in normal mode */
#define DBYTE_AF_DB_LVL_COM_CFG_DBG_MODE_OFS                 28U                                          /*!< DBYTE AF DB LVL COM CFG: DBG_MODE Bit Offset */
#define DBYTE_AF_DB_LVL_COM_CFG_DBG_MODE_VAL(regval)             (BIT(28) & ((uint32_t)(regval) << 28))        /*!< DBYTE AF DB LVL COM CFG: DBG_MODE Bit Value */  
#define DBYTE_AF_DB_LVL_COM_CFG_DBG_CONT                     BIT(29)                                      /*!< 1 - Continue to run training 0 - Pause FSM at analyze state Note: The hardware automatically clears this bit at the next clock cycle. */
#define DBYTE_AF_DB_LVL_COM_CFG_DBG_CONT_OFS                 29U                                          /*!< DBYTE AF DB LVL COM CFG: DBG_CONT Bit Offset */
#define DBYTE_AF_DB_LVL_COM_CFG_DBG_CONT_VAL(regval)             (BIT(29) & ((uint32_t)(regval) << 29))        /*!< DBYTE AF DB LVL COM CFG: DBG_CONT Bit Value */  
#define DBYTE_AF_DB_LVL_COM_CFG_ERR_CLR                      BIT(30)                                      /*!< Setting this field to 1 causes the training status to be cleared. Note: The hardware automatically clears this bit at the next clock cycle. */
#define DBYTE_AF_DB_LVL_COM_CFG_ERR_CLR_OFS                  30U                                          /*!< DBYTE AF DB LVL COM CFG: ERR_CLR Bit Offset */
#define DBYTE_AF_DB_LVL_COM_CFG_ERR_CLR_VAL(regval)              (BIT(30) & ((uint32_t)(regval) << 30))        /*!< DBYTE AF DB LVL COM CFG: ERR_CLR Bit Value */  
 
 /* ===== DBYTE AF DB_WRLVL_CFG Register definition ===== */
#define DBYTE_AF_DB_WRLVL_CFG_CAPT_CNT_MASK                BITS(0,3)                                   /*!< DBYTE AF DB WRLVL CFG: CAPT_CNT Bit Mask */  
#define DBYTE_AF_DB_WRLVL_CFG_CAPT_CNT_OFS                 0U                                          /*!< DBYTE AF DB WRLVL CFG: CAPT_CNT Bit Offset */
#define DBYTE_AF_DB_WRLVL_CFG_CAPT_CNT(regval)             (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB WRLVL CFG: CAPT_CNT Bit Value */  
#define DBYTE_AF_DB_WRLVL_CFG_DLY_STEP_MASK                BITS(8,11)                                   /*!< DBYTE AF DB WRLVL CFG: DLY_STEP Bit Mask */  
#define DBYTE_AF_DB_WRLVL_CFG_DLY_STEP_OFS                 8U                                          /*!< DBYTE AF DB WRLVL CFG: DLY_STEP Bit Offset */
#define DBYTE_AF_DB_WRLVL_CFG_DLY_STEP(regval)             (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB WRLVL CFG: DLY_STEP Bit Value */  
#define DBYTE_AF_DB_WRLVL_CFG_ALL_DQ                       BIT(16)                                      /*!< If DRAM only drives one DQ for write leveling feedback, set this bit to 0. If DRAM drives all DQ for write leveling feedback, set this bit to 1. */
#define DBYTE_AF_DB_WRLVL_CFG_ALL_DQ_OFS                   16U                                          /*!< DBYTE AF DB WRLVL CFG: ALL_DQ Bit Offset */
#define DBYTE_AF_DB_WRLVL_CFG_ALL_DQ_VAL(regval)               (BIT(16) & ((uint32_t)(regval) << 16))        /*!< DBYTE AF DB WRLVL CFG: ALL_DQ Bit Value */  
 
 /* ===== DBYTE AF DB_GTLVL_CFG Register definition ===== */
#define DBYTE_AF_DB_GTLVL_CFG_RESP_WAIT_CYC_MASK           BITS(0,3)                                   /*!< DBYTE AF DB GTLVL CFG: RESP_WAIT_CYC Bit Mask */  
#define DBYTE_AF_DB_GTLVL_CFG_RESP_WAIT_CYC_OFS            0U                                          /*!< DBYTE AF DB GTLVL CFG: RESP_WAIT_CYC Bit Offset */
#define DBYTE_AF_DB_GTLVL_CFG_RESP_WAIT_CYC(regval)        (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB GTLVL CFG: RESP_WAIT_CYC Bit Value */  
#define DBYTE_AF_DB_GTLVL_CFG_CAPT_CNT_MASK                BITS(8,11)                                   /*!< DBYTE AF DB GTLVL CFG: CAPT_CNT Bit Mask */  
#define DBYTE_AF_DB_GTLVL_CFG_CAPT_CNT_OFS                 8U                                          /*!< DBYTE AF DB GTLVL CFG: CAPT_CNT Bit Offset */
#define DBYTE_AF_DB_GTLVL_CFG_CAPT_CNT(regval)             (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB GTLVL CFG: CAPT_CNT Bit Value */  
#define DBYTE_AF_DB_GTLVL_CFG_DLY_STEP_MASK                BITS(12,15)                                   /*!< DBYTE AF DB GTLVL CFG: DLY_STEP Bit Mask */  
#define DBYTE_AF_DB_GTLVL_CFG_DLY_STEP_OFS                 12U                                          /*!< DBYTE AF DB GTLVL CFG: DLY_STEP Bit Offset */
#define DBYTE_AF_DB_GTLVL_CFG_DLY_STEP(regval)             (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< DBYTE AF DB GTLVL CFG: DLY_STEP Bit Value */  
#define DBYTE_AF_DB_GTLVL_CFG_BACK_STEP_MASK               BITS(16,24)                                   /*!< DBYTE AF DB GTLVL CFG: BACK_STEP Bit Mask */  
#define DBYTE_AF_DB_GTLVL_CFG_BACK_STEP_OFS                16U                                          /*!< DBYTE AF DB GTLVL CFG: BACK_STEP Bit Offset */
#define DBYTE_AF_DB_GTLVL_CFG_BACK_STEP(regval)            (BITS(16,24) & ((uint32_t)(regval) << 16))        /*!< DBYTE AF DB GTLVL CFG: BACK_STEP Bit Value */  
 
 /* ===== DBYTE AF DB_RDLVL_CFG0 Register definition ===== */
#define DBYTE_AF_DB_RDLVL_CFG0_CAPT_CNT_MASK                BITS(0,3)                                   /*!< DBYTE AF DB RDLVL CFG0: CAPT_CNT Bit Mask */  
#define DBYTE_AF_DB_RDLVL_CFG0_CAPT_CNT_OFS                 0U                                          /*!< DBYTE AF DB RDLVL CFG0: CAPT_CNT Bit Offset */
#define DBYTE_AF_DB_RDLVL_CFG0_CAPT_CNT(regval)             (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB RDLVL CFG0: CAPT_CNT Bit Value */  
#define DBYTE_AF_DB_RDLVL_CFG0_DLY_STEP_MASK                BITS(4,7)                                   /*!< DBYTE AF DB RDLVL CFG0: DLY_STEP Bit Mask */  
#define DBYTE_AF_DB_RDLVL_CFG0_DLY_STEP_OFS                 4U                                          /*!< DBYTE AF DB RDLVL CFG0: DLY_STEP Bit Offset */
#define DBYTE_AF_DB_RDLVL_CFG0_DLY_STEP(regval)             (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< DBYTE AF DB RDLVL CFG0: DLY_STEP Bit Value */  
#define DBYTE_AF_DB_RDLVL_CFG0_RDLVL_BIT_MASK_MASK          BITS(8,16)                                   /*!< DBYTE AF DB RDLVL CFG0: RDLVL_BIT_MASK Bit Mask */  
#define DBYTE_AF_DB_RDLVL_CFG0_RDLVL_BIT_MASK_OFS           8U                                          /*!< DBYTE AF DB RDLVL CFG0: RDLVL_BIT_MASK Bit Offset */
#define DBYTE_AF_DB_RDLVL_CFG0_RDLVL_BIT_MASK(regval)       (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB RDLVL CFG0: RDLVL_BIT_MASK Bit Value */  
#define DBYTE_AF_DB_RDLVL_CFG0_DQ_DESKEW_EN                 BIT(24)                                      /*!< 1 - Train read dq delay line prior to training read dqs delay line. 0 - Do not train dq delay line. */
#define DBYTE_AF_DB_RDLVL_CFG0_DQ_DESKEW_EN_OFS             24U                                          /*!< DBYTE AF DB RDLVL CFG0: DQ_DESKEW_EN Bit Offset */
#define DBYTE_AF_DB_RDLVL_CFG0_DQ_DESKEW_EN_VAL(regval)         (BIT(24) & ((uint32_t)(regval) << 24))        /*!< DBYTE AF DB RDLVL CFG0: DQ_DESKEW_EN Bit Value */  
 
 /* ===== DBYTE AF DB_RDLVL_CFG1 Register definition ===== */
#define DBYTE_AF_DB_RDLVL_CFG1_PAT_INV_MASK                 BITS(8,15)                                   /*!< DBYTE AF DB RDLVL CFG1: PAT_INV Bit Mask */  
#define DBYTE_AF_DB_RDLVL_CFG1_PAT_INV_OFS                  8U                                          /*!< DBYTE AF DB RDLVL CFG1: PAT_INV Bit Offset */
#define DBYTE_AF_DB_RDLVL_CFG1_PAT_INV(regval)              (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB RDLVL CFG1: PAT_INV Bit Value */  
#define DBYTE_AF_DB_RDLVL_CFG1_PAT_MASK                     BITS(16,31)                                   /*!< DBYTE AF DB RDLVL CFG1: PAT Bit Mask */  
#define DBYTE_AF_DB_RDLVL_CFG1_PAT_OFS                      16U                                          /*!< DBYTE AF DB RDLVL CFG1: PAT Bit Offset */
#define DBYTE_AF_DB_RDLVL_CFG1_PAT(regval)                  (BITS(16,31) & ((uint32_t)(regval) << 16))        /*!< DBYTE AF DB RDLVL CFG1: PAT Bit Value */  
 
 /* ===== DBYTE AF DB_RDLVL_VREF_CFG Register definition ===== */
#define DBYTE_AF_DB_RDLVL_VREF_CFG_STEP_MASK                    BITS(0,4)                                   /*!< DBYTE AF DB RDLVL VREF CFG: STEP Bit Mask */  
#define DBYTE_AF_DB_RDLVL_VREF_CFG_STEP_OFS                     0U                                          /*!< DBYTE AF DB RDLVL VREF CFG: STEP Bit Offset */
#define DBYTE_AF_DB_RDLVL_VREF_CFG_STEP(regval)                 (BITS(0,4) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB RDLVL VREF CFG: STEP Bit Value */  
#define DBYTE_AF_DB_RDLVL_VREF_CFG_MIN_MASK                     BITS(8,15)                                   /*!< DBYTE AF DB RDLVL VREF CFG: MIN Bit Mask */  
#define DBYTE_AF_DB_RDLVL_VREF_CFG_MIN_OFS                      8U                                          /*!< DBYTE AF DB RDLVL VREF CFG: MIN Bit Offset */
#define DBYTE_AF_DB_RDLVL_VREF_CFG_MIN(regval)                  (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB RDLVL VREF CFG: MIN Bit Value */  
#define DBYTE_AF_DB_RDLVL_VREF_CFG_MAX_MASK                     BITS(16,23)                                   /*!< DBYTE AF DB RDLVL VREF CFG: MAX Bit Mask */  
#define DBYTE_AF_DB_RDLVL_VREF_CFG_MAX_OFS                      16U                                          /*!< DBYTE AF DB RDLVL VREF CFG: MAX Bit Offset */
#define DBYTE_AF_DB_RDLVL_VREF_CFG_MAX(regval)                  (BITS(16,23) & ((uint32_t)(regval) << 16))        /*!< DBYTE AF DB RDLVL VREF CFG: MAX Bit Value */  
 
 /* ===== DBYTE AF DB_WDQLVL_CFG0 Register definition ===== */
#define DBYTE_AF_DB_WDQLVL_CFG0_WRDQ_TRAIN_EN                BIT(0)                                      /*!< 1 - Enable write DQ training 0 - Disable write DQ training Note: If setting this filed to 0, wdqlvl is only used for training dram vref. */
#define DBYTE_AF_DB_WDQLVL_CFG0_WRDQ_TRAIN_EN_OFS            0U                                          /*!< DBYTE AF DB WDQLVL CFG0: WRDQ_TRAIN_EN Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG0_WRDQ_TRAIN_EN_VAL(regval)        (BIT(0) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB WDQLVL CFG0: WRDQ_TRAIN_EN Bit Value */  
#define DBYTE_AF_DB_WDQLVL_CFG0_WDQLVL_BIT_MASK_MASK         BITS(1,9)                                   /*!< DBYTE AF DB WDQLVL CFG0: WDQLVL_BIT_MASK Bit Mask */  
#define DBYTE_AF_DB_WDQLVL_CFG0_WDQLVL_BIT_MASK_OFS          1U                                          /*!< DBYTE AF DB WDQLVL CFG0: WDQLVL_BIT_MASK Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG0_WDQLVL_BIT_MASK(regval)      (BITS(1,9) & ((uint32_t)(regval) << 1))        /*!< DBYTE AF DB WDQLVL CFG0: WDQLVL_BIT_MASK Bit Value */  
#define DBYTE_AF_DB_WDQLVL_CFG0_CDLY_STEP_MASK               BITS(12,15)                                   /*!< DBYTE AF DB WDQLVL CFG0: CDLY_STEP Bit Mask */  
#define DBYTE_AF_DB_WDQLVL_CFG0_CDLY_STEP_OFS                12U                                          /*!< DBYTE AF DB WDQLVL CFG0: CDLY_STEP Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG0_CDLY_STEP(regval)            (BITS(12,15) & ((uint32_t)(regval) << 12))        /*!< DBYTE AF DB WDQLVL CFG0: CDLY_STEP Bit Value */  
#define DBYTE_AF_DB_WDQLVL_CFG0_FDLY_STEP_MASK               BITS(16,19)                                   /*!< DBYTE AF DB WDQLVL CFG0: FDLY_STEP Bit Mask */  
#define DBYTE_AF_DB_WDQLVL_CFG0_FDLY_STEP_OFS                16U                                          /*!< DBYTE AF DB WDQLVL CFG0: FDLY_STEP Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG0_FDLY_STEP(regval)            (BITS(16,19) & ((uint32_t)(regval) << 16))        /*!< DBYTE AF DB WDQLVL CFG0: FDLY_STEP Bit Value */  
#define DBYTE_AF_DB_WDQLVL_CFG0_WIN_SIZ_THR_MASK             BITS(20,27)                                   /*!< DBYTE AF DB WDQLVL CFG0: WIN_SIZ_THR Bit Mask */  
#define DBYTE_AF_DB_WDQLVL_CFG0_WIN_SIZ_THR_OFS              20U                                          /*!< DBYTE AF DB WDQLVL CFG0: WIN_SIZ_THR Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG0_WIN_SIZ_THR(regval)          (BITS(20,27) & ((uint32_t)(regval) << 20))        /*!< DBYTE AF DB WDQLVL CFG0: WIN_SIZ_THR Bit Value */  
 
 /* ===== DBYTE AF DB_WDQLVL_CFG1 Register definition ===== */
#define DBYTE_AF_DB_WDQLVL_CFG1_SLV_DLY_START_MASK           BITS(0,11)                                   /*!< DBYTE AF DB WDQLVL CFG1: SLV_DLY_START Bit Mask */  
#define DBYTE_AF_DB_WDQLVL_CFG1_SLV_DLY_START_OFS            0U                                          /*!< DBYTE AF DB WDQLVL CFG1: SLV_DLY_START Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG1_SLV_DLY_START(regval)        (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB WDQLVL CFG1: SLV_DLY_START Bit Value */  
#define DBYTE_AF_DB_WDQLVL_CFG1_LEFT_ABRT_THR_MASK           BITS(12,23)                                   /*!< DBYTE AF DB WDQLVL CFG1: LEFT_ABRT_THR Bit Mask */  
#define DBYTE_AF_DB_WDQLVL_CFG1_LEFT_ABRT_THR_OFS            12U                                          /*!< DBYTE AF DB WDQLVL CFG1: LEFT_ABRT_THR Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG1_LEFT_ABRT_THR(regval)        (BITS(12,23) & ((uint32_t)(regval) << 12))        /*!< DBYTE AF DB WDQLVL CFG1: LEFT_ABRT_THR Bit Value */  
#define DBYTE_AF_DB_WDQLVL_CFG1_TE_JUMP_STEP_MASK            BITS(24,31)                                   /*!< DBYTE AF DB WDQLVL CFG1: TE_JUMP_STEP Bit Mask */  
#define DBYTE_AF_DB_WDQLVL_CFG1_TE_JUMP_STEP_OFS             24U                                          /*!< DBYTE AF DB WDQLVL CFG1: TE_JUMP_STEP Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG1_TE_JUMP_STEP(regval)         (BITS(24,31) & ((uint32_t)(regval) << 24))        /*!< DBYTE AF DB WDQLVL CFG1: TE_JUMP_STEP Bit Value */  
 
 /* ===== DBYTE AF DB_WDQLVL_CFG2 Register definition ===== */
#define DBYTE_AF_DB_WDQLVL_CFG2_SLV_DLY_MIN_MASK             BITS(0,11)                                   /*!< DBYTE AF DB WDQLVL CFG2: SLV_DLY_MIN Bit Mask */  
#define DBYTE_AF_DB_WDQLVL_CFG2_SLV_DLY_MIN_OFS              0U                                          /*!< DBYTE AF DB WDQLVL CFG2: SLV_DLY_MIN Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG2_SLV_DLY_MIN(regval)          (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB WDQLVL CFG2: SLV_DLY_MIN Bit Value */  
#define DBYTE_AF_DB_WDQLVL_CFG2_SLV_DLY_MAX_MASK             BITS(12,23)                                   /*!< DBYTE AF DB WDQLVL CFG2: SLV_DLY_MAX Bit Mask */  
#define DBYTE_AF_DB_WDQLVL_CFG2_SLV_DLY_MAX_OFS              12U                                          /*!< DBYTE AF DB WDQLVL CFG2: SLV_DLY_MAX Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG2_SLV_DLY_MAX(regval)          (BITS(12,23) & ((uint32_t)(regval) << 12))        /*!< DBYTE AF DB WDQLVL CFG2: SLV_DLY_MAX Bit Value */  
#define DBYTE_AF_DB_WDQLVL_CFG2_TE_VLD_SIZE_MASK             BITS(24,31)                                   /*!< DBYTE AF DB WDQLVL CFG2: TE_VLD_SIZE Bit Mask */  
#define DBYTE_AF_DB_WDQLVL_CFG2_TE_VLD_SIZE_OFS              24U                                          /*!< DBYTE AF DB WDQLVL CFG2: TE_VLD_SIZE Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG2_TE_VLD_SIZE(regval)          (BITS(24,31) & ((uint32_t)(regval) << 24))        /*!< DBYTE AF DB WDQLVL CFG2: TE_VLD_SIZE Bit Value */  
 
 /* ===== DBYTE AF DB_WDQLVL_CFG3 Register definition ===== */
#define DBYTE_AF_DB_WDQLVL_CFG3_USR_PAT0_MASK                BITS(0,15)                                   /*!< DBYTE AF DB WDQLVL CFG3: USR_PAT0 Bit Mask */  
#define DBYTE_AF_DB_WDQLVL_CFG3_USR_PAT0_OFS                 0U                                          /*!< DBYTE AF DB WDQLVL CFG3: USR_PAT0 Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG3_USR_PAT0(regval)             (BITS(0,15) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB WDQLVL CFG3: USR_PAT0 Bit Value */  
#define DBYTE_AF_DB_WDQLVL_CFG3_USR_PAT1_MASK                BITS(16,31)                                   /*!< DBYTE AF DB WDQLVL CFG3: USR_PAT1 Bit Mask */  
#define DBYTE_AF_DB_WDQLVL_CFG3_USR_PAT1_OFS                 16U                                          /*!< DBYTE AF DB WDQLVL CFG3: USR_PAT1 Bit Offset */
#define DBYTE_AF_DB_WDQLVL_CFG3_USR_PAT1(regval)             (BITS(16,31) & ((uint32_t)(regval) << 16))        /*!< DBYTE AF DB WDQLVL CFG3: USR_PAT1 Bit Value */  
 
 /* ===== DBYTE AF DB_LVL_STATUS Register definition ===== */
#define DBYTE_AF_DB_LVL_STATUS_R0_WRLVL_ERR                 BIT(0)                                      /*!< Indicates the wrlvl status of rank 0. Note: For X4 device, this field indicates the status of low nibble. For other devices, this field indicates the status of DBYTE. */
#define DBYTE_AF_DB_LVL_STATUS_R0_GTLVL_ERR                 BIT(2)                                      /*!< Indicates the read gate training status of rank 0. For X4 device, this field indicates the status of low nibble. For other devices, this field indicates the status of DBYTE. */
#define DBYTE_AF_DB_LVL_STATUS_R0_RDLVL_ERR                 BITS(4,5)                
#define DBYTE_AF_DB_LVL_STATUS_R0_WDQLVL_ERR                BIT(6)                                      /*!< Indicates the wdqlvl status of rank 0. */
#define DBYTE_AF_DB_LVL_STATUS_R1_WRLVL_ERR                 BIT(8)                                      /*!< Indicates the wrlvl status of rank 1. Note: For X4 device, this field indicates the status of low nibble. For other devices, this field indicates the status of DBYTE. */
#define DBYTE_AF_DB_LVL_STATUS_R1_GTLVL_ERR                 BIT(10)                                      /*!< Indicates the read gate training status of rank 1. For X4 device, this field indicates the status of low nibble. For other devices, this field indicates the status of DBYTE. */
#define DBYTE_AF_DB_LVL_STATUS_R1_RDLVL_ERR                 BITS(12,13)                
#define DBYTE_AF_DB_LVL_STATUS_R1_WDQLVL_ERR                BIT(14)                                      /*!< Indicates the wdqlvl status of rank 1. */
 
 /* ===== DBYTE AF DB_RX_CAL_CFG Register definition ===== */
#define DBYTE_AF_DB_RX_CAL_CFG_EN                           BIT(0)                                      /*!< 1 - Enable rx offset calibration triggered by first DFI init or sw_trg. 0 - Disable rx offset calibration triggered by first DFI init or sw_trg. */
#define DBYTE_AF_DB_RX_CAL_CFG_EN_OFS                       0U                                          /*!< DBYTE AF DB RX CAL CFG: EN Bit Offset */
#define DBYTE_AF_DB_RX_CAL_CFG_EN_VAL(regval)                   (BIT(0) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB RX CAL CFG: EN Bit Value */  
#define DBYTE_AF_DB_RX_CAL_CFG_SW_TRG                       BIT(1)                                      /*!< Setting this field to 1 to trigger rx offset calibration. Note: The hardware automatically clears this bit at the next clock cycle. */
#define DBYTE_AF_DB_RX_CAL_CFG_SW_TRG_OFS                   1U                                          /*!< DBYTE AF DB RX CAL CFG: SW_TRG Bit Offset */
#define DBYTE_AF_DB_RX_CAL_CFG_SW_TRG_VAL(regval)               (BIT(1) & ((uint32_t)(regval) << 1))        /*!< DBYTE AF DB RX CAL CFG: SW_TRG Bit Value */  
#define DBYTE_AF_DB_RX_CAL_CFG_WAIT_CYC_MASK                BITS(8,19)                                   /*!< DBYTE AF DB RX CAL CFG: WAIT_CYC Bit Mask */  
#define DBYTE_AF_DB_RX_CAL_CFG_WAIT_CYC_OFS                 8U                                          /*!< DBYTE AF DB RX CAL CFG: WAIT_CYC Bit Offset */
#define DBYTE_AF_DB_RX_CAL_CFG_WAIT_CYC(regval)             (BITS(8,19) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB RX CAL CFG: WAIT_CYC Bit Value */  
 
 /* ===== DBYTE AF DB_DQ0_RX_CAL_CODE Register definition ===== */
#define DBYTE_AF_DB_DQ0_RX_CAL_CODE_UP_MASK                      BITS(0,5)                                   /*!< DBYTE AF DB DQ0 RX CAL CODE: UP Bit Mask */  
#define DBYTE_AF_DB_DQ0_RX_CAL_CODE_UP_OFS                       0U                                          /*!< DBYTE AF DB DQ0 RX CAL CODE: UP Bit Offset */
#define DBYTE_AF_DB_DQ0_RX_CAL_CODE_UP(regval)                   (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB DQ0 RX CAL CODE: UP Bit Value */  
#define DBYTE_AF_DB_DQ0_RX_CAL_CODE_DOWN_MASK                    BITS(8,13)                                   /*!< DBYTE AF DB DQ0 RX CAL CODE: DOWN Bit Mask */  
#define DBYTE_AF_DB_DQ0_RX_CAL_CODE_DOWN_OFS                     8U                                          /*!< DBYTE AF DB DQ0 RX CAL CODE: DOWN Bit Offset */
#define DBYTE_AF_DB_DQ0_RX_CAL_CODE_DOWN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB DQ0 RX CAL CODE: DOWN Bit Value */  
 
 /* ===== DBYTE AF DB_DQ1_RX_CAL_CODE Register definition ===== */
#define DBYTE_AF_DB_DQ1_RX_CAL_CODE_UP_MASK                      BITS(0,5)                                   /*!< DBYTE AF DB DQ1 RX CAL CODE: UP Bit Mask */  
#define DBYTE_AF_DB_DQ1_RX_CAL_CODE_UP_OFS                       0U                                          /*!< DBYTE AF DB DQ1 RX CAL CODE: UP Bit Offset */
#define DBYTE_AF_DB_DQ1_RX_CAL_CODE_UP(regval)                   (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB DQ1 RX CAL CODE: UP Bit Value */  
#define DBYTE_AF_DB_DQ1_RX_CAL_CODE_DOWN_MASK                    BITS(8,13)                                   /*!< DBYTE AF DB DQ1 RX CAL CODE: DOWN Bit Mask */  
#define DBYTE_AF_DB_DQ1_RX_CAL_CODE_DOWN_OFS                     8U                                          /*!< DBYTE AF DB DQ1 RX CAL CODE: DOWN Bit Offset */
#define DBYTE_AF_DB_DQ1_RX_CAL_CODE_DOWN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB DQ1 RX CAL CODE: DOWN Bit Value */  
 
 /* ===== DBYTE AF DB_DQ2_RX_CAL_CODE Register definition ===== */
#define DBYTE_AF_DB_DQ2_RX_CAL_CODE_UP_MASK                      BITS(0,5)                                   /*!< DBYTE AF DB DQ2 RX CAL CODE: UP Bit Mask */  
#define DBYTE_AF_DB_DQ2_RX_CAL_CODE_UP_OFS                       0U                                          /*!< DBYTE AF DB DQ2 RX CAL CODE: UP Bit Offset */
#define DBYTE_AF_DB_DQ2_RX_CAL_CODE_UP(regval)                   (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB DQ2 RX CAL CODE: UP Bit Value */  
#define DBYTE_AF_DB_DQ2_RX_CAL_CODE_DOWN_MASK                    BITS(8,13)                                   /*!< DBYTE AF DB DQ2 RX CAL CODE: DOWN Bit Mask */  
#define DBYTE_AF_DB_DQ2_RX_CAL_CODE_DOWN_OFS                     8U                                          /*!< DBYTE AF DB DQ2 RX CAL CODE: DOWN Bit Offset */
#define DBYTE_AF_DB_DQ2_RX_CAL_CODE_DOWN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB DQ2 RX CAL CODE: DOWN Bit Value */  
 
 /* ===== DBYTE AF DB_DQ3_RX_CAL_CODE Register definition ===== */
#define DBYTE_AF_DB_DQ3_RX_CAL_CODE_UP_MASK                      BITS(0,5)                                   /*!< DBYTE AF DB DQ3 RX CAL CODE: UP Bit Mask */  
#define DBYTE_AF_DB_DQ3_RX_CAL_CODE_UP_OFS                       0U                                          /*!< DBYTE AF DB DQ3 RX CAL CODE: UP Bit Offset */
#define DBYTE_AF_DB_DQ3_RX_CAL_CODE_UP(regval)                   (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB DQ3 RX CAL CODE: UP Bit Value */  
#define DBYTE_AF_DB_DQ3_RX_CAL_CODE_DOWN_MASK                    BITS(8,13)                                   /*!< DBYTE AF DB DQ3 RX CAL CODE: DOWN Bit Mask */  
#define DBYTE_AF_DB_DQ3_RX_CAL_CODE_DOWN_OFS                     8U                                          /*!< DBYTE AF DB DQ3 RX CAL CODE: DOWN Bit Offset */
#define DBYTE_AF_DB_DQ3_RX_CAL_CODE_DOWN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB DQ3 RX CAL CODE: DOWN Bit Value */  
 
 /* ===== DBYTE AF DB_DQ4_RX_CAL_CODE Register definition ===== */
#define DBYTE_AF_DB_DQ4_RX_CAL_CODE_UP_MASK                      BITS(0,5)                                   /*!< DBYTE AF DB DQ4 RX CAL CODE: UP Bit Mask */  
#define DBYTE_AF_DB_DQ4_RX_CAL_CODE_UP_OFS                       0U                                          /*!< DBYTE AF DB DQ4 RX CAL CODE: UP Bit Offset */
#define DBYTE_AF_DB_DQ4_RX_CAL_CODE_UP(regval)                   (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB DQ4 RX CAL CODE: UP Bit Value */  
#define DBYTE_AF_DB_DQ4_RX_CAL_CODE_DOWN_MASK                    BITS(8,13)                                   /*!< DBYTE AF DB DQ4 RX CAL CODE: DOWN Bit Mask */  
#define DBYTE_AF_DB_DQ4_RX_CAL_CODE_DOWN_OFS                     8U                                          /*!< DBYTE AF DB DQ4 RX CAL CODE: DOWN Bit Offset */
#define DBYTE_AF_DB_DQ4_RX_CAL_CODE_DOWN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB DQ4 RX CAL CODE: DOWN Bit Value */  
 
 /* ===== DBYTE AF DB_DQ5_RX_CAL_CODE Register definition ===== */
#define DBYTE_AF_DB_DQ5_RX_CAL_CODE_UP_MASK                      BITS(0,5)                                   /*!< DBYTE AF DB DQ5 RX CAL CODE: UP Bit Mask */  
#define DBYTE_AF_DB_DQ5_RX_CAL_CODE_UP_OFS                       0U                                          /*!< DBYTE AF DB DQ5 RX CAL CODE: UP Bit Offset */
#define DBYTE_AF_DB_DQ5_RX_CAL_CODE_UP(regval)                   (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB DQ5 RX CAL CODE: UP Bit Value */  
#define DBYTE_AF_DB_DQ5_RX_CAL_CODE_DOWN_MASK                    BITS(8,13)                                   /*!< DBYTE AF DB DQ5 RX CAL CODE: DOWN Bit Mask */  
#define DBYTE_AF_DB_DQ5_RX_CAL_CODE_DOWN_OFS                     8U                                          /*!< DBYTE AF DB DQ5 RX CAL CODE: DOWN Bit Offset */
#define DBYTE_AF_DB_DQ5_RX_CAL_CODE_DOWN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB DQ5 RX CAL CODE: DOWN Bit Value */  
 
 /* ===== DBYTE AF DB_DQ6_RX_CAL_CODE Register definition ===== */
#define DBYTE_AF_DB_DQ6_RX_CAL_CODE_UP_MASK                      BITS(0,5)                                   /*!< DBYTE AF DB DQ6 RX CAL CODE: UP Bit Mask */  
#define DBYTE_AF_DB_DQ6_RX_CAL_CODE_UP_OFS                       0U                                          /*!< DBYTE AF DB DQ6 RX CAL CODE: UP Bit Offset */
#define DBYTE_AF_DB_DQ6_RX_CAL_CODE_UP(regval)                   (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB DQ6 RX CAL CODE: UP Bit Value */  
#define DBYTE_AF_DB_DQ6_RX_CAL_CODE_DOWN_MASK                    BITS(8,13)                                   /*!< DBYTE AF DB DQ6 RX CAL CODE: DOWN Bit Mask */  
#define DBYTE_AF_DB_DQ6_RX_CAL_CODE_DOWN_OFS                     8U                                          /*!< DBYTE AF DB DQ6 RX CAL CODE: DOWN Bit Offset */
#define DBYTE_AF_DB_DQ6_RX_CAL_CODE_DOWN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB DQ6 RX CAL CODE: DOWN Bit Value */  
 
 /* ===== DBYTE AF DB_DQ7_RX_CAL_CODE Register definition ===== */
#define DBYTE_AF_DB_DQ7_RX_CAL_CODE_UP_MASK                      BITS(0,5)                                   /*!< DBYTE AF DB DQ7 RX CAL CODE: UP Bit Mask */  
#define DBYTE_AF_DB_DQ7_RX_CAL_CODE_UP_OFS                       0U                                          /*!< DBYTE AF DB DQ7 RX CAL CODE: UP Bit Offset */
#define DBYTE_AF_DB_DQ7_RX_CAL_CODE_UP(regval)                   (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB DQ7 RX CAL CODE: UP Bit Value */  
#define DBYTE_AF_DB_DQ7_RX_CAL_CODE_DOWN_MASK                    BITS(8,13)                                   /*!< DBYTE AF DB DQ7 RX CAL CODE: DOWN Bit Mask */  
#define DBYTE_AF_DB_DQ7_RX_CAL_CODE_DOWN_OFS                     8U                                          /*!< DBYTE AF DB DQ7 RX CAL CODE: DOWN Bit Offset */
#define DBYTE_AF_DB_DQ7_RX_CAL_CODE_DOWN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB DQ7 RX CAL CODE: DOWN Bit Value */  
 
 /* ===== DBYTE AF DB_DQ8_RX_CAL_CODE Register definition ===== */
#define DBYTE_AF_DB_DQ8_RX_CAL_CODE_UP_MASK                      BITS(0,5)                                   /*!< DBYTE AF DB DQ8 RX CAL CODE: UP Bit Mask */  
#define DBYTE_AF_DB_DQ8_RX_CAL_CODE_UP_OFS                       0U                                          /*!< DBYTE AF DB DQ8 RX CAL CODE: UP Bit Offset */
#define DBYTE_AF_DB_DQ8_RX_CAL_CODE_UP(regval)                   (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB DQ8 RX CAL CODE: UP Bit Value */  
#define DBYTE_AF_DB_DQ8_RX_CAL_CODE_DOWN_MASK                    BITS(8,13)                                   /*!< DBYTE AF DB DQ8 RX CAL CODE: DOWN Bit Mask */  
#define DBYTE_AF_DB_DQ8_RX_CAL_CODE_DOWN_OFS                     8U                                          /*!< DBYTE AF DB DQ8 RX CAL CODE: DOWN Bit Offset */
#define DBYTE_AF_DB_DQ8_RX_CAL_CODE_DOWN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB DQ8 RX CAL CODE: DOWN Bit Value */  
 
 /* ===== DBYTE AF DB_DQS_RX_CAL_CODE Register definition ===== */
#define DBYTE_AF_DB_DQS_RX_CAL_CODE_UP_MASK                      BITS(0,5)                                   /*!< DBYTE AF DB DQS RX CAL CODE: UP Bit Mask */  
#define DBYTE_AF_DB_DQS_RX_CAL_CODE_UP_OFS                       0U                                          /*!< DBYTE AF DB DQS RX CAL CODE: UP Bit Offset */
#define DBYTE_AF_DB_DQS_RX_CAL_CODE_UP(regval)                   (BITS(0,5) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB DQS RX CAL CODE: UP Bit Value */  
#define DBYTE_AF_DB_DQS_RX_CAL_CODE_DOWN_MASK                    BITS(8,13)                                   /*!< DBYTE AF DB DQS RX CAL CODE: DOWN Bit Mask */  
#define DBYTE_AF_DB_DQS_RX_CAL_CODE_DOWN_OFS                     8U                                          /*!< DBYTE AF DB DQS RX CAL CODE: DOWN Bit Offset */
#define DBYTE_AF_DB_DQS_RX_CAL_CODE_DOWN(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB DQS RX CAL CODE: DOWN Bit Value */  
 
 /* ===== DBYTE AF DB_TSEL_CFG Register definition ===== */
#define DBYTE_AF_DB_TSEL_CFG_WR_TSEL_EN                   BIT(0)                                      /*!< When PHY is driving DQ/DQS_DM/DQS in normal mode, this field controls the TSEL of the PADs in current DBYTE. In external loop back mode, the TSEL of the PADs in current DBYTE is always asserted. */
#define DBYTE_AF_DB_TSEL_CFG_WR_TSEL_EN_OFS               0U                                          /*!< DBYTE AF DB TSEL CFG: WR_TSEL_EN Bit Offset */
#define DBYTE_AF_DB_TSEL_CFG_WR_TSEL_EN_VAL(regval)           (BIT(0) & ((uint32_t)(regval) << 0))        /*!< DBYTE AF DB TSEL CFG: WR_TSEL_EN Bit Value */  
#define DBYTE_AF_DB_TSEL_CFG_RD_TSEL_EN                   BIT(1)                                      /*!< When SDRAM is driving DQ/DQS_DM/DQS, this field controls the TSEL of the PADs in current DBYTE. */
#define DBYTE_AF_DB_TSEL_CFG_RD_TSEL_EN_OFS               1U                                          /*!< DBYTE AF DB TSEL CFG: RD_TSEL_EN Bit Offset */
#define DBYTE_AF_DB_TSEL_CFG_RD_TSEL_EN_VAL(regval)           (BIT(1) & ((uint32_t)(regval) << 1))        /*!< DBYTE AF DB TSEL CFG: RD_TSEL_EN Bit Value */  
#define DBYTE_AF_DB_TSEL_CFG_IDLE_TSEL_EN                 BIT(2)                                      /*!< When neither PHY nor SDRAM is driving DQ/DQS_DM/DQS, this field controls the TSEL of the PADs in current DBYTE. */
#define DBYTE_AF_DB_TSEL_CFG_IDLE_TSEL_EN_OFS             2U                                          /*!< DBYTE AF DB TSEL CFG: IDLE_TSEL_EN Bit Offset */
#define DBYTE_AF_DB_TSEL_CFG_IDLE_TSEL_EN_VAL(regval)         (BIT(2) & ((uint32_t)(regval) << 2))        /*!< DBYTE AF DB TSEL CFG: IDLE_TSEL_EN Bit Value */  
#define DBYTE_AF_DB_TSEL_CFG_WR_TSEL_VAL_MASK             BITS(8,15)                                   /*!< DBYTE AF DB TSEL CFG: WR_TSEL_VAL Bit Mask */  
#define DBYTE_AF_DB_TSEL_CFG_WR_TSEL_VAL_OFS              8U                                          /*!< DBYTE AF DB TSEL CFG: WR_TSEL_VAL Bit Offset */
#define DBYTE_AF_DB_TSEL_CFG_WR_TSEL_VAL(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE AF DB TSEL CFG: WR_TSEL_VAL Bit Value */  
#define DBYTE_AF_DB_TSEL_CFG_RD_TSEL_VAL_MASK             BITS(16,23)                                   /*!< DBYTE AF DB TSEL CFG: RD_TSEL_VAL Bit Mask */  
#define DBYTE_AF_DB_TSEL_CFG_RD_TSEL_VAL_OFS              16U                                          /*!< DBYTE AF DB TSEL CFG: RD_TSEL_VAL Bit Offset */
#define DBYTE_AF_DB_TSEL_CFG_RD_TSEL_VAL(regval)          (BITS(16,23) & ((uint32_t)(regval) << 16))        /*!< DBYTE AF DB TSEL CFG: RD_TSEL_VAL Bit Value */  
#define DBYTE_AF_DB_TSEL_CFG_IDLE_TSEL_VAL_MASK           BITS(24,31)                                   /*!< DBYTE AF DB TSEL CFG: IDLE_TSEL_VAL Bit Mask */  
#define DBYTE_AF_DB_TSEL_CFG_IDLE_TSEL_VAL_OFS            24U                                          /*!< DBYTE AF DB TSEL CFG: IDLE_TSEL_VAL Bit Offset */
#define DBYTE_AF_DB_TSEL_CFG_IDLE_TSEL_VAL(regval)        (BITS(24,31) & ((uint32_t)(regval) << 24))        /*!< DBYTE AF DB TSEL CFG: IDLE_TSEL_VAL Bit Value */  
 
 /* ===== DBYTE OBS DB_OBS_CFG Register definition ===== */
#define DBYTE_OBS_DB_OBS_CFG_MSTR_OBS_SEL_MASK            BITS(0,1)                                   /*!< DBYTE OBS DB OBS CFG: MSTR_OBS_SEL Bit Mask */  
#define DBYTE_OBS_DB_OBS_CFG_MSTR_OBS_SEL_OFS             0U                                          /*!< DBYTE OBS DB OBS CFG: MSTR_OBS_SEL Bit Offset */
#define DBYTE_OBS_DB_OBS_CFG_MSTR_OBS_SEL(regval)         (BITS(0,1) & ((uint32_t)(regval) << 0))        /*!< DBYTE OBS DB OBS CFG: MSTR_OBS_SEL Bit Value */  
#define DBYTE_OBS_DB_OBS_CFG_SLV_BIT_OBS_SEL_MASK         BITS(4,7)                                   /*!< DBYTE OBS DB OBS CFG: SLV_BIT_OBS_SEL Bit Mask */  
#define DBYTE_OBS_DB_OBS_CFG_SLV_BIT_OBS_SEL_OFS          4U                                          /*!< DBYTE OBS DB OBS CFG: SLV_BIT_OBS_SEL Bit Offset */
#define DBYTE_OBS_DB_OBS_CFG_SLV_BIT_OBS_SEL(regval)      (BITS(4,7) & ((uint32_t)(regval) << 4))        /*!< DBYTE OBS DB OBS CFG: SLV_BIT_OBS_SEL Bit Value */  
#define DBYTE_OBS_DB_OBS_CFG_SLV_CS_OBS_SEL               BIT(8)                                      /*!< N/A */
#define DBYTE_OBS_DB_OBS_CFG_SLV_CS_OBS_SEL_OFS           8U                                          /*!< DBYTE OBS DB OBS CFG: SLV_CS_OBS_SEL Bit Offset */
#define DBYTE_OBS_DB_OBS_CFG_SLV_CS_OBS_SEL_VAL(regval)       (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE OBS DB OBS CFG: SLV_CS_OBS_SEL Bit Value */  
#define DBYTE_OBS_DB_OBS_CFG_SLV_MISC_OBS_SEL             BIT(12)                                      /*!< N/A */
#define DBYTE_OBS_DB_OBS_CFG_SLV_MISC_OBS_SEL_OFS         12U                                          /*!< DBYTE OBS DB OBS CFG: SLV_MISC_OBS_SEL Bit Offset */
#define DBYTE_OBS_DB_OBS_CFG_SLV_MISC_OBS_SEL_VAL(regval)     (BIT(12) & ((uint32_t)(regval) << 12))        /*!< DBYTE OBS DB OBS CFG: SLV_MISC_OBS_SEL Bit Value */  
#define DBYTE_OBS_DB_OBS_CFG_RDLVL_BIT_OBS_SEL_MASK       BITS(16,19)                                   /*!< DBYTE OBS DB OBS CFG: RDLVL_BIT_OBS_SEL Bit Mask */  
#define DBYTE_OBS_DB_OBS_CFG_RDLVL_BIT_OBS_SEL_OFS        16U                                          /*!< DBYTE OBS DB OBS CFG: RDLVL_BIT_OBS_SEL Bit Offset */
#define DBYTE_OBS_DB_OBS_CFG_RDLVL_BIT_OBS_SEL(regval)    (BITS(16,19) & ((uint32_t)(regval) << 16))        /*!< DBYTE OBS DB OBS CFG: RDLVL_BIT_OBS_SEL Bit Value */  
#define DBYTE_OBS_DB_OBS_CFG_WDQLVL_BIT_OBS_SEL_MASK      BITS(20,23)                                   /*!< DBYTE OBS DB OBS CFG: WDQLVL_BIT_OBS_SEL Bit Mask */  
#define DBYTE_OBS_DB_OBS_CFG_WDQLVL_BIT_OBS_SEL_OFS       20U                                          /*!< DBYTE OBS DB OBS CFG: WDQLVL_BIT_OBS_SEL Bit Offset */
#define DBYTE_OBS_DB_OBS_CFG_WDQLVL_BIT_OBS_SEL(regval)   (BITS(20,23) & ((uint32_t)(regval) << 20))        /*!< DBYTE OBS DB OBS CFG: WDQLVL_BIT_OBS_SEL Bit Value */  
#define DBYTE_OBS_DB_OBS_CFG_RDLVL_MISC_OBS_SEL           BIT(24)                                      /*!< N/A */
#define DBYTE_OBS_DB_OBS_CFG_RDLVL_MISC_OBS_SEL_OFS       24U                                          /*!< DBYTE OBS DB OBS CFG: RDLVL_MISC_OBS_SEL Bit Offset */
#define DBYTE_OBS_DB_OBS_CFG_RDLVL_MISC_OBS_SEL_VAL(regval)   (BIT(24) & ((uint32_t)(regval) << 24))        /*!< DBYTE OBS DB OBS CFG: RDLVL_MISC_OBS_SEL Bit Value */  
#define DBYTE_OBS_DB_OBS_CFG_WDQLVL_MISC_OBS_SEL          BIT(25)                                      /*!< N/A */
#define DBYTE_OBS_DB_OBS_CFG_WDQLVL_MISC_OBS_SEL_OFS      25U                                          /*!< DBYTE OBS DB OBS CFG: WDQLVL_MISC_OBS_SEL Bit Offset */
#define DBYTE_OBS_DB_OBS_CFG_WDQLVL_MISC_OBS_SEL_VAL(regval)  (BIT(25) & ((uint32_t)(regval) << 25))        /*!< DBYTE OBS DB OBS CFG: WDQLVL_MISC_OBS_SEL Bit Value */  
#define DBYTE_OBS_DB_OBS_CFG_RD_FIFO_BIT_OBS_SEL_MASK     BITS(28,31)                                   /*!< DBYTE OBS DB OBS CFG: RD_FIFO_BIT_OBS_SEL Bit Mask */  
#define DBYTE_OBS_DB_OBS_CFG_RD_FIFO_BIT_OBS_SEL_OFS      28U                                          /*!< DBYTE OBS DB OBS CFG: RD_FIFO_BIT_OBS_SEL Bit Offset */
#define DBYTE_OBS_DB_OBS_CFG_RD_FIFO_BIT_OBS_SEL(regval)  (BITS(28,31) & ((uint32_t)(regval) << 28))        /*!< DBYTE OBS DB OBS CFG: RD_FIFO_BIT_OBS_SEL Bit Value */  
 
 /* ===== DBYTE OBS DB_MSTR_OBS Register definition ===== */
#define DBYTE_OBS_DB_MSTR_OBS_OBS                          BITS(0,31)                
 
 /* ===== DBYTE OBS DB_SLV_WRDQS_OBS Register definition ===== */
#define DBYTE_OBS_DB_SLV_WRDQS_OBS_BASE_ENC                     BITS(0,6)                
#define DBYTE_OBS_DB_SLV_WRDQS_OBS_ADDER_ENC                    BITS(8,15)                
#define DBYTE_OBS_DB_SLV_WRDQS_OBS_HALF_CYC                     BIT(16)                                      /*!< N/A */
#define DBYTE_OBS_DB_SLV_WRDQS_OBS_HALF_CYC_SHIFT               BIT(17)                                      /*!< N/A */
#define DBYTE_OBS_DB_SLV_WRDQS_OBS_CYC_SHIFT                    BIT(18)                                      /*!< N/A */
 
 /* ===== DBYTE OBS DB_SLV_WRDQ_OBS Register definition ===== */
#define DBYTE_OBS_DB_SLV_WRDQ_OBS_BASE_ENC                     BITS(0,7)                
#define DBYTE_OBS_DB_SLV_WRDQ_OBS_ADDER_ENC                    BITS(8,15)                
#define DBYTE_OBS_DB_SLV_WRDQ_OBS_HALF_CYC                     BIT(16)                                      /*!< N/A */
#define DBYTE_OBS_DB_SLV_WRDQ_OBS_HALF_CYC_SHIFT               BIT(17)                                      /*!< N/A */
#define DBYTE_OBS_DB_SLV_WRDQ_OBS_CYC_SHIFT                    BITS(18,19)                
 
 /* ===== DBYTE OBS DB_SLV_RDDQS_OBS Register definition ===== */
#define DBYTE_OBS_DB_SLV_RDDQS_OBS_R_BASE_ENC                   BITS(0,6)                
#define DBYTE_OBS_DB_SLV_RDDQS_OBS_R_ADDER_ENC                  BITS(8,15)                
#define DBYTE_OBS_DB_SLV_RDDQS_OBS_F_BASE_ENC                   BITS(16,22)                
#define DBYTE_OBS_DB_SLV_RDDQS_OBS_F_ADDER_ENC                  BITS(24,31)                
 
 /* ===== DBYTE OBS DB_SLV_RDDQS_GT_OBS Register definition ===== */
#define DBYTE_OBS_DB_SLV_RDDQS_GT_OBS_ENC                          BITS(0,9)                
#define DBYTE_OBS_DB_SLV_RDDQS_GT_OBS_HALF_CYC                     BIT(16)                                      /*!< N/A */
 
 /* ===== DBYTE OBS DB_SLV_MISC_OBS Register definition ===== */
#define DBYTE_OBS_DB_SLV_MISC_OBS_OBS                          BITS(0,31)                
 
 /* ===== DBYTE OBS DB_WRLVL_OBS Register definition ===== */
#define DBYTE_OBS_DB_WRLVL_OBS_HARD0_DLY                    BITS(0,9)                
#define DBYTE_OBS_DB_WRLVL_OBS_HARD1_DLY                    BITS(16,25)                
 
 /* ===== DBYTE OBS DB_WRLVL_MISC_OBS Register definition ===== */
#define DBYTE_OBS_DB_WRLVL_MISC_OBS_OBS                          BITS(0,31)                
 
 /* ===== DBYTE OBS DB_GTLVL_OBS Register definition ===== */
#define DBYTE_OBS_DB_GTLVL_OBS_HARD0_DLY                    BITS(0,12)                
#define DBYTE_OBS_DB_GTLVL_OBS_HARD1_DLY                    BITS(16,28)                
 
 /* ===== DBYTE OBS DB_GTLVL_MISC_OBS Register definition ===== */
#define DBYTE_OBS_DB_GTLVL_MISC_OBS_OBS                          BITS(0,31)                
 
 /* ===== DBYTE OBS DB_RDLVL_DQS_R_OBS Register definition ===== */
#define DBYTE_OBS_DB_RDLVL_DQS_R_OBS_LE_DLY                       BITS(0,8)                
#define DBYTE_OBS_DB_RDLVL_DQS_R_OBS_TE_DLY                       BITS(16,24)                
 
 /* ===== DBYTE OBS DB_RDLVL_DQS_F_OBS Register definition ===== */
#define DBYTE_OBS_DB_RDLVL_DQS_F_OBS_LE_DLY                       BITS(0,8)                
#define DBYTE_OBS_DB_RDLVL_DQS_F_OBS_TE_DLY                       BITS(16,24)                
 
 /* ===== DBYTE OBS DB_RDLVL_MISC_OBS Register definition ===== */
#define DBYTE_OBS_DB_RDLVL_MISC_OBS_OBS                          BITS(0,31)                
 
 /* ===== DBYTE OBS DB_WDQLVL_OBS Register definition ===== */
#define DBYTE_OBS_DB_WDQLVL_OBS_LE_DLY                       BITS(0,11)                
#define DBYTE_OBS_DB_WDQLVL_OBS_TE_DLY                       BITS(16,27)                
 
 /* ===== DBYTE OBS DB_WDQLVL_MISC_OBS Register definition ===== */
#define DBYTE_OBS_DB_WDQLVL_MISC_OBS_OBS                          BITS(0,31)                
 
 /* ===== DBYTE OBS DB_RX_CAL_OBS Register definition ===== */
#define DBYTE_OBS_DB_RX_CAL_OBS_OBS                          BITS(0,31)                
 
 /* ===== DBYTE OBS DB_RD_FIFO_OBS Register definition ===== */
#define DBYTE_OBS_DB_RD_FIFO_OBS_OBS                          BITS(0,31)                
 
 /* ===== DBYTE OBS DB_LPBK_OBS Register definition ===== */
#define DBYTE_OBS_DB_LPBK_OBS_OBS                          BITS(0,31)                
 
 /* ===== DBYTE FX DB_F0_MSTR_CFG Register definition ===== */
#define DBYTE_FX_DB_F0_MSTR_CFG_PVT_DRFT_THR_MASK            BITS(0,3)                                   /*!< DBYTE FX DB F0 MSTR CFG: PVT_DRFT_THR Bit Mask */  
#define DBYTE_FX_DB_F0_MSTR_CFG_PVT_DRFT_THR_OFS             0U                                          /*!< DBYTE FX DB F0 MSTR CFG: PVT_DRFT_THR Bit Offset */
#define DBYTE_FX_DB_F0_MSTR_CFG_PVT_DRFT_THR(regval)         (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX DB F0 MSTR CFG: PVT_DRFT_THR Bit Value */  
 
 /* ===== DBYTE FX DB_F0_TMG Register definition ===== */
#define DBYTE_FX_DB_F0_TMG_IE_DLY_MASK                  BITS(0,2)                                   /*!< DBYTE FX DB F0 TMG: IE_DLY Bit Mask */  
#define DBYTE_FX_DB_F0_TMG_IE_DLY_OFS                   0U                                          /*!< DBYTE FX DB F0 TMG: IE_DLY Bit Offset */
#define DBYTE_FX_DB_F0_TMG_IE_DLY(regval)               (BITS(0,2) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX DB F0 TMG: IE_DLY Bit Value */  
#define DBYTE_FX_DB_F0_TMG_TSEL_DLY_MASK                BITS(4,6)                                   /*!< DBYTE FX DB F0 TMG: TSEL_DLY Bit Mask */  
#define DBYTE_FX_DB_F0_TMG_TSEL_DLY_OFS                 4U                                          /*!< DBYTE FX DB F0 TMG: TSEL_DLY Bit Offset */
#define DBYTE_FX_DB_F0_TMG_TSEL_DLY(regval)             (BITS(4,6) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX DB F0 TMG: TSEL_DLY Bit Value */  
#define DBYTE_FX_DB_F0_TMG_RD_FIFO_DLY_MASK             BITS(8,11)                                   /*!< DBYTE FX DB F0 TMG: RD_FIFO_DLY Bit Mask */  
#define DBYTE_FX_DB_F0_TMG_RD_FIFO_DLY_OFS              8U                                          /*!< DBYTE FX DB F0 TMG: RD_FIFO_DLY Bit Offset */
#define DBYTE_FX_DB_F0_TMG_RD_FIFO_DLY(regval)          (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX DB F0 TMG: RD_FIFO_DLY Bit Value */  
#define DBYTE_FX_DB_F0_TMG_WR_PREAMBLE                  BIT(16)                                      /*!< 1 - Set write preamble to 2 tCK 0 - Set write preamble to 1 tCK */
#define DBYTE_FX_DB_F0_TMG_WR_PREAMBLE_OFS              16U                                          /*!< DBYTE FX DB F0 TMG: WR_PREAMBLE Bit Offset */
#define DBYTE_FX_DB_F0_TMG_WR_PREAMBLE_VAL(regval)          (BIT(16) & ((uint32_t)(regval) << 16))        /*!< DBYTE FX DB F0 TMG: WR_PREAMBLE Bit Value */  
#define DBYTE_FX_DB_F0_TMG_WR_POSTAMBLE                 BIT(17)                                      /*!< 1 - Set write postamble to 1.5 tCK 0 - Set write postamble to 0.5 tCK */
#define DBYTE_FX_DB_F0_TMG_WR_POSTAMBLE_OFS             17U                                          /*!< DBYTE FX DB F0 TMG: WR_POSTAMBLE Bit Offset */
#define DBYTE_FX_DB_F0_TMG_WR_POSTAMBLE_VAL(regval)         (BIT(17) & ((uint32_t)(regval) << 17))        /*!< DBYTE FX DB F0 TMG: WR_POSTAMBLE Bit Value */  
#define DBYTE_FX_DB_F0_TMG_RD_PREAMBLE                  BIT(18)                                      /*!< 1 - Set read preamble to 2 tCK 0 - Set read preamble to 1 tCK */
#define DBYTE_FX_DB_F0_TMG_RD_PREAMBLE_OFS              18U                                          /*!< DBYTE FX DB F0 TMG: RD_PREAMBLE Bit Offset */
#define DBYTE_FX_DB_F0_TMG_RD_PREAMBLE_VAL(regval)          (BIT(18) & ((uint32_t)(regval) << 18))        /*!< DBYTE FX DB F0 TMG: RD_PREAMBLE Bit Value */  
#define DBYTE_FX_DB_F0_TMG_RD_POSTAMBLE                 BIT(19)                                      /*!< 1 - Set read postamble to 1.5 tCK 0 - Set read postamble to 0.5 tCK */
#define DBYTE_FX_DB_F0_TMG_RD_POSTAMBLE_OFS             19U                                          /*!< DBYTE FX DB F0 TMG: RD_POSTAMBLE Bit Offset */
#define DBYTE_FX_DB_F0_TMG_RD_POSTAMBLE_VAL(regval)         (BIT(19) & ((uint32_t)(regval) << 19))        /*!< DBYTE FX DB F0 TMG: RD_POSTAMBLE Bit Value */  
#define DBYTE_FX_DB_F0_TMG_WRLVL_RESP_WAIT_CYC_MASK     BITS(20,25)                                   /*!< DBYTE FX DB F0 TMG: WRLVL_RESP_WAIT_CYC Bit Mask */  
#define DBYTE_FX_DB_F0_TMG_WRLVL_RESP_WAIT_CYC_OFS      20U                                          /*!< DBYTE FX DB F0 TMG: WRLVL_RESP_WAIT_CYC Bit Offset */
#define DBYTE_FX_DB_F0_TMG_WRLVL_RESP_WAIT_CYC(regval)  (BITS(20,25) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX DB F0 TMG: WRLVL_RESP_WAIT_CYC Bit Value */  
 
 /* ===== DBYTE FX DB_F0_TMG1 Register definition ===== */
#define DBYTE_FX_DB_F0_TMG1_OE_TMG_MASK                  BITS(0,7)                                   /*!< DBYTE FX DB F0 TMG1: OE_TMG Bit Mask */  
#define DBYTE_FX_DB_F0_TMG1_OE_TMG_OFS                   0U                                          /*!< DBYTE FX DB F0 TMG1: OE_TMG Bit Offset */
#define DBYTE_FX_DB_F0_TMG1_OE_TMG(regval)               (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX DB F0 TMG1: OE_TMG Bit Value */  
#define DBYTE_FX_DB_F0_TMG1_IE_TMG_MASK                  BITS(8,15)                                   /*!< DBYTE FX DB F0 TMG1: IE_TMG Bit Mask */  
#define DBYTE_FX_DB_F0_TMG1_IE_TMG_OFS                   8U                                          /*!< DBYTE FX DB F0 TMG1: IE_TMG Bit Offset */
#define DBYTE_FX_DB_F0_TMG1_IE_TMG(regval)               (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX DB F0 TMG1: IE_TMG Bit Value */  
#define DBYTE_FX_DB_F0_TMG1_RD_TSEL_TMG_MASK             BITS(24,31)                                   /*!< DBYTE FX DB F0 TMG1: RD_TSEL_TMG Bit Mask */  
#define DBYTE_FX_DB_F0_TMG1_RD_TSEL_TMG_OFS              24U                                          /*!< DBYTE FX DB F0 TMG1: RD_TSEL_TMG Bit Offset */
#define DBYTE_FX_DB_F0_TMG1_RD_TSEL_TMG(regval)          (BITS(24,31) & ((uint32_t)(regval) << 24))        /*!< DBYTE FX DB F0 TMG1: RD_TSEL_TMG Bit Value */  
 
 /* ===== DBYTE FX DB_F0_VREF_CFG Register definition ===== */
#define DBYTE_FX_DB_F0_VREF_CFG_EN                           BIT(0)                                      /*!< 1 - Read vref training is enabled. 0 - Read vref training is disabled. */
#define DBYTE_FX_DB_F0_VREF_CFG_EN_OFS                       0U                                          /*!< DBYTE FX DB F0 VREF CFG: EN Bit Offset */
#define DBYTE_FX_DB_F0_VREF_CFG_EN_VAL(regval)                   (BIT(0) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX DB F0 VREF CFG: EN Bit Value */  
#define DBYTE_FX_DB_F0_VREF_CFG_STABLE_TIME_MASK             BITS(8,18)                                   /*!< DBYTE FX DB F0 VREF CFG: STABLE_TIME Bit Mask */  
#define DBYTE_FX_DB_F0_VREF_CFG_STABLE_TIME_OFS              8U                                          /*!< DBYTE FX DB F0 VREF CFG: STABLE_TIME Bit Offset */
#define DBYTE_FX_DB_F0_VREF_CFG_STABLE_TIME(regval)          (BITS(8,18) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX DB F0 VREF CFG: STABLE_TIME Bit Value */  
 
 /* ===== DBYTE FX DB_F0_VREFSEL_CFG Register definition ===== */
#define DBYTE_FX_DB_F0_VREFSEL_CFG_LNIB_VREFSEL_MASK            BITS(0,7)                                   /*!< DBYTE FX DB F0 VREFSEL CFG: LNIB_VREFSEL Bit Mask */  
#define DBYTE_FX_DB_F0_VREFSEL_CFG_LNIB_VREFSEL_OFS             0U                                          /*!< DBYTE FX DB F0 VREFSEL CFG: LNIB_VREFSEL Bit Offset */
#define DBYTE_FX_DB_F0_VREFSEL_CFG_LNIB_VREFSEL(regval)         (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX DB F0 VREFSEL CFG: LNIB_VREFSEL Bit Value */  
#define DBYTE_FX_DB_F0_VREFSEL_CFG_HNIB_VREFSEL_MASK            BITS(8,15)                                   /*!< DBYTE FX DB F0 VREFSEL CFG: HNIB_VREFSEL Bit Mask */  
#define DBYTE_FX_DB_F0_VREFSEL_CFG_HNIB_VREFSEL_OFS             8U                                          /*!< DBYTE FX DB F0 VREFSEL CFG: HNIB_VREFSEL Bit Offset */
#define DBYTE_FX_DB_F0_VREFSEL_CFG_HNIB_VREFSEL(regval)         (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX DB F0 VREFSEL CFG: HNIB_VREFSEL Bit Value */  
 
 /* ===== DBYTE FX DB_F1_MSTR_CFG Register definition ===== */
#define DBYTE_FX_DB_F1_MSTR_CFG_PVT_DRFT_THR_MASK            BITS(0,3)                                   /*!< DBYTE FX DB F1 MSTR CFG: PVT_DRFT_THR Bit Mask */  
#define DBYTE_FX_DB_F1_MSTR_CFG_PVT_DRFT_THR_OFS             0U                                          /*!< DBYTE FX DB F1 MSTR CFG: PVT_DRFT_THR Bit Offset */
#define DBYTE_FX_DB_F1_MSTR_CFG_PVT_DRFT_THR(regval)         (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX DB F1 MSTR CFG: PVT_DRFT_THR Bit Value */  
 
 /* ===== DBYTE FX DB_F1_TMG Register definition ===== */
#define DBYTE_FX_DB_F1_TMG_IE_DLY_MASK                  BITS(0,2)                                   /*!< DBYTE FX DB F1 TMG: IE_DLY Bit Mask */  
#define DBYTE_FX_DB_F1_TMG_IE_DLY_OFS                   0U                                          /*!< DBYTE FX DB F1 TMG: IE_DLY Bit Offset */
#define DBYTE_FX_DB_F1_TMG_IE_DLY(regval)               (BITS(0,2) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX DB F1 TMG: IE_DLY Bit Value */  
#define DBYTE_FX_DB_F1_TMG_TSEL_DLY_MASK                BITS(4,6)                                   /*!< DBYTE FX DB F1 TMG: TSEL_DLY Bit Mask */  
#define DBYTE_FX_DB_F1_TMG_TSEL_DLY_OFS                 4U                                          /*!< DBYTE FX DB F1 TMG: TSEL_DLY Bit Offset */
#define DBYTE_FX_DB_F1_TMG_TSEL_DLY(regval)             (BITS(4,6) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX DB F1 TMG: TSEL_DLY Bit Value */  
#define DBYTE_FX_DB_F1_TMG_RD_FIFO_DLY_MASK             BITS(8,11)                                   /*!< DBYTE FX DB F1 TMG: RD_FIFO_DLY Bit Mask */  
#define DBYTE_FX_DB_F1_TMG_RD_FIFO_DLY_OFS              8U                                          /*!< DBYTE FX DB F1 TMG: RD_FIFO_DLY Bit Offset */
#define DBYTE_FX_DB_F1_TMG_RD_FIFO_DLY(regval)          (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX DB F1 TMG: RD_FIFO_DLY Bit Value */  
#define DBYTE_FX_DB_F1_TMG_WR_PREAMBLE                  BIT(16)                                      /*!< 1 - Set write preamble to 2 tCK 0 - Set write preamble to 1 tCK */
#define DBYTE_FX_DB_F1_TMG_WR_PREAMBLE_OFS              16U                                          /*!< DBYTE FX DB F1 TMG: WR_PREAMBLE Bit Offset */
#define DBYTE_FX_DB_F1_TMG_WR_PREAMBLE_VAL(regval)          (BIT(16) & ((uint32_t)(regval) << 16))        /*!< DBYTE FX DB F1 TMG: WR_PREAMBLE Bit Value */  
#define DBYTE_FX_DB_F1_TMG_WR_POSTAMBLE                 BIT(17)                                      /*!< 1 - Set write postamble to 1.5 tCK 0 - Set write postamble to 0.5 tCK */
#define DBYTE_FX_DB_F1_TMG_WR_POSTAMBLE_OFS             17U                                          /*!< DBYTE FX DB F1 TMG: WR_POSTAMBLE Bit Offset */
#define DBYTE_FX_DB_F1_TMG_WR_POSTAMBLE_VAL(regval)         (BIT(17) & ((uint32_t)(regval) << 17))        /*!< DBYTE FX DB F1 TMG: WR_POSTAMBLE Bit Value */  
#define DBYTE_FX_DB_F1_TMG_RD_PREAMBLE                  BIT(18)                                      /*!< 1 - Set read preamble to 2 tCK 0 - Set read preamble to 1 tCK */
#define DBYTE_FX_DB_F1_TMG_RD_PREAMBLE_OFS              18U                                          /*!< DBYTE FX DB F1 TMG: RD_PREAMBLE Bit Offset */
#define DBYTE_FX_DB_F1_TMG_RD_PREAMBLE_VAL(regval)          (BIT(18) & ((uint32_t)(regval) << 18))        /*!< DBYTE FX DB F1 TMG: RD_PREAMBLE Bit Value */  
#define DBYTE_FX_DB_F1_TMG_RD_POSTAMBLE                 BIT(19)                                      /*!< 1 - Set read postamble to 1.5 tCK 0 - Set read postamble to 0.5 tCK */
#define DBYTE_FX_DB_F1_TMG_RD_POSTAMBLE_OFS             19U                                          /*!< DBYTE FX DB F1 TMG: RD_POSTAMBLE Bit Offset */
#define DBYTE_FX_DB_F1_TMG_RD_POSTAMBLE_VAL(regval)         (BIT(19) & ((uint32_t)(regval) << 19))        /*!< DBYTE FX DB F1 TMG: RD_POSTAMBLE Bit Value */  
#define DBYTE_FX_DB_F1_TMG_WRLVL_RESP_WAIT_CYC_MASK     BITS(20,25)                                   /*!< DBYTE FX DB F1 TMG: WRLVL_RESP_WAIT_CYC Bit Mask */  
#define DBYTE_FX_DB_F1_TMG_WRLVL_RESP_WAIT_CYC_OFS      20U                                          /*!< DBYTE FX DB F1 TMG: WRLVL_RESP_WAIT_CYC Bit Offset */
#define DBYTE_FX_DB_F1_TMG_WRLVL_RESP_WAIT_CYC(regval)  (BITS(20,25) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX DB F1 TMG: WRLVL_RESP_WAIT_CYC Bit Value */  
 
 /* ===== DBYTE FX DB_F1_TMG1 Register definition ===== */
#define DBYTE_FX_DB_F1_TMG1_OE_TMG_MASK                  BITS(0,7)                                   /*!< DBYTE FX DB F1 TMG1: OE_TMG Bit Mask */  
#define DBYTE_FX_DB_F1_TMG1_OE_TMG_OFS                   0U                                          /*!< DBYTE FX DB F1 TMG1: OE_TMG Bit Offset */
#define DBYTE_FX_DB_F1_TMG1_OE_TMG(regval)               (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX DB F1 TMG1: OE_TMG Bit Value */  
#define DBYTE_FX_DB_F1_TMG1_IE_TMG_MASK                  BITS(8,15)                                   /*!< DBYTE FX DB F1 TMG1: IE_TMG Bit Mask */  
#define DBYTE_FX_DB_F1_TMG1_IE_TMG_OFS                   8U                                          /*!< DBYTE FX DB F1 TMG1: IE_TMG Bit Offset */
#define DBYTE_FX_DB_F1_TMG1_IE_TMG(regval)               (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX DB F1 TMG1: IE_TMG Bit Value */  
#define DBYTE_FX_DB_F1_TMG1_RD_TSEL_TMG_MASK             BITS(24,31)                                   /*!< DBYTE FX DB F1 TMG1: RD_TSEL_TMG Bit Mask */  
#define DBYTE_FX_DB_F1_TMG1_RD_TSEL_TMG_OFS              24U                                          /*!< DBYTE FX DB F1 TMG1: RD_TSEL_TMG Bit Offset */
#define DBYTE_FX_DB_F1_TMG1_RD_TSEL_TMG(regval)          (BITS(24,31) & ((uint32_t)(regval) << 24))        /*!< DBYTE FX DB F1 TMG1: RD_TSEL_TMG Bit Value */  
 
 /* ===== DBYTE FX DB_F1_VREF_CFG Register definition ===== */
#define DBYTE_FX_DB_F1_VREF_CFG_EN                           BIT(0)                                      /*!< 1 - Read vref training is enabled. 0 - Read vref training is disabled. */
#define DBYTE_FX_DB_F1_VREF_CFG_EN_OFS                       0U                                          /*!< DBYTE FX DB F1 VREF CFG: EN Bit Offset */
#define DBYTE_FX_DB_F1_VREF_CFG_EN_VAL(regval)                   (BIT(0) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX DB F1 VREF CFG: EN Bit Value */  
#define DBYTE_FX_DB_F1_VREF_CFG_STABLE_TIME_MASK             BITS(8,18)                                   /*!< DBYTE FX DB F1 VREF CFG: STABLE_TIME Bit Mask */  
#define DBYTE_FX_DB_F1_VREF_CFG_STABLE_TIME_OFS              8U                                          /*!< DBYTE FX DB F1 VREF CFG: STABLE_TIME Bit Offset */
#define DBYTE_FX_DB_F1_VREF_CFG_STABLE_TIME(regval)          (BITS(8,18) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX DB F1 VREF CFG: STABLE_TIME Bit Value */  
 
 /* ===== DBYTE FX DB_F1_VREFSEL_CFG Register definition ===== */
#define DBYTE_FX_DB_F1_VREFSEL_CFG_LNIB_VREFSEL_MASK            BITS(0,7)                                   /*!< DBYTE FX DB F1 VREFSEL CFG: LNIB_VREFSEL Bit Mask */  
#define DBYTE_FX_DB_F1_VREFSEL_CFG_LNIB_VREFSEL_OFS             0U                                          /*!< DBYTE FX DB F1 VREFSEL CFG: LNIB_VREFSEL Bit Offset */
#define DBYTE_FX_DB_F1_VREFSEL_CFG_LNIB_VREFSEL(regval)         (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX DB F1 VREFSEL CFG: LNIB_VREFSEL Bit Value */  
#define DBYTE_FX_DB_F1_VREFSEL_CFG_HNIB_VREFSEL_MASK            BITS(8,15)                                   /*!< DBYTE FX DB F1 VREFSEL CFG: HNIB_VREFSEL Bit Mask */  
#define DBYTE_FX_DB_F1_VREFSEL_CFG_HNIB_VREFSEL_OFS             8U                                          /*!< DBYTE FX DB F1 VREFSEL CFG: HNIB_VREFSEL Bit Offset */
#define DBYTE_FX_DB_F1_VREFSEL_CFG_HNIB_VREFSEL(regval)         (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX DB F1 VREFSEL CFG: HNIB_VREFSEL Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQS_INIT_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_INIT_CFG_INIT_CYC_LAT_MASK            BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 WRDQS INIT CFG: INIT_CYC_LAT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_INIT_CFG_INIT_CYC_LAT_OFS             0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQS INIT CFG: INIT_CYC_LAT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_INIT_CFG_INIT_CYC_LAT(regval)         (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQS INIT CFG: INIT_CYC_LAT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_INIT_CFG_INIT_SLV_DLY_MASK            BITS(4,12)                                   /*!< DBYTE FX RX DB F0 R0 WRDQS INIT CFG: INIT_SLV_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_INIT_CFG_INIT_SLV_DLY_OFS             4U                                          /*!< DBYTE FX RX DB F0 R0 WRDQS INIT CFG: INIT_SLV_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_INIT_CFG_INIT_SLV_DLY(regval)         (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F0 R0 WRDQS INIT CFG: INIT_SLV_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQS_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_CFG_CYC_LAT_MASK                 BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 WRDQS CFG: CYC_LAT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_CFG_CYC_LAT_OFS                  0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQS CFG: CYC_LAT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_CFG_CYC_LAT(regval)              (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQS CFG: CYC_LAT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_CFG_SLV_DLY_MASK                 BITS(4,12)                                   /*!< DBYTE FX RX DB F0 R0 WRDQS CFG: SLV_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_CFG_SLV_DLY_OFS                  4U                                          /*!< DBYTE FX RX DB F0 R0 WRDQS CFG: SLV_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_CFG_SLV_DLY(regval)              (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F0 R0 WRDQS CFG: SLV_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_GT_START_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_START_CFG_CYC_LAT_START_MASK           BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS GT START CFG: CYC_LAT_START Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_START_CFG_CYC_LAT_START_OFS            0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS GT START CFG: CYC_LAT_START Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_START_CFG_CYC_LAT_START(regval)        (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS GT START CFG: CYC_LAT_START Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_START_CFG_SLV_DLY_START_MASK           BITS(4,12)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS GT START CFG: SLV_DLY_START Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_START_CFG_SLV_DLY_START_OFS            4U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS GT START CFG: SLV_DLY_START Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_START_CFG_SLV_DLY_START(regval)        (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F0 R0 RDDQS GT START CFG: SLV_DLY_START Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_GT_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_CFG_CYC_LAT_MASK                 BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS GT CFG: CYC_LAT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_CFG_CYC_LAT_OFS                  0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS GT CFG: CYC_LAT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_CFG_CYC_LAT(regval)              (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS GT CFG: CYC_LAT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_CFG_SLV_DLY_MASK                 BITS(4,12)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS GT CFG: SLV_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_CFG_SLV_DLY_OFS                  4U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS GT CFG: SLV_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_CFG_SLV_DLY(regval)              (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F0 R0 RDDQS GT CFG: SLV_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ0_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ1_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ2_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ3_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ4_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ5_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ6_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ7_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ8_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ0_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ0 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ0 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ0 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ1_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ1 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ1 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ1 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ2_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ2 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ2 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ2 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ3_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ3 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ3 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ3 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ4_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ4 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ4 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ4 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ5_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ5 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ5 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ5 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ6_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ6 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ6 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ6 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ7_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ7 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ7 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ7 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ8_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ8 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ8 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ8 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQS_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_BASE_ENC_MASK                BITS(0,6)                                   /*!< DBYTE FX RX DB F0 R0 WRDQS SW CFG: BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_BASE_ENC_OFS                 0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQS SW CFG: BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_BASE_ENC(regval)             (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQS SW CFG: BASE_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_ADDER_ENC_MASK               BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R0 WRDQS SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_ADDER_ENC_OFS                8U                                          /*!< DBYTE FX RX DB F0 R0 WRDQS SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_ADDER_ENC(regval)            (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 WRDQS SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_HALF_CYC                     BIT(16)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_HALF_CYC_OFS                 16U                                          /*!< DBYTE FX RX DB F0 R0 WRDQS SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_HALF_CYC_VAL(regval)             (BIT(16) & ((uint32_t)(regval) << 16))        /*!< DBYTE FX RX DB F0 R0 WRDQS SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_HALF_CYC_SHIFT               BIT(17)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_HALF_CYC_SHIFT_OFS           17U                                          /*!< DBYTE FX RX DB F0 R0 WRDQS SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(17) & ((uint32_t)(regval) << 17))        /*!< DBYTE FX RX DB F0 R0 WRDQS SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_CYC_SHIFT                    BIT(18)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_CYC_SHIFT_OFS                18U                                          /*!< DBYTE FX RX DB F0 R0 WRDQS SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQS_SW_CFG_CYC_SHIFT_VAL(regval)            (BIT(18) & ((uint32_t)(regval) << 18))        /*!< DBYTE FX RX DB F0 R0 WRDQS SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ_SW_CFG_BASE_ENC_MASK                BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ SW CFG: BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ_SW_CFG_BASE_ENC_OFS                 0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ SW CFG: BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ_SW_CFG_BASE_ENC(regval)             (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ SW CFG: BASE_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ0_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ0 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ0 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ0 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ0 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 WRDQ0 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ0 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R0 WRDQ0 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ0 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ0 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ0_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R0 WRDQ0 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ1_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ1 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ1 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ1 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ1 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 WRDQ1 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ1 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R0 WRDQ1 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ1 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ1 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ1_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R0 WRDQ1 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ2_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ2 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ2 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ2 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ2 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 WRDQ2 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ2 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R0 WRDQ2 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ2 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ2 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ2_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R0 WRDQ2 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ3_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ3 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ3 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ3 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ3 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 WRDQ3 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ3 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R0 WRDQ3 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ3 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ3 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ3_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R0 WRDQ3 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ4_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ4 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ4 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ4 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ4 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 WRDQ4 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ4 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R0 WRDQ4 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ4 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ4 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ4_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R0 WRDQ4 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ5_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ5 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ5 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ5 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ5 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 WRDQ5 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ5 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R0 WRDQ5 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ5 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ5 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ5_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R0 WRDQ5 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ6_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ6 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ6 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ6 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ6 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 WRDQ6 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ6 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R0 WRDQ6 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ6 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ6 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ6_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R0 WRDQ6 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ7_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ7 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ7 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ7 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ7 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 WRDQ7 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ7 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R0 WRDQ7 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ7 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ7 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ7_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R0 WRDQ7 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_WRDQ8_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ8 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ8 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 WRDQ8 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ8 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 WRDQ8 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ8 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R0 WRDQ8 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R0 WRDQ8 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R0 WRDQ8 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_WRDQ8_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R0 WRDQ8 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_SW_CFG_R_BASE_ENC_MASK              BITS(0,6)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS SW CFG: R_BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_SW_CFG_R_BASE_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS SW CFG: R_BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_SW_CFG_R_BASE_ENC(regval)           (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS SW CFG: R_BASE_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_SW_CFG_F_BASE_ENC_MASK              BITS(8,14)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS SW CFG: F_BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_SW_CFG_F_BASE_ENC_OFS               8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS SW CFG: F_BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_SW_CFG_F_BASE_ENC(regval)           (BITS(8,14) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS SW CFG: F_BASE_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ0_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ0_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ0 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ1_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ1_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ1 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ2_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ2_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ2 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ3_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ3_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ3 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ4_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ4_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ4 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ5_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ5_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ5 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ6_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ6_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ6 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ7_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ7_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ7 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_DQ8_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_DQ8_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R0 RDDQS DQ8 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R0_RDDQS_GT_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_SW_CFG_ENC_MASK                     BITS(0,9)                                   /*!< DBYTE FX RX DB F0 R0 RDDQS GT SW CFG: ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_SW_CFG_ENC_OFS                      0U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS GT SW CFG: ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_SW_CFG_ENC(regval)                  (BITS(0,9) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R0 RDDQS GT SW CFG: ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_SW_CFG_HALF_CYC                     BIT(12)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_SW_CFG_HALF_CYC_OFS                 12U                                          /*!< DBYTE FX RX DB F0 R0 RDDQS GT SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R0_RDDQS_GT_SW_CFG_HALF_CYC_VAL(regval)             (BIT(12) & ((uint32_t)(regval) << 12))        /*!< DBYTE FX RX DB F0 R0 RDDQS GT SW CFG: HALF_CYC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQS_INIT_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_INIT_CFG_INIT_CYC_LAT_MASK            BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 WRDQS INIT CFG: INIT_CYC_LAT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_INIT_CFG_INIT_CYC_LAT_OFS             0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQS INIT CFG: INIT_CYC_LAT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_INIT_CFG_INIT_CYC_LAT(regval)         (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQS INIT CFG: INIT_CYC_LAT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_INIT_CFG_INIT_SLV_DLY_MASK            BITS(4,12)                                   /*!< DBYTE FX RX DB F0 R1 WRDQS INIT CFG: INIT_SLV_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_INIT_CFG_INIT_SLV_DLY_OFS             4U                                          /*!< DBYTE FX RX DB F0 R1 WRDQS INIT CFG: INIT_SLV_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_INIT_CFG_INIT_SLV_DLY(regval)         (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F0 R1 WRDQS INIT CFG: INIT_SLV_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQS_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_CFG_CYC_LAT_MASK                 BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 WRDQS CFG: CYC_LAT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_CFG_CYC_LAT_OFS                  0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQS CFG: CYC_LAT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_CFG_CYC_LAT(regval)              (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQS CFG: CYC_LAT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_CFG_SLV_DLY_MASK                 BITS(4,12)                                   /*!< DBYTE FX RX DB F0 R1 WRDQS CFG: SLV_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_CFG_SLV_DLY_OFS                  4U                                          /*!< DBYTE FX RX DB F0 R1 WRDQS CFG: SLV_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_CFG_SLV_DLY(regval)              (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F0 R1 WRDQS CFG: SLV_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_GT_START_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_START_CFG_CYC_LAT_START_MASK           BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS GT START CFG: CYC_LAT_START Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_START_CFG_CYC_LAT_START_OFS            0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS GT START CFG: CYC_LAT_START Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_START_CFG_CYC_LAT_START(regval)        (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS GT START CFG: CYC_LAT_START Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_START_CFG_SLV_DLY_START_MASK           BITS(4,12)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS GT START CFG: SLV_DLY_START Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_START_CFG_SLV_DLY_START_OFS            4U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS GT START CFG: SLV_DLY_START Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_START_CFG_SLV_DLY_START(regval)        (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F0 R1 RDDQS GT START CFG: SLV_DLY_START Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_GT_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_CFG_CYC_LAT_MASK                 BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS GT CFG: CYC_LAT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_CFG_CYC_LAT_OFS                  0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS GT CFG: CYC_LAT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_CFG_CYC_LAT(regval)              (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS GT CFG: CYC_LAT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_CFG_SLV_DLY_MASK                 BITS(4,12)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS GT CFG: SLV_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_CFG_SLV_DLY_OFS                  4U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS GT CFG: SLV_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_CFG_SLV_DLY(regval)              (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F0 R1 RDDQS GT CFG: SLV_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ0_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ1_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ2_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ3_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ4_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ5_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ6_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ7_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ8_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ0_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ0 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ0 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ0 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ1_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ1 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ1 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ1 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ2_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ2 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ2 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ2 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ3_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ3 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ3 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ3 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ4_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ4 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ4 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ4 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ5_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ5 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ5 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ5 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ6_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ6 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ6 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ6 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ7_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ7 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ7 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ7 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ8_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ8 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ8 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ8 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQS_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_BASE_ENC_MASK                BITS(0,6)                                   /*!< DBYTE FX RX DB F0 R1 WRDQS SW CFG: BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_BASE_ENC_OFS                 0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQS SW CFG: BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_BASE_ENC(regval)             (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQS SW CFG: BASE_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_ADDER_ENC_MASK               BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R1 WRDQS SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_ADDER_ENC_OFS                8U                                          /*!< DBYTE FX RX DB F0 R1 WRDQS SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_ADDER_ENC(regval)            (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 WRDQS SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_HALF_CYC                     BIT(16)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_HALF_CYC_OFS                 16U                                          /*!< DBYTE FX RX DB F0 R1 WRDQS SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_HALF_CYC_VAL(regval)             (BIT(16) & ((uint32_t)(regval) << 16))        /*!< DBYTE FX RX DB F0 R1 WRDQS SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_HALF_CYC_SHIFT               BIT(17)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_HALF_CYC_SHIFT_OFS           17U                                          /*!< DBYTE FX RX DB F0 R1 WRDQS SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(17) & ((uint32_t)(regval) << 17))        /*!< DBYTE FX RX DB F0 R1 WRDQS SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_CYC_SHIFT                    BIT(18)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_CYC_SHIFT_OFS                18U                                          /*!< DBYTE FX RX DB F0 R1 WRDQS SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQS_SW_CFG_CYC_SHIFT_VAL(regval)            (BIT(18) & ((uint32_t)(regval) << 18))        /*!< DBYTE FX RX DB F0 R1 WRDQS SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ_SW_CFG_BASE_ENC_MASK                BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ SW CFG: BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ_SW_CFG_BASE_ENC_OFS                 0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ SW CFG: BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ_SW_CFG_BASE_ENC(regval)             (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ SW CFG: BASE_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ0_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ0 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ0 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ0 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ0 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 WRDQ0 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ0 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R1 WRDQ0 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ0 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ0 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ0_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R1 WRDQ0 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ1_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ1 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ1 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ1 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ1 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 WRDQ1 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ1 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R1 WRDQ1 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ1 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ1 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ1_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R1 WRDQ1 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ2_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ2 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ2 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ2 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ2 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 WRDQ2 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ2 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R1 WRDQ2 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ2 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ2 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ2_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R1 WRDQ2 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ3_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ3 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ3 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ3 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ3 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 WRDQ3 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ3 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R1 WRDQ3 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ3 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ3 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ3_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R1 WRDQ3 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ4_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ4 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ4 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ4 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ4 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 WRDQ4 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ4 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R1 WRDQ4 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ4 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ4 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ4_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R1 WRDQ4 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ5_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ5 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ5 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ5 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ5 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 WRDQ5 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ5 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R1 WRDQ5 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ5 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ5 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ5_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R1 WRDQ5 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ6_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ6 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ6 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ6 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ6 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 WRDQ6 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ6 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R1 WRDQ6 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ6 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ6 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ6_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R1 WRDQ6 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ7_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ7 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ7 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ7 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ7 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 WRDQ7 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ7 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R1 WRDQ7 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ7 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ7 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ7_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R1 WRDQ7 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_WRDQ8_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ8 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ8 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 WRDQ8 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ8 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 WRDQ8 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ8 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F0 R1 WRDQ8 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F0 R1 WRDQ8 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F0 R1 WRDQ8 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_WRDQ8_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F0 R1 WRDQ8 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_SW_CFG_R_BASE_ENC_MASK              BITS(0,6)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS SW CFG: R_BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_SW_CFG_R_BASE_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS SW CFG: R_BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_SW_CFG_R_BASE_ENC(regval)           (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS SW CFG: R_BASE_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_SW_CFG_F_BASE_ENC_MASK              BITS(8,14)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS SW CFG: F_BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_SW_CFG_F_BASE_ENC_OFS               8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS SW CFG: F_BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_SW_CFG_F_BASE_ENC(regval)           (BITS(8,14) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS SW CFG: F_BASE_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ0_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ0_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ0 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ1_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ1_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ1 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ2_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ2_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ2 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ3_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ3_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ3 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ4_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ4_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ4 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ5_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ5_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ5 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ6_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ6_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ6 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ7_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ7_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ7 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_DQ8_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_DQ8_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F0 R1 RDDQS DQ8 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F0_R1_RDDQS_GT_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_SW_CFG_ENC_MASK                     BITS(0,9)                                   /*!< DBYTE FX RX DB F0 R1 RDDQS GT SW CFG: ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_SW_CFG_ENC_OFS                      0U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS GT SW CFG: ENC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_SW_CFG_ENC(regval)                  (BITS(0,9) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F0 R1 RDDQS GT SW CFG: ENC Bit Value */  
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_SW_CFG_HALF_CYC                     BIT(12)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_SW_CFG_HALF_CYC_OFS                 12U                                          /*!< DBYTE FX RX DB F0 R1 RDDQS GT SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F0_R1_RDDQS_GT_SW_CFG_HALF_CYC_VAL(regval)             (BIT(12) & ((uint32_t)(regval) << 12))        /*!< DBYTE FX RX DB F0 R1 RDDQS GT SW CFG: HALF_CYC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQS_INIT_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_INIT_CFG_INIT_CYC_LAT_MASK            BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 WRDQS INIT CFG: INIT_CYC_LAT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_INIT_CFG_INIT_CYC_LAT_OFS             0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQS INIT CFG: INIT_CYC_LAT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_INIT_CFG_INIT_CYC_LAT(regval)         (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQS INIT CFG: INIT_CYC_LAT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_INIT_CFG_INIT_SLV_DLY_MASK            BITS(4,12)                                   /*!< DBYTE FX RX DB F1 R0 WRDQS INIT CFG: INIT_SLV_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_INIT_CFG_INIT_SLV_DLY_OFS             4U                                          /*!< DBYTE FX RX DB F1 R0 WRDQS INIT CFG: INIT_SLV_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_INIT_CFG_INIT_SLV_DLY(regval)         (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F1 R0 WRDQS INIT CFG: INIT_SLV_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQS_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_CFG_CYC_LAT_MASK                 BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 WRDQS CFG: CYC_LAT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_CFG_CYC_LAT_OFS                  0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQS CFG: CYC_LAT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_CFG_CYC_LAT(regval)              (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQS CFG: CYC_LAT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_CFG_SLV_DLY_MASK                 BITS(4,12)                                   /*!< DBYTE FX RX DB F1 R0 WRDQS CFG: SLV_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_CFG_SLV_DLY_OFS                  4U                                          /*!< DBYTE FX RX DB F1 R0 WRDQS CFG: SLV_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_CFG_SLV_DLY(regval)              (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F1 R0 WRDQS CFG: SLV_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_GT_START_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_START_CFG_CYC_LAT_START_MASK           BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS GT START CFG: CYC_LAT_START Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_START_CFG_CYC_LAT_START_OFS            0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS GT START CFG: CYC_LAT_START Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_START_CFG_CYC_LAT_START(regval)        (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS GT START CFG: CYC_LAT_START Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_START_CFG_SLV_DLY_START_MASK           BITS(4,12)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS GT START CFG: SLV_DLY_START Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_START_CFG_SLV_DLY_START_OFS            4U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS GT START CFG: SLV_DLY_START Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_START_CFG_SLV_DLY_START(regval)        (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F1 R0 RDDQS GT START CFG: SLV_DLY_START Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_GT_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_CFG_CYC_LAT_MASK                 BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS GT CFG: CYC_LAT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_CFG_CYC_LAT_OFS                  0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS GT CFG: CYC_LAT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_CFG_CYC_LAT(regval)              (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS GT CFG: CYC_LAT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_CFG_SLV_DLY_MASK                 BITS(4,12)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS GT CFG: SLV_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_CFG_SLV_DLY_OFS                  4U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS GT CFG: SLV_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_CFG_SLV_DLY(regval)              (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F1 R0 RDDQS GT CFG: SLV_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ0_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ1_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ2_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ3_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ4_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ5_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ6_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ7_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ8_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ0_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ0 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ0 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ0 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ1_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ1 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ1 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ1 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ2_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ2 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ2 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ2 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ3_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ3 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ3 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ3 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ4_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ4 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ4 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ4 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ5_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ5 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ5 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ5 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ6_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ6 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ6 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ6 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ7_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ7 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ7 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ7 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ8_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ8 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ8 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ8 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQS_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_BASE_ENC_MASK                BITS(0,6)                                   /*!< DBYTE FX RX DB F1 R0 WRDQS SW CFG: BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_BASE_ENC_OFS                 0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQS SW CFG: BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_BASE_ENC(regval)             (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQS SW CFG: BASE_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_ADDER_ENC_MASK               BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R0 WRDQS SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_ADDER_ENC_OFS                8U                                          /*!< DBYTE FX RX DB F1 R0 WRDQS SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_ADDER_ENC(regval)            (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 WRDQS SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_HALF_CYC                     BIT(16)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_HALF_CYC_OFS                 16U                                          /*!< DBYTE FX RX DB F1 R0 WRDQS SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_HALF_CYC_VAL(regval)             (BIT(16) & ((uint32_t)(regval) << 16))        /*!< DBYTE FX RX DB F1 R0 WRDQS SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_HALF_CYC_SHIFT               BIT(17)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_HALF_CYC_SHIFT_OFS           17U                                          /*!< DBYTE FX RX DB F1 R0 WRDQS SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(17) & ((uint32_t)(regval) << 17))        /*!< DBYTE FX RX DB F1 R0 WRDQS SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_CYC_SHIFT                    BIT(18)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_CYC_SHIFT_OFS                18U                                          /*!< DBYTE FX RX DB F1 R0 WRDQS SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQS_SW_CFG_CYC_SHIFT_VAL(regval)            (BIT(18) & ((uint32_t)(regval) << 18))        /*!< DBYTE FX RX DB F1 R0 WRDQS SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ_SW_CFG_BASE_ENC_MASK                BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ SW CFG: BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ_SW_CFG_BASE_ENC_OFS                 0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ SW CFG: BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ_SW_CFG_BASE_ENC(regval)             (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ SW CFG: BASE_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ0_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ0 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ0 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ0 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ0 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 WRDQ0 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ0 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R0 WRDQ0 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ0 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ0 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ0_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R0 WRDQ0 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ1_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ1 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ1 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ1 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ1 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 WRDQ1 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ1 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R0 WRDQ1 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ1 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ1 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ1_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R0 WRDQ1 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ2_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ2 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ2 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ2 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ2 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 WRDQ2 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ2 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R0 WRDQ2 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ2 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ2 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ2_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R0 WRDQ2 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ3_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ3 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ3 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ3 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ3 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 WRDQ3 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ3 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R0 WRDQ3 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ3 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ3 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ3_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R0 WRDQ3 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ4_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ4 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ4 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ4 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ4 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 WRDQ4 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ4 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R0 WRDQ4 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ4 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ4 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ4_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R0 WRDQ4 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ5_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ5 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ5 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ5 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ5 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 WRDQ5 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ5 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R0 WRDQ5 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ5 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ5 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ5_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R0 WRDQ5 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ6_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ6 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ6 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ6 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ6 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 WRDQ6 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ6 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R0 WRDQ6 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ6 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ6 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ6_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R0 WRDQ6 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ7_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ7 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ7 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ7 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ7 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 WRDQ7 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ7 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R0 WRDQ7 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ7 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ7 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ7_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R0 WRDQ7 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_WRDQ8_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ8 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ8 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 WRDQ8 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ8 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 WRDQ8 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ8 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R0 WRDQ8 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R0 WRDQ8 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R0 WRDQ8 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_WRDQ8_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R0 WRDQ8 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_SW_CFG_R_BASE_ENC_MASK              BITS(0,6)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS SW CFG: R_BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_SW_CFG_R_BASE_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS SW CFG: R_BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_SW_CFG_R_BASE_ENC(regval)           (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS SW CFG: R_BASE_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_SW_CFG_F_BASE_ENC_MASK              BITS(8,14)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS SW CFG: F_BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_SW_CFG_F_BASE_ENC_OFS               8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS SW CFG: F_BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_SW_CFG_F_BASE_ENC(regval)           (BITS(8,14) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS SW CFG: F_BASE_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ0_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ0_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ0 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ1_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ1_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ1 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ2_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ2_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ2 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ3_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ3_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ3 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ4_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ4_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ4 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ5_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ5_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ5 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ6_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ6_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ6 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ7_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ7_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ7 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_DQ8_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_DQ8_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R0 RDDQS DQ8 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R0_RDDQS_GT_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_SW_CFG_ENC_MASK                     BITS(0,9)                                   /*!< DBYTE FX RX DB F1 R0 RDDQS GT SW CFG: ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_SW_CFG_ENC_OFS                      0U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS GT SW CFG: ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_SW_CFG_ENC(regval)                  (BITS(0,9) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R0 RDDQS GT SW CFG: ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_SW_CFG_HALF_CYC                     BIT(12)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_SW_CFG_HALF_CYC_OFS                 12U                                          /*!< DBYTE FX RX DB F1 R0 RDDQS GT SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R0_RDDQS_GT_SW_CFG_HALF_CYC_VAL(regval)             (BIT(12) & ((uint32_t)(regval) << 12))        /*!< DBYTE FX RX DB F1 R0 RDDQS GT SW CFG: HALF_CYC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQS_INIT_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_INIT_CFG_INIT_CYC_LAT_MASK            BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 WRDQS INIT CFG: INIT_CYC_LAT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_INIT_CFG_INIT_CYC_LAT_OFS             0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQS INIT CFG: INIT_CYC_LAT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_INIT_CFG_INIT_CYC_LAT(regval)         (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQS INIT CFG: INIT_CYC_LAT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_INIT_CFG_INIT_SLV_DLY_MASK            BITS(4,12)                                   /*!< DBYTE FX RX DB F1 R1 WRDQS INIT CFG: INIT_SLV_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_INIT_CFG_INIT_SLV_DLY_OFS             4U                                          /*!< DBYTE FX RX DB F1 R1 WRDQS INIT CFG: INIT_SLV_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_INIT_CFG_INIT_SLV_DLY(regval)         (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F1 R1 WRDQS INIT CFG: INIT_SLV_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQS_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_CFG_CYC_LAT_MASK                 BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 WRDQS CFG: CYC_LAT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_CFG_CYC_LAT_OFS                  0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQS CFG: CYC_LAT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_CFG_CYC_LAT(regval)              (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQS CFG: CYC_LAT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_CFG_SLV_DLY_MASK                 BITS(4,12)                                   /*!< DBYTE FX RX DB F1 R1 WRDQS CFG: SLV_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_CFG_SLV_DLY_OFS                  4U                                          /*!< DBYTE FX RX DB F1 R1 WRDQS CFG: SLV_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_CFG_SLV_DLY(regval)              (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F1 R1 WRDQS CFG: SLV_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_GT_START_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_START_CFG_CYC_LAT_START_MASK           BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS GT START CFG: CYC_LAT_START Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_START_CFG_CYC_LAT_START_OFS            0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS GT START CFG: CYC_LAT_START Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_START_CFG_CYC_LAT_START(regval)        (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS GT START CFG: CYC_LAT_START Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_START_CFG_SLV_DLY_START_MASK           BITS(4,12)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS GT START CFG: SLV_DLY_START Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_START_CFG_SLV_DLY_START_OFS            4U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS GT START CFG: SLV_DLY_START Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_START_CFG_SLV_DLY_START(regval)        (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F1 R1 RDDQS GT START CFG: SLV_DLY_START Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_GT_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_CFG_CYC_LAT_MASK                 BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS GT CFG: CYC_LAT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_CFG_CYC_LAT_OFS                  0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS GT CFG: CYC_LAT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_CFG_CYC_LAT(regval)              (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS GT CFG: CYC_LAT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_CFG_SLV_DLY_MASK                 BITS(4,12)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS GT CFG: SLV_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_CFG_SLV_DLY_OFS                  4U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS GT CFG: SLV_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_CFG_SLV_DLY(regval)              (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE FX RX DB F1 R1 RDDQS GT CFG: SLV_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ0_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ1_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ2_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ3_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ4_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ5_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ6_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ7_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ8_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_CFG_DQS_R_DLY_MASK               BITS(8,16)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 CFG: DQS_R_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_CFG_DQS_R_DLY_OFS                8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 CFG: DQS_R_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_CFG_DQS_R_DLY(regval)            (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 CFG: DQS_R_DLY Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_CFG_DQS_F_DLY_MASK               BITS(20,28)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 CFG: DQS_F_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_CFG_DQS_F_DLY_OFS                20U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 CFG: DQS_F_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_CFG_DQS_F_DLY(regval)            (BITS(20,28) & ((uint32_t)(regval) << 20))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 CFG: DQS_F_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ0_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ0 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ0 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ0 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ1_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ1 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ1 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ1 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ2_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ2 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ2 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ2 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ3_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ3 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ3 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ3 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ4_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ4 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ4 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ4 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ5_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ5 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ5 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ5 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ6_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ6 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ6 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ6 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ7_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ7 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ7 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ7 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ8_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ8 CFG: DQ_DLY Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ8 CFG: DQ_DLY Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ8 CFG: DQ_DLY Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQS_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_BASE_ENC_MASK                BITS(0,6)                                   /*!< DBYTE FX RX DB F1 R1 WRDQS SW CFG: BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_BASE_ENC_OFS                 0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQS SW CFG: BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_BASE_ENC(regval)             (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQS SW CFG: BASE_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_ADDER_ENC_MASK               BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R1 WRDQS SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_ADDER_ENC_OFS                8U                                          /*!< DBYTE FX RX DB F1 R1 WRDQS SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_ADDER_ENC(regval)            (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 WRDQS SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_HALF_CYC                     BIT(16)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_HALF_CYC_OFS                 16U                                          /*!< DBYTE FX RX DB F1 R1 WRDQS SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_HALF_CYC_VAL(regval)             (BIT(16) & ((uint32_t)(regval) << 16))        /*!< DBYTE FX RX DB F1 R1 WRDQS SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_HALF_CYC_SHIFT               BIT(17)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_HALF_CYC_SHIFT_OFS           17U                                          /*!< DBYTE FX RX DB F1 R1 WRDQS SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(17) & ((uint32_t)(regval) << 17))        /*!< DBYTE FX RX DB F1 R1 WRDQS SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_CYC_SHIFT                    BIT(18)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_CYC_SHIFT_OFS                18U                                          /*!< DBYTE FX RX DB F1 R1 WRDQS SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQS_SW_CFG_CYC_SHIFT_VAL(regval)            (BIT(18) & ((uint32_t)(regval) << 18))        /*!< DBYTE FX RX DB F1 R1 WRDQS SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ_SW_CFG_BASE_ENC_MASK                BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ SW CFG: BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ_SW_CFG_BASE_ENC_OFS                 0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ SW CFG: BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ_SW_CFG_BASE_ENC(regval)             (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ SW CFG: BASE_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ0_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ0 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ0 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ0 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ0 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 WRDQ0 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ0 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R1 WRDQ0 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ0 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ0 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ0_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R1 WRDQ0 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ1_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ1 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ1 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ1 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ1 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 WRDQ1 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ1 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R1 WRDQ1 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ1 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ1 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ1_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R1 WRDQ1 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ2_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ2 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ2 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ2 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ2 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 WRDQ2 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ2 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R1 WRDQ2 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ2 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ2 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ2_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R1 WRDQ2 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ3_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ3 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ3 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ3 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ3 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 WRDQ3 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ3 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R1 WRDQ3 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ3 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ3 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ3_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R1 WRDQ3 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ4_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ4 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ4 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ4 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ4 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 WRDQ4 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ4 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R1 WRDQ4 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ4 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ4 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ4_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R1 WRDQ4 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ5_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ5 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ5 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ5 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ5 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 WRDQ5 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ5 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R1 WRDQ5 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ5 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ5 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ5_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R1 WRDQ5 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ6_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ6 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ6 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ6 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ6 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 WRDQ6 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ6 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R1 WRDQ6 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ6 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ6 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ6_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R1 WRDQ6 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ7_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ7 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ7 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ7 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ7 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 WRDQ7 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ7 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R1 WRDQ7 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ7 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ7 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ7_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R1 WRDQ7 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_WRDQ8_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_ADDER_ENC_MASK               BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ8 SW CFG: ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_ADDER_ENC_OFS                0U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ8 SW CFG: ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_ADDER_ENC(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 WRDQ8 SW CFG: ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_HALF_CYC                     BIT(8)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_HALF_CYC_OFS                 8U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ8 SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_HALF_CYC_VAL(regval)             (BIT(8) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 WRDQ8 SW CFG: HALF_CYC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_HALF_CYC_SHIFT               BIT(9)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_HALF_CYC_SHIFT_OFS           9U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ8 SW CFG: HALF_CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_HALF_CYC_SHIFT_VAL(regval)       (BIT(9) & ((uint32_t)(regval) << 9))        /*!< DBYTE FX RX DB F1 R1 WRDQ8 SW CFG: HALF_CYC_SHIFT Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_CYC_SHIFT_MASK               BITS(10,11)                                   /*!< DBYTE FX RX DB F1 R1 WRDQ8 SW CFG: CYC_SHIFT Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_CYC_SHIFT_OFS                10U                                          /*!< DBYTE FX RX DB F1 R1 WRDQ8 SW CFG: CYC_SHIFT Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_WRDQ8_SW_CFG_CYC_SHIFT(regval)            (BITS(10,11) & ((uint32_t)(regval) << 10))        /*!< DBYTE FX RX DB F1 R1 WRDQ8 SW CFG: CYC_SHIFT Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_SW_CFG_R_BASE_ENC_MASK              BITS(0,6)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS SW CFG: R_BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_SW_CFG_R_BASE_ENC_OFS               0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS SW CFG: R_BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_SW_CFG_R_BASE_ENC(regval)           (BITS(0,6) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS SW CFG: R_BASE_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_SW_CFG_F_BASE_ENC_MASK              BITS(8,14)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS SW CFG: F_BASE_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_SW_CFG_F_BASE_ENC_OFS               8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS SW CFG: F_BASE_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_SW_CFG_F_BASE_ENC(regval)           (BITS(8,14) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS SW CFG: F_BASE_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ0_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ0_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ0 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ1_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ1_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ1 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ2_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ2_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ2 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ3_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ3_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ3 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ4_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ4_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ4 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ5_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ5_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ5 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ6_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ6_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ6 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ7_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ7_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ7 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_DQ8_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_SW_CFG_R_ADDER_ENC_MASK             BITS(0,7)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 SW CFG: R_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_SW_CFG_R_ADDER_ENC_OFS              0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 SW CFG: R_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_SW_CFG_R_ADDER_ENC(regval)          (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 SW CFG: R_ADDER_ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_SW_CFG_F_ADDER_ENC_MASK             BITS(8,15)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 SW CFG: F_ADDER_ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_SW_CFG_F_ADDER_ENC_OFS              8U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 SW CFG: F_ADDER_ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_DQ8_SW_CFG_F_ADDER_ENC(regval)          (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE FX RX DB F1 R1 RDDQS DQ8 SW CFG: F_ADDER_ENC Bit Value */  
 
 /* ===== DBYTE FX RX DB_F1_R1_RDDQS_GT_SW_CFG Register definition ===== */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_SW_CFG_ENC_MASK                     BITS(0,9)                                   /*!< DBYTE FX RX DB F1 R1 RDDQS GT SW CFG: ENC Bit Mask */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_SW_CFG_ENC_OFS                      0U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS GT SW CFG: ENC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_SW_CFG_ENC(regval)                  (BITS(0,9) & ((uint32_t)(regval) << 0))        /*!< DBYTE FX RX DB F1 R1 RDDQS GT SW CFG: ENC Bit Value */  
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_SW_CFG_HALF_CYC                     BIT(12)                                      /*!< See details in the ARCH spec */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_SW_CFG_HALF_CYC_OFS                 12U                                          /*!< DBYTE FX RX DB F1 R1 RDDQS GT SW CFG: HALF_CYC Bit Offset */
#define DBYTE_FX_RX_DB_F1_R1_RDDQS_GT_SW_CFG_HALF_CYC_VAL(regval)             (BIT(12) & ((uint32_t)(regval) << 12))        /*!< DBYTE FX RX DB F1 R1 RDDQS GT SW CFG: HALF_CYC Bit Value */  
 
 /* ===== DBYTE BOOT DB_BOOT_TMG Register definition ===== */
#define DBYTE_BOOT_DB_BOOT_TMG_IE_DLY_MASK                  BITS(0,2)                                   /*!< DBYTE BOOT DB BOOT TMG: IE_DLY Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_TMG_IE_DLY_OFS                   0U                                          /*!< DBYTE BOOT DB BOOT TMG: IE_DLY Bit Offset */
#define DBYTE_BOOT_DB_BOOT_TMG_IE_DLY(regval)               (BITS(0,2) & ((uint32_t)(regval) << 0))        /*!< DBYTE BOOT DB BOOT TMG: IE_DLY Bit Value */  
#define DBYTE_BOOT_DB_BOOT_TMG_TSEL_DLY_MASK                BITS(4,6)                                   /*!< DBYTE BOOT DB BOOT TMG: TSEL_DLY Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_TMG_TSEL_DLY_OFS                 4U                                          /*!< DBYTE BOOT DB BOOT TMG: TSEL_DLY Bit Offset */
#define DBYTE_BOOT_DB_BOOT_TMG_TSEL_DLY(regval)             (BITS(4,6) & ((uint32_t)(regval) << 4))        /*!< DBYTE BOOT DB BOOT TMG: TSEL_DLY Bit Value */  
#define DBYTE_BOOT_DB_BOOT_TMG_RD_FIFO_DLY_MASK             BITS(8,11)                                   /*!< DBYTE BOOT DB BOOT TMG: RD_FIFO_DLY Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_TMG_RD_FIFO_DLY_OFS              8U                                          /*!< DBYTE BOOT DB BOOT TMG: RD_FIFO_DLY Bit Offset */
#define DBYTE_BOOT_DB_BOOT_TMG_RD_FIFO_DLY(regval)          (BITS(8,11) & ((uint32_t)(regval) << 8))        /*!< DBYTE BOOT DB BOOT TMG: RD_FIFO_DLY Bit Value */  
#define DBYTE_BOOT_DB_BOOT_TMG_RD_PREAMBLE                  BIT(12)                                      /*!< 1 - Set read preamble to 2 tCK 0 - Set read preamble to 1 tCK */
#define DBYTE_BOOT_DB_BOOT_TMG_RD_PREAMBLE_OFS              12U                                          /*!< DBYTE BOOT DB BOOT TMG: RD_PREAMBLE Bit Offset */
#define DBYTE_BOOT_DB_BOOT_TMG_RD_PREAMBLE_VAL(regval)          (BIT(12) & ((uint32_t)(regval) << 12))        /*!< DBYTE BOOT DB BOOT TMG: RD_PREAMBLE Bit Value */  
#define DBYTE_BOOT_DB_BOOT_TMG_RD_POSTAMBLE                 BIT(13)                                      /*!< 1 - Set read postamble to 1.5 tCK 0 - Set read postamble to 0.5 tCK */
#define DBYTE_BOOT_DB_BOOT_TMG_RD_POSTAMBLE_OFS             13U                                          /*!< DBYTE BOOT DB BOOT TMG: RD_POSTAMBLE Bit Offset */
#define DBYTE_BOOT_DB_BOOT_TMG_RD_POSTAMBLE_VAL(regval)         (BIT(13) & ((uint32_t)(regval) << 13))        /*!< DBYTE BOOT DB BOOT TMG: RD_POSTAMBLE Bit Value */  
 
 /* ===== DBYTE BOOT DB_BOOT_TMG1 Register definition ===== */
#define DBYTE_BOOT_DB_BOOT_TMG1_OE_TMG_MASK                  BITS(0,7)                                   /*!< DBYTE BOOT DB BOOT TMG1: OE_TMG Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_TMG1_OE_TMG_OFS                   0U                                          /*!< DBYTE BOOT DB BOOT TMG1: OE_TMG Bit Offset */
#define DBYTE_BOOT_DB_BOOT_TMG1_OE_TMG(regval)               (BITS(0,7) & ((uint32_t)(regval) << 0))        /*!< DBYTE BOOT DB BOOT TMG1: OE_TMG Bit Value */  
#define DBYTE_BOOT_DB_BOOT_TMG1_IE_TMG_MASK                  BITS(8,15)                                   /*!< DBYTE BOOT DB BOOT TMG1: IE_TMG Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_TMG1_IE_TMG_OFS                   8U                                          /*!< DBYTE BOOT DB BOOT TMG1: IE_TMG Bit Offset */
#define DBYTE_BOOT_DB_BOOT_TMG1_IE_TMG(regval)               (BITS(8,15) & ((uint32_t)(regval) << 8))        /*!< DBYTE BOOT DB BOOT TMG1: IE_TMG Bit Value */  
#define DBYTE_BOOT_DB_BOOT_TMG1_RD_TSEL_TMG_MASK             BITS(24,31)                                   /*!< DBYTE BOOT DB BOOT TMG1: RD_TSEL_TMG Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_TMG1_RD_TSEL_TMG_OFS              24U                                          /*!< DBYTE BOOT DB BOOT TMG1: RD_TSEL_TMG Bit Offset */
#define DBYTE_BOOT_DB_BOOT_TMG1_RD_TSEL_TMG(regval)          (BITS(24,31) & ((uint32_t)(regval) << 24))        /*!< DBYTE BOOT DB BOOT TMG1: RD_TSEL_TMG Bit Value */  
 
 /* ===== DBYTE BOOT DB_BOOT_WRDQS_CFG Register definition ===== */
#define DBYTE_BOOT_DB_BOOT_WRDQS_CFG_CYC_LAT_MASK                 BITS(0,3)                                   /*!< DBYTE BOOT DB BOOT WRDQS CFG: CYC_LAT Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_WRDQS_CFG_CYC_LAT_OFS                  0U                                          /*!< DBYTE BOOT DB BOOT WRDQS CFG: CYC_LAT Bit Offset */
#define DBYTE_BOOT_DB_BOOT_WRDQS_CFG_CYC_LAT(regval)              (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE BOOT DB BOOT WRDQS CFG: CYC_LAT Bit Value */  
#define DBYTE_BOOT_DB_BOOT_WRDQS_CFG_SLV_DLY_MASK                 BITS(4,12)                                   /*!< DBYTE BOOT DB BOOT WRDQS CFG: SLV_DLY Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_WRDQS_CFG_SLV_DLY_OFS                  4U                                          /*!< DBYTE BOOT DB BOOT WRDQS CFG: SLV_DLY Bit Offset */
#define DBYTE_BOOT_DB_BOOT_WRDQS_CFG_SLV_DLY(regval)              (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE BOOT DB BOOT WRDQS CFG: SLV_DLY Bit Value */  
 
 /* ===== DBYTE BOOT DB_BOOT_RDDQS_GT_CFG Register definition ===== */
#define DBYTE_BOOT_DB_BOOT_RDDQS_GT_CFG_CYC_LAT_MASK                 BITS(0,3)                                   /*!< DBYTE BOOT DB BOOT RDDQS GT CFG: CYC_LAT Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_RDDQS_GT_CFG_CYC_LAT_OFS                  0U                                          /*!< DBYTE BOOT DB BOOT RDDQS GT CFG: CYC_LAT Bit Offset */
#define DBYTE_BOOT_DB_BOOT_RDDQS_GT_CFG_CYC_LAT(regval)              (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE BOOT DB BOOT RDDQS GT CFG: CYC_LAT Bit Value */  
#define DBYTE_BOOT_DB_BOOT_RDDQS_GT_CFG_SLV_DLY_MASK                 BITS(4,12)                                   /*!< DBYTE BOOT DB BOOT RDDQS GT CFG: SLV_DLY Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_RDDQS_GT_CFG_SLV_DLY_OFS                  4U                                          /*!< DBYTE BOOT DB BOOT RDDQS GT CFG: SLV_DLY Bit Offset */
#define DBYTE_BOOT_DB_BOOT_RDDQS_GT_CFG_SLV_DLY(regval)              (BITS(4,12) & ((uint32_t)(regval) << 4))        /*!< DBYTE BOOT DB BOOT RDDQS GT CFG: SLV_DLY Bit Value */  
 
 /* ===== DBYTE BOOT DB_BOOT_RDDQ_CFG Register definition ===== */
#define DBYTE_BOOT_DB_BOOT_RDDQ_CFG_DQ_DLY_ENC_MASK              BITS(0,3)                                   /*!< DBYTE BOOT DB BOOT RDDQ CFG: DQ_DLY_ENC Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_RDDQ_CFG_DQ_DLY_ENC_OFS               0U                                          /*!< DBYTE BOOT DB BOOT RDDQ CFG: DQ_DLY_ENC Bit Offset */
#define DBYTE_BOOT_DB_BOOT_RDDQ_CFG_DQ_DLY_ENC(regval)           (BITS(0,3) & ((uint32_t)(regval) << 0))        /*!< DBYTE BOOT DB BOOT RDDQ CFG: DQ_DLY_ENC Bit Value */  
#define DBYTE_BOOT_DB_BOOT_RDDQ_CFG_DQS_DLY_MASK                 BITS(8,16)                                   /*!< DBYTE BOOT DB BOOT RDDQ CFG: DQS_DLY Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_RDDQ_CFG_DQS_DLY_OFS                  8U                                          /*!< DBYTE BOOT DB BOOT RDDQ CFG: DQS_DLY Bit Offset */
#define DBYTE_BOOT_DB_BOOT_RDDQ_CFG_DQS_DLY(regval)              (BITS(8,16) & ((uint32_t)(regval) << 8))        /*!< DBYTE BOOT DB BOOT RDDQ CFG: DQS_DLY Bit Value */  
 
 /* ===== DBYTE BOOT DB_BOOT_WRDQ_CFG Register definition ===== */
#define DBYTE_BOOT_DB_BOOT_WRDQ_CFG_DQ_DLY_MASK                  BITS(0,11)                                   /*!< DBYTE BOOT DB BOOT WRDQ CFG: DQ_DLY Bit Mask */  
#define DBYTE_BOOT_DB_BOOT_WRDQ_CFG_DQ_DLY_OFS                   0U                                          /*!< DBYTE BOOT DB BOOT WRDQ CFG: DQ_DLY Bit Offset */
#define DBYTE_BOOT_DB_BOOT_WRDQ_CFG_DQ_DLY(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))        /*!< DBYTE BOOT DB BOOT WRDQ CFG: DQ_DLY Bit Value */  



#endif
