#include <linux/types.h>
#include <linux/bitops.h>

#define  SYS_CLK_IN_MUX_SEL_OSC_CLK_25M     (0UL)
#define  SYS_CLK_IN_MUX_SEL_OSC_CLK_16M     (1UL)
#define  CPU_CLK_I_MUX_SEL_SYS_CLK_PLL      (1UL)
#define  DDR_FAB_CLK_MUX_SEL_SYS_CLK_PLL    (1UL)
#define  XDC_CLK_IN_MUX_SEL_OSC_CLK_16M     (0UL)
#define  XEC_CLK_MUX_SEL_SYS_CLK_PLL        (3UL)

#define  PLL_CTRL0_SYS_CLK_PLL_OFS          0x78UL 
#define  PLL_CTRL1_XEC_CLK_PLL_OFS          0x7cUL 
#define  PLL_CTRL2_XDC_CLK_PLL_OFS          0x80UL 
#define  PLL_CTRL3_XDC_CLK_PLL_BK_OFS       0x84UL 
#define  PLL_CTRL4_SYS_CLK_PLL_BK_OFS       0x88UL 


#define  PLL_MUL_300MHZ                     ((5<<0)|(94<<8)|(0<<18))
#define  PLL_MUL_384MHZ                     ((5<<0)|(240<<8)|(1<<18))
#define  PLL_MUL_400MHZ                     ((4<<0)|(200<<8)|(1<<18))
#define  PLL_MUL_800MHZ                     ((3<<0)|(150<<8)|(0<<18))
#define  PLL_MUL_1200MHZ                    ((2<<0)|(150<<8)|(0<<18))
#define  PLL_MUL_1600MHZ                    ((2<<0)|(200<<8)|(0<<18))

#define  SOC_MISC_BASE                      0xf8b300000

#define  REG32(addr)                        (*((volatile unsigned int*)(addr)))

enum clock_pll_e{
    SYS_CLK_PLL,
    SYS_CLK_PLL_BK,
    XDC_CLK_PLL,
    XDC_CLK_PLL_BK,
    XEC_CLK_PLL,
    XUC_CLK_PLL,
    SYSCTL_PLL_MAX
};

typedef enum {
    DISABLE = 0,
    ENABLE = !DISABLE
} EventStatus, ControlStatus, FunctionalState;

static void cpu_clk_i_mux_sel(uint32_t src_sel)
{
    REG32(SOC_MISC_BASE + 0x10c) &= ~(GENMASK(18, 16));
    REG32(SOC_MISC_BASE + 0x10c) = src_sel<<16;
    for(uint32_t i = 0; i < 10; i++);
}

static void ddr_fab_clk_mux_sel(uint32_t src_sel)
{
    REG32(SOC_MISC_BASE + 0x110) &= ~(GENMASK(18, 16));
    REG32(SOC_MISC_BASE + 0x110) = src_sel<<16;
	for(uint32_t i = 0; i < 10; i++);
}

static void sys_clk_in_mux_sel(uint32_t src_sel)
{
    REG32(SOC_MISC_BASE + 0x100) &= ~(BIT(16));
    REG32(SOC_MISC_BASE + 0x100) = src_sel<<16;
	for(uint32_t i = 0; i < 10; i++);
}

static void xdc_clk_in_mux_sel(uint32_t src_sel)
{
    REG32(SOC_MISC_BASE + 0x104) &= ~(BIT(16));
    REG32(SOC_MISC_BASE + 0x104) = src_sel<<16;
	for(volatile int i = 0; i < 1000; i++);
}

void sdio_clk_mux_sel(uint32_t src_sel)
{
    REG32(SOC_MISC_BASE + 0x164) &= ~(GENMASK(18, 16));
    REG32(SOC_MISC_BASE + 0x164) = src_sel<<16;
}

void xec_clk_mux_sel(uint32_t src_sel)
{
    REG32(SOC_MISC_BASE + 0x198) &= ~(GENMASK(18,16));
    REG32(SOC_MISC_BASE + 0x198) = src_sel<<16;
}

static void sys_clk_pll_cg_on(ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(SOC_MISC_BASE + 0xc90) |= (1<<2);
    else
        REG32(SOC_MISC_BASE + 0xc90) &= ~(1<<2);
}

static void sys_clk_pll_pwr_on(ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(SOC_MISC_BASE + 0xc90) |= (1<<1);
    else
        REG32(SOC_MISC_BASE + 0xc90) &= ~(1<<1);
}

static void sys_clk_pll_on(ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(SOC_MISC_BASE + 0xc90) |= (1<<3);
    else
        REG32(SOC_MISC_BASE + 0xc90) &= ~(1<<3);
}

#if 0
static void sys_clk_pll_bk_on(ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(SOC_MISC_BASE + 0xc8c) |= (1<<10);
    else
        REG32(SOC_MISC_BASE + 0xc8c) &= ~(1<<10);
}

static void sys_clk_pll_bk_pwr_on(ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(SOC_MISC_BASE + 0xc8c) |= (1<<8);
    else
        REG32(SOC_MISC_BASE + 0xc8c) &= ~(1<<8);
}

static void sys_clk_pll_bk_cg_on(ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(SOC_MISC_BASE + 0xc8c) |= (1<<9);
    else
        REG32(SOC_MISC_BASE + 0xc8c) &= ~(1<<9);
}
#endif

static void xdc_clk_pll_bk_pwr_on(ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(SOC_MISC_BASE + 0xca0) |= (1<<1);
    else
        REG32(SOC_MISC_BASE + 0xca0) &= ~(1<<1);
}

static void xdc_clk_pll_bk_on(ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(SOC_MISC_BASE + 0xca0) |= (1<<3);
    else
        REG32(SOC_MISC_BASE + 0xca0) &= ~(1<<3);
}

static void xdc_clk_pll_bk_cg_on(ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(SOC_MISC_BASE + 0xca0) |= (1<<2);
    else
        REG32(SOC_MISC_BASE + 0xca0) &= ~(1<<2);
}

static void pll_ctrl0_sys_clk_pll_bp(ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(SOC_MISC_BASE + 0x78) |= (1<<24);
    else
        REG32(SOC_MISC_BASE + 0x78) &= ~(1<<24);
}

static void pll_ctrl3_xdc_clk_pll_bk_bp(ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(SOC_MISC_BASE + 0x84) |= (1<<24);
    else
        REG32(SOC_MISC_BASE + 0x84) &= ~(1<<24);
}

static uint32_t pll_ctrl3_xdc_clk_pll_bk_lock(void)
{
    return REG32(SOC_MISC_BASE + 0x84) & (1<<25);
}

static uint32_t pll_ctrl0_sys_clk_pll_lock(void)
{
    return REG32(SOC_MISC_BASE + 0x78) & (1<<25);
}

#if 0
static void pll_ctrl4_sys_clk_pll_bk_bp(ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(SOC_MISC_BASE + 0x88) |= (1<<24);
    else
        REG32(SOC_MISC_BASE + 0x88) &= ~(1<<24);
}
#endif

/*!
 * @brief enable pll power.
 */
static void clock_pll_pwr_en(enum clock_pll_e pll, ControlStatus Status)
{
    switch(pll)
    {
        case SYS_CLK_PLL:
            sys_clk_pll_pwr_on(Status);
            break;
        case SYS_CLK_PLL_BK:    
           // sys_clk_pll_bk_pwr_on(Status);
            break;
        case XDC_CLK_PLL_BK:    
            xdc_clk_pll_bk_pwr_on(Status);
            break;
        case XDC_CLK_PLL:    
           // xdc_clk_pll_pwr_on(Status);
            break;            
        case XUC_CLK_PLL:
            //xuc_clk_pll_pwr_on(Status);
            break;
        default:
            break;
    }
}

static void clock_pll_set_freq(enum clock_pll_e pll, uint32_t freq)
{
    uint32_t tmp;
    switch(pll)
    {
        case SYS_CLK_PLL:
            tmp= REG32(SOC_MISC_BASE+PLL_CTRL0_SYS_CLK_PLL_OFS) ;
            tmp &=~GENMASK(23, 0);
            tmp=tmp|freq;
            REG32(SOC_MISC_BASE+PLL_CTRL0_SYS_CLK_PLL_OFS)=tmp;
            break;
        case SYS_CLK_PLL_BK:    
            /*tmp= REG32(SOC_MISC_BASE+PLL_CTRL4_SYS_CLK_PLL_BK_OFS) ;
            tmp &=~GENMASK(23, 0);
            tmp=tmp|freq;
            REG32(SOC_MISC_BASE+PLL_CTRL4_SYS_CLK_PLL_BK_OFS)=tmp;*/
            break;
        case XDC_CLK_PLL_BK:    
            tmp= REG32(SOC_MISC_BASE+PLL_CTRL3_XDC_CLK_PLL_BK_OFS) ;
            tmp &=~GENMASK(23, 0);
            tmp=tmp|freq;
            REG32(SOC_MISC_BASE+PLL_CTRL3_XDC_CLK_PLL_BK_OFS)=tmp;
            break;
        case XUC_CLK_PLL:    
            /*tmp= readl(SOC_MISC_BASE+PLL_CTRL5_XUC_CLK_PLL_OFS) ;
            tmp &=~GENMASK(23, 0);
            tmp=tmp|freq;
            REG32(SOC_MISC_BASE+PLL_CTRL5_XUC_CLK_PLL_OFS)=tmp;*/
            break;
        default:
            break;
    }
}

static void clock_pll_cg_en(enum clock_pll_e pll, ControlStatus Status)
{
    switch(pll)
    {
        case SYS_CLK_PLL:
            sys_clk_pll_cg_on(Status);
            break;
        case SYS_CLK_PLL_BK:    
            //sys_clk_pll_bk_cg_on(Status);
            break;
        case XDC_CLK_PLL_BK:    
            xdc_clk_pll_bk_cg_on(Status);
            break;
        case XUC_CLK_PLL:
            //xuc_clk_pll_cg_on(Status);
            break;
        default:
            break;
    }
}

static void clock_pll_clk_en(enum clock_pll_e pll, ControlStatus Status)
{
    switch(pll)
    {
        case SYS_CLK_PLL:
            sys_clk_pll_on(Status);
            break;
        case SYS_CLK_PLL_BK:
            //sys_clk_pll_bk_on(Status);
            break;
        case XDC_CLK_PLL_BK:
            xdc_clk_pll_bk_on(Status);
            break;
        case XUC_CLK_PLL:
            //xuc_clk_pll_on(Status);
            break;
        default:
            break;
    }
}

static void clock_pll_bp(enum clock_pll_e pll, ControlStatus Status)
{
    switch(pll)
    {
        case SYS_CLK_PLL:
            pll_ctrl0_sys_clk_pll_bp(Status);
            break;
        case SYS_CLK_PLL_BK:
            //pll_ctrl4_sys_clk_pll_bk_bp(Status);
            break;
        case XDC_CLK_PLL_BK:
            pll_ctrl3_xdc_clk_pll_bk_bp(Status);
            break;
        case XUC_CLK_PLL:
            //pll_ctrl5_xuc_clk_pll_bp(Status);
            break;
        default:
            break;
    }
}

static int clock_pll_is_locked(enum clock_pll_e pll)
{
    uint32_t tmp;
    switch(pll)
    {
        case SYS_CLK_PLL:    
            tmp = pll_ctrl0_sys_clk_pll_lock();
            break;
        case SYS_CLK_PLL_BK:    
           // tmp = pll_ctrl4_sys_clk_pll_bk_lock();
            break;
        case XDC_CLK_PLL_BK:    
            tmp = pll_ctrl3_xdc_clk_pll_bk_lock();
            break;
        case XUC_CLK_PLL:
            //tmp = pll_ctrl5_xuc_clk_pll_lock();
            break;
        default:
            break;
    }
    return tmp;
}

void clock_pll_cfg(enum clock_pll_e pll,uint8_t src, uint32_t freq)
{
    clock_pll_pwr_en( pll, ENABLE);

    clock_pll_set_freq(pll, freq);

    clock_pll_clk_en( pll, ENABLE);

    clock_pll_cg_en( pll, ENABLE);

    clock_pll_bp(pll, DISABLE);

    while(!clock_pll_is_locked(pll));
}

static void misc_clk_div1(uint64_t reg_base, uint32_t offset ,uint8_t div_val ,uint32_t start,uint32_t end) 
{
    REG32(reg_base + offset) &= ~GENMASK(end, start);
    REG32(reg_base + offset) = div_val<<start;
}

static void misc_reset_cfg(uint64_t reg_base, uint32_t offset, uint32_t bit, ControlStatus Status)
{
    if(Status == ENABLE)
        REG32(reg_base + offset) |= BIT(bit);
    else
        REG32(reg_base + offset) &= ~BIT(bit);
}

static inline void misc_clk_cfg1(uint64_t reg_base, uint32_t offset, uint32_t bit, ControlStatus status) 
{
    if(status == ENABLE)
        REG32(reg_base + offset) |= BIT(bit);
    else
        REG32(reg_base + offset) &= ~BIT(bit);
}

static void rtc0_clk_div(uint32_t div_val) 
{
    misc_clk_div1(SOC_MISC_BASE, 0x140,div_val,0,7);
}

static void usart0_clk_div(uint32_t div_val) 
{
    misc_clk_div1(SOC_MISC_BASE, 0x128,div_val,0,7);
}

static void sdio0_clk_div(uint32_t div_val) 
{
    misc_clk_div1(SOC_MISC_BASE, 0x168,div_val,0,7);
}

static void sdio0_data_clk_div(uint32_t div_val) 
{
    misc_clk_div1(SOC_MISC_BASE, 0x16c,div_val,0,7);
}

static void ddr_fab_clk_div(uint32_t div_val) 
{
    misc_clk_div1(SOC_MISC_BASE, 0x11c,div_val,0,7);
}

static void rmii_clk_ref_div(uint32_t div_val)
{
    misc_clk_div1(SOC_MISC_BASE, 0x19c,div_val,0,7);
}

static void xec0_clk_div(uint32_t div_val)
{
    misc_clk_div1(SOC_MISC_BASE, 0x1a0,div_val,0,7);
}

static void ptp_ref_clk_div(uint32_t div_val)
{
    misc_clk_div1(SOC_MISC_BASE, 0x1a8,div_val,0,7);
}

void ddr_top0_set_rst(ControlStatus Status)
{
    misc_reset_cfg(SOC_MISC_BASE,0x20,16,Status);
}

uint32_t ddr_top0_ddr_dfs_req(void)
{
    return ((REG32(SOC_MISC_BASE + 0xc88) & BIT(3)) >> 3);
}

uint32_t ddr_top0_ddr_dfs_freq(void)
{
    return ((REG32(SOC_MISC_BASE + 0xc88) & GENMASK(2,1)) >> 1);
}

void ddr_top0_ddr_dfs_ack_pulse(void)
{
    REG32(SOC_MISC_BASE + 0xc88) |= (1<<4);
    REG32(SOC_MISC_BASE + 0xc88) &= ~(1<<4);
}

void ddr_top0_clk_mux_sel(uint32_t src_sel)
{
    REG32(SOC_MISC_BASE + 0x174) &= ~(GENMASK(17, 16));
    REG32(SOC_MISC_BASE + 0x174) = src_sel<<16;
    for (volatile int i = 0; i < 100; i++);
}

void ddr_top0_clk_en(ControlStatus Status)
{
    misc_clk_cfg1(SOC_MISC_BASE, 0x40, 16, Status);
}

#if 0
int ddr_top_clk_init(uint32_t ddr_pll_sel, uint32_t ddr_pll_freq)
{
    xdc_clk_in_mux_sel(XDC_CLK_IN_MUX_SEL_OSC_CLK_16M);
    clock_pll_cfg(ddr_pll_sel, XDC_CLK_IN_MUX_SEL_OSC_CLK_16M, ddr_pll_freq);
    ddr_fab_clk_mux_sel(DDR_FAB_CLK_MUX_SEL_SYS_CLK_PLL);
}
#endif

void soc_clk_init(void)
{
    /*set cpu to 400MHZ, main_fab_clk_i is 200MHZ, ddr_fab_clk_i is 200MHZ */
    sys_clk_in_mux_sel(SYS_CLK_IN_MUX_SEL_OSC_CLK_16M);
    xdc_clk_in_mux_sel(XDC_CLK_IN_MUX_SEL_OSC_CLK_16M);
    rtc0_clk_div(9);
    ddr_fab_clk_div(1);
    sdio0_data_clk_div(3);
	/* rmii clk is fixed 50MHZ */
	rmii_clk_ref_div(7);
	/* config xec sys_clk to div16,usually 25MHZ */
	xec0_clk_div(15);
	/* config ptp clk to div4, usually 100MHZ */
	ptp_ref_clk_div(3);

    //usart0_clk_div(1);
    clock_pll_cfg(SYS_CLK_PLL, SYS_CLK_IN_MUX_SEL_OSC_CLK_16M, PLL_MUL_400MHZ);
    //clock_pll_cfg(SYS_CLK_PLL, SYS_CLK_IN_MUX_SEL_OSC_CLK_16M, PLL_MUL_384MHZ);
    //clock_pll_cfg(XDC_CLK_PLL_BK, XDC_CLK_IN_MUX_SEL_OSC_CLK_16M, PLL_MUL_1600MHZ);
    clock_pll_cfg(XDC_CLK_PLL_BK, XDC_CLK_IN_MUX_SEL_OSC_CLK_16M, PLL_MUL_1200MHZ);
    //clock_pll_cfg(XDC_CLK_PLL_BK, XDC_CLK_IN_MUX_SEL_OSC_CLK_16M, PLL_MUL_800MHZ);
    cpu_clk_i_mux_sel(CPU_CLK_I_MUX_SEL_SYS_CLK_PLL);
    ddr_fab_clk_mux_sel(DDR_FAB_CLK_MUX_SEL_SYS_CLK_PLL);
    sdio_clk_mux_sel(DDR_FAB_CLK_MUX_SEL_SYS_CLK_PLL);
    xec_clk_mux_sel(XEC_CLK_MUX_SEL_SYS_CLK_PLL);
}
