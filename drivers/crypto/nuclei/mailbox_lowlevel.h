
#ifndef _MAILBOX_LL_H
#define _MAILBOX_LL_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "linux/types.h"
#include "linux/bitops.h"
/* Includes ------------------------------------------------------------------*/
#define ADDR8P(addr)                        ((uint8_t *)(uintptr_t)(addr))

#define ADDR32P(addr)                       ((uint32_t *)(uintptr_t)(addr))

#define ADDR32(addr)                        ((uint32_t)(uintptr_t)(addr))

#define BITS(l,h) GENMASK(h, l)

typedef enum {
    DISABLE = 0,
    ENABLE = !DISABLE
} EventStatus, ControlStatus, FunctionalState;

typedef enum {
    RESET = 0,
    SET = 1,
    MAX = 0X7FFFFFFF
} FlagStatus;

typedef enum {
    ERROR = 0,
    SUCCESS = !ERROR
} ErrStatus;

/**
 ** @brief MAILBOX
 **/
typedef struct {  /*!< MAILBOX Structure */
    uint32_t  IP_VER;                                   /*!< Offset: 0x0 RW IP VER Register */
    uint32_t  GIT_VER;                                  /*!< Offset: 0x4 RW GIT VER Register */
    uint32_t  CSR;                                      /*!< Offset: 0x8 RW CSR Register */
    uint32_t  RESET;                                    /*!< Offset: 0xc RW RESET Register */
    uint32_t  LINKID0;                                  /*!< Offset: 0x10 RW LINKID0 Register */
    uint32_t  LINKID1;                                  /*!< Offset: 0x14 RW LINKID1 Register */
    uint32_t  LOCKOUT0;                                 /*!< Offset: 0x18 RW LOCKOUT0 Register */
    uint32_t  LOCKOUT1;                                 /*!< Offset: 0x1C RW LOCKOUT1 Register */
    uint32_t  LOCKOUT2;                                 /*!< Offset: 0x20 RW LOCKOUT2 Register */
    uint32_t  LOCKOUT3;                                 /*!< Offset: 0x24 RW LOCKOUT3 Register */
    uint32_t  LOCKOUT4;                                 /*!< Offset: 0x28 RW LOCKOUT4 Register */
    uint32_t  LOCKOUT5;                                 /*!< Offset: 0x2C RW LOCKOUT5 Register */
    uint32_t  LOCKOUT6;                                 /*!< Offset: 0x30 RW LOCKOUT6 Register */
    uint32_t  LOCKOUT7;                                 /*!< Offset: 0x34 RW LOCKOUT7 Register */
    uint32_t  INTEN;                                    /*!< Offset: 0x38 RW INTEN Register */
    uint32_t  INTST;                                    /*!< Offset: 0x3c RW INTST Register */
    uint32_t  RESERVECD16[7];                           /*!< Offset: 0x40 RO RESERVECD16[7] Register */
    uint32_t  OPT;                                      /*!< Offset: 0x5c RW OPT Register */
} MAILBOX_TypeDef;

#define MAILBOX_RF_OPT_ACTIVE_HOST_N_OFS                6        /*!< MAILBOX RF OPT: OPT_ACTIVE_HOST_N Bits Offset */                
#define MAILBOX_RF_OPT_MASTER_ID_OFS                    12       /*!< MAILBOX RF OPT: OPT_MASTER_ID Bits Offset */
#define MAILBOX_RF_OPT_MY_ID_OFS                        18       /*!< MAILBOX RF OPT: OPT_MY_ID Bits Offset */

#define MAILBOX_RF_IP_VER_OFFSET                        0x0 /*!< IP Version Number */
#define MAILBOX_RF_GIT_VER_OFFSET                       0x4 /*!< Git Version Number */
#define MAILBOX_RF_CSR_OFFSET                           0x8 /*!< MAILBOX control/status register */
#define MAILBOX_RF_RESET_OFFSET                         0xc /*!< MAILBOX reset register */
#define MAILBOX_RF_LINKID0_OFFSET                       0x10 /*!< MAILBOX link id register */
#define MAILBOX_RF_LOCKOUT0_OFFSET                      0x18 /*!< MAILBOX lockout0 register */
#define MAILBOX_RF_INTEN_OFFSET                         0x38 /*!< Individual enable control bits per interrupt input: 0b = Disable. 1b = Enalbe. */
#define MAILBOX_RF_INTST_OFFSET                         0x3c /*!< These bits reflect the status of the interrupts: 0b = Inactive. 1b = Pending. Write 1 to to clear status bit. */
#define MAILBOX_RF_OPT_OFFSET                           0x5c /*!< MAILBOX internal configure options */

#define MAILBOX_BASE_ADDR_OFFSET                        0x2000
#define MAILBOX_AVALIABLE_MAX_NUM                       1
#define MAILBOX_SIZE_IN_BYTE                            256

 /* ===== MAILBOX RF IP_VER Register definition ===== */
#define MAILBOX_RF_IP_VER_IP_VER                        BITS(0,23)                
 
 /* ===== MAILBOX RF GIT_VER Register definition ===== */
#define MAILBOX_RF_GIT_VER_GIT_VER                      BITS(0,31)                
 
 /* ===== MAILBOX RF CSR Register definition ===== */
#define MAILBOX_RF_CSR_MBX0_IN                          BIT(0)

#define MAILBOX_RF_CSR_MBX0_IN_OFS                      0U   /*!< MAILBOX RF CSR: MBX0_IN Bit Offset */
#define MAILBOX_RF_CSR_MBX0_IN_VAL(regval)              (BIT(0) & ((uint32_t)(regval) << 0))  /*!< MAILBOX RF CSR: MBX0_IN Bit Value */  
#define MAILBOX_RF_CSR_MBX0_IN_NON_FULL                 0x0UL    /*!< NON_FULL */
#define MAILBOX_RF_CSR_MBX0_IN_FULL                     BIT(0)   /*!< FULL */

/**
  * \brief Check the MAILBOX_RF csr mbx0_in bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid mbx0_in bit.
  * \retval 1 This is a valid mbx0_in bit.
  */
#define IS_MAILBOX_RF_CSR_MBX0_IN(regval)               (((regval) == MAILBOX_RF_CSR_MBX0_IN_NON_FULL) || ((regval) == MAILBOX_RF_CSR_MBX0_IN_FULL))

#define MAILBOX_RF_CSR_MBX0_OUT                         BIT(1)
#define MAILBOX_RF_CSR_MBX0_OUT_OFS                     1U                                          /*!< MAILBOX RF CSR: MBX0_OUT Bit Offset */
#define MAILBOX_RF_CSR_MBX0_OUT_VAL(regval)             (BIT(1) & ((uint32_t)(regval) << 1))        /*!< MAILBOX RF CSR: MBX0_OUT Bit Value */  
#define MAILBOX_RF_CSR_MBX0_OUT_NON_FULL                0x0UL    /*!< NON_FULL */
#define MAILBOX_RF_CSR_MBX0_OUT_FULL                    BIT(1)   /*!< FULL */

/**
  * \brief Check the MAILBOX_RF csr mbx0_out bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid mbx0_out bit.
  * \retval 1 This is a valid mbx0_out bit.
  */
#define IS_MAILBOX_RF_CSR_MBX0_OUT(regval)              (((regval) == MAILBOX_RF_CSR_MBX0_OUT_NON_FULL) || ((regval) == MAILBOX_RF_CSR_MBX0_OUT_FULL))

#define MAILBOX_RF_CSR_MBX0                              BIT(2)
#define MAILBOX_RF_CSR_MBX0_OFS                          2U    /*!< MAILBOX RF CSR: MBX0 Bit Offset */
#define MAILBOX_RF_CSR_MBX0_VAL(regval)                  (BIT(2) & ((uint32_t)(regval) << 2))        /*!< MAILBOX RF CSR: MBX0 Bit Value */  
#define MAILBOX_RF_CSR_MBX0_NON_LINK                     0x0UL     /*!< NON_LINK */
#define MAILBOX_RF_CSR_MBX0_LINK                         BIT(2)    /*!< LINK */

/**
  * \brief Check the MAILBOX_RF csr mbx0 bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid mbx0 bit.
  * \retval 1 This is a valid mbx0 bit.
  */
#define IS_MAILBOX_RF_CSR_MBX0(regval)                   (((regval) == MAILBOX_RF_CSR_MBX0_NON_LINK) || ((regval) == MAILBOX_RF_CSR_MBX0_LINK))

#define MAILBOX_RF_CSR_MBX0_UNLINK                       BIT(3)
#define MAILBOX_RF_CSR_MBX0_UNLINK_OFS              3U                                          /*!< MAILBOX RF CSR: MBX0_UNLINK Bit Offset */
#define MAILBOX_RF_CSR_MBX0_UNLINK_VAL(regval)           (BIT(3) & ((uint32_t)(regval) << 3))        /*!< MAILBOX RF CSR: MBX0_UNLINK Bit Value */  
#define MAILBOX_RF_CSR_MBX0_UNLINK_UNAVAILABLE           0x0UL    /*!< UNAVAILABLE */
#define MAILBOX_RF_CSR_MBX0_UNLINK_AVAILABLE             BIT(3)  /*!< AVAILABLE */

/**
  * \brief Check the MAILBOX_RF csr mbx0_unlink bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid mbx0_unlink bit.
  * \retval 1 This is a valid mbx0_unlink bit.
  */
#define IS_MAILBOX_RF_CSR_MBX0_UNLINK(regval)            (((regval) == MAILBOX_RF_CSR_MBX0_UNLINK_UNAVAILABLE) || ((regval) == MAILBOX_RF_CSR_MBX0_UNLINK_AVAILABLE))

 /* ===== MAILBOX RF RESET Register definition ===== */
#define MAILBOX_RF_RESET_MBX0_OUT_EMPTY                  BIT(1)
#define MAILBOX_RF_RESET_MBX0_OUT_EMPTY_OFS              1U  /*!< MAILBOX RF RESET: MBX0_OUT_EMPTY Bit Offset */
#define MAILBOX_RF_RESET_MBX0_OUT_EMPTY_VAL(regval)      (BIT(1) & ((uint32_t)(regval) << 1))        /*!< MAILBOX RF RESET: MBX0_OUT_EMPTY Bit Value */  
#define MAILBOX_RF_RESET_MBX0_UNLINK                     BIT(3) 
#define MAILBOX_RF_RESET_MBX0_UNLINK_OFS              3U                                          /*!< MAILBOX RF RESET: MBX0_UNLINK Bit Offset */
#define MAILBOX_RF_RESET_MBX0_UNLINK_VAL(regval)          (BIT(3) & ((uint32_t)(regval) << 3))        /*!< MAILBOX RF RESET: MBX0_UNLINK Bit Value */  
 
 /* ===== MAILBOX RF LINKID0 Register definition ===== */
#define MAILBOX_RF_LINKID0_MBX0_LINK_ID                 BIT(0)                                      /*!< indicates the Host cpu_id of the Host linked to input Mailbox 0. */
#define MAILBOX_RF_LINKID0_MBX0_ACCESS                  BIT(7)                                      /*!< -  1'b0: non-protected: non-protected: Mailbox 0 can be accessed by the Host using protected or non-protected access. -  1'b1: protected: protected: Mailbox 0 is only accessible if the Host uses protected access. */
#define MAILBOX_RF_LINKID0_MBX0_ACCESS_NON_PROTECTED       ((uint32_t)(0) << 7)   /*!< NON_PROTECTED */
#define MAILBOX_RF_LINKID0_MBX0_ACCESS_PROTECTED           ((uint32_t)(1) << 7)   /*!< PROTECTED */

/**
  * \brief Check the MAILBOX_RF linkid0 mbx0_access bit parameters.
  * \param regval bit value to be checked.
  * \retval 0 This is not a valid mbx0_access bit.
  * \retval 1 This is a valid mbx0_access bit.
  */
#define IS_MAILBOX_RF_LINKID0_MBX0_ACCESS(regval)            (                                      ((regval) == MAILBOX_RF_LINKID0_MBX0_ACCESS_NON_PROTECTED       ) ||                                       ((regval) == MAILBOX_RF_LINKID0_MBX0_ACCESS_PROTECTED           )                                                   )

 /* ===== MAILBOX RF LOCKOUT0 Register definition ===== */
#define MAILBOX_RF_LOCKOUT0_MBX0                         BIT(0)                                      /*!< Bit map indicating which Hosts are blocked from accessing mailbox 0, a 1b in bit [n] blocks Host cpu_id n. */
#define MAILBOX_RF_LOCKOUT0_MBX0_OFS                     0U                                          /*!< MAILBOX RF LOCKOUT0: MBX0 Bit Offset */
#define MAILBOX_RF_LOCKOUT0_MBX0_VAL(regval)                 (BIT(0) & ((uint32_t)(regval) << 0))        /*!< MAILBOX RF LOCKOUT0: MBX0 Bit Value */  
 
 /* ===== MAILBOX RF INTEN Register definition ===== */
#define MAILBOX_RF_INTEN_MBX0_IFRE                    BIT(0)                                      /*!< The Host is linked to mailbox 0, and that mailbox is currently not full. */
#define MAILBOX_RF_INTEN_MBX0_IFRE_OFS                0U                                          /*!< MAILBOX RF INTEN: MBX0_IFRE Bit Offset */
#define MAILBOX_RF_INTEN_MBX0_IFRE_VAL(regval)            (BIT(0) & ((uint32_t)(regval) << 0))        /*!< MAILBOX RF INTEN: MBX0_IFRE Bit Value */  
#define MAILBOX_RF_INTEN_MBX0_DONE                    BIT(1)                                      /*!< the output mailbox 0 contains a result token intended for the Host. */
#define MAILBOX_RF_INTEN_MBX0_DONE_OFS                1U                                          /*!< MAILBOX RF INTEN: MBX0_DONE Bit Offset */
#define MAILBOX_RF_INTEN_MBX0_DONE_VAL(regval)            (BIT(1) & ((uint32_t)(regval) << 1))        /*!< MAILBOX RF INTEN: MBX0_DONE Bit Value */  
#define MAILBOX_RF_INTEN_MBX_LINKABLE                 BIT(16)                                      /*!< This interrupt signal indicates that at least one more mailbox can be linked by the Host accessing this register (there is a non-linked, empty input mailbox where the lockout bit is 0b for this Host). */
#define MAILBOX_RF_INTEN_MBX_LINKABLE_OFS             16U                                          /*!< MAILBOX RF INTEN: MBX_LINKABLE Bit Offset */
#define MAILBOX_RF_INTEN_MBX_LINKABLE_VAL(regval)         (BIT(16) & ((uint32_t)(regval) << 16))        /*!< MAILBOX RF INTEN: MBX_LINKABLE Bit Value */  
 
 /* ===== MAILBOX RF INTST Register definition ===== */
#define MAILBOX_RF_INTST_MBX0_IFRE                    BIT(0)                                      /*!< The Host is linked to mailbox 0, and that mailbox is currently not full. */
#define MAILBOX_RF_INTST_MBX0_IFRE_OFS                0U                                          /*!< MAILBOX RF INTST: MBX0_IFRE Bit Offset */
#define MAILBOX_RF_INTST_MBX0_IFRE_VAL(regval)            (BIT(0) & ((uint32_t)(regval) << 0))        /*!< MAILBOX RF INTST: MBX0_IFRE Bit Value */  
#define MAILBOX_RF_INTST_MBX0_DONE                    BIT(1)                                      /*!< the output mailbox 0 contains a result token intended for the Host. */
#define MAILBOX_RF_INTST_MBX0_DONE_OFS                1U                                          /*!< MAILBOX RF INTST: MBX0_DONE Bit Offset */
#define MAILBOX_RF_INTST_MBX0_DONE_VAL(regval)            (BIT(1) & ((uint32_t)(regval) << 1))        /*!< MAILBOX RF INTST: MBX0_DONE Bit Value */  
#define MAILBOX_RF_INTST_MBX_LINKABLE                 BIT(16)                                      /*!< This interrupt signal indicates that at least one more mailbox can be linked by the Host accessing this register (there is a non-linked, empty input mailbox where the lockout bit is 0b for this Host). */
#define MAILBOX_RF_INTST_MBX_LINKABLE_OFS             16U                                          /*!< MAILBOX RF INTST: MBX_LINKABLE Bit Offset */
#define MAILBOX_RF_INTST_MBX_LINKABLE_VAL(regval)         (BIT(16) & ((uint32_t)(regval) << 16))        /*!< MAILBOX RF INTST: MBX_LINKABLE Bit Value */  
 
 /* ===== MAILBOX RF OPT Register definition ===== */
#define MAILBOX_RF_OPT_MBX_NUM                      BITS(0,3)                
#define MAILBOX_RF_OPT_MBX_SIZE                     BITS(4,5)                
#define MAILBOX_RF_OPT_MBX_SIZE_128_BYTES                ((uint32_t)(0) << 4)                                                   /*!< 128_BYTES */
#define MAILBOX_RF_OPT_MBX_SIZE_256_BYTES                ((uint32_t)(1) << 4)                                                   /*!< 256_BYTES */
#define MAILBOX_RF_OPT_MBX_SIZE_512_BYTES                ((uint32_t)(2) << 4)                                                   /*!< 512_BYTES */
#define MAILBOX_RF_OPT_MBX_SIZE_1K_BYTES                 ((uint32_t)(3) << 4)                                                   /*!< 1K_BYTES */

/**
  * \brief Check the MAILBOX_RF opt mbx_size bits parameters.
  * \param regval bits value to be checked.
  * \retval 0 This is not a valid mbx_size bits.
  * \retval 1 This is a valid mbx_size bits.
  */
#define IS_MAILBOX_RF_OPT_MBX_SIZE(regval)               (                                         ((regval) == MAILBOX_RF_OPT_MBX_SIZE_128_BYTES           ) ||                                          ((regval) == MAILBOX_RF_OPT_MBX_SIZE_256_BYTES           ) ||                                          ((regval) == MAILBOX_RF_OPT_MBX_SIZE_512_BYTES           ) ||                                          ((regval) == MAILBOX_RF_OPT_MBX_SIZE_1K_BYTES            )                                                   )

#define MAILBOX_RF_OPT_ACTIVE_HOST_N                BITS(6,7)                
#define MAILBOX_RF_OPT_MASTER_ID                    BIT(12)                                      /*!< Value on the cpu_id input that designates the master Host. */
#define MAILBOX_RF_OPT_PROT_AV                      BIT(17)                                      /*!< support the protection,This bit is always one. */
#define MAILBOX_RF_OPT_MY_ID                        BIT(18)                                      /*!< Host ID code for the Host that is actually reading this register, the value of the cpu_id input used during the read access. */
#define MAILBOX_RF_OPT_PROT                         BIT(23)                                      /*!< Indicates the current protection bit value of the Host actually reading this register. */
/**
  * @brief  Get mailbox specified LOCKOUT register value.
  * @param  HANDLE MAILBOX handle
  * @param  NUM specifies the LOCKOUT register.
  *          This parameter can be 0 ~ (1 - 1):
  * @retval The value of the appointed LOCKOUT register
  */
#define MAILBOX_LOCKOUT_REG(HANDLE, NUM)     (*ADDR32P((uint32_t)(ADDR32(HANDLE) + MAILBOX_RF_LOCKOUT0_OFFSET + (0x04*(NUM)))))

/**
  * @brief  Get mailbox specified LINKID register value.
  * @param  HANDLE MAILBOX handle
  * @param  NUM specifies the LINKID register.
  *          This parameter can be 0 ~ ((1 - 1) / 4)
  * @retval The value of the appointed LINKID register
  */
#define MAILBOX_LINKID_REG(HANDLE, NUM)     (*ADDR32P((uint32_t)(ADDR32(HANDLE) + MAILBOX_RF_LINKID0_OFFSET + (0x04*(NUM)))))

ErrStatus MAILBOX_HostLinkToMbx(MAILBOX_TypeDef* mailbox, uint8_t mbx_num);
void MAILBOX_HostWriteDataToMailboxIn(MAILBOX_TypeDef* mailbox, uint8_t mbx_num, uint32_t *addr, uint32_t *buf, uint8_t len);
void MAILBOX_HostReadDataFromMailboxOut(MAILBOX_TypeDef* mailbox, uint8_t mbx_num, uint32_t *addr, uint32_t *buf, uint8_t len);

void MAILBOX_LockOutCtrl(MAILBOX_TypeDef* mailbox, uint8_t host_id, uint8_t mbx_num, FunctionalState NewState);
FlagStatus MAILBOX_GetUnlockStatus(MAILBOX_TypeDef* mailbox, uint8_t host_id, uint8_t mbx_num);
void MAILBOX_MastHostUnlinkMbx(MAILBOX_TypeDef* mailbox, uint8_t mbx_num);
void MAILBOX_MbxLink(MAILBOX_TypeDef* mailbox, uint8_t mbx_num);
FlagStatus MAILBOX_GetMbxUnlinkAlbStatus(MAILBOX_TypeDef* mailbox, uint8_t mbx_num);
FlagStatus MAILBOX_GetMbxLinkStatus(MAILBOX_TypeDef* mailbox, uint8_t mbx_num);
uint8_t MAILBOX_GetAlbToLinkMbxNum(MAILBOX_TypeDef* mailbox);
uint8_t MAILBOX_GetLinkedMbxNum(MAILBOX_TypeDef* mailbox);
void MAILBOX_HostUnlinkMbx(MAILBOX_TypeDef* mailbox, uint8_t mbx_num);

void MAILBOX_MasterHostClearMbxOutFull(MAILBOX_TypeDef* mailbox, uint8_t mbx_num);
void MAILBOX_SetMbxInFull(MAILBOX_TypeDef* mailbox, uint8_t mbx_num);
void MAILBOX_SetMbxOutFull(MAILBOX_TypeDef* mailbox, uint8_t mbx_num);
FlagStatus MAILBOX_GetMbxInFullStatus(MAILBOX_TypeDef* mailbox, uint8_t mbx_num);
FlagStatus MAILBOX_GetMbxOutFullStatus(MAILBOX_TypeDef* mailbox, uint8_t mbx_num);

uint8_t MAILBOX_GetMasterId(MAILBOX_TypeDef* mailbox);
uint8_t MAILBOX_GetActivedHostNum(MAILBOX_TypeDef* mailbox);
uint8_t MAILBOX_GetHostId(MAILBOX_TypeDef* mailbox);
uint8_t MAILBOX_GetMbxLinkedId(MAILBOX_TypeDef* mailbox, uint8_t mbx_num);
uint32_t MAILBOX_GetIpVersion(MAILBOX_TypeDef* mailbox);
uint32_t MAILBOX_GetGitVersion(MAILBOX_TypeDef* mailbox);

#ifdef __cplusplus
}
#endif

#endif
 
