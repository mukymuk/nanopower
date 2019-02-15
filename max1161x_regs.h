#ifndef __MAX1161X_REGS_H__
#define __MAX1161X_REGS_H__

#define MAX116X_REG_CONFIG  0
#define MAX116X_REG_SETUP   1

#define MAX1161X_SEL2_VDD_REFERENCE         0
#define MAX1161X_SEL2_EXTERNAL_REFERENCE    0
#define MAX1161X_SEL2_INTERNAL_REFERENCE    1

#define MAX1161X_SEL1_INPUT_ANALOG          0
#define MAX1161X_SEL1_INPUT_REFERENCE       1
#define MAX1161X_SEL1_OUTPUT_REFERENCE      1

#define MAX1161X_SEL0_REFERENCE_ON  1
#define MAX1161X_SEL1_REFERENCE_OFF 0

#define MAX1161X_CLK_EXTERNAL   1
#define MAX1161X_CLK_INTERNAL   0

#define MAX1161X_BIP_UNI_BIPOLAR    1
#define MAX1161X_BIP_UNI_UNIPOLAR	0

#define MAX1161X_RST_DISASSERT      1
#define MAX1161X_RST_ASSERT         0

#define MAX1161X_SCAN_TO_CS         0
#define MAX1161X_SCAN_CS_8X         1
#define MAX1161X_SCAN_UPPER         2
#define MAX1161X_SCAN_CS            3

#define MAX1161X_CS_AIN(x)          x

#define MAX1161X_SGL_DIF_SINGLE_ENDED   1
#define MAX1161X_SGL_DIF_DIFFERENTIAL   0

typedef struct
{
    uint8_t undefined : 1;
    uint8_t rst : 1;
    uint8_t bip_uni : 1;
    uint8_t clk : 1;
    uint8_t     sel0 : 1;
    uint8_t     sel1 : 1;
    uint8_t     sel2 : 1;   // see datasheet Table 6.
    uint8_t reg : 1;
}
max1161x_setup_t;

typedef struct
{
    uint8_t     sgl_dif : 1;
    uint8_t     cs : 4;
    uint8_t     scan : 2;   // see datasheet Table 5.
    uint8_t     reg : 1;
}
max1161x_config_t;



#endif
