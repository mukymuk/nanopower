#include "global.h"
#include "board.h"

#include "adc.h"
#include "meter.h"

#include "spi.h"
#include "gpio.h"
#include "i2c.h"
#include "dma.h"
#include "tmr.h"
#include "lcd.h"
#include "lp.h"

static const gpio_cfg_t s_gpio_cfg_lcd_disp =
{
    .port = PORT_0,
    .mask = PIN_2,
    .func = GPIO_FUNC_OUT,
    .pad = GPIO_PAD_NONE
};

static const gpio_cfg_t s_gpio_cfg_button =
{
    .port = PORT_0,
    .mask = PIN_4,
    .func = GPIO_FUNC_IN,
    .pad = GPIO_PAD_PULL_UP
};

static int32_t s_spi_dma;
static board_i2c_cb_t s_p_board_i2c_cb;

static bool s_button;

bool board_button_event( void )
{
    NVIC_DisableIRQ(TMR1_IRQn);
    bool button = s_button;
    s_button = false;
    NVIC_EnableIRQ(TMR1_IRQn);
    return button;
}

static void button_isr( void *pv )
{
    TMR_Enable(MXC_TMR1);
    GPIO_IntClr(&s_gpio_cfg_button);
}

void board_init( void )
{
    const sys_cfg_tmr_t sys_cfg_tmr =
    {
        .out_en = 0
    };

    board_lcd_disp(false);
    LP_DisableBlockDetect();
    GPIO_Config( &s_gpio_cfg_lcd_disp );

    {
        static const tmr_cfg_t tmr_cfg =
        {
            .cmp_cnt = (2*HIRC96_FREQ/4096/100),
            .mode = TMR_MODE_ONESHOT
        };

        TMR_Init(MXC_TMR1,TMR_PRES_4096,&sys_cfg_tmr);
        TMR_Config(MXC_TMR1,&tmr_cfg);
        NVIC_EnableIRQ(TMR1_IRQn);
    }
    TMR_Init(MXC_TMR0,BOARD_SAMPLE_TIMER_PRESCALE_REG,&sys_cfg_tmr);

    {
        const tmr_cfg_t tmr_cfg =
        {
            .cmp_cnt = (2*HIRC96_FREQ/BOARD_SAMPLE_TIMER_PRESCALE/BOARD_ADC_SAMPLE_FREQ_HZ), // BOARD_ADC_SAMPLE_FREQ_HZ
            .mode = TMR_MODE_CONTINUOUS
        };
        TMR_Init(MXC_TMR0,BOARD_SAMPLE_TIMER_PRESCALE,&sys_cfg_tmr);
        TMR_Config(MXC_TMR0,&tmr_cfg);
    }

    board_enable_sample_timer(false);
    NVIC_EnableIRQ(TMR0_IRQn);

    DMA_Init();
    s_spi_dma = DMA_AcquireChannel();
    DMA_ConfigChannel( s_spi_dma, DMA_PRIO_HIGH, DMA_REQSEL_SPI0TX, 0, DMA_TIMEOUT_512_CLK,
                       DMA_PRESCALE_DISABLE, DMA_WIDTH_WORD, 1, DMA_WIDTH_WORD, 0, MXC_SPI17Y_FIFO_DEPTH>>1, 0, 1 );

    DMA_EnableInterrupt(s_spi_dma);
    NVIC_EnableIRQ(DMA0_IRQn);
    // LCD is connected to SPI0A
    if( SPI_Init( SPI0A, 0, 1000000 ) != E_NO_ERROR )
    {
        while( 1 ); // fatal
    }
    MXC_SPI17Y->ctrl2 |= 8 << MXC_F_SPI17Y_CTRL2_NUMBITS_POS | MXC_S_SPI17Y_CTRL2_SS_POL_SS0_HIGH | MXC_F_SPI17Y_CTRL2_THREE_WIRE;
    MXC_SPI17Y->ctrl0 |= MXC_V_SPI17Y_CTRL0_SS_SS0 | MXC_F_SPI17Y_CTRL0_MASTER | MXC_S_SPI17Y_CTRL0_SS_SS0;
    MXC_SPI17Y->dma = MXC_S_SPI17Y_DMA_TX_DMA_EN_EN | MXC_F_SPI17Y_DMA_TX_FIFO_EN | (MXC_SPI17Y_FIFO_DEPTH>>1) << MXC_F_SPI17Y_DMA_TX_FIFO_LEVEL_POS;

    const sys_cfg_i2c_t sys_i2c_cfg = NULL;
    if(I2C_Init(MXC_I2C0, I2C_STD_MODE, &sys_i2c_cfg) != E_NO_ERROR )
    {
        while( 1 ); // fatal
    }

    GPIO_Config( &s_gpio_cfg_button );
    GPIO_IntConfig( &s_gpio_cfg_button, GPIO_INT_EDGE, GPIO_INT_FALLING );
    GPIO_RegisterCallback( &s_gpio_cfg_button, button_isr, NULL );
    GPIO_IntClr( &s_gpio_cfg_button );
    GPIO_IntEnable( &s_gpio_cfg_button );
    NVIC_ClearPendingIRQ(GPIO0_IRQn);
    NVIC_EnableIRQ(GPIO0_IRQn);

    NVIC_EnableIRQ(I2C0_IRQn);
}

void board_enable_sample_timer( bool enable )
{
    if( enable )
        TMR_Enable( MXC_TMR0 );
    else
    {
        TMR_Disable( MXC_TMR0 );
        TMR_SetCount( MXC_TMR0, (2*HIRC96_FREQ/BOARD_SAMPLE_TIMER_PRESCALE/BOARD_ADC_SAMPLE_FREQ_HZ) );
    }
}

void board_i2c_isr_lock( void )
{
    NVIC_DisableIRQ(I2C0_IRQn);
}

void board_i2c_isr_unlock( void )
{
    NVIC_EnableIRQ(I2C0_IRQn);
}

void board_adc_timer_isr_lock(void)
{
    NVIC_DisableIRQ(TMR0_IRQn);
}

void board_adc_timer_isr_unlock(void)
{
    NVIC_EnableIRQ(TMR0_IRQn);
}


void i2c_cb( i2c_req_t * p_req, int err )
{
    while( err ); // fatal
    s_p_board_i2c_cb();
}

void board_i2c_read( uint8_t i2c_addr, void *pv_data, uint32_t length, bool restart, board_i2c_cb_t p_board_i2c_cb )
{
    static i2c_req_t req =
    {
        .callback = i2c_cb,
        .state = I2C_STATE_READING
    };
    if( p_board_i2c_cb )
    {
        req.addr = i2c_addr;
        req.restart = restart;
        req.rx_data = pv_data;
        req.rx_len = length;
        s_p_board_i2c_cb = p_board_i2c_cb;
        if( I2C_MasterAsync(MXC_I2C0, &req) != E_NO_ERROR )
            while(1);
    }
    else
    {
        while( I2C_MasterRead( MXC_I2C0, i2c_addr, pv_data, length, restart ) != length );
    }
}

void board_i2c_write( uint8_t i2c_addr, const void *pv_data, uint32_t length, bool restart )
{
    while( I2C_MasterWrite( MXC_I2C0, i2c_addr, pv_data, length, restart ) != length );
}

void board_lcd_disp( bool on )
{
    if( on )
    {
        GPIO_OutSet( &s_gpio_cfg_lcd_disp );
        //TMR_Enable(MXC_TMR0);
    }
    else
    {
        TMR_Disable(MXC_TMR0);
        GPIO_OutClr( &s_gpio_cfg_lcd_disp );
    }
}

void board_spi_write32( const void *pv_data, uint32_t length )
{
    // length is in bytes
    // pv_data must be 32-bit aligned

    MXC_SPI17Y->ctrl1 = length;
    DMA_SetSrcDstCnt( s_spi_dma, (void*)pv_data, 0, length );
    DMA_Start(s_spi_dma);
    MXC_SPI17Y->ctrl0 |= MXC_S_SPI17Y_CTRL0_START_START;
}


void I2C0_IRQHandler( void )
{
    I2C_Handler(MXC_I2C0);
}

void DMA0_IRQHandler( void )
{
    lcd_spi_write_complete_isr();
    DMA_ClearFlags(s_spi_dma);
}

void TMR0_IRQHandler( void )
{
    NVIC_DisableIRQ(TMR0_IRQn);
    adc_sample_timer_isr();
    TMR_IntClear(MXC_TMR0);
}

void TMR1_IRQHandler( void )
{
    s_button = !GPIO_InGet(&s_gpio_cfg_button);
    TMR_IntClear(MXC_TMR1);
}

void GPIO0_IRQHandler(void)
{
    GPIO_Handler(0);
}

void board_sleep( void )
{

//  LP_EnterSleepMode();
}

