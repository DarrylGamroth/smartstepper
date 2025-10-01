/*
 * Example usage of STM32 SPI Timer-Triggered DMA for FOC motor control
 * 
 * This example shows how to integrate the timer-triggered SPI DMA with 
 * the PWM timer for synchronized encoder reading in FOC applications.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pwm.h>
#include "spi_ll_stm32_tt.h"

/* Device tree references */
#define SPI_NODE DT_NODELABEL(spi1)
#define PWM_NODE DT_NODELABEL(pwm_timer1)

/* 
 * DMA configuration is in devicetree, e.g.:
 * &spi1 {
 *     dmas = <&dma2 2 TIM1_UP>; // Use timer trigger instead of SPI_RX
 *     dma-names = "rx";
 * };
 */

/* Encoder buffer configuration */
#define ENCODER_BUFFER_SIZE 128
static uint8_t encoder_tx_buffer[ENCODER_BUFFER_SIZE] __aligned(32);
static uint8_t encoder_rx_buffer[ENCODER_BUFFER_SIZE] __aligned(32);

/* SPI buffer structures */
static const struct spi_buf encoder_tx_spi_buf = {
	.buf = encoder_tx_buffer,
	.len = ENCODER_BUFFER_SIZE
};

static const struct spi_buf encoder_rx_spi_buf = {
	.buf = encoder_rx_buffer,
	.len = ENCODER_BUFFER_SIZE
};

/* SPI configuration for encoder reading */
static struct spi_config encoder_spi_config = {
	.frequency = 1000000,  /* 1 MHz */
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	.slave = 0,
	.cs = NULL,  /* Using hardware CS */
};

static const struct device *spi_dev;
static const struct device *pwm_dev;

/**
 * @brief Initialize encoder command buffer
 */
void init_encoder_commands(void)
{
	/* Example: Fill TX buffer with encoder read commands */
	/* This depends on your specific encoder protocol */
	
	/* Example for a typical SPI encoder: */
	for (int i = 0; i < ENCODER_BUFFER_SIZE; i += 4) {
		encoder_tx_buffer[i + 0] = 0xFF;  /* Read position command */
		encoder_tx_buffer[i + 1] = 0x00;  /* Address/register */
		encoder_tx_buffer[i + 2] = 0x00;  /* Padding/checksum */
		encoder_tx_buffer[i + 3] = 0x00;  /* Padding */
	}
}

/**
 * @brief Initialize timer-triggered SPI for encoder reading
 */
int init_encoder_spi(void)
{
	int ret;
	
	/* Initialize encoder command buffer */
	init_encoder_commands();
	
	/* Get device references */
	spi_dev = DEVICE_DT_GET(SPI_NODE);
	if (!device_is_ready(spi_dev)) {
		return -ENODEV;
	}
	
	pwm_dev = DEVICE_DT_GET(PWM_NODE);
	if (!device_is_ready(pwm_dev)) {
		return -ENODEV;
	}
	
	/* Setup timer-triggered DMA for encoder reading */
	ret = spi_stm32_setup_timer_dma(spi_dev, 
					&encoder_spi_config,
					&encoder_tx_spi_buf,
					&encoder_rx_spi_buf);
	if (ret) {
		return ret;
	}
	
	/* Configure PWM timer to trigger SPI at desired frequency */
	/* The PWM driver should already be configured to generate DMA triggers */
	
	return 0;
}

/**
 * @brief Start synchronized encoder reading with PWM
 */
int start_encoder_reading(void)
{
	int ret;
	
	/* Start timer-triggered SPI DMA */
	ret = spi_stm32_start_timer_dma(spi_dev);
	if (ret) {
		return ret;
	}
	
	/* PWM timer will now trigger SPI transfers automatically */
	/* Each PWM cycle will read encoder data into the circular buffer */
	
	return 0;
}

/**
 * @brief Stop synchronized encoder reading
 */
int stop_encoder_reading(void)
{
	return spi_stm32_stop_timer_dma(spi_dev);
}

/**
 * @brief Get pointer to latest encoder data buffer
 */
const uint8_t *get_encoder_data(void)
{
	if (!spi_stm32_is_timer_dma_active(spi_dev)) {
		return NULL;
	}
	
	/* Check if we have fresh data available */
	if (!spi_tt_is_complete(spi_dev)) {
		return NULL;  /* DMA transfer in progress, no fresh data */
	}
	
	/* DMA transfer is complete, return pointer to buffer */
	return encoder_rx_buffer;
}

/**
 * @brief FOC control loop integration example
 * 
 * This would typically be called from the ADC conversion complete ISR
 * after all motor phase currents have been sampled.
 */
void foc_adc_isr_handler(void)
{
	/* Check if fresh encoder data is available */
	if (spi_tt_is_complete(spi_dev)) {
		const uint8_t *encoder_data = get_encoder_data();
		
		if (encoder_data != NULL) {
			/* Process encoder data for position/velocity feedback */
			uint32_t position = (encoder_data[0] << 24) | 
					   (encoder_data[1] << 16) | 
					   (encoder_data[2] << 8) | 
					   encoder_data[3];
			
			/* Use position for FOC calculations and update PWM duty cycles */
			foc_control_update(position);
		}
	}
}

/**
 * @brief FOC control loop integration example (legacy function for reference)
 */
void foc_control_loop(void)
{
	/* This function would be called at the PWM frequency */
	/* The encoder data is already captured by timer-triggered DMA */
	
	const uint8_t *encoder_data = get_encoder_data();
	if (encoder_data != NULL) {
		/* Process encoder data for position/velocity feedback */
		/* Update PWM duty cycles for next cycle */
		
		/* Example: extract position from encoder data */
		uint32_t position = (encoder_data[0] << 24) | 
				   (encoder_data[1] << 16) | 
				   (encoder_data[2] << 8) | 
				   encoder_data[3];
		
		/* Use position for FOC calculations... */
	}
}
