#ifndef _YDLIDAR_TMINIPLUS_STM32_H_
#define _YDLIDAR_TMINIPLUS_STM32_H_

#include "main.h"
#include "ydlidar_tminiplus_base.h"
#include "lidar_data.h"

class YDLIDAR_TminiPlusSTM32 : public YDLIDAR_TminiPlusBase
{
public:
    YDLIDAR_TminiPlusSTM32(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

    /*virtual*/void start_measures() override;

    void init();
    void set_max_scan_frequency();
    void irq_dma();
    void check_and_compute_buffers();

protected :
    UART_HandleTypeDef *m_huart;
    DMA_HandleTypeDef *m_hdma;

    // Réimpléméntation des méthodes de la classe de base YDLIDAR_TminiPlusBase
    void new_packet() override;
    /*virtual*/bool write_serial(const char buff[], unsigned long len) override;
    /*virtual*/bool read_serial(const char buff[], unsigned long len) override;

private :
    CLidarData m_current_lidar_data;
    int m_current_index;

    static const unsigned int UART_RX_BUFFER_SIZE = 140;

    uint8_t m_uart_irq_rxbuff1[UART_RX_BUFFER_SIZE];
    uint8_t m_uart_irq_rxbuff2[UART_RX_BUFFER_SIZE];
    uint8_t m_current_uart_buff;
    bool m_uart_buff1_ready;
    bool m_uart_buff2_ready;
};

#endif // _YDLIDAR_TMINIPLUS_STM32_H_
