#include <stdio.h>
#include "ydlidar_tminiplus_stm32.h"

YDLIDAR_TminiPlusSTM32::YDLIDAR_TminiPlusSTM32(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
	: m_huart(huart),
	  m_hdma(hdma),
	  m_current_uart_buff(0),
	  m_uart_buff1_ready(false),
	  m_uart_buff2_ready(false)
{
}


// _______________________________________________________
void YDLIDAR_TminiPlusSTM32::init()
{
    // fixe la fréquence de rafraichissement au maximum (à 12Hz)
    set_max_scan_frequency();
    HAL_Delay(30);

    init_reconstitution();  // S'assure que la machine d'étatde reconstition démarre bien


    // Lance les mesures
    //start_measures();

}

// ____________________________________________________________
/*!
 * \brief YDLIDAR_TminiPlusBase::start_measures
 */
void YDLIDAR_TminiPlusSTM32::start_measures()
{
	YDLIDAR_TminiPlusBase::start_measures();

	// Configure la réception DMA sur le buffer1 pour commencer
	m_current_uart_buff = 1;
	HAL_UART_Receive_DMA(m_huart, m_uart_irq_rxbuff1, UART_RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(m_hdma, DMA_IT_HT);  // supprime l'IRQ de buffer à moitié plein (il ne restera que l'IRQ de buffer rempli)
}

// _______________________________________________________
/*! \brief Configure le scan frequency à la fréquence maximum (12Hz)
 * Envoie plusieurs requêtes successives "Plus1"
 *
 */

void YDLIDAR_TminiPlusSTM32::set_max_scan_frequency()
{
	for (int i=0; i<10; i++) {
		scan_P1();
		HAL_Delay(10);
	}
}

// _______________________________________________________
void YDLIDAR_TminiPlusSTM32::irq_dma()
{
	if (m_current_uart_buff == 1) {
		m_uart_buff1_ready = 1;
		m_current_uart_buff = 2;
		HAL_UART_Receive_DMA(m_huart, m_uart_irq_rxbuff2, UART_RX_BUFFER_SIZE);
	}
	else {
		m_uart_buff2_ready = 1;
		m_current_uart_buff = 1;
		HAL_UART_Receive_DMA(m_huart, m_uart_irq_rxbuff1, UART_RX_BUFFER_SIZE);
	}

    __HAL_DMA_DISABLE_IT(m_hdma, DMA_IT_HT);  // supprime l'IRQ de buffer à moitié plein (il ne restera que l'IRQ de buffer rempli)
}


// _______________________________________________________
/*!
 * \brief Vérifie si un buffer est disponibleet si oui, reconstitue les paquets
 */
void YDLIDAR_TminiPlusSTM32::check_and_compute_buffers()
{
    if (m_uart_buff1_ready) {
    	HAL_GPIO_WritePin(Mot1_Sens1_GPIO_Port, Mot1_Sens1_Pin, GPIO_PIN_SET);
    	m_uart_buff1_ready = 0;
    	reconstitution(m_uart_irq_rxbuff1, UART_RX_BUFFER_SIZE);
    	HAL_GPIO_WritePin(Mot1_Sens1_GPIO_Port, Mot1_Sens1_Pin, GPIO_PIN_RESET);
    }

    if (m_uart_buff2_ready) {
    	HAL_GPIO_WritePin(Mot1_Sens1_GPIO_Port, Mot1_Sens1_Pin, GPIO_PIN_SET);
    	m_uart_buff2_ready = 0;
    	reconstitution(m_uart_irq_rxbuff2, UART_RX_BUFFER_SIZE);
    	HAL_GPIO_WritePin(Mot1_Sens1_GPIO_Port, Mot1_Sens1_Pin, GPIO_PIN_RESET);
    }
}

// _______________________________________________________
/*!
 * \brief Ecrit des données sur l'UART
 * \param buff le buffer de donnée à transmettre
 * \param len la taille du buffer à transmettre
 */
bool YDLIDAR_TminiPlusSTM32::write_serial(const char buff[], unsigned long len)
{
	if (!m_huart) return false;
	HAL_StatusTypeDef status = HAL_UART_Transmit(m_huart, (uint8_t*)buff, len, 500);
	return (status == HAL_OK);
}


// _______________________________________________________
/*!
 * \brief Lit des données sur l'UART (bloquant)
 * \param buff le buffer de donnée à transmettre
 * \param len la taille du buffer à transmettre
 */
bool YDLIDAR_TminiPlusSTM32::read_serial(const char buff[], unsigned long len)
{
	if (!m_huart) return false;
	HAL_UART_AbortReceive(m_huart); // purge les données restées non traitées dans le buffer de réception
	HAL_StatusTypeDef status = HAL_UART_Receive(m_huart, (uint8_t*)buff, len, 500);
	return (status == HAL_OK);
}

// _______________________________________________________
/*!
 *
 */
void YDLIDAR_TminiPlusSTM32::new_packet()
{
	static unsigned long old_last_cycle_time;
    if (isFirstPacket(&m_packet)) {
        m_current_index = 0;
        m_current_lidar_data.m_start_angle = firstAngle(&m_packet);
    }

    for (unsigned int i=0; i<m_packet.packet_len; i++) {
        unsigned int dist = dataindex2Distance(&m_packet, i);
        m_current_lidar_data.m_dist_measures[m_current_index] = dist;
        if (m_current_index < m_current_lidar_data.MAX_MEASURES_COUNT) m_current_index++;
    }

    if (isLastPacketOfCycle(&m_packet)) {
        m_current_lidar_data.m_angle_step_resolution = 360./m_data_count_in_cycle;
        m_current_lidar_data.m_measures_count = m_data_count_in_cycle;
        if (m_current_lidar_data.m_measures_count <=  m_current_lidar_data.MAX_MEASURES_COUNT ) {
            //emit new_data(m_current_lidar_data);
        	// TODO : filtrage
        	//printf("%d\n\r", HAL_GetTick());
        	printf("DeltaT : %d\n\r", HAL_GetTick() - old_last_cycle_time);
        	old_last_cycle_time = HAL_GetTick();
        }
        // else : il y a un problème dans le transfert, le cycle doit être ignoré car corrompu
    }

#if 0
    if (0) {
        //qDebug() << "-------";
        //qDebug() << "First Angle" << firstAngle(&m_packet);
        //qDebug() << "Last Angle" << lastAngle(&m_packet);
        //qDebug() << "Angle count" << m_packet.packet_len;
        //if (/*m_packet.packet_len>1*/1) qDebug() << "Resolution d'angle" << diffAngles(firstAngle(&m_packet), lastAngle(&m_packet)) / (m_packet.packet_len-1);
        qDebug() << diffAngles(firstAngle(&m_packet), lastAngle(&m_packet)) / (m_packet.packet_len-1);
        CLidarData data;
        data.m_start_angle = firstAngle(&m_packet)-90;
        data.m_measures_count = m_packet.packet_len;
        data.m_angle_step_resolution = diffAngles(firstAngle(&m_packet), lastAngle(&m_packet)) / (m_packet.packet_len-1);
        for (unsigned int i=0; i<m_packet.packet_len; i++) {
            float angle = dataindex2Angle(&m_packet, i);
            unsigned int dist = dataindex2Distance(&m_packet, i);
            bool valid = isDistanceValid(&m_packet, i);
            //if ( (dist < 100) && (dist!=0)) qDebug() << QString("Angle=%1 / Distance=%2 / Valid=%3").arg(angle).arg(dist).arg(valid);
            //if ( (angle>0) && (angle<15)) {
                qDebug() << QString("Angle=%1 / Distance=%2 / Valid=%3").arg(angle).arg(dist).arg(valid);
                data.m_dist_measures[i] = dist;
            //}
            if(0) {
                unsigned int __index=3*i;
                qDebug() << m_packet.data[__index] << m_packet.data[__index+1] << m_packet.data[__index+2];
            }
        } // for
        emit new_data(data);
    }
#endif
}



