#ifndef __STM_EEPROM_H
#define __STM_EEPROM_H



void stm_write_eerpom(uint16_t address,uint32_t data);
void stm_fast_write_eerpom(uint8_t address,uint32_t data);
void stm_read_eerpom(uint32_t address,uint32_t *data);



#define CUURENT_FLASH_ADDRER 0x00   /*当前存储的总目*/
#define CUURENT_BUUFER_INDEX_ADDRER 0x01  /*当前BUFFER idex*/

#define HOME_WEIDU_ADDER  0x02
#define HOME_WEIDU_FLAG_ADDER  0x03
#define HOME_JINDU_ADDER  0x04




#endif



