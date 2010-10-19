/* Deviation = 1.586914 */
/* Base frequency = 433.919830 */
/* Carrier frequency = 433.919830 */
/* Channel number = 0 */
/* Carrier frequency = 433.919830 */
/* Modulation format = ASK/OOK */
/* Manchester enable = false */
/* Sync word qualifier mode = No preamble/sync */
/* Preamble count = 2 */
/* Channel spacing = 25.390625 */
/* Carrier frequency = 433.919830 */
/* Data rate = 2.39897 */
/* RX filter BW = 464.285714 */
/* Data format = Normal mode */
/* Length config = Variable packet length mode. Packet length configured by the first byte after sync word */
/* CRC enable = true */
/* Packet length = 255 */
/* Device address = 0 */
/* Address config = No address check */
/* CRC autoflush = false */
/* PA ramping = false */
/* TX power = 10 */
/***************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings specifed with C-code
 *  compatible #define statements.
 *
 ***************************************************************/

#ifndef GATE_CC430_H
#define GATE_CC430_H

#define GATE_RADIO_CC430

#define GATE_SETTING_IOCFG2        0x29
#define GATE_SETTING_IOCFG1        0x2E
#define GATE_SETTING_IOCFG0        0x06
#define GATE_SETTING_FIFOTHR       0x07
#define GATE_SETTING_SYNC1         0xD3
#define GATE_SETTING_SYNC0         0x91
#define GATE_SETTING_PKTLEN        0x05
#define GATE_SETTING_PKTCTRL1      0x00
#define GATE_SETTING_PKTCTRL0      0x00
#define GATE_SETTING_ADDR          0x00
#define GATE_SETTING_CHANNR        0x00
#define GATE_SETTING_FSCTRL1       0x0C
#define GATE_SETTING_FSCTRL0       0x00
#define GATE_SETTING_FREQ2         0x10
#define GATE_SETTING_FREQ1         0xB0
#define GATE_SETTING_FREQ0         0x71
#define GATE_SETTING_MDMCFG4       0x36
#define GATE_SETTING_MDMCFG3       0xC3
#define GATE_SETTING_MDMCFG2       0x30
#define GATE_SETTING_MDMCFG1       0x00
#define GATE_SETTING_MDMCFG0       0x00
#define GATE_SETTING_DEVIATN       0x00
#define GATE_SETTING_MCSM2         0x07
#define GATE_SETTING_MCSM1         0x30
#define GATE_SETTING_MCSM0         0x10
#define GATE_SETTING_FOCCFG        0x1D
#define GATE_SETTING_BSCFG         0x1C
#define GATE_SETTING_AGCCTRL2      0xC7
#define GATE_SETTING_AGCCTRL1      0x00
#define GATE_SETTING_AGCCTRL0      0xB0
#define GATE_SETTING_WOREVT1       0x80
#define GATE_SETTING_WOREVT0       0x00
#define GATE_SETTING_WORCTRL       0xFB
#define GATE_SETTING_FREND1        0xB6
#define GATE_SETTING_FREND0        0x11
#define GATE_SETTING_FSCAL3        0xE9
#define GATE_SETTING_FSCAL2        0x2A
#define GATE_SETTING_FSCAL1        0x00
#define GATE_SETTING_FSCAL0        0x1F
#define GATE_SETTING_FSTEST        0x59
#define GATE_SETTING_PTEST         0x7F
#define GATE_SETTING_AGCTEST       0x3F
#define GATE_SETTING_TEST2         0x88
#define GATE_SETTING_TEST1         0x31
#define GATE_SETTING_TEST0         0x09
#define GATE_SETTING_PARTNUM       0x00
#define GATE_SETTING_VERSION       0x06
#define GATE_SETTING_FREQEST       0x00
#define GATE_SETTING_LQI           0x00
#define GATE_SETTING_RSSI          0x00
#define GATE_SETTING_MARCSTATE     0x00
#define GATE_SETTING_WORTIME1      0x00
#define GATE_SETTING_WORTIME0      0x00
#define GATE_SETTING_PKTSTATUS     0x00
#define GATE_SETTING_VCO_VC_DAC    0x00
#define GATE_SETTING_TXBYTES       0x00
#define GATE_SETTING_RXBYTES       0x00
#define GATE_SETTING_RF1AIFCTL0    0x00
#define GATE_SETTING_RF1AIFCTL1    0x00
#define GATE_SETTING_RF1AIFCTL2    0x00
#define GATE_SETTING_RF1AIFERR     0x00
#define GATE_SETTING_RF1AIFERRV    0x00
#define GATE_SETTING_RF1AIFIV      0x00
#define GATE_SETTING_RF1AINSTRW    0x00
#define GATE_SETTING_RF1AINSTR1W   0x00
#define GATE_SETTING_RF1AINSTR2W   0x00
#define GATE_SETTING_RF1ADINW      0x00
#define GATE_SETTING_RF1ASTAT0W    0x00
#define GATE_SETTING_RF1ASTAT1W    0x00
#define GATE_SETTING_RF1ASTAT2W    0x00
#define GATE_SETTING_RF1ADOUT0W    0x00
#define GATE_SETTING_RF1ADOUT1W    0x00
#define GATE_SETTING_RF1ADOUT2W    0x00
#define GATE_SETTING_RF1AIN        0x00
#define GATE_SETTING_RF1AIFG       0x00
#define GATE_SETTING_RF1AIES       0x00
#define GATE_SETTING_RF1AIE        0x00
#define GATE_SETTING_RF1AIV        0x00
#define GATE_SETTING_RF1ARXFIFO    0x00
#define GATE_SETTING_RF1ATXFIFO    0x00

void sx_gate (u8 line);
void mx_gate (u8 line);
void display_gate (u8 line, u8 update);



#endif
