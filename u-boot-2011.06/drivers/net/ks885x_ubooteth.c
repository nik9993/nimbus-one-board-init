/*
 *
 * Copyright (C) 2007-2009 Micrel, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "ks885x_glb.h"
/*
    ks885xBoardHwInit

    Description:
         Initialize the related hardware for KS8851 chip, such as extern interrupt configuration,
                          GPIO settings or SPI controller initialization.

    PKS885XHW pks885xHW
            Pointer to hardware instance.
 
    Return : return KS885XERR_NOERROR if successful.
*/
KS885XERR ks885xBoardHwInit(PKS885XHW pks885xHW)
{
       //SPI hardware dependent. call SPIControllerInit in the spi.c.

       return 0;

}

/*
    ks885xBoardHwRelease

    Description:
         Release the related resources and hardware for KS8851 chip.

    PKS885XHW pks885xHW
            Pointer to hardware instance.
  
   Return : return KS885XERR_NOERROR if successful.
*/
KS885XERR ks885xBoardHwRelease(PKS885XHW pks885xHW)
{
//Note: the pks885xHW may is NULL for global clean.
       return 0;

}

/*
    ks885xBoardHwGetIrq

    Description:
         Return a logic interrupt number for ks8851 chip. the number is totally depend on board design
         and configuration.
   
    PKS885XHW pks885xHW
            Pointer to hardware instance.
   
    Return : return a logic interrupt number .
*/
unsigned short ks885xBoardHwGetIrq(PKS885XHW pks885xHW)
{
       PRINTF("current interrut number=%d\n", CONFIG_KS8851_IRQ);
       return CONFIG_KS8851_IRQ;
}

/*
    ks885xBoardHwAckInt

    Description:
      Acknowledge the hardware interrupt (not acknowledge to KS8851 chip interrupt), which is totally
      depend on board design and configuration
   
    Parameters:
           int irq (IN)
    Return : (none).
*/
void ks885xBoardHwAckInt(PKS885XHW pks885xHW, int irq)
{

}

/*
    ks885xBoardHwEnableInt

    Description:
      Enable the hardware interrupt which is totally
      depend on board design and configuration
   
    Parameters:
           int irq (IN)
    Return : (none).
*/
void ks885xBoardHwEnableInt(PKS885XHW pks885xHW, int irq)
{

}

/*
    ks885xBoardHwDisableInt

    Description:
      Disable the hardware interrupt which is totally
      depend on board design and configuration
   
    Parameters:
           int irq (IN)
    Return : (none).
*/
void ks885xBoardHwDisableInt(PKS885XHW pks885xHW, int irq)
{

}                              ///////////////// following code come from target_mll.c //////////////////////////

//input address should be WORD boundary

/*
    HW_READ_WORD

    Description:
        This routine read a 16 bit data from ks8851 MLL chip

     Parameters:
        void *  phw
            Pointer to ks885x hardware instance.
       
        unsigned short addr
            The address to be read, this address should be in 16 bit boundary, which is (0x0000,0x0002,0x0004, 0x0006,......)  
       
        unsigned short * data
                A  unsigned short pointer to hold read back 16 bit data.
    Return: none
*/
void HW_READ_WORD(void *phw, unsigned short addr, unsigned short *data)
{
       KS885XHW *pks885xHW = (KS885XHW *) phw;
       MIO_WORD((pks885xHW)->m_ulIoVirtualAddr + CMD_HIGH) =
           (unsigned short)(addr) | ((BE1 | BE0) << (addr & 0x02));
       *(data) = MIO_WORD((pks885xHW)->m_ulIoVirtualAddr + CMD_LOW);
}

/*
    HW_WRITE_WORD

    Description:
        This routine write a 16 bit data to ks8851 MLL chip

     Parameters:
        void *  phw
            Pointer to ks885x hardware instance.
       
        unsigned short addr
            The address to be read, this address should be in 16 bit boundary, which is (0x0000,0x0002,0x0004, 0x0006,......)  
       
        unsigned short * data
                The 16 bit data value to write to ks8851 chip.
    Return: none
*/
void HW_WRITE_WORD(void *phw, unsigned short addr, unsigned short data)
{
       KS885XHW *pks885xHW = (KS885XHW *) phw;
       MIO_WORD((pks885xHW)->m_ulIoVirtualAddr + CMD_HIGH) =
           (unsigned short)(addr) | ((BE1 | BE0) << (addr & 0x02));
       MIO_WORD((pks885xHW)->m_ulIoVirtualAddr + CMD_LOW) =
           (unsigned short)(data);
}

void HW_READ_16BITDATA(void *phw, unsigned short *data)
{
       KS885XHW *pks885xHW = (KS885XHW *) phw;
       *(data) =
           MIO_WORD(((PKS885XHW) (pks885xHW))->m_ulIoVirtualAddr + CMD_LOW);
}

void HW_WRITE_16BITDATA(void *phw, unsigned short data)
{
       KS885XHW *pks885xHW = (KS885XHW *) phw;
       MIO_WORD(((PKS885XHW) (pks885xHW))->m_ulIoVirtualAddr + CMD_LOW) =
           (unsigned short)(data);
}

////////////// following code come from ks885x_target.c  /////////////////////////////
//unsigned char KS885X_DEFAULT_MAC_ADDRESS[] = { 0x00, 0x22, 0xA1, 0xAA, 0x10, 0x0C };
unsigned char KS885X_DEFAULT_MAC_ADDRESS[] =
    { 0x00, 0x10, 0xA1, 0x88, 0x95, 0x11 };

/*
    ks885xRestartAutoNego

    Description:
        This routine restarts PHY auto-negotiation.

    Parameters: None

    Return:
       autoNegoStatus : 1-> auto-nego completed; 0-> auto-nego not completed.
*/

int ks885xRestartAutoNego(PKS885XHW pks885xHW)
{
       unsigned short RegData;
       int cnTimeOut;

       /* Set restart auto-negitiation to Port Control Register (P1CR) */
       KS885X_REG_READ(pks885xHW, REG_PORT_CTRL, &RegData);
       RegData &= ~(PORT_FORCE_100_MBIT | PORT_FORCE_FULL_DUPLEX);
       RegData |= (PORT_AUTO_NEG_ENABLE | PORT_AUTO_NEG_RESTART);
       KS885X_REG_WRITE(pks885xHW, REG_PORT_CTRL, RegData);
       /* Wait for auto-negotiation completed (P1SR) */
       cnTimeOut = PHY_RESET_TIMEOUT;
       do {
               UDELAY(100);
               KS885X_REG_READ(pks885xHW, REG_PORT_STATUS, &RegData);
       } while (--cnTimeOut && !(RegData & PORT_AUTO_NEG_COMPLETE));

       return ((RegData & PORT_AUTO_NEG_COMPLETE) >> 6);

}                              /* HardwareRestartAutoNego */

/*
    ks885xAcknowledgeInterrupt

    Description:
        This routine acknowledges the specified interrupts.

    Parameters:
        PKS885XHW pks885xHW
            Pointer to ks885x hardware instance.

        unsigned long   ulInterrupt
            The interrupt masks to be acknowledged.

    Return (None):
*/

void ks885xAcknowledgeInterrupt(PKS885XHW pks885xHW, unsigned long ulInterrupt)
{
       //unsigned short w;
       //w=(unsigned short)ulInterrupt&pks885xHW->m_wInterruptMask;
       /* Write 1 to clear interrupt source in Interrupt Status register (ISR) */
       KS885X_REG_WRITE(pks885xHW, REG_INT_STATUS, (unsigned short)ulInterrupt);       // w );

}                              /* HardwareAcknowledgeInterrupt */

/*
    ks885xAllocPacket

    Description:
        This function allocates a packet for transmission.

    Parameters:
        PKS885XHW pks885xHW
            Pointer to hardware instance.

        int length
            The length of the packet.

    Return (int):
        FALSE if not successful; TRUE for buffer copy; otherwise, can use descriptors.
*/

BOOL ks885xAllocPacket(PKS885XHW pks885xHW, int PacketLength)
{

       unsigned short wStatus;
       KS885X_REG_READ(pks885xHW, REG_TXQ_CMD, &wStatus);
       if (wStatus & TXQ_ENQUEUE) {

               PRINTF("last data has not finished send yet\n");

               return FALSE;
       }
       /* Checking ks8851 TXQ memory available for this packet */
       KS885X_REG_READ(pks885xHW, REG_TX_MEM_INFO, &wStatus);

       if ((wStatus & TX_MEM_AVAILABLE_MASK) < (int)(PacketLength + 4)) {
               PRINTF
                   ("No enough ks8851 TXQ memory for size=%d data, send failed \r\n",
                    PacketLength);
               return FALSE;
       } else
               return TRUE;

}                              /* ks885xAllocPacket */

/*
    ks885xEnableWolFrame

    Description:
        This routine is used to enable the Wake-on-LAN that wake-up signal is caused by
        receipting of a 'wake-up' packet. The device can support four different
        'wake-up' frames.
        KS8851 device can support D1, D2, or D3 power state by EEPROM setting.
        By default, device supports D3 power state without EEPROM setting.
        The example here is by default D3 power state.

    Parameters:
        PKS885XHW pks885xHW
            Pointer to hardware instance.

        UINT32    dwFrame
            whose wake up frame to be enabled (0 - Frame 0, 1 - Frame 1, 2 - Frame 2, 3 - Frame 3).

    Return (None):
*/

void ks885xEnableWolFrame(PKS885XHW pks885xHW, unsigned long dwFrame)
{
       unsigned short RegData;
/* Enables the wake up frame pattern detection */
       RegData = 0;
       KS885X_REG_READ(pks885xHW, REG_WOL_CTRL, &RegData);

       switch (dwFrame) {
       case 0:
               RegData |= (WOL_FRAME0_ENABLE);
               break;
       case 1:
               RegData |= (WOL_FRAME1_ENABLE);
               break;
       case 2:
               RegData |= (WOL_FRAME2_ENABLE);
               break;
       case 3:
               RegData |= (WOL_FRAME3_ENABLE);
               break;
       }

       KS885X_REG_WRITE(pks885xHW, REG_WOL_CTRL, RegData);
}

void ks885xDisableWolFrame(PKS885XHW pks885xHW, unsigned long dwFrame)
{
       unsigned short RegData;
       RegData = 0;

       KS885X_REG_READ(pks885xHW, REG_WOL_CTRL, &RegData);

       switch (dwFrame) {
       case 0:
               RegData &= ~(WOL_FRAME0_ENABLE);
               break;
       case 1:
               RegData &= ~(WOL_FRAME1_ENABLE);
               break;
       case 2:
               RegData &= ~(WOL_FRAME2_ENABLE);
               break;
       case 3:
               RegData &= ~(WOL_FRAME3_ENABLE);
               break;
       }

       KS885X_REG_WRITE(pks885xHW, REG_WOL_CTRL, RegData);

}

/*
    ks885xEnableInterrupt

    Description:
        This routine enables the interrupts of the hardware.

    Parameters:
        PKS885XHW pks885xHW
            Pointer to hardware instance.

    Return (None):
*/

void ks885xEnableInterrupt(PKS885XHW pks885xHW)
{
       KS885X_REG_WRITE(pks885xHW, REG_INT_MASK, pks885xHW->m_wInterruptMask);
}                              /* ks885xEnableInterrupt */

/*
    ks885xDisableInterrupt

    Description:
        This routine enables the interrupts of the hardware.

    Parameters:
        PKS885XHW pks885xHW
            Pointer to hardware instance.

    Return (None):
*/

void ks885xDisableInterrupt(PKS885XHW pks885xHW)
{
       KS885X_REG_WRITE(pks885xHW, REG_INT_MASK, 0x0000);
}                              /* ks885xEnableInterrupt */

/*
    ks885xEnable

    Description:
        This routine enables the hardware.

    Parameters:
        PKS885XHW pks885xHW
            Pointer to hardware instance.

    Return (None):
*/

void ks885xEnable(PKS885XHW pks885xHW)
{
       unsigned short txCntl;
       unsigned short rxCntl;
       unsigned short w;
       PRINTF("ks885xEnable\n");

       KS885X_REG_READ(pks885xHW, REG_TX_CTRL, &txCntl);
       /* Enables QMU Transmit (TXCR). */
       txCntl |= TX_CTRL_ENABLE;
       KS885X_REG_WRITE(pks885xHW, REG_TX_CTRL, txCntl);

       //RX Frame Count Threshold Enable and Auto-Dequeue RXQ Frame Enable
       KS885X_REG_READ(pks885xHW, REG_RXQ_CMD, &w);
       w |= RXQ_FRAME_CNT_INT; //RXQ_CMD_CNTL;
       KS885X_REG_WRITE(pks885xHW, REG_RXQ_CMD, w);

       /* Enables QMU Receive (RXCR1). */
       KS885X_REG_READ(pks885xHW, REG_RX_CTRL1, &rxCntl);
       rxCntl |= RX_CTRL_ENABLE;
       KS885X_REG_WRITE(pks885xHW, REG_RX_CTRL1, rxCntl);
       pks885xHW->m_bEnabled = TRUE;

}                              /* ks885xEnable */

/*
    ks885xDisable

    Description:
        This routine disables the hardware.

    Parameters:
        PKS885XHW pks885xHW
            Pointer to hardware instance.

    Return (None):
*/

void ks885xDisable(PKS885XHW pks885xHW)
{
       unsigned short txCntl;
       unsigned short rxCntl;
       PRINTF("ks885xDisable\n");

       KS885X_REG_READ(pks885xHW, REG_TX_CTRL, &txCntl);

       /* Enables QMU Transmit (TXCR). */
       txCntl &= ~TX_CTRL_ENABLE;
       KS885X_REG_WRITE(pks885xHW, REG_TX_CTRL, txCntl);

       /* Enables QMU Receive (RXCR1). */
       KS885X_REG_READ(pks885xHW, REG_RX_CTRL1, &rxCntl);
       rxCntl &= ~RX_CTRL_ENABLE;
       KS885X_REG_WRITE(pks885xHW, REG_RX_CTRL1, rxCntl);

       pks885xHW->m_bEnabled = FALSE;

}                              /* ks885xDisable */

/*
    ks885xSetup

    Description:
        This routine setup KS8851 hardware

    Parameters:
        PKS885XHW pks885xHW
            Pointer to hardware instance.

    Return (none):
     
*/
void ks885xSetup(PKS885XHW pks885xHW)
{
       unsigned short txCntl, rxCntl, w;
       /*
        * Configure QMU Transmit
        */
       PRINTF("ks885xSetup\n");
       /* Setup Transmit Frame Data Pointer Auto-Increment (TXFDPR) */
       KS885X_REG_WRITE(pks885xHW, REG_TX_ADDR_PTR, ADDR_PTR_AUTO_INC);

       /* Setup Receive Frame Data Pointer Auto-Increment */
       KS885X_REG_WRITE(pks885xHW, REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC);

       /* Setup Receive Frame Threshold - 1 frame (RXFCTFC) */
       KS885X_REG_WRITE(pks885xHW, REG_RX_FRAME_CNT_THRES,
                        1 & RX_FRAME_THRESHOLD_MASK);

       /* Setup RxQ Command Control (RXQCR) */
       KS885X_REG_WRITE(pks885xHW, REG_RXQ_CMD, RXQ_CMD_CNTL);

       // set the force mode to half duplex, default is full duplex.
       //because if the auto-negotiation is fail, most switch will use half-duplex.
       KS885X_REG_READ(pks885xHW, REG_PHY_CNTL, &w);
       w &= ~PHY_FULL_DUPLEX;
       KS885X_REG_WRITE(pks885xHW, REG_PHY_CNTL, w);

       txCntl = (TX_CTRL_FLOW_ENABLE |
                 TX_CTRL_PAD_ENABLE |
                 TX_CTRL_CRC_ENABLE | TX_CTRL_IP_CHECKSUM);

       KS885X_REG_WRITE(pks885xHW, REG_TX_CTRL, txCntl);

       rxCntl = (RX_CTRL_MAC_FILTER |
                 RX_CTRL_FLOW_ENABLE |
                 RX_CTRL_BROADCAST | RX_CTRL_ALL_MULTICAST | RX_CTRL_UNICAST);
       KS885X_REG_WRITE(pks885xHW, REG_RX_CTRL1, rxCntl);

}                              /*ks885xSetup */

/* ks885xSetPower
  Description:
     KS8851 device support D1, D2, or D3 power state on hardware
 Parameters:
        PKS885XHW pks885xHW
            Pointer to hardware instance.
        unsigned short
            PM state Value
*/
void ks885xSetPower(PKS885XHW pks885xHW, unsigned short uPowerState)
{
       unsigned short RegData = 0;
       /* Set power management capabilities */

       KS885X_REG_READ(pks885xHW, REG_POWER_CNTL, &RegData);

       RegData &= POWER_STATE_MASK;
       RegData |= (POWER_PME_ENABLE | uPowerState);

       KS885X_REG_WRITE(pks885xHW, REG_POWER_CNTL, RegData);

}

void ks885xCleanTXQueue(PKS885XHW pks885xHW)
{
       unsigned short txCntl;
       KS885X_REG_READ(pks885xHW, REG_TX_CTRL, &txCntl);
       txCntl &= ~TX_CTRL_ENABLE;
       KS885X_REG_WRITE(pks885xHW, REG_TX_CTRL, txCntl);

       KS885X_REG_READ(pks885xHW, REG_TX_CTRL, &txCntl);
       txCntl |= 0x0010;       // clean the queues
       KS885X_REG_WRITE(pks885xHW, REG_TX_CTRL, txCntl);

       KS885X_REG_READ(pks885xHW, REG_TX_CTRL, &txCntl);
       txCntl &= ~0x0010;      //  restore the queues

       KS885X_REG_WRITE(pks885xHW, REG_TX_CTRL, txCntl);

       KS885X_REG_READ(pks885xHW, REG_TX_CTRL, &txCntl);
       txCntl |= TX_CTRL_ENABLE;
       KS885X_REG_WRITE(pks885xHW, REG_TX_CTRL, txCntl);
}

/*
    ks885xClearMulticast

    Description:
        This routine removes all multicast addresses set in the hardware.

    Parameters:
        PKS885XHW pks885xHW
            Pointer to hardware instance.

    Return (None):
*/
void ks885xClearMulticast(PKS885XHW pks885xHW)
{
       unsigned short i;
       for (i = 0; i < KS885X_MULTICAST_SIZE; i++)
               KS885X_REG_WRITE(pks885xHW, REG_MAC_HASH_0 + (2 * i), 0);

}

/*
    ks885xSetMulticast

    Description:
        This routine set all multicast addresses to the hardware.

    Parameters:
        PKS885XHW pks885xHW
            Pointer to hardware instance.

    Return (None):
*/
void ks885xSetMulticast(PKS885XHW pks885xHW, int Index, unsigned short wValue)
{
       if (Index > 3)          // index should from 0-2
               return;
       KS885X_REG_WRITE(pks885xHW, REG_MAC_HASH_0 + (2 * Index), wValue);

}

/*
    ks885xCheckLink

    Description:
        This routine reads PHY registers to determine the current link status.
        If link is up, it also return link speed in 'ulSpeed'
        and duplex mode in 'bDuplex'.

    Parameters:
        unsigned long    *pSpeed
            current link speed (100000: 10BASE-T; 1000000: 100BASE-T)

        unsigned long   *pDuplex;
            current link duplex mode (1: half duplex; 2: full duplex)

    Return:
       linkStatus : 1-> link is up; 0-> link is down.
*/

int ks885xCheckLink(PKS885XHW pks885xHW, unsigned long *pSpeed,
                   unsigned long *pDuplex)
{
       unsigned short RegData = 0;

       /* Read Port Status Register (P1SR) */
       KS885X_REG_READ(pks885xHW, REG_PORT_STATUS, &RegData);

       if ((RegData & PORT_STATUS_LINK_GOOD) == PORT_STATUS_LINK_GOOD) {
               *pSpeed = 100000;
               if ((RegData & PORT_STAT_SPEED_100MBIT) ==
                   PORT_STAT_SPEED_100MBIT)
                       *pSpeed = 1000000;

               *pDuplex = 1;
               if ((RegData & PORT_STAT_FULL_DUPLEX) == PORT_STAT_FULL_DUPLEX)
                       *pDuplex = 2;

               return (1);     /* Link is up */
       } else
               return (0);     /* Link is down */

}

/*
    ks885xSetupInterrupt

    Description:
        This routine setup the interrupt mask for proper operation.

    Parameters:
        PKS885XHW pks885xHW
            Pointer to hardware instance.

    Return (None):
*/

void ks885xSetupInterrupt(PKS885XHW pks885xHW)
{
       unsigned short intMask;
       // in the OALIntrInit function, all interrupt mode has been
       // set to normal IRQ, so we do not need to do it again.
       pks885xHW->m_wInterruptMask = 0x00;
       /* Clear the interrupts status of the hardware. */
       ks885xAcknowledgeInterrupt(pks885xHW, 0xFFFF);

       /* Enables the interrupts of the hardware. */

       intMask = (INT_PHY | INT_TX | INT_RX);

       pks885xHW->m_wInterruptMask = intMask;
       PRINTF("ks885xSetupInterrupt mask=0x%x\n", intMask);

       KS885X_REG_WRITE(pks885xHW, REG_INT_MASK, intMask);

}                              /* ks885xSetupInterrupt */

/*
    ks885xReadAckInt

    Description:
         Read KS8851 interrupt status register.

    Parameters:
        PKS885XHW pks885xHW (IN)
             Pointer the KS885XHW instance structure
            
    Return :
                               return the interrupt status register content.
*/
unsigned short ks885xReadInterrupt(PKS885XHW pks885xHW)
{
       unsigned short uStatus;
       KS885X_REG_READ(pks885xHW, REG_INT_STATUS, &uStatus);
       return uStatus;
}

/*
    ks885xReadSingleFrame

    Description:
         read one frame of data

    Parameters:
        PKS885XHW pks885xHW (IN)
             Pointer the KS885XHW instance structure
        FR_HEADER_INFO * (IN)
             Pointer to FR_HEADER_INFO data structure.
        unsigned char * pBuf (IN)
                    Pointer to a buffer to receive the data.
               unsigned short nBufSize (IN)
                    The buffer size.
            
    Return : If function is successful, return a pointer to data beginning, otherwise return NULL.
*/
static unsigned char *ks885xReadSingleFrame(PKS885XHW pks885xHW,
                                           FR_HEADER_INFO * prxFrameHeader,
                                           unsigned char *pBuf,
                                           unsigned short nBufSize)
{
       unsigned short wDataLen;
       unsigned char *pOut;

       pks885xHW->m_rxFrameCount--;
       pks885xHW->m_uCurFrameIndex++;

       //PRINTF("ks8851recv: remained frames =%d\n",pks885xHW->m_rxFrameCount);

       if (!(prxFrameHeader)->rxStatus & RX_VALID) {
               PRINTF("ks8851_recv error: received packet is invalid\n");
               return NULL;
       }
       if (prxFrameHeader->rxLength == 0) {
               PRINTF("ks8851_recv error: the receive length is zero\n");
               /* Manual DeQueue this error frame (free up this frame memory) */
               KS885X_REG_WRITE(pks885xHW, REG_RXQ_CMD,
                                (RXQ_CMD_CNTL | RXQ_CMD_FREE_PACKET));

               return NULL;
       }

       if (prxFrameHeader->rxLength > nBufSize) {
               PRINTF
                   ("ks8851_recv error: the received packet length=%d is bigger than buffer size=%d, can not handle it\n",
                    prxFrameHeader->rxLength, nBufSize);
               /* Manual DeQueue this error frame (free up this frame memory) */
               KS885X_REG_WRITE(pks885xHW, REG_RXQ_CMD,
                                (RXQ_CMD_CNTL | RXQ_CMD_FREE_PACKET));

               return NULL;
       }

       HW_READ_BUFFER(pks885xHW, pBuf, prxFrameHeader->rxLength, &wDataLen);
       pOut = pBuf;

       //PRINTF("\n datalen=%d wDataLen=%d\n",prxFrameHeader->rxLength,wDataLen);
       //PRINT_DATA(pOut,prxFrameHeader->rxLength);
       return pOut;
}

int ks885xReceive(PKS885XHW pks885xHW, void *pUserData, RECVFLAG RecvFlag)
{

       unsigned short uBufSize, uh;
       unsigned char *pBuf, *pOut;
       int len;
       FR_HEADER_INFO *prxFrameHeader = NULL;
       unsigned short i = 0;

       //first check  if there are some frames which still in the chip's buffer
       // let us read out, and pass to up level

       if (pks885xHW->m_rxFrameCount > 0) {
               //only polling will come here
               prxFrameHeader =
                   pks885xHW->m_pRevHdrInfo + pks885xHW->m_uCurFrameIndex;
               pBuf = ks885xGetRecvBuffer(pUserData, &uBufSize);
               if (pBuf) {
                       pOut =
                           ks885xReadSingleFrame(pks885xHW, prxFrameHeader,
                                                 pBuf, uBufSize);
                       if (pOut) {
                               ks885xPassBufToSystem(pUserData, pOut, prxFrameHeader->rxLength);       //pass to up layer
                               return prxFrameHeader->rxLength;
                       } else
                               return 0;
               } else
                       return 0;
       } else {
               if (RecvFlag != INTERRUPT_RECV) {
                       unsigned short IntStatus;

                       KS885X_REG_READ(pks885xHW, REG_INT_STATUS, &IntStatus);

                       if (IntStatus == 0) {
                               //PRINTF("?? ");
                               return 0;
                       }

                       KS885X_REG_WRITE(pks885xHW, REG_INT_STATUS, IntStatus); //acknowledge
                       if (!(IntStatus & INT_RX)) {
                               //PRINTF("no RX interrupt happen\n");    
                               return 0;       // not receive interrupt happen;
                       }

               }

               KS885X_REG_READ(pks885xHW, REG_RX_FRAME_CNT_THRES, &uh);
               pks885xHW->m_rxFrameCount = (uh & 0xFF00) >> 8;
               //PRINTF("frame count=%u pks885xHW->m_pRevHdrInfo=0x%x\n",pks885xHW->m_rxFrameCount,pks885xHW->m_pRevHdrInfo);

               if (pks885xHW->m_rxFrameCount == 0)
                       return 0;

               if (!pks885xHW->m_pRevHdrInfo)
                       return 0;

               if (pks885xHW->m_rxFrameCount >= pks885xHW->m_uCurRecvFrams) {
                       //when frames numbers are bigger than current header array size
                       // re-allocate the header array
                       FREE_MALLOC(pks885xHW->m_pRevHdrInfo);  // free old one
                       pks885xHW->m_uCurRecvFrams =
                           pks885xHW->m_rxFrameCount + 5;
                       pks885xHW->m_pRevHdrInfo =
                           (FR_HEADER_INFO *) MALLOC(sizeof(FR_HEADER_INFO) *
                                                     pks885xHW->
                                                     m_uCurRecvFrams);
                       if (!pks885xHW->m_pRevHdrInfo) {
                               PRINTF
                                   ("Error: Fail to allocate more memory for frame info structures\n");
                               return 0;
                       }
               }
               //allocate the FR_HEADER_INFO structure
               prxFrameHeader = pks885xHW->m_pRevHdrInfo;

               // read all head information
               for (i = 0; i < pks885xHW->m_rxFrameCount; i++) {
                       /* Checking Received packet status */
                       KS885X_REG_READ(pks885xHW, REG_RX_FHR_STATUS,
                                       &(prxFrameHeader + i)->rxStatus);

                       /* Get received packet length from hardware packet memory */
                       KS885X_REG_READ(pks885xHW, REG_RX_FHR_BYTE_CNT,
                                       &(prxFrameHeader + i)->rxLength);
                       /* Record read packet length in DWORD alignment */
                       (prxFrameHeader + i)->rxLength &= RX_BYTE_CNT_MASK;

               }

               pks885xHW->m_uCurFrameIndex = 0;
               while (pks885xHW->m_rxFrameCount > 0) {

                       prxFrameHeader =
                           pks885xHW->m_pRevHdrInfo +
                           pks885xHW->m_uCurFrameIndex;
                       len = prxFrameHeader->rxLength;
                       pBuf = ks885xGetRecvBuffer(pUserData, &uBufSize);
                       if (pBuf) {
                               //the pks885xHW->m_rxFrameCount counter will decreased
                               //in ks885xReadSingleFrame function
                               pOut =
                                   ks885xReadSingleFrame(pks885xHW,
                                                         prxFrameHeader, pBuf,
                                                         uBufSize);
                               if (!pOut)
                                       continue;
                               else {
                                       ks885xPassBufToSystem(pUserData, pOut, prxFrameHeader->rxLength);       //pass to up layer
                                       if (RecvFlag == POLLING_ONE_BUFFER)     //only poll one frame to up layer
                                               return prxFrameHeader->rxLength;
                                       else
                                               continue;
                               }

                       }
                       return 0;
               }

       }

       return len;
}

/*
    ks885xSendPacket

    Description:
         KS885X send data, the data should be a complete Ethernet packet.

    Parameters:
        PKS885XHW pks885xHW (IN)
             Pointer the KS885XHW instance structure
        void * pUserData (IN)
             Pointer to the user data which will pass to caller provided function GetNextSendData() to
             get next data. (Whole Ethernet packets may be put in several buffers in some system)
        unsigned char * pData (IN)
             Pointer a data buffer.
        unsigned short uDataLen(IN)
             The data size in this buffer.
        unsigned short uOrgPktLen (IN)
            The whole Ethernet packet size passed by upper layer. The size may not fit in 4 byte alignment.
    Return : If function is successful, return sent data otherwise return 0;
*/
int ks885xSendPacket(PKS885XHW pks885xHW, unsigned char *pData,
                    unsigned short uOrgPktLen)
{
       //unsigned short wReg;
       unsigned short uPktLenAfterAlignment;
       KS885XERR err = KS885XERR_NOERROR;

       GET_DATA_ALIGNMENT(uOrgPktLen, &uPktLenAfterAlignment);
       HW_WRITE_START(pks885xHW);

       {

               HW_WRITE_DATA_HEADER(pks885xHW, (unsigned short)uOrgPktLen);
               HW_WRITE_DATA_BUFFER(pks885xHW, pData, uOrgPktLen);

       }

       HW_WRITE_END(pks885xHW);
       /* Issue TXQ Command (Enqueue Tx frame from TX buffer into TXQ for transmit) */
       KS885X_REG_WRITE(pks885xHW, REG_TXQ_CMD, TXQ_ENQUEUE);
// the TXQ_ENQUEUE  will be auto clean after the whole frame sent out
       while (1) {
               unsigned short u;
               KS885X_REG_READ(pks885xHW, REG_TXQ_CMD, &u);
               if ((u & TXQ_ENQUEUE) == 0)
                       break;  //the frame has been sent out.
       }

       if (err == KS885XERR_NOERROR)
               return uOrgPktLen;
       else
               return 0;
}

/*
    ks885xGetMac

    Description:
        This function retrieves the MAC address of the hardware.

    Parameters:
        PKS885XHW pks885xHW
            Pointer to hardware instance.

    Return (none):
*/

void ks885xGetMac(PKS885XHW pks885xHW)
{

       unsigned short u, w, *pDest;
       pDest = (unsigned short *)&pks885xHW->m_MacAddress[0];
       KS885X_REG_READ(pks885xHW, REG_MAC_ADDR_4, &u);
       w = (unsigned short)(((u & 0xFF) << 8) | ((u >> 8) & 0xFF));
       PRINTF("The read mac is %d\n", w);
       *pDest++ = w;
       KS885X_REG_READ(pks885xHW, REG_MAC_ADDR_2, &u);
       w = (unsigned short)(((u & 0xFF) << 8) | ((u >> 8) & 0xFF));
       *pDest++ = w;
       KS885X_REG_READ(pks885xHW, REG_MAC_ADDR_0, &u);
       w = (unsigned short)(((u & 0xFF) << 8) | ((u >> 8) & 0xFF));
       *pDest++ = w;
}                              /* ks885xGetMac */

/*
    ks885xSetMac

    Description:
         Set KS8851 MAC address

    Parameters:
        PKS885XHW pks885xHW (IN)
             Pointer the KS885XHW instance structure
        unsigned char * pByte (IN)
             Pointer a buffer hold a MAC address value;
       
    Return (None):
*/
void ks885xSetMac(PKS885XHW pks885xHW, unsigned char *pByte)
{
       unsigned short *pw;
       unsigned short i, w, u;
       pw = (unsigned short *)pByte;
       u = *pw;
       w = (unsigned short)(((u & 0xFF) << 8) | ((u >> 8) & 0xFF));
       KS885X_REG_WRITE(pks885xHW, REG_MAC_ADDR_4, w);

       pw++;
       u = *pw;
       w = (unsigned short)(((u & 0xFF) << 8) | ((u >> 8) & 0xFF));
       KS885X_REG_WRITE(pks885xHW, REG_MAC_ADDR_2, w);

       pw++;
       u = *pw;
       w = (unsigned short)(((u & 0xFF) << 8) | ((u >> 8) & 0xFF));
       KS885X_REG_WRITE(pks885xHW, REG_MAC_ADDR_0, w);

       for (i = 0; i < 6; i++)
               pks885xHW->m_MacAddress[i] = *(pByte + i);
}

/*
    ks885xReset

    Description:
         Reset KS8851 chip

    Parameters:
        PKS885XHW pks885xHW (IN)
             Pointer the KS885XHW instance structure
       
    Return (None):
*/
void ks885xReset(PKS885XHW pks885xHW)
{
       PRINTF("ks885xReset(1x)\n");
       if (pks885xHW->m_ulIoVirtualAddr == 0) {
               PRINTF
                   ("Error pks885xHW->m_ulIoVirtualAddr is NULL, you must initialize the IO Base at beginning\n");
               return;
       }
       //must disable interrupt first
       KS885X_REG_WRITE(pks885xHW, REG_INT_MASK, 0x0000);
       KS885X_REG_WRITE(pks885xHW, REG_RESET_CTRL, GLOBAL_SOFTWARE_RESET);

       /* Wait for device to reset */
       UDELAY(100);
       /* Write 0 to clear device reset */
       KS885X_REG_WRITE(pks885xHW, REG_RESET_CTRL, 0);
       UDELAY(300);            //MQL need more time to do reset.

}

/*
    ks885xInit

    Description:
         KS8851 chip initialization

    Parameters:
        PKS885XHW pks885xHW (IN)
             Pointer the KS885XHW instance structure
   
        
    Return
                       Return KS885XERR_NOERROE, if the function is successful.
*/

KS885XERR ks885xInit(PKS885XHW pks885xHW)
{
       //unsigned short txCntl,rxCntl,uRegData,w;//intMask;
       unsigned short uRegData;

       PRINTF("ks885xInit\n");
       pks885xHW->m_uCurRecvFrams = MAX_RECV_FRAMES;
       pks885xHW->m_pRevHdrInfo =
           (FR_HEADER_INFO *) MALLOC(sizeof(FR_HEADER_INFO) * MAX_RECV_FRAMES);
       if (!pks885xHW->m_pRevHdrInfo) {
               PRINTF("Error: Fail to allocate frame memory\n");
               return FALSE;
       }
       //check chip ID
       KS885X_REG_READ(pks885xHW, REG_CHIP_ID, &uRegData);
       PRINTF("ks885x chip ID=0x%x\n", uRegData);
       if ((uRegData & 0xFFF0) != 0x8870) {
               PRINTF("Error: the ks885x chip ID is wrong\n");
               return KS885XERR_ERROR;
       }

       /* eric REVISIT: remove the line below, depends on write_hwaddr */
       //ks885xSetMac(pks885xHW, &KS885X_DEFAULT_MAC_ADDRESS[0]);
       ks885xSetMac(pks885xHW, pks885xHW->netdev.enetaddr);
       return KS885XERR_NOERROR;

}

/*
    HW_WRITE_DATA_HEADER

    Description:
         KS8851 chip data QMU header, and do a 32 data aligment

    Parameters:
        PKS885XHW pks885xHW (IN)
             Pointer the KS885XHW instance structure
   
               unsigned short uOrgLength  IN
                  original data length.
        
         Return  (none)
                      
*/
void HW_WRITE_DATA_HEADER(void *pks885xHW, unsigned short uOrgLength)
{
//DATA_ALIGNMENT==2
       {
               HW_WRITE_16BITDATA(pks885xHW, TX_CTRL_INTERRUPT_ON);
               //the data length in the head should be original data length
               HW_WRITE_16BITDATA(pks885xHW, uOrgLength);
       }

       // at beginning of each sending packet, we must clean the m_dwPrevData and m_uPrevByte
       ((PKS885XHW) pks885xHW)->m_dwPrevData = 0;
       ((PKS885XHW) pks885xHW)->m_uPrevByte = 0;
}

/*
    HW_WRITE_DATA_BUFFER

    Description:
        This function write the data to the hardware.

    Parameters:
        PKS885XHW pks885xHW (IN)
            Pointer to hardware instance.
        void *  (IN)
            Pointer to a user data which will pass to caller provided function GetNextSentData()
        unsigned char * data (IN)
            pointer to sending buffer.
        unsigned short uThisDataLen (IN)
            the data length of this buffer
        unsigned short uOrgPktLen
            The whole Ethernet packet size.
           
    Return (none):
*/

//DATA_ALIGNMENT==2
#define HW_WRITE_DATA(pHw,pIn, p, len, sentLen) \
{\
unsigned short * pw;\
pw=(unsigned short *)pIn;\
while(len--)\
{\
sentLen+=DATA_ALIGNMENT;\
HW_WRITE_16BITDATA(pHw,*pw++);\
}\
p=(unsigned char *)pw;\
}

void HW_WRITE_DATA_BUFFER(void *pks885xHW, unsigned char *pData,
                         unsigned short uOrgPktLen)
{
       unsigned short uPkLenAfterAligment, uDataLen, uSentLen, uLen, l;
       unsigned char *p, *pDest, *pSrc;
       GET_DATA_ALIGNMENT(uOrgPktLen, &uPkLenAfterAligment);

       uDataLen = uOrgPktLen;

       uSentLen = 0;

       uLen = uDataLen / DATA_ALIGNMENT;
       if (uLen > 0)
               HW_WRITE_DATA(pks885xHW, pData, p, uLen, uSentLen)
                   else
               p = pData;

       l = uDataLen % DATA_ALIGNMENT;
       if (l != 0) {
               ((PKS885XHW) pks885xHW)->m_uPrevByte = l;
               pDest =
                   (unsigned char *)&((PKS885XHW) pks885xHW)->m_dwPrevData;;
               pSrc = p;
               while (l) {
                       *pDest++ = *pSrc++;
                       l--;

               }
       }
// no more data;
       uDataLen = uPkLenAfterAligment - uSentLen;      //it shout be less then 4 (DWORD aliment);
       if (uDataLen > 4 || (uSentLen > uPkLenAfterAligment)) {
               ////should not goto here, except some unexpected error happen
               PRINTF("error:uDataLen=%d uSentLen=%d uPkLenAfterAligment=%d\n",
                      uDataLen, uSentLen, uPkLenAfterAligment);
               PRINTF
                   ("error: the sent data does not match we expected. The driver fail to handle this case!\n");
       } else if (uDataLen > 0) {
// the rest of data should be in the m_dwPrevData
               uLen = uDataLen / DATA_ALIGNMENT;
               if (uLen > 0)
                       HW_WRITE_DATA(pks885xHW,
                                     &((PKS885XHW) pks885xHW)->m_dwPrevData, p,
                                     uLen, uSentLen)
                           else
               {
                       //should not goto here, except some unexpected error happen
                       PRINTF
                           ("error: the remain the data does not follow data alognment. current driver can not handle this case!\n");
               }
       }

}

/*
    HW_READ_BUFFER

    Description:
        This function read the data from the hardware.

    Parameters:
        PKS885XHW pks885xHW (IN)
            Pointer to hardware instance.
        unsigned char * data (IN)
            pointer to a receive buffer.
        unsigned short len (IN)
            the receiving data length got from header before
        unsigned short * pRealLen (OUT)
            Actually received data length
           
    Return (none):
*/

void HW_READ_BUFFER(void *phw, unsigned char *data, unsigned short len,
                   unsigned short *pRealLen)
{
       unsigned short length;
       KS885XHW *pks885xHW = (KS885XHW *) phw;
       GET_DATA_ALIGNMENT(len, &length);
       HW_READ_START(pks885xHW);

       length = length / DATA_ALIGNMENT;

       //DATA_ALIGNMENT==2
       {
               unsigned short *pwData;
               pwData = (unsigned short *)data;
               //for the 16 bit data bus, there are 2 byte dummy data at beginning
               //we need to skip them
               HW_READ_16BITDATA(pks885xHW, pwData);
               // 4 byte header
               HW_READ_16BITDATA(pks885xHW, pwData);
               HW_READ_16BITDATA(pks885xHW, pwData);
               *pRealLen = *pwData;
               //now is valid data
               while (length--)
                       HW_READ_16BITDATA(pks885xHW, pwData++);
       }

       HW_READ_END(pks885xHW);
}

static KS885XHW gHardware;

extern void NetReceive(volatile uchar *, int);

/******************** UBOOT Ethernet interface **********************/

/*void eth_reset (void)
{
       ks885xReset(&gHardware);
}*/

void ks8851_eth_halt(struct eth_device *dev)
{

}

/*void eth_halt()
{
       ks8851_eth_halt(0);
}*/

/*
    ks885xPassBufToSystem

    Description:
        Pass the received data to up layer

    Parameters:
        void * pUserData (IN)
           a pointer to user data which passed in KS8851_CHIP_RECEIVE function as second parameter.
           the content of user data is totally depend on OS and KS8851 Chip (MLL/MQL/SNL)
        unsigned char * pDataBuf (IN)
           Point to a buffer which hold received data from KS8851 Chip
       unsigned short nDataLen (IN) 
           The data length.   
    Return : (none)
*/
void ks885xPassBufToSystem(void *pUserData, unsigned char *pDataBuf,
                          unsigned short nDataLen)
{
       NetReceive(pDataBuf, nDataLen);
}

/*
    ks885xGetRecvBuffer

    Description:
         return a data buffer pointer to caller.

    Parameters:
        void * pUserData (IN)
           a pointer to user data which passed in KS8851_CHIP_RECEIVE function as second parameter.
           the content of user data is totally depend on OS and KS8851 Chip (MLL/MQL/SNL)
        unsigned short * pnLen
           Point to a int variable to hold the buffer size.    
    Return :
                               return a buffer pointer.
*/
unsigned char *ks885xGetRecvBuffer(void *pUserData, unsigned short *pnLen)
{
       *pnLen = RX_BUF_SIZE;
       return gHardware.m_recvBuffer;
}

int ks8851_eth_recv(struct eth_device *dev)
{
       int len;
       do {
               len = ks885xReceive(&gHardware, 0, POLLING_ONE_BUFFER);
       } while (len < 1);
       //printf(".... recv: %d ....\n", len);
       return len;
}

/*
int eth_rx()
{
        return ks8851_eth_recv(0);
}
*/

// the send function will send a complete Ethernet frame.
int ks8851_eth_send(struct eth_device *dev, volatile void *packet, int length)
{
       //printf("**** send: %d ****\n", length);
       ks885xSendPacket(&gHardware, (unsigned char *)packet, length);
       return 1;
}

/*
int eth_send(volatile void *packet,int length)
{
       return ks8851_eth_send(0,packet,length);
}
*/

int ks8851_eth_init(bd_t * bd)
{
       //must initialize the IO Base at beginning.
       gHardware.m_ulIoVirtualAddr = CONFIG_KS8851_BASE_ADDR;
       ks885xBoardHwInit(&gHardware);
       ks885xReset(&gHardware);
       ks885xInit(&gHardware);
       ks885xDisable(&gHardware);
       ks885xSetup(&gHardware);
       ks885xEnable(&gHardware);

       return 0;
}

extern int eth_getenv_enetaddr_by_index(int index, uchar *enetaddr);
static int ks8851_eth_write_hwaddr(struct eth_device *dev)
{
       ks885xSetMac(&gHardware, dev->enetaddr);
}

int ks8851_initialize(bd_t * bis)
{
       struct eth_device *dev = &(gHardware.netdev);

       dev->init = ks8851_eth_init;
       dev->halt = ks8851_eth_halt;
       dev->send = ks8851_eth_send;
       dev->recv = ks8851_eth_recv;
       dev->write_hwaddr = ks8851_eth_write_hwaddr;
       sprintf(dev->name, "ks8851");

       eth_register(dev);

       return 0;
}

/*
int eth_init(bd_t * bd)
{
       return ks8851_eth_init(bd);
}
*/
