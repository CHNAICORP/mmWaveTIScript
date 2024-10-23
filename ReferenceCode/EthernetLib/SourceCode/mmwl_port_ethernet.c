/****************************************************************************************
 * FileName     : mmwl_port_ethernet.c
 *
 * Description  : This file defines the functions to configure mmwave radar device through
 * 				  the TDA2xx capture card
 *
 ****************************************************************************************
 * (C) Copyright 2014, Texas Instruments Incorporated. - TI web address www.ti.com
 *---------------------------------------------------------------------------------------
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *    Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR CONTRIBUTORS
 *  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

 /*
 ****************************************************************************************
 * Revision History   :
 *---------------------------------------------------------------------------------------
 * Version  Date        Author             Defect No               Description
 *---------------------------------------------------------------------------------------
 * 0.1.0    22Mar2019   Abhijjith V           -           Initial Version
 *
 * 0.2.0    22Jun2019   Abhijjith V           -           Added API's for Capture
 *
 * 0.3.0    25Jul2019   Abhijjith V           -           Added Doxgyen support
 ****************************************************************************************
 */

/******************************************************************************
 * INCLUDE FILES
 ******************************************************************************
 */

#include "stdio.h"	
#include "mmwl_port_ethernet.h"

/******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 ******************************************************************************
 */

Radar_EthDataPacketPrms         pDataPacket[TDA_NUM_CONNECTED_DEVICES_MAX + 1];
DevComm_NetworkCtrlReqPrms      pDevCtrlPrms[TDA_NUM_CONNECTED_DEVICES_MAX + 1];

TDADevCtx_t						gTDA_devCtx[TDA_NUM_CONNECTED_DEVICES_MAX] = { 0 };
volatile UINT32					gSpiRxSignal[TDA_NUM_CONNECTED_DEVICES_MAX] = { 0 };
UINT32							gSpiRxLength[TDA_NUM_CONNECTED_DEVICES_MAX] = { 0 };
UINT8							gSpiRxData[TDA_NUM_CONNECTED_DEVICES_MAX][512];
TDA_EVENT_HANDLER               gTDACARD_Callback;
HANDLE                          gThreadHdl;
captureConfig_t					gCaptureConfigParams;
NetworkRadarDeviceMap_param		gDeviceMapParams;
NetworkTDA_Obj					gNetworkTDA_obj;
Network_SockObj					gNetwork_SockObj;
CRITICAL_SECTION				gNetwork_Write_cs;
	
UINT32							TDA_HostIntrThreadLoop[TDA_NUM_CONNECTED_DEVICES_MAX];
UINT32							TDA_HostIntrExitThread = 0;
UINT32							TDA_NumOfRunningSPIThreads = 0;
UINT32							TDA_NetworkThreadRunning = 0;
UINT32							TDA_StartTraceACK = 0;
UINT32							TDA_GetVersionACK = 0;
UINT32 							magicWord = 0;

/******************************************************************************
 * FUNCTION DEFINITIONS
 ******************************************************************************
 */

/*! \brief
* Function used for logging the messages
*/
extern void DEBUG_PRINT(char *fmt, ...);
extern void CloseTraceFile();

DWORD WINAPI Network_waitConnect(LPVOID lpParam)
{
	UINT32 *pMem;
	UINT32 prmSize = 0;
	pMem = malloc(512);

	if (pMem == NULL)
	{
		DEBUG_PRINT("# ERROR: Unable to allocate memory for response parameters\n");
        return RLS_RET_CODE_EFAIL;
	}
	
	/* Initialize the Rx thread */
	TDA_NetworkThreadRunning = 1;

	while (TDA_NetworkThreadRunning)
	{
		/* Check if any information is available on the socket */
		INT32 status = Network_waitRead(&gNetwork_SockObj);

		/* If information is available */
		if (status == 1)
		{
			/* Read header response */
			INT32 response = RecvResponse(&prmSize);
			if (response != SYSTEM_LINK_STATUS_SOK)
			{
				/* Notify mmWaveStudio about network error */
				gTDACARD_Callback(1, CAPTURE_RESPONSE_NETWORK_ERROR, \
														0, response, NULL);
				break;
			}

			/* If there are bytes to read */
			if (prmSize)
			{
				/* Read data */
				RecvResponseParams((UINT8*)pMem, prmSize);
				/* Process the data */
				Radar_processData((Radar_EthDataPacketPrms *)pMem, prmSize);
			}
		}
	}
	DEBUG_PRINT("# INFO: Rx thread about to die\r\n");
	return RLS_RET_CODE_OK;
}

/** @fn STATUS Radar_formEthDataPacket(Radar_EthDataPacketPrms *pDataPacket,
			   DevComm_NetworkCtrlReqPrms *pDevCtrlPrms, UINT16 responseCode,
									   UINT16 dataLength, UINT8* data)
*
*   @brief Forms the Ethernet packet to be sent across TDA
*   @param[in] pDataPacket - pointer to the Ethernet packet to be formed
*   @param[in] pDevCtrlPrms - pointer to the structure for device Control
*   @param[in] responseCode - command code to be sent
*   @param[in] dataLength - Data length of the packet
*   @param[in] data - pointer to the data
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS Radar_formEthDataPacket(Radar_EthDataPacketPrms *pDataPacket,
		DevComm_NetworkCtrlReqPrms *pDevCtrlPrms, UINT16 responseCode,
										UINT16 dataLength, UINT8* data)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	Radar_EthDataPacketPrms dataPacket = { 0 };
	UINT16 crc = 0;
	UINT8* pDataBuf = (UINT8*)pDevCtrlPrms->respParam;

	dataPacket.syncByte = TX_SYNC_BYTE;
	dataPacket.opcode = responseCode;
	dataPacket.ackCode = 0;
	dataPacket.dataLength = dataLength;
	dataPacket.devSelection = pDataPacket->devSelection;
	dataPacket.ackType = pDataPacket->ackType;
	
	if (data != NULL)
	{
		memcpy(dataPacket.data, data, dataLength - DATA_HEADER_LENGTH);
	}
	
	status = Bsp_ar12xxComputeCrc((UINT8 *)((UINT8 *)&dataPacket + 2), dataLength, \
									RL_CRC_TYPE_16BIT_CCITT, (UINT8 *)&crc);	
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: CRC calculation failed\n");
		return RLS_RET_CODE_EFAIL;
	}
	
	*(UINT16*)(dataPacket.data + dataLength - DATA_HEADER_LENGTH) = crc;
	memcpy(pDevCtrlPrms->respParam, &dataPacket, (dataLength + HEADER_AND_CRC_LENGTH));
	pDevCtrlPrms->respParamSize = dataLength + HEADER_AND_CRC_LENGTH;

	return status;
}

/** @fn STATUS Radar_processData(Radar_EthDataPacketPrms *pDataPacket_ptr, UINT32 prmSize)
*
*   @brief Process the data received from the TDA
*   @param[in] pDataPacket_ptr - pointer to the Ethernet packet received
*   @param[in] prmSize - total size in bytes received
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS Radar_processData(Radar_EthDataPacketPrms *pDataPacket_ptr, UINT32 prmSize)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	UINT16 calculatedCrc, receivedCrc;
	
	if (pDataPacket_ptr->syncByte != RX_SYNC_BYTE)
	{
		DEBUG_PRINT("# ERROR: Received wrong sync byte %x\n", pDataPacket_ptr->syncByte);
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	status = Bsp_ar12xxComputeCrc((UINT8 *)((UINT8 *)pDataPacket_ptr + 2), 
		(prmSize - HEADER_AND_CRC_LENGTH), RL_CRC_TYPE_16BIT_CCITT, 
											(UINT8 *)&calculatedCrc);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: CRC calculation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	receivedCrc = *(UINT16 *)((UINT8 *)pDataPacket_ptr + prmSize - 2);
	if (calculatedCrc != receivedCrc)
	{
		DEBUG_PRINT("# ERROR: CRC did not match. Received CRC : %x\n", receivedCrc);
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	switch (pDataPacket_ptr->opcode)
	{
		case CAPTURE_RESPONSE_ACK:
			/* Received ACK Response from the TDA */
			{
				if (pDataPacket_ptr->ackCode == CAPTURE_CONFIG_TRACE_START)
				{
					TDA_StartTraceACK = 1;
					gTDACARD_Callback((UINT16)pDataPacket_ptr->devSelection, \
													pDataPacket_ptr->opcode, \
									pDataPacket_ptr->ackCode, status, NULL);
				}
				else if (pDataPacket_ptr->ackCode == SENSOR_CONFIG_SPI_WRITE)
				{
					/* Received Host IRQ low */
					if (pDataPacket_ptr->ackType == ACK_ON_PROCESS)
					{
						int dev_id = getDevIdFromDevMap(pDataPacket_ptr->devSelection);
						DEBUG_PRINT("# INFO: Received HOST_IRQ_LOW command from device %d\n", \
							dev_id);
						TDADevCtx_t*   pDevCtx = (TDADevCtx_t *)TDAGetDeviceCtx(dev_id);
						pDevCtx->hostIrqRxHigh = 0;
					}
				}
				else
				{
					gTDACARD_Callback((UINT16)pDataPacket_ptr->devSelection, \
													pDataPacket_ptr->opcode, \
									pDataPacket_ptr->ackCode, status, NULL);
				}
				break;
			}
		case CAPTURE_RESPONSE_NACK:
			/* Received NACK Response from the TDA */
			{
				int dev_id = getDevIdFromDevMap(pDataPacket_ptr->devSelection);
				DEBUG_PRINT("# INFO: Received CAPTURE_RESPONSE_NACK command from device %d\n", \
					dev_id);
				gTDACARD_Callback((UINT16)pDataPacket_ptr->devSelection, \
												pDataPacket_ptr->opcode, \
								pDataPacket_ptr->ackCode, status, NULL);
				break;
			}
		case CAPTURE_RESPONSE_VERSION_INFO:
			/* Received Version Information from TDA */
			{
				DEBUG_PRINT("# INFO: Received CAPTURE_RESPONSE_VERSION_INFO command with TDA Binary Version : %s\n", \
														pDataPacket_ptr->data);
				TDA_GetVersionACK = 1;
				gTDACARD_Callback((UINT16)pDataPacket_ptr->devSelection, \
												pDataPacket_ptr->opcode, \
					pDataPacket_ptr->ackCode, status, pDataPacket_ptr->data);
				break;
			}
		case CAPTURE_RESPONSE_CONFIG_INFO:
			/* Received Capture Config Information from TDA */
			{
				int dev_id = getDevIdFromDevMap(pDataPacket_ptr->devSelection);
				DEBUG_PRINT("# INFO: Received CAPTURE_RESPONSE_CONFIG_INFO command from device %d with Width = %d and Height = %d\n", \
							dev_id, ((captureConfig_t*)pDataPacket_ptr->data)->width, \
							((captureConfig_t*)pDataPacket_ptr->data)->height);
				gTDACARD_Callback((UINT16)pDataPacket_ptr->devSelection, \
												pDataPacket_ptr->opcode, \
					pDataPacket_ptr->ackCode, status, (captureConfig_t*)pDataPacket_ptr->data);
				break;
			}
		case CAPTURE_RESPONSE_TRACE_DATA:
			/* Received Trace data Response from TDA */
			{
				DEBUG_PRINT("# INFO: Received ACK for Stop Trace command \n");
				gTDACARD_Callback((UINT16)pDataPacket_ptr->devSelection, \
												pDataPacket_ptr->opcode, \
								pDataPacket_ptr->ackCode, status, NULL);
				break;
			}
		case CAPTURE_RESPONSE_GPIO_DATA:
			/* Received GPIO data */
			{
				int dev_id = getDevIdFromDevMap(pDataPacket_ptr->devSelection);
				DEBUG_PRINT("# INFO: Received CAPTURE_RESPONSE_GPIO_DATA command from device %d with GPIO Pin = %d and GPIO data = %d\n", \
					dev_id, ((s_gpioConfig_t*)pDataPacket_ptr->data)->gpioPin, \
						((s_gpioConfig_t*)pDataPacket_ptr->data)->gpioValue);
				gTDACARD_Callback((UINT16)pDataPacket_ptr->devSelection, \
												pDataPacket_ptr->opcode, \
					pDataPacket_ptr->ackCode, status, (s_gpioConfig_t*)pDataPacket_ptr->data);
				break;
			}
		case CAPTURE_RESPONSE_HOST_IRQ:
			/* Indicates HOST IRQ from AWR device */
			{
				int dev_id = getDevIdFromDevMap(pDataPacket_ptr->devSelection);
				DEBUG_PRINT("# INFO: Received HOST_IRQ_HIGH command from device %d\n", \
					dev_id);
				TDADevCtx_t*   pDevCtx = (TDADevCtx_t *)TDAGetDeviceCtx(dev_id);
				if (pDevCtx->hostIntrThread.eventHandle != NULL)
				{
					SetEvent(pDevCtx->hostIntrThread.eventHandle);
				}
				pDevCtx->hostIrqRxHigh = 1;
				break;
			}
		case SENSOR_RESPONSE_SPI_DATA:
			/* Received SPI data from the AWR device */
			{
				int dev_id = getDevIdFromDevMap(pDataPacket_ptr->devSelection);
				gSpiRxLength[dev_id] = pDataPacket_ptr->dataLength - DATA_HEADER_LENGTH;
				memcpy(gSpiRxData[dev_id], pDataPacket_ptr->data, gSpiRxLength[dev_id]);
				gSpiRxSignal[dev_id] = 1;
				break;
			}
		case SENSOR_RESPONSE_SOP_INFO:
			/* Received SOP info of the AWR device */
			{
				int dev_id = getDevIdFromDevMap(pDataPacket_ptr->devSelection);
				DEBUG_PRINT("# INFO: Received SENSOR_RESPONSE_SOP_INFO command from device %d with SOP Mode = %d\n", \
					dev_id, *((UINT32*)pDataPacket_ptr->data));
				gTDACARD_Callback((UINT16)pDataPacket_ptr->devSelection, \
												pDataPacket_ptr->opcode, \
					pDataPacket_ptr->ackCode, status, (UINT32*)pDataPacket_ptr->data);
				break; 
			}
		default:
			/* Received unknown command */
			{
				int dev_id = getDevIdFromDevMap(pDataPacket_ptr->devSelection);
				DEBUG_PRINT("# ERROR: Unknown command received from device %d\n", dev_id);
				break;
			}
	}
		
	return status;
}

/** @fn STATUS ethernetConnect(unsigned char *ipAddr, UINT32 configPort, UINT32 deviceMap)
*
*   @brief Configures the Ethernet connection and the port creation
*   @param[in] ipAddr - pointer to the IP Address of the TDA
*   @param[in] configPort - configuration port
*   @param[in] deviceMap - devices to be enabled by the TDA
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS ethernetConnect(unsigned char *ipAddr, UINT32 configPort, UINT32 deviceMap)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;

	// Initializing Network parameters
	memset(&gNetworkTDA_obj, 0, sizeof(gNetworkTDA_obj));
	if (configPort != 0)
	{
		gNetworkTDA_obj.serverPort = configPort;
	}
	else
	{
		gNetworkTDA_obj.serverPort = NETWORK_TDA_SERVER_PORT;
	}

	strcpy_s(gNetworkTDA_obj.ipAddr, _countof(gNetworkTDA_obj.ipAddr), ipAddr);

	//Initializing sockets
	Network_init();

	//Connecting to the server
	status = ConnectToServer();
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Connecting to the server failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	InitializeCriticalSection(&gNetwork_Write_cs);

	//Create Rx thread
	DWORD(WINAPI *pwaitConnect) (LPVOID) = NULL;
	pwaitConnect = Network_waitConnect;

	gThreadHdl = CreateThread(NULL, 0, pwaitConnect, &gNetwork_SockObj, \
																	0, NULL);

	/* wait for the thread to start */
	while (TDA_NetworkThreadRunning == 0)
	{
		Sleep(1);
	}

	//Send Ping command to check the validity of network connection
	status = IsConnected();
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: PING command failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}
	
	//Send trace command for logging all messages between TDA and mmWaveStudio
	char traceBuff[100];
	SYSTEMTIME SystemTime;
	GetLocalTime(&SystemTime);
	/* Generate unique trace name */
	sprintf(traceBuff, "Trace_TDA_[%02d_%02d_%04d_%02d.%02d].txt", SystemTime.wDay, \
		SystemTime.wMonth, SystemTime.wYear, SystemTime.wHour, SystemTime.wMinute);
	status = startTrace(traceBuff);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: TRACE command failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* wait for the ACK to be received for Trace command */
	int cmdCount = 0;
	TDA_StartTraceACK = 0;
	while (TDA_StartTraceACK == 0)
	{
		Sleep(1);
		cmdCount++;
		/* 3 seconds timeout */
		if (cmdCount > 3000)
		{
			DEBUG_PRINT("# ERROR: No ACK received for TRACE command\n");
			return SYSTEM_LINK_STATUS_EFAIL;
		}
	}

	//Send TDA version command
	status = readDLLVersion();
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: TDA version command failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* wait for the ACK to be received for TDA version command */
	cmdCount = 0;
	TDA_GetVersionACK = 0;
	while (TDA_GetVersionACK == 0)
	{
		Sleep(1);
		cmdCount++;
		/* 3 seconds timeout */
		if (cmdCount > 3000)
		{
			DEBUG_PRINT("# ERROR: No ACK received for TDA version command\n");
			return SYSTEM_LINK_STATUS_EFAIL;
		}
	}

	//Send the devices to be enabled for configuration and capture
	status = ConfigureDeviceMap(deviceMap);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Configure DeviceMap command failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	Sleep(50);

	//Initialize the GPIO and other peripherals
	pDataPacket[4].ackType = ACK_ON_PROCESS;
	pDataPacket[4].devSelection = 1;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_CONFIG_CONNECT, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS ethernetDisconnect()
*
*   @brief Disconnects from the TDA
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS ethernetDisconnect()
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	
	//Deinitialize the GPIO and other peripherals
	pDataPacket[4].ackType = ACK_ON_RECEIVE;
	pDataPacket[4].devSelection = 1;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_CONFIG_DISCONNECT, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);

	/* Send the stop trace command to close the trace file */
	status = stopTrace();
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Stop Trace command failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	Sleep(1000);

	// Disable the Rx thread
	TDA_NetworkThreadRunning = 0;

	Sleep(1000);

	/* Close the socket */
	status = CloseConnection();
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Terminates socket operations for all threads */
	status = Network_deInit();

	/* Close the file handle for trace */
	CloseTraceFile();

	DeleteCriticalSection(&gNetwork_Write_cs);
	return status;
}

/** @fn STATUS IsConnected()
*
*   @brief Validate the connection status with the TDA
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS IsConnected()
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_RECEIVE;
	pDataPacket[4].devSelection = 32;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_CONFIG_PING, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS ConfigureDeviceMap(UINT32 deviceMap)
*
*   @brief Notify the TDA about the number of devices to be configured
*   @param[in] deviceMap - indicates the devices to be configured
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS ConfigureDeviceMap(UINT32 deviceMap)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_RECEIVE;
	pDataPacket[4].devSelection = 32;

	DEBUG_PRINT("# INFO: Sending Device Map of %d\n", deviceMap);

	if ((deviceMap & 0x1) == 1)
		gDeviceMapParams.isMasterEnable = 1;
	else
		gDeviceMapParams.isMasterEnable = 0;

	if ((deviceMap & 0x2) == 2)
		gDeviceMapParams.isSlave1Enable = 1;
	else
		gDeviceMapParams.isSlave1Enable = 0;

	if ((deviceMap & 0x4) == 4)
		gDeviceMapParams.isSlave2Enable = 1;
	else
		gDeviceMapParams.isSlave2Enable = 0;

	if ((deviceMap & 0x8) == 8)
		gDeviceMapParams.isSlave3Enable = 1;
	else
		gDeviceMapParams.isSlave3Enable = 0;

	gDeviceMapParams.numDevice = gDeviceMapParams.isMasterEnable + \
								 gDeviceMapParams.isSlave1Enable + \
								 gDeviceMapParams.isSlave2Enable + \
								 gDeviceMapParams.isSlave3Enable;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_CONFIG_DEVICE_MAP, DATA_HEADER_LENGTH + sizeof(gDeviceMapParams),
		(UINT8*)&gDeviceMapParams);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS TDACreateApplication()
*
*   @brief Notify the TDA to create the capture application
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS TDACreateApplication()
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_PROCESS;
	pDataPacket[4].devSelection = 32;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_CONFIG_CREATE_APPLICATION, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS readHWVersion()
*
*   @brief Read the HW version of the board
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS readHWVersion()
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_PROCESS;
	pDataPacket[4].devSelection = 32;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_CONFIG_VERSION_GET, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS readDLLVersion()
*
*   @brief Read the TDA binary image version
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS readDLLVersion()
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_PROCESS;
	pDataPacket[4].devSelection = 32;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_CONFIG_VERSION_GET, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS setWidthAndHeight(UINT8 devSelection, UINT32 width, UINT32 height)
*
*   @brief Configures the width and height for capture application
*   @param[in] devSelection - device map
*   @param[in] width - width for capture application
*   @param[in] height - height for capture application
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS setWidthAndHeight(UINT8 devSelection, UINT32 width, UINT32 height)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	int devId = getDevIdFromDevMap(devSelection);
	pDataPacket[devId].ackType = ACK_ON_PROCESS;
	pDataPacket[devId].devSelection = devSelection;

	gCaptureConfigParams.width = width;
	gCaptureConfigParams.height = height;
	
	status = Radar_formEthDataPacket(&pDataPacket[devId], &pDevCtrlPrms[devId],
		CAPTURE_CONFIG_CONFIG_SET, DATA_HEADER_LENGTH + sizeof(gCaptureConfigParams),
		(UINT8*)&gCaptureConfigParams);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[devId].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[devId].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[devId].respParam, pDevCtrlPrms[devId].respParamSize);
	return status;
}

/** @fn STATUS getWidthAndHeight(UINT8 devSelection)
*
*   @brief Retrieves the width and height for capture application
*   @param[in] devSelection - device map
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS getWidthAndHeight(UINT8 devSelection)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	int devId = getDevIdFromDevMap(devSelection);
	pDataPacket[devId].ackType = ACK_ON_PROCESS;
	pDataPacket[devId].devSelection = devSelection;

	status = Radar_formEthDataPacket(&pDataPacket[devId], &pDevCtrlPrms[devId],
		CAPTURE_CONFIG_CONFIG_GET, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[devId].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[devId].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[devId].respParam, pDevCtrlPrms[devId].respParamSize);
	return status;
}

/** @fn STATUS TDAregisterCallback(UINT8 devSelection, EVENT_HANDLER RF_EventCallback,
*									void* pValue)
*
*   @brief Registers the callback with mmwavelink for the device having the specified 
*		   device index
*   @param[in] devSelection - device map
*   @param[in] RF_EventCallback - function pointer to be registered as interrupt handler
*   @param[in] pValue - pointer to the value which will be passed as an argument to the 
*						registered function
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS TDAregisterCallback(UINT8 devSelection, EVENT_HANDLER RF_EventCallback, void* pValue)
{
	TDADevCtx_t*   pDevCtx;
	pDevCtx = (TDADevCtx_t *)TDAGetDeviceCtx(devSelection);

	if (RF_EventCallback == NULL)
	{
		DEBUG_PRINT("# ERROR: Invalid Callback from mmWaveLink\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	TDALockDevice(pDevCtx);

	pDevCtx->hostIntrThread.handler = RF_EventCallback;
	pDevCtx->hostIntrThread.pValue = pValue;

	TDAUnlockDevice(pDevCtx);

	if (pDevCtx->hostIntrThread.threadHdl != NULL)
	{
		TDAStopIrqPollingThread(pDevCtx);
		pDevCtx->hostIntrThread.threadHdl = NULL;
	}
	return RLS_RET_CODE_OK;
}

/** @fn STATUS registerTDAStatusCallback(TDA_EVENT_HANDLER TDACard_EventCallback)
*
*   @brief Registers the callback with mmWaveStudio
*   @param[in] TDACard_EventCallback - function pointer to be registered as interrupt handler
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS registerTDAStatusCallback(TDA_EVENT_HANDLER TDACard_EventCallback)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;

	if (TDACard_EventCallback == NULL)
	{
		DEBUG_PRINT("# ERROR: Invalid Callback for mmWaveStudio\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	gTDACARD_Callback = TDACard_EventCallback;

	return status;
}

/** @fn STATUS startTrace(unsigned char *filename)
*
*   @brief Notify TDA to start a trace file to log all the messages between 
		   TDA and the application
*   @param[in] filename - filename of the trace file
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS startTrace(unsigned char *filename)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_PROCESS;
	pDataPacket[4].devSelection = 32;
	DEBUG_PRINT("# INFO: Sending trace file name : %s , Length : %d\n", filename, \
											(UINT16)strlen(filename));
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_CONFIG_TRACE_START, DATA_HEADER_LENGTH + (UINT16)strlen(filename),
		(UINT8*)filename);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}
		
	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS stopTrace()
*
*   @brief Notify TDA to close the trace file
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS stopTrace()
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_PROCESS;
	pDataPacket[4].devSelection = 32;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_CONFIG_TRACE_RETREIVE, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS showCPUStats()
*
*   @brief Notify TDA to display the CPU core usage and VIP port data rate 
		   statistics in the console
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS showCPUStats()
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_RECEIVE;
	pDataPacket[4].devSelection = 32;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_CONFIG_START_LOGGING_STATS, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS sendFramePeriodicitySync(unsigned int framePeriodicity)
*
*   @brief Notify TDA about the frame periodicity associated with the device
*   @param[in] framePeriodicity - periodicity of the frames being triggered
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS sendFramePeriodicitySync(unsigned int framePeriodicity)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_RECEIVE;
	pDataPacket[4].devSelection = 32;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_DATA_FRAME_PERIODICITY, DATA_HEADER_LENGTH + sizeof(framePeriodicity),
		(UINT8*)&framePeriodicity);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS sendNumAllocatedFiles(unsigned int numAllocatedFiles)
*
*   @brief Notify TDA about the number of files to be allocated before hand
*   @param[in] numAllocatedFiles - number of files to be allocated
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS sendNumAllocatedFiles(unsigned int numAllocatedFiles)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_RECEIVE;
	pDataPacket[4].devSelection = 32;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_DATA_NUM_ALLOCATED_FILES, DATA_HEADER_LENGTH + sizeof(numAllocatedFiles),
		(UINT8*)&numAllocatedFiles);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS enableDataPackaging(unsigned int enablePackaging)
*
*   @brief Notify TDA about the data format to be stored
*   @param[in] enablePackaging - packing type. 1: enable 12-bit mode ; 0: enable 16-bit mode \n
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS enableDataPackaging(unsigned int enablePackaging)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_RECEIVE;
	pDataPacket[4].devSelection = 32;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_DATA_ENABLE_DATA_PACKAGING, DATA_HEADER_LENGTH + sizeof(enablePackaging),
		(UINT8*)&enablePackaging);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS setSessionDirectory(unsigned char *sessionDirectory)
*
*   @brief Notify TDA about the directory to store the captured files in that session
*   @param[in] sessionDirectory - name of the session directory
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS setSessionDirectory(unsigned char *sessionDirectory)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_RECEIVE;
	pDataPacket[4].devSelection = 32;
	DEBUG_PRINT("# INFO: Sending session directory name : %s , Length : %d\n", \
							sessionDirectory, (UINT16)strlen(sessionDirectory));
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_DATA_SESSION_DIRECTORY, DATA_HEADER_LENGTH + (UINT16)strlen(sessionDirectory),
		(UINT8*)sessionDirectory);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS NumFramesToCapture(unsigned int numFrames)
*
*   @brief Notify TDA about the number of frames to be captured
*   @param[in] numFrames - number of frames to capture. 0: assumes the default case ; Any positive value: number of frames to be captured \n
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS NumFramesToCapture(unsigned int numFrames)
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_RECEIVE;
	pDataPacket[4].devSelection = 32;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_DATA_NUM_FRAMES, DATA_HEADER_LENGTH + sizeof(numFrames),
		(UINT8*)&numFrames);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS spiWriteToDevice(TDADevHandle_t hdl, unsigned char *data,
*									unsigned short ByteCount)
*
*   @brief Writes the specified number of bytes over SPI interface
*   @param[in] hdl - device handle for which the data needs to be written
*   @param[in] data - pointer to the Source buffer
*   @param[in] ByteCount - number of bytes to be written
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS spiWriteToDevice(TDADevHandle_t hdl, unsigned char *data, unsigned short ByteCount)
{
	TDADevCtx_t*    pDevCtx = (TDADevCtx_t*)hdl;
	TDALockDevice(pDevCtx);
	UINT8 devSelection = createDevMapFromDevId(pDevCtx->deviceIndex);
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[pDevCtx->deviceIndex].devSelection = devSelection;

	magicWord = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
	if (magicWord == 0x43211234)
	{
		/* Other than CNYS pattern messages */
		pDataPacket[pDevCtx->deviceIndex].ackType = ACK_ON_RECEIVE;
	}
	else
	{
		/* CNYS pattern messages */
		pDataPacket[pDevCtx->deviceIndex].ackType = ACK_ON_PROCESS;
	}

	status = Radar_formEthDataPacket(&pDataPacket[pDevCtx->deviceIndex], \
									&pDevCtrlPrms[pDevCtx->deviceIndex], \
		SENSOR_CONFIG_SPI_WRITE, DATA_HEADER_LENGTH + ByteCount,
		(UINT8*)data);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	handleDevCtrl((UINT8*)&pDevCtrlPrms[pDevCtx->deviceIndex].respParam, \
						pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize);	

	TDAUnlockDevice(pDevCtx);

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize; i = i + 2)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X%02X ", \
					pDevCtrlPrms[pDevCtx->deviceIndex].respParam[i + 1], \
					pDevCtrlPrms[pDevCtx->deviceIndex].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("Device [%d] [WR]%s", pDevCtx->deviceIndex, tempBuff);

	return ByteCount;
}

/** @fn STATUS spiReadFromDevice(TDADevHandle_t hdl, unsigned char *data,
*									unsigned short ByteCount)
*
*   @brief Reads the specified number of bytes over SPI interface
*   @param[in] hdl - device handle for which the data needs to be read
*   @param[in] data - pointer to the Destination buffer
*   @param[in] ByteCount - number of bytes to be read
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS spiReadFromDevice(TDADevHandle_t hdl, unsigned char *data, unsigned short ByteCount)
{
	TDADevCtx_t*    pDevCtx = (TDADevCtx_t*)hdl;
	TDALockDevice(pDevCtx);
	UINT8 devSelection = createDevMapFromDevId(pDevCtx->deviceIndex);
	INT32 status = SYSTEM_LINK_STATUS_SOK;

	pDataPacket[pDevCtx->deviceIndex].devSelection = devSelection;
	pDataPacket[pDevCtx->deviceIndex].ackType = ACK_ON_PROCESS;
	status = Radar_formEthDataPacket(&pDataPacket[pDevCtx->deviceIndex], \
									&pDevCtrlPrms[pDevCtx->deviceIndex], \
		SENSOR_CONFIG_SPI_READ, DATA_HEADER_LENGTH + sizeof(ByteCount),
		(UINT8*)&ByteCount);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}
	handleDevCtrl((UINT8*)&pDevCtrlPrms[pDevCtx->deviceIndex].respParam, \
						pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize);	

	//Wait till data is read by TDA
	while (gSpiRxSignal[pDevCtx->deviceIndex] == 0)
	{
		Sleep(1);
	}

	if (gSpiRxSignal[pDevCtx->deviceIndex] == 1)
	{
		(void)memcpy(data, gSpiRxData[pDevCtx->deviceIndex], \
								gSpiRxLength[pDevCtx->deviceIndex]);
		gSpiRxSignal[pDevCtx->deviceIndex] = 0;
	}

	TDAUnlockDevice(pDevCtx);

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < gSpiRxLength[pDevCtx->deviceIndex]; i = i + 2)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X%02X ", \
					*(gSpiRxData[pDevCtx->deviceIndex] + i + 1), \
					*(gSpiRxData[pDevCtx->deviceIndex] + i));
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("Device [%d] [RD]%s", pDevCtx->deviceIndex, tempBuff);

	return gSpiRxLength[pDevCtx->deviceIndex];
}

/** @fn STATUS resetDevice(TDADevHandle_t hdl)
*
*   @brief Reset the AWR device
*   @param[in] hdl - handle for the device thats needs to be reset
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS resetDevice(TDADevHandle_t hdl)
{
	TDADevCtx_t*    pDevCtx = (TDADevCtx_t*)hdl;
	UINT8 devSelection = createDevMapFromDevId(pDevCtx->deviceIndex);
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[pDevCtx->deviceIndex].devSelection = devSelection;
	pDataPacket[pDevCtx->deviceIndex].ackType = ACK_ON_PROCESS;
	status = Radar_formEthDataPacket(&pDataPacket[pDevCtx->deviceIndex], \
									&pDevCtrlPrms[pDevCtx->deviceIndex], \
		SENSOR_CONFIG_DEVICE_RESET, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", \
					pDevCtrlPrms[pDevCtx->deviceIndex].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("Device [%d] [GN]%s", pDevCtx->deviceIndex, tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[pDevCtx->deviceIndex].respParam, \
						pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize);
	return status;
}

/** @fn STATUS setSOPMode(TDADevHandle_t hdl, UINT32 SOPmode)
*
*   @brief Configures the SOP mode on the AWR device
*   @param[in] hdl - device handle
*   @param[in] SOPmode - specifies the SOP mode
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS setSOPMode(TDADevHandle_t hdl, UINT32 SOPmode)
{
	TDADevCtx_t*    pDevCtx = (TDADevCtx_t*)hdl;
	UINT8 devSelection = createDevMapFromDevId(pDevCtx->deviceIndex);
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[pDevCtx->deviceIndex].devSelection = devSelection;
	pDataPacket[pDevCtx->deviceIndex].ackType = ACK_ON_PROCESS;
	status = Radar_formEthDataPacket(&pDataPacket[pDevCtx->deviceIndex], \
									&pDevCtrlPrms[pDevCtx->deviceIndex], \
		SENSOR_CONFIG_SET_SOP, DATA_HEADER_LENGTH + sizeof(SOPmode),
		(UINT8*)&SOPmode);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", \
					pDevCtrlPrms[pDevCtx->deviceIndex].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("Device [%d] [GN]%s", pDevCtx->deviceIndex, tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[pDevCtx->deviceIndex].respParam, \
						pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize);
	return status;
}

/** @fn STATUS getSOPMode(TDADevHandle_t hdl)
*
*   @brief Retrieves the SOP mode on the AWR device
*   @param[in] hdl - device handle
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS getSOPMode(TDADevHandle_t hdl)
{
	TDADevCtx_t*    pDevCtx = (TDADevCtx_t*)hdl;
	UINT8 devSelection = createDevMapFromDevId(pDevCtx->deviceIndex);
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[pDevCtx->deviceIndex].devSelection = devSelection;
	pDataPacket[pDevCtx->deviceIndex].ackType = ACK_ON_PROCESS;
	status = Radar_formEthDataPacket(&pDataPacket[pDevCtx->deviceIndex], \
									&pDevCtrlPrms[pDevCtx->deviceIndex], \
		SENSOR_CONFIG_GET_SOP, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", \
					pDevCtrlPrms[pDevCtx->deviceIndex].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("Device [%d] [GN]%s", pDevCtx->deviceIndex, tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[pDevCtx->deviceIndex].respParam, \
						pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize);
	return status;
}

/** @fn STATUS gpioGetValue(unsigned int DeviceMap, unsigned int gpioBase, unsigned int gpioPin)
*
*   @brief Retrieves the value of the given GPIO
*   @param[in] DeviceMap - device map
*   @param[in] gpioBase - GPIO pad
*   @param[in] gpioPin - GPIO pin
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS gpioGetValue(unsigned int DeviceMap, unsigned int gpioBase, unsigned int gpioPin)
{
	unsigned int devId = getDevIdFromDevMap(DeviceMap);
	TDADevHandle_t hdl = TDAGetDeviceCtx(devId);
	TDADevCtx_t*    pDevCtx = (TDADevCtx_t*)hdl;
	UINT8 devSelection = createDevMapFromDevId(pDevCtx->deviceIndex);
	INT32 status = SYSTEM_LINK_STATUS_SOK;

	s_gpioConfig_t gpioConfig;
	gpioConfig.gpioBase = gpioBase;
	gpioConfig.gpioPin = gpioPin;
	gpioConfig.gpioValue = 0;

	pDataPacket[pDevCtx->deviceIndex].devSelection = devSelection;
	pDataPacket[pDevCtx->deviceIndex].ackType = ACK_ON_PROCESS;
	status = Radar_formEthDataPacket(&pDataPacket[pDevCtx->deviceIndex], \
									&pDevCtrlPrms[pDevCtx->deviceIndex], \
		SENSOR_CONFIG_GPIO_GET, DATA_HEADER_LENGTH + sizeof(gpioConfig),
		(UINT8*)&gpioConfig);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", \
					pDevCtrlPrms[pDevCtx->deviceIndex].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("Device [%d] [GN]%s", pDevCtx->deviceIndex, tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[pDevCtx->deviceIndex].respParam, \
						pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize);
	return status;
}

/** @fn STATUS gpioSetValue(unsigned int DeviceMap, unsigned int gpioBase, 
* 							unsigned int gpioPin, unsigned int gpioVal)
*
*   @brief Sets the value of the given GPIO
*   @param[in] DeviceMap - device map
*   @param[in] gpioBase - GPIO pad
*   @param[in] gpioPin - GPIO pin
*   @param[in] gpioVal - GPIO value
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS gpioSetValue(unsigned int DeviceMap, unsigned int gpioBase, unsigned int gpioPin, \
					unsigned int gpioVal)
{
	unsigned int devId = getDevIdFromDevMap(DeviceMap);
	TDADevHandle_t hdl = TDAGetDeviceCtx(devId);
	TDADevCtx_t*    pDevCtx = (TDADevCtx_t*)hdl;
	UINT8 devSelection = createDevMapFromDevId(pDevCtx->deviceIndex);
	INT32 status = SYSTEM_LINK_STATUS_SOK;

	s_gpioConfig_t gpioConfig;
	gpioConfig.gpioBase = gpioBase;
	gpioConfig.gpioPin = gpioPin;
	gpioConfig.gpioValue = gpioVal;

	pDataPacket[pDevCtx->deviceIndex].devSelection = devSelection;
	pDataPacket[pDevCtx->deviceIndex].ackType = ACK_ON_RECEIVE;
	status = Radar_formEthDataPacket(&pDataPacket[pDevCtx->deviceIndex], \
									&pDevCtrlPrms[pDevCtx->deviceIndex], \
		SENSOR_CONFIG_GPIO_SET, DATA_HEADER_LENGTH + sizeof(gpioConfig),
		(UINT8*)&gpioConfig);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", \
					pDevCtrlPrms[pDevCtx->deviceIndex].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("Device [%d] [GN]%s", pDevCtx->deviceIndex, tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[pDevCtx->deviceIndex].respParam, \
						pDevCtrlPrms[pDevCtx->deviceIndex].respParamSize);
	return status;
}

/** @fn STATUS startRecord()
*
*   @brief Start recording the data on CSI2 lines
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS startRecord()
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_PROCESS;
	pDataPacket[4].devSelection = 32;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_DATA_START_RECORD, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/** @fn STATUS stopRecord()
*
*   @brief Stop recording the data on CSI2 lines
*
*   @return int Success - 0, Failure - Error Code
*/
STATUS stopRecord()
{
	INT32 status = SYSTEM_LINK_STATUS_SOK;
	pDataPacket[4].ackType = ACK_ON_PROCESS;
	pDataPacket[4].devSelection = 32;
	status = Radar_formEthDataPacket(&pDataPacket[4], &pDevCtrlPrms[4],
		CAPTURE_DATA_STOP_RECORD, DATA_HEADER_LENGTH,
		NULL);
	if (status != SYSTEM_LINK_STATUS_SOK)
	{
		DEBUG_PRINT("# ERROR: Ethernet packet formation failed\n");
		return SYSTEM_LINK_STATUS_EFAIL;
	}

	/* Create data buffer and print it into file */
	char tempBuff[2048] = { 0 };
	int offset = 0;
	for (unsigned int i = 0; i < pDevCtrlPrms[4].respParamSize; i++)
	{
		int j = 0;
		j = sprintf(&tempBuff[offset], "0x%02X ", pDevCtrlPrms[4].respParam[i]);
		offset = offset + j;
	}
	sprintf(&tempBuff[offset], "\r\n");
	DEBUG_PRINT("[GN]%s", tempBuff);

	handleDevCtrl((UINT8*)&pDevCtrlPrms[4].respParam, pDevCtrlPrms[4].respParamSize);
	return status;
}

/********************** mmWaveLink Hooks related functions *******************/

TDADevCtx_t* TDAInitDeviceCtx(unsigned char ucDevId)
{
	TDADevCtx_t* pDevCtx = NULL;

	if (ucDevId <= TDA_NUM_CONNECTED_DEVICES_MAX)
	{
		pDevCtx = &(gTDA_devCtx[ucDevId]);
		pDevCtx->deviceIndex = ucDevId;
		pDevCtx->deviceEnabled = FALSE;
		pDevCtx->irqMasked = FALSE;
		pDevCtx->hostIntrThread.threadHdl = 0;
		pDevCtx->hostIntrThread.threadID = 0;
		pDevCtx->hostIntrThread.handler = 0;
		pDevCtx->hostIntrThread.pValue = 0;
		InitializeCriticalSection(&(pDevCtx->cs));
	}

	return pDevCtx;
}

int TDAClearDeviceCtx(TDADevCtx_t* pDevCtx)
{
	int retVal = RLS_RET_CODE_EFAIL;

	if (pDevCtx != NULL)
	{
		/* Delete Critical Section */
		DeleteCriticalSection(&(pDevCtx->cs));
		/* Clear device context */
		pDevCtx->deviceEnabled = FALSE;
		retVal = RLS_RET_CODE_OK;
	}

	return retVal;
}

TDADevHandle_t TDAGetDeviceCtx(unsigned char ucDevId)
{
	TDADevCtx_t* pDevCtx = NULL;

	if (ucDevId <= TDA_NUM_CONNECTED_DEVICES_MAX)
	{
		pDevCtx = &(gTDA_devCtx[ucDevId]);
		pDevCtx->deviceIndex = ucDevId;
	}

	return (TDADevHandle_t)pDevCtx;
}

TDADevHandle_t TDACommOpen(unsigned char deviceIndex, unsigned int flags)
{
	TDADevCtx_t*    pDevCtx;

	pDevCtx = TDAInitDeviceCtx(deviceIndex);

	if (NULL != pDevCtx)
	{
		IGNORE_WARNING_UNUSED_VAR(flags);
		return (TDADevHandle_t)pDevCtx;
	}

	return NULL;
}

int TDACommClose(TDADevHandle_t hdl)
{
	TDADevCtx_t*   pDevCtx = (TDADevCtx_t*)hdl;

	TDACloseDevice(pDevCtx);

	return RLS_RET_CODE_OK;
}

int TDACloseDevice(TDADevCtx_t* pDevCtx)
{
	int             error = RLS_RET_CODE_OK;
	unsigned char   Continue = 0;

	if (pDevCtx == NULL)
	{
		return RLS_RET_CODE_EFAIL;
	}

	return RLS_RET_CODE_OK;
}

DWORD WINAPI TDAHostIrqThread(LPVOID pParam)
{
	TDADevCtx_t* pDevCtx = (TDADevCtx_t*)pParam;
	TDA_HostIntrThreadLoop[pDevCtx->deviceIndex] = 1;
	TDA_NumOfRunningSPIThreads++;

	while (TDA_HostIntrThreadLoop[pDevCtx->deviceIndex])
	{
		WaitForSingleObject(pDevCtx->hostIntrThread.eventHandle, INFINITE);
		if ((!pDevCtx->deviceEnabled) || (NULL == pDevCtx->hostIntrThread.handler) || \
			(pDevCtx->irqMasked))
		{
			continue;
		}
		else
		{
			ResetEvent(pDevCtx->hostIntrThread.eventHandle);
			pDevCtx->hostIntrThread.handler(pDevCtx->deviceIndex, (TDADevHandle_t)pDevCtx);
		}

	}
	TDA_NumOfRunningSPIThreads--;
	return RLS_RET_CODE_OK;
}

int TDADisableDeviceThread(unsigned char deviceIndex)
{
	TDADevCtx_t*   pDevCtx;
	pDevCtx = (TDADevCtx_t *)TDAGetDeviceCtx(deviceIndex);
	pDevCtx->deviceEnabled = FALSE;

	if (pDevCtx->hostIntrThread.threadHdl != NULL)
	{
		TDA_HostIntrThreadLoop[pDevCtx->deviceIndex] = 0;
		/* send a summy event to unblock the thread */
		SetEvent(pDevCtx->hostIntrThread.eventHandle);
		/* Wait for polling thread to terminate */
		WaitForSingleObject(pDevCtx->hostIntrThread.threadHdl, INFINITE);
		CloseHandle(pDevCtx->hostIntrThread.eventHandle);
		pDevCtx->hostIntrThread.eventHandle = NULL;
		pDevCtx->hostIntrThread.threadHdl = NULL;

	}

	return RLS_RET_CODE_OK;
}

int TDAEnableDeviceThread(unsigned char deviceIndex)
{
	TDADevCtx_t*    pDevCtx;
	int             retVal = RLS_RET_CODE_OK;

	pDevCtx = (TDADevCtx_t *)TDAGetDeviceCtx(deviceIndex);
	if (NULL != pDevCtx)
	{
		/* if the device is already enabled return */
		if (pDevCtx->deviceEnabled == TRUE)
		{
			return RLS_RET_CODE_OK;
		}

		pDevCtx->deviceEnabled = TRUE;
		/* Starting Interrupt Thread after Power on to avoid Spurious Interrupt */
		if (NULL != pDevCtx->hostIntrThread.handler)
		{
			TDA_HostIntrThreadLoop[pDevCtx->deviceIndex] = 0;
			if (pDevCtx->hostIrqRxHigh == 1)
			{
				pDevCtx->hostIntrThread.eventHandle = CreateEvent(NULL, TRUE, TRUE, NULL);
			}
			else
			{
				pDevCtx->hostIntrThread.eventHandle = CreateEvent(NULL, TRUE, FALSE, NULL);
			}
			/* Create Host IRQ Polling Thread */
			pDevCtx->hostIntrThread.threadHdl = CreateThread(NULL, 0, TDAHostIrqThread, \
												pDevCtx, 0, &pDevCtx->hostIntrThread.threadID);
			/* wait for the thread to start */
			while (TDA_HostIntrThreadLoop[pDevCtx->deviceIndex] == 0)
			{
				Sleep(1);
			}

			retVal = RLS_RET_CODE_OK;
		}
	}
	else
	{
		retVal = RLS_RET_CODE_EFAIL;
	}

	return retVal;
}

int TDADisableDevice(unsigned char deviceIndex)
{
	TDADevCtx_t*   pDevCtx;
	pDevCtx = (TDADevCtx_t *)TDAGetDeviceCtx(deviceIndex);
	pDevCtx->deviceEnabled = FALSE;

	if (pDevCtx->hostIntrThread.threadHdl != NULL)
	{
		TDAStopIrqPollingThread(pDevCtx);
		pDevCtx->hostIntrThread.threadHdl = NULL;
	}

	Sleep(200);

	return RLS_RET_CODE_OK;
}

int TDAEnableDevice(unsigned char deviceIndex)
{
	TDADevCtx_t*    pDevCtx;
	int             retVal = RLS_RET_CODE_OK;

	pDevCtx = (TDADevCtx_t *)TDAGetDeviceCtx(deviceIndex);

	if (NULL != pDevCtx)
	{
		pDevCtx->deviceEnabled = TRUE;
		/* Starting Interrupt Thread after Power on to avoid Spurious Interrupt */
		if (NULL != pDevCtx->hostIntrThread.handler)
		{
			TDAStartIrqPollingThread(pDevCtx);
		}
	}
	else
	{
		retVal = RLS_RET_CODE_EFAIL;
	}

	return retVal;
}

int TDALockDevice(TDADevCtx_t*  pDevCtx)
{
	EnterCriticalSection(&pDevCtx->cs);
	return RLS_RET_CODE_OK;
}

int TDAUnlockDevice(TDADevCtx_t*  pDevCtx)
{
	LeaveCriticalSection(&pDevCtx->cs);
	return RLS_RET_CODE_OK;
}

void TDACommIRQMask(TDADevHandle_t hdl)
{
	TDADevCtx_t* pDevCtx = (TDADevCtx_t *)hdl;

	TDALockDevice(pDevCtx);
	pDevCtx->irqMasked = TRUE;
	TDAUnlockDevice(pDevCtx);
}

void TDACommIRQUnMask(TDADevHandle_t hdl)
{
	TDADevCtx_t* pDevCtx = (TDADevCtx_t *)hdl;

	TDALockDevice(pDevCtx);
	pDevCtx->irqMasked = FALSE;
	TDAUnlockDevice(pDevCtx);
}

int TDADeviceWaitIrqStatus(TDADevHandle_t hdl, unsigned char level)
{
	TDADevCtx_t*    pDevCtx = (TDADevCtx_t*)hdl;
	int loopCount = 0;
	int maxLoopCounts = 0xFFFF;

	do
	{
		Sleep(1);
		loopCount++;
	} while ((1 == pDevCtx->hostIrqRxHigh) && (loopCount < maxLoopCounts));

	if (pDevCtx->hostIrqRxHigh == 0)
	{
		return RLS_RET_CODE_OK;
	}
	else
	{
		return RLS_RET_CODE_EFAIL;
	}
}

int TDAWaitForIrq(TDADevCtx_t* pDevCtx)
{
	int             loopCount = 0;
	/* --------------- Important note-----------------------------*/
	/* This loop was reduced to 100 to prevent a problem that it has an IRQ ready but it ignores
	This loop will cause a fast restart of the IRQ port and will continue normal activity */
	int             MaxWaitLoops = 1;

	do
	{
		Sleep(1);
		loopCount++;
		/* Added this one for letting WIN7 yield for the next thread.
			In case we don't have it - the do/while loop will not yield and
			we will delay other threads by waiting for this thread */
		SwitchToThread();
	} while ((0 == pDevCtx->hostIrqRxHigh) && (loopCount < MaxWaitLoops));

	if (pDevCtx->hostIrqRxHigh == 0)
	{
		return RLS_RET_CODE_EFAIL;
	}

	return RLS_RET_CODE_OK;
}

DWORD WINAPI TDAPollingThreadEntrySpi(LPVOID pParam)
{
	TDADevCtx_t* pDevCtx = (TDADevCtx_t*)pParam;
	int waitIrqRes = 0;
	unsigned int devPolled = 0;

	TDA_HostIntrThreadLoop[pDevCtx->deviceIndex] = 1;
	TDA_NumOfRunningSPIThreads++;

	while (TDA_HostIntrThreadLoop[0])
	{
		pDevCtx = &gTDA_devCtx[devPolled];
		devPolled++;
		devPolled = devPolled % TDA_NUM_CONNECTED_DEVICES_MAX;
		if (!pDevCtx->deviceEnabled)
		{
			continue;
		}

		/* Wait for Host IRQ */
		waitIrqRes = TDAWaitForIrq(pDevCtx);
		/* Check if Device is Enabled, IRQ is received and is not in masked State */

		if ((!pDevCtx->deviceEnabled) || (NULL == pDevCtx->hostIntrThread.handler) || \
			(pDevCtx->irqMasked) || (0 != waitIrqRes))
		{
			continue;
		}

		/* Check if Interrupt handler is registered */
		if (pDevCtx->hostIntrThread.handler)
		{
			pDevCtx->hostIntrThread.handler(pDevCtx->deviceIndex, (TDADevHandle_t)pDevCtx);
		}

		if (TDA_NumOfRunningSPIThreads > TDA_NUM_CONNECTED_DEVICES_MAX)
		{
			TDA_NumOfRunningSPIThreads--;
			pDevCtx->hostIntrThread.threadHdl = NULL;
			return RLS_RET_CODE_OK;
		}
	}
	TDA_NumOfRunningSPIThreads--;
	return RLS_RET_CODE_OK;
}

int TDAStartIrqPollingThread(TDADevCtx_t* pDevCtx)
{
	int             error = RLS_RET_CODE_OK;
	DWORD(WINAPI *pPollThreadFunc) (LPVOID) = NULL;
	DWORD           currThread = GetCurrentThreadId();

	if (pDevCtx->deviceIndex != 0)
	{
		if (gTDA_devCtx[0].hostIntrThread.threadHdl != 0)
		{
			pDevCtx->hostIntrThread.threadHdl = gTDA_devCtx[0].hostIntrThread.threadHdl;
			return RLS_RET_CODE_OK;
		}
		else
		{
			return RLS_RET_CODE_EFAIL;
		}
	}

	TDA_HostIntrThreadLoop[pDevCtx->deviceIndex] = 0;

	/* Host IRQ Polling Thread for SPI */
	pPollThreadFunc = TDAPollingThreadEntrySpi;

	if (0 == pDevCtx->hostIntrThread.threadHdl)
	{
		/* Create Host IRQ Polling Thread */
		pDevCtx->hostIntrThread.threadHdl = CreateThread(NULL, 0, pPollThreadFunc, pDevCtx, 0, \
			&pDevCtx->hostIntrThread.threadID);
	}
	else
	{
		TDALockDevice(pDevCtx);
		if (pDevCtx->hostIntrThread.threadID != currThread)
		{
			TerminateThread(pDevCtx->hostIntrThread.threadHdl, 0);
			Sleep(200);
		}

		pDevCtx->hostIntrThread.threadHdl = CreateThread(NULL, 0, pPollThreadFunc, pDevCtx, 0, \
			&pDevCtx->hostIntrThread.threadID);
		TDAUnlockDevice(pDevCtx);
	}

	/* wait for the thread to start */
	while (TDA_HostIntrThreadLoop[pDevCtx->deviceIndex] == 0)
	{
		Sleep(1);
	}

	return RLS_RET_CODE_OK;
}

int TDAStopIrqPollingThread(TDADevCtx_t* pDevCtx)
{
	int             error = RLS_RET_CODE_OK;
	DWORD           currThread = GetCurrentThreadId();

	if (pDevCtx->deviceIndex != 0)
	{
		pDevCtx->hostIntrThread.threadHdl = NULL;
		return RLS_RET_CODE_OK;
	}

	TDA_HostIntrThreadLoop[pDevCtx->deviceIndex] = 0;

	if (pDevCtx->hostIntrThread.threadHdl && (pDevCtx->hostIntrThread.threadID != currThread))
	{
		/* Wait for polling thread to terminate */
		WaitForSingleObject(pDevCtx->hostIntrThread.threadHdl, INFINITE);
	}

	pDevCtx->hostIntrThread.threadHdl = NULL;
	gTDA_devCtx[1].hostIntrThread.threadHdl = NULL;
	gTDA_devCtx[2].hostIntrThread.threadHdl = NULL;
	gTDA_devCtx[3].hostIntrThread.threadHdl = NULL;

	return RLS_RET_CODE_OK;
}

unsigned int getDevIdFromDevMap(unsigned int deviceMap)
{
	int devInd = 0;
	unsigned int devMap = deviceMap;
	while (devMap != 0)
	{
		if ((devMap & 0x1) == 1)
		{
			break;
		}
		else
		{
			devInd++;
		}
		devMap = (devMap >> 1);
	}
	return devInd;
}

unsigned int createDevMapFromDevId(unsigned int deviceId)
{
	int devMap = 1 << deviceId;
	return devMap;
}

/********************** Network related functions ****************************/

#if !defined(linux)
void socketsStartup()
{
	WORD         wVersionRequested;
	WSADATA      wsaData;

	wVersionRequested = MAKEWORD(1, 1);
	if (WSAStartup(wVersionRequested, &wsaData)) {
		DEBUG_PRINT("\r\nUnable to initialize WinSock for host info");
		exit(EXIT_FAILURE);
	}
}
#endif

int Network_init()
{
	socketsStartup();
	return 0;
}

int Network_deInit()
{
	socketsShutdown();
	return 0;
}

int Network_connect(Network_SockObj *pObj, char *ipAddr, UINT32 port)
{
	int sin_size;
	struct hostent *host;
	struct sockaddr_in server;

	host = gethostbyname(ipAddr);

	strcpy_s(gNetworkTDA_obj.ipAddr, _countof(gNetworkTDA_obj.ipAddr), ipAddr);

	gNetworkTDA_obj.serverPort = port;

	pObj->clientSocketId = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (pObj->clientSocketId == INVALID_SOCKET) {
		DEBUG_PRINT("# ERROR: NETWORK: Socket open failed (%s:%d)!!!\n", \
						gNetworkTDA_obj.ipAddr, gNetworkTDA_obj.serverPort);
		return RLS_RET_CODE_EFAIL;
	}

	int flags = 1;
	if (setsockopt(pObj->clientSocketId, IPPROTO_TCP, TCP_NODELAY, (void *)&flags, \
																		sizeof(flags)))
	{
		DEBUG_PRINT("# ERROR: setsocketopt(), TCP_NODELAY");
        return RLS_RET_CODE_EFAIL;
	};

	int freq = 1; // can be 1..255, default is 2
	int bytes;

	#define SIO_TCP_SET_ACK_FREQUENCY           _WSAIOW(IOC_VENDOR,23)
	
	if (WSAIoctl(pObj->clientSocketId, SIO_TCP_SET_ACK_FREQUENCY, &freq, sizeof(freq), \
												NULL, 0, &bytes, NULL, NULL))
	{
		DEBUG_PRINT("# ERROR: WSAIoctl(), SIO_TCP_SET_ACK_FREQUENCY");
        return RLS_RET_CODE_EFAIL;
	}

	server.sin_family = AF_INET;
	server.sin_port = htons(port);
	server.sin_addr = *((struct in_addr *)host->h_addr);
	memset(&(server.sin_zero), 0, 8);

	sin_size = sizeof(server);

	if (connect(pObj->clientSocketId, (struct sockaddr *)&server, sin_size) == -1)
	{
		DEBUG_PRINT("# ERROR: NETWORK: Server connect Failed (%s:%d)!!!\n", \
						gNetworkTDA_obj.ipAddr, gNetworkTDA_obj.serverPort);
		return RLS_RET_CODE_EFAIL;
	}

	DEBUG_PRINT("# NETWORK: Connected to Server (%s:%d)!!!\n", gNetworkTDA_obj.ipAddr, \
														gNetworkTDA_obj.serverPort);

	return RLS_RET_CODE_OK;
}

int Network_close(Network_SockObj *pObj)
{
	int status = closesocket(pObj->clientSocketId);
	if (status != 0)
	{
		DEBUG_PRINT("ERROR: Closing the socket failed with error: %d\n", WSAGetLastError());
		return RLS_RET_CODE_EFAIL;
	}
	return status;
}

int Network_read(Network_SockObj *pObj, UINT8 *dataBuf, UINT32 *dataSize)
{
	int actDataSize = 0;
	UINT32 tmpDataSize;

	tmpDataSize = *dataSize;

	while (tmpDataSize > 0)
	{
		actDataSize = recv(pObj->clientSocketId, (void*)dataBuf, tmpDataSize, 0);
		if (actDataSize <= 0)
		{
			*dataSize = 0;
			DEBUG_PRINT("ERROR: recv failed in Network_read!\n");
			return RLS_RET_CODE_EFAIL;
		}
		dataBuf += actDataSize;
		tmpDataSize -= actDataSize;
	}

	return 0;
}

int Network_readString(Network_SockObj *pObj, UINT8 *dataBuf, UINT32 maxDataSize)
{
	int actDataSize = 0;

	memset(dataBuf, 0, maxDataSize);

	actDataSize = recv(pObj->clientSocketId, (void*)dataBuf, maxDataSize - 1, 0);
	if (actDataSize <= 0)
	{
		dataBuf[0] = 0;
		DEBUG_PRINT("ERROR: recv failed in Network_readString!\n");
		return RLS_RET_CODE_EFAIL;
	}

	return 0;
}

int Network_write(Network_SockObj *pObj, UINT8 *dataBuf, UINT32 dataSize)
{
	int actDataSize = 0;

	while (dataSize > 0) {
		actDataSize = send(pObj->clientSocketId, (void*)dataBuf, dataSize, 0);

		if (actDataSize <= 0)
			break;
		dataBuf += actDataSize;
		dataSize -= actDataSize;
	}

	if (dataSize > 0)
	{
		DEBUG_PRINT("ERROR: send failed in Network_write!\n");
		return RLS_RET_CODE_EFAIL;
	}

	return 0;
}

INT32 Network_waitRead(Network_SockObj *pObj)
{
	int             status;
	fd_set          master_set;
	struct			timeval timeout;

	FD_ZERO(&master_set);
	FD_SET(pObj->clientSocketId, &master_set);

	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;
	status = select(pObj->clientSocketId + 1, &master_set, NULL, NULL, &timeout);

	if (status < 0)
	{
		DEBUG_PRINT(" NETWORK: Select Failed\n");
		status = -1;
	}
	else if (status == 0)
	{
		status = 0;
	}
	else
	{
		if (FD_ISSET(pObj->clientSocketId, &master_set))
		{
			status = 1;
		}
	}

	/* NO connection, retry */
	return status;
}

int ConnectToServer()
{
	int status;

	DEBUG_PRINT("# INFO: Network: Connecting to the server %s:%d ...\n", \
					gNetworkTDA_obj.ipAddr, gNetworkTDA_obj.serverPort);

	status = Network_connect(&gNetwork_SockObj, gNetworkTDA_obj.ipAddr, \
														gNetworkTDA_obj.serverPort);
	if (status < 0)
	{
		DEBUG_PRINT("# ERROR: Cannot connect to server\n");
		return RLS_RET_CODE_EFAIL;
	}
	return status;
}

int CloseConnection()
{
	return Network_close(&gNetwork_SockObj);
}

int SendCommand(void *params, int prmSize)
{
	NetworkTDA_CmdHeader cmdHeader;
	int status;

	memset(&cmdHeader, 0, sizeof(cmdHeader));

	cmdHeader.prmSize = prmSize;

	if (prmSize == 0)
	{
		return -1;
	}

	status = Network_write(&gNetwork_SockObj, (UINT8*)&cmdHeader, sizeof(cmdHeader));
	if (status < 0)
	{
		DEBUG_PRINT("# ERROR: Could not write header\n");
        return status;
	}

	status = Network_write(&gNetwork_SockObj, (UINT8*)params, prmSize);

	if (status < 0)
	{
		DEBUG_PRINT("# ERROR: Could not send data \n");
		return status;
	}

	return status;
}

int RecvResponseParams(UINT8 *pPrm, UINT32 prmSize)
{
	int status;

	status = Network_read(&gNetwork_SockObj, pPrm, &prmSize);

	if (status < 0)
	{
		DEBUG_PRINT("# ERROR: Could not receive data \n");
		return RLS_RET_CODE_EFAIL;
	}

	return status;
}

int RecvResponse(UINT32 *prmSize)
{
	NetworkTDA_CmdHeader cmdHeader;
	UINT32 dataSize;
	int status;

	memset(&cmdHeader, 0, sizeof(cmdHeader));

	dataSize = sizeof(cmdHeader);

	status = Network_read(&gNetwork_SockObj, (UINT8*)&cmdHeader, &dataSize);

	if (status < 0)
	{
		DEBUG_PRINT("# ERROR: Could not read response header\n");
		return RLS_RET_CODE_EFAIL;
	}

	if (dataSize == 4)
	{
		*prmSize = cmdHeader.prmSize;
	}
	return status;
}

void handleDevCtrl(UINT8 *pDataBuf, UINT32 size)
{
	EnterCriticalSection(&gNetwork_Write_cs);
	SendCommand(pDataBuf, size);
	LeaveCriticalSection(&gNetwork_Write_cs);
}


/********************** Computation of CRC *********************/

const uint16_t  hCrc16Table[BSPDRV_AR12XX_CRC_M_CRC_TABLE_SIZE] = { \
	0x0000U,     \
	0x1021U,     \
	0x2042U,     \
	0x3063U,     \
	0x4084U,     \
	0x50A5U,     \
	0x60C6U,     \
	0x70E7U,     \
	0x8108U,     \
	0x9129U,     \
	0xA14AU,     \
	0xB16BU,     \
	0xC18CU,     \
	0xD1ADU,     \
	0xE1CEU,     \
	0xF1EFU,     \
	0x1231U,     \
	0x0210U,     \
	0x3273U,     \
	0x2252U,     \
	0x52B5U,     \
	0x4294U,     \
	0x72F7U,     \
	0x62D6U,     \
	0x9339U,     \
	0x8318U,     \
	0xB37BU,     \
	0xA35AU,     \
	0xD3BDU,     \
	0xC39CU,     \
	0xF3FFU,     \
	0xE3DEU,     \
	0x2462U,     \
	0x3443U,     \
	0x0420U,     \
	0x1401U,     \
	0x64E6U,     \
	0x74C7U,     \
	0x44A4U,     \
	0x5485U,     \
	0xA56AU,     \
	0xB54BU,     \
	0x8528U,     \
	0x9509U,     \
	0xE5EEU,     \
	0xF5CFU,     \
	0xC5ACU,     \
	0xD58DU,     \
	0x3653U,     \
	0x2672U,     \
	0x1611U,     \
	0x0630U,     \
	0x76D7U,     \
	0x66F6U,     \
	0x5695U,     \
	0x46B4U,     \
	0xB75BU,     \
	0xA77AU,     \
	0x9719U,     \
	0x8738U,     \
	0xF7DFU,     \
	0xE7FEU,     \
	0xD79DU,     \
	0xC7BCU,     \
	0x48C4U,     \
	0x58E5U,     \
	0x6886U,     \
	0x78A7U,     \
	0x0840U,     \
	0x1861U,     \
	0x2802U,     \
	0x3823U,     \
	0xC9CCU,     \
	0xD9EDU,     \
	0xE98EU,     \
	0xF9AFU,     \
	0x8948U,     \
	0x9969U,     \
	0xA90AU,     \
	0xB92BU,     \
	0x5AF5U,     \
	0x4AD4U,     \
	0x7AB7U,     \
	0x6A96U,     \
	0x1A71U,     \
	0x0A50U,     \
	0x3A33U,     \
	0x2A12U,     \
	0xDBFDU,     \
	0xCBDCU,     \
	0xFBBFU,     \
	0xEB9EU,     \
	0x9B79U,     \
	0x8B58U,     \
	0xBB3BU,     \
	0xAB1AU,     \
	0x6CA6U,     \
	0x7C87U,     \
	0x4CE4U,     \
	0x5CC5U,     \
	0x2C22U,     \
	0x3C03U,     \
	0x0C60U,     \
	0x1C41U,     \
	0xEDAEU,     \
	0xFD8FU,     \
	0xCDECU,     \
	0xDDCDU,     \
	0xAD2AU,     \
	0xBD0BU,     \
	0x8D68U,     \
	0x9D49U,     \
	0x7E97U,     \
	0x6EB6U,     \
	0x5ED5U,     \
	0x4EF4U,     \
	0x3E13U,     \
	0x2E32U,     \
	0x1E51U,     \
	0x0E70U,     \
	0xFF9FU,     \
	0xEFBEU,     \
	0xDFDDU,     \
	0xCFFCU,     \
	0xBF1BU,     \
	0xAF3AU,     \
	0x9F59U,     \
	0x8F78U,     \
	0x9188U,     \
	0x81A9U,     \
	0xB1CAU,     \
	0xA1EBU,     \
	0xD10CU,     \
	0xC12DU,     \
	0xF14EU,     \
	0xE16FU,     \
	0x1080U,     \
	0x00A1U,     \
	0x30C2U,     \
	0x20E3U,     \
	0x5004U,     \
	0x4025U,     \
	0x7046U,     \
	0x6067U,     \
	0x83B9U,     \
	0x9398U,     \
	0xA3FBU,     \
	0xB3DAU,     \
	0xC33DU,     \
	0xD31CU,     \
	0xE37FU,     \
	0xF35EU,     \
	0x02B1U,     \
	0x1290U,     \
	0x22F3U,     \
	0x32D2U,     \
	0x4235U,     \
	0x5214U,     \
	0x6277U,     \
	0x7256U,     \
	0xB5EAU,     \
	0xA5CBU,     \
	0x95A8U,     \
	0x8589U,     \
	0xF56EU,     \
	0xE54FU,     \
	0xD52CU,     \
	0xC50DU,     \
	0x34E2U,     \
	0x24C3U,     \
	0x14A0U,     \
	0x0481U,     \
	0x7466U,     \
	0x6447U,     \
	0x5424U,     \
	0x4405U,     \
	0xA7DBU,     \
	0xB7FAU,     \
	0x8799U,     \
	0x97B8U,     \
	0xE75FU,     \
	0xF77EU,     \
	0xC71DU,     \
	0xD73CU,     \
	0x26D3U,     \
	0x36F2U,     \
	0x0691U,     \
	0x16B0U,     \
	0x6657U,     \
	0x7676U,     \
	0x4615U,     \
	0x5634U,     \
	0xD94CU,     \
	0xC96DU,     \
	0xF90EU,     \
	0xE92FU,     \
	0x99C8U,     \
	0x89E9U,     \
	0xB98AU,     \
	0xA9ABU,     \
	0x5844U,     \
	0x4865U,     \
	0x7806U,     \
	0x6827U,     \
	0x18C0U,     \
	0x08E1U,     \
	0x3882U,     \
	0x28A3U,     \
	0xCB7DU,     \
	0xDB5CU,     \
	0xEB3FU,     \
	0xFB1EU,     \
	0x8BF9U,     \
	0x9BD8U,     \
	0xABBBU,     \
	0xBB9AU,     \
	0x4A75U,     \
	0x5A54U,     \
	0x6A37U,     \
	0x7A16U,     \
	0x0AF1U,     \
	0x1AD0U,     \
	0x2AB3U,     \
	0x3A92U,     \
	0xFD2EU,     \
	0xED0FU,     \
	0xDD6CU,     \
	0xCD4DU,     \
	0xBDAAU,     \
	0xAD8BU,     \
	0x9DE8U,     \
	0x8DC9U,     \
	0x7C26U,     \
	0x6C07U,     \
	0x5C64U,     \
	0x4C45U,     \
	0x3CA2U,     \
	0x2C83U,     \
	0x1CE0U,     \
	0x0CC1U,     \
	0xEF1FU,     \
	0xFF3EU,     \
	0xCF5DU,     \
	0xDF7CU,     \
	0xAF9BU,     \
	0xBFBAU,     \
	0x8FD9U,     \
	0x9FF8U,     \
	0x6E17U,     \
	0x7E36U,     \
	0x4E55U,     \
	0x5E74U,     \
	0x2E93U,     \
	0x3EB2U,     \
	0x0ED1U,     \
	0x1EF0U      \
};

/** @fn int32_t Bsp_ar12xxComputeCrc(uint8_t* wMbDataBaseAdd, uint32_t hNBytes,
* 									 uint8_t crcLen, uint8_t* outCrc);
*
*   @brief Computes CRC16-CCITT computation. \n
*		   The mailbox data is is read 64bits (8 bytes) at one shot using LDMA
*		   instruction to reduce the access time, each bytes of the message is 
*		   passed to LUT based CRC algorithm. \n
*   @param[in] wMbDataBaseAdd - The message data source base address
*   @param[in] hNBytes - Num of bytes to read for CRC computation
*   @param[in] crcLen - Length/Type of CRC
*   @param[in] outCrc - Pointer to CRC output buffer
*
*   @return int Success - 0, Failure - Error Code
*/
int32_t Bsp_ar12xxComputeCrc(uint8_t* wMbDataBaseAdd,
	uint32_t hNBytes,
	uint8_t  crcLen,
	uint8_t* outCrc)
{
	uint16_t      hRemainder = (uint16_t)BSPDRV_AR12XX_CRC_INITIAL_REMAINDER;
	uint64_t      lDData;
	uint32_t      lDData_temp;

	uint16_t      hIndex;
	uint16_t      hNDoubleWords;
	uint8_t       cNRemainderBytes;
	uint8_t       cData;
	uint8_t       cByte;
	int32_t       retVal = BSP_SOK;

	/* The length can not be zero */
	if (BSPDRV_AR12XX_CRC_M_ZERO == hNBytes)
	{
		retVal = BSP_EBADARGS;
	}
	if (retVal == BSP_SOK)
	{
		/* To reduce MB access time perform 32 bit bus access at a time */
		/* Num of double words -  8 bytes */
		hNDoubleWords = (uint16_t)(hNBytes >> BSPDRV_AR12XX_CRC_M_TWO);
		/* Num of bytes less than 4 bytes */
		cNRemainderBytes = (uint8_t)(hNBytes & BSPDRV_AR12XX_CRC_M_THREE);

		/* Read 8 bytes at a time and perform CRC calculation */
		for (hIndex = BSPDRV_AR12XX_CRC_M_ZERO; hIndex < hNDoubleWords; hIndex++)
		{
			/*
				* Read 64 bit data from MB in little-endian mode
				* - the first byte of the message is in LSB
				*/
			lDData_temp = BSPDRV_AR12XX_CRC_M_REG_READ32(wMbDataBaseAdd);
			lDData = (uint64_t)lDData_temp;
			wMbDataBaseAdd += (uint32_t)BSPDRV_AR12XX_CRC_M_FOUR;

			/*
				* Divide the message by the polynomial, a byte at a time.
				*/
			for (cByte = BSPDRV_AR12XX_CRC_M_ZERO;
				cByte < BSPDRV_AR12XX_CRC_M_FOUR; cByte++)
			{
				/* Reminder upper byte xor with incoming data before dividing
					* with polynomial */
				cData = (uint8_t)((uint64_t)lDData & (uint64_t)0x000000FF) ^ \
					(uint8_t)(hRemainder >> (BSPDRV_AR12XX_CRC_M_EIGHT));
				/*
					* Perform poly divide on incoming data - the result from the
					* LUT, xor upper byte of result with lower byte of previous
					* reminder - this is to xor previous reminder
					* lower byte with next incoming data
					*/
				hRemainder = hCrc16Table[cData] ^
					((uint16_t)(hRemainder << BSPDRV_AR12XX_CRC_M_EIGHT));
				/* Get next byte in LSB - that is the next byte of message */
				lDData = lDData >> BSPDRV_AR12XX_CRC_M_EIGHT;
			}
		}

		/*
			* Read each bytes one at a time and perform CRC calculation if number
			* of bytes less than 4 bytes
			*/
		for (hIndex = BSPDRV_AR12XX_CRC_M_ZERO;
			hIndex < cNRemainderBytes; hIndex++)
		{
			/* Read 1 byte data from MB */
			cData = BSPDRV_AR12XX_CRC_M_REG_READ8(wMbDataBaseAdd);
			wMbDataBaseAdd += (uint32_t)BSPDRV_AR12XX_CRC_M_ONE;

			/*
				* Divide the message by the polynomial, a byte at a time.
				*/

				/* Reminder upper byte xor with incoming data before
				* dividing with polynomial */
			cData = cData ^ (uint8_t)(hRemainder >> BSPDRV_AR12XX_CRC_M_EIGHT);
			/*
				* Perform poly divide on incoming data - the result from the LUT,
				* xor upper byte of result with lower byte of previous reminder -
				* this is to xor previous reminder lower byte with next incoming
				* data
				*/
			hRemainder = hCrc16Table[cData] ^
				((uint16_t)(hRemainder << BSPDRV_AR12XX_CRC_M_EIGHT));
		}

		/*
			* The final remainder is the CRC.
			*/
		if (outCrc != NULL)
		{
			*((uint16_t*)outCrc) = hRemainder;
		}
	}
	return retVal;
}

/*
 * END OF mmwl_port_ethernet.c FILE
 */
