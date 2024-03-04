/*******************************************************************************
**
**      GSC-18128-1, "Core Flight Executive Version 6.7"
**
**      Copyright (c) 2006-2019 United States Government as represented by
**      the Administrator of the National Aeronautics and Space Administration.
**      All Rights Reserved.
**
**      Licensed under the Apache License, Version 2.0 (the "License");
**      you may not use this file except in compliance with the License.
**      You may obtain a copy of the License at
**
**        http://www.apache.org/licenses/LICENSE-2.0
**
**      Unless required by applicable law or agreed to in writing, software
**      distributed under the License is distributed on an "AS IS" BASIS,
**      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**      See the License for the specific language governing permissions and
**      limitations under the License.
**
** File: rover_app.c
**
** Purpose:
**   This file contains the source code for the ros App.
**
*******************************************************************************/

/*
** Include Files:
*/
#include "rover_app_events.h"
#include "rover_app_version.h"
#include "rover_app.h"
#include "rover_app_table.h"

#include <string.h>

#include <math.h>

/*
** global data
*/
RoverAppData_t RoverAppData;
RoverAppOdometry_t lastOdomMsg;

void HighRateControLoop(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/* RoverAppMain() -- Application entry point and main process loop         */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void RoverAppMain(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
    ** Create the first Performance Log entry
    */
    CFE_ES_PerfLogEntry(ROVER_APP_PERF_ID);

    /*
    ** Perform application specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = RoverAppInit();

    if (status != CFE_SUCCESS)
    {
        RoverAppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    /*
    ** Runloop
    */
    while (CFE_ES_RunLoop(&RoverAppData.RunStatus) == true)
    {
        /*
        ** Performance Log Exit Stamp
        */
        CFE_ES_PerfLogExit(ROVER_APP_PERF_ID);

        /* Pend on receipt of command packet */
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, RoverAppData.CommandPipe, CFE_SB_PEND_FOREVER);

        if (status == CFE_SUCCESS)
        {
            RoverAppProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_EVS_SendEvent(ROVER_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Rover App: SB Pipe Read Error, App Will Exit");

            RoverAppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }

        /*
        ** Performance Log Entry Stamp
        */
        CFE_ES_PerfLogEntry(ROVER_APP_PERF_ID);
    }

    /*
    ** Performance Log Exit Stamp
    */
    CFE_ES_PerfLogExit(ROVER_APP_PERF_ID);

    CFE_ES_ExitApp(RoverAppData.RunStatus);

} /* End of RoverAppMain() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* RoverAppInit() --  initialization                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 RoverAppInit(void)
{
    int32 status;

    RoverAppData.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Initialize app command execution counters
    */
    RoverAppData.CmdCounter = 0;
    RoverAppData.ErrCounter = 0;
    RoverAppData.square_counter = 0;
    RoverAppData.hk_counter = 0;

    RoverAppData.HkTlm.Payload.state.pose.x = 0.0;
    RoverAppData.HkTlm.Payload.state.pose.y = 0.0;
    RoverAppData.HkTlm.Payload.state.pose.z = 0.0;
    RoverAppData.HkTlm.Payload.state.pose.qx = 0.0;
    RoverAppData.HkTlm.Payload.state.pose.qy = 0.0;
    RoverAppData.HkTlm.Payload.state.pose.qz = 0.0;
    RoverAppData.HkTlm.Payload.state.pose.qw = 0.0;

    /*
    ** Initialize app configuration data
    */
    RoverAppData.PipeDepth = ROVER_APP_PIPE_DEPTH;

    strncpy(RoverAppData.PipeName, "ROVER_APP_PIPE", sizeof(RoverAppData.PipeName));
    RoverAppData.PipeName[sizeof(RoverAppData.PipeName) - 1] = 0;

    /*
    ** Initialize event filter table...
    */
    RoverAppData.EventFilters[0].EventID = ROVER_APP_STARTUP_INF_EID;
    RoverAppData.EventFilters[0].Mask    = 0x0000;
    RoverAppData.EventFilters[1].EventID = ROVER_APP_COMMAND_ERR_EID;
    RoverAppData.EventFilters[1].Mask    = 0x0000;
    RoverAppData.EventFilters[2].EventID = ROVER_APP_COMMANDNOP_INF_EID;
    RoverAppData.EventFilters[2].Mask    = 0x0000;
    RoverAppData.EventFilters[3].EventID = ROVER_APP_COMMANDTWIST_INF_EID;
    RoverAppData.EventFilters[3].Mask    = 0x0000;
    RoverAppData.EventFilters[4].EventID = ROVER_APP_INVALID_MSGID_ERR_EID;
    RoverAppData.EventFilters[4].Mask    = 0x0000;
    RoverAppData.EventFilters[5].EventID = ROVER_APP_LEN_ERR_EID;
    RoverAppData.EventFilters[5].Mask    = 0x0000;
    RoverAppData.EventFilters[6].EventID = ROVER_APP_PIPE_ERR_EID;
    RoverAppData.EventFilters[6].Mask    = 0x0000;

    status = CFE_EVS_Register(RoverAppData.EventFilters, ROVER_APP_EVENT_COUNTS, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("RoverApp: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Initialize housekeeping packet (clear user data area).
    */
    CFE_MSG_Init(&RoverAppData.HkTlm.TlmHeader.Msg, CFE_SB_ValueToMsgId(ROVER_APP_HK_TLM_MID), sizeof(RoverAppData.HkTlm));
    CFE_MSG_Init(&RoverAppData.LastTwist.TlmHeader.Msg, CFE_SB_ValueToMsgId(ROVER_APP_TLM_TWIST_MID), sizeof(RoverAppData.LastTwist));

    /*
    ** Create Software Bus message pipe.
    */
    status = CFE_SB_CreatePipe(&RoverAppData.CommandPipe, RoverAppData.PipeDepth, RoverAppData.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Rover App: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Subscribe to Housekeeping request commands
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(ROVER_APP_SEND_HK_MID), RoverAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Rover App: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Subscribe to ground command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(ROVER_APP_CMD_MID), RoverAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Rover App: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }


    /*
    ** Subscribe to flight odom data
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(ROVER_APP_CMD_ODOM_MID), RoverAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Rover App: Error Subscribing to Odom data, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }

    
    /*
    ** Subscribe to HR wakeup
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(ROVER_APP_HR_CONTROL_MID), RoverAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("Rover App: Error Subscribing to HR Wakeup Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }

    CFE_EVS_SendEvent(ROVER_APP_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "Rover App Initialized.%s",
                      ROVER_APP_VERSION_STRING);

    return (CFE_SUCCESS);

} /* End of RoverAppInit() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  RoverAppProcessCommandPacket                                    */
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the ros    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void RoverAppProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
    //printf("RoverAppProcessCommandPacket() -- we're processing the cmd from MID: 0x%04x\n", CFE_SB_MsgIdToValue(MsgId));
    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case ROVER_APP_CMD_MID:
            RoverAppProcessGroundCommand(SBBufPtr);
            break;

        case ROVER_APP_SEND_HK_MID:
            RoverAppReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        case ROVER_APP_CMD_ODOM_MID:
            RoverAppProcessFlightOdom(SBBufPtr);
            break;

        case ROVER_APP_HR_CONTROL_MID:
            HighRateControLoop();
            break;
            
        default:
            CFE_EVS_SendEvent(ROVER_APP_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "rover app: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }

    return;

} /* End RoverAppProcessCommandPacket */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* RoverAppProcessGroundCommand() -- rover app ground commands                */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void RoverAppProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    printf("RoverAppProcessGroundCommand() -- we're getting a ground command...%d\n", CommandCode);

    /*
    ** Process "known" rover app ground commands
    */
    switch (CommandCode)
    {
        case ROVER_APP_NOOP_CC:
            if (RoverAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(RoverAppNoopCmd_t)))
            {
                RoverAppNoop((RoverAppNoopCmd_t *)SBBufPtr);
            }

            break;

        case ROVER_APP_SET_TWIST_CC:
            if (RoverAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(RoverAppTwistCmd_t)))
            {
                RoverAppCmdTwist((RoverAppTwistCmd_t *)SBBufPtr);
            }

            break;

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(ROVER_APP_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }


    return;

} /* End of RoverAppProcessFlightOdom() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* RoverAppProcessFlightOdom() -- rover app flight odometry                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void RoverAppProcessFlightOdom(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    printf("RoverAppProcessGroundCommand() -- we're getting a flight odometry message ...%d\n", CommandCode);

    // Read
    if (RoverAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(RoverAppCmdRobotState_t)))
    {
       RoverAppCmdRobotState_t* state = (RoverAppCmdRobotState_t *)SBBufPtr;
       
       // Fill the lastState
       lastOdomMsg = state->odom;                     
    }


    return;

} /* End of RoverAppProcessFlightCommand() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  RoverAppReportHousekeeping                                          */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 RoverAppReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{
    printf("RoverAppReportHousekeeping() -- sending odom as part of housekeeping...\n");
    
    /*
    ** Get command execution counters...
    */
    RoverAppData.HkTlm.Payload.CommandErrorCounter = RoverAppData.ErrCounter*2;
    RoverAppData.ErrCounter++;
    RoverAppData.HkTlm.Payload.CommandCounter      = RoverAppData.CmdCounter++;

    OS_printf("RoverAppReportHousekeeping reporting: %d\n", RoverAppData.HkTlm.Payload.CommandCounter);

 
    CFE_SB_TimeStampMsg(&RoverAppData.HkTlm.TlmHeader.Msg);
    CFE_SB_TransmitMsg(&RoverAppData.HkTlm.TlmHeader.Msg, true);

    return CFE_SUCCESS;

} /* End of RoverAppReportHousekeeping() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* RoverAppNoop -- ROS NOOP commands                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 RoverAppNoop(const RoverAppNoopCmd_t *Msg)
{
    CFE_EVS_SendEvent(ROVER_APP_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION, "rover app: NOOP command %s",
                      ROVER_APP_VERSION);

    return CFE_SUCCESS;
} /* End of RoverAppNoop */


int32 RoverAppCmdTwist(const RoverAppTwistCmd_t *Msg)
{
    RoverAppData.LastTwist.twist.linear_x = Msg->twist.linear_x;
    RoverAppData.LastTwist.twist.linear_y = Msg->twist.linear_y; 
    RoverAppData.LastTwist.twist.linear_z = Msg->twist.linear_z;
    RoverAppData.LastTwist.twist.angular_x = Msg->twist.angular_x;
    RoverAppData.LastTwist.twist.angular_y = Msg->twist.angular_y;
    RoverAppData.LastTwist.twist.angular_z = Msg->twist.angular_z;


    CFE_EVS_SendEvent(ROVER_APP_COMMANDTWIST_INF_EID, CFE_EVS_EventType_INFORMATION, "rover app: twist command %s",
                      ROVER_APP_VERSION);

    return CFE_SUCCESS;
    
}

void HighRateControLoop(void) {
    
    // 1. Publish the twist to State in rosfsw (it is like sending a command to the robot)
    // (we should use another name, telemetry is not supposed to command anything)

    // if (RoverAppData.square_counter%1000 == 0)    
    {
    CFE_SB_TimeStampMsg(&RoverAppData.LastTwist.TlmHeader.Msg);
    CFE_SB_TransmitMsg(&RoverAppData.LastTwist.TlmHeader.Msg, true);    
    }

 
    
    // 2. Update the telemetry information        
    RoverAppOdometry_t *st = &lastOdomMsg; //RoverAppGoal.StateTlm;

    RoverAppData.HkTlm.Payload.state.pose.x = st->pose.x;
    RoverAppData.HkTlm.Payload.state.pose.y = st->pose.y;
    RoverAppData.HkTlm.Payload.state.pose.z = st->pose.z;
    RoverAppData.HkTlm.Payload.state.pose.qx = st->pose.qx;
    RoverAppData.HkTlm.Payload.state.pose.qy = st->pose.qy;
    RoverAppData.HkTlm.Payload.state.pose.qz = st->pose.qz;
    RoverAppData.HkTlm.Payload.state.pose.qw = st->pose.qw;

    RoverAppData.HkTlm.Payload.state.twist.linear_x = st->twist.linear_x;
    RoverAppData.HkTlm.Payload.state.twist.linear_y = st->twist.linear_y;
    RoverAppData.HkTlm.Payload.state.twist.linear_z = st->twist.linear_z;
    RoverAppData.HkTlm.Payload.state.twist.angular_x = st->twist.angular_x;
    RoverAppData.HkTlm.Payload.state.twist.angular_y = st->twist.angular_y;
    RoverAppData.HkTlm.Payload.state.twist.angular_z = st->twist.angular_z;                

    // This data is sent when a Housekeeping request is received, 
    // (usually, at a low rate) so nothing sent here
    //memcpy(&st->joints, &RoverAppData.HkTlm.Payload.state, sizeof(RoverAppSSRMS_t) );
    
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* RoverAppVerifyCmdLength() -- Verify command packet length                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool RoverAppVerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    bool              result       = true;
    size_t            ActualLength = 0;
    CFE_SB_MsgId_t    MsgId        = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_FcnCode_t FcnCode      = 0;

    printf("RoverAppVerifyCmdLength() --\n");

    CFE_MSG_GetSize(MsgPtr, &ActualLength);

    /*
    ** Verify the command packet length.
    */
    if (ExpectedLength != ActualLength)
    {
        CFE_MSG_GetMsgId(MsgPtr, &MsgId);
        CFE_MSG_GetFcnCode(MsgPtr, &FcnCode);

        CFE_EVS_SendEvent(ROVER_APP_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), (unsigned int)FcnCode, (unsigned int)ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        RoverAppData.ErrCounter++;
    }

    return (result);

} /* End of RoverAppVerifyCmdLength() */
