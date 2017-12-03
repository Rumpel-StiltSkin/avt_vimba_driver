/*=============================================================================
  Copyright (C) 2013 - 2017 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        FrameObserver.cpp

  Description: The frame observer that is used for notifications from VimbaCPP
               regarding the arrival of a newly acquired frame.

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#include <iostream>
#include <iomanip>
#include <time.h>

#include "FrameObserver.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

//
// We pass the camera that will deliver the frames to the constructor
//
// Parameters:
//  [in]    pCamera             The camera the frame was queued at
//  [in]    eFrameInfos         Indicates how the frame will be displayed
//  [in]    eColorProcessing    Indicates how color processing is applied
//
FrameObserver::FrameObserver( CameraPtr pCamera )
    :	IFrameObserver( pCamera )
{
}
// Prints out frame status codes as readable status messages
//
// Parameters:
//  [in]    eFrameStatus    The error code to be converted and printed out
//
void PrintFrameStatus( VmbFrameStatusType eFrameStatus )
{
    switch( eFrameStatus )
    {
    case VmbFrameStatusComplete:
        std::cout<<"Complete";
        break;

    case VmbFrameStatusIncomplete:
        std::cout<<"Incomplete";
        break;

    case VmbFrameStatusTooSmall:
        std::cout<<"Too small";
        break;

    case VmbFrameStatusInvalid:
        std::cout<<"Invalid";
        break;

    default:
        std::cout<<"unknown frame status";
        break;
    }
}

//
// This is our callback routine that will be executed on every received frame.
// Triggered by the API.
//
// Parameters:
//  [in]    pFrame          The frame returned from the API
//
void FrameObserver::FrameReceived( const FramePtr pFrame )
{
    if(! SP_ISNULL( pFrame ) )
    {
        VmbFrameStatusType status;
        VmbErrorType Result;
        Result = SP_ACCESS( pFrame)->GetReceiveStatus( status);
        if( VmbErrorSuccess == Result && VmbFrameStatusComplete == status)
        {
            std::cout<<"frame complete\n";
        }
        else
        {
            std::cout<<"frame incomplete\n";
        }
    }
    else
    {
        std::cout <<" frame pointer NULL\n";
    }

    m_pCamera->QueueFrame( pFrame );
}
}}} // namespace AVT::VmbAPI::Examples
