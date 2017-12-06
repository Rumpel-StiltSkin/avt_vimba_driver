/*=============================================================================
  Copyright (C) 2013 - 2017 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ApiController.cpp

  Description: Implementation file for the ApiController helper class that
               demonstrates how to implement an asynchronous, continuous image
               acquisition with VimbaCPP.

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
#include <sstream>
#include <iostream>

#include "avt_vimba_driver/ApiController.h"

namespace AVT {
namespace VmbAPI {
namespace Examples {

#define NUM_FRAMES 3

ApiController::ApiController()
    // Get a reference to the Vimba singleton
    : m_system ( VimbaSystem::GetInstance() )
{}

ApiController::~ApiController()
{
}

//
// Starts the Vimba API and loads all transport layers
//
// Returns:
//  An API status code
//
VmbErrorType ApiController::StartUp()
{
    VmbErrorType res;

    // Start Vimba
    res = m_system.Startup();
    return res;
}

//
// Shuts down the API
//
void ApiController::ShutDown()
{
    // Release Vimba
    m_system.Shutdown();
}

//
// Opens the given camera
// Sets the maximum possible Ethernet packet size
// Adjusts the image format
// Sets up the observer that will be notified on every incoming frame
// Calls the API convenience function to start image acquisition
// Closes the camera in case of failure
//
// Parameters:
//  [in]    Config      A configuration struct including the camera ID and other settings
//
// Returns:
//  An API status code
//
VmbErrorType ApiController::StartContinuousImageAcquisition()
{
    // Open the desired camera by its ID
    VmbErrorType res = m_system.OpenCameraByID( "192.168.3.2", VmbAccessModeFull, m_pCamera );
    if( VmbErrorSuccess == res )
    {
        // Set the GeV packet size to the highest possible value
        // (In this example we do not test whether this cam actually is a GigE cam)
        FeaturePtr pCommandFeature;
        if( VmbErrorSuccess == SP_ACCESS( m_pCamera )->GetFeatureByName( "GVSPAdjustPacketSize", pCommandFeature ) )
        {
            if( VmbErrorSuccess == SP_ACCESS( pCommandFeature )->RunCommand() )
            {
                bool bIsCommandDone = false;
                do
                {
                    if( VmbErrorSuccess != SP_ACCESS( pCommandFeature )->IsCommandDone( bIsCommandDone ) )
                    {
                        break;
                    }
                } while( false == bIsCommandDone );
            }
        }
              
        // Create a frame observer for this camera (This will be wrapped in a shared_ptr so we don't delete it)
        SP_SET( m_pFrameObserver , new FrameObserver( m_pCamera ) );
        // Start streaming
        res = SP_ACCESS( m_pCamera )->StartContinuousImageAcquisition( NUM_FRAMES,  m_pFrameObserver );

        if ( VmbErrorSuccess != res )
        {
            // If anything fails after opening the camera we close it
            SP_ACCESS( m_pCamera )->Close();
        }
    }

    return res;
}


//
// Calls the API convenience function to stop image acquisition
// Closes the camera
//
// Returns:
//  An API status code
//
VmbErrorType ApiController::StopContinuousImageAcquisition()
{
    // Stop streaming
    SP_ACCESS( m_pCamera )->StopContinuousImageAcquisition();

    // Close camera
    return  m_pCamera->Close();
}

//
// Gets all cameras known to Vimba
//
// Returns:
//  A vector of camera shared pointers
//
CameraPtrVector ApiController::GetCameraList()
{
    CameraPtrVector cameras;
    // Get all known cameras
    if( VmbErrorSuccess == m_system.GetCameras( cameras ) )
    {
        // And return them
        return cameras;
    }
    return CameraPtrVector();
}

//
// Gets the oldest frame that has not been picked up yet
//
// Returns:
//  A frame shared pointer
//
FramePtr ApiController::GetFrame()
{
    return SP_DYN_CAST( m_pFrameObserver, FrameObserver )->GetFrame();
}

}}} // namespace AVT::VmbAPI::Examples
