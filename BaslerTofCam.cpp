///////////////////////////////////////////////////////////////
///                                                         ///
///          BaslerTofCam.cpp - BaslerTofCam.h              ///
///                                                         ///
///                         30/06/16                        ///
/// Guillaume FUSEILLER - XLIM                              ///
///                                                         ///
///////////////////////////////////////////////////////////////

/* This class provides a interface with the basler ToF Camera
 * It use the GenApi and GCBase library and OpenCV library
 * You may install the Balser ToF Camera Driver from http://www.baslerweb.com/
*/


#include "stdafx.h"
#include <string>
#include <ConsumerImplHelper/ToFCamera.h>
#include <GenTL/PFNC.h>
#include <iostream>
#include <iomanip> 
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "StopWatch.h"
#include <algorithm>          // std::max ..

#include <sstream>

#if defined (_MSC_VER) && defined (_WIN32)
// You have to delay load the GenApi libraries used for configuring the camera device.
// Refer to the project settings to see how to instruct the linker to delay load DLLs. 
// ("Properties->Linker->Input->Delay Loaded Dlls" resp. /DELAYLOAD linker option).
#  pragma message( "Remember to delayload these libraries (/DELAYLOAD linker option):")
#  pragma message( "    /DELAYLOAD:\"" DLL_NAME("GCBase") "\"")
#  pragma message( "    /DELAYLOAD:\"" DLL_NAME("GenApi") "\"")
#endif

#ifdef DEBUG
const uint32_t timeout_grabbing = 300000000;
#else
const uint32_t timeout_grabbing = 5000;
#endif

using namespace GenTLConsumerImplHelper;
using namespace GenApi;
using namespace std;


#include "BaslerTofCam.h"

namespace BaslerToF {


///
/// \brief BaslerTofCamera::Send_Info_Message : Send a message to the user
/// \param message : message to send
///
void BaslerTofCamera::Send_Info_Message(string message)
{
    if (eInfoMessage)
    {
        eInfoMessage(message);
    }
}

///
/// \brief BaslerTofCamera::Send_Warning_Message : Send a Warning message to the user
/// \param message : message to send
///
void BaslerTofCamera::Send_Warning_Message(string message)
{
    if (eWarningMessage)
    {
        eWarningMessage(message);
    }
}

///
/// \brief BaslerTofCamera::Send_Warning_Message : Send a Error message to the user
/// \param message : message to send
///
void BaslerTofCamera::Send_Error_Message(string message)
{
    if (eErrorMessage)
    {
        eErrorMessage(message);
    }
}

///
/// \brief BaslerTofCamera::Connect : Connect the first Tof camera seen.
/// \return
///
int BaslerTofCamera::Connect(string serial, string IP)
{
    try
    {
        CToFCamera::InitProducer();
        auto cameras_list = m_Camera.EnumerateCameras();
        Send_Info_Message(std::to_string(cameras_list.size()) + " cameras found");

        for (auto it = cameras_list.begin(); it != cameras_list.end(); it++)
        {
            Send_Info_Message(it->strModelName + " ; " + it->strDisplayName + " ; " + it->strSerialNumber + " ; " + it->strIpAddress);
        }
        if(serial!="")
        {
            try{
                m_Camera.Open(CameraInfoKey::SerialNumber,serial);
            }
            catch (const exception& e)
            {
                Send_Error_Message("Unable to connect to serial " + serial + " : " + e.what());
                return EXIT_FAILURE;
            }
        }
        else if (IP!="")
        {
            try{
                m_Camera.Open(CameraInfoKey::IpAddress,IP);
            }
            catch (const exception& e)
            {
                Send_Error_Message("Unable to connect to IP " + IP + " : " + e.what());
                return EXIT_FAILURE;
            }
        }
        else
        {

            m_Camera.OpenFirstCamera();

        }
    }
    catch (const exception& e)
    {
        Send_Error_Message("Exception occurred: in BaslerTofCamera::connect()");
        return EXIT_FAILURE;
    }
    Send_Info_Message("connected to " + m_Camera.GetCameraInfo().strDisplayName + " at " +m_Camera.GetCameraInfo().strIpAddress);
    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::TestConfigurationFunc : Only for TEST, test all the configuration functions
/// \return
///
int BaslerTofCamera::TestConfigurationFunc()
{
    try
    {
        //Debug Test
        Send_Info_Message("debut test parametrage");
        Send_Info_Message("debut test Acquisition Control");
        Send_Info_Message("Disable LED");
        Set_Led_Disable(true);
        Send_Info_Message("Enable LED");
        Set_Led_Disable(false);
        Send_Info_Message("Set Acquisition Frame Rate to 15");
        Set_Acquisition_Frame_Rate(12);
        Send_Info_Message("Set Acquisition Frame Rate to 20");
        Set_Acquisition_Frame_Rate(20);
        Send_Info_Message("Set Exposition Time");
        Send_Info_Message("10000 µs");
        Set_Exposure_Time(10000);
        Send_Info_Message("50 µs");
        Set_Exposure_Time(50);
        Send_Info_Message("1150 µs");
        Set_Exposure_Time(1150);
        Send_Info_Message("Set Exposition Auto to Off");
        Set_Exposure_Auto(EXPOSURE_AUTO_OFF);
        Send_Info_Message("Set Exposition Auto to Continuous");
        Set_Exposure_Auto(EXPOSURE_AUTO_CONTINUOUS);
        Send_Info_Message("Set Agility");
        Send_Info_Message("0.6");
        Set_Agility(0.6);
        Send_Info_Message("0.25");
        Set_Agility(0.25);
        Send_Info_Message("1.2");
        Set_Agility(1.2);
        Send_Info_Message("0.4");
        Set_Agility(0.4);
        Send_Info_Message("Set Delay");
        Set_Delay(2);

        Send_Info_Message("debut test Image Format Control");
        Send_Info_Message("Set Width");
        Set_Width(320);
        Send_Info_Message("Set Height");
        Set_Height(240);
        Send_Info_Message("Set Off X");
        Set_Offset_X(50);
        Send_Info_Message("Set Off Y");
        Set_Offset_Y(49);
        Send_Info_Message("Set Depth Max");
        Set_Z_Max(10000);
        Send_Info_Message("Set Depth Max");
        Set_Z_Min(20);
        Send_Info_Message("Restore ROI");
        Set_Offset_X();
        Set_Offset_Y();
        Set_Z_Min();
        Set_Width();
        Set_Height();
        Set_Z_Max();
        Send_Info_Message("Disable all Componente");
        Set_Component_Confidence(false);
        Set_Component_Intensity(false);
        Set_Component_Range(false);
        Send_Info_Message("Enable all Componente");
        Set_Component_Confidence();
        Set_Component_Intensity();
        Set_Component_Range();
        Send_Info_Message("Switch Range Pixel Format");
        Set_Component_Range(true, PIXEL_FORMAT_RGB8);
        Set_Component_Range(true, PIXEL_FORMAT_COORD3D_ABC32F);

        Send_Info_Message("debut test Image Quality Control");
        Send_Info_Message("Set Confidence Threshold");
        Set_Confidence_Threshold(10);
        Set_Confidence_Threshold();
        Send_Info_Message("Disable & Enable Spatial Filter");
        Set_Temporal_Filter_Enable(false);
        Set_Temporal_Filter_Enable();
        Send_Info_Message("Disable & Enable Temporal Filter");
        Set_Spatial_Filter_Enable(false);
        Set_Spatial_Filter_Enable();
        Send_Info_Message("Set Strenght");
        Set_Temporal_Filter_Strength(500);
        Set_Temporal_Filter_Strength();
        Send_Info_Message("Set Outlier Tolerance");
        Set_Outlier_Tolerance(0);
        Set_Outlier_Tolerance();
    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Exception occurred: " + string(e.GetDescription()));
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::GrabOne : start acquisition of one frame and grab it
/// \return
///
int BaslerTofCamera::GrabOne()
{
    try
    {
        // Acquire one single image
        BufferParts parts;
        GrabResultPtr ptrGrabResult = m_Camera.GrabSingleImage( timeout_grabbing, &parts );

        if ( ptrGrabResult->status == GrabResult::Timeout )
        {
#ifndef DEBUG
            Send_Error_Message("Timeout occurred. Acquisition stopped.");
            return EXIT_FAILURE;
#endif
        }
        if ( ptrGrabResult->status == GrabResult::Ok )
        {
            OnImageGrabbed(parts);
        }
        else
        {
            Send_Error_Message("Failed to grab an image.");
            return EXIT_FAILURE;
        }

    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Exception occurred: " + string(e.GetDescription()));
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::GrabContinuous : start continous acquisition of frames
/// \return
///
int BaslerTofCamera::GrabContinuous()
{
    try
    {
        Continue_Grabbing = true;
        m_Camera.GrabContinuous(5, timeout_grabbing, this, &BaslerTofCamera::OnImageGrabbed2);
    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Exception occurred when starting grabbing thread :" + string(e.GetDescription()));
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::StopGrab : stop continuous acquisitions of frames
/// \return
///
int BaslerTofCamera::StopGrab()
{
    try
    {
        Continue_Grabbing=false;
    }
    catch ( exception e )
    {
        Send_Error_Message("Exception occurred when stopping grabbing thread");
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::OnImageGrabbed2 : fonction call when an Image is grabbed (in continuous
/// \param grabResult : result of grabbing
/// \param parts : buffer of grabbed image
/// \return
///
bool BaslerTofCamera::OnImageGrabbed2( GrabResult grabResult, BufferParts parts )
{
    if ( grabResult.status == GrabResult::Timeout )
    {
        Send_Warning_Message("Timeout occurred");
#ifdef DEBUG
        return true;
#else
        return true;
#endif
    }
    if ( grabResult.status != GrabResult::Ok )
    {
        Send_Error_Message("Image was not grabbed.");
    }
    else
    {
        OnImageGrabbed(parts);
    }
    return Continue_Grabbing;
}

///
/// \brief BaslerTofCamera::OnImageGrabbed : fonction call when an image is grabbed properly
/// \param parts : buffer of grabbed image
/// \return
///
bool BaslerTofCamera::OnImageGrabbed( BufferParts parts)
{
    try
    {
        if ( parts.empty() )
        {
            Send_Error_Message("No valid image data.");
            return false;
        }

        if(eNewImage)
            eNewImage(parts);

    }
    catch (exception e)
    {
        Send_Error_Message("Error in BaslerTofCamera::OnImageGrabbed");
        return false;
    }

    return true;
}

///
/// \brief BaslerTofCamera::Disconnect : disconnect the camera
/// \return
///
int BaslerTofCamera::Disconnect()
{
    try
    {
        // Clean-up
        m_Camera.Close();

        if ( CToFCamera::IsProducerInitialized() )
            CToFCamera::TerminateProducer();  // Won't throw any exceptions
    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Exception occurred: " + string(e.GetDescription()));
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

// Get and Set parameter fonction
// Acquisition Control

///
/// \brief BaslerTofCamera::Set_Device_Channel : select the camera channel
/// \param Channel : Camera group channel
/// \return
///
int BaslerTofCamera::Set_Device_Channel(int Channel)
{
    return Set_Integer_Parameter("DeviceChannel", Channel);
}
///
/// \brief BaslerTofCamera::Get_Device_Channel : get the camera channel
/// \param Channel : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Device_Channel(int* Channel)
{
    return Get_Integer_Parameter("DeviceChannel", Channel);
}

///
/// \brief BaslerTofCamera::Set_Led_Disable : enable or disable the IR Led, Range acquisition is not possible when IR Led is disable
/// \param Disable_Led : false to enable IR Led, true for disable
/// \return
///
int BaslerTofCamera::Set_Led_Disable(bool Disable_Led)
{
    return Set_Boolean_Parameter("LedDisable",Disable_Led);
}
///
/// \brief BaslerTofCamera::Get_Led_Disable : Get the state of IR Led (true = disable, false = enable)
/// \param Led_Disable : pointer to the output variable
/// \return
///
int BaslerTofCamera::Get_Led_Disable(bool *Led_Disable)
{
    return Get_Boolean_Parameter("LedDisable",Led_Disable);
}

///
/// \brief BaslerTofCamera::Set_Processing_Mode : select the Processing Mode
/// \param Mode : 0=standard, 1=HDR
/// \return
///
int BaslerTofCamera::Set_Processing_Mode(int Mode)
{
    return Set_Enumeration_Parameter("ProcessingMode", Mode);
}
///
/// \brief BaslerTofCamera::Get_Exposure_Auto : get the Processing Mode (0=standard, 1=HDR)
/// \param Mode : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Processing_Mode(int* Mode)
{
    return Get_Enumeration_Parameter("ProcessingMode", Mode);
}

///
/// \brief BaslerTofCamera::Set_Acquisition_Frame_Rate : set the acquisition frame rate
/// \param Frame_Rate : frame rate (fps)
/// \return
///
int BaslerTofCamera::Set_Acquisition_Frame_Rate(float Frame_Rate)
{
    return Set_Float_Parameter("AcquisitionFrameRate",Frame_Rate);
}
///
/// \brief BaslerTofCamera::Get_Acquisition_Frame_Rate : get the acquisition frame rate set (fps).
/// \param Frame_Rate : pointer to the output variable
/// \return
///
int BaslerTofCamera::Get_Acquisition_Frame_Rate(float* Frame_Rate)
{
    return Get_Float_Parameter("AcquisitionFrameRate",Frame_Rate);
}

///
/// \brief BaslerTofCamera::Set_Exposure_Time : set the exposure time (expo auto must be set to manual) (µs)
/// \param Exposure_Time : exposure time in µs
/// \return
///
int BaslerTofCamera::Set_Exposure_Time(float Exposure_Time)
{
    int ret = Set_Integer_Parameter("ExposureTimeSelector", 0);
    if (ret==EXIT_SUCCESS)
        ret = Set_Float_Parameter("ExposureTime",Exposure_Time);
    return ret;
}
///
/// \brief BaslerTofCamera::Get_Exposure_Time : get the exposure time set for manual esposure mode (µs)
/// \param Exposure_Time : pointer to output variable
/// \return
///
///
int BaslerTofCamera::Get_Exposure_Time(float* Exposure_Time)
{
    int ret = Set_Integer_Parameter("ExposureTimeSelector", 0);
    if (ret==EXIT_SUCCESS)
        ret =  Get_Float_Parameter("ExposureTime",Exposure_Time);
    return ret;
}

///
/// \brief BaslerTofCamera::Set_Exposure_Time2 : set the second exposure time in HDR processing mode (expo auto must be set to manual and processing to HDR) (µs)
/// \param Exposure_Time : exposure time in µs
/// \return
///
int BaslerTofCamera::Set_Exposure_Time2(float Exposure_Time)
{
    int ret = Set_Integer_Parameter("ExposureTimeSelector", 1);
    if (ret==EXIT_SUCCESS)
        ret =  Set_Float_Parameter("ExposureTime",Exposure_Time);
    return ret;
}
///
/// \brief BaslerTofCamera::Get_Exposure_Time2 : get the second exposure time set for manual esposure mode in HDR processing mode (µs)
/// \param Exposure_Time : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Exposure_Time2(float* Exposure_Time)
{
    int ret = Set_Integer_Parameter("ExposureTimeSelector", 1);
    if (ret==EXIT_SUCCESS)
        ret =   Get_Float_Parameter("ExposureTime",Exposure_Time);
    return ret;
}

///
/// \brief BaslerTofCamera::Set_Exposure_Auto : select the exposure auto mode
/// \param Exposure_Auto : 0=off, 2=continous
/// \return
///
int BaslerTofCamera::Set_Exposure_Auto(int Exposure_Auto)
{
    return Set_Enumeration_Parameter("ExposureAuto", Exposure_Auto);
}
///
/// \brief BaslerTofCamera::Get_Exposure_Auto : get the exposure auto mode (0=off, 2=continous)
/// \param Exposure_Auto : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Exposure_Auto(int* Exposure_Auto)
{
    return Get_Enumeration_Parameter("ExposureAuto", Exposure_Auto);
}

///
/// \brief BaslerTofCamera::Set_Agility : set agility (gain of auto exposure correction)
/// \param Agility : agility (0.0 - 1.0)
/// \return
///
int BaslerTofCamera::Set_Agility(float Agility)
{
    return Set_Float_Parameter("Agility", Agility);
}
///
/// \brief BaslerTofCamera::Get_Agility : get agility parameter
/// \param Agility : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Agility(float* Agility)
{
    return Get_Float_Parameter("Agility", Agility);
}

///
/// \brief BaslerTofCamera::Set_Delay : set delay (frame before exposure time is update in auto mode)
/// \param Delay : delay (frame)
/// \return
///
int BaslerTofCamera::Set_Delay(int Delay)
{
    return Set_Integer_Parameter("Delay", Delay);
}
///
/// \brief BaslerTofCamera::Get_Delay : get delay parameter
/// \param Delay : pointer to output varaible
/// \return
///
int BaslerTofCamera::Get_Delay(int* Delay)
{
    return Get_Integer_Parameter("Delay", Delay);
}

int BaslerTofCamera::Get_IEEE1588(bool *mode){
    return Get_Boolean_Parameter("GevIEEE1588", mode);
}

int BaslerTofCamera::Set_IEEE1588(bool mode){
    return Set_Boolean_Parameter("GevIEEE1588", mode);
}

///
/// \brief BaslerTofCamera::Send_Trigger_Software : send a Software Trigger to the camera
/// \return
///
int BaslerTofCamera::Send_Trigger_Software()
{
    return Send_Command("TriggerSoftware");
}

///
/// \brief BaslerTofCamera::Set_Trigger_Source : select the trigger source
/// \param Source : 0=software, 1=line1, 2=line2, 3=synchronised timer
/// \return
///
int BaslerTofCamera::Set_Trigger_Source(int Source)
{
    if (Source==TRIGGER_SOURCE_SYNCTIMER)
    {
        auto res = Set_Boolean_Parameter("GevIEEE1588",true);
        if (res==EXIT_FAILURE)
        {
            return res;
        }
    }
    return Set_Enumeration_Parameter("TriggerSource", Source);
}
///
/// \brief BaslerTofCamera::Get_Trigger_Source : get the trigger source (0=software, 1=line1, 2=line2, 3=synchronised timer)
/// \param Source : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Trigger_Source(int* Source)
{
    return Get_Enumeration_Parameter("TriggerSource", Source);
}

///
/// \brief BaslerTofCamera::Set_Trigger_Mode : select the trigger mode
/// \param Mode : 0=off, 1=on
/// \return
///
int BaslerTofCamera::Set_Trigger_Mode(int Mode)
{
    return Set_Enumeration_Parameter("TriggerMode", Mode);
}
///
/// \brief BaslerTofCamera::Get_Trigger_Mode : get the trigger mode (0=off, 1=on)
/// \param Mode : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Trigger_Mode(int* Mode)
{
    return Get_Enumeration_Parameter("TriggerMode", Mode);
}

///
/// \brief BaslerTofCamera::Set_Trigger_Delay : µs
/// \param Delay
/// \return
///
int BaslerTofCamera::Set_Trigger_Delay(float Delay)
{
    return Set_Float_Parameter("TriggerDelay", Delay);
}

///
/// \brief BaslerTofCamera::Get_Trigger_Delay : µs
/// \param Delay
/// \return
///
int BaslerTofCamera::Get_Trigger_Delay(float* Delay)
{
    return Get_Float_Parameter("TriggerDelay", Delay);
}

///
/// \brief BaslerTofCamera::Set_Sync_Rate : set the acquisition frame rate
/// \param Sync_Rate : frame rate (fps)
/// \return
///
int BaslerTofCamera::Set_Sync_Rate(float Sync_Rate)
{
    auto res = Set_Float_Parameter("SyncRate",Sync_Rate);
    if(res==EXIT_FAILURE)
        return res;
    return Send_Command("SyncUpdate");
}
///
/// \brief BaslerTofCamera::Get_Sync_Rate : get the acquisition frame rate set (fps).
/// \param Sync_Rate : pointer to the output variable
/// \return
///
int BaslerTofCamera::Get_Sync_Rate(float* Sync_Rate)
{
    return Get_Float_Parameter("SyncRate",Sync_Rate);
}

// Image Format Control

///
/// \brief BaslerTofCamera::Set_Binning : enable or disable binning
/// \param Enable : false binning disable, true binning enable
/// \return
///
int BaslerTofCamera::Set_Binning(bool Enable)
{
    return Set_Boolean_Parameter("Binning",Enable);
}
///
/// \brief BaslerTofCamera::Get_Binning : Get binning enable
/// \param Enable : pointer to the output variable
/// \return
///
int BaslerTofCamera::Get_Binning(bool *Enable)
{
    return Get_Boolean_Parameter("Binning",Enable);
}

///
/// \brief BaslerTofCamera::Set_Width : set image width
/// \param Width : width (pixel)
/// \return
///
int BaslerTofCamera::Set_Width(int Width)
{
    return Set_Integer_Parameter("Width", Width);
}
///
/// \brief BaslerTofCamera::Get_Width : get image widht (pixel)
/// \param Width : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Width(int* Width)
{
    return Get_Integer_Parameter("Width", Width);
}

///
/// \brief BaslerTofCamera::Set_Height : set image height
/// \param Height : height (pixel)
/// \return
///
int BaslerTofCamera::Set_Height(int Height)
{
    return Set_Integer_Parameter("Height", Height);
}
///
/// \brief BaslerTofCamera::Get_Height : get image (height) 0 to 65535, which corresponds to the camera’s non-ambiguity range (0 m to 13.325 m).
/// \param Height : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Height(int* Height)
{
    return Get_Integer_Parameter("Height", Height);
}

///
/// \brief BaslerTofCamera::Set_Depth : set image depth
/// \param Depth : depth (from 0 to 65535 for 0 to 13.325m, about 0.2mm/unit)
/// \return
///
int BaslerTofCamera::Set_Z_Max(int Z_max)
{
    return Set_Integer_Parameter("DepthMax", Z_max);
}
///
/// \brief BaslerTofCamera::Get_Depth : get image depth (from 0 to 65535 for 0 to 13.325m, about 0.2mm/unit)
/// \param Depth : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Z_Max(int* Z_max)
{
    return Get_Integer_Parameter("DepthMax", Z_max);
}

///
/// \brief BaslerTofCamera::Set_Offset_X : set offset of ROI in X direction from the left of the image
/// \param OffsetX : offset (pixel)
/// \return
///
int BaslerTofCamera::Set_Offset_X(int OffsetX)
{
    return Set_Integer_Parameter("OffsetX", OffsetX);
}
///
/// \brief BaslerTofCamera::Get_Offset_X : get the offset of the image in X direction (in pixel from the left of the image)
/// \param OffsetX : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Offset_X(int* OffsetX)
{
    return Get_Integer_Parameter("OffsetX", OffsetX);
}

///
/// \brief BaslerTofCamera::Set_Offset_Y : set offset of ROI in Y direction from the top of the image
/// \param OffsetY : offset (pixel)
/// \return
///
int BaslerTofCamera::Set_Offset_Y(int OffsetY)
{
    return Set_Integer_Parameter("OffsetY", OffsetY);
}
///
/// \brief BaslerTofCamera::Get_Offset_Y : get the offset of the image in Y direction (in pixel from the top of the image)
/// \param OffsetY : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Offset_Y(int* OffsetY)
{
    return Get_Integer_Parameter("OffsetY", OffsetY);
}

///
/// \brief BaslerTofCamera::Set_Offset_Z : set offset of ROI in Z direction from the front of the camera
/// \param OffsetZ : offset (from 0 to 65535 for 0 to 13.325m, about 0.2mm/unit)
/// \return
///
int BaslerTofCamera::Set_Z_Min(int Z_min)
{
    return Set_Integer_Parameter("DepthMin", Z_min);
}
///
/// \brief BaslerTofCamera::Get_Offset_Z : get the offset of the image in Z direction (from 0 to 65535 for 0 to 13.325m, about 0.2mm/unit)
/// \param OffsetZ : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Z_Min(int* Z_min)
{
    return Get_Integer_Parameter("DepthMin", Z_min);
}

///
/// \brief BaslerTofCamera::Set_Component_Range : Activate or desactivate the image componant Range and set the pixel format
/// \param Enable : true = enable, false = disable
/// \param PixelFormat : 17825799 = Mono16, 35127316 = RGB8, 39846080 = Coord3D
/// \return
///
int BaslerTofCamera::Set_Component_Range(bool Enable, int PixelFormat)
{
    int ret = Set_Enumeration_Parameter("ImageComponentSelector", RANGE);
    if (ret==EXIT_SUCCESS)
        ret = Set_Boolean_Parameter("ImageComponentEnable", Enable);
    if (Enable&ret==EXIT_SUCCESS)
        ret = Set_Enumeration_Parameter("PixelFormat", PixelFormat);
    return ret;
}
///
/// \brief BaslerTofCamera::Get_Component_Range : get image componante Range statut
/// \param Enable : pointer to output variable for the statut (true = enable, false = disable)
/// \param PixelFormat : pointer to the output variable for the pixel format (17825799 = Mono16, 35127316 = RGB8, 39846080 = Coord3D)
/// \return
///
int BaslerTofCamera::Get_Component_Range(bool* Enable, int* PixelFormat)
{
    int ret = Set_Enumeration_Parameter("ImageComponentSelector", RANGE);
    if (ret==EXIT_SUCCESS)
        ret = Get_Boolean_Parameter("ImageComponentEnable", Enable);
    if (ret==EXIT_SUCCESS)
        ret = Get_Enumeration_Parameter("PixelFormat", PixelFormat);
    return ret;
}

///
/// \brief BaslerTofCamera::Set_Component_Intensity : Activate or desactivate the image componant Intensity and set the pixel format
/// \param Enable : true = enable, false = disable
/// \param PixelFormat : 17825799 = Mono16
/// \return
///
int BaslerTofCamera::Set_Component_Intensity(bool Enable, int PixelFormat)
{
    int ret = Set_Enumeration_Parameter("ImageComponentSelector", INTENSITY);
    if (ret==EXIT_SUCCESS)
        ret = Set_Boolean_Parameter("ImageComponentEnable", Enable);
    if (Enable&ret==EXIT_SUCCESS)
        ret = Set_Enumeration_Parameter("PixelFormat", PixelFormat);
    return ret;
}
///
/// \brief BaslerTofCamera::Get_Component_Intensity : get image componante Intensity statut
/// \param Enable : pointer to output variable for the statut (true = enable, false = disable)
/// \param PixelFormat : pointer to the output variable for the pixel format (17825799 = Mono16)
/// \return
///
int BaslerTofCamera::Get_Component_Intensity(bool* Enable, int* PixelFormat)
{
    int ret = Set_Enumeration_Parameter("ImageComponentSelector", INTENSITY);
    if (ret==EXIT_SUCCESS)
        ret = Get_Boolean_Parameter("ImageComponentEnable", Enable);
    if (ret==EXIT_SUCCESS)
        ret = Get_Enumeration_Parameter("PixelFormat", PixelFormat);
    return ret;
}

///
/// \brief BaslerTofCamera::Set_Component_Confidence : Activate or desactivate the image componant Confidence and set the pixel format
/// \param Enable : true = enable, false = disable
/// \param PixelFormat : 17825799 = Mono16
/// \return
///
int BaslerTofCamera::Set_Component_Confidence(bool Enable, int PixelFormat)
{
    int ret = Set_Enumeration_Parameter("ImageComponentSelector", CONFIDENCE);
    if (ret==EXIT_SUCCESS)
        ret = Set_Boolean_Parameter("ImageComponentEnable", Enable);
    if (Enable&ret==EXIT_SUCCESS)
        ret = Set_Enumeration_Parameter("PixelFormat", PixelFormat);
    return ret;
}
///
/// \brief BaslerTofCamera::Get_Component_Confidence : get image componante Confidence statut
/// \param Enable : pointer to output variable for the statut (true = enable, false = disable)
/// \param PixelFormat : pointer to the output variable for the pixel format (17825799 = Mono16)
/// \return
///
int BaslerTofCamera::Get_Component_Confidence(bool* Enable, int* PixelFormat)
{
    int ret = Set_Enumeration_Parameter("ImageComponentSelector", CONFIDENCE);
    if (ret==EXIT_SUCCESS)
        ret = Get_Boolean_Parameter("ImageComponentEnable", Enable);
    if (ret==EXIT_SUCCESS)
        ret = Get_Enumeration_Parameter("PixelFormat", PixelFormat);
    return ret;
}

// Image Quality Control

///
/// \brief BaslerTofCamera::Set_Confidence_Threshold : set the confidence threshold (delete pixel which have a confidence bigger than the threshold)
/// \param Threshold : threshold
/// \return
///
int BaslerTofCamera::Set_Confidence_Threshold(int Threshold)
{
    return Set_Integer_Parameter("ConfidenceThreshold", Threshold);
}
///
/// \brief BaslerTofCamera::Get_Confidence_Threshold : get the confidence threshold
/// \param Threshold : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Confidence_Threshold(int* Threshold)
{
    return Get_Integer_Parameter("ConfidenceThreshold", Threshold);
}

///
/// \brief BaslerTofCamera::Set_Spatial_Filter_Enable : enable or disable the spatial filter
/// \param Enable : true = enable, false = disable
/// \return
///
int BaslerTofCamera::Set_Spatial_Filter_Enable(bool Enable)
{
    return Set_Boolean_Parameter("FilterSpatial", Enable);
}
///
/// \brief BaslerTofCamera::Get_Spatial_Filter_Enable : get the spatial filter statut (true = enable, false = disable)
/// \param Enable : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Spatial_Filter_Enable(bool* Enable)
{
    return Get_Boolean_Parameter("FilterSpatial", Enable);
}

///
/// \brief BaslerTofCamera::Set_Temporal_Filter_Enable : enable or disable the temporal filter
/// \param Enable : true = enable, false = disable
/// \return
///
int BaslerTofCamera::Set_Temporal_Filter_Enable(bool Enable)
{
    return Set_Boolean_Parameter("FilterTemporal", Enable);
}
///
/// \brief BaslerTofCamera::Get_Temporal_Filter_Enable : get the temporal filter statut (true = enable, false = disable)
/// \param Enable : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Temporal_Filter_Enable(bool* Enable)
{
    return Get_Boolean_Parameter("FilterTemporal", Enable);
}

///
/// \brief BaslerTofCamera::Set_Temporal_Filter_Strength : set temporal filter strength
/// \param Strength : strength
/// \return
///
int BaslerTofCamera::Set_Temporal_Filter_Strength(int Strength)
{
    return Set_Integer_Parameter("FilterStrength", Strength);
}
///
/// \brief BaslerTofCamera::Get_Temporal_Filter_Strength : get the temporal filter strenght
/// \param Strength : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Temporal_Filter_Strength(int* Strength)
{
    return Get_Integer_Parameter("FilterStrength", Strength);
}

///
/// \brief BaslerTofCamera::Set_Outlier_Tolerance : set outlier tolerance (max difference (in Z) between two neighbour pixels)
/// \param Tolerance : tolerance
/// \return
///
int BaslerTofCamera::Set_Outlier_Tolerance(int Tolerance)
{
    return Set_Integer_Parameter("OutlierTolerance", Tolerance);
}
///
/// \brief BaslerTofCamera::Get_Outlier_Tolerance : get outlier tolerance (max difference (in Z) between two neighbour pixels)
/// \param Tolerance : pointer to output variable
/// \return
///
int BaslerTofCamera::Get_Outlier_Tolerance(int* Tolerance)
{
    return Get_Integer_Parameter("OutlierTolerance", Tolerance);
}

///
/// \brief BaslerTofCamera::Set_Enumeration_Parameter : this fonction set the value of an Enumeration parameter.
/// \param Parameter_Name : the name of the parameter
/// \param Value : the value to set
/// \return
///
int BaslerTofCamera::Set_Enumeration_Parameter(string Parameter_Name, int Value)
{
    try
    {
        //Get the parameter
        GenApi::CEnumerationPtr ptr_Enumeration = m_Camera.GetParameter(Parameter_Name.c_str());

        //Set parameter value
        ptr_Enumeration->SetIntValue(Value);
    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Error : Impossible to set the parameter " + Parameter_Name + " to value " + std::to_string(Value) + " : " + e.GetDescription());
        return EXIT_FAILURE;
    }
    Send_Info_Message("Parameter " + Parameter_Name + " set to " + std::to_string(Value));
    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::Set_Boolean_Parameter : this fonction set the value of an Boolean parameter.
/// \param Parameter_Name : the name of the parameter
/// \param Value : the value to set
/// \return
///
int BaslerTofCamera::Set_Boolean_Parameter(string Parameter_Name, bool Value)
{
    try
    {
        //Get the parameter
        GenApi::CBooleanPtr ptr_Boolean = m_Camera.GetParameter(Parameter_Name.c_str());

        //Set parameter value
        ptr_Boolean->SetValue(Value);
    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Error : Impossible to set the parameter " + Parameter_Name + " to value " +  std::to_string(Value) + " : " + e.GetDescription());
        return EXIT_FAILURE;
    }
    Send_Info_Message("Parameter " + Parameter_Name + " set to " + std::to_string(Value));
    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::Set_Integer_Parameter : this fonction set the value of an Integer parameter.
/// \param Parameter_Name : the name of the parameter
/// \param Value : the value to set
/// \return
///
int BaslerTofCamera::Set_Integer_Parameter(string Parameter_Name, int Value)
{
    try
    {
        //Get the parameter
        GenApi::CIntegerPtr ptr_Integer = m_Camera.GetParameter(Parameter_Name.c_str());
        
        //Verify parameter limits & validity
        int Max = ptr_Integer->GetMax();
        int Min = ptr_Integer->GetMin();
        int Inc = ptr_Integer->GetInc();
        
        if (Value>Max)
        {
            Value=Max;
            Send_Warning_Message("The Parameter Value is out of range (to Big), the parameter is set to Max Value : " +  std::to_string(Max));
        }
        else if (Value<Min)
        {
            Value=Min;
            Send_Warning_Message("The Parameter Value is out of range (to Small), the parameter is set to Min Value : " +  std::to_string(Min));
        }
        else if ((Value-Min)%Inc != 0)
        {
            if (((Value-Min)/Inc-(Value-Min)%Inc)<0.5)
                Value = (Value-Min)/Inc * Inc + Min;
            else
                Value = (Value-Min)/Inc * Inc + Min + Inc;
        }
        
        //Set parameter value
        ptr_Integer->SetValue(Value);
    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Error : Impossible to set the parameter " + Parameter_Name + " to value " +  std::to_string(Value) + " : " + e.GetDescription());
        return EXIT_FAILURE;
    }
    Send_Info_Message("Parameter " + Parameter_Name + " set to " +  std::to_string(Value));
    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::Set_Integer_Parameter : this fonction set the value of an Integer parameter.
/// \param Parameter_Name : the name of the parameter
/// \param Value : the value to set
/// \return
///
int BaslerTofCamera::Set_Float_Parameter(string Parameter_Name, float Value)
{
    try
    {
        //Get the parameter
        GenApi::CFloatPtr ptr_Float = m_Camera.GetParameter(Parameter_Name.c_str());

        //Verify parameter limits & validity
        float Max = ptr_Float->GetMax();
        float Min = ptr_Float->GetMin();
        float Inc = 0.000001;
        if(ptr_Float->HasInc()){
            float Inc = ptr_Float->GetInc();
        }

        if (Value>Max)
        {
            Value=Max;
            Send_Warning_Message("The Parameter Value is out of range (to Big), the parameter is set to Max Value : " +  std::to_string(Max));
        }
        else if (Value<Min)
        {
            Value=Min;
            Send_Warning_Message("The Parameter Value is out of range (to Small), the parameter is set to Min Value : " +  std::to_string(Min));
        }
        else if ((Value-Min)/Inc != (int)(Value-Min)/Inc)
        {
            if (((Value-Min)/Inc-(int)((Value-Min)/Inc))<0.5)
                Value = (int)((Value-Min)/Inc) * Inc + Min;
            else
                Value = (int)((Value-Min)/Inc) * Inc + Min + Inc;
        }

        //Set parameter value
        ptr_Float->SetValue(Value);
    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Error : Impossible to set the parameter " + Parameter_Name + " to value " + std::to_string(Value) + " : " + e.GetDescription());
        return EXIT_FAILURE;
    }
    Send_Info_Message("Parameter " + Parameter_Name + " set to " + std::to_string(Value));
    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::Send_Command : this fonction set the value of an Boolean parameter.
/// \param Parameter_Name : the name of the parameter
/// \param Value : the value to set
/// \return
///
int BaslerTofCamera::Send_Command(string Parameter_Name)
{
    try
    {
        //Get the parameter
        GenApi::CCommandPtr ptr_Command = m_Camera.GetParameter(Parameter_Name.c_str());

        //Set parameter value
        ptr_Command->Execute();
    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Error : Impossible to send command " + Parameter_Name + " : " + e.GetDescription());
        return EXIT_FAILURE;
    }
    Send_Info_Message("Command " + Parameter_Name + " send");
    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::Get_Enumeration_Parameter : get the value of an Enumeration parameter
/// \param Parameter_Name : the name of the parameter
/// \param value
/// \return
///
int BaslerTofCamera::Get_Enumeration_Parameter(string Parameter_Name, int* value)
{
    try
    {

        //Get the parameter
        GenApi::CEnumerationPtr ptr_Enumeration = m_Camera.GetParameter(Parameter_Name.c_str());

        //Get parameter value
        *value = ptr_Enumeration->GetIntValue();

    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Error : Impossible to get the parameter " + Parameter_Name + " : " + e.GetDescription());
        return EXIT_FAILURE;
    }
    Send_Info_Message("Parameter " + Parameter_Name + " get : " + std::to_string(*value));
    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::Get_Boolean_Parameter : get the value of an String parameter
/// \param Parameter_Name : the name of the parameter
/// \param value
/// \return
///
int BaslerTofCamera::Get_Boolean_Parameter(string Parameter_Name, bool* value)
{
    try
    {
        //Get the parameter
        GenApi::CBooleanPtr ptr_Boolean = m_Camera.GetParameter(Parameter_Name.c_str());

        //Get parameter value
        *value = ptr_Boolean->GetValue();
    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Error : Impossible to get the parameter " + Parameter_Name + " : " + e.GetDescription());
        return EXIT_FAILURE;
    }
    Send_Info_Message("Parameter " + Parameter_Name + " get : " + std::to_string(*value));
    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::Get_Integer_Parameter : get the value of an Integer parameter
/// \param Parameter_Name : the name of the parameter
/// \param value
/// \return
///
int BaslerTofCamera::Get_Integer_Parameter(string Parameter_Name, int* value)
{
    try
    {
        //Get the parameter
        GenApi::CIntegerPtr ptr_Integer = m_Camera.GetParameter(Parameter_Name.c_str());

        //Get parameter value
        *value = ptr_Integer->GetValue();
    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Error : Impossible to get the parameter " + Parameter_Name + " : " + e.GetDescription());
        return EXIT_FAILURE;
    }
    Send_Info_Message("Parameter " + Parameter_Name + " get : " +  std::to_string(*value));
    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::Get_Float_Parameter : get the value of an Float parameter
/// \param Parameter_Name : the name of the parameter
/// \return
///
int BaslerTofCamera::Get_Float_Parameter(string Parameter_Name, float* value)
{
    try
    {
        //Get the parameter
        GenApi::CFloatPtr ptr_Float = m_Camera.GetParameter(Parameter_Name.c_str());

        //Get parameter value
        *value = ptr_Float->GetValue();
    }
    catch ( const GenICam::GenericException& e )
    {
        Send_Error_Message("Error : Impossible to get the parameter " + Parameter_Name + " : " + e.GetDescription());
        return EXIT_FAILURE;
    }
    Send_Info_Message("Parameter " + Parameter_Name + " get : " + std::to_string(*value));
    return EXIT_SUCCESS;
}

///
/// \brief BaslerTofCamera::Send_DataSet_Latch : latch the IEEE 1588 status
/// \return
///
int BaslerTofCamera::Send_DataSet_Latch()
{
    return Send_Command("GevIEEE1588DataSetLatch");
}

///
/// \brief BaslerTofCamera::Send_Timestamp_Latch
/// \return
///
int BaslerTofCamera::Send_Timestamp_Latch()
{
    return Send_Command("TimestampLatch");
}

///
/// \brief BaslerTofCamera::setWorkInPairs : change the status of WorkInPairs
/// \param newWorkInPairs : the new status
///
void BaslerTofCamera::Set_Work_In_Pairs(bool newWorkInPairs){
    WorkInPairs = newWorkInPairs;
}

///
/// \brief BaslerTofCamera::setIsMaster : change the status of IsMaster
/// \param newIsMaster
///
void BaslerTofCamera::Set_Is_Master(bool newIsMaster){
    IsMaster = newIsMaster;
}

///
/// \brief BaslerTofCamera::getWorkInPairs : return the value of WorkInPairs
/// \return
///
bool BaslerTofCamera::Get_Work_In_Pairs(){
    return WorkInPairs;
}

///
/// \brief BaslerTofCamera::getIsMaster : return the value of IsMaster
/// \return
///
bool BaslerTofCamera::Get_Is_Master(){
    return IsMaster;
}

///
/// \brief BaslerTofCamera::Get_Offset_From_Master
/// \param ptrOffsetFromMaster
/// \return
///
int BaslerTofCamera::Get_Offset_From_Master(int* ptrOffsetFromMaster){
    return Get_Integer_Parameter("GevIEEE1588OffsetFromMaster",ptrOffsetFromMaster);
}

///
/// \brief BaslerTofCamera::Send_SyncUpdate
/// \return
///
int BaslerTofCamera::Send_SyncUpdate()
{
    return Send_Command("SyncUpdate");
}

///
/// \brief BaslerTofCamera::GetMaxAbsGevIEE1588OffsetFromMasterInTimeWindow
/// \param timeToMeasureSec
/// \param timeDeltaSec
/// \return
///
int64_t BaslerTofCamera::GetMaxAbsGevIEE1588OffsetFromMasterInTimeWindow(double timeToMeasureSec, double timeDeltaSec){

    this->Send_DataSet_Latch();
    int *ptrGevIEEE1588OffsetFromMaster;
    this->Get_Offset_From_Master(ptrGevIEEE1588OffsetFromMaster);

    StopWatch m_StopWatch;
    m_StopWatch.reset();

    int maxOffset = 0; // Maximum of offsets from master
    uint32_t n(0);  // Number of samples
    double currTime(0); // Current time

    do
    {
        currTime = m_StopWatch.get(false); // Update current time
        if (currTime >= n * timeDeltaSec)
        {
            // Time for next sample has elapsed.
            // Latch IEEE1588 data set to get offset from master.
            this->Send_DataSet_Latch();
            maxOffset = std::max(maxOffset, std::abs(*ptrGevIEEE1588OffsetFromMaster) ); // Maximum of offsets from master.
            n++; // Increase number of samples.
        }
        mSleep(1);
    } while (currTime <= timeToMeasureSec);
    return maxOffset; // Return maximum of offsets from master for given time interval.
}

void BaslerTofCamera::SetupAndSync(){

    /*Partie de setup de la caméra*/
    this->Set_Trigger_Mode(1); // Passe le trigger mode sur on
    this->Set_Trigger_Source(3); // Active IEE1588 (PTP) et passe en SyncTimer



    /*Partie de choix du master*/

    this->Send_DataSet_Latch();// Latch IEEE1588 status

    CEnumerationPtr ptrGevIEEE1588StatusLatched = m_Camera.GetParameter("GevIEEE1588StatusLatched");

    while (ptrGevIEEE1588StatusLatched->ToString() == "Listening") // Wait the satus to be set
    {
        this->Send_DataSet_Latch(); // Latch GevIEEE1588 status.
        cout << "." << std::flush;
        mSleep(1000);
    }

    Send_Info_Message("Status apres negociation : "+ (string)ptrGevIEEE1588StatusLatched->ToString());
    // Mets à jour le status IsMaster de la camera
    if (ptrGevIEEE1588StatusLatched->ToString() == "Master")
    {
        this->Set_Is_Master(true);
    }
    else
    {
        this->Set_Is_Master(false);
    }

    /* Partie de synchronisation */

    const uint64_t tsOffsetMax = 10000;
    Send_Info_Message("En attente de la stabilisation de l'horloge maitre sous "+ std::to_string(tsOffsetMax) +" ns ");
    if(!this->Get_Is_Master()){
        uint64_t tsOffset;
        do{
            tsOffset = GetMaxAbsGevIEE1588OffsetFromMasterInTimeWindow(1.0, 0.1);
            Send_Info_Message("Max offset of the camera = " + std::to_string(tsOffset) + " ns ");
        } while(tsOffset >= tsOffsetMax);
    }
}

void BaslerTofCamera::mSleep(int sleepMs)      // Use a wrapper
{
#ifdef CIH_LINUX_BUILD
    usleep(sleepMs * 1000);   // usleep takes sleep time in us
#endif
#ifdef CIH_WIN_BUILD
    Sleep(sleepMs);           // Sleep expects time in ms
#endif
}



}



