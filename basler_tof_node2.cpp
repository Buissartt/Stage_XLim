#include "stdafx.h"
#include <string.h>
#include <ConsumerImplHelper/ToFCamera.h>
#include <GenTL/PFNC.h>
#include <iostream>
#include <iomanip>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "BaslerTofCam.h"
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <basler_tof/basler_tof_nodeConfig.h>


#if defined (_MSC_VER) && defined (_WIN32)
// You have to delay load the GenApi libraries used for configuring the camera device.
// Refer to the project settings to see how to instruct the linker to delay load DLLs.
// ("Properties->Linker->Input->Delay Loaded Dlls" resp. /DELAYLOAD linker option).
#  pragma message( "Remember to delayload these libraries (/DELAYLOAD linker option):")
#  pragma message( "    /DELAYLOAD:\"" DLL_NAME("GCBase") "\"")
#  pragma message( "    /DELAYLOAD:\"" DLL_NAME("GenApi") "\"")
#endif


using namespace GenTLConsumerImplHelper;
using namespace GenApi;
using namespace std;
using namespace BaslerToF;

void mSleep(int sleepMs)      // Use a wrapper
{
#ifdef CIH_LINUX_BUILD
    usleep(sleepMs * 1000);   // usleep takes sleep time in us
#endif
#ifdef CIH_WIN_BUILD
    Sleep(sleepMs);           // Sleep expects time in ms
#endif
}

///////////Variables///////////

//Pointer vers les ros publisher
ros::Publisher* pDepthImage; //Image de la profondeur
ros::Publisher* pIntensityImage; //Image de l'intensite lumineuse
ros::Publisher* pConfidenceImage; //Image de la precision
ros::Publisher* pPointCloud; //Nuage de point

//Identificateur de trame ROS
ulong seq_ID;
//Timestamp de la frame precedente
ros::Time oldstamp;
ros::Duration delta;
//Objet camera
BaslerTofCamera Camera;
//Frame de la camera
string frame = "basler";


///////////Fonctions///////////

// structure couleur dans l'espace RGB
typedef struct {
    double r;       // percent
    double g;       // percent
    double b;       // percent
} rgb;

// structure couleur dans l'espace HSV
typedef struct {
    double h;       // angle in degrees
    double s;       // percent
    double v;       // percent
} hsv;

// fonction permettant la conversion d'un espace de couleur à l'autre
static hsv   rgb2hsv(rgb in);
static rgb   hsv2rgb(hsv in);

///
/// \brief rgb2hsv : convert a color from rgb space to hsv space
/// \param iframe_raten : color to convert
/// \return
///
hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0
        // s = 0, v is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
        if( in.g >= max )
            out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
        else
            out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}

///
/// \brief hsv2rgb : convert a color from hsv space to rgb space
/// \param in : color to convert
/// \return
///
rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;
}

void cb_dyn(basler_tof::basler_tof_nodeConfig &config, uint32_t level)
{

    if((level & 8192) < 1 && (level & 16384) < 1)
    {
        // Channel
        if((level & 1) > 0)
        {
            Camera.Set_Device_Channel(config.basler_tof_device_channel);
            Camera.Get_Device_Channel(&config.basler_tof_device_channel);
        }
        // disable LED
        if((level & 2) > 0)
        {
            Camera.Set_Led_Disable(config.basler_tof_disable_led);
            Camera.Get_Led_Disable(&config.basler_tof_disable_led);
        }
        // processing mode
        if((level & 4) > 0)
        {
            Camera.Set_Processing_Mode(config.basler_tof_processing_mode);
            Camera.Get_Processing_Mode(&config.basler_tof_processing_mode);
        }
        // trigger mode
        if((level & 8) > 0)
        {
            Camera.Set_Trigger_Mode(config.basler_tof_tigger_mode);
            Camera.Get_Trigger_Mode(&config.basler_tof_tigger_mode);
        }
        // trigger source
        if((level & 16) > 0)
        {
            Camera.Set_Trigger_Source(config.basler_tof_tigger_source);
            Camera.Get_Trigger_Source(&config.basler_tof_tigger_source);
        }
        // frame rate
        if((level & 32) > 0)
        {
            Camera.Set_Acquisition_Frame_Rate(config.basler_tof_acquisition_frame_rate);
            Camera.Get_Acquisition_Frame_Rate((float*)&config.basler_tof_acquisition_frame_rate);
        }
        // exposure time
        if((level & 64) > 0)
        {
            Camera.Set_Exposure_Time(config.basler_tof_exposure_time);
            Camera.Get_Exposure_Time((float*)&config.basler_tof_exposure_time);
        }
        // exposure time 2
        if((level & 128) > 0)
        {
            Camera.Set_Exposure_Time2(config.basler_tof_exposure_time2);
            Camera.Get_Exposure_Time2((float*)&config.basler_tof_exposure_time2);
        }
        // exposure auto
        if((level & 256) > 0)
        {
            Camera.Set_Exposure_Auto(config.basler_tof_exposure_auto);
            Camera.Get_Exposure_Auto(&config.basler_tof_exposure_auto);
        }
        // agility
        if((level & 512) > 0)
        {
            Camera.Set_Agility(config.basler_tof_agility);
            Camera.Get_Agility((float*)&config.basler_tof_agility);
        }
        // delay
        if((level & 1024) > 0)
        {
            Camera.Set_Delay(config.basler_tof_delay);
            Camera.Get_Delay(&config.basler_tof_delay);
        }
        // manual trigger
        if((level & 2048) > 0)
        {
            Camera.Send_Trigger_Software();
        }
        // sync rate
        if((level & 4096) > 0)
        {
            Camera.Set_Sync_Rate(config.basler_tof_acquisition_sync_rate);
            Camera.Get_Sync_Rate((float*)&config.basler_tof_acquisition_sync_rate);
        }
    }
    else if((level & 8192) > 0)
    {
        // binning
        if((level & 1) > 0)
        {
            Camera.Set_Binning(config.basler_tof_binning);
            Camera.Get_Binning(&config.basler_tof_binning);
        }
        // width
        if((level & 2) > 0)
        {
            Camera.Set_Width(config.basler_tof_width);
            Camera.Get_Width(&config.basler_tof_width);
        }
        // height
        if((level & 4) > 0)
        {
            Camera.Set_Height(config.basler_tof_height);
            Camera.Get_Height(&config.basler_tof_height);
        }
        // depth max
        if((level & 8) > 0)
        {
            Camera.Set_Z_Max(config.basler_tof_depth_max);
            Camera.Get_Z_Max(&config.basler_tof_depth_max);
        }
        // offset x
        if((level & 16) > 0)
        {
            Camera.Set_Offset_X(config.basler_tof_offset_x);
            Camera.Get_Offset_X(&config.basler_tof_offset_x);
        }
        // offset y
        if((level & 32) > 0)
        {
            Camera.Set_Offset_Y(config.basler_tof_offset_y);
            Camera.Get_Offset_Y(&config.basler_tof_offset_y);
        }
        // depth min
        if((level & 64) > 0)
        {
            Camera.Set_Z_Min(config.basler_tof_depth_min);
            Camera.Get_Z_Min(&config.basler_tof_depth_min);
        }
        // range enable & pixel format
        if((level & 128) > 0)
        {
            Camera.Set_Component_Range(config.basler_tof_range_enable,config.basler_tof_range_pixel_format);
            Camera.Get_Component_Range(&config.basler_tof_range_enable,&config.basler_tof_range_pixel_format);
        }
        // intensity enable & pixel format
        if((level & 256) > 0)

        {
            Camera.Set_Component_Intensity(config.basler_tof_intensity_enable,config.basler_tof_intensity_pixel_format);
            Camera.Get_Component_Intensity(&config.basler_tof_intensity_enable,&config.basler_tof_intensity_pixel_format);
        }
        // confidence enable & pixel format
        if((level & 512) > 0)
        {
            Camera.Set_Component_Confidence(config.basler_tof_confidence_enable,config.basler_tof_confidence_pixel_format);
            Camera.Get_Component_Confidence(&config.basler_tof_confidence_enable,&config.basler_tof_confidence_pixel_format);
        }
    }
    //quality
    else if((level & 16384) > 0)
    {
        if((level & 1) > 0)
        {
            // confidence threshold
            Camera.Set_Confidence_Threshold(config.basler_tof_confidence_threshold);
            Camera.Get_Confidence_Threshold(&config.basler_tof_confidence_threshold);
        }
        if((level & 2) > 0)
        {
            // spacial filter enable
            Camera.Set_Spatial_Filter_Enable(config.basler_tof_spatial_filter_enable);
            Camera.Get_Spatial_Filter_Enable(&config.basler_tof_spatial_filter_enable);
        }
        if((level & 4) > 0)
        {
            // temporal filter enable
            Camera.Set_Temporal_Filter_Enable(config.basler_tof_temporal_filter_enable);
            Camera.Get_Temporal_Filter_Enable(&config.basler_tof_temporal_filter_enable);
        }

        if((level & 8) > 0)
        {
            // temporal filter strenght
            Camera.Set_Temporal_Filter_Strength(config.basler_tof_temporal_filter_strenght);
            Camera.Get_Temporal_Filter_Strength(&config.basler_tof_temporal_filter_strenght);
        }
        if((level & 16) > 0)
        {
            // outlier tolerance
            Camera.Set_Outlier_Tolerance(config.basler_tof_outlier_tolerance);
            Camera.Get_Outlier_Tolerance(&config.basler_tof_outlier_tolerance);
        }
    }
}

void ErrorMessageDisplay(string message)
{
    ROS_ERROR(message.c_str());
}

void WarningMessageDisplay(string message)
{
    ROS_WARN(message.c_str());
}

void InfoMessageDisplay(string message)
{
    ROS_INFO(message.c_str());
}

void OnNewImage(BufferParts& parts)
{
    // preparation de l'en tête du message

    //recuperation du timestamp
    ros::Time actualstamp = ros::Time::now();
    //incrementation de l'identificateur de trame
    seq_ID++;
    //creation de l'entête
    std_msgs::Header header;
    header.seq = seq_ID;
    header.stamp = actualstamp;
    header.frame_id = frame;

    //calcul des fps
    ulong time_fps = actualstamp.toNSec() - oldstamp.toNSec();
    float FPS = 1000000000.0 / time_fps;
    oldstamp=actualstamp; // mise à jour du timestamp de l'ancienne frame

    // nombre de pixel de l'image
    const size_t nPixel = parts[0].width * parts[0].height;
    for(int i=0; i<parts.size(); i++){

        switch(parts[i].dataFormat)
        {

            case 39846080: //PFNC_Coord3D_ABC32f
            {

                //preparation du message ros point cloud 2
                sensor_msgs::PointCloud2 PointCloud_msg;

                PointCloud_msg.header = header;
                PointCloud_msg.height = parts[i].height;
                PointCloud_msg.width = parts[i].width;
                PointCloud_msg.is_dense = false;
                PointCloud_msg.point_step = sizeof(float) * 3;
                PointCloud_msg.row_step = sizeof(float) * 3 * parts[i].width;
                PointCloud_msg.is_bigendian = false;
                PointCloud_msg.fields.resize(3);
                PointCloud_msg.fields[0].name = "x";
                PointCloud_msg.fields[0].offset = 0;
                PointCloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
                PointCloud_msg.fields[0].count = 1;
                PointCloud_msg.fields[1].name = "y";
                PointCloud_msg.fields[1].offset = sizeof(float);
                PointCloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
                PointCloud_msg.fields[1].count = 1;
                PointCloud_msg.fields[2].name = "z";
                PointCloud_msg.fields[2].offset = 2*sizeof(float);
                PointCloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
                PointCloud_msg.fields[2].count = 1;

                //pointeur vers les points du nuage
                CToFCamera::Coord3D *pPoint = (CToFCamera::Coord3D*) parts[i].pData;

                //preparation du message ros image
                sensor_msgs::Image DepthImage_msg;
                DepthImage_msg.header = header;
                DepthImage_msg.height = parts[i].height;
                DepthImage_msg.width = parts[i].width;
                DepthImage_msg.encoding = sensor_msgs::image_encodings::RGB8;
                DepthImage_msg.step = parts[i].width * 3;

                //initialisation des couleur pour la reconstruction de l'image
                uint8_t R = 0;
                uint8_t G = 0;
                uint8_t B = 0;
                rgb rgbcolor;
                hsv hsvcolor;
                hsvcolor.s=1;
                hsvcolor.v=1;

                // z max (pour reconstruction d'une image)
                float z_max = 10; //10 metres

                // pour chaque pixel
                for ( size_t i = 0; i < nPixel; ++i )
                {
                    //on recupère les données dans le buffer (3 x float) et on les met dans un tableau
                    float XYZ[3];
                    XYZ[0] = pPoint->x / 1000;
                    XYZ[1] = pPoint->y / 1000;
                    XYZ[2] = pPoint->z / 1000;

                    // On cree un pointeur vers le premier byte du tableau
                    float* p_XYZ = &XYZ[0];
                    uint8_t* p_b_XYZ = (uint8_t*) p_XYZ;
                    // On met un à un tout les bytes dans les data du message
                    //X
                    PointCloud_msg.data.push_back(*p_b_XYZ);
                    p_b_XYZ++;
                    PointCloud_msg.data.push_back(*p_b_XYZ);
                    p_b_XYZ++;
                    PointCloud_msg.data.push_back(*p_b_XYZ);
                    p_b_XYZ++;
                    PointCloud_msg.data.push_back(*p_b_XYZ);
                    //Y
                    p_b_XYZ++;
                    PointCloud_msg.data.push_back(*p_b_XYZ);
                    p_b_XYZ++;
                    PointCloud_msg.data.push_back(*p_b_XYZ);
                    p_b_XYZ++;
                    PointCloud_msg.data.push_back(*p_b_XYZ);
                    p_b_XYZ++;
                    PointCloud_msg.data.push_back(*p_b_XYZ);
                    //Z
                    p_b_XYZ++;
                    PointCloud_msg.data.push_back(*p_b_XYZ);
                    p_b_XYZ++;
                    PointCloud_msg.data.push_back(*p_b_XYZ);
                    p_b_XYZ++;
                    PointCloud_msg.data.push_back(*p_b_XYZ);
                    p_b_XYZ++;
                    PointCloud_msg.data.push_back(*p_b_XYZ);

                    // on avance dans le buffer
                    pPoint++;
                }

                // on envoie les messages
                pPointCloud->publish(PointCloud_msg);
                break;
            }
            case 35127316: //PFNC_RGB8
            {
                //on prepare le message
                sensor_msgs::Image DepthImage_msg;
                DepthImage_msg.header = header;
                DepthImage_msg.height = parts[i].height;
                DepthImage_msg.width = parts[i].width;
                DepthImage_msg.encoding = sensor_msgs::image_encodings::RGB8;
                DepthImage_msg.step = parts[i].width * 3;

                //pointeur vers les pixel de l'image
                uint8_t *pDepth = (uint8_t*) parts[i].pData;

                //pour chaque pixel
                for ( size_t i = 0; i < nPixel; ++i )
                {
                    //on met le pixel dans les datas du message
                    DepthImage_msg.data.push_back(*pDepth);
                    pDepth++;
                    DepthImage_msg.data.push_back(*pDepth);
                    pDepth++;
                    DepthImage_msg.data.push_back(*pDepth);
                    pDepth++;
                }

                // on publie le message
                pDepthImage->publish(DepthImage_msg);
                break;
            }
            case 17825976: //PFNC_Coord3D_C16
            {
                //on prepare le message
                sensor_msgs::Image DepthImage_msg;
                DepthImage_msg.header = header;
                DepthImage_msg.height = parts[i].height;
                DepthImage_msg.width = parts[i].width;
                DepthImage_msg.encoding = sensor_msgs::image_encodings::MONO16;
                DepthImage_msg.step = parts[i].width * 2;

                //pointeur vers les pixel de l'image
                uint8_t *pDepth = (uint8_t*) parts[i].pData;

                //pour chaque pixel
                for ( size_t i = 0; i < nPixel; ++i )
                {
                    //on met le pixel dans les datas du message
                    DepthImage_msg.data.push_back(*pDepth);
                    pDepth++;
                    DepthImage_msg.data.push_back(*pDepth);
                    pDepth++;
                }

                // on publie le message
                pDepthImage->publish(DepthImage_msg);
                break;
            }
            case 17825799: //PFNC_Mono16
            {
                //on prepare le message
                sensor_msgs::Image IntensityImage_msg;

                IntensityImage_msg.header = header;
                IntensityImage_msg.height = parts[i].height;
                IntensityImage_msg.width = parts[i].width;
                IntensityImage_msg.encoding = sensor_msgs::image_encodings::MONO16;
                IntensityImage_msg.step = parts[i].width * 2;

                //pointeur vers les pixel de l'image
                uint8_t *pIntensity = (uint8_t*) parts[i].pData;

                //pour chaque pixel
                for ( size_t i = 0; i < nPixel; ++i )
                {
                    //on met le pixel dans les datas du message
                    IntensityImage_msg.data.push_back(*pIntensity);
                    pIntensity++;
                    IntensityImage_msg.data.push_back(*pIntensity);
                    pIntensity++;
                }

                // on publie le message
                pIntensityImage->publish(IntensityImage_msg);
                break;
            }
            case 17825991: //PFNC_Confidence16
            {
                //on prepare le message
                sensor_msgs::Image ConfidenceImage_msg;
                ConfidenceImage_msg.header = header;
                ConfidenceImage_msg.height = parts[i].height;
                ConfidenceImage_msg.width = parts[i].width;
                ConfidenceImage_msg.encoding = sensor_msgs::image_encodings::MONO16;
                ConfidenceImage_msg.step = parts[i].width * 2;

                //pointeur vers les pixel de l'image
                uint8_t *pConfidence = (uint8_t*) parts[i].pData;

                //pour chaque pixel
                for ( size_t i = 0; i < nPixel; ++i )
                {
                    //on met le pixel dans les datas du message
                    ConfidenceImage_msg.data.push_back(*pConfidence);
                    pConfidence++;
                    ConfidenceImage_msg.data.push_back(*pConfidence);
                    pConfidence++;
                }

                // on publie le message
                pConfidenceImage->publish(ConfidenceImage_msg);
                break;
            }
            default:
            {
            }
        }
    }
}

void CameraGrabbingThread()
{
    Camera.GrabContinuous();
}

int main(int argc, char **argv)
{
    int exitCode = EXIT_SUCCESS;

    // Boucle pour savoir s'il y a deux caméras
    int choixNbCamera;
    do {
        cout << "Une ou deux caméra ? (1/2)" << endl;
        //cin >> choixNbCamera;
        choixNbCamera = 2;
        if(choixNbCamera == 1){
            Camera.Set_Work_In_Pairs(false);
        } else if(choixNbCamera == 2){
            Camera.Set_Work_In_Pairs(true);
        } else {
            cout << "Merci d'écrire 1 ou 2 en fonction de votre choix" << endl;
        }
    } while(choixNbCamera !=1 && choixNbCamera !=2);

    // init ROS
    ros::init(argc, argv, "basler_tof_node2");
    ros::NodeHandle n("~");

    // advertise to publish
    ros::Publisher PointCloud_pub = n.advertise<sensor_msgs::PointCloud2>("Cloud", 1000);
    ros::Publisher DepthImage_pub = n.advertise<sensor_msgs::Image>("DepthImage", 1000);
    ros::Publisher IntensityImage_pub = n.advertise<sensor_msgs::Image>("IntensityImage", 1000);
    ros::Publisher ConfidenceImage_pub = n.advertise<sensor_msgs::Image>("ConfidenceImage", 1000);

    // recuperation des addresses des Publishers
    pPointCloud=&PointCloud_pub;
    pDepthImage=&DepthImage_pub;
    pIntensityImage=&IntensityImage_pub;
    pConfidenceImage=&ConfidenceImage_pub;

    string IP, serial;
    if(!n.getParam("IP",IP))
        ErrorMessageDisplay("no IP param");
    if(!n.getParam("serial",serial))
        ErrorMessageDisplay("no serial param");
    if(!n.getParam("frame",frame))
        ErrorMessageDisplay("no frame param");


    WarningMessageDisplay("IP : " + IP);
    WarningMessageDisplay("serial : " + serial);
    WarningMessageDisplay("frame : " + frame);

    try
    {

        // Abonnement aux events de la camera
        Camera.eInfoMessage = &InfoMessageDisplay; // Nouveau message d'info
        Camera.eWarningMessage = &WarningMessageDisplay; // Nouveau message d'avertissement
        Camera.eErrorMessage = &ErrorMessageDisplay; // Nouveau message d'erreur
        Camera.eNewImage = &OnNewImage; // Nouvelle image

        // Connection à la caméra
        Camera.Connect(serial,IP);

        if(Camera.Get_Work_In_Pairs()){
            Camera.SetupAndSync();

            WarningMessageDisplay("Settings a la main");

            Camera.Set_Processing_Mode(0);
            int* processingMode = new int();
            Camera.Get_Processing_Mode(processingMode);

            Camera.Set_Exposure_Auto(0);
            int* value_expo = new int();
            Camera.Get_Exposure_Auto(value_expo);

            Camera.Set_Sync_Rate(20);
            float *syncRate = new float();
            Camera.Get_Sync_Rate(syncRate);

            Camera.Set_Exposure_Time(6000);
            float *expoTime = new float();
            Camera.Get_Exposure_Time(expoTime);

            Camera.Set_IEEE1588(true);
            bool *ptp = new bool();
            Camera.Get_IEEE1588(ptp);

            Camera.Set_Trigger_Mode(1);
            int *triggerMode = new int();
            Camera.Get_Trigger_Mode(triggerMode);

            Camera.Set_Trigger_Source(3);
            int *triggerSource = new int();
            Camera.Get_Trigger_Source(triggerSource);

            if(Camera.Get_Is_Master()){
                Camera.Set_Trigger_Delay(0.0);
                float *triggerDelay = new float();
                Camera.Get_Trigger_Delay(triggerDelay);
            } else {
                Camera.Set_Trigger_Delay(25000.0); //µs
                float *triggerDelay = new float();
                Camera.Get_Trigger_Delay(triggerDelay);
            }

            WarningMessageDisplay("Camera configured !");
        }
        // dynamic reconfigure
        dynamic_reconfigure::Server<basler_tof::basler_tof_nodeConfig> srv;
        dynamic_reconfigure::Server<basler_tof::basler_tof_nodeConfig>::CallbackType f;
        f = boost::bind(&cb_dyn, _1, _2);
        srv.setCallback(f);


        // Lancement de l'acquisition en continue
        thread Thread4Grabbing(CameraGrabbingThread);

        // Reglage de la cadence de la boucle
        ros::Rate looprate(100);

        while (ros::ok())
        {
            // lecture si une donnée est envoyer au noeud
            ros::spinOnce();

            // sleep pour respecter la cadence
            looprate.sleep();

        }// fin de la boucle

        // Arret de l'acquisition
        Camera.StopGrab();

        // Attente de la fin de l'arret de l'acquisition (attente que le thread soit terminé)
        Thread4Grabbing.join();

        // Deconnection de la caméra
        Camera.Disconnect();
    }
    catch ( exception e )
    {
        ErrorMessageDisplay("error in main");
        Camera.Disconnect();
        exitCode = EXIT_FAILURE;
    }


    return exitCode;
}
