#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>

void callbackMsg(const std_msgs::String::ConstPtr& msg);

int rate,pitch,volume;
std::string lang,voice;

int main(int argc, char ** argv){
    ros::init(argc, argv,"vocal_synthesis_subscriber");
    ros::NodeHandle n("~");



    rate = n.param("rate",0);
    volume = n.param("volume",0);
    pitch = n.param("pitch",0);
    lang = n.param<std::string>("language","fr");
    voice = n.param<std::string>("voice","child_female");

    ros::Rate loop_rate(30);
    ros::Subscriber subscriber = n.subscribe<std_msgs::String>("/messageToSay",10,callbackMsg);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


void callbackMsg(const std_msgs::String::ConstPtr& msg)
{
    // Les voix female2 et child_female sont bien
    /* Config1
     * Voix: female2
     * Pitch: 0
     * Rate: -25
     *
     * Command:     spd-say -l fr female2 -r -25 -p 0 "Texte ici"
     */

    /* Config2
     * Voix: child_female
     * Pitch: 10
     * Rate: -20
     * Command:     spd-say -l fr -t child_female -r -20 -p 10 "Texte ici"
     */
    std::string base("spd-say");

    std::string s_lang = " -l " + lang;
    std::string s_rate = " -r " + std::to_string(rate);
    std::string s_volume = " -i "+ std::to_string(volume);
    std::string s_pitch = " -p "+ std::to_string(pitch);
    std::string s_voice = " -t " + voice;

    std::string commande = base+s_rate+s_lang+s_volume+s_pitch+s_voice+" \""+msg->data.c_str()+"\"";
    ROS_ERROR_STREAM(commande);
    system(&commande.at(0));
    /*TODO:
    * spd-say a une voix très robotique
    * En utilisant espeak couplé à mbrola, on obtient de bon résultats
    */
}

/*
 * Liste des options disponibles pour la commande spd-say
 *
 * [min;max;default]
 * {enum1, enum2, ...}
 *
 * -r [-100;100;0]                  Rate (vitesse)
 * -p [-100;100;0]                  Pitch (tonalité)
 * -i [-100;100;0]                  Volume
 * -l {fr,en,etc..}                 Language (ISO code)
 * -t {male1, male2, male3, female1, female2,female3, child_male, child_female} Voix
 *
 *
 * -m {none, some, all}             Punctuation mode
 * -s                               Spell the message
 * -p {important, message,text, notification, progress}  Priority (default:text)
 * -w                               Wait until the end of the message or until the message is discarded
 * -s                               Stop the current message
 * -C                               Cancel all the messages
 *
 *
 * -v                               Version
 * -h                               Help
 * */
