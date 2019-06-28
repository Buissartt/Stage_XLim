#ifndef gui_leenby_MAIN_WINDOW_H
#define gui_leenby_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
//#include "ui_main_window.h"
#include "qnode.hpp"
#include "joystickwidget.hpp"
#include <QApplication>
#include <QMainWindow>
#include <QLineEdit>
#include <QFormLayout>
#include <QMenuBar>
#include <QAction>
#include <QLabel>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QObject>
#include <QGridLayout>
#include <QBoxLayout>
#include <QProgressBar>
#include <QMessageBox>
#include <QRectF>
#include <QPainter>
#include "lidarview.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace gui_leenby {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
public Q_SLOTS:
        void joystickCallback(); // Gestion du joystick
        void cameraCallback(); // Gestion des retours des caméras
        void emergencyStop(); // Arrêt du'grence
        void restoreControls(); // Réinitialisation des commandes
        void bougerTeteAvant();
        void bougerTeteArriere();
        void bougerTeteGauche();
        void bougerTeteDroite();

private:
        QNode qnode; // Thread d'interfaçage avec ROS

        QMenu *menuMode;
            QAction *modeAnimateur;
            QAction *modeUsager;
        QMenu *menuParametre;
            QAction *reset;
            QAction *quitter;

        QTabWidget *zoneCentrale;
            QWidget *ongletTelemetrie;
                QBoxLayout *telemetrieContainer;
                    QHBoxLayout *camContainer;
                        QLabel *m_renduCamGauche; // Affichage du retour caméra gauche
                        QProgressBar *batteryLevel; //Affichage du niveau de wifi
                        QLabel *m_renduCamDroite; // Affichage du retour caméra droite
                    QHBoxLayout *stopContainer;
                        QPushButton *m_boutonStop; // Bouton d'arrêt d'urgence
                    QHBoxLayout *bottomContainer;
                        QVBoxLayout *boutonDeplacementContainer;
                            JoystickWidget *joystick;
                        QVBoxLayout *lidarsOutput;
                            LidarView* teteLidar; // Retour du lidar de tête
                            LidarView* baseLidar; // Retour du lidar de base
                        QVBoxLayout *boutonTeteContainer;
                            QGridLayout *gridBoutonTete;
                                QPushButton *txtTete;
                                QPushButton *m_boutonTeteAvant;
                                QPushButton *m_boutonTeteArriere;
                                QPushButton *m_boutonTeteGauche;
                                QPushButton *m_boutonTeteDroite;
            QWidget *ongletComportements;
                QVBoxLayout* layoutComportements;
};

}  // namespace gui_leenby

#endif // gui_leenby_MAIN_WINDOW_H
