#ifndef gui_leenby_MAIN_WINDOW_H
#define gui_leenby_MAIN_WINDOW_H

#include <QMainWindow>
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
#include <QTextEdit>
#include "lidarview.hpp"
#include "rvizwidget.hpp"

namespace gui_leenby {

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
        void controllerCallback(double,double); // Manette
        void bougerTeteAvant();
        void bougerTeteArriere();
        void bougerTeteGauche();
        void bougerTeteDroite();
        void switchUseController();
        void publishStringMessage();

private:
        bool useController;
        QNode qnode; // Thread d'interfaçage avec ROS

        QMenu *menuMode;
            QAction *modeAnimateur;
            QAction *modeUsager;
        QMenu *menuParametre;
            QAction *switchController;
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
                            QPushButton *controlerState;
                            JoystickWidget *joystick;
                        QVBoxLayout *lidarsOutput;
                            LidarView* retourLidar; // Retour du lidar
                        QVBoxLayout *boutonTeteContainer;
                            QGridLayout *gridBoutonTete;
                                QPushButton *txtTete;
                                QPushButton *m_boutonTeteAvant;
                                QPushButton *m_boutonTeteArriere;
                                QPushButton *m_boutonTeteGauche;
                                QPushButton *m_boutonTeteDroite;
            QWidget *ongletRviz;
                QVBoxLayout* layoutRviz;

            QWidget *ongletParole;
                QVBoxLayout* layoutParole;
                    QTextEdit *messageToSay;
                    QPushButton *sendButton;
};

}  // namespace gui_leenby

#endif // gui_leenby_MAIN_WINDOW_H
