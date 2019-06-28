#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/gui_leenby/main_window.hpp"
#include "../include/gui_leenby/corix_param.hpp"
#include "../include/gui_leenby/lidarview.hpp"
#include <ros/package.h>
#include <string>
#include <QSizePolicy>

using namespace Qt;

void gui_leenby::MainWindow::restoreControls()
{
    //Réactivation des commandes
    qnode.changerCommande(0,0,0,0,0,0);
    joystick->reset();
    joystick->setEnabled(true);
    m_boutonStop->setEnabled(true);
    m_boutonTeteArriere->setEnabled(true);
    m_boutonTeteAvant->setEnabled(true);
    m_boutonTeteDroite->setEnabled(true);
    m_boutonTeteGauche->setEnabled(true);
    /*tocomplete*/
}


void gui_leenby::MainWindow::emergencyStop()
{
    //Procédure d'arrêt d'urgence
    qnode.changerCommande(0,0,0,0,0,0);
    joystick->setEnabled(false);
    m_boutonStop->setEnabled(false);
    m_boutonTeteArriere->setEnabled(false);
    m_boutonTeteAvant->setEnabled(false);
    m_boutonTeteDroite->setEnabled(false);
    m_boutonTeteGauche->setEnabled(false);
    /*tocomplete*/
}

void gui_leenby::MainWindow::joystickCallback()
{
    // Les coefficents 100 peuvent et doivent être modifiés en fonction de l'utilisation
    qnode.changerCommande( (( joystick->getYBase()-joystick->getY() )/100.0) * VIT_LIN_MAX ,0,0,0,0, ((joystick->getXBase()-joystick->getX())/100.0) * VIT_ROT_MAX );
}

void gui_leenby::MainWindow::cameraCallback()
{
    if(!qnode.retour.isNull())
    {
        m_renduCamGauche->setPixmap(QPixmap::fromImage(qnode.retour));
        m_renduCamDroite->setPixmap(QPixmap::fromImage(qnode.retour));
    }
    else
    {
        ROS_WARN("Retour null");
    }
}

void gui_leenby::MainWindow::bougerTeteAvant()
{

    QMessageBox::information(this, "Fonction bougerTeteAvant()", "Fonction a definir");
    /*tocomplete*/
}

void gui_leenby::MainWindow::bougerTeteArriere()
{
    QMessageBox::information(this, "Fonction bougerTeteArriere()", "Fonction a definir");
    /*tocomplete*/
}

void gui_leenby::MainWindow::bougerTeteGauche()
{
    QMessageBox::information(this, "Fonction bougerTeteGauche()", "Fonction a definir");
    /*tocomplete*/
}

void gui_leenby::MainWindow::bougerTeteDroite()
{
    QMessageBox::information(this, "Fonction bougerTeteDroite()", "Fonction a definir");
    /*tocomplete*/
}

gui_leenby::MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    qnode.init();

    this->setMaximumHeight(1000);
    this->setMinimumWidth(1250);

    menuMode = menuBar()->addMenu("&Mode de l'application");
        modeAnimateur = new QAction("&Animateur", this);
        QObject::connect(modeAnimateur, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
        modeAnimateur->setShortcut(QKeySequence("Ctrl+1"));
        modeUsager = new QAction("&Usager", this);
        QObject::connect(modeUsager, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
        modeUsager->setShortcut(QKeySequence("Ctrl+2"));
    menuMode->addAction(modeAnimateur);
    menuMode->addAction(modeUsager);

    menuParametre = menuBar()->addMenu("&Parametres");
        reset = new QAction("&Reset", this);
        QObject::connect(reset, SIGNAL(triggered()), this, SLOT(restoreControls()));
        reset->setShortcut(QKeySequence("Ctrl+Alt+Space"));
        quitter = new QAction("&Quitter", this);
        QObject::connect(quitter, SIGNAL(triggered()), qApp, SLOT(quit()));
        quitter->setShortcut(QKeySequence("Ctrl+q"));
    menuParametre->addAction(reset);
    menuParametre->addAction(quitter);


    zoneCentrale = new QTabWidget;

    ongletTelemetrie = new QWidget;
    ongletComportements = new QWidget;

    telemetrieContainer = new QBoxLayout(QBoxLayout::Direction(2));

    camContainer = new QHBoxLayout;

        m_renduCamGauche = new QLabel();
        m_renduCamGauche->setPixmap(QPixmap(":/images/oeil.jpeg"));

        batteryLevel = new QProgressBar();
        batteryLevel->setOrientation(Qt::Vertical);
        batteryLevel->setTextVisible(true);
        batteryLevel->setStyleSheet("QProgressBar::chunk { background-color:rgb(20,130,20); }");
        batteryLevel->setAlignment(Qt::AlignCenter);
        batteryLevel->setValue(60);

        m_renduCamDroite = new QLabel();
        m_renduCamDroite->setPixmap(QPixmap(":/images/oeil.jpeg"));

        m_renduCamGauche->setAlignment(Qt::AlignCenter);
        batteryLevel->setAlignment(Qt::AlignCenter);
        m_renduCamDroite->setAlignment(Qt::AlignCenter);

    QObject::connect(&qnode, SIGNAL(hasReceivedImage()), this, SLOT(cameraCallback()));
    camContainer->addWidget(m_renduCamGauche);
    camContainer->addWidget(batteryLevel);
    camContainer->addWidget(m_renduCamDroite);

    stopContainer = new QHBoxLayout;
        m_boutonStop = new QPushButton("STOP");
        m_boutonStop->setStyleSheet("QPushButton { background-color : rgb(175,50,50);}");
        m_boutonStop->setShortcut(QKeySequence("Space"));
        QObject::connect(m_boutonStop, SIGNAL(clicked()), this, SLOT(emergencyStop()));
    stopContainer->addWidget(m_boutonStop);

    bottomContainer = new QHBoxLayout;

        boutonDeplacementContainer = new QVBoxLayout;
            joystick = new JoystickWidget(ongletTelemetrie);
            //joystick->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Expanding);
            QObject::connect(joystick, SIGNAL(hasMoved()), this, SLOT(joystickCallback()));

        boutonDeplacementContainer->addWidget(joystick);

        lidarsOutput = new QVBoxLayout;

            teteLidar = new LidarView("/scan");
            //baseLidar = new LidarView("/scan");

            lidarsOutput->addWidget(teteLidar);
            //lidarsOutput->addWidget(baseLidar);

        boutonTeteContainer = new QVBoxLayout;

            gridBoutonTete = new QGridLayout;
                txtTete = new QPushButton("Controle tete");
                txtTete->setStyleSheet("QPushButton { color : black; border:0px}");
                txtTete->setDisabled(true);
                m_boutonTeteAvant = new QPushButton("Avant");
                m_boutonTeteAvant->setShortcut(QKeySequence("z"));
                qnode.connect(m_boutonTeteAvant, SIGNAL(clicked()),this, SLOT(bougerTeteAvant()));
                m_boutonTeteArriere = new QPushButton("Arriere");
                m_boutonTeteArriere->setShortcut(QKeySequence("s"));
                QObject::connect(m_boutonTeteArriere, SIGNAL(clicked()),this, SLOT(bougerTeteArriere()));
                m_boutonTeteGauche = new QPushButton("Gauche");
                m_boutonTeteGauche->setShortcut(QKeySequence("q"));
                QObject::connect(m_boutonTeteGauche, SIGNAL(clicked()),this, SLOT(bougerTeteGauche()));
                m_boutonTeteDroite = new QPushButton("Droite");
                m_boutonTeteDroite->setShortcut(QKeySequence("d"));
                QObject::connect(m_boutonTeteDroite, SIGNAL(clicked()),this, SLOT(bougerTeteDroite()));

            gridBoutonTete->addWidget(txtTete,0,0,1,3);
            gridBoutonTete->addWidget(m_boutonTeteAvant,1,1);
            gridBoutonTete->addWidget(m_boutonTeteArriere,3,1);
            gridBoutonTete->addWidget(m_boutonTeteGauche,2,0);
            gridBoutonTete->addWidget(m_boutonTeteDroite,2,2);

        boutonTeteContainer->addLayout(gridBoutonTete);

    bottomContainer->addLayout(boutonDeplacementContainer);
    bottomContainer->addLayout(lidarsOutput);
    bottomContainer->addLayout(boutonTeteContainer);

    telemetrieContainer->addLayout(camContainer);
    telemetrieContainer->addLayout(stopContainer);
    telemetrieContainer->addLayout(bottomContainer);


    layoutComportements = new QVBoxLayout;

    ongletTelemetrie->setLayout(telemetrieContainer);
    ongletComportements->setLayout(layoutComportements);

    zoneCentrale->addTab(ongletTelemetrie, "Telemetrie");
    zoneCentrale->addTab(ongletComportements, "Comportements");

    setCentralWidget(zoneCentrale);

}

gui_leenby::MainWindow::~MainWindow() {}


