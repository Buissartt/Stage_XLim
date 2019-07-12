#include <QtGui>
#include <QMessageBox>
#include <QTextEdit>
#include <iostream>
#include "../include/gui_leenby/main_window.hpp"
#include "../include/gui_leenby/corix_param.hpp"
#include "../include/gui_leenby/rvizwidget.hpp"
#include <ros/package.h>
#include <string>

using namespace Qt;

// Fonction de publication des messages pour faire parler le robot
void gui_leenby::MainWindow::publishStringMessage()
{
    qnode.publishParoleMsg(messageToSay->toPlainText().toStdString());
    this->messageToSay->clear();
}

// Fonction de réactivation des commandes
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
    qnode.publishParoleMsg("Commandes réactivées !");
    //TODO: En fonction des modifications de l'arrêt d'urgence il faudra modifier cette fonction
}

// Fonction d'arrêt d'urgence
void gui_leenby::MainWindow::emergencyStop()
{
    //Procédure d'arrêt d'urgence 
    qnode.changerCommande(0,0,0,0,0,0);
    joystick->reset();
    useController = false;
    joystick->setEnabled(false);
    m_boutonStop->setEnabled(false);
    m_boutonTeteArriere->setEnabled(false);
    m_boutonTeteAvant->setEnabled(false);
    m_boutonTeteDroite->setEnabled(false);
    m_boutonTeteGauche->setEnabled(false);
    system("mpg123 /home/mix/Téléchargements/r2d2_scream_converted.mp3");
    //TODO: Des tests et vérification sont nécéssaires pour garantir un VRAI arrêt d'urgence
}

// Fonction callback du joystick virtuel
void gui_leenby::MainWindow::joystickCallback()
{
    // Les coefficents 100 peuvent et doivent être modifiés en fonction de l'utilisation
    qnode.changerCommande( (( joystick->getYBase()-joystick->getY() )/100.0) * VIT_LIN_MAX ,0,0,0,0, ((joystick->getXBase()-joystick->getX())/100.0) * VIT_ROT_MAX );
    //TODO: Modifier le traitement des informations publiées
}

// Fonction callback de la manette physique
void gui_leenby::MainWindow::controllerCallback(double x, double y)
{
    if(useController)
    {
        if(x>0.15 || y>0.15 || x<-0.15 || y<-0.15)
        {
            joystick->setX(-y*100+joystick->getXBase());
            joystick->setY(-x*100+joystick->getYBase());
        }
        else
        {
            joystick->setX(joystick->getXBase());
            joystick->setY(joystick->getXBase());
        }
        this->joystickCallback();
    }
}

// Fonction de callback du retour des caméras
void gui_leenby::MainWindow::cameraCallback()
{
    if(!qnode.retourDroite.isNull() && !qnode.retourGauche.isNull())
    {
        m_renduCamGauche->setPixmap(QPixmap::fromImage(qnode.retourGauche));
        m_renduCamDroite->setPixmap(QPixmap::fromImage(qnode.retourDroite));
    }
    else
    {
        ROS_WARN("Retour null");
    }
}

// Fonction pour bouger la tete en avant
void gui_leenby::MainWindow::bougerTeteAvant()
{

    //QMessageBox::information(this, "Fonction bougerTeteAvant()", "Fonction a definir");
    //TODO: Implémentation du mouvement de la tête vers l'avant
    qnode.publishParoleMsg("Hé les romains, vous êtes des romaines");
}

// Fonction pour bouger la tete en arrière
void gui_leenby::MainWindow::bougerTeteArriere()
{
    QMessageBox::information(this, "Fonction bougerTeteArriere()", "Fonction a definir");
    //TODO: Implémentation du mouvement de la tête vers l'arrière
}

// Fonction pour bouger la tete a gauche
void gui_leenby::MainWindow::bougerTeteGauche()
{
    QMessageBox::information(this, "Fonction bougerTeteGauche()", "Fonction a definir");
    //TODO: Implémentation du mouvement de la tête vers la gauche
}

// Fonction pour bouger la tete a droite
void gui_leenby::MainWindow::bougerTeteDroite()
{
    QMessageBox::information(this, "Fonction bougerTeteDroite()", "Fonction a definir");
    //TODO: Implémentation du mouvement de la tête vers la droite
}

// Fonction d'activatino / désactivation de la manette
void gui_leenby::MainWindow::switchUseController()
{
    useController = !useController;
    if(useController)
    {
       controlerState->setText("Manette activée");
       qnode.publishParoleMsg("Manette activée !");
    }
    else
    {
        controlerState->setText("Manette désactivée");
        qnode.publishParoleMsg("Manette désactivée !");
    }
    joystick->reset();
    qnode.changerCommande(0,0,0,0,0,0);
}

// Constructeur de la fenêtre
gui_leenby::MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    qnode.init();
    QObject::connect(&qnode, SIGNAL(manetteMoved(double,double)), this, SLOT(controllerCallback(double,double)) );

    useController = false;
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
        switchController = new QAction("&Activer/Desactiver manette", this);
        QObject::connect(switchController, SIGNAL(triggered()), this, SLOT(switchUseController()));
        switchController->setShortcut(QKeySequence("Ctrl+j"));
        reset = new QAction("&Reset", this);
        QObject::connect(reset, SIGNAL(triggered()), this, SLOT(restoreControls()));
        reset->setShortcut(QKeySequence("Ctrl+Alt+Space"));
        quitter = new QAction("&Quitter", this);
        QObject::connect(quitter, SIGNAL(triggered()), qApp, SLOT(quit()));
        quitter->setShortcut(QKeySequence("Ctrl+q"));
    menuParametre->addAction(switchController);
    menuParametre->addAction(reset);
    menuParametre->addAction(quitter);


    zoneCentrale = new QTabWidget;

    ongletTelemetrie = new QWidget;
    ongletRviz = new QWidget;
    ongletParole = new QWidget;

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
            controlerState= new QPushButton("Manette désactivée");
            QObject::connect(controlerState, SIGNAL(clicked()), this, SLOT(switchUseController()));
            joystick = new JoystickWidget();
            QObject::connect(joystick, SIGNAL(hasMoved()), this, SLOT(joystickCallback()));

        boutonDeplacementContainer->addWidget(controlerState);
        boutonDeplacementContainer->addWidget(joystick);

        lidarsOutput = new QVBoxLayout;
            retourLidar = new LidarView("/scan");
            lidarsOutput->addWidget(retourLidar);

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


    layoutRviz = new QVBoxLayout;
        RvizWidget *viewer = new RvizWidget();
        layoutRviz->addWidget(viewer);

    layoutParole = new QVBoxLayout;
        messageToSay = new QTextEdit();
        sendButton = new QPushButton("Dire");

        layoutParole->addWidget(messageToSay);
        layoutParole->addWidget(sendButton);
        QObject::connect(sendButton, SIGNAL(clicked()), this, SLOT(publishStringMessage()));



    ongletTelemetrie->setLayout(telemetrieContainer);
    ongletRviz->setLayout(layoutRviz);
    ongletParole->setLayout(layoutParole);

    zoneCentrale->addTab(ongletTelemetrie, "Telemetrie");
    zoneCentrale->addTab(ongletRviz, "Rviz");
    zoneCentrale->addTab(ongletParole, "Parole");

    setCentralWidget(zoneCentrale);

}

// Destructeur de la fenêtre
gui_leenby::MainWindow::~MainWindow() {}


