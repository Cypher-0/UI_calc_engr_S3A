#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "reducer.h"
#include <QString>
#include <QProgressBar>
#include <QFile>
#include <QIODevice>
#include <QTextStream>
#include <QMessageBox>
#include <QFileDialog>
#include <QDir>
#include <QKeySequence>

#include <QDebug>

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), ui(new Ui::MainWindow)
{

    ui->setupUi(this);

    //connect input powers value
    connect(ui->sb_motorPwr,SIGNAL(valueChanged(int)),this,SLOT(actPwrResult(int)));
    connect(ui->sb_motorSpeed,SIGNAL(valueChanged(int)),this,SLOT(actPwrResult(int)));
    connect(ui->dsb_loadMass,SIGNAL(valueChanged(double)),this,SLOT(actPwrResult(double)));
    connect(ui->dsb_cableDiam,SIGNAL(valueChanged(double)),this,SLOT(actPwrResult(double)));
    connect(ui->dsb_spoolDiam,SIGNAL(valueChanged(double)),this,SLOT(actPwrResult(double)));
    connect(ui->dsb_loadHeight,SIGNAL(valueChanged(double)),this,SLOT(actPwrResult(double)));
    connect(ui->dsb_safetyCoef,SIGNAL(valueChanged(double)),this,SLOT(actPwrResult(double)));
    connect(ui->dsb_spoolWidth,SIGNAL(valueChanged(double)),this,SLOT(actPwrResult(double)));

    actPwrResult(0);

    //connect input gears value
    connect(ui->sb_minZ,SIGNAL(valueChanged(int)),this,SLOT(actGearsParams(int)));
    connect(ui->sb_maxZ,SIGNAL(valueChanged(int)),this,SLOT(actGearsParams(int)));
    connect(ui->dsb_minModule,SIGNAL(valueChanged(double)),this,SLOT(actGearsParams(double)));
    connect(ui->dsb_maxModule,SIGNAL(valueChanged(double)),this,SLOT(actGearsParams(double)));
    connect(ui->dsb_moduleStep,SIGNAL(valueChanged(double)),this,SLOT(actGearsParams(double)));
    connect(ui->sb_rpeMat,SIGNAL(valueChanged(int)),this,SLOT(actGearsParams(int)));
    connect(ui->dsb_gearsAlignTolerance,SIGNAL(valueChanged(double)),this,SLOT(actGearsParams(double)));
    connect(ui->dsb_motorAxisDiam,SIGNAL(valueChanged(double)),this,SLOT(actGearsParams(double)));
    connect(ui->dsb_lsDiam,SIGNAL(valueChanged(double)),this,SLOT(actGearsParams(double)));
    connect(ui->dsb_wAxisDiam,SIGNAL(valueChanged(double)),this,SLOT(actGearsParams(double)));
    connect(ui->cb_gearsAlignTolerance,SIGNAL(stateChanged(int)),this,SLOT(actGearsParams(int)));
    ui->pb_startGearsCalc->setShortcut(Qt::Key_Return);

    connect(&reduc,SIGNAL(update_PB_gearsCalcRange(int,int)),this,SLOT(update_PB_gearsCalcRange(int,int)));
    connect(&reduc,SIGNAL(update_PB_gearsCalcValue(int)),ui->PB_gearsCalcProgression,SLOT(setValue(int)));

    connect(&reduc,SIGNAL(actGearsResult(int,int,int,int,double,double,double,double,double,double,double,double,double,double,double)),
            this,SLOT(actGearsResult(int,int,int,int,double,double,double,double,double,double,double,double,double,double,double)));
    connect(&reduc,SIGNAL(gearsCalcEnded()),this,SLOT(gearsCalcEnded()));

    //connect GEARS VERIFICATION
    connect(&reduc,SIGNAL(actVerifGearsExpectedValues(double,double)),this,SLOT(actVerifGearsExpectedValues(double,double)));
    connect(&reduc,SIGNAL(actVerifGearsGotValues(double,double,bool,bool)),this,SLOT(actVerifGearsGotValues(double,double,bool,bool)));
    connect(ui->sb_veGears_Z1,SIGNAL(valueChanged(int)),this,SLOT(updateVerifGearsInput(int)));
    connect(ui->sb_veGears_Z2,SIGNAL(valueChanged(int)),this,SLOT(updateVerifGearsInput(int)));
    connect(ui->sb_veGears_Z3,SIGNAL(valueChanged(int)),this,SLOT(updateVerifGearsInput(int)));
    connect(ui->sb_veGears_Z4,SIGNAL(valueChanged(int)),this,SLOT(updateVerifGearsInput(int)));

    actGearsParams(0);

    //load default config file
    QFile defaultFile("default"+FILE_TYPE);
    if(defaultFile.exists())
    {
        qDebug() << "Chargement du fichier par défaut ...";
        loadAllProject("default"+FILE_TYPE);
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::actPwrResult(int useless)
{
    useless++;
    reduc.calcPower(double(ui->sb_motorPwr->value()),double(ui->sb_motorSpeed->value()),ui->dsb_loadMass->value(),
                    ui->dsb_safetyCoef->value(),ui->dsb_loadHeight->value(),ui->dsb_cableDiam->value()
                    ,ui->dsb_spoolWidth->value(),ui->dsb_spoolDiam->value());

    ui->lbl_lSPwr->setText(QString::number(reduc.get_lSPwr(),'f',2));
    ui->lbl_lSInputFreq->setText(QString::number(reduc.get_lSInputFreq(),'f',2));
    ui->lbl_lSOutputFreq->setText(QString::number(reduc.get_lSOutputFreq(),'f',2));
    ui->lbl_lSReducRatio->setText(QString::number(reduc.get_lSReducRatio(),'f',2));
    ui->lbl_lSRatedTorque->setText(QString::number(reduc.get_lSRatedTorque(),'f',2));

    ui->lbl_mSPwr->setText(QString::number(reduc.get_mSPwr(),'f',2));
    ui->lbl_mSInputFreq->setText(QString::number(reduc.get_mSInputFreq(),'f',2));
    ui->lbl_mSOutputFreq->setText(QString::number(reduc.get_mSOutputFreq(),'f',2));
    ui->lbl_mSReducRatio->setText(QString::number(reduc.get_mSReducRatio(),'f',2));
    ui->lbl_mSRatedTorque->setText(QString::number(reduc.get_mSRatedTorque(),'f',2));

    ui->lbl_wPwr->setText(QString::number(reduc.get_wPwr(),'f',2));
    ui->lbl_wInputFreq->setText(QString::number(reduc.get_wInputFreq(),'f',2));
    ui->lbl_wOutputFreq->setText(QString::number(reduc.get_wOutputFreq(),'f',2));
    ui->lbl_wReducRatio->setText(QString::number(reduc.get_wReducRatio(),'f',2));
    ui->lbl_wRatedTorque->setText(QString::number(reduc.get_wRatedTorque(),'f',2));

    ui->lbl_totalReduc->setText(QString::number(reduc.get_totalReduc(),'f',5));
    ui->lbl_calcLoadMass->setText(QString::number(reduc.get_calcLoadMass(),'f',2));
    ui->lbl_loadWeight->setText(QString::number(reduc.get_loadWeight(),'f',2));
    ui->lbl_maxSpoolRad->setText(QString::number(reduc.get_maxSpoolRad(),'f',2));

    reduc.actVerifGearsInput(ui->sb_veGears_Z1->value(),ui->sb_veGears_Z2->value(),ui->sb_veGears_Z3->value(),ui->sb_veGears_Z4->value());
}

void MainWindow::actPwrResult(double useless)
{
    actPwrResult(int(useless));
}


void MainWindow::actGearsParams(int useless)
{
    useless++;

    reduc.actGearsInput(
                        ui->sb_minZ->value(),
                        ui->sb_maxZ->value(),
                        ui->dsb_minModule->value(),
                        ui->dsb_maxModule->value(),
                        ui->dsb_moduleStep->value(),
                        ui->sb_rpeMat->value(),
                        ui->dsb_gearsAlignTolerance->value(),
                        ui->dsb_motorAxisDiam->value(),
                        ui->dsb_lsDiam->value(),
                        ui->dsb_wAxisDiam->value(),
                        ui->cb_gearsAlignTolerance->isChecked()
                        );
}

void MainWindow::actGearsParams(double useless)
{
    actGearsParams(int(useless));
}

void MainWindow::on_pb_startGearsCalc_clicked()
{
    ui->pb_startGearsCalc->setDisabled(true);
    ui->sb_minZ->setDisabled(true);
    ui->sb_maxZ->setDisabled(true);
    ui->dsb_minModule->setDisabled(true);
    ui->dsb_maxModule->setDisabled(true);
    ui->dsb_moduleStep->setDisabled(true);
    ui->sb_rpeMat->setDisabled(true);
    ui->cb_gearsAlignTolerance->setDisabled(true);
    ui->dsb_gearsAlignTolerance->setDisabled(true);
    ui->dsb_motorAxisDiam->setDisabled(true);
    ui->dsb_lsDiam->setDisabled(true);
    ui->dsb_wAxisDiam->setDisabled(true);

    reduc.calcGears();

    reduc.actVerifGearsInput(ui->sb_veGears_Z1->value(),ui->sb_veGears_Z2->value(),ui->sb_veGears_Z3->value(),ui->sb_veGears_Z4->value());
}

void MainWindow::update_PB_gearsCalcRange(int min,int max)
{
    ui->PB_gearsCalcProgression->setRange(min,max);
}

void MainWindow::actGearsResult(int Z1, int Z2, int Z3, int Z4, double m1, double m2, double minM1, double minM2,
                    double b12, double b34, double calculatedReducRatio, double r1, double r2, double r3, double r4)
{
    ui->lbl_Z1->setText(QString::number(Z1,'f',2));
    ui->lbl_Z2->setText(QString::number(Z2,'f',2));
    ui->lbl_Z3->setText(QString::number(Z3,'f',2));
    ui->lbl_Z4->setText(QString::number(Z4,'f',2));

    ui->lbl_r1->setText(QString::number(r1,'f',2));
    ui->lbl_r2->setText(QString::number(r2,'f',2));
    ui->lbl_r3->setText(QString::number(r3,'f',2));
    ui->lbl_r4->setText(QString::number(r4,'f',2));

    ui->lbl_m1->setText(QString::number(m1,'f',2));
    ui->lbl_m2->setText(QString::number(m2,'f',2));
    ui->lbl_minModule1->setText(QString::number(minM1,'f',2));
    ui->lbl_minModule2->setText(QString::number(minM2,'f',2));

    ui->lbl_b12->setText(QString::number(b12,'f',2));
    ui->lbl_b34->setText(QString::number(b34,'f',2));

    ui->lbl_calculatedReducRatio->setText(QString::number(calculatedReducRatio,'f',6));
}

void MainWindow::gearsCalcEnded()
{
    ui->pb_startGearsCalc->setEnabled(true);
    ui->sb_minZ->setEnabled(true);
    ui->sb_maxZ->setEnabled(true);
    ui->dsb_minModule->setEnabled(true);
    ui->dsb_maxModule->setEnabled(true);
    ui->dsb_moduleStep->setEnabled(true);
    ui->sb_rpeMat->setEnabled(true);
    ui->cb_gearsAlignTolerance->setEnabled(true);
    ui->dsb_gearsAlignTolerance->setEnabled(true);
    ui->dsb_motorAxisDiam->setEnabled(true);
    ui->dsb_lsDiam->setEnabled(true);
    ui->dsb_wAxisDiam->setEnabled(true);
}

void MainWindow::actVerifGearsExpectedValues(double alignTolerance,double reducRatio)
{
    ui->lbl_veGears_expectedAlignTolerance->setText(QString::number(alignTolerance,'f',2));
    ui->lbl_veGears_expectedReducRatio->setText(QString::number(reducRatio,'f',5));
}

void MainWindow::actVerifGearsGotValues(double alignTolerance, double reducRatio, bool alignToleranceOK, bool reducRatioOk)
{
    ui->lbl_veGears_GotAlignTolerance->setText(QString::number(alignTolerance,'f',2));
    ui->lbl_veGears_gotReducRatio->setText(QString::number(reducRatio,'f',5));

    if(alignToleranceOK)
        ui->lbl_veGears_GotAlignTolerance->setStyleSheet("font-weight:bold;color:rgb(0,255,0)");
    else
        ui->lbl_veGears_GotAlignTolerance->setStyleSheet("font-weight:bold;color:rgb(255,0,0)");

    if(reducRatioOk)
        ui->lbl_veGears_gotReducRatio->setStyleSheet("font-weight:bold;color:rgb(0,255,0)");
    else
        ui->lbl_veGears_gotReducRatio->setStyleSheet("font-weight:bold;color:rgb(255,0,0)");
}

void MainWindow::updateVerifGearsInput(int useless)
{
    useless++;

    reduc.actVerifGearsInput(ui->sb_veGears_Z1->value(),ui->sb_veGears_Z2->value(),ui->sb_veGears_Z3->value(),ui->sb_veGears_Z4->value());
}

void MainWindow::on_actionSauvegarder_le_projet_triggered()
{
    //QString saveName = "sans nom"+FILE_TYPE;

    QString fileName = QFileDialog::getSaveFileName(this, "Enregistrer sous", QDir::homePath(), "Fichier de calcul d'engrenage (*"+FILE_TYPE+")");

    if(!fileName.isEmpty())
        saveAllProject(fileName);
}

void MainWindow::on_actionCharger_le_projet_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Ouvrir un fichier", QDir::homePath(), "Fichier de calcul d'engrenage (*"+FILE_TYPE+")");

    if(!fileName.isEmpty())
        loadAllProject(fileName);
}

void MainWindow::saveAllProject(QString fileName)
{
    saveProjectInput(fileName);
    reduc.saveProjectOutput(fileName);
}
void MainWindow::loadAllProject(QString fileName)
{
    loadProjectInput(fileName);
    reduc.loadProjectOutput(fileName);
}


void MainWindow::saveProjectInput(QString saveName)
{
    QFile file(saveName);

    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::critical(this,tr("Erreur"),tr("Impossible d'ouvrir le fichier\n")+saveName);
        return;
    }

    QTextStream flux(&file);

    flux <<"motorPwr="<<QString::number(ui->sb_motorPwr->value())<<";\n";
    flux <<"motorSpeed="<<QString::number(ui->sb_motorSpeed->value())<<";\n";
    flux <<"loadMass="<<QString::number(ui->dsb_loadMass->value())<<";\n";
    flux <<"safetyCoef="<<QString::number(ui->dsb_safetyCoef->value())<<";\n";
    flux <<"loadHeight="<<QString::number(ui->dsb_loadHeight->value())<<";\n";
    flux <<"cableDiam="<<QString::number(ui->dsb_cableDiam->value())<<";\n";
    flux <<"spoolWidth="<<QString::number(ui->dsb_spoolWidth->value())<<";\n";
    flux <<"spoolDiam="<<QString::number(ui->dsb_spoolDiam->value())<<";\n";

    flux <<"minZ="<<QString::number(ui->sb_minZ->value())<<";\n";
    flux <<"maxZ="<<QString::number(ui->sb_maxZ->value())<<";\n";
    flux <<"minModule="<<QString::number(ui->dsb_minModule->value())<<";\n";
    flux <<"maxModule="<<QString::number(ui->dsb_maxModule->value())<<";\n";
    flux <<"moduleStep="<<QString::number(ui->dsb_moduleStep->value())<<";\n";
    flux <<"rpeMat="<<QString::number(ui->sb_rpeMat->value())<<";\n";
    flux <<"considerGearsAlignTolerance="<<ui->cb_gearsAlignTolerance->isChecked()<<";\n";
    flux <<"gearsAlignTolerance="<<QString::number(ui->dsb_gearsAlignTolerance->value())<<";\n";
    flux <<"motorAxisDiam="<<QString::number(ui->dsb_motorAxisDiam->value())<<";\n";
    flux <<"lsDiam="<<QString::number(ui->dsb_lsDiam->value())<<";\n";
    flux <<"wAxisDiam="<<QString::number(ui->dsb_wAxisDiam->value())<<";\n";

    flux <<"veGears_Z1="<<QString::number(ui->sb_veGears_Z1->value())<<";\n";
    flux <<"veGears_Z2="<<QString::number(ui->sb_veGears_Z2->value())<<";\n";
    flux <<"veGears_Z3="<<QString::number(ui->sb_veGears_Z3->value())<<";\n";
    flux <<"veGears_Z4="<<QString::number(ui->sb_veGears_Z4->value())<<";\n";

    //flux <<"="<<->text()<<";\n";

    file.close();
}

void MainWindow::loadProjectInput(QString saveName)
{
    QFile file(saveName);

    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QMessageBox::critical(this,tr("Erreur"),tr("Impossible d'ouvrir le fichier\n")+saveName);
        return;
    }

    QString text;
    text = file.readAll();
    file.close();

    //ui->sb->setValue(getStrValueOf(text,"").toInt());
    //ui->dsb->setValue(getStrValueOf(text,"").toDouble());

    ui->sb_motorPwr->setValue(getStrValueOf(text,"motorPwr").toInt());
    ui->sb_motorSpeed->setValue(getStrValueOf(text,"motorSpeed").toInt());
    ui->dsb_loadMass->setValue(getStrValueOf(text,"loadMass").toDouble());
    ui->dsb_safetyCoef->setValue(getStrValueOf(text,"safetyCoef").toDouble());
    ui->dsb_loadHeight->setValue(getStrValueOf(text,"loadHeight").toDouble());
    ui->dsb_cableDiam->setValue(getStrValueOf(text,"cableDiam").toDouble());
    ui->dsb_spoolWidth->setValue(getStrValueOf(text,"spoolWidth").toDouble());
    ui->dsb_spoolDiam->setValue(getStrValueOf(text,"spoolDiam").toDouble());

    ui->sb_minZ->setValue(getStrValueOf(text,"minZ").toInt());
    ui->sb_maxZ->setValue(getStrValueOf(text,"maxZ").toInt());
    ui->dsb_minModule->setValue(getStrValueOf(text,"minModule").toDouble());
    ui->dsb_maxModule->setValue(getStrValueOf(text,"maxModule").toDouble());
    ui->dsb_moduleStep->setValue(getStrValueOf(text,"moduleStep").toDouble());
    ui->sb_rpeMat->setValue(getStrValueOf(text,"rpeMat").toInt());
    ui->cb_gearsAlignTolerance->setChecked(bool(getStrValueOf(text,"considerGearsAlignTolerance").toInt()));
    ui->dsb_gearsAlignTolerance->setValue(getStrValueOf(text,"gearsAlignTolerance").toDouble());
    ui->dsb_motorAxisDiam->setValue(getStrValueOf(text,"motorAxisDiam").toDouble());
    ui->dsb_lsDiam->setValue(getStrValueOf(text,"lsDiam").toDouble());
    ui->dsb_wAxisDiam->setValue(getStrValueOf(text,"wAxisDiam").toDouble());

    ui->sb_veGears_Z1->setValue(getStrValueOf(text,"veGears_Z1").toInt());
    ui->sb_veGears_Z2->setValue(getStrValueOf(text,"veGears_Z2").toInt());
    ui->sb_veGears_Z3->setValue(getStrValueOf(text,"veGears_Z3").toInt());
    ui->sb_veGears_Z4->setValue(getStrValueOf(text,"veGears_Z4").toInt());
}
