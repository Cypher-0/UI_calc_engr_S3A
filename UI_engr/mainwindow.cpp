#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "reducer.h"
#include <QString>

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

    ui->lbl_totalReduc->setText(QString::number(reduc.get_totalReduc(),'f',2));
    ui->lbl_calcLoadMass->setText(QString::number(reduc.get_calcLoadMass(),'f',2));
    ui->lbl_loadWeight->setText(QString::number(reduc.get_loadWeight(),'f',2));
    ui->lbl_maxSpoolRad->setText(QString::number(reduc.get_maxSpoolRad(),'f',2));
}

void MainWindow::actPwrResult(double useless)
{
    actPwrResult(int(useless));
}
