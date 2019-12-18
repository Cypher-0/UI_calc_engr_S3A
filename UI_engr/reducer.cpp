#include "reducer.h"

#include <QtMath>
#include <cmath>

#include <QDebug>
#include <QFile>
#include <QIODevice>
#include <QTextStream>
#include <QMessageBox>
#include <QCoreApplication>

//compiler la version avec modules mini corrigés
// 0 = non
// 1 = oui
#define COMPIL_CORRECT 0

Reducer::Reducer(QObject *parent) : QObject(parent)
{
    //gears

    klist.push_back(0.010);
    klist.push_back(0.006);
}

/////////////////////////////////////////////////////////////////////////
///
///                         ACCESSEURS
///
/////////////////////////////////////////////////////////////////////////


        //////////////////////////////////
        ///         GEARS
        //////////////////////////////////

//input parameters

double Reducer::get_loadHeight()
{
    return loadHeight;
}
double Reducer::get_safetyCoef()
{
    return safetyCoef;
}
double Reducer::get_loadMass()
{
    return loadMass;
}
double Reducer::get_motorPwr()
{
    return motorPwr;
}
double Reducer::get_motorSpeed()
{
    return motorSpeed;
}
double Reducer::get_cableDiam()
{
    return cableDiam;
}
double Reducer::get_spoolDiam()
{
    return spoolDiam;
}
double Reducer::get_spoolWidth()
{
    return spoolWidth;
}


//Results

    //lower shaft
double Reducer::get_lSInputFreq()
{
    return lSInputFreq;
}
double Reducer::get_lSOutputFreq()
{
    return lSOutputFreq;
}
double Reducer::get_lSPwr()
{
    return lSPwr;
}
double Reducer::get_lSRatedTorque()
{
    return lSRatedTorque;
}
double Reducer::get_lSReducRatio()
{
    return lSReducRatio;
}

    //motor shaft
double Reducer::get_mSInputFreq()
{
    return mSInputFreq;
}
double Reducer::get_mSOutputFreq()
{
    return mSOutputFreq;
}
double Reducer::get_mSPwr()
{
    return mSPwr;
}
double Reducer::get_mSRatedTorque()
{
    return mSRatedTorque;
}
double Reducer::get_mSReducRatio()
{
    return mSReducRatio;
}

    //winch
double Reducer::get_wInputFreq()
{
    return wInputFreq;
}
double Reducer::get_wOutputFreq()
{
    return wOutputFreq;
}
double Reducer::get_wPwr()
{
    return wPwr;
}
double Reducer::get_wRatedTorque()
{
    return wRatedTorque;
}
double Reducer::get_wReducRatio()
{
    return wReducRatio;
}

    //misc
double Reducer::get_calcLoadMass()
{
    return calcLoadMass;
}
double Reducer::get_loadWeight()
{
    return loadWeight;
}
double Reducer::get_maxSpoolRad()
{
    return maxSpoolRad;
}


        //////////////////////////////////
        ///         GLOBAL
        //////////////////////////////////

double Reducer::get_totalReduc()
{
    return totalReduc;
}

/////////////////////////////////////////////////////////////////////////
///
///                         Calc
///
/////////////////////////////////////////////////////////////////////////

        //////////////////////////////////
        ///         POWER
        //////////////////////////////////

double Reducer::calcMaxSpoolRad()
{
    double lapNumPerStage = spoolWidth/cableDiam;
    double currentDiam = spoolDiam+cableDiam;
    double currentCableWounded = currentDiam*M_PI*lapNumPerStage;

    qDebug() << "START" << endl << currentCableWounded;

    while(currentCableWounded < loadHeight*1000.0)
    {
        currentDiam += cableDiam*2;
        currentCableWounded += currentDiam*M_PI*lapNumPerStage;
        qDebug() << currentCableWounded;
    }

    maxSpoolRad = currentDiam/2.0;
    qDebug() << "Diam max tambour : " << maxSpoolRad*2.0;
    return maxSpoolRad;
}

void Reducer::actPowerInput(const double &ImotorPwr,const double &ImotorSpeed,const double &IloadMass,const double &IsafetyCoef,
                        const double &IloadHeight,const double &IcableDiam,const double &IspoolWidth,const double &IspoolDiam)
{
    motorPwr = ImotorPwr;
    motorSpeed = ImotorSpeed;
    loadMass = IloadMass;
    safetyCoef = IsafetyCoef;
    loadHeight = IloadHeight;
    cableDiam = IcableDiam;
    spoolWidth = IspoolWidth;
    spoolDiam = IspoolDiam;

    calcPower();
}

void Reducer::calcPower()
{
    calcLoadMass = loadMass*safetyCoef;
    loadWeight = calcLoadMass*9.81;

    wPwr = gearsEfficiency*gearsEfficiency*motorPwr;
    wRatedTorque = loadWeight*calcMaxSpoolRad()*0.001;
    wOutputFreq = wPwr/wRatedTorque;
    mSOutputFreq = motorSpeed*2.0*M_PI/60.0;

    totalReduc = wOutputFreq/mSOutputFreq;

    mSPwr = motorPwr;
    mSInputFreq = mSOutputFreq;
    mSRatedTorque = motorPwr/mSOutputFreq;
    mSReducRatio = mSOutputFreq/mSInputFreq;

    lSPwr = motorPwr*gearsEfficiency;
    lSReducRatio =(veGears_Z2 != 0.0)?veGears_Z1/veGears_Z2:qSqrt(totalReduc);
    lSInputFreq = mSOutputFreq;
    lSOutputFreq = lSReducRatio*lSInputFreq;
    lSRatedTorque = lSPwr/lSOutputFreq;

    wInputFreq = lSOutputFreq;
    wReducRatio = wOutputFreq/wInputFreq;

    emit actVerifGearsExpectedValues(gearsAlignTolerance,totalReduc);
    emit actPowerResult();
}

        //////////////////////////////////
        ///         GEARS
        //////////////////////////////////

void Reducer::actGearsInput(const int &IZmin, const int &IZmax, const double &ImMin, const double &ImMax,
                            const double &ImStep, const int &Irpe, const double &IgearsAlignTolerance,
                            const double &ImotorAxisDiam, const double &IlsDiam, const double &IwAxisDiam,
                            bool IconsiderAlignTolerance)
{
    Zmin = IZmin;
    Zmax = IZmax;
    mMin = ImMin;
    mMax = ImMax;
    mStep = ImStep;
    rpe = Irpe;
    gearsAlignTolerance = IgearsAlignTolerance;
    motorAxisDiam = ImotorAxisDiam;
    lsDiam = IlsDiam;
    wAxisDiam = IwAxisDiam;
    considerAlignTolerance = IconsiderAlignTolerance;

    emit actVerifGearsExpectedValues(gearsAlignTolerance,totalReduc);
}

void Reducer::calcGears()
{
    //init des meilleures valeurs
    bestZ1=Zmax+10;
    bestZ2=Zmax+10;
    bestZ3=Zmax+10;
    bestZ4=Zmax+10;
    bestR1=1000.0;
    bestR2=1000.0;
    bestR3=1000.0;
    bestR4=1000.0;
    bestk1=0.0;
    bestk2=0.0;
    bestm1=0.0;
    bestm2=0.0;


    emit update_PB_gearsCalcRange(Zmin,Zmax-1);
    emit actGearsResult(0,0,0,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);

    quint64 nbreIter = 0;

    int tmpMaxMod = int(mMax*10);

    for(int tmpZ4 = Zmin; tmpZ4 < Zmax;tmpZ4 += 1)
        {
        emit update_PB_gearsCalcValue(tmpZ4);
        QCoreApplication::processEvents();
            for(int tmpZ2 = Zmin; tmpZ2 < Zmax;tmpZ2 += 1)
            {
                for(int tmpm1 = int(mMin*10); tmpm1 <= tmpMaxMod;tmpm1 += 5)
                {
                    for(int tmpm2 = int(mMin*10); tmpm2 <= tmpMaxMod;tmpm2 += 5)
                    {
                        for(auto &k1 : klist)
                        {
                            for(auto &k2 : klist)
                            {
                                nbreIter++;

                                Z2 = double(tmpZ2);
                                Z4 = double(tmpZ4);

                                m1 = double(tmpm1)/10.0;
                                m2 = double(tmpm2)/10.0;

                                Z3 = ((-Z4*m2)+(Z2*m1)+sqrt(pow((Z4*m2)-(Z2*m1),2)+(4*m2*m1*totalReduc*Z2*Z4)))/(2*m2);

                                Z1 = (totalReduc*Z2*Z4)/(Z3);

                                adjustValues(Z1,Z2,Z3,Z4);

                                r1 = m1*Z1/2.0;
                                r2 = m1*Z2/2.0;
                                r3 = m2*Z3/2.0;
                                r4 = m2*Z4/2.0;

                                //conditions nécessaires aux
                                if(
                                    Z1 >= Zmin && Z3 >= Zmin
                                    && (Z1*Z3)/(Z2*Z4) <= totalReduc
                                    && bestR1+bestR2+bestR3+bestR4 >= r1+r2+r3+r4
                                    && r1+r2 >= r3+r4-(gearsAlignTolerance*int(considerAlignTolerance)) && r1+r2 <= r3+r4+(gearsAlignTolerance*int(considerAlignTolerance))
                                    && m1 >= cbrt((5.48*2*mSRatedTorque)/(Z1*k1*rpe)) &&
                                        #if COMPIL_CORRECT == 0
                                        m2 >= cbrt((5.48*2*wRatedTorque)/(Z3*k2*rpe))
                                        #else
                                        m2 >= cbrt((5.48*2*wRatedTorque)/(Z4*k2*rpe))
                                        #endif
                                    && r1*2.0*cos(alpha)-(motorAxisDiam/2.0)-getWedgeHeight(motorAxisDiam) >= wedgeSizeSafety && r2*2.0*cos(alpha)-(lsDiam/2.0)-getWedgeHeight(lsDiam) >= wedgeSizeSafety
                                        && r3*2.0*cos(alpha)-(lsDiam/2.0)-getWedgeHeight(lsDiam) >= wedgeSizeSafety && r4*2.0*cos(alpha)-(wAxisDiam/2.0)-getWedgeHeight(wAxisDiam) >= wedgeSizeSafety
                                )
                                {
                                    bestZ1 = int(Z1);
                                    bestZ2 = int(Z2);
                                    bestZ3 = int(Z3);
                                    bestZ4 = int(Z4);

                                    bestm1 = m1;
                                    bestm2 = m2;

                                    bestk1 = k1;
                                    //qDebug() << "k1 : " << k1 << "  k2 :" << k2;
                                    bestk2 = k2;

                                    bestR1 = r1;
                                    bestR2 = r2;
                                    bestR3 = r3;
                                    bestR4 = r4;

                                    calculatedReducRatio = (Z1*Z3)/(Z2*Z4);

                                    minModule1 = cbrt((5.48*2*mSRatedTorque)/(Z1*k1*rpe));
                                    minModule2 = cbrt((5.48*2*wRatedTorque)/(Z4*k2*rpe));

                                    //qDebug() << cbrt((5.4756*2*lSPwr)/(k1*rpe*double(bestZ1)*mSOutputFreq)) << "   |   " << cbrt((5.48*2*mSRatedTorque)/(Z1*k1*rpe));
                                    //qDebug() << cbrt((5.4756*2*lSPwr*double(bestZ2))/(k2*rpe*double(bestZ3)*mSOutputFreq*double(bestZ1))) << "   |   " << cbrt((5.48*2*wRatedTorque)/(Z4*k2*rpe));


                                    emit actGearsResult(bestZ1,bestZ2,bestZ3,bestZ4,bestm1,bestm2,minModule1,minModule2,bestk1*bestm1*1000,bestk2*bestm2*1000,calculatedReducRatio,bestR1,bestR2,bestR3,bestR4);
                                }
                            }
                        }
                    }
                }
            }
        }
    if((bestZ1 == Zmax-1 && bestZ3 == Zmax-1)
       || (bestZ1*bestZ3)/(bestZ2*bestZ4) > totalReduc
      )
    {
        emit actGearsResult(0,0,0,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
    }

    qDebug() << nbreIter;

    emit gearsCalcEnded();
}

double getWedgeHeight(const double &axisDiam)
{
    //basé sur https://legrand-genie.blogspot.com/2017/02/dimensionnement-dune-clavette.html
    //retourne b-j+d

    if(axisDiam >= 6.0 && axisDiam <= 8.0)
        return 0.8;
    else if(axisDiam > 8.0 && axisDiam < 10.0)
        return 1.2;
    else if(axisDiam >= 10.0 && axisDiam < 12.0)
        return 1.5;
    else if(axisDiam >= 12.0 && axisDiam < 17.0)
        return 2.0;
    else if(axisDiam >= 17.0 && axisDiam < 22.0)
        return 2.5;
    else if(axisDiam >= 22.0 && axisDiam < 30.0)
        return 3.0;
    else if(axisDiam >= 30.0 && axisDiam < 38.0)
        return 3.0;
    else if(axisDiam >= 38.0 && axisDiam < 44.0)
        return 3.0;
    else if(axisDiam >= 44.0 && axisDiam < 50.0)
        return 3.5;
    else if(axisDiam >= 50.0 && axisDiam < 58.0)
        return 4.0;
    else if(axisDiam >= 58.0 && axisDiam < 65.0)
        return 4.0;

    return -1.0;
}



void Reducer::actVerifGearsInput(const int &IZ1,const int &IZ2,const int &IZ3,const int &IZ4)
{
    veGears_Z1=double(IZ1);
    veGears_Z2=double(IZ2);
    veGears_Z3=double(IZ3);
    veGears_Z4=double(IZ4);

    double reducRatio = (veGears_Z1*veGears_Z3)/(veGears_Z2*veGears_Z4);

    double r1 = bestm1*IZ1/2.0;
    double r2 = bestm1*IZ2/2.0;
    double r3 = bestm2*IZ3/2.0;
    double r4 = bestm2*IZ4/2.0;

    emit actVerifGearsGotValues(r3+r4-r1-r2,reducRatio,std::abs(r3+r4-r1-r2) <= gearsAlignTolerance,reducRatio <= totalReduc);
    calcPower();
}

        //////////////////////////////////
        ///         BEARINGS
        //////////////////////////////////

void Reducer::actBearingsInput(const double &IdynLoadB1, const double &IdynLoadB2, const double &IdynLoadB3,
                               const double &IdynLoadB4, const double &j, const double &i, const double &h,
                               const double &q, const double &f, const double &e)
{
    dynLoadB = IdynLoadB1;
    dynLoadB1 = IdynLoadB2;
    dynLoadB2 = IdynLoadB3;
    dynLoadB3 = IdynLoadB4;

    bear_j = j;
    bear_i = i;
    bear_h = h;

    bear_q = q;
    bear_f = f;
    bear_e = e;

    calcBearingsLifeTime();
}

void Reducer::calcBearingsLifeTime()
{
    //constants
    const double tanAlpha = tan(alpha*M_PI/180.0);
    const double lifeTimeConstant = (2.0*M_PI*double(pow(10.0,6.0)))/(31536000);
    const double za = lSRatedTorque/(bestR3*0.001);
    const double zb = lSRatedTorque/(bestR2*0.001);
    qDebug() << "Za et Zb :" << za << endl << zb;
    //temp var
    double L=0.0;
    //output
    double lifeTime_b=0.0,lifeTime_b1=0.0,lifeTime_b2=0.0,lifeTime_b3=0.0;

    //bearing 0
    double yr = ((bear_j+bear_i)*loadWeight-(bear_h*za*tanAlpha))/bear_i;//loadWeight = m*g*ks
    double zr = (bear_h*za)/bear_i;
    L = pow(dynLoadB*1000/sqrt((yr*yr)+(zr*zr)),3);
    lifeTime_b = lifeTimeConstant*L/wOutputFreq;

    //bearing 1
    double yr1 = loadWeight-yr+za*tanAlpha;//loadWeight = m*g*ks
    double zr1 = zr-za;
    L = pow(dynLoadB1*1000/sqrt((yr1*yr1)+(zr1*zr1)),3);
    lifeTime_b1 = lifeTimeConstant*L/wOutputFreq;

    //bearing 2
    double yr2 = tanAlpha*((bear_e*zb)-(za*(bear_f+bear_e)));//loadWeight = m*g*ks
    double zr2 = ((-za*(bear_f+bear_e))-(zb*bear_e))/(bear_q+bear_f+bear_e);
    L = pow(dynLoadB2*1000/sqrt((yr2*yr2)+(zr2*zr2)),3);
    lifeTime_b2 = lifeTimeConstant*L/lSOutputFreq;

    //bearing 3
    double yr3 = yr2+tanAlpha*(zb+za);//loadWeight = m*g*ks
    double zr3 = -za-zr2-zb;
    L = pow(dynLoadB3*1000/sqrt((yr3*yr3)+(zr3*zr3)),3);
    lifeTime_b3 = lifeTimeConstant*L/lSOutputFreq;

    qDebug() << "Fréquences : "<<wOutputFreq << "   |   " << lSOutputFreq;
    qDebug() << "durées de vie : " << lifeTime_b << " | "<< lifeTime_b1 << " | "<< lifeTime_b2 << " | "<< lifeTime_b3;



    emit actBearingsOutput(lifeTime_b,lifeTime_b1,lifeTime_b2,lifeTime_b3);
}


/////////////////////////////////////////////////////////////////////////
///
///                         GLOBAL
///
/////////////////////////////////////////////////////////////////////////


void Reducer::saveProjectOutput(QString saveName)
{
    QFile file(saveName);

    if(!file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
    {
        QMessageBox::critical(nullptr,tr("Erreur"),tr("Impossible d'ouvrir le fichier\n")+saveName);
        return;
    }

    QTextStream flux(&file);

    flux <<"Z1="<<QString::number(bestZ1)<<";\n";
    flux <<"r1="<<QString::number(bestR1)<<";\n";
    flux <<"Z2="<<QString::number(bestZ2)<<";\n";
    flux <<"r2="<<QString::number(bestR2)<<";\n";
    flux <<"Z3="<<QString::number(bestZ3)<<";\n";
    flux <<"r3="<<QString::number(bestR3)<<";\n";
    flux <<"Z4="<<QString::number(bestZ4)<<";\n";
    flux <<"r4="<<QString::number(bestR4)<<";\n";
    flux <<"m1="<<QString::number(bestm1)<<";\n";
    flux <<"m2="<<QString::number(bestm2)<<";\n";
    flux <<"minM1="<<QString::number(minModule1)<<";\n";
    flux <<"minM2="<<QString::number(minModule2)<<";\n";
    flux <<"k1="<<QString::number(bestk1)<<";\n";
    flux <<"k2="<<QString::number(bestk2)<<";\n";
    flux <<"b12="<<QString::number(bestk1*bestm1*1000)<<";\n";
    flux <<"b34="<<QString::number(bestk2*bestm2*1000)<<";\n";
    flux <<"calculatedReducRatio="<<QString::number(calculatedReducRatio)<<";\n";
    //bearings
    flux <<"dynLoadB1="<<QString::number(dynLoadB)<<";\n";
    flux <<"dynLoadB2="<<QString::number(dynLoadB1)<<";\n";
    flux <<"dynLoadB3="<<QString::number(dynLoadB2)<<";\n";
    flux <<"dynLoadB4="<<QString::number(dynLoadB3)<<";\n";
    //flux <<"="<<QString::number()<<";\n";

    file.close();
}

void Reducer::loadProjectOutput(QString saveName)
{
    QFile file(saveName);

    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QMessageBox::critical(nullptr,tr("Erreur"),tr("Impossible d'ouvrir le fichier #4\n")+saveName);
        return;
    }

    QString text;
    text = file.readAll();
    file.close();

    bestZ1 = getStrValueOf(text,"Z1").toInt();
    bestZ2 = getStrValueOf(text,"Z2").toInt();
    bestZ3 = getStrValueOf(text,"Z3").toInt();
    bestZ4 = getStrValueOf(text,"Z4").toInt();

    bestR1 = getStrValueOf(text,"r1").toDouble();
    bestR2 = getStrValueOf(text,"r2").toDouble();
    bestR3 = getStrValueOf(text,"r3").toDouble();
    bestR4 = getStrValueOf(text,"r4").toDouble();

    bestm1 = getStrValueOf(text,"m1").toDouble();
    bestm2 = getStrValueOf(text,"m2").toDouble();
    minModule1 = getStrValueOf(text,"minM1").toDouble();
    minModule2 = getStrValueOf(text,"minM2").toDouble();

    bestk1 = getStrValueOf(text,"k1").toDouble();
    bestk2 = getStrValueOf(text,"k2").toDouble();

    calculatedReducRatio = getStrValueOf(text,"calculatedReducRatio").toDouble();


    double b12 = getStrValueOf(text,"b12").toDouble();
    double b34 = getStrValueOf(text,"b34").toDouble();

    emit actGearsResult(bestZ1,bestZ2,bestZ3,bestZ4,bestm1,bestm2,minModule1,minModule2,b12,b34,calculatedReducRatio,bestR1,bestR2,bestR3,bestR4);
    actVerifGearsInput(int(veGears_Z1),int(veGears_Z2),int(veGears_Z3),int(veGears_Z4));

    calcPower();
    emit actPowerResult();
}

QString getStrValueOf(const QString &input,const QString &param)
{
    int start = input.indexOf(param+"=")+param.length()+1;
    int length = input.mid(start).indexOf(";");

    QString result = input.mid(start,length);

    return (result.length() > 0)?result:"0";
}
