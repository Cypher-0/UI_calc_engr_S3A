#include "reducer.h"

#include <QtMath>
#include <math.h>

#include <QDebug>

Reducer::Reducer(QObject *parent) : QObject(parent)
{

}

void Reducer::calcPower(const double &ImotorPwr,const double &ImotorSpeed,const double &IloadMass,const double &IsafetyCoef,
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

    calcLoadMass = loadMass*safetyCoef;
    loadWeight = calcLoadMass*9.81;

    wPwr = gearsEfficiency*gearsEfficiency*motorPwr;
    wRatedTorque = loadWeight*calcMaxSpoolRad()*0.001;
    wOutputFreq = wPwr/wRatedTorque;
    mSOutputFreq = motorSpeed*2.0*M_PI/60.0;

    totalReduc = wOutputFreq/mSOutputFreq;
    qDebug() << calcMaxSpoolRad() << endl << wRatedTorque;

    mSPwr = motorPwr;
    mSInputFreq = mSOutputFreq;
    mSRatedTorque = motorPwr/mSOutputFreq;
    mSReducRatio = mSOutputFreq/mSInputFreq;

    lSPwr = motorPwr*gearsEfficiency;
    lSReducRatio = qSqrt(totalReduc);
    lSInputFreq = mSOutputFreq;
    lSOutputFreq = lSReducRatio*lSInputFreq;
    lSRatedTorque = mSPwr/lSOutputFreq;

    wInputFreq = lSOutputFreq;
    wReducRatio = wOutputFreq/wInputFreq;
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

    while(currentCableWounded < loadHeight*1000.0)
    {
        currentDiam += cableDiam;
        currentCableWounded += currentDiam*M_PI*lapNumPerStage;
    }

    maxSpoolRad = currentDiam/2.0;
    return maxSpoolRad;
}

        //////////////////////////////////
        ///         GEARS
        //////////////////////////////////

void Reducer::calcGears()
{

}

bool Reducer::isComboOk()
{
    QVector<bool> conditions(0);
    conditions.push_back(Z1 > Zmin && Z3 >= Zmin);
    conditions.push_back(bestR1+bestR2 > r1+r2);
    //if(m1 >= cbrt((5.48*2*C1)/(Z1*k1*rpe)) && m2 >= cbrt((5.48*2*C2)/(Z3*k2*rpe)))
    conditions.push_back(m1 >= cbrt((5.48*2*lSRatedTorque)/(Z1*k1*rpe)) && m2 >= cbrt((5.48*2*wRatedTorque)/(Z3*k2*rpe)));

    for(auto &curVal : conditions)
    {
        if(!curVal)
            return false;
    }

    return true;
}
