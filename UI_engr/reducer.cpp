#include "reducer.h"

#include <QtMath>
#include <math.h>

#include <QDebug>

#define DELTA pow((Z4*m2)-(Z2*m1),2)+(4*m2*m1*totalReduc*Z2*Z4)

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
    //qDebug() << calcMaxSpoolRad() << endl << wRatedTorque;

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

void Reducer::actGearsInput(const int &IZmin, const int &IZmax, const double &ImMin, const double &ImMax, const double &ImStep, const int &Irpe, const double &IgearsAlignTolerance)
{
    Zmin = IZmin;
    Zmax = IZmax;
    mMin = ImMin;
    mMax = ImMax;
    mStep = ImStep;
    rpe = Irpe;
    gearsAlignTolerance = IgearsAlignTolerance;
}

void Reducer::calcGears()
{
    emit update_PB_gearsCalcRange(Zmin,Zmax-1);
    for(int tmpZ4 = Zmin; tmpZ4 < Zmax;tmpZ4 += 1)
        {
        emit update_PB_gearsCalcValue(tmpZ4);
            for(int tmpZ2 = Zmin; tmpZ2 < Zmax;tmpZ2 += 1)
            {
                for(int tmpm1 = int(mMin*10); tmpm1 < int(mMax*10);tmpm1 += 5)
                {
                    for(int tmpm2 = int(mMin*10); tmpm2 < int(mMax*10);tmpm2 += 5)
                    {
                        for(auto &k1 : klist)
                        {
                            for(auto &k2 : klist)
                            {
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


                                if(isComboOk(k1,k2))
                                {
                                    bestZ1 = int(Z1);
                                    bestZ2 = int(Z2);
                                    bestZ3 = int(Z3);
                                    bestZ4 = int(Z4);

                                    bestm1 = m1;
                                    bestm2 = m2;

                                    bestk1 = k1;
                                    bestk2 = k2;

                                    bestR1 = r1;
                                    bestR2 = r2;
                                    bestR3 = r3;
                                    bestR4 = r4;

                                    calculatedReducRatio = (Z1*Z3)/(Z2*Z4);
                                }
                            }
                        }
                    }
                }
            }
        }
    qDebug() << bestZ1 << endl<< bestZ2 << endl<< bestZ3 << endl<< bestZ4 << endl << m1 << endl << m2 << endl << calculatedReducRatio <<endl;
}

bool Reducer::isComboOk(const double &k1, const double &k2)
{
    QVector<bool> conditions(0);
    conditions.push_back(Z1 > Zmin && Z3 >= Zmin);
    conditions.push_back(r1+r2 >= r3+r4-gearsAlignTolerance && r1+r2 <= r3+r4+gearsAlignTolerance);
    conditions.push_back(m1 >= cbrt((5.48*2*lSRatedTorque)/(Z1*k1*rpe)) && m2 >= cbrt((5.48*2*wRatedTorque)/(Z3*k2*rpe)));
    conditions.push_back(bestR1+bestR2 > r1+r2);

    if(
            m1 >= cbrt((5.48*2*lSRatedTorque)/(Z1*k1*rpe)) && m2 >= cbrt((5.48*2*wRatedTorque)/(Z3*k2*rpe))
            && Z1 > Zmin && Z3 >= Zmin
            && r1+r2 >= r3+r4-gearsAlignTolerance && r1+r2 <= r3+r4+gearsAlignTolerance
            && bestR1+bestR2 > r1+r2
      )
      {
        qDebug() << "OK";
            return true;
      }

    /*for(auto &curVal : conditions)
    {
        if(!curVal)
            return false;
    }*/

    return true;
}
