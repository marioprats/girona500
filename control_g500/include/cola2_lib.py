#!/usr/bin/env python

import pylab
import math
from numpy import *

def saturateVector(v, min_max) :
    ret = zeros( len(v) )
    for i in range( len(v) ) :
        if v[i] < -min_max[i] : ret[i] = -min_max[i]
        elif v[i] > min_max[i] : ret[i] = min_max[i]
        else : ret[i] = v[i]
    return ret


def saturateValue(v, min_max) :
    ret = zeros( len(v) )
    for i in range( len(v) ) :
        if v[i] < -min_max : ret[i] = -min_max
        elif v[i] > min_max : ret[i] = min_max
        else : ret[i] = v[i]
    return ret


def computePid6Dof(desired, current, kp, ki, kd, sat, ek_1, eik_1, T):
    ek = zeros(6)
    eik = zeros(6)
    edotk = zeros(6)
    
    # All the operations are done element by element
    ek = desired - current
    edotk = (ek - ek_1) / T
    eik = eik_1 + (ek * T)
    eik = SaturateVector(eik, sat)
    #print "ek: \n" + str(ek) 
    #print "eik: \n" + str(eik)
    #print "edotk: \n" + str(edotk)
    #print "ek_1: \n" + str(ek_1) 
    #print "eik_1: \n" + str(eik_1)

    # Control law
    tau = zeros(6)
    tau = kp * ek + ki * eik + kd * edotk
    tau = SaturateValue(tau, 1.0) 
        
    return [tau, ek, eik]


def normalizeAngle(angle) :
    return (angle + ( 2.0 * math.pi * math.floor( ( math.pi - angle ) / ( 2.0 * math.pi ) ) ) )


def test() :
    print "SaturateVector:"
    print str(SaturateVector([1.8,0.3,-3.2, -0.7], ones(4)))
    
    print "SaturateValue:"
    print str(SaturateValue([1.8,0.3,-3.2, -0.7], 1.0))    

    print "Normalize angle 7.14 = " + str(NormalizeAngle(7.14))

if __name__ == '__main__':
    test()


