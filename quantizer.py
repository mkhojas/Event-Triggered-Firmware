import math
import numpy as np
# Encoder and decoder methods:
def encoder(x):
    z       = x[0]
    ts      = x[1]
    gamma   = x[2]
    b       = x[3]
    A       = x[4]
    ro      = x[5]
    M       = x[6]
    J       = x[7]
    if (A * b * gamma) / math.log(1 + (ro - M * (math.exp(A * gamma) - 1) / (J * A)) / math.exp(A * gamma))<=0:
        g = 1
        q = np.zeros(g)
    elif ts>=0:
        L = 1 + math.log2((A * b * gamma) / math.log(1 + (ro - M * (math.exp(A * gamma) - 1) / (J * A)) / math.exp(A * gamma)))
        g = math.ceil(max(0.1,L))
        #g=1
        q = np.zeros(g)
    else:
        g = 2
        q = -1*np.ones(g)
    if len(q)==1:
        if z>0:
            q[0] = 0
        else:
            q[0] = 1
    if len(q)>1:
        if z>0:
            q[0] = 0
        else:
            q[0] = 1
        q[1] = math.fmod(math.floor(ts/(b*gamma)),2)
        i = 0
        while True:
            ts_interval = [i*b*gamma, (i+1)*b*gamma]
            if ts>=ts_interval[0] and ts<ts_interval[1]:
                break
            i = i + 1
        if len(q)>2:
            for i in range(1,g-1,1):
                ts_interval = [ ts_interval[0], (ts_interval[0]+ts_interval[1])/2 , ts_interval[1]]
                if ts>=ts_interval[0] and ts<=ts_interval[1]:
                    q[i+1] = 0
                    ts_interval = [ts_interval[0],ts_interval[1]]
                else:
                    q[i+1] = 1
                    ts_interval = [ts_interval[1], ts_interval[2]]
    return q
# This is the method for decoding:
def decoder(x):
    q       = x[0]
    tc      = x[1]
    gamma   = x[2]
    b       = x[3]
    A       = x[4]
    J       = x[5]
    if q[0]<0:
        zbar = 0
    elif len(q)==1:
        if q[0]==0:
            zbar = J
        else:
            zbar = -J
    elif len(q)>1:
        i = 0
        while True:
            dummy_interval1 = [i*b*gamma, (i+1)*b*gamma]
            if (tc-gamma)<0:
                tc_lowerbound_index = i
                break
            elif (tc-gamma)>=dummy_interval1[0] and (tc-gamma)<dummy_interval1[1]:
                tc_lowerbound_index = i
                break
            i = i + 1
        j = 0
        while True:
            dummy_interval2 = [j*b*gamma,(j+1)*b*gamma]
            if tc>=dummy_interval2[0] and tc<dummy_interval2[1]:
                tc_upperbound_index = j
                break
            j = j + 1
        if tc_lowerbound_index == tc_upperbound_index:
            ts_interval = [tc_lowerbound_index*b*gamma, (tc_lowerbound_index+1)*b*gamma]
        elif math.fmod(tc_lowerbound_index,2) == q[1]:
            ts_interval = [tc_lowerbound_index*b*gamma, (tc_lowerbound_index+1)*b*gamma]
        else:
            ts_interval = [tc_upperbound_index*b*gamma, (tc_upperbound_index+1)*b*gamma]
        loop_length = len(q) - 2
        ts_interval = [ts_interval[0], (ts_interval[0] + ts_interval[1]) / 2, ts_interval[1]]
        if loop_length>0:
            for ii in range(1,loop_length+1,1):
                if q[ii+1]==0:
                    ts_interval = [ts_interval[0], ts_interval[1]]
                else:
                    ts_interval = [ts_interval[1], ts_interval[2]]
                ts_interval = [ts_interval[0], (ts_interval[0] + ts_interval[1]) / 2, ts_interval[1]]
                if ii==loop_length:
                    ts = ts_interval[1]
        else:
            ts = ts_interval[1]
        if q[0]==0:
            zbar = J*math.exp(A*(tc-ts))
        else:
            zbar = -J*math.exp(A*(tc-ts))
    return zbar
