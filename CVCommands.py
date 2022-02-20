import numpy as np
import time
import pandas as pd
import sys
import random
from os import path
from simple_pid import PID
from tqdm.auto import tqdm
from datetime import datetime

def SMU_configure():
    SMU.write('*RST')
    SMU.write(':SYST:BEEP 1e6, .5')
    SMU.write(':ROUT:TERM REAR')

def LCR_configure():
    LCR.write(':TRIG EXT')
    LCR.write(':TRIG:DEL '+str(trig_delay))
    LCR.write(':AVER '+str(num_avg))
    LCR.write(':SPEE '+meas_speed)
    LCR.write(':RANG:LOWZ '+lowz)
    
    LCR.timeout = LCR_timeout
    
def LCR_CGD():
    LCR.write('PAR1 CP')
    LCR.write('PAR2 G')
    LCR.write('PAR3 D')
    LCR.write('PAR4 OFF')

def LCR_ZTD():
    LCR.write('PAR1 Z')
    LCR.write('PAR2 PHASE')
    LCR.write('PAR3 D')
    LCR.write('PAR4 OFF')    

def voltFunc(x):
    
    v = np.interp(x,fit_df['Vactual'],fit_df['Vsmu'])

    return v

def voltFit(Vmin,Vmax,fittype):
    
    print('Creating voltage fit for: '+sampName+'...')
    
    Vset = []
    Vact = []
    Vsmu = []
    
    SMU.write(':SOUR:FUNC VOLT')
    SMU.write(':SOUR:VOLT:MODE FIXED')
    SMU.write(':SOUR:VOLT:RANG 40')
    #SMU.write(':SOUR:VOLT:LEV '+str(volt-reset_i))
    SMU.write(':SENS:CURR:PROT 1E-1')
    SMU.write(':SENS:FUNC \"CURR\"')
    SMU.write(':SENS:CURR:RANG 1E-1')
    SMU.write(':OUTP 1')
    for i in tqdm(np.linspace(Vmin,Vmax,100)):
        
        if fittype == 'CONTROL':
            setBias(i, True)
        elif fittype == 'SMU':
            SMU.write(':SOUR:VOLT:LEV '+str(i))
        else:
            sys.exit('Invalid fit type: '+fittype)


        time.sleep(1)

        Vset.append(i)
        Vact.append(float(DMM.query(':READ?')))
        Vsmu.append(float(SMU.query(':SOUR:VOLT?')))

    SMU.write(':OUTP 0')
    
    
    tempdict = {
        'Vsetpoint' : Vset,
        'Vactual' : Vact,
        'Vsmu' : Vsmu
    }  
    
    df = pd.DataFrame(tempdict)

    df.to_csv(''.join(['/home/pi/Desktop/VoltageFits/Vfitdata_',sampName]),sep='\t')    
    
    print('Fitting complete')
    print('Fit from Vmin = '+str(Vact[0])+' to Vmax = '+str(Vact[-1]))
    print('Saved as '+''.join(['Vfitdata_',sampName]))
    

#Function to collect LCR output (num = number of points to average over, delay = delay between measurements in s)
def getData():
    
    
    #Read output from LCR display, process incoming string and save to list
    C_avg = []
    G_avg = []
    D_avg = []
    
    try:
        out = LCR.query('*TRG;:MEAS?')
        
        if abs(float(out.split(',')[0]))<1e25:
            C_avg.append(float(out.split(',')[0]))
        else:
            C_avg.append(np.nan)
        if abs(float(out.split(',')[1]))<1e25:
            G_avg.append(float(out.split(',')[1]))
        else:
            G_avg.append(np.nan)        
        if abs(float(out.split(',')[2]))<1e25:
            D_avg.append(float(out.split(',')[2]))
        else:
            D_avg.append(np.nan)        
    except ValueError:
        C_avg.append(np.nan)
        D_avg.append(np.nan)
        G_avg.append(np.nan)
    
    #Average values over defined length
    C = np.mean(C_avg)
    G = np.mean(G_avg)
    D = np.mean(D_avg)
        
    return C, G, D

#Sets LCR frequency (freq = setpoint freq in Hz)
def setFreq(freq):
    LCR.write(':FREQ '+str(freq))

#Sets AC voltage amplitude (volt = setpoint voltage in V)
def set_oscVolt(volt):
    LCR.write(':LEV CV')
    LCR.write(':LEV:CVOLT '+str(volt))  

    print('AC Voltage set to '+str(round(volt,3))+'V                        ',end='\r')

#Sets DC bias (volt = setpoint voltage in V, outp_on = print output on or off Bool)
def setBias(volt,outp_on,fastScan):
    if fastScan == False:
        SMU.write(':SOUR:FUNC VOLT')
        SMU.write(':SOUR:VOLT:MODE FIXED')
        SMU.write(':SOUR:VOLT:RANG 40')
        #SMU.write(':SOUR:VOLT:LEV '+str(volt-reset_i))
        SMU.write(':SENS:CURR:PROT 1E-1')
        SMU.write(':SENS:FUNC \"CURR\"')
        SMU.write(':SENS:CURR:RANG 1E-1')

        mv_avg = [99]*(mvavg_num-1)

        SMU.write(':OUTP 1')
        voltnew = 0
        iters = 0
        pid = PID(P, I, D, setpoint=volt)
        while True:

            try:
                measvolt = float(DMM.query(':READ?'))
            except ValueError:
                pass

            if outp_on is True:
                print('V = '+str(measvolt)+',   V_SMU = '+str(float(SMU.query(':SOUR:VOLT?')))+'V                               ',end='\r')


            mv_avg.append(measvolt)
            #Moving average of V
            if abs(np.average(mv_avg[-mvavg_num:])-volt) <= offset and abs(measvolt-volt) <= offset:
                break


            time.sleep(.1)

            controlV = pid(measvolt)

            if controlV >= v_ceiling:
                controlV = v_ceiling
            elif controlV <= -v_ceiling:
                controlV = -v_ceiling
            else:
                pass

            SMU.write(':SOUR:VOLT:LEV '+str(controlV))

            iters = iters+1

            if iters%reset_val == 0:
                SMU.write(':SOUR:VOLT:LEV '+str(volt-reset))
                print('Resetting V setpoint, '+str(iters)+' of '+str(shutoff_val)+' iterations...                        \n')

            if iters >= shutoff_val:
                print('Error achieving bias voltage of V = '+str(volt))
                break

        if outp_on is True:
            print('DC Bias set to '+str(round(volt,3))+'V                        \n')
    elif fastScan == True:
        SMU.write(':SOUR:FUNC VOLT')
        SMU.write(':SOUR:VOLT:MODE FIXED')
        SMU.write(':SOUR:VOLT:RANG 40')
        #SMU.write(':SOUR:VOLT:LEV '+str(volt-reset_i))
        SMU.write(':SENS:CURR:PROT 1E-1')
        SMU.write(':SENS:FUNC \"CURR\"')
        SMU.write(':SENS:CURR:RANG 1E-1')



        SMU.write(':OUTP 1')
        voltnew = 0
        iters = 0

        Vact = voltFunc(volt)

        if Vact >= v_ceiling:
            Vact = v_ceiling
        elif Vact <= -v_ceiling:
            Vact = -v_ceiling
        else:
            pass
        SMU.write(':SOUR:VOLT:LEV '+str(Vact))

        time.sleep(.01)
        try:
            measvolt = float(DMM.query(':READ?'))
        except ValueError:
            pass            
        print('V = '+str(measvolt)+',   V_SMU = '+str(float(SMU.query(':SOUR:VOLT?')))+'V                               ',end='\r')


notes = {
    'E3' : 164.81,
    'F3' : 174.61,
    'F#3' : 185.0,
    'G3' : 196.0,
    'G#3' : 207.65,
    'A3' : 220.0,
    'A#3' : 223.08,
    'B3' : 246.94,
    'C4' : 261.63,
    'C#4' : 277.18,
    'D4' : 293.66,
    'D#4' : 311.13,
    'E4' : 329.63,
    'F4' : 349.23,
    'F#4' : 369.99,
    'G4' : 392.0,
    'G#4' : 415.30,
    'A4' : 440.0,
    'A#4' : 466.16,
    'B4' : 493.88,
    'C5' : 523.25,
    'C#5' : 554.37,
    'D5' : 587.33,
    'D#5' : 622.25,
    'E5' : 659.25,
    'F5' : 698.46,
    'F#5' : 739.99,
    'G5' : 783.99,
    'G#5' : 830.61,
    'A5' : 880.0
}

def playnote(note,duration):
    SMU.write(':SYST:BEEP '+str(2*notes[note])+str(1))
    time.sleep(duration) 
    
def playchest():
    chest = [('G4',.1),('A4',.1),('B4',.1),('C#5',.1),
        ('G4',.1),('A4',.1),('B4',.1),('C#5',.1),
        ('G#4',.1),('A#4',.1),('C5',.1),('D5',.1),
        ('G#4',.1),('A#4',.1),('C5',.1),('D5',.1),
        ('A4',.1),('B4',.1),('C#5',.1),('D#5',.1),
        ('A4',.1),('B4',.1),('C#5',.1),('D#5',.1),
        ('E3',.2),('F3',.2),('F#3',.2),('G3',.2)]

    for i in chest:
        playnote(*i)
        
def playrand(num,length):
    notelist = list(notes.keys())
    for i in range(num):
        playnote(random.choice(notelist),length)
    time.sleep(0.5)

def playstair():
    for i in notes:
        playnote(i,0.03)
    time.sleep(0.5)

def playon():
    playnote('C4',0.2)
    playnote('C4',0.1)
    playnote('G4',0.2)

def playoff():
    playnote('G4',0.2)
    playnote('E4',0.2)
    playnote('C4',0.2)
    
#Turn bias off            
def offBias():
    SMU.write(':OUTP 0')
    DMM.write('SYST:LOC')
    playoff()

