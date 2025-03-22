TITLE Mod file for component: Component(id=neuron_to_neuron_exc_syn type=expTwoSynapse)

COMMENT

    This NEURON file has been generated by org.neuroml.export (see https://github.com/NeuroML/org.neuroml.export)
         org.neuroml.export  v1.11.0
         org.neuroml.model   v1.11.0
         jLEMS               v0.12.0

ENDCOMMENT

NEURON {
    POINT_PROCESS neuron_to_neuron_exc_syn
    RANGE tauRise                           : parameter
    RANGE tauDecay                          : parameter
    RANGE peakTime                          : parameter
    RANGE waveformFactor                    : parameter
    RANGE gbase                             : parameter
    RANGE erev                              : parameter
    RANGE g                                 : exposure
    RANGE i                                 : exposure
    
    
    NONSPECIFIC_CURRENT i 
    
}

UNITS {
    
    (nA) = (nanoamp)
    (uA) = (microamp)
    (mA) = (milliamp)
    (A) = (amp)
    (mV) = (millivolt)
    (mS) = (millisiemens)
    (uS) = (microsiemens)
    (nF) = (nanofarad)
    (molar) = (1/liter)
    (kHz) = (kilohertz)
    (mM) = (millimolar)
    (um) = (micrometer)
    (umol) = (micromole)
    (pC) = (picocoulomb)
    (S) = (siemens)
    
}

PARAMETER {
    
    tauRise = 3 (ms)                       : was: 0.003 (time)
    tauDecay = 10 (ms)                     : was: 0.01 (time)
    peakTime = 5.1598835 (ms)              : was: 0.005159883447111155 (time)
    waveformFactor = 2.3932765             : was: 2.3932763589352217 (none)
    gbase = 1.0E-5 (uS)                    : was: 1.0000000000000001E-11 (conductance)
    erev = 0 (mV)                          : was: 0.0 (voltage)
}

ASSIGNED {
    ? Standard Assigned variables with baseSynapse
    v (mV)
    celsius (degC)
    temperature (K)
    g (uS)                                  : derived variable
    i (nA)                                  : derived variable
    rate_A (/ms)
    rate_B (/ms)
    
}

STATE {
    A  : dimension: none
    B  : dimension: none
    
}

INITIAL {
    temperature = celsius + 273.15
    
    rates()
    rates() ? To ensure correct initialisation.
    
    A = 0
    
    B = 0
    
}

BREAKPOINT {
    
    SOLVE states METHOD cnexp
    
    
}

NET_RECEIVE(weight) {
    
    : paramMappings . : {neuron_to_neuron_exc_syn={A=A, B=B, tauRise=tauRise, tauDecay=tauDecay, peakTime=peakTime, waveformFactor=waveformFactor, gbase=gbase, erev=erev, g=g, i=i}}
    : state_discontinuity(A, A  + (weight *   waveformFactor  )) : From neuron_to_neuron_exc_syn
    A = A  + (weight *   waveformFactor  ) : From neuron_to_neuron_exc_syn
    
    : paramMappings . : {neuron_to_neuron_exc_syn={A=A, B=B, tauRise=tauRise, tauDecay=tauDecay, peakTime=peakTime, waveformFactor=waveformFactor, gbase=gbase, erev=erev, g=g, i=i}}
    : state_discontinuity(B, B  + (weight *   waveformFactor  )) : From neuron_to_neuron_exc_syn
    B = B  + (weight *   waveformFactor  ) : From neuron_to_neuron_exc_syn
    
}

DERIVATIVE states {
    rates()
    A' = rate_A 
    B' = rate_B 
    
}

PROCEDURE rates() {
    
    g = gbase  * (  B   -   A  ) ? evaluable
    i = -1 * g  * (  erev   - v) ? evaluable
    rate_A = -  A   /  tauRise ? Note units of all quantities used here need to be consistent!
    rate_B = -  B   /  tauDecay ? Note units of all quantities used here need to be consistent!
    
     
    
}

