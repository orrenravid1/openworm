TITLE Mod file for component: Component(id=Leak type=ionChannelPassive)

COMMENT

    This NEURON file has been generated by org.neuroml.export (see https://github.com/NeuroML/org.neuroml.export)
         org.neuroml.export  v1.11.0
         org.neuroml.model   v1.11.0
         jLEMS               v0.12.0

ENDCOMMENT

NEURON {
    SUFFIX Leak
    NONSPECIFIC_CURRENT i
    RANGE e
    
    RANGE gion
    RANGE i__Leak : a copy of the variable for current which makes it easier to access from outside the mod file
    RANGE gmax                              : Will be changed when ion channel mechanism placed on cell!
    RANGE conductance                       : parameter
    RANGE g                                 : exposure
    RANGE fopen                             : exposure
    
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
    
    gmax = 0  (S/cm2)                       : Will be changed when ion channel mechanism placed on cell!
    
    conductance = 1.0E-5 (uS)              : was: 1.0E-11 (conductance)
}

ASSIGNED {
    
    gion   (S/cm2)                          : Transient conductance density of the channel? Standard Assigned variables with ionChannel
    v (mV)
    celsius (degC)
    temperature (K)
    e (mV)
    i (mA/cm2)
    i__Leak (mA/cm2)
    
    fopen                                   : derived variable
    g (uS)                                  : derived variable
    
}

STATE {
    
}

INITIAL {
    temperature = celsius + 273.15
    
    rates()
    rates() ? To ensure correct initialisation.
    
}

BREAKPOINT {
    
    rates()
    fopen = 1 ? evaluable
    g = conductance ? evaluable
    gion = gmax * fopen 
    
    i = gion * (v - e)
    i__Leak = -1 * i  : set this variable to the current also - note -1 as channel current convention for LEMS used!
    
}

PROCEDURE rates() {
    
    
     
    
}

