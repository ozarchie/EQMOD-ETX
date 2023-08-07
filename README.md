# EQMOD-ETX-ESP32
## Introduction
This project describes a possible arduino interface between the ASCOM-EQMOD software and the 'HBX' interface to the Az and Alt motors controller. The device investigated was the ETX60AT. It provides a protocol translater to enable the features of EQMOD to be applied to some ETX scopes using the low-level HBX motor control commands. It is for personal experimental use only. It requires an appropriate knowledge of software and hardware to attempt. 
  
*The project is experimental and does not claim to be a specification of the protocol or its' interface, nor does it claim continuous error-free operation. As always, it is the responsibilty of the user to assess whether this information, program and hardware is useful and appropriate to them and to assess any likely impact on their systems and equipment, including the possibilty of any damage.*  

## Interface Design

The design is now published on EasyEDA:  

https://easyeda.com/jmarchbold/eqg2hbxv12

You can prototype this using standard Arduino modules, or simply order boards from JLPCB. I have some spare units, but the postage is usually higher than ordering them direct!
If you use arduino 3.3v<->5v level translators, the level translator will need to be modified for the different resistances required. Standard level translators use 10k resistors. The ETX60 needs these changed to 47k or 56k. I am not sure about other telescopes, but the early ETX60 has all data lines joined (which effectively connects these resistors in parallel). As a result, the pullup is too strong and the communications fail.


## Notices
Information in this document was collected from multiple sources on the Weasner site. It is a summary of those documents and retains the original warnings/copyrights where they existed.  

The main information comes from -:
1. http://www.weasner.com/etx/menu.html    
2. http://www.lycee-ferry-versailles.fr/si-new/4_5_programmation/3_tp/DIAGRAMME_ETATS/1_ASTROLAB/Telescope_Dossier_technique.pdf
3. Some information assumed from observations of the interactions between the Autostar 494 controller and the DH series motors. The information in this document was not derived via disassembly of the Autostar or motor processor firmware    

See information about relevant intellectual property in the original documents   

This work would not be possible without the prior and ongoing contributions of Dick Seymour, Gene Chimahusky, Andrew Johansen and others and is summarised, to the best of my ability, in the Documents section. Please refer to other contributors in the documents.
 
  
