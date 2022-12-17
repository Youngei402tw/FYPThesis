# FYPThesis

This is repo for Liu Yang Che CST1904 Thesis title "INVESTIGATING DIVISION PROPERTY AIDED CUBE ATTACK ON STREAM CIPHERS" code and logs.

TinyJAMBU is a stream cipher as one of the ten finalists in NIST LWC competition, since there are only few applications of three-subset division property without unknown subset, and no applications of this new attack method on TinyJAMBU. This thesis attempts to address this research gap by analyzing the feasibility and possible superpoly recovery of applying three-subset division property without unknown subset on the feedback function of TinyJAMBU.
In this study, three-subset division property without unknown subset is applied on the feedback function of TinyJAMBU, the results of the application showed that the superpoly recovery is not trivial to conducted using the MILP modelling method used in this thesis. However, the MILP model for core function is implemented and introduced for the first time, which can be beneficial to future study. Lastly, correlations between the non-passing round and excluding cube index used in cube attack on TinyJAMBU can be observed. Where there is a drop and increase from 367-round to 371-round and 373-round to 375-round of non-passing round when excluding cube index is 23 to 27 and 29 to 31 respectively. While the non-passing round for other excluding cube index remains roughly around 375-round to 377-round.

Keywords: Three-subset division property without unknown subset, TinyJAMBU, Cube Attack, Stream ciphers, NIST LWC

