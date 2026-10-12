#!/usr/bin/env pybricks-micropython


# =================================================
# PROGRAMM AUSWÄHLEN
# =================================================

# Möglichkeiten:
#
# "pd_basic"
# "pd_basic_log"
# "pd_advanced"
# "q_learning_hybrid"
# "q_learning_ad_hybrid"

MODE = "q_learning_ad_hybrid"


# =================================================
# PROGRAMM STARTEN
# =================================================

if MODE == "q_learning_hybrid":

    import pd_basic


elif MODE == "pd_basic_log":

    import pd_basic_with_log


elif MODE == "pd_advanced":

    import pd_advanced


elif MODE == "q_learning_hybrid":

    import q_learning_hybrid


elif MODE == "q_learning_ad_hybrid":

    import q_learning_ad_hybrid


else:

    print("Unbekannter MODE:")
    print(MODE)