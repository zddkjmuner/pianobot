from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass


# Extracted from the notebook; preserve legacy spelling for compatibility.
def generate_chord_library():

    chord_list = []

    chord_list.append("none")

    chord_list.append("C")

    chord_list.append("C#")

    chord_list.append("D")

    chord_list.append("D#")

    chord_list.append("E")

    chord_list.append("F")

    chord_list.append("F#")

    chord_list.append("G")

    chord_list.append("G#")

    chord_list.append("A")

    chord_list.append("A#")

    chord_list.append("B")



    chord_list.append("CM")

    chord_list.append("C#M")

    chord_list.append("DM")

    chord_list.append("D#M")

    chord_list.append("EM")

    chord_list.append("FM")

    chord_list.append("F#M")

    chord_list.append("GM")

    chord_list.append("G#M")

    chord_list.append("AM")

    chord_list.append("A#M")

    chord_list.append("BM")



    chord_list.append("Cm")

    chord_list.append("C#m")

    chord_list.append("Dm")

    chord_list.append("D#m")

    chord_list.append("Em")

    chord_list.append("Fm")

    chord_list.append("F#m")

    chord_list.append("Gm")

    chord_list.append("G#m")

    chord_list.append("Am")

    chord_list.append("A#m")

    chord_list.append("Bm")



    chord_dictionary = {}

    chord_dictionary["none"] =["none","none","none","none"]

    chord_dictionary["C"] = ["C"]

    chord_dictionary["C#"] = ["C#"]

    chord_dictionary["D"] = ["D"]

    chord_dictionary["D#"] = ["D#"]

    chord_dictionary["E"] = ["E"]

    chord_dictionary["F"] =["F"]

    chord_dictionary["F#"] = ["F#"]

    chord_dictionary["G"] = ["G"]

    chord_dictionary["G#"] = ["G#"]

    chord_dictionary["A"] = ["A"]

    chord_dictionary["A#"] = ["A#"]

    chord_dictionary["B"] = ["B"]



    chord_dictionary["CM"] = ["C","E","G"]

    chord_dictionary["C#M"] = ["C#","F","G#"]

    chord_dictionary["DM"] = ["D","F#","A"]

    chord_dictionary["D#M"] = ["D#","G","A#"]

    chord_dictionary["EM"] = ["E","G#","B"]

    chord_dictionary["FM"] =["F","A","C"]

    chord_dictionary["F#M"] = ["F#","A#","C#"]

    chord_dictionary["GM"] = ["G","B","D"]

    chord_dictionary["G#M"] = ["G#","C","D#"]

    chord_dictionary["AM"] = ["A","C#","E"]

    chord_dictionary["A#M"] = ["A#","D","F"]

    chord_dictionary["BM"] = ["B","D#","F#"]



    chord_dictionary["Cm"] = ["C","D#","G"]

    chord_dictionary["C#m"] = ["C#","E","G#"]

    chord_dictionary["Dm"] = ["D","F","A"]

    chord_dictionary["D#m"] = ["D#","F#","A#"]

    chord_dictionary["Em"] = ["E","G","B"]

    chord_dictionary["Fm"] =["F","G#","C"]

    chord_dictionary["F#m"] = ["F#","A","C#"]

    chord_dictionary["Gm"] = ["G","A#","D"]

    chord_dictionary["G#m"] = ["G#","B","D#"]

    chord_dictionary["Am"] = ["A","C","E"]

    chord_dictionary["A#m"] = ["A#","C#","F"]

    chord_dictionary["Bm"] = ["B","D","F#"]



    chord_scale_index = {}

    chord_scale_index["none"] =[0,0,0,0]

    chord_scale_index["C"] = [0]

    chord_scale_index["C#"] = [0]

    chord_scale_index["D"] = [0]

    chord_scale_index["D#"] = [0]

    chord_scale_index["E"] = [0]

    chord_scale_index["F"] =[0]

    chord_scale_index["F#"] = [0]

    chord_scale_index["G"] = [0]

    chord_scale_index["G#"] = [0]

    chord_scale_index["A"] = [0]

    chord_scale_index["A#"] = [0]

    chord_scale_index["B"] = [0]



    chord_scale_index["CM"] = [0,0,0]

    chord_scale_index["C#M"] = [0,0,0]

    chord_scale_index["DM"] = [0,0,0]

    chord_scale_index["D#M"] = [0,0,0]

    chord_scale_index["EM"] = [0,0,0]

    chord_scale_index["FM"] =[0,0,1]

    chord_scale_index["F#M"] = [0,0,1]

    chord_scale_index["GM"] = [0,0,1]

    chord_scale_index["G#M"] = [0,1,1]

    chord_scale_index["AM"] = [0,1,1]

    chord_scale_index["A#M"] =[0,1,1]

    chord_scale_index["BM"] = [0,1,1]



    chord_scale_index["Cm"] = [0,0,0]

    chord_scale_index["C#m"] = [0,0,0]

    chord_scale_index["Dm"] = [0,0,0]

    chord_scale_index["D#m"] = [0,0,0]

    chord_scale_index["Em"] = [0,0,0]

    chord_scale_index["Fm"] = [0,0,1]

    chord_scale_index["F#m"] = [0,0,1]

    chord_scale_index["Gm"] = [0,0,1]

    chord_scale_index["G#m"] = [0,0,1]

    chord_scale_index["Am"] = [0,1,1]

    chord_scale_index["A#m"] = [0,1,1]

    chord_scale_index["Bm"] = [0,1,1]

    # Append Dominant7, Major7, Minor7 Later



    return chord_list, chord_dictionary, chord_scale_index

def add_chord_library_song_salute_de_amure(chord_list, chord_dictionary, chord_scale_index):

    chord_list.append("G#BE")

    chord_list.append("F#C#E")

    chord_list.append("F#BD#")

    chord_list.append("BD#")

    chord_list.append("G#C")

    chord_list.append("C#E")

    chord_list.append("F#BE")

    chord_list.append("F#A#E")

    chord_list.append("G#B")

    chord_list.append("C#F")

    chord_list.append("C#A")

    chord_list.append("C#G#")

    chord_list.append("C#F#")

    chord_list.append("AC#")

    chord_list.append("F#B")

    chord_list.append("AB")





    chord_list.append("C#C#")

    chord_list.append("BB")

    chord_list.append("AA")

    chord_list.append("G#EG#")

    chord_list.append("EG#")

    chord_list.append("D#C#F#")

    chord_list.append("C#F#")



    chord_list.append("AE")

    chord_list.append("EG#E")

    chord_list.append("G#E")





    chord_dictionary["G#BE"] = ["G#","B","E"]

    chord_dictionary["F#C#E"] = ["F#","C#","E"]

    chord_dictionary["F#BD#"] = ["F#","B","D#"]

    chord_dictionary["BD#"] = ["B","D#"]

    chord_dictionary["G#C"] = ["G#","C"]

    chord_dictionary["C#E"] = ["C#","E"]

    chord_dictionary["F#BE"] = ["F#","B","E"]

    chord_dictionary["F#A#E"] = ["F#","A#","E"]

    chord_dictionary["G#B"] = ["G#","B"]

    chord_dictionary["C#F"] = ["C#","F"]

    chord_dictionary["C#A"] = ["C#","A"]

    chord_dictionary["C#G#"] = ["C#","G#"]

    chord_dictionary["C#F#"] = ["C#","F#"]

    chord_dictionary["AC#"] = ["A","C#"]

    chord_dictionary["F#B"] = ["F#","B"]

    chord_dictionary["AB"] = ["A","B"]



    chord_dictionary["C#C#"] = ["C#","C#"]

    chord_dictionary["BB"] = ["B","B"]

    chord_dictionary["AA"] = ["A","A"]

    chord_dictionary["G#EG#"] = ["G#","E","G#"]

    chord_dictionary["EG#"] = ["E","G#"]

    chord_dictionary["D#C#F#"] = ["D#","C#","F#"]

    chord_dictionary["C#F#"] = ["C#","F#"]

    chord_dictionary["AE"] = ["A","E"]

    chord_dictionary["EG#E"] = ["E","G#","E"]

    chord_dictionary["G#E"] = ["G#","E"]



    chord_scale_index["G#BE"] = [0,0,1]

    chord_scale_index["F#C#E"] = [0,1,1]

    chord_scale_index["F#BD#"] = [0,0,1]

    chord_scale_index["BD#"] = [0,1]

    chord_scale_index["G#C"] = [0,1]

    chord_scale_index["C#E"] = [0,0]

    chord_scale_index["F#BE"] = [0,0,1]

    chord_scale_index["F#A#E"] = [0,0,1]

    chord_scale_index["G#B"] = [0,0]

    chord_scale_index["C#F"] = [0,0]

    chord_scale_index["C#A"] = [0,0]

    chord_scale_index["C#G#"] = [0,0]

    chord_scale_index["C#F#"] = [0,0]

    chord_scale_index["AC#"] = [0,1]

    chord_scale_index["F#B"] = [0,0]

    chord_scale_index["AB"] = [0,0]





    chord_scale_index["C#C#"] = [0,1]

    chord_scale_index["BB"] = [0,1]

    chord_scale_index["AA"] = [0,1]

    chord_scale_index["G#EG#"] = [0,1,1]

    chord_scale_index["EG#"] = [0,0]

    chord_scale_index["D#C#F#"] = [0,1,1]

    chord_scale_index["C#F#"] = [0,0]

    chord_scale_index["AE"] = [0,1]

    chord_scale_index["EG#E"] = [0,0,1]

    chord_scale_index["G#E"] = [0,1]





    

    return chord_list, chord_dictionary, chord_scale_index

CHORD_LIST, CHORD_DICTIONARY, CHORD_SCALE_INDEX = generate_chord_library()
CHORD_LIST, CHORD_DICTIONARY, CHORD_SCALE_INDEX = add_chord_library_song_salute_de_amure(
    CHORD_LIST, CHORD_DICTIONARY, CHORD_SCALE_INDEX
)

# Notebook-compatible global names expected by extracted functions.
chord_list = CHORD_LIST
chord_dictionary = CHORD_DICTIONARY
chord_scale_index = CHORD_SCALE_INDEX

def generate_note_from_chord(chord_name, scale=0):

    notes = chord_dictionary[chord_name]

    scale_offset = chord_scale_index[chord_name]



    C_out = []

    for idx, note in enumerate(notes):

        s = scale+scale_offset[idx]

        C_out.append(note+f"_{s}")

    return C_out

@dataclass(slots=True)
class MusicNote:
    name: str
    chord: Sequence[str] | None = None
    key_scale: int = 0
    t_push: float = 0.16
    t_hold: float = 0.4
    t_release: float = 0.12
    t_transition: float = 0.32
    t_scale: float = 1.0

    def __post_init__(self) -> None:
        if self.chord is None:
            self.chord = CHORD_DICTIONARY[self.name]
        raw_push = self.t_push
        raw_hold = self.t_hold
        raw_release = self.t_release
        raw_transition = self.t_transition
        my_scaler = 1.0
        self.t_push = raw_push * my_scaler * self.t_scale
        self.t_release = raw_release * my_scaler * self.t_scale
        self.t_transition = raw_transition * my_scaler * self.t_scale
        self.t_hold = (
            (raw_hold + raw_push + raw_release + raw_transition) * self.t_scale * my_scaler
            - self.t_push
            - self.t_release
            - self.t_transition
        )


# Backward-compatible class name used by extracted notebook functions.
Music_Note = MusicNote

def generate_salute_de_amur_lowchord_5actave():

    bpm = 67

    ts = 60/bpm



    note_seq = []





    note_size = 1.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#BE",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#BE",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("E",key_scale =1, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("G#BE",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#BE",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("E",key_scale =1, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("G#BE",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#BE",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#",key_scale =0, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("G#BE",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#BE",key_scale =1, t_scale = t_note))

#---------------------------------------------------------

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("F#",key_scale =0, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("F#C#E",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("F#C#E",key_scale =1, t_scale = t_note))

#--------------------------------------------------------------------------

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("B",key_scale =0, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("F#BD#",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("BD#",key_scale =1, t_scale = t_note))

#--------------------------------------------------------------------------



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("E",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#BE",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("E",key_scale =1, t_scale = t_note))





    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#C",key_scale =1, t_scale = t_note))







    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("C#",key_scale =1, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("C#E",key_scale =2, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("C#E",key_scale =2, t_scale = t_note))

#--------------------------------------------------------------------------



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("F#",key_scale =0, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("F#BE",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("F#A#E",key_scale =1, t_scale = t_note))

#--------------------------------------------------------------------------



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("B",key_scale =0, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("F#BD#",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("BD#",key_scale =1, t_scale = t_note))



#--------------------------------------------------------------------------

    bpm = 60

    ts = 60/bpm



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("none",key_scale =0, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("G#BE",key_scale =1, t_scale = t_note))





    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#BE",key_scale =1, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#",key_scale =0, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("G#B",key_scale =1, t_scale = t_note)) 



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#B",key_scale =1, t_scale = t_note)) 



#--------------------------------------------------------------------------

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("A",key_scale =0, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("C#E",key_scale =2, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#",key_scale =0, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("C#F",key_scale =2, t_scale = t_note)) 





    bpm = 50

    ts = 60/bpm



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("F#",key_scale =0, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("C#A",key_scale =2, t_scale = t_note)) 



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("C#G#",key_scale =2, t_scale = t_note)) 



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("C#F#",key_scale =2, t_scale = t_note)) 



#--------------------------------------------------------------------------

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("B",key_scale =0, t_scale = t_note)) 



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("B",key_scale =1, t_scale = t_note)) 



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("AC#",key_scale =1, t_scale = t_note)) 



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("C#E",key_scale =2, t_scale = t_note)) 

#----------------------------------------------------------------



    bpm = 40

    ts = 60/bpm

    

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("B",key_scale =0, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("F#B",key_scale =1, t_scale = t_note)) 





    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("B",key_scale =0, t_scale = t_note))

    

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("AB",key_scale =1, t_scale = t_note)) 



#----------------------------------------------------------------

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("E",key_scale =0, t_scale = t_note))



    note_size = 1.5

    t_note = note_size*ts

    note_seq.append(Music_Note("E",key_scale =1, t_scale = t_note)) 





    return note_seq

def generate_salute_de_amur_highchord_5actave():

    bpm = 67

    ts = 60/bpm



    note_seq = []



    note_size = 4

    t_note = note_size*ts

    note_seq.append(Music_Note("none",key_scale =1, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("G#",key_scale =3, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("B",key_scale =2, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#",key_scale =3, t_scale = t_note))





    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("F#",key_scale =3, t_scale = t_note))

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("E",key_scale =3, t_scale = t_note))

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("D#",key_scale =3, t_scale = t_note))

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("E",key_scale =3, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("A",key_scale =3, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("A",key_scale =3, t_scale = t_note))







#--------------------------------------------------------------------------



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("A",key_scale =3, t_scale = t_note))

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("B",key_scale =2, t_scale = t_note))

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("none",key_scale =1, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("G#",key_scale =3, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("C",key_scale =3, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#",key_scale =3, t_scale = t_note))







    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("F#",key_scale =3, t_scale = t_note))

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("E",key_scale =3, t_scale = t_note))

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("D#",key_scale =3, t_scale = t_note))

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("E",key_scale =3, t_scale = t_note))







    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("F#",key_scale =3, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("F#",key_scale =3, t_scale = t_note))



    note_size = 1.5

    t_note = note_size*ts

    note_seq.append(Music_Note("F#",key_scale =3, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G",key_scale =3, t_scale = t_note))



    #--------------------------------------------------

    bpm = 60

    ts = 60/bpm



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("G#",key_scale =3, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("B",key_scale =2, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("G#",key_scale =3, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("F#",key_scale =3, t_scale = t_note))

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("E",key_scale =3, t_scale = t_note))

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("D#",key_scale =3, t_scale = t_note))

    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("E",key_scale =3, t_scale = t_note))

    ##



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("C#",key_scale =4, t_scale = t_note))



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("C#",key_scale =4, t_scale = t_note))





    bpm = 50

    ts = 60/bpm





    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("C#",key_scale =4, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("B",key_scale =3, t_scale = t_note))





    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("A",key_scale =3, t_scale = t_note))







    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("EG#",key_scale =3, t_scale = t_note))



    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("C#F#",key_scale =3, t_scale = t_note))





    note_size = 0.5

    t_note = note_size*ts

    note_seq.append(Music_Note("AE",key_scale =2, t_scale = t_note))





#----------------------------------------------------------------



    bpm = 40

    ts = 60/bpm



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("AC#",key_scale =2, t_scale = t_note)) 



    note_size = 1

    t_note = note_size*ts

    note_seq.append(Music_Note("BD#",key_scale =2, t_scale = t_note)) 





    note_size = 2

    t_note = note_size*ts

    note_seq.append(Music_Note("G#E",key_scale =2, t_scale = t_note)) 





    return note_seq


def generate_salut_damour_low_chord_score() -> list[MusicNote]:
    return generate_salute_de_amur_lowchord_5actave()


def generate_salut_damour_high_note_score() -> list[MusicNote]:
    return generate_salute_de_amur_highchord_5actave()
