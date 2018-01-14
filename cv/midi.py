"""
Utility function for converting an audio file
to a pretty_midi.PrettyMIDI object. Note that this method is nowhere close
to the state-of-the-art in automatic music transcription.
This just serves as a fun example for rough
transcription which can be expanded on for anyone motivated.
"""

import sys


import pretty_midi
# import librosa

if __name__ == '__main__':
  
    pm = pretty_midi.PrettyMIDI(initial_tempo=80)
    inst = pretty_midi.Instrument(program=0, is_drum=False, name='my cello')
    pm.instruments.append(inst)

    velocity = 100
    pitch = 50
    for start, end in zip([0.2, 0.6, 1.0,2.2, 4.5], [1.1, 1.7, 2.3,2.5,5.0]):
        inst.notes.append(pretty_midi.Note(velocity, pitch, start, end))
    print inst.notes

    pm.write('out2.mid')

    # pr = np.array([(120,0.4,0.2,1.4), (120,0.4,0.4,2.3),(30,0.5,1.4, 4.5)])
    # print pr
    # pm = piano_roll_to_pretty_midi(pr, fs=80)
    # pm.write('out2.mid')
    # pm.write(parameters['output_midi'])
