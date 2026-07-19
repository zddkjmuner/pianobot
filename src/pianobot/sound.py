"""Piano sound output LeafSystem backed by pygame and piano-sounds samples."""

import numpy as np
import pygame as pg
from pydrake.all import BasicVector, LeafSystem

from pianobot.paths import piano_sounds_dir


class PianoOutputSystem(LeafSystem):
    """Wrapper system for Piano Sound Output. 
    """ 

    def __init__(self, plant,key_list):#, diffik_fun):
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        key_indices = np.arange(1,61)

        self.q_port = self.DeclareVectorInputPort("q", BasicVector(83))
        self.w_port = self.DeclareVectorInputPort("w", BasicVector(83))

        self.DeclareVectorOutputPort("sound_output", BasicVector(60),self.adjust_sound)
        self.DeclareVectorOutputPort("state_output", BasicVector(83),self.data_pipe)
    
        self.key_list = key_list
        self.N_key = len(key_list)

        pg.mixer.init()
        pg.init()
        pg.mixer.set_num_channels(self.N_key)
        self.init_keys(key_list)

        
    class SoundNote:
        def __init__(self, index, name, path):
            self.index = index
            self.name = name
            self.soundpath = path
            self.channel = pg.mixer.Sound(path)
            
            self.t_on = 0
            self.t_off = 0
            self.q_max = 0

            self.volume = 0
            self.play_state = "off"

        def set_volume(self, new_volume=1):
            self.channel.set_volume(new_volume)
            self.volume = new_volume
        
        def play(self, new_volume =1):
            self.channel.play(fade_ms=70)
            self.set_volume(new_volume)
        
        def fadeout(self, t_off = 400):
            self.channel.fadeout(t_off) # fadeout after t_off ms
        
        def stop(self):
            self.channel.stop()

    def init_keys(self,key_list):
        # the sound used in this module 
        # is forked from https://github.com/ledlamp/piano-sounds
        # mp3 files should be reprocessed to wave file
        base_path = str(piano_sounds_dir("Steinway_Grand")) + "/"

        self.Notes =[]
        n_const =1
        
        for idx, key in enumerate(key_list):            
            key_path = key
            key_path = key_path.replace("_","") #remove underscore
            key_path = key_path.replace("#","s")
            n = int(key_path[-1])+n_const

            key_path = key_path[:-1].lower()
            
            path = base_path+key_path+str(n)+".mp3.wav"
            note = self.SoundNote(idx,key,path)
            self.Notes.append(note)

    def data_pipe(self, context,output):

        q_all =  self.q_port.Eval(context)

        q_state = np.zeros((71+12,1))
        q_state[:,0] = q_all
        self.adjust_sound(context,output)
        output.SetFromVector(q_state)
        
    def adjust_sound(self, context,output):
        time_step = 1e-3
        q_all =  self.q_port.Eval(context)
        w_all =  self.w_port.Eval(context)
        
        q0 = 0.0025
        q_piano = np.zeros((48+12,1))
        w_piano = np.zeros((48+12,1))

        q_piano[:,0] = q_all[1:49+12]-q0
        w_piano[:,0] = w_all[1:49+12]


        q_on_threshold = 0.001 #1degree 
        q_off_threshold = 0.002 #1degree 
        
        q_conv_const = 20
        w_conv_const = 0.3
        def update_note(idx):
            note = self.Notes[idx]
            q_note = q_piano[idx,0]
            w_note = w_piano[idx,0]

            play_state = note.play_state
            
            if play_state =="off":

                if q_note >q_on_threshold:
                    note.play(q_note*q_conv_const)
                    print("idx : {}, name: {}, q: {}, volume: {}".format(note.index, note.name, q_note, q_note*q_conv_const))
                    self.Notes[idx].q_max = q_note
                    play_state = "on_rising"
            
            elif play_state == "on_rising":
                self.Notes[idx].t_on+=time_step

                if self.Notes[idx].q_max < q_note+1e-5:
                    # note.set_volume(q_note*q_conv_const)
                    self.Notes[idx].q_max = q_note
                else:
                    play_state = "on_falling"
                    # note.play(self.Notes[idx].q_max*q_conv_const)
                    note.set_volume(self.Notes[idx].q_max*q_conv_const)
                    print("idx : {}, name: {}, q: {}, volume: {}".format(note.index, note.name, q_note, self.Notes[idx].q_max*q_conv_const))
                    
            elif play_state == "on_falling":
                self.Notes[idx].t_on+=time_step

                if q_note< q_off_threshold:
                    # note.set_volume(self.Notes[idx].q_max*0.8*q_conv_const)
                    # note.stop(self.Notes[idx].t_on*10)
                    print("idx : {}, name: {}, t_on: {}".format(note.index, note.name, self.Notes[idx].t_on))
                    note.set_volume(self.Notes[idx].q_max*q_conv_const)
                    # self.Notes[idx].t_off += time_step
                    self.Notes[idx].q_max = 0
                    play_state = "turning_off"

            elif play_state =="turning_off":
                self.Notes[idx].t_off += time_step
                # note.set_volume(self.Notes[idx].q_max*q_conv_const * (1-(self.Notes[idx].t_off/self.Notes[idx].t_on*4)**2))

                if self.Notes[idx].t_off>= self.Notes[idx].t_on*1.8:

                    print("idx : {}, name: {}, t_off: {}, t_on: {}".format(note.index, note.name, self.Notes[idx].t_on, self.Notes[idx].t_off))
                    
                    note.fadeout(int(self.Notes[idx].t_off*1e3))    
                    # note.stop()
                    print("fadeout : {}".format(int(self.Notes[idx].t_off*1e3)))
                    
                    self.Notes[idx].t_on = 0
                    self.Notes[idx].t_off = 0
                    play_state = "off"
                
                elif q_note >=q_off_threshold-1e-5:
                    print("Estop idx : {}, name: {}, t_on: {}, t_off: {}".format(note.index, note.name, self.Notes[idx].t_on, self.Notes[idx].t_off))

                    note.fadeout(int(self.Notes[idx].t_on*1e3))   
                    self.Notes[idx].t_on = 0
                    self.Notes[idx].t_off = 0
                    play_state = "off"
                    # note.stop()
                    
            self.Notes[idx].play_state = play_state

        def update_note_w(idx):
            note = self.Notes[idx]
            q_note = q_piano[idx,0]
            w_note = w_piano[idx,0]
            
            play_state = note.play_state
            
            if play_state =="off":

                if q_note >q_on_threshold:
                    note.play(w_note*w_conv_const)
                    print("idx : {}, name: {}, q: {}, volume: {}".format(note.index, note.name, q_note, w_note*w_conv_const))
                    self.Notes[idx].q_max = q_note
                    play_state = "on_rising"
            
            elif play_state == "on_rising":
                self.Notes[idx].t_on+=time_step

                if self.Notes[idx].q_max < q_note+1e-5:
                    self.Notes[idx].q_max = q_note
                else:
                    play_state = "on_falling"
                    print("idx : {}, name: {}, q: {}, volume: {}".format(note.index, note.name, q_note, self.Notes[idx].q_max*q_conv_const))
                    
            elif play_state == "on_falling":
                self.Notes[idx].t_on+=time_step

                if q_note< q_off_threshold:
                    print("idx : {}, name: {}, t_on: {}".format(note.index, note.name, self.Notes[idx].t_on))
                    # note.set_volume(self.Notes[idx].q_max*q_conv_const)
                    self.Notes[idx].q_max = 0
                    play_state = "turning_off"

            elif play_state =="turning_off":
                self.Notes[idx].t_off += time_step
                # note.set_volume(self.Notes[idx].q_max*q_conv_const * (1-(self.Notes[idx].t_off/self.Notes[idx].t_on*4)**2))

                if self.Notes[idx].t_off>= self.Notes[idx].t_on*1.7:

                    print("idx : {}, name: {}, t_off: {}, t_on: {}".format(note.index, note.name, self.Notes[idx].t_on, self.Notes[idx].t_off))
                    
                    note.fadeout(int(self.Notes[idx].t_off*5*1e3))    
                    # note.stop()
                    print("fadeout : {}".format(int(self.Notes[idx].t_off*1e3)))
                    
                    self.Notes[idx].t_on = 0
                    self.Notes[idx].t_off = 0
                    play_state = "off"
                
                elif q_note >=q_off_threshold-1e-5:
                    print("Estop idx : {}, name: {}, t_on: {}, t_off: {}".format(note.index, note.name, self.Notes[idx].t_on, self.Notes[idx].t_off))

                    note.fadeout(int(self.Notes[idx].t_off*5*1e3))    
                    self.Notes[idx].t_on = 0
                    self.Notes[idx].t_off = 0
                    play_state = "off"
                    # note.stop()

            self.Notes[idx].play_state = play_state


        for i in range(self.N_key):
            # update_note(i)
            update_note_w(i)
