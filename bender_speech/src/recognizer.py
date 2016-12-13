#!/usr/bin/env python
import roslib
import rospy
roslib.load_manifest('bender_speech')

import pygtk
pygtk.require('2.0')
import gtk

import gobject
import pygst
pygst.require('0.10')
gobject.threads_init()
import gst

from std_msgs.msg import String as std_string
from std_srvs.srv import Empty, EmptyResponse
from bender_srvs.srv import String, StringResponse

from threading import Thread
import sys


class recognizer(object):
    """ GStreamer based speech recognizer. """
 
    def __init__(self):        
        
        rospy.init_node('recognizer')
        
        # node variables
        self.dictionary_loaded = False        
        self.pipeline = ""
        self.merge_dict = dict()
        self.available_dictionaries = dict()
        self.mic = []
        self.pkg_dir = roslib.packages.get_pkg_dir('bender_speech')

        # -- publishers --
        self.output_pub = rospy.Publisher('~output', std_string, queue_size=5)
        self.partial_output_pub = rospy.Publisher('~partial_output', std_string, queue_size=10)
        self.silent_pub = rospy.Publisher('~vad_silent',std_string, queue_size=10)

        # -- services --
        rospy.Service("~start", Empty, self.start)
        rospy.Service("~stop", Empty, self.stop)
        rospy.Service("~load_dictionary", String,self.load_dictionary)

        # register function fot shutdown signal
        rospy.on_shutdown(self.shutdown)
        
        # start thread 
        self.t = Thread()
        self.v_t = Thread()
        
        # hmm acustic model
        self.hmm = self.pkg_dir+'/config/acustic_models/'

        # -- work --
        self.load_parameters()
        rospy.loginfo("Please load a dictionary")
        rospy.spin()
            
    
    def load_parameters(self):
               
        if rospy.has_param('~mic_number'):
            number = str(rospy.get_param('~mic_number'))
            self.mic = rospy.get_param('~mic_'+number)
            self.mic['device'] = 'plughw:'+number+',0' # plughw:1,0
            print str(self.mic)
        if rospy.has_param('~hmm'):
            self.hmm += rospy.get_param('~hmm')
        if rospy.has_param('~available_dictionaries'):
            self.available_dictionaries = rospy.get_param('~available_dictionaries')
            rospy.loginfo("Parameters loaded")
        else:
            rospy.loginfo("Can't find available_dictionaries")
            
            
    def load_dictionary(self,req):
        """ Initialize the speech pipeline components. """
        
        if rospy.has_param('~current_dictionary'):
            current_dict = rospy.get_param('~current_dictionary')
            if current_dict == req.data:
                rospy.loginfo("Dictionary '" + req.data + "' already loaded, ready for speech recognition")
                return StringResponse("Dictionary '" + req.data + "' already loaded, ready for recognition")

        rospy.set_param('~current_dictionary',req.data)

        rospy.loginfo("Loading dictionary '" + req.data + "'")
        dictionary_found = False

        dict_ = ""
        fsg_ = ""
        lm_ = ""
        jsgf_ = ""
        hmm_ = self.hmm

        if req.data in self.available_dictionaries.keys():
            dic = self.available_dictionaries[req.data]
            dictionary = dic[0]
            model = dic[1]
            dictionary_found = True
            dict_ = self.pkg_dir+'/Grammar/'+dictionary+'.dic'
            fsg_ = self.pkg_dir+'/Grammar/'+dictionary+'.fsg'
            lm_ = self.pkg_dir+'/Grammar/'+dictionary+'.lm'
            jsgf_ = self.pkg_dir+'/Grammar/'+dictionary+'.jsgf'
        
        if dictionary_found is True:

            if self.dictionary_loaded:
                gtk.main_quit()
                self.pipeline.set_state(gst.STATE_NULL)

            rospy.logwarn('Using Device: '+ str(self.mic['device']))

            self.launch_config = 'alsasrc device='+self.mic['device']
            self.launch_config += ' ! rgvolume pre-amp='+self.mic['gain'] \
                                + ' ! audioconvert ' \
                                + ' ! vader name=vad ' \
                                + ' ! pocketsphinx name=asr '\
                                + ' ! fakesink ' \
                                
#                               + ' headroom='+self.mic['gain'] \
            # launch pocketsphinx with the given configuration
            #rospy.loginfo(self.launch_config)
            self.pipeline = gst.parse_launch(self.launch_config)
            self.asr = self.pipeline.get_by_name('asr')
            self.asr.connect('partial_result', self.asr_partial_result)
            self.asr.connect('result', self.asr_result)
            self.asr.set_property('dsratio', 1)
            # self.asr.set_property('maxwpf', 5)
            self.asr.set_property('bestpath', True)
            #self.asr.set_property('remove_noise', True)
            
            # set dictionary for recognition
            self.asr.set_property('dict',dict_ )
            #self.asr.set_property('hmm',hmm_ )
            #self.asr.set_property('hmm','/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k' )

            if model == 'fsg':
                self.asr.set_property('fsg',fsg_ )
                rospy.logwarn("Using FSG")
            elif model == 'lm':
                self.asr.set_property('lm',lm_ )
                rospy.logwarn("Using LM")
            else:
                self.asr.set_property('fsg',fsg_ )
                rospy.logwarn("Using FSG")      

            rospy.logwarn('setting %s dictionary \n'
                          'with %s acustic model \n'
                          'and %s grammar',dictionary,hmm_,model)

            self.vad = self.pipeline.get_by_name('vad')
            if self.mic['vad_threshold'] == 'auto':
                self.vad.set_property('auto-threshold', True)
            elif self.mic['vad_threshold'] in range(-1,1):
                self.vad.set_property('threshold', self.mic['vad_threshold'])
            self.vad.set_property('dump-dir',self.pkg_dir+"/debug_audio" )
            # record .raw data to debug (play with audacity 8khz 16-bit PCM)

            self.asr.set_property('configured', True)
            
            self.bus = self.pipeline.get_bus()
            self.bus.add_signal_watch()
            self.bus.connect('message::application', self.application_message)
            
            self.t = Thread(target=gtk.main)
            self.t.start()
            # self.v_t = Thread(target=self.vad_status())
            # self.v_t.start()   

            self.dictionary_loaded = True

            # parse dictionary for merge variables      
            if not self.parse_dictionary(jsgf_):
                return StringResponse("Dictionary loaded, BUT found errors on merge definitions") 

            #self.start(Empty)
            rospy.loginfo("Dictionary '" + req.data + "' loaded, ready for speech recognition")
            return StringResponse("Dictionary '" + req.data + "' loaded, ready for recognition")

        else:
            rospy.logerr("Dictionary '" + req.data + "' not found")
            return StringResponse("Dictionary '" + req.data + "' not found")

        rospy.loginfo(self.launch_config)
    
#     def vad_status(self,bus,state):
#         # while self.pipeline.get_state(): 
# # - - - - - get vad_silent param - - -
#         self.silent = 'Not Listening' if self.vad.get_property('silent') else 'Listening'
#         self.silent_pub.publish(self.silent)

    def shutdown(self):
        """ Shutdown the GTK thread. """
        gtk.main_quit()

    def start(self, msg):

        if self.dictionary_loaded:
            rospy.logwarn("Recognition started. Listening...")
            self.pipeline.set_state(gst.STATE_PLAYING)
        else:
            rospy.logwarn("Please load dictionary first")
        return EmptyResponse()


    def stop(self, msg):
        
        if self.dictionary_loaded:
            rospy.logwarn("Recognition stopped")
            self.pipeline.set_state(gst.STATE_NULL)
            self.vad.set_property('silent', True)
            return EmptyResponse()
        else:
            rospy.logwarn("I can't stop because I haven't started")
        return EmptyResponse()

    def asr_partial_result(self, asr, text, uttid):
        """ Forward partial result signals on the bus to the main thread. """
        struct = gst.Structure('partial_result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        self.asr.post_message(gst.message_new_application(asr, struct))

        #print 'in asr_partial_result'

    def asr_result(self, asr, text, uttid):
        """ Forward result signals on the bus to the main thread. """
        struct = gst.Structure('result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        self.asr.post_message(gst.message_new_application(asr, struct))

        #print 'in asr_result'

    def application_message(self, bus, msg):
        """ Receive application messages from the bus. """
        msgtype = msg.structure.get_name()
        
        # - - - - - get hypothesis - - - - -
        hyp = msg.structure['hyp']
        uttid = msg.structure['uttid']
        
        # - - - - - analyze hypothesis - - -
        hyp = self.apply_dictionary_plugin(hyp)
        
        hyp = hyp.lower()
        
        # - - - - - publish results - - - - -
        if msgtype == 'partial_result':
            self.partial_result(hyp, uttid)
            
        if msgtype == 'result':
            self.final_result(hyp, uttid)

        #print 'in application_message'

    def partial_result(self, hyp, uttid):
        """ publish the partial result"""
        self.partial_output_pub.publish(hyp)
        #print 'in partial_result'

    def final_result(self, hyp, uttid):
        """ publish the final result. """
        self.output_pub.publish(hyp)
        #print 'pub final result: ' + hyp
        
    def parse_dictionary(self, dict_path):
        
        rospy.loginfo('parsing dictionary for rules ...')
        
        # clear dictionary
        self.merge_dict.clear()
        rule = '//-->>'
        
        # open for reading
        file = open(dict_path,'r')
        
        # check file for merge rules
        mapping = dict()
        try:
            for line in file:
                if line[0:len(rule)] == rule:
                    (key, value) = self.get_mapping(line[len(rule):len(line)])
                    mapping[key] = value
                
        except Exception:
            '''
            catch exception from self.get_mapping()
            (if the line is malformed, then a exception
            will be thrown)
            '''
            return False
        
        # return to initial line
        file.seek(0)
        
        # analyze for the found mappings
        try:
            count = 0
            for line in file:
                
                line = line.strip()
                i1 = line.find('<')
                i2 = line.find('>')
                
                if not (i1 == 0 and i2 > 1):
                    count += 1
                    continue
                
                # get key
                key = line[0:i2+1]
                
                # check for useful key 
                if key in mapping:
                    
                    # if useful, then analyze
                    str_ = line[i2+1:len(line)]
                    str_ = str_[str_.find('=')+1:str_.find(';')]
                    str_ = str_.strip()
                    self.analyze_jsgf_line(mapping[key],str_)
                    
                count+=1
                
        except Exception:
            return False
        
        print "fill merge plugin dictionary . . . OK"
        
        print self.merge_dict
        
        return True
        
    def analyze_jsgf_line(self, value, str_):
        
        pos = 0
        word = ''
        not_word = '()| '
        
        #str_ = str_.lower()
        
        while pos < len(str_):
                        
            char = str_[pos]
            
            if (char not in not_word) and (pos+1 < len(str_)):
                # build word
                word = word + char
                
            else:
                # word built or empty
                if len(word) > 0:
                    
                    self.merge_dict[word] = value
                    print "Adding word:'" + word + "' for value:'" + value + "'"
                    word = ''
                
            pos += 1
                    
    def get_mapping(self,rule):
        
        # lowercase for all inputs
        line = rule.lower().strip()
        
        # get key
        line = line[line.find('key') + len('key'): len(line)]
        line = line[line.find('=')   + len('=')  : len(line)]
        line = line[line.find('"')   + len('"')  : len(line)]
        index = line.find('"')
        key = line[0:index]
        line = line[index+1:len(line)]
        
        # get value
        line = line[line.find('value') + len('value'): len(line)]
        line = line[line.find('=')   + len('=')  : len(line)]
        line = line[line.find('"')   + len('"')  : len(line)]
        index = line.find('"')
        tag = line[0:index]
        line = line[index+1:len(line)]

        return (key, tag)
       
    def apply_dictionary_plugin(self,hyp):
        
        for key in self.merge_dict.keys():
            
            i1 = hyp.find(key)           
            i2 = i1 + len(key)
            
            if i1 < 0:
                continue
            
            if (i1 == 0  or hyp[i1-1] == ' ') and ( i2 == len(hyp) or hyp[i2] == ' ' ):
                
                if i2 == len(hyp):
                    hyp = hyp[0:i1] + self.merge_dict[key]
                else:
                    hyp = hyp[0:i1] + self.merge_dict[key] + hyp[i2:len(hyp)]            
                 
        return  hyp
    
if __name__=="__main__":
    r = recognizer()
