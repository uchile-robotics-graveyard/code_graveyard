#!/usr/bin/env python
import socket
import struct
import sys
import os
import math
import subprocess
import time
import signal
import threading
from pocketsphinx import *
from sphinxbase import *
import roslib; roslib.load_manifest('bender_listen')
import ros


class speechrecognizer():

    def __init__(self):

    	# files path
    	pkg_path = roslib.packages.get_pkg_dir('bender_listen')
        config_path = pkg_path + '/conf/en-us/'
        gram_path = pkg_path + '/Grammar/voice_cmd/'
        self.sh_path = pkg_path + '/scripts/'

        hmm= config_path + 'en-us_params'
        #lm = config_path + 'en-us.lm.dmp'
        #dic = config_path + 'cmudict-en-us.dict'
        lm = gram_path + 'voice_cmd.lm'
        dic = gram_path + 'voice_cmd.dic'	
        fsg = gram_path + 'voice_cmd.fsg'

        print lm
        print fsg

        #print 'Model Parameters: '+hmm+'\n','Languaje Model: '+lm+'\n','Dictionary: '+dic+'\n'
        LM = False

        config = Decoder.default_config()

        if LM:
            config.set_string('-lm',str(lm))
        else:
            config.set_string('-fsg',str(fsg))

        config.set_string('-hmm', str(hmm))
        config.set_string('-dict', str(dic))
        config.set_string('-logfn', '/dev/null')
        #config.set_string('-rawlogdir', pkg_path + '/log')
        config.set_string('-input_endian','little')

        self.decoder = Decoder(config)

        print '\nwaiting for a connection'

        threading.Thread(target=self.recog).start()


    def recog(self):

        HOST, PORT = "localhost", 5530
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            s.bind((HOST, PORT))
        except socket.error , msg:
            print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
            sys.exit()

        s.listen(1)

        network = 'sh ' + self.sh_path + 'kn_test.sh'
        print network

        hark_process = subprocess.Popen(['xterm', '-e',str(network)],stderr=subprocess.STDOUT)    
        time.sleep(2)    
        if hark_process.poll() == 0:    
            print "Hark crashed, check 'arecord -l' for an kinect usb on plughw:1,0"
            sys.exit()

        self.connection, client_address = s.accept()
        print 'Connected with HARK @' + client_address[0] + ':' + str(client_address[1])

        # data structures
        header = struct.Struct('3i 2q')
        srcinfo = struct.Struct('i 4f')
        numsrc = struct.Struct('i')
        srcdata = struct.Struct('2i')

        simsrc = 5
        debug = True
        raw = [{'active': False, 'raw': '', 'ID': -1, 'azimuth': 0.0} for x in range(simsrc)]

        while True:
            #receive header (a)
            data = self.connection.recv(header.size)
            header_data = header.unpack(data)
            #print >>sys.stderr, 'Header:', header_data

            #receive number of sound sources (g)        
            data = self.connection.recv(numsrc.size)
            numsrc_data = numsrc.unpack(data)
            #print '#sources:', numsrc_data[0]

            #reset flags in database
            for dat_0 in raw:    
                dat_0['active'] = False
            
            if numsrc_data[0] > 0:

                for x in range(numsrc_data[0]):

                    #receive source info (h)        
                    data = self.connection.recv(srcinfo.size)
                    srcinfo_data = srcinfo.unpack(data)
                    #print >>sys.stderr, 'SrcInfo:', srcinfo_data

                    #store the ID, set active, and store azimuth           
                    raw[srcinfo_data[0]%simsrc]['ID'] = srcinfo_data[0]
                    raw[srcinfo_data[0]%simsrc]['active'] = True
                    raw[srcinfo_data[0]%simsrc]['azimuth'] = 180/math.pi*math.atan2(srcinfo_data[2],srcinfo_data[1])

                    #receive source data (i)
                    data = self.connection.recv(srcdata.size)    
                    srcdata_data = srcdata.unpack(data)
                    #print >>sys.stderr, 'SrcData:', srcdata_data

                    #receive raw audio (j)
                    raw[srcinfo_data[0]%simsrc]['raw'] += self.connection.recv(srcdata_data[1])
                    #print 'SrcRaw:', srcraw.unpack(data)

            for dat in raw:
                if (dat['active']) and (dat['ID'] >= 0):
                    if debug: print "\ndecoding", dat['ID'], len(dat['raw'])
                
                    try:
                        self.decoder.start_utt()
                        self.decoder.process_raw(dat['raw'], False, False)
                        self.decoder.end_utt()
                        result = self.decoder.hyp().hypstr
                        if True:
                            print('\nAzimuth: %.2f (deg)' % dat['azimuth']),
                            if dat['azimuth'] > 0: print '(left)' 
                            else: print '(right)'
                            print 'Source',dat['ID'],'Result:', result

                    except:
                        print "I couldn't decode source ", dat['ID']


if __name__ == "__main__":

    recognizer = speechrecognizer()