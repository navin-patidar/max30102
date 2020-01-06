#!/usr/bin/python

import json
import logging
import threading
import time
import Queue
import copy

import paho.mqtt.client as mqtt

from scipy.signal import filtfilt
from scipy.signal import butter
import peakutils

import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.ptime import time

def put_data(msg):
	queue_lock.acquire()
	publish_queue.put(msg)
	queue_lock.release()
	event.set()


def on_connect(client, userdata, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("#")

def on_message(client, userdata, msg):
    data = json.loads(str (msg.payload))
    x = data['data']['red']
    put_data(x)

mqtt_thread = None

def mqtt_client():
    client = mqtt.Client()
#    client.connect("192.168.1.80", 1883, 60)
#    client.connect("10.170.153.172", 1883)
    client.connect("127.0.0.1", 1883)
    client.on_connect = on_connect
    client.on_message = on_message
    client.loop_forever()

def start_bno_thread():

    global mqtt_thread
    mqtt_thread = threading.Thread(target=mqtt_client)
    mqtt_thread.daemon = True
    mqtt_thread.start()


red=[]
queue_lock = threading.Lock()
publish_queue = Queue.Queue(1000)
event = threading.Event()
event.clear()

app = QtGui.QApplication([])
win = pg.GraphicsWindow(title="PPG Signal Analysis")
win.resize(1000,600)
win.setWindowTitle('PPG Signal Analysis')

p = win.addPlot(title="Raw data from MAX30100")
raw_data = p.plot()

win.nextRow()

q = win.addPlot(title="Filtered data from MAX30100")
filter_data = q.plot()
peaks_data = q.plot()

sample_rate = 100.0
lowcut = 0.4
highcut = 4.5

w1 = lowcut/(sample_rate/2)
w2 = highcut/(sample_rate/2)

if __name__ == '__main__':
    start_bno_thread()

    b, a = butter(4, [w1,w2] ,btype='bandpass', analog=False, output='ba')

    while True:
        event.wait()
        queue_lock.acquire()
        while not publish_queue.empty():
            msg = publish_queue.get()
            red.append(int(msg))

        queue_lock.release()
        event.clear()

        if len(red) > 1000:
            filtered_signal = filtfilt(b, a,red)
            baseline = peakutils.baseline(filtered_signal, 3)
            fixed_baseline_signal =  filtered_signal - baseline
#            peaks = peakutils.indexes(fixed_baseline_signal, thres=0.02/max(fixed_baseline_signal), min_dist=40)
            peaks_index = peakutils.indexes(fixed_baseline_signal, thres=0.4, min_dist=40)
            j = 0
            peaks = copy.deepcopy(fixed_baseline_signal)
            for i in range(0,len(peaks)):
                if i != peaks_index[j]:
                    peaks[i] = 0
                else:
                    if j < len(peaks_index)-1 :
                        j = j +1

            print  'BPM =',(float(len(peaks_index))/float(len(red)))*6000
            raw_data.setData(red)
            filter_data.setData(fixed_baseline_signal)
            peaks_data.setData(peaks, pen=(0,255,0))
            app.processEvents()
            clear = len(red) - 1000
            red = red[clear:]

