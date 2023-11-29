import cv2 as cv
import time
from djitellopy import Tello
from telloManager import tellomanager
from GUI import userinterface
from arucoDetection import detector
from Controller import controller
import multiprocessing as mp

def managerfunc(data,video,control,detection):
	print("Alt Process Starting")
	manager = tellomanager(data,video,control,detection)
	manager.Run()

def guifunc(video,control,i_q):
	ui = userinterface(video,control,i_q)
	ui.run()

def ctrlfunc(video,control,i_q):
	ctrl = controller(video,control,i_q)
	ctrl._run()

def main():
	d_q = mp.Queue()
	v_q = mp.Queue()
	c_q = mp.Queue()
	i_q = mp.Queue()
	detection_queue = mp.Queue()


	man_proc = mp.Process(target=managerfunc, args=(d_q,v_q,c_q,detection_queue))
	gui_proc = mp.Process(target=guifunc, args=(v_q,c_q,i_q))
	ctrl_proc = mp.Process(target=ctrlfunc, args=(detection_queue,c_q,i_q))

	man_proc.start()
	gui_proc.start()
	ctrl_proc.start()
	run = True

	print("Main Loop Starting")
	
	man_proc.join()
	gui_proc.join()
	ctrl_proc.join()

	print("All done!")


if __name__ == '__main__':
	main()