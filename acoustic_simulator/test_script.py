#!/usr/bin/env python3

from datawriter_class import datawriter
from acoustic_sim_class import acousticSimulation
from localisation_sim_class import localisationSimulation
from dataloader_class import dataLoader
from plot_class import plot

from tqdm import tqdm
import json
import os
import threading
import numpy as np


class simulation():
    def __init__(self):
        self.plot1 = plot()
        tmp = os.path.dirname(__file__)
        file_path_acoustic_config = os.path.join(tmp, '../config/acoustic_config.json')
        print(f'Acoustic config file: {file_path_acoustic_config}')
        f = open(file_path_acoustic_config)
        self.acoustic_config = json.load(f)
        f.close()

        file_path_filter_config = os.path.join(tmp, '../config/filter_config.json')
        print(f'Filter config file: {file_path_filter_config}')
        f = open(file_path_filter_config)
        self.filter_config = json.load(f)
        f.close()

        self.t0 = 0
        self.dt = 0
        self.t = self.t0
        self.last_t = self.t0
        self.z = None
        self.x0 = self.filter_config["config"][1]["settings"]["InitState"]
        self.x = None
        self.x_est = None
        self.statePre = None
        self.covarPre = None
        self.p_mat = None
        self.ros = self.acoustic_config["config"][0]["RosRun"]

        self.lock = threading.RLock()
        self.position = [0, 0, 0]
        self.velocity = [0, 0, 0]
        self.depth = 0

        # settings from Configfile
        self.f_pre = self.filter_config["config"][0]["PredictFrequency"]
        self.f_ac = self.acoustic_config["config"][0]["FrequencyAcousticSim"]
        
        self.acoustic_sim = acousticSimulation()
        self.localisation_sim = localisationSimulation()
        self.dataloader = dataLoader()
        self.dataWriter = datawriter()

    def measErrDist(self, AnchorPos, meas, x):
        x = np.array(x)
        zhat = np.linalg.norm(AnchorPos-x)
        err = meas - zhat
        return err, zhat

    def run(self):
        time_gps, UTMPosx, UTMPosy, depth, vx, vy, vz, meas_data = self.dataloader.inputClearedData()
                
        for i in tqdm(range(len(time_gps)), ncols=100):
            x = np.array([UTMPosx[i], UTMPosy[i], depth[i]]).T
            self.x0 = x
            preInput = np.array([vx[i], vy[i], vz[i]]).T + np.random.normal(self.filter_config["config"][0]["MeasErrLoc"],self.filter_config["config"][0]["MeasErrScale"],3)
            meas = self.acoustic_sim.simulate(x, time_gps[i])
            self.dataWriter.fillState(x[0],x[1],x[2],time_gps[i])
            self.dataWriter.writeCSVState()
            if meas is not None:
                err, zhat = self.measErrDist(meas["ModemPos"], meas["dist"], x)
                self.dataWriter.fillMeas(meas["time_published"], meas["tr"], meas["dist"], meas["realDist"], meas["ModemID"], meas["Error"])
                self.dataWriter.fillErr(err)
                self.dataWriter.writeCSVMeas()
                self.plot1.addMeas(meas["dist"], meas["Error"], meas["time_published"], meas["ModemID"]) #dist, time, ID
                XFilter = self.localisation_sim.locate(preInput, time_gps[i], x[2], meas)
                self.plot1.addPosFilter(time_gps[i], XFilter)
            elif meas is None:
                XFilter = self.localisation_sim.locate(preInput, time_gps[i], x[2], meas=None)
                self.plot1.addPosFilter(time_gps[i], XFilter)
            else:
                print("[Simulation_Class]: Error in manual dataInput") 

            self.plot1.addPosGPS(time_gps[i], x)
        self.plot1.plot(meas_data)
        self.plot1.plotPath()
        self.plot1.plotMeas(meas_data)
        self.plot1.show_plots()
        
        return
 


def main():
    simu = simulation()
    simu.run()


if __name__ == '__main__':
    main()
