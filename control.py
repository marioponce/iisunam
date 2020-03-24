# -*- coding: utf-8 -*-
"""
Created on Mon Mar 23 21:14:12 2020

@author: mapp2
"""
import serial
import os, time, datetime
import numpy as np
#import gpiozero as gpio0

class azotea:
    threshold = [30, 80]  # PMP y CC
    tank_range = [20, 80]  # Rango de operacion del tanque en cm, min max

    # Inicializar valores de lectura de humedad del suelo para cada sensor
    # en total 16 sensores en ceros
    soilMoisture = {0 : 0.0, 1 : 0.0, 2 : 0.0, 3 : 0.0, 4 : 0.0, 5 : 0.0,
                    6 : 0.0, 7 : 0.0, 8 : 0.0, 9 : 0.0, 10 : 0.0, 11 : 0.0,
                    12 : 0.0, 13 : 0.0, 14 : 0.0, 15 :0.0}
    # Secciones de riego
    secc = {0 : [0, 1, 2, 3, 4], 1 : [5, 6, 7, 8], 2 : [9, 10], 3 : [11, 12],
            4 : [13, 14, 15]}
    # Valvulas (valves), False = cerrado, True = abierto
    vals = {0 : False, 1 : False, 2 : False, 3 : False, 4 : False}
    wl = 0.0  # Water level
    HR = 0.0  # Humidity relative
    Temp = 0.0  # Temperature
    pump = False  # Estado de la bomba, False = cerrada
    time_to_irri = "05:00:00"  # Hora de riego
    
    prior = np.array([0,1,2,3,4])  # Prioridad de las secciones de riego
    mois_min = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Valores minimos humedad
    mois_max = np.array([100.0, 100.0, 100.0, 100.0, 100.0])  # y los maximos
    q = np.array([0.1, 0.47, 0.53, 0.4, 0.5])  # Gasto requerido por seccion
    Q = 1.0  # Total
    to_irri = [prior[0]]
    
    is_need_to_irri = False
    def __init__(self):
        pass
    
    def update_moisture(self, new_values):
        """
        Actualiza los valores de los sensores de humedad 0-15
        param: new_vals, valores de humedad del suelo obtenidos del sensor
        return: null
        """
        for i in self.soilMoisture.keys():
            self.soilMoisture.update({i: float(new_values[i])})
            
                
    def get_min_max_mois(self):
        """
        Regresa los valores minimo y maximo de humedad del suelo de
        la seccion i
        param: i, la seccion actual 
        return: tupla con valores minimo y maximo
        """
        for i in self.prior:
            mois = [self.soilMoisture[j] for j in self.secc[i]]
            self.mois_min[i], self.mois_max[i] = min(mois), max(mois)
    
    def read_serial(self):
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
        ser.flush()
        while True:
        #if ser.in_waiting > 0:
            try:
                ln = ser.readline().decode('utf-8').rstrip()
                break
            except:
                pass
        values = ln.split(',')
        self.update_moisture(values)
        self.wl = float(values[-3])
        self.HR = float(values[-2])
        self.Temp = float(values[-1])
        
    def check_mois(self):
        if (sum(self.mois_max >= self.threshold[1]) == len(self.mois_max)) and (sum(self.mois_min >= self.threshold[0]) == len(self.mois_max)):
            self.is_need_to_irri = False
            pump0 = False
            for i in self.vals.keys():
                self.vals.update({i : False})
        else:
            self.is_need_to_irri = True
            self.get_min_max_mois()
            self.prior = np.argsort(self.mois_min)
            self.mois_min = self.mois_min[self.prior]
            self.mois_max = self.mois_max[self.prior]
            self.to_irri = []
            j = 0
            for i in self.prior:
                if (self.mois_min[i] < self.threshold[0]) and (self.mois_max[i] < self.threshold[1]):
                    self.to_irri = [self.prior[i]]
                    j = i
                    break
            while True:
                if (j + 1 > len[self.q]) or (self.to_irri == []) :
                    break
                if sum(self.q[self.to_irri]) + self.q[j+1] < self.Q:
                    if (self.mois_min[j+1] < self.threshold[0]) and (self.mois_max[j+1] < self.threshold[1]):
                        j += 1 
                        self.to_irri.append(self.prior[j])
                else:
                    break
            for i in self.prior.keys():
                self.vals.update({i : True})
            pump0 = True
            
            if self.wl < self.tank_range[0]:
                self.pump = False
            # Si el nivel es suficiente y se debe regar, encender la bomba
            elif self.wl > self.tank_range[0] and pump0:
                self.pump = True
    
    def update_gpio(self):
        pass
    
    def record(self):
        pass
    def send_report(self):
        pass
        
    def is_time_to_irri(self,time_to_irri):
        time_0 = time_to_irri.split(":")
        time_1 = datetime.datetime.now().ctime()
        time_1 = time_1.split(" ")[3].split(":")
        return time_0 == time_1
    
if __name__ == '__main__':
    
    schedule = ["00:00:00",
                "00:06:00",
                "00:12:00",
                "00:18:00"]
    