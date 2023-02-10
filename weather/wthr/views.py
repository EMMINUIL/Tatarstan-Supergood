import json
from django.shortcuts import render
from django.http import HttpResponse
import requests
from django.shortcuts import render

# Create your views here.
def index(request):
    r =  requests.get("https://jsonkit.ru/read/?token=20f90438eaaac6d84bc7ad530750dbb3")
    data = json.loads(r.text)
    #fire
    IR = data["IR"]
    IR = float(IR)
    IR = int(IR)
    data.update({'IRnew': IR})
    #co2
    eco2 = data["eco2"]
    eco2 = float(eco2)
    eco2 = int(eco2)
    data.update({'eco2new': eco2})
    #waterlvl
    WaterLVL = data["WaterLVL"]
    WaterLVL = float(WaterLVL)
    WaterLVL = int(WaterLVL)
    data.update({'WaterLVLnew': WaterLVL})
    #HUM
    Hum = data["Hum"]
    Hum = float(Hum)
    Hum = int(Hum)
    data.update({'HUMnew': Hum})
    #TEMP
    TEMP = data["TEMP"]
    TEMP = float(TEMP)
    TEMP = int(TEMP)
    data.update({'TEMPnew': TEMP})
    #Press
    Press = data["Press"]
    press = int(float(Press))
    pressure = ( press * 100 ) / 133
    pris = round(pressure)
    data.update({'Pressnew': pris})
    FIR = TEMP + IR
    FIR2 = FIR // 2
    data.update({'FIR': FIR2})

    return render(request, "index.html", context=data)