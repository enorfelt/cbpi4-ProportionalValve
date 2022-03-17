
# -*- coding: utf-8 -*-
import board
import busio
import adafruit_mcp4725
import asyncio
import logging
import time
from cbpi.api import *

logger = logging.getLogger(__name__)

@parameters([
    Property.Select(label="SamplingTime", options=[1,2,3,4,5],description="Time in seconds for power base interval (Default:3)"),
    Property.Sensor("VolumeSensor",description="Select Volume Sensor that you want to use to be able to auto hold volume"),
    Property.Actor(label="PumpActor", description="If selected starts the pump actor when auto hold is enabled"),
    Property.Number(label="maxOutput",configurable = True, default_value = 100, description="Maximum output %"),
    Property.Number(label="minOutput",configurable = True, default_value = 0, description="Minimum output %"),
    Property.Number(label="P",configurable = True, default_value = 0, description="P Value of PID"),
    Property.Number(label="I",configurable = True, default_value = 0, description="I Value of PID"),
    Property.Number(label="D",configurable = True, default_value = 0, description="D Value of PID"),
])
class ProportionalValveActor(CBPiActor):

    @action("Set Open", parameters=[Property.Number(label="Open", configurable=True,description="Open Setting [0-100]")])
    async def setopen(self, Open = 0, **kwargs):
        self.open = int(Open)
        if self.open < 0:
            self.open = 0
        if self.open > 100:
            self.open = 100
        await self.set_open(self.open)
        pass
    
    @action("Set Target Volume", parameters=[Property.Number(label="Target", configurable=True,description="Target Volume for auto hold")])
    async def settarget(self, Target = 0, **kwargs):
        self.target = float(Target)
        if self.target < 0:
            self.target = 0
        pass
        
    
    async def on_start(self):
        self.state = False
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.dac = adafruit_mcp4725.MCP4725(self.i2c)
        self.target = None
        self.volumeSensor = self.props.get("VolumeSensor", None)

        await self.set_open(0)
        pass

    async def on(self, power = None):
        logger.info("ACTOR 1111 %s ON" % self.id)
        await self.set_open(30)
        if self.props.get("PumpActor", None) is not None:
            await self.cbpi.actor.on(self.props.get("PumpActor"), 100)
        if self.target is None:
            if self.volumeSensor is not None:
                sensorValue = self.cbpi.sensor.get_sensor_value(self.volumeSensor).get("value")
                logger.info(sensorValue)
                target = round(float(sensorValue),2)
                logger.info("Target hold volume: %s" % target)
                self.target = target

        self.state = True

    async def off(self):
        logger.info("ACTOR %s OFF " % self.id)
        await self.set_open(0)
        if self.props.get("PumpActor", None) is not None:
            await self.cbpi.actor.off(self.props.get("PumpActor"))
        self.state = False

    def get_state(self):
        return self.state
    
    async def run(self):
        try:
            samplingTime = int(self.props.get("SamplingTime", 3))
            p = float(self.props.get("P", 30))
            i = float(self.props.get("I", 0.4))
            d = float(self.props.get("D", 0))
            openPercentOld = self.open
            maxOutput = float(self.props.get("maxOutput", 100))
            minOutput = float(self.props.get("minOutput", 0))

            pid = PIDArduino(samplingTime, p, i, d, minOutput, maxOutput)


            while self.running == True:
                if self.state == True:
                    current_valve_open= self.open
                    sensor_value = current_volume = self.cbpi.sensor.get_sensor_value(self.volumeSensor).get("value")
                    targetVolume = self.target
                    openPercent = round(pid.calc(sensor_value, targetVolume), 1)
                    logger.info("Calculated open percent %s " % openPercent)
                    

                    if (openPercentOld != openPercent) or (openPercent != current_valve_open):
                        await self.set_open(openPercent)
                        openPercentOld= openPercent
                    await asyncio.sleep(samplingTime)
                else:
                    await asyncio.sleep(1)
                    
        except asyncio.CancelledError as e:
            pass
        except Exception as e:
            logging.error("ProportionalValve Error {}".format(e))
        finally:
            self.running = False
            self.state = False

    async def set_open(self, Open):
        logger.info("set_open %s" % Open)
        self.open = Open
        self.dac.normalized_value = float(self.open / 100)
        await self.cbpi.actor.actor_update(self.id, Open)
        pass

    async def set_power(self, power):
        pass

# Based on Arduino PID Library
# See https://github.com/br3ttb/Arduino-PID-Library
class PIDArduino(object):

    def __init__(self, sampleTimeSec, kp, ki, kd, outputMin=float('-inf'),
                 outputMax=float('inf'), getTimeMs=None):
        if kp is None:
            raise ValueError('kp must be specified')
        if ki is None:
            raise ValueError('ki must be specified')
        if kd is None:
            raise ValueError('kd must be specified')
        if float(sampleTimeSec) <= float(0):
            raise ValueError('sampleTimeSec must be greater than 0')
        if outputMin >= outputMax:
            raise ValueError('outputMin must be less than outputMax')

        self._logger = logging.getLogger(type(self).__name__)
        self._Kp = kp
        self._Ki = ki * sampleTimeSec
        self._Kd = kd / sampleTimeSec
        self._sampleTime = sampleTimeSec * 1000
        self._outputMin = outputMin
        self._outputMax = outputMax
        self._iTerm = 0
        self._lastInput = 0
        self._lastOutput = 0
        self._lastCalc = 0

        if getTimeMs is None:
            self._getTimeMs = self._currentTimeMs
        else:
            self._getTimeMs = getTimeMs

    def calc(self, inputValue, setpoint):
        now = self._getTimeMs()

        if (now - self._lastCalc) < self._sampleTime:
            return self._lastOutput

        # Compute all the working error variables
        error = inputValue - setpoint
        dInput = inputValue - self._lastInput

        # In order to prevent windup, only integrate if the process is not saturated
        if self._lastOutput < self._outputMax and self._lastOutput > self._outputMin:
            self._iTerm += self._Ki * error
            self._iTerm = min(self._iTerm, self._outputMax)
            self._iTerm = max(self._iTerm, self._outputMin)

        p = self._Kp * error
        i = self._iTerm
        d = -(self._Kd * dInput)

        # Compute PID Output
        self._lastOutput = p + i + d
        self._lastOutput = min(self._lastOutput, self._outputMax)
        self._lastOutput = max(self._lastOutput, self._outputMin)

        # Log some debug info
        self._logger.debug('P: {0}'.format(p))
        self._logger.debug('I: {0}'.format(i))
        self._logger.debug('D: {0}'.format(d))
        self._logger.debug('output: {0}'.format(self._lastOutput))

        # Remember some variables for next time
        self._lastInput = inputValue
        self._lastCalc = now
        return self._lastOutput

    def _currentTimeMs(self):
        return time.time() * 1000

def setup(cbpi):
    cbpi.plugin.register("ProportionalValveActor", ProportionalValveActor)
    pass