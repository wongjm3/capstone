import serial
import serial.tools.list_ports
import time
import math
from numpy import std
from matplotlib import pyplot as plt
from fractions import Fraction

'''
SERIAL_CHANGE_MODE = 255
MODE_NONE = 0
MODE_BUILD = 1
MODE_RUN = 2
MODE_DEBUG = 255
SERIAL_CHARACTERIZE_RESISTOR = 0
RESISTOR_SAMPLE_SIZE = 500
SERIAL_CHARACTERIZE_CAPACITOR = 1
CAPACITOR_SAMPLE_SIZE = 300
SERIAL_CHARACTERIZE_DIODE = 2
DIODE_SAMPLE_SIZE = 100
'''

# Originally correct
#SUPPLY_VOLTAGE = 20
SUPPLY_VOLTAGE = 5.0
#REFERENCE_VOLTAGE = 2.56
#REFERENCE_VOLTAGE = SUPPLY_VOLTAGE * (6.81)/(6.81+parallelResistance())
# Originally correct
#REFERENCE_VOLTAGE = 2.59
REFERENCE_VOLTAGE = 5.0
#REFERENCE_VOLTAGE = 5
#ADC_RESISTOR_HIGH = 8150000
#ADC_RESISTOR_LOW = 1023000
# Originally correct
ADC_RESISTOR_HIGH = 774.2e3
ADC_RESISTOR_LOW = 101.8e3
# With 5V supply
# The last 3 are placeholder values. They can be used or be spare
REFERENCE_RESISTANCES = [470, 1002, 9880, 46900, 220200, 999999, 999999]
REFERENCE_RESISTOR_ADC_RESISTOR_HIGH = [(300e3+474e3), (302e3+475e3), (303e3+474e3), (300e3+473e3), 1]
REFERENCE_RESISTOR_ADC_RESISTOR_LOW = [(51100+50900), (50700+50900), (50900+51400), (50900+51100), 1]
#REFERENCE_RESISTOR_ADC_RESISTOR_HIGH = [8150e3, (302e3+475e3), (303e3+474e3), 8220e3]
#REFERENCE_RESISTOR_ADC_RESISTOR_LOW = [1023e3, (50700+50900), (50900+51400), 1020e3]

RAIL_READ_RATIO = [1.0, 0.5, 1.0/3, 1.0/4]

#arduinoFilepath = "C:\Users\Jameson Wong\Documents\University\Year 5\Eng Phys 4A06\Misc\_20181111_characterization_circuit\_20181111_characterization_circuit.ino"
arduinoFilepath = "C:\Users\Jameson Wong\Documents\University\Year 5\Eng Phys 4A06\Misc\_20181129_characterization_circuit\_20181129_characterization_circuit.ino"

CONSTANTS = {}

REPORT_STATE = True
DEBUG = True

#REPORT_STATE = False
#DEBUG = False

def main():
    # This is still not necessarily the real deal?
    # We want to read serial, call the appropriate function, then dump everything out for like a second?
    # In the real program we should call the appropriate function and then only stop once we've read everything we said we were going to read
    # Note: This method will require a lot of explicit handling for lengths of some serial outputs. Maybe there's an option either at bootup or when returning valid command to inform the python program what size of serial output to expect

    if REPORT_STATE: print "**STARTING PROGRAM**"

    # Read arduino file and grab all its constants
    arduinoFile = open(arduinoFilepath, "r")
    arduinoCode = arduinoFile.read()
    arduinoFile.close()
    arduinoLines = arduinoCode.split("\n")
    for line in arduinoLines:
        words = line.split()
        if (len(words) == 3 and words[0] == "#define"):
            if (isInt(words[2])):
                CONSTANTS[words[1]] = int(words[2])
            else:
                CONSTANTS[words[1]] = words[2]

    if REPORT_STATE: print "**ARDUINO CONSTANTS FOUND**"

    #for keys,values in CONSTANTS.items():
    #    print keys, values

    if REPORT_STATE: print "**LOCATING ARDUINO**"
    arduino = None
    try:
        arduino = ArduinoInterface()
    except AssertionError as error:
        print error
        print "Exiting as a result"
        exit()

    if REPORT_STATE: print "**ARDUINO LOCATED**"

    if (not arduino.initAcknowledge()):
        print "ERROR: Arduino did not acknowledge serial communications. Exiting"
        exit()

    if REPORT_STATE: print "**ARDUINO COMMUNICATIONS ACKNOWLEDGED**"

    arduino.initCharacterize()

    if REPORT_STATE: print "**ARDUINO PARAMETERS INITIALIZED**"

    arduino.changeMode(CONSTANTS["MODE_BUILD"])

    if REPORT_STATE: print "**SET TO BUILD MODE**"

    while (True):
        text = raw_input("Please enter a request: ")
        print(text)
        if (text == "kill"):
            break
        else:
            contents = text.split()
            if (len(contents) == 2) and (contents[0] == "mode"):
                print arduino.changeMode(int(contents[1]))
            elif (len(contents) == 2) and (contents[0] == "change"):
                print arduino.changeReferenceResistor(int(contents[1]))
            elif (len(contents) >= 3) and (contents[0] == "resistor"):
                # This test is for serial only. I'll probably need a different input test to target parallel components
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildResistor(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 3) and (contents[0] == "capacitor"):
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildCapacitor(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 3) and (contents[0] == "capacitor2"):
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildCapacitor2(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 3) and (contents[0] == "diode"):
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildDiode(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 3) and (contents[0] == "resistorComp"):
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildResistorComp(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 3) and (contents[0] == "capacitorComp"):
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildCapacitorComp(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 3) and (contents[0] == "capacitorMix"):
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildCapacitorMix(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 2) and (contents[0] == "power"):
                if (contents[1] == "on"):
                    arduino.runPower(CONSTANTS["SWITCH_ON"])
                elif (contents[1] == "off"):
                    arduino.runPower(CONSTANTS["SWITCH_OFF"])
                elif (contents[1] == "toggle"):
                    arduino.runPower(CONSTANTS["SWITCH_TOGGLE"])
            elif (len(contents) >= 2) and (contents[0] == "voltage"):
                railNums = []
                for i in contents[1:]:
                    if (not isInt(i)):
                        print "Invalid input"
                        continue
                    else:
                        railNums.append(int(i))
                ans = arduino.runVoltage(railNums)
                for i in ans:
                    print i, ans[i]
            elif (all(isInt(x) for x in contents)):
                # Need to do more extensive test here in case inputs are not ints, but for now this can do
                if DEBUG: print "Hello let's debug"
                print arduino.debugSerial(text)
            elif (text.split(":")[0] == "eval"):
                eval(text.split(":")[1:])
            else:
                print "Error: Unexpected input. Discarding command"







class ArduinoInterface:
    def __init__(self):
        self.ser = None
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if str(port.device).find("Arduino"):
                self.port = port
                self.ser = serial.Serial(str(port.device), CONSTANTS["BAUD_RATE"], timeout=30)
        assert self.ser, "Could not find Arduino!"
        self.mode = CONSTANTS["MODE_INIT"]
        # Define constants of the system
        # Supply voltage of the circuit
        self.supplyVoltage = SUPPLY_VOLTAGE
        # Internal Arduino reference voltage
        self.referenceVoltage = REFERENCE_VOLTAGE
        # Voltage divider ratio for rail read
        #self.adcVoltageDrop = 1.0*ADC_RESISTOR_LOW/(ADC_RESISTOR_HIGH+ADC_RESISTOR_LOW)
        self.adcVoltageDrop = 1.0
        # Converts ADC reading from Arduino to voltage reading at circuit
        self.adcToVoltage = 1.0*self.referenceVoltage/(CONSTANTS["ADC_SIZE"]*self.adcVoltageDrop)
        # Voltage divider ratio for reference resistor reads. List of size equal to number of reference resistors
        #self.referenceResistorAdcVoltageDrop = [(1.0*REFERENCE_RESISTOR_ADC_RESISTOR_LOW[i]/(REFERENCE_RESISTOR_ADC_RESISTOR_HIGH[i]+REFERENCE_RESISTOR_ADC_RESISTOR_LOW[i])) for i in range(CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"])]
        self.referenceResistorAdcVoltageDrop = [1.0]*CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]
        # Converts ADC reading from Arduino to voltage reading at circuit
        self.referenceResistorAdcToVoltage = [1.0*self.referenceVoltage/(CONSTANTS["ADC_SIZE"]*self.referenceResistorAdcVoltageDrop[i]) for i in range(CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"])]
        # Reference resistor values
        self.referenceResistance = REFERENCE_RESISTANCES
        # Total resistance off reference resistor leg, taking into account ADC voltage divider
        # Use this whenever you're trying to determine current for reference resistance
        self.referenceResistanceWithAdc = []
        for i in range(CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]):
            #self.referenceResistanceWithAdc.append(parallelResistance([self.referenceResistance[i], REFERENCE_RESISTOR_ADC_RESISTOR_HIGH[i]+REFERENCE_RESISTOR_ADC_RESISTOR_LOW[i]]))
            self.referenceResistanceWithAdc.append(self.referenceResistance[i])
            if DEBUG:
                print "Reference resistances with parallel voltage divider. Should be slightly lower than original reference resistance"
                print "Original:", self.referenceResistance[i], "\tParallel:", self.referenceResistanceWithAdc[i]
        # Resistance of mux demux combo, at 0 0 position. Set during initCharacterization
        self.muxDemuxResistance = None
        # Variance in mux demux resistance at each individual rail. Set during initCharacterization
        self.muxDemuxResistanceVariance = [None]*CONSTANTS["MUX_SELECT_SIZE"]
        # Resistance of mux that selects reference resistance
        self.referenceResistorMuxResistance = [None]*CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]
        # Sum of resistance of reference resistance and mux resistance of that select
        self.totalReferenceResistance = [None]*CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]
        # Total resistance of all components from rail ADC tap and downwards. Different options indicate different effective resistance based on reference resistor
        # Set during initCharacterization
        # Use this whenever you are calculating resistance and stuff from sample circuit
        self.totalReferenceResistanceWithAdc = [None]*CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]
        # Current reference resistor index number
        self.currRR = 0
        # CHANGE 20181108
        time.sleep(1)
        self.flushBuffer()

    # Check to make sure serial communications work on Arduino
    def initAcknowledge(self):
        #print CONSTANTS["SERIAL_INIT_ACKNOWLEDGE"]
        #print type(CONSTANTS["SERIAL_INIT_ACKNOWLEDGE"])
        self.ser.write([CONSTANTS["SERIAL_INIT_ACKNOWLEDGE"]])
        timeReference = time.time()
        while (time.time() - timeReference < 3):
            #time.sleep(1)
            #output = self.ser.read(9999)
            #output = self.ser.read_until(size=9999)
            #output = self.ser.readline().strip()
            output = self.readToNewline()
            if DEBUG: print "ACKNOWLEDGE:", output
            if output == "1":
                return True
            elif output == "0":
                return False
            elif DEBUG:
                print output
        return False

    # Change the mode of the Arduino
    # Available modes:
    #  INIT
    #  BUILD
    #  RUN
    #  DEBUG
    def changeMode(self, mode):
        if REPORT_STATE: print "**CHANGING MODES"
        self.ser.write([CONSTANTS["SERIAL_CHANGE_MODE"], mode])
        if (not self.acknowledgeRequest()):
            return False
        self.mode = mode

    # Automatically calculate resistance of muxes
    def initCharacterize(self):
        if REPORT_STATE: print "**BEGIN INIT CHARACTERIZE**"
        if self.mode != CONSTANTS["MODE_INIT"]:
            return False
        self.ser.write([CONSTANTS["SERIAL_INIT_CHARACTERIZE"]])
        if (not self.acknowledgeRequest()):
            return False
        # Characterize reference resistor mux
        if REPORT_STATE: print "CHARACTERIZING REFERENCE RESISTORS"
        for i in range(CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]):
            # Get voltage readings
            voltageReadRail = float(self.readToNewline()) * self.adcToVoltage
            voltageReadResistor = float(self.readToNewline()) * self.referenceResistorAdcToVoltage[i]
            if DEBUG:
                print "Reference resistor: voltageReadRail: Raw value:", voltageReadRail/self.adcToVoltage, \
                        "\tActual voltage:", voltageReadRail
                print "Reference resistor: voltageReadResistor: Raw value:", voltageReadResistor/self.referenceResistorAdcToVoltage[i], \
                        "\tActual voltage:", voltageReadResistor
            # ADC reference resistance change
            # Current through mux is the voltage divided by the total reference resistance plus ADC resistance
            current = voltageReadResistor/self.referenceResistanceWithAdc[i]
            # Mux resistance is the voltage difference across that mux, using Ohm's law
            self.referenceResistorMuxResistance[i] = (voltageReadRail-voltageReadResistor)/current
            # Sum of reference resistance and mux resistance
            self.totalReferenceResistance[i] = self.referenceResistance[i] + self.referenceResistorMuxResistance[i]
            # Resistance when taking into account the ADC rail
            self.totalReferenceResistanceWithAdc[i] = parallelResistance([self.totalReferenceResistance[i], ADC_RESISTOR_HIGH+ADC_RESISTOR_LOW])
            if DEBUG:
                print "Reference resistor mux reference resistance, #", i, ":", self.referenceResistorMuxResistance[i]
                print "Total reference resistance, #", i, ":", self.totalReferenceResistance[i]
                print "Total reference resistance with ADC, #", i, ":", self.totalReferenceResistanceWithAdc[i]
        ## TODO: FOR DEBUGGING ONLY
        #return
        # Characterize mux and demuxes
        # Note that this characterization is always done with the 0th reference resistance
        if REPORT_STATE: print "CHARACTERIZING MUX AND DEMUX RESISTANCES"
        # Get first mux resistance and use as a baseline
        voltageReadRail = float(self.readToNewline()) * self.adcToVoltage
        voltageReadResistor = float(self.readToNewline()) * self.referenceResistorAdcToVoltage[0]
        #current = voltageReadResistor/self.referenceResistanceWithAdc[0]
        current = voltageReadRail/self.totalReferenceResistanceWithAdc[0]
        self.muxDemuxResistance = (self.supplyVoltage-voltageReadRail)/current
        self.muxDemuxResistanceVariance[0] = 0.0
        if DEBUG:
            print "Mux/Demux: voltageReadRail: Raw value:", voltageReadRail/self.adcToVoltage, \
                    "\tActual voltage:", voltageReadRail
            print "Mux/Demux: voltageReadResistor: Raw value:", voltageReadResistor/self.adcToVoltage, \
                    "\tActual voltage:", voltageReadResistor
            print "Mux/Demux resistance:", self.muxDemuxResistance
        for i in range(1, CONSTANTS["MUX_SELECT_SIZE"]):
            voltageReadRail = float(self.readToNewline()) * self.adcToVoltage
            voltageReadResistor = float(self.readToNewline()) * self.referenceResistorAdcToVoltage[0]
            current = voltageReadRail/self.totalReferenceResistanceWithAdc[0]
            self.muxDemuxResistanceVariance[i] = (self.supplyVoltage-voltageReadRail)/current - self.muxDemuxResistance
            if DEBUG:
                print "Mux/Demux: voltageReadRail: Raw value:", voltageReadRail / self.adcToVoltage, \
                    "\tActual voltage:", voltageReadRail
                print "Mux/Demux: voltageReadResistor: Raw value:", voltageReadResistor / self.adcToVoltage, \
                    "\tActual voltage:", voltageReadResistor
                print "Mux/Demux resistance:", self.muxDemuxResistance

        if DEBUG:
            print "FINAL MUX RESISTANCE VALUES:"
            for i in range(CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]):
                print "Reference resistor mux resistance: " + str(self.referenceResistorMuxResistance[i])
            print "Mux-demux resistance", self.muxDemuxResistance
            for i in range(CONSTANTS["MUX_SELECT_SIZE"]):
                print "Demux resistance variance: ", self.muxDemuxResistanceVariance[i]

    # Change reference resistor being used
    def changeReferenceResistor(self, select):
        if REPORT_STATE: print "**CHANGE REFERENCE RESISTOR**"
        if self.mode != CONSTANTS["MODE_BUILD"]:
            return False
        time.sleep(1)
        if DEBUG:
            print "Old reference resistance:", self.currRR
            print "New reference resistance:", select
        self.ser.write([CONSTANTS["SERIAL_BUILD_CHANGE_REFERENCE_RESISTOR"], select])
        if (not self.acknowledgeRequest()):
            return False
        self.currRR = select
        return True

    def buildResistor(self, rail1, rail2, otherComponents=[]):
        if REPORT_STATE: print "**BUILD RESISTOR**"
        if self.mode != CONSTANTS["MODE_BUILD"]:
            return False
        # Syntax of otherComponents:
        # eg. No parallel components: []
        #     1 parallel component:   [["resistor", 1000]]
        #     2 parallel components:  [["resistor", 1000], ["capacitor", 1]]
        #     Loopback components:    [["custom", someUglyContents]]
        numMeasurements = 255
        parallelResistors = []
        parallelCapacitors = []
        parallelDiodes = []
        for component in otherComponents:
            if (component[0] == "resistor"):
                parallelResistors.append(component[1])
            elif (component[0] == "capacitor"):
                parallelCapacitors.append(component[1])
            elif (component[0] == "diode"):
                parallelDiodes.append(component[1])
        if (len(parallelDiodes) > 0):
            return "Error: Resistor cannot be measured with diode in parallel"
        elif (len(parallelCapacitors) > 0):
            #return "I will do this later. Only valid in certain values and using sympy"

            steadyStateRatio = 0.99
            contentMeasurement = False
            self.changeReferenceResistor(0)
            totalCapacitance = sum(parallelCapacitors) * 1.0e-6
            totalResistance = self.totalReferenceResistanceWithAdc[self.currRR] + self.muxDemuxResistance + (
                    self.muxDemuxResistanceVariance[rail1] + self.muxDemuxResistanceVariance[rail2]) / 2
            timeDelay = min(math.fabs(math.log(1-steadyStateRatio) * totalResistance * totalCapacitance)*10, 255)
            resistance = 0
            measurementCount = 0
            while not contentMeasurement:
                measurementCount += 1
                totalResistance = self.totalReferenceResistanceWithAdc[self.currRR] + self.muxDemuxResistance + (
                        self.muxDemuxResistanceVariance[rail1] + self.muxDemuxResistanceVariance[rail2]) / 2
                timeDelay = min(int(abs(math.log(1 - steadyStateRatio) * totalResistance * totalCapacitance) * 10),
                                255)
                self.ser.write([CONSTANTS["SERIAL_BUILD_VOLTAGE_READ_DELAY"], rail1, rail2, numMeasurements, timeDelay])
                if not self.acknowledgeRequest():
                    return False
                time.sleep(timeDelay/10.0)
                voltageReadRail = []
                voltageReadReferenceResistor = []
                for i in range(numMeasurements):
                    voltageReadRail.append(float(self.readToNewline()))
                    voltageReadReferenceResistor.append(float(self.readToNewline()))
                time.sleep(timeDelay/10.0)
                # Remove potentially bad deviating values.
                voltage = average(removeDeviations(voltageReadRail)) * self.adcToVoltage
                if DEBUG:
                    print "Original voltage read:", average(voltageReadRail), "+-", std(voltageReadRail)
                    print "Parsed voltage read:", average(removeDeviations(voltageReadRail)), "+-", std(removeDeviations(voltageReadRail))
                voltageReferenceResistor = average(removeDeviations(voltageReadReferenceResistor)) * self.referenceResistorAdcToVoltage[self.currRR]
                current = voltage/self.totalReferenceResistanceWithAdc[self.currRR]
                # Naive check
                if current <= 0:
                    if self.currRR < CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]-1:
                        self.changeReferenceResistor(self.currRR+1)
                        continue
                    else:
                        return "Error: Resistance = 0"
                resistance = (self.supplyVoltage-voltage)/current - \
                                (self.muxDemuxResistance+(self.muxDemuxResistanceVariance[rail1]+self.muxDemuxResistanceVariance[rail2])/2)
                if (resistance > self.totalReferenceResistanceWithAdc[self.currRR]) and \
                   (self.currRR < CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]-1) and \
                   (resistance/self.totalReferenceResistanceWithAdc[self.currRR] > self.totalReferenceResistanceWithAdc[self.currRR+1]/resistance):
                    self.changeReferenceResistor(self.currRR+1)
                elif (resistance < self.totalReferenceResistanceWithAdc[self.currRR]) and \
                    (self.currRR > 0) and \
                     (self.totalReferenceResistanceWithAdc[self.currRR]/resistance > resistance/self.totalReferenceResistanceWithAdc[self.currRR-1]):
                    self.changeReferenceResistor(self.currRR-1)
                elif (measurementCount == 10):
                    # Super naive exit routine. Need to change to an average later
                    contentMeasurement = True
                else:
                    contentMeasurement = True
                if DEBUG: print "Calculated total resistance:", resistance, "\tExiting:", contentMeasurement
            unknownResistance = reverseParallelResistance(resistance, parallelResistors)
            if (abs(resistance-parallelResistance(parallelResistors))/resistance < 0.01):
                return "Error: Resistance is too high to be measured"
            self.changeReferenceResistor(0)
            return unknownResistance
        else:
            #if (not otherComponents):
            # Serial
            contentMeasurement = False
            self.changeReferenceResistor(0)
            resistance = 0
            measurementCount = 0
            while not contentMeasurement:
                measurementCount += 1
                self.ser.write([CONSTANTS["SERIAL_BUILD_VOLTAGE_READ"], rail1, rail2, numMeasurements])
                if not self.acknowledgeRequest():
                    return False
                voltageReadRail = []
                voltageReadReferenceResistor = []
                for i in range(numMeasurements):
                    voltageReadRail.append(float(self.readToNewline()))
                    voltageReadReferenceResistor.append(float(self.readToNewline()))
                # Remove potentially bad deviating values.
                voltage = average(removeDeviations(voltageReadRail)) * self.adcToVoltage
                if DEBUG:
                    print "Original voltage read:", average(voltageReadRail), "+-", std(voltageReadRail)
                    print "Parsed voltage read:", average(removeDeviations(voltageReadRail)), "+-", std(removeDeviations(voltageReadRail))
                voltageReferenceResistor = average(removeDeviations(voltageReadReferenceResistor)) * self.referenceResistorAdcToVoltage[self.currRR]
                current = voltage/self.totalReferenceResistanceWithAdc[self.currRR]
                # Naive check
                if current <= 0:
                    if self.currRR < CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]-1:
                        self.changeReferenceResistor(self.currRR+1)
                        continue
                    else:
                        return "Error: Resistance = 0"
                resistance = (self.supplyVoltage-voltage)/current - \
                                (self.muxDemuxResistance+(self.muxDemuxResistanceVariance[rail1]+self.muxDemuxResistanceVariance[rail2])/2)
                if (resistance > self.totalReferenceResistanceWithAdc[self.currRR]) and \
                   (self.currRR < CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]-1) and \
                   (resistance/self.totalReferenceResistanceWithAdc[self.currRR] > self.totalReferenceResistanceWithAdc[self.currRR+1]/resistance):
                    self.changeReferenceResistor(self.currRR+1)
                elif (resistance < self.totalReferenceResistanceWithAdc[self.currRR]) and \
                    (self.currRR > 0) and \
                     (self.totalReferenceResistanceWithAdc[self.currRR]/resistance > resistance/self.totalReferenceResistanceWithAdc[self.currRR-1]):
                    self.changeReferenceResistor(self.currRR-1)
                elif (measurementCount == 10):
                    # Super naive exit routine. Need to change to an average later
                    contentMeasurement = True
                else:
                    contentMeasurement = True
                if DEBUG: print "Calculated total resistance:", resistance, "\tExiting:", contentMeasurement
            unknownResistance = reverseParallelResistance(resistance, parallelResistors)
            if (abs(resistance-parallelResistance(parallelResistors))/resistance < 0.01):
                return "Error: Resistance is too high to be measured"
            self.changeReferenceResistor(0)
            return unknownResistance
            #averageVoltageReadRail = sum(voltageReadRail)/len(voltageReadRail)
            #stddevVoltageReadRail = std(voltageReadRail)
            #averageVoltageReadReferenceResistor = sum(voltageReadReferenceResistor)/len(voltageReadReferenceResistor)
            #stddevVoltageReadReferenceResistor = std(voltageReadReferenceResistor)
            #voltage = averageVoltageReadRail * self.adcToVoltage
            #voltageReferenceResistor = averageVoltageReadReferenceResistor * self.referenceResistorAdcToVoltage[self.currRR]
            #current = voltage/(self.totalReferenceResistanceWithAdc[self.currRR])
            #current = voltageReferenceResistor /self.referenceResistanceWithAdc[self.currRR]
            #resistance = (self.supplyVoltage-voltage)/current - (self.muxDemuxResistance+(self.muxDemuxResistanceVariance[rail1]+self.muxDemuxResistanceVariance[rail2])/2)
            #return resistance
        return None

    def buildResistorComp(self, rail1, rail2, otherComponents=[]):
        if REPORT_STATE: print "**BUILD RESISTOR COMP**"
        if self.mode != CONSTANTS["MODE_BUILD"]:
            return False

        parallelResistors = []
        parallelCapacitors = []
        parallelDiodes = []
        for component in otherComponents:
            if (component[0] == "resistor"):
                parallelResistors.append(component[1])
            elif (component[0] == "capacitor"):
                parallelCapacitors.append(component[1])
            elif (component[0] == "diode"):
                parallelDiodes.append(component[1])
        if (len(parallelDiodes) > 0):
            return "Error: Resistor cannot be measured with diode in parallel"
        elif (len(parallelCapacitors) > 0):
            return "I will do this later. Only valid in certain values and using sympy"
        else:
            #if (not otherComponents):
            # Serial
            self.changeReferenceResistor(0)
            resistance = 0
            self.ser.write([CONSTANTS["SERIAL_BUILD_DIGITAL_COMP"], rail1, rail2])
            if not self.acknowledgeRequest():
                return False
            pot0 = int(self.readToNewline())
            #pot0 = int(self.readToNewline())
            pot1 = int(self.readToNewline())
            potRR = int(self.readToNewline())
            pot0Res = linPotRes(pot0)
            pot1Res = linPotRes(pot1)
            # Mux resistance is a general placeholder
            #potRRRes = parallelResistance([linPotRes(potRR) + self.referenceResistorMuxResistance[0], ADC_RESISTOR_HIGH+ADC_RESISTOR_LOW])
            potRRRes = linPotRes(potRR) + self.referenceResistorMuxResistance[0]
            resistance = potRRRes * pot0Res/pot1Res - self.muxDemuxResistance-(self.muxDemuxResistanceVariance[rail1]+self.muxDemuxResistanceVariance[rail2])/2
            if DEBUG:
                print "pot0:", pot0, "=", pot0Res
                print "pot1:", pot1, "=", pot1Res
                print "potRR:", potRR, "=", potRRRes
                print "Resistance:", resistance
            pot0 = int(self.readToNewline())
            #pot0 = int(self.readToNewline())
            pot1 = int(self.readToNewline())
            potRR = int(self.readToNewline())
            pot0Res = linPotRes(pot0)
            pot1Res = linPotRes(pot1)
            # Mux resistance is a general placeholder
            #potRRRes = parallelResistance([linPotRes(potRR) + self.referenceResistorMuxResistance[0], ADC_RESISTOR_HIGH+ADC_RESISTOR_LOW])
            potRRRes = linPotRes(potRR) + self.referenceResistorMuxResistance[0]
            resistance2 = potRRRes * pot0Res/pot1Res - self.muxDemuxResistance-(self.muxDemuxResistanceVariance[rail1]+self.muxDemuxResistanceVariance[rail2])/2
            if DEBUG:
                print "pot0:", pot0, "=", pot0Res
                print "pot1:", pot1, "=", pot1Res
                print "potRR:", potRR, "=", potRRRes
                print "Resistance:", resistance2
            resistance = (resistance+resistance2)/2
            unknownResistance = reverseParallelResistance(resistance, parallelResistors)
            if (abs(resistance - parallelResistance(parallelResistors)) / resistance < 0.01):
                return "Error: Resistance is too high to be measured"
            self.changeReferenceResistor(0)
            return unknownResistance

    def buildCapacitor(self, rail1, rail2, otherComponents=[]):
        if REPORT_STATE: print "**BUILD CAPACITOR**"
        if self.mode != CONSTANTS["MODE_BUILD"]:
            return False

        numMeasurements = 255
        RCthreshold = 0.001
        parallelResistors = []
        parallelCapacitors = []
        parallelDiodes = []
        for component in otherComponents:
            if (component[0] == "resistor"):
                parallelResistors.append(component[1])
            elif (component[0] == "capacitor"):
                parallelCapacitors.append(component[1])
            elif (component[0] == "diode"):
                parallelDiodes.append(component[1])
        if (len(parallelDiodes) > 0):
            return "Error: Capacitor cannot be measured with diode in parallel"
        elif (len(parallelResistors) > 0):
            return "I will do this later. Only works with very specific values and sympy"
        else:
            #if (not otherComponents):
            # Serial
            capacitanceNew = 0
            unknownCapacitance = 0
            contentMeasurement = False
            self.changeReferenceResistor(0)
            while not contentMeasurement:
                if DEBUG: print "Calling Arduino buildVoltageTimeRead"
                self.ser.write([CONSTANTS["SERIAL_BUILD_VOLTAGE_TIME_READ"], rail1, rail2, numMeasurements])
                if not self.acknowledgeRequest():
                    return False
                totalResistance = self.totalReferenceResistanceWithAdc[self.currRR]+self.muxDemuxResistance+(self.muxDemuxResistanceVariance[rail1]+self.muxDemuxResistanceVariance[rail2])/2
                voltage = []
                timeList = []
                capacitance = []
                averageCapacitance = 0
                for i in range(numMeasurements):
                    timeList.append(float(self.readToNewline())*1e-6)
                    voltage.append(float(self.readToNewline()) * self.adcToVoltage)
                # Split in 2 for loops to prevent buffer overflow
                lowerVoltageThreshold = float(self.supplyVoltage)/CONSTANTS["ADC_SIZE"]*5
                if DEBUG: print "lowerVoltageThreshold:", lowerVoltageThreshold
                for i in range(numMeasurements):
                    if (voltage[i] < lowerVoltageThreshold):
                        continue
                    current = voltage[i]/self.totalReferenceResistanceWithAdc[self.currRR]
                    voltageCapacitor = self.supplyVoltage - current*totalResistance
                    # Lazy catch for late inaccuracies
                    # May have to add an additional check to account for getting stuck at like 1 or 1022
                    if (voltageCapacitor > 0) and (voltageCapacitor < self.supplyVoltage):
                        capacitance.append(-timeList[i]/(totalResistance*math.log(1-voltageCapacitor/self.supplyVoltage))*1e6)
                if DEBUG: print "All time values:", timeList
                if DEBUG: print "All voltage values:", voltage
                if DEBUG: print "Len capacitance:", len(capacitance), "\tAll capacitance values", capacitance
                if DEBUG: print "Original capacitance:", average(capacitance), "+-", std(capacitance)
                capacitanceNew = removeDeviations(capacitance)
                if DEBUG: print "Old stddev:", std(capacitanceNew), "\tNew stddev:", std(removeDeviations(capacitanceNew))
                while (std(capacitanceNew) > average(capacitanceNew)*0.2) and std(removeDeviations(capacitanceNew)) < std(capacitanceNew):
                    capacitanceNew = removeDeviations(capacitanceNew)
                if DEBUG: print "Parsed capacitance:", average(capacitanceNew), "+-", std(capacitanceNew)
                averageCapacitance = average(capacitanceNew)
                numValidMeasurementsThreshold = 0.2
                if (averageCapacitance*1e-6*self.totalReferenceResistanceWithAdc[self.currRR] < 0.001 or len(capacitanceNew) < len(timeList)*numValidMeasurementsThreshold) and \
                        (self.currRR < CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]-1):
                    if DEBUG: print "Time constant used too small. Stepping up resistor value. RC=", averageCapacitance*self.totalReferenceResistanceWithAdc[self.currRR]
                    #while (True):
                    #    var = self.readToNewline()
                    #    print var
                    #    if var == "FINISH":
                    #        break
                    time.sleep(1)
                    self.ser.write([CONSTANTS["SERIAL_BUILD_CHANGE_REFERENCE_RESISTOR"], self.currRR+1])
                    if (not self.acknowledgeRequest()):
                        return "ERROR: Could not change reference resistance from", self.currRR, "to", self.currRR+1
                    self.currRR += 1
                    #self.changeReferenceResistor(self.currRR+1)
                elif (averageCapacitance*1e-6*self.totalReferenceResistanceWithAdc[self.currRR] > 1) and \
                        (self.currRR > 0):
                    if DEBUG: print "Time constant used too large. Stepping down resistor value. RC=", averageCapacitance*self.totalReferenceResistanceWithAdc[self.currRR]
                    time.sleep(1)
                    self.ser.write([CONSTANTS["SERIAL_BUILD_CHANGE_REFERENCE_RESISTOR"], self.currRR-1])
                    if (not self.acknowledgeRequest()):
                        return "ERROR: Could not change reference resistance from", self.currRR, "to", self.currRR-11
                    self.currRR -= 1
                    #self.changeReferenceResistor(self.currRR-1)
                else:
                    contentMeasurement = True
                unknownCapacitance = averageCapacitance - sum(parallelCapacitors)
                if DEBUG: print "Unknown capacitance:", unknownCapacitance
            if (unknownCapacitance <= 0):
                return "Error: Capacitance is too small to be detected"
            self.flushBuffer()
            self.changeReferenceResistor(0)
            #self.changeReferenceResistor(0)
            # Somewhere I converted to actual capacitance, and I just want to return in uF
            return unknownCapacitance
        return None

    def buildCapacitor2(self, rail1, rail2, otherComponents=[]):
        if REPORT_STATE: print "**BUILD CAPACITOR 2**"
        if self.mode != CONSTANTS["MODE_BUILD"]:
            return False
        maxTime = 2**16-1
        stopRatioStart = 0.632
        stopRatioMin = 0.1
        stopRatioMax = 0.9

        parallelResistors = []
        parallelCapacitors = []
        parallelDiodes = []
        for component in otherComponents:
            if (component[0] == "resistor"):
                parallelResistors.append(component[1])
            elif (component[0] == "capacitor"):
                parallelCapacitors.append(component[1])
            elif (component[0] == "diode"):
                parallelDiodes.append(component[1])
        if (len(parallelDiodes) > 0):
            return "Error: Capacitor cannot be measured with diode in parallel"
        elif (len(parallelResistors) > 0):
            print "I will do this later. Requires sympy, and only for specific ranges"
        else:
            contentMeasurement = False
            capacitance = 0
            timeElapsedUs = 0
            self.changeReferenceResistor(0)
            stopRatio = stopRatioStart
            while (not contentMeasurement):
                # We will first try to use a e^-1 setting
                totalUpperResistance = self.muxDemuxResistance+(self.muxDemuxResistanceVariance[rail1]+self.muxDemuxResistanceVariance[rail2])/2
                totalLowerResistance = self.totalReferenceResistanceWithAdc[self.currRR]
                startVoltageRead = self.supplyVoltage * totalLowerResistance/(totalUpperResistance+totalLowerResistance)
                endVoltageRead = self.supplyVoltage * totalLowerResistance/(totalUpperResistance+totalLowerResistance) * (1-stopRatio)
                adcStopAt = int(endVoltageRead/self.adcToVoltage)
                if DEBUG:
                    print "ADC measurement should start at:", startVoltageRead/self.adcToVoltage
                    print "Stop ADC measurement when it reaches: ", adcStopAt, "which is", endVoltageRead
                self.ser.write([CONSTANTS["SERIAL_BUILD_TIME_READ"], rail1, rail2, int(adcStopAt/256), adcStopAt%256, int(maxTime/256), maxTime%256])
                if not self.acknowledgeRequest():
                    return False
                timeElapsedUs = float(self.readToNewline())
                if DEBUG: print "timeElapsedUs:", timeElapsedUs
                if (timeElapsedUs < 10000):
                    if (self.currRR < CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]-1):
                        self.changeReferenceResistor(self.currRR+1)
                    elif (stopRatio < stopRatioMax):
                        stopRatio = stopRatioMax
                    else:
                        contentMeasurement = True
                elif (timeElapsedUs > 1e6):
                    if (self.currRR > 0):
                        self.changeReferenceResistor(self.currRR-1)
                    elif (stopRatio > stopRatioMin):
                        stopRatio = stopRatioMin
                    else:
                        contentMeasurement = True
                else:
                    contentMeasurement = True
                capacitance = -timeElapsedUs/((totalUpperResistance+totalLowerResistance)*math.log(1-stopRatio))
                if DEBUG: print "Calculated capacitance:", capacitance
            unknownCapacitance = capacitance - sum(parallelCapacitors)
            if (unknownCapacitance <= 0):
                return "Error: Capacitance is too small to be detected"
            self.changeReferenceResistor(0)
            return unknownCapacitance

    def buildCapacitorComp(self, rail1, rail2, otherComponents=[]):
        if REPORT_STATE: print "**BUILD CAPACITOR COMP**"
        if self.mode != CONSTANTS["MODE_BUILD"]:
            return False
        maxTime = 2**16-1
        stopRatioStart = 0.632
        stopRatioMin = 0.1
        stopRatioMax = 0.9

        parallelResistors = []
        parallelCapacitors = []
        parallelDiodes = []
        for component in otherComponents:
            if (component[0] == "resistor"):
                parallelResistors.append(component[1])
            elif (component[0] == "capacitor"):
                parallelCapacitors.append(component[1])
            elif (component[0] == "diode"):
                parallelDiodes.append(component[1])
        if (len(parallelDiodes) > 0):
            return "Error: Capacitor cannot be measured with diode in parallel"
        elif (len(parallelResistors) > 0):
            print "I will do this later. Requires sympy, and only for specific ranges"
        else:
            contentMeasurement = False
            capacitance = 0
            timeElapsedUs = 0
            #self.changeReferenceResistor(0)
            # DEBUG: Increase reference resistance to see the changes more slowly
            self.changeReferenceResistor(2)
            stopRatio = stopRatioStart
            while (not contentMeasurement):
                # We will first try to use a e^-1 setting
                totalUpperResistance = self.muxDemuxResistance+(self.muxDemuxResistanceVariance[rail1]+self.muxDemuxResistanceVariance[rail2])/2
                totalLowerResistance = self.totalReferenceResistanceWithAdc[self.currRR]
                startFraction = totalLowerResistance/(totalUpperResistance+totalLowerResistance)
                endFraction = startFraction * (1-stopRatio)
                if DEBUG:
                    print "Calculating new capacitance"
                    print "totalUpperResistance", totalUpperResistance
                    print "totalLowerResistance", totalLowerResistance
                    print "startFraction:", startFraction
                    print "endFraction:", endFraction
                compRatio = 1.0/endFraction - 1
                potHigh = None
                potLow = None
                if (compRatio < 1):
                    # Add to 256 functionality later
                    compResValues = Fraction(str(compRatio)).limit_denominator(255)
                    factorUp = min(int(255/compResValues.numerator), int(255/compResValues.denominator))
                    potHigh = compResValues.numerator * factorUp
                    potLow = compResValues.denominator * factorUp
                else:
                    compResValues = Fraction(str(1.0/compRatio)).limit_denominator(255)
                    factorUp = min(int(255/compResValues.numerator), int(255/compResValues.denominator))
                    potHigh = compResValues.denominator * factorUp
                    potLow = compResValues.numerator * factorUp
                # DEBUG
                #potHigh = 255
                #potLow = 255
                actualEndFraction = linPotRes(potLow)/(linPotRes(potHigh)+linPotRes(potLow))
                actualStopRatio = 1 - actualEndFraction/startFraction
                #actualStopRatio = actualEndFraction/startFraction
                if DEBUG:
                    print "potHigh:", potHigh, "= ", linPotRes(potHigh), "Ohms"
                    print "potLow:", potLow, "= ", linPotRes(potLow), "Ohms"
                    print "Initially set stop ratio:", stopRatio, "\t Actual stop ratio:", actualStopRatio
                    print "Current end voltage fraction", endFraction, "\tActual end fraction:", actualEndFraction
                    print "Equivalent ADC stop level:", actualEndFraction*1024
                self.ser.write([CONSTANTS["SERIAL_BUILD_DIGITAL_INTERRUPT"], rail1, rail2, potHigh, potLow])
                if not self.acknowledgeRequest():
                    return False

                timeElapsedUs = self.readToNewline(22)
                if DEBUG: print "timeElapsedUs", timeElapsedUs
                timeElapsedUs = float(timeElapsedUs)
                if DEBUG: print "timeElapsedUs:", timeElapsedUs
                if (timeElapsedUs < 1000):
                    if (self.currRR < CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]-1):
                        self.changeReferenceResistor(self.currRR+1)
                    elif (stopRatio < stopRatioMax):
                        stopRatio = stopRatioMax
                    else:
                        contentMeasurement = True
                elif (timeElapsedUs > 1e6):
                    if (self.currRR > 0):
                        self.changeReferenceResistor(self.currRR-1)
                    elif (stopRatio > stopRatioMin):
                        stopRatio = stopRatioMin
                    else:
                        contentMeasurement = True
                else:
                    contentMeasurement = True
                capacitance = -timeElapsedUs/((totalUpperResistance+totalLowerResistance)*math.log(1-actualStopRatio))
                if DEBUG: print "Calculated capacitance:", capacitance
            unknownCapacitance = capacitance - sum(parallelCapacitors)
            if (unknownCapacitance <= 0):
                return "Error: Capacitance is too small to be detected"
            self.changeReferenceResistor(0)
            return unknownCapacitance

    def buildCapacitorMix(self, rail1, rail2, otherComponents=[]):
        ans = self.buildCapacitor2(rail1, rail2, otherComponents)
        if (isFloat(ans) and ans > 0.05):
            return self.buildCapacitor(rail1, rail2, otherComponents)
        else:
            return ans

    def buildDiode(self, rail1, rail2, otherComponents=[]):
        if self.mode != CONSTANTS["MODE_BUILD"]:
            return False
        numMeasurements = 100
        #self.changeReferenceResistor(CONSTANTS["REFERENCE_RESISTOR_SELECT_SIZE"]-1)
        self.changeReferenceResistor(2)
        parallelResistors = []
        parallelCapacitors = []
        parallelDiodes = []
        numDiodesForward = 0
        numDiodesReverse = 0
        thresholdVoltageDifference = self.supplyVoltage*0.2
        for component in otherComponents:
            if (component[0] == "resistor"):
                parallelResistors.append(component[1])
            elif (component[0] == "capacitor"):
                parallelCapacitors.append(component[1])
            elif (component[0] == "diode"):
                parallelDiodes.append(component[1])
                if component[1] == "forward":
                    numDiodesForward += 1
                elif component[1] == "reverse":
                    numDiodesReverse += 1
        if (len(parallelCapacitors) > 0):
            # parallel capacitance stuff
            # NOTE: CAPACITORS CANNOT BE ELECTROLYTIC OR IT WILL BLOW UP
            self.ser.write([CONSTANTS["SERIAL_BUILD_VOLTAGE_TIME_READ"], rail1, rail2, numMeasurements])
            if not self.acknowledgeRequest():
                return False
            voltageReadRailForward = []
            for i in range(numMeasurements):
                self.readToNewline()
                voltageReadRailForward.append(int(self.readToNewline()))
            voltageForward = average(removeDeviations(voltageReadRailForward)) * self.adcToVoltage
            self.ser.write([CONSTANTS["SERIAL_BUILD_VOLTAGE_TIME_READ"], rail2, rail1, numMeasurements])
            if not self.acknowledgeRequest():
                return False
            voltageReadRailReverse = []
            for i in range(numMeasurements):
                self.readToNewline()
                voltageReadRailReverse.append(int(self.readToNewline()))
            voltageNetForward = average(removeDeviations([voltageReadRailForward[i]-voltageReadRailReverse[i] for i in range(numMeasurements)]))
            if voltageNetForward > 0:
                return "Forward Biased. Flows from rail " + str(rail1) + " to " + str(rail2)
            else:
                return "Reverse Biased. Flows from rail " + str(rail2) + " to " + str(rail1)
        else:
            # Diodes only. Do the regular double sided voltage measurements and then interpret them based on diode direction
            # We will assume all the diodes are approximately identical
            if (numDiodesForward > 0 and numDiodesReverse > 0):
                return "Error: Cannot consistently check new diodes when forward and reverse diodes already exist"
            self.ser.write([CONSTANTS["SERIAL_BUILD_VOLTAGE_READ"], rail1, rail2, numMeasurements])
            if not self.acknowledgeRequest():
                return False
            voltageReadRail = []
            voltageReadReferenceResistor = []
            for i in range(numMeasurements):
                voltageReadRail.append(int(self.readToNewline()))
                voltageReadReferenceResistor.append(int(self.readToNewline()))
            voltageForward = average(removeDeviations(voltageReadRail)) * self.adcToVoltage
            self.ser.write([CONSTANTS["SERIAL_BUILD_VOLTAGE_READ"], rail2, rail1, numMeasurements])
            if not self.acknowledgeRequest():
                return False
            voltageReadRail = []
            voltageReadReferenceResistor = []
            for i in range(numMeasurements):
                voltageReadRail.append(int(self.readToNewline()))
                voltageReadReferenceResistor.append(int(self.readToNewline()))
            voltageReverse = average(removeDeviations(voltageReadRail)) * self.adcToVoltage
            # The higher voltage indicates the correct bias, since that means current is flowing through the circuit. If there is no current flow, the voltage will be tied to ground
            #  It should be all the same, as foward biased diodes will always produce a lower voltage drop than reverse biased
            if DEBUG:
                print "Forward voltage:", voltageForward
                print "Reverse voltage:", voltageReverse

            if numDiodesForward > 0:
                if (voltageForward-voltageReverse > thresholdVoltageDifference):
                    return "Forward Biased. Flows from rail " + str(rail1) + " to " + str(rail2)
                else:
                    return "Reverse Biased. Flows from rail " + str(rail2) + " to " + str(rail1)
            elif numDiodesReverse > 0:
                if (voltageReverse-voltageForward > thresholdVoltageDifference):
                    return "Reverse Biased. Flows from rail " + str(rail2) + " to " + str(rail1)
                else:
                    return "Forward Biased. Flows from rail " + str(rail1) + " to " + str(rail2)
            else:
                if (voltageForward > voltageReverse):
                    return "Forward Biased. Flows from rail " + str(rail1) + " to " + str(rail2)
                else:
                    return "Reverse Biased. Flows from rail " + str(rail2) + " to " + str(rail1)
        return None

    def runPower(self, option):
        if REPORT_STATE: print "**SWITCHING POWER"
        if self.mode != CONSTANTS["MODE_RUN"]:
            return False
        self.ser.write([CONSTANTS["SERIAL_RUN_POWER"], option])
        if (not self.acknowledgeRequest()):
            return False

    def runVoltage(self, rails, numMeasurements=10):
        if REPORT_STATE: print "**MEASURING VOLTAGE ON RAILS"
        if DEBUG: print rails
        if self.mode != CONSTANTS["MODE_RUN"]:
            return False
        railVoltages = {}
        readRailSerialSend = [0]*int(CONSTANTS["MUX_SELECT_SIZE"]/8)
        # Uniquify the list
        rails = list(set(rails))
        for num in rails:
            readRailSerialSend[int(num/8)] += 2**(num%8)
        if DEBUG: print readRailSerialSend
        self.ser.write([CONSTANTS["SERIAL_RUN_VOLTAGE"], numMeasurements]+readRailSerialSend)
        if (not self.acknowledgeRequest()):
            return False
        for i in range(len(rails)):
            railNum = self.readToNewline()
            analogReadIndex = self.readToNewline()
            analogReadValue = self.readToNewline()
            railVoltages[int(railNum)] = self.referenceVoltage/RAIL_READ_RATIO[int(analogReadIndex)]*int(analogReadValue)/CONSTANTS["ADC_SIZE"]
            if DEBUG:
                print "railNum:", railNum, "\tanalogReadIndex:", analogReadIndex, "\tanalogReadValue:", analogReadValue
        return railVoltages

    def runCurrent(self):
        return None

    def acknowledgeRequest(self):
        if REPORT_STATE: print "**CHECKING ARDUINO FOR REQUEST ACKNOWLEDGEMENT**"
        timeReference = time.time()
        while ((time.time() - timeReference) < 5):
            output = self.readToNewline()
            if (output == "1"):
                return True
            elif (output == "0"):
                return False
            elif DEBUG:
                print output
        return False

    def flushBuffer(self):
        if REPORT_STATE: print "**FLUSHING SERIAL BUFFER**"
        #print "Flushing Buffer"
        out = ""
        while (self.ser.inWaiting()):
            out += self.ser.read()
        if DEBUG: print out
        #print "Done flushing buffer"

    def readToNewline(self, timeLimit=5):
        line = ""
        timeReference = time.time()
        while ((time.time() - timeReference) < timeLimit):
            if (self.ser.inWaiting()):
                line += self.ser.read()
                timeReference = time.time()
                #print "This is char:", char
                if (line[-1] == "\n"):
                    if ("DEBUG" in line):
                        if DEBUG:
                            print line
                        line = ""
                    else:
                        return line.strip()

    def debugSerial(self, byteString, waitTime=1):
        if REPORT_STATE: print "**OVERRIDE SERIAL INPUT**"
        bytes = [int(byteString) for byteString in byteString.strip().split(" ")]
        self.ser.write(bytes)
        time.sleep(waitTime)
        result = ""
        while (self.ser.inWaiting()):
            result += self.ser.read()
        return result

def isFloat(string):
    try:
        num = float(string)
    except ValueError:
        return False
    return True

def isInt(string):
    try:
        num = int(string)
    except ValueError:
        return False
    return True

def parallelResistance(aList):
    if not aList:
        return 0
    resistance = 0
    for res in aList:
        resistance += 1.0/res
    resistance = 1.0/resistance
    return resistance

def reverseParallelResistance(totalResistance, resistorList):
    if not resistorList:
        return totalResistance
    calc = 1.0/totalResistance
    for resistor in resistorList:
        calc = calc - 1.0/resistor
    calc = 1.0/calc
    return calc

def average(values):
    return sum(values)/len(values)

def removeDeviations(values, stdThreshold=2):
    stddev = std(values)
    newValues = []
    for i in values:
        if (abs(average(values)-i) <= stddev*stdThreshold):
            newValues.append(i)
    return newValues

def linPotRes(value):
    return 75 + 10000.0/256*value

'''
class Component:
    COMPONENT_RESISTOR = 0
    COMPONENT_CAPACITOR = 1
    COMPONENT_DIODE = 2
    def __init__(self, connection, componentList):
'''

# Call main function
if __name__ == '__main__':
    main()
