import arduinoController
from arduinoController import ArduinoInterface



def main():
    # This is still not necessarily the real deal?
    # We want to read serial, call the appropriate function, then dump everything out for like a second?
    # In the real program we should call the appropriate function and then only stop once we've read everything we said we were going to read
    # Note: This method will require a lot of explicit handling for lengths of some serial outputs. Maybe there's an option either at bootup or when returning valid command to inform the python program what size of serial output to expect

    #if REPORT_STATE: print "**STARTING PROGRAM**"

    #for keys,values in CONSTANTS.items():
    #    print keys, values

    #if REPORT_STATE: print "**LOCATING ARDUINO**"
    arduino = None
    try:
        arduino = ArduinoInterface()
    except AssertionError as error:
        print error
        print "Exiting as a result"
        exit()


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
                        if arduinoController.isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildResistor(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 3) and (contents[0] == "capacitor"):
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if arduinoController.isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildCapacitorComp(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 3) and (contents[0] == "capacitor2"):
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if arduinoController.isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildCapacitor2(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 3) and (contents[0] == "diode"):
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if arduinoController.isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildDiode(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 3) and (contents[0] == "resistorComp"):
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if arduinoController.isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildResistorComp(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 3) and (contents[0] == "capacitorComp"):
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if arduinoController.isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildCapacitorComp(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 3) and (contents[0] == "capacitorMix"):
                otherComponents = []
                if (len(contents) > 3):
                    for i in contents[3:]:
                        split = i.split(",")
                        if arduinoController.isFloat(split[1]):
                            otherComponents.append([split[0], float(split[1])])
                        else:
                            otherComponents.append(split)
                print arduino.buildCapacitorMix(int(contents[1]), int(contents[2]), otherComponents)
            elif (len(contents) >= 2) and (contents[0] == "power"):
                if (contents[1] == "on"):
                    arduino.runPower(arduinoController.CONSTANTS["SWITCH_ON"])
                elif (contents[1] == "off"):
                    arduino.runPower(arduinoController.CONSTANTS["SWITCH_OFF"])
                elif (contents[1] == "toggle"):
                    arduino.runPower(arduinoController.CONSTANTS["SWITCH_TOGGLE"])
            elif (len(contents) >= 2) and (contents[0] == "voltage"):
                railNums = []
                for i in contents[1:]:
                    if (not arduinoController.isInt(i)):
                        print "Invalid input"
                        continue
                    else:
                        railNums.append(int(i))
                ans = arduino.runVoltage(railNums)
                for i in ans:
                    print i, ans[i]
            elif (all(arduinoController.isInt(x) for x in contents)):
                # Need to do more extensive test here in case inputs are not ints, but for now this can do
                if arduinoController.DEBUG: print "Hello let's debug"
                print arduino.debugSerial(text)
            elif (text.split(":")[0] == "eval"):
                eval(text.split(":")[1:])
            else:
                print "Error: Unexpected input. Discarding command"


# Call main function
if __name__ == '__main__':
    main()
