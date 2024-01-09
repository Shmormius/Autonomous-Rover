from flask import Flask, render_template, request
import serial

app = Flask(__name__)

commands=[]

@app.route("/")
def main():
    templateData = {'commands' : commands} #Stores the Command sent string so the page can use it
    return render_template('interaction.html', **templateData)

@app.route("/", methods=['POST'])
def interaction(): #The Serial Processing Code
    action = request.form['action']
    command_serial = "{}\n".format(action) 
    ser.write(command_serial.encode('utf-8')) #writes the command to the serial
    line = ser.readline().decode('utf-8').rstrip() #reads from serial
    commands.insert(0, line)
    templateData = {'commands' : commands}
    return render_template('interaction.html', **templateData)

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1) #Opens serial device. /dev/tty*** seems to change each time you unplug the arduino
    ser.reset_input_buffer()
    app.run(host="0.0.0.0")
