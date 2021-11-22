import numpy as np
import serial

serial_com = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

print('Serial Comm Initialized')

data = []

try :
    
	while serial_com.is_open:

		content = serial_com.readline()

		if len(content) > 0 : 
			
			print(content)

			data.append(content.split(b','))
 
except KeyboardInterrupt:

	print("\nClosing connection")
	serial_com.close()

data = np.array(data, dtype=float)
# print(data)
np.savetxt('Motor PWM Duty Cycle Response.csv', data, delimiter=',')