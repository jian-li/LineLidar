import numpy
import math
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def func(x, a):
    return  a/(240 - x) 

#calculate the parameter use the measured data
focal_length = 3.1 #unit mm
base_line = 80
pixel_len = 6.35/5*4/1280
theoral_value =  focal_length * base_line / pixel_len 
print theoral_value

# Generate artificial data = straight line with a=0 and b=1
# plus some noise.
xdata = numpy.array([86.2, 117.4, 133.9, 146, 166.5, 179.6, 194.6])
ydata = numpy.array([40, 60, 80, 100, 200, 300, 400])
#weights = [(240 - x)*(240 -  x)  for x in xdata]
weights = [pow((240 - x), 0.8) for x in xdata]
#popt, pcov = curve_fit(func, xdata, ydata,  theoral_value, weights)
popt, pcov = curve_fit(func, xdata, ydata)#,  theoral_value, weights)

print popt, pcov


#the error of each point
predicted_dist = [popt/(240 - x) for x in xdata]

plt.plot(xdata, ydata, 'ro')
plt.show()

error = xdata
for i in range(0, 7):
	error[i] = predicted_dist[i] - ydata[i]
#error = list(predicted_dist - ydata)
for data in predicted_dist:
	print data
for data in error: 
	print data
