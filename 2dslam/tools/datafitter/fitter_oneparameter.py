import numpy
import math
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def func(x, a, b):
    return  a/(239.5 - x) + b

#calculate the parameter use the measured data
focal_length = 3.1 #unit mm
base_line = 80
pixel_len = 6.35/5*4/1280
theoral_value =  focal_length * base_line / pixel_len 
print theoral_value

# Generate artificial data = straight line with a=0 and b=1
# plus some noise.
xdata = numpy.array([59.9, 102.50, 148.30, 171.47, 183.42, 191.10, 195.04, 198.75])
ydata = numpy.array([30, 40, 70, 110, 160, 220, 300, 370])
weights = [(239.5 - x)  for x in xdata]
#weights = [pow((240 - x), 0.5) for x in xdata]
#popt, pcov = curve_fit(func, xdata, ydata,  theoral_value, weights)
popt, pcov = curve_fit(func, xdata, ydata)#,  theoral_value, weights)

print popt, pcov


#the error of each point
predicted_dist = [popt[0]/(239.5 - x) + popt[1] for x in xdata]

#plot the raw data
plt.plot(xdata, ydata, 'ro')


#plot the fitted function curce
x_value = numpy.arange(10, 200, 0.1 )
y_value = [ popt[0] / (239.5 - x) + popt[1] for x in x_value]

plt.plot(x_value, y_value)
plt.show()

error = xdata
for i in range(0, 7):
	error[i] = predicted_dist[i] - ydata[i]
#error = list(predicted_dist - ydata)
for data in predicted_dist:
	print data
for data in error: 
	print data
