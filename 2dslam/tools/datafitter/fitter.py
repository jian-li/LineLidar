import numpy
import math
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def func(x,a,b):
	z = (239.5 - x) * a + b;
	return z

#calculate the parameter use the measured data
focal_length = 3.1 #unit mm
base_line = 80
pixel_len = 6.35/5*4/1280
theoral_value =  focal_length * base_line / pixel_len 
print theoral_value

# Generate artificial data = straight line with a=0 and b=1
# plus some noise.
xdata = numpy.array([11.9719, 55.5505, 95.96, 111.00, 125.07, 144.451, 156.266, 162.811, 169.82, 176.493 ]);
#xdata = numpy.array([54.0, 94.98, 143.65, 165.69, 178.29, 185.40, 191.00, 192.00])
ydata = numpy.array([34.7, 42.2, 54.7, 61.7, 72.22, 95.00, 118.0, 136.9, 172.3, 223.1])
theta = [math.atan(1.0 * 8/y) for y in ydata]

#for data in theta:
#	print data

weights = [(240 - x)  for x in xdata]
#weights = [pow((240 - x), 0.5) for x in xdata]
popt, pcov = curve_fit(func, xdata, theta)#,  theoral_value, weights)
#popt, pcov = curve_fit(func, xdata, theta)#,  theoral_value, weights)

print popt#, pcov


#the error of each point
predicted_dist = [8/math.tan((239.5 - x)* popt[0] + popt[1]) for x in xdata]

#plot the raw data
plt.plot(xdata, ydata, 'ro')


#plot the fitted function curce
x_value = numpy.arange(10, 190, 0.1 )
y_value = [ 8/ math.tan((239.5 - x) * popt[0] + popt[1]) for x in x_value]

plt.plot(x_value, y_value)
plt.show()

error = xdata
for i in range(0, 8):
	error[i] = predicted_dist[i] - ydata[i]
#error = list(predicted_dist - ydata)
for data in predicted_dist:
	print data
for data in error: 
	print data
